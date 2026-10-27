/*
 * drawterm inside a terminal emulator.
 *
 * Output uses the kitty graphics protocol.  The Plan 9 screen is
 * a grid of cell-aligned tiles, each an image placed once.  Damage
 * is sent as in-place edits of the tiles' root frames (a=f), so the
 * terminal moves only the damaged pixels to the GPU: Ghostty and
 * kitty upload a changed image whole.  Terminals that cannot edit
 * frames get each damaged tile retransmitted whole.  The Plan 9
 * cursor is a small image placed
 * above the screen and moved by re-placing it.  Pixels go through
 * POSIX shared memory when the terminal can open it (same host),
 * otherwise inline as zlib-compressed base64.
 *
 * Input uses the kitty keyboard protocol (press, repeat and release
 * of every key, modifiers included), SGR mouse reporting in pixels
 * (mode 1016), in-band resize notifications (mode 2048), focus
 * events and bracketed paste.  Snarf goes to the host clipboard
 * with OSC 52.  Terminals without the keyboard protocol fall back
 * to legacy key decoding.
 *
 * Ghostty is the reference terminal; kitty works too.  Options are
 * taken from $DTKITTY, see drawterm(1).
 */

/* same feature macros as unix.h; system headers must come before u.h */
#define _DEFAULT_SOURCE 1
#if !defined(__APPLE__) && !defined(__OpenBSD__)
#	define _XOPEN_SOURCE 1000
#	define _XOPEN_SOURCE_EXTENDED 1
#endif
#define _BSD_SOURCE 1
#define _NETBSD_SOURCE 1
#define __BSD_VISIBLE 1
#define _SVID_SOURCE 1
#define _DARWIN_C_SOURCE 1

#include <pthread.h>
#include <termios.h>
#include <sys/select.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <zlib.h>

/*
 * wait up to ms for input on a or b; set *ra, *rb if there is.
 * select, not poll: poll does not do ttys on macOS.  it lives
 * up here because the fd_set macros break under u.h's defines.
 */
static int
waitfd(int a, int b, int ms, int *ra, int *rb)
{
	struct timeval tv;
	fd_set rd;
	int n;

	FD_ZERO(&rd);
	FD_SET(a, &rd);
	FD_SET(b, &rd);
	tv.tv_sec = ms/1000;
	tv.tv_usec = (ms%1000)*1000;
	n = select((a > b ? a : b)+1, &rd, NULL, NULL, &tv);
	*ra = n > 0 && FD_ISSET(a, &rd);
	*rb = n > 0 && FD_ISSET(b, &rd);
	return n;
}

#include "u.h"
#include "lib.h"
#include "dat.h"
#include "fns.h"
#include "error.h"

#include <draw.h>
#include <memdraw.h>
#include <keyboard.h>
#include <cursor.h>
#include "screen.h"

#undef atexit	/* we want the host's: restore the terminal on any exit */

extern void	exits(char*);	/* from user.h, which we cannot include */
extern int	panicking;	/* kern/devcons.c */

typedef struct Buf Buf;
typedef struct Csi Csi;
typedef struct Down Down;

struct Buf
{
	char	*p;
	int	n;
	int	cap;
};

enum
{
	Nfield	= 8,
	Nsub	= 16,
};

struct Csi
{
	int	prefix;		/* < = > ? or 0 */
	int	inter;		/* intermediate byte or 0 */
	int	final;
	int	nf;
	int	ns[Nfield];
	int	v[Nfield][Nsub];	/* -1 when empty */
};

struct Down
{
	int	id;		/* physical key */
	Rune	r;		/* what we reported */
};

enum
{
	/* image ids, unlikely to clash with anybody else's */
	Idbase		= 0x44540000,
	Idcursor	= Idbase+2,
	Idprobe		= Idbase+3,	/* 3 of them */
	Idtile		= Idbase+0x100,

	Ndirty		= 32,
	Slack		= 32*32,	/* merge damage that wastes less */
	Nshm		= 4096,		/* > 2 frames of Maxtiles */
	Chunk		= 4096,		/* base64 bytes per APC */
	Maxstr		= 8*1024*1024,	/* longest OSC/APC/DCS we accept */
	Ndown		= 64,
	Tilesz		= 256,
	Maxtiles	= 1024,

	/* kitty keyboard flags: disambiguate, events, alternates, all keys, text */
	Kbdflags	= 1|2|4|8|16,

	/* kitty modifier bits, after subtracting 1 */
	Mshift		= 1,
	Malt		= 2,
	Mctl		= 4,
	Msuper		= 8,
	Mcaps		= 64,

	/* key event types */
	Epress		= 1,
	Erepeat		= 2,
	Erelease	= 3,
};

static struct
{
	int	fd;			/* the terminal */
	struct termios	tio;		/* its settings before us */
	int	raw;
	int	sigfd[2];		/* SIGWINCH self-pipe */
	int	savefd[3];		/* stdout, stderr while drawing */
	int	logfd;
	int	debugfd;
	int	probing;

	/* what the terminal can do */
	int	gfx;
	int	shm;
	int	frames;
	int	kbd;
	int	pixels;
	int	sync;
	int	inband;
	int	da;
	char	name[128];
	int	probecw, probech;	/* CSI 16 t */
	int	probew, probeh;		/* CSI 14 t */

	/* geometry */
	int	cols, rows;
	int	cw, ch;			/* cell size in pixels */

	/* options */
	int	optshm;			/* -1 auto, 0 never, 1 always */
	int	optframes;
	int	fps;
	int	zlevel;
	int	tilesz;
	int	optcursor;
	int	emu;
	int	clipread;

	/* output state, under lk */
	pthread_mutex_t	lk;
	pthread_cond_t	cond;
	int	full;			/* everything must be sent again */
	Rectangle	dirty[Ndirty];
	int	ndirty;
	int	curshape;		/* cursor image changed */
	int	curmove;		/* cursor moved */
	Point	curpt;
	Buf	pend;			/* escape sequences to send */
	int	errors;			/* graphics errors since the last full */
	ulong	lastfull;
	int	ackok;			/* terminal answers CSI 5 n */
	ulong	sent;			/* CSI 5 n sent */
	ulong	acks;			/* and answered */
	int	curhide;		/* plan 9 cursor not drawn */
	int	curshown;

	/* clipboard, under lk */
	char	*snarf;
	char	*clip;
	int	clipwait;

	/* shared memory objects we may still own */
	char	shmname[Nshm][32];
	int	shmi;
	uint	shmseq;

	/* input */
	int	buttons;
	int	tbutton[3];		/* plan 9 button of each terminal button */
	int	altmouse;
	Down	down[Ndown];
	int	ndown;

	/* the session is over but the kernel hangs to show why */
	char	*dead;
} kt = {
	.fd = -1,
	.logfd = -1,
	.debugfd = -1,
	.lk = PTHREAD_MUTEX_INITIALIZER,
	.cond = PTHREAD_COND_INITIALIZER,
};

Memimage *gscreen;

static void	inbyte(int);
static void	kick(void);
static void	quit(void);

static void
dbg(char *fmt, ...)
{
	char buf[512];
	va_list arg;
	int n;

	if(kt.debugfd < 0)
		return;
	n = snprint(buf, sizeof buf, "%lud ", ticks());
	va_start(arg, fmt);
	n += vsnprint(buf+n, sizeof buf-n-1, fmt, arg);
	va_end(arg);
	buf[n++] = '\n';
	write(kt.debugfd, buf, n);
}

/*
 * growable byte buffers
 */
static void
bgrow(Buf *b, int n)
{
	if(b->n+n <= b->cap)
		return;
	b->cap = (b->n+n)*2;
	if(b->cap < 4096)
		b->cap = 4096;
	b->p = realloc(b->p, b->cap);
	if(b->p == nil)
		panic("kitty: out of memory");
}

static void
bput(Buf *b, void *p, int n)
{
	bgrow(b, n);
	memmove(b->p+b->n, p, n);
	b->n += n;
}

static void
bputs(Buf *b, char *s)
{
	bput(b, s, strlen(s));
}

static void
bprint(Buf *b, char *fmt, ...)
{
	char tmp[512];
	va_list arg;
	int n;

	va_start(arg, fmt);
	n = vsnprint(tmp, sizeof tmp, fmt, arg);
	va_end(arg);
	bput(b, tmp, n);
}

static void
b64(Buf *b, uchar *p, int n)
{
	int m;

	m = 4*((n+2)/3);
	bgrow(b, m+1);
	b->n += enc64(b->p+b->n, m+1, p, n);
}

/*
 * the terminal
 */
static void
ttywrite(char *p, int n)
{
	int m;

	while(n > 0){
		m = write(kt.fd, p, n);
		if(m < 0){
			if(errno == EINTR || errno == EAGAIN)
				continue;
			return;
		}
		p += m;
		n -= m;
	}
}

static void
ttyputs(char *s)
{
	ttywrite(s, strlen(s));
}

static void
ttyprint(char *fmt, ...)
{
	char buf[256];
	va_list arg;
	int n;

	va_start(arg, fmt);
	n = vsnprint(buf, sizeof buf, fmt, arg);
	va_end(arg);
	ttywrite(buf, n);
}

static char modeson[] =
	"\033[?1049h"		/* alternate screen */
	"\033[?25l"		/* no text cursor */
	"\033[22;2t"		/* save title */
	"\033[?1000h\033[?1002h\033[?1003h"	/* all mouse motion */
	"\033[?1006h\033[?1016h"	/* SGR, in pixels */
	"\033[?1004h"		/* focus events */
	"\033[?2004h"		/* bracketed paste */
	"\033[?2048h";		/* in-band resize reports */

static char modesoff[] =
	"\033_Ga=d,d=A,q=2\033\\"	/* our images */
	"\033[?2048l\033[?2004l\033[?1004l"
	"\033[?1016l\033[?1006l"
	"\033[?1003l\033[?1002l\033[?1000l"
	"\033[23;2t"
	"\033[?25h"
	"\033[?1049l";

/* only async-signal-safe calls in here */
static void
ttyrestore(void)
{
	int i;

	if(kt.fd < 0 || !kt.raw)
		return;
	kt.raw = 0;
	if(kt.kbd)
		ttyputs("\033[<u");
	ttyputs(modesoff);
	tcsetattr(kt.fd, TCSAFLUSH, &kt.tio);
	for(i = 0; i < Nshm; i++)
		if(kt.shmname[i][0] != 0)
			shm_unlink(kt.shmname[i]);
}

/* give back the terminal; show what was printed meanwhile */
static void
ttyexit(void)
{
	char buf[8192];
	int n;
	off_t o;

	ttyrestore();
	if(kt.logfd < 0)
		return;
	dup2(kt.savefd[1], 1);
	dup2(kt.savefd[2], 2);
	o = lseek(kt.logfd, 0, SEEK_END);
	if(o > sizeof buf)
		lseek(kt.logfd, o-sizeof buf, SEEK_SET);
	else
		lseek(kt.logfd, 0, SEEK_SET);
	while((n = read(kt.logfd, buf, sizeof buf)) > 0)
		write(2, buf, n);
	close(kt.logfd);
	kt.logfd = -1;
}

static void
sigdie(int sig)
{
	ttyrestore();
	signal(sig, SIG_DFL);
	raise(sig);
}

static void
sigwinch(int sig)
{
	int e;

	USED(sig);
	e = errno;
	write(kt.sigfd[1], "w", 1);
	errno = e;
}

static void
ttyinit(void)
{
	struct termios t;

	kt.fd = open("/dev/tty", O_RDWR|O_NOCTTY);
	if(kt.fd < 0 && isatty(0))
		kt.fd = dup(0);
	if(kt.fd < 0)
		panic("kitty: no terminal: %s", strerror(errno));
	fcntl(kt.fd, F_SETFD, FD_CLOEXEC);
	if(tcgetattr(kt.fd, &kt.tio) < 0)
		panic("kitty: tcgetattr: %s", strerror(errno));
	t = kt.tio;
	t.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IXOFF);
	t.c_oflag &= ~OPOST;
	t.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	t.c_cflag &= ~(CSIZE|PARENB);
	t.c_cflag |= CS8;
	t.c_cc[VMIN] = 1;
	t.c_cc[VTIME] = 0;
	if(tcsetattr(kt.fd, TCSAFLUSH, &t) < 0)
		panic("kitty: tcsetattr: %s", strerror(errno));
	kt.raw = 1;
	atexit(ttyexit);
	signal(SIGTERM, sigdie);
	signal(SIGINT, sigdie);
	signal(SIGQUIT, sigdie);
	signal(SIGHUP, sigdie);
	signal(SIGPIPE, SIG_IGN);
	if(pipe(kt.sigfd) < 0)
		panic("kitty: pipe: %s", strerror(errno));
	fcntl(kt.sigfd[0], F_SETFL, O_NONBLOCK);
	fcntl(kt.sigfd[1], F_SETFL, O_NONBLOCK);
	signal(SIGWINCH, sigwinch);
	ttyputs(modeson);
}

/*
 * stray prints to the terminal would scribble over the picture:
 * collect them in a file and show them when we are done.
 */
static void
hideoutput(void)
{
	char tmp[] = "/tmp/drawterm.XXXXXX";

	if(!isatty(1) && !isatty(2))
		return;
	if((kt.logfd = mkstemp(tmp)) < 0)
		return;
	unlink(tmp);
	kt.savefd[1] = dup(1);
	kt.savefd[2] = dup(2);
	if(isatty(1))
		dup2(kt.logfd, 1);
	if(isatty(2))
		dup2(kt.logfd, 2);
}

/*
 * kitty graphics
 */

/* a fresh shared memory object holding p[0:n] */
static char*
shmput(uchar *p, int n)
{
	char *name;
	void *v;
	int fd;

	name = kt.shmname[kt.shmi];
	if(name[0] != 0)
		shm_unlink(name);	/* the terminal should have done it long ago */
	snprint(name, sizeof kt.shmname[0], "/dtk%d.%ud", getpid(), kt.shmseq++);
	kt.shmi = (kt.shmi+1) % Nshm;
	fd = shm_open(name, O_CREAT|O_EXCL|O_RDWR, 0600);
	if(fd < 0){
		name[0] = 0;
		return nil;
	}
	if(ftruncate(fd, n) < 0
	|| (v = mmap(nil, n, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED){
		close(fd);
		shm_unlink(name);
		name[0] = 0;
		return nil;
	}
	memmove(v, p, n);
	munmap(v, n);
	close(fd);
	return name;
}

/*
 * a graphics command whose payload is n bytes of pixels.
 * keys has everything but the transmission keys t, S, o and m.
 */
static void
gfxdata(Buf *b, char *keys, uchar *p, int n)
{
	static Buf z, e;
	uLongf zn;
	char *name, *o;
	int i, m;

	if(kt.shm && (name = shmput(p, n)) != nil){
		bprint(b, "\033_G%s,t=s,S=%d;", keys, n);
		b64(b, (uchar*)name, strlen(name));
		bputs(b, "\033\\");
		return;
	}
	o = "";
	if(kt.zlevel > 0 && n > 128){
		zn = compressBound(n);
		z.n = 0;
		bgrow(&z, zn);
		if(compress2((Bytef*)z.p, &zn, p, n, kt.zlevel) == Z_OK && zn < n){
			p = (uchar*)z.p;
			n = zn;
			o = ",o=z";
		}
	}
	e.n = 0;
	b64(&e, p, n);
	for(i = 0; i < e.n; i += m){
		m = e.n - i;
		if(m > Chunk)
			m = Chunk;
		if(i == 0)
			bprint(b, "\033_G%s,t=d%s%s;", keys, o, m < e.n ? ",m=1" : "");
		else
			bprint(b, "\033_Gm=%d,q=1;", i+m < e.n);
		bput(b, e.p+i, m);
		bputs(b, "\033\\");
	}
}

/* put the terminal cursor at pixel p, return the offset within the cell */
static Point
cellat(Buf *b, Point p)
{
	bprint(b, "\033[%d;%dH", p.y/kt.ch+1, p.x/kt.cw+1);
	return Pt(p.x%kt.cw, p.y%kt.ch);
}

/* XRGB32 to packed RGB, which kitty calls f=24 */
static void
torgb(uchar *d, Memimage *m, Rectangle r)
{
	uchar *s;
	int x, y, w;

	w = Dx(r);
	for(y = r.min.y; y < r.max.y; y++){
		s = byteaddr(m, Pt(r.min.x, y));
		for(x = 0; x < w; x++){
			d[0] = s[2];
			d[1] = s[1];
			d[2] = s[0];
			d += 3;
			s += 4;
		}
	}
}

/*
 * the size of the tiles covering r: whole cells, at least
 * kt.tilesz pixels, no more than Maxtiles of them.
 * tilesz 0 makes the whole screen one image.
 */
static void
tiling(Rectangle r, int *tw, int *th)
{
	int w, h;

	if(kt.tilesz == 0){
		*tw = Dx(r) > 0 ? Dx(r) : 1;
		*th = Dy(r) > 0 ? Dy(r) : 1;
		return;
	}
	w = kt.cw * ((kt.tilesz+kt.cw-1)/kt.cw);
	h = kt.ch * ((kt.tilesz+kt.ch-1)/kt.ch);
	while(((Dx(r)+w-1)/w) * ((Dy(r)+h-1)/h) > Maxtiles){
		w *= 2;
		h *= 2;
	}
	*tw = w;
	*th = h;
}

/*
 * the writer: everything sent to the terminal goes through here,
 * at most fps frames a second; damage accumulates meanwhile.
 */
static void
outproc(void *v)
{
	static Buf b, pix;
	static Rectangle *box;
	static int *sent;
	static int nsent;
	Rectangle d[Ndirty], r, t, sc;
	int nd, full, shape, move, i, j, k, n, tw, th, ntx, nty, frames;
	uchar *mark, rgba[16*16*4], *p;
	Point pt, off, curoff;
	Cursorinfo c;
	ulong now, last, gap;
	char keys[256];
	struct timespec ts;
	struct timeval tv;

	USED(v);
	last = 0;
	mark = nil;
	curoff = ZP;
	for(;;){
		pthread_mutex_lock(&kt.lk);
		while(!kt.full && kt.ndirty == 0 && !kt.curshape && !kt.curmove && kt.pend.n == 0)
			pthread_cond_wait(&kt.cond, &kt.lk);
		/*
		 * keep at most one frame in flight: the terminal
		 * catches up and the damage meanwhile coalesces.
		 */
		if(kt.ackok && kt.sent - kt.acks > 1){
			gettimeofday(&tv, nil);
			ts.tv_sec = tv.tv_sec + 3;
			ts.tv_nsec = tv.tv_usec*1000;
			while(kt.sent - kt.acks > 1)
				if(pthread_cond_timedwait(&kt.cond, &kt.lk, &ts) == ETIMEDOUT){
					dbg("no ack from the terminal, flow control off");
					kt.ackok = 0;
					break;
				}
		}
		pthread_mutex_unlock(&kt.lk);

		gap = 1000/kt.fps;
		now = ticks();
		if(now - last < gap)
			osmsleep(gap - (now - last));
		last = ticks();

		qlock(&drawlock);
		pthread_mutex_lock(&kt.lk);
		full = kt.full;
		kt.full = 0;
		nd = kt.ndirty;
		memmove(d, kt.dirty, nd*sizeof d[0]);
		kt.ndirty = 0;
		shape = kt.curshape || full;
		kt.curshape = 0;
		move = kt.curmove || shape;
		kt.curmove = 0;
		pt = kt.curpt;
		b.n = 0;
		if(kt.sync)
			bputs(&b, "\033[?2026h");
		bput(&b, kt.pend.p, kt.pend.n);
		kt.pend.n = 0;
		if(full){
			kt.errors = 0;
			kt.lastfull = ticks();
		}
		frames = kt.frames;
		pthread_mutex_unlock(&kt.lk);

		if(gscreen == nil){
			qunlock(&drawlock);
			ttywrite(b.p, b.n);
			continue;
		}
		sc = gscreen->r;

		/*
		 * what to send: the damage within each tile, or
		 * all of it when it is new or cannot be edited.
		 */
		tiling(sc, &tw, &th);
		ntx = (Dx(sc)+tw-1)/tw;
		nty = (Dy(sc)+th-1)/th;
		mark = realloc(mark, ntx*nty);
		box = realloc(box, ntx*nty*sizeof box[0]);
		sent = realloc(sent, ntx*nty*sizeof sent[0]);
		memset(mark, 0, ntx*nty);
		if(full){
			nd = 1;
			d[0] = sc;
		}
		for(i = 0; i < nd; i++){
			r = d[i];
			if(!rectclip(&r, sc))
				continue;
			for(j = r.min.y/th; j <= (r.max.y-1)/th; j++)
				for(k = r.min.x/tw; k <= (r.max.x-1)/tw; k++){
					n = j*ntx + k;
					t = Rect(k*tw, j*th, (k+1)*tw, (j+1)*th);
					rectclip(&t, sc);
					if(full || !frames)
						box[n] = t;
					else if(!mark[n]){
						box[n] = r;
						rectclip(&box[n], t);
					}else{
						combinerect(&box[n], r);
						rectclip(&box[n], t);
					}
					mark[n] = 1;
				}
		}
		nsent = 0;
		for(i = 0; i < ntx*nty; i++)
			if(mark[i])
				sent[nsent++] = i;

		/* copy the pixels out quickly */
		n = 0;
		for(i = 0; i < nsent; i++){
			r = box[sent[i]];
			n += Dx(r)*Dy(r)*3;
		}
		pix.n = 0;
		bgrow(&pix, n);
		p = (uchar*)pix.p;
		for(i = 0; i < nsent; i++){
			r = box[sent[i]];
			torgb(p, gscreen, r);
			p += Dx(r)*Dy(r)*3;
		}
		qunlock(&drawlock);

		if(shape){
			lock(&cursor.lk);
			c = cursor;
			unlock(&cursor.lk);
			curoff = c.offset;
			kt.curhide = kt.optcursor == 0
				|| kt.optcursor == 2 && memcmp(c.clr, arrow.clr, sizeof c.clr) == 0
				&& memcmp(c.set, arrow.set, sizeof c.set) == 0;
			for(i = 0; i < 16*16; i++){
				j = (i/16)*2 + (i%16)/8;
				k = 0x80 >> (i%8);
				p = rgba + 4*i;
				if(c.set[j] & k){
					p[0] = p[1] = p[2] = 0;
					p[3] = 0xFF;
				}else if(c.clr[j] & k){
					p[0] = p[1] = p[2] = 0xFF;
					p[3] = 0xFF;
				}else
					p[0] = p[1] = p[2] = p[3] = 0;
			}
		}

		/* encode */
		if(full)
			bputs(&b, "\033_Ga=d,d=A,q=2\033\\");
		p = (uchar*)pix.p;
		for(i = 0; i < nsent; i++){
			r = box[sent[i]];
			n = Dx(r)*Dy(r)*3;
			t = rectaddpt(Rect(0, 0, tw, th), Pt((sent[i]%ntx)*tw, (sent[i]/ntx)*th));
			if(frames && !full)
				snprint(keys, sizeof keys, "a=f,i=%d,r=1,x=%d,y=%d,s=%d,v=%d,X=1,f=24,q=1",
					Idtile+sent[i], r.min.x-t.min.x, r.min.y-t.min.y, Dx(r), Dy(r));
			else{
				/* placing it again replaces the placement */
				off = cellat(&b, r.min);
				snprint(keys, sizeof keys, "a=T,i=%d,p=1,s=%d,v=%d,X=%d,Y=%d,C=1,z=0,f=24,q=1",
					Idtile+sent[i], Dx(r), Dy(r), off.x, off.y);
			}
			gfxdata(&b, keys, p, n);
			p += n;
		}

		if(shape && !kt.curhide){
			/* retransmission drops the old placement */
			snprint(keys, sizeof keys, "a=t,i=%d,s=16,v=16,f=32,q=1", Idcursor);
			gfxdata(&b, keys, rgba, sizeof rgba);
			kt.curshown = 0;
		}
		if(move && !kt.curhide){
			r = rectaddpt(Rect(0, 0, 16, 16), addpt(pt, curoff));
			j = r.min.x < 0 ? -r.min.x : 0;
			k = r.min.y < 0 ? -r.min.y : 0;
			if(rectclip(&r, sc)){
				off = cellat(&b, r.min);
				bprint(&b, "\033_Ga=p,i=%d,p=1,x=%d,y=%d,w=%d,h=%d,X=%d,Y=%d,C=1,z=1,q=1\033\\",
					Idcursor, j, k, Dx(r), Dy(r), off.x, off.y);
				kt.curshown = 1;
			}else if(kt.curshown){
				bprint(&b, "\033_Ga=d,d=i,i=%d,p=1,q=2\033\\", Idcursor);
				kt.curshown = 0;
			}
		}
		if(kt.curhide && kt.curshown && !full){
			bprint(&b, "\033_Ga=d,d=i,i=%d,p=1,q=2\033\\", Idcursor);
			kt.curshown = 0;
		}
		if(full && kt.curhide)
			kt.curshown = 0;
		if(kt.sync)
			bputs(&b, "\033[?2026l");
		if(kt.ackok && (nsent > 0 || shape || move)){
			bputs(&b, "\033[5n");
			pthread_mutex_lock(&kt.lk);
			kt.sent++;
			pthread_mutex_unlock(&kt.lk);
		}
		ttywrite(b.p, b.n);
		dbg("frame full=%d frames=%d tiles=%d/%d cursor=%d/%d bytes=%d", full, frames, nsent, ntx*nty, shape, move, b.n);
		if(b.cap > 1<<20){
			free(b.p);
			memset(&b, 0, sizeof b);
		}
	}
}

/* call with kt.lk held */
static void
kick(void)
{
	pthread_cond_broadcast(&kt.cond);
}

static vlong
area(Rectangle r)
{
	return (vlong)Dx(r)*Dy(r);
}

/* call with kt.lk held */
static void
adddirty(Rectangle r)
{
	Rectangle u;
	vlong a, ba;
	int i, bi;

Again:
	for(i = 0; i < kt.ndirty; i++){
		u = kt.dirty[i];
		combinerect(&u, r);
		if(area(u) <= area(kt.dirty[i]) + area(r) + Slack){
			r = u;
			kt.dirty[i] = kt.dirty[--kt.ndirty];
			goto Again;
		}
	}
	if(kt.ndirty == Ndirty){
		bi = 0;
		ba = -1;
		for(i = 0; i < kt.ndirty; i++){
			u = kt.dirty[i];
			combinerect(&u, r);
			a = area(u) - area(kt.dirty[i]);
			if(ba < 0 || a < ba){
				ba = a;
				bi = i;
			}
		}
		combinerect(&r, kt.dirty[bi]);
		kt.dirty[bi] = kt.dirty[--kt.ndirty];
		goto Again;
	}
	kt.dirty[kt.ndirty++] = r;
}

/* the terminal lost our pictures (or never got them): start over */
static void
resend(void)
{
	pthread_mutex_lock(&kt.lk);
	if(ticks() - kt.lastfull > 1000){
		kt.full = 1;
		kick();
	}
	pthread_mutex_unlock(&kt.lk);
}

/*
 * keyboard
 */

/* kitty functional key numbers (CSI number u) */
static struct {
	int	code;
	Rune	r;
} fkeys[] = {
	27,	Kesc,
	13,	'\n',
	9,	'\t',
	127,	Kbs,
	57358,	0,	/* caps lock: the text already has it */
	57359,	Kscroll,
	57360,	Knum,
	57361,	Kprint,
	57362,	Kbreak,
	57363,	0,	/* menu */
	57399,	'0',
	57400,	'1',
	57401,	'2',
	57402,	'3',
	57403,	'4',
	57404,	'5',
	57405,	'6',
	57406,	'7',
	57407,	'8',
	57408,	'9',
	57409,	'.',
	57410,	'/',
	57411,	'*',
	57412,	'-',
	57413,	'+',
	57414,	'\n',
	57415,	'=',
	57416,	',',
	57417,	Kleft,
	57418,	Kright,
	57419,	Kup,
	57420,	Kdown,
	57421,	Kpgup,
	57422,	Kpgdown,
	57423,	Khome,
	57424,	Kend,
	57425,	Kins,
	57426,	Kdel,
	57427,	0,	/* keypad begin */
	57428,	Kpause,	/* media play */
	57429,	Kpause,	/* media pause */
	57430,	Kpause,	/* media play/pause */
	57434,	Ksfwd,	/* media next */
	57435,	Ksbwd,	/* media previous */
	57438,	Kvoldn,
	57439,	Kvolup,
	57440,	Kmute,
	57441,	Kshift,
	57442,	Kctl,
	57443,	Kalt,
	57444,	Kmod4,
	57445,	0,	/* hyper */
	57446,	0,	/* meta */
	57447,	Kshift,
	57448,	Kctl,
	57449,	Kalt,
	57450,	Kmod4,
	57451,	0,
	57452,	0,
	57453,	Kaltgr,	/* iso level 3 shift */
	57454,	0,	/* iso level 5 shift */
};

/* final byte of CSI 1;mods X */
static Rune
letterkey(int c)
{
	switch(c){
	case 'A':	return Kup;
	case 'B':	return Kdown;
	case 'C':	return Kright;
	case 'D':	return Kleft;
	case 'E':	return 0;	/* keypad begin */
	case 'H':	return Khome;
	case 'F':	return Kend;
	case 'P':	return KF|1;
	case 'Q':	return KF|2;
	case 'R':	return KF|3;
	case 'S':	return KF|4;
	}
	return 0;
}

/* CSI number;mods ~ */
static Rune
tildekey(int n)
{
	switch(n){
	case 1:	case 7:	return Khome;
	case 2:	return Kins;
	case 3:	return Kdel;
	case 4:	case 8:	return Kend;
	case 5:	return Kpgup;
	case 6:	return Kpgdown;
	case 11:	return KF|1;
	case 12:	return KF|2;
	case 13:	return KF|3;
	case 14:	return KF|4;
	case 15:	return KF|5;
	case 17:	return KF|6;
	case 18:	return KF|7;
	case 19:	return KF|8;
	case 20:	return KF|9;
	case 21:	return KF|10;
	case 23:	return KF|11;
	case 24:	return KF|12;
	}
	return 0;
}

static Rune
codekey(int code)
{
	int i;

	for(i = 0; i < nelem(fkeys); i++)
		if(fkeys[i].code == code)
			return fkeys[i].r;
	if(code >= 57376 && code <= 57398)
		return 0;	/* F13 and up, plan 9 has none */
	if(code >= 0xE000 && code < 0xF900)
		return 0;	/* other private use: media keys and such */
	return code;
}

static int
ismod(Rune r)
{
	return r == Kshift || r == Kctl || r == Kalt || r == Kmod4 || r == Kaltgr;
}

static void
keydown(int id, Rune r, int repeat)
{
	int i;

	if(r == 0)
		return;
	if(kt.dead != nil)
		quit();
	for(i = 0; i < kt.ndown; i++)
		if(kt.down[i].id == id){
			if(kt.down[i].r != r)
				kbdkey(kt.down[i].r, 0);
			kt.down[i].r = r;
			goto Send;
		}
	if(repeat && ismod(r))
		return;
	if(kt.ndown < Ndown){
		kt.down[kt.ndown].id = id;
		kt.down[kt.ndown].r = r;
		kt.ndown++;
	}
Send:
	dbg("key %d down %#x", id, r);
	kbdkey(r, 1);
}

static void
keyup(int id, Rune r)
{
	int i;

	for(i = 0; i < kt.ndown; i++)
		if(kt.down[i].id == id){
			r = kt.down[i].r;
			kt.down[i] = kt.down[--kt.ndown];
			break;
		}
	dbg("key %d up %#x", id, r);
	if(r == 0)
		return;
	kbdkey(r, 0);
	if(r == Kalt && kt.altmouse){
		/* the alt of an alt-click must not start a compose */
		kt.altmouse = 0;
		kbdkey(Kalt, 1);
		kbdkey(Kalt, 0);
	}
}

/* forget everything held: we may never see the releases */
static void
releaseall(void)
{
	int alt;

	alt = 0;
	while(kt.ndown > 0){
		kt.ndown--;
		if(kt.down[kt.ndown].r == Kalt)
			alt = 1;
		kbdkey(kt.down[kt.ndown].r, 0);
	}
	if(alt){
		kbdkey(Kalt, 1);
		kbdkey(Kalt, 0);
	}
	if(kt.buttons){
		kt.buttons = 0;
		absmousetrack(kt.curpt.x, kt.curpt.y, 0, ticks());
	}
}

/* a key typed whole, as in legacy input or a paste */
static void
typekey(Rune r)
{
	if(r == 0)
		return;
	if(kt.dead != nil)
		quit();
	kbdkey(r, 1);
	kbdkey(r, 0);
}

static int
field(Csi *c, int f, int s, int def)
{
	if(f >= c->nf || s >= c->ns[f] || c->v[f][s] < 0)
		return def;
	return c->v[f][s];
}

/*
 * CSI key[:shifted[:base]] ; mods[:event] ; text u
 * and the legacy-looking CSI 1;mods:event A..S and CSI n;mods:event ~
 */
static void
kittykey(Csi *c)
{
	int code, shifted, mods, ev, i, id;
	Rune r, t;

	code = field(c, 0, 0, 1);
	shifted = field(c, 0, 1, 0);
	mods = field(c, 1, 0, 1) - 1;
	ev = field(c, 1, 1, Epress);
	if(mods < 0)
		mods = 0;
	switch(c->final){
	case 'u':
		r = codekey(code);
		id = code;
		break;
	case '~':
		r = tildekey(code);
		id = 0x110000 + code;
		break;
	default:
		r = letterkey(c->final);
		id = 0x120000 + c->final;
		break;
	}
	if(ev == Erelease){
		keyup(id, r);
		return;
	}
	if(r == 0)
		return;
	if(c->final == 'u' && !ismod(r)){
		/* prefer what the terminal says the key types */
		t = field(c, 2, 0, 0);
		if(t >= 0x20 && t != 0x7f && (mods & Mctl) == 0){
			keydown(id, t, ev == Erepeat);
			for(i = 1; i < c->ns[2]; i++)
				typekey(field(c, 2, i, 0));
			return;
		}
		if((mods & Mshift) && shifted > 0)
			r = shifted;
		if((mods & Mctl) && r < 0x80 && r != Kesc && r != Kbs && r != '\n' && r != '\t')
			r &= 0x1f;
	}
	keydown(id, r, ev == Erepeat);
}

/* input from terminals without the kitty keyboard protocol */
static void
legacykey(Csi *c)
{
	Rune r;

	if(c->final == '~')
		r = tildekey(field(c, 0, 0, 0));
	else
		r = letterkey(c->final);
	typekey(r);
}

/*
 * mouse: CSI < b;x;y M or m, x and y in pixels from mode 1016
 */
static void
sgrmouse(Csi *c, int release)
{
	int b, x, y, bit;

	b = field(c, 0, 0, 0);
	x = field(c, 1, 0, 0);
	y = field(c, 2, 0, 0);
	if(b & 256)
		return;	/* kitty: the pointer left the window */
	if(!kt.pixels){
		/* cells, 1-based: aim at the middle of the cell */
		x = (x-1)*kt.cw + kt.cw/2;
		y = (y-1)*kt.ch + kt.ch/2;
	}
	kt.curpt = Pt(x, y);
	if(b & 64){
		if(release || (b & 32))
			return;
		switch(b & 3){
		case 0:	bit = 8; break;
		case 1:	bit = 16; break;
		default:	return;	/* horizontal wheel */
		}
		absmousetrack(x, y, kt.buttons|bit, ticks());
		absmousetrack(x, y, kt.buttons, ticks());
		goto Move;
	}
	if((b & 128) == 0 && (b & 32) == 0 && (b & 3) != 3){
		b &= 3;
		if(release)
			kt.buttons &= ~kt.tbutton[b];
		else{
			bit = 1<<b;
			if(b == 0 && kt.emu){
				if(c->v[0][0] & 8){	/* alt-click: middle */
					bit = 2;
					kt.altmouse = 1;
				}else if(c->v[0][0] & 16)	/* ctl-click: right */
					bit = 4;
			}
			kt.tbutton[b] = bit;
			kt.buttons |= bit;
		}
	}
	absmousetrack(x, y, kt.buttons, ticks());
Move:
	if(kt.optcursor){
		pthread_mutex_lock(&kt.lk);
		kt.curpt = Pt(x, y);
		kt.curmove = 1;
		kick();
		pthread_mutex_unlock(&kt.lk);
	}
}

/*
 * what the terminal says
 */
enum
{
	Sground,
	Sesc,
	Scsi,
	Sss3,
	Sstr,		/* OSC, APC, DCS: up to ST or BEL */
	Sstresc,
	Spaste,
};

static struct {
	int	state;
	Csi	csi;
	int	num;		/* parameter being parsed, -1 if empty */
	int	strkind;	/* ] _ P */
	Buf	str;
	Buf	paste;
	int	overflow;
	uchar	utf[UTFmax];
	int	nutf;
} in;

/* where an escape timed out or a sequence ended: forget partial runes */
static void
legacyrune(Rune r)
{
	switch(r){
	case '\r':
		r = '\n';
		break;
	case 0x7f:
		r = Kbs;
		break;
	case 0:
		return;
	}
	typekey(r);
}

static void
groundbyte(int c)
{
	Rune r;

	if(c < 0x80 && in.nutf == 0){
		legacyrune(c);
		return;
	}
	in.utf[in.nutf++] = c;
	if(fullrune((char*)in.utf, in.nutf) || in.nutf == UTFmax){
		chartorune(&r, (char*)in.utf);
		in.nutf = 0;
		legacyrune(r);
	}
}

static void
pasted(void)
{
	char *p, *e;
	Rune r;
	int n, cr;

	p = in.paste.p;
	e = p + in.paste.n;
	cr = 0;
	while(p < e){
		if(!fullrune(p, e-p))
			break;
		n = chartorune(&r, p);
		p += n;
		if(r == '\n' && cr){
			cr = 0;
			continue;
		}
		cr = r == '\r';
		if(r == '\r')
			r = '\n';
		if(r < 0x20 && r != '\n' && r != '\t')
			continue;
		typekey(r);
	}
	in.paste.n = 0;
}

static void
gfxreply(char *s)
{
	char *msg, *f[16], *p;
	int i, n, id;

	if((msg = strchr(s, ';')) == nil)
		return;
	*msg++ = 0;
	id = 0;
	n = getfields(s, f, nelem(f), 1, ",");
	for(i = 0; i < n; i++)
		if(f[i][0] == 'i' && f[i][1] == '=')
			id = strtoul(f[i]+2, nil, 10);
	if((id & 0xFFFF0000) != Idbase)
		return;
	if(kt.probing){
		if(strcmp(msg, "OK") != 0){
			dbg("probe %d: %s", id-Idbase, msg);
			return;
		}
		switch(id){
		case Idprobe:	kt.gfx = 1; break;
		case Idprobe+1:	kt.shm = 1; break;
		case Idprobe+2:	kt.frames = 1; break;
		}
		return;
	}
	if(strcmp(msg, "OK") == 0)
		return;
	if((p = strchr(msg, ':')) != nil)
		*p = 0;
	dbg("graphics error: image %d: %s", id-Idbase, msg);
	pthread_mutex_lock(&kt.lk);
	kt.errors++;
	pthread_mutex_unlock(&kt.lk);
	if(strcmp(msg, "ENOENT") == 0 || strcmp(msg, "ENOPARENT") == 0)
		resend();
}

static void
oscreply(char *s)
{
	char *p;
	int n;

	if(strncmp(s, "52;", 3) != 0)
		return;
	if((p = strchr(s+3, ';')) == nil)
		return;
	p++;
	n = strlen(p);
	pthread_mutex_lock(&kt.lk);
	free(kt.clip);
	kt.clip = malloc(n+1);
	if(kt.clip != nil){
		n = dec64((uchar*)kt.clip, n, p, n);
		if(n < 0)
			n = 0;
		kt.clip[n] = 0;
	}
	kt.clipwait = 0;
	pthread_cond_broadcast(&kt.cond);
	pthread_mutex_unlock(&kt.lk);
}

static void
strdone(void)
{
	bput(&in.str, "", 1);
	if(in.overflow){
		in.overflow = 0;
		return;
	}
	switch(in.strkind){
	case '_':
		if(in.str.p[0] == 'G')
			gfxreply(in.str.p+1);
		break;
	case ']':
		oscreply(in.str.p);
		break;
	case 'P':
		if(strncmp(in.str.p, ">|", 2) == 0)
			strecpy(kt.name, kt.name+sizeof kt.name, in.str.p+2);
		break;
	}
}

static void	resize(int, int, int, int);

static void
csidone(void)
{
	Csi *c;
	int v0, v1;

	c = &in.csi;
	v0 = field(c, 0, 0, -1);
	v1 = field(c, 1, 0, -1);
	switch(c->prefix){
	case '<':
		if(c->final == 'M' || c->final == 'm')
			sgrmouse(c, c->final == 'm');
		return;
	case '?':
		switch(c->final){
		case 'u':
			kt.kbd = 1;
			break;
		case 'c':
			kt.da = 1;
			break;
		case 'y':
			if(c->inter != '$')
				break;
			if(v1 < 1 || v1 > 4)
				break;
			switch(v0){
			case 2026:	kt.sync = 1; break;
			case 1016:	kt.pixels = 1; break;
			case 2048:	kt.inband = 1; break;
			}
			break;
		}
		return;
	case 0:
		break;
	default:
		return;
	}
	if(c->inter != 0)
		return;
	switch(c->final){
	case 't':
		switch(v0){
		case 4:
			kt.probeh = v1;
			kt.probew = field(c, 2, 0, 0);
			break;
		case 6:
			kt.probech = v1;
			kt.probecw = field(c, 2, 0, 0);
			break;
		case 48:
			resize(field(c, 2, 0, 0), v1, field(c, 4, 0, 0), field(c, 3, 0, 0));
			break;
		}
		return;
	case 'n':
		if(v0 == 0){
			pthread_mutex_lock(&kt.lk);
			kt.acks++;
			pthread_cond_broadcast(&kt.cond);
			pthread_mutex_unlock(&kt.lk);
		}
		return;
	case 'I':
		return;
	case 'O':
		releaseall();
		return;
	case 'Z':
		typekey('\t');
		return;
	case '~':
		if(v0 == 200){
			in.state = Spaste;
			in.paste.n = 0;
			return;
		}
		break;
	case 'u':
		kittykey(c);
		return;
	}
	if(c->final == '~' || letterkey(c->final) != 0){
		if(kt.kbd)
			kittykey(c);
		else
			legacykey(c);
	}
}

static void
csibyte(int c)
{
	Csi *s;

	s = &in.csi;
	if(c >= '0' && c <= '9'){
		if(in.num < 0)
			in.num = 0;
		if(in.num < 1000000)
			in.num = in.num*10 + c-'0';
		return;
	}
	if(c == ':' || c == ';'){
		if(s->nf == 0)
			s->nf = 1;
		if(s->nf <= Nfield && s->ns[s->nf-1] < Nsub)
			s->v[s->nf-1][s->ns[s->nf-1]++] = in.num;
		in.num = -1;
		if(c == ';')
			s->nf++;
		return;
	}
	if(c >= '<' && c <= '?'){
		s->prefix = c;
		return;
	}
	if(c >= 0x20 && c <= 0x2f){
		s->inter = c;
		return;
	}
	if(c >= 0x40 && c <= 0x7e){
		if(s->nf == 0)
			s->nf = 1;
		if(s->nf <= Nfield && s->ns[s->nf-1] < Nsub)
			s->v[s->nf-1][s->ns[s->nf-1]++] = in.num;
		if(s->nf > Nfield)
			s->nf = Nfield;
		s->final = c;
		in.state = Sground;
		csidone();
		return;
	}
	in.state = Sground;	/* garbage */
}

static void
inbyte(int c)
{
	static char endpaste[] = "\033[201~";
	char b;

	b = c;

	switch(in.state){
	case Sground:
		if(c == 0x1b){
			in.state = Sesc;
			return;
		}
		groundbyte(c);
		return;
	case Sesc:
		switch(c){
		case '[':
			in.state = Scsi;
			memset(&in.csi, 0, sizeof in.csi);
			in.num = -1;
			return;
		case 'O':
			in.state = Sss3;
			return;
		case ']':
		case '_':
		case 'P':
			in.state = Sstr;
			in.strkind = c;
			in.str.n = 0;
			in.overflow = 0;
			return;
		case 0x1b:
			typekey(Kesc);
			return;
		}
		/* alt-something in legacy mode: plan 9 has no meta */
		in.state = Sground;
		typekey(Kesc);
		groundbyte(c);
		return;
	case Scsi:
		csibyte(c);
		return;
	case Sss3:
		in.state = Sground;
		typekey(letterkey(c));
		return;
	case Sstr:
		if(c == 0x07){
			in.state = Sground;
			strdone();
			return;
		}
		if(c == 0x1b){
			in.state = Sstresc;
			return;
		}
		if(in.str.n < Maxstr)
			bput(&in.str, &b, 1);
		else
			in.overflow = 1;
		return;
	case Sstresc:
		if(c == '\\'){
			in.state = Sground;
			strdone();
			return;
		}
		/* not ST: an aborted string, take the escape anew */
		in.state = Sesc;
		inbyte(c);
		return;
	case Spaste:
		bput(&in.paste, &b, 1);
		if(in.paste.n >= sizeof endpaste-1
		&& memcmp(in.paste.p+in.paste.n-(sizeof endpaste-1), endpaste, sizeof endpaste-1) == 0){
			in.paste.n -= sizeof endpaste-1;
			in.state = Sground;
			pasted();
		}
		return;
	}
}

/*
 * geometry
 */
static void
resize(int cols, int rows, int w, int h)
{
	Rectangle r;
	int cw, ch, same;

	if(cols <= 0 || rows <= 0)
		return;
	if(w <= 0 || h <= 0){
		if(kt.cw <= 0 || kt.ch <= 0)
			return;
		w = cols*kt.cw;
		h = rows*kt.ch;
	}
	cw = w/cols;
	ch = h/rows;
	if(cw <= 0 || ch <= 0)
		return;
	pthread_mutex_lock(&kt.lk);
	if(cols == kt.cols && rows == kt.rows && cw == kt.cw && ch == kt.ch){
		pthread_mutex_unlock(&kt.lk);
		return;
	}
	kt.cols = cols;
	kt.rows = rows;
	kt.cw = cw;
	kt.ch = ch;
	/* only whole cells: placements are addressed by cell */
	r = Rect(0, 0, cols*cw, rows*ch);
	same = gscreen == nil || eqrect(r, gscreen->r);
	if(same){
		/* same pixels, different cells: placements move */
		kt.full = 1;
		kick();
	}
	pthread_mutex_unlock(&kt.lk);
	dbg("size %dx%d cells of %dx%d", cols, rows, cw, ch);
	/*
	 * even if r is the current size: it replaces any other size
	 * still pending in resizeproc (docker reports stale sizes).
	 * when it differs, screensize will send it all.
	 */
	if(gscreen != nil)
		screenresize(r);
}

static void
winsize(void)
{
	struct winsize ws;

	if(ioctl(kt.fd, TIOCGWINSZ, &ws) < 0)
		return;
	if(ws.ws_xpixel == 0 || ws.ws_ypixel == 0){
		ws.ws_xpixel = 0;
		ws.ws_ypixel = 0;
		if(kt.probecw > 0 && kt.probech > 0){
			ws.ws_xpixel = ws.ws_col*kt.probecw;
			ws.ws_ypixel = ws.ws_row*kt.probech;
		}else if(kt.probew > 0 && kt.probeh > 0){
			ws.ws_xpixel = kt.probew;
			ws.ws_ypixel = kt.probeh;
		}
	}
	resize(ws.ws_col, ws.ws_row, ws.ws_xpixel, ws.ws_ypixel);
}

/*
 * the reader
 */
static int
readtty(int timeout)
{
	uchar buf[8192];
	char c;
	int i, n, intty, insig;

	n = waitfd(kt.fd, kt.sigfd[0], timeout, &intty, &insig);
	if(n < 0)
		return errno == EINTR ? 0 : -1;
	if(n == 0)
		return 0;
	if(insig){
		while(read(kt.sigfd[0], &c, 1) == 1)
			;
		/* in-band reports (mode 2048) are exact and never stale */
		if(!kt.probing && !kt.inband)
			winsize();
	}
	if(intty){
		n = read(kt.fd, buf, sizeof buf);
		if(n < 0)
			return errno == EINTR || errno == EAGAIN ? 0 : -1;
		if(n == 0)
			return -1;
		for(i = 0; i < n; i++)
			inbyte(buf[i]);
	}
	return 1;
}

/*
 * leave without exits: its handlers would panic and hang.
 * the host atexit handler gives back the terminal.
 */
static void
quit(void)
{
	ttyexit();
	if(kt.dead != nil && strcmp(kt.dead, "ending") != 0)
		fprintf(stderr, "drawterm: panic: %s\n", kt.dead);
	exit(kt.dead != nil && strcmp(kt.dead, "ending") != 0);
}

/*
 * the kernel panicked and hangs to keep the message on the
 * screen, as it would in a window.  the end of the session
 * also comes this way, as panic("ending"): just leave then;
 * otherwise wait for a key so the message can be read.
 */
static void
dying(void)
{
	char msg[256], *p, *e;
	int n;

	osmsleep(200);	/* let panic draw */
	ilock(&kmesg.lk);
	n = kmesg.n;
	if(n > sizeof kmesg.buf)
		n = sizeof kmesg.buf;
	memmove(msg, "?", 2);
	for(p = kmesg.buf+n-7; p >= kmesg.buf; p--)
		if(memcmp(p, "panic: ", 7) == 0){
			p += 7;
			e = memchr(p, '\n', kmesg.buf+n-p);
			if(e == nil)
				e = kmesg.buf+n;
			if(e-p >= sizeof msg)
				e = p+sizeof msg-1;
			memmove(msg, p, e-p);
			msg[e-p] = 0;
			break;
		}
	iunlock(&kmesg.lk);
	kt.dead = strdup(msg);
	dbg("panic: %s", kt.dead);
	if(strcmp(kt.dead, "ending") == 0)
		quit();
	ttyprint("\033]2;drawterm: %s (press a key)\033\\", kt.dead);
}

static void
inproc(void *v)
{
	int n, t;

	USED(v);
	for(;;){
		/*
		 * a lone escape is the escape key, unless the protocol
		 * says otherwise.  look out for a panic now and then.
		 */
		t = 500;
		if(in.state == Sesc)
			t = 30;
		n = readtty(t);
		if(n < 0)
			break;
		if(n == 0 && in.state == Sesc){
			in.state = Sground;
			typekey(Kesc);
		}
		if(panicking && kt.dead == nil)
			dying();
	}
	/* the terminal went away */
	ttyrestore();
	exit(0);
}

/*
 * find out what the terminal can do: it answers in order,
 * so once the device attributes come back we know.
 */
static void
probe(void)
{
	static uchar zero[3];
	ulong t0;
	char *name;
	Buf b;

	memset(&b, 0, sizeof b);
	kt.probing = 1;
	bprint(&b, "\033_Gi=%d,s=1,v=1,a=q,t=d,f=24;AAAA\033\\", Idprobe);
	if(kt.optshm != 0 && (name = shmput(zero, sizeof zero)) != nil){
		bprint(&b, "\033_Gi=%d,s=1,v=1,a=q,t=s,f=24,S=3;", Idprobe+1);
		b64(&b, (uchar*)name, strlen(name));
		bputs(&b, "\033\\");
	}
	if(kt.optframes != 0){
		bprint(&b, "\033_Ga=t,i=%d,s=1,v=1,f=24,t=d,q=1;AAAA\033\\", Idprobe+2);
		bprint(&b, "\033_Ga=f,i=%d,r=1,x=0,y=0,s=1,v=1,f=24,t=d;AAAA\033\\", Idprobe+2);
		bprint(&b, "\033_Ga=d,d=I,i=%d,q=2\033\\", Idprobe+2);
	}
	bputs(&b, "\033[?u\033[?2026$p\033[?1016$p\033[?2048$p\033[14t\033[16t\033[>q\033[5n\033[c");
	ttywrite(b.p, b.n);
	free(b.p);

	t0 = ticks();
	while(!kt.da && ticks()-t0 < 5000)
		if(readtty(100) < 0)
			break;
	kt.probing = 0;
	kt.ackok = kt.acks > 0;
	kt.acks = 0;

	if(kt.optshm >= 0)
		kt.shm = kt.optshm;
	if(kt.optframes >= 0)
		kt.frames = kt.optframes;
	dbg("terminal '%s': da=%d gfx=%d shm=%d frames=%d kbd=%d pixels=%d sync=%d inband=%d ack=%d cell=%dx%d text=%dx%d (%lud ms)",
		kt.name, kt.da, kt.gfx, kt.shm, kt.frames, kt.kbd, kt.pixels, kt.sync, kt.inband, kt.ackok,
		kt.probecw, kt.probech, kt.probew, kt.probeh, ticks()-t0);
	if(!kt.da)
		panic("kitty: terminal does not answer");
	if(!kt.gfx)
		panic("kitty: %s does not speak the kitty graphics protocol",
			kt.name[0] ? kt.name : "terminal");
	if(kt.kbd)
		ttyprint("\033[>%du", Kbdflags);
	ttyputs("\033]2;drawterm\033\\");
}

/*
 * $DTKITTY: comma separated
 *	shm, noshm	pixels via shared memory, or never (default: if it works)
 *	frames, noframes	edit tiles in place, or send damaged tiles whole (default: if it works)
 *	fps=n	frames a second at most (60)
 *	z=n	zlib level for inline pixels, 0 for none (1)
 *	tile=n	tile size in pixels, 0 for one image (256)
 *	cursor=n	0: never draw the plan 9 cursor, 1: always, 2: unless it is the arrow (2)
 *	emu	alt-click is button 2, ctl-click is button 3
 *	clipread	read the host clipboard (OSC 52) on paste
 *	debug=file	log what happens
 */
static void
options(void)
{
	char *s, *f[32], *p;
	int i, n;

	kt.optshm = -1;
	kt.optframes = -1;
	kt.fps = 60;
	kt.zlevel = 1;
	kt.tilesz = Tilesz;
	kt.optcursor = 2;
	if((s = getenv("DTKITTY")) == nil)
		return;
	s = strdup(s);
	n = getfields(s, f, nelem(f), 1, ", ");
	for(i = 0; i < n; i++){
		if((p = strchr(f[i], '=')) != nil)
			*p++ = 0;
		if(strcmp(f[i], "shm") == 0)
			kt.optshm = 1;
		else if(strcmp(f[i], "noshm") == 0)
			kt.optshm = 0;
		else if(strcmp(f[i], "frames") == 0)
			kt.optframes = 1;
		else if(strcmp(f[i], "noframes") == 0)
			kt.optframes = 0;
		else if(strcmp(f[i], "emu") == 0)
			kt.emu = 1;
		else if(strcmp(f[i], "clipread") == 0)
			kt.clipread = 1;
		else if(p == nil)
			continue;
		else if(strcmp(f[i], "fps") == 0)
			kt.fps = atoi(p);
		else if(strcmp(f[i], "z") == 0)
			kt.zlevel = atoi(p);
		else if(strcmp(f[i], "tile") == 0)
			kt.tilesz = atoi(p);
		else if(strcmp(f[i], "cursor") == 0)
			kt.optcursor = atoi(p);
		else if(strcmp(f[i], "debug") == 0)
			kt.debugfd = open(p, O_WRONLY|O_CREAT|O_APPEND|O_CLOEXEC, 0600);
	}
	if(kt.fps < 1)
		kt.fps = 1;
	if(kt.fps > 1000)
		kt.fps = 1000;
	if(kt.zlevel > 9)
		kt.zlevel = 9;
	if(kt.tilesz != 0 && kt.tilesz < 16)
		kt.tilesz = 16;
	free(s);
}

/*
 * the interface to the rest of drawterm
 */
void
screeninit(void)
{
	Rectangle r;

	options();
	hideoutput();
	ttyinit();
	probe();
	winsize();
	if(kt.cols <= 0 || kt.cw <= 0)
		panic("kitty: cannot tell the size of the terminal in pixels");
	memimageinit();
	r = Rect(0, 0, kt.cols*kt.cw, kt.rows*kt.ch);
	screensize(r, XRGB32);
	if(gscreen == nil)
		panic("kitty: no memory for the screen");
	gscreen->clipr = r;
	kproc("kittyout", outproc, nil);
	kproc("kittyin", inproc, nil);

	qlock(&drawlock);
	terminit();
	flushmemscreen(gscreen->clipr);
	qunlock(&drawlock);
}

void
screensize(Rectangle r, ulong chan)
{
	Memimage *i;

	USED(chan);
	if((i = allocmemimage(r, XRGB32)) == nil)
		return;
	memfillcolor(i, DBlack);
	if(gscreen != nil)
		freememimage(gscreen);
	gscreen = i;
	gscreen->clipr = ZR;
	pthread_mutex_lock(&kt.lk);
	kt.full = 1;
	kt.ndirty = 0;
	kick();
	pthread_mutex_unlock(&kt.lk);
}

void
flushmemscreen(Rectangle r)
{
	if(gscreen == nil || !rectclip(&r, gscreen->r))
		return;
	pthread_mutex_lock(&kt.lk);
	if(!kt.full){
		adddirty(r);
		kick();
	}
	pthread_mutex_unlock(&kt.lk);
}

Memdata*
attachscreen(Rectangle *r, ulong *chan, int *depth, int *width, int *softscreen)
{
	*r = gscreen->clipr;
	*chan = gscreen->chan;
	*depth = gscreen->depth;
	*width = gscreen->width;
	*softscreen = 1;

	gscreen->data->ref++;
	return gscreen->data;
}

void
getcolor(ulong i, ulong *r, ulong *g, ulong *b)
{
	ulong v;

	v = cmap2rgb(i);
	*r = (v>>16)&0xFF;
	*g = (v>>8)&0xFF;
	*b = v&0xFF;
}

void
setcolor(ulong i, ulong r, ulong g, ulong b)
{
	USED(i, r, g, b);
}

void
setcursor(void)
{
	pthread_mutex_lock(&kt.lk);
	kt.curshape = 1;
	kick();
	pthread_mutex_unlock(&kt.lk);
}

/* the terminal's pointer cannot be moved: just draw ours there */
void
mouseset(Point p)
{
	pthread_mutex_lock(&kt.lk);
	kt.curpt = p;
	kt.curmove = 1;
	kick();
	pthread_mutex_unlock(&kt.lk);
}

char*
clipread(void)
{
	struct timespec ts;
	struct timeval tv;
	char *s;

	pthread_mutex_lock(&kt.lk);
	if(kt.clipread){
		bputs(&kt.pend, "\033]52;c;?\033\\");
		kt.clipwait = 1;
		kick();
		gettimeofday(&tv, nil);
		ts.tv_sec = tv.tv_sec + 2;
		ts.tv_nsec = tv.tv_usec*1000;
		while(kt.clipwait)
			if(pthread_cond_timedwait(&kt.cond, &kt.lk, &ts) == ETIMEDOUT)
				break;
		if(!kt.clipwait && kt.clip != nil){
			free(kt.snarf);
			kt.snarf = kt.clip;
			kt.clip = nil;
		}
		kt.clipwait = 0;
	}
	s = kt.snarf != nil ? strdup(kt.snarf) : nil;
	pthread_mutex_unlock(&kt.lk);
	return s;
}

int
clipwrite(char *buf)
{
	int n;

	n = strlen(buf);
	pthread_mutex_lock(&kt.lk);
	free(kt.snarf);
	kt.snarf = strdup(buf);
	bputs(&kt.pend, "\033]52;c;");
	b64(&kt.pend, (uchar*)buf, n);
	bputs(&kt.pend, "\033\\");
	kick();
	pthread_mutex_unlock(&kt.lk);
	return 0;
}

void
guimain(void)
{
	cpubody();
}

