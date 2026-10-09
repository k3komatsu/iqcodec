// iqcodec command line: lossless compression of IQ capture files.
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "crc32c.h"
#include "iqc.h"

#define VERSION "0.2.2"
#define CHUNK (1 << 21)           // complex samples per independently coded chunk
#define K_ORDER 24
#define LEAF 8192
#define PREC 11

// File: "IQCD" u8 version, u8 fmt, u8 K, u8 prec, u32 leaf, f32 scale; then chunks:
//   u32 nsamples (0 = end), u8 shift, 3 reserved bytes, u32 nbytes, u32 crc32c of the samples, payload
// end marker: u32 0, u64 total samples. (Version 2; version 1 of iqcodec 0.1.0 is not supported.)
static const char MAGIC[4] = {'I', 'Q', 'C', 'D'};
enum { CMD_C, CMD_D, CMD_T, CMD_I };

static void usage(FILE *f) {
    fprintf(f,
        "iqcodec " VERSION " - lossless compression for IQ captures\n\n"
        "usage: iqcodec c [options] INPUT [OUTPUT]  compress (default OUTPUT: INPUT.iqc)\n"
        "       iqcodec d [options] INPUT [OUTPUT]  decompress, checksums verified (default: INPUT without .iqc)\n"
        "       iqcodec t [options] INPUT           test: decompress and verify checksums, no output\n"
        "       iqcodec i INPUT                     info: format, samples, size (structure checked, data not decoded)\n"
        "       (use - for stdin / stdout)\n\n"
        "options:\n"
        "  -f FMT    input sample format for c: fc32 (complex float32, default) or sc16 (complex int16)\n"
        "  -s SCALE  fc32 values are int16 / SCALE (default 32767, as UHD converts sc16 to fc32)\n"
        "  -l        lossy: allow fc32 input that is not exactly int16 / SCALE (it is quantized)\n"
        "  -t        with c: decode every chunk right after encoding and compare with the input\n"
        "  -j N      threads (default: number of CPUs, at most 8)\n"
        "  -v        print statistics\n"
        "  --rm      remove INPUT after success (c implies -t)\n"
        "  --skip N  with d / t: start at sample N (complex samples, counted from 0)\n"
        "  --count N with d / t: at most N samples; only the chunks covering the range are read\n"
        "  --salvage with d / t: keep going past damage: a corrupt chunk becomes zeros, a truncated or damaged\n"
        "            stream ends at the last good chunk (exit status 3 when anything was lost; OUTPUT is kept)\n"
        "  -h, -V    help, version\n\n"
        "A file OUTPUT is written to a temporary file and renamed when complete, so an existing file is\n"
        "replaced only on success (stdout, pipes and devices receive data as it is decoded). INPUT and\n"
        "OUTPUT must not be the same file. A default OUTPUT never replaces an existing file.\n"
        "Options go before INPUT / OUTPUT.\n");
}

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static size_t read_full(FILE *f, void *buf, size_t n) {
    size_t got = 0;
    while (got < n) { size_t r = fread((char *)buf + got, 1, n - got, f); if (!r) break; got += r; }
    return got;
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

// Progress on a terminal stderr: one line, rewritten at most 4 times a second and cleared before any message.
static int prog_on, prog_shown;
static void progress_end(void) { if (prog_shown) { fputs("\r\033[K", stderr); prog_shown = 0; } }
static void msg(const char *fmt, ...) {
    va_list ap;
    progress_end();
    fputs("iqcodec: ", stderr);
    va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
}
// done / total: bytes of INPUT (total 0: unknown); raw: sample bytes so far, for the rate
static void progress(const char *what, uint64_t done, uint64_t total, uint64_t raw, double t0) {
    static double last;
    double t = now(), mb = done / 1e6, rate = raw / 1e6 / (t - t0 + 1e-9);
    if (!prog_on || t - last < 0.25) return;
    last = t;
    if (total) fprintf(stderr, "\r%s %.0f%% (%.0f of %.0f MB), %.0f MB/s\033[K", what, 100.0 * done / total, mb, total / 1e6, rate);
    else fprintf(stderr, "\r%s %.0f MB, %.0f MB/s\033[K", what, mb, rate);
    prog_shown = 1;
}

// ---------------- output: same-file guard, temporary file + rename ----------------
static char tmp_path[PATH_MAX + 32];   // non-empty while a temporary output exists
static char dst_path[PATH_MAX];        // regular-file OUTPUT after following symlinks (rename target)

static const int fatal_sigs[] = {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGXFSZ};
static volatile sig_atomic_t graceful_int;   // compressing a stream: Ctrl-C finishes the file instead of aborting
static volatile sig_atomic_t n_int;          // SIGINTs received in graceful mode
static volatile sig_atomic_t in_fd = -1;     // that stream

static void on_signal(int sig) {
    if (sig == SIGINT && graceful_int && n_int < 2) {
        static const char m1[] = "\niqcodec: interrupted: finishing when the input ends "
                                 "(Ctrl-C again: stop reading now; a third time: abort)\n";
        static const char m2[] = "\niqcodec: stopped reading, finishing the file\n";
        n_int++;
        int e = errno, z;
        ssize_t r = n_int == 1 ? write(STDERR_FILENO, m1, sizeof m1 - 1) : write(STDERR_FILENO, m2, sizeof m2 - 1);
        // second: the input becomes /dev/null, so a read blocked on a stalled producer (resumed by SA_RESTART)
        // and every later read see EOF
        if (n_int == 2 && (z = open("/dev/null", O_RDONLY)) >= 0) { dup2(z, in_fd); close(z); }
        (void)r; errno = e;
        return;
    }
    if (tmp_path[0]) unlink(tmp_path);
    signal(sig, SIG_DFL);
    raise(sig);
}
// Cleans up on fatal signals, except those the caller ignores (nohup, background jobs). SA_RESTART keeps reads of
// a recording pipe going after the first Ctrl-C.
static void install_signal_handlers(void) {
    for (size_t i = 0; i < sizeof fatal_sigs / sizeof *fatal_sigs; i++) {
        struct sigaction old, sa;
        memset(&sa, 0, sizeof sa);
        sa.sa_handler = on_signal; sa.sa_flags = SA_RESTART; sigemptyset(&sa.sa_mask);
        if (sigaction(fatal_sigs[i], NULL, &old) == 0 && old.sa_handler != SIG_IGN) sigaction(fatal_sigs[i], &sa, NULL);
    }
}

static int same_file(const struct stat *a, const struct stat *b) { return a->st_dev == b->st_dev && a->st_ino == b->st_ino; }

// Descriptor behind /dev/stdout, /dev/stderr, /dev/fd/N or /proc/self/fd/N, else -1.
static int fd_alias(const char *p) {
    if (!strcmp(p, "/dev/stdout")) return STDOUT_FILENO;
    if (!strcmp(p, "/dev/stderr")) return STDERR_FILENO;
    const char *d = !strncmp(p, "/dev/fd/", 8) ? p + 8 : !strncmp(p, "/proc/self/fd/", 14) ? p + 14 : NULL;
    if (!d || !*d) return -1;
    char *end;
    long v = strtol(d, &end, 10);
    return *end || v < 0 || v > INT_MAX ? -1 : (int)v;
}
// Follows OUTPUT's symlinks one level at a time (relative targets against the link's directory) and stops
// at a descriptor alias (*fd >= 0) or at a path that is not a symlink (stored in out; it may not exist).
static int resolve_output(const char *op, char *out, size_t cap, int *fd) {
    char cur[PATH_MAX], tgt[PATH_MAX], nxt[PATH_MAX];
    if (snprintf(cur, sizeof cur, "%s", op) >= (int)sizeof cur) { errno = ENAMETOOLONG; return -1; }
    for (int i = 0; i < 40; i++) {
        struct stat lo;
        if ((*fd = fd_alias(cur)) >= 0) return 0;
        if (lstat(cur, &lo) != 0 || !S_ISLNK(lo.st_mode)) {
            if (snprintf(out, cap, "%s", cur) >= (int)cap) { errno = ENAMETOOLONG; return -1; }
            return 0;
        }
        ssize_t k = readlink(cur, tgt, sizeof tgt - 1);
        if (k < 0) return -1;
        tgt[k] = 0;
        const char *slash = strrchr(cur, '/');
        int dl = tgt[0] == '/' || !slash ? 0 : (int)(slash - cur) + 1;
        if (snprintf(nxt, sizeof nxt, "%.*s%s", dl, cur, tgt) >= (int)sizeof nxt) { errno = ENAMETOOLONG; return -1; }
        memcpy(cur, nxt, sizeof cur);
    }
    errno = ELOOP;
    return -1;
}

// Refuses when OUTPUT is the input (path, link or redirection). A regular-file OUTPUT (reached through any
// symlinks, which are kept) is written to a temporary file next to it and renamed on success. stdout and
// descriptor aliases (/dev/stdout, /dev/fd/N, ...) are written through the existing descriptor (so >> still
// appends); devices and FIFOs are opened in place.
static FILE *open_output(const char *op, FILE *in) {
    struct stat si, so;
    int have_in = fstat(fileno(in), &si) == 0 && S_ISREG(si.st_mode), afd = -1;
    if (!*op) { msg("empty output path\n"); return NULL; }
    if (!strcmp(op, "-")) afd = STDOUT_FILENO;
    else if (resolve_output(op, dst_path, sizeof dst_path, &afd) != 0) { msg("%s: %s\n", op, strerror(errno)); return NULL; }
    if (afd >= 0) {
        if (fstat(afd, &so) != 0) { msg("%s: %s\n", op, strerror(errno)); return NULL; }
        if (have_in && same_file(&si, &so)) { msg("%s: output is the input file\n", op); return NULL; }
        if (afd == STDOUT_FILENO) return stdout;
        int nfd = dup(afd);
        FILE *f = nfd >= 0 ? fdopen(nfd, "wb") : NULL;   // fdopen never truncates
        if (!f) { msg("%s: %s\n", op, strerror(errno)); if (nfd >= 0) close(nfd); }
        return f;
    }
    int exists = stat(dst_path, &so) == 0;
    if (exists && have_in && same_file(&si, &so)) { msg("%s: output is the input file\n", op); return NULL; }
    if (exists && !S_ISREG(so.st_mode)) {   // device, FIFO, ...
        FILE *f = fopen(dst_path, "wb");
        if (!f) msg("%s: %s\n", op, strerror(errno));
        return f;
    }
    const char *slash = strrchr(dst_path, '/');
    int dl = slash ? (int)(slash - dst_path) + 1 : 0;
    snprintf(tmp_path, sizeof tmp_path, "%.*s.iqcodec-XXXXXX", dl, dst_path);
    int fd = mkstemp(tmp_path);
    if (fd < 0) {
        msg("%s: cannot create a temporary file in %.*s: %s\n", op, dl ? dl : 1, dl ? dst_path : ".", strerror(errno));
        tmp_path[0] = 0;
        return NULL;
    }
    mode_t um = umask(0); umask(um);
    (void)fchmod(fd, exists ? so.st_mode & 0777 : 0666 & ~um);   // no setuid/setgid carried over
    FILE *f = fdopen(fd, "wb");
    if (!f) { msg("%s: %s\n", op, strerror(errno)); close(fd); unlink(tmp_path); tmp_path[0] = 0; }
    return f;
}
// Closes OUTPUT; on success syncs and moves the temporary file into place, otherwise removes it.
static int close_output(FILE *out, const char *op, int ok) {
    if (out == stdout) {
        if (fflush(out) != 0) { if (ok) msg("write error: %s\n", strerror(errno)); return 0; }
        return ok;
    }
    if (ok && tmp_path[0] && (fflush(out) != 0 || fsync(fileno(out)) != 0)) {
        msg("%s: %s\n", op, strerror(errno)); ok = 0;
    }
    if (fclose(out) != 0) { if (ok) msg("%s: %s\n", op, strerror(errno)); ok = 0; }
    if (tmp_path[0]) {
        if (ok && rename(tmp_path, dst_path) != 0) { msg("%s: %s\n", op, strerror(errno)); ok = 0; }
        if (!ok) unlink(tmp_path);
        else {   // make the rename itself durable (best effort)
            char dir[PATH_MAX];
            const char *slash = strrchr(dst_path, '/');
            snprintf(dir, sizeof dir, "%.*s", slash ? (int)(slash - dst_path) + 1 : 1, slash ? dst_path : ".");
            int dfd = open(dir, O_RDONLY);
            if (dfd >= 0) { (void)fsync(dfd); close(dfd); }
        }
        tmp_path[0] = 0;
    }
    return ok;
}

// ---------------- chunk jobs ----------------
typedef struct {
    int fmt, shift, verify; float scale;
    void *raw; int64_t n;             // samples
    uint64_t first;                   // d: index of the chunk's first sample in the stream
    uint8_t *enc; int64_t size, cap;  // coded chunk
    void *chk;                        // decoded copy for -t
    float *qnt;                       // quantized input of a lossy (-l) chunk
    uint32_t crc;                     // crc32c of the samples (stored / expected)
    int64_t inexact; int err;         // 1: coding failure, 2: checksum / verification mismatch
} Job;

static size_t ssize_of(int fmt) { return fmt == IQC_FC32 ? 8 : 4; }

static void *enc_job(void *p) {
    Job *j = p;
    size_t bytes = (size_t)j->n * ssize_of(j->fmt);
    j->size = iqc_encode(j->raw, j->fmt, j->n, j->scale, j->shift, K_ORDER, LEAF, PREC, j->enc, j->cap, &j->inexact);
    j->err = j->size < 0 || j->size > j->cap;
    if (j->err) return NULL;
    const void *expect = j->raw;   // what decoding must reproduce (and what the checksum covers)
    if (j->inexact) {               // lossy chunk: its quantized values
        if (!j->qnt && !(j->qnt = malloc((size_t)CHUNK * 8))) { j->err = 1; return NULL; }
        iqc_quantize(j->raw, 2 * j->n, j->scale, j->qnt);
        expect = j->qnt;
    }
    j->crc = crc32c(0, expect, bytes);
    if (j->verify) {
        if (iqc_decode(j->enc, j->size, j->chk, j->fmt, j->n, j->scale, j->shift, K_ORDER, LEAF)) j->err = 1;
        else if (memcmp(j->chk, expect, bytes)) j->err = 2;
    }
    return NULL;
}
static void *dec_job(void *p) {
    Job *j = p;
    j->err = iqc_decode(j->enc, j->size, j->raw, j->fmt, j->n, j->scale, j->shift, K_ORDER, LEAF) != 0;
    if (!j->err && crc32c(0, j->raw, (size_t)j->n * ssize_of(j->fmt)) != j->crc) j->err = 2;
    return NULL;
}
static void free_jobs(Job *jobs, int nth) { for (int i = 0; i < nth; i++) { free(jobs[i].raw); free(jobs[i].enc); free(jobs[i].chk); free(jobs[i].qnt); } }

// Picks the number of low bits for the sigma-delta tracker by trial on a prefix.
static int pick_shift(const void *raw, int fmt, int64_t n, float scale) {
    int64_t m = n < 65536 ? n : 65536, best = -1;
    int shift = 0;
    uint8_t *buf = malloc((size_t)m * 8 + (1 << 20));
    if (!buf) return 0;
    for (int s = 0; s < 4; s++) {
        int64_t inexact, sz = iqc_encode(raw, fmt, m, scale, s, K_ORDER, LEAF, PREC, buf, m * 8 + (1 << 20), &inexact);
        if (sz >= 0 && (best < 0 || sz < best)) { best = sz; shift = s; }
    }
    free(buf);
    return shift;
}

// ---------------- pipeline: the main thread reads and writes in order, workers code slots as they come ----------------
// Slot states: FREE -> (read) READY -> (worker) BUSY -> DONE -> (written) FREE. Workers take slots in sequence order,
// so no chunk waits for a whole batch (P/E cores), and reading / writing overlap with coding.
enum { S_FREE, S_READY, S_BUSY, S_DONE };
#define MAX_TH 64
#define MAX_SLOTS (MAX_TH + 2)
typedef struct {
    Job *jobs; int *state, ns, quit;
    int64_t take;                      // next sequence number a worker takes
    void *(*fn)(void *);
    pthread_mutex_t mu; pthread_cond_t work, done;
} Pool;
static void *worker(void *p) {
    Pool *pl = p;
    pthread_mutex_lock(&pl->mu);
    for (;;) {
        int k = (int)(pl->take % pl->ns);
        while (!pl->quit && pl->state[k] != S_READY) { pthread_cond_wait(&pl->work, &pl->mu); k = (int)(pl->take % pl->ns); }
        if (pl->quit) break;
        pl->take++; pl->state[k] = S_BUSY;
        pthread_mutex_unlock(&pl->mu);
        pl->fn(&pl->jobs[k]);
        pthread_mutex_lock(&pl->mu);
        pl->state[k] = S_DONE;
        pthread_cond_broadcast(&pl->done);
    }
    pthread_mutex_unlock(&pl->mu);
    return NULL;
}
// Starts the workers with the fatal signals blocked, so the main thread is the one that handles them.
static int pool_start(Pool *pl, pthread_t *th, int nth) {
    sigset_t block, old;
    sigemptyset(&block);
    for (size_t i = 0; i < sizeof fatal_sigs / sizeof *fatal_sigs; i++) sigaddset(&block, fatal_sigs[i]);
    pthread_sigmask(SIG_BLOCK, &block, &old);
    int n = 0;
    for (; n < nth; n++) if (pthread_create(&th[n], NULL, worker, pl)) break;
    pthread_sigmask(SIG_SETMASK, &old, NULL);
    return n;
}
static void pool_stop(Pool *pl, pthread_t *th, int n) {
    pthread_mutex_lock(&pl->mu); pl->quit = 1; pthread_cond_broadcast(&pl->work); pthread_mutex_unlock(&pl->mu);
    for (int i = 0; i < n; i++) pthread_join(th[i], NULL);
}
static void set_state(Pool *pl, int k, int st) {
    pthread_mutex_lock(&pl->mu); pl->state[k] = st; if (st == S_READY) pthread_cond_broadcast(&pl->work); pthread_mutex_unlock(&pl->mu);
}
static int get_state(Pool *pl, int k) { pthread_mutex_lock(&pl->mu); int st = pl->state[k]; pthread_mutex_unlock(&pl->mu); return st; }
static void wait_done(Pool *pl, int k) {
    pthread_mutex_lock(&pl->mu); while (pl->state[k] != S_DONE) pthread_cond_wait(&pl->done, &pl->mu); pthread_mutex_unlock(&pl->mu);
}

// stream: INPUT is not a regular file (a recording pipe): Ctrl-C finishes the file, a trailing partial sample is dropped.
static int compress(FILE *in, FILE *out, int fmt, float scale, int lossy, int verify, int nth, int verbose, int stream) {
    size_t ssz = ssize_of(fmt);
    struct stat ist;
    uint64_t in_size = !stream && fstat(fileno(in), &ist) == 0 ? (uint64_t)ist.st_size : 0;
    int ns = nth + 2, state[MAX_SLOTS] = {0}, nrun = 0, rc = 1;
    Job jobs[MAX_SLOTS];
    pthread_t th[MAX_TH];
    memset(jobs, 0, sizeof jobs);
    Pool pl = {jobs, state, ns, 0, 0, enc_job, PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER};
    for (int i = 0; i < ns; i++) {
        jobs[i].raw = malloc(CHUNK * ssz); jobs[i].cap = (int64_t)CHUNK * 8 + (1 << 20); jobs[i].enc = malloc(jobs[i].cap);
        if (verify) jobs[i].chk = malloc(CHUNK * ssz);
        if (!jobs[i].raw || !jobs[i].enc || (verify && !jobs[i].chk)) { msg("out of memory\n"); goto fail; }
    }
    uint8_t hdr[16];
    memcpy(hdr, MAGIC, 4); hdr[4] = 2; hdr[5] = (uint8_t)fmt; hdr[6] = K_ORDER; hdr[7] = PREC;
    put32(hdr + 8, LEAF); memcpy(hdr + 12, &scale, 4);
    if (fwrite(hdr, 1, 16, out) != 16) goto werr;
    int shift = -1, eof = 0;
    int64_t tot_in = 0, tot_out = 16, tot_inexact = 0, rd = 0, wr = 0;
    uint64_t tot_samples = 0;
    double t0 = now();
    nrun = pool_start(&pl, th, nth);
    if (!nrun) { msg("cannot start threads\n"); goto fail; }
    in_fd = fileno(in); graceful_int = stream;
    while (!eof || wr < rd) {
        if (!eof && rd - wr < ns) {   // read the next chunk into the next slot (free: it was written)
            int k = (int)(rd % ns);
            Job *j = &jobs[k];
            size_t got = read_full(in, j->raw, CHUNK * ssz);
            if (ferror(in)) { msg("read error: %s\n", strerror(errno)); goto fail; }
            if (got % ssz) {
                if (!stream) { msg("input size is not a multiple of %zu bytes\n", ssz); goto fail; }
                msg("input ended inside a sample: %zu trailing bytes dropped\n", got % ssz);
                got -= got % ssz;
                eof = 1;
            }
            if (got < CHUNK * ssz || n_int >= 2) eof = 1;
            if (got) {
                j->n = got / ssz; tot_in += got;
                if (shift < 0) shift = pick_shift(j->raw, fmt, j->n, scale);
                j->fmt = fmt; j->scale = scale; j->shift = shift; j->verify = verify;
                set_state(&pl, k, S_READY); rd++;
            }
            if (!eof && rd - wr < ns && get_state(&pl, (int)(wr % ns)) != S_DONE) continue;   // keep reading ahead
        }
        if (wr == rd) continue;
        int k = (int)(wr % ns);
        wait_done(&pl, k);
        Job *j = &jobs[k];
        if (j->err == 2) { msg("chunk %lld: verification failed (decoded data differs)\n", (long long)wr); goto fail; }
        if (j->err) { msg("chunk %lld: encoding failed\n", (long long)wr); goto fail; }
        tot_inexact += j->inexact;
        if (j->inexact && !lossy) {
            msg("input is not exactly int16 / %g (%lld values); use -s or -l\n", scale, (long long)j->inexact);
            goto fail;
        }
        uint8_t ch[16];
        put32(ch, (uint32_t)j->n); ch[4] = (uint8_t)shift; ch[5] = ch[6] = ch[7] = 0;
        put32(ch + 8, (uint32_t)j->size); put32(ch + 12, j->crc);
        if (fwrite(ch, 1, 16, out) != 16 || fwrite(j->enc, 1, j->size, out) != (size_t)j->size) goto werr;
        tot_out += 16 + j->size; tot_samples += j->n;
        progress("compressing", (uint64_t)tot_in, in_size, (uint64_t)tot_in, t0);
        set_state(&pl, k, S_FREE); wr++;
    }
    uint8_t end[12] = {0};
    put32(end + 4, (uint32_t)tot_samples); put32(end + 8, (uint32_t)(tot_samples >> 32));
    if (fwrite(end, 1, 12, out) != 12) goto werr;
    tot_out += 12;
    progress_end();
    if (verbose || n_int)
        fprintf(stderr, "%lld -> %lld bytes (%.2f%%, %.3fx), %.2f s%s%s%s\n", (long long)tot_in, (long long)tot_out,
                tot_in ? 100.0 * tot_out / tot_in : 0, tot_out ? (double)tot_in / tot_out : 0, now() - t0,
                verify ? ", verified" : "", tot_inexact ? " (lossy: values quantized)" : "", n_int ? " (stopped by Ctrl-C)" : "");
    rc = 0;
    goto done;
werr:
    msg("write error: %s\n", strerror(errno));
fail:
done:
    progress_end();
    pool_stop(&pl, th, nrun);
    free_jobs(jobs, ns);
    return rc;
}

// out == NULL: test only. Samples [skip, lim) only: chunks before skip are passed over (seeked when possible), and
// reading stops at the first chunk past lim, so only the chunks that overlap the range are decoded and verified.
// info: print what the headers say and decode nothing. salvage: a chunk that fails its checksum becomes zeros, and a
// truncated or damaged stream ends at the last good chunk; returns 3 when anything was lost.
static int decompress(FILE *in, FILE *out, int nth, int verbose, uint64_t skip, uint64_t lim, int info, int salvage) {
    uint8_t hdr[16];
    if (read_full(in, hdr, 16) != 16 || memcmp(hdr, MAGIC, 4) || hdr[4] != 2 || hdr[5] > 1 || hdr[6] != K_ORDER || get32(hdr + 8) != LEAF) {
        msg("not an iqcodec stream (or an unsupported version)\n");
        return 1;
    }
    int fmt = hdr[5];
    float scale; memcpy(&scale, hdr + 12, 4);
    size_t ssz = ssize_of(fmt);
    int ns = nth + 2, state[MAX_SLOTS] = {0}, nrun = 0, rc = 1, eof = 0, lost = 0;
    uint64_t nchunks = 0, csize = 16 + 12;
    const char *why;
    if (info) skip = lim = UINT64_MAX;
    Job jobs[MAX_SLOTS];
    pthread_t th[MAX_TH];
    memset(jobs, 0, sizeof jobs);
    Pool pl = {jobs, state, ns, 0, 0, dec_job, PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER};
    int64_t rawcap[MAX_SLOTS] = {0}, rd = 0, wr = 0;
    uint64_t tot = 0, have = 0;
    static uint8_t skipbuf[1 << 16];
    struct stat st;
    int seekable = fstat(fileno(in), &st) == 0 && S_ISREG(st.st_mode);
    double t0 = now();
    nrun = pool_start(&pl, th, nth);
    if (!nrun) { msg("cannot start threads\n"); goto fail; }
    while (!eof || wr < rd) {
        if (!eof && have >= lim) eof = 1;
        if (!eof && rd - wr < ns) {
            uint8_t ch[16];
            why = "truncated input";
            if (read_full(in, ch, 4) != 4) goto bad;
            uint32_t n = get32(ch);
            if (!n) {   // end marker: total sample count, then nothing
                uint8_t t[8];
                if (read_full(in, t, 8) != 8) goto bad;
                uint64_t want = get32(t) | (uint64_t)get32(t + 4) << 32;
                why = "sample count mismatch (corrupt end marker or missing chunks)";
                if (want != have) goto bad;
                why = "unexpected data after the end of the stream";
                if (fgetc(in) != EOF) goto bad;
                if (ferror(in)) { msg("read error: %s\n", strerror(errno)); goto fail; }
                eof = 1;
            } else {
                if (read_full(in, ch + 4, 12) != 12) goto bad;
                uint32_t sz = get32(ch + 8);
                why = "corrupt chunk header";
                if (n > CHUNK || ch[4] > 3 || sz < 24 || sz > (uint64_t)n * 8 + (1 << 20)) goto bad;
                why = "truncated input";
                nchunks++; csize += 16 + (uint64_t)sz;
                if (have + n <= skip) {   // before the range
                    have += n;
                    if (seekable) { if (fseeko(in, sz, SEEK_CUR)) { msg("seek error: %s\n", strerror(errno)); goto fail; } continue; }
                    for (uint32_t left = sz, m; left; left -= m) {   // not seekable: read it
                        m = left < sizeof skipbuf ? left : (uint32_t)sizeof skipbuf;
                        if (read_full(in, skipbuf, m) != m) break;
                    }
                    if (feof(in) || ferror(in)) goto bad;
                    continue;
                }
                int k = (int)(rd % ns);
                Job *j = &jobs[k];
                if (sz > j->cap) { free(j->enc); j->cap = sz; j->enc = malloc(sz); }
                if ((int64_t)n > rawcap[k]) { free(j->raw); rawcap[k] = n; j->raw = malloc((size_t)n * ssz); }
                if (!j->enc || !j->raw) { msg("out of memory\n"); goto fail; }
                if (read_full(in, j->enc, sz) != sz) goto bad;
                j->n = n; j->size = sz; j->shift = ch[4]; j->fmt = fmt; j->scale = scale; j->crc = get32(ch + 12); j->first = have;
                have += n;
                set_state(&pl, k, S_READY); rd++;
                if (rd - wr < ns && get_state(&pl, (int)(wr % ns)) != S_DONE) continue;   // keep reading ahead
            }
            if (0) {
            bad:   // the stream itself is damaged: give up, or with salvage keep what came before
                if (ferror(in)) { msg("read error: %s\n", strerror(errno)); goto fail; }
                msg("%s after %llu samples%s\n", why, (unsigned long long)have, salvage ? ": stopping there" : "");
                if (!salvage) goto fail;
                lost = 1; eof = 1;
            }
        }
        if (wr == rd) continue;
        int k = (int)(wr % ns);
        wait_done(&pl, k);
        Job *j = &jobs[k];
        if (j->err) {
            msg("samples %llu-%llu: %s%s\n", (unsigned long long)j->first, (unsigned long long)(j->first + j->n - 1),
                    j->err == 2 ? "checksum mismatch (corrupt data)" : "corrupt data", salvage ? ", replaced by zeros" : "");
            if (!salvage) goto fail;
            memset(j->raw, 0, (size_t)j->n * ssz); lost = 1;
        }
        uint64_t lo = skip > j->first ? skip - j->first : 0, hi = lim - j->first < (uint64_t)j->n ? lim - j->first : (uint64_t)j->n;
        if (out && fwrite((char *)j->raw + lo * ssz, ssz, hi - lo, out) != hi - lo) { msg("write error: %s\n", strerror(errno)); goto fail; }
        tot += hi - lo;
        if (!info) progress(out ? "decoding" : "testing", csize, seekable ? (uint64_t)st.st_size : 0, tot * ssz, t0);
        set_state(&pl, k, S_FREE); wr++;
    }
    progress_end();
    if (info) {
        printf("format   %s (int16 / %g)\nsamples  %llu (%llu bytes)\nchunks   %llu\nsize     %llu bytes (%.2f%%)\n",
               fmt == IQC_FC32 ? "fc32" : "sc16", scale, (unsigned long long)have, (unsigned long long)(have * ssz),
               (unsigned long long)nchunks, (unsigned long long)csize, have ? 100.0 * csize / (have * ssz) : 0);
    } else if ((lim != UINT64_MAX && tot < lim - skip) || (have < skip && lim > skip))
        msg("the stream ends inside the requested range (%llu samples)\n", (unsigned long long)tot);
    if (verbose && !info)
        fprintf(stderr, "%llu samples (%llu bytes) %s, %.2f s, checksums verified\n", (unsigned long long)tot,
                (unsigned long long)(tot * ssz), out ? "written" : "OK", now() - t0);
    rc = lost ? 3 : 0;
fail:
    progress_end();
    pool_stop(&pl, th, nrun);
    free_jobs(jobs, ns);
    return rc;
}

int main(int argc, char **argv) {
    if (argc >= 2 && (!strcmp(argv[1], "-V") || !strcmp(argv[1], "--version"))) { puts("iqcodec " VERSION); return 0; }
    if (argc >= 2 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) { usage(stdout); return 0; }
    if (argc < 2 || strlen(argv[1]) != 1 || !strchr("cdti", argv[1][0])) { usage(stderr); return 2; }
    int cmd = argv[1][0] == 'c' ? CMD_C : argv[1][0] == 'd' ? CMD_D : argv[1][0] == 't' ? CMD_T : CMD_I;
    int fmt = IQC_FC32, lossy = 0, verify = 0, verbose = 0, opt;
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    prog_on = isatty(STDERR_FILENO);
    int nth = ncpu < 1 ? 1 : ncpu > 8 ? 8 : (int)ncpu;
    float scale = 32767.0f;
    optind = 2;
#ifdef __GLIBC__
    const char *optstr = "+f:s:ltj:vhV";   // like BSD: options before operands only
#else
    const char *optstr = "f:s:ltj:vhV";
#endif
    char *end;
    uint64_t skip = 0, count = UINT64_MAX;
    int ranged = 0, salvage = 0, rm = 0;
    static const struct option longopts[] = {{"skip", required_argument, NULL, 'S'}, {"count", required_argument, NULL, 'C'},
                                             {"salvage", no_argument, NULL, 'R'}, {"rm", no_argument, NULL, 'X'}, {0, 0, 0, 0}};
    while ((opt = getopt_long(argc, argv, optstr, longopts, NULL)) != -1) {
        switch (opt) {
        case 'f':
            if (!strcmp(optarg, "fc32")) fmt = IQC_FC32;
            else if (!strcmp(optarg, "sc16")) fmt = IQC_SC16;
            else { msg("unknown format %s\n", optarg); return 2; }
            break;
        case 's':
            scale = strtof(optarg, &end);
            if (*end || end == optarg || !(scale > 0) || !(scale < 1e30f) || !(1.0f / scale > 0)) {   // library limits
                msg("bad scale %s\n", optarg); return 2;
            }
            break;
        case 'l': lossy = 1; break;
        case 't': verify = 1; break;
        case 'j': {
            long v = strtol(optarg, &end, 10);
            if (*end || end == optarg || v < 1 || v > 64) { msg("-j must be 1..64\n"); return 2; }
            nth = (int)v;
            break;
        }
        case 'S': case 'C': {
            errno = 0;
            unsigned long long v = strtoull(optarg, &end, 10);
            if (*end || end == optarg || *optarg == '-' || errno) { msg("bad sample count %s\n", optarg); return 2; }
            if (opt == 'S') skip = v; else count = v;
            ranged = 1;
            break;
        }
        case 'R': salvage = 1; break;
        case 'X': rm = 1; break;
        case 'v': verbose = 1; break;
        case 'V': puts("iqcodec " VERSION); return 0;
        case 'h': usage(stdout); return 0;
        default: usage(stderr); return 2;
        }
    }
    int nop = argc - optind;
    if (cmd >= CMD_T ? nop != 1 : nop < 1 || nop > 2) { usage(stderr); return 2; }
    if ((ranged || salvage) && (cmd == CMD_C || cmd == CMD_I)) { msg("--skip, --count and --salvage are for d and t\n"); return 2; }
    uint64_t lim = count > UINT64_MAX - skip ? UINT64_MAX : skip + count;
    if (rm && (cmd >= CMD_T || ranged || salvage)) { msg("--rm is for plain c and d\n"); return 2; }
    const char *ip = argv[optind], *op = cmd >= CMD_T ? NULL : argv[optind + 1];
    char auto_op[PATH_MAX];
    if (cmd < CMD_T && nop == 1) {   // like gzip: FILE <-> FILE.iqc, never replacing an existing file
        size_t l = strlen(ip);
        if (!strcmp(ip, "-")) { msg("OUTPUT is needed with stdin\n"); return 2; }
        if (cmd == CMD_D && (l <= 4 || strcmp(ip + l - 4, ".iqc"))) { msg("%s: no .iqc suffix; give OUTPUT\n", ip); return 2; }
        if (snprintf(auto_op, sizeof auto_op, "%.*s%s", (int)(cmd == CMD_D ? l - 4 : l), ip, cmd == CMD_D ? "" : ".iqc") >= (int)sizeof auto_op) {
            msg("%s: name too long\n", ip); return 2;
        }
        struct stat st;
        if (lstat(auto_op, &st) == 0) { msg("%s already exists\n", auto_op); return 1; }
        op = auto_op;
    }
    if (rm && !strcmp(ip, "-")) { msg("--rm needs a file INPUT\n"); return 2; }
    if (rm && cmd == CMD_C) verify = 1;   // nothing is deleted that has not been decoded and compared
    FILE *in = strcmp(ip, "-") ? fopen(ip, "rb") : stdin;
    if (!in) { msg("%s: %s\n", ip, strerror(errno)); return 1; }
    if (cmd >= CMD_T) {
        int rc = decompress(in, NULL, nth, verbose, skip, lim, cmd == CMD_I, salvage);
        if (in != stdin) fclose(in);
        return rc;
    }
    install_signal_handlers();
    FILE *out = open_output(op, in);
    if (!out) { if (in != stdin) fclose(in); return 1; }
    if (rm && !tmp_path[0]) {   // the data must end up in a file of its own
        msg("--rm needs a regular file OUTPUT\n");
        close_output(out, op, 0); if (in != stdin) fclose(in); return 2;
    }
    struct stat ist;
    int stream = !(fstat(fileno(in), &ist) == 0 && S_ISREG(ist.st_mode));
    int rc = cmd == CMD_D ? decompress(in, out, nth, verbose, skip, lim, 0, salvage) : compress(in, out, fmt, scale, lossy, verify, nth, verbose, stream);
    if (!close_output(out, op, rc == 0 || rc == 3)) rc = 1;
    if (in != stdin) fclose(in);
    if (rm && rc == 0 && !stream && unlink(ip)) { msg("cannot remove %s: %s\n", ip, strerror(errno)); rc = 1; }
    return rc;
}
