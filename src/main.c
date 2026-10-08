// iqcodec command line: lossless compression of IQ capture files.
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "crc32c.h"
#include "iqc.h"

#define VERSION "0.2.1"
#define CHUNK (1 << 21)           // complex samples per independently coded chunk
#define K_ORDER 24
#define LEAF 8192
#define PREC 11

// File: "IQCD" u8 version, u8 fmt, u8 K, u8 prec, u32 leaf, f32 scale; then chunks:
//   u32 nsamples (0 = end), u8 shift, 3 reserved bytes, u32 nbytes, u32 crc32c of the samples, payload
// end marker: u32 0, u64 total samples. (Version 2; version 1 of iqcodec 0.1.0 is not supported.)
static const char MAGIC[4] = {'I', 'Q', 'C', 'D'};
enum { CMD_C, CMD_D, CMD_T };

static void usage(FILE *f) {
    fprintf(f,
        "iqcodec " VERSION " - lossless compression for IQ captures\n\n"
        "usage: iqcodec c [options] INPUT OUTPUT    compress\n"
        "       iqcodec d [options] INPUT OUTPUT    decompress (checksums verified)\n"
        "       iqcodec t [options] INPUT           test: decompress and verify checksums, no output\n"
        "       (use - for stdin / stdout)\n\n"
        "options:\n"
        "  -f FMT    input sample format for c: fc32 (complex float32, default) or sc16 (complex int16)\n"
        "  -s SCALE  fc32 values are int16 / SCALE (default 32767, as UHD converts sc16 to fc32)\n"
        "  -l        lossy: allow fc32 input that is not exactly int16 / SCALE (it is quantized)\n"
        "  -t        with c: decode every chunk right after encoding and compare with the input\n"
        "  -j N      threads (default: number of CPUs, at most 8)\n"
        "  -v        print statistics\n"
        "  -h, -V    help, version\n\n"
        "A file OUTPUT is written to a temporary file and renamed when complete, so an existing file is\n"
        "replaced only on success (stdout, pipes and devices receive data as it is decoded). INPUT and\n"
        "OUTPUT must not be the same file. Options go before INPUT / OUTPUT.\n");
}

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static size_t read_full(FILE *f, void *buf, size_t n) {
    size_t got = 0;
    while (got < n) { size_t r = fread((char *)buf + got, 1, n - got, f); if (!r) break; got += r; }
    return got;
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

// ---------------- output: same-file guard, temporary file + rename ----------------
static char tmp_path[PATH_MAX + 32];   // non-empty while a temporary output exists
static char dst_path[PATH_MAX];        // regular-file OUTPUT after following symlinks (rename target)

static void on_signal(int sig) { if (tmp_path[0]) unlink(tmp_path); signal(sig, SIG_DFL); raise(sig); }
// Cleans up on fatal signals, except those the caller ignores (nohup, background jobs).
static void install_signal_handlers(void) {
    static const int sigs[] = {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGXFSZ};
    for (size_t i = 0; i < sizeof sigs / sizeof *sigs; i++) {
        struct sigaction old;
        if (sigaction(sigs[i], NULL, &old) == 0 && old.sa_handler != SIG_IGN) signal(sigs[i], on_signal);
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
    if (!*op) { fprintf(stderr, "iqcodec: empty output path\n"); return NULL; }
    if (!strcmp(op, "-")) afd = STDOUT_FILENO;
    else if (resolve_output(op, dst_path, sizeof dst_path, &afd) != 0) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); return NULL; }
    if (afd >= 0) {
        if (fstat(afd, &so) != 0) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); return NULL; }
        if (have_in && same_file(&si, &so)) { fprintf(stderr, "iqcodec: %s: output is the input file\n", op); return NULL; }
        if (afd == STDOUT_FILENO) return stdout;
        int nfd = dup(afd);
        FILE *f = nfd >= 0 ? fdopen(nfd, "wb") : NULL;   // fdopen never truncates
        if (!f) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); if (nfd >= 0) close(nfd); }
        return f;
    }
    int exists = stat(dst_path, &so) == 0;
    if (exists && have_in && same_file(&si, &so)) { fprintf(stderr, "iqcodec: %s: output is the input file\n", op); return NULL; }
    if (exists && !S_ISREG(so.st_mode)) {   // device, FIFO, ...
        FILE *f = fopen(dst_path, "wb");
        if (!f) fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno));
        return f;
    }
    const char *slash = strrchr(dst_path, '/');
    int dl = slash ? (int)(slash - dst_path) + 1 : 0;
    snprintf(tmp_path, sizeof tmp_path, "%.*s.iqcodec-XXXXXX", dl, dst_path);
    int fd = mkstemp(tmp_path);
    if (fd < 0) {
        fprintf(stderr, "iqcodec: %s: cannot create a temporary file in %.*s: %s\n", op, dl ? dl : 1, dl ? dst_path : ".", strerror(errno));
        tmp_path[0] = 0;
        return NULL;
    }
    mode_t um = umask(0); umask(um);
    (void)fchmod(fd, exists ? so.st_mode & 0777 : 0666 & ~um);   // no setuid/setgid carried over
    FILE *f = fdopen(fd, "wb");
    if (!f) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); close(fd); unlink(tmp_path); tmp_path[0] = 0; }
    return f;
}
// Closes OUTPUT; on success syncs and moves the temporary file into place, otherwise removes it.
static int close_output(FILE *out, const char *op, int ok) {
    if (out == stdout) {
        if (fflush(out) != 0) { if (ok) fprintf(stderr, "iqcodec: write error: %s\n", strerror(errno)); return 0; }
        return ok;
    }
    if (ok && tmp_path[0] && (fflush(out) != 0 || fsync(fileno(out)) != 0)) {
        fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); ok = 0;
    }
    if (fclose(out) != 0) { if (ok) fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); ok = 0; }
    if (tmp_path[0]) {
        if (ok && rename(tmp_path, dst_path) != 0) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); ok = 0; }
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
static void run_jobs(Job *jobs, int nj, void *(*fn)(void *)) {
    pthread_t th[64];
    int started[64] = {0};
    for (int i = 1; i < nj; i++) started[i] = pthread_create(&th[i], NULL, fn, &jobs[i]) == 0;
    for (int i = 0; i < nj; i++) if (!started[i]) fn(&jobs[i]);   // job 0, and any thread that failed to start
    for (int i = 1; i < nj; i++) if (started[i]) pthread_join(th[i], NULL);
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

static int compress(FILE *in, FILE *out, int fmt, float scale, int lossy, int verify, int nth, int verbose) {
    size_t ssz = ssize_of(fmt);
    Job jobs[64];
    memset(jobs, 0, sizeof jobs);
    for (int i = 0; i < nth; i++) {
        jobs[i].raw = malloc(CHUNK * ssz); jobs[i].cap = (int64_t)CHUNK * 8 + (1 << 20); jobs[i].enc = malloc(jobs[i].cap);
        if (verify) jobs[i].chk = malloc(CHUNK * ssz);
        if (!jobs[i].raw || !jobs[i].enc || (verify && !jobs[i].chk)) { fprintf(stderr, "iqcodec: out of memory\n"); goto fail; }
    }
    uint8_t hdr[16];
    memcpy(hdr, MAGIC, 4); hdr[4] = 2; hdr[5] = (uint8_t)fmt; hdr[6] = K_ORDER; hdr[7] = PREC;
    put32(hdr + 8, LEAF); memcpy(hdr + 12, &scale, 4);
    if (fwrite(hdr, 1, 16, out) != 16) goto werr;
    int shift = -1, done = 0;
    int64_t tot_in = 0, tot_out = 16, tot_inexact = 0, nchunk = 0;
    uint64_t tot_samples = 0;
    double t0 = now();
    while (!done) {
        int nj = 0;
        for (; nj < nth; nj++) {
            size_t got = read_full(in, jobs[nj].raw, CHUNK * ssz);
            if (ferror(in)) { fprintf(stderr, "iqcodec: read error: %s\n", strerror(errno)); goto fail; }
            if (got % ssz) { fprintf(stderr, "iqcodec: input size is not a multiple of %zu bytes\n", ssz); goto fail; }
            jobs[nj].n = got / ssz;
            if (got < CHUNK * ssz) done = 1;
            if (!got) break;
            tot_in += got;
            if (done) { nj++; break; }
        }
        if (!nj) break;
        if (shift < 0) shift = pick_shift(jobs[0].raw, fmt, jobs[0].n, scale);
        for (int i = 0; i < nj; i++) { jobs[i].fmt = fmt; jobs[i].scale = scale; jobs[i].shift = shift; jobs[i].verify = verify; }
        run_jobs(jobs, nj, enc_job);
        for (int i = 0; i < nj; i++, nchunk++) {
            if (jobs[i].err == 2) { fprintf(stderr, "iqcodec: chunk %lld: verification failed (decoded data differs)\n", (long long)nchunk); goto fail; }
            if (jobs[i].err) { fprintf(stderr, "iqcodec: chunk %lld: encoding failed\n", (long long)nchunk); goto fail; }
            tot_inexact += jobs[i].inexact;
            if (jobs[i].inexact && !lossy) {
                fprintf(stderr, "iqcodec: input is not exactly int16 / %g (%lld values); use -s or -l\n", scale, (long long)jobs[i].inexact);
                goto fail;
            }
            uint8_t ch[16];
            put32(ch, (uint32_t)jobs[i].n); ch[4] = (uint8_t)shift; ch[5] = ch[6] = ch[7] = 0;
            put32(ch + 8, (uint32_t)jobs[i].size); put32(ch + 12, jobs[i].crc);
            if (fwrite(ch, 1, 16, out) != 16 || fwrite(jobs[i].enc, 1, jobs[i].size, out) != (size_t)jobs[i].size) goto werr;
            tot_out += 16 + jobs[i].size;
            tot_samples += jobs[i].n;
        }
    }
    uint8_t end[12] = {0};
    put32(end + 4, (uint32_t)tot_samples); put32(end + 8, (uint32_t)(tot_samples >> 32));
    if (fwrite(end, 1, 12, out) != 12) goto werr;
    tot_out += 12;
    if (verbose)
        fprintf(stderr, "%lld -> %lld bytes (%.2f%%, %.3fx), %.2f s%s%s\n", (long long)tot_in, (long long)tot_out,
                tot_in ? 100.0 * tot_out / tot_in : 0, tot_out ? (double)tot_in / tot_out : 0, now() - t0,
                verify ? ", verified" : "", tot_inexact ? " (lossy: values quantized)" : "");
    free_jobs(jobs, nth);
    return 0;
werr:
    fprintf(stderr, "iqcodec: write error: %s\n", strerror(errno));
fail:
    free_jobs(jobs, nth);
    return 1;
}

// out == NULL: test only.
static int decompress(FILE *in, FILE *out, int nth, int verbose) {
    uint8_t hdr[16];
    if (read_full(in, hdr, 16) != 16 || memcmp(hdr, MAGIC, 4) || hdr[4] != 2 || hdr[5] > 1 || hdr[6] != K_ORDER || get32(hdr + 8) != LEAF) {
        fprintf(stderr, "iqcodec: not an iqcodec stream (or an unsupported version)\n");
        return 1;
    }
    int fmt = hdr[5];
    float scale; memcpy(&scale, hdr + 12, 4);
    size_t ssz = ssize_of(fmt);
    Job jobs[64];
    memset(jobs, 0, sizeof jobs);
    int64_t rawcap[64] = {0}, nchunk = 0;
    uint64_t tot = 0;
    double t0 = now();
    for (int done = 0; !done;) {
        int nj = 0;
        for (; nj < nth; nj++) {
            uint8_t ch[16];
            if (read_full(in, ch, 4) != 4) goto trunc;
            uint32_t n = get32(ch);
            if (!n) {   // end marker: total sample count
                uint8_t t[8];
                if (read_full(in, t, 8) != 8) goto trunc;
                uint64_t want = get32(t) | (uint64_t)get32(t + 4) << 32, have = tot;
                for (int i = 0; i < nj; i++) have += jobs[i].n;
                if (want != have) { fprintf(stderr, "iqcodec: sample count mismatch (%llu expected, %llu found)\n", (unsigned long long)want, (unsigned long long)have); goto fail; }
                if (fgetc(in) != EOF) { fprintf(stderr, "iqcodec: unexpected data after the end of the stream\n"); goto fail; }
                if (ferror(in)) { fprintf(stderr, "iqcodec: read error: %s\n", strerror(errno)); goto fail; }
                done = 1; break;
            }
            if (read_full(in, ch + 4, 12) != 12) goto trunc;
            uint32_t sz = get32(ch + 8);
            if (n > CHUNK || ch[4] > 3 || sz < 24 || sz > (uint64_t)n * 8 + (1 << 20)) goto corrupt;
            Job *j = &jobs[nj];
            if (sz > j->cap) { free(j->enc); j->cap = sz; j->enc = malloc(sz); }
            if ((int64_t)n > rawcap[nj]) { free(j->raw); rawcap[nj] = n; j->raw = malloc((size_t)n * ssz); }
            if (!j->enc || !j->raw) { fprintf(stderr, "iqcodec: out of memory\n"); goto fail; }
            if (read_full(in, j->enc, sz) != sz) goto trunc;
            j->n = n; j->size = sz; j->shift = ch[4]; j->fmt = fmt; j->scale = scale;
            j->crc = get32(ch + 12);
        }
        if (!nj) break;
        run_jobs(jobs, nj, dec_job);
        for (int i = 0; i < nj; i++, nchunk++) {
            if (jobs[i].err == 2) { fprintf(stderr, "iqcodec: chunk %lld: checksum mismatch (corrupt data)\n", (long long)nchunk); goto fail; }
            if (jobs[i].err) { fprintf(stderr, "iqcodec: chunk %lld: corrupt data\n", (long long)nchunk); goto fail; }
            if (out && fwrite(jobs[i].raw, ssz, jobs[i].n, out) != (size_t)jobs[i].n) {
                fprintf(stderr, "iqcodec: write error: %s\n", strerror(errno));
                goto fail;
            }
            tot += jobs[i].n;
        }
    }
    if (verbose)
        fprintf(stderr, "%llu samples (%llu bytes) %s, %.2f s, checksums verified\n", (unsigned long long)tot,
                (unsigned long long)(tot * ssz), out ? "written" : "OK", now() - t0);
    free_jobs(jobs, nth);
    return 0;
trunc:
    fprintf(stderr, "iqcodec: truncated input\n");
    goto fail;
corrupt:
    fprintf(stderr, "iqcodec: chunk %lld: corrupt header\n", (long long)nchunk);
fail:
    free_jobs(jobs, nth);
    return 1;
}

int main(int argc, char **argv) {
    if (argc >= 2 && (!strcmp(argv[1], "-V") || !strcmp(argv[1], "--version"))) { puts("iqcodec " VERSION); return 0; }
    if (argc >= 2 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) { usage(stdout); return 0; }
    if (argc < 2 || strlen(argv[1]) != 1 || !strchr("cdt", argv[1][0])) { usage(stderr); return 2; }
    int cmd = argv[1][0] == 'c' ? CMD_C : argv[1][0] == 'd' ? CMD_D : CMD_T;
    int fmt = IQC_FC32, lossy = 0, verify = 0, verbose = 0, opt;
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    int nth = ncpu < 1 ? 1 : ncpu > 8 ? 8 : (int)ncpu;
    float scale = 32767.0f;
    optind = 2;
#ifdef __GLIBC__
    const char *optstr = "+f:s:ltj:vhV";   // like BSD: options before operands only
#else
    const char *optstr = "f:s:ltj:vhV";
#endif
    char *end;
    while ((opt = getopt(argc, argv, optstr)) != -1) {
        switch (opt) {
        case 'f':
            if (!strcmp(optarg, "fc32")) fmt = IQC_FC32;
            else if (!strcmp(optarg, "sc16")) fmt = IQC_SC16;
            else { fprintf(stderr, "iqcodec: unknown format %s\n", optarg); return 2; }
            break;
        case 's':
            scale = strtof(optarg, &end);
            if (*end || end == optarg || !(scale > 0) || !(scale < 1e30f) || !(1.0f / scale > 0)) {   // library limits
                fprintf(stderr, "iqcodec: bad scale %s\n", optarg); return 2;
            }
            break;
        case 'l': lossy = 1; break;
        case 't': verify = 1; break;
        case 'j': {
            long v = strtol(optarg, &end, 10);
            if (*end || end == optarg || v < 1 || v > 64) { fprintf(stderr, "iqcodec: -j must be 1..64\n"); return 2; }
            nth = (int)v;
            break;
        }
        case 'v': verbose = 1; break;
        case 'V': puts("iqcodec " VERSION); return 0;
        case 'h': usage(stdout); return 0;
        default: usage(stderr); return 2;
        }
    }
    if (argc - optind != (cmd == CMD_T ? 1 : 2)) { usage(stderr); return 2; }
    const char *ip = argv[optind], *op = cmd == CMD_T ? NULL : argv[optind + 1];
    FILE *in = strcmp(ip, "-") ? fopen(ip, "rb") : stdin;
    if (!in) { fprintf(stderr, "iqcodec: %s: %s\n", ip, strerror(errno)); return 1; }
    if (cmd == CMD_T) {
        int rc = decompress(in, NULL, nth, verbose);
        if (in != stdin) fclose(in);
        return rc;
    }
    install_signal_handlers();
    FILE *out = open_output(op, in);
    if (!out) { if (in != stdin) fclose(in); return 1; }
    int rc = cmd == CMD_D ? decompress(in, out, nth, verbose) : compress(in, out, fmt, scale, lossy, verify, nth, verbose);
    if (!close_output(out, op, rc == 0)) rc = 1;
    if (in != stdin) fclose(in);
    return rc;
}
