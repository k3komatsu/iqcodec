// iqcodec command line: lossless compression of IQ capture files.
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "iqc.h"

#define VERSION "0.1.0"
#define CHUNK (1 << 21)           // complex samples per independently coded chunk
#define MAX_CHUNK (1 << 26)       // sanity bound when reading
#define K_ORDER 24
#define LEAF 8192
#define PREC 11

// File: "IQCD" u8 version, u8 fmt, u8 K, u8 prec, u32 leaf, f32 scale; then chunks:
// u32 nsamples (0 = end), u8 shift, 3 reserved bytes, u32 nbytes, payload.
static const char MAGIC[4] = {'I', 'Q', 'C', 'D'};

static void usage(FILE *f) {
    fprintf(f,
        "iqcodec " VERSION " - lossless compression for IQ captures\n\n"
        "usage: iqcodec c [options] INPUT OUTPUT    compress\n"
        "       iqcodec d [options] INPUT OUTPUT    decompress\n"
        "       (use - for stdin / stdout)\n\n"
        "options:\n"
        "  -f FMT    input sample format for c: fc32 (complex float32, default) or sc16 (complex int16)\n"
        "  -s SCALE  fc32 values are int16 / SCALE (default 32767, as UHD converts sc16 to fc32)\n"
        "  -l        lossy: allow fc32 input that is not exactly int16 / SCALE (it is quantized)\n"
        "  -j N      threads (default: number of CPUs, at most 8)\n"
        "  -v        print statistics\n"
        "  -h, -V    help, version\n");
}

static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static size_t read_full(FILE *f, void *buf, size_t n) {
    size_t got = 0;
    while (got < n) { size_t r = fread((char *)buf + got, 1, n - got, f); if (!r) break; got += r; }
    return got;
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

typedef struct {
    int fmt, shift; float scale;
    void *raw; int64_t n;             // samples
    uint8_t *enc; int64_t size, cap;  // coded chunk
    int64_t inexact; int err;
} Job;

static void *enc_job(void *p) {
    Job *j = p;
    j->size = iqc_encode(j->raw, j->fmt, j->n, j->scale, j->shift, K_ORDER, LEAF, PREC, j->enc, j->cap, &j->inexact);
    j->err = j->size < 0 || j->size > j->cap;
    return NULL;
}
static void *dec_job(void *p) {
    Job *j = p;
    j->err = iqc_decode(j->enc, j->size, j->raw, j->fmt, j->n, j->scale, j->shift, K_ORDER, LEAF) != 0;
    return NULL;
}
static void run_jobs(Job *jobs, int nj, void *(*fn)(void *)) {
    pthread_t th[64];
    int started[64] = {0};
    for (int i = 1; i < nj; i++) started[i] = pthread_create(&th[i], NULL, fn, &jobs[i]) == 0;
    for (int i = 0; i < nj; i++) if (!started[i]) fn(&jobs[i]);   // job 0, and any thread that failed to start
    for (int i = 1; i < nj; i++) if (started[i]) pthread_join(th[i], NULL);
}

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

static int compress(FILE *in, FILE *out, int fmt, float scale, int lossy, int nth, int verbose) {
    size_t ssz = fmt == IQC_FC32 ? 8 : 4;
    Job jobs[64];
    memset(jobs, 0, sizeof jobs);
    for (int i = 0; i < nth; i++) {
        jobs[i].raw = malloc(CHUNK * ssz); jobs[i].cap = (int64_t)CHUNK * 8 + (1 << 20); jobs[i].enc = malloc(jobs[i].cap);
        if (!jobs[i].raw || !jobs[i].enc) { fprintf(stderr, "iqcodec: out of memory\n"); return 1; }
    }
    uint8_t hdr[16];
    memcpy(hdr, MAGIC, 4); hdr[4] = 1; hdr[5] = (uint8_t)fmt; hdr[6] = K_ORDER; hdr[7] = PREC;
    put32(hdr + 8, LEAF); memcpy(hdr + 12, &scale, 4);
    if (fwrite(hdr, 1, 16, out) != 16) goto werr;
    int shift = -1, done = 0;
    int64_t tot_in = 0, tot_out = 16, tot_inexact = 0;
    double t0 = now();
    while (!done) {
        int nj = 0;
        for (; nj < nth; nj++) {
            size_t got = read_full(in, jobs[nj].raw, CHUNK * ssz);
            if (got % ssz) { fprintf(stderr, "iqcodec: input size is not a multiple of %zu bytes\n", ssz); return 1; }
            jobs[nj].n = got / ssz;
            if (got < CHUNK * ssz) done = 1;
            if (!got) break;
            tot_in += got;
            if (done) { nj++; break; }
        }
        if (!nj) break;
        if (shift < 0) shift = pick_shift(jobs[0].raw, fmt, jobs[0].n, scale);
        for (int i = 0; i < nj; i++) { jobs[i].fmt = fmt; jobs[i].scale = scale; jobs[i].shift = shift; }
        run_jobs(jobs, nj, enc_job);
        for (int i = 0; i < nj; i++) {
            if (jobs[i].err) { fprintf(stderr, "iqcodec: encoding failed\n"); return 1; }
            tot_inexact += jobs[i].inexact;
            if (jobs[i].inexact && !lossy) {
                fprintf(stderr, "iqcodec: input is not exactly int16 / %g (%lld values); use -s or -l\n", scale, (long long)jobs[i].inexact);
                return 1;
            }
            uint8_t ch[12];
            put32(ch, (uint32_t)jobs[i].n); ch[4] = (uint8_t)shift; ch[5] = ch[6] = ch[7] = 0; put32(ch + 8, (uint32_t)jobs[i].size);
            if (fwrite(ch, 1, 12, out) != 12 || fwrite(jobs[i].enc, 1, jobs[i].size, out) != (size_t)jobs[i].size) goto werr;
            tot_out += 12 + jobs[i].size;
        }
    }
    uint8_t end[4] = {0, 0, 0, 0};
    if (fwrite(end, 1, 4, out) != 4 || fflush(out)) goto werr;
    tot_out += 4;
    if (verbose)
        fprintf(stderr, "%lld -> %lld bytes (%.2f%%, %.3fx), %.2f s%s\n", (long long)tot_in, (long long)tot_out,
                tot_in ? 100.0 * tot_out / tot_in : 0, tot_out ? (double)tot_in / tot_out : 0, now() - t0,
                tot_inexact ? " (lossy: values quantized)" : "");
    for (int i = 0; i < nth; i++) { free(jobs[i].raw); free(jobs[i].enc); }
    return 0;
werr:
    fprintf(stderr, "iqcodec: write error: %s\n", strerror(errno));
    return 1;
}

static int decompress(FILE *in, FILE *out, int nth, int verbose) {
    uint8_t hdr[16];
    if (read_full(in, hdr, 16) != 16 || memcmp(hdr, MAGIC, 4) || hdr[4] != 1 || hdr[5] > 1 || hdr[6] != K_ORDER || get32(hdr + 8) != LEAF) {
        fprintf(stderr, "iqcodec: not an iqcodec v1 stream\n");
        return 1;
    }
    int fmt = hdr[5];
    float scale; memcpy(&scale, hdr + 12, 4);
    size_t ssz = fmt == IQC_FC32 ? 8 : 4;
    Job jobs[64];
    memset(jobs, 0, sizeof jobs);
    int64_t tot = 0, rawcap[64] = {0};
    double t0 = now();
    for (int done = 0; !done;) {
        int nj = 0;
        for (; nj < nth; nj++) {
            uint8_t ch[12];
            if (read_full(in, ch, 4) != 4) goto rerr;
            uint32_t n = get32(ch);
            if (!n) { done = 1; break; }
            if (read_full(in, ch + 4, 8) != 8) goto rerr;
            uint32_t sz = get32(ch + 8);
            if (n > MAX_CHUNK || ch[4] > 3 || sz > (uint64_t)n * 16 + (1 << 20)) goto rerr;
            Job *j = &jobs[nj];
            if (sz > j->cap) { free(j->enc); j->cap = sz; j->enc = malloc(sz); }
            if ((int64_t)n > rawcap[nj]) { free(j->raw); rawcap[nj] = n; j->raw = malloc((size_t)n * ssz); }
            if (!j->enc || !j->raw) { fprintf(stderr, "iqcodec: out of memory\n"); return 1; }
            if (read_full(in, j->enc, sz) != sz) goto rerr;
            j->n = n; j->size = sz; j->shift = ch[4]; j->fmt = fmt; j->scale = scale;
        }
        if (!nj) break;
        run_jobs(jobs, nj, dec_job);
        for (int i = 0; i < nj; i++) {
            if (jobs[i].err) { fprintf(stderr, "iqcodec: corrupt chunk\n"); return 1; }
            if (fwrite(jobs[i].raw, ssz, jobs[i].n, out) != (size_t)jobs[i].n) {
                fprintf(stderr, "iqcodec: write error: %s\n", strerror(errno));
                return 1;
            }
            tot += jobs[i].n * ssz;
        }
    }
    if (fflush(out)) { fprintf(stderr, "iqcodec: write error: %s\n", strerror(errno)); return 1; }
    if (verbose) fprintf(stderr, "%lld bytes written, %.2f s\n", (long long)tot, now() - t0);
    for (int i = 0; i < nth; i++) { free(jobs[i].raw); free(jobs[i].enc); }
    return 0;
rerr:
    fprintf(stderr, "iqcodec: truncated or corrupt input\n");
    return 1;
}

int main(int argc, char **argv) {
    if (argc >= 2 && (!strcmp(argv[1], "-V") || !strcmp(argv[1], "--version"))) { puts("iqcodec " VERSION); return 0; }
    if (argc < 2 || (strcmp(argv[1], "c") && strcmp(argv[1], "d"))) { usage(argc >= 2 && !strcmp(argv[1], "-h") ? stdout : stderr); return argc >= 2 && !strcmp(argv[1], "-h") ? 0 : 2; }
    int dec = argv[1][0] == 'd', fmt = IQC_FC32, lossy = 0, verbose = 0, opt;
    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    int nth = ncpu < 1 ? 1 : ncpu > 8 ? 8 : (int)ncpu;
    float scale = 32767.0f;
    optind = 2;
    while ((opt = getopt(argc, argv, "f:s:lj:vhV")) != -1) {
        switch (opt) {
        case 'f':
            if (!strcmp(optarg, "fc32")) fmt = IQC_FC32;
            else if (!strcmp(optarg, "sc16")) fmt = IQC_SC16;
            else { fprintf(stderr, "iqcodec: unknown format %s\n", optarg); return 2; }
            break;
        case 's': scale = strtof(optarg, NULL); if (!(scale > 0)) { fprintf(stderr, "iqcodec: bad scale\n"); return 2; } break;
        case 'l': lossy = 1; break;
        case 'j': nth = atoi(optarg); nth = nth < 1 ? 1 : nth > 64 ? 64 : nth; break;
        case 'v': verbose = 1; break;
        case 'V': puts("iqcodec " VERSION); return 0;
        case 'h': usage(stdout); return 0;
        default: usage(stderr); return 2;
        }
    }
    if (argc - optind != 2) { usage(stderr); return 2; }
    const char *ip = argv[optind], *op = argv[optind + 1];
    FILE *in = strcmp(ip, "-") ? fopen(ip, "rb") : stdin;
    if (!in) { fprintf(stderr, "iqcodec: %s: %s\n", ip, strerror(errno)); return 1; }
    FILE *out = strcmp(op, "-") ? fopen(op, "wb") : stdout;
    if (!out) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); return 1; }
    int rc = dec ? decompress(in, out, nth, verbose) : compress(in, out, fmt, scale, lossy, nth, verbose);
    if (out != stdout && fclose(out) && !rc) { fprintf(stderr, "iqcodec: %s: %s\n", op, strerror(errno)); rc = 1; }
    if (rc && out != stdout) remove(op);   // do not leave a partial output behind
    if (in != stdin) fclose(in);
    return rc;
}
