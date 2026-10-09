// Round-trip check of the chunk codec on synthetic signals, both sample formats and every shift.
// Usage: roundtrip               run the checks (exit 1 on the first failure)
//        roundtrip gen N FILE [SIGNAL [sc16]]   write N samples of signal 0-7 (default 5: tone + sigma-delta low
//                                bits), fc32 (exact int16/32767) or sc16
//        roundtrip wrap K LEAF SHIFT IN.sc16 OUT.iqc   write a stream with any K / leaf length / shift (for testing
//                                readers; iqcodec itself writes K 24, leaf 8192 and picks the shift)
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "crc32c.h"
#include "iqc.h"

static uint64_t rs = 88172645463325252ull;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return (uint32_t)rs; }
static double gauss(void) { double s = 0; for (int i = 0; i < 12; i++) s += rnd() / 4294967296.0; return s - 6; }
static int16_t clip(double v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)lrint(v); }

static void make(int c, int16_t *x, int64_t n) {
    double acc = 0;
    for (int64_t t = 0; t < n; t++) {
        int16_t *p = x + 2 * t;
        switch (c) {
        case 0: p[0] = rnd(); p[1] = rnd(); break;                                     // white int16
        case 1: { static const int16_t v[3] = {-32768, 32767, 0}; p[0] = v[rnd() % 3]; p[1] = v[rnd() % 3]; break; }
        case 2: p[0] = p[1] = 0; break;                                                // zeros
        case 3: p[0] = clip(30000 * sin(t * 0.01)); p[1] = clip(30000 * sin(t * 0.0103)); break;   // pure tone
        case 4: case 5: case 6: {                                                      // tone + bits-bit sigma-delta
            int bits = c - 3;
            double prev = floor(acc + 0.2);
            acc += 0.37 + 0.3 * sin(t / 5000.0) / 1000;
            int l = (int)(floor(acc + 0.2) - prev) & ((1 << bits) - 1);
            int base = (int)lrint(700 * sin(t * 0.013)) * (1 << bits);
            p[0] = clip(base + l); p[1] = clip(base / 3 + l); break;
        }
        case 7: { double g = (t / 1000) % 2 ? 2000 : 2; p[0] = clip(g * gauss()); p[1] = clip(g * gauss()); break; }   // bursts
        }
    }
}

static int check(const char *name, const int16_t *x, int64_t n) {
    float *f = malloc(n * 8 + 8), *fo = malloc(n * 8 + 8);
    int16_t *so = malloc(n * 4 + 4);
    int64_t cap = n * 8 + (1 << 20);
    uint8_t *buf = malloc(cap);
    for (int64_t i = 0; i < 2 * n; i++) f[i] = (float)x[i] * (1.0f / 32767.0f);
    int ok = 1;
    for (int shift = 0; shift < 4 && ok; shift++) {
        int64_t inexact, sz = iqc_encode(x, IQC_SC16, n, 32767.f, shift, 24, 8192, 11, buf, cap, &inexact);
        ok &= sz > 0 && sz <= cap && !iqc_decode(buf, sz, so, IQC_SC16, n, 32767.f, shift, 24, 8192) && !memcmp(so, x, n * 4);
        sz = iqc_encode(f, IQC_FC32, n, 32767.f, shift, 24, 8192, 11, buf, cap, &inexact);
        ok &= sz > 0 && sz <= cap && inexact == 0 && !iqc_decode(buf, sz, fo, IQC_FC32, n, 32767.f, shift, 24, 8192) && !memcmp(fo, f, n * 8);
    }
    // fc32 that is not int16 / scale must be reported
    f[0] += 1e-6f;
    int64_t inexact = 0;
    iqc_encode(f, IQC_FC32, n, 32767.f, 2, 24, 8192, 11, buf, cap, &inexact);
    ok &= inexact >= 1;
    printf("%s  %s\n", ok ? "ok  " : "FAIL", name);
    free(f); free(fo); free(so); free(buf);
    return ok;
}

// Corrupt or crafted chunks must be rejected or decoded without memory errors (run under ASan/UBSan in CI).
static int robustness(void) {
    int ok = 1;
    {   // table frequency that wraps around 32 bits (used to overflow the decoder's lookup table)
        uint8_t b[64] = {0}; int bp = 0;
#define BIT(v) do { if (v) b[16 + bp / 8] |= 1 << (bp % 8); bp++; } while (0)
        BIT(1); BIT(0); BIT(1); BIT(1);
        for (int i = 0; i < 31; i++) BIT(0);
        BIT(1);
        for (int i = 0; i < 31; i++) BIT(1);
#undef BIT
        uint32_t lens[4] = {(uint32_t)(bp + 7) / 8, 0, 0, 8};
        for (int i = 0; i < 4; i++) for (int k = 0; k < 4; k++) b[4 * i + k] = (uint8_t)(lens[i] >> (8 * k));
        int16_t out[2];
        ok &= iqc_decode(b, 16 + lens[0] + 8, out, IQC_SC16, 1, 32767.f, 0, 24, 8192) == -1;
    }
    int64_t n = 20000, cap = n * 8 + (1 << 20), inexact;
    int16_t *x = malloc(n * 4), *y = malloc(n * 4);
    uint8_t *buf = malloc(cap), *mut = malloc(cap);
    make(5, x, n);
    int64_t sz = iqc_encode(x, IQC_SC16, n, 32767.f, 2, 24, 8192, 11, buf, cap, &inexact);
    for (int it = 0; it < 3000; it++) {   // mutated valid chunks, then random bytes
        int64_t len = sz;
        if (it < 2000) { memcpy(mut, buf, sz); for (int k = 1 + rnd() % 4; k--;) mut[rnd() % sz] ^= (uint8_t)(1 + rnd() % 255); }
        else { len = 16 + rnd() % 4096; for (int64_t i = 0; i < len; i++) mut[i] = (uint8_t)rnd(); }
        int r = iqc_decode(mut, len, y, IQC_SC16, n, 32767.f, rnd() % 4, 24, 8192);
        ok &= r == 0 || r == -1;
    }
    // invalid parameters are rejected, never undefined behaviour
    ok &= iqc_encode(x, IQC_SC16, n, 32767.f, 4, 24, 8192, 11, buf, cap, &inexact) == -1;
    ok &= iqc_encode(x, IQC_SC16, n, 32767.f, 2, 0, 8192, 11, buf, cap, &inexact) == -1;
    ok &= iqc_encode(x, IQC_SC16, n, 32767.f, 2, 33, 8192, 11, buf, cap, &inexact) == -1;
    ok &= iqc_encode(x, IQC_SC16, n, 32767.f, 2, 24, 0, 11, buf, cap, &inexact) == -1;
    ok &= iqc_encode(x, IQC_SC16, -1, 32767.f, 2, 24, 8192, 11, buf, cap, &inexact) == -1;
    ok &= iqc_decode(buf, sz, y, IQC_SC16, n, 32767.f, 9, 24, 8192) == -1;
    ok &= iqc_decode(buf, sz, y, IQC_SC16, n, 32767.f, 2, 24, 0) == -1;
    // tiny leaf blocks (lots of side information) still encode and round-trip
    make(0, x, n);
    sz = iqc_encode(x, IQC_SC16, n, 32767.f, 0, 32, 16, 11, buf, cap, &inexact);
    ok &= sz > 0 && sz <= cap && !iqc_decode(buf, sz, y, IQC_SC16, n, 32767.f, 0, 32, 16) && !memcmp(x, y, n * 4);
    free(x); free(y); free(buf); free(mut);
    printf("%s  corrupt input / parameters\n", ok ? "ok  " : "FAIL");
    return ok;
}

int main(int argc, char **argv) {
    if (argc >= 4 && argc <= 6 && !strcmp(argv[1], "gen")) {
        int64_t n = atoll(argv[2]);
        int sig = argc > 4 ? atoi(argv[4]) : 5, sc16 = argc > 5 && !strcmp(argv[5], "sc16");
        int16_t *x = malloc(n * 4);
        float *f = malloc(n * 8);
        make(sig, x, n);
        for (int64_t i = 0; i < 2 * n; i++) f[i] = (float)x[i] * (1.0f / 32767.0f);
        FILE *o = fopen(argv[3], "wb");
        int bad = !o || (sc16 ? fwrite(x, 4, n, o) : fwrite(f, 8, n, o)) != (size_t)n || fclose(o);
        free(x); free(f);
        return bad;
    }
    if (argc == 7 && !strcmp(argv[1], "wrap")) {   // the container of FORMAT.md around iqc_encode chunks
        int K = atoi(argv[2]), leaf = atoi(argv[3]), shift = atoi(argv[4]);
        enum { CH = 1 << 21 };
        FILE *in = fopen(argv[5], "rb"), *o = fopen(argv[6], "wb");
        int16_t *x = malloc((size_t)CH * 4);
        uint8_t *buf = malloc((size_t)CH * 8 + (1 << 20)), h[16] = {'I', 'Q', 'C', 'D', 2, IQC_SC16, (uint8_t)K, 11};
        float scale = 32767.f;
        uint64_t tot = 0;
        int64_t m, inexact;
        if (!in || !o || !x || !buf) return 1;
        h[8] = leaf; h[9] = leaf >> 8; h[10] = leaf >> 16; h[11] = leaf >> 24; memcpy(h + 12, &scale, 4);
        fwrite(h, 1, 16, o);
        while ((m = (int64_t)fread(x, 4, CH, in)) > 0) {
            int64_t sz = iqc_encode(x, IQC_SC16, m, scale, shift, K, leaf, 11, buf, (int64_t)CH * 8 + (1 << 20), &inexact);
            uint32_t c = crc32c(0, x, (size_t)m * 4), hd[4] = {(uint32_t)m, (uint32_t)shift, (uint32_t)sz, c};
            if (sz < 0) return 1;
            for (int i = 0; i < 16; i++) h[i] = (uint8_t)(hd[i / 4] >> (8 * (i % 4)));   // little-endian u32s
            fwrite(h, 1, 16, o); fwrite(buf, 1, (size_t)sz, o);
            tot += (uint64_t)m;
        }
        memset(h, 0, 12);
        for (int i = 0; i < 8; i++) h[4 + i] = (uint8_t)(tot >> (8 * i));
        fwrite(h, 1, 12, o);
        free(x); free(buf); fclose(in);
        return fclose(o) != 0;
    }
    {   // CRC-32C: check value, and hardware / table paths against a bitwise reference on odd lengths
        uint8_t b[1027];
        for (int i = 0; i < 1027; i++) b[i] = (uint8_t)rnd();
        uint32_t ref = ~0u;
        for (int i = 0; i < 1027; i++) { ref ^= b[i]; for (int k = 0; k < 8; k++) ref = ref & 1 ? (ref >> 1) ^ 0x82F63B78u : ref >> 1; }
        int ok = crc32c(0, "123456789", 9) == 0xE3069283u && crc32c(0, b, 1027) == ~ref && crc32c(crc32c(0, b, 500), b + 500, 527) == ~ref;
        printf("%s  crc32c\n", ok ? "ok  " : "FAIL");
        if (!ok) return 1;
    }
    if (!robustness()) return 1;
    static const char *names[8] = {"white int16", "full-scale extremes", "zeros", "pure tone (ill-conditioned LPC)",
                                   "tone + 1-bit sigma-delta", "tone + 2-bit sigma-delta", "tone + 3-bit sigma-delta", "bursty noise"};
    static const int64_t lens[3] = {1, 7, 300001};
    int ok = 1;
    for (int c = 0; c < 8; c++)
        for (int k = 0; k < 3; k++) {
            if (k < 2 && c) continue;   // tiny lengths: once is enough
            int16_t *x = malloc(lens[k] * 4);
            make(c, x, lens[k]);
            char name[96]; snprintf(name, sizeof name, "%s, n=%lld", names[c], (long long)lens[k]);
            ok &= check(name, x, lens[k]);
            free(x);
        }
    return !ok;
}
