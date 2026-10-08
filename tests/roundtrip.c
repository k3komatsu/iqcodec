// Round-trip check of the chunk codec on synthetic signals, both sample formats and every shift.
// Usage: roundtrip               run the checks (exit 1 on the first failure)
//        roundtrip gen N FILE    write N fc32 samples (tone + sigma-delta low bits, exact int16/32767)
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

int main(int argc, char **argv) {
    if (argc == 4 && !strcmp(argv[1], "gen")) {
        int64_t n = atoll(argv[2]);
        int16_t *x = malloc(n * 4);
        float *f = malloc(n * 8);
        make(5, x, n);
        for (int64_t i = 0; i < 2 * n; i++) f[i] = (float)x[i] * (1.0f / 32767.0f);
        FILE *o = fopen(argv[3], "wb");
        return !o || fwrite(f, 8, n, o) != (size_t)n || fclose(o);
    }
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
