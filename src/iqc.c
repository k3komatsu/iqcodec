// iqcodec: lossless codec for IQ captures (int16 samples, or complex float32 that are int16 / scale).
// Chunk codec (speed-oriented, FLAC-like asymmetry):
//   low bits : sigma-delta / digital-straight-line tracker; deterministic stretches cost nothing,
//              exceptions are gap-coded, ambiguous steps cost one adaptive bit
//   high part: per-block widely-linear (joint I/Q) LPC fitted by the encoder (block length chosen per
//              superblock), integer coefficients in the stream -> the decoder only runs an int32 FIR
//   residual : class symbol rANS-coded with per-chunk static tables selected by a backward
//              magnitude context; remaining mantissa bits and sign go to a raw bit stream
// The decoder is integer-only; floating point is used only by the encoder's least-squares fit.
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "iqc.h"
#if defined(__aarch64__) && defined(__ARM_NEON)
#define IQC_NEON 1
#include <arm_neon.h>
#endif

#define NCTX 64
#define MANT_BITS 2      // top mantissa bits folded into the rANS symbol (part of the stream format)
#define MAXB 16          // residual magnitude < 2^16
#define NSYM 64          // >= symbols in use: class 0, then per class b >= 1: 2^min(b-1, MANT_BITS)
#define PROB_BITS 12
#define PROB_SCALE (1 << PROB_BITS)

static inline int nbits(uint32_t m) { return m ? 32 - __builtin_clz(m) : 0; }

// ---------------- forward bit streams (LSB first) ----------------
typedef struct { uint8_t *buf; size_t cap, pos; uint64_t acc; int n; } BitW;
static inline void bw_put(BitW *w, uint32_t v, int nb) {  // nb <= 32, v < 2^nb
    w->acc |= (uint64_t)v << w->n; w->n += nb;
    if (w->n >= 32) {
        if (w->pos + 4 <= w->cap) { uint32_t lo = (uint32_t)w->acc; uint8_t *p = w->buf + w->pos; p[0] = lo; p[1] = lo >> 8; p[2] = lo >> 16; p[3] = lo >> 24; }
        w->pos += 4; w->acc >>= 32; w->n -= 32;
    }
}
static void bw_flush(BitW *w) {
    while (w->n > 0) { if (w->pos < w->cap) w->buf[w->pos] = (uint8_t)w->acc; w->pos++; w->acc >>= 8; w->n -= 8; }
    w->n = 0;
}

typedef struct { const uint8_t *buf; size_t len, pos; uint64_t acc; int n; } BitR;
static inline uint32_t br_get(BitR *r, int nb) {  // nb <= 32
    if (r->n < nb) {
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        if (r->pos + 8 <= r->len) {   // branch-light refill: load 8 bytes, keep whole bytes that fit
            uint64_t w; memcpy(&w, r->buf + r->pos, 8);   // little-endian hosts
            r->acc |= w << r->n; r->pos += (63 - r->n) >> 3; r->n |= 56;
        } else
#endif
        while (r->n < nb) { r->acc |= (uint64_t)(r->pos < r->len ? r->buf[r->pos] : 0) << r->n; r->pos++; r->n += 8; }
    }
    uint32_t v = (uint32_t)(r->acc & ((1ull << nb) - 1));
    r->acc >>= nb; r->n -= nb;
    return v;
}
static void gamma_put(BitW *w, uint32_t v) { int nb = nbits(v); bw_put(w, 1u << (nb - 1), nb); bw_put(w, v & ((1u << (nb - 1)) - 1), nb - 1); }
static uint32_t gamma_get(BitR *r) { int z = 0; while (!br_get(r, 1) && z < 31) z++; return (1u << z) | br_get(r, z); }

// ---------------- binary range coder (low-bit events only) ----------------
typedef struct { uint64_t low; uint32_t range; uint8_t cache; uint64_t csize; uint8_t *out; size_t pos, cap; } Enc;
typedef struct { uint32_t range, code; const uint8_t *in; size_t pos, len; } Dec;
typedef struct { Enc *e; Dec *d; } Coder;  // e != NULL -> encoding

static void shift_low(Enc *e) {
    if ((uint32_t)e->low < 0xFF000000u || (e->low >> 32)) {
        uint8_t carry = (uint8_t)(e->low >> 32), t = e->cache;
        do { if (e->pos < e->cap) e->out[e->pos] = t + carry; e->pos++; t = 0xFF; } while (--e->csize);
        e->cache = (uint8_t)(e->low >> 24);
    }
    e->csize++;
    e->low = (e->low & 0x00FFFFFFu) << 8;
}
static uint8_t in_byte(Dec *d) { return d->pos < d->len ? d->in[d->pos++] : 0; }

typedef struct { uint16_t p, n; } Ctr;  // adaptive P(1), 16 bit, rate 1/(n+2) down to 1/32
#define CTR_INIT ((Ctr){32768, 0})
static inline int code_ctr(Coder *c, Ctr *t, int bit) {
    int p1 = t->p >> 4; p1 = p1 < 1 ? 1 : p1 > 4095 ? 4095 : p1;
    if (c->e) {
        Enc *e = c->e;
        uint32_t bound = (e->range >> 12) * (uint32_t)p1;
        if (bit) e->range = bound; else { e->low += bound; e->range -= bound; }
        while (e->range < (1u << 24)) { e->range <<= 8; shift_low(e); }
    } else {
        Dec *d = c->d;
        uint32_t bound = (d->range >> 12) * (uint32_t)p1;
        if (d->code < bound) { d->range = bound; bit = 1; } else { d->code -= bound; d->range -= bound; bit = 0; }
        while (d->range < (1u << 24)) { d->range <<= 8; d->code = (d->code << 8) | in_byte(d); }
    }
    int target = bit ? 65535 : 0;
    t->p += (target - t->p) / (t->n + 2);
    if (t->n < 30) t->n++;
    return bit;
}
static int tree_code(Coder *c, Ctr *t, int nbit, int sym) {
    int node = 1;
    for (int i = nbit - 1; i >= 0; i--) { int bit = code_ctr(c, &t[node], (sym >> i) & 1); node = node * 2 + bit; }
    return node - (1 << nbit);
}
static uint32_t gap_code(Coder *c, Ctr *t, uint32_t v) {  // Elias-gamma of v >= 1, adaptive prefix
    int nb = c->e ? nbits(v) : 0, z = 0;
    while (!code_ctr(c, &t[z < 31 ? z : 31], c->e ? z == nb - 1 : 0) && z < 31) z++;
    uint32_t r = 1;
    for (int i = z - 1; i >= 0; i--) { Ctr half = {32768, 30}; r = r * 2 + code_ctr(c, &half, c->e ? (v >> i) & 1 : 0); }
    return r;
}

// ---------------- low bits: digital straight segment tracker ----------------
// The symbols behave like first-order error diffusion of a slowly drifting value, so the cumulative sum
// Y_t of the unwrapped symbols is a digital straight line over long stretches. With base step k the path
// (t, Y_t - k*t) has steps {0, 1}; it is tracked with the arithmetic DSS recognition of Debled-Rennesson
// (integer, O(1) per point): mu <= a*x - b*y < mu + b with leaning points U (r = mu) and L (r = mu + b - 1).
// The next point is predicted by testing both candidate steps; on a break the longest straight suffix of
// the recent history is rebuilt by running the same recognition backwards.
#define DSL_HIST 64   // power of two: hist[t & (DSL_HIST - 1)]
typedef struct { int64_t x, y; } Pt;
typedef struct { int64_t a, b, mu, rl; Pt Uf, Ul, Lf, Ll, last; } Dss;   // rl = a*last.x - b*last.y
typedef struct {
    Dss d; int has, k;
    int64_t t, Y;                    // time of the last point, cumulative sum of unwrapped symbols
    int16_t hist[DSL_HIST];          // unwrapped symbols by time
} Trk;

static void dss_start(Dss *d, Pt p) { d->a = 0; d->b = 1; d->mu = d->rl = -p.y; d->Uf = d->Ul = d->Lf = d->Ll = d->last = p; }
// Add m (m.x = last.x + 1, m.y - last.y = step in {0, 1}); returns 0 if the points are no longer straight.
static inline int dss_add(Dss *d, Pt m, int step) {
    int64_t r = d->rl + d->a - step * d->b;   // a*m.x - b*m.y, incrementally
    if (r >= d->mu && r < d->mu + d->b) {
        if (r == d->mu) d->Ul = m;
        if (r == d->mu + d->b - 1) d->Ll = m;
        d->rl = r;
    } else if (r == d->mu - 1) {
        d->Lf = d->Ll; d->Ul = m;
        d->a = m.y - d->Uf.y; d->b = m.x - d->Uf.x; d->rl = d->mu = d->a * m.x - d->b * m.y;
    } else if (r == d->mu + d->b) {
        d->Uf = d->Ul; d->Ll = m;
        d->a = m.y - d->Lf.y; d->b = m.x - d->Lf.x; d->mu = d->a * m.x - d->b * m.y - d->b + 1; d->rl = d->mu + d->b - 1;
    } else return 0;
    d->last = m;
    return 1;
}

enum { MODE_D = 0, MODE_A = 1, MODE_N = 2 };
// Prediction for the next symbol: returns mode; *v0 = lowest candidate (unwrapped); *actx for MODE_A.
static inline int trk_predict(const Trk *k, int *v0, int *actx) {
    if (!k->has) return MODE_N;
    const Dss *d = &k->d;
    int64_t r0 = d->rl + d->a, r1 = r0 - d->b;
    int ok0 = r0 >= d->mu - 1 && r0 <= d->mu + d->b, ok1 = r1 >= d->mu - 1;
    *v0 = k->k + !ok0;
    if (ok0 & ok1) { int lb = nbits((uint32_t)(d->b < 32767 ? d->b : 32767)); *actx = (r0 == d->mu + d->b) | lb << 1; return MODE_A; }
    return MODE_D;
}
static void trk_rebuild(Trk *k) {
    // longest run of recent steps (newest first) spanning at most two adjacent values
    int hn = k->t + 1 < DSL_HIST ? (int)k->t + 1 : DSL_HIST, lo = k->hist[k->t & (DSL_HIST - 1)], hi = lo, L = 1;
    for (int j = 1; j < hn; j++) {
        int u = k->hist[(k->t - j) & (DSL_HIST - 1)];
        int nlo = u < lo ? u : lo, nhi = u > hi ? u : hi;
        if (nhi - nlo > 1) break;
        lo = nlo; hi = nhi; L++;
    }
    k->k = lo;
    // backwards recognition on mirrored points p' = M' - p (steps stay in {0, 1})
    Dss r; Pt p = {0, 0};
    dss_start(&r, p);
    for (int j = 0; j < L; j++) {
        int u = k->hist[(k->t - j) & (DSL_HIST - 1)];
        Pt q = {p.x + 1, p.y + (u - k->k)};
        if (!dss_add(&r, q, u - k->k)) break;
        p = q;
    }
    // back to forward orientation: mirrored upper/lower and first/last swap
    Pt M = {k->t, k->Y - (int64_t)k->k * k->t};
    Dss *d = &k->d;
    int64_t C = r.a * M.x - r.b * M.y;
    d->a = r.a; d->b = r.b; d->mu = C - r.mu - r.b + 1; d->rl = C;
#define CONV(q) ((Pt){M.x - (q).x, M.y - (q).y})
    d->Uf = CONV(r.Ll); d->Ul = CONV(r.Lf); d->Lf = CONV(r.Ul); d->Ll = CONV(r.Uf); d->last = M;
#undef CONV
    k->has = 1;
}
static void trk_init(Trk *k) { memset(k, 0, sizeof *k); k->t = -1; }
static inline void trk_update(Trk *k, int u) {
    k->t++; k->Y += u;
    k->hist[k->t & (DSL_HIST - 1)] = (int16_t)u;
    int step = u - k->k;
    if (k->has && (step == 0 || step == 1) && dss_add(&k->d, (Pt){k->t, k->d.last.y + step}, step)) return;
    trk_rebuild(k);
}
// Unwrapped value of a raw symbol: the representative closest to the previous unwrapped symbol.
static inline int unwrap(const Trk *k, int sym, int M) {
    int ref = k->t >= 0 ? k->hist[k->t & (DSL_HIST - 1)] : sym;
    int d = (sym - ref) & (M - 1);
    return ref + (d * 2 > M ? d - M : d);
}

typedef struct { Ctr amb[32], exc[3][16], gap[32]; } LowModel;
static void low_init(LowModel *m) {
    Ctr *p = (Ctr *)m;
    for (size_t i = 0; i < sizeof *m / sizeof(Ctr); i++) p[i] = CTR_INIT;
}

// ---------------- rANS (32-bit state, 16-bit little-endian words, 12-bit probabilities) ----------------
#define RANS_L (1u << 16)
typedef struct { uint16_t freq[NSYM], start[NSYM]; } SymTab;
// Division-free encoding: q = floor(x / f) = (x * ceil(2^64 / f)) >> 64 is exact for x < 2^32, f <= 2^12.
// (The 32-bit reciprocal trick of ryg_rans needs x < 2^31, which 16-bit-word rANS does not guarantee.)
typedef struct { uint64_t x_max, rcp; uint32_t freq, start; } EncSym;
static EncSym enc_sym(uint32_t freq, uint32_t start) {
    EncSym e = {((uint64_t)(RANS_L >> PROB_BITS) << 16) * freq, freq == 1 ? 0 : UINT64_MAX / freq + 1, freq, start};
    return e;
}

static int normalize(const uint32_t *cnt, SymTab *t) {  // returns 0 if the context is unused
    uint64_t tot = 0; for (int s = 0; s < NSYM; s++) tot += cnt[s];
    memset(t, 0, sizeof *t);
    if (!tot) return 0;
    int sum = 0, big = 0;
    for (int s = 0; s < NSYM; s++) {
        if (!cnt[s]) continue;
        int f = (int)((cnt[s] * (uint64_t)PROB_SCALE + tot / 2) / tot);
        t->freq[s] = f < 1 ? 1 : f; sum += t->freq[s];
        if (t->freq[s] > t->freq[big]) big = s;
    }
    t->freq[big] += PROB_SCALE - sum;
    for (int s = 0, c = 0; s < NSYM; s++) { t->start[s] = c; c += t->freq[s]; }
    return 1;
}

// ---------------- residual symbolization ----------------
// Symbol = magnitude class b = nbits(|r|) plus its top min(b-1, MANT_BITS) mantissa bits;
// the remaining mantissa bits and the sign go to the raw stream.
static int cls_base[MAXB + 2];
typedef struct { uint8_t nraw; uint32_t base; } SymInfo;   // raw bit count, magnitude without the raw mantissa
static SymInfo sym_info[NSYM];
static void init_syms_once(void) {
    int idx = 1;
    sym_info[0] = (SymInfo){0, 0}; cls_base[0] = 0;
    for (int b = 1; b <= MAXB; b++) {
        int nm = b - 1, tb = nm < MANT_BITS ? nm : MANT_BITS;
        cls_base[b] = idx;
        for (int top = 0; top < 1 << tb; top++, idx++)
            sym_info[idx] = (SymInfo){(uint8_t)(nm - tb + 1), (1u << nm) | ((uint32_t)top << (nm - tb))};
    }
}
static void init_syms(void) { static pthread_once_t once = PTHREAD_ONCE_INIT; pthread_once(&once, init_syms_once); }
static inline int res_sym(int r, uint32_t *raw, int *nraw) {
    uint32_t m = r < 0 ? -r : r;
    int b = nbits(m);
    if (b == 0) { *nraw = 0; *raw = 0; return 0; }
    int nm = b - 1, tb = nm < MANT_BITS ? nm : MANT_BITS, lowb = nm - tb;
    *nraw = lowb + 1;
    *raw = ((m & ((1u << lowb) - 1)) << 1) | (r < 0);
    return cls_base[b] + (int)((m >> lowb) & ((1u << tb) - 1));
}
static inline int res_from(int sym, BitR *raw) {
    SymInfo si = sym_info[sym];
    uint32_t v = br_get(raw, si.nraw);
    int m = (int)(si.base + (v >> 1)), sg = -(int)(v & 1);
    return (m ^ sg) - sg;
}

// ---------------- prediction ----------------
static inline int32_t fir(const int32_t *w, const int32_t *z, int len) {  // z: interleaved I/Q history
    int32_t s = 0;
    for (int i = 0; i < len; i++) s += w[i] * z[i];
    return s;
}
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

#define RTILE 512   // residual tile (tstep > 1 evaluates only every tstep-th tile, for cost estimates)
// Encoder-side arrays hold the high parts as interleaved int16 (I, Q) pairs with K zero samples of padding.
// IQC_NO_SIMD=1 selects the portable code paths (same output), read once.
static int no_simd;
static void read_no_simd(void) { no_simd = getenv("IQC_NO_SIMD") != NULL; }
static int simd_disabled(void) { static pthread_once_t once = PTHREAD_ONCE_INIT; pthread_once(&once, read_no_simd); return no_simd; }
// Exact int64 sums C_{0,j} = sum_{t in [s,e)} z_t z_{t-j}^T (additive over adjacent ranges).
// Portable version on planar int32 copies (vectorizes over t without SIMD intrinsics).
static void cov_sums(const int32_t *I, const int32_t *Q, int64_t s, int64_t e, int K, int64_t (*S)[4]) {
    for (int j = 0; j <= K; j++) {
        int64_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;
        for (int64_t t = s; t < e; t++) {
            s0 += (int64_t)I[t] * I[t - j]; s1 += (int64_t)I[t] * Q[t - j];
            s2 += (int64_t)Q[t] * I[t - j]; s3 += (int64_t)Q[t] * Q[t - j];
        }
        S[j][0] = s0; S[j][1] = s1; S[j][2] = s2; S[j][3] = s3;
    }
}
#if defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__))
#define IQC_X86 1
#include <immintrin.h>
static int avx512vnni_ok;
static void detect_avx512vnni(void) {
    __builtin_cpu_init();
    avx512vnni_ok = __builtin_cpu_supports("avx512f") && __builtin_cpu_supports("avx512bw") &&
                    __builtin_cpu_supports("avx512vnni") && !simd_disabled();
}
static int have_avx512vnni(void) {
    static pthread_once_t once = PTHREAD_ONCE_INIT;
    pthread_once(&once, detect_avx512vnni);
    return avx512vnni_ok;
}
// Same sums from int16 (I, Q) pairs with vpdpwssd: (I,Q).(I',Q') = s0+s3, (I,Q).(Q',I') = s1+s2, and with Q
// negated the differences. Needs |z| <= 2^13: each step adds < 2^27 per int32 lane, flushed to int64 every 8.
__attribute__((target("avx512f,avx512bw,avx512vnni")))
static void cov_sums_avx512(const int16_t *z, int64_t s, int64_t e, int K, int64_t (*S)[4]) {
    const __m512i Z = _mm512_setzero_si512();
    for (int j = 0; j <= K; j++) {
        __m512i a64[4] = {Z, Z, Z, Z}, a32[4] = {Z, Z, Z, Z};
        int cnt = 0;
        for (int64_t t = s; t < e; t += 16) {
            __mmask32 m = e - t >= 16 ? 0xFFFFFFFFu : (__mmask32)((1ull << (2 * (e - t))) - 1);
            __m512i A = _mm512_maskz_loadu_epi16(m, z + 2 * t), B = _mm512_maskz_loadu_epi16(m, z + 2 * (t - j));
            __m512i An = _mm512_mask_sub_epi16(A, 0xAAAAAAAAu, Z, A), Bs = _mm512_rol_epi32(B, 16);
            a32[0] = _mm512_dpwssd_epi32(a32[0], A, B);  a32[1] = _mm512_dpwssd_epi32(a32[1], A, Bs);
            a32[2] = _mm512_dpwssd_epi32(a32[2], An, B); a32[3] = _mm512_dpwssd_epi32(a32[3], An, Bs);
            if (++cnt == 8 || t + 16 >= e) {
                for (int k = 0; k < 4; k++) {
                    a64[k] = _mm512_add_epi64(a64[k], _mm512_cvtepi32_epi64(_mm512_castsi512_si256(a32[k])));
                    a64[k] = _mm512_add_epi64(a64[k], _mm512_cvtepi32_epi64(_mm512_extracti64x4_epi64(a32[k], 1)));
                    a32[k] = Z;
                }
                cnt = 0;
            }
        }
        int64_t p0 = _mm512_reduce_add_epi64(a64[0]), p1 = _mm512_reduce_add_epi64(a64[1]);
        int64_t m0 = _mm512_reduce_add_epi64(a64[2]), m1 = _mm512_reduce_add_epi64(a64[3]);
        S[j][0] = (p0 + m0) / 2; S[j][3] = (p0 - m0) / 2; S[j][1] = (p1 + m1) / 2; S[j][2] = (p1 - m1) / 2;
    }
}
// Residuals, 16 samples per vector: one vpdpwssd per tap adds a*I + b*Q (int16 pairs x coefficient pair).
// Same integers as the scalar version (int16 coefficients, no overflow).
__attribute__((target("avx512f,avx512bw,avx512vnni")))
static void block_residuals_avx512(const int16_t *z, int64_t s, int64_t e, int K, const int32_t *q, int sh,
                                   int hmin, int hmax, int32_t *rI, int32_t *rQ, int tstep) {
    __m512i cI[32], cQ[32];
    for (int j = 1; j <= K; j++) {
        cI[j - 1] = _mm512_set1_epi32((int32_t)((uint32_t)(q[2 * j - 2] & 0xFFFF) | (uint32_t)q[2 * j - 1] << 16));
        cQ[j - 1] = _mm512_set1_epi32((int32_t)((uint32_t)(q[2 * K + 2 * j - 2] & 0xFFFF) | (uint32_t)q[2 * K + 2 * j - 1] << 16));
    }
    const __m512i rnd = _mm512_set1_epi32(sh ? 1 << (sh - 1) : 0), c0 = _mm512_set1_epi32(q[4 * K]), shv = _mm512_set1_epi32(sh);
    const __m512i lo = _mm512_set1_epi32(hmin), hi = _mm512_set1_epi32(hmax);
    for (int64_t t0 = s; t0 < e; t0 += (int64_t)RTILE * tstep) {
        int64_t t1 = t0 + RTILE < e ? t0 + RTILE : e;
        for (int64_t t = t0; t < t1; t += 16) {
            __mmask16 m = t1 - t >= 16 ? 0xFFFF : (__mmask16)((1u << (t1 - t)) - 1);
            __m512i cur = _mm512_maskz_loadu_epi32(m, z + 2 * t);   // dword = (I, Q)
            __m512i I = _mm512_srai_epi32(_mm512_slli_epi32(cur, 16), 16), Q = _mm512_srai_epi32(cur, 16);
            __m512i aI = rnd, aQ = _mm512_add_epi32(rnd, _mm512_mullo_epi32(c0, I));
            for (int j = 1; j <= K; j++) {
                __m512i h = _mm512_loadu_si512(z + 2 * (t - j));
                aI = _mm512_dpwssd_epi32(aI, h, cI[j - 1]);
                aQ = _mm512_dpwssd_epi32(aQ, h, cQ[j - 1]);
            }
            __m512i pI = _mm512_min_epi32(_mm512_max_epi32(_mm512_srav_epi32(aI, shv), lo), hi);
            __m512i pQ = _mm512_min_epi32(_mm512_max_epi32(_mm512_srav_epi32(aQ, shv), lo), hi);
            _mm512_mask_storeu_epi32(rI + (t - s), m, _mm512_sub_epi32(I, pI));
            _mm512_mask_storeu_epi32(rQ + (t - s), m, _mm512_sub_epi32(Q, pQ));
        }
    }
}
#endif
// Residuals of block [s,e) for natural-order coefficients q (encoder only): same integer results as the
// decoder's per-sample FIR (no overflow, so summation order is irrelevant), but vectorized over t.
// tstep > 1 computes only every tstep-th tile (cost estimation).
static void block_residuals(const int32_t *I, const int32_t *Q, int64_t s, int64_t e, int K, const int32_t *q, int sh,
                            int hmin, int hmax, int32_t *rI, int32_t *rQ, int tstep) {
    int32_t acc[RTILE], rnd = sh ? 1 << (sh - 1) : 0;
    for (int64_t t0 = s; t0 < e; t0 += (int64_t)RTILE * tstep) {
        int n = (int)(e - t0 < RTILE ? e - t0 : RTILE);
        const int32_t *Is = I + t0, *Qs = Q + t0;
        for (int t = 0; t < n; t++) acc[t] = rnd;
        for (int j = 1; j <= K; j++) {
            int32_t a = q[2 * j - 2], b = q[2 * j - 1];
            for (int t = 0; t < n; t++) acc[t] += a * Is[t - j] + b * Qs[t - j];
        }
        for (int t = 0; t < n; t++) rI[t0 - s + t] = Is[t] - clampi(acc[t] >> sh, hmin, hmax);
        for (int t = 0; t < n; t++) acc[t] = rnd + q[4 * K] * Is[t];
        for (int j = 1; j <= K; j++) {
            int32_t a = q[2 * K + 2 * j - 2], b = q[2 * K + 2 * j - 1];
            for (int t = 0; t < n; t++) acc[t] += a * Is[t - j] + b * Qs[t - j];
        }
        for (int t = 0; t < n; t++) rQ[t0 - s + t] = Qs[t] - clampi(acc[t] >> sh, hmin, hmax);
    }
}
#if IQC_NEON
// Covariance sums from interleaved int16 pairs with SMLAL: A*B gives lanes (I I', Q Q') -> s0, s3; A*rev(B) gives
// (I Q', Q I') -> s1, s2. Two lags per pass share the loads of z_t. Needs |z| <= 2^13: one product < 2^26 per lane
// per step, 16 steps per int32 accumulator before it is widened to int64. Same sums as cov_sums.
static void cov_sums_neon(const int16_t *z, int64_t s, int64_t e, int K, int64_t (*S)[4]) {
    for (int j = 0; j <= K; j += 2) {
        int j2 = j + 1 <= K ? j + 1 : j;
        int64x2_t P0 = vdupq_n_s64(0), X0 = P0, P1 = P0, X1 = P0;
        int64_t t = s;
        while (t + 4 <= e) {
            int32x4_t pl = vdupq_n_s32(0), ph = pl, xl = pl, xh = pl, ql = pl, qh = pl, yl = pl, yh = pl;
            for (int c = 0; c < 16 && t + 4 <= e; c++, t += 4) {
                int16x8_t A = vld1q_s16(z + 2 * t), B = vld1q_s16(z + 2 * (t - j)), Bs = vrev32q_s16(B);
                int16x8_t C = vld1q_s16(z + 2 * (t - j2)), Cs = vrev32q_s16(C);
                pl = vmlal_s16(pl, vget_low_s16(A), vget_low_s16(B)); ph = vmlal_high_s16(ph, A, B);
                xl = vmlal_s16(xl, vget_low_s16(A), vget_low_s16(Bs)); xh = vmlal_high_s16(xh, A, Bs);
                ql = vmlal_s16(ql, vget_low_s16(A), vget_low_s16(C)); qh = vmlal_high_s16(qh, A, C);
                yl = vmlal_s16(yl, vget_low_s16(A), vget_low_s16(Cs)); yh = vmlal_high_s16(yh, A, Cs);
            }
            P0 = vaddw_s32(vaddw_s32(vaddw_high_s32(vaddw_high_s32(P0, pl), ph), vget_low_s32(pl)), vget_low_s32(ph));
            X0 = vaddw_s32(vaddw_s32(vaddw_high_s32(vaddw_high_s32(X0, xl), xh), vget_low_s32(xl)), vget_low_s32(xh));
            P1 = vaddw_s32(vaddw_s32(vaddw_high_s32(vaddw_high_s32(P1, ql), qh), vget_low_s32(ql)), vget_low_s32(qh));
            X1 = vaddw_s32(vaddw_s32(vaddw_high_s32(vaddw_high_s32(X1, yl), yh), vget_low_s32(yl)), vget_low_s32(yh));
        }
        for (int w = 0; w < 2; w++) {   // tail samples, then store (j2 == j repeats the same lag)
            int jj = w ? j2 : j;
            int64x2_t PP = w ? P1 : P0, XX = w ? X1 : X0;
            int64_t s0 = vgetq_lane_s64(PP, 0), s3 = vgetq_lane_s64(PP, 1), s1 = vgetq_lane_s64(XX, 0), s2 = vgetq_lane_s64(XX, 1);
            for (int64_t u = t; u < e; u++) {
                int64_t I = z[2 * u], Q = z[2 * u + 1], Ij = z[2 * (u - jj)], Qj = z[2 * (u - jj) + 1];
                s0 += I * Ij; s1 += I * Qj; s2 += Q * Ij; s3 += Q * Qj;
            }
            S[jj][0] = s0; S[jj][1] = s1; S[jj][2] = s2; S[jj][3] = s3;
        }
    }
}
// Residuals on the planar int32 copies, 8 samples held in registers across all taps (same integers).
static void block_residuals_neon(const int32_t *I, const int32_t *Q, int64_t s, int64_t e, int K, const int32_t *q, int sh,
                                 int hmin, int hmax, int32_t *rI, int32_t *rQ, int tstep) {
    int32_t ca[36] = {0}, cb[36] = {0}, cc[36] = {0}, cd[36] = {0}, rnd = sh ? 1 << (sh - 1) : 0;
    for (int j = 1; j <= K; j++) { ca[j - 1] = q[2 * j - 2]; cb[j - 1] = q[2 * j - 1]; cc[j - 1] = q[2 * K + 2 * j - 2]; cd[j - 1] = q[2 * K + 2 * j - 1]; }
    const int32x4_t R = vdupq_n_s32(rnd), SH = vdupq_n_s32(-sh), LO = vdupq_n_s32(hmin), HI = vdupq_n_s32(hmax);
    for (int64_t t0 = s; t0 < e; t0 += (int64_t)RTILE * tstep) {
        int64_t t1 = t0 + RTILE < e ? t0 + RTILE : e, t = t0;
        for (; t + 8 <= t1; t += 8) {
            int32x4_t I0 = vld1q_s32(I + t), I1 = vld1q_s32(I + t + 4);
            int32x4_t aI0 = R, aI1 = R, aQ0 = vmlaq_n_s32(R, I0, q[4 * K]), aQ1 = vmlaq_n_s32(R, I1, q[4 * K]);
            for (int j0 = 0; j0 < K; j0 += 4) {
                int32x4_t A = vld1q_s32(ca + j0), B = vld1q_s32(cb + j0), C = vld1q_s32(cc + j0), D = vld1q_s32(cd + j0);
#define TAP(l) if (j0 + l < K) { const int32_t *Ih = I + t - (j0 + l + 1), *Qh = Q + t - (j0 + l + 1); \
                    int32x4_t h0 = vld1q_s32(Ih), h1 = vld1q_s32(Ih + 4), g0 = vld1q_s32(Qh), g1 = vld1q_s32(Qh + 4); \
                    aI0 = vmlaq_laneq_s32(vmlaq_laneq_s32(aI0, h0, A, l), g0, B, l); aI1 = vmlaq_laneq_s32(vmlaq_laneq_s32(aI1, h1, A, l), g1, B, l); \
                    aQ0 = vmlaq_laneq_s32(vmlaq_laneq_s32(aQ0, h0, C, l), g0, D, l); aQ1 = vmlaq_laneq_s32(vmlaq_laneq_s32(aQ1, h1, C, l), g1, D, l); }
                TAP(0) TAP(1) TAP(2) TAP(3)
#undef TAP
            }
            int32x4_t Q0 = vld1q_s32(Q + t), Q1 = vld1q_s32(Q + t + 4);
            vst1q_s32(rI + (t - s), vsubq_s32(I0, vminq_s32(vmaxq_s32(vshlq_s32(aI0, SH), LO), HI)));
            vst1q_s32(rI + (t - s) + 4, vsubq_s32(I1, vminq_s32(vmaxq_s32(vshlq_s32(aI1, SH), LO), HI)));
            vst1q_s32(rQ + (t - s), vsubq_s32(Q0, vminq_s32(vmaxq_s32(vshlq_s32(aQ0, SH), LO), HI)));
            vst1q_s32(rQ + (t - s) + 4, vsubq_s32(Q1, vminq_s32(vmaxq_s32(vshlq_s32(aQ1, SH), LO), HI)));
        }
        for (; t < t1; t++) {
            int32_t aI = rnd, aQ = rnd + q[4 * K] * I[t];
            for (int j = 1; j <= K; j++) { aI += q[2 * j - 2] * I[t - j] + q[2 * j - 1] * Q[t - j]; aQ += q[2 * K + 2 * j - 2] * I[t - j] + q[2 * K + 2 * j - 1] * Q[t - j]; }
            rI[t - s] = I[t] - clampi(aI >> sh, hmin, hmax); rQ[t - s] = Q[t] - clampi(aQ >> sh, hmin, hmax);
        }
    }
}
#endif
// Residuals for a block: SIMD on the interleaved int16 array when available (int16 coefficients are
// guaranteed by quantize()), otherwise the portable version on the planar copies.
// I, Q: planar copies whose index 0 is sample `base` (valid from index -K).
static void residuals(int simd, const int16_t *z, const int32_t *I, const int32_t *Q, int64_t base, int64_t s, int64_t e, int K,
                      const int32_t *q, int sh, int hmin, int hmax, int32_t *rI, int32_t *rQ, int tstep) {
#if IQC_X86
    if (simd) { block_residuals_avx512(z, s, e, K, q, sh, hmin, hmax, rI, rQ, tstep); return; }
#endif
    (void)simd; (void)z;
#if IQC_NEON
    if (!simd_disabled()) { block_residuals_neon(I, Q, s - base, e - base, K, q, sh, hmin, hmax, rI, rQ, tstep); return; }
#endif
    block_residuals(I, Q, s - base, e - base, K, q, sh, hmin, hmax, rI, rQ, tstep);
}
// Least squares for block [s,e) of padded int32 history z: I_t on lags, Q_t on lags + I_t. Normal equations
// from the sums S plus the shift structure C_{i,j} = C_{i-1,j-1} + edge terms.
// c receives 2K coefficients for I then 2K+1 for Q, in natural lag order (I lag1, Q lag1, I lag2, ...).
static void solve_ls(const int16_t *z, int64_t s, int64_t e, int K, int64_t (*S)[4], double *c) {
    int n = 2 * K + 2;
    double *A = calloc((size_t)n * n, sizeof(double));
    int64_t C[4], prev[4];
    for (int d = 0; d <= K; d++) {
        for (int k = 0; k < 4; k++) C[k] = S[d][k];
        for (int i = 0; i + d <= K; i++) {
            int j = i + d;
            if (i > 0) {
                int64_t si[2] = {z[2 * (s - i)], z[2 * (s - i) + 1]}, sj[2] = {z[2 * (s - j)], z[2 * (s - j) + 1]};
                int64_t ei[2] = {z[2 * (e - i)], z[2 * (e - i) + 1]}, ej[2] = {z[2 * (e - j)], z[2 * (e - j) + 1]};
                for (int a = 0; a < 2; a++) for (int b = 0; b < 2; b++) C[a * 2 + b] = prev[a * 2 + b] + si[a] * sj[b] - ei[a] * ej[b];
            }
            memcpy(prev, C, sizeof C);
            int vi = i == 0 ? 2 * K : 2 * (i - 1), vj = j == 0 ? 2 * K : 2 * (j - 1);
            for (int a = 0; a < 2; a++) for (int b = 0; b < 2; b++) { A[(vi + a) * n + vj + b] = (double)C[a * 2 + b]; A[(vj + b) * n + vi + a] = (double)C[a * 2 + b]; }
        }
    }
    double tr = 0; for (int i = 0; i < n; i++) tr += A[i * n + i];
    double ridge = 1.0 + 1e-7 * tr / n;
    for (int i = 0; i < n; i++) A[i * n + i] += ridge;
    memset(c, 0, (4 * K + 1) * sizeof(double));
    for (int k = 0; k < n; k++) {  // right-looking Cholesky, upper triangle U (A = U^T U)
        double *Uk = A + k * n, dg = Uk[k];
        if (dg <= 0) goto out;
        dg = sqrt(dg); Uk[k] = dg;
        double inv = 1.0 / dg;
        for (int i = k + 1; i < n; i++) Uk[i] *= inv;
        for (int j = k + 1; j < n; j++) { double f = Uk[j], *Uj = A + j * n; for (int i = j; i < n; i++) Uj[i] -= f * Uk[i]; }
    }
    for (int which = 0; which < 2; which++) {  // regress var m on vars 0..m-1: U11 beta = U[0..m-1, m]
        int m = 2 * K + which;
        double *beta = which ? c + 2 * K : c;
        for (int i = m - 1; i >= 0; i--) {
            double sum = A[i * n + m];
            for (int k = i + 1; k < m; k++) sum -= A[i * n + k] * beta[k];
            beta[i] = sum / A[i * n + i];
        }
    }
out:
    free(A);
}

// One shift for all coefficients: the int32 FIR sum provably stays below 2^30 for |x| <= xmax, and every
// coefficient fits int16 (lets the decoder use 16-bit dot-product instructions).
static int quantize(const double *c, int m, int prec, int xmax, int32_t *q) {
    double mx = 0, l1 = 0;
    for (int i = 0; i < m; i++) { double a = fabs(c[i]); mx = a > mx ? a : mx; l1 += a; }
    int sh = mx > 0 ? prec - (int)ceil(log2(mx)) : 0;
    sh = sh < 0 ? 0 : sh > 30 ? 30 : sh;
    while (sh > 0 && ((l1 * ldexp(1.0, sh) + m) * xmax >= 1073741824.0 || mx * ldexp(1.0, sh) >= 32767.0)) sh--;
    // Ill-conditioned fits (e.g. a pure tone) can exceed the bound even at shift 0: shrink the predictor.
    double f = (l1 + m) * xmax >= 1073741824.0 ? 0.99 * (1073741824.0 / xmax - m) / l1 : 1.0;
    if (mx * f >= 32767.0) f = 32000.0 / mx;
    for (int i = 0; i < m; i++) q[i] = (int32_t)lround(ldexp(c[i] * f, sh));
    return sh;
}
static void coefs_put(BitW *w, const int32_t *q, int m) {
    uint32_t mx = 0;
    for (int i = 0; i < m; i++) { uint32_t zz = ((uint32_t)q[i] << 1) ^ (uint32_t)(q[i] >> 31); mx = zz > mx ? zz : mx; }
    int wd = nbits(mx); bw_put(w, wd, 6);
    for (int i = 0; i < m; i++) bw_put(w, ((uint32_t)q[i] << 1) ^ (uint32_t)(q[i] >> 31), wd);
}
static void coefs_get(BitR *r, int32_t *q, int m) {
    int wd = br_get(r, 6); wd = wd > 32 ? 32 : wd;
    for (int i = 0; i < m; i++) { uint32_t zz = wd ? br_get(r, wd) : 0; q[i] = (int32_t)(zz >> 1) ^ -(int32_t)(zz & 1); }
}
// Natural lag order -> FIR weights over z[2(t-K) .. 2t); Q gets its current-I weight at index 2K.
static void load_fir(const int32_t *q, int K, int32_t *wI, int32_t *wQ) {
    for (int j = 1; j <= K; j++) {
        wI[2 * (K - j)] = q[2 * (j - 1)]; wI[2 * (K - j) + 1] = q[2 * (j - 1) + 1];
        wQ[2 * (K - j)] = q[2 * K + 2 * (j - 1)]; wQ[2 * (K - j) + 1] = q[2 * K + 2 * (j - 1) + 1];
    }
    wQ[2 * K] = q[4 * K];
}

// Split bits of a superblock (bit0: split in halves, bit1/bit2: split half 0/1 in leaves) -> block bounds in leaves.
static void split_bounds(int sb, int *bounds, int *nblocks) {
    int k = 0; bounds[k++] = 0;
    if (!(sb & 1)) bounds[k++] = 4;
    else { if (sb & 2) bounds[k++] = 1; bounds[k++] = 2; if (sb & 4) bounds[k++] = 3; bounds[k++] = 4; }
    *nblocks = k - 1;
}
// Contexts use magnitude averages lagged by one sample (residuals up to t-2): this keeps the decoder's
// serial dependency chain short and costs nothing in size.
// Context = min(qlog(v), NCTX - 1) with qlog ~4 buckets per octave: 0 for v = 0, else 1 + 4b + (2 bits below the
// leading one), b = floor(log2 v) (part of the stream format). This branch-light form equals it for all 2^32 v.
static inline int ctx_of(uint32_t v) {
    static const uint8_t small[4] = {0, 1, 5, 7};
    int s = 29 - __builtin_clz(v | 4), c = 4 * s + 5 + (int)(v >> s);
    c = v < 4 ? small[v & 3] : c;
    return c > NCTX - 1 ? NCTX - 1 : c;
}
static inline int ctx_I(uint32_t aI, uint32_t aQ) { return ctx_of(2 * aI + aQ); }
static inline int ctx_Q(uint32_t aI, uint32_t aQ) { return ctx_of(aQ + aI / 2); }

static void put_u32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint32_t get_u32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

// fc32 -> int16 the way the encoder does it: x * scale, NaN -> 0, clamp, round half to even (like rint).
static inline int32_t quant(float x, float scale) {
    float v = x * scale;
    v = v == v ? v : 0; v = v > 32767.f ? 32767.f : v < -32768.f ? -32768.f : v;
    return (int32_t)((v + 12582912.0f) - 12582912.0f);   // exact for |v| < 2^22
}
void iqc_quantize(const float *x, int64_t nvals, float scale, float *out) {
    float inv = 1.0f / scale;
    for (int64_t i = 0; i < nvals; i++) out[i] = (float)quant(x[i], scale) * inv;
}
static int bad_params(int fmt, int64_t n, float scale, int shift, int K, int BL) {
    return (fmt != IQC_FC32 && fmt != IQC_SC16) || n < 0 || n > IQC_MAX_N ||
           (fmt == IQC_FC32 && (!(scale > 0) || !(1.0f / scale > 0) || !(scale < 1e30f))) ||   // scale unused for sc16
           shift < 0 || shift > 3 || K < 1 || K > 32 || BL < 1 || BL > (1 << 20);
}

// ---------------- chunk codec ----------------
// Layout: [u32 side][u32 low][u32 raw][u32 rans] + sections.
// side: per-chunk symbol tables, then per superblock: split bits, then per LPC block: shift, I coefs, Q coefs.
int64_t iqc_encode(const void *in, int fmt, int64_t n, float scale, int shift, int K, int BL, int prec,
                   uint8_t *out, int64_t cap, int64_t *inexact) {
    const float *x = in; const int16_t *x16 = in;
    if (inexact) *inexact = 0;
    if (bad_params(fmt, n, scale, shift, K, BL) || prec < 1 || prec > 15) return -1;
    float inv = 1.0f / scale;
    int64_t nbad = 0;
    int M = 1 << shift, lmask = M - 1, hmax = 32767 >> shift, hmin = -32768 >> shift, nc = 4 * K + 1;
    int64_t ret = -1;
    init_syms();
    int16_t *z16 = calloc(2 * (n + K) + 32, sizeof(int16_t)), *zp = z16 + 2 * K;   // high parts, zero-padded history
    int32_t zmax = 0;
    uint8_t *l = malloc(2 * n + 1), *sym = malloc(2 * n + 1), *ctx = malloc(2 * n + 1);
    int nb = (int)((n + BL - 1) / BL);   // leaves (upper bound on LPC blocks)
    // side: tables (< 26 KB) + per block <= 11 + nc * 16 bits + 3 split bits per superblock
    size_t caps[4] = {(size_t)nb * (nc * 16 + 14) / 8 + 65536, (size_t)n + 4096, (size_t)n * 4 + 64, (size_t)n * 5 + 64};
    uint8_t *sec[4]; for (int i = 0; i < 4; i++) sec[i] = malloc(caps[i]);
    int32_t *cq = malloc(sizeof(int32_t) * (size_t)nb * nc);
    int *csh = malloc(sizeof(int) * nb), cand_sh[7];
    uint8_t *sbits = malloc((size_t)nb / 4 + 2);
    int32_t *cand_q = malloc(sizeof(int32_t) * 7 * nc), *tmpI = malloc(sizeof(int32_t) * 8 * (size_t)BL), *tmpQ = tmpI + 4 * (size_t)BL;
    int32_t *plI = malloc(sizeof(int32_t) * (4 * (size_t)BL + K)), *plQ = malloc(sizeof(int32_t) * (4 * (size_t)BL + K));
    int64_t (*LS)[K + 1][4] = malloc(4 * sizeof *LS), (*Ssum)[4] = malloc((K + 1) * sizeof *Ssum);
    double *cf = malloc(sizeof(double) * nc);
    uint32_t (*cnt)[NCTX][NSYM] = calloc(2, sizeof *cnt);
    SymTab (*tab)[NCTX] = calloc(2, sizeof *tab);
    EncSym *es = malloc(sizeof(EncSym) * 2 * NCTX * NSYM);
    uint16_t *ev = NULL; Trk *tk = NULL;
    if (!es || !l || !sym || !ctx || !sec[0] || !sec[1] || !sec[2] || !sec[3] || !cq || !csh || !cf || !cnt || !tab
        || !sbits || !cand_q || !tmpI || !z16 || !plI || !plQ || !LS || !Ssum) goto done;
    if (fmt == IQC_SC16)
        for (int64_t i = 0; i < 2 * n; i++) {
            int32_t q = x16[i], h = q >> shift, a = h < 0 ? -h : h;
            zp[i] = (int16_t)h; l[i] = (uint8_t)(q & lmask); zmax = a > zmax ? a : zmax;
        }
    else
        for (int64_t i = 0; i < 2 * n; i++) {
            int32_t q = quant(x[i], scale), h = q >> shift, a = h < 0 ? -h : h;
            float back = (float)q * inv;   // what the decoder will produce, compared bit for bit
            uint32_t bb, xb; memcpy(&bb, &back, 4); memcpy(&xb, &x[i], 4);
            nbad += bb != xb;
            zp[i] = (int16_t)h; l[i] = (uint8_t)(q & lmask); zmax = a > zmax ? a : zmax;
        }
    int simd = 0, simd_cov = 0;
#if IQC_X86
    simd = have_avx512vnni() && K <= 32;
    simd_cov = simd && zmax <= 8192;
#endif

    // --- low bits. Pass 1: run the trackers on the true symbols, record events.
    Enc le = {0, 0xFFFFFFFFu, 0, 1, sec[1], 0, caps[1]};
    Coder lc = {&le, NULL};
    if (shift) {
        ev = malloc((2 * n + 1) * sizeof(uint16_t));   // bits 0-1 mode, bit 2 exception, bit 3 choice, bits 4+ ambiguity ctx
        tk = calloc(2, sizeof(Trk));
        if (!ev || !tk) goto done;
        trk_init(&tk[0]); trk_init(&tk[1]);
        for (int64_t t = 0; t < n; t++) for (int ch = 0; ch < 2; ch++) {
            Trk *k = &tk[ch];
            if (k->has) {   // fast path (same result as below): a mode-D step that hits and stays strictly inside the strip
                Dss *ds = &k->d;
                int64_t r0 = ds->rl + ds->a, mu = ds->mu, b = ds->b;
                int ok0 = r0 >= mu - 1 && r0 <= mu + b, ok1 = r0 - b >= mu - 1, step = !ok0, uu = k->k + step;
                int64_t r = r0 - (step ? b : 0);
                if (!(ok0 & ok1) && ((l[2 * t + ch] - uu) & lmask) == 0 && r > mu && r < mu + b - 1) {
                    ev[2 * t + ch] = 0;
                    ds->rl = r; ds->last.x++; ds->last.y += step;
                    k->t++; k->Y += uu; k->hist[k->t & (DSL_HIST - 1)] = (int16_t)uu;
                    continue;
                }
            }
            int v0 = 0, ac = 0, mode = trk_predict(k, &v0, &ac), s = l[2 * t + ch], u, e = 0, chs = 0;
            if (mode == MODE_D && ((s - v0) & lmask) == 0) u = v0;
            else if (mode == MODE_A && ((s - v0) & lmask) < 2) { chs = (s - v0) & lmask; u = v0 + chs; }
            else { u = unwrap(k, s, M); e = mode != MODE_N; }
            ev[2 * t + ch] = (uint16_t)(mode | e << 2 | chs << 3 | ac << 4);
            trk_update(k, u);
        }
        // Pass 2: emit in decoding order.
        LowModel lm[2]; low_init(&lm[0]); low_init(&lm[1]);
        int need[2] = {1, 1};
        for (int64_t i = 0; i < 2 * n; i++) {
            if (!(need[0] | need[1]))   // plain mode-D steps (ev == 0) emit nothing: skip them four at a time
                while (i + 4 <= 2 * n && !(ev[i] | ev[i + 1] | ev[i + 2] | ev[i + 3])) i += 4;
            if (i >= 2 * n) break;
            int64_t t = i >> 1;
            int ch = (int)(i & 1);
            uint16_t v = ev[i];
            int mode = v & 3, s = l[i];
            if (mode == MODE_N) { tree_code(&lc, lm[ch].exc[2], shift, s); continue; }
            if (need[ch]) {   // number of regular D/A steps before the next exception (forward scan, O(n) overall)
                uint32_t g = 0;
                int64_t u = t;
                for (; u < n && u < 1; u++) { uint16_t w = ev[2 * u + ch]; if ((w & 3) == MODE_N) continue; if (w & 4) goto gap_done; g++; }
                for (; u + 2 <= n && !((ev[2 * u + ch] | ev[2 * u + 2 + ch]) & 4); u += 2) g += 2;   // no mode N after t = 0
                for (; u < n; u++) { if (ev[2 * u + ch] & 4) break; g++; }
            gap_done:
                gap_code(&lc, lm[ch].gap, g + 1); need[ch] = 0;
            }
            if (v & 4) { tree_code(&lc, lm[ch].exc[mode], shift, s); need[ch] = 1; }
            else if (mode == MODE_A) code_ctr(&lc, &lm[ch].amb[v >> 4], (v >> 3) & 1);
        }
    }
    for (int i = 0; i < 5; i++) shift_low(&le);

    // --- high part. Superblocks of 4 leaves (BL samples each); the encoder picks the split (1x4, 2x2, 2+1+1, 4x1
    // leaves per LPC block) with the smallest estimated cost; covariance sums are computed once per leaf.
    uint32_t aI = 0, aQ = 0, oI = 0, oQ = 0;   // current and one-sample-lagged magnitude averages
    BitW rw = {sec[2], caps[2], 0, 0, 0};
    int nblk = 0;
    for (int64_t S0 = 0; S0 < n; S0 += 4 * (int64_t)BL) {
        int64_t lb[5];
        for (int i = 0; i <= 4; i++) lb[i] = S0 + i * (int64_t)BL < n ? S0 + i * (int64_t)BL : n;
        int32_t *Ip = plI + K, *Qp = plQ + K;   // planar copies: index t - S0, valid for t in [S0 - K, lb[4])
        if (!simd_cov)
            for (int64_t t = S0 - K; t < lb[4]; t++) { Ip[t - S0] = zp[2 * t]; Qp[t - S0] = zp[2 * t + 1]; }
        for (int i = 0; i < 4; i++) {
#if IQC_X86
            if (simd_cov) { cov_sums_avx512(zp, lb[i], lb[i + 1], K, LS[i]); continue; }
#endif
#if IQC_NEON
            if (zmax <= 8192 && !simd_disabled()) { cov_sums_neon(zp, lb[i], lb[i + 1], K, LS[i]); continue; }
#endif
            cov_sums(Ip, Qp, lb[i] - S0, lb[i + 1] - S0, K, LS[i]);
        }
        // candidates: 0 = whole, 1-2 = halves, 3-6 = leaves
        static const int cl[7] = {0, 0, 2, 0, 1, 2, 3}, cr[7] = {4, 2, 4, 1, 2, 3, 4};
        double cost[7];
        for (int c = 0; c < 7; c++) {
            int64_t s = lb[cl[c]], e = lb[cr[c]];
            if (e <= s) { cost[c] = 0; cand_sh[c] = 0; continue; }
            for (int j = 0; j <= K; j++) for (int k = 0; k < 4; k++) {
                int64_t v = 0; for (int i = cl[c]; i < cr[c]; i++) v += LS[i][j][k];
                Ssum[j][k] = v;
            }
            solve_ls(zp, s, e, K, Ssum, cf);
            int32_t *q = cand_q + (size_t)c * nc;
            int sh = cand_sh[c] = quantize(cf, nc, prec, hmax + 1, q);
            residuals(simd, zp, Ip, Qp, S0, s, e, K, q, sh, hmin, hmax, tmpI, tmpQ, 8);
            double sI = 0, sQ = 0, cnt_t = 0;
            for (int64_t t0 = 0; t0 < e - s; t0 += 8 * RTILE)
                for (int64_t t = t0; t < t0 + RTILE && t < e - s; t++) { sI += tmpI[t] < 0 ? -tmpI[t] : tmpI[t]; sQ += tmpQ[t] < 0 ? -tmpQ[t] : tmpQ[t]; cnt_t++; }
            uint32_t mx = 0;
            for (int i = 0; i < nc; i++) { uint32_t zz = ((uint32_t)q[i] << 1) ^ (uint32_t)(q[i] >> 31); mx = zz > mx ? zz : mx; }
            double len = (double)(e - s);   // Laplacian code length estimate + coefficient bits
            cost[c] = len * (log2(sI / cnt_t + 0.5) + log2(sQ / cnt_t + 0.5)) + nc * nbits(mx) + 11;
        }
        // decide: per half (leaves vs half block), then whole vs best halves
        int splitH[2], pick[4], np = 0;
        double ch[2];
        for (int h = 0; h < 2; h++) { splitH[h] = cost[3 + 2 * h] + cost[4 + 2 * h] + 1 < cost[1 + h]; ch[h] = splitH[h] ? cost[3 + 2 * h] + cost[4 + 2 * h] + 1 : cost[1 + h]; }
        int splitT = ch[0] + ch[1] + 1 < cost[0];
        if (!splitT) pick[np++] = 0;
        else for (int h = 0; h < 2; h++) { if (splitH[h]) { pick[np++] = 3 + 2 * h; pick[np++] = 4 + 2 * h; } else pick[np++] = 1 + h; }
        sbits[S0 / (4 * (int64_t)BL)] = (uint8_t)(splitT | splitH[0] << 1 | splitH[1] << 2);
        for (int p = 0; p < np; p++) {
            int c = pick[p];
            int64_t s = lb[cl[c]], e = lb[cr[c]];
            if (e <= s) continue;
            memcpy(cq + (size_t)nblk * nc, cand_q + (size_t)c * nc, nc * sizeof(int32_t)); csh[nblk++] = cand_sh[c];
            int32_t *crI = tmpI, *crQ = tmpQ;
            residuals(simd, zp, Ip, Qp, S0, s, e, K, cand_q + (size_t)c * nc, cand_sh[c], hmin, hmax, crI, crQ, 1);
            for (int64_t t = s; t < e; t++) {   // symbols + contexts; remaining mantissa bits and signs go to the raw stream
                int rI = crI[t - s], rQ = crQ[t - s], n0, n1;
                uint32_t w0, w1, mI = rI < 0 ? -rI : rI, mQ = rQ < 0 ? -rQ : rQ;
                int c0 = ctx_I(oI, oQ), c1 = ctx_Q(oI, oQ), s0 = res_sym(rI, &w0, &n0), s1 = res_sym(rQ, &w1, &n1);
                ctx[2 * t] = (uint8_t)c0; sym[2 * t] = (uint8_t)s0; ctx[2 * t + 1] = (uint8_t)c1; sym[2 * t + 1] = (uint8_t)s1;
                bw_put(&rw, w0, n0); bw_put(&rw, w1, n1);
                cnt[0][c0][s0]++; cnt[1][c1][s1]++;
                oI = aI; oQ = aQ;
                aI = aI - (aI >> 3) + (mI << 1); aQ = aQ - (aQ >> 3) + (mQ << 1);
            }
        }
    }

    // --- side info: tables + coefficients
    BitW side = {sec[0], caps[0], 0, 0, 0};
    for (int ch = 0; ch < 2; ch++) for (int c = 0; c < NCTX; c++) {
        int used = normalize(cnt[ch][c], &tab[ch][c]);
        bw_put(&side, used, 1);
        if (used) for (int sy = 0; sy < NSYM; sy++) gamma_put(&side, tab[ch][c].freq[sy] + 1u);
    }
    for (int64_t S0 = 0, bi = 0; S0 < n; S0 += 4 * (int64_t)BL) {   // per superblock: split bits, then its blocks
        int sb = sbits[S0 / (4 * (int64_t)BL)], nleaf = 0;
        bw_put(&side, sb & 1, 1);
        if (sb & 1) { bw_put(&side, (sb >> 1) & 1, 1); bw_put(&side, (sb >> 2) & 1, 1); }
        int64_t lb[5];
        for (int i = 0; i <= 4; i++) lb[i] = S0 + i * (int64_t)BL < n ? S0 + i * (int64_t)BL : n;
        int bounds[5]; split_bounds(sb, bounds, &nleaf);
        for (int i = 0; i < nleaf; i++) if (lb[bounds[i + 1]] > lb[bounds[i]]) { bw_put(&side, csh[bi], 5); coefs_put(&side, cq + (size_t)bi * nc, nc); bi++; }
    }
    bw_flush(&side);

    // --- raw bits (written above) and rANS (reverse, two interleaved states sharing one word stream)
    bw_flush(&rw);
    uint8_t *rend = sec[3] + caps[3], *rp = rend;
    uint32_t xs[2] = {RANS_L, RANS_L};
#define PUTW(w) do { rp -= 2; rp[0] = (uint8_t)(w); rp[1] = (uint8_t)((w) >> 8); } while (0)
    for (int ch = 0; ch < 2; ch++) for (int c = 0; c < NCTX; c++) for (int sy = 0; sy < NSYM; sy++)
        if (tab[ch][c].freq[sy]) es[(ch * NCTX + c) * NSYM + sy] = enc_sym(tab[ch][c].freq[sy], tab[ch][c].start[sy]);
    for (int64_t i = 2 * n - 1; i >= 0; i--) {
        const EncSym *e = &es[((i & 1) * NCTX + ctx[i]) * NSYM + sym[i]];
        uint32_t xv = xs[i & 1];
        int f = xv >= e->x_max;   // branchless renormalisation: always store the word, keep it only when needed
        rp[-2] = (uint8_t)xv; rp[-1] = (uint8_t)(xv >> 8);
        rp -= 2 * f; xv >>= 16 * f;
#ifdef __SIZEOF_INT128__
        uint32_t qt = e->rcp ? (uint32_t)(((unsigned __int128)xv * e->rcp) >> 64) : xv;
#else
        uint32_t qt = xv / e->freq;
#endif
        xs[i & 1] = (qt << PROB_BITS) + (xv - qt * e->freq) + e->start;
    }
    PUTW(xs[1] >> 16); PUTW(xs[1] & 0xFFFF); PUTW(xs[0] >> 16); PUTW(xs[0] & 0xFFFF);
#undef PUTW

    size_t lens[4] = {side.pos, le.pos, rw.pos, (size_t)(rend - rp)};
    if (lens[0] > caps[0] || lens[1] > caps[1] || lens[2] > caps[2]) goto done;
    ret = 16 + (int64_t)(lens[0] + lens[1] + lens[2] + lens[3]);
    if (ret <= cap) {
        uint8_t *o = out + 16;
        for (int i = 0; i < 4; i++) put_u32(out + 4 * i, (uint32_t)lens[i]);
        for (int i = 0; i < 3; i++) { memcpy(o, sec[i], lens[i]); o += lens[i]; }
        memcpy(o, rp, lens[3]);
    }
done:
    if (inexact) *inexact = nbad;
    free(l); free(sym); free(ctx); for (int i = 0; i < 4; i++) free(sec[i]);
    free(cq); free(csh); free(cf); free(cnt); free(tab); free(ev); free(tk);
    free(sbits); free(cand_q); free(es); free(tmpI); free(z16); free(plI); free(plQ); free(LS); free(Ssum);
    return ret;
}

// ---------------- decoder ----------------
typedef struct {
    Dec ld; Coder lc; BitR rr; const uint8_t *rp, *rend;
    SymTab (*tab)[NCTX]; uint8_t (*lut)[NCTX][PROB_SCALE];
    uint32_t xs[2], aI, aQ, oI, oQ;
    Trk tk[2]; LowModel lm[2]; int64_t gap[2];
    int shift, M, lmask, hmin, hmax;
    float inv;
    int f32;   // output fc32 (else sc16)
    int err;   // corrupt stream detected
} DecSt;

#define ALWAYS_INLINE static inline __attribute__((always_inline))
ALWAYS_INLINE void dec_low(DecSt *d, int lo[2]) {
    for (int ch = 0; ch < 2; ch++) {
        Trk *k = &d->tk[ch];
        if (k->has && d->gap[ch] > 0) {   // fast path (same result as below): a regular mode-D step strictly inside the strip
            Dss *ds = &k->d;
            int64_t r0 = ds->rl + ds->a, mu = ds->mu, b = ds->b;
            int ok0 = r0 >= mu - 1 && r0 <= mu + b, ok1 = r0 - b >= mu - 1, step = !ok0;
            int64_t r = r0 - (step ? b : 0);
            if (!(ok0 & ok1) && r > mu && r < mu + b - 1) {
                int u = k->k + step;
                d->gap[ch]--;
                ds->rl = r; ds->last.x++; ds->last.y += step;
                k->t++; k->Y += u; k->hist[k->t & (DSL_HIST - 1)] = (int16_t)u;
                lo[ch] = u & d->lmask;
                continue;
            }
        }
        int v0 = 0, ac = 0, mode = trk_predict(k, &v0, &ac), s, u;
        if (mode == MODE_N) { s = tree_code(&d->lc, d->lm[ch].exc[2], d->shift, 0); u = unwrap(k, s, d->M); }
        else {
            if (d->gap[ch] < 0) d->gap[ch] = (int64_t)gap_code(&d->lc, d->lm[ch].gap, 0) - 1;
            if (d->gap[ch] == 0) { s = tree_code(&d->lc, d->lm[ch].exc[mode], d->shift, 0); u = unwrap(k, s, d->M); d->gap[ch] = -1; }
            else {
                d->gap[ch]--;
                u = v0 + (mode == MODE_A ? code_ctr(&d->lc, &d->lm[ch].amb[ac], 0) : 0);
                s = u & d->lmask;
            }
        }
        lo[ch] = s;
        trk_update(k, u);
    }
}
ALWAYS_INLINE int dec_sym(DecSt *d, int ch, int c) {
    uint32_t xv = d->xs[ch], sl = xv & (PROB_SCALE - 1);
    int sy = d->lut[ch][c][sl];
    xv = d->tab[ch][c].freq[sy] * (xv >> PROB_BITS) + sl - d->tab[ch][c].start[sy];
    if (xv < RANS_L && d->rp + 2 <= d->rend) { xv = (xv << 16) | d->rp[0] | d->rp[1] << 8; d->rp += 2; }
    d->xs[ch] = xv;
    return res_from(sy, &d->rr);
}
ALWAYS_INLINE void dec_res(DecSt *d, int *rI, int *rQ) {   // residuals never depend on the prediction
    int c0 = ctx_I(d->oI, d->oQ), c1 = ctx_Q(d->oI, d->oQ);
    *rI = dec_sym(d, 0, c0);
    *rQ = dec_sym(d, 1, c1);
    uint32_t mI = *rI < 0 ? -*rI : *rI, mQ = *rQ < 0 ? -*rQ : *rQ;
    d->oI = d->aI; d->oQ = d->aQ;
    d->aI = d->aI - (d->aI >> 3) + (mI << 1); d->aQ = d->aQ - (d->aQ >> 3) + (mQ << 1);
}

// A valid stream reproduces values inside [hmin, hmax]; anything else is corruption. Stopping there keeps
// every later FIR sum within int32 and the SIMD and portable paths in agreement.
ALWAYS_INLINE int out_of_range(DecSt *d, int v) {
    if ((unsigned)(v - d->hmin) <= (unsigned)(d->hmax - d->hmin)) return 0;
    d->err = 1;
    return 1;
}
// Direct-form FIR over the interleaved history (any platform).
ALWAYS_INLINE void dec_out(const DecSt *d, void *out, int64_t i, int v) {
    if (d->f32) ((float *)out)[i] = (float)v * d->inv; else ((int16_t *)out)[i] = (int16_t)v;
}
static void dec_block_generic(DecSt *d, int32_t *zp, int64_t b0, int64_t b1, const int32_t *q, int K, int sh, void *out) {
    int32_t wI[64], wQ[65], rnd = sh ? 1 << (sh - 1) : 0;
    load_fir(q, K, wI, wQ);
    for (int64_t t = b0; t < b1; t++) {
        int lo[2] = {0, 0}, rI, rQ;
        if (d->shift) dec_low(d, lo);
        dec_res(d, &rI, &rQ);
        const int32_t *h = zp + 2 * (t - K);
        int vI = clampi((fir(wI, h, 2 * K) + rnd) >> sh, d->hmin, d->hmax) + rI;
        if (out_of_range(d, vI)) return;
        zp[2 * t] = vI;
        int vQ = clampi((fir(wQ, h, 2 * K) + wQ[2 * K] * vI + rnd) >> sh, d->hmin, d->hmax) + rQ;
        if (out_of_range(d, vQ)) return;
        zp[2 * t + 1] = vQ;
        dec_out(d, out, 2 * t, vI * d->M + lo[0]);
        dec_out(d, out, 2 * t + 1, vQ * d->M + lo[1]);
    }
}

#if IQC_NEON
// Transposed FIR in NEON registers: lane j of P*[r] is the partial prediction for time t + 4r + j (rounding constant
// included). Each new (I, Q) pair is added with SMLAL by lane (int16 coefficients, guaranteed by the block check);
// lane 0 of the next prediction is updated on the scalar side so the vector work stays off the dependency chain.
// Integer results are identical to the direct form (sums bounded, see quantize()).
ALWAYS_INLINE void dec_block_neon_t(DecSt *d, int32_t *zp, int64_t b0, int64_t b1, const int32_t *q, int K, int sh, void *out, const int NR) {
    int16_t ca[32] = {0}, cb[32] = {0}, da[32] = {0}, db[32] = {0};
    int32_t pi[32], pq[32], rnd = sh ? 1 << (sh - 1) : 0, c0 = q[4 * K];
    for (int j = 1; j <= K; j++) { ca[j - 1] = q[2 * j - 2]; cb[j - 1] = q[2 * j - 1]; da[j - 1] = q[2 * K + 2 * j - 2]; db[j - 1] = q[2 * K + 2 * j - 1]; }
    for (int j = 0; j < 4 * NR; j++) pi[j] = pq[j] = rnd;
    for (int j = 0; j < K; j++)   // contributions of samples before b0 to predictions b0 + j
        for (int i = j + 1; i <= K; i++) {
            int64_t s = b0 + j - i;
            pi[j] += q[2 * i - 2] * zp[2 * s] + q[2 * i - 1] * zp[2 * s + 1];
            pq[j] += q[2 * K + 2 * i - 2] * zp[2 * s] + q[2 * K + 2 * i - 1] * zp[2 * s + 1];
        }
    const int32x4_t R = vdupq_n_s32(rnd);   // enters at the top lane
    int16x8_t CA[4], CB[4], DA[4], DB[4];
    int32x4_t PI[8], PQ[8];
    for (int r = 0; r < (NR + 1) / 2; r++) { CA[r] = vld1q_s16(ca + 8 * r); CB[r] = vld1q_s16(cb + 8 * r); DA[r] = vld1q_s16(da + 8 * r); DB[r] = vld1q_s16(db + 8 * r); }
    for (int r = 0; r < NR; r++) { PI[r] = vld1q_s32(pi + 4 * r); PQ[r] = vld1q_s32(pq + 4 * r); }
    int32_t a1 = ca[0], g1 = cb[0], e1 = da[0], f1 = db[0], hmin = d->hmin, hmax = d->hmax;
    int32_t pI = vgetq_lane_s32(PI[0], 0), pQ = vgetq_lane_s32(PQ[0], 0);
    for (int64_t t = b0; t < b1; t++) {
        int lo[2] = {0, 0}, rI, rQ;
        if (d->shift) dec_low(d, lo);
        dec_res(d, &rI, &rQ);
        int32_t xI = vgetq_lane_s32(PI[0], 1), xQ = vgetq_lane_s32(PQ[0], 1);
        int vI = clampi(pI >> sh, hmin, hmax) + rI;
        if (out_of_range(d, vI)) return;
        int vQ = clampi((pQ + c0 * vI) >> sh, hmin, hmax) + rQ;
        if (out_of_range(d, vQ)) return;
        pI = xI + a1 * vI + g1 * vQ; pQ = xQ + e1 * vI + f1 * vQ;
        zp[2 * t] = vI; zp[2 * t + 1] = vQ;
        dec_out(d, out, 2 * t, vI * d->M + lo[0]);
        dec_out(d, out, 2 * t + 1, vQ * d->M + lo[1]);
        int16x4_t V = vset_lane_s16((int16_t)vQ, vdup_n_s16((int16_t)vI), 1);
        for (int r = 0; r < NR; r++) {
            int32x4_t nI = vextq_s32(PI[r], r + 1 < NR ? PI[r + 1] : R, 1), nQ = vextq_s32(PQ[r], r + 1 < NR ? PQ[r + 1] : R, 1);
            if (r & 1) {
                nI = vmlal_high_lane_s16(nI, CA[r / 2], V, 0); nI = vmlal_high_lane_s16(nI, CB[r / 2], V, 1);
                nQ = vmlal_high_lane_s16(nQ, DA[r / 2], V, 0); nQ = vmlal_high_lane_s16(nQ, DB[r / 2], V, 1);
            } else {
                nI = vmlal_lane_s16(nI, vget_low_s16(CA[r / 2]), V, 0); nI = vmlal_lane_s16(nI, vget_low_s16(CB[r / 2]), V, 1);
                nQ = vmlal_lane_s16(nQ, vget_low_s16(DA[r / 2]), V, 0); nQ = vmlal_lane_s16(nQ, vget_low_s16(DB[r / 2]), V, 1);
            }
            PI[r] = nI; PQ[r] = nQ;
        }
    }
}
// One instance per register count (K <= 4 NR) so the accumulators stay in registers.
static void dec_block_neon(DecSt *d, int32_t *zp, int64_t b0, int64_t b1, const int32_t *q, int K, int sh, void *out) {
    switch ((K + 3) / 4) {
    case 1: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 1); break;
    case 2: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 2); break;
    case 3: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 3); break;
    case 4: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 4); break;
    case 5: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 5); break;
    case 6: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 6); break;
    case 7: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 7); break;
    default: dec_block_neon_t(d, zp, b0, b1, q, K, sh, out, 8); break;
    }
}
#endif
#if IQC_X86
// Transposed FIR in two zmm registers per predictor: lane j holds the partial prediction for time t+j.
// Each new (I, Q) pair is added to every lane with one vpdpwssd (int16 pairs x int16 coefficient pairs),
// then the lanes shift by one. No load of just-stored samples sits on the dependency chain.
// Integer results are identical to the direct form (no overflow, see quantize()).
__attribute__((target("avx512f,avx512bw,avx512vnni")))
static void dec_block_avx512(DecSt *d, int32_t *zp, int64_t b0, int64_t b1, const int32_t *q, int K, int sh, void *out) {
    int32_t ci[32] = {0}, cq[32] = {0}, pi[32] = {0}, pq[32] = {0}, rnd = sh ? 1 << (sh - 1) : 0, c0 = q[4 * K];
    for (int j = 1; j <= K; j++) {   // lane j-1: coefficient pair for lag j
        ci[j - 1] = (int32_t)((uint32_t)(q[2 * j - 2] & 0xFFFF) | (uint32_t)q[2 * j - 1] << 16);
        cq[j - 1] = (int32_t)((uint32_t)(q[2 * K + 2 * j - 2] & 0xFFFF) | (uint32_t)q[2 * K + 2 * j - 1] << 16);
    }
    for (int j = 0; j < K; j++)      // contributions of samples before b0 to predictions b0 + j
        for (int i = j + 1; i <= K; i++) {
            int64_t s = b0 + j - i;
            pi[j] += q[2 * i - 2] * zp[2 * s] + q[2 * i - 1] * zp[2 * s + 1];
            pq[j] += q[2 * K + 2 * i - 2] * zp[2 * s] + q[2 * K + 2 * i - 1] * zp[2 * s + 1];
        }
    __m512i CI0 = _mm512_loadu_si512(ci), CI1 = _mm512_loadu_si512(ci + 16), CQ0 = _mm512_loadu_si512(cq), CQ1 = _mm512_loadu_si512(cq + 16);
    __m512i PI0 = _mm512_loadu_si512(pi), PI1 = _mm512_loadu_si512(pi + 16), PQ0 = _mm512_loadu_si512(pq), PQ1 = _mm512_loadu_si512(pq + 16);
    const __m512i Z = _mm512_setzero_si512();
    for (int64_t t = b0; t < b1; t++) {
        int lo[2] = {0, 0}, rI, rQ;
        if (d->shift) dec_low(d, lo);
        dec_res(d, &rI, &rQ);
        int vI = clampi((_mm_cvtsi128_si32(_mm512_castsi512_si128(PI0)) + rnd) >> sh, d->hmin, d->hmax) + rI;
        if (out_of_range(d, vI)) return;
        int vQ = clampi((_mm_cvtsi128_si32(_mm512_castsi512_si128(PQ0)) + c0 * vI + rnd) >> sh, d->hmin, d->hmax) + rQ;
        if (out_of_range(d, vQ)) return;
        zp[2 * t] = vI; zp[2 * t + 1] = vQ;
        dec_out(d, out, 2 * t, vI * d->M + lo[0]);
        dec_out(d, out, 2 * t + 1, vQ * d->M + lo[1]);
        __m512i pair = _mm512_set1_epi32((int32_t)((uint32_t)(vI & 0xFFFF) | (uint32_t)vQ << 16));
        PI0 = _mm512_dpwssd_epi32(_mm512_alignr_epi32(PI1, PI0, 1), pair, CI0);
        PI1 = _mm512_dpwssd_epi32(_mm512_alignr_epi32(Z, PI1, 1), pair, CI1);
        PQ0 = _mm512_dpwssd_epi32(_mm512_alignr_epi32(PQ1, PQ0, 1), pair, CQ0);
        PQ1 = _mm512_dpwssd_epi32(_mm512_alignr_epi32(Z, PQ1, 1), pair, CQ1);
    }
}
#endif

int iqc_decode(const uint8_t *in, int64_t len, void *out, int fmt, int64_t n, float scale, int shift, int K, int BL) {
    if (len < 16 || bad_params(fmt, n, scale, shift, K, BL)) return -1;
    size_t lens[4], off = 16;
    for (int i = 0; i < 4; i++) lens[i] = get_u32(in + 4 * i);
    if (16 + lens[0] + lens[1] + lens[2] + lens[3] > (size_t)len || lens[3] < 8) return -1;
    DecSt *d = calloc(1, sizeof *d);
    int nc = 4 * K + 1, ret = -1;
    int32_t *z = calloc(2 * (n + K) + 2, sizeof(int32_t)), *zp = z + 2 * K, q[129];
    if (!d || !z) goto done;
    BitR side = {in + off, lens[0], 0, 0, 0}; off += lens[0];
    d->ld = (Dec){0xFFFFFFFFu, 0, in + off, 0, lens[1]}; off += lens[1];
    for (int i = 0; i < 5; i++) d->ld.code = (d->ld.code << 8) | in_byte(&d->ld);
    d->lc = (Coder){NULL, &d->ld};
    d->rr = (BitR){in + off, lens[2], 0, 0, 0}; off += lens[2];
    d->rp = in + off; d->rend = d->rp + lens[3];
    d->shift = shift; d->M = 1 << shift; d->lmask = d->M - 1; d->hmax = 32767 >> shift; d->hmin = -32768 >> shift; d->inv = 1.0f / scale; d->f32 = fmt == IQC_FC32;
    d->tab = calloc(2, sizeof *d->tab);
    d->lut = calloc(2, sizeof *d->lut);
    if (!d->tab || !d->lut) goto done;
    for (int ch = 0; ch < 2; ch++) for (int c = 0; c < NCTX; c++) {
        if (!br_get(&side, 1)) continue;
        SymTab *t = &d->tab[ch][c];
        int st = 0;
        for (int sy = 0; sy < NSYM; sy++) {
            uint32_t f = gamma_get(&side) - 1;
            if (f > (uint32_t)(PROB_SCALE - st)) goto done;   // (st + f could wrap around in 32 bits)
            t->freq[sy] = (uint16_t)f; t->start[sy] = (uint16_t)st;
            memset(d->lut[ch][c] + st, sy, f); st += f;
        }
        if (st != PROB_SCALE || t->freq[60] | t->freq[61] | t->freq[62] | t->freq[63]) goto done;   // FORMAT.md 4.5
    }
    init_syms();
    const uint8_t *rp = d->rp;
    d->xs[0] = rp[0] | rp[1] << 8 | (uint32_t)rp[2] << 16 | (uint32_t)rp[3] << 24;
    d->xs[1] = rp[4] | rp[5] << 8 | (uint32_t)rp[6] << 16 | (uint32_t)rp[7] << 24;
    d->rp += 8;
    trk_init(&d->tk[0]); trk_init(&d->tk[1]); low_init(&d->lm[0]); low_init(&d->lm[1]);
    d->gap[0] = d->gap[1] = -1;
    for (int64_t S0 = 0; S0 < n; S0 += 4 * (int64_t)BL) {   // superblock: split structure, then its LPC blocks
        int sb = br_get(&side, 1), bounds[5], nbk;
        if (sb) { sb |= br_get(&side, 1) << 1; sb |= br_get(&side, 1) << 2; }
        split_bounds(sb, bounds, &nbk);
        for (int i = 0; i < nbk; i++) {
            int64_t b0 = S0 + bounds[i] * (int64_t)BL, b1 = S0 + bounds[i + 1] * (int64_t)BL;
            b0 = b0 < n ? b0 : n; b1 = b1 < n ? b1 : n;
            if (b1 <= b0) continue;
            int sh = br_get(&side, 5);
            coefs_get(&side, q, nc);
            // what quantize() guarantees: int16 coefficients and |FIR sum| < 2^30 for in-range history
            int64_t l1 = 0;
            for (int j = 0; j < nc; j++) { if (q[j] < -32767 || q[j] > 32767) goto done; l1 += q[j] < 0 ? -q[j] : q[j]; }
            if (sh > 30 || l1 * (d->hmax + 1) >= (1ll << 30)) goto done;
#if IQC_X86
            if (have_avx512vnni()) dec_block_avx512(d, zp, b0, b1, q, K, sh, out);
            else
#endif
#if IQC_NEON
            if (!simd_disabled()) dec_block_neon(d, zp, b0, b1, q, K, sh, out);
            else
#endif
            dec_block_generic(d, zp, b0, b1, q, K, sh, out);
            if (d->err) goto done;
        }
    }
    ret = 0;
done:
    if (d) { free(d->tab); free(d->lut); }
    free(d); free(z);
    return ret;
}
