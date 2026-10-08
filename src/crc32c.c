// CRC-32C (Castagnoli): SSE4.2 / ARMv8 CRC instructions when available, slice-by-8 table otherwise.
#include <pthread.h>
#include <string.h>
#include "crc32c.h"

static uint32_t tab[8][256];
static void init_tab(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = c & 1 ? (c >> 1) ^ 0x82F63B78u : c >> 1;
        tab[0][i] = c;
    }
    for (int s = 1; s < 8; s++)
        for (int i = 0; i < 256; i++) tab[s][i] = (tab[s - 1][i] >> 8) ^ tab[0][tab[s - 1][i] & 0xFF];
}
static uint32_t crc_sw(uint32_t c, const uint8_t *p, size_t n) {
    static pthread_once_t once = PTHREAD_ONCE_INIT;
    pthread_once(&once, init_tab);
    for (; n >= 8; n -= 8, p += 8) {
        uint32_t lo = c ^ (p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24);
        c = tab[7][lo & 0xFF] ^ tab[6][(lo >> 8) & 0xFF] ^ tab[5][(lo >> 16) & 0xFF] ^ tab[4][lo >> 24] ^
            tab[3][p[4]] ^ tab[2][p[5]] ^ tab[1][p[6]] ^ tab[0][p[7]];
    }
    while (n--) c = (c >> 8) ^ tab[0][(c ^ *p++) & 0xFF];
    return c;
}

#if defined(__x86_64__) && (defined(__GNUC__) || defined(__clang__))
#include <nmmintrin.h>
__attribute__((target("sse4.2"))) static uint32_t crc_hw(uint32_t c, const uint8_t *p, size_t n) {
    uint64_t c64 = c;
    for (; n >= 8; n -= 8, p += 8) { uint64_t v; memcpy(&v, p, 8); c64 = _mm_crc32_u64(c64, v); }
    c = (uint32_t)c64;
    while (n--) c = _mm_crc32_u8(c, *p++);
    return c;
}
static int hw_ok;
static void detect(void) { __builtin_cpu_init(); hw_ok = __builtin_cpu_supports("sse4.2"); }
static int have_hw(void) { static pthread_once_t once = PTHREAD_ONCE_INIT; pthread_once(&once, detect); return hw_ok; }
#elif defined(__aarch64__) && defined(__ARM_FEATURE_CRC32) && defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#include <arm_acle.h>
static uint32_t crc_hw(uint32_t c, const uint8_t *p, size_t n) {
    for (; n >= 8; n -= 8, p += 8) { uint64_t v; memcpy(&v, p, 8); c = __crc32cd(c, v); }
    while (n--) c = __crc32cb(c, *p++);
    return c;
}
static int have_hw(void) { return 1; }
#else
static uint32_t crc_hw(uint32_t c, const uint8_t *p, size_t n) { return crc_sw(c, p, n); }
static int have_hw(void) { return 0; }
#endif

uint32_t crc32c(uint32_t crc, const void *buf, size_t n) {
    crc = ~crc;
    crc = have_hw() ? crc_hw(crc, buf, n) : crc_sw(crc, buf, n);
    return ~crc;
}
