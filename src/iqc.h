// iqcodec chunk codec. Samples are interleaved I/Q; n counts complex samples.
#ifndef IQC_H
#define IQC_H
#include <stdint.h>

enum { IQC_FC32 = 0, IQC_SC16 = 1 };   // complex float32 (int16 / scale) | complex int16

// Encodes one chunk. shift: low bits handled by the sigma-delta tracker (0..3); K: LPC order (1..32);
// BL: leaf block length; prec: coefficient precision. For fc32, *inexact receives the number of values that
// do not round-trip exactly (they are quantized to int16 / scale). Returns the coded size (> cap: it did
// not fit and out is untouched), or -1 on error.
int64_t iqc_encode(const void *in, int fmt, int64_t n, float scale, int shift, int K, int BL, int prec,
                   uint8_t *out, int64_t cap, int64_t *inexact);

// Decodes one chunk into out (n complex samples of fmt). Returns 0 on success.
int iqc_decode(const uint8_t *in, int64_t len, void *out, int fmt, int64_t n, float scale, int shift, int K, int BL);

#endif
