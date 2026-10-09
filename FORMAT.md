# iqcodec stream format, version 2

This document specifies the `.iqc` stream format written by iqcodec 0.2.0 and later. It is complete enough to write
a decoder without the iqcodec sources. A decoder that follows it reproduces the original samples bit for bit.
`tests/fixtures` holds reference streams with the SHA-256 of their decoded samples; use them to check a new decoder.

Compatibility promise: every version-2 stream that iqcodec writes is readable by every later iqcodec release, and
from 0.3.1 on, iqcodec reads every stream that is valid under this document (0.2.x and 0.3.0 read only K = 24 and
leaf = 8192, the values iqcodec writes). A change that a version-2 reader could not decode gets a new version number
in the stream header, and the readers keep reading version 2. (Version 1, written only by iqcodec 0.1.0, is not
supported.)

The rules of sections 2 to 4 are normative: a reader rejects a stream that breaks them ("must"). Where a stream is
corrupt in a way these rules do not catch, the chunk CRC does.

Conventions:

- Integers in the container are unsigned little-endian (`u8`, `u32`, `u64`); `f32` is a little-endian IEEE 754
  binary32.
- In pseudocode, `>>` on a signed value is an arithmetic shift (rounds towards minus infinity), `/` on integers
  truncates towards zero, and `&` on a negative value acts on its two's complement (as in C and Python).
- Arithmetic is exact (no overflow) unless a width is stated. Every quantity fits in 64-bit signed integers, and the
  bounds in 4.6 keep every prediction sum within 32-bit signed integers.
- `nbits(v)` for `v >= 0` is the number of bits needed to write v: 0 for 0, else floor(log2 v) + 1.
- `a .. b` is the inclusive range a, a+1, …, b (empty if b < a).
- Reads from a stream happen in the order they are written, left to right, also within one expression.

## 1. Samples

A stream holds N complex samples. Each sample is an I value and a Q value, both signed 16-bit integers (the
*sample values*). The output of a decoder is, per sample, I then Q, in one of two formats:

| fmt | name | output per value |
|---|---|---|
| 0 | fc32 | `f32(v) * f32(1 / scale)`, computed in binary32 with round-to-nearest-even (see 2.1) |
| 1 | sc16 | `v` as a little-endian signed 16-bit integer |

## 2. Container

```
stream  = header chunk* end
header  = "IQCD" u8 version u8 fmt u8 K u8 prec u32 leaf f32 scale        (16 bytes)
chunk   = u32 n  u8 shift  u8 0 u8 0 u8 0  u32 nbytes  u32 crc  payload  (16 bytes + nbytes)
end     = u32 0  u64 total                                                 (12 bytes)
```

### 2.1 Header

| field | value |
|---|---|
| magic | the 4 bytes `I Q C D` |
| version | 2 |
| fmt | 0 = fc32, 1 = sc16 (output format, section 1) |
| K | prediction order, 1 to 32 (iqcodec writes 24) |
| prec | coefficient precision the encoder aimed at (iqcodec writes 11). Informational: any value, ignored by readers. |
| leaf | leaf block length L in samples, 1 to 2^20 (iqcodec writes 8192) |
| scale | fc32: values are int16 / scale; a reader must check that scale > 0, scale < 1e30 compared in binary32 (scale < 0x7149F2CA as a bit pattern of a positive value), and that `f32(1 / scale)` is greater than 0. iqcodec writes 32767 unless told otherwise. sc16: ignored, any value. |

For fc32, compute `inv = f32(1.0 / scale)` once (a binary32 division), then each output value is the binary32
product `f32(v) * inv` (not `v / scale`). This matches UHD, which converts sc16 to fc32 as `v * (1/32767)`.
(Computing `1.0 / scale` in binary64 and rounding it once to binary32 gives the same `inv`.)

### 2.2 Chunks

Each chunk codes `n` samples independently of every other chunk.

- `n` is 1 to 2^21. Every chunk except the last one has exactly n = 2^21 (2097152).
- `shift` (0 to 3) is the number of *low bits* of each sample value that are coded separately (section 3).
- The three reserved bytes are 0.
- `nbytes` is the length of `payload`, at least 24 and at most 8n + 2^20.
- `crc` is the CRC-32C of the chunk's decoded output: 4n bytes for sc16 or 8n bytes for fc32, in the output
  byte order of section 1. CRC-32C is the Castagnoli CRC: reflected polynomial 0x82F63B78, initial value 0xFFFFFFFF
  and final XOR 0xFFFFFFFF. Its check value over the ASCII string `123456789` is 0xE3069283.
- `payload` is specified in section 4.

### 2.3 End

The end marker is a `u32` 0 where the next chunk's n would be, followed by `total`, the number of samples in all
chunks. Nothing may follow it. An empty stream (N = 0) is the header followed directly by the end marker
(28 bytes).

A reader must reject a stream that breaks any rule of section 2, or whose chunk does not decode (section 4) or does
not match its `crc`.

## 3. Sample values, high part and low bits

Within a chunk, with `M = 2^shift`, each sample value v (I or Q) is split into

```
high = v >> shift          (arithmetic shift)
low  = v & (M - 1)         (0 .. M - 1)
v    = high * M + low
```

High parts are predicted (4.6) and the prediction residuals are coded (4.5). Low bits are coded by a separate model
(4.4) when shift > 0. With shift = 0 every low is 0 and the low-bit section is unused.

`hmax = 32767 >> shift` and `hmin = -32768 >> shift` bound the high parts.

## 4. Chunk payload

```
payload = u32 len_side u32 len_low u32 len_raw u32 len_rans  side low raw rans
```

The four sections follow the 16-byte table in this order. Their lengths must add up to at most nbytes - 16. (iqcodec
writes exactly nbytes - 16, and readers ignore any bytes after the four sections.) len_rans is at least 8.

The decoder reads the sections in parallel, sample by sample (4.7):

- **side**: a bit stream (4.1) with the rANS frequency tables, then the block structure and prediction coefficients.
- **low**: a binary range-coder stream (4.2) with the low bits.
- **raw**: a bit stream (4.1) with the low mantissa bits and signs of the residuals.
- **rans**: the rANS stream (4.3) with the residual symbols.

### 4.1 Bit streams (side, raw)

Bits are read least significant bit first: byte 0 bit 0, byte 0 bit 1, …, byte 1 bit 0, and so on. `bits(k)`
reads k bits (0 to 32) and returns them as an unsigned integer, the first bit read being the least significant. So
the k bits are the next k bits of the stream, read as a little-endian integer. `bits(0)` is 0. Reading past the end
of the section yields zero bits.

`gamma()` reads an Elias gamma code: count zero bits until a one bit (at most 31 zeros; after 31 zeros the next bit
is not read), giving z, then return `2^z + bits(z)`.

```
gamma():
    z = 0
    while z < 31 and bits(1) == 0: z = z + 1
    return 2^z + bits(z)
```

### 4.2 Binary range decoder (low)

A binary range decoder in the style of LZMA's, but not identical to it: here `code < bound` decodes a 1,
probabilities have 16 bits with a variable adaptation rate, and normalisation follows each bit. Follow the
pseudocode, not LZMA.

State: 32-bit unsigned `range` and `code`, and a byte position in the section. A byte read past the end of the
section is 0.

```
init:        range = 0xFFFFFFFF; code = 0
             repeat 5 times: code = ((code << 8) | next_byte()) mod 2^32
```

A *counter* is a pair (p, n) of a 16-bit probability that the bit is 1, and a count. It starts as (32768, 0).

```
decode_bit(ctr):
    p1 = clamp(ctr.p >> 4, 1, 4095)
    bound = (range >> 12) * p1                      (32-bit)
    if code < bound: range = bound; bit = 1
    else:            code = code - bound; range = range - bound; bit = 0
    while range < 2^24:
        range = (range << 8) mod 2^32
        code  = ((code << 8) | next_byte()) mod 2^32
    target = 65535 if bit else 0
    ctr.p = ctr.p + (target - ctr.p) / (ctr.n + 2)  (integer division truncating towards zero)
    if ctr.n < 30: ctr.n = ctr.n + 1
    return bit
```

Note that `(target - p) / (n + 2)` truncates towards zero for negative values (as in C).

```
tree(ctrs, nbit):                         (ctrs: an array of counters indexed 1 .. 2^nbit - 1)
    node = 1
    repeat nbit times: node = 2 * node + decode_bit(ctrs[node])
    return node - 2^nbit

read_gap(ctrs):                           (ctrs: 32 counters; returns an integer >= 1)
    z = 0
    while true:
        b = decode_bit(ctrs[min(z, 31)])
        if b == 1 or z == 31: break
        z = z + 1
    r = 1
    repeat z times: r = 2 * r + decode_bit(fresh counter (32768, 30))
    return r
```

`fresh counter (32768, 30)` is a new counter for every bit, so these bits have probability exactly 1/2
(p1 = 2048).

### 4.3 rANS decoder (rans)

There are two interleaved rANS states, x[0] for I and x[1] for Q. Both are 32-bit unsigned, and they share one stream
of 16-bit little-endian words. Probabilities have 12 bits.

```
init:  x[0] = u32 at bytes 0..3 of the section, x[1] = u32 at bytes 4..7; the word position starts at byte 8

decode_symbol(ch, table):          (table: freq[64], start[64] from 4.5.1)
    s  = x[ch] & 4095
    sy = the symbol with start[sy] <= s < start[sy] + freq[sy]
    x[ch] = freq[sy] * (x[ch] >> 12) + s - start[sy]
    if x[ch] < 2^16 and at least 2 bytes remain in the section:
        x[ch] = (x[ch] << 16) | next u16 word
    return sy
```

### 4.4 Low bits (only when shift > 0)

Each channel (I and Q) has its own *tracker*, its own *gap* state, and its own model of counters:

- `amb[32]`;
- `exc[3][16]`, one tree per mode, nodes 1 .. 2^shift - 1;
- `gapctr[32]`.

All counters start as (32768, 0), and the gap state `gap[ch]` starts as -1.

The low bits of a channel form a sequence of symbols `s_t` in 0 .. M-1. Each one is *unwrapped* to an integer `u_t`
congruent to `s_t` mod M. Over long stretches the cumulative sum of the unwrapped values is a digital straight line
(the pattern left by first-order error diffusion). The tracker recognises that line with Debled-Rennesson's
arithmetic recognition of digital straight segments, and predicts the next value from it.

#### 4.4.1 Tracker state

```
Point = (x, y), integers

DSS: a, b, mu, rl                   integers (rl = a * last.x - b * last.y)
     Uf, Ul, Lf, Ll, last           Points (first/last upper and lower leaning points, last point)

Tracker: dss                        a DSS
         has                        0 or 1 (dss is valid)
         k                          integer, the base value: steps of the path are u - k in {0, 1}
         t                          integer, time of the last symbol (-1 before the first)
         Y                          integer, sum of all u so far
         hist[64]                   16-bit signed integers: hist[t mod 64] = u_t, stored with two's complement
                                    wraparound (u_t mod 2^16, mapped to -32768 .. 32767)

init: has = 0, k = 0, t = -1, Y = 0, dss and hist all zero
```

`hist` really is 16-bit. A steadily drifting low-bit pattern (for example 0, 1, 2, 3, 0, 1, … unwrapping to 0, 1, 2,
3, 4, 5, …) pushes `u` just past the int16 range; it is stored wrapped, and later values continue from the wrapped
value. `Y` is the sum of the unwrapped `u` (not wrapped). It only enters through `Mp` in `rebuild`, where a constant
offset of the y coordinates does not change any later result, so a decoder may also keep Y = 0.

```
dss_start(p):
    a = 0; b = 1; mu = rl = -p.y; Uf = Ul = Lf = Ll = last = p

dss_add(m, step):                  m.x = last.x + 1, m.y = last.y + step; returns 1 if still straight
    r = rl + a - step * b
    if mu <= r < mu + b:
        if r == mu:         Ul = m
        if r == mu + b - 1: Ll = m
        rl = r
    elif r == mu - 1:
        Lf = Ll; Ul = m
        a = m.y - Uf.y; b = m.x - Uf.x
        mu = a * m.x - b * m.y; rl = mu
    elif r == mu + b:
        Uf = Ul; Ll = m
        a = m.y - Lf.y; b = m.x - Lf.x
        mu = a * m.x - b * m.y - b + 1; rl = mu + b - 1
    else:
        return 0
    last = m
    return 1
```

The two `if` lines of the first case are both evaluated, in this order (when b = 1 both apply).

#### 4.4.2 Prediction

```
predict(tr):                        returns (mode, v0, actx)
    if not tr.has: return (N, -, -)
    d = tr.dss
    r0 = d.rl + d.a;  r1 = r0 - d.b
    ok0 = (d.mu - 1 <= r0 <= d.mu + d.b)
    ok1 = (r1 >= d.mu - 1)
    v0 = tr.k + (0 if ok0 else 1)
    if ok0 and ok1:
        actx = (1 if r0 == d.mu + d.b else 0) + 2 * nbits(min(d.b, 32767))
        return (A, v0, actx)                       (ambiguous: v0 or v0 + 1)
    return (D, v0, -)                              (determined: v0)
```

Modes are D = 0, A = 1, N = 2. These are also the indexes into `exc`.

#### 4.4.3 Update and rebuild

```
unwrap(tr, s):                      the integer congruent to s mod M closest to the previous value
    ref = tr.hist[tr.t mod 64] if tr.t >= 0 else s
    d = (s - ref) & (M - 1)
    return ref + (d - M if 2 * d > M else d)

update(tr, u):
    tr.t = tr.t + 1;  tr.Y = tr.Y + u
    tr.hist[tr.t mod 64] = int16(u)
    step = u - tr.k
    if tr.has and step in {0, 1} and dss_add(tr.dss, (tr.t, tr.dss.last.y + step), step) == 1:
        return
    rebuild(tr)

rebuild(tr):
    # longest run of recent values (newest first, at most 64) spanning at most two adjacent integers
    hn = min(tr.t + 1, 64)
    lo = hi = tr.hist[tr.t mod 64];  L = 1
    for j = 1 .. hn - 1:
        w = tr.hist[(tr.t - j) mod 64]
        if max(hi, w) - min(lo, w) > 1: break
        lo = min(lo, w); hi = max(hi, w); L = L + 1
    tr.k = lo
    # recognise the run backwards, on mirrored points
    r = new DSS; p = (0, 0); dss_start of r at p
    for j = 0 .. L - 1:
        w = tr.hist[(tr.t - j) mod 64]
        q = (p.x + 1, p.y + (w - tr.k))
        if dss_add of r with (q, w - tr.k) == 0: break
        p = q
    # back to forward orientation
    Mp = (tr.t, tr.Y - tr.k * tr.t)
    C = r.a * Mp.x - r.b * Mp.y
    d = tr.dss
    d.a = r.a; d.b = r.b; d.mu = C - r.mu - r.b + 1; d.rl = C
    d.Uf = Mp - r.Ll; d.Ul = Mp - r.Lf; d.Lf = Mp - r.Ul; d.Ll = Mp - r.Uf     (component-wise differences)
    d.last = Mp
    tr.has = 1
```

`mod 64` means the non-negative remainder, the same as `& 63`. In the backward loop the steps `w - tr.k` are 0 or 1.

#### 4.4.4 Decoding one low symbol

For each sample, the I channel is decoded and then the Q channel, both from the same range decoder:

```
low_symbol(ch):
    tr = tracker[ch]; mdl = model[ch]
    (mode, v0, actx) = predict(tr)
    if mode == N:
        s = tree(mdl.exc[2], shift); u = unwrap(tr, s)
    else:
        if gap[ch] < 0: gap[ch] = read_gap(mdl.gapctr) - 1   (number of regular steps before the next exception)
        if gap[ch] == 0:                                 (exception: the symbol is coded explicitly)
            s = tree(mdl.exc[mode], shift); u = unwrap(tr, s); gap[ch] = -1
        else:
            gap[ch] = gap[ch] - 1
            u = v0 + (decode_bit(mdl.amb[actx]) if mode == A else 0)
            s = u & (M - 1)
    update(tr, u)
    return s                                            (the low bits of this sample value)
```

Mode N only happens for the first symbol of a channel, because every update leaves `has = 1`.

### 4.5 Residuals

#### 4.5.1 Frequency tables (start of side)

There are 2 × 64 tables, one per channel (I = 0, Q = 1) and context (0 to 63), read in that order: channel-major,
then context.

```
for ch in 0, 1:
    for c in 0 .. 63:
        used = bits(1)
        start = 0
        for sy in 0 .. 63:
            f = gamma() - 1 if used else 0
            freq[ch][c][sy] = f;  start[ch][c][sy] = start;  start = start + f
```

A reader must reject a used table whose frequencies do not add up to exactly 4096, or that gives one of the symbols
60 to 63 a nonzero frequency (4.5.2). Valid streams never reference an unused table; a reader may reject that, or
leave it to the CRC. (iqcodec 0.2.x and 0.3.0 checked only that the running sum stays within 4096; 0.3.1 checks the
full rule. Streams written by any iqcodec satisfy it.)

#### 4.5.2 Symbols

A residual r is coded as a symbol (rANS) plus raw bits. The symbol carries the magnitude class
`b = nbits(|r|)` (0 to 16) and the top `tb = min(b - 1, 2)` mantissa bits. The raw stream carries, for b >= 1, the
sign (first bit) and then the remaining `b - 1 - tb` mantissa bits, as one `nraw`-bit field. The symbol table is:

```
symbol 0: r = 0, no raw bits
then, for b = 1 .. 16, nm = b - 1, tb = min(nm, 2), for top = 0 .. 2^tb - 1 (in this order):
    next symbol: base = 2^nm + top * 2^(nm - tb),  nraw = nm - tb + 1
```

This gives symbols 0 to 59; 60 to 63 have zero frequency in every table. To decode a residual from symbol sy:

```
v = bits_raw(nraw[sy])                        (from the raw stream, 4.1)
magnitude = base[sy] + (v >> 1)
r = -magnitude if (v & 1) else magnitude
```

#### 4.5.3 Contexts

The context of a residual depends on running magnitude averages, lagged by one sample. Per chunk, start with
aI = aQ = oI = oQ = 0 (unsigned integers).

```
qlog(v) = 0                         if v == 0
          1 + 4 * e + f             otherwise, with e = floor(log2 v), f = ((4 * v) >> e) & 3
                                    (the two bits after the leading one, zero-padded)
ctx(v)  = min(qlog(v), 63)

for each sample:
    cI = ctx(2 * oI + oQ);  cQ = ctx(oQ + oI / 2)          (/ is integer division)
    rI = residual from decode_symbol(0, table[0][cI])
    rQ = residual from decode_symbol(1, table[1][cQ])
    oI = aI;  oQ = aQ
    aI = aI - (aI >> 3) + 2 * |rI|;  aQ = aQ - (aQ >> 3) + 2 * |rQ|
```

The two symbols and raw-bit groups of a sample are decoded in this order: I symbol, I raw bits, Q symbol, Q raw bits.

### 4.6 Prediction

#### 4.6.1 Blocks (rest of side)

The chunk is divided into superblocks of 4L samples (the last one may be short), each made of 4 leaves of L
samples. For each superblock, in order, the side stream holds its split and then, for each of its blocks in order, the
block's coefficients:

```
for S0 = 0, 4L, 8L, ... while S0 < n:
    split = bits(1)
    if split == 1:
        split = split + 2 * bits(1)                  (first)
        split = split + 4 * bits(1)                  (second)
    bounds (in leaves):  split bit 0 clear:           [0, 4]
                         otherwise: [0] + ([1] if bit 1) + [2] + ([3] if bit 2) + [4]
    for each pair of consecutive bounds (i0, i1):
        b0 = min(S0 + i0 * L, n);  b1 = min(S0 + i1 * L, n)
        if b1 <= b0: skip (no data in the stream)
        sh = bits(5)
        w = min(bits(6), 32)
        for i in 0 .. 4K:  z = bits(w) (0 if w == 0);  q[i] = (z >> 1) if z even else -((z + 1) >> 1)
        decode samples b0 .. b1 - 1 with (q, sh) (4.7)
```

The coefficients are zigzag-coded: 0, -1, 1, -2, 2, … are written as 0, 1, 2, 3, 4, …

A reader must reject a block with sh > 30, with any |q[i]| > 32767, or with `sum |q[i]| * (hmax + 1) >= 2^30`.
These bounds keep every prediction sum below 2^30 in magnitude.

#### 4.6.2 Predictor

The predictor is widely linear and joint over I and Q, and works on high parts. Let I[t] and Q[t] be the high parts
of sample t. Values before the start of the chunk (t < 0) are 0.

```
rnd = 2^(sh - 1) if sh > 0 else 0
pI = sum over j = 1 .. K of  q[2j - 2] * I[t - j] + q[2j - 1] * Q[t - j]
I[t] = clamp((pI + rnd) >> sh, hmin, hmax) + rI
pQ = sum over j = 1 .. K of  q[2K + 2j - 2] * I[t - j] + q[2K + 2j - 1] * Q[t - j]   +   q[4K] * I[t]
Q[t] = clamp((pQ + rnd) >> sh, hmin, hmax) + rQ
```

The Q prediction uses the current I[t]. A decoded I[t] or Q[t] outside [hmin, hmax] means the stream is corrupt.

### 4.7 Decoding a chunk

```
read the section table; init the low decoder (4.2) and the rANS states (4.3)
read the frequency tables (4.5.1)
init the trackers, models and gap states (4.4); aI = aQ = oI = oQ = 0
for each block (4.6.1), for t = b0 .. b1 - 1:
    lowI = low_symbol(0); lowQ = low_symbol(1)             (only if shift > 0, else both 0)
    rI, rQ = residuals (4.5.3)
    I[t], Q[t] = predictions + residuals (4.6.2)
    output I[t] * M + lowI, then Q[t] * M + lowQ             (format of section 1)
```

The low decoder, the rANS words and the raw bits are three independent streams, so only the order within each
stream matters. Each is consumed in sample order, with I before Q.

Finally compare the CRC-32C of the chunk's output with `crc`.

Corruption inside the sections is detected by the checks above (tables, block parameters, the [hmin, hmax] range)
and by the CRC. Readers are not required to check that sections are consumed exactly, or the final rANS and range
coder states. Reads past the end of a section give zero bits or zero bytes, and the rANS decoder skips
renormalisation when fewer than 2 bytes remain; valid streams never depend on either.

## 5. Writers (informative)

iqcodec 0.2.0 to 0.3.x write K = 24, prec = 11, leaf = 8192 and scale = 32767 (or `-s`). They also:

- choose the shift once per stream, from the first 65536 samples;
- choose the split of each superblock and the coefficients by least squares.

None of these choices is needed to decode (the chunk sizes of 2.2, by contrast, are a rule of the format). Readers
from iqcodec 0.3.1 on accept any K, leaf and prec in the ranges above. 0.2.x and 0.3.0 readers require K = 24 and leaf = 8192. The streams in `tests/fixtures` marked `wrap` use
other values.
