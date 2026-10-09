# iqcodec

Lossless compression for IQ captures from software-defined radios (e.g. USRP / UHD).

It takes interleaved complex samples, either `sc16` (int16 I/Q) or `fc32` (float32 I/Q that are
int16 / 32767, which is what UHD produces when it converts `sc16` to `fc32`). Decompression
gives back the input bit for bit.

On a 630.8 MB USRP `fc32` capture (78.8 M samples):

| codec | size | of original | single-thread time vs FLAC (compress / decompress) |
|---|---|---|---|
| zlib | 145.7 MB | 23.1 % | |
| FLAC (int16, stereo) | 135.1 MB | 21.4 % | 1.0 / 1.0 |
| **iqcodec** | **81.2 MB** | **12.9 %** | ~1.4 / ~1.3 (Ryzen 9 9950X) |

## Install

```sh
brew install k3komatsu/tap/iqcodec
```

or build from source (C11, no dependencies):

```sh
make && make test && make install PREFIX=/usr/local
```

## Usage

```sh
iqcodec c capture.dat                    # compress fc32 input to capture.dat.iqc
iqcodec c --rm capture.dat               # ... verify by decoding, then remove capture.dat
iqcodec d capture.dat.iqc                # decompress to capture.dat (checksums verified)
iqcodec c capture.dat out.iqc            # explicit OUTPUT (replaces an existing file on success)
iqcodec t capture.iqc                    # verify only, no output
iqcodec i capture.iqc                    # format, sample count, size
iqcodec c -f sc16 capture.sc16           # int16 input
uhd_rx_cfile ... | iqcodec c - out.iqc   # stdin / stdout with -
```

**Part of a capture.** `--skip N --count M` decodes samples N to N+M-1 (complex samples, from 0). Only the
chunks that overlap the range are read and checked, so this is fast anywhere in a large file. If the stream
ends inside the range, the exit status is 3.

```sh
iqcodec d --skip 70000000 --count 1000000 capture.iqc part.dat
```

**Damaged files.** `d --salvage` decodes what it can: a chunk that fails its checksum becomes zeros (and is
reported), and a truncated or damaged stream ends at the last good chunk. The exit status is 3 when anything
was lost, and OUTPUT is kept. This also recovers the temporary file (`.iqcodec-XXXXXX`) that a failed recording, or one killed
with SIGKILL or by a power failure, leaves behind, up to its last fully written chunk. `t --salvage` lists every bad chunk.

**SigMF.** For `NAME.sigmf-data`, the input format comes from `core:datatype` in `NAME.sigmf-meta`
(`cf32_le` or `ci16_le`) unless `-f` is given.

Without OUTPUT, `c` writes INPUT.iqc and `d` strips `.iqc`, and neither replaces an existing file. `--rm`
removes INPUT only after success into a regular file that has been synced to disk, and only if INPUT is still
the same file and was not modified during the run (`c --rm` implies `-t`; it refuses `-l` and symbolic links). On a terminal, progress is
shown on stderr.

**Recording through a pipe.** When INPUT is a pipe or other stream, Ctrl-C does not throw the capture away.
The first Ctrl-C lets iqcodec read on until the recorder closes the pipe, then finish the file. A second Ctrl-C
stops reading at once and finishes the file with what has arrived, and a third aborts. SIGHUP (a closed
terminal or SSH session) and SIGTERM also finish the file. If a recording fails (disk full, an invalid value) or
is aborted, the data written so far is kept in a temporary file whose name is printed; `d --salvage` decodes it. A partial last sample is dropped with a warning. The
recorder must write only samples to the pipe; status messages belong on stderr.

| option | |
|---|---|
| `-f fc32\|sc16` | input sample format (compress) |
| `-s SCALE` | fc32 values are int16 / SCALE (default 32767) |
| `-l` | allow fc32 input that is not exactly int16 / SCALE (it gets quantized, and checksums and `-t` then cover the quantized values; otherwise iqcodec refuses) |
| `-t` | compress: decode each chunk after encoding and compare with the input |
| `-j N` | threads (default: CPUs, at most 8); chunks are coded independently |
| `-v` | statistics |
| `--skip N`, `--count M` | `d`, `t`: only samples N to N+M-1 |
| `--salvage` | `d`, `t`: keep going past damage (see above) |
| `--rm` | remove INPUT after success |

Options go before INPUT and OUTPUT.

Every chunk stores a CRC-32C of its samples, and the stream ends with the total sample count.
`d` and `t` check both, so corruption or truncation is an error instead of wrong data.
`c -t` also catches encoder faults before you delete the original.

A file OUTPUT goes to a temporary file in the same directory. It is synced and renamed into place only on
success, so a failed or interrupted run never leaves partial output and never replaces an existing file.
A symlink OUTPUT keeps its link, and its target receives the data. stdout, pipes and devices get data as it
is decoded, so a failure there can leave partial data. iqcodec refuses an output that is the input file,
whether by the same path, a symbolic or hard link, or a shell redirection.

## How it works

Per chunk of 2^21 samples:

- **Low bits.** USRP DC-offset correction leaves a first-order error-diffusion (sigma-delta) pattern in
  the low bits. Its cumulative sum is a digital straight line, tracked with Debled-Rennesson's arithmetic
  recognition. Deterministic steps cost nothing, ambiguous steps cost about a bit, and breaks are gap-coded.
  The number of low bits (0-3) is chosen by trial.
- **Prediction.** Widely-linear (joint I/Q) LPC of order 24, fitted by the encoder per block of 8192-32768
  samples (block length chosen per superblock). The integer coefficients are stored, so the decoder only
  runs an integer FIR.
- **Residuals.** Magnitude class plus 2 mantissa bits are rANS-coded with per-chunk static tables. The
  context is the recent residual magnitude. Remaining mantissa bits and the sign are stored raw.

The decoder is integer-only, so streams are identical across platforms. On x86-64, AVX-512 VNNI paths are
selected at run time (`IQC_NO_SIMD=1` disables them).

## Limitations

- fc32 input must be int16 / SCALE to be lossless. `-0.0` and NaN do not occur in UHD output and are not
  preserved (`-l` maps them to 0).
- Samples are complex pairs. sc8/sc12 data can be widened to sc16 first.

## License

MIT
