# Changelog

Each entry says whether the stream format changed. The format is specified in [FORMAT.md](FORMAT.md). Every
version-2 stream stays readable by all later releases.

## 0.3.1

Format: unchanged (version 2). The encoder writes the same bytes as 0.2.0 to 0.3.0.

- FORMAT.md: complete specification of the version-2 stream, checked by an independent decoder written from the
  document alone.
- Format regression tests: `tests/fixtures` holds reference streams. CI checks that they decode to the recorded
  SHA-256, and that the encoder still reproduces them byte for byte.
- Readers take K and the leaf length from the stream header (any value the format allows). They no longer require
  K = 24 and leaf = 8192.
- The reserved bytes of a chunk header must be 0.
- `c -t` with a file OUTPUT also reads the written file back and tests it (checksums, sample count).
- A partial last sample on a pipe is still dropped with a warning, but the exit status is now 3 (incomplete).

## 0.3.0

Format: unchanged (version 2). Same encoder output as 0.2.2.

- `d` / `t --skip N --count M`: decode a sample range.
- `i`: stream info.
- `--salvage`: decode damaged streams (exit status 3).
- gzip-like default names and `--rm`.
- SigMF `core:datatype`.
- Progress on a terminal.
- SIGHUP and SIGTERM finish a recording. A failed recording keeps its data for `--salvage`.
- Options before operands on every platform.
- Every chunk except the last must be full.

## 0.2.2

Format: unchanged (version 2). Same encoder output as 0.2.1.

- Ctrl-C while recording from a pipe finishes the file instead of losing it.
- 20–30 % faster: NEON paths on ARM, and reading, coding and writing overlap.

## 0.2.1

Format: unchanged (version 2).

- Checksums of lossy (`-l`) files now cover the quantized values. Lossy files written by 0.2.0 fail their
  checksum and must be recompressed. Lossless 0.2.0 files are unaffected.
- Decoder hardening against crafted input.
- Output safety fixes.

## 0.2.0

Format: **new, version 2** (CRC-32C per chunk, end marker with the total sample count). Version-1 files of 0.1.0
cannot be read.

- `t` command and `c -t`.
- Safe output: temporary file plus rename, and an output that is the input is refused.

## 0.1.0

Format: version 1 (not supported by later releases).

- First release.
