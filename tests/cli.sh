#!/bin/sh
# CLI round trips: files and pipes, both formats, threads, lossy guard.  Usage: sh tests/cli.sh ./iqcodec
set -e
BIN=${1:-./iqcodec}
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT

./tests/roundtrip gen 5000000 "$T/a.fc32"                       # > 2 chunks
head -c 1000000 /dev/urandom > "$T/b.sc16"

"$BIN" c "$T/a.fc32" "$T/a.iqc"
"$BIN" d "$T/a.iqc" "$T/a.out"
cmp "$T/a.fc32" "$T/a.out"
"$BIN" c -j 1 - - < "$T/a.fc32" | "$BIN" d -j 3 - - | cmp - "$T/a.fc32"
"$BIN" c -f sc16 "$T/b.sc16" "$T/b.iqc"
"$BIN" d "$T/b.iqc" - | cmp - "$T/b.sc16"
: > "$T/empty"
"$BIN" c "$T/empty" "$T/e.iqc" && "$BIN" d "$T/e.iqc" - | cmp - "$T/empty"

# fc32 that is not int16 / 32767: refused (and no partial output), accepted with -l
head -c 800000 /dev/urandom | "$BIN" c -f sc16 - "$T/r.iqc"
"$BIN" d "$T/r.iqc" "$T/r.sc16"
if "$BIN" c "$T/r.sc16" "$T/r2.iqc" 2>/dev/null; then echo "FAIL: inexact fc32 accepted"; exit 1; fi
test ! -e "$T/r2.iqc"
"$BIN" c -l "$T/r.sc16" "$T/r2.iqc"
if "$BIN" d "$T/b.sc16" - >/dev/null 2>&1; then echo "FAIL: garbage accepted"; exit 1; fi
echo "ok   cli"
