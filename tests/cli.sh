#!/bin/sh
# CLI checks: round trips (files, pipes, formats, threads), checksums, and output safety.
# Usage: sh tests/cli.sh ./iqcodec
set -e
BIN=${1:-./iqcodec}
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
fail() { echo "FAIL: $*"; exit 1; }

./tests/roundtrip gen 5000000 "$T/a.fc32"                       # > 2 chunks
head -c 1000000 /dev/urandom > "$T/b.sc16"

# round trips
"$BIN" c "$T/a.fc32" "$T/a.iqc"
"$BIN" d "$T/a.iqc" "$T/a.out"
cmp "$T/a.fc32" "$T/a.out"
"$BIN" c -j 1 - - < "$T/a.fc32" | "$BIN" d -j 3 - - | cmp - "$T/a.fc32"
"$BIN" c -t -f sc16 "$T/b.sc16" "$T/b.iqc"
"$BIN" d "$T/b.iqc" - | cmp - "$T/b.sc16"
"$BIN" t "$T/a.iqc" && "$BIN" t - < "$T/b.iqc"
: > "$T/empty"
"$BIN" c "$T/empty" "$T/e.iqc" && "$BIN" d "$T/e.iqc" - | cmp - "$T/empty"

# corruption: one changed payload byte, a truncated file, garbage
cp "$T/a.iqc" "$T/bad.iqc"
pos=200000
old=$(od -An -tu1 -j $pos -N1 "$T/bad.iqc" | tr -d ' ')
printf "$(printf '\\%03o' $(( (old + 1) % 256 )))" | dd of="$T/bad.iqc" bs=1 seek=$pos conv=notrunc 2>/dev/null
if "$BIN" t "$T/bad.iqc" 2>/dev/null; then fail "corrupt payload passed t"; fi
if "$BIN" d "$T/bad.iqc" "$T/bad.out" 2>/dev/null; then fail "corrupt payload decoded"; fi
test ! -e "$T/bad.out" || fail "partial output left behind"
head -c 1000000 "$T/a.iqc" > "$T/short.iqc"
if "$BIN" t "$T/short.iqc" 2>/dev/null; then fail "truncated stream passed t"; fi
if "$BIN" d "$T/b.sc16" - >/dev/null 2>&1; then fail "garbage accepted"; fi

# output safety: never write over the input, never clobber an existing file on failure
cp "$T/b.sc16" "$T/keep"
if "$BIN" c -f sc16 "$T/keep" "$T/keep" 2>/dev/null; then fail "same file accepted"; fi
ln -s "$T/keep" "$T/link"; ln "$T/keep" "$T/hard"
if "$BIN" c -f sc16 "$T/keep" "$T/link" 2>/dev/null; then fail "symlink to input accepted"; fi
if "$BIN" c -f sc16 "$T/keep" "$T/hard" 2>/dev/null; then fail "hard link to input accepted"; fi
if "$BIN" c -f sc16 "$T/keep" - >> "$T/keep" 2>/dev/null; then fail "stdout appended to input accepted"; fi
cmp "$T/keep" "$T/b.sc16" || fail "input modified"
echo previous > "$T/old.iqc"
if "$BIN" c "$T/b.sc16" "$T/old.iqc" 2>/dev/null; then fail "inexact fc32 accepted"; fi   # sc16 bytes read as fc32
test "$(cat "$T/old.iqc")" = previous || fail "existing output clobbered on failure"
ls -a "$T" | grep -q '^\.iqcodec-' && fail "temporary file left behind"
"$BIN" c -l "$T/b.sc16" "$T/old.iqc"
echo "ok   cli"
