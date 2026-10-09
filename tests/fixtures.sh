#!/bin/sh
# Format regression test: the stream files in tests/fixtures must keep decoding to the same samples (FORMAT.md,
# version 2), and the ones written by iqcodec c must keep being reproduced byte for byte by the encoder.
# Usage: sh tests/fixtures.sh ./iqcodec
set -e
BIN=${1:-./iqcodec}
case $BIN in /*) ;; *) BIN=$(pwd)/$BIN ;; esac
cd "$(dirname "$0")/fixtures"
sha() { if command -v sha256sum >/dev/null 2>&1; then sha256sum; else shasum -a 256; fi | cut -d' ' -f1; }
n=0
while read -r sum name how opts || [ -n "$sum" ]; do
  case $sum in '#'*|'') continue ;; esac
  [ -f "$name" ] || { echo "FAIL: $name is missing"; exit 1; }
  "$BIN" t "$name" || { echo "FAIL: $name does not decode"; exit 1; }
  got=$("$BIN" d "$name" - | sha)
  [ "$got" = "$sum" ] || { echo "FAIL: $name decodes to different samples"; exit 1; }
  if [ "$how" = c ]; then
    "$BIN" d "$name" - | "$BIN" c $opts - - | cmp -s - "$name" || { echo "FAIL: $name is no longer reproduced by c"; exit 1; }
  fi
  n=$((n + 1))
done < MANIFEST
[ $n -gt 0 ] || { echo "FAIL: no fixtures"; exit 1; }
echo "ok   fixtures ($n files)"
