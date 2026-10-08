#!/bin/sh
# CLI checks: round trips (files, pipes, formats, threads), checksums, signals and output safety.
# Usage: sh tests/cli.sh ./iqcodec
set -e
BIN=${1:-./iqcodec}
case $BIN in /*) ;; *) BIN=$(pwd)/$BIN ;; esac
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
fail() { echo "FAIL: $*"; exit 1; }
must_fail() { if "$@" >/dev/null 2>&1; then fail "succeeded: $*"; fi; }
no_temp() { if ls -a "$T" | grep -q '^\.iqcodec-'; then fail "temporary file left behind ($1)"; fi; }

./tests/roundtrip gen 5000000 "$T/a.fc32"                       # > 2 chunks
head -c 1000000 /dev/urandom > "$T/b.sc16"

# round trips (every command checked on its own: set -e ignores failures left of && and inside pipelines)
"$BIN" c "$T/a.fc32" "$T/a.iqc"
"$BIN" d "$T/a.iqc" "$T/a.out"
cmp "$T/a.fc32" "$T/a.out"
"$BIN" c -j 1 - - < "$T/a.fc32" > "$T/p.iqc"
"$BIN" d -j 3 - - < "$T/p.iqc" > "$T/p.out"
cmp "$T/p.out" "$T/a.fc32"
"$BIN" c -t -f sc16 "$T/b.sc16" "$T/b.iqc"
"$BIN" d "$T/b.iqc" - > "$T/b.out"
cmp "$T/b.out" "$T/b.sc16"
"$BIN" t "$T/a.iqc"
"$BIN" t - < "$T/b.iqc"
: > "$T/empty"
"$BIN" c "$T/empty" "$T/e.iqc"
"$BIN" d "$T/e.iqc" "$T/e.out"
cmp "$T/e.out" "$T/empty"
"$BIN" d "$T/a.iqc" /dev/stdout > "$T/s.out"                    # descriptor aliases: written in place
cmp "$T/s.out" "$T/a.fc32"
echo HEAD > "$T/app.out"; cp "$T/app.out" "$T/app.ref"; cat "$T/b.sc16" >> "$T/app.ref"
"$BIN" d "$T/b.iqc" /dev/stdout >> "$T/app.out"                  # ... and >> still appends
cmp "$T/app.out" "$T/app.ref"
if [ -d /dev/shm ] && [ -w /dev/shm ]; then                       # regular files under /dev are files
  S="/dev/shm/iqcodec-test-$$"; echo previous > "$S"
  if "$BIN" c "$T/b.sc16" "$S" 2>/dev/null; then rm -f "$S"; fail "inexact fc32 accepted"; fi
  test "$(cat "$S")" = previous || { rm -f "$S"; fail "/dev/shm output clobbered on failure"; }
  rm -f "$S"
fi

# lossy (-l): output decodes and verifies (to the quantized values, which are then exact)
"$BIN" c -l -t "$T/b.sc16" "$T/l.iqc"                           # sc16 bytes read as fc32: inexact
"$BIN" t "$T/l.iqc"
"$BIN" d "$T/l.iqc" "$T/l.fc32"
"$BIN" c "$T/l.fc32" "$T/l2.iqc"
must_fail "$BIN" c "$T/b.sc16" "$T/l3.iqc"                      # refused without -l

# corruption: one changed payload byte, truncation, trailing data, garbage, an empty chunk
cp "$T/a.iqc" "$T/bad.iqc"
pos=200000
old=$(od -An -tu1 -j $pos -N1 "$T/bad.iqc" | tr -d ' ')
printf "$(printf '\\%03o' $(( (old + 1) % 256 )))" | dd of="$T/bad.iqc" bs=1 seek=$pos conv=notrunc 2>/dev/null
must_fail "$BIN" t "$T/bad.iqc"
must_fail "$BIN" d "$T/bad.iqc" "$T/bad.out"
test ! -e "$T/bad.out" || fail "partial output left behind"
head -c 1000000 "$T/a.iqc" > "$T/short.iqc"
must_fail "$BIN" t "$T/short.iqc"
cat "$T/b.iqc" "$T/b.iqc" > "$T/twice.iqc"
must_fail "$BIN" t "$T/twice.iqc"
must_fail "$BIN" d "$T/b.sc16" "$T/g.out"
{ head -c 16 "$T/b.iqc"; printf '\001\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000'; } > "$T/z.iqc"
if "$BIN" t "$T/z.iqc" 2> "$T/z.err"; then fail "empty chunk accepted"; fi
grep -q corrupt "$T/z.err" || fail "empty chunk: $(cat "$T/z.err")"

# output safety: never write over the input, never clobber an existing file on failure
cp "$T/b.sc16" "$T/keep"
must_fail "$BIN" c -f sc16 "$T/keep" "$T/keep"
ln -s "$T/keep" "$T/link"; ln "$T/keep" "$T/hard"
must_fail "$BIN" c -f sc16 "$T/keep" "$T/link"
must_fail "$BIN" c -f sc16 "$T/keep" "$T/hard"
if "$BIN" c -f sc16 "$T/keep" - >> "$T/keep" 2>/dev/null; then fail "stdout appended to input accepted"; fi
cmp "$T/keep" "$T/b.sc16" || fail "input modified"
echo previous > "$T/old.iqc"
must_fail "$BIN" c "$T/b.sc16" "$T/old.iqc"                     # inexact fc32: fails after opening OUTPUT
test "$(cat "$T/old.iqc")" = previous || fail "existing output clobbered on failure"
no_temp "failed run"
ln -s nothere "$T/dang"                                           # dangling symlink: nothing created on failure
must_fail "$BIN" d "$T/bad.iqc" "$T/dang"
test ! -e "$T/nothere" || fail "dangling symlink target created by a failed run"
"$BIN" d "$T/b.iqc" "$T/dang"
cmp "$T/nothere" "$T/b.sc16"
echo old > "$T/target"; ln -s "$T/target" "$T/out.lnk"            # writes go through a symlink OUTPUT
"$BIN" d "$T/b.iqc" "$T/out.lnk"
test -L "$T/out.lnk" || fail "symlink replaced"
cmp "$T/target" "$T/b.sc16"

# signals: inherited SIG_IGN is respected (nohup), fatal signals remove the temporary file
mkfifo "$T/fifo"
(sleep 1; cat "$T/b.sc16") > "$T/fifo" &
(trap '' HUP; exec "$BIN" c -f sc16 "$T/fifo" "$T/h.iqc") & pid=$!
sleep 0.5; kill -HUP $pid
wait $pid || fail "ignored SIGHUP killed the run"
"$BIN" d "$T/h.iqc" "$T/h.out"
cmp "$T/h.out" "$T/b.sc16"
(sleep 2; cat "$T/b.sc16") > "$T/fifo" &                    # (background jobs start with INT/QUIT ignored)
"$BIN" c -f sc16 "$T/fifo" "$T/q.iqc" & pid=$!
sleep 0.5; kill -TERM $pid
if wait $pid; then fail "SIGTERM did not stop the run"; fi
wait
no_temp SIGTERM
test ! -e "$T/q.iqc" || fail "output created by an interrupted run"
if (ulimit -f 20; exec "$BIN" c -f sc16 "$T/b.sc16" "$T/f.iqc") 2>/dev/null; then fail "file size limit ignored"; fi
no_temp SIGXFSZ

# options
must_fail "$BIN" c -j x "$T/a.fc32" "$T/o.iqc"
must_fail "$BIN" c -j 0 "$T/a.fc32" "$T/o.iqc"
must_fail "$BIN" c -s 1e39 "$T/a.fc32" "$T/o.iqc"
must_fail "$BIN" c -s 32767abc "$T/a.fc32" "$T/o.iqc"
must_fail "$BIN" c "$T/a.fc32" ""
echo "ok   cli"
