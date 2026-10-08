CC      ?= cc
CFLAGS  ?= -O3
PREFIX  ?= /usr/local
# -ffp-contract=off keeps the float -> int16 rounding identical to rint() (no fused multiply-add).
ALL_CFLAGS = $(CFLAGS) -std=gnu11 -ffp-contract=off -Wall -Wextra
LDLIBS  = -lm -lpthread

iqcodec: src/main.c src/iqc.c src/iqc.h src/crc32c.c src/crc32c.h
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ src/main.c src/iqc.c src/crc32c.c $(LDLIBS)

tests/roundtrip: tests/roundtrip.c src/iqc.c src/iqc.h src/crc32c.c src/crc32c.h
	$(CC) $(ALL_CFLAGS) -Isrc $(LDFLAGS) -o $@ tests/roundtrip.c src/iqc.c src/crc32c.c $(LDLIBS)

test: iqcodec tests/roundtrip
	./tests/roundtrip
	sh tests/cli.sh ./iqcodec

install: iqcodec
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 755 iqcodec $(DESTDIR)$(PREFIX)/bin/iqcodec

clean:
	rm -f iqcodec tests/roundtrip

.PHONY: test install clean
