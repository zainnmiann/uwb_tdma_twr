# Host-side regression tests. Requires only a native C compiler.
CC      ?= cc
CFLAGS  ?= -std=c11 -O1 -Wall -Wextra -Wno-unused-parameter

all: run

test_host: test_host.c
	$(CC) $(CFLAGS) -o $@ $< -lm

run: test_host
	./test_host

clean:
	rm -f test_host

.PHONY: all run clean
