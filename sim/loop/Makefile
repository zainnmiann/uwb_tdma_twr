# Fork-per-node harness: the real role loops, off target. Links role_*.c,
# app.c, output.c, serial.c UNMODIFIED from ../../core, against this
# directory's own IPC-backed radio/UART and ../port's persist/sys/defaults
# stubs (shared with sim/'s own harness -- both are equally unreachable
# there: no console input, so $SAVE/$RESET/the OLED are never exercised).
CC      ?= cc
CORE    := ../../core
PORTH   := ../../port
CFLAGS  ?= -std=c11 -O1 -g -Wall -Wextra
CFLAGS  += -I. -I.. -I../port -I$(CORE) -I$(PORTH)
# Header dependencies. Their absence in sim/Makefile once cost an afternoon to
# a stale object holding an older struct layout.
CFLAGS  += -MMD -MP

LOCAL := harness.o ipc.o node_radio.o node_uart.o \
         ../port/port_persist.o ../port/port_sys.o ../port/board_defaults.o
FW    := role_initiator.o role_responder.o app.o cfg.o timing.o slots.o \
         sync.o ranging.o output.o serial.o
OBJS  := $(LOCAL) $(FW)

all: run

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<
%.o: $(CORE)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

harness: $(OBJS)
	$(CC) -o $@ $(OBJS) -lm

run: harness
	./harness

clean:
	rm -f harness *.o *.d ../port/*.o ../port/*.d

.PHONY: all run clean

-include $(OBJS:.o=.d)
