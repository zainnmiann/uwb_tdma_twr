# Host simulator. Links the firmware's portable modules unmodified from
# ../core, against this directory's own port/ stubs (a third port, alongside
# nrf52 and esp32). No deca_driver / nrf.h dependency any more: ranging.c
# gets its DW1000 constants from core/dw1000_consts.h and cfg.c reaches
# storage through port_persist.h -- both platform-neutral now.
CC      ?= cc
CORE    := ../core
PORTH   := ../port
CFLAGS  ?= -std=c11 -O1 -g -Wall -Wextra -I. -Iport -I$(CORE) -I$(PORTH)
# Header dependencies. Without these a stale object silently survives an
# edit to sim.h -- which cost an afternoon: sim_radio.o kept an older
# nclk_t layout and read drift_ppm where phase_dtu now lives.
CFLAGS  += -MMD -MP

OBJS := sim_main.o sim_radio.o sim_trace.o \
        port/port_persist.o port/port_sys.o port/board_defaults.o \
        cfg.o timing.o slots.o ranging.o sync.o

all: run

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<
%.o: $(CORE)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

sim: $(OBJS)
	$(CC) -o $@ $(OBJS) -lm

run: sim
	./sim

clean:
	rm -f sim *.o *.d port/*.o port/*.d

-include $(OBJS:.o=.d)

.PHONY: all run clean
