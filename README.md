# uwb_tdma_twr — unified UWB TDMA DS-TWR firmware

One firmware, one protocol, one API, for two hardware platforms:

- **DWM1001-DEV** (nRF52832 + DW1000), Decawave `deca_driver`, GNU Make.
- **Makerfabs ESP32 UWB Pro with Display** (ESP32-WROVER + DW1000), PlatformIO.

Any board of either type can be a tag (initiator) or an anchor (responder)
in the same network — confirmed on hardware (see below).

## Source trees (READ-ONLY references)

`~/Documents/dwm1001_tdma_twr` and `~/Documents/esp32_uwb_tdma_twr` are
**never modified**. This repo was built by copying from them.

## Layout

```
core/           the protocol core -- byte-identical across every target.
                proto/slots/sync/timing/ranging/cfg/role_*/output/serial/app
port/           the port contract: radio.h, uart_io.h, port_persist.h,
                port_sys.h, board_defaults.h
port/nrf52/     DWM1001-DEV: deca_driver, nRF5 SDK 14.2.0 (vendored)
port/esp32/     ESP32 UWB Pro: Makerfabs DW1000 library (vendored),
                display.cpp (OLED)
sim/            host simulator (two harnesses -- a third "port")
test/           host regression tests
Makefile        -> nRF52 .hex, via core/ + port/nrf52/
platformio.ini  -> ESP32 .bin, via core/ + port/esp32/
docs/PLAN.md    the staged implementation plan
measurements/   captured hardware data from the sessions that built this
tools/          host scripts used to measure/reconfigure boards over serial
```

## Build

```sh
make                # nRF52: _build/uwb_tdma_twr.hex
export PATH="$HOME/Library/Python/3.9/bin:$PATH"
pio run              # ESP32: .pio/build/esp32_uwb_pro_display/firmware.bin
```

## Verify

```sh
make lint-core       # core/ has picked up no platform dependency
make lint-externc    # every core/port header guards extern "C" correctly
(cd sim && make)        # host simulator, 4 tags x 12 anchors, two scenarios
(cd sim/loop && make)   # fork harness: the REAL role_*.c/app.c/serial.c linked
                        # unmodified against this core, over IPC
(cd test && make)       # host regression tests on the estimators/hash/sync
```

All five pass clean from a fresh checkout (2026-09-14): nRF52 build 43428 B
flash / 7832 B RAM, ESP32 build 337997 B flash / 29904 B RAM (25.8%/9.1%),
sim PASS (0 collisions, 0 desync), fork harness PASS (41497 transmissions,
0 tx_late, 0 desync across 4 tags), test_host PASS (27764 checks).

## What changed from the two source trees

- **Core extracted once.** The protocol layer was already identical between
  the two trees (copied, not diverged) -- this repo makes that explicit with
  one `core/` and a real port contract (`port/*.h`), rather than two
  near-duplicate source trees.
- **PHY-exactness fix (`port/esp32/radio.cpp`).** The ESP32 previously
  snapped `datarate`/`prf`/`preamble_len` onto one of four fixed library
  `MODE_*` triples, silently transmitting the wrong PHY away from the
  defaults (`RATE_850` ran at 6800 kbps; `RATE_110` forced PRF64/2048).
  `preamble_len`, `pac_size`, `sfd_mode` and `channel` are all in
  `cfg_hash()`, so two boards could agree on the hash while running
  incompatible PHYs -- the one check meant to catch exactly that. Replaced
  with exact per-field programming (`setDataRate`/`setPulseFrequency`/
  `setPreambleLength` called directly, an explicit SFD override since the
  library hardcodes SFD from data rate alone, a manual PAC override path,
  and `DRX_SFDTOC` -- never programmed before -- computed the same way
  `radio.c` does on the nRF52 side).
- **SPI-cost fixes (`port/esp32/radio.cpp`).** Removed the dummy
  `setDelay()` calls (their DX_TIME write was always immediately
  overwritten) in favour of setting `TXDLYS`/`RXDLYS` directly on the
  library's own cached `_sysctrl` buffer; switched `rx_set_timeout()` to the
  library's cached `_syscfg` instead of a read-modify-write every RX arm;
  folded `radio_quality()`'s register reads into the `radio_rx_poll()` path
  that already reads `RX_FINFO`, removing a duplicate read. Left
  `tx_status_clear()`'s full-register clear in place after checking it is
  **not** redundant with the library's `newTransmit()` -- the library's
  internal clear does not cover `HPDWARN`/`TXPUTE`, and dropping it would
  reintroduce the "one late TX permanently latches transmit failure" bug the
  DWM1001 reference's own `radio.c` comment warns about.
- **Measured, not guessed, mixed-network timing** (see
  `measurements/pitch_sweep_2026-09-14.json`). All four boards were put on
  one config and walked down from `resp_pitch_us=1895` to `500`; `1100` is
  the confirmed floor where both platforms hold `tx_late=0` at `N_R=2`.
  Cross-platform DS-TWR was confirmed working in both directions (ESP32 tag
  -> DWM1001 anchor and DWM1001 tag -> ESP32 anchor) before this repo's fixes
  were applied to the radio layer.

## Known follow-ups (see docs/PLAN.md)

- `test/test_host.c` transcribes the estimator from `ranging.c` rather than
  linking it -- passes today, but could silently drift from `core/ranging.c`
  undetected. `sim/loop`'s fork harness links the real code and is the
  stronger gate; consider rewriting `test_host.c` to link `core/ranging.c`
  directly instead of transcribing it.
- Re-run `$MEASURE`/`$AUTOTIME` on the ESP32 with the SPI fixes applied and
  at `N_R=8` to get a fresh `board_defaults.c` timing set -- the shipped
  defaults are still the pre-fix, conservative numbers (see the comment in
  `port/esp32/board_defaults.c`).
- `TC_PGDELAY` in the ESP32's `large_power_init()` (vendored library) is
  hardcoded correct for channel 5 only; a channel change needs this ported
  from `radio.c`'s `k_pgdelay[]` table if other channels are ever used.
