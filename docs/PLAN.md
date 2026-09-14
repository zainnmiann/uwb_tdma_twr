# Unified UWB TDMA DS-TWR firmware — DWM1001-DEV + ESP32 UWB Pro

## Context

Two firmware trees implement the same initiator-driven fixed-slot DS-TWR protocol:

- `dwm1001_tdma_twr` — DWM1001-DEV (nRF52832 + DW1000), Decawave `deca_driver`, GNU Make.
- `esp32_uwb_tdma_twr` — Makerfabs ESP32 UWB Pro with Display (ESP32-WROVER + DW1000), Makerfabs `DW1000Class`, PlatformIO.

Goal: one firmware, one protocol, one API, where any board of either type can be a tag (initiator) or an anchor (responder) in the same network.

**Hard constraint: both existing trees are READ-ONLY.** No edits, no new files, nothing added inside either. All work goes in a new third repo.

### The protocol is already unified

The ESP32 tree was seeded by copying files and has not drifted. Byte-identical in both: `proto.h`, `slots.h/.c`, `sync.h/.c`, `timing.h/.c`, `ranging.h`, `app.h/.c`, `cfg.h`, `output.h`, `serial.h`. Trivially different: `ranging.c` (one include), `output.c` (display hook), `role_initiator.c` (+2 hunks), `role_responder.c` (+3 hunks), `radio.h`, `serial.c`. Genuinely platform: `cfg.c`, `uart_io`, `main`, `radio`.

So this is a **refactor into one tree with a real port layer**, not a protocol merge. The seam is already proven: `dwm1001_tdma_twr/sim/loop/` runs the real role loops off-target replacing only `radio.c` + `uart_io.c`.

### Measured on hardware (all four boards, this session)

Baseline + restore scripts: `scratchpad/baseline_all.json`, `scratchpad/restore_*.txt`.

| board | platform | role | `proc_resp` | `proc_rep` | `proc_poll` | `tx_late` |
|---|---|---|---:|---:|---:|---:|
| `0x0101` ANC1 | DWM1001 | RESP | **317** | **754** | — | 0 |
| `0x0F70` TAG1 | DWM1001 | INIT | — | — | **143** | **13142** |
| `0x8053` | ESP32 | RESP | **581** | **865** | — | 0 |
| `0x8052` | ESP32 | INIT | — | — | **396** | 1 |

DWM1001 at `N_R=8, D=2, pitch 615, hash 0x2C33`. ESP32 at `N_R=2, D=1, pitch 1895, hash 0xDD53`.

**The ESP32 is ~2× slower per path, not 5×** — yet it is configured with `proc_us=1500` against a measured worst case of 865. Both platforms' configured budgets are far above measurement.

Three findings that drive the plan:

1. **`proc_us` is the one number nothing measures.** It sets both pitches — 86% of the frame at `N_R=8` — and the path it must cover (initiator inter-slot RX re-arm, and Final arming) has no counter. Both trees guessed it.
2. **The DWM1001 tag is currently 79%+ `tx_late`** with `ss_fallback` on ~97% of ranges. `proc_poll` is only 143 µs against a 1000 µs budget, so it is the **Final**, bounded by `resp_pitch_us=615`. Partly an artifact of running 1 anchor against `resp_count=8` (seven empty slots burn `rx_timeout_us`), but it shows 615 is not comfortably sufficient.
3. **The ESP32 is slow for a fixable reason.** `DW1000.cpp:2193` charges ~14 µs per register access: an unconditional `delayMicroseconds(5)`, two FreeRTOS mutex takes, a per-byte `SPI.transfer()` loop, and Arduino `digitalWrite` for CS. The nRF52 does one EasyDMA burst at ~4–5 µs. **SPI clock is not the issue** — the ESP32 already runs 16 MHz vs the nRF's 8 MHz.

### The interop landmine

`cfg_hash()` hashes `preamble_len`, `pac_size` and `sfd_mode`. `apply_cfg()` in `radio_esp32.cpp` maps config onto one of four fixed `MODE_*` arrays and **ignores exactly those three fields** — so `RATE_850` silently transmits at 6.8 Mbps and `RATE_110` forces PRF64/2048. Two boards can **agree on the hash while transmitting different PHYs**, defeating the one check designed to catch that. Also: hardcoded `TX_POWER=0x1F1F1F1F` and `TC_PGDELAY=0xC0` (correct for channel 5 only) versus the DWM1001's per-part OTP reads — a systematic cross-platform range bias no schedule can correct.

## Stage 0 — measure the real pitch on a mixed network (hardware, do first)

All four boards are connected. Baseline is already captured and restorable.

1. Set one common config on all four: `init_count=2`, `resp_count=2`, `rate_div=1`, starting with the ESP32's known-safe generous timing (`resp_pitch 1895`, `report_pitch 1891`, `guard 400`). PHY already matches (ch5 / PRF64 / code 9 / 128 / 6.8 Mbps / STD).
2. `$SAVE` + `$RESET`. Confirm all four report the **same hash** and that `$RNG` shows **cross-platform pairs** — DWM1001 tag ↔ ESP32 anchor and vice versa. This premise is currently untested.
3. Walk `resp_pitch_us` down — 1895 → 1200 → 900 → 700 → 615 → lower — re-hashing all four at each step, watching `tx_late` and `ss_fallback` on both tags.
4. Stop at the smallest pitch where **both** platforms hold `tx_late=0`. That is the unified number, measured rather than guessed.

Keep the boards a couple of metres apart with clear line of sight — `rx_errors` was 44k–65k on the DWM1001s, and a bad physical setup would contaminate the readings.

Note: only `resp_count` values with real anchors present give meaningful `tx_late`. Empty slots burn `rx_timeout_us` and produce false lateness — this is what TAG1's 79% is showing.

## Stage 1 — new repo, core extracted

```
uwb_tdma_twr/
  core/        proto slots sync timing ranging cfg role_* output serial app
  port/        radio.h uart_io.h port_persist.h port_sys.h board_defaults.h
  port/nrf52/  radio.c uart_io.c port_persist.c port_sys.c board_defaults.c main.c syscalls.c
  port/esp32/  radio.cpp uart_io.cpp port_persist.cpp port_sys.cpp board_defaults.c
               display.* board_pins.h lib/DW1000/
  sim/         host sim + fork harness (a third port)
  test/ host/ pi_bridge/ vendor/
  Makefile          -> nRF52 hex
  platformio.ini    -> ESP32 bin
```

Core from the DWM1001 tree (keeps the doc comments), **except** take the ESP32's additive improvements: `radio_device_id()`, `cfg_persist_*`, `radio_part_id()` in the banner, the `boards.h` no-op shim, `deca_cfo.h`. The ESP32's `role_initiator.c` is a strict superset — it populates `m_rxp`/`m_fpp` from local RX quality, which the DWM1001 leaves stale when `report_enable=0`.

Port contract (minimal):
- `radio.h` (18 fns) + `radio_device_id()`; `uart_io.h` (8 fns)
- `bool port_persist_load/save(cfg_t*)` — replaces the hard-coded `CFG_FLASH_ADDR` + `nrf_nvmc_*`
- `void port_reset(void)` — replaces `NVIC_SystemReset()` / `esp32_system_reset()`
- `void port_led_toggle(port_led_t)` — replaces `LEDS_INVERT(BSP_LED_0_MASK)`
- `void port_ui_range(...)` — no-op on nRF52, OLED on ESP32
- `const board_defaults_t *board_defaults(void)` — per-board `proc_us`/`isr_us`/`margin_us` (not hashed, so they may legitimately differ). A missing definition must be a **link error**, not a weak-symbol fallback.

Mechanical fixes: `key_t` → `cfgkey_t`; move the five `_Static_assert`s from `main.c` into `proto.h` behind a `PROTO_STATIC_ASSERT` macro (C++ has no `_Static_assert`); guard **all** core headers with `extern "C"` and never `#include` inside a guard — `radio.h` currently includes `cfg.h` inside its block, a latent link bug. Set `-std=gnu11` for C and `-std=gnu++11` for C++ separately via a PlatformIO `extra_scripts` hook (`build_flags` applies to both and would break Arduino `.cpp`).

**Gate:** `sim/` and `sim/loop/` build and pass against `core/` with only their `SRC :=` path changed. This is the only verification available without hardware.

## Stage 2 — instrument the missing path

Add a counter for the Final-arming / inter-slot re-arm path so `proc_us` stops being guessed. This is why both trees mis-set it and why TAG1's lateness went unexplained. Note the existing `$MEASURE` figures are **survivor-biased** — recorded only after `radio_tx_at()` succeeds — so the new counter must record attempts, not successes.

## Stage 3 — make the ESP32 fast and exact

Cheap fixes first (hours, no library changes, ~496 → ~340 µs on the Final path):
- drop the dummy `setDelay(z)` whose `SYS_TIME` read and `DX_TIME` write are immediately overwritten (~42 µs)
- cache `SYS_CFG` instead of read-modify-write per RX arm (~40 µs)
- fold `radio_quality()` into the RX read — `RX_FINFO` is read twice and `FP_AMPL1` is adjacent to the RX timestamp (~45 µs × N_R)
- skip `radio_carrier_integrator()` when `report_enable=1` — it only feeds the SS-TWR fallback (~20 µs × N_R)
- drop the redundant `tx_status_clear()` + `idle()` before `newTransmit()` (~42 µs)
- drop RSSI from the OLED (user's decision) — removes the per-Response quality read entirely

Then the real fix: replace the SPI primitive — bulk transfer, no `delayMicroseconds(5)`, hoisted transaction, direct GPIO CS (~14 µs → ~2.5 µs fixed, 1.2 → ~0.55 µs/byte). **Write it as `readfromspi`/`writetospi` so it doubles as the `deca_driver` port**, which also fixes the PHY mismatch and TX-power/OTP calibration divergence, and collapses two radio implementations into one.

Downsides to weigh: it is a real port (~1–2 days), replaces a currently-working radio layer, `large_power_init()`'s PA/GPIO bits must be ported by hand, `dwt_initialise`/`dwt_otpread` must run at 2 MHz, and ESP32 antenna delay needs re-checking afterwards.

## Stage 4 — converge and merge tooling

`$MEASURE` on both platforms, take the fleet worst case, `$AUTOTIME`, freeze one hashed timing set, reprovision all boards together via `host/provision.py`. Keep the DWM1001 tree's `host/` (a superset of the ESP32's two scripts); make `provision.py` hardware-type aware. Fix `test/test_host.c` to **link** `core/ranging.c` rather than transcribe it.

## Verification

No hardware needed until Stage 4.

1. `make -C sim` and `make -C sim/loop` — the fork harness runs the real role loops (4 initiators × 12 responders) and asserts on real `$RNG` output. Primary gate, must pass at every stage.
2. `make -C test` — once it links the real core.
3. `make` → nRF52 hex; `pio run` → ESP32 bin, both from one `core/`.
4. Mixed bench: same hash on both, `hash_mismatch=0`, `tx_late=0`, `$RNG` with flag bit0 clear (proves full DS-TWR across hardware types).
5. Swap roles (ESP32 anchor + DWM1001 tag, then the reverse).
6. Scale to the mixed fleet, confirm per-tag rate matches `$RATE`, recalibrate antenna delay per board.

## Risks

1. **PlatformIO compiling the wrong source set** — `build_src_filter` with `src_dir = .` is the least-tested part. Mitigate: `rm -rf .pio` first; assert the core file count in a verbose build; fall back to `env.BuildSources()`.
2. **Silent `cfg_t` layout or `cfg_hash()` drift** — the struct is `memcpy`'d to flash/NVS and hashed into every Poll. The `key_t`→`cfgkey_t` rename touches ~60 `offsetof` rows. Mitigate: static-assert `sizeof(cfg_t)`, print `cfg_hash()` in the sim output.
3. **Moving a hashed field into the per-board defaults** — `resp_pitch_us` is hashed, `proc_us` is not. Confusing them means two boards that look identical refuse to range.

---

## Status update (2026-09-14, same session)

Stages 0, 1, 2, 3 (partial) and the build-system half of Stage 4 are **done**:

- **Stage 0** (hardware pitch walk): done. `resp_pitch_us=1100` confirmed as
  the safe floor for a mixed 2-anchor network; see `measurements/`.
- **Stage 1** (core extraction + port contract): done. `core/`, `port/`,
  `port/nrf52/`, `port/esp32/` built and verified against both `sim/` and
  `sim/loop/`.
- **Stage 2** (missing `proc_us` instrumentation): **not done this pass** --
  deferred in favour of the SPI/PHY fixes, which had bigger, verifiable
  payoff. `$MEASURE` still doesn't cover the inter-slot re-arm path.
- **Stage 3** (ESP32 fast + exact): done for the cheap fixes and the PHY
  exactness fix. The `deca_driver` migration (sharing one radio
  implementation across platforms) was **not** done -- the cheaper fixes
  plus the PHY correction were judged sufficient for now; revisit if a
  future pitch walk still finds the platforms too far apart.
- **Stage 4** (build systems): done. `host/provision.py` awareness of mixed
  hardware types and `test/test_host.c` linking the real `ranging.c` are
  still open (see README's "Known follow-ups").

All verification in this plan's "Verification" section passes as of this
commit; see the README for the exact numbers.
