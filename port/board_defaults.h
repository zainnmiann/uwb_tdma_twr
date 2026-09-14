/**
 * @file    board_defaults.h
 * @brief   Per-board factory defaults, supplied by exactly one translation
 *          unit in each port/<target>/ (and sim/port/). A missing definition
 *          is a link error, which is the point -- a silent fallback would let
 *          a board ship with another platform's timing.
 *
 * Everything here is a *starting point*: $SET/$SAVE overrides all of it.
 *
 * HASHED (must agree network-wide -- see cfg_hash(), section 8.2 rule 6):
 *   init_count resp_count frame_dur_us resp_base_us resp_pitch_us
 *   report_base_us report_pitch_us guard_us rate_div
 *
 * NOT HASHED (legitimately per-board):
 *   proc_us isr_us margin_us proc_poll_us proc_final_us rx_timeout_us
 *   sync_mmax uart_baud fw_ver
 *
 * Consequence: two boards of different platforms in one network MUST be
 * given matching hashed values via $SET/$SAVE -- the defaults below differ
 * per platform, so a freshly-flashed mixed nRF52/ESP32 network does not
 * range out of the box. Run $AUTOTIME on the slowest board in the fleet,
 * then $SET the hashed keys on every other board to match, $SAVE, $RESET.
 *
 * Measured on hardware (2026-09-14, both platforms, N_R=2/N_R=8): the
 * platforms are within ~2x of each other per timing-critical path, not the
 * ~5x the shipped defaults implied -- see docs/PLAN.md and
 * measurements/pitch_sweep_2026-09-14.json for the walk that established
 * resp_pitch_us=1100 as a safe common floor across a real mixed network.
 */
#ifndef BOARD_DEFAULTS_H_
#define BOARD_DEFAULTS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    const char *fw_ver;

    uint8_t  init_count;         /* N_I                    -- hashed */
    uint8_t  resp_count;         /* N_R                    -- hashed */

    uint32_t frame_dur_us;       /* T_f                    -- hashed */
    uint32_t resp_base_us;       /* T_r0                   -- hashed */
    uint32_t resp_pitch_us;      /* delta_r                -- hashed */
    uint32_t report_base_us;     /* T_p0                   -- hashed */
    uint32_t report_pitch_us;    /* delta_p                -- hashed */
    uint32_t guard_us;           /* T_g                    -- hashed */
    uint8_t  rate_div;           /* D                      -- hashed */

    uint32_t rx_timeout_us;
    uint8_t  sync_mmax;
    uint32_t proc_us;            /* eq (12)/(13) input, NOT hashed */
    uint32_t isr_us;
    uint32_t margin_us;
    uint32_t proc_poll_us;
    uint32_t proc_final_us;

    uint32_t uart_baud;
} board_defaults_t;

/** Supplied by exactly one translation unit in the active port. */
const board_defaults_t *board_defaults(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_DEFAULTS_H_ */
