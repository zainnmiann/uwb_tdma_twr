/**
 * @file    board_defaults.c
 * @brief   ESP32 UWB Pro factory defaults -- see board_defaults.h for the
 *          hashed/not-hashed split. These are the values a freshly-flashed,
 *          unprovisioned board boots with.
 *
 * IMPORTANT for a mixed fleet: these hashed values do NOT match the DWM1001
 * port's defaults (frame_dur_us 33047 vs 4000). A network mixing platforms
 * MUST be provisioned to one common hashed timing set via $SET/$SAVE on
 * every board -- see docs/PLAN.md. Measured on hardware (2026-09-14, this
 * session): resp_pitch_us=1100 is a safe common floor for a 2-anchor mixed
 * network; the DWM1001's own deployed fleet runs resp_pitch_us=615 at
 * resp_count=8, which this port could not yet hold reliably (see
 * measurements/pitch_sweep_2026-09-14.json) until the SPI-primitive fix in
 * this radio port narrows that gap.
 */
#include "board_defaults.h"

static const board_defaults_t k_defaults = {
    .fw_ver          = "1.0.0-esp32",

    /* Network size. Spec section 8.1 defaults (init_count 4, resp_count 4).
     * A deployment resizes these via $SET/$SAVE; the shipped default must
     * match the specification so a freshly-wiped board is conformant. */
    .init_count      = 4,
    .resp_count      = 4,

    /* Timing: specification section 8.1 defaults, identical to the nRF52
     * port and to the spec's own table.
     *
     * NOTE these defaults do NOT fit this hardware and are not meant to be
     * run as-is. Measured on real boards: an anchor needs 321 us to arm its
     * Response, but resp_base_us 400 leaves only 400 - 181 (air Poll) =
     * 219 us. The specification anticipates exactly this -- section 7.2
     * says the measured column is filled from device-time captures and "all
     * later tables are derived from the measured values", and section 13
     * step 5 says to fill Table A by measurement then recompute Table B and
     * (14). So: ship the spec defaults, then run $MEASURE + $AUTOTIME and
     * $SAVE the derived set on every node before deployment.
     *
     * CAVEAT on $AUTOTIME, measured 2026-09-17 (2 ESP32 tags + 2 DWM1001
     * anchors, preamble_len=256): $AUTOTIME derived a LOOSER set (resp_base
     * 750, resp_pitch 1244, report_base 1239, report_pitch 1240, T_f 7218)
     * that performed materially WORSE than the hand-verified set below --
     * anchor Poll reception 74-59% vs 98-99%. $MEASURE's proc_* stats are
     * survivor-biased (only recorded after radio_tx_at() SUCCEEDS), so
     * $AUTOTIME inflates the slot budget when reception is already poor,
     * which lowers the rate without buying reliability. Treat $AUTOTIME as
     * a starting point and verify against measured reception, not as final.
     *
     * VERIFIED operating point for this 2-tag / 2-anchor mixed fleet
     * (measurements/rate_reception_2026-09-17.json), cfg_hash 0x3F27:
     *     init_count 2   resp_count 2   preamble_len 256
     *     frame_dur_us 6701  guard_us 300
     *     resp_base_us 700   resp_pitch_us 1100
     *     report_base_us 1100  report_pitch_us 1100
     *     rx_timeout_us 900
     * -> 74.8-76.7 Hz per tag per anchor, anchor Poll rx 98.1-99.2%,
     *    finals_rx 98.8-99.9%, tx_late 0, SD 2.0-7.7 cm, no SS-TWR outliers.
     *
     * Two traps found while establishing that point:
     *   - rx_timeout_us MUST stay well below resp_pitch_us. Raising it to
     *     1200 against resp_pitch 1244 let slot 0's RX window swallow slot
     *     1's Response and doubled resp_lost to 8.7%.
     *   - guard_us is non-monotonic here: 300 beat both 200 and 400/600.
     *     600 cost ~4 points of Poll reception AND ~5 Hz. */
    .frame_dur_us    = 4000,
    .resp_base_us    = 400,
    .resp_pitch_us   = 300,
    .report_base_us  = 400,
    .report_pitch_us = 290,
    .guard_us        = 200,
    .rate_div        = 1,

    .rx_timeout_us   = 900,
    .sync_mmax       = 10,
    /* ESP32 + Makerfabs SPI needs more headroom than nRF, but see radio.cpp:
     * measured worst case on hardware was proc_resp_max=623us, proc_rep_max
     * =865us (N_R=2) -- well under this 1500 budget. Left conservative here
     * pending a full $AUTOTIME re-run at N_R=8 with the SPI fixes applied. */
    .proc_us         = 1500,
    .isr_us          = 50,
    .margin_us       = 150,
    .proc_poll_us    = 1500,
    .proc_final_us   = 1500,

    .uart_baud       = 921600,
};

const board_defaults_t *board_defaults(void)
{
    return &k_defaults;
}
