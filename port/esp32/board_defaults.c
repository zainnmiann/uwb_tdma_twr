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

    /* Network size — ESP32 Pro deployment target: 16 tags x 8 anchors */
    .init_count      = 16,
    .resp_count      = 8,

    /* Timing. Table B ($AUTOTIME) output for proc_us=1500/isr_us=50/
     * margin_us=150, N_R=8, BEFORE the port_sys/radio SPI-cost fixes applied
     * in this repo (see radio.cpp's changelog comment). The previous
     * defaults here (800/500/800/500) were picked independently of
     * proc_us/isr_us/margin_us rather than derived via eq (12)/(13), and
     * understated the real budget by ~4x on this hardware -- confirmed on a
     * real board pair: resp_pitch_us=500 left the tag's Final arming at
     * 293/299 (98%) tx_late, and every range fell back to SS-TWR because the
     * anchor never once received a Final. Running $AUTOTIME fixed it
     * immediately -- tx_late dropped to 0/34 and DS-TWR (flags=0x00) became
     * the normal path. These are conservative pre-fix numbers, baked in so a
     * freshly wiped board works without the extra step; $MEASURE + $AUTOTIME
     * after the SPI fixes will recommend a smaller pitch. */
    .frame_dur_us    = 33047,
    .resp_base_us    = 1683,
    .resp_pitch_us   = 1895,
    .report_base_us  = 1749,
    .report_pitch_us = 1891,
    .guard_us        = 400,
    .rate_div        = 1,

    .rx_timeout_us   = 1200,
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
