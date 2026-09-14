/**
 * @file    board_defaults.c
 * @brief   DWM1001-DEV factory defaults -- see board_defaults.h for the
 *          hashed/not-hashed split. These are the values a freshly-flashed,
 *          unprovisioned board boots with; a deployed fleet is provisioned
 *          on top via $SET/$SAVE (see host/provision.py in the reference
 *          tree, which measured 313us/682us to arm a Response/Report on
 *          this hardware and runs resp_pitch_us=615 in production).
 */
#include "board_defaults.h"

static const board_defaults_t k_defaults = {
    .fw_ver          = "1.0.0",

    /* Network size */
    .init_count      = 4,
    .resp_count      = 4,

    /* Timing. NOT the section 7.4 worked-example values (60/15/25/60/60).
     * Those are a budget, not a measurement -- on real nRF52832 @ 64 MHz +
     * DW1000 SPI @ 8 MHz hardware they caused the Poll and Response
     * transmissions to miss their scheduled instant essentially every time
     * (measured: 100% tx_late at the spec defaults). These are the values
     * that were hardware-verified to work end-to-end. Per section 13 step 5,
     * a logic-analyser measurement of the true minimum would very likely
     * allow these to shrink and the ranging rate to rise. */
    .frame_dur_us    = 4000,
    .resp_base_us    = 400,
    .resp_pitch_us   = 300,
    .report_base_us  = 400,
    .report_pitch_us = 290,
    .guard_us        = 200,
    .rate_div        = 1,

    .rx_timeout_us   = 900,
    .sync_mmax       = 10,
    .proc_us         = 1000,
    .isr_us          = 15,
    .margin_us       = 100,
    .proc_poll_us    = 1000,
    .proc_final_us   = 1000,

    .uart_baud       = 921600,
};

const board_defaults_t *board_defaults(void)
{
    return &k_defaults;
}
