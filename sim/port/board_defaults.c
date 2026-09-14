/* Host stub. Test scenarios in sim_main.c call cfg_defaults() then
 * overwrite the fields they care about (resp_count, etc.) directly, so
 * these values are just a reasonable starting point, not load-bearing. */
#include "board_defaults.h"

static const board_defaults_t k_defaults = {
    .fw_ver          = "sim",
    .init_count      = 4,
    .resp_count      = 4,
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

const board_defaults_t *board_defaults(void) { return &k_defaults; }
