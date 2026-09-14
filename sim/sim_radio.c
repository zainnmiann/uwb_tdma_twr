/**
 * @file    sim_radio.c
 * @brief   The parts of radio.h and the nRF drivers that sync.c and cfg.c
 *          reference, implemented against the virtual clock.
 *
 * Only the entry points the portable modules actually call are provided. The
 * frame schedule itself is driven by sim_main.c rather than by the blocking
 * role loops, so radio_rx()/radio_tx_at() are deliberately absent -- see the
 * scope note in sim_main.c.
 */
#include "sim.h"
#include "radio.h"

static const nclk_t *s_clk;
static int64_t       s_now;

void sim_bind_clock(const nclk_t *c, int64_t now_dtu)
{
    s_clk = c;
    s_now = now_dtu;
}

int64_t sim_global_from_hi32(const nclk_t *c, uint32_t hi32, int64_t after)
{
    int64_t  base    = sim_local_dtu(c, after);
    uint32_t base_hi = (uint32_t)((uint64_t)base >> 8);
    /* Signed hi32 distance, so the 32-bit wrap is handled without a magnitude
     * guess: alarms are always well inside +/-8.6 s. Align to the hi32
     * boundary rather than adding a delta to `base`, which would keep base's
     * own low 8 bits and inject a spurious 0..255 DTU offset -- mean 127 DTU,
     * 2.0 ns, which lands as ~0.6 m of range error once it reaches a TX
     * timestamp. */
    int32_t  d     = (int32_t)(hi32 - base_hi);
    int64_t  local = (base - (base & 0xFF)) + ((int64_t)d << 8);

    return sim_global_from_local(c, local);
}

uint32_t radio_systime_hi32(void)
{
    return (uint32_t)((uint64_t)sim_local_dtu(s_clk, s_now) >> 8);
}

int32_t radio_hi32_diff_us(uint32_t a, uint32_t b)
{
    /* Identical to the target implementation: one hi32 unit is 4.0064 ns. */
    int32_t d = (int32_t)(a - b);
    return (int32_t)(((int64_t)d * 1000LL) / 249600LL);
}

/* --- stub for the one entry point the portable modules still reference
 * from this file but never actually reach --------------------------------- */
bool uart_io_baud_supported(uint32_t baud)                           { (void)baud; return true; }
