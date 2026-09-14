#include "app.h"
#include "radio.h"

static stats_t m_stats;
static bool    m_ranging;
static volatile bool m_resync_req;

stats_t *app_stats(void)       { return &m_stats; }
void     app_stats_clear(void) { __builtin_memset(&m_stats, 0, sizeof(m_stats)); }
bool     app_ranging(void)     { return m_ranging; }
void     app_ranging_set(bool on) { m_ranging = on; }

/* $RANGE,START asks the role loop to re-acquire from scratch. Clearing a
 * latched SYNC_FAULT alone left cold_deadline_hi32 at whatever sync_start()
 * wrote earlier; the hi32 counter wraps every 17.2 s, so a stale deadline can
 * read as already expired and the node adopts a local epoch with no listen
 * window, transmitting straight into another initiator's slot. */
void     app_request_resync(void) { m_resync_req = true; }
bool     app_take_resync(void)
{
    bool r = m_resync_req;
    m_resync_req = false;
    return r;
}

void app_stat_proc(uint32_t *last, uint32_t *max, uint32_t start_hi32)
{
    int32_t d = radio_hi32_diff_us(radio_systime_hi32(), start_hi32);

    if (d < 0) { d = 0; }
    *last = (uint32_t)d;
    if ((uint32_t)d > *max) { *max = (uint32_t)d; }
}
