/**
 * @file    node_radio.c
 * @brief   radio.h implemented in the child, over the harness IPC.
 *
 * This is the only firmware module the harness replaces. role_initiator.c and
 * role_responder.c are linked unmodified and cannot tell the difference: every
 * blocking radio call becomes one request/reply round trip with the parent,
 * which owns virtual time.
 *
 * Local time is whatever the parent last told us. That is exactly right: a
 * node's own compute does not advance the shared timeline, so between two
 * round trips radio_systime_hi32() must be constant. The parent charges the
 * processing cost explicitly (see --proc-us) rather than letting host CPU time
 * leak into the model.
 */
#include "ipc.h"

#include "radio.h"
#include "timing.h"

#include <string.h>
#include <stdlib.h>

static int64_t          s_now_dtu;      /* this node's local time */
static int32_t          s_carrier;
static radio_quality_t  s_quality = { -800, -880, 40, 1024 };
static uint32_t         s_part_id;

void node_radio_set_part_id(uint32_t id) { s_part_id = id; }

uint32_t node_uart_take_bytes(void);

static void trip(const ipc_req_t *rq, ipc_rep_t *rp)
{
    ipc_req_t q = *rq;

    /* Hand over the UART bytes written since we last spoke so the parent can
     * bill the serialisation time against this node's clock. */
    q.uart_bytes = node_uart_take_bytes();

    ipc_round_trip(&q, rp);
    s_now_dtu = rp->now_dtu;
}

/* --- time ----------------------------------------------------------------- */
uint32_t radio_systime_hi32(void)
{
    return (uint32_t)((uint64_t)s_now_dtu >> 8);
}

int32_t radio_hi32_diff_us(uint32_t a, uint32_t b)
{
    /* Identical to the target implementation: one hi32 unit is 4.0064 ns. */
    int32_t d = (int32_t)(a - b);

    return (int32_t)(((int64_t)d * 1000LL) / 249600LL);
}

uint64_t radio_tx_ts_from_hi32(uint32_t tx_hi32)
{
    /* Bit 0 masked off, exactly as radio.c does: dwt_setdelayedtrxtime()
     * writes DX_TIME at offset 1 because the low 9 bits are ignored. Antenna
     * delay is zero in the harness, so a recovered tau is the true flight
     * time up to device-time quantization. */
    return (((uint64_t)(tx_hi32 & 0xFFFFFFFEu)) << 8) & DTU_MASK_40;
}

/* --- transmit ------------------------------------------------------------- */
int radio_tx_at(const uint8_t *buf, uint16_t len, uint32_t tx_hi32)
{
    ipc_req_t rq;
    ipc_rep_t rp;

    if (len > IPC_MAX_FRAME) { return RADIO_TX_LATE; }

    memset(&rq, 0, sizeof(rq));
    rq.kind = REQ_TX;
    rq.hi32 = tx_hi32;
    rq.len  = len;
    memcpy(rq.buf, buf, len);

    trip(&rq, &rp);
    return (rp.kind == REP_TX_OK) ? RADIO_TX_OK : RADIO_TX_LATE;
}

void radio_tx_wait(void)
{
    ipc_req_t rq;
    ipc_rep_t rp;

    memset(&rq, 0, sizeof(rq));
    rq.kind = REQ_TXWAIT;
    trip(&rq, &rp);
}

void radio_idle(void)
{
    ipc_req_t rq;
    ipc_rep_t rp;

    memset(&rq, 0, sizeof(rq));
    rq.kind = REQ_IDLE;
    trip(&rq, &rp);
}

/* --- receive -------------------------------------------------------------- */

/* Lift a 32-bit hi32 onto the running 40-bit timeline, nearest wrap to now.
 *
 * The low 8 bits belong to the dtu below hi32's resolution and must come from
 * the hi32 value itself, shifted -- not from now_dtu. Carrying now's low bits
 * across leaves up to 255 dtu (~2.0 ns, 0.6 m of range) of error, which is
 * exactly the bug that made an earlier harness disagree with the target. */
static int64_t lift_hi32(uint32_t hi32)
{
    int64_t  base = s_now_dtu >> 8;
    int64_t  out  = (int64_t)hi32;
    int64_t  span = 1LL << 32;

    while (out - base >  (span / 2)) { out -= span; }
    while (base - out >  (span / 2)) { out += span; }

    return out << 8;
}

/* An armed-but-not-collected receiver. The harness has no concurrency, so the
 * window is remembered here and the round trip happens in radio_rx_poll();
 * the parent still sees the same absolute interval either way, which is what
 * decides whether the frame is inside it. */
static bool    s_armed;
static int64_t s_arm_from_dtu;
static int64_t s_arm_until_dtu;

bool radio_rx_arm(uint32_t timeout_us, bool delayed, uint32_t start_hi32)
{
    /* Zero means "open when the parent services the request". An immediate
     * arm must not carry this child's own timestamp: that reading is stale by
     * whatever the parent is about to charge for UART output, and sending it
     * would hand back exactly the time the emit was supposed to cost. */
    int64_t from = delayed ? lift_hi32(start_hi32) : 0;

    /* radio.c returns false when a delayed start has already passed, because
     * dwt_rxenable() rejects it -- the harness must refuse it identically or
     * the caller's fallback path never gets exercised. */
    if (delayed && from < s_now_dtu)
    {
        return false;               /* dwt_rxenable() would reject it too */
    }

    s_armed         = true;
    s_arm_from_dtu  = from;
    s_arm_until_dtu = (delayed ? from : s_now_dtu)
                    + (int64_t)us_to_dtu(timeout_us ? timeout_us : 1u);
    return true;
}

int radio_rx_poll(radio_rx_t *rx)
{
    ipc_req_t rq;
    ipc_rep_t rp;

    if (!s_armed)
    {
        return RADIO_RX_TIMEOUT;
    }
    s_armed = false;

    memset(&rq, 0, sizeof(rq));
    rq.kind      = REQ_RX;
    rq.until_dtu = s_arm_until_dtu;
    rq.from_dtu  = s_arm_from_dtu;

    trip(&rq, &rp);

    if (rp.kind != REP_FRAME) { return RADIO_RX_TIMEOUT; }

    /* A frame that landed before the window opened was not received by a
     * delayed arm, however convenient the parent's bookkeeping makes it. */
    if ((int64_t)rp.rx_ts < s_arm_from_dtu)
    {
        return RADIO_RX_TIMEOUT;
    }

    memset(rx, 0, sizeof(*rx));
    rx->len   = rp.len;
    rx->rx_ts = rp.rx_ts;
    memcpy(rx->buf, rp.buf, rp.len);

    s_carrier          = rp.carrier_int;
    s_quality.rx_dbm10 = rp.rx_dbm10;
    s_quality.fp_dbm10 = rp.fp_dbm10;

    return RADIO_RX_FRAME;
}

int radio_rx(radio_rx_t *rx, uint32_t timeout_us, bool delayed, uint32_t start_hi32)
{
    if (!radio_rx_arm(timeout_us, delayed, start_hi32))
    {
        return RADIO_RX_TIMEOUT;
    }
    return radio_rx_poll(rx);
}

int32_t radio_carrier_integrator(void) { return s_carrier; }

void radio_quality(radio_quality_t *q) { *q = s_quality; }

/* --- the rest of the surface, stubbed ------------------------------------- */
bool     radio_init(const cfg_t *c)        { (void)c; return true; }
bool     radio_reconfigure(const cfg_t *c) { (void)c; return true; }
uint32_t radio_txpower_reg(void)           { return 0x0E082848u; }
bool     radio_txpower_from_otp(void)      { return false; }
uint16_t radio_ant_tx(void)                { return 0u; }
uint16_t radio_ant_rx(void)                { return 0u; }
uint32_t radio_part_id(void)               { return s_part_id; }
uint32_t radio_device_id(void)             { return 0xDECA0130u; }
