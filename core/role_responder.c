/**
 * @file    role_responder.c
 * @brief   Section 10.2 -- the responder state machine.
 *
 *   RX_ON       : continuous reception
 *   RX(Poll,k)  : verify pan and config hash; t_B_R,poll = rx_ts
 *                 t_B_T,resp = t_B_R,poll + T_r0 + slot*delta_r        (eq 1)
 *                 delayed TX(Resp) carrying (t_B_R,poll, t_B_T,resp)
 *                 arm the RX window for the Final
 *   RX(Final,k) : t_B_R,final = rx_ts
 *                 tau = DS-TWR (eq 6), rejected outside the plausibility band
 *                 t_B_T,rep = t_B_R,final + T_p0 + slot*delta_p        (eq 3)
 *                 delayed TX(Report) carrying (tau, rxp, fpp, nstd)
 *   timeout     : drop the exchange, return to RX_ON
 *
 * Responders never keep a frame alarm and never transmit unsolicited. A
 * responder that misses a Poll stays silent for that frame, which leaves the
 * remaining slots untouched.
 */

#include "app.h"
#include "proto.h"
#include "radio.h"
#include "ranging.h"
#include "slots.h"
#include "timing.h"
#include "output.h"
#include "serial.h"
#include "port_sys.h"

#include <string.h>

#define RESP_RX_SLICE_US    60000u

/* One exchange in flight per initiator frame index.
 *
 * src/sfn are not decoration: the Final is broadcast, so hardware address
 * filtering cannot bind it to the Poll that armed this slot. Without matching
 * both, two initiators misconfigured onto the same frame_index silently
 * cross-contaminate -- A's Poll arms the slot, B's Final closes it, and the
 * Report is unicast to A carrying B's timestamps. armed_hi32 bounds how long
 * an exchange may stay open (section 10.2 "timeout : drop the exchange"). */
typedef struct
{
    bool     armed;
    uint16_t src;
    uint16_t sfn;
    uint32_t armed_hi32;    /* device time the Poll was answered */
    uint32_t deadline_hi32; /* by when this exchange's Final must arrive   */
    uint64_t poll_rx;       /* t_B_R,poll */
    uint64_t resp_tx;       /* t_B_T,resp */
} exchange_t;

static exchange_t m_ex[CFG_MAX_INIT];
/* The Final is sent delta_r after the *last* Response slot, so the wait is not
 * rx_timeout_us from our own Response -- a low-numbered slot has to sit through
 * every later slot first. rx_timeout_us is the tolerance around that expected
 * instant, which is what section 8.1's "frame wait timeout per expected
 * message" means here. The deadline is per exchange (see exchange_t): one
 * shared deadline would let each new Poll extend or destroy the pending
 * exchanges of other initiators. */
static uint8_t    m_buf[MAX_FRAME_LEN];
static uint8_t    m_seq;


/* Time until the soonest armed exchange's Final is due, in microseconds.
 * Returns false when nothing is waiting. */
static bool soonest_deadline_us(int32_t *out)
{
    uint32_t now = radio_systime_hi32();
    bool     any = false;
    int32_t  best = 0;
    uint32_t i;

    for (i = 0; i < CFG_MAX_INIT; i++)
    {
        int32_t rem;

        if (!m_ex[i].armed) { continue; }

        rem = radio_hi32_diff_us(m_ex[i].deadline_hi32, now);
        if (!any || rem < best) { best = rem; any = true; }
    }

    if (any) { *out = best; }
    return any;
}

/* Section 11 row 2: the Final never arrived, so no Report is emitted for that
 * frame and the slot is released. Only exchanges whose own deadline has passed
 * are reaped -- reaping all of them would discard initiators whose Finals were
 * still legitimately in flight. */
static void drop_expired_exchanges(void)
{
    uint32_t now = radio_systime_hi32();
    uint32_t i;

    for (i = 0; i < CFG_MAX_INIT; i++)
    {
        if (m_ex[i].armed
            && radio_hi32_diff_us(m_ex[i].deadline_hi32, now) <= 0)
        {
            m_ex[i].armed = false;
            app_stats()->finals_lost++;
        }
    }
}

/* ------------------------------------------------------------------------- */

static void handle_poll(const cfg_t *c, const radio_rx_t *rx)
{
    uint8_t  k;
    uint16_t sfn, src, hash;
    uint32_t resp_hi32;
    uint64_t resp_tx;

    if (rx->len < (POLL_LEN - FCS_LEN)) { return; }

    k    = g_u8 (rx->buf, IDX_FRAME_IDX);
    sfn  = g_u16(rx->buf, IDX_SFN);
    src  = g_u16(rx->buf, MHR_IDX_SRC);
    hash = g_u16(rx->buf, POLL_IDX_CFG_HASH);

    app_stats()->polls_rx++;

    /* Section 11: hash mismatch -> ignore the frame, flag bit5. The responder
     * has no output line of its own here, so it is counted and dropped. */
    if (hash != cfg_hash(c))
    {
        app_stats()->hash_mismatch++;
        return;
    }

    if (k >= CFG_MAX_INIT) { return; }

    /* Equation (1). */
    resp_hi32 = slot_resp_hi32(c, (uint32_t)(rx->rx_ts >> 8), c->slot_index);
    resp_tx   = radio_tx_ts_from_hi32(resp_hi32);

    proto_hdr(m_buf, m_seq++, c->pan_id, src, c->node_id,
              MSG_RESP, sfn, k);
    p_u8(m_buf, RESP_IDX_SLOT, c->slot_index);
    p_ts(m_buf, RESP_IDX_POLL_RX, rx->rx_ts);
    p_ts(m_buf, RESP_IDX_RESP_TX, resp_tx);

    if (radio_tx_at(m_buf, (uint16_t)RESP_LEN, resp_hi32) != RADIO_TX_OK)
    {
        app_stats()->tx_late++;
        m_ex[k].armed = false;
        return;
    }

    /* Section 7.2 device-time capture: this is the real T_r0 requirement. */
    app_stat_proc(&app_stats()->proc_resp_last_us,
                  &app_stats()->proc_resp_max_us,
                  (uint32_t)(rx->rx_ts >> 8));
    radio_tx_wait();

    app_stats()->resp_tx++;

    m_ex[k].deadline_hi32 =
        slot_exchange_deadline_hi32(c, (uint32_t)(rx->rx_ts >> 8));

    m_ex[k].armed      = true;
    m_ex[k].src        = src;
    m_ex[k].sfn        = sfn;
    m_ex[k].armed_hi32 = (uint32_t)(rx->rx_ts >> 8);
    m_ex[k].poll_rx    = rx->rx_ts;
    m_ex[k].resp_tx    = resp_tx;
}

static void handle_final(const cfg_t *c, const radio_rx_t *rx)
{
    uint8_t         k;
    uint16_t        mask;
    twr_ts_t        ts;
    int64_t         tau;
    uint32_t        rep_hi32;
    radio_quality_t q;
    uint32_t        need = FINAL_LEN(c->resp_count) - FCS_LEN;

    if (rx->len < need) { return; }

    k = g_u8(rx->buf, IDX_FRAME_IDX);
    if (k >= CFG_MAX_INIT || !m_ex[k].armed) { return; }

    /* The Final must belong to the exchange this slot actually armed. */
    if (g_u16(rx->buf, IDX_SFN) != m_ex[k].sfn
        || g_u16(rx->buf, MHR_IDX_SRC) != m_ex[k].src)
    {
        app_stats()->rejected++;
        return;
    }

    /* Section 10.2: an exchange that outlives its own frame is stale. */
    if (radio_hi32_diff_us(m_ex[k].armed_hi32, radio_systime_hi32())
        < -(int32_t)c->frame_dur_us)
    {
        m_ex[k].armed = false;
        app_stats()->finals_lost++;
        return;
    }

    app_stats()->finals_rx++;

    mask = g_mask(rx->buf, FIN_IDX_RX_MASK(c->resp_count), c->resp_count);
    if ((mask & (1u << c->slot_index)) == 0u)
    {
        /* The initiator did not hear our Response, so it carries no timestamp
         * for this slot. Nothing to close. */
        m_ex[k].armed = false;
        return;
    }

    ts.poll_tx  = g_ts(rx->buf, FIN_IDX_POLL_TX);
    ts.poll_rx  = m_ex[k].poll_rx;
    ts.resp_tx  = m_ex[k].resp_tx;
    ts.resp_rx  = g_ts(rx->buf, FIN_IDX_RESP_RX(c->resp_count)
                                 + (uint32_t)c->slot_index * TS_LEN);
    ts.final_tx = g_ts(rx->buf, FIN_IDX_FINAL_TX(c->resp_count));
    ts.final_rx = rx->rx_ts;

    m_ex[k].armed = false;

    tau = ranging_tof_dtu(&ts);
    if (tau < 0)
    {
        app_stats()->rejected++;
        return;
    }

    app_stats()->ranges++;

    if (!c->report_enable)
    {
        radio_quality(&q);
        const cfg_anchor_t *a = cfg_anchor_by_slot(c->slot_index);
        int32_t d_mm = ranging_ps_to_mm(ranging_dtu_to_ps(tau), a ? (int32_t)a->ant : 0);
        port_ui_range(c->slot_index, c->node_id, d_mm, q.rx_dbm10, true);
        return;                     /* Report group removed, section 8.1 */
    }

    /* Link quality of the Final, i.e. the initiator -> responder direction. */
    radio_quality(&q);

    /* Equation (3). */
    rep_hi32 = slot_report_hi32(c, (uint32_t)(rx->rx_ts >> 8), c->slot_index);

    proto_hdr(m_buf, m_seq++, c->pan_id, m_ex[k].src, c->node_id,
              MSG_REPORT, m_ex[k].sfn, k);
    p_u8 (m_buf, REP_IDX_SLOT,   c->slot_index);
    p_u32(m_buf, REP_IDX_TOF_PS, ranging_dtu_to_ps(tau));
    p_u8 (m_buf, REP_IDX_RXP,    PWR_TO_BYTE(q.rx_dbm10));
    p_u8 (m_buf, REP_IDX_FPP,    PWR_TO_BYTE(q.fp_dbm10));
    p_u8 (m_buf, REP_IDX_NSTD,   (uint8_t)(q.std_noise >> 4));

    if (radio_tx_at(m_buf, (uint16_t)REPORT_LEN, rep_hi32) != RADIO_TX_OK)
    {
        app_stats()->tx_late++;
        return;
    }

    /* Section 7.2 device-time capture: this is the real T_p0 requirement. */
    app_stat_proc(&app_stats()->proc_rep_last_us,
                  &app_stats()->proc_rep_max_us,
                  (uint32_t)(rx->rx_ts >> 8));
    radio_tx_wait();
    app_stats()->reports_tx++;

    {
        const cfg_anchor_t *a = cfg_anchor_by_slot(c->slot_index);
        int32_t d_mm = ranging_ps_to_mm(ranging_dtu_to_ps(tau), a ? (int32_t)a->ant : 0);
        port_ui_range(c->slot_index, c->node_id, d_mm, q.rx_dbm10, true);
    }

    port_led_toggle(PORT_LED_ACTIVITY);
}

static void handle_frame(const cfg_t *c, const radio_rx_t *rx)
{
    uint16_t dst;

    if (!proto_hdr_ok(rx->buf, rx->len, c->pan_id)) { return; }

    dst = g_u16(rx->buf, MHR_IDX_DST);
    if (dst != ADDR_BROADCAST && dst != c->node_id) { return; }

    switch (MSG_TYPE(rx->buf[IDX_TYPEVER]))
    {
        case MSG_POLL:  handle_poll(c, rx);  break;
        case MSG_FINAL: handle_final(c, rx); break;
        default:                             break;
    }
}

/* ------------------------------------------------------------------------- */

void app_run_responder(void)
{
    const cfg_t *c = cfg();
    radio_rx_t   rx;

    memset(m_ex, 0, sizeof(m_ex));

    out_linef("I,responder,node=0x%04X,slot=%u,N_R=%u,Tr0=%luus,dr=%luus,"
              "Tp0=%luus,dp=%luus",
              c->node_id, c->slot_index, c->resp_count,
              (unsigned long)c->resp_base_us, (unsigned long)c->resp_pitch_us,
              (unsigned long)c->report_base_us, (unsigned long)c->report_pitch_us);

    for (;;)
    {
        int      r;
        uint32_t slice;

        serial_poll();

        if (!app_ranging())
        {
            radio_idle();
            continue;
        }

        /* Section 10.2 "arm RX window for Final": while an exchange is open,
         * bound the wait with rx_timeout_us ("frame wait timeout per expected
         * message", section 8.1) instead of the idle slice, so a lost Final is
         * detected promptly and its exchange dropped rather than lingering. */
        slice = RESP_RX_SLICE_US;

        {
            int32_t rem;

            if (soonest_deadline_us(&rem))
            {
                if (rem <= 0)
                {
                    /* Section 11 row 2 / section 10.2 timeout. */
                    drop_expired_exchanges();
                }
                else
                {
                    slice = ((uint32_t)rem < RESP_RX_SLICE_US) ? (uint32_t)rem
                                                              : RESP_RX_SLICE_US;
                }
            }
        }

        r = radio_rx(&rx, slice, false, 0);
        if (r == RADIO_RX_FRAME)
        {
            handle_frame(c, &rx);
        }
        else if (r == RADIO_RX_TIMEOUT)
        {
            /* Not necessarily the Final's deadline -- the slice is also capped
             * at RESP_RX_SLICE_US. The deadline test at the top of the loop is
             * what expires an exchange. */
            app_stats()->timeouts++;
        }
        else
        {
            app_stats()->rx_errors++;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Listener: passive gateway. apply_cfg() turns the DW1000 frame filter off for
 * this role (see radio.c), so it sees the Response and Report frames that are
 * unicast to the initiators and can reconstruct the ranging stream without
 * taking part in it.                                                          */

void app_run_listener(void)
{
    const cfg_t *c = cfg();
    radio_rx_t   rx;

    out_linef("I,listener,node=0x%04X,pan=0x%04X", c->node_id, c->pan_id);

    for (;;)
    {
        serial_poll();

        if (!app_ranging())
        {
            radio_idle();
            continue;
        }

        if (radio_rx(&rx, RESP_RX_SLICE_US, false, 0) != RADIO_RX_FRAME)
        {
            continue;
        }
        if (!proto_hdr_ok(rx.buf, rx.len, c->pan_id))
        {
            continue;
        }

        switch (MSG_TYPE(rx.buf[IDX_TYPEVER]))
        {
            case MSG_POLL:
                app_stats()->polls_rx++;
                break;

            case MSG_RESP:
                app_stats()->resp_rx++;
                break;

            case MSG_FINAL:
                app_stats()->finals_rx++;
                break;

            case MSG_REPORT:
            {
                rng_record_t rr;
                const cfg_anchor_t *a;

                if (rx.len < (REPORT_LEN - FCS_LEN)) { break; }

                app_stats()->reports_rx++;

                memset(&rr, 0, sizeof(rr));
                rr.sfn        = g_u16(rx.buf, IDX_SFN);
                rr.frame_idx  = g_u8 (rx.buf, IDX_FRAME_IDX);
                rr.tag_id     = g_u16(rx.buf, MHR_IDX_DST);
                rr.anchor_id  = g_u16(rx.buf, MHR_IDX_SRC);
                rr.slot       = g_u8 (rx.buf, REP_IDX_SLOT);
                rr.tof_ps     = g_u32(rx.buf, REP_IDX_TOF_PS);
                rr.rxp_dbm10  = BYTE_TO_PWR_DBM10(g_u8(rx.buf, REP_IDX_RXP));
                rr.fpp_dbm10  = BYTE_TO_PWR_DBM10(g_u8(rx.buf, REP_IDX_FPP));
                rr.t_local_us = out_local_us();

                a = cfg_anchor_by_id(rr.anchor_id);
                rr.d_mm = ranging_ps_to_mm(rr.tof_ps, a ? (int32_t)a->ant : 0);

                if ((rr.rxp_dbm10 - rr.fpp_dbm10) > NLOS_THRESHOLD_DBM10)
                {
                    rr.flags |= RNG_FLAG_NLOS;
                }

                app_stats()->ranges++;
                out_range(&rr);
                break;
            }

            default:
                break;
        }
    }
}
