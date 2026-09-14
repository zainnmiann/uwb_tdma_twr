/**
 * @file    role_initiator.c
 * @brief   Section 10.1 -- the initiator state machine.
 *
 *   BOOT -> LOAD_CFG -> if(autostart) LISTEN else IDLE
 *   IDLE      : accept $RANGE,START -> LISTEN
 *   LISTEN    : RX enabled, duration 2 T_F
 *               on RX(Poll,m): rearm per eq (9)
 *               on timeout with no Poll: t_next = now (we are first)
 *   ARMED     : wait for the frame alarm, rearming on every overheard Poll
 *   TX_POLL   : delayed TX(Poll), store t_A_T,poll, arm the Response windows
 *   RX_RESP   : collect t_A_R,resp[s] and the payload timestamps, build the mask
 *   TX_FINAL  : delayed TX(Final) at t_A_R,resp[last] + delta_r  (eq 2)
 *   RX_REPORT : collect tau[s], emit $RNG
 */

#include "app.h"
#include "proto.h"
#include "radio.h"
#include "ranging.h"
#include "slots.h"
#include "timing.h"
#include "output.h"
#include "serial.h"

#include <string.h>
#include "port_sys.h"

/* How long before the scheduled Poll instant to stop listening and start
 * building/arming the delayed transmit. Measured empirically on nRF52832 @
 * 64 MHz + DW1000 SPI @ 8 MHz: sync bookkeeping, frame construction and the
 * dwt_setdelayedtrxtime/writetxdata/writetxfctrl/starttx SPI sequence together
 * cost noticeably more than the protocol spec's guard_us (typically 200 us,
 * sized for RX-side alarm drift, not TX-side prep) -- every attempt missed its
 * slot at that margin. This is deliberately generous; it costs receive time
 * only (an initiator alone still free-runs fine), never ranging accuracy. */
#define TX_PREP_FLOOR_US        200u

/* The margin is proc_poll_us, the settable key that $MEASURE reports and that
 * app.h already documents as "Directly comparable to TX_PREP_MARGIN_US". As a
 * file-local constant it could not be tuned at all: an operator following
 * $MEASURE's own advice changed a value that fed only eq (14) while the receive
 * window kept closing 1500 us early -- and that window is where sync_on_poll()
 * has to hear the other initiators, so the measurement round prescribed by
 * section 13 step 5 could never actually shrink it. */
static uint32_t tx_prep_margin_us(const cfg_t *c)
{
    return (c->proc_poll_us > TX_PREP_FLOOR_US) ? c->proc_poll_us
                                                : TX_PREP_FLOOR_US;
}

static uint8_t  m_buf[MAX_FRAME_LEN];
static uint8_t  m_seq;

/* Per-superframe collection. */
static uint64_t m_resp_rx[CFG_MAX_RESP];    /* t_A_R,resp,s              */
static uint64_t m_poll_rx[CFG_MAX_RESP];    /* t_B_R,poll from the Resp  */
static uint64_t m_resp_tx[CFG_MAX_RESP];    /* t_B_T,resp from the Resp  */
static uint16_t m_resp_src[CFG_MAX_RESP];   /* MHR src of the Resp     */
static int32_t  m_integ  [CFG_MAX_RESP];
static uint32_t m_tau_ps [CFG_MAX_RESP];
static int16_t  m_rxp    [CFG_MAX_RESP];
static int16_t  m_fpp    [CFG_MAX_RESP];
static uint16_t m_resp_mask;
static uint16_t m_rep_mask;
/* Section 11 row 7: a Poll was seen this cycle whose config hash did not match
 * ours. Raised on the cycle's $RNG lines as bit5, then cleared. */
static bool     m_hash_mismatch_seen;

static sync_t   m_sync;

/* A Poll overheard by the watch armed at the end of our own frame, held until
 * the main loop has advanced the alarm. It cannot be applied where it is
 * received: sync_on_poll() adopts the absolute instant eq (9) predicts, and
 * the caller runs sync_advance() straight after run_frame(), which would then
 * add a second superframe on top and walk the alarm a whole frame forward. */
static bool     m_peer_pending;
static uint32_t m_peer_rx_hi32;
static uint8_t  m_peer_m;
static uint16_t m_peer_sfn;
static uint16_t m_peer_src;

const sync_t *app_sync(void) { return &m_sync; }

/* ------------------------------------------------------------------------- */

static bool decode_poll(const radio_rx_t *rx, const cfg_t *c, uint8_t *m,
                        uint16_t *sfn, uint16_t *src)
{
    if (!proto_hdr_ok(rx->buf, rx->len, c->pan_id))          { return false; }
    if (MSG_TYPE(rx->buf[IDX_TYPEVER]) != MSG_POLL)          { return false; }
    if (rx->len < (POLL_LEN - FCS_LEN))                      { return false; }

    /* Section 11 "Hash mismatch | Poll hash != local | ignore the frame".
     * Section 8.2 rule 6 makes the hash the run-time enforcement of
     * network-wide parameter agreement, so it binds here too, not only on the
     * responder: an initiator that rearmed eq (9) off a node running a
     * different slot map would silently adopt a foreign schedule. */
    if (g_u16(rx->buf, POLL_IDX_CFG_HASH) != cfg_hash(c))
    {
        app_stats()->hash_mismatch++;
        m_hash_mismatch_seen = true;
        return false;
    }

    *m   = g_u8 (rx->buf, IDX_FRAME_IDX);
    *sfn = g_u16(rx->buf, IDX_SFN);
    *src = g_u16(rx->buf, MHR_IDX_SRC);
    return true;
}

static void collect_resp(const radio_rx_t *rx, const cfg_t *c)
{
    uint8_t s;

    if (!proto_hdr_ok(rx->buf, rx->len, c->pan_id))          { return; }
    if (MSG_TYPE(rx->buf[IDX_TYPEVER]) != MSG_RESP)          { return; }
    if (rx->len < (RESP_LEN - FCS_LEN))                      { return; }
    if (g_u16(rx->buf, MHR_IDX_DST) != c->node_id)           { return; }

    s = g_u8(rx->buf, RESP_IDX_SLOT);
    if (s >= c->resp_count)                                  { return; }

    m_resp_rx[s] = rx->rx_ts;
    /* The responder identifies itself in the MHR. Kept so a slot that is not
     * in the local anchor list is still reported by its real address instead
     * of 0x0000, which is indistinguishable from an unprovisioned node. */
    m_resp_src[s] = g_u16(rx->buf, MHR_IDX_SRC);
    m_poll_rx[s] = g_ts(rx->buf, RESP_IDX_POLL_RX);
    m_resp_tx[s] = g_ts(rx->buf, RESP_IDX_RESP_TX);
    m_integ  [s] = radio_carrier_integrator();
    m_resp_mask |= (uint16_t)(1u << s);

    {
        radio_quality_t q;
        radio_quality(&q);
        m_rxp[s] = q.rx_dbm10;
        m_fpp[s] = q.fp_dbm10;
    }

    app_stats()->resp_rx++;
}

static void collect_report(const radio_rx_t *rx, const cfg_t *c)
{
    uint8_t s;

    if (!proto_hdr_ok(rx->buf, rx->len, c->pan_id))          { return; }
    if (MSG_TYPE(rx->buf[IDX_TYPEVER]) != MSG_REPORT)        { return; }
    if (rx->len < (REPORT_LEN - FCS_LEN))                    { return; }
    if (g_u16(rx->buf, MHR_IDX_DST) != c->node_id)           { return; }

    s = g_u8(rx->buf, REP_IDX_SLOT);
    if (s >= c->resp_count)                                  { return; }

    {
        /* Bounded like ranging_tof_dtu()/ranging_tof_ss_s(). This was the one
         * path by which a tau reached the output without passing either: a
         * corrupt or cross-talking Report carrying 0xFFFFFFFF was emitted as a
         * clean 1287 km $RNG line with no flag set. Rejecting it leaves the
         * mask bit clear, so the slot degrades to the SS-TWR fallback. */
        uint32_t ps = g_u32(rx->buf, REP_IDX_TOF_PS);

        if (ps > RANGING_MAX_TOF_PS)
        {
            app_stats()->rejected++;
            return;
        }
        m_tau_ps[s] = ps;
    }
    m_rxp   [s] = BYTE_TO_PWR_DBM10(g_u8(rx->buf, REP_IDX_RXP));
    m_fpp   [s] = BYTE_TO_PWR_DBM10(g_u8(rx->buf, REP_IDX_FPP));
    m_rep_mask |= (uint16_t)(1u << s);

    app_stats()->reports_rx++;
}

/* ------------------------------------------------------------------------- */

static void emit_ranges(const cfg_t *c, uint64_t poll_tx, uint64_t final_tx)
{
    uint32_t s;

    for (s = 0; s < c->resp_count; s++)
    {
        const cfg_anchor_t *a = cfg_anchor_by_slot((uint8_t)s);
        rng_record_t        r;
        bool                have_resp = (m_resp_mask & (1u << s)) != 0u;
        bool                have_rep  = (m_rep_mask  & (1u << s)) != 0u;

        memset(&r, 0, sizeof(r));
        r.sfn        = (uint16_t)m_sync.sfn;
        r.frame_idx  = c->frame_index;
        r.tag_id     = c->node_id;
        r.anchor_id  = a ? a->id : (have_resp ? m_resp_src[s] : 0u);
        r.slot       = (uint8_t)s;
        /* Sampled per line: section 9.4's example advances t_local_us
         * between the two lines of one cycle. */
        r.t_local_us = out_local_us();

        if (sync_is_desynced(&m_sync))     { r.flags |= RNG_FLAG_DESYNC; }
        if (m_hash_mismatch_seen)         { r.flags |= RNG_FLAG_HASH_MISMATCH; }

        if (!have_resp)
        {
            /* Section 11 clears bit s of the mask and continues. Section 9.4
             * still defines a "Response lost" flag (bit2), and section 9.4's
             * "one line per initiator-responder pair" is read here as one line
             * per configured pair per cycle -- otherwise bit2 could never be
             * observed, since a slot that produced no Response also produces
             * no Report to decode. Distance and power are left zero. */
            app_stats()->resp_lost++;
            r.flags |= RNG_FLAG_RESP_LOST;
            out_range(&r);
            continue;
        }

        if (have_rep)
        {
            r.tof_ps    = m_tau_ps[s];
            r.rxp_dbm10 = m_rxp[s];
            r.fpp_dbm10 = m_fpp[s];
        }
        else
        {
            /* Section 11: Report lost -> fall back to eq (7), flag bit0.
             *
             * With report_enable = 0 there is no Report group at all, so this
             * is the configured steady state rather than a loss: flagging bit3
             * and counting reports_lost on every slot of every frame made a
             * real Report loss indistinguishable from the option being off and
             * left the loss statistics unusable. SS_FALLBACK still applies --
             * eq (7) is genuinely what produced this range. */
            double k    = ranging_k_hat(m_integ[s], c->channel, c->datarate);
            double tof  = ranging_tof_ss_s(poll_tx, m_resp_rx[s],
                                           m_poll_rx[s], m_resp_tx[s], k);
            if (c->report_enable)
            {
                app_stats()->reports_lost++;
                r.flags |= RNG_FLAG_REPORT_LOST;
            }
            app_stats()->ss_fallback++;
            r.flags |= RNG_FLAG_SS_FALLBACK;

            if (tof < 0.0)
            {
                app_stats()->rejected++;
                continue;
            }
            r.tof_ps = (uint32_t)(tof * 1.0e12);
            r.rxp_dbm10 = m_rxp[s];
            r.fpp_dbm10 = m_fpp[s];
        }

        /* Equation (6): d = c (tau - delta_A - delta_B_s). The DW1000 antenna
         * delay registers already carry the bulk of delta, so only the residual
         * per-responder trim from the anchor list is applied here. */
        r.d_mm = ranging_ps_to_mm(r.tof_ps, a ? (int32_t)a->ant : 0);

        if ((r.rxp_dbm10 - r.fpp_dbm10) > NLOS_THRESHOLD_DBM10)
        {
            r.flags |= RNG_FLAG_NLOS;
        }

        app_stats()->ranges++;
        out_range(&r);

        /* have_resp is necessarily true here -- the !have_resp branch above
         * continues -- so this is unconditional. final_rx is passed 0 because
         * the initiator never observes it; it lives only on the responder. */
        out_raw_ts((uint16_t)m_sync.sfn, r.anchor_id, poll_tx, m_poll_rx[s],
                   m_resp_tx[s], m_resp_rx[s], final_tx, 0);
    }

    /* Consumed for this cycle. Cleared here rather than at frame start because
     * it is raised during the preceding listen window. */
    m_hash_mismatch_seen = false;
}

/* ------------------------------------------------------------------------- */

/* Fold in whatever the end-of-frame watch heard. Applied after the alarm has
 * advanced, which is where the ARMED receive window's own Polls have always
 * landed, so eq (9) adopts the same instant either way. */
static void apply_peer_poll(const cfg_t *c)
{
    if (!m_peer_pending)
    {
        return;
    }
    m_peer_pending = false;

    app_stats()->polls_rx++;
    (void)sync_on_poll(&m_sync, c, m_peer_rx_hi32, m_peer_m, m_peer_sfn,
                       m_peer_src != c->node_id);
}

/* Open the window for the next initiator's Poll, then emit this frame's range
 * records while the DW1000 receives it. Arming first is the whole point: the
 * emit costs ~688 us for eight records and eq (14) never budgeted for it, so
 * doing it the other way round left the receiver armed inside the peer's
 * preamble and frame slot 0 never heard frame slot 1. */
static void watch_peer_and_emit(const cfg_t *c, uint32_t poll_hi32,
                                uint64_t poll_tx, uint64_t final_tx)
{
    radio_rx_t rx;
    bool       armed = false;
    int        r;

    /* Every initiator needs this, including the one in the last frame slot:
     * the next initiator's Poll is always exactly one frame after ours, and
     * for the last slot that simply wraps into the next superframe's slot 0.
     * Only a sole initiator has no peer to watch for, and arming there would
     * land on its own next Poll. */
    if (c->init_count > 1u)
    {
        armed = radio_rx_arm(slot_peer_watch_us(c), true,
                             slot_peer_watch_start_hi32(c, poll_hi32));
    }

    emit_ranges(c, poll_tx, final_tx);

    if (!armed)
    {
        return;
    }

    do
    {
        r = radio_rx_poll(&rx);
    }
    while (r == RADIO_RX_PENDING);

    if (r == RADIO_RX_FRAME)
    {
        uint8_t  m;
        uint16_t sfn, src;
        if (decode_poll(&rx, c, &m, &sfn, &src))
        {
            m_peer_rx_hi32 = (uint32_t)(rx.rx_ts >> 8);
            m_peer_m       = m;
            m_peer_sfn     = sfn;
            m_peer_src     = src;
            m_peer_pending = true;
        }
    }
    else if (r == RADIO_RX_ERROR)
    {
        app_stats()->rx_errors++;
    }

    radio_idle();
}

static void run_frame(const cfg_t *c)
{
    uint32_t prep_start_hi32 = radio_systime_hi32();
    uint32_t poll_hi32 = m_sync.t_next_hi32;
    uint64_t poll_tx, final_tx = 0;
    uint32_t final_hi32;
    uint32_t win_end;
    uint32_t s;
    radio_rx_t rx;

    m_resp_mask = 0;
    m_rep_mask  = 0;

    /* --- TX_POLL --------------------------------------------------------- */
    proto_hdr(m_buf, m_seq++, c->pan_id, ADDR_BROADCAST, c->node_id,
              MSG_POLL, (uint16_t)m_sync.sfn, c->frame_index);
    p_u16(m_buf, POLL_IDX_CFG_HASH, cfg_hash(c));

    if (radio_tx_at(m_buf, (uint16_t)POLL_LEN, poll_hi32) != RADIO_TX_OK)
    {
        app_stats()->tx_late++;
        /* No Poll went out, so there is no cycle to report -- but the flag is
         * raised during the listen window and would otherwise carry into the
         * next cycle's lines and misattribute the mismatch. */
        m_hash_mismatch_seen = false;
        return;
    }

    /* Section 7.2 device-time capture: the true TX_PREP_MARGIN_US requirement,
     * measured from the alarm decision to the Poll actually being armed. */
    app_stat_proc(&app_stats()->proc_poll_last_us,
                  &app_stats()->proc_poll_max_us, prep_start_hi32);
    radio_tx_wait();

    poll_tx = radio_tx_ts_from_hi32(poll_hi32);
    app_stats()->polls_tx++;
    app_stats()->frames++;

    /* --- RX_RESP: the Response group ------------------------------------- */
    /* One continuous receive across the group, re-armed after each frame.
     * Per-response delayed windows were measured to cost more SPI time than
     * T_proc allows at delta_r = 300 us. */
    win_end = slot_resp_window_end_hi32(c, poll_hi32);

    for (;;)
    {
        int32_t rem = radio_hi32_diff_us(win_end, radio_systime_hi32());
        int     r;

        if (rem <= 0) { break; }
        if (rem > 60000) { rem = 60000; }

        r = radio_rx(&rx, (uint32_t)rem, false, 0);
        if (r == RADIO_RX_FRAME)
        {
            collect_resp(&rx, c);
            if (m_resp_mask == (uint16_t)((1u << c->resp_count) - 1u))
            {
                break;              /* all received */
            }
        }
        else if (r == RADIO_RX_TIMEOUT)
        {
            app_stats()->timeouts++;
            break;
        }
        else
        {
            app_stats()->rx_errors++;
        }
    }

    if (m_resp_mask == 0u)
    {
        /* Section 11: no Response at all -> whole frame lost, flagged only
         * once the run reaches three consecutive frames. */
        app_stats()->whole_frame_lost++;
        app_stats()->whole_frame_lost_run++;
        if (app_stats()->whole_frame_lost_run >= 3u)
        {
            app_stats()->whole_frame_lost_flag = true;
        }
        radio_idle();
        /* Still emit the cycle. Returning silently made total loss -- the case
         * an operator most needs to see -- the only outcome that produces no
         * output at all, so a consumer could not tell it from a hung node. The
         * every-slot-lost path below stamps bit2 on each line. */
        emit_ranges(c, poll_tx, 0);
        return;
    }

    /* Any Response at all breaks the run. */
    app_stats()->whole_frame_lost_run = 0;

    /* --- TX_FINAL: equation (2) ------------------------------------------ */
    /* t_A_T,final = t_A_R,resp,N_R-1 + delta_r, taken from the last Response
     * actually received; if the last slot was lost we fall back to the nominal
     * instant so the responders that did answer still close their loop. */
    {
        int32_t last = -1;
        for (s = 0; s < c->resp_count; s++)
        {
            if (m_resp_mask & (1u << s)) { last = (int32_t)s; }
        }

        final_hi32 = slot_final_hi32(c, poll_hi32,
                                     (last >= 0)
                                         ? (uint32_t)(m_resp_rx[last] >> 8) : 0u,
                                     last);
    }

    proto_hdr(m_buf, m_seq++, c->pan_id, ADDR_BROADCAST, c->node_id,
              MSG_FINAL, (uint16_t)m_sync.sfn, c->frame_index);
    final_tx = radio_tx_ts_from_hi32(final_hi32);

    p_ts(m_buf, FIN_IDX_POLL_TX, poll_tx);
    for (s = 0; s < c->resp_count; s++)
    {
        p_ts(m_buf, FIN_IDX_RESP_RX(c->resp_count) + s * TS_LEN,
             (m_resp_mask & (1u << s)) ? m_resp_rx[s] : 0ULL);
    }
    p_ts(m_buf, FIN_IDX_FINAL_TX(c->resp_count), final_tx);
    p_mask(m_buf, FIN_IDX_RX_MASK(c->resp_count), c->resp_count, m_resp_mask);

    if (radio_tx_at(m_buf, (uint16_t)FINAL_LEN(c->resp_count), final_hi32) != RADIO_TX_OK)
    {
        app_stats()->tx_late++;
        radio_idle();
        emit_ranges(c, poll_tx, 0);
        return;
    }
    radio_tx_wait();
    app_stats()->finals_tx++;

    /* --- RX_REPORT: the Report group ------------------------------------- */
    if (c->report_enable)
    {
        win_end = slot_report_window_end_hi32(c, final_hi32);

        for (;;)
        {
            int32_t rem = radio_hi32_diff_us(win_end, radio_systime_hi32());
            int     r;

            if (rem <= 0) { break; }
            if (rem > 60000) { rem = 60000; }

            r = radio_rx(&rx, (uint32_t)rem, false, 0);
            if (r == RADIO_RX_FRAME)
            {
                collect_report(&rx, c);
                if (m_rep_mask == m_resp_mask) { break; }
            }
            else if (r == RADIO_RX_TIMEOUT)
            {
                app_stats()->timeouts++;
                break;
            }
            else
            {
                app_stats()->rx_errors++;
            }
        }
    }

    radio_idle();
    watch_peer_and_emit(c, poll_hi32, poll_tx, final_tx);
    port_led_toggle(PORT_LED_ACTIVITY);
}

/* ------------------------------------------------------------------------- */

void app_run_initiator(void)
{
    const cfg_t *c = cfg();

    sync_start(&m_sync, c);
    m_peer_pending = false;

    out_linef("I,initiator,node=0x%04X,frame=%u,N_I=%u,N_R=%u,Tf=%luus,TF=%luus",
              c->node_id, c->frame_index, c->init_count, c->resp_count,
              (unsigned long)c->frame_dur_us,
              (unsigned long)timing_superframe_us(c));

    bool was_ranging = app_ranging();

    for (;;)
    {
        radio_rx_t rx;
        int32_t    rem;
        int        r;

        serial_poll();

        /* Mirrored here rather than in the ARMED branch: a drift reset moves
         * the state machine into COLD, and the ARMED branch that used to carry
         * this assignment is then not reached, so $STAT under-reported the very
         * event it counts. */
        app_stats()->desync = m_sync.n_drift_reset;

        if (!app_ranging())
        {
            radio_idle();
            was_ranging = false;
            continue;
        }

        if (app_take_resync())
        {
            sync_start(&m_sync, c);
            m_peer_pending = false;
            was_ranging = true;
            continue;
        }

        if (!was_ranging)
        {
            /* Restarting after a stop: t_next_hi32 refers to an instant that
             * may be arbitrarily far in the past, and the DW1000 hi32 counter
             * wraps every 17.2 s, so the stale value can read as a plausible
             * near-future alarm and fire into another node's slot. Re-enter
             * COLD and re-acquire from overheard Polls per section 6. */
            was_ranging = true;
            sync_start(&m_sync, c);
            m_peer_pending = false;
            continue;
        }

        /* --- COLD: listen for 2 T_F ------------------------------------- */
        if (m_sync.state == SYNC_COLD)
        {
            rem = radio_hi32_diff_us(m_sync.cold_deadline_hi32, radio_systime_hi32());
            if (rem <= 0)
            {
                sync_adopt_local_epoch(&m_sync, c);
                out_line("I,cold start, no Poll heard, adopting local epoch");
                continue;
            }
            if (rem > 60000) { rem = 60000; }

            r = radio_rx(&rx, (uint32_t)rem, false, 0);
            if (r == RADIO_RX_FRAME)
            {
                uint8_t  m;
                uint16_t sfn, src;
                if (decode_poll(&rx, c, &m, &sfn, &src))
                {
                    app_stats()->polls_rx++;
                    (void)sync_on_poll(&m_sync, c, (uint32_t)(rx.rx_ts >> 8),
                                       m, sfn, src != c->node_id);
                }
            }
            continue;
        }

        if (m_sync.state == SYNC_FAULT)
        {
            app_stats()->dup_index = m_sync.n_dup;
            radio_idle();
            continue;                   /* transmission suppressed */
        }

        /* --- ARMED: wait for the alarm, resyncing on overheard Polls ----- */
        rem = radio_hi32_diff_us(m_sync.t_next_hi32, radio_systime_hi32());

        /* More than a whole superframe behind: the alarm is meaningless now,
         * so drop back to listening rather than firing into someone's slot. */
        if (rem < -(int32_t)timing_superframe_us(c))
        {
            sync_start(&m_sync, c);
            m_peer_pending = false;
            continue;
        }

        if (rem <= (int32_t)tx_prep_margin_us(c))
        {
            /* Section 6: a frame elapsed; sync_on_poll clears this again the
             * moment any initiator is heard. */
            sync_missed_frame(&m_sync, c);

            if (rem > -(int32_t)c->guard_us
                && sync_due_this_superframe(&m_sync, c)
                && sync_may_transmit(&m_sync))
            {
                run_frame(c);
            }
            sync_advance(&m_sync, c);
            apply_peer_poll(c);
            continue;
        }

        /* Close the receive window well before the alarm so there is time to
         * build the Poll and arm the delayed transmit (see tx_prep_margin_us). */
        rem -= (int32_t)tx_prep_margin_us(c);
        if (rem > 60000) { rem = 60000; }

        r = radio_rx(&rx, (uint32_t)rem, false, 0);
        if (r == RADIO_RX_FRAME)
        {
            uint8_t  m;
            uint16_t sfn, src;
            if (decode_poll(&rx, c, &m, &sfn, &src))
            {
                app_stats()->polls_rx++;
                (void)sync_on_poll(&m_sync, c, (uint32_t)(rx.rx_ts >> 8),
                                   m, sfn, src != c->node_id);
            }
        }
        else if (r == RADIO_RX_TIMEOUT)
        {
            /* no Poll from anyone in this window */
        }
        else
        {
            app_stats()->rx_errors++;
        }
    }
}
