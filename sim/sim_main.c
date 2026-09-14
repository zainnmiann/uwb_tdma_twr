/**
 * @file    sim_main.c
 * @brief   Host simulator: 4 initiators x 12 responders, no hardware.
 *
 * WHAT THIS TESTS
 *   The distributed schedule and the estimators, using the firmware's own
 *   code: sync.c (section 6, equations 8 and 9), timing.c (the section 7 air
 *   model and equations 11-14), slots.c (the fixed-slot instants of
 *   equations 1, 2 and 3 plus the receive windows and the exchange deadline),
 *   ranging.c (the section 4 estimators) and cfg.c (section 8 validation) are
 *   compiled unmodified and linked in.
 *
 *   Every transmit instant is derived through the real equations from
 *   timestamps taken on per-node clocks that each have their own crystal
 *   error, a slow thermal drift and a phase offset, so equation (9) and the
 *   estimators do real work rather than agree trivially. Transmit instants
 *   carry the hi32 quantization and the bit-0 masking the DW1000 applies to
 *   DX_TIME, and the timestamps handed to the estimators are masked to the
 *   40-bit counter exactly as they would be on the wire.
 *
 *   Two scenarios run. IDEAL has static clocks and no loss, and asserts exact
 *   equality of every count -- it is a test of the schedule arithmetic.
 *   REALISTIC adds thermal drift and random frame loss, and asserts that no
 *   responder is starved, that the rate holds, and that the schedule still
 *   never collides.
 *
 * WHAT THIS DOES NOT TEST
 *   role_initiator.c and role_responder.c are blocking run-to-completion
 *   loops driven by radio_rx(), and their per-node state is file-scope static,
 *   so 4 initiator instances cannot coexist in one process; they are not
 *   linked. What remains out of scope is therefore the polling loop itself --
 *   the SYS_STATUS waits, the RX re-arm sequence, the Response/Report mask
 *   assembly and the serial surface. The arithmetic those loops used to hold
 *   inline now lives in slots.c and IS exercised here.
 *   Antenna delay is zero on every node, so a recovered tau equals the true
 *   flight time up to device-time quantization.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sim.h"
#include "cfg.h"
#include "sync.h"
#include "timing.h"
#include "ranging.h"
#include "proto.h"
#include "slots.h"
#include "sim_trace.h"

#define N_TAGS          4u
#define N_ANCH          12u
#define SUPERFRAMES     2000u

#define DTU_PER_NS      63.8976
/* One device-time tick expressed as range: c * 15.650040 ps = 4.69 mm. */
#define SIM_QUANT_MM    4.69

static int64_t ns_to_dtu_g(uint32_t ns)
{
    return (int64_t)llround((double)ns * DTU_PER_NS);
}

/* --- deterministic PRNG, so a failure is always reproducible -------------- */
static uint64_t s_rng = 0x2545F4914F6CDD1DULL;

static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13; s_rng ^= s_rng >> 7; s_rng ^= s_rng << 17;
    return (uint32_t)(s_rng >> 32);
}

static bool lost(uint32_t pct_x100)
{
    return pct_x100 && ((rnd() % 10000u) < pct_x100);
}

/* --- nodes ---------------------------------------------------------------- */
typedef struct
{
    cfg_t    cfg;
    sync_t   sync;
    nclk_t   clk;
    double   x, y;
    int64_t  alarm_g;
    bool     has_alarm;
    /* The cold deadline is an absolute global instant, latched when the node
     * enters COLD. Re-deriving it from cold_deadline_hi32 on every iteration
     * is wrong once it lies in the past: the hi32 view wraps every 17.2 s, so
     * "the next time the counter reads X" jumps a whole wrap into the future
     * and the node sits deaf instead of adopting a local epoch. */
    int64_t  cold_g;
    bool     in_cold;
    uint32_t polls, frames_lost;
    uint32_t ranges[N_ANCH];
    double   err_max_mm[N_ANCH];
    double   err_sum_mm[N_ANCH];
    int64_t  first_poll_g, last_poll_g, prev_poll_g;
    int64_t  gap_min, gap_max;
} tag_t;

typedef struct
{
    cfg_t    cfg;
    nclk_t   clk;
    double   x, y;
    uint32_t resp_tx, rep_tx;
} anch_t;

static tag_t  T[N_TAGS];
static anch_t A[N_ANCH];
static uint32_t g_loss_x100;
static uint32_t g_trace;
static const char *g_trace_base;      /* --trace= prefix, or NULL */
static uint32_t    g_trace_win = 20u;
static bool        g_live;
static double      g_realtime;
static uint32_t g_trace2;

/* --- transmission log, for the slot-overlap check ------------------------- */
typedef struct { int64_t s, e; int who; const char *kind; } txr_t;

static txr_t  *g_tx;
static size_t  g_tx_n, g_tx_cap;

static void tx_log(int64_t s, int64_t e, int who, const char *kind);

/* Kind string -> the trace's enum, so one call site feeds both. */
static tkind_t tk_of(const char *kind)
{
    if (!strcmp(kind, "Poll"))  { return TK_POLL; }
    if (!strcmp(kind, "Resp"))  { return TK_RESP; }
    if (!strcmp(kind, "Final")) { return TK_FINAL; }
    return TK_REPORT;
}

static void tx_log(int64_t s, int64_t e, int who, const char *kind)
{
    if (g_tx_n == g_tx_cap)
    {
        g_tx_cap = g_tx_cap ? g_tx_cap * 2u : 1u << 16;
        g_tx = realloc(g_tx, g_tx_cap * sizeof(*g_tx));
        if (!g_tx) { fprintf(stderr, "oom\n"); exit(2); }
    }
    g_tx[g_tx_n].s = s; g_tx[g_tx_n].e = e;
    g_tx[g_tx_n].who = who; g_tx[g_tx_n].kind = kind;
    g_tx_n++;

    if (trace_active())
    {
        uint32_t node = (who >= 100) ? (uint32_t)(N_TAGS + (who - 100))
                                     : (uint32_t)who;
        uint32_t slot = (who >= 100) ? (uint32_t)(who - 100) : 0u;

        trace_tx(s, e, node, tk_of(kind), slot, 0u);
    }
}

static int tx_cmp(const void *a, const void *b)
{
    const txr_t *x = a, *y = b;
    return (x->s < y->s) ? -1 : (x->s > y->s) ? 1 : 0;
}

/* Latch the cold deadline on entry to COLD; clear the latch on the way out. */
static void note_state(tag_t *t, int64_t now)
{
    if (t->sync.state == SYNC_COLD)
    {
        if (!t->in_cold)
        {
            sim_bind_clock(&t->clk, now);
            t->cold_g  = sim_global_from_hi32(&t->clk,
                                              t->sync.cold_deadline_hi32, now);
            t->in_cold = true;
        }
    }
    else
    {
        t->in_cold = false;
    }
}

static double dist_mm(double ax, double ay, double bx, double by)
{
    return sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

/* A delayed transmit: the firmware adds a hi32 offset to the hi32 view of a
 * received timestamp, and the DW1000 then ignores the low 9 bits of DX_TIME so
 * the launch lands on an even hi32 boundary. Kept on the FULL-WIDTH local
 * timeline -- folding it through the 40-bit mask first loses the epoch, since
 * the counter wraps every 17.2 s. The mask belongs only on wire values, where
 * the estimator's own sub40() resolves the wrap. */
/* Lift a firmware slot function's result back onto the unbounded timeline.
 *
 * The slot_* functions take and return 32-bit device time, which wraps every
 * 17.2 s; the simulator needs the absolute instant. They are called with the
 * genuine low 32 bits of the reference -- so the wraparound arithmetic inside
 * them is exercised for real -- and the difference from that reference is a
 * small positive hi32 delta that lifts cleanly.
 *
 * The DW1000 then ignores the low 9 bits of DX_TIME, so the launch lands on an
 * even hi32 boundary, exactly as radio_tx_ts_from_hi32() reports it. */
static int64_t launch_from(int64_t ref_full, uint32_t out_hi32)
{
    uint32_t ref32 = (uint32_t)(ref_full >> 8);
    int64_t  delta = (int64_t)(uint32_t)(out_hi32 - ref32);
    int64_t  hi    = (ref_full >> 8) + delta;

    return (hi & ~1LL) << 8;
}

#define WIRE(x)     ((uint64_t)(x) & DTU_MASK_40)

/* ------------------------------------------------------------------------- */
static void run_frame(unsigned ti, int64_t poll_g, uint32_t sfn)
{
    tag_t *t = &T[ti];
    const cfg_t *c = &t->cfg;
    uint32_t nr = c->resp_count;

    int64_t  poll_tx_full = sim_local_dtu(&t->clk, poll_g);
    uint64_t poll_tx_w    = WIRE(poll_tx_full);
    int64_t  resp_g[N_ANCH], prop[N_ANCH];
    int64_t  poll_rx_full[N_ANCH], resp_tx_full[N_ANCH], resp_rx_full[N_ANCH];
    uint64_t poll_rx_w[N_ANCH], resp_tx_w[N_ANCH], resp_rx_w[N_ANCH];
    bool     got_resp[N_ANCH];
    int64_t  final_g, final_tx_full;
    uint64_t final_tx_w;
    int32_t  last = -1;
    uint32_t s;

    tx_log(poll_g, poll_g + ns_to_dtu_g(timing_air_poll_ns(c)), (int)ti, "Poll");
    t->polls++;
    if (t->first_poll_g == 0) { t->first_poll_g = poll_g; }
    if (t->prev_poll_g != 0)
    {
        int64_t gap = poll_g - t->prev_poll_g;

        if (t->gap_min == 0 || gap < t->gap_min) { t->gap_min = gap; }
        if (gap > t->gap_max)                    { t->gap_max = gap; }
    }
    t->prev_poll_g = poll_g;
    t->last_poll_g = poll_g;

    /* --- Response group: equation (1), on each responder's own clock ------ */
    for (s = 0; s < nr; s++)
    {
        prop[s]      = sim_prop_dtu(dist_mm(t->x, t->y, A[s].x, A[s].y));
        got_resp[s]  = false;

        if (lost(g_loss_x100)) { continue; }        /* Poll not heard here */

        poll_rx_full[s] = sim_local_dtu(&A[s].clk, poll_g + prop[s]);
        poll_rx_w[s]    = WIRE(poll_rx_full[s]);

        /* Equation (1), evaluated by the firmware's own slot_resp_hi32(). */
        resp_tx_full[s] = launch_from(
            poll_rx_full[s],
            slot_resp_hi32(c, (uint32_t)(poll_rx_full[s] >> 8), (uint8_t)s));
        resp_tx_w[s]    = WIRE(resp_tx_full[s]);
        resp_g[s]       = sim_global_from_local(&A[s].clk, resp_tx_full[s]);

        tx_log(resp_g[s], resp_g[s] + ns_to_dtu_g(timing_air_resp_ns(c)),
               100 + (int)s, "Resp");
        A[s].resp_tx++;

        if (lost(g_loss_x100)) { continue; }        /* Response not received */

        resp_rx_full[s] = sim_local_dtu(&t->clk, resp_g[s] + prop[s]);
        resp_rx_w[s]    = WIRE(resp_rx_full[s]);
        got_resp[s]     = true;
        last            = (int32_t)s;
    }

    if (last < 0)
    {
        t->frames_lost++;               /* section 11: whole frame lost */
        trace_frame_lost(poll_g, ti, sfn);
        return;
    }

    /* --- Final: equation (2), from the last Response actually received ----
     *
     * role_initiator.c only applies equation (2) when the LAST slot was the
     * one received; otherwise it falls back to the nominal instant. That
     * branch is load-bearing, not defensive: a responder whose Response the
     * initiator missed still transmitted it, so anchoring the Final to an
     * earlier slot's reception walks the Final straight into the slot of the
     * responder that was missed. Modelling it faithfully is what makes this
     * scenario collision-free. */
    /* Equation (2) and its nominal fallback, both from the firmware's own
     * slot_final_hi32(). The fallback is load-bearing: without it a responder
     * whose Response the initiator missed still transmitted, and the Final
     * walks into that responder's slot. */
    final_tx_full = launch_from(
        poll_tx_full,
        slot_final_hi32(c, (uint32_t)(poll_tx_full >> 8),
                        (uint32_t)(resp_rx_full[last] >> 8), last));
    final_tx_w    = WIRE(final_tx_full);
    final_g       = sim_global_from_local(&t->clk, final_tx_full);

    tx_log(final_g, final_g + ns_to_dtu_g(timing_air_final_ns(c)),
           (int)ti, "Final");

    /* --- Report group: equation (3), and the responder's DS-TWR ----------- */
    for (s = 0; s < nr; s++)
    {
        double   d_true;
        int64_t  final_rx_full;
        twr_ts_t ts;
        int64_t  tau, rep_g, rep_full;

        if (!got_resp[s])              { continue; }
        if (lost(g_loss_x100))         { continue; }   /* Final not heard */

        d_true        = dist_mm(t->x, t->y, A[s].x, A[s].y);
        final_rx_full = sim_local_dtu(&A[s].clk, final_g + prop[s]);

        ts.poll_tx  = poll_tx_w;
        ts.poll_rx  = poll_rx_w[s];
        ts.resp_tx  = resp_tx_w[s];
        ts.resp_rx  = resp_rx_w[s];
        ts.final_tx = final_tx_w;
        ts.final_rx = WIRE(final_rx_full);

        tau = ranging_tof_dtu(&ts);

        /* Equation (3), evaluated by the firmware's own slot_report_hi32(). */
        rep_full = launch_from(
            final_rx_full,
            slot_report_hi32(c, (uint32_t)(final_rx_full >> 8), (uint8_t)s));
        rep_g = sim_global_from_local(&A[s].clk, rep_full);

        tx_log(rep_g, rep_g + ns_to_dtu_g(timing_air_report_ns(c)),
               100 + (int)s, "Report");
        A[s].rep_tx++;

        if (tau < 0)                   { continue; }
        if (lost(g_loss_x100))         { continue; }   /* Report not received */

        {
            int32_t d_mm = ranging_ps_to_mm(ranging_dtu_to_ps(tau), 0);
            double  err  = (double)d_mm - d_true;

            t->ranges[s]++;
            t->err_sum_mm[s] += err;
            if (fabs(err) > t->err_max_mm[s]) { t->err_max_mm[s] = fabs(err); }

            if (trace_active())
            {
                uint64_t six[6] = { poll_tx_w, poll_rx_w[s], resp_tx_w[s],
                                    resp_rx_w[s], final_tx_w,
                                    WIRE(final_rx_full) };

                trace_range(rep_g, ti, (uint32_t)(0x0101u + s), s, sfn,
                            d_mm, d_true, ranging_dtu_to_ps(tau), 0u, six);
            }
        }
    }

    /* --- every other initiator overhears this Poll: equations (8) and (9) - */
    for (s = 0; s < N_TAGS; s++)
    {
        tag_t  *o = &T[s];
        int64_t rx_g;

        if (s == ti)           { continue; }
        if (lost(g_loss_x100)) { continue; }

        rx_g = poll_g + sim_prop_dtu(dist_mm(t->x, t->y, o->x, o->y));

        sim_bind_clock(&o->clk, rx_g);
        *cfg() = o->cfg;
        {
            sync_state_t was = o->sync.state;
            uint32_t     re0 = o->sync.n_rearm, dr0 = o->sync.n_drift_reset;
            bool         ok;

            ok = sync_on_poll(&o->sync, &o->cfg,
                              (uint32_t)(sim_local_dtu(&o->clk, rx_g) >> 8),
                              t->cfg.frame_index, (uint16_t)sfn, true);
            if (ok)
            {
                o->alarm_g   = sim_global_from_hi32(&o->clk,
                                                    o->sync.t_next_hi32, rx_g);
                o->has_alarm = true;
            }
            if (trace_active())
            {
                trace_sync_ev(rx_g, s, sync_state_str(o->sync.state),
                              (o->sync.n_rearm > re0) ? "rearm"
                              : (o->sync.n_drift_reset > dr0) ? "drift_reset"
                              : "none",
                              o->sync.last_err_us, o->sync.beta_ppb,
                              (uint32_t)(((int)t->cfg.frame_index
                                          <= (int)o->cfg.frame_index)
                                         ? (o->cfg.frame_index - t->cfg.frame_index)
                                         : 0));
            }
            if (getenv("SIM_TRACE2") && g_trace2++ < 24)
            {
                printf("      poll t%u->t%u @%.3fms %s->%s %s err=%dus "
                       "alarm=%.3fms\n",
                       ti, s, (double)rx_g / DTU_PER_NS / 1e6,
                       sync_state_str(was), sync_state_str(o->sync.state),
                       (o->sync.n_rearm > re0) ? "REARM" :
                       (o->sync.n_drift_reset > dr0) ? "DRIFTRESET" : "-",
                       o->sync.last_err_us,
                       (double)o->alarm_g / DTU_PER_NS / 1e6);
            }
        }
        note_state(o, rx_g);
    }
}

/* ------------------------------------------------------------------------- */
/* Equivalence check: slots.c must reproduce, bit for bit, the expressions      */
/* that used to sit inline in role_initiator.c and role_responder.c. This is    */
/* what makes the refactor safe rather than merely plausible.                   */
/* ------------------------------------------------------------------------- */
static int check_slot_equivalence(const cfg_t *base)
{
    static const uint32_t refs[] = { 0u, 1u, 255u, 0x7FFFFFFFu, 0x80000000u,
                                     0xFFFFFF00u, 0xFFFFFFFFu, 123456789u };
    cfg_t c = *base;
    int   bad = 0;
    unsigned nr, ri, s;

    for (nr = 2u; nr <= CFG_MAX_RESP; nr++)
    {
        c.resp_count = (uint8_t)nr;

        for (ri = 0; ri < sizeof(refs) / sizeof(refs[0]); ri++)
        {
            uint32_t t = refs[ri];

            /* Equation (1), as role_responder.c had it. */
            for (s = 0; s < nr; s++)
            {
                uint32_t want = t + us_to_hi32(c.resp_base_us
                                               + (uint32_t)s * c.resp_pitch_us);
                if (slot_resp_hi32(&c, t, (uint8_t)s) != want) { bad++; }

                /* Equation (3). */
                want = t + us_to_hi32(c.report_base_us
                                      + (uint32_t)s * c.report_pitch_us);
                if (slot_report_hi32(&c, t, (uint8_t)s) != want) { bad++; }
            }

            /* Equation (2) and its nominal fallback. */
            for (s = 0; s < nr; s++)
            {
                uint32_t want = (s == nr - 1u)
                    ? (t + us_to_hi32(c.resp_pitch_us))
                    : (t + us_to_hi32(c.resp_base_us
                                      + (uint32_t)nr * c.resp_pitch_us));
                if (slot_final_hi32(&c, t, t, (int32_t)s) != want) { bad++; }
            }

            /* The two receive windows and the exchange deadline. */
            {
                /* A bounded capture allowance, not guard_us/2: the window
                 * must not eat the Final's arming budget -- see slots.c. */
                uint32_t want = t
                    + us_to_hi32(c.resp_base_us
                                 + (uint32_t)(nr - 1u) * c.resp_pitch_us + 8u)
                    + ns_to_hi32(timing_air_resp_ns(&c));
                if (slot_resp_window_end_hi32(&c, t) != want) { bad++; }

                want = t
                    + us_to_hi32(c.report_base_us
                                 + (uint32_t)(nr - 1u) * c.report_pitch_us)
                    + ns_to_hi32(timing_air_report_ns(&c))
                    + us_to_hi32(c.guard_us / 2u);
                if (slot_report_window_end_hi32(&c, t) != want) { bad++; }

                want = t
                    + us_to_hi32(c.resp_base_us
                                 + (uint32_t)nr * c.resp_pitch_us
                                 + c.rx_timeout_us)
                    + ns_to_hi32(timing_air_final_ns(&c));
                if (slot_exchange_deadline_hi32(&c, t) != want) { bad++; }
            }
        }
    }

    printf("slots.c matches the pre-refactor inline expressions: %s\n\n",
           bad ? "NO" : "yes");
    if (bad) { printf("  FAIL %d slot mismatches\n\n", bad); }
    return bad ? 1 : 0;
}

/* ------------------------------------------------------------------------- */
static int scenario(const char *name, bool drift, uint32_t loss_x100,
                    const cfg_t *base)
{
    unsigned i, s;
    int      fail = 0;
    uint32_t sfn = 0;
    int64_t  now = 0;
    double   hz0 = 0.0;

    memset(T, 0, sizeof(T));
    memset(A, 0, sizeof(A));
    g_tx_n      = 0;
    g_loss_x100 = loss_x100;
    g_trace     = 0;
    g_trace2    = 0;
    s_rng       = 0x2545F4914F6CDD1DULL;

    /* One trace per scenario, so the UI can show the clean baseline and the
     * lossy run side by side rather than only whichever ran first. */
    if (g_trace_base)
    {
        char path[512];

        snprintf(path, sizeof(path), "%s.%s.json", g_trace_base,
                 loss_x100 ? "realistic" : "ideal");
        trace_open(path, g_trace_win);
    }
    if (g_live) { trace_live(g_realtime); }

    printf("=== %s ===\n", name);
    printf("  clocks: %s   frame loss: %.2f%% per hop\n",
           drift ? "static offset + thermal drift" : "static offset only",
           (double)loss_x100 / 100.0);

    for (i = 0; i < N_TAGS; i++)
    {
        T[i].cfg = *base;
        T[i].cfg.role        = ROLE_INIT;
        T[i].cfg.frame_index = (uint8_t)i;
        T[i].cfg.node_id     = (uint16_t)(0x0001u + i);
        T[i].clk.ppm         = -18.0 + 11.0 * (double)i;
        T[i].clk.phase_dtu   = (int64_t)(i * 7919);
        T[i].clk.drift_ppm   = drift ? (2.0 + 0.7 * (double)i) : 0.0;
        T[i].clk.drift_hz    = drift ? (0.05 + 0.011 * (double)i) : 0.0;
        T[i].x = 3000.0 + 4000.0 * (double)i;
        T[i].y = 5000.0 + 900.0 * (double)(i % 3);
    }
    for (s = 0; s < N_ANCH; s++)
    {
        A[s].cfg = *base;
        A[s].cfg.role       = ROLE_RESP;
        A[s].cfg.slot_index = (uint8_t)s;
        A[s].cfg.node_id    = (uint16_t)(0x0101u + s);
        A[s].clk.ppm        = 20.0 - 3.4 * (double)s;
        A[s].clk.phase_dtu  = (int64_t)(s * 104729);
        A[s].clk.drift_ppm  = drift ? (1.5 + 0.3 * (double)s) : 0.0;
        A[s].clk.drift_hz   = drift ? (0.07 + 0.005 * (double)s) : 0.0;
        A[s].x = 10000.0 + 9000.0 * cos(2.0 * M_PI * (double)s / (double)N_ANCH);
        A[s].y = 6000.0  + 9000.0 * sin(2.0 * M_PI * (double)s / (double)N_ANCH);
        for (i = 0; i < N_TAGS; i++)
        {
            T[i].cfg.anchor[s].id   = A[s].cfg.node_id;
            T[i].cfg.anchor[s].slot = (uint8_t)s;
            T[i].cfg.anchor[s].en   = 1;
            T[i].cfg.anchor[s].ant  = 0;
        }
    }

    for (i = 0; i < N_TAGS + N_ANCH; i++)
    {
        const cfg_t *n = (i < N_TAGS) ? &T[i].cfg : &A[i - N_TAGS].cfg;
        const char  *k = "", *r = "";
        int          v;

        *cfg() = *n;
        v = cfg_validate(n, &k, &r);
        if (v != 0)
        {
            printf("  FAIL node %u config invalid: %d %s %s\n", i, v, k, r);
            return 1;
        }
    }

    if (trace_active())
    {
        unsigned k;

        trace_meta(base, name, N_TAGS, N_ANCH, 0x2545F4914F6CDD1DULL,
                   SUPERFRAMES, loss_x100);
        for (k = 0; k < N_TAGS; k++)
        {
            trace_node(k, ROLE_INIT, T[k].cfg.node_id, T[k].x, T[k].y,
                       T[k].clk.ppm, T[k].clk.drift_ppm, T[k].clk.drift_hz);
        }
        for (k = 0; k < N_ANCH; k++)
        {
            trace_node(N_TAGS + k, ROLE_RESP, A[k].cfg.node_id, A[k].x, A[k].y,
                       A[k].clk.ppm, A[k].clk.drift_ppm, A[k].clk.drift_hz);
        }
        trace_nodes_done();
    }

    sim_bind_clock(&T[0].clk, 0);
    sync_adopt_local_epoch(&T[0].sync, &T[0].cfg);
    T[0].alarm_g   = sim_global_from_hi32(&T[0].clk, T[0].sync.t_next_hi32, 0);
    T[0].has_alarm = true;
    for (i = 1; i < N_TAGS; i++)
    {
        sim_bind_clock(&T[i].clk, 0);
        sync_start(&T[i].sync, &T[i].cfg);
        note_state(&T[i], 0);
    }

    /* The role loop of section 10.1, faithfully: a COLD initiator only
     * listens until its cold window expires, and a LOCKED or FREERUN one
     * advances its alarm every time that alarm elapses -- whether or not it
     * actually transmitted. Advancing only the transmitter leaves a
     * non-transmitting node's t_next a superframe stale, which then reads as a
     * T_F prediction error and drift-resets forever. */
    while (sfn < SUPERFRAMES)
    {
        unsigned best = N_TAGS;
        int64_t  best_g = 0;
        bool     best_is_cold = false;

        for (i = 0; i < N_TAGS; i++)
        {
            int64_t when;
            bool    is_cold = T[i].in_cold;

            if (is_cold)
            {
                when = T[i].cold_g;
            }
            else if (T[i].has_alarm)
            {
                when = T[i].alarm_g;
            }
            else
            {
                continue;
            }

            if (best == N_TAGS || when < best_g)
            {
                best = i; best_g = when; best_is_cold = is_cold;
            }
        }
        if (best == N_TAGS) { printf("  FAIL every initiator is stuck\n"); return 1; }

        now = best_g;
        sim_bind_clock(&T[best].clk, now);
        *cfg() = T[best].cfg;

        if (getenv("SIM_TRACE") && g_trace++ < 40)
        {
            printf("    [%8.3f ms] tag%u %-7s %s due=%d tx=%d missed=%u\n",
                   (double)now / DTU_PER_NS / 1e6, best,
                   sync_state_str(T[best].sync.state),
                   best_is_cold ? "COLDDL" : "ALARM ",
                   sync_due_this_superframe(&T[best].sync, &T[best].cfg),
                   sync_may_transmit(&T[best].sync), T[best].sync.missed);
        }

        if (best_is_cold)
        {
            /* Cold window expired with nothing heard: we are first. */
            sync_adopt_local_epoch(&T[best].sync, &T[best].cfg);
            T[best].alarm_g   = sim_global_from_hi32(&T[best].clk,
                                                     T[best].sync.t_next_hi32, now);
            T[best].has_alarm = true;
            note_state(&T[best], now);
            continue;
        }

        /* The alarm elapsed. */
        sync_missed_frame(&T[best].sync, &T[best].cfg);
        note_state(&T[best], now);

        if (sync_may_transmit(&T[best].sync)
            && sync_due_this_superframe(&T[best].sync, &T[best].cfg))
        {
            run_frame(best, now, T[best].sync.sfn);
        }

        sim_bind_clock(&T[best].clk, now);
        *cfg() = T[best].cfg;
        sync_advance(&T[best].sync, &T[best].cfg);
        T[best].alarm_g = sim_global_from_hi32(&T[best].clk,
                                               T[best].sync.t_next_hi32, now);
        note_state(&T[best], now);
        if (best == 0)
        {
            sfn++;
            if (trace_active()) { trace_superframe(sfn, now); }
        }
        if (trace_active()) { trace_live_draw(now, false); }
    }
    if (trace_active()) { trace_live_draw(now, true); }

    /* ---------------- per tag per anchor ---------------- */
    printf("\n  tag  polls  lost  min/anch  max/anch  spread  peak err   bias   state\n");
    for (i = 0; i < N_TAGS; i++)
    {
        uint32_t mn = 0xFFFFFFFFu, mx = 0;
        double   emx = 0.0, bias = 0.0;
        uint32_t nz = 0;

        for (s = 0; s < N_ANCH; s++)
        {
            if (T[i].ranges[s] < mn) { mn = T[i].ranges[s]; }
            if (T[i].ranges[s] > mx) { mx = T[i].ranges[s]; }
            if (T[i].err_max_mm[s] > emx) { emx = T[i].err_max_mm[s]; }
            if (T[i].ranges[s]) { bias += T[i].err_sum_mm[s] / (double)T[i].ranges[s]; nz++; }
        }
        if (nz) { bias /= (double)nz; }

        printf("  %3u %6u %5u %9u %9u %7u %6.1f mm %+6.2f  %s\n",
               i, T[i].polls, T[i].frames_lost, mn, mx, mx - mn, emx, bias,
               sync_state_str(T[i].sync.state));

        if (mn == 0)
        {
            printf("    FAIL tag %u: an anchor never ranged at all\n", i);
            fail = 1;
        }
        if (emx > SIM_QUANT_MM * 3.0)
        {
            printf("    FAIL tag %u: peak error %.1f mm exceeds %.1f mm\n",
                   i, emx, SIM_QUANT_MM * 3.0);
            fail = 1;
        }
        if (fabs(bias) > SIM_QUANT_MM * 0.5)
        {
            printf("    FAIL tag %u: mean bias %.2f mm exceeds %.2f mm\n",
                   i, bias, SIM_QUANT_MM * 0.5);
            fail = 1;
        }

        if (loss_x100 == 0u)
        {
            /* No loss: every anchor must answer every single Poll. */
            if (mx != mn)
            {
                printf("    FAIL tag %u: anchors disagree by %u samples\n", i, mx - mn);
                fail = 1;
            }
            if (mn != T[i].polls)
            {
                printf("    FAIL tag %u: %u polls but %u ranges per anchor\n",
                       i, T[i].polls, mn);
                fail = 1;
            }
        }
        else
        {
            /* With independent per-hop loss, the counts are binomial. Four
             * hops must survive, so p = (1-l)^4; require every anchor within
             * 5 sigma of that and no anchor systematically starved. */
            double l  = (double)loss_x100 / 10000.0;
            double p  = pow(1.0 - l, 4.0);
            double n  = (double)T[i].polls;
            double mu = n * p;
            double sd = sqrt(n * p * (1.0 - p));

            for (s = 0; s < N_ANCH; s++)
            {
                if (fabs((double)T[i].ranges[s] - mu) > 5.0 * sd)
                {
                    printf("    FAIL tag %u anchor %u: %u ranges, expected "
                           "%.0f +/- %.0f (5 sigma)\n",
                           i, s, T[i].ranges[s], mu, 5.0 * sd);
                    fail = 1;
                }
            }
        }
    }

    /* ---------------- rate ---------------- */
    printf("\n  rate per tag per anchor:");
    for (i = 0; i < N_TAGS; i++)
    {
        double span = (double)(T[i].last_poll_g - T[i].first_poll_g) / SIM_DTU_PER_S_D;
        double hz   = (span > 0.0) ? ((double)(T[i].polls - 1u) / span) : 0.0;

        printf("  t%u %.3f Hz", i, hz);
        if (i == 0) { hz0 = hz; }
        else if (fabs(hz - hz0) / hz0 > 0.01)
        {
            printf("\n    FAIL tag %u rate %.3f Hz differs from tag 0 %.3f Hz by >1%%\n",
                   i, hz, hz0);
            fail = 1;
        }
    }
    printf("\n");

    /* ---------------- interval jitter ---------------- */
    printf("  poll interval (nominal %u us):", timing_superframe_us(base));
    for (i = 0; i < N_TAGS; i++)
    {
        double j_ns = ((double)(T[i].gap_max - T[i].gap_min) / DTU_PER_NS);

        printf("  t%u %.0f ns", i, j_ns);
        if (j_ns / 1000.0 > (double)base->guard_us)
        {
            printf("\n    FAIL tag %u jitter %.1f us exceeds guard_us %u\n",
                   i, j_ns / 1000.0, base->guard_us);
            fail = 1;
        }
    }
    printf("\n");

    /* ---------------- section 6 health ---------------- */
    printf("  sync: ");
    for (i = 0; i < N_TAGS; i++)
    {
        printf(" t%u rearm=%u free=%u dup=%u drift=%u err=%dus",
               i, T[i].sync.n_rearm, T[i].sync.n_freerun,
               T[i].sync.n_dup, T[i].sync.n_drift_reset, T[i].sync.last_err_us);
        if (T[i].sync.n_rearm == 0u)
        {
            printf("\n    FAIL tag %u never rearmed off an overheard Poll\n", i);
            fail = 1;
        }
        if (T[i].sync.n_dup != 0u)
        {
            printf("\n    FAIL tag %u raised %u duplicate-index faults\n",
                   i, T[i].sync.n_dup);
            fail = 1;
        }
    }
    printf("\n");

    /* ---------------- slot overlap ---------------- */
    qsort(g_tx, g_tx_n, sizeof(*g_tx), tx_cmp);
    {
        size_t  j, k, clashes = 0;
        int64_t worst = 0, gap_min = 0;

        for (j = 0; j + 1u < g_tx_n; j++)
        {
            int64_t gap = g_tx[j + 1u].s - g_tx[j].e;

            if (gap >= 0 && (gap_min == 0 || gap < gap_min)) { gap_min = gap; }

            for (k = j + 1u; k < g_tx_n && g_tx[k].s < g_tx[j].e; k++)
            {
                if (g_tx[k].who == g_tx[j].who) { continue; }
                if (clashes < 5u)
                {
                    printf("    CLASH %s(%d) vs %s(%d) overlap %.1f us\n",
                           g_tx[j].kind, g_tx[j].who, g_tx[k].kind, g_tx[k].who,
                           (double)(g_tx[j].e - g_tx[k].s) / DTU_PER_NS / 1000.0);
                }
                if (g_tx[j].e - g_tx[k].s > worst) { worst = g_tx[j].e - g_tx[k].s; }
                trace_collision(g_tx[k].s, (uint32_t)g_tx[j].who,
                                (uint32_t)g_tx[k].who,
                                (double)(g_tx[j].e - g_tx[k].s) / DTU_PER_NS / 1000.0);
                clashes++;
            }
        }
        printf("  slots: %zu transmissions, %zu collisions, tightest gap %.1f us\n",
               g_tx_n, clashes, (double)gap_min / DTU_PER_NS / 1000.0);
        if (clashes)
        {
            printf("    FAIL %zu slot collisions, worst %.1f us\n",
                   clashes, (double)worst / DTU_PER_NS / 1000.0);
            fail = 1;
        }
    }

    if (trace_active())
    {
        unsigned a, b;

        for (a = 0; a < N_TAGS; a++)
        {
            for (b = 0; b < N_ANCH; b++)
            {
                double span = (double)(T[a].last_poll_g - T[a].first_poll_g)
                            / SIM_DTU_PER_S_D;
                double hz   = (span > 0.0)
                            ? ((double)T[a].ranges[b] / span) : 0.0;
                double bias = T[a].ranges[b]
                            ? T[a].err_sum_mm[b] / (double)T[a].ranges[b] : 0.0;

                trace_summary_pair(a, b, T[a].ranges[b], hz, bias,
                                   T[a].err_max_mm[b]);
            }
        }
        for (a = 0; a < N_TAGS; a++)
        {
            double span = (double)(T[a].last_poll_g - T[a].first_poll_g)
                        / SIM_DTU_PER_S_D;

            trace_summary_tag(a, T[a].polls,
                              (span > 0.0) ? ((double)(T[a].polls - 1u) / span) : 0.0,
                              (int64_t)((double)(T[a].gap_max - T[a].gap_min)
                                        / DTU_PER_NS),
                              sync_state_str(T[a].sync.state),
                              T[a].sync.n_rearm, T[a].sync.n_freerun,
                              T[a].sync.n_dup, T[a].sync.n_drift_reset,
                              T[a].sync.last_err_us);
        }
        trace_summary_run((uint32_t)g_tx_n, 0u, 0.0, fail == 0);
        trace_close();
    }

    printf("  %s\n\n", fail ? "FAIL" : "PASS");
    return fail;
}

/* ------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    cfg_t       base;
    int         fail = 0;
    const char *trace_path = NULL;
    uint32_t    trace_win  = 20u;
    bool        live       = false;
    double      realtime   = 0.0;
    int         i;

    for (i = 1; i < argc; i++)
    {
        if      (!strncmp(argv[i], "--trace=", 8))        { trace_path = argv[i] + 8; }
        else if (!strncmp(argv[i], "--trace-window=", 15)) { trace_win = (uint32_t)atoi(argv[i]+15); }
        else if (!strcmp(argv[i], "--live"))              { live = true; }
        else if (!strncmp(argv[i], "--realtime=", 11))    { live = true; realtime = atof(argv[i]+11); }
        else if (!strcmp(argv[i], "--realtime"))          { live = true; realtime = 1.0; }
        else if (!strcmp(argv[i], "--help"))
        {
            printf("usage: sim [--trace=FILE] [--trace-window=N] "
                   "[--live] [--realtime[=MULT]]\n");
            return 0;
        }
    }
    g_trace_base = trace_path;
    g_trace_win  = trace_win;
    g_live       = live;
    g_realtime   = realtime;

    cfg_defaults(&base);
    base.resp_count    = (uint8_t)N_ANCH;
    base.init_count    = (uint8_t)N_TAGS;
    base.report_enable = 1;
    base.ant_delay_tx  = 0;
    base.ant_delay_rx  = 0;
    base.node_id       = 0x0001u;
    /* Equation (14) is not satisfied by the section 8.1 default of 4000 us at
     * 12 responders, so size the frame from the model, as $AUTOTIME would. */
    base.frame_dur_us  = timing_frame_dur_us(&base);

    printf("%u tags x %u anchors, %u superframes each\n",
           N_TAGS, N_ANCH, SUPERFRAMES);
    printf("T_f = %u us   T_F = %u us   nominal %.2f Hz per tag per anchor\n\n",
           base.frame_dur_us, timing_superframe_us(&base),
           1.0e6 / (double)timing_superframe_us(&base));

    fail |= check_slot_equivalence(&base);
    fail |= scenario("IDEAL      static clocks, no loss", false, 0u,   &base);
    fail |= scenario("REALISTIC  thermal drift, 2% loss", true,  200u, &base);

    printf("%s\n", fail ? "FAIL" : "ALL SCENARIOS PASS");
    free(g_tx);
    return fail;
}
