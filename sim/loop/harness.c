/**
 * @file    harness.c
 * @brief   Fork-per-node harness: the real role loops, off target.
 *
 * 4 initiators + 12 responders, each a forked child running the firmware's own
 * app_run_initiator() / app_run_responder() unmodified. Only radio.c and
 * uart_io.c are replaced. Forking is what makes it possible at all: the role
 * files keep their per-node state in file-scope statics, so four initiators
 * cannot share one address space.
 *
 * This covers what sim/ cannot reach -- the RX re-arm sequence across twelve
 * consecutive Responses, the Response/Report mask assembly, the tx_late paths
 * and radio_tx_wait() -- and it asserts on the real $RNG output each node
 * formats through the firmware's own output.c.
 *
 * The parent owns virtual time. REQ_TX and REQ_IDLE do not advance the
 * timeline, so they are answered on arrival; only REQ_RX and REQ_TXWAIT park a
 * child, so the barrier is simply "every live child is parked".
 */
#include "ipc.h"
#include "../sim.h"

#include "cfg.h"
#include "app.h"
#include "timing.h"
#include "proto.h"
#include "sync.h"
#include "output.h"
#include "dw1000_consts.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <sys/wait.h>

#define N_TAGS          4u
#define N_ANCH          12u
#define N_NODES         (N_TAGS + N_ANCH)
#define DTU_PER_NS      63.8976
#define SIM_QUANT_MM    4.69
#define MAX_AIR         256u

void app_run_initiator(void);
void app_run_responder(void);
void node_radio_set_part_id(uint32_t id);
void node_uart_open(const char *path);

/* --- per-node bookkeeping in the parent ----------------------------------- */
typedef enum { P_RUNNING = 0, P_RX, P_TXWAIT } park_t;

typedef struct
{
    pid_t     pid;
    int       req_fd, rep_fd;
    bool      alive;
    park_t    park;
    nclk_t    clk;
    double    x, y;
    uint8_t   role;
    uint16_t  addr;
    uint8_t   idx;              /* frame_index or slot_index */
    int64_t   t_g;              /* THIS node's own current global instant.
                                 * A single shared clock is wrong: charging one
                                 * node's processing cost to it advances every
                                 * other node too, which collapsed all the
                                 * launch instants together. */
    int64_t   rx_until_g;
    int64_t   rx_open_g;        /* when this node's receiver came on */
    int       armed;            /* index into g_air, or -1 */
    char      outpath[256];
} node_t;

typedef struct
{
    bool     used;
    int      tx_node;
    int64_t  start_g, end_g;
    uint16_t len;
    uint8_t  buf[IPC_MAX_FRAME];
    bool     seen[N_NODES];     /* delivered or written off, per receiver */
    bool     clash_counted;     /* so one overlap is not counted per receiver */
} airtx_t;

static node_t  g_n[N_NODES];
static airtx_t g_air[MAX_AIR];
static int64_t g_now;
static cfg_t   g_base;
static uint32_t g_sf, g_sf_limit = 400u;
static uint32_t g_loss_x100;
static uint32_t g_proc_us = 60u;
/* The deployment's link. Serialising a byte costs 10 bits at 8N1, and the
 * initiator writes N_ANCH binary records at the end of every frame -- about
 * 688 us for eight, which eq (14) never budgeted for. Charging it is what
 * makes the simulated frame as tight as the real one. */
static uint32_t g_uart_baud = 115200u;
/* How far ahead of the RMARKER the receiver must already be listening.
 *
 * Zero means only "you cannot timestamp an RMARKER you were not listening
 * for", which is the weakest true statement and the default, so the bundled
 * scenario keeps asserting exactly what it did before. Set it to the
 * preamble length (128 symbols at ~1.0256 us = 131 us for the deployment's
 * PHY) to model acquisition properly: the DW1000 locks onto the preamble, so
 * a receiver armed after it began gets nothing. That is the setting under
 * which an initiator arming its peer watch after writing its range records
 * misses the peer entirely -- run --preamble-us=131 to see it. */
static uint32_t g_preamble_us;
static uint64_t g_rng = 0x2545F4914F6CDD1DULL;
static uint32_t g_collisions, g_tx_late, g_txs;
static uint32_t g_late_kind[4];   /* Poll, Resp, Final, Report */
static uint32_t g_tr, g_tr2;
#define TR(...) do { if (getenv("HARNESS_TRACE") && g_tr++ < 60) { printf("    "); printf(__VA_ARGS__); } } while (0)

static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17;
    return (uint32_t)(g_rng >> 32);
}
static bool lost(void) { return g_loss_x100 && ((rnd() % 10000u) < g_loss_x100); }

static double dist_mm(const node_t *a, const node_t *b)
{
    return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

static int64_t node_global_from_hi32(const node_t *n, uint32_t hi32, int64_t after)
{
    int64_t  base    = sim_local_dtu(&n->clk, after);
    uint32_t base_hi = (uint32_t)((uint64_t)base >> 8);
    /* Signed hi32 distance, so the 32-bit wrap is handled without a magnitude
     * guess: alarms are always well inside +/-8.6 s. Align to the hi32
     * boundary rather than adding a delta to `base`, which would keep base's
     * own low 8 bits and inject a spurious 0..255 DTU offset -- mean 127 DTU,
     * 2.0 ns, which lands as ~0.6 m of range error once it reaches a TX
     * timestamp. */
    int32_t  d     = (int32_t)(hi32 - base_hi);
    int64_t  local = (base - (base & 0xFF)) + ((int64_t)d << 8);

    return sim_global_from_local(&n->clk, local);
}

static int64_t air_dtu(uint16_t len)
{
    /* timing_air_ns() takes the payload length; the frame carries FCS. */
    uint32_t ns = timing_air_ns(&g_base, (uint32_t)(len > FCS_LEN ? len - FCS_LEN : len));

    return (int64_t)llround((double)ns * DTU_PER_NS);
}

/* --- replies -------------------------------------------------------------- */
static void reply(node_t *n, uint32_t kind)
{
    ipc_rep_t rp;

    memset(&rp, 0, sizeof(rp));
    rp.kind    = kind;
    rp.now_dtu = sim_local_dtu(&n->clk, n->t_g);
    if (!ipc_write_all(n->rep_fd, &rp, sizeof(rp))) { n->alive = false; }
    n->park = P_RUNNING;
}

/* Invert ranging_k_hat() so the receiver sees the integrator the DW1000 would
 * actually report for this pair.
 *
 * Equation (7) needs (1 + k_hat) to carry the receiver's clock rate over the
 * transmitter's, so k_hat ~= (ppm_rx - ppm_tx) * 1e-6, and
 * ranging_k_hat() decodes k_hat = -(integrator * freq_mult * hz_to_ppm / 1e6).
 * Returning zero here left the SS-TWR fallback with no crystal correction at
 * all, which is metres of error at tens of ppm across a 3.7 ms reply delay --
 * it made the loss scenario's accuracy assertion meaningless. */
static int32_t carrier_for(const node_t *rx, const node_t *tx)
{
    double k  = (rx->clk.ppm - tx->clk.ppm) * 1.0e-6;
    double hz = HERTZ_TO_PPM_MULTIPLIER_CHAN_5;      /* channel 5 default */
    double fm = FREQ_OFFSET_MULTIPLIER;

    return (int32_t)llround(-k * 1.0e6 / (fm * hz));
}

static void reply_frame(node_t *n, const airtx_t *tx, int64_t rmarker_g)
{
    ipc_rep_t rp;

    memset(&rp, 0, sizeof(rp));
    rp.kind        = REP_FRAME;
    rp.now_dtu     = sim_local_dtu(&n->clk, n->t_g);
    rp.rx_ts       = sim_local_ts40(&n->clk, rmarker_g);
    rp.carrier_int = carrier_for(n, &g_n[tx->tx_node]);
    rp.rx_dbm10    = -800;
    rp.fp_dbm10    = -880;
    rp.len         = tx->len > FCS_LEN ? (uint16_t)(tx->len - FCS_LEN) : tx->len;
    memcpy(rp.buf, tx->buf, rp.len);
    if (!ipc_write_all(n->rep_fd, &rp, sizeof(rp))) { n->alive = false; }
    n->park = P_RUNNING;
}

/* --- the air -------------------------------------------------------------- */
static bool tx_collides(int ai)
{
    uint32_t j;

    for (j = 0; j < MAX_AIR; j++)
    {
        if (!g_air[j].used || (int)j == ai)               { continue; }
        if (g_air[j].tx_node == g_air[ai].tx_node)        { continue; }
        if (g_air[j].start_g < g_air[ai].end_g
            && g_air[ai].start_g < g_air[j].end_g)        { return true; }
    }
    return false;
}

static int air_alloc(void)
{
    uint32_t j;

    for (j = 0; j < MAX_AIR; j++) { if (!g_air[j].used) { return (int)j; } }
    fprintf(stderr, "harness: air table full\n");
    exit(2);
}

static void air_retire(void)
{
    uint32_t j;
    unsigned i;

    /* Retire on time, not on delivery. Waiting for every receiver to be
     * handed a frame assumed every receiver eventually would -- but a node
     * whose receiver came on after the preamble never takes it, so those
     * entries pinned the table until it overflowed. The bound that actually
     * holds is the slowest node's clock: once every node is past the frame's
     * end plus the collision margin, nobody can be given it any more. */
    int64_t oldest = g_now;

    for (i = 0; i < N_NODES; i++)
    {
        if (g_n[i].alive && g_n[i].t_g < oldest) { oldest = g_n[i].t_g; }
    }

    for (j = 0; j < MAX_AIR; j++)
    {
        if (!g_air[j].used) { continue; }

        if (g_air[j].end_g + (int64_t)us_to_dtu(20000u) < oldest)
        {
            g_air[j].used = false;
        }
    }
}

/* Earliest instant at which an undelivered frame completes at node i. */
static bool next_frame_for(unsigned i, int64_t *when, int *which, int64_t *rmark)
{
    uint32_t j;
    bool     found = false;

    for (j = 0; j < MAX_AIR; j++)
    {
        int64_t prop, rm, done;

        if (!g_air[j].used || g_air[j].seen[i])   { continue; }
        if (g_air[j].tx_node == (int)i)           { continue; }

        prop = sim_prop_dtu(dist_mm(&g_n[i], &g_n[g_air[j].tx_node]));
        rm   = g_air[j].start_g + prop;
        done = rm + (g_air[j].end_g - g_air[j].start_g);

        /* A receiver that came on after the preamble had already started does
         * not get the frame. Without this the harness receives anything whose
         * RMARKER lands before the window closes, no matter how late the
         * receiver was actually armed -- so it could not model an arm that
         * misses its slot, which is the whole failure mode the end-of-frame
         * peer watch exists to prevent. The preamble precedes the RMARKER, so
         * the receiver has to be on before it, not merely before the mark. */
        if (rm - (int64_t)us_to_dtu(g_preamble_us) < g_n[i].rx_open_g)
        {
            continue;
        }

        if (!found || done < *when)
        {
            *when = done; *which = (int)j; *rmark = rm; found = true;
        }
    }
    return found;
}

/* --- request handling ----------------------------------------------------- */
static void handle_tx(node_t *n, const ipc_req_t *rq)
{
    int64_t launch;
    int     ai;

    /* Charge the processing cost of building and arming the frame to THIS
     * node's clock. That is what makes the tx_late paths reachable: the real
     * question is whether the armed instant is still in the future once the
     * work is done. */
    n->t_g = sim_global_from_local(&n->clk,
                                   sim_local_dtu(&n->clk, n->t_g)
                                   + (int64_t)us_to_dtu(g_proc_us));

    launch = node_global_from_hi32(n, rq->hi32 & 0xFFFFFFFEu, n->t_g);

    if (launch <= n->t_g)
    {
        g_tx_late++;
        /* Which message missed, and by how much: without this the counter
         * says only that something was late, and the obvious suspect is not
         * always the culprit. */
        if (rq->len >= COMMON_LEN)
        {
            uint8_t  ty = MSG_TYPE(rq->buf[IDX_TYPEVER]);
            unsigned k  = (ty == MSG_POLL)  ? 0u : (ty == MSG_RESP) ? 1u
                        : (ty == MSG_FINAL) ? 2u : 3u;

            g_late_kind[k]++;
            if (getenv("HARNESS_LATE") && g_late_kind[k] < 4u)
            {
                printf("    LATE %-6s n%02u short by %.1f us\n",
                       (k==0)?"Poll":(k==1)?"Resp":(k==2)?"Final":"Report",
                       (unsigned)(n - g_n),
                       (double)(n->t_g - launch) / DTU_PER_NS / 1000.0);
            }
        }
        reply(n, REP_TX_LATE);
        return;
    }

    TR("n%02u ARMED launch=%.3fms (now %.3fms)\n", (unsigned)(n - g_n),
       (double)launch / DTU_PER_NS / 1e6, (double)n->t_g / DTU_PER_NS / 1e6);
    ai = air_alloc();
    memset(&g_air[ai], 0, sizeof(g_air[ai]));
    g_air[ai].used    = true;
    g_air[ai].tx_node = (int)(n - g_n);
    g_air[ai].start_g = launch;
    g_air[ai].end_g   = launch + air_dtu(rq->len);
    g_air[ai].len     = rq->len;
    memcpy(g_air[ai].buf, rq->buf, rq->len);
    g_air[ai].seen[g_air[ai].tx_node] = true;

    /* Decode a Report on the way past and check the responder's DS-TWR against
     * ground truth. The parent knows every true flight time, so this isolates
     * an estimator error from a delivery or bookkeeping error. */
    if (rq->len >= REPORT_LEN - FCS_LEN
        && MSG_TYPE(rq->buf[IDX_TYPEVER]) == MSG_REPORT)
    {
        uint32_t s   = g_u8(rq->buf, REP_IDX_SLOT);
        uint16_t dst = g_u16(rq->buf, MHR_IDX_DST);
        uint32_t ps  = g_u32(rq->buf, REP_IDX_TOF_PS);
        unsigned ti;

        for (ti = 0; ti < N_TAGS; ti++)
        {
            if (g_n[ti].addr != dst) { continue; }
            {
                double d_true = dist_mm(&g_n[ti], &g_n[N_TAGS + s]);
                double t_true = d_true / SIM_C_MM_PER_S * 1.0e12;   /* ps */

                if (getenv("HARNESS_REPORTS") && g_tr2++ < 14)
                printf("    REPORT n%02u->t%u slot %2u  tof %8u ps  true %8.0f ps  "
                   "err %+7.0f ps (%+7.1f mm)\n",
                   (unsigned)(n - g_n), ti, s, ps, t_true,
                   (double)ps - t_true,
                   ((double)ps - t_true) * SIM_C_MM_PER_S / 1.0e12);
            }
        }
    }
    n->armed = ai;
    g_txs++;

    /* Count superframes off the frame-0 initiator's Polls. */
    if (n->role == ROLE_INIT && n->idx == 0u
        && rq->len >= COMMON_LEN
        && MSG_TYPE(rq->buf[IDX_TYPEVER]) == MSG_POLL)
    {
        g_sf++;
    }

    reply(n, REP_TX_OK);
}

/* Drain one node until it parks, dies, or the run ends. */
static void pump(node_t *n)
{
    while (n->alive && n->park == P_RUNNING)
    {
        ipc_req_t rq;

        if (!ipc_read_all(n->req_fd, &rq, sizeof(rq))) { n->alive = false; return; }

        if (g_sf >= g_sf_limit)
        {
            ipc_rep_t rp;
            memset(&rp, 0, sizeof(rp));
            rp.kind = REP_STOP;
            (void)ipc_write_all(n->rep_fd, &rp, sizeof(rp));
            n->alive = false;
            return;
        }

        /* Bill the UART time before servicing the request: the bytes were
         * written before the child got here. */
        if (rq.uart_bytes && g_uart_baud)
        {
            uint64_t us = ((uint64_t)rq.uart_bytes * 10ULL * 1000000ULL)
                        / (uint64_t)g_uart_baud;
            n->t_g += (int64_t)us_to_dtu((uint32_t)us);
        }

        switch (rq.kind)
        {
            case REQ_TX:
                TR("n%02u REQ_TX  hi32=%u len=%u t=%.3fms\n",
                   (unsigned)(n - g_n), rq.hi32, rq.len,
                   (double)n->t_g / DTU_PER_NS / 1e6);
                handle_tx(n, &rq);
                break;
            case REQ_IDLE:   reply(n, REP_OK); break;
            case REQ_RX:
                TR("n%02u REQ_RX  t=%.3fms until=%.3fms\n",
                   (unsigned)(n - g_n), (double)n->t_g / DTU_PER_NS / 1e6,
                   (double)sim_global_from_local(&n->clk, rq.until_dtu)
                       / DTU_PER_NS / 1e6);
                n->park       = P_RX;
                n->rx_until_g = sim_global_from_local(&n->clk, rq.until_dtu);
                /* A delayed arm opens the receiver at its programmed instant,
                 * not when the child finally asked -- the child may have been
                 * busy emitting in between, which is the whole point of
                 * arming before the emit rather than after it. */
                n->rx_open_g  = rq.from_dtu
                                ? sim_global_from_local(&n->clk, rq.from_dtu)
                                : n->t_g;
                break;
            case REQ_TXWAIT: n->park = P_TXWAIT; break;
            case REQ_DONE:   n->alive = false; break;
            default:
                fprintf(stderr, "harness: bad request %u\n", rq.kind);
                exit(2);
        }
    }
}

/* --- the step ------------------------------------------------------------- */
static bool step(void)
{
    unsigned i;
    int64_t  best = 0;
    bool     any  = false;

    for (i = 0; i < N_NODES; i++)
    {
        int64_t when = 0, rm = 0;
        int     which = -1;

        if (!g_n[i].alive) { continue; }

        if (g_n[i].park == P_TXWAIT)
        {
            when = (g_n[i].armed >= 0) ? g_air[g_n[i].armed].end_g : g_now;
        }
        else if (g_n[i].park == P_RX)
        {
            /* Whichever comes first: a frame finishing arriving, or the
             * timeout. A frame whose RMARKER lands inside the window is
             * delivered even if it completes after it, as the hardware would. */
            when = g_n[i].rx_until_g;
            if (next_frame_for(i, &rm, &which, &rm))
            {
                (void)which;
            }
            {
                int64_t w = 0, r = 0;
                int     j = -1;

                if (next_frame_for(i, &w, &j, &r) && r <= g_n[i].rx_until_g)
                {
                    when = w;
                }
            }
        }
        else
        {
            continue;
        }

        if (!any || when < best) { best = when; any = true; }
    }

    if (!any) { return false; }
    if (best > g_now) { g_now = best; }
    (void)0;

    /* Release everyone due now. */
    for (i = 0; i < N_NODES; i++)
    {
        if (!g_n[i].alive) { continue; }

        if (g_n[i].park == P_TXWAIT)
        {
            int64_t end = (g_n[i].armed >= 0) ? g_air[g_n[i].armed].end_g : g_now;

            if (end <= g_now)
            {
                g_n[i].armed = -1;
                g_n[i].t_g   = end;
                reply(&g_n[i], REP_OK);
            }
        }
        else if (g_n[i].park == P_RX)
        {
            int64_t when = 0, rm = 0;
            int     which = -1;

            if (next_frame_for(i, &when, &which, &rm)
                && when <= g_now && rm <= g_n[i].rx_until_g)
            {
                g_air[which].seen[i] = true;
                g_n[i].t_g = when;

                if (tx_collides(which))
                {
                    /* Counted once per overlapping frame, not once per
                     * receiver, or a single clash inflates by twelve. */
                    if (!g_air[which].clash_counted)
                    {
                        g_air[which].clash_counted = true;
                        g_collisions++;
                    }
                }
                else if (!lost())
                {
                    TR("n%02u RECV from n%02u rm=%.3fms done=%.3fms\n",
                       i, (unsigned)g_air[which].tx_node,
                       (double)rm / DTU_PER_NS / 1e6,
                       (double)when / DTU_PER_NS / 1e6);
                    reply_frame(&g_n[i], &g_air[which], rm);
                }
                else
                {
                    g_n[i].park = P_RX;    /* frame dropped: keep listening */
                }
            }
            else if (g_n[i].rx_until_g <= g_now)
            {
                g_n[i].t_g = g_n[i].rx_until_g;
                reply(&g_n[i], REP_TIMEOUT);
            }
        }
    }

    air_retire();
    return true;
}

/* --- $RNG parsing --------------------------------------------------------- */
static uint32_t g_pair[N_TAGS][N_ANCH];
static uint32_t g_max_sfn[N_TAGS];
static double   g_err_max[N_TAGS][N_ANCH];
static double   g_err_sum[N_TAGS][N_ANCH];
static uint32_t g_flag_resp_lost[N_TAGS][N_ANCH];
static uint32_t g_zero_anchor;
/* Frame sync is the point of the multi-initiator harness, so count what
 * the nodes themselves report about it rather than inferring it. */
static uint32_t g_desync[N_TAGS];
static uint32_t g_records[N_TAGS];

/* The run is cut off mid-superframe, so the highest sfn a tag emitted is
 * partial: the early slots got a range and the late ones did not. Counting it
 * shows up as a spread of one and looks like starvation. Both passes below
 * therefore ignore each tag's highest sfn -- an honest exclusion of an
 * incomplete superframe, not a slackened assertion. */
static void parse_output(void)
{
    unsigned t;
    int      pass;

    for (pass = 0; pass < 2; pass++)
    for (t = 0; t < N_TAGS; t++)
    {
        FILE *f = fopen(g_n[t].outpath, "r");
        char  line[512];

        if (!f) { continue; }

        while (fgets(line, sizeof(line), f))
        {
            unsigned long sfn, tof; unsigned frame, tag, anch, slot, flags;
            long dmm; char rxp[16], fpp[16]; unsigned long tus;

            if (sscanf(line, "$RNG,%lu,%u,0x%04X,0x%04X,%u,%ld,%lu,%15[^,],%15[^,],0x%02X,%lu",
                       &sfn, &frame, &tag, &anch, &slot, &dmm, &tof,
                       rxp, fpp, &flags, &tus) != 11)
            {
                continue;
            }
            if (slot >= N_ANCH) { continue; }

            if (pass == 0)
            {
                if ((uint32_t)sfn > g_max_sfn[t]) { g_max_sfn[t] = (uint32_t)sfn; }
                continue;
            }
            if ((uint32_t)sfn >= g_max_sfn[t]) { continue; }   /* partial */
            if (anch == 0u) { g_zero_anchor++; }

            g_records[t]++;
            if (flags & RNG_FLAG_DESYNC) { g_desync[t]++; }

            if (flags & RNG_FLAG_RESP_LOST)
            {
                g_flag_resp_lost[t][slot]++;
            }
            else
            {
                double d_true = dist_mm(&g_n[t], &g_n[N_TAGS + slot]);
                double err    = (double)dmm - d_true;

                g_pair[t][slot]++;
                g_err_sum[t][slot] += err;
                if (fabs(err) > g_err_max[t][slot]) { g_err_max[t][slot] = fabs(err); }
            }
        }
        fclose(f);
    }
}

/* --- setup ---------------------------------------------------------------- */
static void place_nodes(void)
{
    unsigned i, s;

    for (i = 0; i < N_TAGS; i++)
    {
        g_n[i].role  = ROLE_INIT;
        g_n[i].idx   = (uint8_t)i;
        g_n[i].addr  = (uint16_t)(0x0001u + i);
        g_n[i].clk.ppm       = -18.0 + 11.0 * (double)i;
        g_n[i].clk.phase_dtu = (int64_t)(i * 7919);
        g_n[i].x = 3000.0 + 4000.0 * (double)i;
        g_n[i].y = 5000.0 + 900.0 * (double)(i % 3);
    }
    for (s = 0; s < N_ANCH; s++)
    {
        node_t *a = &g_n[N_TAGS + s];

        a->role  = ROLE_RESP;
        a->idx   = (uint8_t)s;
        a->addr  = (uint16_t)(0x0101u + s);
        a->clk.ppm       = 20.0 - 3.4 * (double)s;
        a->clk.phase_dtu = (int64_t)(s * 104729);
        a->x = 10000.0 + 9000.0 * cos(2.0 * M_PI * (double)s / (double)N_ANCH);
        a->y = 6000.0  + 9000.0 * sin(2.0 * M_PI * (double)s / (double)N_ANCH);
    }
}

static void child_main(unsigned i, int req_fd, int rep_fd)
{
    cfg_t *c;

    ipc_child_init(req_fd, rep_fd, i);
    node_uart_open(g_n[i].outpath);
    node_radio_set_part_id(0xD0000000u | i);

    cfg_defaults(cfg());
    c = cfg();
    *c = g_base;
    c->role    = g_n[i].role;
    c->node_id = g_n[i].addr;
    if (g_n[i].role == ROLE_INIT) { c->frame_index = g_n[i].idx; }
    else                          { c->slot_index  = g_n[i].idx; }

    /* Every anchor provisioned and enabled on every node. */
    {
        unsigned s;
        for (s = 0; s < N_ANCH; s++)
        {
            c->anchor[s].id   = (uint16_t)(0x0101u + s);
            c->anchor[s].slot = (uint8_t)s;
            c->anchor[s].en   = 1;
            c->anchor[s].ant  = 0;
        }
    }

    app_ranging_set(true);

    if (g_n[i].role == ROLE_INIT) { app_run_initiator(); }
    else                          { app_run_responder(); }

    {
        ipc_req_t rq;
        ipc_rep_t rp;
        memset(&rq, 0, sizeof(rq));
        rq.kind = REQ_DONE;
        ipc_round_trip(&rq, &rp);
    }
    _exit(0);
}

/* --- assertions ----------------------------------------------------------- */
static int report(void)
{
    unsigned t, s;
    int      fail = 0;

    printf("\n  tag  min/anch  max/anch  spread  peak err   bias    resp_lost\n");
    for (t = 0; t < N_TAGS; t++)
    {
        uint32_t mn = 0xFFFFFFFFu, mx = 0, rl = 0;
        double   emx = 0.0, bias = 0.0;
        uint32_t nz = 0;

        for (s = 0; s < N_ANCH; s++)
        {
            if (g_pair[t][s] < mn) { mn = g_pair[t][s]; }
            if (g_pair[t][s] > mx) { mx = g_pair[t][s]; }
            if (g_err_max[t][s] > emx) { emx = g_err_max[t][s]; }
            if (g_pair[t][s]) { bias += g_err_sum[t][s] / (double)g_pair[t][s]; nz++; }
            rl += g_flag_resp_lost[t][s];
        }
        if (nz) { bias /= (double)nz; }

        printf("  %3u %9u %9u %7u  %6.1f mm %+6.2f %10u\n",
               t, mn, mx, mx - mn, emx, bias, rl);

        if (mn == 0u)
        {
            printf("    FAIL tag %u: an anchor produced no ranges at all\n", t);
            fail = 1;
        }
        if (g_loss_x100 == 0u && mx != mn)
        {
            printf("    FAIL tag %u: anchors disagree by %u samples\n", t, mx - mn);
            fail = 1;
        }
        if (emx > SIM_QUANT_MM * 3.0)
        {
            printf("    FAIL tag %u: peak error %.1f mm exceeds %.1f mm\n",
                   t, emx, SIM_QUANT_MM * 3.0);
            fail = 1;
        }
        if (fabs(bias) > SIM_QUANT_MM)
        {
            printf("    FAIL tag %u: mean bias %.2f mm\n", t, bias);
            fail = 1;
        }
    }

    printf("\n  superframes %u   transmissions %u   collisions %u   tx_late %u\n",
           g_sf, g_txs, g_collisions, g_tx_late);

    /* Every initiator should hold a mutual lock. The node in frame slot 0 is
     * the one that used to fail: its only chance to overhear a peer is the
     * slot straight after its own frame, and listening for it whenever the
     * frame's work happened to end was a race it lost. */
    {
        unsigned tt;

        printf("\n  frame sync per initiator\n");
        for (tt = 0; tt < N_TAGS; tt++)
        {
            printf("    tag %u  frame slot %u  %7u records  desync %6u  (%.2f%%)\n",
                   tt, tt, g_records[tt], g_desync[tt],
                   g_records[tt] ? 100.0 * (double)g_desync[tt] / (double)g_records[tt]
                                 : 0.0);
        }
        for (tt = 0; tt < N_TAGS; tt++)
        {
            if (g_records[tt] && g_desync[tt] * 100u > g_records[tt] * 5u)
            {
                printf("    FAIL tag %u free-ran on %.1f%% of its records: "
                       "it is not hearing its peers\n", tt,
                       100.0 * (double)g_desync[tt] / (double)g_records[tt]);
                fail = 1;
            }
        }
    }

    if (g_collisions)
    {
        printf("    FAIL %u slot collisions\n", g_collisions);
        fail = 1;
    }
    if (g_zero_anchor)
    {
        printf("    FAIL %u $RNG lines carried anchor_id 0x0000\n", g_zero_anchor);
        fail = 1;
    }
    /* Asserted in both scenarios now. It used to be gated on loss == 0
     * because equation (2)'s fallback left only 4 us to arm the Final, so any
     * lost last Response was guaranteed tx_late; slots.c now gives that branch
     * the same margin as the normal path, so a late transmit is a real defect
     * whether or not frames are being dropped. */
    if (g_tx_late)
    {
        printf("    FAIL %u tx_late (proc_us=%u): Poll %u, Resp %u, "
               "Final %u, Report %u\n", g_tx_late, g_proc_us,
               g_late_kind[0], g_late_kind[1], g_late_kind[2], g_late_kind[3]);
        fail = 1;
    }
    return fail;
}

/* ------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    unsigned i;
    int      fail = 0;
    const char *dir = getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp";

    for (i = 1; i < (unsigned)argc; i++)
    {
        if      (!strncmp(argv[i], "--superframes=", 14)) { g_sf_limit  = (uint32_t)atoi(argv[i]+14); }
        else if (!strncmp(argv[i], "--loss=",         7)) { g_loss_x100 = (uint32_t)(atof(argv[i]+7)*100.0); }
        else if (!strncmp(argv[i], "--proc-us=",     10)) { g_proc_us   = (uint32_t)atoi(argv[i]+10); }
        else if (!strncmp(argv[i], "--uart-baud=",   12)) { g_uart_baud = (uint32_t)atoi(argv[i]+12); }
        else if (!strncmp(argv[i], "--preamble-us=", 14)) { g_preamble_us = (uint32_t)atoi(argv[i]+14); }
    }

    cfg_defaults(&g_base);
    g_base.resp_count    = (uint8_t)N_ANCH;
    g_base.init_count    = (uint8_t)N_TAGS;
    g_base.report_enable = 1;
    g_base.ant_delay_tx  = 0;
    g_base.ant_delay_rx  = 0;
    g_base.node_id       = 0x0001u;
    g_base.autostart     = 1;
    g_base.proc_poll_us  = g_proc_us;
    g_base.frame_dur_us  = timing_frame_dur_us(&g_base);

    place_nodes();

    printf("fork harness: %u initiators + %u responders, real role loops\n",
           N_TAGS, N_ANCH);
    printf("T_f = %u us   T_F = %u us   nominal %.3f Hz per tag per anchor\n",
           g_base.frame_dur_us, timing_superframe_us(&g_base),
           1.0e6 / (double)timing_superframe_us(&g_base));
    printf("superframes %u   loss %.2f%%/hop   proc %u us\n",
           g_sf_limit, (double)g_loss_x100 / 100.0, g_proc_us);

    for (i = 0; i < N_NODES; i++)
    {
        int down[2], up[2];

        snprintf(g_n[i].outpath, sizeof(g_n[i].outpath),
                 "%s/harness_node_%02u.txt", dir, i);

        if (pipe(down) || pipe(up)) { perror("pipe"); return 2; }

        g_n[i].pid = fork();
        if (g_n[i].pid < 0) { perror("fork"); return 2; }

        if (g_n[i].pid == 0)
        {
            close(down[0]); close(up[1]);
            child_main(i, down[1], up[0]);
            _exit(0);
        }
        close(down[1]); close(up[0]);
        g_n[i].req_fd = down[0];
        g_n[i].rep_fd = up[1];
        g_n[i].alive  = true;
        g_n[i].park   = P_RUNNING;
        g_n[i].armed  = -1;
    }

    /* Barrier, step, repeat. */
    for (;;)
    {
        bool any_alive = false;

        for (i = 0; i < N_NODES; i++)
        {
            pump(&g_n[i]);
            if (g_n[i].alive) { any_alive = true; }
        }
        if (!any_alive) { break; }
        if (!step())    { break; }
    }

    for (i = 0; i < N_NODES; i++)
    {
        close(g_n[i].req_fd);
        close(g_n[i].rep_fd);
    }
    for (i = 0; i < N_NODES; i++)
    {
        int st;
        if (g_n[i].pid > 0) { (void)waitpid(g_n[i].pid, &st, 0); }
    }

    parse_output();
    fail = report();

    printf("\n%s\n", fail ? "FAIL" : "PASS");
    return fail;
}
