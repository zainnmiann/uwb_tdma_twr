#include "sim_trace.h"
#include "timing.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define DTU_PER_NS      63.8976
#define MAX_TAGS        8u
#define MAX_ANCH        16u
#define LIVE_ROWS       (MAX_TAGS + MAX_ANCH)
#define STRIP_W         96u
#define ANOM_KEEP       8u

static FILE    *s_f;
static uint32_t s_window;
static bool     s_live;
static double   s_realtime;
static bool     s_first_ev = true;
static bool     s_first_node = true;
static bool     s_first_rng = true;
static bool     s_first_tag = true;
static uint32_t s_sfn;
static int64_t  s_sf_t0;
static uint32_t s_n_tags, s_n_anch;
static uint32_t s_tf_us, s_tF_us;
static const char *s_scenario = "";

/* --- live view state ------------------------------------------------------ */
static char     s_strip[LIVE_ROWS][STRIP_W + 1];
static uint32_t s_count[MAX_TAGS][MAX_ANCH];
static char     s_state[MAX_TAGS][12];
static int32_t  s_err_us[MAX_TAGS];
static uint32_t s_rearm[MAX_TAGS], s_drift[MAX_TAGS];
static char     s_anom[ANOM_KEEP][120];
static uint32_t s_anom_n;
static struct timespec s_wall0;
static double   s_last_draw;
static bool     s_drew;

const char *trace_kind_str(tkind_t k)
{
    switch (k)
    {
        case TK_POLL:   return "Poll";
        case TK_RESP:   return "Resp";
        case TK_FINAL:  return "Final";
        default:        return "Report";
    }
}

static char kind_ch(tkind_t k)
{
    switch (k)
    {
        case TK_POLL:   return 'P';
        case TK_RESP:   return 'r';
        case TK_FINAL:  return 'F';
        default:        return 'p';
    }
}

static double wall_s(void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)(t.tv_sec - s_wall0.tv_sec)
         + (double)(t.tv_nsec - s_wall0.tv_nsec) / 1e9;
}

static bool in_window(void) { return s_f && s_sfn < s_window; }

static void ev(const char *fmt, ...)
{
    va_list ap;

    if (!in_window()) { return; }
    fprintf(s_f, "%s\n    ", s_first_ev ? "" : ",");
    s_first_ev = false;
    va_start(ap, fmt);
    (void)vfprintf(s_f, fmt, ap);
    va_end(ap);
}

/* --- lifecycle ------------------------------------------------------------ */
void trace_open(const char *path, uint32_t window)
{
    if (window == 0u) { return; }
    if (s_f) { (void)fclose(s_f); }
    /* Re-openable: one file per scenario, so the per-run state resets too. */
    s_first_ev = s_first_node = s_first_rng = s_first_tag = true;
    s_sfn = 0;
    s_f = fopen(path, "w");
    if (!s_f) { fprintf(stderr, "trace: cannot write %s\n", path); return; }
    s_window = window;
}

void trace_live(double realtime)
{
    s_live     = true;
    s_realtime = realtime;
    clock_gettime(CLOCK_MONOTONIC, &s_wall0);
    /* Called once per scenario, so the per-run displays reset with it --
     * otherwise the pair matrix keeps counting across scenarios and shows
     * totals that match neither run's assertions. */
    memset(s_strip, ' ', sizeof(s_strip));
    memset(s_count, 0, sizeof(s_count));
    memset(s_rearm, 0, sizeof(s_rearm));
    memset(s_drift, 0, sizeof(s_drift));
    memset(s_anom,  0, sizeof(s_anom));
    s_anom_n    = 0;
    s_last_draw = 0.0;
    s_drew      = false;
}

bool trace_active(void) { return s_f != NULL || s_live; }

void trace_meta(const cfg_t *c, const char *scenario, uint32_t n_tags,
                uint32_t n_anch, uint64_t seed, uint32_t superframes,
                uint32_t loss_x100)
{
    s_n_tags   = n_tags;
    s_n_anch   = n_anch;
    s_tf_us    = c->frame_dur_us;
    s_tF_us    = timing_superframe_us(c);
    s_scenario = scenario;

    if (!s_f) { return; }
    fprintf(s_f,
        "{\n  \"version\": 1,\n"
        "  \"meta\": {\n"
        "    \"scenario\": \"%s\", \"seed\": %llu,\n"
        "    \"n_tags\": %u, \"n_anch\": %u,\n"
        "    \"superframes\": %u, \"window\": %u, \"loss_pct\": %.2f,\n"
        "    \"t_f_us\": %u, \"t_F_us\": %u, \"nominal_hz\": %.4f,\n"
        "    \"channel\": %u, \"prf\": %u, \"preamble_len\": %u,\n"
        "    \"datarate\": %u, \"resp_base_us\": %u, \"resp_pitch_us\": %u,\n"
        "    \"report_base_us\": %u, \"report_pitch_us\": %u,\n"
        "    \"guard_us\": %u, \"rx_timeout_us\": %u, \"report_enable\": %u,\n"
        "    \"air\": { \"poll_ns\": %u, \"resp_ns\": %u, \"final_ns\": %u, \"report_ns\": %u }\n"
        "  },\n",
        scenario, (unsigned long long)seed, n_tags, n_anch,
        superframes, s_window, (double)loss_x100 / 100.0,
        s_tf_us, s_tF_us, 1.0e6 / (double)s_tF_us,
        c->channel, c->prf, c->preamble_len, c->datarate,
        c->resp_base_us, c->resp_pitch_us, c->report_base_us,
        c->report_pitch_us, c->guard_us, c->rx_timeout_us, c->report_enable,
        timing_air_poll_ns(c), timing_air_resp_ns(c),
        timing_air_final_ns(c), timing_air_report_ns(c));
    fprintf(s_f, "  \"nodes\": [");
}

void trace_node(uint32_t id, uint8_t role, uint16_t addr, double x, double y,
                double ppm, double drift_ppm, double drift_hz)
{
    if (!s_f) { return; }
    fprintf(s_f, "%s\n    {\"id\":%u,\"role\":\"%s\",\"addr\":\"0x%04X\","
                 "\"x_mm\":%.1f,\"y_mm\":%.1f,\"ppm\":%.2f,"
                 "\"drift_ppm\":%.2f,\"drift_hz\":%.4f}",
            s_first_node ? "" : ",", id,
            (role == ROLE_INIT) ? "tag" : "anchor", addr, x, y,
            ppm, drift_ppm, drift_hz);
    s_first_node = false;
}

void trace_nodes_done(void)
{
    if (!s_f) { return; }
    fprintf(s_f, "\n  ],\n  \"events\": [");
}

void trace_superframe(uint32_t sfn, int64_t t_dtu)
{
    s_sfn   = sfn;
    s_sf_t0 = t_dtu;
    if (s_live) { memset(s_strip, ' ', sizeof(s_strip)); }
}

/* --- events --------------------------------------------------------------- */
static void live_mark(uint32_t node, tkind_t kind, int64_t t0, int64_t t1)
{
    uint32_t a, b, i;

    if (!s_live || node >= LIVE_ROWS || s_tF_us == 0u) { return; }

    a = (uint32_t)(((double)(t0 - s_sf_t0) / DTU_PER_NS / 1000.0)
                   / (double)s_tF_us * (double)STRIP_W);
    b = (uint32_t)(((double)(t1 - s_sf_t0) / DTU_PER_NS / 1000.0)
                   / (double)s_tF_us * (double)STRIP_W);
    if (a >= STRIP_W) { return; }
    if (b >= STRIP_W) { b = STRIP_W - 1u; }

    for (i = a; i <= b; i++)
    {
        char c = s_strip[node][i];

        /* An occupied cell that another node also wants is an overlap. */
        s_strip[node][i] = (c == ' ') ? kind_ch(kind) : '!';
    }
}

void trace_tx(int64_t t0, int64_t t1, uint32_t node, tkind_t kind,
              uint32_t slot, uint32_t sfn)
{
    ev("{\"t\":\"tx\",\"t0\":%lld,\"t1\":%lld,\"node\":%u,"
       "\"kind\":\"%s\",\"slot\":%u,\"sfn\":%u}",
       (long long)t0, (long long)t1, node, trace_kind_str(kind), slot, sfn);
    live_mark(node, kind, t0, t1);
}

void trace_loss(int64_t t, uint32_t tx, uint32_t rx, tkind_t kind)
{
    ev("{\"t\":\"loss\",\"t0\":%lld,\"node\":%u,\"rx\":%u,\"kind\":\"%s\"}",
       (long long)t, tx, rx, trace_kind_str(kind));
}

void trace_sync_ev(int64_t t, uint32_t node, const char *state,
                   const char *event, int32_t err_us, int32_t beta_ppb,
                   uint32_t lambda)
{
    ev("{\"t\":\"sync\",\"t0\":%lld,\"node\":%u,\"state\":\"%s\","
       "\"ev\":\"%s\",\"err_us\":%d,\"beta_ppb\":%d,\"lambda\":%u}",
       (long long)t, node, state, event, err_us, beta_ppb, lambda);

    if (s_live && node < MAX_TAGS)
    {
        snprintf(s_state[node], sizeof(s_state[node]), "%s", state);
        s_err_us[node] = err_us;
        if (!strcmp(event, "rearm"))       { s_rearm[node]++; }
        if (!strcmp(event, "drift_reset")) { s_drift[node]++; }
    }
}

static void anom(int64_t t, const char *fmt, ...)
{
    va_list ap;
    char    line[100];

    if (!s_live) { return; }
    va_start(ap, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    snprintf(s_anom[s_anom_n % ANOM_KEEP], 120, "%10.3f ms  %s",
             (double)t / DTU_PER_NS / 1e6, line);
    s_anom_n++;
}

void trace_collision(int64_t t, uint32_t a, uint32_t b, double overlap_us)
{
    ev("{\"t\":\"collision\",\"t0\":%lld,\"node\":%u,\"other\":%u,"
       "\"overlap_us\":%.2f}", (long long)t, a, b, overlap_us);
    anom(t, "COLLISION n%u vs n%u, %.1f us", a, b, overlap_us);
}

void trace_frame_lost(int64_t t, uint32_t tag, uint32_t sfn)
{
    ev("{\"t\":\"frame_lost\",\"t0\":%lld,\"node\":%u,\"sfn\":%u}",
       (long long)t, tag, sfn);
    anom(t, "whole frame lost, tag %u sfn %u", tag, sfn);
}

void trace_range(int64_t t, uint32_t tag, uint32_t anchor, uint32_t slot,
                 uint32_t sfn, int32_t d_mm, double d_true_mm,
                 uint32_t tof_ps, uint8_t flags, const uint64_t ts6[6])
{
    if (in_window())
    {
        fprintf(s_f, "%s\n    {\"t\":\"range\",\"t0\":%lld,\"tag\":%u,"
                     "\"anchor\":\"0x%04X\",\"slot\":%u,\"sfn\":%u,"
                     "\"d_mm\":%d,\"d_true_mm\":%.1f,\"err_mm\":%.2f,"
                     "\"tof_ps\":%u,\"flags\":%u,"
                     "\"ts\":[%llu,%llu,%llu,%llu,%llu,%llu]}",
                s_first_ev ? "" : ",", (long long)t, tag, anchor, slot, sfn,
                d_mm, d_true_mm, (double)d_mm - d_true_mm, tof_ps, flags,
                (unsigned long long)ts6[0], (unsigned long long)ts6[1],
                (unsigned long long)ts6[2], (unsigned long long)ts6[3],
                (unsigned long long)ts6[4], (unsigned long long)ts6[5]);
        s_first_ev = false;
    }
    if (s_live && tag < MAX_TAGS && slot < MAX_ANCH) { s_count[tag][slot]++; }
}

/* --- the live terminal view ---------------------------------------------- */
void trace_live_draw(int64_t now_dtu, bool force)
{
    double   vt = (double)now_dtu / DTU_PER_NS / 1e9;   /* simulated seconds */
    double   wt = wall_s();
    uint32_t t, s, i;

    if (!s_live) { return; }

    /* Pace simulated time onto wall time when asked. */
    if (s_realtime > 0.0)
    {
        double want = vt / s_realtime;

        while (wall_s() < want)
        {
            struct timespec ts = { 0, 2000000 };        /* 2 ms */
            (void)nanosleep(&ts, NULL);
        }
        wt = wall_s();
    }

    /* Redraw is throttled independently of simulated time, so the display is
     * never the bottleneck. */
    if (!force && (wt - s_last_draw) < 0.05) { return; }
    s_last_draw = wt;

    if (s_drew) { printf("\033[%uA", (unsigned)(s_n_tags + s_n_anch + 12u)); }
    s_drew = true;

    printf("\033[2K  %s   sfn %-6u  virtual %8.3f s  wall %6.2f s  %s\n",
           s_scenario, s_sfn, vt, wt,
           s_realtime > 0.0 ? "paced" : "flat out");
    printf("\033[2K  T_f %u us   T_F %u us   one cell = %.1f us\n",
           s_tf_us, s_tF_us, (double)s_tF_us / (double)STRIP_W);
    printf("\033[2K  P=Poll  r=Response  F=Final  p=Report  !=OVERLAP\n");

    for (i = 0; i < s_n_tags + s_n_anch; i++)
    {
        char row[STRIP_W + 1];

        memcpy(row, s_strip[i], STRIP_W);
        row[STRIP_W] = '\0';
        printf("\033[2K  %-7s|%s|\n",
               (i < s_n_tags) ? "tag" : "anchor", row);
    }

    printf("\033[2K\n\033[2K  ranges per tag per anchor\n");
    for (t = 0; t < s_n_tags && t < MAX_TAGS; t++)
    {
        printf("\033[2K  t%u ", t);
        for (s = 0; s < s_n_anch && s < MAX_ANCH; s++)
        {
            printf("%6u", s_count[t][s]);
        }
        printf("   %-7s err %+6d us  rearm %u  drift %u\n",
               s_state[t][0] ? s_state[t] : "-", s_err_us[t],
               s_rearm[t], s_drift[t]);
    }

    printf("\033[2K\n\033[2K  anomalies\n");
    for (i = 0; i < ANOM_KEEP; i++)
    {
        uint32_t k = (s_anom_n > ANOM_KEEP)
                   ? ((s_anom_n - ANOM_KEEP + i) % ANOM_KEEP) : i;

        printf("\033[2K    %s\n", (i < s_anom_n) ? s_anom[k] : "");
    }
    (void)fflush(stdout);
}

/* --- summary -------------------------------------------------------------- */
void trace_summary_pair(uint32_t tag, uint32_t anchor, uint32_t count,
                        double hz, double bias_mm, double peak_mm)
{
    if (!s_f) { return; }
    if (s_first_rng)
    {
        fprintf(s_f, "\n  ],\n  \"pairs\": [");
        s_first_rng = false;
    }
    else
    {
        fprintf(s_f, ",");
    }
    fprintf(s_f, "\n    {\"tag\":%u,\"slot\":%u,\"count\":%u,\"hz\":%.4f,"
                 "\"bias_mm\":%.3f,\"peak_mm\":%.2f}",
            tag, anchor, count, hz, bias_mm, peak_mm);
}

void trace_summary_tag(uint32_t tag, uint32_t polls, double hz,
                       int64_t jitter_ns, const char *state,
                       uint32_t rearm, uint32_t freerun, uint32_t dup,
                       uint32_t drift, int32_t last_err_us)
{
    if (!s_f) { return; }
    if (s_first_tag)
    {
        fprintf(s_f, "\n  ],\n  \"tags\": [");
        s_first_tag = false;
    }
    else
    {
        fprintf(s_f, ",");
    }
    fprintf(s_f, "\n    {\"tag\":%u,\"polls\":%u,\"hz\":%.4f,\"jitter_ns\":%lld,"
                 "\"state\":\"%s\",\"rearm\":%u,\"freerun\":%u,\"dup\":%u,"
                 "\"drift_reset\":%u,\"last_err_us\":%d}",
            tag, polls, hz, (long long)jitter_ns, state,
            rearm, freerun, dup, drift, last_err_us);
}

void trace_summary_run(uint32_t transmissions, uint32_t collisions,
                       double tightest_gap_us, bool pass)
{
    if (!s_f) { return; }
    fprintf(s_f, "\n  ],\n  \"run\": {\"transmissions\":%u,\"collisions\":%u,"
                 "\"tightest_gap_us\":%.2f,\"pass\":%s}\n}\n",
            transmissions, collisions, tightest_gap_us,
            pass ? "true" : "false");
}

void trace_close(void)
{
    if (s_f) { (void)fclose(s_f); s_f = NULL; }
    if (s_live && s_drew) { printf("\n"); }
}
