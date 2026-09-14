#include "serial.h"
#include "cfg.h"
#include "app.h"
#include "timing.h"
#include "radio.h"
#include "output.h"
#include "uart_io.h"
#include "sync.h"
#include "port_sys.h"
#include "board_defaults.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------------- */
/* Key table                                                                  */

typedef enum
{
    T_U8, T_U16, T_U32, T_BOOL, T_HEX16, T_HEX32, T_STR,
    T_E_ROLE, T_E_PRF, T_E_RATE, T_E_SFD, T_E_TXP, T_E_OUT, T_E_IFACE
} ktype_t;

typedef struct
{
    const char *name;
    ktype_t     type;
    uint16_t    off;
    uint32_t    min;
    uint32_t    max;
    uint8_t     slen;       /**< buffer size for T_STR   */
    uint8_t     live;       /**< settable while ranging  */
} cfgkey_t;

#define K(n, t, f, lo, hi)      { n, t, (uint16_t)offsetof(cfg_t, f), lo, hi, 0, 0 }
#define KL(n, t, f, lo, hi)     { n, t, (uint16_t)offsetof(cfg_t, f), lo, hi, 0, 1 }
#define KS(n, f, l)             { n, T_STR, (uint16_t)offsetof(cfg_t, f), 0, 0, l, 0 }

static const cfgkey_t k_keys[] = {
    /* Role and identity */
    K ("role",            T_E_ROLE,  role,            0, 2),
    K ("node_id",         T_HEX16,   node_id,         1, 0xFFFEu),
    K ("pan_id",          T_HEX16,   pan_id,          0, 0xFFFFu),
    K ("frame_index",     T_U8,      frame_index,     0, CFG_MAX_INIT - 1u),
    K ("slot_index",      T_U8,      slot_index,      0, CFG_MAX_RESP - 1u),

    /* Network size */
    K ("init_count",      T_U8,      init_count,      1, CFG_MAX_INIT),
    K ("resp_count",      T_U8,      resp_count,      2, CFG_MAX_RESP),

    /* Timing */
    K ("frame_dur_us",    T_U32,     frame_dur_us,    1000, 200000),
    K ("resp_base_us",    T_U32,     resp_base_us,     100,  20000),
    K ("resp_pitch_us",   T_U32,     resp_pitch_us,    100,  20000),
    K ("report_base_us",  T_U32,     report_base_us,   100,  20000),
    K ("report_pitch_us", T_U32,     report_pitch_us,  100,  20000),
    K ("guard_us",        T_U32,     guard_us,          20,   5000),
    K ("rate_div",        T_U8,      rate_div,           1,     64),
    K ("rx_timeout_us",   T_U32,     rx_timeout_us,    100,  65000),
    K ("sync_mmax",       T_U8,      sync_mmax,          1,    255),

    /* Processing budget, feeds equations (12) and (13) */
    K ("proc_us",         T_U32,     proc_us,            0,  20000),
    K ("isr_us",          T_U32,     isr_us,             0,  20000),
    K ("margin_us",       T_U32,     margin_us,          0,  20000),
    K ("proc_poll_us",    T_U32,     proc_poll_us,       0,  20000),
    K ("proc_final_us",   T_U32,     proc_final_us,      0,  20000),
    K ("sync_comp_us",    T_U32,     sync_comp_us,       0,  20000),

    /* Section 7.2 measured air times, nanoseconds; 0 = use the model */
    /* Section 13 step 5 fills these from measurement, so they must be
     * settable without stopping ranging first. */
    KL("air_poll_ns",     T_U32,     air.poll_ns,        0, 20000000),
    KL("air_resp_ns",     T_U32,     air.resp_ns,        0, 20000000),
    KL("air_final_ns",    T_U32,     air.final_ns,       0, 20000000),
    KL("air_report_ns",   T_U32,     air.report_ns,      0, 20000000),

    /* Radio */
    K ("channel",         T_U8,      channel,            1,      7),   /* 6 rejected in cfg_validate */
    K ("prf",             T_E_PRF,   prf,                0,      1),
    K ("preamble_code",   T_U8,      preamble_code,      1,     24),
    K ("preamble_len",    T_U16,     preamble_len,      64,   4096),
    K ("pac_size",        T_U8,      pac_size,           0,     64),
    K ("datarate",        T_E_RATE,  datarate,           0,      2),
    K ("sfd_mode",        T_E_SFD,   sfd_mode,           0,      1),
    K ("txpower_mode",    T_E_TXP,   txpower_mode,       0,      1),
    K ("txpower_coarse",  T_U8,      txpower_coarse,     0,      6),
    K ("txpower_fine",    T_U8,      txpower_fine,       0,     31),
    K ("txpower_reg",     T_HEX32,   txpower_reg,        0, 0xFFFFFFFFu),
    K ("smart_power",     T_BOOL,    smart_power,        0,      1),
    K ("ant_delay_tx",    T_U16,     ant_delay_tx,       0,  65535),
    K ("ant_delay_rx",    T_U16,     ant_delay_rx,       0,  65535),

    /* Operation */
    K ("autostart",       T_BOOL,    autostart,          0,      1),
    K ("report_enable",   T_BOOL,    report_enable,      0,      1),
    KL("out_format",      T_E_OUT,   out_format,         0,      2),
    KL("out_mask",        T_U8,      out_mask,           0,   0xFFu),

    /* Interfaces */
    K ("iface_cfg",       T_E_IFACE, iface_cfg,          0,      3),
    K ("iface_data",      T_E_IFACE, iface_data,         0,      3),
    K ("uart_baud",       T_U32,     uart_baud,       9600, 3000000),
    KS("ble_name",        ble_name,  CFG_NAME_LEN),
    KS("wifi_ssid",       wifi_ssid, CFG_SSID_LEN),
    KS("wifi_pass",       wifi_pass, CFG_SSID_LEN),
};

#define N_KEYS  (sizeof(k_keys) / sizeof(k_keys[0]))

/* ------------------------------------------------------------------------- */

static void ok(const char *key, const char *val)
{
    out_linef("OK,%s,%s", key, val);
}

static void err(int code, const char *key, const char *reason)
{
    out_linef("ERR,%d,%s,%s", code, key, reason);
}

static bool parse_u32(const char *s, uint32_t *out)
{
    char    *end;
    long long v;

    if (s == NULL || *s == '\0') { return false; }
    v = strtoll(s, &end, 0);
    if (*end != '\0' || v < 0 || v > 0xFFFFFFFFLL) { return false; }
    *out = (uint32_t)v;
    return true;
}

static int enum_lookup(const char *s, const char *const *names, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++)
    {
        if (strcmp(s, names[i]) == 0) { return (int)i; }
    }
    return -1;
}

static const char *const k_role[]  = { "INIT", "RESP", "LSTN" };
static const char *const k_prf[]   = { "16", "64" };
static const char *const k_rate[]  = { "110", "850", "6800" };
static const char *const k_sfd[]   = { "STD", "DW" };
static const char *const k_txp[]   = { "AUTO", "MANUAL" };
static const char *const k_out[]   = { "CSV", "JSON", "BIN" };
static const char *const k_iface[] = { "USB", "UART", "BLE", "WIFI" };

static const cfgkey_t *find_key(const char *name)
{
    uint32_t i;
    for (i = 0; i < N_KEYS; i++)
    {
        if (strcmp(name, k_keys[i].name) == 0) { return &k_keys[i]; }
    }
    return NULL;
}

static void key_format(const cfgkey_t *k, char *dst, uint32_t n)
{
    const uint8_t *base = (const uint8_t *)cfg() + k->off;

    switch (k->type)
    {
        case T_U8:   (void)snprintf(dst, n, "%u", *base); break;
        case T_BOOL: (void)snprintf(dst, n, "%u", *base); break;
        case T_U16:  (void)snprintf(dst, n, "%u", *(const uint16_t *)(const void *)base); break;
        case T_U32:  (void)snprintf(dst, n, "%lu", (unsigned long)*(const uint32_t *)(const void *)base); break;
        case T_HEX16:(void)snprintf(dst, n, "0x%04X", *(const uint16_t *)(const void *)base); break;
        case T_HEX32:(void)snprintf(dst, n, "0x%08lX", (unsigned long)*(const uint32_t *)(const void *)base); break;
        case T_STR:  (void)snprintf(dst, n, "%s", (const char *)base); break;
        case T_E_ROLE:  (void)snprintf(dst, n, "%s", k_role[*base % 3u]);  break;
        case T_E_PRF:   (void)snprintf(dst, n, "%s", k_prf[*base % 2u]);   break;
        case T_E_RATE:  (void)snprintf(dst, n, "%s", k_rate[*base % 3u]);  break;
        case T_E_SFD:   (void)snprintf(dst, n, "%s", k_sfd[*base % 2u]);   break;
        case T_E_TXP:   (void)snprintf(dst, n, "%s", k_txp[*base % 2u]);   break;
        case T_E_OUT:   (void)snprintf(dst, n, "%s", k_out[*base % 3u]);   break;
        case T_E_IFACE: (void)snprintf(dst, n, "%s", k_iface[*base % 4u]); break;
        default: dst[0] = '\0'; break;
    }
}

static int key_assign(const cfgkey_t *k, const char *val)
{
    uint8_t *base = (uint8_t *)cfg() + k->off;
    uint32_t v;
    int      e;

    switch (k->type)
    {
        case T_E_ROLE:  e = enum_lookup(val, k_role, 3);  break;
        case T_E_PRF:   e = enum_lookup(val, k_prf, 2);   break;
        case T_E_RATE:  e = enum_lookup(val, k_rate, 3);  break;
        case T_E_SFD:   e = enum_lookup(val, k_sfd, 2);   break;
        case T_E_TXP:   e = enum_lookup(val, k_txp, 2);   break;
        case T_E_OUT:   e = enum_lookup(val, k_out, 3);   break;
        case T_E_IFACE: e = enum_lookup(val, k_iface, 4); break;

        case T_STR:
            if (strlen(val) >= k->slen) { return SERR_OUT_OF_RANGE; }
            strcpy((char *)base, val);
            return 0;

        default:
            if (!parse_u32(val, &v)) { return SERR_OUT_OF_RANGE; }
            if (v < k->min || v > k->max) { return SERR_OUT_OF_RANGE; }
            switch (k->type)
            {
                case T_U8: case T_BOOL:      *base = (uint8_t)v; break;
                case T_U16: case T_HEX16:    *(uint16_t *)(void *)base = (uint16_t)v; break;
                default:                     *(uint32_t *)(void *)base = v; break;
            }
            return 0;
    }

    if (e < 0) { return SERR_OUT_OF_RANGE; }
    *base = (uint8_t)e;
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Commands                                                                   */

static void cmd_cfg(void)
{
    uint32_t i;
    char     val[40];

    out_linef("CFGBEGIN,fw=%s,hash=0x%04X", board_defaults()->fw_ver, cfg_hash(cfg()));
    for (i = 0; i < N_KEYS; i++)
    {
        key_format(&k_keys[i], val, sizeof(val));
        out_linef("C,%s,%s", k_keys[i].name, val);
    }
    out_line("CFGEND");
}

static void cmd_stat(void)
{
    const stats_t *s = app_stats();
    const cfg_t   *c = cfg();

    out_linef("STAT,frames,%lu,polls_tx,%lu,polls_rx,%lu",
              (unsigned long)s->frames, (unsigned long)s->polls_tx,
              (unsigned long)s->polls_rx);
    out_linef("STAT,resp_tx,%lu,resp_rx,%lu,resp_lost,%lu",
              (unsigned long)s->resp_tx, (unsigned long)s->resp_rx,
              (unsigned long)s->resp_lost);
    out_linef("STAT,finals_tx,%lu,finals_rx,%lu,reports_tx,%lu,reports_rx,%lu,reports_lost,%lu",
              (unsigned long)s->finals_tx, (unsigned long)s->finals_rx,
              (unsigned long)s->reports_tx, (unsigned long)s->reports_rx,
              (unsigned long)s->reports_lost);
    out_linef("STAT,whole_frame_lost_run,%lu,whole_frame_lost_flag,%u,finals_lost,%lu",
              (unsigned long)s->whole_frame_lost_run,
              s->whole_frame_lost_flag ? 1u : 0u,
              (unsigned long)s->finals_lost);
    out_linef("STAT,ranges,%lu,rejected,%lu,ss_fallback,%lu,whole_frame_lost,%lu",
              (unsigned long)s->ranges, (unsigned long)s->rejected,
              (unsigned long)s->ss_fallback, (unsigned long)s->whole_frame_lost);
    out_linef("STAT,timeouts,%lu,rx_errors,%lu,tx_late,%lu",
              (unsigned long)s->timeouts, (unsigned long)s->rx_errors,
              (unsigned long)s->tx_late);
    out_linef("STAT,desync,%lu,hash_mismatch,%lu,dup_index,%lu,tx_dropped,%lu",
              (unsigned long)s->desync, (unsigned long)s->hash_mismatch,
              (unsigned long)s->dup_index, (unsigned long)uart_io_dropped());

    if (c->role == ROLE_INIT)
    {
        const sync_t *y = app_sync();
        out_linef("STAT,sync,%s,sfn,%lu,beta_ppb,%ld,err_us,%ld,rearm,%lu,freerun,%lu",
                  sync_state_str(y->state), (unsigned long)y->sfn,
                  (long)y->beta_ppb, (long)y->last_err_us,
                  (unsigned long)y->n_rearm, (unsigned long)y->n_freerun);
    }
    out_linef("STAT,ranging,%u,rx_dropped,%lu",
              app_ranging() ? 1u : 0u, (unsigned long)uart_io_rx_dropped());
}

static void cmd_table_a(void)
{
    uint32_t i;

    out_line("TABLEA,id,prf,rate,pre,msg,shr_ns,phr_ns,data_ns,total_ns");
    for (i = 0; i < OPPOINT_COUNT; i++)
    {
        const table_a_row_t *r = timing_table_a((oppoint_t)i);
        out_linef("TA,%s,%s,%s,%u,Poll,%lu,%lu,%lu,%lu", r->id,
                  (r->prf == PRF_64) ? "64" : "16", cfg_rate_str(r->rate),
                  r->preamble, (unsigned long)r->shr_ns, (unsigned long)r->phr_ns,
                  (unsigned long)r->data_poll_ns, (unsigned long)r->tot_poll_ns);
        out_linef("TA,%s,%s,%s,%u,Report,%lu,%lu,%lu,%lu", r->id,
                  (r->prf == PRF_64) ? "64" : "16", cfg_rate_str(r->rate),
                  r->preamble, (unsigned long)r->shr_ns, (unsigned long)r->phr_ns,
                  (unsigned long)r->data_report_ns, (unsigned long)r->tot_report_ns);
        out_linef("TA,%s,%s,%s,%u,Resp,%lu,%lu,%lu,%lu", r->id,
                  (r->prf == PRF_64) ? "64" : "16", cfg_rate_str(r->rate),
                  r->preamble, (unsigned long)r->shr_ns, (unsigned long)r->phr_ns,
                  (unsigned long)r->data_resp_ns, (unsigned long)r->tot_resp_ns);
        out_linef("TA,%s,%s,%s,%u,Final,%lu,%lu,%lu,%lu", r->id,
                  (r->prf == PRF_64) ? "64" : "16", cfg_rate_str(r->rate),
                  r->preamble, (unsigned long)r->shr_ns, (unsigned long)r->phr_ns,
                  (unsigned long)r->data_final_ns, (unsigned long)r->tot_final_ns);
    }

    /* Model and measured values for the configuration actually loaded. */
    {
        const cfg_t *c = cfg();
        oppoint_t    p = timing_oppoint(c);
        out_linef("TA,now,%s,%s,%u,bytes,%lu,%lu,%lu,%lu",
                  (c->prf == PRF_64) ? "64" : "16", cfg_rate_str(c->datarate),
                  c->preamble_len,
                  (unsigned long)timing_len_poll(c), (unsigned long)timing_len_resp(c),
                  (unsigned long)timing_len_final(c), (unsigned long)timing_len_report(c));
        out_linef("TA,now,model_ns,%lu,%lu,%lu,%lu,oppoint,%s",
                  (unsigned long)timing_air_ns(c, timing_len_poll(c)),
                  (unsigned long)timing_air_ns(c, timing_len_resp(c)),
                  (unsigned long)timing_air_ns(c, timing_len_final(c)),
                  (unsigned long)timing_air_ns(c, timing_len_report(c)),
                  (p == OPPOINT_NONE) ? "none" : timing_table_a(p)->id);
        out_linef("TA,now,used_ns,%lu,%lu,%lu,%lu",
                  (unsigned long)timing_air_poll_ns(c),
                  (unsigned long)timing_air_resp_ns(c),
                  (unsigned long)timing_air_final_ns(c),
                  (unsigned long)timing_air_report_ns(c));
    }
}

static void cmd_table_b(void)
{
    const cfg_t *c = cfg();
    table_b_t    b;

    timing_table_b(c, &b);

    out_linef("TB,delta_r_us,%lu,delta_p_us,%lu,t_r0_us,%lu,t_p0_us,%lu",
              (unsigned long)b.delta_r_us, (unsigned long)b.delta_p_us,
              (unsigned long)b.t_r0_us, (unsigned long)b.t_p0_us);
    out_linef("TB,guard_us,%lu,frame_dur_us_required,%lu,frame_dur_us_configured,%lu",
              (unsigned long)b.guard_us, (unsigned long)b.frame_dur_us,
              (unsigned long)c->frame_dur_us);
    out_linef("TB,guard_required_20ppm_us,%lu",
              (unsigned long)timing_guard_required_us(c, 20u, c->isr_us));
    out_linef("TB,superframe_us,%lu,N_I,%u,N_R,%u",
              (unsigned long)timing_superframe_us(c), c->init_count, c->resp_count);

    /* Section 8.2 makes only eq (14) a hard rule, so eq (10), (12) and (13)
     * are reported as advisory pass/fail rather than refused: a configuration
     * can be legal yet still be running tighter slot pitches than the measured
     * processing budget supports. */
    {
        uint32_t g_req = timing_guard_required_us(c, 20u, c->isr_us);

        out_linef("TB,eq12,%s,configured_dr,%lu,configured_dp,%lu",
                  (c->resp_pitch_us   >= b.delta_r_us
                   && c->report_pitch_us >= b.delta_p_us) ? "ok" : "TIGHT",
                  (unsigned long)c->resp_pitch_us,
                  (unsigned long)c->report_pitch_us);
        out_linef("TB,eq13,%s,configured_Tr0,%lu,configured_Tp0,%lu",
                  (c->resp_base_us   >= b.t_r0_us
                   && c->report_base_us >= b.t_p0_us) ? "ok" : "TIGHT",
                  (unsigned long)c->resp_base_us,
                  (unsigned long)c->report_base_us);
        out_linef("TB,eq14,%s", (c->frame_dur_us >= b.frame_dur_us) ? "ok" : "FAIL");
        out_linef("TB,eq10,%s,required_us,%lu",
                  (c->guard_us >= g_req) ? "ok" : "TIGHT", (unsigned long)g_req);
    }
}


/* Section 13 step 5: report what the critical paths actually cost on this
 * hardware, so Table A/B can be refilled from measurement instead of from the
 * section 7.4 worked example. Compare each measured value against the
 * configured budget it has to fit inside. */
static void cmd_measure(void)
{
    const cfg_t   *c = cfg();
    const stats_t *s = app_stats();

    out_linef("MEAS,proc_resp_us,last,%lu,max,%lu,budget_Tr0,%lu,%s",
              (unsigned long)s->proc_resp_last_us,
              (unsigned long)s->proc_resp_max_us,
              (unsigned long)c->resp_base_us,
              (s->proc_resp_max_us <= c->resp_base_us) ? "fits" : "OVER");
    out_linef("MEAS,proc_rep_us,last,%lu,max,%lu,budget_Tp0,%lu,%s",
              (unsigned long)s->proc_rep_last_us,
              (unsigned long)s->proc_rep_max_us,
              (unsigned long)c->report_base_us,
              (s->proc_rep_max_us <= c->report_base_us) ? "fits" : "OVER");
    out_linef("MEAS,proc_poll_us,last,%lu,max,%lu",
              (unsigned long)s->proc_poll_last_us,
              (unsigned long)s->proc_poll_max_us);
    out_linef("MEAS,tx_late,%lu,note,set-proc_us-from-max-then-AUTOTIME",
              (unsigned long)s->tx_late);
}

static void cmd_rate(void)
{
    const cfg_t *c = cfg();
    static const uint8_t ni[] = { 1, 2, 4, 6, 8, 12, 16, 32 };
    uint32_t i;
    cfg_t    t = *c;

    out_line("RATE,N_I,T_F_us,D=1_mHz,D=2_mHz,D=4_mHz");
    for (i = 0; i < sizeof(ni) / sizeof(ni[0]); i++)
    {
        t.init_count = ni[i];
        out_linef("RT,%u,%lu,%lu,%lu,%lu", ni[i],
                  (unsigned long)timing_superframe_us(&t),
                  (unsigned long)timing_rate_mhz(&t, 1),
                  (unsigned long)timing_rate_mhz(&t, 2),
                  (unsigned long)timing_rate_mhz(&t, 4));
    }
}

static void cmd_anchor(char **p);
static void cmd_tag(char **p);

/* ------------------------------------------------------------------------- */

static char *tok(char **p, char sep)
{
    char *s = *p;
    char *start;

    if (s == NULL || *s == '\0') { *p = s; return NULL; }

    start = s;
    while (*s != '\0' && *s != sep) { s++; }
    if (*s != '\0') { *s = '\0'; s++; }
    *p = s;
    return start;
}

static bool locked(void)
{
    return app_ranging();
}

static void dispatch(char *payload)
{
    char *p = payload;
    char *cmd = tok(&p, ',');

    if (cmd == NULL) { return; }

    if (strcmp(cmd, "SET") == 0)
    {
        char *key = tok(&p, ',');
        char *val = tok(&p, ',');
        const cfgkey_t *k;
        int rc;

        if (key == NULL || val == NULL) { err(SERR_UNKNOWN_KEY, "", "SET,<key>,<value>"); return; }

        k = find_key(key);
        if (k == NULL) { err(SERR_UNKNOWN_KEY, key, "unknown key"); return; }
        if (locked() && !k->live) { err(SERR_READONLY, key, "ranging active"); return; }

        /* key_assign() writes straight into the live configuration, so a value
         * that passes the key's own range check but fails a section 8.2 rule
         * would otherwise stay resident after the ERR -- $CFG would report it,
         * and $SAVE and $RANGE,START would keep refusing with no way back.
         * Snapshot first and restore on rejection. */
        {
            cfg_t saved = *cfg();

            rc = key_assign(k, val);
            if (rc != 0) { *cfg() = saved; err(rc, key, "out of range"); return; }

            {
                const char *bk = "", *br = "";
                int v = cfg_validate(cfg(), &bk, &br);
                if (v != 0) { *cfg() = saved; err(v, bk, br); return; }
            }
        }
        {
            char out[40];
            key_format(k, out, sizeof(out));
            ok(key, out);
        }
        /* uart_io_init() otherwise runs only at boot, so the console kept the
         * old rate while $CFG and $GET reported the new one -- exactly the
         * mismatch cfg_validate()'s baud check exists to prevent. The reply
         * above goes out at the old rate, then the port moves. */
        if (strcmp(key, "uart_baud") == 0)
        {
            uart_io_flush();
            uart_io_init(cfg()->uart_baud);
        }
        return;
    }

    if (strcmp(cmd, "GET") == 0)
    {
        char *key = tok(&p, ',');
        const cfgkey_t *k;
        char out[40];

        if (key == NULL) { err(SERR_UNKNOWN_KEY, "", "GET,<key>"); return; }
        k = find_key(key);
        if (k == NULL) { err(SERR_UNKNOWN_KEY, key, "unknown key"); return; }
        key_format(k, out, sizeof(out));
        ok(key, out);
        return;
    }

    if (strcmp(cmd, "CFG") == 0)      { cmd_cfg();     return; }
    if (strcmp(cmd, "STAT") == 0)
    {
        /* $STAT,CLEAR zeroes the counters so successive measurement
         * runs (section 13 step 5) are comparable without a reboot. */
        char *arg = tok(&p, ',');

        if (arg != NULL && strcmp(arg, "CLEAR") == 0)
        {
            app_stats_clear();
            ok("STAT", "CLEAR");
        }
        else
        {
            cmd_stat();
        }
        return;
    }
    if (strcmp(cmd, "TABLEA") == 0)   { cmd_table_a(); return; }
    if (strcmp(cmd, "TABLEB") == 0)   { cmd_table_b(); return; }
    if (strcmp(cmd, "RATE") == 0)     { cmd_rate();    return; }
    if (strcmp(cmd, "MEASURE") == 0)  { cmd_measure(); return; }

    if (strcmp(cmd, "AUTOTIME") == 0)
    {
        /* Table B is computed from proc/isr/margin, each legal up to 20000 us,
         * so it can easily land outside the validated ranges of the keys it
         * writes. Snapshot and roll back, exactly as $SET does, rather than
         * leaving a configuration that $SAVE and $RANGE,START both refuse
         * while naming keys the operator never typed. */
        cfg_t       saved;
        const char *bk = "", *br = "";
        int         v;

        if (locked()) { err(SERR_READONLY, "AUTOTIME", "ranging active"); return; }

        saved = *cfg();
        timing_apply_table_b(cfg());
        v = cfg_validate(cfg(), &bk, &br);
        if (v != 0) { *cfg() = saved; err(v, bk, br); return; }

        cmd_table_b();
        ok("AUTOTIME", "applied");
        return;
    }

    if (strcmp(cmd, "SAVE") == 0)
    {
        const char *bk = "", *br = "";
        int v;

        /* A flash page erase stalls the CPU for ~22 ms -- several frame
         * durations -- so it must not happen mid-schedule. */
        if (locked()) { err(SERR_READONLY, "SAVE", "ranging active"); return; }

        v = cfg_validate(cfg(), &bk, &br);
        if (v != 0) { err(v, bk, br); return; }
        if (cfg_save()) { ok("SAVE", "1"); }
        else            { err(SERR_NVM, "SAVE", "flash write failed"); }
        return;
    }

    if (strcmp(cmd, "LOAD") == 0)
    {
        if (locked()) { err(SERR_READONLY, "LOAD", "ranging active"); return; }
        if (!cfg_reload()) { err(SERR_NVM, "LOAD", "no valid image"); return; }
        ok("LOAD", "1");
        return;
    }

    if (strcmp(cmd, "DEFAULT") == 0)
    {
        if (locked()) { err(SERR_READONLY, "DEFAULT", "ranging active"); return; }
        cfg_defaults(cfg());
        /* Re-derive the identity the same way boot does, so $DEFAULT leaves a
         * configuration that can actually be started and saved. */
        cfg_apply_identity(cfg(), radio_part_id());
        ok("DEFAULT", "1");
        return;
    }

    if (strcmp(cmd, "RANGE") == 0)
    {
        char *what = tok(&p, ',');
        if (what == NULL) { err(SERR_UNKNOWN_KEY, "RANGE", "START|STOP"); return; }
        if (strcmp(what, "START") == 0)
        {
            const char *bk = "", *br = "";
            int v = cfg_validate(cfg(), &bk, &br);
            if (v != 0) { err(v, bk, br); return; }
            /* Radio keys are refused while ranging, so they can only have
             * changed since the last start. Push them to the DW1000 now rather
             * than silently running on stale hardware settings -- but only when
             * actually starting: dwt_forcetrxoff()/dwt_configure() on a live
             * node mid-frame is not a reconfiguration, it is a glitch. */
            if (!app_ranging())
            {
                (void)radio_reconfigure(cfg());
            }
            /* A duplicate-index fault latches; restarting is the operator's
             * signal that the offending node has been dealt with. */
            sync_clear_fault((sync_t *)app_sync());
            /* Re-acquire rather than trusting a stale alarm or cold deadline. */
            app_request_resync();
            app_ranging_set(true);
            ok("RANGE", "START");
            return;
        }
        if (strcmp(what, "STOP") == 0)
        {
            app_ranging_set(false);
            radio_idle();
            ok("RANGE", "STOP");
            return;
        }
        err(SERR_OUT_OF_RANGE, "RANGE", "START|STOP");
        return;
    }

    if (strcmp(cmd, "ANCHOR") == 0)   { cmd_anchor(&p); return; }
    if (strcmp(cmd, "TAG") == 0)      { cmd_tag(&p);    return; }

    if (strcmp(cmd, "LIST") == 0)
    {
        char *what = tok(&p, ',');
        uint32_t i;

        if (what != NULL && strcmp(what, "ANCHOR") == 0)
        {
            for (i = 0; i < CFG_MAX_RESP; i++)
            {
                const cfg_anchor_t *a = &cfg()->anchor[i];
                out_linef("A,%lu,0x%04X,%u,%ld,%ld,%ld,%u,%u", (unsigned long)i,
                          a->id, a->slot, (long)a->x_mm, (long)a->y_mm,
                          (long)a->z_mm, (int)a->ant, a->en);
            }
            return;
        }
        if (what != NULL && strcmp(what, "TAG") == 0)
        {
            for (i = 0; i < CFG_MAX_INIT; i++)
            {
                const cfg_tag_t *t = &cfg()->tag[i];
                out_linef("T,%lu,0x%04X,%u,%u,%u", (unsigned long)i,
                          t->id, t->frame, t->div, t->en);
            }
            return;
        }
        err(SERR_OUT_OF_RANGE, "LIST", "ANCHOR|TAG");
        return;
    }

    if (strcmp(cmd, "RESET") == 0)
    {
        ok("RESET", "1");
        uart_io_flush();
        port_reset();
        return;
    }

    err(SERR_UNKNOWN_KEY, cmd, "unknown command");
}

static void cmd_anchor(char **p)
{
    /* Signed: x/y/z are int32 and the antenna trim is a signed correction, so
     * parsing through uint32 silently turned "-100" into 65436 DTU and put a
     * 307 m offset on that responder for good, reporting OK. */
    long f[8];
    uint32_t i;
    cfg_anchor_t *a;

    if (locked()) { err(SERR_READONLY, "ANCHOR", "ranging active"); return; }

    for (i = 0; i < 8u; i++)
    {
        char *t = tok(p, ',');
        long  v;
        char *end;

        if (t == NULL) { err(SERR_OUT_OF_RANGE, "ANCHOR", "i,id,slot,x,y,z,ant,en"); return; }
        v = strtol(t, &end, 0);
        if (*end != '\0') { err(SERR_OUT_OF_RANGE, "ANCHOR", "bad field"); return; }
        f[i] = v;
    }

    if (f[0] < 0 || f[0] >= (long)CFG_MAX_RESP)
                               { err(SERR_OUT_OF_RANGE, "ANCHOR", "index");  return; }
    if (f[1] < 0 || f[1] > 0xFFFF)
                               { err(SERR_OUT_OF_RANGE, "ANCHOR", "id");     return; }
    if (f[2] < 0 || f[2] >= (long)CFG_MAX_RESP)
                               { err(SERR_OUT_OF_RANGE, "ANCHOR", "slot");   return; }
    if (f[6] < -32768 || f[6] > 32767)
                               { err(SERR_OUT_OF_RANGE, "ANCHOR", "ant range -32768..32767"); return; }
    if (f[7] < 0 || f[7] > 1)  { err(SERR_OUT_OF_RANGE, "ANCHOR", "en");     return; }

    a = &cfg()->anchor[f[0]];
    a->id    = (uint16_t)f[1];
    a->slot  = (uint8_t)f[2];
    a->x_mm  = (int32_t)f[3];
    a->y_mm  = (int32_t)f[4];
    a->z_mm  = (int32_t)f[5];
    a->ant   = (int16_t)f[6];
    a->en    = (uint8_t)(f[7] ? 1u : 0u);

    ok("ANCHOR", "1");
}

static void cmd_tag(char **p)
{
    uint32_t f[5];
    uint32_t i;
    cfg_tag_t *t;

    if (locked()) { err(SERR_READONLY, "TAG", "ranging active"); return; }

    for (i = 0; i < 5u; i++)
    {
        char *s = tok(p, ',');
        long  v;
        char *end;

        if (s == NULL) { err(SERR_OUT_OF_RANGE, "TAG", "j,id,frame,div,en"); return; }
        v = strtol(s, &end, 0);
        if (*end != '\0') { err(SERR_OUT_OF_RANGE, "TAG", "bad field"); return; }
        f[i] = (uint32_t)v;
    }

    if (f[0] >= CFG_MAX_INIT) { err(SERR_OUT_OF_RANGE, "TAG", "index"); return; }
    if (f[2] >= CFG_MAX_INIT) { err(SERR_OUT_OF_RANGE, "TAG", "frame"); return; }
    if (f[3] < 1u || f[3] > 64u) { err(SERR_OUT_OF_RANGE, "TAG", "div"); return; }

    t = &cfg()->tag[f[0]];
    t->id    = (uint16_t)f[1];
    t->frame = (uint8_t)f[2];
    t->div   = (uint8_t)f[3];
    t->en    = (uint8_t)(f[4] ? 1u : 0u);

    ok("TAG", "1");
}

/* ------------------------------------------------------------------------- */

void serial_init(void)
{
    /* USB and UART both resolve to the DWM1001-DEV J-Link virtual COM port;
     * the module carries no USB device controller of its own. */
    uart_io_init(cfg()->uart_baud);
}

void serial_banner(void)
{
    const cfg_t *c = cfg();

    out_linef("I,fw=%s,proto=%u,dev=0x%08lX,part=0x%08lX", board_defaults()->fw_ver, 1u,
              (unsigned long)radio_device_id(), (unsigned long)radio_part_id());
    out_linef("I,role=%s,node=0x%04X,pan=0x%04X,hash=0x%04X",
              cfg_role_str(c->role), c->node_id, c->pan_id, cfg_hash(c));
    out_linef("I,txpower=0x%08lX,%s,ant_tx=%u,ant_rx=%u",
              (unsigned long)radio_txpower_reg(),
              radio_txpower_from_otp() ? "otp" : "table",
              radio_ant_tx(), radio_ant_rx());

    if (c->iface_data > IFACE_UART)
    {
        err(SERR_OUT_OF_RANGE, "iface_data", "no BLE/WiFi radio on this build, using UART");
    }
    if (c->iface_cfg > IFACE_UART)
    {
        err(SERR_OUT_OF_RANGE, "iface_cfg", "no BLE/WiFi radio on this build, using UART");
    }
}

void serial_poll(void)
{
    char *line = uart_io_poll_line();
    char *body;
    char *star;

    if (line == NULL) { return; }

    /* "$" payload "*" CS -- the checksum is verified when present. */
    body = (line[0] == '$') ? line + 1 : line;

    star = strchr(body, '*');
    if (star != NULL)
    {
        uint8_t want = (uint8_t)strtol(star + 1, NULL, 16);
        uint8_t have = 0;
        char   *q = body;

        *star = '\0';
        while (q < star) { have ^= (uint8_t)*q++; }

        if (star[1] != '\0' && have != want)
        {
            err(SERR_OUT_OF_RANGE, "cs", "checksum mismatch");
            return;
        }
    }

    dispatch(body);
}
