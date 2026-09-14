#include "timing.h"
#include "proto.h"

#include <string.h>

/* Equation (11) constants. */
#define T_PSYM_PS_64        1017630ULL     /* 1.01763 us at 64 MHz PRF */
#define T_PSYM_PS_16         993590ULL     /* 0.99359 us at 16 MHz PRF */
#define T_PHR_PS_FAST      21500000ULL     /* 21.5 us  at 850 kbps / 6.8 Mbps */
#define T_PHR_PS_110      172300000ULL     /* 172.3 us at 110 kbps */
#define ETA_RS_NUM               378ULL    /* 1 + 48/330 = 378/330 */
#define ETA_RS_DEN               330ULL

/* ------------------------------------------------------------------------- */
/* Table A, section 7.2, exactly as published.                                */

static const table_a_row_t m_table_a[OPPOINT_COUNT] = {
    { "C1", PRF_64, RATE_6800,  128,  138400, 21500,
      21100, 30500, 39900, 58700,   181100, 190500, 199900, 218700 },
    { "C2", PRF_64, RATE_6800,   64,   73300, 21500,
      21100, 30500, 39900, 58700,   116000, 125400, 134800, 153600 },
    { "C3", PRF_64, RATE_850,   256,  268700, 21500,
      169200, 244400, 319600, 469900, 459400, 534600, 609700, 760100 },
    { "C4", PRF_16, RATE_850,   512,  516700, 21500,
      169200, 244400, 319600, 469900, 707400, 782600, 857800, 1008100 },
    { "C5", PRF_64, RATE_110,  1024, 1107200, 172300,
      1353400, 1954900, 2556400, 3759400, 2632900, 3234400, 3835900, 5038900 },
};

const table_a_row_t *timing_table_a(oppoint_t p)
{
    return (p < OPPOINT_COUNT) ? &m_table_a[p] : NULL;
}

oppoint_t timing_oppoint(const cfg_t *c)
{
    uint32_t i;
    for (i = 0; i < OPPOINT_COUNT; i++)
    {
        if (m_table_a[i].prf == c->prf && m_table_a[i].rate == c->datarate
            && m_table_a[i].preamble == c->preamble_len)
        {
            return (oppoint_t)i;
        }
    }
    return OPPOINT_NONE;
}

/* ------------------------------------------------------------------------- */
/* Equation (11).                                                             */

static uint32_t rate_bps(uint8_t rate)
{
    switch (rate)
    {
        case RATE_110: return 110000u;
        case RATE_850: return 850000u;
        default:       return 6800000u;
    }
}

static uint32_t n_sfd(const cfg_t *c)
{
    /* 8 at 850 kbps and 6.8 Mbps, 64 at 110 kbps. */
    return (c->datarate == RATE_110) ? 64u : 8u;
}

uint32_t timing_air_ns(const cfg_t *c, uint32_t n_bytes)
{
    uint64_t t_psym = (c->prf == PRF_16) ? T_PSYM_PS_16 : T_PSYM_PS_64;
    uint64_t shr_ps = ((uint64_t)c->preamble_len + n_sfd(c)) * t_psym;
    uint64_t phr_ps = (c->datarate == RATE_110) ? T_PHR_PS_110 : T_PHR_PS_FAST;
    uint64_t bits   = 8ULL * ((uint64_t)n_bytes + 2ULL);
    uint64_t data_ps;

    data_ps  = (bits * 1000000000000ULL) / (uint64_t)rate_bps(c->datarate);
    data_ps  = (data_ps * ETA_RS_NUM) / ETA_RS_DEN;

    return (uint32_t)((shr_ps + phr_ps + data_ps + 500ULL) / 1000ULL);
}

/* ------------------------------------------------------------------------- */
/* Actual message lengths of the current configuration (payload, no FCS --    */
/* timing_air_ns adds the two FCS bytes itself).                              */

uint32_t timing_shr_ns(const cfg_t *c)
{
    uint64_t t_psym = (c->prf == PRF_16) ? T_PSYM_PS_16 : T_PSYM_PS_64;

    return (uint32_t)((((uint64_t)c->preamble_len + n_sfd(c)) * t_psym + 500ULL)
                      / 1000ULL);
}

uint32_t timing_tail_ns(const cfg_t *c, uint32_t air_ns)
{
    uint32_t shr = timing_shr_ns(c);

    /* Everything that arrives *after* the RX RMARKER: the PHR and the payload.
     * All the delayed-TX instants of equations (1) and (3) are referenced to
     * the RMARKER, but the frame is not fully received -- and the handler not
     * yet running -- until this much later. */
    return (air_ns > shr) ? (air_ns - shr) : 0u;
}

uint32_t timing_len_poll  (const cfg_t *c) { (void)c; return POLL_LEN   - FCS_LEN; }
uint32_t timing_len_resp  (const cfg_t *c) { (void)c; return RESP_LEN   - FCS_LEN; }
uint32_t timing_len_report(const cfg_t *c) { (void)c; return REPORT_LEN - FCS_LEN; }
uint32_t timing_len_final (const cfg_t *c) { return FINAL_LEN(c->resp_count) - FCS_LEN; }

uint32_t timing_air_poll_ns(const cfg_t *c)
{
    return c->air.poll_ns ? c->air.poll_ns : timing_air_ns(c, timing_len_poll(c));
}
uint32_t timing_air_resp_ns(const cfg_t *c)
{
    return c->air.resp_ns ? c->air.resp_ns : timing_air_ns(c, timing_len_resp(c));
}
uint32_t timing_air_final_ns(const cfg_t *c)
{
    return c->air.final_ns ? c->air.final_ns : timing_air_ns(c, timing_len_final(c));
}
uint32_t timing_air_report_ns(const cfg_t *c)
{
    return c->air.report_ns ? c->air.report_ns : timing_air_ns(c, timing_len_report(c));
}

/* ------------------------------------------------------------------------- */
/* Table B, equations (12) to (14).                                           */

static uint32_t ns_to_us_up(uint32_t ns)
{
    return (ns + 999u) / 1000u;
}

void timing_table_b(const cfg_t *c, table_b_t *out)
{
    uint32_t slack = c->proc_us + c->isr_us + c->margin_us;

    out->delta_r_us   = ns_to_us_up(timing_air_resp_ns(c))   + slack;
    out->delta_p_us   = ns_to_us_up(timing_air_report_ns(c)) + slack;
    out->t_r0_us      = ns_to_us_up(timing_air_poll_ns(c))   + c->proc_poll_us;
    out->t_p0_us      = ns_to_us_up(timing_air_final_ns(c))  + c->proc_final_us;
    out->guard_us     = c->guard_us;
    out->frame_dur_us = timing_frame_dur_us(c);
}

uint32_t timing_frame_dur_us(const cfg_t *c)
{
    /* Equation (14):
     *   T_f = T_air(Poll) + T_r0 + (N_R-1) dr + T_air(Resp) + dr
     *       + T_air(Final) + T_p0 + (N_R-1) dp + T_air(Report) + T_g
     *
     * With report_enable = 0 the Report group is removed and T_f shortens by
     * T_p0 + (N_R-1) dp + T_air(Report) (section 8.1). */
    uint32_t nr = (c->resp_count > 0u) ? c->resp_count : 1u;
    uint32_t t;

    t  = ns_to_us_up(timing_air_poll_ns(c));
    t += c->resp_base_us;
    t += (nr - 1u) * c->resp_pitch_us;
    t += ns_to_us_up(timing_air_resp_ns(c));
    t += c->resp_pitch_us;
    t += ns_to_us_up(timing_air_final_ns(c));

    if (c->report_enable)
    {
        t += c->report_base_us;
        t += (nr - 1u) * c->report_pitch_us;
        t += ns_to_us_up(timing_air_report_ns(c));
    }

    t += c->guard_us;
    return t;
}

uint32_t timing_superframe_us(const cfg_t *c)
{
    return c->frame_dur_us * (uint32_t)c->init_count;
}

uint32_t timing_rate_mhz(const cfg_t *c, uint8_t div)
{
    uint64_t period_us = (uint64_t)timing_superframe_us(c) * (uint64_t)(div ? div : 1u);
    if (period_us == 0ULL)
    {
        return 0;
    }
    return (uint32_t)((1000000000ULL) / period_us);   /* mHz */
}

uint32_t timing_guard_required_us(const cfg_t *c, uint32_t eps_ppm, uint32_t jitter_us)
{
    /* Equation (10): T_g ~ 2 eps T_F + tau_max + J_ISR.
     * tau_max is taken as 1 us, i.e. ~300 m of propagation. */
    uint64_t drift_us = (2ULL * (uint64_t)eps_ppm * (uint64_t)timing_superframe_us(c))
                        / 1000000ULL;
    return (uint32_t)drift_us + 1u + jitter_us;
}

void timing_apply_table_b(cfg_t *c)
{
    table_b_t b;

    timing_table_b(c, &b);

    c->resp_pitch_us   = b.delta_r_us;
    c->report_pitch_us = b.delta_p_us;
    if (c->resp_base_us   < b.t_r0_us) { c->resp_base_us   = b.t_r0_us; }
    if (c->report_base_us < b.t_p0_us) { c->report_base_us = b.t_p0_us; }

    c->frame_dur_us = timing_frame_dur_us(c);
}
