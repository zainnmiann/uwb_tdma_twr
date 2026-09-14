/**
 * @file    timing.h
 * @brief   Section 7 -- air-time model, delay tables and the frame budget.
 */

#ifndef TIMING_H_
#define TIMING_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The five operating points of Table A. */
typedef enum
{
    OPPOINT_C1 = 0,     /* 64 MHz / 6800 kbps / 128 sym */
    OPPOINT_C2,         /* 64 MHz / 6800 kbps /  64 sym */
    OPPOINT_C3,         /* 64 MHz /  850 kbps / 256 sym */
    OPPOINT_C4,         /* 16 MHz /  850 kbps / 512 sym */
    OPPOINT_C5,         /* 64 MHz /  110 kbps /1024 sym */
    OPPOINT_COUNT,
    OPPOINT_NONE = 0xFF
} oppoint_t;

/** One row of Table A, exactly as printed in the spec. Durations in ns. */
typedef struct
{
    const char *id;
    uint8_t     prf;            /* cfg_prf_t   */
    uint8_t     rate;           /* cfg_rate_t  */
    uint16_t    preamble;
    uint32_t    shr_ns;
    uint32_t    phr_ns;
    uint32_t    data_poll_ns;   /* Poll   16 B */
    uint32_t    data_report_ns; /* Report 24 B */
    uint32_t    data_resp_ns;   /* Resp   32 B */
    uint32_t    data_final_ns;  /* Final  48 B */
    uint32_t    tot_poll_ns;
    uint32_t    tot_report_ns;
    uint32_t    tot_resp_ns;
    uint32_t    tot_final_ns;
} table_a_row_t;

/** Table A as published (predicted column). */
const table_a_row_t *timing_table_a(oppoint_t p);

/** Which Table A row, if any, the current radio configuration corresponds to. */
oppoint_t timing_oppoint(const cfg_t *c);

/**
 * Equation (11): on-air duration of a frame carrying @p n_bytes of payload
 * (excluding the FCS, which the model adds), in nanoseconds.
 *
 *   T_air(n_b) = (N_pre + N_SFD) T_psym  +  T_PHR  +  8(n_b+2)/R * eta_RS
 *
 * with T_psym = 1.01763 us at 64 MHz PRF and 0.99359 us at 16 MHz, N_SFD = 8
 * at 850 kbps and 6.8 Mbps and 64 at 110 kbps, and eta_RS = 1 + 48/330.
 */
uint32_t timing_air_ns(const cfg_t *c, uint32_t n_bytes);

/** The four message lengths of the current configuration, payload + FCS. */
uint32_t timing_len_poll(const cfg_t *c);
uint32_t timing_len_resp(const cfg_t *c);
uint32_t timing_len_final(const cfg_t *c);
uint32_t timing_len_report(const cfg_t *c);

/**
 * Air time actually used for the budget. Section 7.2: "the developer fills the
 * measured column ... and all later tables are derived from the measured
 * values", so a non-zero measured override wins over the model.
 */
/** Duration of the preamble + SFD, i.e. everything before the RX RMARKER. */
uint32_t timing_shr_ns(const cfg_t *c);
/** Everything after the RMARKER: PHR + payload. @p air_ns is a timing_air_*. */
uint32_t timing_tail_ns(const cfg_t *c, uint32_t air_ns);

uint32_t timing_air_poll_ns(const cfg_t *c);
uint32_t timing_air_resp_ns(const cfg_t *c);
uint32_t timing_air_final_ns(const cfg_t *c);
uint32_t timing_air_report_ns(const cfg_t *c);

/** Table B, equations (12) and (13). Microseconds, rounded up. */
typedef struct
{
    uint32_t delta_r_us;        /* eq (12) */
    uint32_t delta_p_us;        /* eq (12) */
    uint32_t t_r0_us;           /* eq (13), minimum */
    uint32_t t_p0_us;           /* eq (13), minimum */
    uint32_t guard_us;
    uint32_t frame_dur_us;      /* eq (14) */
} table_b_t;

void timing_table_b(const cfg_t *c, table_b_t *out);

/** Equation (14) evaluated with the configured delays. Microseconds. */
uint32_t timing_frame_dur_us(const cfg_t *c);

/** Superframe duration T_F = N_I * T_f, microseconds. */
uint32_t timing_superframe_us(const cfg_t *c);

/** Section 7.5 ranging rate f_r = 1/(D * N_I * T_f), in milli-hertz. */
uint32_t timing_rate_mhz(const cfg_t *c, uint8_t div);

/** Equation (10): the guard interval the alarm error demands, microseconds. */
uint32_t timing_guard_required_us(const cfg_t *c, uint32_t eps_ppm, uint32_t jitter_us);

/** Overwrite the configured delays with the Table B recommendation. */
void timing_apply_table_b(cfg_t *c);

/* --- device-time conversion ---------------------------------------------- */
/* The DW1000 delayed transmit/receive registers take the top 32 bits of the
 * 40-bit device clock: one unit is 512/499.2 us / 256 = 4.0064 ns. */

#define DTU_MASK_40             0xFFFFFFFFFFULL
/** Device time units per microsecond, x1000 to keep the integer maths exact. */
#define DTU_PER_US_X1000        63897600ULL     /* 63897.6 DTU per us */

static inline uint32_t us_to_hi32(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * DTU_PER_US_X1000) / (1000ULL * 256ULL));
}
static inline uint32_t ns_to_hi32(uint32_t ns)
{
    return (uint32_t)(((uint64_t)ns * DTU_PER_US_X1000) / (1000000ULL * 256ULL));
}
static inline uint64_t us_to_dtu(uint32_t us)
{
    return ((uint64_t)us * DTU_PER_US_X1000) / 1000ULL;
}
static inline uint32_t dtu_to_ps(uint64_t dtu)
{
    /* 1 DTU = 15.650040 ps, rounded to nearest -- truncating biased every
     * converted tau low. */
    return (uint32_t)((dtu * 1565004ULL + 50000ULL) / 100000ULL);
}

#ifdef __cplusplus
}
#endif

#endif /* TIMING_H_ */
