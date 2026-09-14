/**
 * @file    ranging.h
 * @brief   Section 4 -- the ranging estimator.
 */

#ifndef RANGING_H_
#define RANGING_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Speed of light in air, millimetres per second, as used for d = c*tau. */
#define C_AIR_MM_PER_S      299702547000.0

/* Plausibility ceiling on a time of flight, shared by the estimators and by
 * the Report decoder: MAX_TAU_DTU (~1000 m) expressed in picoseconds. */
#define RANGING_MAX_TOF_PS  3333500u

/**
 * The four intervals of equations (4) and (5), in DW1000 device time units.
 *
 *   T_rd^(1) = t_A_R,resp,s - t_A_T,poll      (initiator round 1)
 *   T_rp^(1) = t_B_T,resp   - t_B_R,poll      (responder reply 1)
 *   T_rd^(2) = t_B_R,final  - t_B_T,resp      (responder round 2)
 *   T_rp^(2) = t_A_T,final  - t_A_R,resp,s    (initiator reply 2)
 */
typedef struct
{
    uint64_t poll_tx;       /* t_A_T,poll     */
    uint64_t poll_rx;       /* t_B_R,poll     */
    uint64_t resp_tx;       /* t_B_T,resp     */
    uint64_t resp_rx;       /* t_A_R,resp,s   */
    uint64_t final_tx;      /* t_A_T,final    */
    uint64_t final_rx;      /* t_B_R,final    */
} twr_ts_t;

/**
 * Equation (6): the asymmetric DS-TWR combination.
 *
 *   tau = (T_rd1*T_rd2 - T_rp1*T_rp2) / (T_rd1 + T_rd2 + T_rp1 + T_rp2)
 *
 * The residual clock-induced bias is (e_A + e_B)/2 * tau, below one millimetre
 * for |e| < 40 ppm, so the reply delays may be long and T_r0 / T_p0 may be
 * sized purely from processing time.
 *
 * @return tau in device time units, or negative on an implausible set.
 */
int64_t ranging_tof_dtu(const twr_ts_t *ts);

/**
 * Equation (7): the single-sided fallback available at the initiator from the
 * Response payload alone, used only when the Report is lost.
 *
 *   tau_SS = 1/2 [ T_rd1 - (1 + k_hat) T_rp1 ]
 *
 * @param k_hat carrier frequency offset ratio read from the receiver.
 * @return tau in seconds, or negative on an implausible set.
 */
double ranging_tof_ss_s(uint64_t poll_tx, uint64_t resp_rx,
                        uint64_t poll_rx, uint64_t resp_tx, double k_hat);

/** Carrier frequency offset ratio k_hat for the last reception. */
/** Crystal offset ratio k_hat in the sign convention of equation (7)
 *  (the negation of the driver's clockOffsetRatio). */
double ranging_k_hat(int32_t carrier_integrator, uint8_t channel, uint8_t datarate);

/** tau (device time units) -> picoseconds. */
uint32_t ranging_dtu_to_ps(int64_t tau_dtu);

/**
 * Equation (6) second half: d = c (tau - delta_A - delta_B_s).
 *
 * The DW1000 antenna delay registers are programmed from ant_delay_tx and
 * ant_delay_rx, so the timestamps handed to the estimator are already antenna
 * referenced and the bulk of delta is absorbed there. @p extra_dtu is the
 * residual per-responder trim from the anchor list, applied here.
 */
int32_t ranging_ps_to_mm(uint32_t tau_ps, int32_t extra_dtu);

#ifdef __cplusplus
}
#endif

#endif /* RANGING_H_ */
