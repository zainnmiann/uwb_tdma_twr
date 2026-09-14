/**
 * @file    slots.h
 * @brief   The fixed-slot instants of section 3, as pure arithmetic.
 *
 * Equations (1), (2) and (3) place every transmission of an exchange, and the
 * receive windows and the exchange deadline follow from them. All of it used
 * to live inline inside the blocking role loops, where it could not be
 * exercised without a DW1000. It is factored out here so role_initiator.c,
 * role_responder.c and the host simulator all evaluate the same lines.
 *
 * Every instant is a device-time hi32 value: one unit is 4.0064 ns, and the
 * DW1000 ignores the low 9 bits of DX_TIME, so a delayed transmit lands on an
 * even hi32 boundary (see radio_tx_ts_from_hi32).
 */

#ifndef SLOTS_H_
#define SLOTS_H_

#include <stdint.h>
#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Equation (1): t_B_T,resp = t_B_R,poll + T_r0 + s dr.
 * @param poll_rx_hi32 the responder's hi32 view of the Poll RMARKER.
 * @param slot         this responder's slot index s.
 */
uint32_t slot_resp_hi32(const cfg_t *c, uint32_t poll_rx_hi32, uint8_t slot);

/**
 * Equation (3): t_B_T,rep = t_B_R,final + T_p0 + s dp.
 * @param final_rx_hi32 the responder's hi32 view of the Final RMARKER.
 */
uint32_t slot_report_hi32(const cfg_t *c, uint32_t final_rx_hi32, uint8_t slot);

/**
 * Equation (2): t_A_T,final = t_A_R,resp,N_R-1 + dr, taken from the last
 * Response actually received.
 *
 * When the last slot was NOT the one received, the nominal instant is used
 * instead. That branch is load-bearing rather than defensive: a responder
 * whose Response the initiator missed still transmitted it, so anchoring the
 * Final to an earlier slot's reception walks the Final into the slot of the
 * responder that was missed.
 *
 * @param poll_hi32 the initiator's own armed Poll instant.
 * @param last_resp_rx_hi32 hi32 view of the last Response received.
 * @param last_slot slot index of that Response, or negative if none.
 */
uint32_t slot_final_hi32(const cfg_t *c, uint32_t poll_hi32,
                         uint32_t last_resp_rx_hi32, int32_t last_slot);

/** End of the initiator's Response receive window, from its Poll instant. */
uint32_t slot_resp_window_end_hi32(const cfg_t *c, uint32_t poll_hi32);

/** End of the initiator's Report receive window, from its Final instant. */
uint32_t slot_report_window_end_hi32(const cfg_t *c, uint32_t final_hi32);

/** When a responder gives up waiting for the Final of this exchange. */
uint32_t slot_exchange_deadline_hi32(const cfg_t *c, uint32_t poll_rx_hi32);

/**
 * Start of the window in which to watch for the *next* initiator's Poll,
 * measured from our own Poll instant.
 *
 * Section 6 has every initiator rearm eq (9) off any Poll it overhears, and
 * the one it can always count on is its immediate successor, exactly one
 * frame later -- for the last frame slot that wraps into the next
 * superframe's slot 0. Listening for it whenever the frame's own work happens
 * to finish is a race, because the range records still have to be written
 * out, so this places the window explicitly and opens it a guard early so the
 * receiver is live before the preamble arrives.
 */
uint32_t slot_peer_watch_start_hi32(const cfg_t *c, uint32_t poll_hi32);

/** Length of that watch window: the guard opened early, the Poll's air time,
 *  and one more guard for skew in the peer's own alarm. */
uint32_t slot_peer_watch_us(const cfg_t *c);

#ifdef __cplusplus
}
#endif

#endif /* SLOTS_H_ */
