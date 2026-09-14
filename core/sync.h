/**
 * @file    sync.h
 * @brief   Section 6 -- initiator frame synchronization.
 *
 * Initiators hold no common time base at start-up and receive no beacon. Each
 * initiator listens to the Poll transmissions of the other initiators and
 * rearms its own frame alarm on every reception. Frame boundaries are defined
 * by the Poll transmission instant, so the offset of the observation is known.
 *
 *   Lambda(k,m) = (k - m) mod N_I                                        (8)
 *   t_next      = t_R - tau_hat - T_ISR + Lambda(k,m) T_f                (9)
 *
 * Every later Poll supersedes the earlier estimate, so the alarm always rests
 * on the most recent observation.
 *
 * Responders need no frame alarm. They stay in continuous reception and derive
 * every transmit instant from the received Poll or Final.
 */

#ifndef SYNC_H_
#define SYNC_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SYNC_COLD = 0,      /**< listening for 2 T_F before deciding      */
    SYNC_LOCKED,        /**< alarm rests on a recent observation      */
    SYNC_FREERUN,       /**< no Poll for M_max frames, running on beta*/
    SYNC_FAULT          /**< duplicate frame index, transmission off  */
} sync_state_t;

typedef struct
{
    sync_state_t state;
    uint32_t     t_next_hi32;   /**< device time of our next Poll        */
    uint32_t     sfn;           /**< superframe counter n                */
    int32_t      beta_ppb;      /**< drift estimate, parts per billion   */
    uint32_t     missed;        /**< consecutive frames with no Poll     */
    int32_t      last_err_us;   /**< t_observed - t_predicted            */
    uint32_t     cold_deadline_hi32;
    uint32_t     n_rearm;
    uint32_t     n_freerun;
    uint32_t     n_dup;
    uint32_t     n_drift_reset;
    /* A Poll from another initiator was heard since the last alarm. Without
     * this, sync_missed_frame() would charge a missed frame on every alarm,
     * including alarms immediately preceded by a successful rearm. */
    bool         heard_since_alarm;
    /* We have been LOCKED to another initiator at least once. A node that
     * adopted its own epoch because nothing was heard is operating exactly as
     * section 6 intends and is not "desynchronized" -- without this the sole
     * initiator in a network stamps RNG_FLAG_DESYNC on 100% of its output. */
    bool         ever_locked;
} sync_t;

/** Enter COLD: listen for 2 T_F starting now. */
void sync_start(sync_t *s, const cfg_t *c);

/** Cold start with nothing heard: we are first, start at the local epoch. */
void sync_adopt_local_epoch(sync_t *s, const cfg_t *c);

/**
 * Apply equations (8) and (9) to a Poll observed from frame index @p m at
 * device time @p t_rx_hi32. @p foreign is true when the source address is not
 * our own. Returns false and raises SYNC_FAULT on a duplicate frame index.
 */
bool sync_on_poll(sync_t *s, const cfg_t *c, uint32_t t_rx_hi32,
                  uint8_t m, uint16_t sfn_m, bool foreign);

/** Advance the alarm one superframe, applying the drift estimate. */
void sync_advance(sync_t *s, const cfg_t *c);

/** Called when a frame elapsed with no Poll heard from anyone. */
void sync_missed_frame(sync_t *s, const cfg_t *c);

/** True when section 9.4's bit4 "desynchronized" genuinely applies: we had a
 *  lock and lost it, as opposed to never having heard anyone. */
bool sync_is_desynced(const sync_t *s);

/** Clear a latched configuration fault, e.g. on $RANGE,START after the
 *  offending node has been reconfigured or removed. */
void sync_clear_fault(sync_t *s);

/** True when this node may transmit (not FAULT, not still COLD). */
bool sync_may_transmit(const sync_t *s);

/** True when this superframe is a transmit superframe for divider D. */
bool sync_due_this_superframe(const sync_t *s, const cfg_t *c);

const char *sync_state_str(sync_state_t st);

#ifdef __cplusplus
}
#endif

#endif /* SYNC_H_ */
