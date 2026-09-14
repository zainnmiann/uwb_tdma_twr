/**
 * @file    app.h
 * @brief   Node state machines (section 10) and the $STAT counters.
 */

#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"
#include "sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Counters behind $STAT: frames, losses, timeouts, desync events. */
typedef struct
{
    uint32_t frames;            /**< frames owned and transmitted / served    */
    uint32_t polls_tx;
    uint32_t polls_rx;
    uint32_t resp_tx;
    uint32_t resp_rx;
    uint32_t resp_lost;
    uint32_t finals_tx;
    uint32_t finals_rx;
    uint32_t finals_lost;
    uint32_t reports_tx;
    uint32_t reports_rx;
    uint32_t reports_lost;
    uint32_t ranges;
    uint32_t rejected;          /**< outside the plausibility band            */
    uint32_t ss_fallback;
    uint32_t timeouts;
    uint32_t rx_errors;
    uint32_t tx_late;
    uint32_t desync;
    uint32_t hash_mismatch;
    uint32_t dup_index;
    /* Section 7.2 "device-time captures": the measured cost of each critical
     * path, in microseconds, taken from the DW1000 system clock.
     *
     *   proc_resp_*  RMARKER of a received Poll  -> Response delayed TX armed.
     *                Directly comparable to T_r0 (resp_base_us).
     *   proc_rep_*   RMARKER of a received Final -> Report delayed TX armed.
     *                Directly comparable to T_p0 (report_base_us).
     *   proc_poll_*  initiator alarm decision    -> Poll delayed TX armed.
     *                Directly comparable to TX_PREP_MARGIN_US.
     *
     * These are what section 13 step 5 means by filling Table A/B from
     * measurement rather than from the section 7.4 worked example. */
    uint32_t proc_resp_last_us, proc_resp_max_us;
    uint32_t proc_rep_last_us,  proc_rep_max_us;
    uint32_t proc_poll_last_us, proc_poll_max_us;

    uint32_t whole_frame_lost;
    /* Section 11 "Whole frame lost | no Response at all | flag after three
     * consecutive frames". The spec allocates no section 9.4 flag bit for this
     * and prescribes no action beyond "flag", so it is surfaced as a sticky
     * status in $STAT and deliberately does not suppress transmission (unlike
     * the Alarm drift and Duplicate index rows, which do say so explicitly). */
    uint32_t whole_frame_lost_run;      /* current consecutive run    */
    bool     whole_frame_lost_flag;     /* sticky, set at a run of 3  */
} stats_t;

stats_t *app_stats(void);

/** Record one section 7.2 device-time capture into a last/max pair. */
void app_stat_proc(uint32_t *last, uint32_t *max, uint32_t start_hi32);
void     app_stats_clear(void);

/** Ranging active flag, driven by autostart and $RANGE,START|STOP. */
bool     app_ranging(void);
void     app_ranging_set(bool on);

/** Current sync state, for $STAT on an initiator. */
/** Ask the role loop to drop back to COLD and re-acquire (see $RANGE,START). */
void     app_request_resync(void);
/** Consume a pending resync request. */
bool     app_take_resync(void);

const sync_t *app_sync(void);

/** Section 10.1 / 10.2 / listener. None of these return. */
void app_run_initiator(void);
void app_run_responder(void);
void app_run_listener(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
