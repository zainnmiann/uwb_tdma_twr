#include "sync.h"
#include "timing.h"
#include "radio.h"

#include <string.h>

/* Drift tracking gain, section 6: beta <- beta + gamma (t~ - t_next)/T_F,
 * gamma ~ 0.05, expressed as a numerator over 1000. */
#define SYNC_GAMMA_NUM      50
#define SYNC_GAMMA_DEN      1000

/* Cold-start listen window, section 10.1: 2 T_F.
 *
 * radio_hi32_diff_us() measures with a signed 32-bit subtraction, so an
 * interval only stays representable below ~8.6 s. cfg_validate permits
 * frame_dur_us 200000 with init_count 32, i.e. T_F up to 6.4 s and 2 T_F up to
 * 12.8 s -- past which the deadline reads as already expired, every initiator
 * independently concludes "nothing heard, I am first", and they all transmit
 * into each other with no listening phase to break the tie. Clamp instead. */
#define SYNC_COLD_MAX_US    4000000u

static uint32_t cold_window_us(const cfg_t *c)
{
    uint32_t tf = timing_superframe_us(c);

    if (tf > (SYNC_COLD_MAX_US / 2u))
    {
        return SYNC_COLD_MAX_US;
    }
    return 2u * tf;
}

void sync_start(sync_t *s, const cfg_t *c)
{
    /* The counters are lifetime diagnostics, not per-acquisition state. Zeroing
     * them here reset $STAT's rearm/freerun/dup_index/desync at precisely the
     * moments the events they count occur -- a drift reset or a stale alarm is
     * what calls sync_start() -- so the operator lost the history exactly when
     * it became interesting. $STAT,CLEAR is the way to zero them. */
    uint32_t n_rearm  = s->n_rearm;
    uint32_t n_free   = s->n_freerun;
    uint32_t n_dup    = s->n_dup;
    uint32_t n_drift  = s->n_drift_reset;

    memset(s, 0, sizeof(*s));
    s->n_rearm       = n_rearm;
    s->n_freerun     = n_free;
    s->n_dup         = n_dup;
    s->n_drift_reset = n_drift;
    s->state = SYNC_COLD;
    s->cold_deadline_hi32 = radio_systime_hi32() + us_to_hi32(cold_window_us(c));
}


void sync_adopt_local_epoch(sync_t *s, const cfg_t *c)
{
    /* Nothing heard in 2 T_F: we are first, the others will align to us.
     *
     * The frame index has to be in here. Section 3 gives frame k the k-th
     * frame of the superframe, so a node that believes it is first -- and
     * therefore that its own epoch defines the superframe -- belongs at offset
     * k T_f from that epoch, not at T_f like everyone else.
     *
     * Without the term, N initiators powered on together are lethal to each
     * other: cold_window_us() is the same for all of them, so they all leave
     * COLD within microseconds (only crystal phase separates them), all place
     * their first Poll one T_f out, and collide. Nobody then hears anybody, so
     * nobody ever rearms, and they re-collide every superframe. At +/-20 ppm
     * they need thousands of superframes to drift a frame width apart. Found
     * by the fork harness, which starts all sixteen nodes at once; the
     * single-process simulator could not see it because it hand-starts one
     * initiator ahead of the others. */
    s->t_next_hi32 = radio_systime_hi32()
                   + us_to_hi32((1u + (uint32_t)c->frame_index)
                                * c->frame_dur_us);
    s->sfn         = 0;
    s->state       = SYNC_FREERUN;
    s->missed      = 0;
}

bool sync_on_poll(sync_t *s, const cfg_t *c, uint32_t t_rx_hi32,
                  uint8_t m, uint16_t sfn_m, bool foreign)
{
    uint8_t  k  = c->frame_index;
    uint8_t  ni = c->init_count;
    uint32_t lambda;
    uint32_t t_pred;
    int32_t  err_us;

    if (ni == 0u)
    {
        return false;
    }

    /* A frame index outside the configured map cannot be reduced meaningfully:
     * the unsigned arithmetic below would wrap and yield a plausible-looking
     * lambda, and any m congruent to our own k would latch SYNC_FAULT for good.
     * frame_index is deliberately not part of cfg_hash(), so the hash check
     * upstream does not filter this. */
    if (m >= ni)
    {
        return false;
    }

    /* Equation (8). */
    lambda = ((uint32_t)k + (uint32_t)ni - (uint32_t)m) % (uint32_t)ni;

    if (lambda == 0u)
    {
        if (foreign)
        {
            /* Duplicate frame index: configuration fault, suppress transmission. */
            s->state = SYNC_FAULT;
            s->n_dup++;
            return false;
        }
        return true;        /* our own frame index, nothing to do */
    }

    /* Equation (9). tau_hat + T_ISR are lumped into sync_comp_us; with polled
     * operation and a hardware RX timestamp both terms are essentially zero. */
    t_pred = t_rx_hi32
           - us_to_hi32(c->sync_comp_us)
           + us_to_hi32(lambda * c->frame_dur_us);

    if (s->state == SYNC_LOCKED || s->state == SYNC_FREERUN)
    {
        err_us = radio_hi32_diff_us(t_pred, s->t_next_hi32);
        s->last_err_us = err_us;

        /* Section 6 drift tracking, applied to the free-running interval. */
        {
            int64_t tf_us = (int64_t)timing_superframe_us(c);
            if (tf_us > 0)
            {
                int64_t d = ((int64_t)err_us * 1000000000LL * SYNC_GAMMA_NUM)
                          / (tf_us * SYNC_GAMMA_DEN);
                s->beta_ppb += (int32_t)d;
                if (s->beta_ppb >  100000) { s->beta_ppb =  100000; }
                if (s->beta_ppb < -100000) { s->beta_ppb = -100000; }
            }
        }

        /* Section 11: alarm drift beyond T_g/2 means we cannot trust the slot
         * boundary any more. Re-enter listening and suppress transmission.
         *
         * This returns immediately rather than falling through: the tail of
         * this function adopts t_pred and re-locks, which would silently undo
         * the reset and adopt the very observation we just rejected. */
        {
            int32_t lim = (int32_t)(c->guard_us / 2u);
            if (err_us > lim || err_us < -lim)
            {
                s->n_drift_reset++;
                s->state = SYNC_COLD;
                s->cold_deadline_hi32 = radio_systime_hi32()
                                      + us_to_hi32(cold_window_us(c));
                return true;
            }
        }
    }

    s->t_next_hi32      = t_pred;
    s->missed           = 0;
    s->heard_since_alarm = true;
    s->n_rearm++;

    /* A shared superframe counter emerges without a manager: our frame follows
     * theirs inside this superframe when k > m, otherwise in the next one. */
    s->sfn = (uint32_t)sfn_m + ((k < m) ? 1u : 0u);

    /* Hearing a valid Poll promotes us out of COLD (section 10.1 LISTEN ->
     * ARMED) and refreshes a LOCKED/FREERUN lock. Only a configuration fault
     * latches. */
    if (s->state != SYNC_FAULT)
    {
        s->state       = SYNC_LOCKED;
        s->ever_locked = true;
    }

    return true;
}

void sync_advance(sync_t *s, const cfg_t *c)
{
    uint32_t tf_us = timing_superframe_us(c);
    uint32_t step  = us_to_hi32(tf_us);

    /* Apply the drift estimate to the free-running interval. */
    if (s->beta_ppb != 0)
    {
        int64_t adj = ((int64_t)step * (int64_t)s->beta_ppb) / 1000000000LL;
        step = (uint32_t)((int64_t)step + adj);
    }

    s->t_next_hi32 += step;
    s->sfn++;
}

void sync_missed_frame(sync_t *s, const cfg_t *c)
{
    if (s->state == SYNC_FAULT)
    {
        return;
    }

    /* Called on every alarm, including alarms where a Poll was just heard and
     * reset the counter. Charging those would make M_max unreachable in one
     * direction and trivially reachable in the other. */
    if (s->heard_since_alarm)
    {
        s->heard_since_alarm = false;
        return;
    }

    /* Section 6 counts M_max consecutive *frames*, but this is reached once per
     * superframe (the initiator's alarm period), during which N_I frame slots
     * elapse -- one per initiator. Counting superframes here would stretch the
     * threshold by a factor of init_count. */
    {
        uint32_t elapsed = (c->init_count > 0u) ? c->init_count : 1u;

        /* Never let one silent superframe alone cross the threshold: with
         * init_count >= sync_mmax (both individually legal) the very first
         * alarm would flip a perfectly locked node into FREERUN and stamp
         * RNG_FLAG_DESYNC on every line thereafter. */
        /* Floor of 1: at the legal setting sync_mmax = 1 the old clamp yielded
         * 0, so s->missed never advanced and SYNC_FREERUN -- and therefore
         * RNG_FLAG_DESYNC -- was unreachable. sync_mmax = 1 means "one missed
         * frame is a desync", the most aggressive setting, so it must cross on
         * the first silent superframe rather than never. */
        if (elapsed >= c->sync_mmax)
        {
            elapsed = (c->sync_mmax > 1u) ? ((uint32_t)c->sync_mmax - 1u) : 1u;
        }
        s->missed += elapsed;
    }

    if (s->missed >= c->sync_mmax)
    {
        if (s->state != SYNC_FREERUN)
        {
            s->n_freerun++;
        }
        /* Section 6: free-run on the local timer with the estimated drift and
         * continue transmitting. */
        s->state = SYNC_FREERUN;
    }
}

bool sync_may_transmit(const sync_t *s)
{
    return (s->state == SYNC_LOCKED) || (s->state == SYNC_FREERUN);
}

bool sync_due_this_superframe(const sync_t *s, const cfg_t *c)
{
    uint8_t d = (c->rate_div > 0u) ? c->rate_div : 1u;
    return (s->sfn % (uint32_t)d) == 0u;
}

const char *sync_state_str(sync_state_t st)
{
    switch (st)
    {
        case SYNC_COLD:    return "COLD";
        case SYNC_LOCKED:  return "LOCKED";
        case SYNC_FREERUN: return "FREERUN";
        default:           return "FAULT";
    }
}

bool sync_is_desynced(const sync_t *s)
{
    return (s->state == SYNC_FREERUN) && s->ever_locked;
}

void sync_clear_fault(sync_t *s)
{
    if (s->state == SYNC_FAULT)
    {
        s->state  = SYNC_COLD;
        s->missed = 0;
    }
}
