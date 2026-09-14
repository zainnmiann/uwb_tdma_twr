#include "slots.h"
#include "timing.h"

/* How late the last Response may still arrive and be captured.
 *
 * Two bounded terms: the one-way flight time, which the estimators cap at the
 * ~1000 m plausibility band (3.3 us), and the responder's clock error over the
 * Response group, 40 ppm of roughly 4 ms (0.2 us). 8 us covers both with room
 * to spare while leaving equation (2) nearly all of its Dr - air(Resp) arming
 * budget. */
#define RESP_CAPTURE_SLACK_US   8u

uint32_t slot_resp_hi32(const cfg_t *c, uint32_t poll_rx_hi32, uint8_t slot)
{
    /* Equation (1). */
    return poll_rx_hi32
         + us_to_hi32(c->resp_base_us + (uint32_t)slot * c->resp_pitch_us);
}

uint32_t slot_report_hi32(const cfg_t *c, uint32_t final_rx_hi32, uint8_t slot)
{
    /* Equation (3). */
    return final_rx_hi32
         + us_to_hi32(c->report_base_us + (uint32_t)slot * c->report_pitch_us);
}

uint32_t slot_final_hi32(const cfg_t *c, uint32_t poll_hi32,
                         uint32_t last_resp_rx_hi32, int32_t last_slot)
{
    if (last_slot == (int32_t)(c->resp_count - 1u))
    {
        /* Equation (2). */
        return last_resp_rx_hi32 + us_to_hi32(c->resp_pitch_us);
    }

    /* Nominal instant: see the note in slots.h. */
    return poll_hi32
         + us_to_hi32(c->resp_base_us
                      + (uint32_t)c->resp_count * c->resp_pitch_us);
}

uint32_t slot_resp_window_end_hi32(const cfg_t *c, uint32_t poll_hi32)
{
    /* Ends where the last Response can no longer still be arriving, and not a
     * microsecond later: everything past that instant is the Final's arming
     * budget, which equation (2) sizes as Dr - air(Resp) = 104 us at the
     * defaults.
     *
     * This used to carry a further guard_us/2. That borrowed 100 of those 104
     * us, so whenever the Response mask did not complete -- any lost Response,
     * not just a lost last one -- the loop waited out the whole window and
     * then had 4 us to build an 87-byte Final and arm it. Every such frame was
     * tx_late. The guard belongs at the frame boundary, where equation (14)
     * already spends T_g, not inside the Response group.
     *
     * Nothing is lost by trimming it: a Response is placed off the responder's
     * own reception of the Poll, so its arrival spread is propagation plus a
     * few ppm over ~4 ms -- microseconds, not a hundred of them. Found by the
     * fork harness, which attributed all 37 late transmits to the Final. */
    return poll_hi32
         + us_to_hi32(c->resp_base_us
                      + (uint32_t)(c->resp_count - 1u) * c->resp_pitch_us
                      + RESP_CAPTURE_SLACK_US)
         + ns_to_hi32(timing_air_resp_ns(c));
}

uint32_t slot_report_window_end_hi32(const cfg_t *c, uint32_t final_hi32)
{
    return final_hi32
         + us_to_hi32(c->report_base_us
                      + (uint32_t)(c->resp_count - 1u) * c->report_pitch_us)
         + ns_to_hi32(timing_air_report_ns(c))
         + us_to_hi32(c->guard_us / 2u);
}

uint32_t slot_exchange_deadline_hi32(const cfg_t *c, uint32_t poll_rx_hi32)
{
    /* The nominal Final instant is an RMARKER, but radio_rx() does not return
     * until the whole frame has arrived, so the Final's own air time has to be
     * inside the deadline. Without it, at 110 kbps the Final needs ~3.9 ms of
     * air after its RMARKER against a 900 us rx_timeout_us, so every exchange
     * is reaped before its Final can land and DS-TWR never completes. */
    return poll_rx_hi32
         + us_to_hi32(c->resp_base_us
                      + (uint32_t)c->resp_count * c->resp_pitch_us
                      + c->rx_timeout_us)
         + ns_to_hi32(timing_air_final_ns(c));
}

uint32_t slot_peer_watch_start_hi32(const cfg_t *c, uint32_t poll_hi32)
{
    /* The next initiator polls exactly one frame after we did -- eq (8) with
     * lambda = 1 read in the other direction. */
    return poll_hi32
         + us_to_hi32(c->frame_dur_us)
         - us_to_hi32(c->guard_us);
}

uint32_t slot_peer_watch_us(const cfg_t *c)
{
    /* Round the air time up, as timing.c does throughout: a window short by
     * part of a microsecond can clip the very frame it exists to catch. */
    uint32_t air_us = (timing_air_poll_ns(c) + 999u) / 1000u;

    return 2u * c->guard_us + air_us;
}
