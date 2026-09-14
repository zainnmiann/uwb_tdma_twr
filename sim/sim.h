/**
 * @file    sim.h
 * @brief   Virtual clock and air medium for the host simulator.
 *
 * Global time is kept in device time units (1 DTU = 15.650040 ps) on a single
 * int64 timeline. Every node has its own crystal offset and phase, so the
 * timestamps a node reports are its own view of that timeline -- which is what
 * makes the section 6 synchronization and the section 4 estimators do real
 * work rather than agree trivially.
 */
#ifndef SIM_H_
#define SIM_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "cfg.h"
#include "timing.h"

/* --- propagation ---------------------------------------------------------- */
#define SIM_C_MM_PER_S      299702547000.0
#define SIM_DTU_PER_S_D     (1.0 / 15.650040e-12)
#define SIM_DTU_PER_S       SIM_DTU_PER_S_D

/** One-way flight time over @p mm, in global DTU. */
static inline int64_t sim_prop_dtu(double mm)
{
    return (int64_t)llround((mm / SIM_C_MM_PER_S) * SIM_DTU_PER_S_D);
}

/* One node's clock: a mean frequency error, a slow sinusoidal drift on top of
 * it, and a constant phase offset. The drift matters -- with static offsets
 * alone the whole system is exactly self-consistent, equation (9) predicts
 * every alarm with zero residual, and the synchronization is never actually
 * exercised. Real crystals wander with temperature. */
typedef struct
{
    double  ppm;            /**< mean crystal error, parts per million  */
    double  drift_ppm;      /**< peak of the sinusoidal component, ppm   */
    double  drift_hz;       /**< how fast that component cycles         */
    int64_t phase_dtu;      /**< offset of this clock from global time   */
} nclk_t;

/* Elapsed local time is the integral of the instantaneous rate, so the
 * sinusoidal rate term integrates to a (1 - cos)/w displacement. */
static inline double sim_clk_disp_dtu(const nclk_t *c, double t_s)
{
    double d = c->ppm * 1e-6 * t_s;

    if (c->drift_ppm != 0.0 && c->drift_hz != 0.0)
    {
        double w = 2.0 * M_PI * c->drift_hz;

        d += c->drift_ppm * 1e-6 * (1.0 - cos(w * t_s)) / w;
    }
    return d * SIM_DTU_PER_S_D;
}

/** This node's reading of global time @p g, as a full-width signed count. */
static inline int64_t sim_local_dtu(const nclk_t *c, int64_t g)
{
    double t_s = (double)g / SIM_DTU_PER_S_D;

    return c->phase_dtu + (int64_t)llround((double)g + sim_clk_disp_dtu(c, t_s));
}

/** This node's reading of global time @p g, masked to the 40-bit counter. */
static inline uint64_t sim_local_ts40(const nclk_t *c, int64_t g)
{
    return (uint64_t)sim_local_dtu(c, g) & DTU_MASK_40;
}

/** Global time at which this node's clock reads @p l. Inverse of the above.
 *  The drift term makes this non-closed-form, so it is solved by a few
 *  Newton steps -- the rate is within a few ppm of unity, so it converges
 *  immediately. */
static inline int64_t sim_global_from_local(const nclk_t *c, int64_t l)
{
    double target = (double)(l - c->phase_dtu);
    double g      = target;
    int    it;

    for (it = 0; it < 6; it++)
    {
        double t_s = g / SIM_DTU_PER_S_D;
        double f   = g + sim_clk_disp_dtu(c, t_s) - target;

        g -= f;                     /* d/dg is 1 + O(ppm); plain iteration */
        if (fabs(f) < 0.25) { break; }
    }
    return (int64_t)llround(g);
}

/** Global time at which this node's hi32 counter reaches @p hi32, at or after
 *  @p after. The hi32 view wraps every 17.2 s, so the caller supplies the
 *  epoch to disambiguate. */
int64_t sim_global_from_hi32(const nclk_t *c, uint32_t hi32, int64_t after);

/* --- the radio stub's notion of "which node is running" ------------------- */
/** sync.c calls radio_systime_hi32(); point it at a node before stepping it. */
void sim_bind_clock(const nclk_t *c, int64_t now_dtu);

#endif /* SIM_H_ */
