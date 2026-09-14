#include "ranging.h"
#include "timing.h"

#include <math.h>
#include "dw1000_consts.h"

/* Plausibility band. A 20 ms interval is far beyond any legal reply delay in
 * this protocol, and 1 km of flight is beyond any DW1000 link. */
#define MAX_INTERVAL_DTU    (1277952000LL)      /* 20 ms  */
#define MAX_TAU_DTU         (213000LL)          /* ~1000 m */

int64_t ranging_tof_dtu(const twr_ts_t *ts)
{
    int64_t rd1, rp1, rd2, rp2, num, den;

    rd1 = (int64_t)((ts->resp_rx  - ts->poll_tx) & DTU_MASK_40);
    rp1 = (int64_t)((ts->resp_tx  - ts->poll_rx) & DTU_MASK_40);
    rd2 = (int64_t)((ts->final_rx - ts->resp_tx) & DTU_MASK_40);
    rp2 = (int64_t)((ts->final_tx - ts->resp_rx) & DTU_MASK_40);

    if (rd1 <= 0 || rp1 <= 0 || rd2 <= 0 || rp2 <= 0)
    {
        return -1;
    }
    if (rd1 > MAX_INTERVAL_DTU || rp1 > MAX_INTERVAL_DTU ||
        rd2 > MAX_INTERVAL_DTU || rp2 > MAX_INTERVAL_DTU)
    {
        return -2;
    }

    /* Worst case 1.28e9^2 = 1.6e18, inside int64. */
    num = (rd1 * rd2) - (rp1 * rp2);
    den = rd1 + rd2 + rp1 + rp2;

    /* Rejected before the division so the rounding below cannot lift a
     * negative numerator into the plausible band. */
    if (num < 0)
    {
        return -1;
    }

    if (den <= 0)
    {
        return -3;
    }

    /* Round to nearest, not toward zero. Truncating here cost on average half
     * a device time unit, and one unit of tau is c * 15.650040 ps = 4.7 mm, so
     * every reported range carried a systematic ~2.4 mm shortfall -- measured
     * as a -3 mm mean bias by the host simulator, together with the two
     * truncations further down the conversion chain. */
    num = (num + den / 2) / den;

    if (num > MAX_TAU_DTU)
    {
        return -4;
    }
    return num;
}

double ranging_k_hat(int32_t carrier_integrator, uint8_t channel, uint8_t datarate)
{
    double hz_to_ppm;
    double freq_mult;

    switch (channel)
    {
        case 1:  hz_to_ppm = HERTZ_TO_PPM_MULTIPLIER_CHAN_1; break;
        case 2:  hz_to_ppm = HERTZ_TO_PPM_MULTIPLIER_CHAN_2; break;
        case 3:  hz_to_ppm = HERTZ_TO_PPM_MULTIPLIER_CHAN_3; break;
        /* Channel 4 shares channel 2's 3993.6 MHz centre frequency, and
         * channel 7 shares channel 5's 6489.6 MHz. */
        case 4:  hz_to_ppm = HERTZ_TO_PPM_MULTIPLIER_CHAN_2; break;
        case 5:
        case 7:
        default: hz_to_ppm = HERTZ_TO_PPM_MULTIPLIER_CHAN_5; break;
    }

    /* The carrier integrator accumulates over a different window at 110 kbps,
     * so the driver ships a separate multiplier -- a factor of 8 apart. */
    freq_mult = (datarate == RATE_110) ? FREQ_OFFSET_MULTIPLIER_110KB
                                       : FREQ_OFFSET_MULTIPLIER;

    /* Negated deliberately. The expression above is the driver's
     * clockOffsetRatio, for which Decawave's own SS-TWR uses
     *     tof = (T_rd1 - T_rp1 * (1 - clockOffsetRatio)) / 2,
     * whereas equation (7) of the specification is written
     *     tau = (T_rd1 - (1 + k_hat) * T_rp1) / 2.
     * The two agree only when k_hat = -clockOffsetRatio. Returning the spec's
     * convention here lets ranging_tof_ss_s() transcribe eq (7) literally;
     * without the negation the crystal-offset correction is applied backwards
     * and roughly doubles the error it exists to remove. */
    return -((double)carrier_integrator * (freq_mult * hz_to_ppm / 1.0e6));
}

double ranging_tof_ss_s(uint64_t poll_tx, uint64_t resp_rx,
                        uint64_t poll_rx, uint64_t resp_tx, double k_hat)
{
    int64_t rd1 = (int64_t)((resp_rx - poll_tx) & DTU_MASK_40);
    int64_t rp1 = (int64_t)((resp_tx - poll_rx) & DTU_MASK_40);

    if (rd1 <= 0 || rp1 <= 0 || rd1 <= rp1)
    {
        return -1.0;
    }
    /* Same plausibility band as ranging_tof_dtu(). Without it a corrupt or
     * mismatched Response payload yields a tau of seconds, and the caller's
     * (uint32_t)(tof * 1e12) conversion is then undefined behaviour -- the
     * garbage lands in tof_ps and is emitted as a valid-looking range. */
    if (rd1 > MAX_INTERVAL_DTU || rp1 > MAX_INTERVAL_DTU)
    {
        return -1.0;
    }
    {
        double tof = (((double)rd1 - (1.0 + k_hat) * (double)rp1) / 2.0);

        if (tof < 0.0 || tof > (double)MAX_TAU_DTU)
        {
            return -1.0;
        }
        return tof * DWT_TIME_UNITS;
    }
}

uint32_t ranging_dtu_to_ps(int64_t tau_dtu)
{
    if (tau_dtu < 0)
    {
        return 0;
    }
    return dtu_to_ps((uint64_t)tau_dtu);
}

int32_t ranging_ps_to_mm(uint32_t tau_ps, int32_t extra_dtu)
{
    /* c in air is 299702547 m/s = 0.299702547 mm/ps. */
    int64_t ps = (int64_t)tau_ps
               - (((int64_t)extra_dtu * 1565004LL + 50000LL) / 100000LL);

    if (ps < 0)
    {
        ps = 0;
    }
    return (int32_t)((ps * 299702547LL + 500000000LL) / 1000000000LL);
}
