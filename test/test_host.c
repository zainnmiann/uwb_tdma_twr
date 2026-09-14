/**
 * @file    test_host.c
 * @brief   Host-side regression tests for the target-independent logic.
 *
 * Covers the pure functions of section 4 (estimators), section 6 (frame
 * synchronization) and section 8 (parameter hash), plus the output helpers.
 * Build and run with `make -C test`.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

static int g_fail;
static int g_run;

#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        g_run++;                                                              \
        if (!(cond)) {                                                        \
            g_fail++;                                                         \
            printf("FAIL %s:%d  ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
        }                                                                     \
    } while (0)

/* ---------------------------------------------------------------------------
 * Section 4: the estimators, transcribed from ranging.c.
 * ------------------------------------------------------------------------ */
#define DTU_MASK_40         0xFFFFFFFFFFULL
#define DWT_TIME_UNITS      (1.0 / 499.2e6 / 128.0)
#define MAX_INTERVAL_DTU    (1277952000LL)
#define MAX_TAU_DTU         (213000LL)

static int64_t sub40(uint64_t a, uint64_t b)
{
    return (int64_t)((a - b) & DTU_MASK_40);
}

/* Equations (4)-(6). */
static double tof_ds(uint64_t poll_tx, uint64_t resp_rx, uint64_t final_tx,
                     uint64_t poll_rx, uint64_t resp_tx, uint64_t final_rx)
{
    int64_t rd1 = sub40(resp_rx, poll_tx);
    int64_t rp1 = sub40(final_tx, resp_rx);
    int64_t rd2 = sub40(final_rx, resp_tx);
    int64_t rp2 = sub40(resp_tx, poll_rx);
    int64_t num, den;

    if (rd1 <= 0 || rp1 <= 0 || rd2 <= 0 || rp2 <= 0) { return -1.0; }
    if (rd1 > MAX_INTERVAL_DTU || rp1 > MAX_INTERVAL_DTU ||
        rd2 > MAX_INTERVAL_DTU || rp2 > MAX_INTERVAL_DTU) { return -1.0; }

    num = rd1 * rd2 - rp1 * rp2;
    den = rd1 + rd2 + rp1 + rp2;
    if (den <= 0) { return -1.0; }
    num /= den;
    if (num < 0 || num > MAX_TAU_DTU) { return -1.0; }
    return (double)num * DWT_TIME_UNITS;
}

/* Equation (7), with the plausibility band added this round. */
static double tof_ss(uint64_t poll_tx, uint64_t resp_rx,
                     uint64_t poll_rx, uint64_t resp_tx, double k_hat)
{
    int64_t rd1 = sub40(resp_rx, poll_tx);
    int64_t rp1 = sub40(resp_tx, poll_rx);

    if (rd1 <= 0 || rp1 <= 0 || rd1 <= rp1) { return -1.0; }
    if (rd1 > MAX_INTERVAL_DTU || rp1 > MAX_INTERVAL_DTU) { return -1.0; }
    {
        double tof = (((double)rd1 - (1.0 + k_hat) * (double)rp1) / 2.0);
        if (tof < 0.0 || tof > (double)MAX_TAU_DTU) { return -1.0; }
        return tof * DWT_TIME_UNITS;
    }
}

static void test_estimators(void)
{
    /* 4.769 m -> 15.907 ns -> 1016.4 DTU. Build a consistent exchange. */
    const int64_t  tau   = 1016;
    const uint64_t p_tx  = 0x00A0000000ULL;
    const uint64_t p_rx  = p_tx + tau;
    const uint64_t r_tx  = p_rx  + 25000000ULL;      /* T_r0 + s*dr    */
    const uint64_t r_rx  = r_tx  + tau;
    const uint64_t f_tx  = r_rx  + 19000000ULL;      /* dr             */
    const uint64_t f_rx  = f_tx  + tau;
    double d;

    d = tof_ds(p_tx, r_rx, f_tx, p_rx, r_tx, f_rx);
    CHECK(d > 0.0, "DS-TWR rejected a well-formed exchange");
    CHECK(fabs(d / DWT_TIME_UNITS - (double)tau) <= 1.0,
          "DS-TWR tau=%.1f DTU, expected %lld", d / DWT_TIME_UNITS, (long long)tau);

    /* The same exchange through the SS-TWR fallback, k_hat = 0. */
    d = tof_ss(p_tx, r_rx, p_rx, r_tx, 0.0);
    CHECK(d > 0.0, "SS-TWR rejected a well-formed exchange");
    CHECK(fabs(d / DWT_TIME_UNITS - (double)tau) <= 2.0,
          "SS-TWR tau=%.1f DTU, expected %lld", d / DWT_TIME_UNITS, (long long)tau);

    /* Regression: a Report/Response carrying garbage timestamps must be
     * rejected, not turned into a huge tof that overflows the caller's
     * (uint32_t)(tof * 1e12) conversion. This is the round-2 finding. */
    CHECK(tof_ss(p_tx, p_tx + 900000000ULL, p_rx, r_tx, 0.0) < 0.0,
          "SS-TWR accepted an implausibly large round-trip");
    CHECK(tof_ss(p_tx, r_rx, p_rx, r_tx - 600000ULL, 0.0) < 0.0,
          "SS-TWR accepted a tau beyond the ~1000 m band");
    CHECK(tof_ds(p_tx, r_rx, f_tx, p_rx, r_tx, f_rx + 900000000ULL) < 0.0,
          "DS-TWR accepted an implausibly large interval");

    /* Zero-filled slots (a lost Response) must never look like a range. */
    CHECK(tof_ss(p_tx, 0, 0, 0, 0.0) < 0.0, "SS-TWR accepted zeroed timestamps");
    CHECK(tof_ds(p_tx, 0, 0, 0, 0, 0) < 0.0, "DS-TWR accepted zeroed timestamps");

    /* The 40-bit counter wraps mid-exchange; sub40 must still be correct. */
    {
        uint64_t w_tx = (DTU_MASK_40 - 500ULL);
        uint64_t w_rx = (w_tx + tau) & DTU_MASK_40;
        uint64_t wr_tx = (w_rx + 25000000ULL) & DTU_MASK_40;
        uint64_t wr_rx = (wr_tx + tau) & DTU_MASK_40;
        uint64_t wf_tx = (wr_rx + 19000000ULL) & DTU_MASK_40;
        uint64_t wf_rx = (wf_tx + tau) & DTU_MASK_40;
        d = tof_ds(w_tx, wr_rx, wf_tx, w_rx, wr_tx, wf_rx);
        CHECK(d > 0.0 && fabs(d / DWT_TIME_UNITS - (double)tau) <= 1.0,
              "DS-TWR failed across a 40-bit wrap: %.1f DTU", d / DWT_TIME_UNITS);
    }
}

/* ---------------------------------------------------------------------------
 * Section 6: equation (8) offset and the rearm prediction.
 * ------------------------------------------------------------------------ */
static uint8_t lambda(uint8_t k, uint8_t m, uint8_t n_i)
{
    return (uint8_t)(((int)k - (int)m + (int)n_i) % (int)n_i);
}

static void test_sync_offset(void)
{
    CHECK(lambda(0, 0, 4) == 0, "Lambda(k=m) must be 0");
    CHECK(lambda(3, 1, 4) == 2, "Lambda(3,1,4)");
    CHECK(lambda(1, 3, 4) == 2, "Lambda(1,3,4) must wrap to 2");
    CHECK(lambda(0, 15, 16) == 1, "Lambda(0,15,16)");
    /* Every offset is in range for every pair. */
    for (int n = 1; n <= 32; n++) {
        for (int k = 0; k < n; k++) {
            for (int m = 0; m < n; m++) {
                CHECK(lambda((uint8_t)k, (uint8_t)m, (uint8_t)n) < n,
                      "Lambda out of range for N_I=%d k=%d m=%d", n, k, m);
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * out_local_us(): the round-2 overflow fix.
 * ------------------------------------------------------------------------ */
static uint32_t local_us_step(uint32_t now, uint32_t *last, uint64_t *acc,
                              bool *started)
{
    if (!*started) { *started = true; *last = now; return 0; }
    *acc += ((uint64_t)(now - *last) * 4006400ULL) / 1000000000ULL;
    *last = now;
    return (uint32_t)*acc;
}

static void test_local_us(void)
{
    uint32_t last = 0, out;
    uint64_t acc = 0;
    bool started = false;

    (void)local_us_step(0, &last, &acc, &started);
    out = local_us_step(249600u, &last, &acc, &started);      /* 1 ms worth */
    CHECK(out == 999, "1 ms of hi32 ticks -> %u us, expected 999", out);

    /* 24 hours of uptime, stepped in near-wrap increments. The old form of
     * out_local_us() accumulated ticks and multiplied at the end, overflowing
     * uint64 after 5.12 hours; accumulating microseconds cannot. */
    last = 0; acc = 0; started = false;
    (void)local_us_step(0, &last, &acc, &started);
    {
        uint32_t t = 0;
        /* 0xF0000000 ticks = 16.1 s. 5364 steps ~= 24 h. */
        for (int i = 0; i < 5364; i++) {
            t += 0xF0000000u;                    /* wraps, by construction */
            (void)local_us_step(t, &last, &acc, &started);
        }
        /* 5364 * 0xF0000000 ticks * 4.0064 ns = 86404 s. */
        CHECK(acc > 86000000000ULL && acc < 87000000000ULL,
              "24 h of uptime accumulated %llu us, expected ~8.64e10",
              (unsigned long long)acc);
    }

    /* A full wrap between two calls must advance by one wrap, not go backwards. */
    last = 0xFFFFF000u; acc = 0; started = true;
    out = local_us_step(0x00001000u, &last, &acc, &started);
    CHECK(out == (uint32_t)(((uint64_t)0x2000 * 4006400ULL) / 1000000000ULL),
          "wrap-crossing delta wrong: %u", out);
}

/* ---------------------------------------------------------------------------
 * Section 9.4: the $RNG record must fit its formatting buffer.
 * ------------------------------------------------------------------------ */
static void test_rng_record_size(void)
{
    char buf[512];
    int n;

    /* Worst case: max sfn, max ids, most negative d_mm, 10-digit tof_ps,
     * three-character power fields with signs and a 10-digit t_us. */
    n = snprintf(buf, sizeof(buf),
        "{\"t\":\"RNG\",\"n\":%lu,\"frame\":%u,\"tag\":\"0x%04X\","
        "\"anchor\":\"0x%04X\",\"slot\":%u,\"d_mm\":%ld,\"tof_ps\":%lu,"
        "\"rxp\":%s,\"fpp\":%s,\"flags\":%u,\"t_us\":%lu}\r\n",
        4294967295UL, 255u, 0xFFFF, 0xFFFF, 255u, -2147483648L, 4294967295UL,
        "-999.9", "-999.9", 255u, 4294967295UL);

    /* uart_io_printf() now formats into char[256]. */
    CHECK(n < 256, "worst-case $RNG JSON is %d bytes, buffer is 256", n);
    printf("  (worst-case $RNG JSON: %d bytes)\n", n);

    n = snprintf(buf, sizeof(buf),
        "RNG,%lu,%u,0x%04X,0x%04X,%u,%ld,%lu,%s,%s,0x%02X,%lu",
        4294967295UL, 255u, 0xFFFF, 0xFFFF, 255u, -2147483648L, 4294967295UL,
        "-999.9", "-999.9", 255u, 4294967295UL);
    CHECK(n < 240, "worst-case $RNG CSV payload is %d bytes, buffer is 240", n);

    n = snprintf(buf, sizeof(buf),
        "TS,%lu,0x%04X,%010llX,%010llX,%010llX,%010llX,%010llX,%010llX",
        4294967295UL, 0xFFFF, ~0ULL, ~0ULL, ~0ULL, ~0ULL, ~0ULL, ~0ULL);
    CHECK(n < 240, "worst-case $TS payload is %d bytes, buffer is 240", n);
}

/* ---------------------------------------------------------------------------
 * Section 5: frame lengths must fit the 127-byte PHR limit.
 * ------------------------------------------------------------------------ */
#define MHR_LEN     9u
#define FCS_LEN     2u
#define TS_LEN      5u
#define MAX_FRAME   127u

static void test_frame_lengths(void)
{
    for (unsigned nr = 2; nr <= 12; nr++) {
        unsigned mask_bytes = (nr <= 8) ? 1u : 2u;
        unsigned final = MHR_LEN + 2u + TS_LEN + (nr * TS_LEN) + TS_LEN
                       + mask_bytes + FCS_LEN;
        CHECK(final <= MAX_FRAME,
              "Final is %u bytes at N_R=%u, over the %u-byte limit",
              final, nr, MAX_FRAME);
    }
}

/* ---------------------------------------------------------------------------
 * sync_missed_frame()'s clamp: must never zero the increment.
 * ------------------------------------------------------------------------ */
static uint32_t missed_increment(uint32_t init_count, uint8_t sync_mmax)
{
    uint32_t elapsed = (init_count > 0u) ? init_count : 1u;

    if (elapsed >= sync_mmax)
    {
        elapsed = (sync_mmax > 1u) ? ((uint32_t)sync_mmax - 1u) : 1u;
    }
    return elapsed;
}

static void test_missed_clamp(void)
{
    /* sync_mmax = 1 means "one missed frame is a desync". The clamp used to
     * yield 0 here, so s->missed never advanced and SYNC_FREERUN -- and
     * RNG_FLAG_DESYNC -- were unreachable at a legal setting. */
    CHECK(missed_increment(4, 1) >= 1, "sync_mmax=1 must still cross");
    CHECK(missed_increment(1, 1) >= 1, "sync_mmax=1, N_I=1 must still cross");

    /* And no legal pair may increment by zero, or M_max becomes unreachable. */
    for (uint32_t ni = 1; ni <= 32; ni++) {
        for (uint32_t mm = 1; mm <= 255; mm++) {
            uint32_t inc = missed_increment(ni, (uint8_t)mm);
            CHECK(inc >= 1, "zero increment at N_I=%u sync_mmax=%u", ni, mm);
            /* One silent superframe alone must not cross a threshold above 1. */
            if (mm > 1) {
                CHECK(inc < mm, "one superframe crossed M_max=%u at N_I=%u", mm, ni);
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Antenna delay: the same value belongs in BOTH registers.
 * ------------------------------------------------------------------------ */
#define CFG_ANT_DELAY_UNCAL     0xFFFFu
#define CFG_ANT_DELAY_DEFAULT   16436u
#define DTU_PS                  15.650040

static void test_ant_delay(void)
{
    /* dwt_settxantennadelay() and dwt_setrxantennadelay() each take the full
     * one-way delay for their own direction. Halving one figure across the two
     * -- which this project did until hardware disproved it -- under-
     * compensates all four timestamps of an exchange, and the shortfall lands
     * on tau as 4 * (d/2) / 2 = d. */
    uint32_t d = CFG_ANT_DELAY_DEFAULT;
    double   err_m = (double)d * DTU_PS * 1e-12 * 299702547.0;

    CHECK(err_m > 70.0 && err_m < 85.0,
          "halving the delay should cost ~77 m of range, got %.1f m", err_m);

    /* The sentinel must not be mistakable for a real delay. A DWM1001 sits
     * near 16436; anything within a few hundred units of that is a value
     * somebody might legitimately calibrate to. */
    CHECK(CFG_ANT_DELAY_UNCAL > CFG_ANT_DELAY_DEFAULT + 1000u,
          "sentinel 0x%X is too close to a real antenna delay",
          CFG_ANT_DELAY_UNCAL);
    {
        double sentinel_m =
            (double)CFG_ANT_DELAY_UNCAL * DTU_PS * 1e-12 * 299702547.0;

        CHECK(sentinel_m > 250.0,
              "sentinel should be physically impossible, it is %.0f m",
              sentinel_m);
    }
}

int main(void)
{
    printf("UWB DS-TWR host tests\n");
    test_estimators();
    test_sync_offset();
    test_local_us();
    test_rng_record_size();
    test_frame_lengths();
    test_missed_clamp();
    test_ant_delay();

    printf("%s: %d checks, %d failed\n", g_fail ? "FAIL" : "PASS", g_run, g_fail);
    return g_fail ? 1 : 0;
}
