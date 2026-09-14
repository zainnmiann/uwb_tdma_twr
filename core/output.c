#include "output.h"
#include "radio.h"
#include "uart_io.h"
#include "port_sys.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Section 9: XOR checksum over the payload between '$' and '*'.
 *
 * NOTE ON THE SPEC: the illustrative checksums printed in sections 9.3 and 9.4
 * ("$CFG*4A", "...*7C") do not match this rule for their payloads, so they are
 * read as placeholders. The stated rule is what is implemented. */
static uint8_t xor_cs(const char *p)
{
    uint8_t c = 0;
    while (*p != '\0')
    {
        c ^= (uint8_t)*p++;
    }
    return c;
}

void out_line(const char *payload)
{
    char buf[8];

    uart_io_write("$", 1);
    uart_io_puts(payload);
    (void)snprintf(buf, sizeof(buf), "*%02X\r\n", xor_cs(payload));
    uart_io_puts(buf);
}

void out_linef(const char *fmt, ...)
{
    char    payload[240];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(payload, sizeof(payload), fmt, ap);
    va_end(ap);

    out_line(payload);
}

uint32_t out_local_us(void)
{
    /* The DW1000 system clock's top 32 bits wrap every 2^32 * 4.0064 ns =
     * 17.207 s. Section 9.4's t_local_us is a running local timestamp, so the
     * wraps are accumulated here rather than letting the field jump back to
     * zero.
     *
     * Microseconds are accumulated, not ticks: multiplying an ever-growing
     * tick total by 4006400 overflows uint64 after ~5.1 hours of uptime. Each
     * delta is bounded by one wrap, so the intermediate stays small, and a
     * delta computed with unsigned wraparound arithmetic is correct even when
     * a whole wrap elapses between calls (during a $RANGE,STOP, say). */
    static uint32_t s_last_hi32;
    static uint64_t s_us;
    static bool     s_started;

    uint32_t now = radio_systime_hi32();

    if (!s_started)
    {
        s_started   = true;
        s_last_hi32 = now;
        return 0;
    }

    /* Unsigned subtraction wraps correctly across the 32-bit boundary. */
    s_us += ((uint64_t)(now - s_last_hi32) * 4006400ULL) / 1000000000ULL;
    s_last_hi32 = now;

    return (uint32_t)s_us;
}

void out_fmt_dbm(char *dst, uint32_t n, int16_t dbm10)
{
    int32_t v = dbm10;
    int32_t whole, frac;
    const char *sign = "";

    if (v < 0) { sign = "-"; v = -v; }
    whole = v / 10;
    frac  = v % 10;
    (void)snprintf(dst, n, "%s%ld.%ld", sign, (long)whole, (long)frac);
}

/* --- binary framing ------------------------------------------------------ */

#define BIN_SYNC0   0xA5u
#define BIN_SYNC1   0x5Au
#define BIN_ID_RNG  0x01u
#define BIN_ID_POS  0x02u

static uint8_t crc8(const uint8_t *d, uint32_t n)
{
    uint8_t c = 0xFFu;
    uint32_t i, b;
    for (i = 0; i < n; i++)
    {
        c ^= d[i];
        for (b = 0; b < 8; b++)
        {
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
        }
    }
    return c;
}

static void bin_emit(uint8_t id, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[4];

    hdr[0] = BIN_SYNC0;
    hdr[1] = BIN_SYNC1;
    hdr[2] = id;
    hdr[3] = len;

    uart_io_write((const char *)hdr, sizeof(hdr));
    uart_io_write((const char *)payload, len);
    {
        uint8_t c = crc8(payload, len);
        uart_io_write((const char *)&c, 1);
    }
}

/* --- $RNG ---------------------------------------------------------------- */

void out_range(const rng_record_t *r)
{
    port_ui_range(r->slot, r->anchor_id, r->d_mm, r->rxp_dbm10,
                      (r->flags & RNG_FLAG_RESP_LOST) == 0u);

    const cfg_t *c = cfg();

    if ((c->out_mask & OUT_MASK_RANGE) == 0u)
    {
        return;
    }

    if (c->out_format == OUT_BIN)
    {
        uint8_t p[25];
        uint32_t o = 0;
        /* Suppressed the same way as the text formats. Packing them regardless
         * made out_mask's meaning depend on out_format, which neither the mask
         * bits nor section 9.4 describe. */
        bool     diag = (c->out_mask & OUT_MASK_DIAG) != 0u;
        int16_t  rxp  = diag ? r->rxp_dbm10 : 0;
        int16_t  fpp  = diag ? r->fpp_dbm10 : 0;
        p[o++] = (uint8_t)r->sfn;        p[o++] = (uint8_t)(r->sfn >> 8);
        p[o++] = r->frame_idx;
        p[o++] = (uint8_t)r->tag_id;     p[o++] = (uint8_t)(r->tag_id >> 8);
        p[o++] = (uint8_t)r->anchor_id;  p[o++] = (uint8_t)(r->anchor_id >> 8);
        p[o++] = r->slot;
        p[o++] = (uint8_t)r->d_mm;       p[o++] = (uint8_t)(r->d_mm >> 8);
        p[o++] = (uint8_t)(r->d_mm >> 16); p[o++] = (uint8_t)(r->d_mm >> 24);
        p[o++] = (uint8_t)r->tof_ps;     p[o++] = (uint8_t)(r->tof_ps >> 8);
        p[o++] = (uint8_t)(r->tof_ps >> 16); p[o++] = (uint8_t)(r->tof_ps >> 24);
        p[o++] = (uint8_t)rxp;           p[o++] = (uint8_t)(rxp >> 8);
        p[o++] = (uint8_t)fpp;           p[o++] = (uint8_t)(fpp >> 8);
        p[o++] = r->flags;
        p[o++] = (uint8_t)r->t_local_us; p[o++] = (uint8_t)(r->t_local_us >> 8);
        p[o++] = (uint8_t)(r->t_local_us >> 16); p[o++] = (uint8_t)(r->t_local_us >> 24);
        bin_emit(BIN_ID_RNG, p, (uint8_t)o);
        return;
    }

    {
        char rxp[12], fpp[12];
        bool diag = (c->out_mask & OUT_MASK_DIAG) != 0u;

        out_fmt_dbm(rxp, sizeof(rxp), diag ? r->rxp_dbm10 : 0);
        out_fmt_dbm(fpp, sizeof(fpp), diag ? r->fpp_dbm10 : 0);

        if (c->out_format == OUT_JSON)
        {
            uart_io_printf(
                "{\"t\":\"RNG\",\"n\":%lu,\"frame\":%u,\"tag\":\"0x%04X\","
                "\"anchor\":\"0x%04X\",\"slot\":%u,\"d_mm\":%ld,\"tof_ps\":%lu,"
                "\"rxp\":%s,\"fpp\":%s,\"flags\":%u,\"t_us\":%lu}\r\n",
                (unsigned long)r->sfn, r->frame_idx, r->tag_id, r->anchor_id,
                r->slot, (long)r->d_mm, (unsigned long)r->tof_ps,
                rxp, fpp, r->flags, (unsigned long)r->t_local_us);
            return;
        }

        out_linef("RNG,%lu,%u,0x%04X,0x%04X,%u,%ld,%lu,%s,%s,0x%02X,%lu",
                  (unsigned long)r->sfn, r->frame_idx, r->tag_id, r->anchor_id,
                  r->slot, (long)r->d_mm, (unsigned long)r->tof_ps,
                  rxp, fpp, r->flags, (unsigned long)r->t_local_us);
    }
}

void out_position(uint16_t sfn, uint16_t tag_id, int32_t x, int32_t y, int32_t z,
                  uint8_t n_used, uint32_t gdop_x100, uint32_t sigma_mm)
{
    const cfg_t *c = cfg();

    if ((c->out_mask & OUT_MASK_POSITION) == 0u)
    {
        return;
    }

    out_linef("POS,%lu,0x%04X,%ld,%ld,%ld,%u,%lu.%02lu,%lu",
              (unsigned long)sfn, tag_id, (long)x, (long)y, (long)z, n_used,
              (unsigned long)(gdop_x100 / 100u), (unsigned long)(gdop_x100 % 100u),
              (unsigned long)sigma_mm);
}

void out_raw_ts(uint16_t sfn, uint16_t anchor_id,
                uint64_t poll_tx, uint64_t poll_rx, uint64_t resp_tx,
                uint64_t resp_rx, uint64_t final_tx, uint64_t final_rx)
{
    if ((cfg()->out_mask & OUT_MASK_RAW_TS) == 0u)
    {
        return;
    }

    out_linef("TS,%lu,0x%04X,%010llX,%010llX,%010llX,%010llX,%010llX,%010llX",
              (unsigned long)sfn, anchor_id,
              (unsigned long long)poll_tx,  (unsigned long long)poll_rx,
              (unsigned long long)resp_tx,  (unsigned long long)resp_rx,
              (unsigned long long)final_tx, (unsigned long long)final_rx);
}
