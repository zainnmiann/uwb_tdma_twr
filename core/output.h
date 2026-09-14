/**
 * @file    output.h
 * @brief   Section 9.4 -- the ranging stream, and the section 11 flag set.
 */

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Section 9.4 flags. */
#define RNG_FLAG_SS_FALLBACK    0x01u   /**< bit0 SS-TWR fallback used        */
#define RNG_FLAG_NLOS           0x02u   /**< bit1 P_RX - P_FP > 10 dB         */
#define RNG_FLAG_RESP_LOST      0x04u   /**< bit2 Response lost               */
#define RNG_FLAG_REPORT_LOST    0x08u   /**< bit3 Report lost                 */
#define RNG_FLAG_DESYNC         0x10u   /**< bit4 desynchronized              */
#define RNG_FLAG_HASH_MISMATCH  0x20u   /**< bit5 configuration hash mismatch */

/** NLOS threshold from section 9.4, in dBm x 10. */
#define NLOS_THRESHOLD_DBM10    100

typedef struct
{
    uint16_t sfn;
    uint8_t  frame_idx;
    uint16_t tag_id;
    uint16_t anchor_id;
    uint8_t  slot;
    int32_t  d_mm;
    uint32_t tof_ps;
    int16_t  rxp_dbm10;
    int16_t  fpp_dbm10;
    uint8_t  flags;
    uint32_t t_local_us;
} rng_record_t;

/** Emit a framed ASCII line: "$" + payload + "*" + XOR checksum + CRLF. */
void out_line(const char *payload);

/** printf into the framing above. */
void out_linef(const char *fmt, ...);

/** Section 9.4 $RNG, honouring out_format and out_mask. */
void out_range(const rng_record_t *r);

/** Optional $POS line, emitted once per initiator cycle when a solver runs. */
void out_position(uint16_t sfn, uint16_t tag_id, int32_t x, int32_t y, int32_t z,
                  uint8_t n_used, uint32_t gdop_x100, uint32_t sigma_mm);

/** Raw timestamp trace, out_mask bit 3. */
void out_raw_ts(uint16_t sfn, uint16_t anchor_id,
                uint64_t poll_tx, uint64_t poll_rx, uint64_t resp_tx,
                uint64_t resp_rx, uint64_t final_tx, uint64_t final_rx);

/** Local device time in microseconds, for the t_local_us field. */
uint32_t out_local_us(void);

/** Format a dBm x 10 value as a signed decimal with one fractional digit. */
void out_fmt_dbm(char *dst, uint32_t n, int16_t dbm10);

#ifdef __cplusplus
}
#endif

#endif /* OUTPUT_H_ */
