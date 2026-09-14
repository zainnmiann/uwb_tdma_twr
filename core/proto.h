/**
 * @file    proto.h
 * @brief   Section 5 -- message formats.
 *
 * Fixed-length payloads. Timestamps are 5-byte DW1000 device-time values.
 *
 *   Field                                    Bytes  Poll Resp Final Report
 *   MHR (FC, seq, PAN, dst, src)               9      x    x    x     x
 *   Message type, protocol version             1      x    x    x     x
 *   Superframe counter n                       2      x    x    x     x
 *   Frame index k                              1      x    x    x     x
 *   Slot index s                               1      -    x    -     x
 *   t_B_R,poll / t_B_T,resp                   10      -    x    -     -
 *   t_A_T,poll / t_A_R,resp[0..NR-1] / t_A_T,final
 *                                        5(N_R+2)     -    -    x     -
 *   Response reception mask                    1      -    -    x     -
 *   tau (32-bit, ps units)                     4      -    -    -     x
 *   Link quality (P_RX, P_FP, std noise)       3      -    -    -     x
 *   Configuration hash                         2      x    -    -     -
 *   FCS                                        2      x    x    x     x
 *
 * NOTE ON THE SPEC: the "Total at N_R = 4" row gives 17/25/44/23. Poll (17)
 * and Report (23) match the field matrix exactly; Resp and Final do not --
 * summing the marked fields gives 26 and 46. The field matrix is unambiguous
 * and self-consistent, so it is what is implemented here; the totals row
 * appears to be an arithmetic slip in revision 0.1.
 */

#ifndef PROTO_H_
#define PROTO_H_

#include <stdint.h>
#include <string.h>
#include "cfg.h"          /* CFG_MAX_RESP, for the compile-time checks below */

/* Portable static-assert: C11 and C++ spell it differently, and pre-C11
 * toolchains have neither. This must resolve before the extern "C" block --
 * static_assert in C++ is a declaration, not a linkage-affected symbol, but
 * keeping it outside avoids any question. */
#if defined(__cplusplus)
#  define PROTO_STATIC_ASSERT(c, msg) static_assert(c, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#  define PROTO_STATIC_ASSERT(c, msg) _Static_assert(c, msg)
#else
#  define PROTO_SA_CAT_(a, b) a##b
#  define PROTO_SA_CAT(a, b)  PROTO_SA_CAT_(a, b)
#  define PROTO_STATIC_ASSERT(c, msg) \
      typedef char PROTO_SA_CAT(proto_static_assert_, __LINE__)[(c) ? 1 : -1]
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_VERSION           1u

/* Message type is packed with the protocol version into one byte:
 *   bits 7..4 message type, bits 3..0 protocol version. */
#define MSG_POLL                0x1u
#define MSG_RESP                0x2u
#define MSG_FINAL               0x3u
#define MSG_REPORT              0x4u

#define MSG_TYPE(b)             (((b) >> 4) & 0x0Fu)
#define MSG_VER(b)              ((b) & 0x0Fu)
#define MSG_TYPEVER(t)          ((uint8_t)(((t) << 4) | PROTO_VERSION))

/* MHR ---------------------------------------------------------------------
 * IEEE 802.15.4-2011 data frame, PAN ID compressed, 16-bit short addresses. */
#define MHR_FC0                 0x41u
#define MHR_FC1                 0x88u
#define MHR_LEN                 9u
#define FCS_LEN                 2u
#define MAX_FRAME_LEN           127u

#define MHR_IDX_FC              0u
#define MHR_IDX_SEQ             2u
#define MHR_IDX_PAN             3u
#define MHR_IDX_DST             5u
#define MHR_IDX_SRC             7u

#define ADDR_BROADCAST          0xFFFFu

#define TS_LEN                  5u

/* Common header after the MHR ---------------------------------------------- */
#define IDX_TYPEVER             (MHR_LEN + 0u)          /* u8  */
#define IDX_SFN                 (MHR_LEN + 1u)          /* u16 superframe counter n */
#define IDX_FRAME_IDX           (MHR_LEN + 3u)          /* u8  frame index k        */
#define COMMON_LEN              (MHR_LEN + 4u)

/* Poll --------------------------------------------------------------------- */
#define POLL_IDX_CFG_HASH       (COMMON_LEN + 0u)       /* u16 */
#define POLL_LEN                (COMMON_LEN + 2u + FCS_LEN)

/* Response ----------------------------------------------------------------- */
#define RESP_IDX_SLOT           (COMMON_LEN + 0u)       /* u8  */
#define RESP_IDX_POLL_RX        (COMMON_LEN + 1u)       /* ts5 t_B_R,poll  */
#define RESP_IDX_RESP_TX        (COMMON_LEN + 6u)       /* ts5 t_B_T,resp  */
#define RESP_LEN                (COMMON_LEN + 11u + FCS_LEN)

/* Final -------------------------------------------------------------------- */
/* t_A_T,poll, then t_A_R,resp[0..N_R-1], then t_A_T,final. */
#define FIN_IDX_POLL_TX         (COMMON_LEN + 0u)       /* ts5 */
#define FIN_IDX_RESP_RX(nr)     (COMMON_LEN + 5u)       /* ts5[nr] */
#define FIN_IDX_FINAL_TX(nr)    (COMMON_LEN + 5u + ((uint32_t)(nr) * TS_LEN))
#define FIN_IDX_RX_MASK(nr)     (FIN_IDX_FINAL_TX(nr) + TS_LEN)
/* SPEC CONFLICT, resolved here: section 5 gives the Response reception mask as
 * one byte, but section 8.1 lets resp_count run to 12, which one byte cannot
 * address. The field is therefore one byte for N_R <= 8 -- which keeps the
 * section 5 byte counts exact at the tabulated N_R = 4 -- and two bytes above
 * that, so the whole configured range stays representable. */
#define FIN_MASK_BYTES(nr)      (((nr) <= 8u) ? 1u : 2u)
#define FINAL_LEN(nr)           (FIN_IDX_RX_MASK(nr) + FIN_MASK_BYTES(nr) + FCS_LEN)

static inline void p_mask(uint8_t *b, uint32_t o, uint32_t nr, uint16_t m)
{
    b[o] = (uint8_t)m;
    if (FIN_MASK_BYTES(nr) == 2u) { b[o + 1] = (uint8_t)(m >> 8); }
}
static inline uint16_t g_mask(const uint8_t *b, uint32_t o, uint32_t nr)
{
    return (FIN_MASK_BYTES(nr) == 2u)
         ? (uint16_t)((uint16_t)b[o] | ((uint16_t)b[o + 1] << 8))
         : (uint16_t)b[o];
}

/* Report ------------------------------------------------------------------- */
#define REP_IDX_SLOT            (COMMON_LEN + 0u)       /* u8  */
#define REP_IDX_TOF_PS          (COMMON_LEN + 1u)       /* u32 tau, picoseconds */
#define REP_IDX_RXP             (COMMON_LEN + 5u)       /* u8  -dBm, 0.5 dB steps */
#define REP_IDX_FPP             (COMMON_LEN + 6u)       /* u8  -dBm, 0.5 dB steps */
#define REP_IDX_NSTD            (COMMON_LEN + 7u)       /* u8  std noise >> 4      */
#define REPORT_LEN              (COMMON_LEN + 8u + FCS_LEN)

/* Link-quality byte packing: P is reported as -dBm in 0.5 dB steps, so
 * -84.5 dBm -> 169. Range -0.0 .. -127.5 dBm. */
#define PWR_TO_BYTE(dbm10)      ((uint8_t)(((dbm10) > 0) ? 0u : \
                                 (((-(int32_t)(dbm10)) / 5) > 255 ? 255u : \
                                  (uint8_t)((-(int32_t)(dbm10)) / 5))))
#define BYTE_TO_PWR_DBM10(b)    ((int16_t)(-5 * (int16_t)(b)))

/* Little-endian accessors -------------------------------------------------- */

static inline void p_u8(uint8_t *b, uint32_t o, uint8_t v)      { b[o] = v; }
static inline uint8_t g_u8(const uint8_t *b, uint32_t o)        { return b[o]; }

static inline void p_u16(uint8_t *b, uint32_t o, uint16_t v)
{
    b[o] = (uint8_t)v; b[o + 1] = (uint8_t)(v >> 8);
}
static inline uint16_t g_u16(const uint8_t *b, uint32_t o)
{
    return (uint16_t)((uint16_t)b[o] | ((uint16_t)b[o + 1] << 8));
}
static inline void p_u32(uint8_t *b, uint32_t o, uint32_t v)
{
    b[o] = (uint8_t)v;         b[o + 1] = (uint8_t)(v >> 8);
    b[o + 2] = (uint8_t)(v >> 16); b[o + 3] = (uint8_t)(v >> 24);
}
static inline uint32_t g_u32(const uint8_t *b, uint32_t o)
{
    return ((uint32_t)b[o]) | ((uint32_t)b[o + 1] << 8)
         | ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
}
static inline void p_ts(uint8_t *b, uint32_t o, uint64_t ts)
{
    uint32_t i;
    for (i = 0; i < TS_LEN; i++) { b[o + i] = (uint8_t)ts; ts >>= 8; }
}
static inline uint64_t g_ts(const uint8_t *b, uint32_t o)
{
    uint64_t v = 0; int32_t i;
    for (i = (int32_t)TS_LEN - 1; i >= 0; i--) { v = (v << 8) | b[o + (uint32_t)i]; }
    return v;
}

static inline void proto_hdr(uint8_t *b, uint8_t seq, uint16_t pan, uint16_t dst,
                             uint16_t src, uint8_t type, uint16_t sfn, uint8_t k)
{
    b[MHR_IDX_FC]     = MHR_FC0;
    b[MHR_IDX_FC + 1] = MHR_FC1;
    b[MHR_IDX_SEQ]    = seq;
    p_u16(b, MHR_IDX_PAN, pan);
    p_u16(b, MHR_IDX_DST, dst);
    p_u16(b, MHR_IDX_SRC, src);
    p_u8 (b, IDX_TYPEVER, MSG_TYPEVER(type));
    p_u16(b, IDX_SFN, sfn);
    p_u8 (b, IDX_FRAME_IDX, k);
}

/** Header sanity check on a received buffer. @p len excludes the FCS. */
static inline int proto_hdr_ok(const uint8_t *b, uint32_t len, uint16_t pan)
{
    if (len < COMMON_LEN)                                   { return 0; }
    if (b[MHR_IDX_FC] != MHR_FC0 || b[MHR_IDX_FC + 1] != MHR_FC1) { return 0; }
    if (g_u16(b, MHR_IDX_PAN) != pan)                       { return 0; }
    return (MSG_VER(b[IDX_TYPEVER]) == PROTO_VERSION);
}

/* Compile-time checks on the message formats of section 5 -- moved here
 * from main.c so neither port can lose them. */
PROTO_STATIC_ASSERT(FINAL_LEN(CFG_MAX_RESP) <= MAX_FRAME_LEN,
               "Final frame exceeds the 127-byte standard PHR limit at CFG_MAX_RESP");
PROTO_STATIC_ASSERT(REPORT_LEN <= MAX_FRAME_LEN, "Report frame too long");
PROTO_STATIC_ASSERT(RESP_LEN   <= MAX_FRAME_LEN, "Response frame too long");
PROTO_STATIC_ASSERT(POLL_LEN   <= MAX_FRAME_LEN, "Poll frame too long");
PROTO_STATIC_ASSERT(CFG_MAX_RESP <= 16,
               "the Response reception mask is at most two bytes in the Final "
               "(see FIN_MASK_BYTES in proto.h)");

#ifdef __cplusplus
}
#endif

#endif /* PROTO_H_ */
