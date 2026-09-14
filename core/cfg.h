/**
 * @file    cfg.h
 * @brief   Section 8 -- the complete parameter set, persistence and validation.
 *
 * Every key in the spec's table is present, with the spec's type, range and
 * default. All parameters live in non-volatile memory and are reloaded at boot.
 */

#ifndef CFG_H_
#define CFG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- caps ---------------------------------------------------------------- */
#define CFG_MAX_INIT            32u     /* init_count range 1..32  */
#define CFG_MAX_RESP            12u     /* resp_count range 2..12  */
#define CFG_NAME_LEN            17u     /* 16 chars + NUL          */

/* Sentinel written by cfg_defaults() for ant_delay_tx/rx. Section 8.1's
 * default for these is "calibrated", so it means "not yet calibrated":
 * radio.c substitutes the part's own OTP antenna delay when it sees it, and
 * section 13 step 1 replaces it with a surveyed-baseline measurement.
 *
 * 0xFFFF, not 16385. 16385 is a perfectly plausible delay -- it is within 51
 * units of the canonical DWM1001 figure -- so it could not be told apart from
 * a board deliberately calibrated to it, and "calibrate to 16385" silently
 * meant "ignore my calibration". 0xFFFF is 1.02 us, 307 m of range, which no
 * antenna has. */
#define CFG_ANT_DELAY_UNCAL     0xFFFFu

/* Used when the sentinel is set and the part has no OTP value. Decawave's
 * canonical DWM1001 antenna delay; section 13 step 1 replaces it per board. */
#define CFG_ANT_DELAY_DEFAULT   16436u
#define CFG_SSID_LEN            33u     /* 32 chars + NUL          */

/* --- enums --------------------------------------------------------------- */
typedef enum { ROLE_INIT = 0, ROLE_RESP = 1, ROLE_LSTN = 2 } cfg_role_t;
typedef enum { PRF_16 = 0, PRF_64 = 1 }                      cfg_prf_t;
typedef enum { RATE_110 = 0, RATE_850 = 1, RATE_6800 = 2 }   cfg_rate_t;
typedef enum { SFD_STD = 0, SFD_DW = 1 }                     cfg_sfd_t;
typedef enum { TXP_AUTO = 0, TXP_MANUAL = 1 }                cfg_txpmode_t;
typedef enum { OUT_CSV = 0, OUT_JSON = 1, OUT_BIN = 2 }      cfg_outfmt_t;
typedef enum { IFACE_USB = 0, IFACE_UART = 1,
               IFACE_BLE = 2, IFACE_WIFI = 3 }               cfg_iface_t;

/* out_mask bits */
#define OUT_MASK_RANGE          0x01u
#define OUT_MASK_DIAG           0x02u
#define OUT_MASK_POSITION       0x04u
#define OUT_MASK_RAW_TS         0x08u

/* --- list entries -------------------------------------------------------- */
typedef struct
{
    uint16_t id;            /**< responder short address        */
    uint8_t  slot;          /**< slot index s                   */
    uint8_t  en;
    int32_t  x_mm, y_mm, z_mm;
    int16_t  ant;           /**< delta_B_s trim, device time units, signed */
    uint16_t pad;
} cfg_anchor_t;

typedef struct
{
    uint16_t id;            /**< initiator short address        */
    uint8_t  frame;         /**< frame index k                  */
    uint8_t  div;           /**< rate divider D                 */
    uint8_t  en;
    uint8_t  pad[3];
} cfg_tag_t;

/* --- measured air times (section 7.2) ------------------------------------ */
/* Nanoseconds. Zero means "not measured", and the predicted value is used. */
typedef struct
{
    uint32_t poll_ns;
    uint32_t resp_ns;
    uint32_t final_ns;
    uint32_t report_ns;
} cfg_airmeas_t;

/* --- the parameter set --------------------------------------------------- */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    /* Role and identity */
    uint8_t  role;
    uint8_t  frame_index;
    uint8_t  slot_index;
    uint8_t  pad_a;
    uint16_t node_id;
    uint16_t pan_id;

    /* Network size */
    uint8_t  init_count;
    uint8_t  resp_count;
    uint8_t  pad_b[2];

    /* Timing (microseconds) */
    uint32_t frame_dur_us;
    uint32_t resp_base_us;
    uint32_t resp_pitch_us;
    uint32_t report_base_us;
    uint32_t report_pitch_us;
    uint32_t guard_us;
    uint32_t rx_timeout_us;
    uint8_t  rate_div;
    uint8_t  sync_mmax;
    uint8_t  pad_c[2];

    /* Processing-budget terms. Deliberately NOT in the section 8.1 table --
     * T_proc/T_ISR/T_margin appear nowhere in it, only inside equations (12)
     * and (13) and the section 7.4 worked example. They cannot be evaluated
     * without values, so they are exposed as keys here as an extension.
     *
     * The defaults are NOT the section 7.4 example's 60/15/25/60/60: those are
     * an illustration ("we obtain"), and section 13 step 5 requires filling
     * Table A/B from measurement instead. See cfg_defaults() and $MEASURE.
     *
     * These feed $TABLEB and $AUTOTIME only; the runtime uses resp_pitch_us
     * and friends. */
    uint32_t proc_us;
    uint32_t isr_us;
    uint32_t margin_us;
    uint32_t proc_poll_us;
    uint32_t proc_final_us;

    /* Equation (9) compensation: tau_hat + T_ISR, subtracted from the observed
     * Poll reception when rearming. This implementation polls SYS_STATUS and
     * uses the DW1000 hardware RX timestamp, so there is no interrupt latency
     * in t_R and the default is 0. */
    uint32_t sync_comp_us;

    /* Radio */
    uint8_t  channel;
    uint8_t  prf;
    uint8_t  preamble_code;
    uint8_t  pac_size;          /**< 8/16/32/64, 0 = auto  */
    uint16_t preamble_len;      /**< 64..4096 IEEE values  */
    uint8_t  datarate;
    uint8_t  sfd_mode;
    uint8_t  txpower_mode;
    uint8_t  txpower_coarse;
    uint8_t  txpower_fine;
    uint8_t  smart_power;
    uint32_t txpower_reg;
    uint16_t ant_delay_tx;
    uint16_t ant_delay_rx;

    /* Operation */
    uint8_t  autostart;
    uint8_t  report_enable;
    uint8_t  out_format;
    uint8_t  out_mask;

    /* Interfaces */
    uint8_t  iface_cfg;
    uint8_t  iface_data;
    uint8_t  pad_d[2];
    uint32_t uart_baud;
    char     ble_name[CFG_NAME_LEN];
    char     wifi_ssid[CFG_SSID_LEN];
    char     wifi_pass[CFG_SSID_LEN];
    uint8_t  pad_e[1];

    /* Section 7.2 measured air times, per operating point is not stored --
     * these apply to the currently configured radio settings. */
    cfg_airmeas_t air;

    /* Lists */
    cfg_anchor_t anchor[CFG_MAX_RESP];
    cfg_tag_t    tag[CFG_MAX_INIT];

    uint32_t crc;
} cfg_t;

/* --- API ----------------------------------------------------------------- */
void        cfg_load(void);
cfg_t      *cfg(void);
void        cfg_defaults(cfg_t *c);
bool        cfg_save(void);
bool        cfg_reload(void);

/** Section 8.2 consistency rules. Returns 0 on success, else the spec's error
 *  code (3 inconsistent with (14), 4 illegal preamble code, 5 read-only while
 *  ranging, ...). @p key receives the offending key name. */
int         cfg_validate(const cfg_t *c, const char **key, const char **reason);

/** Section 8.2 rule 6 / section 5: 16-bit hash over every parameter that must
 *  match network-wide. Carried in the Poll. */
uint16_t    cfg_hash(const cfg_t *c);

/** Fill in node_id and ble_name from the DW1000 part id when unprovisioned. */
void cfg_apply_identity(cfg_t *c, uint32_t part_id);

/** This node's short address. */
uint16_t    cfg_addr(void);

/** Look up a responder by slot index; NULL if absent or disabled. */
const cfg_anchor_t *cfg_anchor_by_slot(uint8_t slot);
/** Look up a responder by node id; NULL if absent. */
const cfg_anchor_t *cfg_anchor_by_id(uint16_t id);
/** Look up an initiator by frame index; NULL if absent or disabled. */
const cfg_tag_t    *cfg_tag_by_frame(uint8_t frame);

/* Enum <-> string, for the serial interface. */
const char *cfg_role_str(uint8_t v);
const char *cfg_rate_str(uint8_t v);
const char *cfg_sfd_str(uint8_t v);
const char *cfg_txpmode_str(uint8_t v);
const char *cfg_outfmt_str(uint8_t v);
const char *cfg_iface_str(uint8_t v);

/** Legal preamble codes for a channel/PRF pair (section 8.2 rule 3). */
bool cfg_preamble_code_legal(uint8_t chan, uint8_t prf, uint8_t code);
/** PAC size implied by a preamble length (section 8.2 rule 4). */
uint8_t cfg_pac_for_preamble(uint16_t plen);

#ifdef __cplusplus
}
#endif

#endif /* CFG_H_ */
