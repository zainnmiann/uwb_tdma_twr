/**
 * @file    radio.h
 * @brief   Section 12 -- DW1000 library mapping. Port contract: implemented
 *          separately by port/nrf52/radio.c (Decawave deca_driver) and
 *          port/esp32/radio.cpp (Makerfabs DW1000Class).
 *
 * The stock Decawave driver is used as the PHY on nRF52; the MAC of this
 * specification sits above it. The high-level ranging module of the library
 * is deliberately not used: it performs discovery, keeps a dynamic device
 * list, and replies immediately rather than at a scheduled instant.
 *
 * The spec lists five facilities as "absent from most library versions" and to
 * be added as thin register accesses. In DW1000 API v04.00.06 all five are
 * already exposed, so no register-level additions were needed:
 *
 *   frame wait timeout, RX_FWTO / SYS_CFG.RXWTOE  -> dwt_setrxtimeout()
 *   delayed RX, SYS_CTRL.RXENAB with RXDLYE       -> dwt_rxenable(DWT_START_RX_DELAYED)
 *   SFD timeout, DRX_SFDTOC                       -> dwt_config_t.sfdTO
 *   PAC alignment, DRX_TUNE0b / DRX_TUNE1a        -> dwt_configure()
 *   manual TX power, TX_POWER + SYS_CFG.DIS_STXP  -> dwt_configuretxrf() / dwt_setsmarttxpower()
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"
#include "proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t  buf[MAX_FRAME_LEN];
    uint16_t len;       /**< payload length, FCS excluded */
    uint64_t rx_ts;     /**< 40-bit adjusted RX timestamp  */
    uint32_t status;
} radio_rx_t;

#define RADIO_RX_FRAME      1
#define RADIO_RX_TIMEOUT    0
#define RADIO_RX_ERROR    (-1)
#define RADIO_RX_PENDING    2   /**< armed, nothing has landed yet */

#define RADIO_TX_OK         0
#define RADIO_TX_LATE     (-1)

/** Reset, initialise and fully configure the DW1000 from @p c. */
bool     radio_init(const cfg_t *c);

/** Re-apply radio settings after a configuration change. */
bool     radio_reconfigure(const cfg_t *c);

/** Receive. A @p timeout_us that rounds down to zero uus is raised to the
 *  smallest representable nonzero timeout -- dwt_setrxtimeout(0) disables
 *  the frame wait timeout entirely and would block forever. */
int      radio_rx(radio_rx_t *rx, uint32_t timeout_us, bool delayed, uint32_t start_hi32);

/** Arm the receiver and return immediately, so the caller can do other work
 *  while the DW1000 receives. Returns false when @p delayed was requested and
 *  @p start_hi32 has already passed. Collect with radio_rx_poll(). */
bool     radio_rx_arm(uint32_t timeout_us, bool delayed, uint32_t start_hi32);

/** Non-blocking collect for a receiver armed by radio_rx_arm(). Returns
 *  RADIO_RX_PENDING until the DW1000 reports a frame, a timeout or an error. */
int      radio_rx_poll(radio_rx_t *rx);

/** Transmit at an exact device time. @p len includes the two FCS bytes. */
int      radio_tx_at(const uint8_t *buf, uint16_t len, uint32_t tx_hi32);
void     radio_tx_wait(void);
void     radio_idle(void);

uint32_t radio_systime_hi32(void);
int32_t  radio_hi32_diff_us(uint32_t a, uint32_t b);
uint64_t radio_tx_ts_from_hi32(uint32_t tx_hi32);
int32_t  radio_carrier_integrator(void);

/** Link quality, section 5's Report payload and section 9.4's output. */
typedef struct
{
    int16_t  rx_dbm10;      /**< P_RX, dBm x 10  */
    int16_t  fp_dbm10;      /**< P_FP, dBm x 10  */
    uint16_t std_noise;
    uint16_t rxpacc;
} radio_quality_t;

void     radio_quality(radio_quality_t *q);

/** TX power register actually programmed, for the configuration dump. */
uint32_t radio_txpower_reg(void);
/** True when the AUTO power came from OTP rather than the fallback table. */
bool     radio_txpower_from_otp(void);

/** Antenna delays actually programmed. */
uint16_t radio_ant_tx(void);
uint16_t radio_ant_rx(void);

/** DW1000 lot/part identifier, unique per device. Valid after radio_init(). */
uint32_t radio_part_id(void);

/** DW1000 device identifier register (DEV_ID). Valid after radio_init(). */
uint32_t radio_device_id(void);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_H_ */
