/**
 * @file radio_esp32.cpp
 * @brief radio.h implemented on Makerfabs DW1000Class (library unmodified).
 *
 * Absolute delayed TX/RX and RX frame-wait timeout use public writeBytes /
 * readBytes (Makerfabs ships //private: commented out). Antenna delay is
 * programmed via setAntennaDelay() + commitConfiguration().
 */

#include "radio.h"
#include "timing.h"
#include "board_pins.h"

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

#include <DW1000.h>

/* Registers / bits not exposed as high-level helpers. */
#ifndef RX_FWTO
#define RX_FWTO           0x0C
#define LEN_RX_FWTO       2
#endif
#ifndef RXWTOE_BIT
#define RXWTOE_BIT        28
#endif
#ifndef HPDWARN_BIT
#define HPDWARN_BIT       27
#endif
#ifndef TXPUTE_BIT
#define TXPUTE_BIT        10  /* byte 4 bit 2 => absolute bit 34; handled specially */
#endif

static uint16_t s_ant_tx = CFG_ANT_DELAY_DEFAULT;
static uint16_t s_ant_rx = CFG_ANT_DELAY_DEFAULT;
static uint32_t s_txpwr;
static bool     s_txpwr_otp;
static uint8_t  s_prf64;
static uint8_t  s_datarate;
static uint8_t  s_ns_sfd;
static uint32_t s_part_id;
static uint32_t s_device_id;

/* Link-quality registers, captured once when radio_rx_poll() sees a good
 * frame, so radio_quality() (called separately by role_initiator.c right
 * after) does not re-read RX_FINFO (already read here for the frame length)
 * or pay a second round trip for RX_FQUAL / RX_TIME:FP_AMPL1. Valid only
 * for the most recently received frame, which is exactly the caller's
 * assumption -- role_initiator.c calls radio_quality() immediately after
 * the radio_rx()/radio_rx_poll() call that produced RADIO_RX_FRAME. */
static uint16_t s_q_std_noise;
static uint16_t s_q_f1, s_q_f2, s_q_f3, s_q_cir_pwr;
static uint32_t s_q_rxpacc;
static bool     s_q_valid;

static uint32_t us_to_uus(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * 10000ULL) / 10256ULL);
}

static void write_u16(uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    DW1000.writeBytes(reg, NO_SUB, b, 2);
}

static void write_u32(uint8_t reg, uint32_t v)
{
    uint8_t b[4] = {
        (uint8_t)(v), (uint8_t)(v >> 8),
        (uint8_t)(v >> 16), (uint8_t)(v >> 24)
    };
    DW1000.writeBytes(reg, NO_SUB, b, 4);
}

static uint32_t read_u32(uint8_t reg)
{
    uint8_t b[4] = {0};
    DW1000.readBytes(reg, NO_SUB, b, 4);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static uint64_t read_u40(uint8_t reg)
{
    uint8_t b[5] = {0};
    DW1000.readBytes(reg, NO_SUB, b, 5);
    return (uint64_t)b[0] | ((uint64_t)b[1] << 8) | ((uint64_t)b[2] << 16) |
           ((uint64_t)b[3] << 24) | ((uint64_t)b[4] << 32);
}

static void write_dx_time_hi32(uint32_t tx_hi32)
{
    /* DX_TIME low 9 bits ignored; hi32 is bits [39:8] with bit0 of hi32 forced 0. */
    uint8_t dx[5] = {0};
    uint32_t v = tx_hi32 & 0xFFFFFFFEu;
    dx[1] = (uint8_t)(v & 0xFF);
    dx[2] = (uint8_t)((v >> 8) & 0xFF);
    dx[3] = (uint8_t)((v >> 16) & 0xFF);
    dx[4] = (uint8_t)((v >> 24) & 0xFF);
    DW1000.writeBytes(DX_TIME, NO_SUB, dx, LEN_DX_TIME);
}

static void set_bit_local(uint8_t *buf, uint16_t len, uint16_t bit, bool val)
{
    uint16_t idx = bit / 8;
    uint8_t  m   = (uint8_t)(1u << (bit % 8));
    if (idx >= len) { return; }
    if (val) { buf[idx] |= m; }
    else     { buf[idx] &= (uint8_t)~m; }
}

/* Uses the library's own SYS_CFG cache (DW1000Class::_syscfg, public in
 * this vendored copy) instead of reading the register back on every call --
 * apply_cfg() ends with writeSystemConfigurationRegister(), which keeps that
 * cache in sync with the hardware, so a fresh read here was always
 * redundant. Cuts one SPI transaction from a call made on every RX arm. */
static void rx_set_timeout(uint32_t timeout_us)
{
    uint32_t uus = us_to_uus(timeout_us);

    if (timeout_us == 0u)
    {
        set_bit_local(DW1000Class::_syscfg, LEN_SYS_CFG, RXWTOE_BIT, false);
        DW1000.writeSystemConfigurationRegister();
        return;
    }

    if (uus == 0u) { uus = 1u; }
    if (uus > 65535u) { uus = 65535u; }
    write_u16(RX_FWTO, (uint16_t)uus);
    set_bit_local(DW1000Class::_syscfg, LEN_SYS_CFG, RXWTOE_BIT, true);
    DW1000.writeSystemConfigurationRegister();
}

static void clear_irq_mask(void)
{
    /* Polled SYS_STATUS — keep IRQ line quiet (ISR still attached by begin). */
    uint8_t mask[LEN_SYS_MASK];
    memset(mask, 0, sizeof(mask));
    DW1000.writeBytes(SYS_MASK, NO_SUB, mask, LEN_SYS_MASK);
}

static void apply_antenna(const cfg_t *c)
{
    uint16_t d = CFG_ANT_DELAY_DEFAULT;
    if (c->ant_delay_tx != CFG_ANT_DELAY_UNCAL)
    {
        d = c->ant_delay_tx;
    }
    else if (c->ant_delay_rx != CFG_ANT_DELAY_UNCAL)
    {
        d = c->ant_delay_rx;
    }
    s_ant_tx = d;
    s_ant_rx = (c->ant_delay_rx != CFG_ANT_DELAY_UNCAL) ? c->ant_delay_rx : d;

    /* Same value both directions (see DWM1001 radio.c commentary). */
    DW1000.setAntennaDelay(s_ant_tx);
    /* Force HW write: setAntennaDelay alone only updates RAM. */
    {
        uint8_t bytes[2] = { (uint8_t)(s_ant_tx & 0xFF), (uint8_t)(s_ant_tx >> 8) };
        DW1000.writeBytes(TX_ANTD, NO_SUB, bytes, LEN_TX_ANTD);
        DW1000.writeBytes(LDE_IF, LDE_RXANTD_SUB, bytes, LEN_LDE_RXANTD);
        if (s_ant_rx != s_ant_tx)
        {
            bytes[0] = (uint8_t)(s_ant_rx & 0xFF);
            bytes[1] = (uint8_t)(s_ant_rx >> 8);
            DW1000.writeBytes(LDE_IF, LDE_RXANTD_SUB, bytes, LEN_LDE_RXANTD);
        }
    }
}

/* Translate the config's own datarate/PRF/preamble_len encoding to the
 * library's, instead of snapping to one of its four fixed MODE_* triples.
 * The snap silently dropped precision: RATE_850 fell into the generic
 * "else" branch and transmitted at 6800 kbps regardless of what was
 * configured, RATE_110 forced PRF_64 and a 2048-symbol preamble regardless
 * of cfg.prf/cfg.preamble_len, and preamble_len/pac_size/sfd_mode away from
 * the shipped defaults were simply ignored. All of channel, prf,
 * preamble_code, preamble_len, pac_size, datarate and sfd_mode are in
 * cfg_hash() -- so two boards could agree on the hash while transmitting on
 * different PHYs, defeating the one check meant to catch exactly that. */
static byte rate_to_dw1000(uint8_t rate)
{
    switch (rate)
    {
        case RATE_110:  return DW1000Class::TRX_RATE_110KBPS;
        case RATE_850:  return DW1000Class::TRX_RATE_850KBPS;
        default:        return DW1000Class::TRX_RATE_6800KBPS;
    }
}

static byte prf_to_dw1000(uint8_t prf)
{
    return (prf == PRF_16) ? DW1000Class::TX_PULSE_FREQ_16MHZ
                           : DW1000Class::TX_PULSE_FREQ_64MHZ;
}

/* cfg_validate() (cfg.c) restricts preamble_len to exactly the IEEE set
 * below, so the default case is unreachable for a validated config; it
 * falls back to 128 (this board's shipped default) rather than silently
 * mis-programming the chip if it is ever reached anyway. */
static byte preamble_len_to_dw1000(uint16_t plen)
{
    switch (plen)
    {
        case 64:   return DW1000Class::TX_PREAMBLE_LEN_64;
        case 128:  return DW1000Class::TX_PREAMBLE_LEN_128;
        case 256:  return DW1000Class::TX_PREAMBLE_LEN_256;
        case 512:  return DW1000Class::TX_PREAMBLE_LEN_512;
        case 1024: return DW1000Class::TX_PREAMBLE_LEN_1024;
        case 1536: return DW1000Class::TX_PREAMBLE_LEN_1536;
        case 2048: return DW1000Class::TX_PREAMBLE_LEN_2048;
        case 4096: return DW1000Class::TX_PREAMBLE_LEN_4096;
        default:   return DW1000Class::TX_PREAMBLE_LEN_128;
    }
}

/* DW1000.setDataRate() hardcodes DWSFD/TNSSFD/RNSSFD and the SFD length
 * purely from the data rate (110k -> non-standard 64, 850k -> non-standard
 * 16, 6800k -> standard 8), with no way to ask for the other combination at
 * a given rate. cfg_t's sfd_mode is independently hashed and independently
 * configurable on the DWM1001 side (deca_driver's dwt_config_t.nsSFD is a
 * free field), so this overrides the rate-derived default to match
 * whatever was actually configured -- matching radio.c's mapping,
 * cfg_pac_for_preamble()'s sibling table in cfg.c:
 *   SFD_STD               -> standard 8-symbol SFD, every rate
 *   SFD_DW  at RATE_850    -> non-standard, length 16
 *   SFD_DW  elsewhere      -> non-standard, length 8 (110k already got 64
 *                             above and is left alone: forcing SFD_STD at
 *                             110 kbps is off-spec and cfg_validate() does
 *                             not offer it as a legal combination). */
static void apply_sfd_override(uint8_t rate, uint8_t sfd_mode)
{
    bool want_ns;
    byte sfd_len;

    if (rate == RATE_110)
    {
        return;   /* setDataRate() already programmed the only legal combination. */
    }

    want_ns = (sfd_mode == SFD_DW);
    sfd_len = want_ns ? (byte)((rate == RATE_850) ? 0x10 : 0x08) : (byte)0x08;

    DW1000Class::setBit(DW1000Class::_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT,  want_ns);
    DW1000Class::setBit(DW1000Class::_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, want_ns && rate != RATE_850);
    DW1000Class::setBit(DW1000Class::_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, want_ns && rate != RATE_850);
    DW1000.writeBytes(USR_SFD, SFD_LENGTH_SUB, &sfd_len, LEN_SFD_LENGTH);
}

/* DRX_SFDTOC (DRX_TUNE sub 0x20, 2 bytes) -- never programmed by the
 * Makerfabs library at all, so it always ran at its power-on default
 * regardless of configuration. Same formula as radio.c's sfd_timeout():
 * preamble_len + 1 + SFD length - PAC size, clamped to a nonzero uint16.
 * The register must never be zero (Decawave's own deca_regs.h carries the
 * same warning) -- zero disables SFD detection timeout entirely. */
#ifndef DRX_SFDTOC_SUB
#define DRX_SFDTOC_SUB    0x20
#define LEN_DRX_SFDTOC    2
#endif

static uint16_t sfd_len_symbols(uint8_t rate, uint8_t sfd_mode)
{
    if (rate == RATE_110)    { return 64u; }
    if (sfd_mode == SFD_DW)  { return (rate == RATE_850) ? 16u : 8u; }
    return 8u;
}

static void apply_sfd_timeout(const cfg_t *c)
{
    uint8_t pac = (c->pac_size != 0u) ? c->pac_size : DW1000Class::_pacSize;
    int32_t t = (int32_t)c->preamble_len + 1
              + (int32_t)sfd_len_symbols(c->datarate, c->sfd_mode) - (int32_t)pac;
    uint16_t sfdto;

    if (t < 1)     { t = 1; }
    if (t > 65535) { t = 65535; }
    sfdto = (uint16_t)t;

    {
        uint8_t b[LEN_DRX_SFDTOC] = { (uint8_t)(sfdto & 0xFF), (uint8_t)(sfdto >> 8) };
        DW1000.writeBytes(DRX_TUNE, DRX_SFDTOC_SUB, b, LEN_DRX_SFDTOC);
    }
}

static void apply_cfg(const cfg_t *c)
{
    s_prf64    = (c->prf == PRF_64) ? 1u : 0u;
    s_datarate = c->datarate;
    s_ns_sfd   = (c->sfd_mode == SFD_DW) ? 1u : 0u;

    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDataRate(rate_to_dw1000(c->datarate));
    DW1000.setPulseFrequency(prf_to_dw1000(c->prf));
    DW1000.setPreambleLength(preamble_len_to_dw1000(c->preamble_len));
    /* Manual PAC override. cfg_pac_for_preamble()'s own auto table (8/16/
     * 32/64, keyed on preamble length) matches what setPreambleLength()
     * just derived above, so this is a no-op at pac_size=0 (auto). */
    if (c->pac_size != 0u)
    {
        DW1000Class::_pacSize = (byte)c->pac_size;
    }
    apply_sfd_override(c->datarate, c->sfd_mode);
    apply_sfd_timeout(c);
    DW1000.setChannel((byte)c->channel);
    DW1000.setPreambleCode((byte)c->preamble_code);
    DW1000.setNetworkId(c->pan_id);
    DW1000.setDeviceAddress(c->node_id);
    DW1000.commitConfiguration();

    apply_antenna(c);

    /* large_power_init after commit — Makerfabs Pro PA / GPIO tweaks. Sets
     * TX_POWER = 0x1F1F1F1F (max), appropriate as the TXP_AUTO default for a
     * board with an external PA. */
    DW1000.large_power_init();

    /* txpower_mode/coarse/fine/reg are live, settable serial keys ($STAT
     * reports the programmed register), but until here nothing ever read
     * them back -- TXP_MANUAL was silently a no-op, with no way to turn
     * power down for close-range bench testing. TXP_MANUAL semantics match
     * the DWM1001 reference's apply_txpower(): txpower_reg verbatim if
     * nonzero, else (coarse<<5)|fine replicated across all four bytes
     * (smart power is off, so all four must carry the same value). */
    if (c->txpower_mode == TXP_MANUAL)
    {
        uint32_t reg;
        if (c->txpower_reg != 0u)
        {
            reg = c->txpower_reg;
        }
        else
        {
            uint8_t b = (uint8_t)(((c->txpower_coarse & 0x07u) << 5) |
                                   (c->txpower_fine & 0x1Fu));
            reg = ((uint32_t)b << 24) | ((uint32_t)b << 16) |
                  ((uint32_t)b << 8) | (uint32_t)b;
        }
        write_u32(TX_POWER, reg);
    }

    /* Makerfabs tune() writes LDE_CFG1 = 0x0D (PMULT=0). Decawave's own
     * parameter table (PEAK_MULTPLIER 0x60 | N_STD_FACTOR 13 = 0x6D) is what
     * the DWM1001 reference firmware runs, and this field shifts the LDE
     * leading-edge threshold, i.e. every RX timestamp. A fleet mixing this
     * board against the DWM1001 default carries a fixed range bias unless
     * corrected here. */
    {
        uint8_t lde_cfg1 = 0x6D;
        DW1000.writeBytes(LDE_IF, LDE_CFG1_SUB, &lde_cfg1, LEN_LDE_CFG1);
    }

    /* setDefaults() (called above via newConfiguration path) leaves frame
     * filtering off. Software already checks dst address in handle_frame(),
     * so this is not a correctness bug, but at resp_count=8 every anchor
     * would otherwise read out and discard 7 other anchors' unicast
     * Responses/Reports on every frame -- real critical-path SPI time this
     * firmware's timing budget does not have room for. Listener role wants
     * everything, so it stays unfiltered. */
    if (c->role != ROLE_LSTN)
    {
        DW1000.setFrameFilter(true);
        DW1000.setFrameFilterAllowData(true);
        DW1000.writeSystemConfigurationRegister();
    }
    else
    {
        DW1000.setFrameFilter(false);
        DW1000.writeSystemConfigurationRegister();
    }

    s_txpwr = read_u32(TX_POWER);
    s_txpwr_otp = false;

    clear_irq_mask();
    rx_set_timeout(0);
}

bool radio_init(const cfg_t *c)
{
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    DW1000.begin(PIN_UWB_IRQ, PIN_UWB_RST);
    /* begin() unconditionally attaches handleInterrupt() on RISING, and that
     * handler performs its own SPI transactions -- if it ever fired mid
     * critical-path (arming a delayed TX/RX from the ranging loop, itself
     * doing SPI) it would corrupt the transaction. SYS_MASK is 0 out of
     * reset and clear_irq_mask() keeps it 0 outside the narrow SPI-only
     * config window in apply_cfg(), which currently makes this safe by
     * construction (nothing is ever receiving or transmitting during that
     * window). Detaching outright removes the dependency on that ordering
     * entirely -- this firmware is fully polled and never needs the ISR. */
    detachInterrupt(digitalPinToInterrupt(PIN_UWB_IRQ));
    DW1000.select(PIN_UWB_SS);

    /* dev=0x... in the boot banner exists as a hardware sanity check: every
     * genuine DW1000 reads back the fixed constant 0xDECA0130 from DEV_ID
     * (0x00), which is what the DWM1001 reference reports via
     * dwt_readdevid(). Hashing the printable identifier STRING instead (the
     * previous implementation here) produces a plausible-looking number
     * regardless of whether the chip identity is actually correct, which
     * defeats the check -- caught on real hardware, where it printed
     * 0xBA26FA53 for a board that was in fact working correctly. */
    s_device_id = read_u32(DEV_ID);

    /* cfg_apply_identity() folds this into node_id (the on-air address) when
     * node_id is unset, so it MUST be unique per board. It is not safe to
     * derive it from the DW1000's own identity here: confirmed on hardware
     * that two different boards produced the exact same part_id
     * (0xB8F941FD) and therefore the exact same auto-assigned node_id
     * (0xF904) -- these Makerfabs boards evidently ship with unprogrammed
     * OTP, so getPrintableExtendedUniqueIdentifier() reads back the same
     * default EUI on every unit, and hashing it just reproduces that
     * collision deterministically. The ESP32 itself carries a factory-burned
     * globally-unique MAC (Espressif-provisioned, not something this board
     * shares with any other), which is the only identity source here that
     * is actually guaranteed unique per physical unit. */
    {
        uint64_t mac = ESP.getEfuseMac();
        s_part_id = (uint32_t)(mac ^ (mac >> 32));
    }

    apply_cfg(c);
    return true;
}

bool radio_reconfigure(const cfg_t *c)
{
    DW1000.idle();
    apply_cfg(c);
    return true;
}

uint32_t radio_systime_hi32(void)
{
    uint64_t t = read_u40(SYS_TIME);
    return (uint32_t)((t >> 8) & 0xFFFFFFFFu);
}

int32_t radio_hi32_diff_us(uint32_t a, uint32_t b)
{
    int32_t d = (int32_t)(a - b);
    return (int32_t)(((int64_t)d * 1000LL) / 249600LL);
}

uint64_t radio_tx_ts_from_hi32(uint32_t tx_hi32)
{
    return ((((uint64_t)(tx_hi32 & 0xFFFFFFFEu)) << 8) + (uint64_t)s_ant_tx)
           & DTU_MASK_40;
}

int32_t radio_carrier_integrator(void)
{
    /* DRX_TUNE (0x27) sub 0x28, 3 bytes, 21-bit signed value -- register
     * layout verified against vendor/deca_driver's dwt_readcarrierintegrator()
     * (DRX_CARRIER_INT_OFFSET 0x28, LEN 3, sign bit 20). Only feeds the
     * SS-TWR fallback (ranging_k_hat()), used when a Report frame is lost;
     * the primary DS-TWR path (ranging_tof_dtu(), the default with
     * report_enable=1) does not read this at all. */
    uint8_t b[3] = {0};
    DW1000.readBytes(DRX_TUNE, 0x28, b, 3);
    uint32_t v = ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[0];
    if (v & 0x00100000u) { v |= 0xFFF00000u; }
    else                 { v &= 0x001FFFFFu; }
    return (int32_t)v;
}

void radio_idle(void) { DW1000.idle(); }

uint32_t radio_txpower_reg(void) { return s_txpwr; }
bool     radio_txpower_from_otp(void) { return s_txpwr_otp; }
uint16_t radio_ant_tx(void) { return s_ant_tx; }
uint16_t radio_ant_rx(void) { return s_ant_rx; }
uint32_t radio_part_id(void) { return s_part_id; }
uint32_t radio_device_id(void) { return s_device_id; }

static uint64_t read_rx_ts(void)
{
    /* The DW1000Time& overload silently calls correctTimestamp(), which does
     * an extra getReceivePower() SPI round trip and subtracts a power-
     * dependent range-bias table calibrated for Decawave's own reference
     * board. The DWM1001 firmware this is ported from reads the raw
     * timestamp and does its own antenna-delay calibration with no bias
     * table (ranging.c's round-to-nearest arithmetic assumes that). Stacking
     * this library's bias on top would silently corrupt every range by a
     * signal-strength-dependent amount and cost an unbudgeted SPI read on
     * every RX event. Use the raw byte[5] overload instead. */
    byte ts[5];
    DW1000.getReceiveTimestamp(ts);
    return ((uint64_t)ts[0]) | ((uint64_t)ts[1] << 8) | ((uint64_t)ts[2] << 16) |
           ((uint64_t)ts[3] << 24) | ((uint64_t)ts[4] << 32);
}

/* NOT redundant with DW1000.newTransmit()'s internal clearTransmitStatus(),
 * despite both writing SYS_STATUS right before a delayed TX: this clears
 * ALL latched bits (0xFF), including HPDWARN and TXPUTE, which the
 * library's clearTransmitStatus() does not touch (it only clears
 * TXFRB/TXPRS/TXPHS/TXFRS). HPDWARN/TXPUTE are exactly the bits the late
 * check in radio_tx_at() reads back -- leaving them latched from a
 * previous late TX would permanently read back "late" on every subsequent
 * attempt (the DWM1001 reference's radio.c carries the same two bits in
 * its own tx_status_clear() for the same reason). Do not remove this call. */
static void tx_status_clear(void)
{
    uint8_t st[LEN_SYS_STATUS];
    memset(st, 0xFF, sizeof(st));
    DW1000.writeBytes(SYS_STATUS, NO_SUB, st, LEN_SYS_STATUS);
}

bool radio_rx_arm(uint32_t timeout_us, bool delayed, uint32_t start_hi32)
{
    rx_set_timeout(timeout_us);
    DW1000.newReceive();
    DW1000.setDefaults();

    if (delayed)
    {
        /* Arm RXDLYS directly. DW1000.setDelay() would do this too, but at
         * the cost of a SYS_TIME read and a DX_TIME write that
         * write_dx_time_hi32() immediately overwrites -- setBit()/_sysctrl
         * are public in this vendored copy (its `private:` is commented
         * out), so the one bit this path actually needs can be set without
         * paying for the rest of setDelay(). */
        DW1000Class::setBit(DW1000Class::_sysctrl, LEN_SYS_CTRL, RXDLYS_BIT, true);
        write_dx_time_hi32(start_hi32);

        /* If start already passed, chip will set HPDWARN on enable. */
        DW1000.startReceive();
        {
            uint8_t st[LEN_SYS_STATUS];
            DW1000.readBytes(SYS_STATUS, NO_SUB, st, LEN_SYS_STATUS);
            if ((st[3] & (1u << (HPDWARN_BIT - 24))) != 0)
            {
                DW1000.idle();
                return false;
            }
        }
        return true;
    }

    DW1000.startReceive();
    return true;
}

int radio_rx_poll(radio_rx_t *rx)
{
    uint8_t st[LEN_SYS_STATUS];
    DW1000.readBytes(SYS_STATUS, NO_SUB, st, LEN_SYS_STATUS);

    bool good = ((st[RXFCG_BIT / 8] & (1u << (RXFCG_BIT % 8))) != 0);
    bool to   = ((st[RXRFTO_BIT / 8] & (1u << (RXRFTO_BIT % 8))) != 0) ||
                ((st[RXPTO_BIT / 8] & (1u << (RXPTO_BIT % 8))) != 0) ||
                ((st[RXSFDTO_BIT / 8] & (1u << (RXSFDTO_BIT % 8))) != 0);
    bool err  = ((st[RXPHE_BIT / 8] & (1u << (RXPHE_BIT % 8))) != 0) ||
                ((st[RXFCE_BIT / 8] & (1u << (RXFCE_BIT % 8))) != 0) ||
                ((st[RXRFSL_BIT / 8] & (1u << (RXRFSL_BIT % 8))) != 0) ||
                ((st[LDEERR_BIT / 8] & (1u << (LDEERR_BIT % 8))) != 0);

    if (!good && !to && !err)
    {
        return RADIO_RX_PENDING;
    }

    rx->status = (uint32_t)st[0] | ((uint32_t)st[1] << 8) |
                 ((uint32_t)st[2] << 16) | ((uint32_t)st[3] << 24);

    if (good)
    {
        uint8_t finfo[LEN_RX_FINFO];
        DW1000.readBytes(RX_FINFO, NO_SUB, finfo, LEN_RX_FINFO);
        uint16_t frame_len = (uint16_t)(finfo[0] | ((finfo[1] & 0x03) << 8));
        if (frame_len < (MHR_LEN + FCS_LEN) || frame_len > MAX_FRAME_LEN)
        {
            DW1000.idle();
            return RADIO_RX_ERROR;
        }
        rx->len = (uint16_t)(frame_len - FCS_LEN);
        DW1000.getData(rx->buf, rx->len);
        rx->rx_ts = read_rx_ts();

        /* Link quality, captured now while we already know a frame is here
         * -- see the s_q_* cache comment above. rxpacc comes from the same
         * finfo bytes already read for the frame length, so this is one
         * fewer RX_FINFO round trip than reading it again from
         * radio_quality(). */
        {
            uint8_t fq[8] = {0};
            DW1000.readBytes(RX_FQUAL, NO_SUB, fq, 8);
            s_q_std_noise = (uint16_t)fq[0] | ((uint16_t)fq[1] << 8);
            s_q_f2        = (uint16_t)fq[2] | ((uint16_t)fq[3] << 8);
            s_q_f3        = (uint16_t)fq[4] | ((uint16_t)fq[5] << 8);
            s_q_cir_pwr   = (uint16_t)fq[6] | ((uint16_t)fq[7] << 8);

            uint8_t fp1[2] = {0};
            DW1000.readBytes(RX_TIME, FP_AMPL1_SUB, fp1, 2);
            s_q_f1 = (uint16_t)fp1[0] | ((uint16_t)fp1[1] << 8);

            s_q_rxpacc = ((((uint32_t)finfo[2]) >> 4) & 0xFFu) | (((uint32_t)finfo[3]) << 4);
            s_q_valid  = true;
        }

        /* Clear RX events */
        uint8_t clr[LEN_SYS_STATUS];
        memset(clr, 0, sizeof(clr));
        set_bit_local(clr, LEN_SYS_STATUS, RXFCG_BIT, true);
        DW1000.writeBytes(SYS_STATUS, NO_SUB, clr, LEN_SYS_STATUS);
        return RADIO_RX_FRAME;
    }

    if (to)
    {
        uint8_t clr[LEN_SYS_STATUS];
        memset(clr, 0xFF, sizeof(clr));
        DW1000.writeBytes(SYS_STATUS, NO_SUB, clr, LEN_SYS_STATUS);
        DW1000.idle();
        return RADIO_RX_TIMEOUT;
    }

    {
        uint8_t clr[LEN_SYS_STATUS];
        memset(clr, 0xFF, sizeof(clr));
        DW1000.writeBytes(SYS_STATUS, NO_SUB, clr, LEN_SYS_STATUS);
        DW1000.idle();
        return RADIO_RX_ERROR;
    }
}

int radio_rx(radio_rx_t *rx, uint32_t timeout_us, bool delayed, uint32_t start_hi32)
{
    int r;
    if (!radio_rx_arm(timeout_us, delayed, start_hi32))
    {
        return RADIO_RX_TIMEOUT;
    }
    do { r = radio_rx_poll(rx); } while (r == RADIO_RX_PENDING);
    return r;
}

int radio_tx_at(const uint8_t *buf, uint16_t len, uint32_t tx_hi32)
{
    /* len includes FCS (2). Makerfabs setData expects payload without FCS when
     * frame check is enabled (default). */
    uint16_t payload = (len >= FCS_LEN) ? (uint16_t)(len - FCS_LEN) : len;

    tx_status_clear();
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData((byte *)buf, payload);

    /* Arm TXDLYS directly -- see the matching comment in radio_rx_arm(). */
    DW1000Class::setBit(DW1000Class::_sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
    write_dx_time_hi32(tx_hi32);

    DW1000.startTransmit();

    /* Late? HPDWARN / TXPUTE */
    {
        uint8_t st[LEN_SYS_STATUS];
        DW1000.readBytes(SYS_STATUS, NO_SUB, st, LEN_SYS_STATUS);
        bool late = ((st[3] & (1u << (HPDWARN_BIT - 24))) != 0) ||
                    ((st[4] & 0x04u) != 0); /* TXPUTE */
        if (late)
        {
            DW1000.idle();
            tx_status_clear();
            return RADIO_TX_LATE;
        }
    }
    return RADIO_TX_OK;
}

#define TX_WAIT_MAX_US 20000u

void radio_tx_wait(void)
{
    uint32_t start = radio_systime_hi32();
    uint32_t n = 0;
    for (;;)
    {
        uint8_t st[LEN_SYS_STATUS];
        DW1000.readBytes(SYS_STATUS, NO_SUB, st, LEN_SYS_STATUS);
        if ((st[0] & (1u << TXFRS_BIT)) != 0)
        {
            uint8_t clr[LEN_SYS_STATUS];
            memset(clr, 0, sizeof(clr));
            set_bit_local(clr, LEN_SYS_STATUS, TXFRS_BIT, true);
            DW1000.writeBytes(SYS_STATUS, NO_SUB, clr, LEN_SYS_STATUS);
            return;
        }
        if ((++n & 0x3Fu) == 0u &&
            radio_hi32_diff_us(radio_systime_hi32(), start) > (int32_t)TX_WAIT_MAX_US)
        {
            DW1000.idle();
            tx_status_clear();
            return;
        }
    }
}

void radio_quality(radio_quality_t *q)
{
    /* DW1000.getReceivePower()/getFirstPathPower() apply an extra "Fig. 22"
     * correction (estPwr += (estPwr+88)*corrFac above -88 dBm) that the
     * DWM1001 reference's radio_quality() never applies. That correction is
     * affine with the same intercept on both quantities, so it scales
     * (rx_dbm10 - fp_dbm10) by (1+corrFac) -- ~2.17x at 64 MHz PRF -- above
     * -88 dBm, which is the common indoor case. The reference's
     * NLOS_THRESHOLD_DBM10 (10.0 dB) is calibrated against the uncorrected
     * figure, so feeding it the library's corrected numbers would trip NLOS
     * far too easily. Compute the raw formula directly from registers
     * instead, matching radio.c's radio_quality() verbatim -- but from the
     * cache radio_rx_poll() filled in when the frame landed, not a fresh
     * read here (see the s_q_* comment above). */
    memset(q, 0, sizeof(*q));

    if (!s_q_valid) { return; }

    uint16_t std_noise = s_q_std_noise;
    uint16_t f2        = s_q_f2;
    uint16_t f3        = s_q_f3;
    uint16_t cir_pwr   = s_q_cir_pwr;
    uint16_t f1        = s_q_f1;
    uint32_t rxpacc    = s_q_rxpacc;

    uint32_t adj;
    if (s_datarate == RATE_110)      { adj = 64u; }
    else if (s_ns_sfd == 0u)         { adj = 5u; }
    else if (s_datarate == RATE_850) { adj = 8u; }
    else                             { adj = 10u; }

    q->std_noise = std_noise;
    q->rxpacc    = (uint16_t)rxpacc;

    if (rxpacc <= adj) { return; }
    float n = (float)(rxpacc - adj);
    float a = s_prf64 ? 121.74f : 113.77f;

    if (cir_pwr > 0u)
    {
        float v = 10.0f * log10f(((float)cir_pwr * 131072.0f) / (n * n)) - a;
        q->rx_dbm10 = (int16_t)lroundf(v * 10.0f);
    }
    if ((uint32_t)f1 + f2 + f3 > 0u)
    {
        float v = 10.0f * log10f(((float)f1 * f1 + (float)f2 * f2 + (float)f3 * f3) / (n * n)) - a;
        q->fp_dbm10 = (int16_t)lroundf(v * 10.0f);
    }
}
