#include "radio.h"
#include "timing.h"

#include <string.h>
#include <math.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

/* DW1000 OTP map: TX power calibration, two words per channel (16 MHz then
 * 64 MHz PRF), for channels 1,2,3,4,5,7. Antenna delay lives at 0x1C. */
#define OTP_TXPOWER_BASE    0x10u
#define OTP_ANTDLY_ADDR     0x1Cu

/* Decawave reference TX power, used when OTP is blank. Smart power disabled,
 * so all four bytes of the register carry the same value. */
static const uint32_t k_txpwr_ref[2][8] = {
    /* 16 MHz PRF, indexed by channel (0 and 6 unused) */
    { 0, 0x75757575u, 0x75757575u, 0x6F6F6F6Fu, 0x5F5F5F5Fu, 0x48484848u, 0, 0x92929292u },
    /* 64 MHz PRF */
    { 0, 0x67676767u, 0x67676767u, 0x8B8B8B8Bu, 0x9A9A9A9Au, 0x85858585u, 0, 0xD1D1D1D1u },
};

/* PG delay, indexed by channel. */
static const uint8_t k_pgdelay[8] = {
    0, 0xC9u, 0xC2u, 0xC5u, 0x95u, 0xC0u, 0, 0x93u
};

static dwt_config_t m_dw;
static uint16_t     m_ant_tx;
static uint16_t     m_ant_rx;
static uint32_t     m_txpwr;
static bool         m_txpwr_otp;
static uint8_t      m_prf;

/* ------------------------------------------------------------------------- */

static uint8_t plen_to_dwt(uint16_t p)
{
    switch (p)
    {
        case 64:   return DWT_PLEN_64;
        case 128:  return DWT_PLEN_128;
        case 256:  return DWT_PLEN_256;
        case 512:  return DWT_PLEN_512;
        case 1024: return DWT_PLEN_1024;
        case 1536: return DWT_PLEN_1536;
        case 2048: return DWT_PLEN_2048;
        default:   return DWT_PLEN_4096;
    }
}

static uint8_t pac_to_dwt(uint8_t pac)
{
    switch (pac)
    {
        case 8:  return DWT_PAC8;
        case 16: return DWT_PAC16;
        case 32: return DWT_PAC32;
        default: return DWT_PAC64;
    }
}

static uint8_t rate_to_dwt(uint8_t r)
{
    switch (r)
    {
        case RATE_110: return DWT_BR_110K;
        case RATE_850: return DWT_BR_850K;
        default:       return DWT_BR_6M8;
    }
}

/** Standard SFD is 64 symbols at 110 kbps and 8 otherwise; the Decawave
 *  non-standard SFD is 64/16/8 by rate (deca_regs.h DW_NS_SFD_LEN_*). */
static uint16_t sfd_len(const cfg_t *c)
{
    if (c->datarate == RATE_110) { return 64u; }
    if (c->sfd_mode == SFD_DW)   { return (c->datarate == RATE_850) ? 16u : 8u; }
    return 8u;
}

static uint16_t sfd_timeout(const cfg_t *c, uint8_t pac)
{
    /* preamble length + 1 + SFD length - PAC size */
    int32_t t = (int32_t)c->preamble_len + 1 + (int32_t)sfd_len(c) - (int32_t)pac;
    if (t < 1)     { t = 1; }
    if (t > 65535) { t = 65535; }
    return (uint16_t)t;
}

static uint32_t us_to_uus(uint32_t us)
{
    /* 1 uus = 512/499.2 us = 1.025641 us */
    return (uint32_t)(((uint64_t)us * 10000ULL) / 10256ULL);
}

/* ------------------------------------------------------------------------- */

/* channel -> {16 MHz PRF word, 64 MHz PRF word}, read once at boot. Cached
 * because dwt_otpread() switches the DW1000's own digital logic onto its
 * external XTI clock for the duration of the read (see _dwt_enableclocks() in
 * deca_device.c) -- a sequence that is only reliable at the SPI rate used
 * during dwt_initialise() (2 MHz). Calling it again later, after the port has
 * already moved to the 8 MHz "fast rate" used for normal operation, corrupts
 * every subsequent register read (observed: dwt_readdevid() returning
 * 0xFFFFFFFF instead of 0xDECA0130 for the remainder of the session). Reading
 * every channel's calibration once, while still at slow rate, avoids ever
 * exercising that path again. */
static uint32_t m_otp_txpwr[8][2];
static bool     m_otp_txpwr_valid[8][2];

/* Section 8.1 gives ant_delay_tx/rx the default "calibrated", and section 13
 * step 1 calibrates delta at a surveyed baseline. The DW1000 carries a
 * factory antenna-delay word at OTP 0x1C (16 MHz PRF in the low half, 64 MHz
 * in the high half); it is read here, in the same slow-rate window, because
 * of the dwt_otpread() constraint described above. */
static uint16_t m_otp_antdly[2];
static bool     m_otp_antdly_valid[2];

static void cache_otp_txpower(void)
{
    static const uint8_t ch_idx[8] = { 0, 0, 1, 2, 3, 4, 0, 5 };
    uint8_t chan;

    {
        uint32_t word = 0;

        dwt_otpread(OTP_ANTDLY_ADDR, &word, 1);

        if (word != 0u && word != 0xFFFFFFFFu)
        {
            uint16_t d16 = (uint16_t)(word & 0xFFFFu);
            uint16_t d64 = (uint16_t)((word >> 16) & 0xFFFFu);

            if (d16 != 0u && d16 != 0xFFFFu)
            {
                m_otp_antdly[0]       = d16;
                m_otp_antdly_valid[0] = true;
            }
            if (d64 != 0u && d64 != 0xFFFFu)
            {
                m_otp_antdly[1]       = d64;
                m_otp_antdly_valid[1] = true;
            }
        }
    }

    for (chan = 1; chan <= 7; chan++)
    {
        uint8_t prf;

        if (chan == 6u) { continue; }

        for (prf = 0; prf < 2; prf++)
        {
            uint32_t word = 0;
            uint32_t addr = OTP_TXPOWER_BASE + (uint32_t)ch_idx[chan] * 2u + prf;

            dwt_otpread(addr, &word, 1);

            /* Blank OTP reads as 0x00000000 or 0xFFFFFFFF. */
            if (word != 0u && word != 0xFFFFFFFFu)
            {
                m_otp_txpwr[chan][prf]       = word;
                m_otp_txpwr_valid[chan][prf] = true;
            }
        }
    }
}

static uint32_t otp_txpower(uint8_t chan, uint8_t prf)
{
    if (chan == 0u || chan == 6u || chan > 7u) { return 0; }
    return m_otp_txpwr_valid[chan][prf] ? m_otp_txpwr[chan][prf] : 0u;
}

static void apply_txpower(const cfg_t *c)
{
    dwt_txconfig_t tx;

    if (c->txpower_mode == TXP_MANUAL)
    {
        if (c->txpower_reg != 0u)
        {
            m_txpwr = c->txpower_reg;
        }
        else
        {
            uint8_t b = (uint8_t)(((c->txpower_coarse & 0x07u) << 5)
                                 | (c->txpower_fine & 0x1Fu));
            m_txpwr = ((uint32_t)b << 24) | ((uint32_t)b << 16)
                    | ((uint32_t)b << 8)  | (uint32_t)b;
        }
        m_txpwr_otp = false;
    }
    else
    {
        uint8_t prf_idx = (c->prf == PRF_64) ? 1u : 0u;

        m_txpwr = otp_txpower(c->channel, prf_idx);
        m_txpwr_otp = (m_txpwr != 0u);
        if (!m_txpwr_otp)
        {
            m_txpwr = k_txpwr_ref[prf_idx][c->channel & 7u];
        }
    }

    tx.PGdly = k_pgdelay[c->channel & 7u];
    tx.power = m_txpwr;
    dwt_configuretxrf(&tx);

    /* Constant power per slot: smart TX power boosts short frames, which would
     * make the link budget differ between message types. */
    dwt_setsmarttxpower(c->smart_power ? 1 : 0);
}

static void apply_cfg(const cfg_t *c)
{
    uint8_t pac = (c->pac_size != 0u) ? c->pac_size
                                      : cfg_pac_for_preamble(c->preamble_len);

    m_dw.chan           = c->channel;
    m_dw.prf            = (c->prf == PRF_64) ? DWT_PRF_64M : DWT_PRF_16M;
    m_dw.txPreambLength = plen_to_dwt(c->preamble_len);
    m_dw.rxPAC          = pac_to_dwt(pac);
    m_dw.txCode         = c->preamble_code;
    m_dw.rxCode         = c->preamble_code;
    m_dw.nsSFD          = (c->sfd_mode == SFD_DW) ? 1u : 0u;
    m_dw.dataRate       = rate_to_dwt(c->datarate);
    m_dw.phrMode        = DWT_PHRMODE_STD;
    m_dw.sfdTO          = sfd_timeout(c, pac);

    m_prf = m_dw.prf;

    dwt_configure(&m_dw);
    apply_txpower(c);

    /* Section 8.1 default is "calibrated": prefer this part's own OTP value
     * when the configuration still carries the uncalibrated sentinel. An
     * explicitly configured delay always wins.
     *
     * The SAME value goes into both registers. This used to be halved across
     * them, which is wrong: dwt_settxantennadelay() and
     * dwt_setrxantennadelay() each take the full one-way delay for their own
     * direction, and Decawave's own examples program the identical figure to
     * both. Halving under-compensates every one of the four timestamps in an
     * exchange, and the shortfall lands on tau as
     *   4 * (delay/2) / 2 = delay
     * -- 16436 device time units, 257 ns, 77.2 m. Measured on hardware as
     * exactly that: four ranges that should have read about 1 m all read
     * 77.8 m, and setting the registers to the full value collapsed them to
     * 0.81-1.00 m. An earlier round reasoned the opposite way from a
     * previously calibrated board and got this backwards. */
    m_ant_tx = c->ant_delay_tx;
    m_ant_rx = c->ant_delay_rx;

    {
        uint8_t prf_idx = (c->prf == PRF_64) ? 1u : 0u;
        bool    uncal   = (c->ant_delay_tx == CFG_ANT_DELAY_UNCAL
                           && c->ant_delay_rx == CFG_ANT_DELAY_UNCAL);

        if (uncal)
        {
            uint16_t d = m_otp_antdly_valid[prf_idx]
                       ? m_otp_antdly[prf_idx]
                       : CFG_ANT_DELAY_DEFAULT;

            m_ant_tx = d;
            m_ant_rx = d;
        }
    }

    dwt_settxantennadelay(m_ant_tx);
    dwt_setrxantennadelay(m_ant_rx);

    /* Hardware address filtering: Response and Report are unicast to the
     * initiator, so every other responder drops them inside the DW1000.
     *
     * The listener is the exception -- it exists to observe traffic addressed
     * to other nodes, so filtering must be off or it would drop every Response
     * and Report and see only the broadcast Poll and Final. */
    dwt_setpanid(c->pan_id);
    dwt_setaddress16(c->node_id);
    dwt_enableframefilter((c->role == ROLE_LSTN) ? DWT_FF_NOTYPE_EN
                                                 : DWT_FF_DATA_EN);

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
}

bool radio_init(const cfg_t *c)
{
    reset_DW1000();
    port_set_dw1000_slowrate();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        return false;
    }

    /* Must happen here, still at the 2 MHz slow rate -- see cache_otp_txpower(). */
    cache_otp_txpower();

    port_set_dw1000_fastrate();

    apply_cfg(c);
    dwt_setleds(DWT_LEDS_ENABLE);
    return true;
}

bool radio_reconfigure(const cfg_t *c)
{
    dwt_forcetrxoff();
    apply_cfg(c);
    return true;
}

/* ------------------------------------------------------------------------- */

uint32_t radio_systime_hi32(void)          { return dwt_readsystimestamphi32(); }
int32_t  radio_carrier_integrator(void)    { return dwt_readcarrierintegrator(); }
uint32_t radio_part_id(void)               { return (uint32_t)dwt_getpartid(); }
uint32_t radio_device_id(void)             { return (uint32_t)dwt_readdevid(); }
uint16_t radio_ant_tx(void)                { return m_ant_tx; }
uint16_t radio_ant_rx(void)                { return m_ant_rx; }
uint32_t radio_txpower_reg(void)           { return m_txpwr; }
bool     radio_txpower_from_otp(void)      { return m_txpwr_otp; }
void     radio_idle(void)                  { dwt_forcetrxoff(); }

int32_t radio_hi32_diff_us(uint32_t a, uint32_t b)
{
    /* One hi32 unit is 4.0064 ns, so 249.6 units per microsecond. */
    int32_t d = (int32_t)(a - b);
    return (int32_t)(((int64_t)d * 1000LL) / 249600LL);
}

uint64_t radio_tx_ts_from_hi32(uint32_t tx_hi32)
{
    /* Bit 0 must be masked off. dwt_setdelayedtrxtime() writes this value at
     * DX_TIME offset 1 because "the lower 9 bits of this register are ignored"
     * -- bit 0 here is bit 8 of the 40-bit DX_TIME, so the chip discards it.
     * Without the mask the timestamp we advertise is 256 device-time units
     * (~4.0 ns, ~1.2 m) later than the frame actually left, on the ~50% of
     * frames where bit 0 happens to be set. That value goes on the air in the
     * Poll/Response/Final payloads and straight into the estimator. */
    return ((((uint64_t)(tx_hi32 & 0xFFFFFFFEu)) << 8) + (uint64_t)m_ant_tx)
           & DTU_MASK_40;
}

static uint64_t read_rx_ts(void)
{
    uint8_t  ts[5];
    uint64_t v = 0;
    int      i;

    dwt_readrxtimestamp(ts);
    for (i = 4; i >= 0; i--) { v = (v << 8) | (uint64_t)ts[i]; }
    return v;
}

/* dwt_setrxtimeout(0) disables the frame-wait timeout in the driver
 * (SYS_CFG.RXWTOE cleared), which blocks forever rather than timing out.
 * A genuine zero-length window should be treated as "expired immediately",
 * not "wait forever" -- so any caller-requested timeout that rounds down
 * to zero uus is bumped to the smallest representable nonzero timeout
 * instead. Callers that truly want no timeout must not be on this path;
 * none in this codebase do. */
static void rx_set_timeout(uint32_t timeout_us)
{
    uint32_t uus = us_to_uus(timeout_us);

    if (timeout_us > 0u && uus == 0u) { uus = 1u; }
    if (uus > 65535u) { uus = 65535u; }
    dwt_setrxtimeout((uint16_t)uus);
}

/* Arming and collecting are separated so a caller can do useful work while the
 * DW1000 receives. The initiator needs exactly that: it must open the window
 * for the next initiator's Poll *before* spending ~688 us writing its eight
 * range records to the UART, because eq (14) sizes the frame from air time,
 * slot pitches and guard and never accounted for that output. With the two
 * fused together the write had to finish first, which left the receiver armed
 * within ~200 us of the peer's preamble -- and the node in frame slot 0 lost
 * that race on all but 7 of 12690 frames, then free-ran and stamped
 * RNG_FLAG_DESYNC on everything it emitted. */
bool radio_rx_arm(uint32_t timeout_us, bool delayed, uint32_t start_hi32)
{
    rx_set_timeout(timeout_us);

    if (delayed)
    {
        dwt_setdelayedtrxtime(start_hi32);
        if (dwt_rxenable(DWT_START_RX_DELAYED | DWT_IDLE_ON_DLY_ERR) != DWT_SUCCESS)
        {
            return false;               /* start instant already passed */
        }
        return true;
    }

    (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return true;
}

int radio_rx_poll(radio_rx_t *rx)
{
    uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
    uint16_t frame_len;

    if ((status & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO |
                   SYS_STATUS_ALL_RX_ERR)) == 0)
    {
        return RADIO_RX_PENDING;
    }

    rx->status = status;

    if (status & SYS_STATUS_RXFCG)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        frame_len = (uint16_t)(dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK);
        if (frame_len < (MHR_LEN + FCS_LEN) || frame_len > MAX_FRAME_LEN)
        {
            dwt_rxreset();
            return RADIO_RX_ERROR;
        }

        rx->len = (uint16_t)(frame_len - FCS_LEN);
        dwt_readrxdata(rx->buf, rx->len, 0);
        rx->rx_ts = read_rx_ts();
        return RADIO_RX_FRAME;
    }

    if (status & SYS_STATUS_ALL_RX_TO)
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
        dwt_rxreset();
        return RADIO_RX_TIMEOUT;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    dwt_rxreset();
    return RADIO_RX_ERROR;
}

int radio_rx(radio_rx_t *rx, uint32_t timeout_us, bool delayed, uint32_t start_hi32)
{
    int r;

    if (!radio_rx_arm(timeout_us, delayed, start_hi32))
    {
        return RADIO_RX_TIMEOUT;        /* start instant already passed */
    }

    do
    {
        r = radio_rx_poll(rx);
    }
    while (r == RADIO_RX_PENDING);

    return r;
}


/* Clear every TX-completion and TX-error latch.
 *
 * SYS_STATUS_ALL_TX is only (AAT|TXFRB|TXPRS|TXPHS|TXFRS) -- it does NOT
 * include HPDWARN or TXPUTE, the two bits dwt_starttx() actually tests to
 * decide a delayed transmit was too late. Neither dwt_starttx() nor
 * dwt_forcetrxoff() clears them, so once a single transmission misses its
 * instant the latch stays set and EVERY later delayed TX returns DWT_ERROR --
 * the node stops transmitting for good while tx_late climbs once per frame.
 * TXPUTE lives in the fifth status byte, hence the separate 8-bit write. */
static void tx_status_clear(void)
{
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_HPDWARN);
    dwt_write8bitoffsetreg(SYS_STATUS_ID, 4,
                           (uint8_t)(SYS_STATUS_TXPUTE >> 32));
}

static int tx_start(const uint8_t *buf, uint16_t len, int mode)
{
    dwt_writetxdata(len, (uint8_t *)buf, 0);
    dwt_writetxfctrl(len, 0, 1);            /* ranging bit set */

    if (dwt_starttx((uint8_t)mode) != DWT_SUCCESS)
    {
        tx_status_clear();
        return RADIO_TX_LATE;
    }
    return RADIO_TX_OK;
}

int radio_tx_at(const uint8_t *buf, uint16_t len, uint32_t tx_hi32)
{
    dwt_setdelayedtrxtime(tx_hi32);
    return tx_start(buf, len, DWT_START_TX_DELAYED);
}


/* Upper bound on how long a single transmit may legitimately take: the longest
 * legal frame in this protocol (a Final at N_R = 12) sent at 110 kbps behind a
 * 4096-symbol preamble is a little over 10 ms. 20 ms leaves margin without
 * letting a wedged transmit stall the frame loop. */
#define TX_WAIT_MAX_US      20000u

void radio_tx_wait(void)
{
    /* The bound is expressed in device time rather than in loop iterations.
     * The previous 40000-iteration guard measured SPI transactions, so its
     * wall-clock meaning changed with the SPI clock -- ~240 ms at 8 MHz and
     * nearly a second at the 2 MHz rate used during init, two orders of
     * magnitude longer than the frame being scheduled. Reading SYS_TIME costs
     * an SPI transaction too, so it is sampled every 64th poll: that caps the
     * added traffic at ~1.5% while keeping the deadline granularity well
     * inside the bound. */
    uint32_t start = radio_systime_hi32();
    uint32_t n     = 0;

    while ((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) == 0)
    {
        if ((++n & 0x3Fu) == 0u
            && radio_hi32_diff_us(radio_systime_hi32(), start)
               > (int32_t)TX_WAIT_MAX_US)
        {
            dwt_forcetrxoff();
            tx_status_clear();
            return;
        }
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

/* ------------------------------------------------------------------------- */
/* DW1000 User Manual 4.7.1 / 4.7.2.                                          */

void radio_quality(radio_quality_t *q)
{
    dwt_rxdiag_t diag;
    double       n, a;

    dwt_readdiagnostics(&diag);

    q->rxpacc    = diag.rxPreamCount;
    q->std_noise = diag.stdNoise;
    q->rx_dbm10  = 0;
    q->fp_dbm10  = 0;

    a = (m_prf == DWT_PRF_16M) ? 113.77 : 121.74;

    /* RXPACC counts preamble and SFD symbols; the SFD contribution has to come
     * back out before the power formulas are applied. */
    {
        /* DW1000 user manual RXPACC adjustment: keyed on the data rate AND the
         * SFD type, not the rate alone. Standard SFD is 5 at 850k/6.8M and 64
         * at 110k; the Decawave non-standard SFD is 8 at 850k and 10 at 6.8M.
         * sfd_len() above already models the same dependency. */
        uint32_t adj;

        if (m_dw.dataRate == DWT_BR_110K)      { adj = 64u; }
        else if (m_dw.nsSFD == 0u)             { adj = 5u;  }
        else if (m_dw.dataRate == DWT_BR_850K) { adj = 8u;  }
        else                                   { adj = 10u; }

        if (diag.rxPreamCount <= adj) { return; }
        n = (double)(diag.rxPreamCount - adj);
    }

    if (diag.maxGrowthCIR > 0u)
    {
        double v = 10.0 * log10(((double)diag.maxGrowthCIR * 131072.0) / (n * n)) - a;
        q->rx_dbm10 = (int16_t)lround(v * 10.0);
    }

    {
        double f1 = (double)diag.firstPathAmp1;
        double f2 = (double)diag.firstPathAmp2;
        double f3 = (double)diag.firstPathAmp3;

        if ((f1 + f2 + f3) > 0.0)
        {
            double v = 10.0 * log10(((f1 * f1) + (f2 * f2) + (f3 * f3)) / (n * n)) - a;
            q->fp_dbm10 = (int16_t)lround(v * 10.0);
        }
    }
}
