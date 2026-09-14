#include "cfg.h"
#include "proto.h"
#include "timing.h"
#include "uart_io.h"
#include "port_persist.h"
#include "board_defaults.h"

#include <string.h>
#include <stdio.h>

#define CFG_MAGIC       0x32525754u     /* "TWR2" */
#define CFG_VERSION     2u

static cfg_t m_cfg;

/* ------------------------------------------------------------------------- */

static uint32_t crc32(const uint8_t *d, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFu, i, b;
    for (i = 0; i < n; i++)
    {
        crc ^= d[i];
        for (b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

static uint16_t crc16(uint16_t crc, const void *data, uint32_t n)
{
    const uint8_t *d = (const uint8_t *)data;
    uint32_t i, b;
    for (i = 0; i < n; i++)
    {
        crc ^= (uint16_t)d[i] << 8;
        for (b = 0; b < 8; b++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ------------------------------------------------------------------------- */

void cfg_defaults(cfg_t *c)
{
    uint32_t i;
    const board_defaults_t *b = board_defaults();

    memset(c, 0, sizeof(*c));
    c->magic   = CFG_MAGIC;
    c->version = CFG_VERSION;
    c->size    = (uint16_t)sizeof(cfg_t);

    /* Role and identity */
    c->role            = ROLE_RESP;         /* spec default */
    /* Section 8.1 gives node_id no default. 0x0000 is outside its stated range
     * (0x0001-0xFFFE) so it serves as the "unprovisioned" marker; main.c
     * derives a unique address from the DW1000 part ID at boot if it is still
     * unset, which is also what makes ble_name's "UWB-id" default meaningful.
     * node_id is not part of cfg_hash(), so deriving it cannot desynchronise
     * a network. */
    c->node_id         = 0x0000u;
    c->pan_id          = 0xDECAu;
    c->frame_index     = 0;
    c->slot_index      = 0;

    /* Network size, timing and per-board processing budget all come from the
     * port's board_defaults() -- see board_defaults.h for which fields are
     * network-hashed (must agree on every node) vs per-board (may legitimately
     * differ, e.g. proc_us). This is the ONE place cfg_defaults() varies by
     * platform; everything else in this file is shared, unmodified core. */
    c->init_count      = b->init_count;
    c->resp_count      = b->resp_count;

    c->frame_dur_us    = b->frame_dur_us;
    c->resp_base_us    = b->resp_base_us;
    c->resp_pitch_us   = b->resp_pitch_us;
    c->report_base_us  = b->report_base_us;
    c->report_pitch_us = b->report_pitch_us;
    c->guard_us        = b->guard_us;
    c->rate_div        = b->rate_div;
    c->rx_timeout_us   = b->rx_timeout_us;
    c->sync_mmax       = b->sync_mmax;
    c->proc_us         = b->proc_us;
    c->isr_us          = b->isr_us;
    c->margin_us       = b->margin_us;
    c->proc_poll_us    = b->proc_poll_us;
    c->proc_final_us   = b->proc_final_us;
    c->sync_comp_us    = 0;

    /* Radio */
    c->channel         = 5;
    c->prf             = PRF_64;
    c->preamble_code   = 9;
    c->preamble_len    = 128;
    c->pac_size        = 0;                 /* auto */
    c->datarate        = RATE_6800;
    /* Section 8.1's Default column for sfd_mode is the conditional "DW at
     * 110 kbps" -- the Decawave non-standard SFD is the default only at the
     * 110 kbps rate, standard elsewhere. */
    c->sfd_mode        = (c->datarate == RATE_110) ? SFD_DW : SFD_STD;
    c->txpower_mode    = TXP_AUTO;
    c->txpower_coarse  = 0;
    c->txpower_fine    = 0;
    c->txpower_reg     = 0;
    c->smart_power     = 0;
    c->ant_delay_tx    = CFG_ANT_DELAY_UNCAL;
    c->ant_delay_rx    = CFG_ANT_DELAY_UNCAL;

    /* Operation */
    c->autostart       = 1;
    c->report_enable   = 1;
    c->out_format      = OUT_CSV;
    c->out_mask        = OUT_MASK_RANGE | OUT_MASK_DIAG;    /* 0x03 */

    /* Interfaces */
    c->iface_cfg       = IFACE_USB;
    c->iface_data      = IFACE_USB;
    c->uart_baud       = b->uart_baud;
    c->ble_name[0]     = '\0';              /* filled from node_id at boot */

    /* Lists: identity-mapped and disabled until provisioned. */
    for (i = 0; i < CFG_MAX_RESP; i++)
    {
        c->anchor[i].id   = (uint16_t)(0x0100u + i);
        c->anchor[i].slot = (uint8_t)i;
        c->anchor[i].en   = 0;
    }
    for (i = 0; i < CFG_MAX_INIT; i++)
    {
        c->tag[i].id    = (uint16_t)(0x0001u + i);
        c->tag[i].frame = (uint8_t)i;
        c->tag[i].div   = 1;
        c->tag[i].en    = 0;
    }

    c->crc = crc32((const uint8_t *)c, sizeof(*c) - sizeof(uint32_t));
}

static bool cfg_struct_ok(const cfg_t *c)
{
    if (c->magic != CFG_MAGIC || c->version != CFG_VERSION)  { return false; }
    if (c->size  != (uint16_t)sizeof(cfg_t))                 { return false; }
    return c->crc == crc32((const uint8_t *)c, sizeof(*c) - sizeof(uint32_t));
}

void cfg_load(void)
{
    cfg_t tmp;
    if (port_persist_load(&tmp) && cfg_struct_ok(&tmp))
    {
        m_cfg = tmp;
    }
    else
    {
        cfg_defaults(&m_cfg);
    }
}

bool cfg_reload(void)
{
    cfg_t tmp;
    if (!port_persist_load(&tmp) || !cfg_struct_ok(&tmp))
    {
        return false;
    }
    m_cfg = tmp;
    return true;
}

cfg_t *cfg(void)
{
    return &m_cfg;
}

bool cfg_save(void)
{
    m_cfg.magic   = CFG_MAGIC;
    m_cfg.version = CFG_VERSION;
    m_cfg.size    = (uint16_t)sizeof(cfg_t);
    m_cfg.crc     = crc32((const uint8_t *)&m_cfg, sizeof(m_cfg) - sizeof(uint32_t));

    return port_persist_save(&m_cfg);
}

/* Section 8.1 gives node_id no default and ble_name the default "UWB-id".
 * Derive both from this part's unique DW1000 identifier when the stored
 * configuration has never been provisioned. Kept here rather than inline in
 * main() so $DEFAULT can reach it too -- otherwise $DEFAULT reports OK and
 * leaves a configuration that every later $SAVE and $RANGE,START rejects with
 * ERR,2,node_id, naming a key the operator never touched.
 * node_id is not part of cfg_hash(), so deriving it cannot desynchronise a
 * network. */
void cfg_apply_identity(cfg_t *c, uint32_t part_id)
{
    if (c->node_id == 0x0000u || c->node_id == 0xFFFFu)
    {
        uint16_t id = (uint16_t)((part_id ^ (part_id >> 16)) & 0xFFFFu);

        if (id == 0x0000u || id == 0xFFFFu) { id = 0x0001u; }
        c->node_id = id;
    }

    if (c->ble_name[0] == '\0')
    {
        (void)snprintf(c->ble_name, CFG_NAME_LEN, "UWB-%04X", c->node_id);
    }
}

uint16_t cfg_addr(void)
{
    return m_cfg.node_id;
}

/* ------------------------------------------------------------------------- */
/* Section 8.2 rule 3: legal preamble codes per channel and PRF.              */

bool cfg_preamble_code_legal(uint8_t chan, uint8_t prf, uint8_t code)
{
    static const uint8_t p16[8][2] = {
        {0,0}, {1,2}, {3,4}, {5,6}, {7,8}, {3,4}, {0,0}, {7,8}
    };
    uint8_t lo, hi;

    if (chan > 7u || chan == 0u || chan == 6u) { return false; }

    if (prf == PRF_16)
    {
        return (code == p16[chan][0]) || (code == p16[chan][1]);
    }

    /* 64 MHz PRF: channels 4 and 7 use the 17..20 set, the rest 9..12. */
    if (chan == 4u || chan == 7u) { lo = 17u; hi = 20u; }
    else                          { lo =  9u; hi = 12u; }

    return (code >= lo) && (code <= hi);
}

/* Section 8.2 rule 4: PAC implied by preamble length. */
uint8_t cfg_pac_for_preamble(uint16_t plen)
{
    if (plen <= 128u)  { return 8u;  }
    if (plen <= 512u)  { return 16u; }
    if (plen <= 1024u) { return 32u; }
    return 64u;
}

static bool preamble_len_legal(uint16_t p)
{
    return p == 64u || p == 128u || p == 256u || p == 512u
        || p == 1024u || p == 1536u || p == 2048u || p == 4096u;
}

/* ------------------------------------------------------------------------- */

/* Declared in timing.h; forward declared to keep the header dependency one-way. */

int cfg_validate(const cfg_t *c, const char **key, const char **reason)
{
    const char *k = "";
    const char *r = "";
    int         code = 0;

    if (c->role > ROLE_LSTN)                    { k = "role";        r = "unknown role";        code = 2; goto out; }
    if (c->node_id == 0x0000u || c->node_id == 0xFFFFu)
                                                { k = "node_id";     r = "reserved address";    code = 2; goto out; }
    if (c->init_count < 1u || c->init_count > CFG_MAX_INIT)
                                                { k = "init_count";  r = "range 1..32";         code = 2; goto out; }
    if (c->resp_count < 2u || c->resp_count > CFG_MAX_RESP)
                                                { k = "resp_count";  r = "range 2..12";         code = 2; goto out; }

    /* Rule 2 */
    if (c->role == ROLE_INIT && c->frame_index >= c->init_count)
                                                { k = "frame_index"; r = "must be < init_count"; code = 2; goto out; }
    if (c->role == ROLE_RESP && c->slot_index >= c->resp_count)
                                                { k = "slot_index";  r = "must be < resp_count"; code = 2; goto out; }

    /* Section 8.1 lists the channel set as {1,2,3,4,5,7}; checked before rule 3
     * so an illegal channel is reported against "channel" rather than blamed on
     * preamble_code. */
    if (c->channel == 0u || c->channel == 6u || c->channel > 7u)
                                                { k = "channel";     r = "must be 1,2,3,4,5 or 7"; code = 2; goto out; }

    /* Rule 3 */
    if (!cfg_preamble_code_legal(c->channel, c->prf, c->preamble_code))
                                                { k = "preamble_code"; r = "illegal for channel/prf"; code = 4; goto out; }

    if (!preamble_len_legal(c->preamble_len))   { k = "preamble_len"; r = "not an IEEE value";  code = 2; goto out; }

    /* Rule 4 */
    if (c->pac_size != 0u && c->pac_size != cfg_pac_for_preamble(c->preamble_len))
                                                { k = "pac_size";    r = "does not match preamble_len"; code = 2; goto out; }
    if (c->pac_size != 0u && c->pac_size != 8u && c->pac_size != 16u
        && c->pac_size != 32u && c->pac_size != 64u)
                                                { k = "pac_size";    r = "must be 8/16/32/64";  code = 2; goto out; }

    if (c->datarate > RATE_6800)                { k = "datarate";    r = "110/850/6800";        code = 2; goto out; }

    /* Rule 5 */
    if (c->txpower_mode == TXP_MANUAL && c->txpower_reg == 0u)
    {
        if (c->txpower_coarse > 6u)             { k = "txpower_coarse"; r = "range 0..6";       code = 2; goto out; }
        if (c->txpower_fine  > 31u)             { k = "txpower_fine";   r = "range 0..31";      code = 2; goto out; }
    }

    if (c->rate_div < 1u || c->rate_div > 64u)  { k = "rate_div";    r = "range 1..64";         code = 2; goto out; }
    if (c->sync_mmax < 1u)                      { k = "sync_mmax";   r = "range 1..255";        code = 2; goto out; }
    if (c->rx_timeout_us < 100u || c->rx_timeout_us > 65000u)
                                                { k = "rx_timeout_us"; r = "range 100..65000";  code = 2; goto out; }
    if (c->guard_us < 20u || c->guard_us > 5000u)
                                                { k = "guard_us";    r = "range 20..5000";      code = 2; goto out; }
    if (c->resp_base_us   < 100u || c->resp_base_us   > 20000u)
                                                { k = "resp_base_us";   r = "range 100..20000"; code = 2; goto out; }
    if (c->resp_pitch_us  < 100u || c->resp_pitch_us  > 20000u)
                                                { k = "resp_pitch_us";  r = "range 100..20000"; code = 2; goto out; }
    if (c->report_base_us < 100u || c->report_base_us > 20000u)
                                                { k = "report_base_us"; r = "range 100..20000"; code = 2; goto out; }
    if (c->report_pitch_us< 100u || c->report_pitch_us> 20000u)
                                                { k = "report_pitch_us";r = "range 100..20000"; code = 2; goto out; }
    if (c->frame_dur_us < 1000u || c->frame_dur_us > 200000u)
                                                { k = "frame_dur_us"; r = "range 1000..200000"; code = 2; goto out; }

    /* Equations (1) and (3) reference T_r0 and T_p0 to the RX RMARKER, so a
     * base delay shorter than the rest of that frame's air time schedules a
     * transmit that is already in the past by the time the handler runs. At
     * 850 kbps and below this made every Response (or Report) fail with
     * tx_late, on every frame, with nothing refusing the configuration. */
    {
        uint32_t poll_tail  = timing_tail_ns(c, timing_air_poll_ns(c))  / 1000u;
        uint32_t final_tail = timing_tail_ns(c, timing_air_final_ns(c)) / 1000u;

        if (c->resp_base_us < poll_tail)
        {
            k = "resp_base_us";
            r = "shorter than the Poll's air time at this operating point";
            code = 3; goto out;
        }
        if (c->report_enable && c->report_base_us < final_tail)
        {
            k = "report_base_us";
            r = "shorter than the Final's air time at this operating point";
            code = 3; goto out;
        }
    }

    /* Rule 1: frame_dur_us must satisfy (14). */
    if (c->frame_dur_us < timing_frame_dur_us(c))
    {
        k = "frame_dur_us";
        r = "shorter than eq (14) requires";
        code = 3;
        goto out;
    }

    /* Section 8.1's range is 9600..3000000, but only the rates the nRF52 UART
     * enumerates are actually reachable -- accepting others would run the
     * console at a different speed than the configuration reports. */
    if (!uart_io_baud_supported(c->uart_baud))
                                                { k = "uart_baud";   r = "not an nRF52 UART rate"; code = 2; goto out; }

    if (c->out_format > OUT_BIN)                { k = "out_format";  r = "CSV/JSON/BIN";        code = 2; goto out; }
    if (c->iface_cfg  > IFACE_WIFI)             { k = "iface_cfg";   r = "USB/UART/BLE/WIFI";   code = 2; goto out; }
    if (c->iface_data > IFACE_WIFI)             { k = "iface_data";  r = "USB/UART/BLE/WIFI";   code = 2; goto out; }

out:
    if (key)    { *key = k; }
    if (reason) { *reason = r; }
    return code;
}

/* ------------------------------------------------------------------------- */
/* Section 8.2 rule 6 / section 5: the network-wide agreement hash.           */

static uint16_t cfg_hash_compute(const cfg_t *c)
{
    uint16_t h = 0xFFFFu;
    uint8_t  pac = (c->pac_size != 0u) ? c->pac_size : cfg_pac_for_preamble(c->preamble_len);

    /* Radio */
    h = crc16(h, &c->pan_id,        sizeof(c->pan_id));
    h = crc16(h, &c->channel,       1);
    h = crc16(h, &c->prf,           1);
    h = crc16(h, &c->preamble_code, 1);
    h = crc16(h, &c->preamble_len,  sizeof(c->preamble_len));
    h = crc16(h, &pac,              1);
    h = crc16(h, &c->datarate,      1);
    h = crc16(h, &c->sfd_mode,      1);

    /* Network size */
    h = crc16(h, &c->init_count,    1);
    h = crc16(h, &c->resp_count,    1);

    /* Timing */
    h = crc16(h, &c->frame_dur_us,    sizeof(c->frame_dur_us));
    h = crc16(h, &c->resp_base_us,    sizeof(c->resp_base_us));
    h = crc16(h, &c->resp_pitch_us,   sizeof(c->resp_pitch_us));
    h = crc16(h, &c->report_base_us,  sizeof(c->report_base_us));
    h = crc16(h, &c->report_pitch_us, sizeof(c->report_pitch_us));
    h = crc16(h, &c->guard_us,        sizeof(c->guard_us));
    h = crc16(h, &c->report_enable,   1);

    return h;
}

/* ------------------------------------------------------------------------- */

const cfg_anchor_t *cfg_anchor_by_slot(uint8_t slot)
{
    uint32_t i;
    for (i = 0; i < CFG_MAX_RESP; i++)
    {
        if (m_cfg.anchor[i].en && m_cfg.anchor[i].slot == slot)
        {
            return &m_cfg.anchor[i];
        }
    }
    return NULL;
}

const cfg_anchor_t *cfg_anchor_by_id(uint16_t id)
{
    uint32_t i;
    for (i = 0; i < CFG_MAX_RESP; i++)
    {
        /* `en` is tested here too. Without it a listener applied the antenna
         * trim of a disabled entry while the initiator (via by_slot) did not,
         * so the two reported different distances for the same exchange -- and
         * a listener picked up trims from every stale default entry. */
        if (m_cfg.anchor[i].en && m_cfg.anchor[i].id == id)
        {
            return &m_cfg.anchor[i];
        }
    }
    return NULL;
}

const cfg_tag_t *cfg_tag_by_frame(uint8_t frame)
{
    uint32_t i;
    for (i = 0; i < CFG_MAX_INIT; i++)
    {
        if (m_cfg.tag[i].en && m_cfg.tag[i].frame == frame)
        {
            return &m_cfg.tag[i];
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------- */

const char *cfg_role_str(uint8_t v)
{
    return (v == ROLE_INIT) ? "INIT" : (v == ROLE_RESP) ? "RESP" : "LSTN";
}
const char *cfg_rate_str(uint8_t v)
{
    return (v == RATE_110) ? "110" : (v == RATE_850) ? "850" : "6800";
}
const char *cfg_sfd_str(uint8_t v)      { return (v == SFD_DW) ? "DW" : "STD"; }
const char *cfg_txpmode_str(uint8_t v)  { return (v == TXP_MANUAL) ? "MANUAL" : "AUTO"; }
const char *cfg_outfmt_str(uint8_t v)
{
    return (v == OUT_JSON) ? "JSON" : (v == OUT_BIN) ? "BIN" : "CSV";
}
const char *cfg_iface_str(uint8_t v)
{
    switch (v)
    {
        case IFACE_UART: return "UART";
        case IFACE_BLE:  return "BLE";
        case IFACE_WIFI: return "WIFI";
        default:         return "USB";
    }
}

/* cfg_hash() sits on the Poll-to-Response critical path (handle_poll) and runs
 * for every Poll sent and received, but the value only changes when the
 * configuration does. Memoised against a copy of exactly the fields that feed
 * it: self-validating, so unlike explicit invalidation it cannot go stale if a
 * future mutation site forgets to call it -- and a stale hash would silently
 * partition the network. */
uint16_t cfg_hash(const cfg_t *c)
{
    struct hash_key
    {
        uint16_t pan_id, preamble_len;
        uint8_t  channel, prf, preamble_code, pac_size;
        uint8_t  datarate, sfd_mode, init_count, resp_count, report_enable;
        uint32_t frame_dur_us, resp_base_us, resp_pitch_us;
        uint32_t report_base_us, report_pitch_us, guard_us;
    };

    static struct hash_key s_key;
    static uint16_t        s_hash;
    static bool            s_valid;

    struct hash_key key;

    memset(&key, 0, sizeof(key));
    key.pan_id          = c->pan_id;
    key.preamble_len    = c->preamble_len;
    key.channel         = c->channel;
    key.prf             = c->prf;
    key.preamble_code   = c->preamble_code;
    key.pac_size        = c->pac_size;
    key.datarate        = c->datarate;
    key.sfd_mode        = c->sfd_mode;
    key.init_count      = c->init_count;
    key.resp_count      = c->resp_count;
    key.report_enable   = c->report_enable;
    key.frame_dur_us    = c->frame_dur_us;
    key.resp_base_us    = c->resp_base_us;
    key.resp_pitch_us   = c->resp_pitch_us;
    key.report_base_us  = c->report_base_us;
    key.report_pitch_us = c->report_pitch_us;
    key.guard_us        = c->guard_us;

    if (s_valid && memcmp(&key, &s_key, sizeof(key)) == 0)
    {
        return s_hash;
    }

    s_key   = key;
    s_hash  = cfg_hash_compute(c);
    s_valid = true;
    return s_hash;
}
