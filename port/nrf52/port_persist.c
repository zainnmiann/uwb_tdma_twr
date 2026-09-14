/**
 * @file    port_persist.c
 * @brief   nRF52832 backend for port_persist.h -- last flash page, NVMC.
 */
#include "port_persist.h"

#include "nrf.h"
#include "nrf_nvmc.h"

#include <string.h>

#define CFG_FLASH_ADDR  0x0007F000u     /* last page of the 512 kB nRF52832 */

bool port_persist_load(cfg_t *out)
{
    memcpy(out, (const void *)CFG_FLASH_ADDR, sizeof(*out));
    return true;   /* caller (cfg_load/cfg_reload) validates magic/version/crc */
}

bool port_persist_save(const cfg_t *in)
{
    nrf_nvmc_page_erase(CFG_FLASH_ADDR);
    nrf_nvmc_write_words(CFG_FLASH_ADDR, (const uint32_t *)in,
                         (sizeof(*in) + 3u) / 4u);

    return memcmp((const void *)CFG_FLASH_ADDR, in, sizeof(*in)) == 0;
}
