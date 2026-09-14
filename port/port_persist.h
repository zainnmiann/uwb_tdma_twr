/**
 * @file    port_persist.h
 * @brief   Non-volatile storage for the configuration blob. Port contract:
 *          port/nrf52/port_persist.c (last nRF52 flash page, NVMC),
 *          port/esp32/port_persist.cpp (NVS via Preferences),
 *          sim/port/port_persist.c (no-op, always returns false so cfg_load()
 *          falls through to cfg_defaults() -- matching the original
 *          simulator's behaviour).
 */
#ifndef PORT_PERSIST_H_
#define PORT_PERSIST_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

bool port_persist_load(cfg_t *out);
bool port_persist_save(const cfg_t *in);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PERSIST_H_ */
