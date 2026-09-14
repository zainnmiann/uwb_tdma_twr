/**
 * @file display.h
 * @brief OLED status display for Makerfabs ESP32 UWB Pro.
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize OLED display and launch background refresh task on Core 0. */
void display_init(void);

/** Update range measurement for a slot (thread-safe, called from Core 1). */
void display_set_range(uint8_t slot, uint16_t anchor_id, int32_t d_mm, int16_t rxp_dbm10, bool valid);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H_ */
