/**
 * @file    port_sys.cpp
 * @brief   ESP32 backend for port_sys.h: reset (ESP.restart), no dedicated
 *          activity LED wired on the Makerfabs board, board UI forwards to
 *          the OLED driver in display.cpp.
 */
#include "port_sys.h"
#include "display.h"

#include <Arduino.h>

void port_reset(void)
{
    ESP.restart();
    for (;;) { delay(200); }   /* unreachable; ESP.restart() does not return */
}

void port_led_set(port_led_t led, bool on)
{
    (void)led; (void)on;   /* no LED wired for this purpose on this board */
}

void port_led_toggle(port_led_t led)
{
    (void)led;
}

void port_ui_range(uint8_t slot, uint16_t anchor_id, int32_t d_mm,
                   int16_t rxp_dbm10, bool valid)
{
    display_set_range(slot, anchor_id, d_mm, rxp_dbm10, valid);
}
