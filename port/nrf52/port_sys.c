/**
 * @file    port_sys.c
 * @brief   nRF52832 backend for port_sys.h: reset (NVIC), activity LED
 *          (Nordic BSP), no board UI (no display on the DWM1001-DEV).
 */
#include "port_sys.h"

#include "nrf.h"
#include "boards.h"

void port_reset(void)
{
    NVIC_SystemReset();
    for (;;) { /* unreachable */ }
}

void port_led_set(port_led_t led, bool on)
{
    uint32_t mask = (led == PORT_LED_FAULT) ? BSP_LED_2_MASK : BSP_LED_0_MASK;
    if (on) { LEDS_ON(mask); } else { LEDS_OFF(mask); }
}

void port_led_toggle(port_led_t led)
{
    uint32_t mask = (led == PORT_LED_FAULT) ? BSP_LED_2_MASK : BSP_LED_0_MASK;
    LEDS_INVERT(mask);
}

void port_ui_range(uint8_t slot, uint16_t anchor_id, int32_t d_mm,
                   int16_t rxp_dbm10, bool valid)
{
    (void)slot; (void)anchor_id; (void)d_mm; (void)rxp_dbm10; (void)valid;
    /* No local display on the DWM1001-DEV. */
}
