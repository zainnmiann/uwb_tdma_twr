/**
 * @file    port_sys.h
 * @brief   Reset, activity LED and optional board UI -- the three
 *          board-level concerns too small to warrant their own headers.
 *          Port contract: implemented by port/nrf52/port_sys.c,
 *          port/esp32/port_sys.cpp and sim/port/port_sys.c.
 */
#ifndef PORT_SYS_H_
#define PORT_SYS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** $RESET. Does not return.
 *  nRF52: NVIC_SystemReset(). ESP32: ESP.restart(). sim: abort() -- loudly,
 *  so a test that reaches $RESET fails visibly instead of silently
 *  pretending a reboot happened. */
#if defined(__GNUC__) || defined(__clang__)
void port_reset(void) __attribute__((noreturn));
#else
void port_reset(void);
#endif

/** Activity/fault LEDs. Replaces LEDS_INVERT(BSP_LED_0_MASK) from the
 *  Nordic BSP -- a no-op on boards with no LED wired for this purpose. */
typedef enum
{
    PORT_LED_ACTIVITY = 0,   /**< toggled once per served/owned frame */
    PORT_LED_FAULT    = 1    /**< fatal_blink() */
} port_led_t;

void port_led_set(port_led_t led, bool on);
void port_led_toggle(port_led_t led);

/** One range result for the board's optional local display (e.g. the ESP32
 *  Pro's OLED). Must be safe to call from the TDMA loop -- nRF52 and sim are
 *  no-ops; the ESP32 port copies the fields under a spinlock and a Core-0
 *  FreeRTOS task renders them. Signature matches the ESP32 tree's
 *  display_set_range() so that port is a one-line forwarder. */
void port_ui_range(uint8_t slot, uint16_t anchor_id, int32_t d_mm,
                   int16_t rxp_dbm10, bool valid);

#ifdef __cplusplus
}
#endif

#endif /* PORT_SYS_H_ */
