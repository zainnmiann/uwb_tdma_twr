/* Host stub. sim/ (unlike sim/loop/) never links serial.c or the role
 * loops, so none of these are ever actually called; declared only so
 * cfg.c's callers link cleanly if that ever changes. port_reset() aborts
 * loudly rather than silently pretending a reset happened, matching the
 * original sim/stub/nrf.h's NVIC_SystemReset() stub. */
#include "port_sys.h"
#include <stdlib.h>

void port_reset(void) { abort(); }
void port_led_set(port_led_t led, bool on) { (void)led; (void)on; }
void port_led_toggle(port_led_t led) { (void)led; }
void port_ui_range(uint8_t slot, uint16_t anchor_id, int32_t d_mm,
                   int16_t rxp_dbm10, bool valid)
{
    (void)slot; (void)anchor_id; (void)d_mm; (void)rxp_dbm10; (void)valid;
}
