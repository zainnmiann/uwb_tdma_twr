/**
 * @file    main.c
 * @brief   UWB DS-TWR, initiator-driven fixed-slot architecture (DW1000).
 *
 * Implements revision 0.1 of the protocol specification:
 *
 *   section 3   timing hierarchy, superframe of N_I frames of N_R slot pairs
 *   section 4   asymmetric DS-TWR estimator, SS-TWR fallback
 *   section 5   message formats
 *   section 6   initiator frame synchronization from overheard Polls
 *   section 7   air-time model, delay tables, ranging rate
 *   section 8   parameter set, persistence, consistency rules
 *   section 9   serial interface
 *   section 10  node state machines
 *   section 11  loss handling
 *   section 12  DW1000 library mapping
 *
 * No node acts as network manager and no responder initiates an exchange.
 * The PHY is the stock Decawave DW1000 driver, vendored unmodified.
 */

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"

#include "cfg.h"
#include "app.h"
#include "radio.h"
#include "timing.h"
#include "serial.h"
#include "output.h"
#include "uart_io.h"

#include "port_platform.h"
#include "port_sys.h"

static void fatal_blink(void)
{
    for (;;)
    {
        port_led_toggle(PORT_LED_FAULT);
        nrf_delay_ms(120);
    }
}

int main(void)
{
    const cfg_t *c;

    /* Run the UART and the DW1000 SPI off the crystal rather than the internal
     * RC, which is only +-1.5% -- marginal at the section 8.1 default console
     * rate of 921600, and needed by the DW1000 SPI regardless of baud. */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }

    LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);
    LEDS_OFF(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK);

    /* BOOT -> LOAD_CFG */
    cfg_load();
    c = cfg();

    serial_init();

    /* The DW1000 IRQ line stays a plain input: SYS_STATUS is polled, so the
     * poll-to-response turnaround carries no interrupt-latency term. */
    nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL);

    if (!radio_init(c))
    {
        out_line("ERR,6,radio,DW1000 init failed");
        uart_io_flush();
        fatal_blink();
    }

    cfg_apply_identity(cfg(), radio_part_id());

    /* node_id feeds the DW1000 address filter, so push it to the radio. */
    (void)radio_reconfigure(cfg());

    serial_banner();

    {
        const char *k = "", *r = "";
        int v = cfg_validate(c, &k, &r);
        if (v != 0)
        {
            out_linef("ERR,%d,%s,%s", v, k, r);
        }
        else
        {
            /* if(autostart) LISTEN/RX_ON else IDLE */
            app_ranging_set(c->autostart != 0u);
        }
    }

    out_linef("I,Tf=%luus,TF=%luus,rate_mHz=%lu,N_I=%u,N_R=%u,D=%u,report=%u",
              (unsigned long)c->frame_dur_us,
              (unsigned long)timing_superframe_us(c),
              (unsigned long)timing_rate_mhz(c, c->rate_div),
              c->init_count, c->resp_count, c->rate_div, c->report_enable);

    switch (c->role)
    {
        case ROLE_INIT: app_run_initiator(); break;
        case ROLE_RESP: app_run_responder(); break;
        default:        app_run_listener();  break;
    }

    fatal_blink();
    return 0;
}
