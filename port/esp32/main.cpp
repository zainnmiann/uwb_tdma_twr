/**
 * @file main.cpp
 * @brief ESP32 UWB Pro with Display — TDMA DS-TWR entry point.
 *
 * Protocol port of dwm1001_tdma_twr (that tree is never modified).
 * Radio: Makerfabs DW1000 library (vendored unmodified under lib/DW1000).
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "cfg.h"
#include "app.h"
#include "radio.h"
#include "timing.h"
#include "serial.h"
#include "output.h"
#include "uart_io.h"
#include "proto.h"
#include "port_sys.h"

#include "board_pins.h"
#include "display.h"

void setup()
{
    const cfg_t *c;

    cfg_load();
    c = cfg();

    serial_init();

    if (!radio_init(c))
    {
        out_line("ERR,6,radio,DW1000 init failed");
        uart_io_flush();
        for (;;) { port_led_toggle(PORT_LED_FAULT); delay(200); }
    }

    cfg_apply_identity(cfg(), radio_part_id());
    (void)radio_reconfigure(cfg());
    display_init();

    serial_banner();

    {
        const char *k = "", *r = "";
        int v = cfg_validate(c, &k, &r);
        if (v != 0)
        {
            out_linef("ERR,%d,%s,%s", v, k, r);
            out_line("I,hint=run $AUTOTIME then $SAVE after provisioning");
        }
        else
        {
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
}

void loop()
{
    /* role loops never return; keep Arduino runtime happy if they do */
    delay(1000);
}
