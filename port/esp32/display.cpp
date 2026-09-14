/**
 * @file display.cpp
 * @brief Role-aware OLED display implementation for Makerfabs ESP32 UWB Pro.
 *
 * For Anchors (ROLE_RESP):
 *   Displays just the anchor label (e.g. "Anchor 1") large and centered.
 *
 * For Tags (ROLE_INIT):
 *   Displays all 8 anchors simultaneously across the 8 rows (8 pixels per row).
 *   Each row: Anchor label, range measurement, and RSSI value.
 *
 * Runs on Core 0 via FreeRTOS so microsecond-level TDMA on Core 1 is never disturbed.
 */

#include "display.h"
#include "board_pins.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>
#include <string.h>

#include "cfg.h"
#include "output.h"

#define NUM_SLOTS         8
#define SLOT_STALE_MS     2500

typedef struct {
    uint16_t anchor_id;
    int32_t  d_mm;
    int16_t  rxp_dbm10;
    uint32_t last_update_ms;
    bool     has_valid;
} slot_data_t;

static Adafruit_SSD1306 s_display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static portMUX_TYPE     s_mux = portMUX_INITIALIZER_UNLOCKED;

static slot_data_t      s_slots[NUM_SLOTS];

static void format_anchor_label(char *buf, size_t sz, uint8_t slot, uint16_t id)
{
    /* Label by slot POSITION, not node_id. node_id is now derived from the
     * ESP32's factory MAC (see radio_esp32.cpp) to guarantee uniqueness
     * across boards -- it is an arbitrary 16-bit value with no pattern, so
     * matching against a fixed list of specific IDs (the previous
     * implementation) matches nothing on any board provisioned after that
     * change and silently falls back to a raw hex ID. Slot index is always
     * well-defined (it's how the anchor is provisioned: $SET,slot_index,N)
     * and is exactly what the anchor's own splash screen shows, so the two
     * agree with no extra configuration needed. */
    (void)id;
    snprintf(buf, sz, "A%u", (unsigned)(slot + 1));
}

void display_set_range(uint8_t slot, uint16_t anchor_id, int32_t d_mm, int16_t rxp_dbm10, bool valid)
{
    if (slot >= NUM_SLOTS)
    {
        return;
    }

    portENTER_CRITICAL(&s_mux);
    if (anchor_id != 0)
    {
        s_slots[slot].anchor_id = anchor_id;
    }
    if (valid)
    {
        s_slots[slot].d_mm = d_mm;
        s_slots[slot].rxp_dbm10 = rxp_dbm10;
        s_slots[slot].last_update_ms = millis();
        s_slots[slot].has_valid = true;
    }
    portEXIT_CRITICAL(&s_mux);
}

static void display_task(void *pvParameters)
{
    (void)pvParameters;

    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    Wire.setClock(400000);

    bool begun = false;
    for (int retry = 0; retry < 5; retry++)
    {
        if (s_display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR))
        {
            begun = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!begun)
    {
        vTaskDelete(NULL);
        return;
    }

    {
        /* Boot-time identity splash: whichever role this board is, show
         * which one it is before anything else. For an anchor this label is
         * simply left on screen permanently (see the ROLE_RESP branch
         * below); for a tag there is otherwise no persistent place to show
         * it once the ranging grid takes over every row, so it gets a fixed
         * few seconds up front, every time the board (re)boots. */
        const cfg_t *c = cfg();
        char label[20];
        if (c->role == ROLE_RESP)
        {
            snprintf(label, sizeof(label), "Anchor %u", (unsigned)(c->slot_index + 1));
        }
        else if (c->role == ROLE_INIT)
        {
            snprintf(label, sizeof(label), "Tag %u", (unsigned)(c->frame_index + 1));
        }
        else
        {
            snprintf(label, sizeof(label), "Listener");
        }

        s_display.clearDisplay();
        s_display.setTextSize(2);
        s_display.setTextColor(SSD1306_WHITE);
        int len = strlen(label);
        int w = len * 12;
        int x = (OLED_WIDTH - w) / 2;
        if (x < 0) x = 0;
        int y = (OLED_HEIGHT - 16) / 2;
        s_display.setCursor(x, y);
        s_display.print(label);
        s_display.display();

        if (c->role != ROLE_RESP)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    for (;;)
    {
        const cfg_t *c = cfg();

        if (c->role == ROLE_RESP)
        {
            // --- ANCHOR DISPLAY: Just the anchor label, by slot position ---
            char label[20];
            snprintf(label, sizeof(label), "Anchor %u", (unsigned)(c->slot_index + 1));

            s_display.clearDisplay();
            s_display.setTextSize(2);
            s_display.setTextColor(SSD1306_WHITE);

            int len = strlen(label);
            int w = len * 12;
            int x = (OLED_WIDTH - w) / 2;
            if (x < 0) x = 0;
            int y = (OLED_HEIGHT - 16) / 2;

            s_display.setCursor(x, y);
            s_display.print(label);
            s_display.display();

            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // --- TAG DISPLAY (ROLE_INIT): 8 anchors simultaneously ---
        uint32_t now = millis();

        slot_data_t copy[NUM_SLOTS];
        portENTER_CRITICAL(&s_mux);
        for (int i = 0; i < NUM_SLOTS; i++)
        {
            copy[i] = s_slots[i];
        }
        portEXIT_CRITICAL(&s_mux);

        s_display.clearDisplay();
        s_display.setTextSize(1);
        s_display.setTextColor(SSD1306_WHITE);

        for (uint8_t s = 0; s < NUM_SLOTS; s++)
        {
            int16_t y = s * 8;
            bool active = copy[s].has_valid && ((now - copy[s].last_update_ms) <= SLOT_STALE_MS);

            // 1. Anchor label (Column 0: x = 0)
            char anc_str[8];
            format_anchor_label(anc_str, sizeof(anc_str), s, copy[s].anchor_id);
            s_display.setCursor(0, y);
            s_display.print(anc_str);

            // 2. Range measurement (Column 1: x = 36)
            char rng_str[12];
            if (active)
            {
                if (copy[s].d_mm < 0 && copy[s].d_mm > -50)
                {
                    snprintf(rng_str, sizeof(rng_str), "0.00m");
                }
                else
                {
                    snprintf(rng_str, sizeof(rng_str), "%.2fm", (double)copy[s].d_mm / 1000.0);
                }
            }
            else
            {
                snprintf(rng_str, sizeof(rng_str), "--");
            }
            s_display.setCursor(36, y);
            s_display.print(rng_str);

            // 3. RSSI (Column 2: x = 76)
            char rssi_str[12];
            if (active)
            {
                char dbm[12];
                out_fmt_dbm(dbm, sizeof(dbm), copy[s].rxp_dbm10);
                snprintf(rssi_str, sizeof(rssi_str), "%sdBm", dbm);
            }
            else
            {
                snprintf(rssi_str, sizeof(rssi_str), "--");
            }
            s_display.setCursor(76, y);
            s_display.print(rssi_str);
        }

        s_display.display();

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz refresh
    }
}

void display_init(void)
{
    portENTER_CRITICAL(&s_mux);
    for (uint8_t s = 0; s < NUM_SLOTS; s++)
    {
        s_slots[s].anchor_id      = 0;
        s_slots[s].d_mm           = 0;
        s_slots[s].rxp_dbm10      = 0;
        s_slots[s].last_update_ms = 0;
        s_slots[s].has_valid      = false;

        const cfg_anchor_t *a = cfg_anchor_by_slot(s);
        if (a && a->en)
        {
            s_slots[s].anchor_id = a->id;
        }
    }
    portEXIT_CRITICAL(&s_mux);

    xTaskCreatePinnedToCore(
        display_task,
        "display_task",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}
