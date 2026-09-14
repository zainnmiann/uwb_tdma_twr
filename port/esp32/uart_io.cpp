#include "uart_io.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char     s_tx[UART_TX_RING_SZ];
static volatile uint32_t s_tx_head;
static volatile uint32_t s_tx_tail;
static volatile uint32_t s_dropped;

static char     s_line[UART_RX_LINE_SZ];
static uint16_t s_line_len;
static char     s_ready[UART_RX_LINE_SZ];
static bool     s_have_line;
static uint32_t s_rx_dropped;

static void tx_kick(void)
{
    while (s_tx_tail != s_tx_head && Serial.availableForWrite() > 0)
    {
        char c = s_tx[s_tx_tail];
        s_tx_tail = (s_tx_tail + 1u) & (UART_TX_RING_SZ - 1u);
        Serial.write((uint8_t)c);
    }
}

void uart_io_init(uint32_t baud)
{
    Serial.begin(baud);
    s_tx_head = s_tx_tail = 0;
    s_dropped = 0;
    s_line_len = 0;
    s_have_line = false;
    s_rx_dropped = 0;
}

bool uart_io_baud_supported(uint32_t baud)
{
    return baud >= 9600u && baud <= 2000000u;
}

void uart_io_write(const char *data, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++)
    {
        uint32_t next = (s_tx_head + 1u) & (UART_TX_RING_SZ - 1u);
        if (next == s_tx_tail)
        {
            s_dropped++;
            break;
        }
        s_tx[s_tx_head] = data[i];
        s_tx_head = next;
    }
    tx_kick();
}

void uart_io_puts(const char *s)
{
    if (s) { uart_io_write(s, (uint32_t)strlen(s)); }
}

void uart_io_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    int n;
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0)
    {
        if (n >= (int)sizeof(buf)) { n = (int)sizeof(buf) - 1; }
        uart_io_write(buf, (uint32_t)n);
    }
}

void uart_io_flush(void)
{
    tx_kick();
    Serial.flush();
    while (s_tx_tail != s_tx_head)
    {
        tx_kick();
        delay(1);
    }
}

uint32_t uart_io_dropped(void) { return s_dropped; }
uint32_t uart_io_rx_dropped(void) { return s_rx_dropped; }

char *uart_io_poll_line(void)
{
    tx_kick();

    while (Serial.available() > 0)
    {
        int c = Serial.read();
        if (c < 0) { break; }

        if (c == '\n' || c == '\r')
        {
            if (s_line_len == 0) { continue; }
            s_line[s_line_len] = '\0';
            memcpy(s_ready, s_line, s_line_len + 1u);
            s_line_len = 0;
            s_have_line = true;
            break;
        }

        if (s_line_len + 1u >= UART_RX_LINE_SZ)
        {
            s_rx_dropped++;
            s_line_len = 0;
            continue;
        }
        s_line[s_line_len++] = (char)c;
    }

    if (!s_have_line) { return NULL; }
    s_have_line = false;
    return s_ready;
}
