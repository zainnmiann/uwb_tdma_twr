/**
 * @file    node_uart.c
 * @brief   uart_io.h in the child: the node's real output, to a file.
 *
 * output.c and serial.c are linked unmodified, so the $RNG lines the harness
 * asserts on are produced by the firmware's own formatter. Each child writes
 * to its own file (path from HARNESS_OUT) and the parent parses them after the
 * run -- files rather than pipes, so the parent never has to interleave
 * draining output with the time barrier.
 */
#include "uart_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

static FILE *s_out;

void node_uart_open(const char *path)
{
    s_out = fopen(path, "w");
    if (s_out == NULL) { s_out = stderr; }
}

void uart_io_init(uint32_t baud) { (void)baud; }

bool uart_io_baud_supported(uint32_t baud)
{
    /* Same set the target can actually produce. */
    switch (baud)
    {
        case 1200: case 2400: case 4800: case 9600: case 14400:
        case 19200: case 28800: case 38400: case 57600: case 76800:
        case 115200: case 230400: case 250000: case 460800:
        case 921600: case 1000000:
            return true;
        default:
            return false;
    }
}

/* Bytes written since the parent last charged for them. The harness bills
 * these as serialisation time, which is what makes the frame budget in the
 * simulation as tight as it is on the target. */
static uint32_t s_pending_bytes;

uint32_t node_uart_take_bytes(void)
{
    uint32_t n = s_pending_bytes;

    s_pending_bytes = 0;
    return n;
}

void uart_io_write(const char *data, uint32_t len)
{
    s_pending_bytes += len;
    if (s_out) { (void)fwrite(data, 1, len, s_out); }
}

void uart_io_puts(const char *s)
{
    s_pending_bytes += (uint32_t)strlen(s);
    if (s_out) { (void)fputs(s, s_out); }
}

void uart_io_printf(const char *fmt, ...)
{
    va_list ap;
    int     n;

    /* vsnprintf into nothing first: the byte count has to be charged even
     * when the run is not capturing output, or timing would depend on
     * whether anyone was listening. */
    va_start(ap, fmt);
    n = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    if (n > 0) { s_pending_bytes += (uint32_t)n; }

    if (!s_out) { return; }
    va_start(ap, fmt);
    (void)vfprintf(s_out, fmt, ap);
    va_end(ap);
}

void     uart_io_flush(void)      { if (s_out) { (void)fflush(s_out); } }
uint32_t uart_io_dropped(void)    { return 0u; }
uint32_t uart_io_rx_dropped(void) { return 0u; }

/* No console input in the harness, so serial_poll() finds nothing -- which is
 * the path it takes on hardware between commands anyway. */
char *uart_io_poll_line(void) { return NULL; }
