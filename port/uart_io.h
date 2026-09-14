/**
 * @file    uart_io.h
 * @brief   Non-blocking UART console. Port contract: implemented separately
 *          by port/nrf52/uart_io.c (interrupt-driven ring, J-Link VCOM) and
 *          port/esp32/uart_io.cpp (Arduino HardwareSerial).
 *
 * TX is a ring buffer drained opportunistically, so range reports never
 * stall the TDMA state machine. If the ring overflows, the bytes that did
 * not fit are dropped and a counter is bumped rather than blocking -- the
 * queued backlog is preserved, the newest output is lost.
 */

#ifndef UART_IO_H_
#define UART_IO_H_

#include <stdint.h>
#include <stdbool.h>

/** TX ring size; must be a power of two on ports that mask instead of modulo. */
#define UART_TX_RING_SZ     4096u
/** Longest accepted command line. $ANCHOR carries eight fields. */
#define UART_RX_LINE_SZ     160u

#ifdef __cplusplus
extern "C" {
#endif

void     uart_io_init(uint32_t baud);

/** True if this port's UART can actually produce this rate. Rates outside
 *  the hardware's enumerated set would otherwise run at a different speed
 *  than the configuration reports. */
bool     uart_io_baud_supported(uint32_t baud);
void     uart_io_write(const char *data, uint32_t len);
void     uart_io_puts(const char *s);

/** printf-style, formatted into a stack buffer then queued. No float support. */
void     uart_io_printf(const char *fmt, ...);

/** Block until the TX ring has drained. Only for shutdown paths. */
void     uart_io_flush(void);

/** Number of bytes dropped because the TX ring was full. */
uint32_t uart_io_dropped(void);

/** Poll for a completed input line. Returns NULL until '\n' or '\r' arrives. */
char    *uart_io_poll_line(void);

/** Number of received bytes lost because the RX FIFO was full. */
uint32_t uart_io_rx_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_IO_H_ */
