/**
 * @file    serial.h
 * @brief   Section 9 -- the ASCII line protocol.
 *
 * "$" + payload + "*" + XOR checksum over the payload, CRLF terminated.
 * The command grammar is identical on every transport.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void serial_init(void);
void serial_poll(void);
void serial_banner(void);

/* Section 9.2 error codes. */
#define SERR_UNKNOWN_KEY    1
#define SERR_OUT_OF_RANGE   2
#define SERR_EQ14           3
#define SERR_PREAMBLE       4
#define SERR_READONLY       5
#define SERR_NVM            6

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H_ */
