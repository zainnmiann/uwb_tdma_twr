/**
 * @file    ipc.h
 * @brief   The request/reply contract between the harness and each node.
 *
 * One process per node, so every node gets its own copy of the firmware's
 * file-scope state (m_sync, m_buf, m_resp_mask, m_ex[], m_cfg, m_stats, the
 * output.c accumulators) with no firmware changes at all. That is the whole
 * reason for forking rather than threading.
 *
 * The parent is the sole authority on virtual time and on the air medium.
 * Every blocking radio call in the child becomes exactly one round trip, so
 * the parent always knows when every child is parked and can safely advance
 * time. Pipes rather than shared-memory semaphores: macOS does not implement
 * unnamed POSIX semaphores (sem_init with pshared fails with ENOSYS), and
 * pipes need no shared synchronisation primitives.
 */
#ifndef IPC_H_
#define IPC_H_

#include <stdint.h>
#include <stdbool.h>

#define IPC_MAX_FRAME       127u
#define IPC_MAX_NODES       64u

/* --- child -> parent ------------------------------------------------------ */
typedef enum
{
    REQ_RX = 1,         /* receive until `until_dtu`                        */
    REQ_TX,             /* arm a delayed transmit at `hi32`                 */
    REQ_TXWAIT,         /* block until the armed transmit has gone out      */
    REQ_IDLE,           /* radio_idle(): drop out of receive                */
    REQ_DONE            /* loop exited                                      */
} req_kind_t;

typedef struct
{
    uint32_t kind;
    uint32_t node;
    int64_t  until_dtu;     /* REQ_RX: give up at this local instant        */
    /* REQ_RX: the instant the receiver actually opens. A delayed arm enables
     * the DW1000 at a programmed time whatever the MCU is doing afterwards,
     * so the parent cannot infer this from when the request arrives -- that
     * is precisely the difference the end-of-frame peer watch depends on. */
    int64_t  from_dtu;
    /* Bytes handed to the UART since the last round trip. Serialising them
     * costs real time on the target and the frame budget never allowed for
     * it, so the parent charges it here rather than letting output look
     * free. */
    uint32_t uart_bytes;
    uint32_t hi32;          /* REQ_TX: the armed instant                   */
    uint16_t len;           /* REQ_TX: frame length, FCS included          */
    uint8_t  buf[IPC_MAX_FRAME];
} ipc_req_t;

/* --- parent -> child ------------------------------------------------------ */
typedef enum
{
    REP_TIMEOUT = 1,    /* radio_rx: nothing arrived                        */
    REP_FRAME,          /* radio_rx: a frame, fully received                */
    REP_TX_OK,          /* radio_tx_at: armed                               */
    REP_TX_LATE,        /* radio_tx_at: the instant had already passed      */
    REP_OK,             /* REQ_TXWAIT / REQ_IDLE acknowledged               */
    REP_STOP            /* the run is over: exit                            */
} rep_kind_t;

typedef struct
{
    uint32_t kind;
    int64_t  now_dtu;       /* this node's local time on release            */
    uint64_t rx_ts;         /* REP_FRAME: local RMARKER, 40-bit             */
    int32_t  carrier_int;   /* REP_FRAME: carrier integrator                */
    int16_t  rx_dbm10;      /* REP_FRAME: link quality                      */
    int16_t  fp_dbm10;
    uint16_t len;
    uint8_t  buf[IPC_MAX_FRAME];
} ipc_rep_t;

/* --- child side ----------------------------------------------------------- */
/** Wire this child up to its pipes and tell it which node it is. */
void ipc_child_init(int req_fd, int rep_fd, uint32_t node);
/** Send a request and block for the reply. Exits the process on REP_STOP. */
void ipc_round_trip(const ipc_req_t *rq, ipc_rep_t *rp);

/** Full read/write helpers: pipes may deliver a struct in pieces. */
bool ipc_write_all(int fd, const void *p, uint32_t n);
bool ipc_read_all(int fd, void *p, uint32_t n);

#endif /* IPC_H_ */
