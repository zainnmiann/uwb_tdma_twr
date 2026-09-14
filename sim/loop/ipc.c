#include "ipc.h"

#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

bool ipc_write_all(int fd, const void *p, uint32_t n)
{
    const uint8_t *b = p;
    uint32_t done = 0;

    while (done < n)
    {
        ssize_t w = write(fd, b + done, n - done);

        if (w < 0)
        {
            if (errno == EINTR) { continue; }
            return false;
        }
        done += (uint32_t)w;
    }
    return true;
}

bool ipc_read_all(int fd, void *p, uint32_t n)
{
    uint8_t *b = p;
    uint32_t done = 0;

    while (done < n)
    {
        ssize_t r = read(fd, b + done, n - done);

        if (r == 0) { return false; }               /* EOF: peer gone */
        if (r < 0)
        {
            if (errno == EINTR) { continue; }
            return false;
        }
        done += (uint32_t)r;
    }
    return true;
}

/* --- child side ----------------------------------------------------------- */
static int      s_req_fd = -1;
static int      s_rep_fd = -1;
static uint32_t s_node;

void ipc_child_init(int req_fd, int rep_fd, uint32_t node)
{
    s_req_fd = req_fd;
    s_rep_fd = rep_fd;
    s_node   = node;
}

void ipc_round_trip(const ipc_req_t *rq, ipc_rep_t *rp)
{
    ipc_req_t r = *rq;

    r.node = s_node;

    if (!ipc_write_all(s_req_fd, &r, sizeof(r))) { _exit(0); }
    if (!ipc_read_all(s_rep_fd, rp, sizeof(*rp))) { _exit(0); }

    /* The parent ends the run by answering the next request with STOP, so a
     * child never has to poll for termination. */
    if (rp->kind == REP_STOP) { _exit(0); }
}
