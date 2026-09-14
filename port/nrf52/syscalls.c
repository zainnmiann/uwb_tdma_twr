/**
 * @file    syscalls.c
 * @brief   Minimal newlib retargeting.
 *
 * Only _write is real: it points stdout/stderr at the same non-blocking UART
 * ring the rest of the firmware uses, so a stray printf() cannot stall the
 * TDMA loop. The rest exist to keep libc_nano from pulling in nosys stubs.
 */

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "uart_io.h"

extern int  _write(int fd, const void *buf, size_t count);
extern int  _read(int fd, void *buf, size_t count);
extern int  _close(int fd);
extern int  _fstat(int fd, struct stat *st);
extern int  _isatty(int fd);
extern off_t _lseek(int fd, off_t offset, int whence);
extern int  _getpid(void);
extern int  _kill(int pid, int sig);
extern void _exit(int status);
extern caddr_t _sbrk(int incr);

int _write(int fd, const void *buf, size_t count)
{
    (void)fd;
    uart_io_write((const char *)buf, (uint32_t)count);
    return (int)count;
}

int _read(int fd, void *buf, size_t count)
{
    (void)fd; (void)buf; (void)count;
    return 0;
}

int _close(int fd)
{
    (void)fd;
    return -1;
}

int _fstat(int fd, struct stat *st)
{
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd)
{
    (void)fd;
    return 1;
}

off_t _lseek(int fd, off_t offset, int whence)
{
    (void)fd; (void)offset; (void)whence;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid; (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    for (;;)
    {
    }
}

caddr_t _sbrk(int incr)
{
    extern char   __HeapBase;
    extern char   __HeapLimit;
    static char  *heap = 0;
    char         *prev;

    if (heap == 0)
    {
        heap = &__HeapBase;
    }
    if ((heap + incr) > &__HeapLimit)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }
    prev = heap;
    heap += incr;
    return (caddr_t)prev;
}
