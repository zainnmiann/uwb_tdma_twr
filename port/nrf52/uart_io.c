#include "uart_io.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"

#if (UART_TX_RING_SZ & (UART_TX_RING_SZ - 1u)) != 0u
#error "UART_TX_RING_SZ must be a power of two"
#endif

#define RING_MASK   (UART_TX_RING_SZ - 1u)

static volatile uint8_t  m_tx_ring[UART_TX_RING_SZ];
static volatile uint32_t m_tx_head;      /* producer */
static volatile uint32_t m_tx_tail;      /* consumer (IRQ) */
static volatile bool     m_tx_active;
static volatile uint32_t m_dropped;

/* RX is a byte FIFO, not a line buffer. The previous double-buffer scheme
 * dropped every byte arriving while a completed line awaited pickup, and that
 * window is a whole role-loop iteration wide -- one run_frame() spans the frame
 * duration, 4 ms by default -- so a host writing "$RANGE,STOP\r\n$SET,...\r\n"
 * in a single 921600 baud burst lost the second command entirely, with no error.
 * m_head is written only by the ISR and m_tail only by the main loop, so the
 * 32-bit aligned volatile accesses need no critical section.
 *
 * Legacy note: the main loop parses the line handed out by uart_io_poll_line().
 * serial_poll() writes NULs into
 * that buffer while tokenizing, so a single shared buffer would let a second
 * command arriving in the same burst (routine at 921600) overwrite the one
 * being parsed. Volatile because the ISR is the sole writer of the indices. */
#define UART_RX_FIFO_SZ     256u

static volatile uint8_t  m_fifo[UART_RX_FIFO_SZ];
static volatile uint32_t m_head;         /* ISR writes            */
static volatile uint32_t m_tail;         /* main loop reads       */
static volatile uint32_t m_rx_dropped;   /* FIFO-full byte count  */
static char              m_line[UART_RX_LINE_SZ];
static uint32_t          m_line_len;     /* partial line, main loop only */

/* The nRF52 UART takes a fixed register value per rate, so only the rates the
 * hardware actually enumerates are reachable. Returns 0 for anything else so
 * the caller can reject it rather than silently running at a different rate
 * than the configuration claims. */
static uint32_t baud_to_reg(uint32_t baud)
{
    switch (baud)
    {
        case 1200u:    return UART_BAUDRATE_BAUDRATE_Baud1200;
        case 2400u:    return UART_BAUDRATE_BAUDRATE_Baud2400;
        case 4800u:    return UART_BAUDRATE_BAUDRATE_Baud4800;
        case 9600u:    return UART_BAUDRATE_BAUDRATE_Baud9600;
        case 14400u:   return UART_BAUDRATE_BAUDRATE_Baud14400;
        case 19200u:   return UART_BAUDRATE_BAUDRATE_Baud19200;
        case 28800u:   return UART_BAUDRATE_BAUDRATE_Baud28800;
        case 38400u:   return UART_BAUDRATE_BAUDRATE_Baud38400;
        case 57600u:   return UART_BAUDRATE_BAUDRATE_Baud57600;
        case 76800u:   return UART_BAUDRATE_BAUDRATE_Baud76800;
        case 115200u:  return UART_BAUDRATE_BAUDRATE_Baud115200;
        case 230400u:  return UART_BAUDRATE_BAUDRATE_Baud230400;
        case 250000u:  return UART_BAUDRATE_BAUDRATE_Baud250000;
        case 460800u:  return UART_BAUDRATE_BAUDRATE_Baud460800;
        case 921600u:  return UART_BAUDRATE_BAUDRATE_Baud921600;
        case 1000000u: return UART_BAUDRATE_BAUDRATE_Baud1M;
        default:       return 0u;
    }
}

bool uart_io_baud_supported(uint32_t baud)
{
    return baud_to_reg(baud) != 0u;
}

static void tx_kick(void)
{
    if (m_tx_active)
    {
        return;
    }
    if (m_tx_head == m_tx_tail)
    {
        return;
    }

    m_tx_active = true;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->TXD = m_tx_ring[m_tx_tail & RING_MASK];
    m_tx_tail++;
}

void UARTE0_UART0_IRQHandler(void)
{
    if (NRF_UART0->EVENTS_TXDRDY != 0)
    {
        NRF_UART0->EVENTS_TXDRDY = 0;

        if (m_tx_head != m_tx_tail)
        {
            NRF_UART0->TXD = m_tx_ring[m_tx_tail & RING_MASK];
            m_tx_tail++;
        }
        else
        {
            m_tx_active = false;
            NRF_UART0->TASKS_STOPTX = 1;
        }
    }

    if (NRF_UART0->EVENTS_RXDRDY != 0)
    {
        uint8_t c;

        NRF_UART0->EVENTS_RXDRDY = 0;
        c = (uint8_t)NRF_UART0->RXD;

        {
            uint32_t nh = m_head + 1u;

            if (nh >= UART_RX_FIFO_SZ) { nh = 0u; }

            if (nh != m_tail)
            {
                m_fifo[m_head] = c;
                m_head         = nh;
            }
            else
            {
                m_rx_dropped++;         /* FIFO full: the host is ahead of us */
            }
        }
    }

    if (NRF_UART0->EVENTS_ERROR != 0)
    {
        NRF_UART0->EVENTS_ERROR = 0;
        (void)NRF_UART0->ERRORSRC;
    }
}

void uart_io_init(uint32_t baud)
{
    m_tx_head   = 0;
    m_tx_tail   = 0;
    m_tx_active = false;
    m_dropped   = 0;
    m_head       = 0;
    m_tail       = 0;
    m_line_len   = 0;
    m_rx_dropped = 0;

    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_pin_set(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->ENABLE      = UART_ENABLE_ENABLE_Disabled;
    NRF_UART0->PSELTXD     = TX_PIN_NUMBER;
    NRF_UART0->PSELRXD     = RX_PIN_NUMBER;
    NRF_UART0->PSELRTS     = 0xFFFFFFFFu;
    NRF_UART0->PSELCTS     = 0xFFFFFFFFu;
    NRF_UART0->CONFIG      = 0;                       /* no parity, no HW flow control */
    {
        uint32_t reg = baud_to_reg(baud);
        NRF_UART0->BAUDRATE = (reg != 0u) ? reg
                                          : UART_BAUDRATE_BAUDRATE_Baud115200;
    }

    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_ERROR  = 0;

    NRF_UART0->ENABLE      = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->INTENSET    = UART_INTENSET_TXDRDY_Msk
                           | UART_INTENSET_RXDRDY_Msk
                           | UART_INTENSET_ERROR_Msk;

    NVIC_SetPriority(UARTE0_UART0_IRQn, 6);
    NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);

    NRF_UART0->TASKS_STARTRX = 1;
}

void uart_io_write(const char *data, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        uint32_t next = m_tx_head + 1u;

        if ((next - m_tx_tail) > UART_TX_RING_SZ)
        {
            m_dropped += (len - i);
            break;
        }
        m_tx_ring[m_tx_head & RING_MASK] = (uint8_t)data[i];
        m_tx_head = next;
    }

    NVIC_DisableIRQ(UARTE0_UART0_IRQn);
    if (!m_tx_active)
    {
        NRF_UART0->TASKS_STARTTX = 1;
        tx_kick();
    }
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);
}

void uart_io_puts(const char *s)
{
    uart_io_write(s, (uint32_t)strlen(s));
}

void uart_io_printf(const char *fmt, ...)
{
    /* Worst-case $RNG JSON record measures 174 bytes; 160 truncated it
     * mid-field, losing the closing brace and CRLF and corrupting every
     * subsequent record for a streaming parser. */
    char    buf[256];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0)
    {
        if ((uint32_t)n > sizeof(buf) - 1u)
        {
            n = (int)sizeof(buf) - 1;
        }
        uart_io_write(buf, (uint32_t)n);
    }
}

void uart_io_flush(void)
{
    while (m_tx_head != m_tx_tail || m_tx_active)
    {
        __WFE();
    }
}

uint32_t uart_io_dropped(void)
{
    return m_dropped;
}

char *uart_io_poll_line(void)
{
    while (m_tail != m_head)
    {
        uint8_t  c = m_fifo[m_tail];
        uint32_t nt = m_tail + 1u;

        if (nt >= UART_RX_FIFO_SZ) { nt = 0u; }
        m_tail = nt;

        if (c == '\n' || c == '\r')
        {
            if (m_line_len > 0u)
            {
                m_line[m_line_len] = '\0';
                m_line_len = 0;
                return m_line;          /* one command; the rest stay queued */
            }
            continue;                   /* bare CR/LF or CRLF pair */
        }

        if (m_line_len < (UART_RX_LINE_SZ - 1u))
        {
            m_line[m_line_len++] = (char)c;
        }
        else
        {
            m_line_len = 0;             /* overlong line: discard to terminator */
        }
    }
    return NULL;
}

uint32_t uart_io_rx_dropped(void)
{
    return m_rx_dropped;
}
