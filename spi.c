#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#ifdef SPI_STATS
#include <math.h>
#endif

#include "spi.h"
#include "compat.h"
#include "logging.h"

#include <windows.h>
#include "USBIOX.H"

/* Default SPI clock rate, in kHz */
#define SPIMAXCLOCK     750

static uint8_t *ch_out_buf = NULL, *ch_in_buf = NULL;
static size_t ch_buf_size = 0;
static unsigned int ch_out_buf_offset;

static int spi_dev_open = 0;
static int spi_nrefs = 0;

static int ch341_index = -1;

#ifdef SPI_STATS
static struct spi_stats {
    long reads, writes;
    long read_bytes, write_bytes;
    struct timeval tv_xfer_begin, tv_xfer;
    struct timeval tv_open_begin, tv_open;
    unsigned long spi_clock_max, spi_clock_min;
    unsigned long slowdowns;
} spi_stats;
#endif

#define SPI_MAX_PORTS   16
static struct spi_port spi_ports[SPI_MAX_PORTS];
static int spi_nports = 0;

unsigned long spi_clock = 0, spi_max_clock = SPIMAXCLOCK;


static char *spi_err_buf = NULL;
static size_t spi_err_buf_sz = 0;

void spi_set_err_buf(char *buf, size_t sz)
{
    if (buf && sz) {
        spi_err_buf = buf;
        spi_err_buf_sz = sz;
    } else {
        spi_err_buf = NULL;
        spi_err_buf_sz = 0;
    }
}

#define SPI_ERR(...)   do { \
        LOG(ERR, __VA_ARGS__); \
        spi_err(__VA_ARGS__); \
    } while (0)

static void spi_err(const char *fmt, ...) {
    static char buf[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    if (spi_err_buf) {
        strncpy(spi_err_buf, buf, spi_err_buf_sz);
        spi_err_buf[spi_err_buf_sz - 1] = '\0';
    }
    va_end(args);
}

/*
 * CH341 transfer data forth and back in synchronous mode
 */

static int spi_ch_xfer(uint8_t *out_buf, uint8_t *in_buf, int size)
{
    BOOL rc;
    uint8_t *bufp;
    int len;
    int lost_buffer_counter = 0;

    // We need to copy the buffer from out_buf to in_buf.
    memcpy(in_buf, out_buf, size);

    rc = USBIO_StreamSPI4(ch341_index, 0x80, size, in_buf);
    if (!rc) {
        SPI_ERR("CH341: write data failed: [%d]", rc);
        return -1;
    }

    return 0;
}

/*
 * spi_xfer_*() use global output and input buffers. Output buffer is flushed
 * on the following conditions:
 *  * when buffer becomes full;
 *  * on the clock change;
 *  * before read operation in spi_xfer();
 *  * if the running status of the CPU is requested from spi_xfer_begin();
 *  * at the closure of CH341 device.
 * Read operations are only done in spi_xfer() and spi_xfer_begin(), in other
 * situations we may safely discard what was read into the input buffer buffer
 * by spi_ch_xfer().
 */

int spi_xfer_begin(int get_status)
{
    unsigned int status_offset = 0;
    int status;

    LOG(DEBUG, "");

    if (spi_clock == 0) {
        SPI_ERR("SPI clock not initialized");
        return -1;
    }

#ifdef SPI_STATS
    if (gettimeofday(&spi_stats.tv_xfer_begin, NULL) < 0)
        LOG(WARN, "gettimeofday failed: %s", strerror(errno));
#endif

    /* Check if there is enough space in the buffer */
    if (ch_buf_size - ch_out_buf_offset < 7) {
        /* There is no room in the buffer for the following operations, flush
         * the buffer */
        if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
            return -1;
        /* The data in the buffer is useless, discard it */
        ch_out_buf_offset = 0;
    }

    /* BlueCore chip SPI port reset sequence: deassert CS, wait at least two
     * clock cycles */
	ch_out_buf[ch_out_buf_offset++] = 0;
	ch_out_buf[ch_out_buf_offset++] = 0;

    /* Start transfer */

	ch_out_buf[ch_out_buf_offset++] = 0;

    if (get_status) {
        /*
         * Read the stopped status of the CPU. From CSR8645 datasheet: "When
         * CSR8645 BGA is deselected (SPI_CS# = 1), the SPI_MISO line does not
         * float. Instead, CSR8645 BGA outputs 0 if the processor is running or
         * 1 if it is stopped". However in practice this is not entirely true.
         * Reading MISO while the CPU is deselected gives wrong result. But
         * reading it just after selecting gives the actual status. Also both
         * sources I consulted (CsrSpiDrivers and CsrUsbSpiDeviceRE) are
         * reading the status after setting CS# to 0.
         */

        status_offset = ch_out_buf_offset;
		ch_out_buf[ch_out_buf_offset++] = 0;

        if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
            return -1;

        if (ch_in_buf[status_offset] & 0x1)
            status = SPI_CPU_STOPPED;
        else
            status = SPI_CPU_RUNNING;

        /* Other data in the buffer is useless, discard it */
        ch_out_buf_offset = 0;

        return status;
    }

    return 0;
}

int spi_xfer_end(void)
{
    LOG(DEBUG, "");

    /* Check if there is enough space in the buffer */
    if (ch_buf_size - ch_out_buf_offset < 2) {
        /* There is no room in the buffer for the following operations, flush
         * the buffer */
        if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
            return -1;
        /* The data in the buffer is useless, discard it */
        ch_out_buf_offset = 0;
    }

    /* Commit the last ch_pin_state after spi_xfer() */
    ch_out_buf[ch_out_buf_offset++] = 0;

    ch_out_buf[ch_out_buf_offset++] = 0;

    /* Buffer flush is done on close */

#ifdef SPI_STATS
    {
        struct timeval tv;

        if (gettimeofday(&tv, NULL) < 0)
            LOG(WARN, "gettimeofday failed: %s", strerror(errno));
        timersub(&tv, &spi_stats.tv_xfer_begin, &tv);
        timeradd(&spi_stats.tv_xfer, &tv, &spi_stats.tv_xfer);
    }
#endif
    return 0;
}

int spi_xfer(int cmd, int iosize, void *buf, int size)
{
    unsigned int write_offset, read_offset, ch_in_buf_offset;
    uint16_t bit, word;

    LOG(DEBUG, "(%d, %d, %p, %d)", cmd, iosize, buf, size);

    write_offset = 0;
    read_offset = 0;

    do {
        /* The read, if any, will start at current buffer offset */
        ch_in_buf_offset = ch_out_buf_offset;

        while (write_offset < size) {
            /* 2 bytes per bit */
            if (ch_buf_size - ch_out_buf_offset < iosize * 2) {
                /* There is no room in the buffer for following word write,
                 * flush the buffer */
                if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
                    return -1;
                ch_out_buf_offset = 0;

                /* Let following part to parse from ch_in_buf if needed */
                break;
            }

            if (iosize == 8)
                word = ((uint8_t *)buf)[write_offset];
            else
                word = ((uint16_t *)buf)[write_offset];

            /* MOSI is sensed by BlueCore on the rising edge of CLK, MISO is
             * changed on the falling edge of CLK. */
            for (bit = (1 << (iosize - 1)); bit != 0; bit >>= 1) {
                if (cmd & SPI_XFER_WRITE) {
                    /* Set output bit */
                    if (word & bit)
                        ch_pin_state |= spi_pins->mosi;
                    else
                        ch_pin_state &= ~spi_pins->mosi;
                } else {
                    /* Write 0 during a read */
                    ch_pin_state &= ~spi_pins->mosi;
                }

                ch_out_buf[ch_out_buf_offset++] = ch_pin_state;

                /* Clock high */
                ch_pin_state |= spi_pins->clk;
                ch_out_buf[ch_out_buf_offset++] = ch_pin_state;

                /* Clock low */
                ch_pin_state &= ~spi_pins->clk;
            }
            write_offset++;
        }

        if (cmd & SPI_XFER_READ) {
            if (ch_out_buf_offset) {
                if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
                    return -1;
                ch_out_buf_offset = 0;
            }
            while (read_offset < write_offset) {
                word = 0;
                for (bit = (1 << (iosize - 1)); bit != 0; bit >>= 1) {
                    /* Input bit */
                    ch_in_buf_offset++;
                    if (ch_in_buf[ch_in_buf_offset] & spi_pins->miso)
                        word |= bit;
                    ch_in_buf_offset++;
                }

                if (iosize == 8)
                    ((uint8_t *)buf)[read_offset] = (uint8_t)word;
                else
                    ((uint16_t *)buf)[read_offset] = word;

                read_offset++;
            }
            /* Reading done, reset buffer */
            ch_out_buf_offset = 0;
            ch_in_buf_offset = 0;
        }

    } while (write_offset < size);

#ifdef SPI_STATS
    if (cmd & SPI_XFER_WRITE) {
        spi_stats.writes++;
        spi_stats.write_bytes += size * iosize / 8;
    } else {
        spi_stats.reads++;
        spi_stats.read_bytes += size * iosize / 8;
    }
#endif

    return size;
}

/* Fills spi_ports array with discovered devices, sets spi_nports */
static int spi_enumerate_ports(void)
{
    int id, index;
    const char* name = NULL;

    spi_nports = 0;

    for (index = 0; (name = USBIO_GetDeviceName(index)) && spi_nports < SPI_MAX_PORTS; index++) {
        spi_ports[spi_nports].ch341_index = index;
        strcpy(spi_ports[spi_nports].name, name);
        LOG(INFO, "Found device: name=\"%s\", id=0x%04x", spi_ports[spi_nports].name, spi_ports[spi_nports].ch341_index);

        spi_nports++;
    }

    return 0;
}

int spi_init(void)
{
    LOG(DEBUG, "spi_nrefs=%d, spi_dev_open=%d", spi_nrefs, spi_dev_open);

    spi_nrefs++;

    if (spi_nrefs > 1) {
        LOG(WARN, "Superfluos call to spi_init()");
        return 0;
    }

    if (spi_enumerate_ports() < 0) {
        spi_deinit();
        return -1;
    }

    spi_pins = &spi_pin_presets[spi_pinout];

    return 0;
}

int spi_get_port_list(struct spi_port **pportlist, int *pnports)
{
    if (spi_nrefs < 1) {
        SPI_ERR("CH341: spi not initialized");
        return -1;
    }

    if (pportlist)
        *pportlist = spi_ports;
    if (pnports)
        *pnports = spi_nports;

    return 0;
}

int spi_deinit(void)
{
    LOG(DEBUG, "spi_nrefs=%d, spi_dev_open=%d", spi_nrefs, spi_dev_open);

    if (spi_nrefs) {
        if (spi_dev_open)
            if (spi_close() < 0)
                return -1;
        spi_nrefs = 0;
    }
    return 0;
}

int spi_set_clock(unsigned long spi_clk) {
    uint16_t mode_word = 0x81;
    BOOL rc;

    LOG(DEBUG, "(%lu)", spi_clk);

    if (!spi_isopen()) {
        SPI_ERR("CH341: setting SPI clock failed: SPI device is not open");
        return -1;
    }

    if (spi_clk >= spi_max_clock) {
		mode_word = 0x83;
        spi_clk = spi_max_clock;
    } else if (spi_clk >= 400) {
        spi_clk = 400;
        mode_word = 0x82;
    } else if (spi_clk >= 100) {
        spi_clk = 100;
        mode_word = 0x81;
    } else {
        spi_clk = 20;
        mode_word = 0x80;
    }

    spi_clock = spi_clk;

    /* Flush the write buffer before setting clock */
    if (ch_out_buf_offset) {
        if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
            return -1;
        /* The data in the buffer is useless, discard it */
        ch_out_buf_offset = 0;
    }

    LOG(INFO, "CH341: setting SPI clock to %lu (CH341 mode %lu)", spi_clk, mode_word);
    rc = USBIO_SetStream(ch341_index, mode_word);
    if (!rc) {
        SPI_ERR("CH341: set baudrate %lu failed.", mode_word);
        return -1;
    }

#ifdef SPI_STATS
    if (spi_stats.spi_clock_max == 0)
        spi_stats.spi_clock_max = spi_max_clock;
    if (spi_stats.spi_clock_min == 0)
        spi_stats.spi_clock_min = spi_max_clock;
    /* Don't account for slow cmds, that are executing at 20 kHz,
     * they are short and not representative */
    if (spi_clock > 20 && spi_clock < spi_stats.spi_clock_min)
            spi_stats.spi_clock_min = spi_clock;
#endif
    return 0;
}

void spi_set_max_clock(unsigned long clk) {
    LOG(INFO, "CH341: setting SPI max clock: %lu", clk);
    spi_max_clock = clk;
}

int spi_clock_slowdown(void) {
    unsigned long clk = spi_clock;

    /* Slow SPI clock down by 1.5 */
    if (clk >= 750) {
        clk = 400;
    } else if (clk >= 400) {
        clk = 100;
    } else if (clk >= 100) {
        clk = 20;
    } else {
        // No way to slow down.
        spi_clock = 20;
    }

#ifdef SPI_STATS
    spi_stats.slowdowns++;
#endif

    LOG(INFO, "CH341: SPI clock slowdown, set SPI clock to %lu", clk);
    return spi_set_clock(clk);
}

unsigned long spi_get_max_clock(void) {
    return spi_max_clock;
}

unsigned long spi_get_clock(void) {
    return spi_clock;
}

int spi_open(int nport)
{
    BOOL rc;
    char *serial;
    uint8_t output_pins;

    LOG(DEBUG, "(%d) spi_dev_open=%d", nport, spi_dev_open);

    if (spi_dev_open > 0) {
        LOG(WARN, "Superfluos call to spi_open()");
        return 0;
    }

    if (spi_nports == 0 || nport < spi_nports - 1) {
        SPI_ERR("No CH341 device found");
        goto open_err;
    }

#ifdef SPI_STATS
    memset(&spi_stats, 0, sizeof(spi_stats));
    if (gettimeofday(&spi_stats.tv_open_begin, NULL) < 0)
        LOG(WARN, "gettimeofday failed: %s", strerror(errno));
#endif
    if (!USBIO_OpenDevice(spi_ports[nport].ch341_index)) {
        SPI_ERR("CH341: USBIO_OpenDevice() failed");
        goto open_err;
    }

    rc = USBIO_SetStream(spi_ports[nport].ch341_index, 0x83);
    if (!rc)
    {
        SPI_ERR("CH341: USBIO_SetStream() failed.");
        goto open_err;
    }

    spi_dev_open++;

	LOG(INFO, "CH341: using CH341 device: \"%s\"", spi_ports[nport].name);

	ch_buf_size = 8192;

    /* Initialize xfer buffers */
    ch_out_buf = malloc(ch_buf_size);
    ch_in_buf = malloc(ch_buf_size);
    if (ch_out_buf == NULL || ch_in_buf == NULL) {
        SPI_ERR("Not enough memory");
        goto open_err;
    }
    ch_out_buf_offset = 0;

    /* Set initial pin state: CS high, MISO high as pullup, MOSI and CLK low, LEDs off */
    ch_pin_state = spi_pins->ncs | spi_pins->miso;
    if (spi_pins->nledr)
        ch_pin_state |= spi_pins->nledr;
    if (spi_pins->nledw)
        ch_pin_state |= spi_pins->nledw;
    ch_out_buf[ch_out_buf_offset++] = ch_pin_state;

    ch341_index = spi_ports[nport].ch341_index;

    return 0;

open_err:
    if (spi_dev_open > 0)
        USBIO_CloseDevice(spi_ports[nport].ch341_index);
    spi_dev_open = 0;

    return -1;
}

int spi_isopen(void)
{
    return spi_dev_open ? 1 : 0;
}

#ifdef SPI_STATS
void spi_output_stats(void)
{
    double xfer_pct, avg_read, avg_write, rate, iops;
    struct timeval tv;
    long inxfer_ms;
    FILE *fp;

    fp = log_get_dest();
    if (!fp)
        return;

    /* Calculate timeranges until now */
    if (gettimeofday(&tv, NULL) < 0)
        LOG(WARN, "gettimeofday failed: %s", strerror(errno));
    timersub(&tv, &spi_stats.tv_open_begin, &tv);
    timeradd(&spi_stats.tv_open, &tv, &spi_stats.tv_open);

    xfer_pct = avg_read = avg_write = rate = iops = NAN;

    if (spi_stats.tv_open.tv_sec || spi_stats.tv_open.tv_usec) {
        xfer_pct = (spi_stats.tv_xfer.tv_sec * 1000 + spi_stats.tv_xfer.tv_usec / 1000);
        xfer_pct *= 100;
        xfer_pct /= (spi_stats.tv_open.tv_sec * 1000 + spi_stats.tv_open.tv_usec / 1000);
    }

    if (spi_stats.reads) {
        avg_read = spi_stats.read_bytes;
        avg_read /= spi_stats.reads;
    }

    if (spi_stats.writes) {
        avg_write = spi_stats.write_bytes;
        avg_write /= spi_stats.writes;
    }

    inxfer_ms = spi_stats.tv_xfer.tv_sec * 1000 + spi_stats.tv_xfer.tv_usec / 1000;
    if (inxfer_ms > 0) {
        rate = ((spi_stats.read_bytes + spi_stats.write_bytes) * 1000) /
            inxfer_ms;
        rate /= 1024;   /* In KB/s */

        iops = ((spi_stats.reads + spi_stats.writes) * 1000) / inxfer_ms;
    }

    fprintf(fp,
            "*** CH341 Statistics ********************************************************\n"
            "csr-spi-ch341 version: " VERSION " (git rev " GIT_REVISION ")\n"
            "Time open: %ld.%02ld s\n"
            "Time in xfer: %ld.%02ld s (%.2f%% of open time)\n"
            "Reads: %ld (%ld bytes, %.2f bytes avg read size)\n"
            "Writes: %ld (%ld bytes, %.2f bytes avg write size)\n"
            "Xfer data rate: %.2f KB/s (%ld bytes in %ld.%02ld s)\n"
            "IOPS: %.2f IO/s (%ld IOs in %ld.%02ld s)\n"
            "SPI max clock: %lu kHz, min clock: %lu kHz, slowdowns: %lu\n"
            "****************************************************************************\n",
            spi_stats.tv_open.tv_sec, spi_stats.tv_open.tv_usec / 10000,
            spi_stats.tv_xfer.tv_sec, spi_stats.tv_xfer.tv_usec / 10000, xfer_pct,
            spi_stats.reads, spi_stats.read_bytes, avg_read,
            spi_stats.writes, spi_stats.write_bytes, avg_write,
            rate, spi_stats.read_bytes + spi_stats.write_bytes,
                spi_stats.tv_xfer.tv_sec, spi_stats.tv_xfer.tv_usec / 10000,
            iops, spi_stats.reads + spi_stats.writes,
                spi_stats.tv_xfer.tv_sec, spi_stats.tv_xfer.tv_usec / 10000,
            spi_stats.spi_clock_max, spi_stats.spi_clock_min, spi_stats.slowdowns
    );
}
#endif

int spi_close(void)
{
    LOG(DEBUG, "spi_nrefs=%d, spi_dev_open=%d", spi_nrefs, spi_dev_open);

    if (spi_dev_open) {
#ifdef ENABLE_LEDS
        spi_led(SPI_LED_OFF);
#endif
        /* Flush and reset the buffers */
        if (ch_out_buf_offset) {
            if (spi_ch_xfer(ch_out_buf, ch_in_buf, ch_out_buf_offset) < 0)
                return -1;
            ch_out_buf_offset = 0;
        }

        USBIO_CloseDevice(ch341_index);

#ifdef SPI_STATS
        spi_output_stats();
#endif

        free(ch_out_buf);
        free(ch_in_buf);
        ch_out_buf = ch_in_buf = NULL;
        ch_buf_size = 0;

        spi_dev_open = 0;
    }

    return 0;
}
