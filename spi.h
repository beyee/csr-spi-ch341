#ifndef _SPI_H
#define _SPI_H

#include <stdint.h>

/* Bit field */
#define SPI_XFER_READ   (1 << 0)
#define SPI_XFER_WRITE  (1 << 1)

/* XAP CPU running status */
#define SPI_CPU_STOPPED 1
#define SPI_CPU_RUNNING 0

struct spi_port {
    uint16_t ch341_index;
    char name[256];
};

struct spi_pins {
	uint8_t ncs, clk, mosi, miso;
};

/*
* Pinouts. Change at will. Beware that FTDI adapters provide 5V or 3V3 I/O
* levels, but CSR chips require 3V3 or 1V8 I/O level.
*/

/*
* Default pinout, this leaves TX and RX pins free for UART connection.
*
* CS - D0, CLK - D3, MOSI - D5, MISO - D7
*/
#define SPI_PIN_PRESET_DEFAULT \
    { (1 << 0), (1 << 3), (1 << 5), (1 << 7) }

#define SPI_PIN_PRESETS { \
        SPI_PIN_PRESET_DEFAULT, \
    }

enum spi_pinouts {
	SPI_PINOUT_DEFAULT = 0,
};

#ifdef __cplusplus
extern "C" {
#endif

void spi_set_err_buf(char *buf, size_t sz);
void spi_set_pinout(enum spi_pinouts pinout);
int spi_init(void);
int spi_deinit(void);
int spi_get_port_list(struct spi_port **pportlist, int *pnports);

int spi_open(int nport);
int spi_isopen(void);
int spi_close(void);

int spi_set_clock(unsigned long spi_clk);
void spi_set_max_clock(unsigned long clk);
int spi_clock_slowdown(void);
unsigned long spi_get_max_clock(void);
unsigned long spi_get_clock(void);

int spi_xfer_begin(int get_status);
int spi_xfer(int cmd, int iosize, void *buf, int size);
int spi_xfer_end(void);

#ifdef SPI_STATS
void spi_output_stats(void);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
