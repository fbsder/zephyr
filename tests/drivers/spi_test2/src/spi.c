/* spi.c - SPI test source file */

/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <string.h>
#include <spi.h>
#include <misc/printk.h>

#define SPI_DRV_NAME CONFIG_SPI_5_NAME
#define SPI_MAX_CLK_FREQ_5MHZ MHZ(5)

unsigned char wbuf[16] = "Hello";
unsigned char rbuf[16] = {};

static void print_buf_hex(unsigned char *b, uint32_t len)
{
	for (; len > 0; len--) {
		printk("0x%x ", *(b++));
	}

	printk("\n");
}

struct spi_config spi_conf = {
	.config = SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB,
	.max_sys_freq = SPI_MAX_CLK_FREQ_5MHZ,
};

static void _spi_show(struct spi_config *spi_conf)
{
	printk("SPI Configuration:\n");
	printk("\tMode: %u\n", SPI_MODE(spi_conf->config));
	printk("\tMax speed Hz: 0x%X\n", spi_conf->max_sys_freq);
}

void main(void)
{
	struct device *spi;
	uint32_t len = 0;

	printk("==== SPI Test Application ====\n");

	spi = device_get_binding(SPI_DRV_NAME);
	if (!spi) {
		printk("SPI device not found\n");
		return;
	}

	printk("Running...\n");

	if (spi_configure(spi, &spi_conf) != 0) {
		printk("SPI config failed\n");
		return;
	}

	_spi_show(&spi_conf);

	printk("Writing...\n");

	if (spi_write(spi, (uint8_t *) wbuf, 6) != 0) {
		printk("SPI write failed\n");
		return;
	}

	printk("SPI sent: %s\n", wbuf);
	print_buf_hex(wbuf, 6);

	strcpy((char *)wbuf, "So what then?");

	len = strlen(wbuf);
	/*
	 * len does not include string terminator.
	 * Let's sent the terminator as well.
	 * Also make sure tx and rx have the same length.
	 */
	if (spi_transceive(spi, wbuf, len + 1, rbuf, len +1) != 0) {
		printk("SPI transceive failed\n");
		return;
	}

	printk("SPI transceived: %s\n", rbuf);
	print_buf_hex(rbuf, len);
}
