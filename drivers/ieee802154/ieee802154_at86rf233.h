/*
 * ieee802154_at86rf233.h
 *
 *  Created on: 2017年1月14日
 *      Author: ldc
 */

#ifndef __IEEE802154_AT86RF233_H__
#define __IEEE802154_AT86RF233_H__

#include <sections.h>
#include <atomic.h>
#include <spi.h>

/* Runtime context structure
 ***************************
 */
struct at86rf233_spi {
	struct device *dev;
	uint32_t slave;
	/**
	 * cmd_buf will use at most 9 bytes:
	 * dummy bytes + 8 ieee address bytes
	 */
	uint8_t cmd_buf[12];
};

struct at86rf233_context {
	struct net_if *iface;
	/**************************/
	struct gpio_callback irqb_cb;
	struct at86rf233_spi spi;
	uint8_t mac_addr[8];
	struct device *gpio;
	/*********TX + CCA*********/
	atomic_t tx;
	/************RX************/
	char __stack at86rf233_rx_stack[CONFIG_ATMEL_AT86RF233_RX_STACK_SIZE];
	struct k_sem rx_lock;
	uint8_t lqi;
	uint8_t rssi;
	uint8_t rx_status;
};

#include "ieee802154_at86rf233_regs.h"

#endif /* __IEEE802154_AT86RF233_H__ */
