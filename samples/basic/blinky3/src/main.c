/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>

/* Change this if you have an LED connected to a custom port */
#define PORTNAME	CONFIG_GPIO_ATMEL_SAMR21_PORT_DEV_NAME

/* Change this if you have an LED connected to a custom pin */
// LED0

/* 1000 msec = 1 sec */
#define SLEEP_TIME 	1000

void main(void)
{
	int cnt = 0;
	struct device *dev;

	dev = device_get_binding(PORTNAME);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED0, GPIO_DIR_OUT);

	while (1) {
		/* Set pin to HIGH/LOW every 1 second */
		gpio_pin_write(dev, LED0, cnt % 2);
		cnt++;
		k_sleep(SLEEP_TIME);
	}
}
