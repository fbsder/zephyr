/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <misc/util.h>
#include <misc/printk.h>

#define PORTNAME	CONFIG_GPIO_ATMEL_SAMR21_PORT_DEV_NAME

/*
 * If SW0_GPIO_INT_CONF not defined used default EDGE value.
 * Change this to use a different interrupt trigger
 */
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)

/* change this to enable pull-up/pull-down */
#define PULL_UP GPIO_PUD_PULL_UP

/* Sleep time */
#define SLEEP_TIME	500

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %d\n", k_cycle_get_32());
}

static struct gpio_callback gpio_cb;

void main(void)
{
	struct device *gpiob;
	int extint = 0;

	printk("Press the user defined button on the board\n");
	gpiob = device_get_binding(PORTNAME);
	if (!gpiob) {
		printk("error\n");
		return;
	}

	gpio_pin_configure(gpiob, BUTTONSW0,
			   GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);
	extint = 1U << find_ext_int(gpiob, BUTTONSW0);
	gpio_init_callback(&gpio_cb, button_pressed, extint);

	gpio_add_callback(gpiob, &gpio_cb);
	gpio_pin_enable_callback(gpiob, BUTTONSW0);

	while (1) {
		uint32_t val = 0;

		gpio_pin_read(gpiob, BUTTONSW0, &val);
		k_sleep(SLEEP_TIME);
	}
}
