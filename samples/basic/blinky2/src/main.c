/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME 	1000

void main(void)
{
	int cnt = 0;
	printk("start \n");

	while (1) {
		/* Set pin to HIGH/LOW every 1 second */
		printk("%d\n", cnt);
		cnt++;
		k_sleep(SLEEP_TIME);
	}
}
