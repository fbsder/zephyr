/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INC_BOARD_H
#define __INC_BOARD_H

#include <soc.h>

#ifndef _ASMLANGUAGE
/**
 * \brief Macros for the pin and port group, lower 5
 * bits stands for pin number in the group, higher 3
 * bits stands for port group
 */

#define EDBG_COM_TX GPIOGROUP(GPIO_PORTA, 4)
#define EDBG_COM_RX GPIOGROUP(GPIO_PORTA, 5)
#define PA12 		GPIOGROUP(GPIO_PORTA, 12)
#define PA13 		GPIOGROUP(GPIO_PORTA, 13)
#define PA14 		GPIOGROUP(GPIO_PORTA, 14)
#define LED0 		GPIOGROUP(GPIO_PORTA, 19)
#define BUTTONSW0	GPIOGROUP(GPIO_PORTA, 28)

#define RF233IRQPIN	GPIOGROUP(GPIO_PORTB, 0)
#define RF233RSTPIN	GPIOGROUP(GPIO_PORTB, 15)
#define RF233SLPPIN GPIOGROUP(GPIO_PORTA, 20)

#define AT86RF233_PARTNUM (0xB)

#endif // !_ASMLANGUAGE

#endif /* __INC_BOARD_H */
