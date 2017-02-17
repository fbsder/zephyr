/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the Atmel SAM3 family processors.
 */

#ifndef _ATMEL_SAMR21_SOC_H_
#define _ATMEL_SAMR21_SOC_H_

/******  Cortex-M0+ Processor Exceptions Numbers ******************************/
#define IRQ_EIC			4	/**<  4 SAMR21G18A External Interrupt Controller (EIC) */
#define IRQ_DMAC		6	/**<  6 SAMR21G18A Direct Memory Access Controller (DMAC) */
#define IRQ_EVSYS		8	/**<  8 SAMR21G18A Event System Interface (EVSYS) */
#define IRQ_SERCOM0		9	/**<  9 SAMR21G18A Serial Communication Interface 0 (SERCOM0) */
#define IRQ_SERCOM1		10	/**< 10 SAMR21G18A Serial Communication Interface 1 (SERCOM1) */
#define IRQ_SERCOM2		11	/**< 11 SAMR21G18A Serial Communication Interface 2 (SERCOM2) */
#define IRQ_SERCOM3		12	/**< 12 SAMR21G18A Serial Communication Interface 3 (SERCOM3) */
#define IRQ_SERCOM4		13	/**< 13 SAMR21G18A Serial Communication Interface 4 (SERCOM4) */
#define IRQ_SERCOM5		14	/**< 14 SAMR21G18A Serial Communication Interface 5 (SERCOM5) */

#if CONFIG_SOC_CLOCK_USE_PLL
/* edit these values to adjust the PLL output frequency */
#define CLOCK_PLL_MUL       (47U)               /* must be >= 31 & <= 95 */
#define CLOCK_PLL_DIV       (1U)                /* adjust to your needs */
/* generate the actual used core clock frequency */
#define CLOCK_CORECLOCK     (((CLOCK_PLL_MUL + 1) * 1000000U) / CLOCK_PLL_DIV)
#else
/* edit this value to your needs */
#define CLOCK_DIV           (1U)
/* generate the actual core clock frequency */
#define CLOCK_CORECLOCK     (8000000 / CLOCK_DIV)
#endif

#define UART_IRQ_FLAGS					0 /* Default */

#ifndef _ASMLANGUAGE

#include <sam.h>
#include <device.h>

#define GPIOPIN(n) (((n)&0x1Fu) << 0)
#define GPIOPORT(n) (((n) >> 5) & 0x7u)
#define GPIOPORTPIN(n)	((n) & 0xFFu)
#define GPIOFUNC(n) ((n) >> 8)
#define GPIOGROUP(port, pin) ((((port)&0x7u) << 5) + ((pin)&0x1Fu))
#define GPIOPINFUNCTIONOFF 0xffffffff

enum FUNCS {FUNCA = 0, FUNCB, FUNCC, FUNCD, FUNCE, FUNCF, FUNCG, FUNCH};

void set_pin_func(uint32_t portpin, enum FUNCS func);


/**
 * \brief PORT group abstraction
 */

enum gpio_port { GPIO_PORTA, GPIO_PORTB, GPIO_PORTC };

int8_t find_ext_int(struct device *dev, uint32_t portpin);

#endif

/** @} */

#endif /* _ATMEL_SAM3_SOC_H_ */
