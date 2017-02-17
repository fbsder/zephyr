/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief System/hardware module for Atmel SAM3 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Atmel SAM3 family processor.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <arch/cpu.h>

#include <sam.h>
#include <soc.h>

int8_t irq_pin_map[2][32] =
{
		  /* 0 */ /* 1 */ /* 2 */ /* 3 */ /* 4 */ /* 5 */ /* 6 */ /* 7 */ /* 8 */ /* 9 */ /* 10 *//* 11 *//* 12 *//* 13 *//* 14 *//* 15 */
   {/* A */ -1,		 1,	    -1,	    -1,	     4,	     5,	     6,	     7,	    -1,	 	 9,	     10,	 11,	 12,	 13,	 14,	 15,
    	  /* 16 *//* 17 *//* 18 *//* 19 *//* 20 *//* 21 *//* 22 *//* 23 *//* 24 *//* 25 *//* 26 *//* 27 *//* 28 *//* 29 *//* 30 *//* 31 */
	/* A */ -1,		 1,	     2,	     3,	     -1,     -1,     6,	     7,	     12,     13,     -1,     15,	 8,      -1,	 10,     11,
   },
	      /* 0 */ /* 1 */ /* 2 */ /* 3 */ /* 4 */ /* 5 */ /* 6 */ /* 7 */ /* 8 */ /* 9 */ /* 10 *//* 11 *//* 12 *//* 13 *//* 14 *//* 15 */
   {/* B */  0,		-1,	     2,	     3,	    -1,	    -1,	    -1,	    -1,	    -1,	 	-1,	     -1,	 -1,	 -1,	 -1,	 -1,	 -1,
	      /* 16 *//* 17 *//* 18 *//* 19 *//* 20 *//* 21 *//* 22 *//* 23 *//* 24 *//* 25 *//* 26 *//* 27 *//* 28 *//* 29 *//* 30 *//* 31 */
	/* B */  0,		 1,	     -1,     -1,     -1,     -1,     6,	     7,	     -1,     -1,     -1,     -1,	 -1,     -1,	 -1,     -1,
   },
};

int8_t find_ext_int(struct device *dev, uint32_t portpin)
{
	(void)dev;
	int8_t port = GPIOPORT(portpin);
	int8_t pin = GPIOPIN(portpin);

	if (port >= GPIO_PORTC)
	{
		return -1;
	}

	return irq_pin_map[port][pin];
}

void set_pin_func(uint32_t portpin, enum FUNCS func)
{
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);
    PORT->Group[port].PINCFG[pin].bit.PMUXEN = PORT_PINCFG_PMUXEN;
    if ((pin & 0x1) == 0x1)
    {
    	PORT->Group[port].PMUX[pin >> 1].bit.PMUXO = func;
    } else {
        PORT->Group[port].PMUX[pin >> 1].bit.PMUXE = func;
    }
}

/**
 * @brief Setup various clock on SoC.
 *
 * Setup the SoC clocks according to section 28.12 in datasheet.
 *
 * Assumption:
 * SLCK = 32.768kHz
 */
static ALWAYS_INLINE void clock_init(void)
{
    /* enable clocks for the power, sysctrl and gclk modules */
    PM->APBAMASK.reg = (PM_APBAMASK_PM | PM_APBAMASK_SYSCTRL |
                        PM_APBAMASK_GCLK);

    /* configure internal 8MHz oscillator to run without prescaler */
    SYSCTRL->OSC8M.bit.PRESC = 0;
    SYSCTRL->OSC8M.bit.ONDEMAND = 1;
    SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
    SYSCTRL->OSC8M.bit.ENABLE = 1;
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY)) {}

#if CONFIG_SOC_CLOCK_USE_PLL
    /* reset the GCLK module so it is in a known state */
    GCLK->CTRL.reg = GCLK_CTRL_SWRST;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* setup generic clock 1 to feed DPLL with 1MHz */
    GCLK->GENDIV.reg = (GCLK_GENDIV_DIV(8) |
                        GCLK_GENDIV_ID(1));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_OSC8M |
                         GCLK_GENCTRL_ID(1));
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_GEN(1) |
                         GCLK_CLKCTRL_ID(1) |
                         GCLK_CLKCTRL_CLKEN);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* enable PLL */
    SYSCTRL->DPLLRATIO.reg = (SYSCTRL_DPLLRATIO_LDR(CLOCK_PLL_MUL));
    SYSCTRL->DPLLCTRLB.reg = (SYSCTRL_DPLLCTRLB_REFCLK_GCLK);
    SYSCTRL->DPLLCTRLA.reg = (SYSCTRL_DPLLCTRLA_ENABLE);
    while(!(SYSCTRL->DPLLSTATUS.reg &
           (SYSCTRL_DPLLSTATUS_CLKRDY | SYSCTRL_DPLLSTATUS_LOCK))) {}

    /* select the PLL as source for clock generator 0 (CPU core clock) */
    GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(CLOCK_PLL_DIV) |
                        GCLK_GENDIV_ID(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_FDPLL |
                         GCLK_GENCTRL_ID(0));
#else /* do not use PLL, use internal 8MHz oscillator directly */
    GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(CLOCK_DIV) |
                        GCLK_GENDIV_ID(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN |
                         GCLK_GENCTRL_SRC_OSC8M |
                         GCLK_GENCTRL_ID(0));
#endif

    /* make sure we synchronize clock generator 0 before we go on */
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* Setup Clock generator 2 with divider 1 (32.768kHz) */
    GCLK->GENDIV.reg  = (GCLK_GENDIV_ID(2)  | GCLK_GENDIV_DIV(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN |
            GCLK_GENCTRL_RUNSTDBY |
            GCLK_GENCTRL_SRC_OSCULP32K);

    while (GCLK->STATUS.bit.SYNCBUSY) {}

    /* redirect all peripherals to a disabled clock generator (7) by default */
    for (int i = 0x3; i <= 0x22; i++) {
        GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID(i) | GCLK_CLKCTRL_GEN_GCLK7 );
        while (GCLK->STATUS.bit.SYNCBUSY) {}
    }
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int atmel_samr21_init(struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	/* Note:
	 * Magic numbers below are obtained by reading the registers
	 * when the SoC was running the SAM-BA bootloader
	 * (with reserved bits set to 0).
	 */

	key = irq_lock();

#if CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC > 24000000
	/* CORECLOCK > 24000000
	 * adjust NVM wait states, see table 42.30 (p. 1070) in the datasheet
	 */
    PM->APBAMASK.reg |= PM_AHBMASK_NVMCTRL;
    NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS(1);
    PM->APBAMASK.reg &= ~PM_AHBMASK_NVMCTRL;
#endif

	/* Disable watchdog timer, not used by system */
	WDT->CTRL.bit.ENABLE = 0;

    /* Setup master clock */
    clock_init();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_samr21_init, PRE_KERNEL_1, 0);
