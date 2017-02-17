/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Driver for the Atmel SAMR21 PIO Controller.
 */

#include <errno.h>

#include <kernel.h>

#include <device.h>
#include <init.h>

#include <sam.h>

#include <soc.h>

#include <gpio.h>

#include "gpio_utils.h"

typedef void (*config_func_t)(struct device *port);

/* Configuration data */
struct gpio_samr21_config {
	volatile Port 	*port;
	config_func_t	config_func;
};

struct gpio_samr21_runtime {
	/* callbacks */
	sys_slist_t		cb;
};

static int ext_irq_setup(struct device *dev, uint32_t portpin, int flags)
{
	enum { NONE = 0, RISE, FALL, BOTH, HIGH, LOW };
	uint32_t old_ext_irq = 0, old_wakeup = 0, old_config = 0;
	int8_t extint = -1, index = 0, offset = 0;
	uint8_t int_mode = 0;

	extint = find_ext_int(dev, portpin);
	if (extint == -1)
	{
		return -EINVAL;
	}
	index = extint >> 0x3;
	offset = (extint & 0x7) << 2;

    if (flags & GPIO_INT_DEBOUNCE)
    {
    	int_mode = 1U << 3;
    }
	if (flags & GPIO_INT_DOUBLE_EDGE) {
		int_mode |= BOTH;
	} else {
		if (flags & GPIO_INT_EDGE) {
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				int_mode |= RISE;
			} else {
				int_mode |= FALL;
			}
		} else {
			if (flags & GPIO_INT_ACTIVE_HIGH) {
				int_mode |= HIGH;
			} else {
				int_mode |= LOW;
			}
		}
	}

    while (EIC->STATUS.bit.SYNCBUSY) {}
    if (EIC->CTRL.bit.ENABLE == 1)
    {
    	old_ext_irq = EIC->INTENSET.vec.EXTINT;
    	old_wakeup = EIC->WAKEUP.vec.WAKEUPEN;
    	old_config = EIC->CONFIG[index].reg;
    }
    EIC->CTRL.bit.SWRST = 1;
    while (EIC->STATUS.bit.SYNCBUSY) {}

    set_pin_func(portpin, FUNCA);

    EIC->WAKEUP.reg = old_wakeup | (1U << extint);
	EIC->CONFIG[index].reg = old_config | (int_mode << offset);

	EIC->INTENSET.reg = old_ext_irq | (1U << extint);

	while (EIC->STATUS.bit.SYNCBUSY) {}
	EIC->CTRL.bit.ENABLE = 1;

	return 0;
}

/**
 * @brief Configurate pin or port
 *
 * @param dev Device struct
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_samr21_config(struct device *dev, int access_op,
			    uint32_t portpin, int flags)
{
	if (access_op == GPIO_ACCESS_BY_PIN)
	{
		const struct gpio_samr21_config *cfg = dev->config->config_info;

		uint8_t port = GPIOPORT(portpin);
		uint8_t pin = GPIOPIN(portpin);

		uint32_t mask = 1U << pin;

		/* Disable the pin and return as setup is meaningless now */
		if (flags & GPIO_PIN_DISABLE) {
			cfg->port->Group[port].DIRCLR.reg = mask;
			cfg->port->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
									(mask & 0x0000ffff);
			cfg->port->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
								   PORT_WRCONFIG_WRPINCFG |
								   ((mask & 0xffff0000) >> 16);
			return 0;
		}

		/* Setup the pin direction */
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_OUT) {
			cfg->port->Group[port].DIRSET.reg = mask;
			cfg->port->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
									(mask & 0x0000ffff);
			cfg->port->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
								   PORT_WRCONFIG_WRPINCFG |
								   ((mask & 0xffff0000) >> 16);
		} else {
			cfg->port->Group[port].DIRCLR.reg = mask;
			cfg->port->Group[port].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
										  PORT_WRCONFIG_INEN |
										  (mask & 0x0000ffff);
			cfg->port->Group[port].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
								   PORT_WRCONFIG_WRPINCFG |
								   PORT_WRCONFIG_INEN     |
								   ((mask & 0xffff0000) >> 16);
		}

		/* Pull-up? */
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			/* Enable pull-up */
			cfg->port->Group[port].DIRCLR.reg = mask;
			cfg->port->Group[port].PINCFG[pin].bit.PULLEN = 1;
			cfg->port->Group[port].OUTSET.reg = mask;
		} else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			/* Disable pull-up */
			cfg->port->Group[port].DIRCLR.reg = mask;
			cfg->port->Group[port].PINCFG[pin].bit.PULLEN = 1;
			cfg->port->Group[port].OUTCLR.reg = mask;
		} else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			cfg->port->Group[port].PINCFG[pin].bit.PULLEN = 0;
		}

		/* Setup interrupt config */
		if (flags & GPIO_INT) {
			int rt = ext_irq_setup(dev, pin, flags);
			if (rt != 0)
			{
				return rt;
			}
		}
	}
	else if (access_op == GPIO_ACCESS_BY_PORT)
	{
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Set the pin or port output
 *
 * @param dev Device struct
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_samr21_write(struct device *dev, int access_op,
			   uint32_t portpin, uint32_t value)
{
	const struct gpio_samr21_config *cfg = dev->config->config_info;
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);
	uint32_t mask = 1U << pin;

	if (access_op == GPIO_ACCESS_BY_PORT)
	{
		return -ENOTSUP;
	}

	if (value)
	{
		cfg->port->Group[port].OUTSET.reg = mask;
	}
	else
	{
		cfg->port->Group[port].OUTCLR.reg = mask;
	}

	return 0;
}

/**
 * @brief Read the pin or port status
 *
 * @param dev Device struct
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_samr21_read(struct device *dev, int access_op,
				       uint32_t portpin, uint32_t *value)
{
	const struct gpio_samr21_config *cfg = dev->config->config_info;
	uint8_t port = GPIOPORT(portpin);
	uint8_t pin = GPIOPIN(portpin);

	if (access_op == GPIO_ACCESS_BY_PORT)
	{
		return -ENOTSUP;
	}

	*value = (cfg->port->Group[port].IN.reg >> pin) & 0x01;

	return 0;
}

static void gpio_samr21_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct gpio_samr21_runtime *context = dev->driver_data;
	uint32_t int_stat;

	int_stat = EIC->INTFLAG.reg;
	EIC->INTFLAG.reg = int_stat;

	_gpio_fire_callbacks(&context->cb, dev, int_stat);
}

static int gpio_samr21_manage_callback(struct device *dev,
				     struct gpio_callback *callback,
				     bool set)
{
	struct gpio_samr21_runtime *context = dev->driver_data;

	_gpio_manage_callback(&context->cb, callback, set);

	return 0;
}

static int gpio_samr21_enable_callback(struct device *dev,
				     int access_op, uint32_t portpin)
{
	const struct gpio_samr21_config *cfg = dev->config->config_info;
	int8_t extint;

	(void)cfg;

	if (access_op == GPIO_ACCESS_BY_PORT)
	{
		return -ENOTSUP;
	}

	extint = find_ext_int(dev, portpin);
	EIC->INTENSET.reg = 1U << extint;

	return 0;
}

static int gpio_samr21_disable_callback(struct device *dev,
				      int access_op, uint32_t portpin)
{
	const struct gpio_samr21_config *cfg = dev->config->config_info;
	int8_t extint;

	(void)cfg;

	if (access_op == GPIO_ACCESS_BY_PORT)
	{
		return -ENOTSUP;
	}

	extint = find_ext_int(dev, portpin);
	EIC->INTENCLR.reg = 1U << extint;

	return 0;
}

static const struct gpio_driver_api gpio_samr21_drv_api_funcs = {
	.config = gpio_samr21_config,
	.write = gpio_samr21_write,
	.read = gpio_samr21_read,
	.manage_callback = gpio_samr21_manage_callback,
	.enable_callback = gpio_samr21_enable_callback,
	.disable_callback = gpio_samr21_disable_callback,
};

/**
 * @brief Initialization function of MMIO
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_samr21_init(struct device *dev)
{
	const struct gpio_samr21_config *cfg = dev->config->config_info;

	cfg->config_func(dev);

	return 0;
}

/* Port */
#ifdef CONFIG_GPIO_ATMEL_SAMR21_PORT
static void gpio_samr21_dev_config(struct device *dev);

static const struct gpio_samr21_config gpio_samr21_cfg = {
	.port = PORT,

	.config_func = gpio_samr21_dev_config,
};

static struct gpio_samr21_runtime gpio_samr21_runtime;

DEVICE_AND_API_INIT(gpio_samr21, CONFIG_GPIO_ATMEL_SAMR21_PORT_DEV_NAME,
		    gpio_samr21_init, &gpio_samr21_runtime, &gpio_samr21_cfg,
			PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &gpio_samr21_drv_api_funcs);

static void gpio_samr21_dev_config(struct device *dev)
{
    /* enable sync and async clocks */
    PM->APBBMASK.reg |= PM_APBBMASK_PORT;
    PM->APBAMASK.reg |= PM_APBAMASK_EIC;
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EIC);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

	IRQ_CONNECT(IRQ_EIC, CONFIG_GPIO_ATMEL_SAMR21_PORT_IRQ_PRI,
		    gpio_samr21_isr, DEVICE_GET(gpio_samr21), 0);
	irq_enable(IRQ_EIC);
}
#endif /* CONFIG_GPIO_ATMEL_SAMR21_PORTA */
