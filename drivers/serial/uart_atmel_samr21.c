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
 * @brief Driver for UART on Atmel samr21 family processor.
 *
 * Note that there is only one UART controller on the SoC.
 * It has two wires for RX and TX, and does not have other such as
 * CTS or RTS. Also, the RX and TX are connected directly to
 * bit shifters and there is no FIFO.
 *
 * For full serial function, use the USART controller.
 *
 * (used uart_atmel_sam3.c as template)
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <soc.h>
#include <board.h>
#include <init.h>
#include <uart.h>
#include <sections.h>

/* Device data structure */
struct uart_samr21_dev_data_t {
	uint32_t baud_rate;	/* Baud rate */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_t     cb;     /**< Callback function pointer */
#endif
};

/* convenience defines */
#define DEV_CFG(dev) \
	((const struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_samr21_dev_data_t * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile SercomUsart *)(DEV_CFG(dev))->base)

static const struct uart_driver_api uart_samr21_driver_api;

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_samr21_init(struct device *dev)
{
    volatile SercomUsart *uart = UART_STRUCT(dev);
    const struct uart_device_config *const cfg = DEV_CFG(dev);
    struct uart_samr21_dev_data_t *const data = DEV_DATA(dev);

    /* calculate baudrate */
    uint32_t baud = (( (cfg->sys_clk_freq * 10) / data->baud_rate) / 16);

    /* disable interrupts */
	int old_level = irq_lock();

    /* enable sync and async clocks */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM0_CORE);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* configure pins */
    set_pin_func(EDBG_COM_TX, FUNCD);
    set_pin_func(EDBG_COM_RX, FUNCD);

    /* reset the UART device */
    uart->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (uart->SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST) { }

    /* set asynchronous mode w/o parity, LSB first, TX and RX pad as specified
     * by the board in the periph_conf.h, x16 sampling and use internal clock */
    uart->CTRLA.reg = (SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_SAMPR(0x1)
            | SERCOM_USART_CTRLA_TXPO(0)
            | SERCOM_USART_CTRLA_RXPO(1)
            | SERCOM_USART_CTRLA_MODE_USART_INT_CLK);

    /* set baudrate */
    uart->BAUD.FRAC.FP = (baud % 10);
    uart->BAUD.FRAC.BAUD = (baud / 10);

    /* enable receiver and transmitter, use 1 stop bit */
    uart->CTRLB.reg = (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);
    while (uart->SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB)
    {
    }

    /* finally, enable the device */
    uart->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;

    irq_unlock(old_level);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config_func(dev);
#endif

    return 0;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */

static int uart_samr21_poll_in(struct device *dev, unsigned char *c)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	if (!(uart->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC))
	        return (-1);

	*c = (unsigned char)uart->DATA.reg;

    return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_samr21_poll_out(struct device *dev, unsigned char c)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	while (!(uart->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE))
	    ;
	uart->DATA.reg = (uint32_t)c;

    return c;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param len Number of bytes to send
 *
 * @return number of bytes sent
 */
static int uart_samr21_fifo_fill(struct device *dev, const uint8_t *tx_data,
			      int len)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);
	uint8_t num_tx = 0;

	while ((len - num_tx > 0) && ((uart->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == SERCOM_USART_INTFLAG_DRE)) {
		uart->DATA.reg = tx_data[num_tx++];
	}

	return num_tx;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rx_data Pointer to data container
 * @param size Container size in bytes
 *
 * @return number of bytes read
 */
static int uart_samr21_fifo_read(struct device *dev, uint8_t *rx_data,
			      const int size)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);
	uint8_t num_rx = 0;

	while ((size - num_rx > 0) && ((uart->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) == SERCOM_USART_INTFLAG_RXC)) {
		rx_data[num_rx++] = uart->DATA.reg;
	}

	return num_rx;
}

/**
 * @brief Enable TX interrupt
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_samr21_irq_tx_enable(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	uart->INTENSET.bit.TXC = 1;
}

/**
 * @brief Disable TX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_samr21_irq_tx_disable(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	uart->INTENCLR.bit.TXC = 1;
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_samr21_irq_tx_ready(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	return ((uart->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == SERCOM_USART_INTFLAG_DRE);
}

/**
 * @brief Enable RX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_samr21_irq_rx_enable(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	uart->INTENSET.bit.RXC = 1;
}

/**
 * @brief Disable RX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_samr21_irq_rx_disable(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	uart->INTENCLR.bit.RXC = 1;
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_samr21_irq_rx_ready(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	return ((uart->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) == SERCOM_USART_INTFLAG_RXC);
}

/**
 * @brief Enable error interrupt
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_samr21_irq_err_enable(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	uart->INTENSET.bit.ERROR = 1;
}

/**
 * @brief Disable error interrupt
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_samr21_irq_err_disable(struct device *dev)
{
	volatile SercomUsart *uart = UART_STRUCT(dev);

	uart->INTENCLR.bit.ERROR = 1;
}

/**
 * @brief Check if Tx or Rx IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if a Tx or Rx IRQ is pending, 0 otherwise
 */
static int uart_samr21_irq_is_pending(struct device *dev)
{

	return uart_samr21_irq_tx_ready(dev) || uart_samr21_irq_rx_ready(dev);
}

/**
 * @brief Update IRQ status
 *
 * @param dev UART device struct
 *
 * @return always 1
 */
static int uart_samr21_irq_update(struct device *dev)
{
	return 1;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 *
 * @return N/A
 */
static void uart_samr21_irq_callback_set(struct device *dev,
				      uart_irq_callback_t cb)
{
	struct uart_samr21_dev_data_t * const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
void uart_samr21_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_samr21_dev_data_t * const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_samr21_driver_api = {
	.poll_in = uart_samr21_poll_in,
	.poll_out = uart_samr21_poll_out,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

	.fifo_fill = uart_samr21_fifo_fill,
	.fifo_read = uart_samr21_fifo_read,
	.irq_tx_enable = uart_samr21_irq_tx_enable,
	.irq_tx_disable = uart_samr21_irq_tx_disable,
	.irq_tx_ready = uart_samr21_irq_tx_ready,
	.irq_rx_enable = uart_samr21_irq_rx_enable,
	.irq_rx_disable = uart_samr21_irq_rx_disable,
	.irq_rx_ready = uart_samr21_irq_rx_ready,
	.irq_err_enable = uart_samr21_irq_err_enable,
	.irq_err_disable = uart_samr21_irq_err_disable,
	.irq_is_pending = uart_samr21_irq_is_pending,
	.irq_update = uart_samr21_irq_update,
	.irq_callback_set = uart_samr21_irq_callback_set,

#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_0(struct device *dev);
#endif

static const struct uart_device_config uart_samr21_dev_cfg_0 = {
    .base = (uint8_t *)SERCOM0,
    .sys_clk_freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = irq_config_func_0,
#endif
};

static struct uart_samr21_dev_data_t uart_samr21_dev_data_0 = {
    .baud_rate = CONFIG_UART_ATMEL_SAMR21_BAUD_RATE,
};

DEVICE_AND_API_INIT(uart_samr21_0, CONFIG_UART_ATMEL_SAMR21_NAME, &uart_samr21_init,
		    &uart_samr21_dev_data_0, &uart_samr21_dev_cfg_0,
			PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_samr21_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_0(struct device *dev)
{
	IRQ_CONNECT(IRQ_SERCOM0,
			CONFIG_UART_ATMEL_SAMR21_IRQ_PRI,
		    uart_samr21_isr, DEVICE_GET(uart_samr21_0),
		    UART_IRQ_FLAGS);
	irq_enable(IRQ_SERCOM0);
}
#endif
