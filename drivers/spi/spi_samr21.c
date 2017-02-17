/*
 * spi_samr21.c
 *
 *  Created on: 2017年1月14日
 *      Author: ldc
 */

#include <errno.h>

#include <device.h>
#include <spi.h>
#include <gpio.h>
#include <soc.h>
#include <board.h>

#define SPIACTIVE 	0
#define SPIDEACTIVE 1

/* convenience defines */
#define DEV_CFG(dev) \
	((struct spi_samr21_config *)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct spi_samr21_data *)(dev)->driver_data)
#define SPI_STRUCT(dev) \
	((volatile SercomSpi *)(DEV_CFG(dev))->base)

typedef void (*spi_samr21_config_t)(void);
typedef void (*pin_set_t)(struct device *);
typedef void (*select_set_t)(struct device *, uint8_t);

struct spi_samr21_config {
	uint8_t *base;					/* base address of SPI module registers */
	spi_samr21_config_t config_func;	/* IRQ configuration function pointer */
	// uint32_t irq;					/* SPI module IRQ number */
	uint32_t id;					/* SPI indent */
	uint32_t baudrate;				/* SPI baud rate */
	uint16_t mux_group;				/* see DS: SPI CTRLA DOPO*/
	uint16_t mux_pad0;				/* see DS: SPI CTRLA DIPO */
	uint16_t pad0_pin;
	uint16_t pad1_pin;
	uint16_t pad2_pin;
	uint16_t pad3_pin;

	pin_set_t pinmuxset;
	select_set_t select_cs;
};

struct spi_samr21_data {
	uint8_t frame_sz;				/* frame/word size, in bits */
	uint8_t cont_pcs_sel;			/* continuous slave/PCS selection enable */
	uint8_t pcs;					/* slave/PCS selection */
	const uint8_t *tx_buf;
	uint32_t tx_cnt;
	uint32_t tx_buf_len;
	uint8_t *rx_buf;
	uint32_t rx_cnt;
	uint32_t rx_buf_len;
	uint32_t xfer_len;
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	uint32_t device_power_state;
#endif
#ifdef CONFIG_SPI_CS_GPIO
	struct device *port_dev;
	uint16_t cs_portpin;
#endif
};

static void pinmuxset_default(struct device *dev)
{
	struct spi_samr21_config *cfg = DEV_CFG(dev);
	volatile SercomSpi *spi = SPI_STRUCT(dev);

    set_pin_func(GPIOPORTPIN(cfg->pad0_pin), GPIOFUNC(cfg->pad0_pin));
	set_pin_func(GPIOPORTPIN(cfg->pad1_pin), GPIOFUNC(cfg->pad1_pin));
	set_pin_func(GPIOPORTPIN(cfg->pad2_pin), GPIOFUNC(cfg->pad2_pin));
    set_pin_func(GPIOPORTPIN(cfg->pad3_pin), GPIOFUNC(cfg->pad3_pin));

    spi->CTRLB.reg |= SERCOM_SPI_CTRLB_MSSEN;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_CTRLB) { }
}

static void select_cs_default(struct device *dev, uint8_t val)
{
	return;
}

#if defined(CONFIG_SPI_CS_GPIO)
static void pinmuxset_port(struct device *dev)
{
	struct spi_samr21_config *cfg = DEV_CFG(dev);
	struct spi_samr21_data *samr21_data = DEV_DATA(dev);

	set_pin_func(GPIOPORTPIN(cfg->pad0_pin), GPIOFUNC(cfg->pad0_pin));

    if ((cfg->mux_group == 0) || (cfg->mux_group == 2))
    {
    	set_pin_func(GPIOPORTPIN(cfg->pad1_pin), GPIOFUNC(cfg->pad1_pin));
    }
    else if ((cfg->mux_group == 1) || (cfg->mux_group == 3))
    {
    	set_pin_func(GPIOPORTPIN(cfg->pad2_pin), GPIOFUNC(cfg->pad2_pin));
    }

    // set pin output
    samr21_data->port_dev = device_get_binding(CONFIG_GPIO_ATMEL_SAMR21_PORT_DEV_NAME);
    gpio_pin_configure(samr21_data->port_dev, samr21_data->cs_portpin, GPIO_DIR_OUT);

    set_pin_func(GPIOPORTPIN(cfg->pad3_pin), GPIOFUNC(cfg->pad3_pin));
}

static void select_cs_port(struct device *dev, uint8_t val)
{
	struct spi_samr21_data *samr21_data = DEV_DATA(dev);

	gpio_pin_write(samr21_data->port_dev, samr21_data->cs_portpin, val);
}

#endif

/**
 * @brief Configure the SPI host controller for operating against slaves
 * @param dev Pointer to the device structure for the driver instance
 * @param config Pointer to the application provided configuration
 *
 * @return 0 if successful, another DEV_* code otherwise.
 */
static int spi_samr21_configure(struct device *dev, struct spi_config *config)
{
	uint32_t cfg;
	volatile SercomSpi *spi = SPI_STRUCT(dev);

	spi->CTRLA.reg &= (~SERCOM_SPI_CTRLA_ENABLE);
	while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }

	spi->BAUD.reg = ((float)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / (float)(2 * config->max_sys_freq)) - 1;
	cfg = spi->CTRLA.reg;

	/*
	 * DS p487
	 *  mode          CPOL          CPHA          Leading Edge          Trailing Edge
	 *  0x0           0             0            Rising,sample         Failing,change
	 *  0x1           0             1            Rising,change         Failing,sample
	 *  0x2           1             0            Failing,sample        Rising,change
	 *  0x3           1             1            Failing,change        Rising,sample
	 */
	if ((config->config & SPI_MODE_CPOL) == SPI_MODE_CPOL)
	{
		cfg &= ~SERCOM_SPI_CTRLA_CPOL;	/* low when idle */
	} else {
		cfg |= SERCOM_SPI_CTRLA_CPOL;	/* high when idle */
	}
	if ((config->config & SPI_MODE_CPHA) == SPI_MODE_CPHA)
	{
		cfg &= ~SERCOM_SPI_CTRLA_CPHA;	/* sample on leading edge */
	} else {
		cfg |= SERCOM_SPI_CTRLA_CPHA;	/* sample on trailing edge */
	}
	if ((config->config & SPI_TRANSFER_MSB) == SPI_TRANSFER_MSB)
	{
		cfg &= ~SERCOM_SPI_CTRLA_DORD;
	} else {
		cfg |= SERCOM_SPI_CTRLA_DORD;
	}

	cfg |= SERCOM_SPI_CTRLA_ENABLE;

	spi->CTRLA.reg = cfg;
	while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }

	return 0;
}

/** Check interrupt flag of RXC and update transaction runtime information. */
static inline void _spi_rx_check_and_receive(volatile SercomSpi *spi, struct spi_samr21_data *samr21_data)
{
	uint32_t data;

	while (!(spi->INTFLAG.reg & SERCOM_SPI_INTFLAG_RXC)) { }

	data = spi->DATA.reg;

	if (samr21_data->rx_cnt < samr21_data->rx_buf_len)
	{
		*samr21_data->rx_buf++ = (uint8_t)data;
	}

	samr21_data->rx_cnt++;
}

/** Check interrupt flag of DRE and update transaction runtime information. */
static inline void _spi_tx_check_and_send(volatile SercomSpi *spi, struct spi_samr21_data *samr21_data)
{
	uint32_t data;

	while (!(spi->INTFLAG.reg & SERCOM_SPI_INTFLAG_DRE)) { }

	if (samr21_data->tx_cnt < samr21_data->tx_buf_len) {
		data = *samr21_data->tx_buf++;
	} else {
		data = 0x0;
	}

	samr21_data->tx_cnt++;
	spi->DATA.reg = data;
}

/** Check interrupt flag of ERROR and update transaction runtime information. */
static inline int32_t _spi_err_check(volatile SercomSpi *spi)
{
	if (spi->INTFLAG.reg & SERCOM_SPI_INTFLAG_ERROR)
	{
		spi->STATUS.reg = ~0;
		spi->INTFLAG.reg = SERCOM_SPI_INTFLAG_ERROR;
		return -EIO;
	}

	return 0;
}

/**
 * @brief Read and/or write a defined amount of data through an SPI driver
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param tx_buf Memory buffer that data should be transferred from
 * @param tx_buf_len Size of the memory buffer available for reading from
 * @param rx_buf Memory buffer that data should be transferred to
 * @param rx_buf_len Size of the memory buffer available for writing to
 *
 * @return 0 if successful, another DEV_* code otherwise.
 */
static int spi_samr21_transceive(struct device *dev,
				const void *tx_buf, uint32_t tx_buf_len,
				void *rx_buf, uint32_t rx_buf_len)
{
	volatile SercomSpi *spi = SPI_STRUCT(dev);
	struct spi_samr21_data *samr21_data = DEV_DATA(dev);
	struct spi_samr21_config *cfg = DEV_CFG(dev);
	int rc;

	samr21_data->tx_cnt = samr21_data->rx_cnt = 0;
	samr21_data->rx_buf = rx_buf;
	samr21_data->tx_buf = tx_buf;
	samr21_data->tx_buf_len = tx_buf_len;
	samr21_data->rx_buf_len = rx_buf_len;
	if (tx_buf_len > rx_buf_len)
	{
		samr21_data->xfer_len = tx_buf_len;
	}
	else
	{
		samr21_data->xfer_len = rx_buf_len;
	}

	cfg->select_cs(dev, SPIACTIVE);

	/* If settings are not applied (pending), we can not go on */
	if (spi->SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE | SERCOM_SPI_SYNCBUSY_CTRLB))
	{
		return -EBUSY;
	}

	if (!spi->CTRLA.bit.ENABLE)
	{
		return -EIO;
	}

	for (int i = 0; i < samr21_data->xfer_len; i++)
	{
		_spi_tx_check_and_send(spi, samr21_data);

		_spi_rx_check_and_receive(spi, samr21_data);

		rc = _spi_err_check(spi);
		if (rc < 0)
		{
			break;
		}
	}

	/* Wait until SPI bus idle */
	while (! (spi->INTFLAG.reg & (SERCOM_SPI_INTFLAG_TXC | SERCOM_SPI_INTFLAG_DRE))) { }
	spi->INTFLAG.reg |= SERCOM_SPI_INTFLAG_TXC | SERCOM_SPI_INTFLAG_DRE;

	cfg->select_cs(dev, SPIDEACTIVE);

	return rc != 0 ? rc : 0;
}

static const struct spi_driver_api samr21_spi_api = {
	.configure = spi_samr21_configure,
	.transceive = spi_samr21_transceive,
	.slave_select = NULL,
};

int spi_samr21_init(struct device *dev)
{
	volatile SercomSpi *spi = SPI_STRUCT(dev);
	struct spi_samr21_data *samr21_data = DEV_DATA(dev);
	struct spi_samr21_config *cfg = DEV_CFG(dev);
	uint8_t id = cfg->id;

	(void)samr21_data;

	int old_level = irq_lock();

    /* enable sync and async clocks */
    PM->APBCMASK.reg |= 1U << (PM_APBCMASK_SERCOM0_Pos + id);
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
    				GCLK_CLKCTRL_GEN_GCLK0 |
					((GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + id) << GCLK_CLKCTRL_ID_Pos);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* reset the Sercom device */
    spi->CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
    while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_SWRST) { }

    /* set master mode */
    spi->CTRLA.reg = SERCOM_SPI_CTRLA_DIPO(cfg->mux_pad0) |
    				SERCOM_SPI_CTRLA_DOPO(cfg->mux_group) |
					SERCOM_SPI_CTRLA_MODE_SPI_MASTER;
    spi->CTRLB.reg &= (~(SERCOM_SPI_CTRLB_MSSEN | SERCOM_SPI_CTRLB_AMODE_Msk | SERCOM_SPI_CTRLB_SSDE | SERCOM_SPI_CTRLB_PLOADEN)) | SERCOM_SPI_CTRLB_RXEN;
	while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_CTRLB) { }

    spi->BAUD.reg = ((float)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / (float)(2 * cfg->baudrate)) - 1;
    spi->DBGCTRL.reg = 0;

    /* configure pins */
    cfg->pinmuxset(dev);

	cfg->config_func();

	irq_unlock(old_level);

	cfg->select_cs(dev, SPIDEACTIVE);

	/* finally, enable the device */
	spi->CTRLB.reg |= SERCOM_SPI_CTRLB_RXEN;
	while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_CTRLB) { }
	spi->CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	while (spi->SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_ENABLE) { }

#if 0
	irq_enable(cfg->irq);
#endif

	return 0;
}

#ifdef CONFIG_SPI_0

void spi_config_0_cfg(void);

struct spi_samr21_data spi_samr21_data_port_0 = {
#ifdef CONFIG_SPI_CS_GPIO
	.cs_portpin = GPIOGROUP(CONFIG_AT_SPI_0_CS_PORT, CONFIG_AT_SPI_0_CS_PIN),
	.port_dev = NULL,
#endif
};

static const struct spi_samr21_config spi_samr21_config_0 = {
	.base = (uint8_t *)SERCOM0,
	.config_func = spi_config_0_cfg,
	//.irq = IRQ_SERCOM0,
	.id = 0,
	.baudrate = CONFIG_SPI_0_BAUDRATE,
	.mux_group = CONFIG_SPI_0_PORT_GROUP,
	.mux_pad0 = CONFIG_SPI_0_PORT_GROUP_DIPIN,
	.pad0_pin = CONFIG_SPI_0_PAD0_PORTPIN,
	.pad1_pin = CONFIG_SPI_0_PAD1_PORTPIN,
	.pad2_pin = CONFIG_SPI_0_PAD2_PORTPIN,
	.pad3_pin = CONFIG_SPI_0_PAD3_PORTPIN,
#if defined(CONFIG_SPI_CS_GPIO) && defined(CONFIG_AT_SPI_0_CS_GPIO)
	.pinmuxset = pinmuxset_port,
	.select_cs = select_cs_port,
#else
	.pinmuxset = pinmuxset_default,
	.select_cs = select_cs_default,
#endif
};

DEVICE_AND_API_INIT(spi_samr21_port_0, CONFIG_SPI_0_NAME, spi_samr21_init,
	      &spi_samr21_data_port_0, &spi_samr21_config_0, PRE_KERNEL_1,
	      CONFIG_SPI_INIT_PRIORITY, &samr21_spi_api);

void spi_config_0_cfg(void)
{
#if 0
	IRQ_CONNECT(IRQ_SERCOM0, CONFIG_SPI_0_IRQ_PRI,
		spi_samr21_isr, DEVICE_GET(spi_samr21_port_0), 0);
#endif
}

#endif

#ifdef CONFIG_SPI_1

void spi_config_1_cfg(void);

struct spi_samr21_data spi_samr21_data_port_1 = {
#ifdef CONFIG_SPI_CS_GPIO
	.cs_portpin = GPIOGROUP(CONFIG_AT_SPI_1_CS_PORT, CONFIG_AT_SPI_1_CS_PIN),
	.port_dev = NULL,
#endif
};

static const struct spi_samr21_config spi_samr21_config_1 = {
	.base = (uint8_t *)SERCOM1,
	.config_func = spi_config_1_cfg,
	//.irq = IRQ_SERCOM1,
	.id = 1,
	.baudrate = CONFIG_SPI_1_BAUDRATE,
	.mux_group = CONFIG_SPI_1_PORT_GROUP,
	.mux_pad0 = CONFIG_SPI_1_PORT_GROUP_DIPIN,
	.pad0_pin = CONFIG_SPI_1_PAD0_PORTPIN,
	.pad1_pin = CONFIG_SPI_1_PAD1_PORTPIN,
	.pad2_pin = CONFIG_SPI_1_PAD2_PORTPIN,
	.pad3_pin = CONFIG_SPI_1_PAD3_PORTPIN,
#if defined(CONFIG_SPI_CS_GPIO) && defined(CONFIG_AT_SPI_1_CS_GPIO)
	.pinmuxset = pinmuxset_port,
	.select_cs = select_cs_port,
#else
	.pinmuxset = pinmuxset_default,
	.select_cs = select_cs_default,
#endif
};

DEVICE_AND_API_INIT(spi_samr21_port_1, CONFIG_SPI_1_NAME, spi_samr21_init,
	      &spi_samr21_data_port_1, &spi_samr21_config_1, PRE_KERNEL_1,
	      CONFIG_SPI_INIT_PRIORITY, &samr21_spi_api);

void spi_config_1_cfg(void)
{
#if 0
	IRQ_CONNECT(IRQ_SERCOM1, CONFIG_SPI_1_IRQ_PRI,
		spi_samr21_isr, DEVICE_GET(spi_samr21_port_1), 0);
#endif
}

#endif

#ifdef CONFIG_SPI_2

void spi_config_2_cfg(void);

struct spi_samr21_data spi_samr21_data_port_2 = {
#ifdef CONFIG_SPI_CS_GPIO
	.cs_portpin = GPIOGROUP(CONFIG_AT_SPI_2_CS_PORT, CONFIG_AT_SPI_2_CS_PIN),
	.port_dev = NULL,
#endif
};

static const struct spi_samr21_config spi_samr21_config_2 = {
	.base = (uint8_t *)SERCOM2,
	.config_func = spi_config_2_cfg,
	//.irq = IRQ_SERCOM2,
	.id = 2,
	.baudrate = CONFIG_SPI_2_BAUDRATE,
	.mux_group = CONFIG_SPI_2_PORT_GROUP,
	.mux_pad0 = CONFIG_SPI_2_PORT_GROUP_DIPIN,
	.pad0_pin = CONFIG_SPI_2_PAD0_PORTPIN,
	.pad1_pin = CONFIG_SPI_2_PAD1_PORTPIN,
	.pad2_pin = CONFIG_SPI_2_PAD2_PORTPIN,
	.pad3_pin = CONFIG_SPI_2_PAD3_PORTPIN,
#if defined(CONFIG_SPI_CS_GPIO) && defined(CONFIG_AT_SPI_2_CS_GPIO)
	.pinmuxset = pinmuxset_port,
	.select_cs = select_cs_port,
#else
	.pinmuxset = pinmuxset_default,
	.select_cs = select_cs_default,
#endif
};

DEVICE_AND_API_INIT(spi_samr21_port_2, CONFIG_SPI_2_NAME, spi_samr21_init,
	      &spi_samr21_data_port_2, &spi_samr21_config_2, PRE_KERNEL_1,
	      CONFIG_SPI_INIT_PRIORITY, &samr21_spi_api);

void spi_config_2_cfg(void)
{
#if 0
	IRQ_CONNECT(IRQ_SERCOM2, CONFIG_SPI_2_IRQ_PRI,
		spi_samr21_isr, DEVICE_GET(spi_samr21_port_2), 0);
#endif
}

#endif

#ifdef CONFIG_SPI_3

void spi_config_3_cfg(void);

struct spi_samr21_data spi_samr21_data_port_3 = {
#ifdef CONFIG_SPI_CS_GPIO
	.cs_portpin = GPIOGROUP(CONFIG_AT_SPI_3_CS_PORT, CONFIG_AT_SPI_3_CS_PIN),
	.port_dev = NULL,
#endif
};

static const struct spi_samr21_config spi_samr21_config_3 = {
	.base = (uint8_t *)SERCOM3,
	.config_func = spi_config_3_cfg,
	//.irq = IRQ_SERCOM3,
	.id = 3,
	.baudrate = CONFIG_SPI_3_BAUDRATE,
	.mux_group = CONFIG_SPI_3_PORT_GROUP,
	.mux_pad0 = CONFIG_SPI_3_PORT_GROUP_DIPIN,
	.pad0_pin = CONFIG_SPI_3_PAD0_PORTPIN,
	.pad1_pin = CONFIG_SPI_3_PAD1_PORTPIN,
	.pad2_pin = CONFIG_SPI_3_PAD2_PORTPIN,
	.pad3_pin = CONFIG_SPI_3_PAD3_PORTPIN,
#if defined(CONFIG_SPI_CS_GPIO) && defined(CONFIG_AT_SPI_3_CS_GPIO)
	.pinmuxset = pinmuxset_port,
	.select_cs = select_cs_port,
#else
	.pinmuxset = pinmuxset_default,
	.select_cs = select_cs_default,
#endif
};

DEVICE_AND_API_INIT(spi_samr21_port_3, CONFIG_SPI_3_NAME, spi_samr21_init,
	      &spi_samr21_data_port_3, &spi_samr21_config_3, PRE_KERNEL_1,
	      CONFIG_SPI_INIT_PRIORITY, &samr21_spi_api);

void spi_config_3_cfg(void)
{
#if 0
	IRQ_CONNECT(IRQ_SERCOM3, CONFIG_SPI_3_IRQ_PRI,
		spi_samr21_isr, DEVICE_GET(spi_samr21_port_3), 0);
#endif
}

#endif

#ifdef CONFIG_SPI_4

void spi_config_4_cfg(void);

struct spi_samr21_data spi_samr21_data_port_4 = {
#ifdef CONFIG_SPI_CS_GPIO
	.cs_portpin = GPIOGROUP(CONFIG_AT_SPI_4_CS_PORT, CONFIG_AT_SPI_4_CS_PIN),
	.port_dev = NULL,
#endif
};

static const struct spi_samr21_config spi_samr21_config_4 = {
	.base = (uint8_t *)SERCOM4,
	.config_func = spi_config_4_cfg,
	//.irq = IRQ_SERCOM4,
	.id = 4,
	.baudrate = CONFIG_SPI_4_BAUDRATE,
	.mux_group = CONFIG_SPI_4_PORT_GROUP,
	.mux_pad0 = CONFIG_SPI_4_PORT_GROUP_DIPIN,
	.pad0_pin = CONFIG_SPI_4_PAD0_PORTPIN,
	.pad1_pin = CONFIG_SPI_4_PAD1_PORTPIN,
	.pad2_pin = CONFIG_SPI_4_PAD2_PORTPIN,
	.pad3_pin = CONFIG_SPI_4_PAD3_PORTPIN,
#if defined(CONFIG_SPI_CS_GPIO) && defined(CONFIG_AT_SPI_4_CS_GPIO)
	.pinmuxset = pinmuxset_port,
	.select_cs = select_cs_port,
#else
	.pinmuxset = pinmuxset_default,
	.select_cs = select_cs_default,
#endif
};

DEVICE_AND_API_INIT(spi_samr21_port_4, CONFIG_SPI_4_NAME, spi_samr21_init,
	      &spi_samr21_data_port_4, &spi_samr21_config_4, PRE_KERNEL_1,
	      CONFIG_SPI_INIT_PRIORITY, &samr21_spi_api);

void spi_config_4_cfg(void)
{
#if 0
	IRQ_CONNECT(IRQ_SERCOM4, CONFIG_SPI_4_IRQ_PRI,
		spi_samr21_isr, DEVICE_GET(spi_samr21_port_4), 0);
#endif
}

#endif

#ifdef CONFIG_SPI_5

void spi_config_5_cfg(void);

struct spi_samr21_data spi_samr21_data_port_5 = {
#ifdef CONFIG_SPI_CS_GPIO
	.cs_portpin = GPIOGROUP(CONFIG_AT_SPI_5_CS_PORT, CONFIG_AT_SPI_5_CS_PIN),
	.port_dev = NULL,
#endif
};

static const struct spi_samr21_config spi_samr21_config_5 = {
	.base = (uint8_t *)SERCOM5,
	.config_func = spi_config_5_cfg,
	//.irq = IRQ_SERCOM5,
	.id = 5,
	.baudrate = CONFIG_SPI_5_BAUDRATE,
	.mux_group = CONFIG_SPI_5_PORT_GROUP,
	.mux_pad0 = CONFIG_SPI_5_PORT_GROUP_DIPIN,
	.pad0_pin = CONFIG_SPI_5_PAD0_PORTPIN,
	.pad1_pin = CONFIG_SPI_5_PAD1_PORTPIN,
	.pad2_pin = CONFIG_SPI_5_PAD2_PORTPIN,
	.pad3_pin = CONFIG_SPI_5_PAD3_PORTPIN,
#if defined(CONFIG_SPI_CS_GPIO) && defined(CONFIG_AT_SPI_5_CS_GPIO)
	.pinmuxset = pinmuxset_port,
	.select_cs = select_cs_port,
#else
	.pinmuxset = pinmuxset_default,
	.select_cs = select_cs_default,
#endif
};

DEVICE_AND_API_INIT(spi_samr21_port_5, CONFIG_SPI_5_NAME, spi_samr21_init,
	      &spi_samr21_data_port_5, &spi_samr21_config_5, PRE_KERNEL_1,
	      CONFIG_SPI_INIT_PRIORITY, &samr21_spi_api);

void spi_config_5_cfg(void)
{
#if 0
	IRQ_CONNECT(IRQ_SERCOM5, CONFIG_SPI_5_IRQ_PRI,
		spi_samr21_isr, DEVICE_GET(spi_samr21_port_5), 0);
#endif
}

#endif
