/*
 * ieee802154_at86rf233.c
 *
 *  Created on: 2017年1月14日
 *      Author: ldc
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_IEEE802154_DRIVER_LEVEL
#define SYS_LOG_DOMAIN "dev/at86rf233"
#include <logging/sys_log.h>

#include <errno.h>

#include <kernel.h>
#include <arch/cpu.h>

#include <board.h>
#include <device.h>
#include <init.h>
#include <net/net_if.h>
#include <net/nbuf.h>

#include <misc/byteorder.h>
#include <string.h>
#include <rand32.h>

#include <gpio.h>
#include <spi.h>

#include <net/ieee802154_radio.h>

#include "ieee802154_at86rf233.h"

/* AUTOACK should be enabled by default, disable it only for testing */
#define AT86RF233_AUTOACK_ENABLED		(true)

#define AT86RF233_FCS_LENGTH		(2)
#define AT86RF233_PSDU_LENGTH		(125)
#define AT86RF233_WAKEUP_DELAY 		(1000)

enum {LOW = 0, HIGH = 1};

/* convenience defines */
#define DEV_CTX(dev) \
	((struct at86rf233_context *)(dev)->driver_data)

#define _usleep(usec) k_busy_wait(usec)

static inline void set_reset(struct device *dev, uint32_t value)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	gpio_pin_write(at86rf233->gpio, RF233RSTPIN, value);
}

static uint8_t at86rf233_read_reg(struct at86rf233_spi *spi, uint8_t addr)
{
	uint8_t len = 2;

	spi->cmd_buf[0] = RF_CMD_REG_R | addr;

	spi_slave_select(spi->dev, spi->slave);

	if (spi_transceive(spi->dev, spi->cmd_buf, len - 1, spi->cmd_buf, len) == 0) {
		return spi->cmd_buf[len - 1];
	}

	return 0;
}

static bool at86rf233_write_reg(struct at86rf233_spi *spi, uint8_t addr, uint8_t value)
{
	uint8_t len = 2;

	spi->cmd_buf[0] = RF_CMD_REG_W | addr;
	spi->cmd_buf[1] = value;

	spi_slave_select(spi->dev, spi->slave);

	return (spi_write(spi->dev, spi->cmd_buf, len) == 0);
}

bool at86rf233_sram_write_raw_frame(struct at86rf233_spi *spi, uint8_t *data, uint8_t len)
{
    uint8_t cmd[3 + AT86RF233_PSDU_LENGTH];

	if (len > AT86RF233_PSDU_LENGTH)
	{
		SYS_LOG_ERR("Payload too long");
		return false;
	}

    cmd[0] = RF_CMD_SRAM_W;
    cmd[1] = 0x0;
    cmd[2] = len;
    memcpy(&cmd[3], data, len);

    spi_slave_select(spi->dev, spi->slave);
    return (spi_write(spi->dev, cmd, (3 + len)) == 0);
}

bool at86rf233_sram_read(struct at86rf233_spi *spi, uint8_t offset, uint8_t *data, uint8_t len)
{
	uint8_t rx_buf[2 + AT86RF233_PSDU_LENGTH];

	if (len > AT86RF233_PSDU_LENGTH) {
		SYS_LOG_ERR("Payload too long");
		return false;
	}

    spi->cmd_buf[0] = RF_CMD_SRAM_R;
    spi->cmd_buf[1] = offset;

    spi_slave_select(spi->dev, spi->slave);
    spi_transceive(spi->dev, spi->cmd_buf, 2, rx_buf, len + 2);

    memcpy(data, &rx_buf[2], len);

    return true;
}

static uint8_t at86rf233_status(struct at86rf233_spi *spi)
{
	return (at86rf233_read_reg(spi, TRX_STATUS_REG) & TRX_STATUS_MASK);
}

static void at86rf233_reset_state_machine(struct device *dev)
{
    uint8_t old_state;
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

    /* Wait for any state transitions to complete before forcing TRX_OFF */
    do {
        old_state = at86rf233_status(spi);
    } while (old_state == TRX_STATUS_STATE_TRANSITION);

    at86rf233_write_reg(spi, TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
}

static uint8_t at86rf233_get_lqi(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	return at86rf233->lqi;
}

static int at86rf233_cca(struct device *dev)
{
	uint8_t ccareg;
	bool off = false;

	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	if (at86rf233_status(spi) != TRX_STATUS_RX_ON)
	{
		off = true;
		at86rf233_write_reg(spi, TRX_STATE_REG, TRX_CMD_RX_ON);
		_usleep(200);
	}

	ccareg = at86rf233_read_reg(spi, PHY_CC_CCA_REG);
	ccareg |= PHY_CC_CCA_DO_CCA | PHY_CC_CCA_MODE_CS_OR_ED;
	at86rf233_write_reg(spi, PHY_CC_CCA_REG, ccareg);

	do
	{
		ccareg = at86rf233_read_reg(spi, TRX_STATUS_REG);
	} while ((ccareg & CCA_DONE) == 0);

	/* return to previous state */
	if (off == true)
	{
		at86rf233_write_reg(spi, TRX_STATE_REG, TRX_CMD_TRX_OFF);
	}

	/* check CCA */
	if ((ccareg & CCA_DONE) && (ccareg & CCA_STATUS))
	{
		return 0;
	}

	return -EBUSY;
}

static int at86rf233_set_channel(struct device *dev, uint16_t channel)
{
	uint8_t cc;
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	if (channel < 11 || channel > 26)
	{
		SYS_LOG_ERR("Unsupported channel");
		return -EINVAL;
	}

	cc = at86rf233_read_reg(spi, PHY_CC_CCA_REG);
	cc &= ~(0x1F);
	cc |= channel;
	at86rf233_write_reg(spi, PHY_CC_CCA_REG, cc);

	return 0;
}

static int at86rf233_set_pan_id(struct device *dev, uint16_t pan_id)
{
	uint8_t *p;
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	pan_id = sys_le16_to_cpu(pan_id);
	p = (uint8_t *)&pan_id;

	at86rf233_write_reg(spi, PAN_ID_0_REG, p[0]);
	at86rf233_write_reg(spi, PAN_ID_1_REG, p[1]);

	return 0;
}

static int at86rf233_set_short_addr(struct device *dev, uint16_t short_addr)
{
	uint8_t *a;
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	short_addr = sys_le16_to_cpu(short_addr);
	a = (uint8_t *)&short_addr;

	at86rf233_write_reg(spi, SHORT_ADDR_0_REG, a[0]);
	at86rf233_write_reg(spi, SHORT_ADDR_1_REG, a[1]);

	return 0;
}

static int at86rf233_set_ieee_addr(struct device *dev, const uint8_t *ieee_addr)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	for (int i = 0; i < 8; i++)
	{
		at86rf233_write_reg(spi, IEEE_ADDR_0_REG + i, ieee_addr[i]);
	}

	return 0;
}

static int at86rf233_set_txpower(struct device *dev, int16_t dbm)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	uint8_t val;

	switch (dbm)
	{
	case 4:
		val = 0x0;
		break;
	case 3:
		val = 0x3;
		break;
	case 2:
		val = 0x5;
		break;
	case 1:
		val = 0x6;
		break;
	case 0:
		val = 0x7;
		break;
	case -1:
		val = 0x8;
		break;
	case -2:
		val = 0x9;
		break;
	case -3:
		val = 0xa;
		break;
	case -4:
		val = 0xb;
		break;
	case -6:
		val = 0xc;
		break;
	case -8:
		val = 0xd;
		break;
	case -12:
		val = 0xe;
		break;
	case -17:
		val = 0xf;
		break;
	default:
		{
			SYS_LOG_DBG("Failed");
			return -EIO;
		}
	}
	at86rf233_write_reg(spi, PHY_TX_PWR_REG, val);

	return 0;
}

static int at86rf233_tx(struct device *dev,
		    struct net_buf *buf,
		    struct net_buf *frag)
{
	uint8_t state;
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	uint8_t *frame = frag->data - net_nbuf_ll_reserve(buf);
	uint8_t len = net_nbuf_ll_reserve(buf) + frag->len;

	SYS_LOG_DBG("%p (%u)", frag, len);

    /* make sure ongoing transmissions are finished */
    do
    {
        state = at86rf233_status(spi);
    } while (state == TRX_STATUS_BUSY_RX_AACK || state == TRX_STATUS_BUSY_TX_ARET);

    atomic_set(&at86rf233->tx, 1);

    at86rf233_write_reg(spi, TRX_STATE_REG, TRX_CMD_TX_ARET_ON);

    at86rf233_sram_write_raw_frame(spi, frame, len);

    /* trigger sending of pre-loaded frame */
    at86rf233_write_reg(spi, TRX_STATE_REG, TRX_CMD_TX_START);

    /* atomic_set(&at86rf233->tx, 0) in interrupt handler */

    return 0;
}

static void enable_irqb_interrupt(struct at86rf233_context *at86rf233, bool enable)
{
	if (enable)
	{
		gpio_pin_enable_callback(at86rf233->gpio, RF233IRQPIN);
	}
	else
	{
		gpio_pin_disable_callback(at86rf233->gpio, RF233IRQPIN);
	}
}

static int at86rf233_start(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	at86rf233_reset_state_machine(dev);
	gpio_pin_write(at86rf233->gpio, RF233SLPPIN, LOW);
	_usleep(AT86RF233_WAKEUP_DELAY);

	enable_irqb_interrupt(at86rf233, true);

	SYS_LOG_DBG("started");
	return 0;
}

static int at86rf233_stop(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	enable_irqb_interrupt(at86rf233, false);

	at86rf233_reset_state_machine(dev);
	gpio_pin_write(at86rf233->gpio, RF233SLPPIN, HIGH);

	SYS_LOG_DBG("stopped");
	return 0;
}

static void at86rf233_rx_thread(void *arg1, void *unused1, void *unused2)
{
	struct device *dev = (struct device *)arg1;
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	struct net_buf *pkt_buf = NULL;
	struct net_buf *buf;
	uint8_t pkt_len;
	uint8_t phr;

	while (1)
	{
		buf = NULL;

		k_sem_take(&at86rf233->rx_lock, K_FOREVER);

		/* CHECK RX OverFlow SYS_LOG_ERR("RX overflow!"); */

		at86rf233_sram_read(spi, 0, &phr, sizeof phr);
		pkt_len = (phr & 0x7f) - AT86RF233_FCS_LENGTH;

		buf = net_nbuf_get_reserve_rx(0, K_NO_WAIT);
		if (!buf) {
			SYS_LOG_ERR("No buf available");
			goto out;
		}

#if defined(CONFIG_IEEE802154_AT86RF233_RAW)
		/**
		 * Reserve 1 byte for length
		 */
		pkt_buf = net_nbuf_get_reserve_data(1, K_NO_WAIT);
#else
		pkt_buf = net_nbuf_get_reserve_data(0, K_NO_WAIT);
#endif
		if (!pkt_buf) {
			SYS_LOG_ERR("No pkt_buf available");
			goto out;
		}

		net_buf_frag_insert(buf, pkt_buf);

		if (!at86rf233_sram_read(spi, sizeof phr, pkt_buf->data, pkt_len))
		{
			SYS_LOG_ERR("No content read");
			goto out;
		}
		net_buf_add(pkt_buf, pkt_len);

		if (ieee802154_radio_handle_ack(at86rf233->iface, buf) == NET_OK)
		{
			SYS_LOG_DBG("ACK packet handled");
			goto out;
		}

		uint8_t temp[3];	/* LQI + ED + RX_STATUS */
		at86rf233_sram_read(spi, sizeof phr + pkt_len + AT86RF233_FCS_LENGTH, temp, sizeof temp / sizeof temp[0]);
		at86rf233->lqi = temp[0];
		at86rf233->rssi = temp[1];
		at86rf233->rx_status = temp[2];

		if ((at86rf233->rx_status & (1<<7)) == 0)
		{
			SYS_LOG_ERR("Bad packet CRC");
			goto out;
		}

		SYS_LOG_DBG("Caught a packet (%u) (LQI: %u, RSSI: %u)",
			    pkt_len, at86rf233->lqi, at86rf233->rssi);

#if defined(CONFIG_IEEE802154_AT86RF233_RAW)
		net_buf_add_u8(pkt_buf, at86rf233->lqi);
#endif

		if (net_recv_data(at86rf233->iface, buf) < 0)
		{
			SYS_LOG_DBG("Packet dropped by NET stack");
			goto out;
		}

		net_analyze_stack("at86rf233 Rx Fiber stack", (unsigned char *)at86rf233->at86rf233_rx_stack, CONFIG_IEEE802154_AT86RF233_RX_STACK_SIZE);

		continue;

out:
		if (buf) {
			net_buf_unref(buf);
		}
	}
}

static void at86rf233_set_csma_backoff_exp(struct device *dev, uint8_t min, uint8_t max)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

    max = (max > 8) ? 8 : max;
    min = (min > max) ? max : min;
    SYS_LOG_INF("Set min BE=%u, max BE=%u", min, max);

    at86rf233_write_reg(spi, CSMA_BE_REG, (max << 4) | (min));
}

static void at86rf233_set_frame_max_retries(struct device *dev, int8_t retries)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

    retries = (retries > 7) ? 7 : retries; /* valid values: 0-7 */
    retries = (retries < 0) ? 7 : retries;
    SYS_LOG_INF("Set Frame retries to %u", retries);

    uint8_t tmp = at86rf233_read_reg(spi, XAH_CTRL_0_REG);
    tmp &= ~(0xf << 4);
    tmp |= (retries << 4);
    at86rf233_write_reg(spi, XAH_CTRL_0_REG, tmp);
}

static void at86rf233_set_csma_max_retries(struct device *dev, int8_t retries)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

    retries = (retries > 5) ? 5 : retries; /* valid values: 0-5 */
    retries = (retries < 0) ? 7 : retries; /* max < 0 => disable CSMA (set to 7) */
    SYS_LOG_INF("Set CSMA retries to %u", retries);

    uint8_t tmp = at86rf233_read_reg(spi, XAH_CTRL_0_REG);
    tmp &= ~(7u << 1);
    tmp |= (retries << 1);
    at86rf233_write_reg(spi, XAH_CTRL_0_REG, tmp);
}

static void at86rf233_set_csma_seed(struct device *dev, uint8_t entropy[2])
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

    if (entropy == NULL) {
    	SYS_LOG_ERR("CSMA seed entropy is nullpointer");
        return;
    }
    SYS_LOG_INF("Set CSMA seed to 0x%x 0x%x", entropy[0], entropy[1]);

	at86rf233_write_reg(spi, CSMA_SEED_0_REG, entropy[0]);

    uint8_t tmp = at86rf233_read_reg(spi, CSMA_SEED_1_REG);
    tmp &= ~0x7;
    tmp |= entropy[1] & 0x7;
    at86rf233_write_reg(spi, CSMA_SEED_1_REG, tmp);
}

static inline void irqb_int_handler(struct device *dev,
				    struct gpio_callback *cb, uint32_t pins)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	uint8_t irq_source = at86rf233_read_reg(spi, IRQ_STATUS_REG);
	if (irq_source & (1u << TRX_END))
	{
		if (atomic_get(&at86rf233->tx) == 1)
		{
			// completion of a frame transmission
			atomic_set(&at86rf233->tx, 0);

			return;
		}

		// completion of a frame reception
		k_sem_give(&at86rf233->rx_lock);

		return;
	}
}

static int setup_gpio_callbacks(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	int extint = 1U << find_ext_int(at86rf233->gpio, RF233IRQPIN);

	gpio_init_callback(&at86rf233->irqb_cb, irqb_int_handler, extint);
	gpio_add_callback(at86rf233->gpio, &at86rf233->irqb_cb);

	return 0;
}

static int power_on_and_setup(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;
	uint8_t tmp;

	set_reset(dev, LOW);
	_usleep(1000);
	set_reset(dev, HIGH);
	_usleep(100);

	/* test if the SPI is set up correctly and the device is responding */
	if (at86rf233_read_reg(spi, PART_NUM_REG) != AT86RF233_PARTNUM)
	{
		SYS_LOG_ERR("unable to read correct part number");
		return -EIO;
	}

	at86rf233_reset_state_machine(dev);

    /* auto ack */
	tmp = at86rf233_read_reg(spi, CSMA_SEED_1_REG);
	tmp &= ~(1U << AACK_DIS_ACK);
	at86rf233_write_reg(spi, CSMA_SEED_1_REG, tmp);

	/* csma */
	at86rf233_set_csma_seed(dev, at86rf233->mac_addr);
	at86rf233_set_csma_max_retries(dev, CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MAX_BO);
	at86rf233_set_csma_backoff_exp(dev, CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MIN_BE, CONFIG_NET_L2_IEEE802154_RADIO_CSMA_CA_MAX_BE);

	at86rf233_set_frame_max_retries(dev, 3);

	at86rf233_write_reg(spi, TRX_RPC_REG, 0xff);

    /* switch on RX/TX end interrupt, disable others */
    at86rf233_write_reg(spi, IRQ_MASK_REG, (1u << TRX_END));

    /* enable safe mode (protect RX FIFO until reading data starts) */
    at86rf233_write_reg(spi, TRX_CTRL_2_REG, ((1u << RX_SAFE_MODE) | (0u << OQPSK_SCRAM_EN) | (0u << OQPSK_DATA_RATE)));

    /* don't populate masked interrupt flags to IRQ_STATUS register */
    at86rf233_write_reg(spi, TRX_CTRL_1_REG, ((1u << SPI_CMD_MODE) | (1u << TX_AUTO_CRC_ON) | (0u << IRQ_MASK_MODE) | (0u << IRQ_POLARITY)));

    /* disable clock output to save power */
    at86rf233_write_reg(spi, TRX_CTRL_0_REG, 0x0);

    /* clear interrupt flags */
    at86rf233_read_reg(spi, IRQ_STATUS_REG);

    /* go into RX state */
    at86rf233_write_reg(spi, TRX_STATE_REG, TRX_CMD_RX_AACK_ON);

	setup_gpio_callbacks(dev);

	return 0;
}

static inline int configure_gpios(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	/* setup gpio for the modem interrupt */
	at86rf233->gpio = device_get_binding(CONFIG_GPIO_ATMEL_SAMR21_PORT_DEV_NAME);
	if (at86rf233->gpio == NULL)
	{
		SYS_LOG_ERR("Failed to get pointer to %s device",
				CONFIG_GPIO_ATMEL_SAMR21_PORT_DEV_NAME);
		return -EINVAL;
	}

	gpio_pin_configure(at86rf233->gpio, RF233IRQPIN,
			GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_PUD_PULL_DOWN | GPIO_INT_ACTIVE_HIGH);

	/* setup gpio for the sleep & reset */
	gpio_pin_configure(at86rf233->gpio, RF233RSTPIN, GPIO_DIR_OUT);
	gpio_pin_write(at86rf233->gpio, RF233RSTPIN, LOW);

	gpio_pin_configure(at86rf233->gpio, RF233SLPPIN, GPIO_DIR_OUT);
	gpio_pin_write(at86rf233->gpio, RF233SLPPIN, LOW);

	return 0;
}

static inline int configure_spi(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);
	struct at86rf233_spi *spi = &at86rf233->spi;

	struct spi_config spi_conf = {
		.config = SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB,
		.max_sys_freq = CONFIG_SPI_4_BAUDRATE,
	};

	spi->dev = device_get_binding(CONFIG_SPI_4_NAME);
	if (!spi->dev)
	{
		SYS_LOG_ERR("Unable to get SPI device");
		return -ENODEV;
	}

	spi->slave = 0;

	if (spi_configure(spi->dev, &spi_conf) != 0)
	{
		spi->dev = NULL;
		return -EIO;
	}

	SYS_LOG_DBG("SPI configured %s, %d", CONFIG_SPI_4_NAME, 0);

	return 0;
}

static int at86rf233_init(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	atomic_set(&at86rf233->tx, 0);
	k_sem_init(&at86rf233->rx_lock, 0, UINT_MAX);

	SYS_LOG_DBG("\nInitialize AT86RF233 Transceiver\n");

	if (configure_gpios(dev) != 0) {
		SYS_LOG_ERR("Configuring GPIOS failed");
		return -EIO;
	}

	if (configure_spi(dev) != 0) {
		SYS_LOG_ERR("Configuring SPI failed");
		return -EIO;
	}

	SYS_LOG_DBG("GPIO and SPI configured");

	if (power_on_and_setup(dev) != 0) {
		SYS_LOG_ERR("Configuring MCR20A failed");
		return -EIO;
	}

	k_thread_spawn(at86rf233->at86rf233_rx_stack, CONFIG_IEEE802154_AT86RF233_RX_STACK_SIZE, (k_thread_entry_t)at86rf233_rx_thread,
		       	   dev, NULL, NULL, K_PRIO_COOP(2), 0, 0);

	return 0;
}

static inline uint8_t *get_mac(struct device *dev)
{
	struct at86rf233_context *at86rf233 = DEV_CTX(dev);

	uint32_t *ptr = (uint32_t *)(at86rf233->mac_addr);

	UNALIGNED_PUT(sys_rand32_get(), ptr);
	ptr = (uint32_t *)(at86rf233->mac_addr + 4);
	UNALIGNED_PUT(sys_rand32_get(), ptr);

	at86rf233->mac_addr[0] = (at86rf233->mac_addr[0] & ~0x01) | 0x02;

	return at86rf233->mac_addr;
}

static void at86rf233_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct at86rf233_context *at86rf233 = dev->driver_data;
	uint8_t *mac = get_mac(dev);

	SYS_LOG_DBG("");

	net_if_set_link_addr(iface, mac, 8, NET_LINK_IEEE802154);

	at86rf233->iface = iface;

	ieee802154_init(iface);
}

static struct at86rf233_context at86rf233_context_data;

static struct ieee802154_radio_api at86rf233_radio_api = {
	.iface_api.init		= at86rf233_iface_init,
	.iface_api.send		= ieee802154_radio_send,

	.cca			= at86rf233_cca,
	.set_channel		= at86rf233_set_channel,
	.set_pan_id		= at86rf233_set_pan_id,
	.set_short_addr		= at86rf233_set_short_addr,
	.set_ieee_addr		= at86rf233_set_ieee_addr,
	.set_txpower		= at86rf233_set_txpower,
	.tx			= at86rf233_tx,
	.start			= at86rf233_start,
	.stop			= at86rf233_stop,
	.get_lqi	= at86rf233_get_lqi,
};

NET_DEVICE_INIT(at86rf233, CONFIG_IEEE802154_AT86RF233_DRV_NAME, at86rf233_init, &at86rf233_context_data, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &at86rf233_radio_api, IEEE802154_L2,
		NET_L2_GET_CTX_TYPE(IEEE802154_L2), 125);
