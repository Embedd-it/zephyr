/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip USART driver.
 *
 */

#define DT_DRV_COMPAT microchip_usart_g1_uart

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/uart.h>
#if defined(CONFIG_UART_MCHP_ASYNC)
#include <zephyr/drivers/dma.h>
#endif /* CONFIG_UART_MCHP_ASYNC */
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_mchp_usart_g1, CONFIG_UART_LOG_LEVEL);

#define TIMEOUT_POLL_OUT 10000u
/* Period of the polling task for async reception, in microseconds */
#define ASYNC_RX_POLL_US 1000

#define BAUDRATE_CLOCK_SOURCE_DIV 1u
#define BAUDRATE_CLOCK_SOURCE_PCK 2u
#define PROGRAMMABLE_CLOCK        ((clock_control_subsys_t)CLOCK_MCHP_PCK4)

struct uart_mchp_usart_config {
	/* Common UART config */
	struct uart_config uart_config;

	/* Register block */
	usart_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Main clock subsystem. */
	clock_control_subsys_t clock_subsys;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_MCHP_ASYNC)
	/* IRQ setup function */
	void (*irq_config)(const struct device *dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC */

#if defined(CONFIG_UART_MCHP_ASYNC)
	/* DMA data */
	struct {
		const struct device *dev;
		uint32_t tx_channel;
		uint32_t tx_perid;
		uint32_t rx_channel;
		uint32_t rx_perid;
	} dma;
#endif /* CONFIG_UART_MCHP_ASYNC */

	/* The clock source to use to generate the baudrate */
	const uint8_t baudrate_clock_source;

	/* RS485 transmitter timeguard (US_TTGR.TG), in bit periods. */
	uint8_t rs485_tx_guard_time;

	/* Channel mode */
	const uint8_t channel_mode;
};

struct uart_mchp_usart_data {
	/* Stored baudrate value */
	uint32_t baudrate;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_MCHP_ASYNC)
	/* Interrupt Callback */
	uart_irq_callback_user_data_t irq_cb;
	/* Interrupt Callback Arg */
	void *irq_cb_data;

#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC */

#if defined(CONFIG_UART_MCHP_ASYNC)
	/* Device structure */
	const struct device *dev;

	/* Asynchronous callback function */
	uart_callback_t async_cb;

	/* Asynchronous callback user data */
	void *async_cb_data;

	/* Queue of events pending delivery to async_cb */
	struct k_msgq evt_msgq;

	/* Storage for evt_msgq */
	struct uart_event evt_buf[CONFIG_UART_MCHP_USART_G1_ASYNC_EVT_QUEUE_DEPTH];

	/* Work item delivering queued events to async_cb in thread context */
	struct k_work evt_work;

	/* TX timeout work structure */
	struct k_work_delayable tx_timeout_work;

	/* TX buffer */
	const uint8_t *tx_buf;

	/* TX buffer length */
	size_t tx_len;

	/* RX timeout work structure */
	struct k_work_delayable rx_timeout_work;

	/* RX timeout time, in microseconds */
	int64_t rx_timeout_time;

	/* Timestamp of the last RX activity, in microseconds */
	int64_t rx_timeout_timestamp;

	/* Timestamp of the last RX poll, in microseconds */
	int64_t rx_last_poll;

	/* RX buffer */
	uint8_t *rx_buf;

	/* RX buffer length */
	size_t rx_len;

	/* RX processed length */
	size_t rx_processed_len;

	/* RX processed length from previous timeout */
	size_t previous_rx_processed_len;

	/* Whether the received buffer has been sent to the application */
	bool rx_buf_claimed;

	/* Next RX buffer */
	uint8_t *rx_next_buf;

	/* Next RX buffer length */
	size_t rx_next_len;
#endif /* CONFIG_UART_MCHP_ASYNC */

#if defined(CONFIG_UART_LINE_CTRL)
	/* Last value driven onto RTS via line_ctrl_set(); the hardware has no readback of the
	 * output state, only of the CTS/DSR/DCD *inputs*.
	 */
	bool rts_asserted: 1;

	/* Last value driven onto DTR via line_ctrl_set(); same readback limitation as RTS. */
	bool dtr_asserted: 1;
#endif /* CONFIG_UART_LINE_CTRL */
};

#if defined(CONFIG_UART_MCHP_ASYNC)

static void uart_mchp_usart_rx_done_callback(const struct device *dma_dev, void *arg, uint32_t id,
					     int error_code);
static void uart_mchp_usart_rx_complete_buffer(struct uart_mchp_usart_data *dev_data);

/*
 * Queue an event for the application callback. The callback itself runs from the system workqueue.
 */
static void async_user_callback(struct uart_mchp_usart_data *dev_data, struct uart_event *event)
{
	int ret = k_msgq_put(&dev_data->evt_msgq, event, K_NO_WAIT);

	if (ret != 0) {
		LOG_ERR("Async event queue full, event %d dropped", event->type);
		__ASSERT(false, "Async event queue full");
		return;
	}

	(void)k_work_submit(&dev_data->evt_work);
}

/*
 * Deliver the queued events to the application callback, in order.
 */
static void uart_mchp_usart_evt_work_handler(struct k_work *work)
{
	struct uart_mchp_usart_data *dev_data =
		CONTAINER_OF(work, struct uart_mchp_usart_data, evt_work);
	struct uart_event evt = {0u};

	while (k_msgq_get(&dev_data->evt_msgq, &evt, K_NO_WAIT) == 0) {
		if (dev_data->async_cb != NULL) {
			dev_data->async_cb(dev_data->dev, &evt, dev_data->async_cb_data);
		}
	}
}

static int uart_mchp_usart_configure_rx_dma(const struct uart_mchp_usart_config *dev_config,
					    struct uart_mchp_usart_data *dev_data)
{
	int ret = 0;
	usart_registers_t *const regs = dev_config->regs;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.user_data = dev_data;
	dma_cfg.dma_callback = uart_mchp_usart_rx_done_callback;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = dev_config->dma.rx_perid;

	dma_blk.block_size = 1;
	dma_blk.source_address = (uint32_t)&regs->US_RHR;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	ret = dma_config(dev_config->dma.dev, dev_config->dma.rx_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("Unable to reconfigure the DMA: %i", ret);
		return ret;
	}

	return 0;
}

/*
 * Notifies that UART RX data has been processed.
 */
static void uart_mchp_usart_notify_rx_processed(struct uart_mchp_usart_data *dev_data,
						size_t processed)
{
	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx.buf = dev_data->rx_buf,
		.data.rx.offset = dev_data->rx_processed_len,
		.data.rx.len = processed - dev_data->rx_processed_len,
	};

	/* No new data, do nothing */
	if (dev_data->rx_processed_len >= processed) {
		return;
	}

	dev_data->rx_processed_len = processed;

	async_user_callback(dev_data, &evt);
}

/*
 * Request the next RX buffer. Raised when reception starts on the current
 * buffer, so the application has the whole buffer duration to respond.
 */
static void uart_mchp_usart_request_rx_buf(struct uart_mchp_usart_data *dev_data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	async_user_callback(dev_data, &evt);
}

/*
 * Current uptime in microseconds, used for the RX timeout.
 */
static int64_t rx_uptime_us(void)
{
	return (int64_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

/*
 * Reschedule the rx timeout during async reception.
 */
static void reschedule_rx_timeout(struct uart_mchp_usart_data *dev_data)
{
	int64_t elapsed = rx_uptime_us() - dev_data->rx_timeout_timestamp;
	int64_t remaining = 1;

	if (dev_data->rx_timeout_time > elapsed) {
		remaining = dev_data->rx_timeout_time - elapsed;
	}

	k_work_reschedule(&dev_data->rx_timeout_work, K_USEC(MIN(ASYNC_RX_POLL_US, remaining)));
}

/*
 * Restart the RX timeout from the current time.
 */
static void restart_rx_timeout(struct uart_mchp_usart_data *dev_data)
{
	dev_data->rx_timeout_timestamp = rx_uptime_us();
	dev_data->rx_last_poll = dev_data->rx_timeout_timestamp;
}

/*
 * Halts the UART transmission and stops the DMA transfer.
 */
static int uart_mchp_usart_halt_tx(struct uart_mchp_usart_data *dev_data)
{
	size_t tx_active = 0u;
	struct dma_status dma_stat;
	const struct uart_mchp_usart_config *dev_config = dev_data->dev->config;
	usart_registers_t *const regs = dev_config->regs;

	unsigned int key = irq_lock();

	regs->US_IDR = US_IDR_USART_TXEMPTY_Msk;

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {.buf = dev_data->tx_buf, .len = 0u},
	};

	tx_active = dev_data->tx_len;
	dev_data->tx_buf = NULL;
	dev_data->tx_len = 0u;

	dma_stop(dev_config->dma.dev, dev_config->dma.tx_channel);

	irq_unlock(key);

	if (dma_get_status(dev_config->dma.dev, dev_config->dma.tx_channel, &dma_stat) == 0) {
		evt.data.tx.len = tx_active - dma_stat.pending_length;
	}

	if (tx_active != 0) {
		async_user_callback(dev_data, &evt);
	} else {
		return -EINVAL;
	}

	return 0;
}

#endif /* CONFIG_UART_MCHP_ASYNC */

static void uart_mchp_usart_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	/* Wait for transmitter to be ready */
	if (!WAIT_FOR(((regs->US_CSR & US_CSR_USART_TXRDY_Msk) != 0u), TIMEOUT_POLL_OUT,
		      (void)NULL)) {
		LOG_ERR("Poll out timed out");
		return;
	}

	/* send a character */
	regs->US_THR = c;
}

static int uart_mchp_usart_err_check(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	int errors = 0;

	if ((regs->US_CSR & US_CSR_USART_OVRE_Msk) != 0u) {
		errors |= UART_ERROR_OVERRUN;
	}

	if ((regs->US_CSR & US_CSR_USART_PARE_Msk) != 0u) {
		errors |= UART_ERROR_PARITY;
	}

	if ((regs->US_CSR & US_CSR_USART_FRAME_Msk) != 0u) {
		errors |= UART_ERROR_FRAMING;
	}

	regs->US_CR = US_CR_USART_RSTSTA_Msk;

	return errors;
}

static int uart_mchp_usart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	/* No data available, return */
	if ((regs->US_CSR & US_CSR_USART_RXRDY_Msk) == 0u) {
		return -EAGAIN;
	}

	/* got a character */
	*c = (unsigned char)regs->US_RHR;

	return 0;
}

#if defined(CONFIG_UART_WIDE_DATA)
static void uart_mchp_usart_poll_out_u16(const struct device *dev, uint16_t out_u16)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	/* Wait for transmitter to be ready */
	if (!WAIT_FOR(((regs->US_CSR & US_CSR_USART_TXRDY_Msk) != 0u), TIMEOUT_POLL_OUT,
		      (void)NULL)) {
		LOG_ERR("Poll out timed out");
		return;
	}

	/* send a character */
	regs->US_THR = out_u16 & US_THR_Msk;
}

static int uart_mchp_usart_poll_in_u16(const struct device *dev, uint16_t *in_u16)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	/* No data available, return */
	if ((regs->US_CSR & US_CSR_USART_RXRDY_Msk) == 0u) {
		return -EAGAIN;
	}

	/* got a character */
	*in_u16 = (uint16_t)regs->US_RHR;

	return 0;
}
#endif /* CONFIG_UART_WIDE_DATA */

#if defined(CONFIG_UART_USE_RUNTIME_CONFIGURE)

/*
 * Convert from the Zephyr stop bits representation to the register value
 */
static int uart_mchp_usart_config_stopbits_to_nbstop(enum uart_config_stop_bits stop_bits,
						     uint8_t *usart_nbstop)
{
	int ret = 0;

	switch (stop_bits) {
	case UART_CFG_STOP_BITS_1:
		*usart_nbstop = US_MR_USART_NBSTOP_1_BIT_Val;
		break;
	case UART_CFG_STOP_BITS_1_5:
		*usart_nbstop = US_MR_USART_NBSTOP_1_5_BIT_Val;
		break;
	case UART_CFG_STOP_BITS_2:
		*usart_nbstop = US_MR_USART_NBSTOP_2_BIT_Val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * Convert from the Zephyr parity representation to the register value
 */
static int uart_mchp_usart_config_parity_to_par(enum uart_config_parity parity, uint8_t *usart_par)
{
	int ret = 0;

	switch (parity) {
	case UART_CFG_PARITY_EVEN:
		*usart_par = US_MR_USART_PAR_EVEN_Val;
		break;
	case UART_CFG_PARITY_ODD:
		*usart_par = US_MR_USART_PAR_ODD_Val;
		break;
	case UART_CFG_PARITY_SPACE:
		*usart_par = US_MR_USART_PAR_SPACE_Val;
		break;
	case UART_CFG_PARITY_MARK:
		*usart_par = US_MR_USART_PAR_MARK_Val;
		break;
	case UART_CFG_PARITY_NONE:
		*usart_par = US_MR_USART_PAR_NO_Val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * Convert from the Zephyr frame size representation to the register value
 */
static int uart_mchp_usart_config_data_bits_to_chrl(enum uart_config_data_bits data_bits,
						    uint8_t *usart_chrl)
{
	int ret = 0;

	switch (data_bits) {
	case UART_CFG_DATA_BITS_5:
		*usart_chrl = US_MR_USART_CHRL_5_BIT_Val;
		break;
	case UART_CFG_DATA_BITS_6:
		*usart_chrl = US_MR_USART_CHRL_6_BIT_Val;
		break;
	case UART_CFG_DATA_BITS_7:
		*usart_chrl = US_MR_USART_CHRL_7_BIT_Val;
		break;
	case UART_CFG_DATA_BITS_8:
		*usart_chrl = US_MR_USART_CHRL_8_BIT_Val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * Convert from the Zephyr flow control representation to the register value
 */
static int uart_mchp_usart_config_flow_control_to_mode(enum uart_config_flow_control flow_ctrl,
						       uint8_t *usart_mode)
{
	int ret = 0;

	switch (flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		*usart_mode = US_MR_USART_MODE_NORMAL_Val;
		break;
	case UART_CFG_FLOW_CTRL_RS485:
		*usart_mode = US_MR_USART_MODE_RS485_Val;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		*usart_mode = US_MR_USART_MODE_HW_HANDSHAKING_Val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int uart_mchp_usart_configure(const struct device *dev, const struct uart_config *uart_cfg)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;
	uint32_t divisor = 0u;
	uint32_t rate = 0u;
	uint32_t peripheral_clock_rate = 0u;
	uint32_t mr = 0u;
	uint8_t usart_par = 0u;
	uint8_t usart_chrl = 0u;
	uint8_t usart_nbstop = 0u;
	uint8_t usart_mode = 0u;
	int ret = 0;

	/* Baudrate must be > 0 */
	if (uart_cfg->baudrate == 0) {
		return -ENOTSUP;
	}

	/* Driver doesn't support 0.5 stop bits or DTR DSR flow control; also baudrate must be > 0
	 */
	if (uart_cfg->stop_bits == UART_CFG_STOP_BITS_0_5 ||
	    uart_cfg->flow_ctrl == UART_CFG_FLOW_CTRL_DTR_DSR || uart_cfg->baudrate == 0) {
		return -ENOTSUP;
	}

	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys,
				     &peripheral_clock_rate);
	if (ret != 0) {
		LOG_ERR("Unable to fetch the peripheral clock rate: %i", ret);
		return ret;
	}

	if (dev_config->baudrate_clock_source == BAUDRATE_CLOCK_SOURCE_PCK) {
		ret = clock_control_get_rate(dev_config->clock, PROGRAMMABLE_CLOCK, &rate);
		if (ret != 0) {
			LOG_ERR("Unable to fetch the programmable clock rate: %i", ret);
			return ret;
		}

		/* PCK4 must run at less than one third of the peripheral clock rate in order to be
		 * used for baudrate generation
		 */
		if (rate * 3 > peripheral_clock_rate) {
			LOG_ERR("The PCK4 frequency must be lower than one third of the peripheral "
				"clock frequency");
			return -EINVAL;
		}
	} else {
		rate = peripheral_clock_rate;
	}

	/* The rate is the peripheral clock divided by 8 */
	if (dev_config->baudrate_clock_source == BAUDRATE_CLOCK_SOURCE_DIV) {
		rate /= 8;
	}

	if (rate / 16u < uart_cfg->baudrate) {
		LOG_ERR("MCK frequency is too small to set required baud rate");
		return -ENOTSUP;
	}

	divisor = (rate / 16u) / uart_cfg->baudrate;

	if (divisor > 0xFFFF) {
		return -EINVAL;
	}

	/* Reset and disable USART */
	regs->US_CR = US_CR_USART_RSTRX_Msk | US_CR_USART_RSTTX_Msk | US_CR_USART_RXDIS_Msk |
		      US_CR_USART_TXDIS_Msk | US_CR_USART_RSTSTA_Msk;

	/* Clean slate */
	if (uart_cfg->data_bits == UART_CFG_DATA_BITS_9) {
		mr |= US_MR_USART_MODE9_Msk;
	}
	/* from 5 to 8 data bits the CHRL field handles it */
	else {
		mr &= ~US_MR_USART_MODE9_Msk;

		ret = uart_mchp_usart_config_data_bits_to_chrl(uart_cfg->data_bits, &usart_chrl);
		if (ret != 0) {
			return ret;
		}
		mr &= ~US_MR_USART_CHRL_Msk;
		mr |= US_MR_USART_CHRL(usart_chrl);
	}

	ret = uart_mchp_usart_config_stopbits_to_nbstop(uart_cfg->stop_bits, &usart_nbstop);
	if (ret != 0) {
		return ret;
	}
	mr &= ~US_MR_USART_NBSTOP_Msk;
	mr |= US_MR_USART_NBSTOP(usart_nbstop);

	ret = uart_mchp_usart_config_parity_to_par(uart_cfg->parity, &usart_par);
	if (ret != 0) {
		return ret;
	}
	mr &= ~US_MR_USART_PAR_Msk;
	mr |= US_MR_USART_PAR(usart_par);

	ret = uart_mchp_usart_config_flow_control_to_mode(uart_cfg->flow_ctrl, &usart_mode);
	if (ret != 0) {
		return ret;
	}
	mr &= ~US_MR_USART_MODE_Msk;
	mr |= US_MR_USART_MODE(usart_mode);

	mr |= US_MR_USART_CHMODE(dev_config->channel_mode);

	/* Configure the clock source */
	mr |= US_MR_USART_USCLKS(dev_config->baudrate_clock_source);

	regs->US_MR = mr;

	/* Set baudrate */
	regs->US_BRGR = US_BRGR_CD(divisor);

	/* Transmitter timeguard, only meaningful in RS485 mode. Holds the RTS/driver-enable line
	 * asserted past the last character.
	 */
	regs->US_TTGR = US_TTGR_USART_TG(dev_config->rs485_tx_guard_time);

	/* Enable receiver and transmitter */
	regs->US_CR = US_CR_USART_RXEN_Msk | US_CR_USART_TXEN_Msk;

	dev_data->baudrate = uart_cfg->baudrate;

	return 0;
}

static int uart_mchp_usart_config_get(const struct device *dev, struct uart_config *uart_cfg)
{
	const struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	uart_cfg->baudrate = dev_data->baudrate;

	/* Fetch parity */
	switch (regs->US_MR & US_MR_USART_PAR_Msk) {
	case US_MR_USART_PAR_EVEN:
		uart_cfg->parity = UART_CFG_PARITY_EVEN;
		break;
	case US_MR_USART_PAR_ODD:
		uart_cfg->parity = UART_CFG_PARITY_ODD;
		break;
	case US_MR_USART_PAR_SPACE:
		uart_cfg->parity = UART_CFG_PARITY_SPACE;
		break;
	case US_MR_USART_PAR_MARK:
		uart_cfg->parity = UART_CFG_PARITY_MARK;
		break;
	case US_MR_USART_PAR_NO:
	default:
		uart_cfg->parity = UART_CFG_PARITY_NONE;
		break;
	}

	/* Fetch stop bits */
	switch (regs->US_MR & US_MR_USART_NBSTOP_Msk) {
	case US_MR_USART_NBSTOP_1_5_BIT:
		uart_cfg->stop_bits = UART_CFG_STOP_BITS_1_5;
		break;
	case US_MR_USART_NBSTOP_2_BIT:
		uart_cfg->stop_bits = UART_CFG_STOP_BITS_2;
		break;
	case US_MR_USART_NBSTOP_1_BIT:
	default:
		uart_cfg->stop_bits = UART_CFG_STOP_BITS_1;
		break;
	}

	/* Fetch data bits */
	if ((regs->US_MR & US_MR_USART_MODE9_Msk) != 0) {
		uart_cfg->data_bits = UART_CFG_DATA_BITS_9;
	} else {
		switch (regs->US_MR & US_MR_USART_CHRL_Msk) {
		case US_MR_USART_CHRL_5_BIT:
			uart_cfg->data_bits = UART_CFG_DATA_BITS_5;
			break;
		case US_MR_USART_CHRL_6_BIT:
			uart_cfg->data_bits = UART_CFG_DATA_BITS_6;
			break;
		case US_MR_USART_CHRL_7_BIT:
			uart_cfg->data_bits = UART_CFG_DATA_BITS_7;
			break;
		case US_MR_USART_CHRL_8_BIT:
		default:
			uart_cfg->data_bits = UART_CFG_DATA_BITS_8;
			break;
		}
	}

	/* Fetch flow control */
	switch (regs->US_MR & US_MR_USART_MODE_Msk) {
	case US_MR_USART_MODE_HW_HANDSHAKING:
		uart_cfg->flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
		break;
	case US_MR_USART_MODE_RS485:
		uart_cfg->flow_ctrl = UART_CFG_FLOW_CTRL_RS485;
		break;
	case US_MR_USART_MODE_NORMAL:
	default:
		uart_cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
		break;
	}

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#if defined(CONFIG_UART_LINE_CTRL)

/*
 * RTS/DTR are driven via one-shot action bits in US_CR (write-only, no readback of the output
 * state), so the last requested value is shadowed in dev_data for line_ctrl_get(). DCD/DSR
 * are inputs from the remote device and can only be read, never driven from this side.
 */
static int uart_mchp_usart_line_ctrl_set(const struct device *dev, uint32_t ctrl, uint32_t val)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	switch (ctrl) {
	case UART_LINE_CTRL_RTS:
		regs->US_CR = (val != 0u) ? US_CR_USART_RTSEN_Msk : US_CR_USART_RTSDIS_Msk;
		dev_data->rts_asserted = (val != 0u);
		return 0;
	case UART_LINE_CTRL_DTR:
		regs->US_CR = (val != 0u) ? US_CR_USART_DTREN_Msk : US_CR_USART_DTRDIS_Msk;
		dev_data->dtr_asserted = (val != 0u);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int uart_mchp_usart_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t *val)
{
	const struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	switch (ctrl) {
	case UART_LINE_CTRL_RTS:
		*val = dev_data->rts_asserted ? 1u : 0u;
		return 0;
	case UART_LINE_CTRL_DTR:
		*val = dev_data->dtr_asserted ? 1u : 0u;
		return 0;
	case UART_LINE_CTRL_DCD:
		*val = ((regs->US_CSR & US_CSR_USART_DCD_Msk) != 0u) ? 1u : 0u;
		return 0;
	case UART_LINE_CTRL_DSR:
		*val = ((regs->US_CSR & US_CSR_USART_DSR_Msk) != 0u) ? 1u : 0u;
		return 0;
	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_UART_LINE_CTRL */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)

static int uart_mchp_usart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	if (size == 0u) {
		return 0;
	}

	/* No room */
	if ((regs->US_CSR & US_CSR_USART_TXRDY_Msk) == 0u) {
		return 0;
	}

	/* Only one byte in our FIFO */
	regs->US_THR = *tx_data;

	return 1;
}

static int uart_mchp_usart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;
	int bytes_read = 0;

	while (bytes_read < size) {
		if (regs->US_CSR & US_CSR_USART_RXRDY_Msk) {
			rx_data[bytes_read] = regs->US_RHR;
			bytes_read++;
		} else {
			break;
		}
	}

	return bytes_read;
}

static void uart_mchp_usart_irq_rx_enable(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	regs->US_IER = US_IER_USART_RXRDY_Msk;
}

static void uart_mchp_usart_irq_rx_disable(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	regs->US_IDR = US_IDR_USART_RXRDY_Msk;
}

static void uart_mchp_usart_irq_tx_enable(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	regs->US_IER = US_IER_USART_TXRDY_Msk;
}

static void uart_mchp_usart_irq_tx_disable(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	regs->US_IDR = US_IDR_USART_TXRDY_Msk;
}

static int uart_mchp_usart_irq_tx_ready(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	/* Check that the transmitter is ready but only
	 * return true if the interrupt is also enabled
	 */
	return ((regs->US_CSR & US_CSR_USART_TXRDY_Msk) != 0u) &&
	       ((regs->US_IMR & US_IMR_USART_TXRDY_Msk) != 0u);
}

static int uart_mchp_usart_irq_tx_complete(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	return ((regs->US_CSR & US_CSR_USART_TXRDY_Msk) != 0u) &&
	       ((regs->US_CSR & US_CSR_USART_TXEMPTY_Msk) != 0u);
}

static int uart_mchp_usart_irq_rx_ready(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	return (regs->US_CSR & US_CSR_USART_RXRDY_Msk) != 0u;
}

static void uart_mchp_usart_irq_err_enable(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	regs->US_IER = US_IER_USART_OVRE_Msk | US_IER_USART_FRAME_Msk | US_IER_USART_PARE_Msk;
}

static void uart_mchp_usart_irq_err_disable(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	regs->US_IDR = US_IDR_USART_OVRE_Msk | US_IDR_USART_FRAME_Msk | US_IDR_USART_PARE_Msk;
}

static int uart_mchp_usart_irq_is_pending(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;

	return ((regs->US_IMR & (US_IMR_USART_TXRDY_Msk | US_IMR_USART_RXRDY_Msk)) &
		(regs->US_CSR & (US_CSR_USART_TXRDY_Msk | US_CSR_USART_RXRDY_Msk))) != 0;
}

static void uart_mchp_usart_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void uart_mchp_usart_irq_callback_set(const struct device *dev,
					     uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_mchp_usart_data *dev_data = dev->data;

	dev_data->irq_cb = cb;
	dev_data->irq_cb_data = cb_data;

#if defined(CONFIG_UART_MCHP_ASYNC) && defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->async_cb = NULL;
	dev_data->async_cb_data = NULL;
#endif /* CONFIG_UART_MCHP_ASYNC && CONFIG_UART_EXCLUSIVE_API_CALLBACKS */
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if defined(CONFIG_UART_MCHP_ASYNC)
static int uart_mchp_usart_callback_set(const struct device *dev, uart_callback_t callback,
					void *user_data)
{
	struct uart_mchp_usart_data *dev_data = dev->data;

	dev_data->async_cb = callback;
	dev_data->async_cb_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
	dev_data->irq_cb = NULL;
	dev_data->irq_cb_data = NULL;
#endif /* CONFIG_UART_EXCLUSIVE_API_CALLBACKS */

	return 0;
}

static int uart_mchp_usart_tx(const struct device *dev, const uint8_t *buf, size_t len,
			      int32_t timeout)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;
	int ret = 0;
	unsigned int key = 0;

	if (dev_config->dma.tx_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len > 0xFFFFU) {
		return -EINVAL;
	}

	key = irq_lock();

	/* Some transaction is already in progress */
	if (dev_data->tx_len != 0u) {
		irq_unlock(key);
		return -EBUSY;
	}

	dev_data->tx_buf = buf;
	dev_data->tx_len = len;

	irq_unlock(key);

	ret = dma_reload(dev_config->dma.dev, dev_config->dma.tx_channel, (uint32_t)buf,
			 (uint32_t)&regs->US_THR, len);
	if (ret != 0) {
		LOG_ERR("Unable to reload the DMA: %i", ret);
		return ret;
	}

	/* The TX timeout is only meaningful with hardware flow control */
	if ((timeout != SYS_FOREVER_US) &&
	    ((regs->US_MR & US_MR_USART_MODE_Msk) == US_MR_USART_MODE_HW_HANDSHAKING)) {
		k_work_reschedule(&dev_data->tx_timeout_work, K_USEC(timeout));
	}

	return dma_start(dev_config->dma.dev, dev_config->dma.tx_channel);
}

static int uart_mchp_usart_tx_abort(const struct device *dev)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;

	if (dev_config->dma.tx_channel == 0xFFU) {
		return -ENOTSUP;
	}

	k_work_cancel_delayable(&dev_data->tx_timeout_work);

	return uart_mchp_usart_halt_tx(dev_data);
}

static int uart_mchp_usart_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
				     int32_t timeout)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;
	int ret = 0;
	unsigned int key = 0u;

	if (dev_config->dma.rx_channel == 0xFFU) {
		return -ENOTSUP;
	}

	if (len == 0 || len > 0xFFFFU || buf == NULL) {
		return -EINVAL;
	}

	key = irq_lock();

	/* A reception is already in progress */
	if (dev_data->rx_len != 0u) {
		irq_unlock(key);
		return -EBUSY;
	}

	ret = uart_mchp_usart_configure_rx_dma(dev_config, dev_data);
	if (ret != 0) {
		LOG_ERR("Unable to reconfigure the DMA: %i", ret);
		irq_unlock(key);
		return ret;
	}

	ret = dma_reload(dev_config->dma.dev, dev_config->dma.rx_channel, (uint32_t)&regs->US_RHR,
			 (uint32_t)buf, len);
	if (ret != 0) {
		LOG_ERR("Unable to reload the DMA: %i", ret);
		irq_unlock(key);
		return ret;
	}

	dev_data->rx_buf_claimed = false;
	dev_data->rx_buf = buf;
	dev_data->rx_len = len;
	dev_data->rx_processed_len = 0u;
	dev_data->rx_timeout_time = timeout;

	dev_data->previous_rx_processed_len = 0u;
	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		restart_rx_timeout(dev_data);
		reschedule_rx_timeout(dev_data);
	}

	/*
	 * Start the DMA channel immediately so it is already watching the
	 * peripheral's hardware request line before the first byte can
	 * possibly arrive.
	 */
	ret = dma_start(dev_config->dma.dev, dev_config->dma.rx_channel);
	if (ret != 0) {
		LOG_ERR("Could not start DMA: %i", ret);
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0u;
		irq_unlock(key);
		return ret;
	}

	uart_mchp_usart_request_rx_buf(dev_data);

	irq_unlock(key);

	return 0;
}

static int uart_mchp_usart_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	unsigned int key = 0;

	if (len == 0 || len > 0xFFFFU || buf == NULL) {
		return -EINVAL;
	}

	key = irq_lock();

	/* No operation is currently pending, no point in providing the next buffer */
	if (dev_data->rx_len == 0u) {
		irq_unlock(key);
		return -EACCES;
	}
	/* Another buffer was already provided */
	if (dev_data->rx_next_len != 0u) {
		irq_unlock(key);
		return -EBUSY;
	}
	dev_data->rx_next_buf = buf;
	dev_data->rx_next_len = len;

	irq_unlock(key);

	return 0;
}

static int uart_mchp_usart_rx_disable(const struct device *dev)
{
	struct uart_mchp_usart_data *dev_data = dev->data;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	struct dma_status dma_stat;
	struct uart_event rdy_evt = {.type = UART_RX_RDY};
	struct uart_event buf_rel_evt = {.type = UART_RX_BUF_RELEASED};
	struct uart_event dis_evt = {.type = UART_RX_DISABLED};
	uint8_t *rx_buf = NULL;
	uint8_t *rx_next_buf = NULL;
	unsigned int key = 0;
	int ret = 0;

	k_work_cancel_delayable(&dev_data->rx_timeout_work);

	key = irq_lock();

	/* No operation is pending */
	if (dev_data->rx_len == 0u) {
		irq_unlock(key);
		return -EACCES;
	}

	/* Stop the DMA */
	dma_stop(dev_config->dma.dev, dev_config->dma.rx_channel);

	/* If the operation was already completed point it out */
	ret = dma_get_status(dev_config->dma.dev, dev_config->dma.rx_channel, &dma_stat);
	if (ret != 0) {
		LOG_ERR("Unable to get DMA channel status: %i", ret);
		irq_unlock(key);
		return ret;
	}

	rx_buf = dev_data->rx_buf;

	rdy_evt.data.rx.buf = rx_buf;
	rdy_evt.data.rx.offset = dev_data->rx_processed_len;
	rdy_evt.data.rx.len =
		(dev_data->rx_len - dma_stat.pending_length) - dev_data->rx_processed_len;

	buf_rel_evt.data.rx_buf.buf = rx_buf;

	rx_next_buf = (dev_data->rx_next_len != 0u) ? dev_data->rx_next_buf : NULL;

	dev_data->rx_buf = NULL;
	dev_data->rx_len = 0u;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0u;
	dev_data->rx_processed_len = 0u;

	irq_unlock(key);

	if (rdy_evt.data.rx.len > 0u) {
		async_user_callback(dev_data, &rdy_evt);
	}

	async_user_callback(dev_data, &buf_rel_evt);

	if (rx_next_buf != NULL) {
		buf_rel_evt.data.rx_buf.buf = rx_next_buf;
		async_user_callback(dev_data, &buf_rel_evt);
	}

	async_user_callback(dev_data, &dis_evt);

	return 0;
}
#endif /* CONFIG_UART_MCHP_ASYNC */

static DEVICE_API(uart, uart_mchp_usart_driver_api) = {
	.poll_in = uart_mchp_usart_poll_in,
	.poll_out = uart_mchp_usart_poll_out,
	.err_check = uart_mchp_usart_err_check,
#if defined(CONFIG_UART_WIDE_DATA)
	.poll_in_u16 = uart_mchp_usart_poll_in_u16,
	.poll_out_u16 = uart_mchp_usart_poll_out_u16,
#endif /* CONFIG_UART_WIDE_DATA */

#if defined(CONFIG_UART_USE_RUNTIME_CONFIGURE)
	.configure = uart_mchp_usart_configure,
	.config_get = uart_mchp_usart_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#if defined(CONFIG_UART_LINE_CTRL)
	.line_ctrl_set = uart_mchp_usart_line_ctrl_set,
	.line_ctrl_get = uart_mchp_usart_line_ctrl_get,
#endif /* CONFIG_UART_LINE_CTRL */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	.fifo_fill = uart_mchp_usart_fifo_fill,
	.fifo_read = uart_mchp_usart_fifo_read,
	.irq_tx_enable = uart_mchp_usart_irq_tx_enable,
	.irq_tx_disable = uart_mchp_usart_irq_tx_disable,
	.irq_tx_ready = uart_mchp_usart_irq_tx_ready,
	.irq_rx_enable = uart_mchp_usart_irq_rx_enable,
	.irq_rx_disable = uart_mchp_usart_irq_rx_disable,
	.irq_tx_complete = uart_mchp_usart_irq_tx_complete,
	.irq_rx_ready = uart_mchp_usart_irq_rx_ready,
	.irq_err_enable = uart_mchp_usart_irq_err_enable,
	.irq_err_disable = uart_mchp_usart_irq_err_disable,
	.irq_is_pending = uart_mchp_usart_irq_is_pending,
	.irq_update = uart_mchp_usart_irq_update,
	.irq_callback_set = uart_mchp_usart_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if defined(CONFIG_UART_MCHP_ASYNC)
	.callback_set = uart_mchp_usart_callback_set,
	.tx = uart_mchp_usart_tx,
	.tx_abort = uart_mchp_usart_tx_abort,
	.rx_enable = uart_mchp_usart_rx_enable,
	.rx_buf_rsp = uart_mchp_usart_rx_buf_rsp,
	.rx_disable = uart_mchp_usart_rx_disable,
#endif /* CONFIG_UART_MCHP_ASYNC */
};

#if defined(CONFIG_UART_MCHP_ASYNC)
/*
 * UART TX timeout handler.
 */
static void uart_mchp_usart_tx_timeout_callback(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_mchp_usart_data *dev_data =
		CONTAINER_OF(dwork, struct uart_mchp_usart_data, tx_timeout_work);

	uart_mchp_usart_halt_tx(dev_data);
}

/*
 * UART RX timeout handler.
 */
static void uart_mchp_usart_rx_timeout_callback(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_mchp_usart_data *dev_data =
		CONTAINER_OF(dwork, struct uart_mchp_usart_data, rx_timeout_work);
	const struct uart_mchp_usart_config *dev_config = dev_data->dev->config;
	usart_registers_t *const regs = dev_config->regs;
	struct dma_status dma_stat;
	int ret = 0;
	uint8_t *rx_dma_start = NULL;
	size_t rx_processed = 0;
	int64_t now = 0;
	unsigned int key = irq_lock();

	/* No data left to be received, do nothing */
	if (dev_data->rx_len == 0u) {
		irq_unlock(key);
		return;
	}

	ret = dma_get_status(dev_config->dma.dev, dev_config->dma.rx_channel, &dma_stat);
	if (ret != 0) {
		LOG_ERR("Unable to get DMA status: %i", ret);
		irq_unlock(key);
		return;
	}

	rx_dma_start = dev_data->rx_buf + dev_data->rx_len - dma_stat.pending_length;
	rx_processed = rx_dma_start - dev_data->rx_buf;

	now = rx_uptime_us();

	/* Some data was received, the operation can continue; also restart the inactivity timer. */
	if (rx_processed != dev_data->previous_rx_processed_len) {
		dev_data->rx_timeout_timestamp = dev_data->rx_last_poll;
		dev_data->rx_last_poll = now;
		dev_data->previous_rx_processed_len = rx_processed;
		reschedule_rx_timeout(dev_data);
		irq_unlock(key);
		return;
	}

	dev_data->rx_last_poll = now;

	/* Nothing to report: the inactivity period only starts after receiving data */
	if (rx_processed == dev_data->rx_processed_len) {
		dev_data->rx_timeout_timestamp = now;
		reschedule_rx_timeout(dev_data);
		irq_unlock(key);
		return;
	}

	/* Nothing new, check for timeout */
	if ((now - dev_data->rx_timeout_timestamp) < dev_data->rx_timeout_time) {
		/* Continue */
		reschedule_rx_timeout(dev_data);
		irq_unlock(key);
		return;
	}

	/*
	 * The operation timed out, stop the DMA. This prevents
	 * `uart_mchp_usart_rx_done_callback` from running.
	 */
	dma_stop(dev_config->dma.dev, dev_config->dma.rx_channel);

	/* Fetch the data again after stopping DMA */
	ret = dma_get_status(dev_config->dma.dev, dev_config->dma.rx_channel, &dma_stat);
	if (ret != 0) {
		LOG_ERR("Unable to get DMA status: %i", ret);
		irq_unlock(key);
		return;
	}

	rx_dma_start = dev_data->rx_buf + dev_data->rx_len - dma_stat.pending_length;
	rx_processed = rx_dma_start - dev_data->rx_buf;

	/*
	 * The operation was done anyway, signal it to the application
	 */
	if (dma_stat.pending_length == 0u) {
		if (dev_data->rx_buf_claimed) {
			/* uart_mchp_usart_rx_done_callback() already completed this transaction */
			irq_unlock(key);
			return;
		}
		dev_data->rx_buf_claimed = true;

		uart_mchp_usart_rx_complete_buffer(dev_data);

		if ((dev_data->rx_len != 0u) && (dev_data->rx_timeout_time != SYS_FOREVER_US)) {
			/* The previous call restarted the DMA; keep the periodic poll alive for
			 * whichever buffer we just switched to
			 */
			reschedule_rx_timeout(dev_data);
		}

		irq_unlock(key);
		return;
	}

	ret = uart_mchp_usart_configure_rx_dma(dev_config, dev_data);
	if (ret != 0) {
		LOG_ERR("Unable to reconfigure the DMA: %i", ret);
		irq_unlock(key);
		return;
	}

	/*
	 * We know we still have space, since the above will catch the
	 * empty buffer, so always restart the transfer.
	 */
	ret = dma_reload(dev_config->dma.dev, dev_config->dma.rx_channel, (uint32_t)&regs->US_RHR,
			 (uint32_t)rx_dma_start, dev_data->rx_len - rx_processed);
	if (ret != 0) {
		LOG_ERR("Unable to reload the DMA: %i", ret);
		irq_unlock(key);
		return;
	}

	dev_data->rx_buf_claimed = false;
	/* The DMA residue still counts from the start of the buffer */
	dev_data->previous_rx_processed_len = rx_processed;
	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		restart_rx_timeout(dev_data);
		reschedule_rx_timeout(dev_data);
	}

	/* Restart the DMA */
	dma_start(dev_config->dma.dev, dev_config->dma.rx_channel);

	uart_mchp_usart_notify_rx_processed(dev_data, rx_processed);

	irq_unlock(key);
}

/*
 * Notify the application to release the receive buffer
 */
static void uart_mchp_usart_notify_buf_released(struct uart_mchp_usart_data *dev_data,
						struct uart_event *rx_rdy_evt)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf = {.buf = rx_rdy_evt->data.rx.buf},
	};

	/* Report the data held by the finished buffer, then release it */
	async_user_callback(dev_data, rx_rdy_evt);
	async_user_callback(dev_data, &evt);
}

/*
 * End the reception
 */
static void uart_mchp_usart_end_reception(struct uart_mchp_usart_data *dev_data)
{
	struct uart_event buf_rel_evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf = {.buf = dev_data->rx_next_buf},
	};
	struct uart_event rx_dis_evt = {
		.type = UART_RX_DISABLED,
	};

	dev_data->rx_buf = NULL;
	dev_data->rx_len = 0u;

	/* Cancel any previous timeout who might still be ticking */
	k_work_cancel_delayable(&dev_data->rx_timeout_work);

	/* Release any next buffer that was queued but never used */
	if (dev_data->rx_next_len != 0u) {
		dev_data->rx_next_buf = NULL;
		dev_data->rx_next_len = 0u;

		async_user_callback(dev_data, &buf_rel_evt);
	}

	async_user_callback(dev_data, &rx_dis_evt);
}

/*
 * Completes the RX buffer currently pointed to by dev_data->rx_buf: hands
 * off to the buffer queued via uart_mchp_usart_rx_buf_rsp() (if any) or ends the
 * reception, then notifies the application of the data the finished buffer
 * held. The notifications are queued and delivered from the system workqueue.
 * The caller must hold irq_lock().
 */
static void uart_mchp_usart_rx_complete_buffer(struct uart_mchp_usart_data *dev_data)
{
	const struct device *dev = dev_data->dev;
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;
	struct uart_event rx_rdy_evt = {
		.type = UART_RX_RDY,
		.data.rx.buf = dev_data->rx_buf,
		.data.rx.offset = dev_data->rx_processed_len,
		.data.rx.len = dev_data->rx_len - dev_data->rx_processed_len,
	};
	int ret = 0;

	/* No next buffer, so end the transfer */
	if (dev_data->rx_next_len == 0u) {
		uart_mchp_usart_notify_buf_released(dev_data, &rx_rdy_evt);
		uart_mchp_usart_end_reception(dev_data);
		return;
	}

	ret = dma_reload(dev_config->dma.dev, dev_config->dma.rx_channel, (uint32_t)&regs->US_RHR,
			 (uint32_t)dev_data->rx_next_buf, dev_data->rx_next_len);
	if (ret != 0) {
		LOG_ERR("Unable to reload the DMA: %i", ret);
		uart_mchp_usart_notify_buf_released(dev_data, &rx_rdy_evt);
		uart_mchp_usart_end_reception(dev_data);
		return;
	}

	/* Restart the DMA immediately; see uart_mchp_usart_rx_enable() for why. */
	dma_start(dev_config->dma.dev, dev_config->dma.rx_channel);

	/* Move to next buffer */
	dev_data->rx_buf_claimed = false;
	dev_data->rx_buf = dev_data->rx_next_buf;
	dev_data->rx_len = dev_data->rx_next_len;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0u;
	dev_data->rx_processed_len = 0u;
	dev_data->previous_rx_processed_len = 0u;

	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		/* Refresh the timeout timestamp */
		restart_rx_timeout(dev_data);
	}

	uart_mchp_usart_notify_buf_released(dev_data, &rx_rdy_evt);
	uart_mchp_usart_request_rx_buf(dev_data);
}

/*
 * UART DMA TX done handler.
 */
static void uart_mchp_usart_tx_done_callback(const struct device *dma_dev, void *arg, uint32_t id,
					     int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);

	struct uart_mchp_usart_data *dev_data = (struct uart_mchp_usart_data *)arg;
	const struct uart_mchp_usart_config *dev_config = dev_data->dev->config;
	usart_registers_t *const regs = dev_config->regs;
	struct dma_status dma_stat;
	unsigned int key = irq_lock();
	struct uart_event evt = {0};

	/* Success */
	if (error_code >= 0) {
		/* Enable the TXEMPTY interrupt */
		regs->US_IER = US_IER_USART_TXEMPTY_Msk;
	}
	/* Error condition */
	else {
		LOG_ERR("DMA operation aborted: %i", error_code);

		evt.type = UART_TX_ABORTED;
		evt.data.tx.buf = dev_data->tx_buf;

		if (dma_get_status(dev_config->dma.dev, dev_config->dma.tx_channel, &dma_stat) ==
		    0) {
			evt.data.tx.len = dev_data->tx_len - dma_stat.pending_length;
		}

		dev_data->tx_buf = NULL;
		dev_data->tx_len = 0u;

		async_user_callback(dev_data, &evt);
	}

	k_work_cancel_delayable(&dev_data->tx_timeout_work);

	irq_unlock(key);
}

/*
 * UART DMA RX done handler.
 */
static void uart_mchp_usart_rx_done_callback(const struct device *dma_dev, void *arg, uint32_t id,
					     int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct uart_mchp_usart_data *dev_data = (struct uart_mchp_usart_data *)arg;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0u) {
		irq_unlock(key);
		return;
	}

	/* The timeout already completed this transaction */
	if (dev_data->rx_buf_claimed) {
		irq_unlock(key);
		return;
	}
	dev_data->rx_buf_claimed = true;

	uart_mchp_usart_rx_complete_buffer(dev_data);

	irq_unlock(key);
}

#endif /* CONFIG_UART_MCHP_ASYNC */

/*
 * Initializes the driver
 */
static int uart_mchp_usart_init(const struct device *dev)
{
	const struct uart_mchp_usart_config *dev_config = dev->config;
	int ret = 0;
#if defined(CONFIG_UART_MCHP_ASYNC)
	usart_registers_t *const regs = dev_config->regs;
	int dma_ch = 0;
	int dma_ch_request = 0;
	struct uart_mchp_usart_data *dev_data = dev->data;

	dev_data->dev = dev;
#endif /* CONFIG_UART_MCHP_ASYNC */

	/* Enable peripheral clock */
	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Unable to activate peripheral clock");
		return ret;
	}

	if (dev_config->baudrate_clock_source == BAUDRATE_CLOCK_SOURCE_PCK) {
		ret = clock_control_on(dev_config->clock, PROGRAMMABLE_CLOCK);
		if (ret != 0) {
			LOG_ERR("Unable to activate PCK4: %i", ret);
			return -EINVAL;
		}
	}

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Unable to configure pins");
		return ret;
	}

#if defined(CONFIG_UART_MCHP_ASYNC)
	if (!device_is_ready(dev_config->dma.dev)) {
		return -ENODEV;
	}

	k_msgq_init(&dev_data->evt_msgq, (char *)dev_data->evt_buf, sizeof(struct uart_event),
		    ARRAY_SIZE(dev_data->evt_buf));
	k_work_init(&dev_data->evt_work, uart_mchp_usart_evt_work_handler);
	k_work_init_delayable(&dev_data->tx_timeout_work, uart_mchp_usart_tx_timeout_callback);
	k_work_init_delayable(&dev_data->rx_timeout_work, uart_mchp_usart_rx_timeout_callback);

	dma_ch = dev_config->dma.tx_channel;
	dma_ch_request = dma_request_channel(dev_config->dma.dev, (void *)&dma_ch);

	if ((dev_config->dma.tx_channel != 0xFFU) &&
	    (dma_ch_request == dev_config->dma.tx_channel)) {
		struct dma_config dma_cfg = {0};
		struct dma_block_config dma_blk = {0};

		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.source_burst_length = 1;
		dma_cfg.dest_burst_length = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = uart_mchp_usart_tx_done_callback;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = dev_config->dma.tx_perid;

		dma_blk.block_size = 1;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_blk.dest_address = (uint32_t)&regs->US_THR;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		ret = dma_config(dev_config->dma.dev, dev_config->dma.tx_channel, &dma_cfg);
		if (ret != 0) {
			return ret;
		}
	} else {
		LOG_WRN("UART TX DMA unavailable (configured=%d, allocated=%d)",
			dev_config->dma.tx_channel, dma_ch_request);
	}

	dma_ch = dev_config->dma.rx_channel;
	dma_ch_request = dma_request_channel(dev_config->dma.dev, (void *)&dma_ch);

	if ((dev_config->dma.rx_channel != 0xFFU) &&
	    (dma_ch_request == dev_config->dma.rx_channel)) {
		ret = uart_mchp_usart_configure_rx_dma(dev_config, dev_data);
		if (ret != 0) {
			return ret;
		}
	} else {
		LOG_WRN("UART RX DMA unavailable (configured=%d, allocated=%d)",
			dev_config->dma.rx_channel, dma_ch_request);
	}

#endif /* CONFIG_UART_MCHP_ASYNC */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_MCHP_ASYNC)
	dev_config->irq_config(dev);
#endif /* defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_MCHP_ASYNC) */

	return uart_mchp_usart_configure(dev, &dev_config->uart_config);
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_MCHP_ASYNC)
static void uart_mchp_usart_isr(const struct device *dev)
{
	struct uart_mchp_usart_data *dev_data = dev->data;

#if defined(CONFIG_UART_MCHP_ASYNC)
	const struct uart_mchp_usart_config *dev_config = dev->config;
	usart_registers_t *const regs = dev_config->regs;
	struct uart_event evt = {0};

	if (((regs->US_CSR & US_CSR_USART_TXEMPTY_Msk) != 0u) &&
	    ((regs->US_IMR & US_IMR_USART_TXEMPTY_Msk) != 0u)) {
		regs->US_IDR = US_IDR_USART_TXEMPTY_Msk;

		evt.type = UART_TX_DONE;
		evt.data.tx.buf = dev_data->tx_buf;
		evt.data.tx.len = dev_data->tx_len;

		dev_data->tx_buf = NULL;
		dev_data->tx_len = 0u;

		async_user_callback(dev_data, &evt);
	}
#endif /* CONFIG_UART_MCHP_ASYNC */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	if (dev_data->irq_cb != NULL) {
		dev_data->irq_cb(dev, dev_data->irq_cb_data);
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
}

#define IRQ_CONFIG_FUNC(n)                                                                         \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_mchp_usart_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
#define IRQ_CONFIG_INIT(n) .irq_config = irq_config_##n,
#else
#define IRQ_CONFIG_FUNC(n)
#define IRQ_CONFIG_INIT(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_MCHP_ASYNC */

#if defined(CONFIG_UART_MCHP_ASYNC)
#define DMA_CONFIG_INIT(n)                                                                         \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                                                \
		(                                                                                  \
			.dma.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                \
			.dma.tx_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),               \
			.dma.tx_perid = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid),                   \
			.dma.rx_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),               \
			.dma.rx_perid = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),                   \
		),                                                                                 \
		(                                                                                  \
			.dma.dev = NULL,                                                           \
		)                                                                                  \
	)
#else
#define DMA_CONFIG_INIT(n)
#endif /* CONFIG_UART_MCHP_ASYNC */

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	IRQ_CONFIG_FUNC(n);                                                                        \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct uart_mchp_usart_config mchp_usart_##n##_config = {                     \
		.regs = (usart_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.baudrate_clock_source = DT_INST_ENUM_IDX(n, baudrate_clock_source),               \
		.rs485_tx_guard_time = DT_INST_PROP_OR(n, rs485_tx_guard_time, 0),                 \
		.channel_mode = DT_INST_ENUM_IDX(n, channel_mode),                                 \
		.uart_config =                                                                     \
			{                                                                          \
				.baudrate = DT_INST_PROP(n, current_speed),                        \
				.parity = DT_INST_ENUM_IDX(n, parity),                             \
				.stop_bits = DT_INST_ENUM_IDX(n, stop_bits),                       \
				.data_bits = DT_INST_ENUM_IDX(n, data_bits),                       \
			},                                                                         \
		IRQ_CONFIG_INIT(n) DMA_CONFIG_INIT(n)};                                            \
                                                                                                   \
	static struct uart_mchp_usart_data mchp_usart_##n##_data = {};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, uart_mchp_usart_init, NULL, &mchp_usart_##n##_data,               \
			      &mchp_usart_##n##_config, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, \
			      &uart_mchp_usart_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
