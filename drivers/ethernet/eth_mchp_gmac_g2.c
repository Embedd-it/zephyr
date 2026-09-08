/*
 * SPDX-FileCopyrightText: Copyright (c) 2016 Piotr Mienkowski
 * SPDX-FileCopyrightText: Copyright (c) 2018 Antmicro Ltd
 * SPDX-FileCopyrightText: Copyright (c) 2023 Gerson Fernando Budke
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip MCU family Ethernet MAC (GMAC) driver.
 *
 * This is a zero-copy networking implementation of an Ethernet driver. To
 * prepare for the incoming frames the driver will permanently reserve a defined
 * amount of RX data net buffers when the interface is brought up and thus
 * reduce the total amount of RX data net buffers available to the application.
 *
 * Limitations:
 * - one shot PHY setup, no support for PHY disconnect/reconnect
 * - no statistics collection
 */

#define DT_DRV_COMPAT microchip_gmac_g2_eth

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdbool.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include "eth_mchp_gmac_g2_priv.h"

#include "eth.h"

#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/net/gptp.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_mchp_gmac_g2, CONFIG_ETHERNET_LOG_LEVEL);

#if defined(__DCACHE_PRESENT) || defined(CONFIG_DCACHE)
static bool eth_mchp_gmac_dcache_enabled;

static inline void eth_mchp_gmac_dcache_is_enabled(void)
{
#if defined(__DCACHE_PRESENT)
	eth_mchp_gmac_dcache_enabled = (SCB->CCR & SCB_CCR_DC_Msk);
#else
	eth_mchp_gmac_dcache_enabled = true;
#endif
}
static inline void eth_mchp_gmac_dcache_invalidate(uint32_t addr, uint32_t size)
{
	if (!eth_mchp_gmac_dcache_enabled) {
		return;
	}

	/* Make sure it is aligned to 32B */
	uint32_t start_addr = addr & (uint32_t)~(GMAC_DCACHE_ALIGNMENT - 1);
	uint32_t size_full = size + addr - start_addr;

	sys_cache_data_invd_range((uint32_t *)start_addr, size_full);
}

static inline void eth_mchp_gmac_dcache_clean(uint32_t addr, uint32_t size)
{
	if (!eth_mchp_gmac_dcache_enabled) {
		return;
	}

	/* Make sure it is aligned to 32B */
	uint32_t start_addr = addr & (uint32_t)~(GMAC_DCACHE_ALIGNMENT - 1);
	uint32_t size_full = size + addr - start_addr;

	sys_cache_data_flush_range((uint32_t *)start_addr, size_full);
}
#else
#define eth_mchp_gmac_dcache_is_enabled()
#define eth_mchp_gmac_dcache_invalidate(addr, size)
#define eth_mchp_gmac_dcache_clean(addr, size)
#endif

/*
 * Verify Kconfig configuration
 */
/* No need to verify things for unit tests */
#if !defined(CONFIG_NET_TEST)
#if CONFIG_NET_BUF_DATA_SIZE * CONFIG_ETH_MCHP_GMAC_G2_BUF_RX_COUNT < GMAC_FRAME_SIZE_MAX
#error CONFIG_NET_BUF_DATA_SIZE * CONFIG_ETH_MCHP_GMAC_G2_BUF_RX_COUNT is \
	not large enough to hold a full frame
#endif

#if CONFIG_NET_BUF_DATA_SIZE * (CONFIG_NET_BUF_RX_COUNT - CONFIG_ETH_MCHP_GMAC_G2_BUF_RX_COUNT) <  \
	GMAC_FRAME_SIZE_MAX
#error (CONFIG_NET_BUF_RX_COUNT - CONFIG_ETH_MCHP_GMAC_G2_BUF_RX_COUNT) * \
	CONFIG_NET_BUF_DATA_SIZE are not large enough to hold a full frame
#endif

#if CONFIG_NET_BUF_DATA_SIZE & 0x3F
#pragma message "CONFIG_NET_BUF_DATA_SIZE should be a multiple of 64 bytes "                       \
		"due to the granularity of RX DMA"
#endif

#if (CONFIG_ETH_MCHP_GMAC_G2_BUF_RX_COUNT + 1) * GMAC_ACTIVE_QUEUE_NUM > CONFIG_NET_BUF_RX_COUNT
#error Not enough RX buffers to allocate descriptors for each HW queue
#endif
#endif /* !CONFIG_NET_TEST */

/* if GMAC_UR_MIM_RGMII (new for sama7g5) is defined, the media interface mode
 * supported are: mii, rmii and gmii. Otherwise mii and rmii are supported.
 */
#if !defined(GMAC_UR_MIM_RGMII)
#define MCHP_GMAC_PHY_CONNECTION_TYPE_MAX 1
#else
#define MCHP_GMAC_PHY_CONNECTION_TYPE_MAX 3
#endif

/* RX descriptors list */
#define DEFN_RX_DESC(n)                                                                            \
	static struct gmac_desc rx_desc##n##_que[PRIORITY_QUEUE_RX_DESC_COUNT] __nocache           \
		__aligned(GMAC_DESC_ALIGNMENT);

/* TX descriptors list */
#define DEFN_TX_DESC(n)                                                                            \
	static struct gmac_desc tx_desc##n##_que[PRIORITY_QUEUE_TX_DESC_COUNT] __nocache           \
		__aligned(GMAC_DESC_ALIGNMENT);

/* RX buffer accounting list */
#define DEFN_RX_FRAG(n) static struct net_buf *rx_frag_list##n##_que[PRIORITY_QUEUE_RX_DESC_COUNT];

#if GMAC_MULTIPLE_TX_PACKETS == 1
/* TX buffer accounting list */
#define DEFN_TX_FRAG(n) static struct net_buf *tx_frag_list##n##_que[PRIORITY_QUEUE_TX_DESC_COUNT];

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
#define NET_PKT_PER_QUE (CONFIG_NET_PKT_TX_COUNT + 1)
/* TX frames accounting list */
#define DEFN_TX_FRAME(n)                                                                           \
	static struct net_pkt                                                                      \
		*tx_frame_list##n##_que[NET_PKT_PER_QUE * (GMAC_ACTIVE_PRIORITY_QUEUE_NUM + 1)];
#else
#define DEFN_TX_FRAME(n)
#endif /* CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK */
#else
#define DEFN_TX_FRAG(n)
#define DEFN_TX_FRAME(n)
#endif /* GMAC_MULTIPLE_TX_PACKETS */

#define MCHP_GMAC_LISTS_DEFN(n)                                                                    \
	DEFN_RX_DESC(n)                                                                            \
	DEFN_TX_DESC(n)                                                                            \
	DEFN_RX_FRAG(n)                                                                            \
	DEFN_TX_FRAG(n)                                                                            \
	DEFN_TX_FRAME(n)

#define MODULO_INC(val, max)                                                                       \
	{                                                                                          \
		val = (++val < max) ? val : 0;                                                     \
	}

static int eth_mchp_gmac_rx_descriptors_init(gmac_registers_t *const gmac,
					     struct gmac_queue *queue);
static void eth_mchp_gmac_tx_descriptors_init(gmac_registers_t *const gmac,
					      struct gmac_queue *queue);
static int eth_mchp_gmac_nonpriority_queue_init(gmac_registers_t *const gmac,
						struct gmac_queue *queue);

#if GMAC_PRIORITY_QUEUE_NUM >= 1
static inline void eth_mchp_gmac_set_receive_buf_queue_pointer(gmac_registers_t *const gmac,
							       struct gmac_queue *queue)
{
	/* Set Receive Buffer Queue Pointer Register */
	if (queue->que_idx == GMAC_QUE_0) {
		gmac->GMAC_RBQB = (uint32_t)queue->rx_desc_list.buf;
	} else {
		gmac->GMAC_RBQBAPQ[queue->que_idx - 1] = (uint32_t)queue->rx_desc_list.buf;
	}
}

static inline void eth_mchp_gmac_disable_all_priority_queue_interrupt(gmac_registers_t *const gmac)
{
	uint32_t idx;

	for (idx = 0; idx < GMAC_PRIORITY_QUEUE_NUM; idx++) {
		gmac->GMAC_IDRPQ[idx] = UINT32_MAX;
		(void)gmac->GMAC_ISRPQ[idx];
	}
}

static int eth_mchp_gmac_priority_queue_init(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
	int ret;
	int queue_index;

	__ASSERT_NO_MSG(queue->rx_desc_list.len > 0);
	__ASSERT_NO_MSG(queue->tx_desc_list.len > 0);
	__ASSERT(!((uint32_t)queue->rx_desc_list.buf & ~GMAC_RBQB_ADDR_Msk),
		 "RX descriptors have to be word aligned");
	__ASSERT(!((uint32_t)queue->tx_desc_list.buf & ~GMAC_TBQB_ADDR_Msk),
		 "TX descriptors have to be word aligned");

	/* Extract queue index for easier referencing */
	queue_index = queue->que_idx - 1;

	/* Setup descriptor lists */
	ret = eth_mchp_gmac_rx_descriptors_init(gmac, queue);
	if (ret < 0) {
		return ret;
	}

	eth_mchp_gmac_tx_descriptors_init(gmac, queue);

#if GMAC_MULTIPLE_TX_PACKETS == 0
	k_sem_init(&queue->tx_sem, 0, 1);
#else
	k_sem_init(&queue->tx_desc_sem, queue->tx_desc_list.len - 1, queue->tx_desc_list.len - 1);
#endif

	/* Setup RX buffer size for DMA */
	gmac->GMAC_RBSRPQ[queue_index] = GMAC_RBSRPQ_RBS(CONFIG_NET_BUF_DATA_SIZE >> 6);

	/* Set Receive Buffer Queue Pointer Register */
	gmac->GMAC_RBQBAPQ[queue_index] = (uint32_t)queue->rx_desc_list.buf;
	/* Set Transmit Buffer Queue Pointer Register */
	gmac->GMAC_TBQBAPQ[queue_index] = (uint32_t)queue->tx_desc_list.buf;

	/* Enable RX/TX completion and error interrupts */
	gmac->GMAC_IERPQ[queue_index] = GMAC_INTPQ_EN_FLAGS;

	queue->err_rx_frames_dropped = 0U;
	queue->err_rx_flushed_count = 0U;
	queue->err_tx_flushed_count = 0U;

	LOG_INF("ethernet@%x Queue %d activated", (uint32_t)gmac, queue->que_idx);

	return 0;
}

static int eth_mchp_gmac_priority_queue_init_as_idle(gmac_registers_t *const gmac,
						     struct gmac_queue *queue)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct gmac_desc_list *tx_desc_list = &queue->tx_desc_list;

	__ASSERT(!((uint32_t)rx_desc_list->buf & ~GMAC_RBQB_ADDR_Msk),
		 "RX descriptors have to be word aligned");
	__ASSERT(!((uint32_t)tx_desc_list->buf & ~GMAC_TBQB_ADDR_Msk),
		 "TX descriptors have to be word aligned");
	__ASSERT((rx_desc_list->len == 1U) && (tx_desc_list->len == 1U),
		 "Priority queues are currently not supported, descriptor "
		 "list has to have a single entry");

	/* Setup RX descriptor lists */
	/* Take ownership from GMAC and set the wrap bit */
	rx_desc_list->buf[0].w0 = GMAC_RXW0_WRAP;
	rx_desc_list->buf[0].w1 = 0U;
	/* Setup TX descriptor lists */
	tx_desc_list->buf[0].w0 = 0U;
	/* Take ownership from GMAC and set the wrap bit */
	tx_desc_list->buf[0].w1 = GMAC_TXW1_USED | GMAC_TXW1_WRAP;

	/* Set Receive Buffer Queue Pointer Register */
	gmac->GMAC_RBQBAPQ[queue->que_idx - 1] = (uint32_t)rx_desc_list->buf;
	/* Set Transmit Buffer Queue Pointer Register */
	gmac->GMAC_TBQBAPQ[queue->que_idx - 1] = (uint32_t)tx_desc_list->buf;

	LOG_INF("ethernet@%x Queue %d set to idle", (uint32_t)gmac, queue->que_idx);

	return 0;
}

static int eth_mchp_gmac_queue_init(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
	if (queue->que_idx == GMAC_QUE_0) {
		return eth_mchp_gmac_nonpriority_queue_init(gmac, queue);
	} else if (queue->que_idx <= GMAC_ACTIVE_PRIORITY_QUEUE_NUM) {
		return eth_mchp_gmac_priority_queue_init(gmac, queue);
	} else {
		return eth_mchp_gmac_priority_queue_init_as_idle(gmac, queue);
	}
}

#else

static inline void eth_mchp_gmac_set_receive_buf_queue_pointer(gmac_registers_t *const gmac,
							       struct gmac_queue *queue)
{
	gmac->GMAC_RBQB = (uint32_t)queue->rx_desc_list.buf;
}

static int eth_mchp_gmac_queue_init(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
	return eth_mchp_gmac_nonpriority_queue_init(gmac, queue);
}

#define eth_mchp_gmac_disable_all_priority_queue_interrupt(gmac)

#endif

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
static int eth_mchp_gmac_setup_qav(gmac_registers_t *const gmac, int queue_id, bool enable);

static inline void eth_mchp_gmac_init_qav(gmac_registers_t *const gmac)
{
	uint32_t idx;

	for (idx = GMAC_QUE_1; idx <= GMAC_ACTIVE_PRIORITY_QUEUE_NUM; idx++) {
		eth_mchp_gmac_setup_qav(gmac, idx, true);
	}
}

#else

#define eth_mchp_gmac_init_qav(gmac)

#endif

/*
 * Free pre-reserved RX buffers
 */
static void eth_mchp_gmac_free_rx_bufs(struct net_buf **rx_frag_list, uint16_t len)
{
	for (int i = 0; i < len; i++) {
		if (rx_frag_list[i]) {
			net_buf_drop(&rx_frag_list[i]);
		}
	}
}

/*
 * Set MAC Address for frame filtering logic
 */
static void eth_mchp_gmac_mac_addr_set(gmac_registers_t *const gmac, uint8_t index,
				       uint8_t mac_addr[6])
{
	__ASSERT(index < 4, "index has to be in the range 0..3");

	gmac->GMAC_SA[index].GMAC_SAB =
		(mac_addr[3] << 24) | (mac_addr[2] << 16) | (mac_addr[1] << 8) | (mac_addr[0]);
	gmac->GMAC_SA[index].GMAC_SAT = (mac_addr[5] << 8) | (mac_addr[4]);
}

/*
 * Initialize RX descriptor list
 */
static int eth_mchp_gmac_rx_descriptors_init(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct net_buf **rx_frag_list = queue->rx_frag_list;
	struct net_buf *rx_buf;
	uint8_t *rx_buf_addr;

	__ASSERT_NO_MSG(rx_frag_list);

	rx_desc_list->tail = 0U;

	for (int i = 0; i < rx_desc_list->len; i++) {
		rx_buf = net_pkt_get_reserve_rx_data(CONFIG_NET_BUF_DATA_SIZE, K_NO_WAIT);
		if (rx_buf == NULL) {
			eth_mchp_gmac_free_rx_bufs(rx_frag_list, rx_desc_list->len);
			LOG_ERR("Failed to reserve data net buffers");
			return -ENOBUFS;
		}

		rx_frag_list[i] = rx_buf;

		rx_buf_addr = rx_buf->data;
		__ASSERT(!((uint32_t)rx_buf_addr & ~GMAC_RXW0_ADDR),
			 "Misaligned RX buffer address");
		__ASSERT(rx_buf->size == CONFIG_NET_BUF_DATA_SIZE,
			 "Incorrect length of RX data buffer");
		/* Give ownership to GMAC and remove the wrap bit */
		rx_desc_list->buf[i].w0 = (uint32_t)rx_buf_addr & GMAC_RXW0_ADDR;
		rx_desc_list->buf[i].w1 = 0U;
	}

	/* Set the wrap bit on the last descriptor */
	rx_desc_list->buf[rx_desc_list->len - 1U].w0 |= GMAC_RXW0_WRAP;

	return 0;
}

/*
 * Initialize TX descriptor list
 */
static void eth_mchp_gmac_tx_descriptors_init(gmac_registers_t *const gmac,
					      struct gmac_queue *queue)
{
	struct gmac_desc_list *tx_desc_list = &queue->tx_desc_list;

	tx_desc_list->head = 0U;
	tx_desc_list->tail = 0U;

	for (int i = 0; i < tx_desc_list->len; i++) {
		tx_desc_list->buf[i].w0 = 0U;
		tx_desc_list->buf[i].w1 = GMAC_TXW1_USED;
	}

	/* Set the wrap bit on the last descriptor */
	tx_desc_list->buf[tx_desc_list->len - 1U].w1 |= GMAC_TXW1_WRAP;

#if GMAC_MULTIPLE_TX_PACKETS == 1
	/* Reset TX frame list */
	sys_ringq_reset(&queue->tx_frag_list);
#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
	sys_ringq_reset(&queue->tx_frames);
#endif
#endif
}

#if defined(CONFIG_NET_GPTP)
static struct gptp_hdr *eth_mchp_gmac_check_gptp_msg(struct net_if *iface, struct net_pkt *pkt,
						     bool is_tx)
{
	uint8_t *msg_start = net_pkt_data(pkt);
	struct gptp_hdr *gptp_hdr;
	int eth_hlen;
	struct net_eth_hdr *hdr;

	hdr = (struct net_eth_hdr *)msg_start;
	if (net_ntohs(hdr->type) != NET_ETH_PTYPE_PTP) {
		return NULL;
	}

	eth_hlen = sizeof(struct net_eth_hdr);

	/* In TX, the first net_buf contains the Ethernet header
	 * and the actual gPTP header is in the second net_buf.
	 * In RX, the Ethernet header + other headers are in the
	 * first net_buf.
	 */
	if (is_tx) {
		if (pkt->frags->frags == NULL) {
			return false;
		}

		gptp_hdr = (struct gptp_hdr *)pkt->frags->frags->data;
	} else {
		gptp_hdr = (struct gptp_hdr *)(pkt->frags->data + eth_hlen);
	}

	return gptp_hdr;
}

static bool eth_mchp_gmac_need_timestamping(struct gptp_hdr *hdr)
{
	switch (hdr->message_type) {
	case GPTP_SYNC_MESSAGE:
	case GPTP_PATH_DELAY_RESP_MESSAGE:
		return true;
	default:
		return false;
	}
}

static void eth_mchp_gmac_update_pkt_priority(struct gptp_hdr *hdr, struct net_pkt *pkt)
{
	if (GPTP_IS_EVENT_MSG(hdr->message_type)) {
		net_pkt_set_priority(pkt, NET_PRIORITY_CA);
	} else {
		net_pkt_set_priority(pkt, NET_PRIORITY_IC);
	}
}

static inline struct net_ptp_time eth_mchp_gmac_get_ptp_event_rx_ts(gmac_registers_t *const gmac)
{
	struct net_ptp_time ts;

	ts.second = ((uint64_t)(gmac->GMAC_EFRSH & 0xffff) << 32) | gmac->GMAC_EFRSL;
	ts.nanosecond = gmac->GMAC_EFRN;

	return ts;
}

static inline struct net_ptp_time
eth_mchp_gmac_get_ptp_peer_event_rx_ts(gmac_registers_t *const gmac)
{
	struct net_ptp_time ts;

	ts.second = ((uint64_t)(gmac->GMAC_PEFRSH & 0xffff) << 32) | gmac->GMAC_PEFRSL;
	ts.nanosecond = gmac->GMAC_PEFRN;

	return ts;
}

static inline struct net_ptp_time eth_mchp_gmac_get_ptp_event_tx_ts(gmac_registers_t *const gmac)
{
	struct net_ptp_time ts;

	ts.second = ((uint64_t)(gmac->GMAC_EFTSH & 0xffff) << 32) | gmac->GMAC_EFTSL;
	ts.nanosecond = gmac->GMAC_EFTN;

	return ts;
}

static inline struct net_ptp_time
eth_mchp_gmac_get_ptp_peer_event_tx_ts(gmac_registers_t *const gmac)
{
	struct net_ptp_time ts;

	ts.second = ((uint64_t)(gmac->GMAC_PEFTSH & 0xffff) << 32) | gmac->GMAC_PEFTSL;
	ts.nanosecond = gmac->GMAC_PEFTN;

	return ts;
}

static inline struct net_ptp_time eth_mchp_gmac_get_current_ts(gmac_registers_t *const gmac)
{
	struct net_ptp_time ts;

	ts.second = ((uint64_t)(gmac->GMAC_TSH & 0xffff) << 32) | gmac->GMAC_TSL;
	ts.nanosecond = gmac->GMAC_TN;

	return ts;
}

static inline void eth_mchp_gmac_timestamp_tx_pkt(gmac_registers_t *const gmac,
						  struct gptp_hdr *hdr, struct net_pkt *pkt)
{
	struct net_ptp_time timestamp;

	if (hdr) {
		switch (hdr->message_type) {
		case GPTP_SYNC_MESSAGE:
			timestamp = eth_mchp_gmac_get_ptp_event_tx_ts(gmac);
			break;
		default:
			timestamp = eth_mchp_gmac_get_ptp_peer_event_tx_ts(gmac);
			break;
		}
	} else {
		timestamp = eth_mchp_gmac_get_current_ts(gmac);
	}

	net_pkt_set_timestamp(pkt, &timestamp);
}

static inline void eth_mchp_gmac_timestamp_rx_pkt(gmac_registers_t *const gmac,
						  struct gptp_hdr *hdr, struct net_pkt *pkt)
{
	struct net_ptp_time timestamp;

	if (hdr) {
		switch (hdr->message_type) {
		case GPTP_SYNC_MESSAGE:
			timestamp = eth_mchp_gmac_get_ptp_event_rx_ts(gmac);
			break;
		default:
			timestamp = eth_mchp_gmac_get_ptp_peer_event_rx_ts(gmac);
			break;
		}
	} else {
		timestamp = eth_mchp_gmac_get_current_ts(gmac);
	}

	net_pkt_set_timestamp(pkt, &timestamp);
	net_pkt_set_rx_timestamping(pkt, true);
}

#endif

static inline struct net_if *eth_mchp_gmac_get_iface(struct eth_mchp_dev_data *ctx)
{
	return ctx->iface;
}

/*
 * Process successfully sent packets
 */
static void eth_mchp_gmac_tx_completed(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
#if GMAC_MULTIPLE_TX_PACKETS == 0
	k_sem_give(&queue->tx_sem);
#else
	struct gmac_desc_list *tx_desc_list = &queue->tx_desc_list;
	struct gmac_desc *tx_desc;
	struct net_buf *frag;
#if defined(CONFIG_NET_GPTP)
	struct net_pkt *pkt;
	struct gptp_hdr *hdr;
	struct eth_mchp_dev_data *dev_data =
		CONTAINER_OF(queue, struct eth_mchp_dev_data, queue_list[queue->que_idx]);
#endif

	__ASSERT(tx_desc_list->buf[tx_desc_list->tail].w1 & GMAC_TXW1_USED,
		 "first buffer of a frame is not marked as own by GMAC");

	while (tx_desc_list->tail != tx_desc_list->head) {

		tx_desc = &tx_desc_list->buf[tx_desc_list->tail];
		MODULO_INC(tx_desc_list->tail, tx_desc_list->len);
		k_sem_give(&queue->tx_desc_sem);

		/* Release net buffer to the buffer pool */
		if (sys_ringq_get(&queue->tx_frag_list, &frag)) {
			__ASSERT(false, "Failed to get frag from tx_frag_list ringq");
		}
		net_pkt_frag_unref(frag);
		LOG_DBG("Dropping frag %p", frag);

		if (tx_desc->w1 & GMAC_TXW1_LASTBUFFER) {
#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
			/* Release net packet to the packet pool */
			if (sys_ringq_get(&queue->tx_frames, &pkt)) {
				__ASSERT(false, "Failed to get pkt from tx_frames ringq");
			}

#if defined(CONFIG_NET_GPTP)
			hdr = eth_mchp_gmac_check_gptp_msg(eth_mchp_gmac_get_iface(dev_data), pkt,
							   true);

			eth_mchp_gmac_timestamp_tx_pkt(gmac, hdr, pkt);

			if (hdr && eth_mchp_gmac_need_timestamping(hdr)) {
				net_if_add_tx_timestamp(pkt);
			}
#endif
			net_pkt_unref(pkt);
			LOG_DBG("Dropping pkt %p", pkt);
#endif
			break;
		}
	}
#endif
}

/*
 * Reset TX queue when errors are detected
 */
static void eth_mchp_gmac_tx_error_handler(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
#if GMAC_MULTIPLE_TX_PACKETS == 1
	struct net_buf *frag;
	struct sys_ringq *tx_frag_list = &queue->tx_frag_list;
#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
	struct net_pkt *pkt;
	struct sys_ringq *tx_frames = &queue->tx_frames;
#endif
#endif

	queue->err_tx_flushed_count++;

	/* Stop transmission, clean transmit pipeline and control registers */
	gmac->GMAC_NCR &= ~GMAC_NCR_TXEN_Msk;

#if GMAC_MULTIPLE_TX_PACKETS == 1
	/* Free all frag resources in the TX path */
	while (!sys_ringq_empty(tx_frag_list)) {
		/* Release net buffer to the buffer pool */
		sys_ringq_get(tx_frag_list, &frag);
		net_pkt_frag_unref(frag);
		LOG_DBG("Dropping frag %p", frag);
	}

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
	/* Free all pkt resources in the TX path */
	while (!sys_ringq_empty(tx_frames)) {
		/* Release net packet to the packet pool */
		sys_ringq_get(tx_frames, &pkt);
		net_pkt_unref(pkt);
		LOG_DBG("Dropping pkt %p", pkt);
	}
#endif

	/* Reinitialize TX descriptor list */
	k_sem_reset(&queue->tx_desc_sem);
	for (int i = 0; i < queue->tx_desc_list.len - 1; i++) {
		k_sem_give(&queue->tx_desc_sem);
	}
#endif
	eth_mchp_gmac_tx_descriptors_init(gmac, queue);

#if GMAC_MULTIPLE_TX_PACKETS == 0
	/* Reinitialize TX mutex */
	k_sem_give(&queue->tx_sem);
#endif

	/* Restart transmission */
	gmac->GMAC_NCR |= GMAC_NCR_TXEN_Msk;
}

/*
 * Clean RX queue, any received data still stored in the buffers is abandoned.
 */
static void eth_mchp_gmac_rx_error_handler(gmac_registers_t *const gmac, struct gmac_queue *queue)
{
	queue->err_rx_flushed_count++;

	/* Stop reception */
	gmac->GMAC_NCR &= ~GMAC_NCR_RXEN_Msk;

	queue->rx_desc_list.tail = 0U;

	for (int i = 0; i < queue->rx_desc_list.len; i++) {
		queue->rx_desc_list.buf[i].w1 = 0U;
		queue->rx_desc_list.buf[i].w0 &= ~GMAC_RXW0_OWNERSHIP;
	}

	eth_mchp_gmac_set_receive_buf_queue_pointer(gmac, queue);

	/* Restart reception */
	gmac->GMAC_NCR |= GMAC_NCR_RXEN_Msk;
}

/*
 * Set MCK to MDC clock divisor.
 *
 * According to 802.3 MDC should be less then 2.5 MHz.
 */
static int eth_mchp_gmac_get_mck_clock_divisor(uint32_t mck)
{
	uint32_t mck_divisor;

	if (mck <= 20000000U) {
		mck_divisor = GMAC_NCFGR_CLK_MCK_8;
	} else if (mck <= 40000000U) {
		mck_divisor = GMAC_NCFGR_CLK_MCK_16;
	} else if (mck <= 80000000U) {
		mck_divisor = GMAC_NCFGR_CLK_MCK_32;
	} else if (mck <= 120000000U) {
		mck_divisor = GMAC_NCFGR_CLK_MCK_48;
	} else if (mck <= 160000000U) {
		mck_divisor = GMAC_NCFGR_CLK_MCK_64;
	} else if (mck <= 240000000U) {
		mck_divisor = GMAC_NCFGR_CLK_MCK_96;
	} else {
		LOG_ERR("No valid MDC clock");
		mck_divisor = -ENOTSUP;
	}

	return mck_divisor;
}

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
static int eth_mchp_gmac_setup_qav(gmac_registers_t *const gmac, int queue_id, bool enable)
{
	/* Verify queue id */
	if (queue_id < GMAC_QUE_1 || queue_id > GMAC_ACTIVE_PRIORITY_QUEUE_NUM) {
		return -EINVAL;
	}

	if (queue_id == GMAC_QUE_2) {
		if (enable) {
			gmac->GMAC_CBSCR |= GMAC_CBSCR_QAE;
		} else {
			gmac->GMAC_CBSCR &= ~GMAC_CBSCR_QAE;
		}
	} else {
		if (enable) {
			gmac->GMAC_CBSCR |= GMAC_CBSCR_QBE;
		} else {
			gmac->GMAC_CBSCR &= ~GMAC_CBSCR_QBE;
		}
	}

	return 0;
}

static int eth_mchp_gmac_get_qav_status(gmac_registers_t *const gmac, int queue_id, bool *enabled)
{
	/* Verify queue id */
	if (queue_id < GMAC_QUE_1 || queue_id > GMAC_ACTIVE_PRIORITY_QUEUE_NUM) {
		return -EINVAL;
	}

	if (queue_id == GMAC_QUE_2) {
		*enabled = gmac->GMAC_CBSCR & GMAC_CBSCR_QAE;
	} else {
		*enabled = gmac->GMAC_CBSCR & GMAC_CBSCR_QBE;
	}

	return 0;
}

static int eth_mchp_gmac_setup_qav_idle_slope(gmac_registers_t *const gmac, int queue_id,
					      unsigned int idle_slope)
{
	uint32_t cbscr_val;

	/* Verify queue id */
	if (queue_id < GMAC_QUE_1 || queue_id > GMAC_ACTIVE_PRIORITY_QUEUE_NUM) {
		return -EINVAL;
	}

	cbscr_val = gmac->GMAC_CBSISQA;

	if (queue_id == GMAC_QUE_2) {
		gmac->GMAC_CBSCR &= ~GMAC_CBSCR_QAE;
		gmac->GMAC_CBSISQA = idle_slope;
	} else {
		gmac->GMAC_CBSCR &= ~GMAC_CBSCR_QBE;
		gmac->GMAC_CBSISQB = idle_slope;
	}

	gmac->GMAC_CBSCR = cbscr_val;

	return 0;
}

static uint32_t eth_mchp_gmac_get_bandwidth(gmac_registers_t *const gmac)
{
	uint32_t bandwidth;

	/* See if we operate in 10Mbps or 100Mbps mode,
	 * Note: according to the manual, portTransmitRate is 0x07735940 for
	 * 1Gbps - therefore we cannot use the KB/MB macros - we have to
	 * multiply it by a round 1000 to get it right.
	 */
	if (gmac->GMAC_NCFGR & GMAC_NCFGR_SPD_Msk) {
		/* 100Mbps */
		bandwidth = (100 * 1000 * 1000) / 8;
	} else {
		/* 10Mbps */
		bandwidth = (10 * 1000 * 1000) / 8;
	}

	return bandwidth;
}

static int eth_mchp_gmac_get_qav_idle_slope(gmac_registers_t *const gmac, int queue_id,
					    unsigned int *idle_slope)
{
	/* Verify queue id */
	if (queue_id < GMAC_QUE_1 || queue_id > GMAC_ACTIVE_PRIORITY_QUEUE_NUM) {
		return -EINVAL;
	}

	if (queue_id == GMAC_QUE_2) {
		*idle_slope = gmac->GMAC_CBSISQA;
	} else {
		*idle_slope = gmac->GMAC_CBSISQB;
	}

	/* Convert to bps as expected by upper layer */
	*idle_slope *= 8U;

	return 0;
}

static int eth_mchp_gmac_get_qav_delta_bandwidth(gmac_registers_t *const gmac, int queue_id,
						 unsigned int *delta_bandwidth)
{
	uint32_t bandwidth;
	unsigned int idle_slope;
	int ret;

	ret = eth_mchp_gmac_get_qav_idle_slope(gmac, queue_id, &idle_slope);
	if (ret) {
		return ret;
	}

	/* Calculate in Bps */
	idle_slope /= 8U;

	/* Get bandwidth and convert to bps */
	bandwidth = eth_mchp_gmac_get_bandwidth(gmac);

	/* Calculate percentage - instead of multiplying idle_slope by 100,
	 * divide bandwidth - these numbers are so large that it should not
	 * influence the outcome and saves us from employing larger data types.
	 */
	*delta_bandwidth = idle_slope / (bandwidth / 100U);

	return 0;
}

static int eth_mchp_gmac_setup_qav_delta_bandwidth(gmac_registers_t *const gmac, int queue_id,
						   int queue_share)
{
	uint32_t bandwidth;
	uint32_t idle_slope;

	/* Verify queue id */
	if (queue_id < GMAC_QUE_1 || queue_id > GMAC_ACTIVE_PRIORITY_QUEUE_NUM) {
		return -EINVAL;
	}

	bandwidth = eth_mchp_gmac_get_bandwidth(gmac);

	idle_slope = (bandwidth * queue_share) / 100U;

	return eth_mchp_gmac_setup_qav_idle_slope(gmac, queue_id, idle_slope);
}
#endif

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
static int eth_mchp_gmac_setup_ptp_clock_divisors(const struct eth_mchp_dev_cfg *const cfg)
{
	gmac_registers_t *const gmac = cfg->regs;
	int mck_divs[] = {10, 5, 2};
	double min_cycles;
	double min_period;
	uint32_t clkin_freq;
	uint8_t cns, acns, nit;
	int div, i, ret;

	ret = clock_control_get_rate(cfg->clock, cfg->clock_cfg, &clkin_freq);
	if (ret) {
		return ret;
	}

	min_cycles = clkin_freq;
	min_period = NSEC_PER_SEC;

	for (i = 0; i < ARRAY_SIZE(mck_divs); ++i) {
		div = mck_divs[i];
		while ((double)(min_cycles / div) == (int)(min_cycles / div) &&
		       (double)(min_period / div) == (int)(min_period / div)) {
			min_cycles /= div;
			min_period /= div;
		}
	}

	nit = min_cycles - 1;
	cns = 0U;
	acns = 0U;

	while ((cns + 2) * nit < min_period) {
		cns++;
	}

	acns = min_period - (nit * cns);

	gmac->GMAC_TI = GMAC_TI_CNS(cns) | GMAC_TI_ACNS(acns) | GMAC_TI_NIT(nit);
	gmac->GMAC_TISUBN = 0;

	return 0;
}
#endif

static int eth_mchp_gmac_init(const struct eth_mchp_dev_cfg *const cfg, uint32_t gmac_ncfgr_val)
{
	gmac_registers_t *const gmac = cfg->regs;
	uint32_t clkin_freq;
	int mck_divisor, ret;

	ret = clock_control_get_rate(cfg->clock, cfg->clock_cfg, &clkin_freq);
	if (ret) {
		return ret;
	}

	mck_divisor = eth_mchp_gmac_get_mck_clock_divisor(clkin_freq);
	if (mck_divisor < 0) {
		return mck_divisor;
	}

	/* Set Network Control Register to its default value, clear stats. */
	gmac->GMAC_NCR = GMAC_NCR_CLRSTAT_Msk | GMAC_NCR_MPE_Msk;

	/* Disable all interrupts */
	gmac->GMAC_IDR = UINT32_MAX;
	/* Clear all interrupts */
	(void)gmac->GMAC_ISR;
	eth_mchp_gmac_disable_all_priority_queue_interrupt(gmac);

	/* Setup Hash Registers - enable reception of all multicast frames when
	 * GMAC_NCFGR_MTIHEN_Msk is set.
	 */
	gmac->GMAC_HRB = UINT32_MAX;
	gmac->GMAC_HRT = UINT32_MAX;
	/* Setup Network Configuration Register */
	gmac->GMAC_NCFGR = gmac_ncfgr_val | mck_divisor;

	switch (cfg->phy_conn_type) {
	case 0:
		/* mii */
		gmac->GMAC_UR = 0x1;
		break;
	case 1:
		/* rmii */
		gmac->GMAC_UR = 0x0;
		break;
#if defined(GMAC_UR_MIM_RGMII)
	case 3:
		/* rgmii */
		gmac->GMAC_UR = GMAC_UR_MIM_RGMII;
		break;
#endif
	default:
		/* Build assert in this file should catch this case */
		LOG_ERR("The phy connection type is invalid");

		return -EINVAL;
	}
#if defined(GMAC_UR_REFCLK_Msk)
	if (cfg->ref_clk_source == 1) { /* Source for the GMAC Reference Clock is EXTERNAL */
		gmac->GMAC_UR |= GMAC_UR_REFCLK_Msk;
	}
#endif

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
	/* Initialize PTP Clock Registers */
	ret = eth_mchp_gmac_setup_ptp_clock_divisors(cfg);
	if (ret) {
		return ret;
	}

	gmac->GMAC_TN = 0;
	gmac->GMAC_TSH = 0;
	gmac->GMAC_TSL = 0;
#endif

	/* Enable Qav if priority queues are used, and setup the default delta
	 * bandwidth according to IEEE802.1Qav (34.3.1)
	 */
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM == 1
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 1, 75);
#elif GMAC_ACTIVE_PRIORITY_QUEUE_NUM == 2
	/* For multiple priority queues, 802.1Qav suggests using 75% for the
	 * highest priority queue, and 0% for the lower priority queues.
	 * This is because the lower priority queues are supposed to be using
	 * the bandwidth available from the higher priority queues AND its own
	 * available bandwidth (see 802.1Q 34.3.1 for more details).
	 * This does not work like that in GMAC - the lower priority queues
	 * are not using the bandwidth reserved for the higher priority queues
	 * at all. Thus we still set the default to a total of the recommended
	 * 75%, but split the bandwidth between them manually.
	 */
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 1, 25);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 2, 50);
#elif GMAC_ACTIVE_PRIORITY_QUEUE_NUM == 3
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 1, 25);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 2, 25);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 3, 25);
#elif GMAC_ACTIVE_PRIORITY_QUEUE_NUM == 4
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 1, 21);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 2, 18);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 3, 18);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 4, 18);
#elif GMAC_ACTIVE_PRIORITY_QUEUE_NUM == 5
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 1, 15);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 2, 15);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 3, 15);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 4, 15);
	eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, 5, 15);
#endif

	eth_mchp_gmac_init_qav(gmac);

	return 0;
}

static void eth_mchp_gmac_link_configure(gmac_registers_t *const gmac, enum phy_link_speed speed)
{
	uint32_t val;

	val = gmac->GMAC_NCFGR;

	val &= ~(GMAC_NCFGR_FD_Msk | GMAC_NCFGR_SPD_Msk);
	val |= PHY_LINK_IS_FULL_DUPLEX(speed) ? GMAC_NCFGR_FD_Msk : 0;
	val |= PHY_LINK_IS_SPEED_100M(speed) ? GMAC_NCFGR_SPD_Msk : 0;
#if defined(GMAC_NCFGR_GBE)
	val |= PHY_LINK_IS_SPEED_1000M(speed) ? GMAC_NCFGR_GBE : 0;
#endif

	gmac->GMAC_NCFGR = val;

	gmac->GMAC_NCR |= (GMAC_NCR_RXEN_Msk | GMAC_NCR_TXEN_Msk);
}

static int eth_mchp_gmac_nonpriority_queue_init(gmac_registers_t *const gmac,
						struct gmac_queue *queue)
{
	int ret;

	__ASSERT_NO_MSG(queue->rx_desc_list.len > 0);
	__ASSERT_NO_MSG(queue->tx_desc_list.len > 0);
	__ASSERT(!((uint32_t)queue->rx_desc_list.buf & ~GMAC_RBQB_ADDR_Msk),
		 "RX descriptors have to be word aligned");
	__ASSERT(!((uint32_t)queue->tx_desc_list.buf & ~GMAC_TBQB_ADDR_Msk),
		 "TX descriptors have to be word aligned");

	/* Setup descriptor lists */
	ret = eth_mchp_gmac_rx_descriptors_init(gmac, queue);
	if (ret < 0) {
		return ret;
	}

	eth_mchp_gmac_tx_descriptors_init(gmac, queue);

#if GMAC_MULTIPLE_TX_PACKETS == 0
	/* Initialize TX semaphore. This semaphore is used to wait until the TX
	 * data has been sent.
	 */
	k_sem_init(&queue->tx_sem, 0, 1);
#else
	/* Initialize TX descriptors semaphore. The semaphore is required as the
	 * size of the TX descriptor list is limited while the number of TX data
	 * buffers is not.
	 */
	k_sem_init(&queue->tx_desc_sem, queue->tx_desc_list.len - 1, queue->tx_desc_list.len - 1);
#endif

	/* Set Receive Buffer Queue Pointer Register */
	gmac->GMAC_RBQB = (uint32_t)queue->rx_desc_list.buf;
	/* Set Transmit Buffer Queue Pointer Register */
	gmac->GMAC_TBQB = (uint32_t)queue->tx_desc_list.buf;

	/* Configure GMAC DMA transfer */
	gmac->GMAC_DCFGR =
		/* Receive Buffer Size (defined in multiples of 64 bytes) */
		GMAC_DCFGR_DRBS(CONFIG_NET_BUF_DATA_SIZE >> 6) |
#if defined(GMAC_DCFGR_RXBMS)
		/* Use full receive buffer size on parts where this is selectable */
		GMAC_DCFGR_RXBMS(3) |
#endif
		/* Attempt to use INCR4 AHB bursts (Default) */
		GMAC_DCFGR_FBLDO_INCR4 |
		/* DMA Queue Flags */
		GMAC_DMA_QUEUE_FLAGS;

	/* Setup RX/TX completion and error interrupts */
	gmac->GMAC_IER = GMAC_INT_EN_FLAGS;

	queue->err_rx_frames_dropped = 0U;
	queue->err_rx_flushed_count = 0U;
	queue->err_tx_flushed_count = 0U;

	LOG_INF("ethernet@%x Queue %d activated", (uint32_t)gmac, queue->que_idx);

	return 0;
}

static struct net_pkt *eth_mchp_gmac_frame_get(struct gmac_queue *queue)
{
	struct gmac_desc_list *rx_desc_list = &queue->rx_desc_list;
	struct gmac_desc *rx_desc;
	struct net_buf **rx_frag_list = queue->rx_frag_list;
	struct net_pkt *rx_frame;
	bool frame_is_complete;
	struct net_buf *frag;
	struct net_buf *new_frag;
	struct net_buf *last_frag = NULL;
	uint8_t *frag_data;
	uint32_t frag_len;
	uint32_t frame_len = 0U;
	uint16_t tail;
	uint8_t wrap;

	/* Check if there exists a complete frame in RX descriptor list */
	tail = rx_desc_list->tail;
	rx_desc = &rx_desc_list->buf[tail];
	frame_is_complete = false;
	while ((rx_desc->w0 & GMAC_RXW0_OWNERSHIP) && !frame_is_complete) {
		frame_is_complete = (bool)(rx_desc->w1 & GMAC_RXW1_EOF);
		MODULO_INC(tail, rx_desc_list->len);
		rx_desc = &rx_desc_list->buf[tail];
	}
	/* Frame which is not complete can be dropped by GMAC. Do not process
	 * it, even partially.
	 */
	if (!frame_is_complete) {
		return NULL;
	}

	rx_frame = net_pkt_rx_alloc(K_NO_WAIT);

	/* Process a frame */
	tail = rx_desc_list->tail;
	rx_desc = &rx_desc_list->buf[tail];
	frame_is_complete = false;

	__ASSERT(rx_desc->w1 & GMAC_RXW1_SOF, "First RX fragment is missing SOF bit");

	while ((rx_desc->w0 & GMAC_RXW0_OWNERSHIP) && !frame_is_complete) {
		frag = rx_frag_list[tail];
		frag_data = (uint8_t *)(rx_desc->w0 & GMAC_RXW0_ADDR);
		__ASSERT(frag->data == frag_data, "RX descriptor and buffer list desynchronized");
		frame_is_complete = (bool)(rx_desc->w1 & GMAC_RXW1_EOF);
		if (frame_is_complete) {
			frag_len = (rx_desc->w1 & GMAC_RXW1_LEN) - frame_len;
		} else {
			frag_len = CONFIG_NET_BUF_DATA_SIZE;
		}

		frame_len += frag_len;

		/* Link frame fragments only if RX net buffer is valid */
		if (rx_frame != NULL) {
			/* Assure cache coherency after DMA write operation */
			eth_mchp_gmac_dcache_invalidate((uint32_t)frag_data, frag->size);

			/* Get a new data net buffer from the buffer pool */
			new_frag = net_pkt_get_frag(rx_frame, CONFIG_NET_BUF_DATA_SIZE, K_NO_WAIT);
			if (new_frag == NULL) {
				queue->err_rx_frames_dropped++;
				net_pkt_unref(rx_frame);
				rx_frame = NULL;
			} else {
				net_buf_add(frag, frag_len);
				if (!last_frag) {
					net_pkt_frag_insert(rx_frame, frag);
				} else {
					net_buf_frag_insert(last_frag, frag);
				}
				last_frag = frag;
				frag = new_frag;
				rx_frag_list[tail] = frag;
			}
		}

		/* Update buffer descriptor status word */
		rx_desc->w1 = 0U;
		/* Guarantee that status word is written before the address
		 * word to avoid race condition.
		 */
		barrier_dmem_fence_full();
		/* Update buffer descriptor address word */
		wrap = (tail == rx_desc_list->len - 1U ? GMAC_RXW0_WRAP : 0);
		rx_desc->w0 = ((uint32_t)frag->data & GMAC_RXW0_ADDR) | wrap;

		MODULO_INC(tail, rx_desc_list->len);
		rx_desc = &rx_desc_list->buf[tail];
	}

	rx_desc_list->tail = tail;
	LOG_DBG("Frame complete: rx=%p, tail=%d", rx_frame, tail);
	__ASSERT_NO_MSG(frame_is_complete);

	return rx_frame;
}

static void eth_mchp_gmac_eth_rx(struct gmac_queue *queue)
{
	struct eth_mchp_dev_data *dev_data =
		CONTAINER_OF(queue, struct eth_mchp_dev_data, queue_list[queue->que_idx]);
	struct net_pkt *rx_frame;
#if defined(CONFIG_NET_GPTP)
	const struct device *const dev = net_if_get_device(dev_data->iface);
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	gmac_registers_t *const gmac = cfg->regs;
	struct gptp_hdr *hdr;
#endif

	/* More than one frame could have been received by GMAC, get all
	 * complete frames stored in the GMAC RX descriptor list.
	 */
	rx_frame = eth_mchp_gmac_frame_get(queue);
	while (rx_frame) {
		LOG_DBG("ETH rx");

#if defined(CONFIG_NET_GPTP)
		hdr = eth_mchp_gmac_check_gptp_msg(eth_mchp_gmac_get_iface(dev_data), rx_frame,
						   false);

		eth_mchp_gmac_timestamp_rx_pkt(gmac, hdr, rx_frame);

		if (hdr) {
			eth_mchp_gmac_update_pkt_priority(hdr, rx_frame);
		}
#endif /* CONFIG_NET_GPTP */

		if (net_recv_data(eth_mchp_gmac_get_iface(dev_data), rx_frame) < 0) {
			eth_stats_update_errors_rx(eth_mchp_gmac_get_iface(dev_data));
			net_pkt_unref(rx_frame);
		}

		rx_frame = eth_mchp_gmac_frame_get(queue);
	}
}

#if !defined(CONFIG_ETH_MCHP_GMAC_G2_FORCE_QUEUE) &&                                               \
	((GMAC_ACTIVE_QUEUE_NUM != NET_TC_TX_COUNT) ||                                             \
	 ((NET_TC_TX_COUNT != NET_TC_RX_COUNT) && defined(CONFIG_NET_VLAN)))
static int priority2queue(enum net_priority priority)
{
	static const uint8_t queue_priority_map[] = {
#if GMAC_ACTIVE_QUEUE_NUM == 1
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0
#endif
#if GMAC_ACTIVE_QUEUE_NUM == 2
		0,
		0,
		0,
		0,
		1,
		1,
		1,
		1
#endif
#if GMAC_ACTIVE_QUEUE_NUM == 3
		0,
		0,
		0,
		0,
		1,
		1,
		2,
		2
#endif
#if GMAC_ACTIVE_QUEUE_NUM == 4
		0,
		0,
		0,
		0,
		1,
		1,
		2,
		3
#endif
#if GMAC_ACTIVE_QUEUE_NUM == 5
		0,
		0,
		0,
		0,
		1,
		2,
		3,
		4
#endif
#if GMAC_ACTIVE_QUEUE_NUM == 6
		0,
		0,
		0,
		1,
		2,
		3,
		4,
		5
#endif
	};

	return queue_priority_map[priority];
}
#endif

static int eth_mchp_gmac_eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	struct eth_mchp_dev_data *const dev_data = dev->data;
	gmac_registers_t *const gmac = cfg->regs;
	struct gmac_queue *queue;
	struct gmac_desc_list *tx_desc_list;
	struct gmac_desc *tx_desc;
	struct gmac_desc *tx_first_desc;
	struct net_buf *frag;
	uint8_t *frag_data;
	uint16_t frag_len;
	uint32_t err_tx_flushed_count_at_entry;
#if GMAC_MULTIPLE_TX_PACKETS == 1
	unsigned int key;
#endif
	uint8_t pkt_prio;
#if GMAC_MULTIPLE_TX_PACKETS == 0
#if defined(CONFIG_NET_GPTP)
	struct gptp_hdr *hdr;
#endif
#endif

	__ASSERT(pkt, "buf pointer is NULL");
	__ASSERT(pkt->frags, "Frame data missing");

	LOG_DBG("%s tx", dev->name);

	/* Decide which queue should be used */
	pkt_prio = net_pkt_priority(pkt);

#if defined(CONFIG_ETH_MCHP_GMAC_G2_FORCE_QUEUE)
	/* Route everything to the forced queue */
	queue = &dev_data->queue_list[CONFIG_ETH_MCHP_GMAC_G2_FORCED_QUEUE];
#elif GMAC_ACTIVE_QUEUE_NUM == CONFIG_NET_TC_TX_COUNT
	/* Prefer to chose queue based on its traffic class */
	queue = &dev_data->queue_list[net_tx_priority2tc(pkt_prio)];
#else
	/* If that's not possible due to config - use builtin mapping */
	queue = &dev_data->queue_list[priority2queue(pkt_prio)];
#endif

	tx_desc_list = &queue->tx_desc_list;
	err_tx_flushed_count_at_entry = queue->err_tx_flushed_count;

	frag = pkt->frags;

	/* Keep reference to the descriptor */
	tx_first_desc = &tx_desc_list->buf[tx_desc_list->head];

	while (frag) {
		frag_data = frag->data;
		frag_len = frag->len;

		/* Assure cache coherency before DMA read operation */
		eth_mchp_gmac_dcache_clean((uint32_t)frag_data, frag->size);

#if GMAC_MULTIPLE_TX_PACKETS == 1
		k_sem_take(&queue->tx_desc_sem, K_FOREVER);

		/* The following section becomes critical and requires IRQ lock
		 * / unlock protection only due to the possibility of executing
		 * eth_mchp_gmac_tx_error_handler() function.
		 */
		key = irq_lock();

		/* Check if eth_mchp_gmac_tx_error_handler() function was executed */
		if (queue->err_tx_flushed_count != err_tx_flushed_count_at_entry) {
			irq_unlock(key);
			return -EIO;
		}
#endif

		tx_desc = &tx_desc_list->buf[tx_desc_list->head];

		/* Update buffer descriptor address word */
		tx_desc->w0 = (uint32_t)frag_data;

		/* Update buffer descriptor status word (clear used bit except
		 * for the first frag).
		 */
		tx_desc->w1 = (frag_len & GMAC_TXW1_LEN) |
			      (!frag->frags ? GMAC_TXW1_LASTBUFFER : 0) |
			      (tx_desc_list->head == tx_desc_list->len - 1U ? GMAC_TXW1_WRAP : 0) |
			      (tx_desc == tx_first_desc ? GMAC_TXW1_USED : 0);

		/* Update descriptor position */
		MODULO_INC(tx_desc_list->head, tx_desc_list->len);

#if GMAC_MULTIPLE_TX_PACKETS == 1
		__ASSERT(tx_desc_list->head != tx_desc_list->tail, "tx_desc_list overflow");

		/* Account for a sent frag */
		if (sys_ringq_put(&queue->tx_frag_list, &frag)) {
			__ASSERT(false, "tx_frag_list ringq full");
		}

		/* frag is internally queued, so it requires to hold a reference */
		net_pkt_frag_ref(frag);

		irq_unlock(key);
#endif

		/* Continue with the rest of fragments (only data) */
		frag = frag->frags;
	}

#if GMAC_MULTIPLE_TX_PACKETS == 1
	key = irq_lock();

	/* Check if eth_mchp_gmac_tx_error_handler() function was executed */
	if (queue->err_tx_flushed_count != err_tx_flushed_count_at_entry) {
		irq_unlock(key);
		return -EIO;
	}
#endif

	/* Ensure the descriptor following the last one is marked as used */
	tx_desc_list->buf[tx_desc_list->head].w1 = GMAC_TXW1_USED;

	/* Guarantee that all the fragments have been written before removing
	 * the used bit to avoid race condition.
	 */
	barrier_dmem_fence_full();

	/* Remove the used bit of the first fragment to allow the controller
	 * to process it and the following fragments.
	 */
	tx_first_desc->w1 &= ~GMAC_TXW1_USED;

#if GMAC_MULTIPLE_TX_PACKETS == 1
#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
	/* Account for a sent frame */
	if (sys_ringq_put(&queue->tx_frames, &pkt)) {
		__ASSERT(false, "tx_frames ringq full");
	}

	/* pkt is internally queued, so it requires to hold a reference */
	net_pkt_ref(pkt);
#endif

	irq_unlock(key);
#endif

	/* Guarantee that the first fragment got its bit removed before starting
	 * sending packets to avoid packets getting stuck.
	 */
	barrier_dmem_fence_full();

	/* Start transmission */
	gmac->GMAC_NCR |= GMAC_NCR_TSTART_Msk;

#if GMAC_MULTIPLE_TX_PACKETS == 0
	/* Wait until the packet is sent */
	k_sem_take(&queue->tx_sem, K_FOREVER);

	/* Check if transmit successful or not */
	if (queue->err_tx_flushed_count != err_tx_flushed_count_at_entry) {
		return -EIO;
	}
#if defined(CONFIG_NET_GPTP)
#if defined(CONFIG_NET_GPTP)
	hdr = eth_mchp_gmac_check_gptp_msg(eth_mchp_gmac_get_iface(dev_data), pkt, true);
	eth_mchp_gmac_timestamp_tx_pkt(gmac, hdr, pkt);
	if (hdr && eth_mchp_gmac_need_timestamping(hdr)) {
		net_if_add_tx_timestamp(pkt);
	}
#endif
#endif
#endif

	return 0;
}

static void eth_mchp_gmac_queue0_isr(const struct device *dev)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	struct eth_mchp_dev_data *const dev_data = dev->data;
	gmac_registers_t *const gmac = cfg->regs;
	struct gmac_queue *queue;
	struct gmac_desc_list *rx_desc_list;
	struct gmac_desc_list *tx_desc_list;
	struct gmac_desc *tail_desc;
	uint32_t isr;

	/* Interrupt Status Register is cleared on read */
	isr = gmac->GMAC_ISR;
	LOG_DBG("%s GMAC_ISR=0x%08x", dev->name, isr);

	queue = &dev_data->queue_list[0];
	rx_desc_list = &queue->rx_desc_list;
	tx_desc_list = &queue->tx_desc_list;

	/* RX packet */
	if (isr & GMAC_INT_RX_ERR_BITS) {
		eth_mchp_gmac_rx_error_handler(gmac, queue);
	} else if (isr & GMAC_ISR_RCOMP_Msk) {
		tail_desc = &rx_desc_list->buf[rx_desc_list->tail];
		LOG_DBG("%s rx.w1=0x%08x, tail=%d", dev->name, tail_desc->w1, rx_desc_list->tail);
		eth_mchp_gmac_eth_rx(queue);
	}

	/* TX packet */
	if (isr & GMAC_INT_TX_ERR_BITS) {
		eth_mchp_gmac_tx_error_handler(gmac, queue);
	} else if (isr & GMAC_ISR_TCOMP_Msk) {
#if GMAC_MULTIPLE_TX_PACKETS == 1
		tail_desc = &tx_desc_list->buf[tx_desc_list->tail];
		LOG_DBG("%s tx.w1=0x%08x, tail=%d", dev->name, tail_desc->w1, tx_desc_list->tail);
#endif

		eth_mchp_gmac_tx_completed(gmac, queue);
	}

	if (isr & GMAC_IER_HRESP_Msk) {
		LOG_DBG("%s IER HRESP", dev->name);
	}
}

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
static inline void eth_mchp_gmac_priority_queue_isr(const struct device *dev,
						    unsigned int queue_idx)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	struct eth_mchp_dev_data *const dev_data = dev->data;
	gmac_registers_t *const gmac = cfg->regs;
	struct gmac_queue *queue;
	struct gmac_desc_list *rx_desc_list;
	struct gmac_desc_list *tx_desc_list;
	struct gmac_desc *tail_desc;
	uint32_t isrpq;

	isrpq = gmac->GMAC_ISRPQ[queue_idx - 1];
	LOG_DBG("%s GMAC_ISRPQ%d=0x%08x", dev->name, queue_idx - 1, isrpq);

	queue = &dev_data->queue_list[queue_idx];
	rx_desc_list = &queue->rx_desc_list;
	tx_desc_list = &queue->tx_desc_list;

	/* RX packet */
	if (isrpq & GMAC_INTPQ_RX_ERR_BITS) {
		eth_mchp_gmac_rx_error_handler(gmac, queue);
	} else if (isrpq & GMAC_ISRPQ_RCOMP) {
		tail_desc = &rx_desc_list->buf[rx_desc_list->tail];
		LOG_DBG("%s rx.w1=0x%08x, tail=%d", dev->name, tail_desc->w1, rx_desc_list->tail);
		eth_mchp_gmac_eth_rx(queue);
	}

	/* TX packet */
	if (isrpq & GMAC_INTPQ_TX_ERR_BITS) {
		eth_mchp_gmac_tx_error_handler(gmac, queue);
	} else if (isrpq & GMAC_ISRPQ_TCOMP) {
#if GMAC_MULTIPLE_TX_PACKETS == 1
		tail_desc = &tx_desc_list->buf[tx_desc_list->tail];
		LOG_DBG("%s tx.w1=0x%08x, tail=%d", dev->name, tail_desc->w1, tx_desc_list->tail);
#endif

		eth_mchp_gmac_tx_completed(gmac, queue);
	}

	if (isrpq & GMAC_IERPQ_HRESP_Msk) {
		LOG_DBG("%s IERPQ%d HRESP", dev->name, queue_idx - 1);
	}
}
#endif

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
static void eth_mchp_gmac_queue1_isr(const struct device *dev)
{
	eth_mchp_gmac_priority_queue_isr(dev, 1);
}
#endif

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 2
static void eth_mchp_gmac_queue2_isr(const struct device *dev)
{
	eth_mchp_gmac_priority_queue_isr(dev, 2);
}
#endif

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 3
static void eth_mchp_gmac_queue3_isr(const struct device *dev)
{
	eth_mchp_gmac_priority_queue_isr(dev, 3);
}
#endif

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 4
static void eth_mchp_gmac_queue4_isr(const struct device *dev)
{
	eth_mchp_gmac_priority_queue_isr(dev, 4);
}
#endif

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 5
static void eth_mchp_gmac_queue5_isr(const struct device *dev)
{
	eth_mchp_gmac_priority_queue_isr(dev, 5);
}
#endif

static int eth_mchp_gmac_eth_initialize(const struct device *dev)
{
	const struct eth_mchp_dev_cfg *const dev_cfg = dev->config;
	int ret;

	dev_cfg->config_func();

	/* Enable GMAC module's clock */
	ret = clock_control_on(dev_cfg->clock, dev_cfg->clock_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to enable GMAC clock (%d)", ret);
		return -ENODEV;
	}

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);

	return ret;
}

static void eth_mchp_gmac_phy_link_state_changed(const struct device *pdev,
						 struct phy_link_state *state, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct eth_mchp_dev_data *const dev_data = dev->data;
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	bool is_up;

	is_up = state->is_up;

	if (is_up && !dev_data->link_up) {
		/* Announce link up status */
		dev_data->link_up = true;
		net_eth_carrier_on(dev_data->iface);

		/* Set up link */
		eth_mchp_gmac_link_configure(cfg->regs, state->speed);
	} else if (!is_up && dev_data->link_up) {
		/* Announce link down status */
		dev_data->link_up = false;
		net_eth_carrier_off(dev_data->iface);
	}
}

static const struct device *eth_mchp_gmac_get_phy(const struct device *dev, struct net_if *iface)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;

	ARG_UNUSED(iface);

	return cfg->phy_dev;
}

static void eth_mchp_gmac_eth_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_mchp_dev_data *const dev_data = dev->data;
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	uint32_t gmac_ncfgr_val;
	int ret;
	int i;

	dev_data->iface = iface;

	ethernet_init(iface);

	/* Check the status of data caches */
	eth_mchp_gmac_dcache_is_enabled();

	/* Initialize GMAC driver */
	gmac_ncfgr_val = GMAC_NCFGR_MTIHEN_Msk  /* Multicast Hash Enable */
			 | GMAC_NCFGR_LFERD_Msk /* Length Field Error Frame Discard */
			 | GMAC_NCFGR_RFCS_Msk  /* Remove Frame Check Sequence */
#if defined(CONFIG_NET_VLAN)
			 | GMAC_NCFGR_MAXFS
#endif
			 | GMAC_NCFGR_RXCOEN_Msk; /* Receive Checksum Offload Enable */
	ret = eth_mchp_gmac_init(cfg, gmac_ncfgr_val);
	if (ret < 0) {
		LOG_ERR("%s Unable to initialize ETH driver", dev->name);
		return;
	}

	ret = net_eth_mac_load(&cfg->mcfg, dev_data->mac_addr);
	if (ret < 0) {
		LOG_ERR("Failed to load MAC (%d)", ret);
		return;
	}

	LOG_INF("%s MAC: %02x:%02x:%02x:%02x:%02x:%02x", dev->name, dev_data->mac_addr[0],
		dev_data->mac_addr[1], dev_data->mac_addr[2], dev_data->mac_addr[3],
		dev_data->mac_addr[4], dev_data->mac_addr[5]);

	/* Set MAC Address for frame filtering logic */
	eth_mchp_gmac_mac_addr_set(cfg->regs, 0, dev_data->mac_addr);

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, dev_data->mac_addr, sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);

	/* Initialize GMAC queues */
	for (i = GMAC_QUE_0; i < cfg->num_queues; i++) {
		ret = eth_mchp_gmac_queue_init(cfg->regs, &dev_data->queue_list[i]);
		if (ret < 0) {
			LOG_ERR("%s Unable to initialize ETH queue%d", dev->name, i);
			return;
		}
	}

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
#if defined(CONFIG_ETH_MCHP_GMAC_G2_FORCE_QUEUE)
	for (i = 0; i < CONFIG_NET_TC_RX_COUNT; ++i) {
		cfg->regs->GMAC_ST1RPQ[i] = GMAC_ST1RPQ_DSTCM(i) |
					    GMAC_ST1RPQ_QNB(CONFIG_ETH_MCHP_GMAC_G2_FORCED_QUEUE);
	}
#elif GMAC_ACTIVE_QUEUE_NUM == NET_TC_RX_COUNT
	/* If TC configuration is compatible with HW configuration, setup the
	 * screening registers based on the DS/TC values.
	 * Map them 1:1 - TC 0 -> Queue 0, TC 1 -> Queue 1 etc.
	 */
	for (i = 0; i < CONFIG_NET_TC_RX_COUNT; ++i) {
		cfg->regs->GMAC_ST1RPQ[i] = GMAC_ST1RPQ_DSTCM(i) | GMAC_ST1RPQ_QNB(i);
	}
#elif defined(CONFIG_NET_VLAN)
	/* If VLAN is enabled, route packets according to VLAN priority */
	int j;

	i = 0;
	for (j = NET_PRIORITY_NC; j >= 0; --j) {
		if (priority2queue(j) == 0) {
			/* No point to set rules for the regular queue */
			continue;
		}

		if (i >= ARRAY_SIZE(cfg->regs->GMAC_ST2RPQ)) {
			/* No more screening registers available */
			break;
		}

		cfg->regs->GMAC_ST2RPQ[i++] = GMAC_ST2RPQ_QNB(priority2queue(j)) |
					      GMAC_ST2RPQ_VLANP(j) | GMAC_ST2RPQ_VLANE;
	}

#endif
#endif
	if (device_is_ready(cfg->phy_dev)) {
		net_if_carrier_off(iface);

		phy_link_callback_set(cfg->phy_dev, &eth_mchp_gmac_phy_link_state_changed,
				      (void *)dev);

	} else {
		LOG_ERR("%s PHY device not ready", dev->name);
	}
}

static enum ethernet_hw_caps eth_mchp_gmac_get_capabilities(const struct device *dev,
							    struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	return ETHERNET_LINK_10BASE |
#if defined(CONFIG_NET_VLAN)
	       ETHERNET_HW_VLAN |
#endif
	       ETHERNET_PRIORITY_QUEUES |
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
	       ETHERNET_QAV |
#endif
	       ETHERNET_LINK_100BASE;
}

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
static int eth_mchp_gmac_set_qav_param(const struct device *dev, enum ethernet_config_type type,
				       const struct ethernet_config *config)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	gmac_registers_t *const gmac = cfg->regs;
	enum ethernet_qav_param_type qav_param_type;
	unsigned int delta_bandwidth;
	unsigned int idle_slope;
	int queue_id;
	bool enable;

	/* Priority queue IDs start from 1 for GMAC */
	queue_id = config->qav_param.queue_id + 1;

	qav_param_type = config->qav_param.type;

	switch (qav_param_type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		enable = config->qav_param.enabled;
		return eth_mchp_gmac_setup_qav(gmac, queue_id, enable);
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		delta_bandwidth = config->qav_param.delta_bandwidth;

		return eth_mchp_gmac_setup_qav_delta_bandwidth(gmac, queue_id, delta_bandwidth);
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		idle_slope = config->qav_param.idle_slope;

		/* The standard uses bps, GMAC uses Bps - convert now */
		idle_slope /= 8U;

		return eth_mchp_gmac_setup_qav_idle_slope(gmac, queue_id, idle_slope);
	default:
		break;
	}

	return -ENOTSUP;
}
#endif

static int eth_mchp_gmac_set_config(const struct device *dev, struct net_if *iface,
				    enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	struct eth_mchp_dev_data *const dev_data = dev->data;
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	int ret = 0;

	ARG_UNUSED(iface);

	switch (type) {
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return eth_mchp_gmac_set_qav_param(dev, type, config);
#endif
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(dev_data->mac_addr, config->mac_address.addr, sizeof(dev_data->mac_addr));

		/* Set MAC Address for frame filtering logic */
		eth_mchp_gmac_mac_addr_set(cfg->regs, 0, dev_data->mac_addr);

		LOG_INF("%s MAC set to %02x:%02x:%02x:%02x:%02x:%02x", dev->name,
			dev_data->mac_addr[0], dev_data->mac_addr[1], dev_data->mac_addr[2],
			dev_data->mac_addr[3], dev_data->mac_addr[4], dev_data->mac_addr[5]);

		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
static int eth_mchp_gmac_get_qav_param(const struct device *dev, enum ethernet_config_type type,
				       struct ethernet_config *config)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	gmac_registers_t *const gmac = cfg->regs;
	enum ethernet_qav_param_type qav_param_type;
	int queue_id;
	bool *enabled;
	unsigned int *idle_slope;
	unsigned int *delta_bandwidth;

	/* Priority queue IDs start from 1 for GMAC */
	queue_id = config->qav_param.queue_id + 1;

	qav_param_type = config->qav_param.type;

	switch (qav_param_type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		enabled = &config->qav_param.enabled;
		return eth_mchp_gmac_get_qav_status(gmac, queue_id, enabled);
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		idle_slope = &config->qav_param.idle_slope;
		return eth_mchp_gmac_get_qav_idle_slope(gmac, queue_id, idle_slope);
	case ETHERNET_QAV_PARAM_TYPE_OPER_IDLE_SLOPE:
		idle_slope = &config->qav_param.oper_idle_slope;
		return eth_mchp_gmac_get_qav_idle_slope(gmac, queue_id, idle_slope);
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		delta_bandwidth = &config->qav_param.delta_bandwidth;
		return eth_mchp_gmac_get_qav_delta_bandwidth(gmac, queue_id, delta_bandwidth);
	case ETHERNET_QAV_PARAM_TYPE_TRAFFIC_CLASS:
#if GMAC_ACTIVE_QUEUE_NUM == NET_TC_TX_COUNT
		config->qav_param.traffic_class = queue_id;
		return 0;
#else
		/* Invalid configuration - no direct TC to queue mapping */
		return -ENOTSUP;
#endif
	default:
		break;
	}

	return -ENOTSUP;
}
#endif

static int eth_mchp_gmac_get_config(const struct device *dev, struct net_if *iface,
				    enum ethernet_config_type type, struct ethernet_config *config)
{
	ARG_UNUSED(config);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PRIORITY_QUEUES_NUM:
		config->priority_queues_num = GMAC_ACTIVE_PRIORITY_QUEUE_NUM;
		return 0;
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return eth_mchp_gmac_get_qav_param(dev, type, config);
#endif
	default:
		break;
	}

	return -ENOTSUP;
}

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
static const struct device *eth_mchp_gmac_get_ptp_clock(const struct device *dev,
							struct net_if *iface)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;

	ARG_UNUSED(iface);

	return cfg->ptp_clock;
}
#endif

static DEVICE_API(ethernet, eth_mchp_gmac_api) = {
	.iface_api.init = eth_mchp_gmac_eth_iface_init,

	.get_capabilities = eth_mchp_gmac_get_capabilities,
	.set_config = eth_mchp_gmac_set_config,
	.get_config = eth_mchp_gmac_get_config,
	.get_phy = eth_mchp_gmac_get_phy,
	.send = eth_mchp_gmac_eth_tx,

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
	.get_ptp_clock = eth_mchp_gmac_get_ptp_clock,
#endif
};

#define DEFN_IRQ_CONFIG(n, x, name)                                                                \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq), DT_INST_IRQ_BY_NAME(n, name, priority),     \
		    eth_mchp_gmac_queue##x##_isr, DEVICE_DT_INST_GET(n), 0);                       \
	irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));
#define DEFN_IRQ_CONFIG_0(n) DEFN_IRQ_CONFIG(n, 0, gmac)
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1
#define DEFN_IRQ_CONFIG_1(n) DEFN_IRQ_CONFIG(n, 1, q1)
#else
#define DEFN_IRQ_CONFIG_1(n)
#endif
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 2
#define DEFN_IRQ_CONFIG_2(n) DEFN_IRQ_CONFIG(n, 2, q2)
#else
#define DEFN_IRQ_CONFIG_2(n)
#endif
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 3
#define DEFN_IRQ_CONFIG_3(n) DEFN_IRQ_CONFIG(n, 3, q3)
#else
#define DEFN_IRQ_CONFIG_3(n)
#endif
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 4
#define DEFN_IRQ_CONFIG_4(n) DEFN_IRQ_CONFIG(n, 4, q4)
#else
#define DEFN_IRQ_CONFIG_4(n)
#endif
#if GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 5
#define DEFN_IRQ_CONFIG_5(n) DEFN_IRQ_CONFIG(n, 5, q5)
#else
#define DEFN_IRQ_CONFIG_5(n)
#endif
#define MCHP_GMAC_IRQ_CONFIG_DEFN(n)                                                               \
	static void mchp_gmac_##n##_irq_config(void)                                               \
	{                                                                                          \
		DEFN_IRQ_CONFIG_0(n)                                                               \
		DEFN_IRQ_CONFIG_1(n)                                                               \
		DEFN_IRQ_CONFIG_2(n)                                                               \
		DEFN_IRQ_CONFIG_3(n)                                                               \
		DEFN_IRQ_CONFIG_4(n)                                                               \
		DEFN_IRQ_CONFIG_5(n)                                                               \
	}

#define MCHP_GMAC_PINCTRL_DEFN(n) PINCTRL_DT_INST_DEFINE(n);

#define MCHP_GMAC_CFG_DEFN(n)                                                                      \
	BUILD_ASSERT(DT_INST_ENUM_IDX(n, phy_connection_type) <=                                   \
			     MCHP_GMAC_PHY_CONNECTION_TYPE_MAX,                                    \
		     "Invalid PHY connection");                                                    \
	IF_ENABLED(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK,                                          \
			(DEVICE_DECLARE(gmac_ptp_clock_##n);))   \
	static const struct eth_mchp_dev_cfg mchp_gmac_##n##_config = {                            \
		.regs = (gmac_registers_t *)DT_REG_ADDR(DT_INST_PARENT(n)),                        \
		.clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),                         \
		.clock_cfg = (clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), subsystem), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.config_func = mchp_gmac_##n##_irq_config,                                         \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                          \
		.num_queues = DT_INST_PROP(n, num_queues),                                         \
		.phy_conn_type = DT_INST_ENUM_IDX(n, phy_connection_type),                         \
		.ref_clk_source = DT_INST_ENUM_IDX(n, ref_clk_source),                             \
		.mcfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(n),                                        \
		IF_ENABLED(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK,                                  \
				(.ptp_clock = DEVICE_GET(gmac_ptp_clock_##n),)) };

#define DEFN_RX_FLAG_LIST_0(n) .rx_frag_list = &rx_frag_list##n##_que[PRIORITY_QUEUE0_RX_DESC_IDX],
#if (GMAC_PRIORITY_QUEUE_NUM >= 1) && (GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 1)
#define DEFN_RX_FLAG_LIST_1(n) .rx_frag_list = &rx_frag_list##n##_que[PRIORITY_QUEUE1_RX_DESC_IDX],
#else
#define DEFN_RX_FLAG_LIST_1(n)
#endif
#if (GMAC_PRIORITY_QUEUE_NUM >= 2) && (GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 2)
#define DEFN_RX_FLAG_LIST_2(n) .rx_frag_list = &rx_frag_list##n##_que[PRIORITY_QUEUE2_RX_DESC_IDX],
#else
#define DEFN_RX_FLAG_LIST_2(n)
#endif
#if (GMAC_PRIORITY_QUEUE_NUM >= 3) && (GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 3)
#define DEFN_RX_FLAG_LIST_3(n) .rx_frag_list = &rx_frag_list##n##_que[PRIORITY_QUEUE3_RX_DESC_IDX],
#else
#define DEFN_RX_FLAG_LIST_3(n)
#endif
#if (GMAC_PRIORITY_QUEUE_NUM >= 4) && (GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 4)
#define DEFN_RX_FLAG_LIST_4(n) .rx_frag_list = &rx_frag_list##n##_que[PRIORITY_QUEUE4_RX_DESC_IDX],
#else
#define DEFN_RX_FLAG_LIST_4(n)
#endif
#if (GMAC_PRIORITY_QUEUE_NUM >= 5) && (GMAC_ACTIVE_PRIORITY_QUEUE_NUM >= 5)
#define DEFN_RX_FLAG_LIST_5(n) .rx_frag_list = &rx_frag_list##n##_que[PRIORITY_QUEUE5_RX_DESC_IDX],
#else
#define DEFN_RX_FLAG_LIST_5(n)
#endif

#if GMAC_MULTIPLE_TX_PACKETS == 1
#define BUF_TX_FRAG_LIST_QUE(n, x)                                                                 \
	(uint8_t *)&tx_frag_list##n##_que[PRIORITY_QUEUE##x##_TX_DESC_IDX]
#define DEFN_TX_FLAG_LIST(n, x)                                                                    \
	.tx_frag_list = SYS_RINGQ_INIT(BUF_TX_FRAG_LIST_QUE(n, x), sizeof(struct net_buf *),       \
				       MAIN_QUEUE_TX_DESC_COUNT),

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
#define BUF_TX_FRAME_LIST_QUE(n, x) (uint8_t *)&tx_frame_list##n##_que[x * NET_PKT_PER_QUE]
#define DEFN_TX_FRAME_LIST(n, x)                                                                   \
	.tx_frames = SYS_RINGQ_INIT(BUF_TX_FRAME_LIST_QUE(n, x), sizeof(struct net_pkt *),         \
				    NET_PKT_PER_QUE),
#else /* CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK */
#define DEFN_TX_FRAME_LIST(n, x)
#endif /* CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK */
#else  /*GMAC_MULTIPLE_TX_PACKETS*/
#define DEFN_TX_FLAG_LIST(n, x)
#define DEFN_TX_FRAME_LIST(n, x)
#endif /*GMAC_MULTIPLE_TX_PACKETS*/

#define BUF_RX_DESC_QUE(n, x) &rx_desc##n##_que[PRIORITY_QUEUE##x##_RX_DESC_IDX]
#define BUF_TX_DESC_QUE(n, x) &tx_desc##n##_que[PRIORITY_QUEUE##x##_TX_DESC_IDX]
#define DEFN_DATA_QUEUE_LIST(n, x)                                                                 \
	{.que_idx = GMAC_QUE_##x,                                                                  \
	 .rx_desc_list =                                                                           \
		 {                                                                                 \
			 .buf = BUF_RX_DESC_QUE(n, x),                                             \
			 .len = MAIN_QUEUE_RX_DESC_COUNT,                                          \
		 },                                                                                \
	 .tx_desc_list =                                                                           \
		 {                                                                                 \
			 .buf = BUF_TX_DESC_QUE(n, x),                                             \
			 .len = MAIN_QUEUE_TX_DESC_COUNT,                                          \
		 },                                                                                \
	 DEFN_RX_FLAG_LIST_##x(n) DEFN_TX_FLAG_LIST(n, x) DEFN_TX_FRAME_LIST(n, x)},
#define DEFN_DATA_QUEUE_LIST_0(n) DEFN_DATA_QUEUE_LIST(n, 0)
#if GMAC_PRIORITY_QUEUE_NUM >= 1

#define DEFN_DATA_QUEUE_LIST_1(n) DEFN_DATA_QUEUE_LIST(n, 1)
#else
#define DEFN_DATA_QUEUE_LIST_1(n)
#endif
#if GMAC_PRIORITY_QUEUE_NUM >= 2
#define DEFN_DATA_QUEUE_LIST_2(n) DEFN_DATA_QUEUE_LIST(n, 2)
#else
#define DEFN_DATA_QUEUE_LIST_2(n)
#endif
#if GMAC_PRIORITY_QUEUE_NUM >= 3
#define DEFN_DATA_QUEUE_LIST_3(n) DEFN_DATA_QUEUE_LIST(n, 3)
#else
#define DEFN_DATA_QUEUE_LIST_3(n)
#endif
#if GMAC_PRIORITY_QUEUE_NUM >= 4
#define DEFN_DATA_QUEUE_LIST_4(n) DEFN_DATA_QUEUE_LIST(n, 4)
#else
#define DEFN_DATA_QUEUE_LIST_4(n)
#endif
#if GMAC_PRIORITY_QUEUE_NUM >= 5
#define DEFN_DATA_QUEUE_LIST_5(n) DEFN_DATA_QUEUE_LIST(n, 5)
#else
#define DEFN_DATA_QUEUE_LIST_5(n)
#endif
#define MCHP_GMAC_DATA_DEFN(n)                                                                     \
	BUILD_ASSERT(GMAC_QUEUE_NUM >= DT_INST_PROP(n, num_queues),                                \
		     "The size of array queue_list[] is too small");                               \
	static struct eth_mchp_dev_data mchp_gmac_##n##_data = {                                   \
		.mac_addr = DT_INST_PROP_OR(n, local_mac_address, {0U}),                           \
		.queue_list = {DEFN_DATA_QUEUE_LIST_0(n) DEFN_DATA_QUEUE_LIST_1(                   \
			n) DEFN_DATA_QUEUE_LIST_2(n) DEFN_DATA_QUEUE_LIST_3(n)                     \
				       DEFN_DATA_QUEUE_LIST_4(n) DEFN_DATA_QUEUE_LIST_5(n)},       \
	};

#define MCHP_GMAC_DT_INST_DEFN(n)                                                                  \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_mchp_gmac_eth_initialize, NULL,                       \
				      &mchp_gmac_##n##_data, &mchp_gmac_##n##_config,              \
				      CONFIG_ETH_INIT_PRIORITY, &eth_mchp_gmac_api, GMAC_MTU);

#if defined(CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK)
static int eth_mchp_gmac_ptp_clock_set(const struct device *dev, struct net_ptp_time *tm)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	gmac_registers_t *const gmac = cfg->regs;

	gmac->GMAC_TSH = tm->_sec.high & 0xffff;
	gmac->GMAC_TSL = tm->_sec.low & 0xffffffff;
	gmac->GMAC_TN = tm->nanosecond & 0xffffffff;

	return 0;
}

static int eth_mchp_gmac_ptp_clock_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	gmac_registers_t *const gmac = cfg->regs;

	tm->second = ((uint64_t)(gmac->GMAC_TSH & 0xffff) << 32) | gmac->GMAC_TSL;
	tm->nanosecond = gmac->GMAC_TN;

	return 0;
}

static int eth_mchp_gmac_ptp_clock_mchp_adjust(const struct device *dev, int increment)
{
	const struct eth_mchp_dev_cfg *const cfg = dev->config;
	gmac_registers_t *const gmac = cfg->regs;

	if ((increment <= -(int)NSEC_PER_SEC) || (increment >= (int)NSEC_PER_SEC)) {
		return -EINVAL;
	}

	if (increment < 0) {
		gmac->GMAC_TA = GMAC_TA_ADJ_Msk | GMAC_TA_ITDT(-increment);
	} else {
		gmac->GMAC_TA = GMAC_TA_ITDT(increment);
	}

	return 0;
}

static int eth_mchp_gmac_ptp_clock_mchp_rate_adjust(const struct device *dev, double ratio)
{
	return -ENOTSUP;
}

static DEVICE_API(ptp_clock, eth_mchp_gmac_ptp_api) = {
	.set = eth_mchp_gmac_ptp_clock_set,
	.get = eth_mchp_gmac_ptp_clock_get,
	.adjust = eth_mchp_gmac_ptp_clock_mchp_adjust,
	.rate_adjust = eth_mchp_gmac_ptp_clock_mchp_rate_adjust,
};

#define MCHP_GMAC_PTP_CLOCK_DEFN(n)                                                                \
	DEVICE_DEFINE(gmac_ptp_clock_##n, PTP_CLOCK_NAME, NULL, NULL, &mchp_gmac_##n##_data,       \
		      &mchp_gmac_##n##_config, POST_KERNEL, CONFIG_ETH_INIT_PRIORITY,              \
		      &eth_mchp_gmac_ptp_api);
#else
#define MCHP_GMAC_PTP_CLOCK_DEFN(n)
#endif /* CONFIG_ETH_MCHP_GMAC_G2_PTP_CLOCK */

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	MCHP_GMAC_LISTS_DEFN(n)                                                                    \
                                                                                                   \
	MCHP_GMAC_IRQ_CONFIG_DEFN(n)                                                               \
	MCHP_GMAC_PINCTRL_DEFN(n)                                                                  \
                                                                                                   \
	MCHP_GMAC_CFG_DEFN(n)                                                                      \
	MCHP_GMAC_DATA_DEFN(n)                                                                     \
                                                                                                   \
	MCHP_GMAC_DT_INST_DEFN(n)                                                                  \
                                                                                                   \
	MCHP_GMAC_PTP_CLOCK_DEFN(n)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
