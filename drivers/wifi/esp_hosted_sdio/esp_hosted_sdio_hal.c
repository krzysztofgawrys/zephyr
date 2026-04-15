/*
 * Copyright (c) 2026 Krzysztof Gawrys
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted-MCU SDIO WiFi driver — main file.
 * Ported from project src/esp_hosted.c + src/shell.c init/rx sequence.
 *
 * Stage 1: card init + IRQ-driven RX + PRIV_IF event handling.
 * Stage 2 (TODO): WiFi mgmt API (scan/connect/disconnect).
 * Stage 3 (TODO): net_if / data path.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>
#include <endian.h>
#include <inttypes.h>

#include "esp_hosted_sdio.h"
#include "esp_hosted_sdio_cmd.h"
#include <pb_encode.h>
#include <pb_decode.h>

LOG_MODULE_REGISTER(esp_hosted_sdio, LOG_LEVEL_INF);

/* ── Thread stacks (single-instance assumption) ─────────────────────────── */
#define ESP_INIT_STACK_SIZE  8192
#define ESP_RX_STACK_SIZE    8192
#define ESP_THREAD_PRIO      5

K_THREAD_STACK_DEFINE(esp_init_stack, ESP_INIT_STACK_SIZE);
K_THREAD_STACK_DEFINE(esp_rx_stack,   ESP_RX_STACK_SIZE);

/* ── FN1 block/TX helpers (use driver data's io_buf as scratch) ─────────── */

static int hosted_set_blocksize(const struct device *sdhc,
				uint8_t fn, uint16_t bs,
				uint8_t *io_scratch)
{
	size_t off = SD_IO_FBR_START * fn;
	const uint8_t *bsu8 = (const uint8_t *)&bs;
	uint16_t bsr = 0;
	uint8_t *bsr_u8 = (uint8_t *)&bsr;

	sdio_cmd52_write(sdhc, SDIO_FUNC_0, off + SD_IO_CCCR_BLKSIZEL,
			 bsu8[0], NULL);
	sdio_cmd52_write(sdhc, SDIO_FUNC_0, off + SD_IO_CCCR_BLKSIZEH,
			 bsu8[1], NULL);
	sdio_cmd52_read(sdhc, SDIO_FUNC_0, off + SD_IO_CCCR_BLKSIZEL,
			&bsr_u8[0]);
	sdio_cmd52_read(sdhc, SDIO_FUNC_0, off + SD_IO_CCCR_BLKSIZEH,
			&bsr_u8[1]);

	LOG_INF("FN%d block size: %d", fn, bsr);
	return (bsr == bs) ? 0 : -EIO;
}

static int hosted_card_fn_init(const struct device *sdhc, uint8_t *io_scratch)
{
	uint8_t ioe = 0, ior = 0, ie = 0;
	int i;

	sdio_cmd52_read(sdhc, SDIO_FUNC_0, SD_IO_CCCR_FN_ENABLE, &ioe);
	LOG_INF("IOE: 0x%02x", ioe);

	ioe |= FUNC1_EN_MASK;
	sdio_cmd52_write(sdhc, SDIO_FUNC_0, SD_IO_CCCR_FN_ENABLE, ioe, &ioe);
	LOG_INF("IOE after enable: 0x%02x", ioe);

	for (i = 0; i < SDIO_INIT_MAX_RETRY; i++) {
		sdio_cmd52_read(sdhc, SDIO_FUNC_0, SD_IO_CCCR_FN_READY, &ior);
		if (ior & FUNC1_EN_MASK) {
			break;
		}
		k_msleep(10);
	}
	if (i >= SDIO_INIT_MAX_RETRY) {
		LOG_ERR("FN1 never became ready");
		return -ETIMEDOUT;
	}
	LOG_INF("IOR: 0x%02x", ior);

	sdio_cmd52_read(sdhc, SDIO_FUNC_0, SD_IO_CCCR_INT_ENABLE, &ie);
	ie |= BIT(0) | FUNC1_EN_MASK;
	sdio_cmd52_write(sdhc, SDIO_FUNC_0, SD_IO_CCCR_INT_ENABLE, ie, NULL);
	LOG_INF("IE: 0x%02x", ie);

	hosted_set_blocksize(sdhc, SDIO_FUNC_0, 512, io_scratch);
	hosted_set_blocksize(sdhc, SDIO_FUNC_1, 512, io_scratch);

	return 0;
}

/* ── RX buffer management ───────────────────────────────────────────────── */

static uint8_t *rx_get_buffer(esp_dbl_buf_t *db, uint32_t len)
{
	len = ((len + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;
	int idx = db->write_idx;

	if (len > db->buf_size[idx]) {
		k_free(db->buf[idx]);
		db->buf[idx] = k_malloc(len);
		db->buf_size[idx] = db->buf[idx] ? len : 0;
		if (!db->buf[idx]) {
			return NULL;
		}
	}
	return db->buf[idx];
}

/* ── RX length from slave ───────────────────────────────────────────────── */

static int get_len_from_slave(uint32_t *rx_size, uint32_t reg_val,
			      uint32_t *byte_count)
{
	uint32_t len = reg_val & ESP_SLAVE_LEN_MASK;

	if (len >= *byte_count) {
		len = (len + ESP_RX_BYTE_MAX - *byte_count) % ESP_RX_BYTE_MAX;
	} else {
		len = (ESP_RX_BYTE_MAX - *byte_count) + len;
	}

	*rx_size = len;
	*byte_count = (*byte_count + len) % ESP_RX_BYTE_MAX;
	return 0;
}

/* ── PRIV_IF event processing ───────────────────────────────────────────── */

static int process_init_event(const struct device *dev,
			      uint8_t *evt_buf, uint16_t len);

static void process_event(const struct device *dev,
			  uint8_t *evt_buf, uint16_t len)
{
	if (!evt_buf || !len) {
		return;
	}

	struct esp_priv_event *event = (struct esp_priv_event *)evt_buf;

	if (event->event_type == ESP_PRIV_EVENT_INIT) {
		LOG_INF("ESPInit event received");
		LOG_HEXDUMP_INF(event->event_data, event->event_len,
				"init_evt");
		process_init_event(dev, event->event_data, event->event_len);
	} else {
		LOG_WRN("Unknown priv event 0x%02x", event->event_type);
	}
}

/* ── TX path ────────────────────────────────────────────────────────────── */

int esp_hosted_sdio_tx(const struct device *dev, uint8_t if_type,
		       uint8_t if_num, uint8_t *buf, uint16_t len)
{
	const struct esp_hosted_sdio_config *cfg = dev->config;
	struct esp_hosted_sdio_data *data = dev->data;
	struct esp_payload_header *hdr;
	uint32_t total, blocks_len;

	total = len + ESP_FRAME_HEADER_SIZE;
	memset(data->tx_buf, 0, total);

	hdr = (struct esp_payload_header *)data->tx_buf;
	hdr->if_type  = if_type;
	hdr->if_num   = if_num;
	hdr->len      = htole16(len);
	hdr->offset   = htole16(ESP_FRAME_HEADER_SIZE);
	memcpy(data->tx_buf + ESP_FRAME_HEADER_SIZE, buf, len);

	blocks_len = ((total + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE) *
		     ESP_BLOCK_SIZE;

	return sdio_cmd53_write_blocks(cfg->sdhc, SDIO_FUNC_1,
				       ESP_SLAVE_CMD53_END_ADDR - total,
				       data->tx_buf, blocks_len,
				       data->io_buf);
}

/* ── send_slave_config ──────────────────────────────────────────────────── */

static int send_slave_config(const struct device *dev,
			     uint8_t chip_id)
{
#define LEN_1 1
	uint8_t *sendbuf = k_malloc(512);

	if (!sendbuf) {
		return -ENOMEM;
	}

	memset(sendbuf, 0, 512);
	struct esp_priv_event *event = (struct esp_priv_event *)sendbuf;

	event->event_type = ESP_PRIV_EVENT_INIT;

	uint8_t *pos = event->event_data;
	uint16_t tlen = 0;

	/* HOST_CAPABILITIES TLV */
	*pos++ = HOST_CAPABILITIES; tlen++;
	*pos++ = LEN_1;             tlen++;
	*pos++ = 0;                 tlen++;  /* no special host capabilities */

	/* RCVD_ESP_FIRMWARE_CHIP_ID TLV */
	*pos++ = RCVD_ESP_FIRMWARE_CHIP_ID; tlen++;
	*pos++ = LEN_1;                     tlen++;
	*pos++ = chip_id;                   tlen++;

	/* SLV_CONFIG_TEST_RAW_TP TLV */
	*pos++ = SLV_CONFIG_TEST_RAW_TP; tlen++;
	*pos++ = LEN_1;                  tlen++;
	*pos++ = 0;                      tlen++;

	/* SLV_CONFIG_THROTTLE_HIGH_THRESHOLD TLV */
	*pos++ = SLV_CONFIG_THROTTLE_HIGH_THRESHOLD; tlen++;
	*pos++ = LEN_1;                              tlen++;
	*pos++ = 80;                                 tlen++;

	/* SLV_CONFIG_THROTTLE_LOW_THRESHOLD TLV */
	*pos++ = SLV_CONFIG_THROTTLE_LOW_THRESHOLD; tlen++;
	*pos++ = LEN_1;                             tlen++;
	*pos++ = 60;                                tlen++;

	event->event_len = tlen;

	int ret = esp_hosted_sdio_tx(dev, ESP_PRIV_IF, 0,
				     sendbuf,
				     sizeof(struct esp_priv_event) + tlen);
	k_free(sendbuf);
	return ret;
}

/* ── process_init_event ─────────────────────────────────────────────────── */

static int process_init_event(const struct device *dev,
			      uint8_t *evt_buf, uint16_t len)
{
	struct esp_hosted_sdio_data *data = dev->data;
	uint8_t len_left = len;
	uint8_t *pos = evt_buf;
	uint8_t chip_id = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;

	if (!evt_buf) {
		return -EINVAL;
	}

	while (len_left >= 2) {
		uint8_t tag     = *pos;
		uint8_t tag_len = *(pos + 1);

		if (tag == ESP_PRIV_FIRMWARE_CHIP_ID) {
			chip_id = *(pos + 2);
			LOG_INF("ESPInit: chip=0x%02x", chip_id);
		} else if (tag == ESP_PRIV_CAPABILITY) {
			LOG_INF("ESPInit: cap=0x%02x", *(pos + 2));
		} else if (tag == ESP_PRIV_FIRMWARE_VERSION) {
			uint32_t fw = (uint32_t)(*(pos + 2)) |
				      ((uint32_t)(*(pos + 3)) << 8) |
				      ((uint32_t)(*(pos + 4)) << 16) |
				      ((uint32_t)(*(pos + 5)) << 24);
			LOG_INF("ESPInit: fw=0x%08" PRIx32, fw);
		} else if (tag == ESP_PRIV_RX_Q_SIZE) {
			LOG_INF("ESPInit: rx_q=%u", *(pos + 2));
		} else if (tag == ESP_PRIV_TX_Q_SIZE) {
			LOG_INF("ESPInit: tx_q=%u", *(pos + 2));
		}

		pos      += tag_len + 2;
		len_left -= tag_len + 2;
	}

	data->chip_type = chip_id;

	return send_slave_config(dev, chip_id);
}

/* ── RX one packet ──────────────────────────────────────────────────────── */

static int esp_hosted_rx_one(const struct device *dev)
{
	const struct esp_hosted_sdio_config *cfg = dev->config;
	struct esp_hosted_sdio_data *data = dev->data;
	uint8_t regs[REG_BUF_LEN] = {0};
	uint32_t interrupts;
	uint32_t len_from_slave;
	int ret;

	/* Poll up to 5s for new-packet bit */
	for (int i = 0; i < 100; i++) {
		ret = sdio_reg_read(cfg->sdhc, ESP_SLAVE_INT_RAW_REG,
				    regs, REG_BUF_LEN, data->io_buf);
		if (ret < 0) {
			return ret;
		}
		interrupts = *(uint32_t *)regs;
		if (interrupts & ESP_SLAVE_RX_NEW_PACKET_INT) {
			break;
		}
		k_msleep(50);
	}

	if (!(interrupts & ESP_SLAVE_RX_NEW_PACKET_INT)) {
		LOG_WRN("No new-packet int (raw=0x%08x)", interrupts);
		return -ETIMEDOUT;
	}

	LOG_DBG("INT_RAW=0x%08" PRIx32, interrupts);

	uint32_t pkt_len_reg = *(uint32_t *)&regs[PACKET_LEN_INDEX];

	ret = get_len_from_slave(&len_from_slave, pkt_len_reg,
				 &data->sdio_rx_byte_count);
	if (ret < 0 || len_from_slave == 0) {
		LOG_WRN("Bad rx len: %d", ret);
		return ret ? ret : -ENODATA;
	}
	LOG_DBG("RX len: %u", len_from_slave);

	/* Clear interrupt */
	ret = sdio_reg_write(cfg->sdhc, ESP_SLAVE_INT_CLR_REG,
			     &interrupts, sizeof(interrupts), data->io_buf);
	if (ret < 0) {
		return ret;
	}

	/* Allocate RX buffer */
	uint8_t *rxbuf = rx_get_buffer(&data->dbl_buf, len_from_slave);

	if (!rxbuf) {
		LOG_ERR("No RX buffer");
		return -ENOMEM;
	}

	/* Read packet (block-aligned) */
	uint32_t read_len = ((len_from_slave + ESP_BLOCK_SIZE - 1) /
			     ESP_BLOCK_SIZE) * ESP_BLOCK_SIZE;
	ret = esp_sdio_read_fromio(cfg->sdhc, SDIO_FUNC_1,
			       ESP_SLAVE_CMD53_END_ADDR - len_from_slave,
			       rxbuf, read_len, data->io_buf);
	if (ret < 0) {
		LOG_ERR("Block read failed: %d", ret);
		return ret;
	}

	/* Parse header */
	struct esp_payload_header *hdr = (struct esp_payload_header *)rxbuf;
	uint16_t plen   = le16toh(hdr->len);
	uint16_t offset = le16toh(hdr->offset);

	if (!plen || plen > MAX_PAYLOAD_SIZE ||
	    offset != ESP_FRAME_HEADER_SIZE) {
		LOG_WRN("Bad pkt header (len=%u off=%u)", plen, offset);
		return -EINVAL;
	}

	interface_buffer_handle_t bh = {
		.buf_handle  = rxbuf,
		.if_type     = hdr->if_type,
		.if_num      = hdr->if_num,
		.payload     = rxbuf + offset,
		.payload_len = plen,
		.seq_num     = le16toh(hdr->seq_num),
		.flag        = hdr->flags,
	};

	/* Dispatch */
	if (bh.if_type == ESP_PRIV_IF) {
		process_event(dev, bh.payload, bh.payload_len);
	} else if (bh.if_type == ESP_SERIAL_IF) {
		/* TLV: [0x01][6][0]["RPCRsp"][0x02][pb_lo][pb_hi][pb...] */
		if (bh.payload_len < ESP_TLV_HDR_LEN) {
			LOG_WRN("SERIAL_IF: payload too short (%u)", bh.payload_len);
			return -EINVAL;
		}
		uint16_t pb_len = bh.payload[10] |
				  ((uint16_t)bh.payload[11] << 8);
		if ((uint32_t)ESP_TLV_HDR_LEN + pb_len > bh.payload_len) {
			LOG_WRN("SERIAL_IF: pb_len %u exceeds payload %u",
				pb_len, bh.payload_len);
			return -EINVAL;
		}
		Rpc *resp = data->rpc_resp_ptr;

		if (resp) {
			/* Decode into caller's buffer (callbacks pre-set by caller).
			 * Use pb_decode_noinit so nanopb does NOT zero the struct
			 * first — callers pre-set decode callbacks before calling
			 * rpc_call and those must survive into the decode. */
			pb_istream_t is = pb_istream_from_buffer(
					bh.payload + ESP_TLV_HDR_LEN, pb_len);
			if (pb_decode_noinit(&is, Rpc_fields, resp)) {
				LOG_DBG("SERIAL_IF: RPC decoded ok (msg_id=%d)",
					resp->msg_id);
				k_sem_give(&data->rpc_resp_sem);
			} else {
				LOG_ERR("SERIAL_IF pb_decode: %s",
					PB_GET_ERROR(&is));
			}
		} else {
			/* Unsolicited packet — could be an async event */
			esp_hosted_process_serial_event(dev,
					bh.payload + ESP_TLV_HDR_LEN, pb_len);
		}
	} else if (bh.if_type == ESP_STA_IF || bh.if_type == ESP_AP_IF) {
		esp_hosted_iface_recv(dev, bh.payload, bh.payload_len);
	} else {
		LOG_WRN("Unknown if_type=%d", bh.if_type);
	}

	/* Advance double-buffer write index */
	data->dbl_buf.write_idx ^= 1;

	return 0;
}

/* ── IRQ callback ───────────────────────────────────────────────────────── */

static void esp_sdio_irq_cb(const struct device *sdhc, int reason,
			    const void *user_data)
{
	const struct device *dev = user_data;
	struct esp_hosted_sdio_data *data = dev->data;

	ARG_UNUSED(sdhc);

	if (reason & SDHC_INT_SDIO)
		k_sem_give(&data->irq_sem);
}

/* ── RX thread ──────────────────────────────────────────────────────────── */

static void rx_thread_fn(void *a, void *b, void *c)
{
	const struct device *dev = a;
	const struct esp_hosted_sdio_config *cfg = dev->config;
	struct esp_hosted_sdio_data *data = dev->data;

	LOG_INF("rx_thread: started");

	while (true) {
		int ret = k_sem_take(&data->irq_sem, K_FOREVER);

		if (ret == 0) {
			esp_hosted_rx_one(dev);
			/* Re-arm SDIO interrupt for next packet */
			sdhc_enable_interrupt(cfg->sdhc, esp_sdio_irq_cb,
					      SDHC_INT_SDIO,
					      (void *)dev);
		}
	}
}

/* ── Init thread: full card init sequence ───────────────────────────────── */

static void init_thread_fn(void *a, void *b, void *c)
{
	const struct device *dev = a;
	const struct esp_hosted_sdio_config *cfg = dev->config;
	struct esp_hosted_sdio_data *data = dev->data;
	int ret;
	uint16_t rca;

	LOG_INF("esp_hosted_sdio: init_thread started");

	/* 1. Set IO: 400kHz, 1-bit, push-pull, 3.3V */
	struct sdhc_io io = {
		.clock         = 400000,
		.bus_mode      = SDHC_BUSMODE_PUSHPULL,
		.power_mode    = SDHC_POWER_ON,
		.bus_width     = SDHC_BUS_WIDTH1BIT,
		.timing        = SDHC_TIMING_LEGACY,
		.signal_voltage = SD_VOL_3_3_V,
	};
	ret = sdhc_set_io(cfg->sdhc, &io);
	if (ret) {
		LOG_ERR("sdhc_set_io failed: %d", ret);
	}

	/* 2. CCCR reset (expect -ETIMEDOUT for ESP32C5, normal) */
	sdio_cccr_reset(cfg->sdhc);

	/* 3. CMD0 */
	ret = sdio_cmd0_go_idle(cfg->sdhc);
	if (ret) {
		LOG_ERR("CMD0 failed: %d", ret);
	}

	/* 4. CMD5 x2 */
	ret = sdio_cccr_io_init(cfg->sdhc);
	if (ret) {
		LOG_ERR("CMD5 io init failed: %d", ret);
		return;
	}

	/* 5. CMD3 — get RCA */
	ret = sdio_cmd3_send_relative_addr(cfg->sdhc, &rca);
	if (ret) {
		LOG_WRN("CMD3 failed (%d), using RCA=0x0001", ret);
		rca = 0x0001;
	}
	LOG_INF("RCA: 0x%04x", rca);

	/* 6. CMD7 — select card */
	ret = sdio_cmd7_select_card(cfg->sdhc, rca);
	if (ret) {
		LOG_ERR("CMD7 failed: %d", ret);
		return;
	}

	/* 7. FN1 init: enable FN1, interrupts, set block size 512 */
	ret = hosted_card_fn_init(cfg->sdhc, data->io_buf);
	if (ret) {
		LOG_ERR("FN1 init failed: %d", ret);
		return;
	}

	/* 8. Send OPEN_DATA_PATH interrupt to slave */
	uint8_t intr_mask = BIT(ESP_OPEN_DATA_PATH + ESP_SDIO_CONF_OFFSET);
	uint32_t reg = HOST_TO_SLAVE_INTR & ESP_ADDRESS_MASK;

	ret = sdio_cmd52_write(cfg->sdhc, SDIO_FUNC_1, reg, intr_mask, NULL);
	if (ret) {
		LOG_ERR("OPEN_DATA_PATH failed: %d", ret);
		return;
	}
	LOG_INF("OPEN_DATA_PATH sent");

	data->initialized = true;

	/* 9. Enable SDIO interrupt and start RX thread */
	sdhc_enable_interrupt(cfg->sdhc, esp_sdio_irq_cb,
			      SDHC_INT_SDIO, (void *)dev);

	k_thread_create(&data->rx_thread, esp_rx_stack,
			K_THREAD_STACK_SIZEOF(esp_rx_stack),
			rx_thread_fn, (void *)dev, NULL, NULL,
			ESP_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "esp_rx");

	/* 10. WifiInit — cfg must be populated (slave validates magic) */
	{
		Rpc req = Rpc_init_zero, resp = Rpc_init_zero;

		req.msg_type      = RpcType_Req;
		req.msg_id        = RpcId_Req_WifiInit;
		req.which_payload = Rpc_req_wifi_init_tag;
		req.payload.req_wifi_init.has_cfg        = true;
		req.payload.req_wifi_init.cfg.magic           = 0x1F2F3F4F;
		req.payload.req_wifi_init.cfg.static_rx_buf_num  = 10;
		req.payload.req_wifi_init.cfg.dynamic_rx_buf_num = 32;
		req.payload.req_wifi_init.cfg.tx_buf_type        = 1;
		req.payload.req_wifi_init.cfg.static_tx_buf_num  = 0;
		req.payload.req_wifi_init.cfg.dynamic_tx_buf_num = 32;
		req.payload.req_wifi_init.cfg.cache_tx_buf_num   = 0;
		req.payload.req_wifi_init.cfg.ampdu_rx_enable    = 1;
		req.payload.req_wifi_init.cfg.ampdu_tx_enable    = 1;
		req.payload.req_wifi_init.cfg.nvs_enable         = 0;
		req.payload.req_wifi_init.cfg.nano_enable        = 0;
		req.payload.req_wifi_init.cfg.rx_ba_win          = 6;
		req.payload.req_wifi_init.cfg.wifi_task_core_id  = 0;
		req.payload.req_wifi_init.cfg.beacon_max_len     = 752;
		req.payload.req_wifi_init.cfg.mgmt_sbuf_num      = 32;
		req.payload.req_wifi_init.cfg.feature_caps       = 121;
		if (esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(5000))) {
			LOG_WRN("WifiInit RPC failed");
		} else {
			LOG_INF("WifiInit ok");
		}
	}

	/* 11. SetWifiMode(STA) */
	{
		Rpc req = Rpc_init_zero, resp = Rpc_init_zero;

		req.msg_type      = RpcType_Req;
		req.msg_id        = RpcId_Req_SetWifiMode;
		req.which_payload = Rpc_req_set_wifi_mode_tag;
		req.payload.req_set_wifi_mode.mode = 1; /* WIFI_MODE_STA */
		if (esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(3000))) {
			LOG_WRN("SetWifiMode(STA) RPC failed");
		} else {
			LOG_INF("SetWifiMode(STA) ok");
		}
	}

	/* 12. WifiStart */
	{
		Rpc req = Rpc_init_zero, resp = Rpc_init_zero;

		req.msg_type      = RpcType_Req;
		req.msg_id        = RpcId_Req_WifiStart;
		req.which_payload = Rpc_req_wifi_start_tag;
		if (esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(3000))) {
			LOG_WRN("WifiStart RPC failed");
		} else {
			LOG_INF("WifiStart ok");
		}
	}

	/* 13. GetMACAddress — retry: WifiStart on slave is async, the WiFi
	 *     stack may not be ready immediately after the RPC response. */
	{
		int attempt;

		for (attempt = 0; attempt < 5; attempt++) {
			k_msleep(200);
			if (esp_hosted_get_mac(dev) == 0) {
				break;
			}
			LOG_DBG("GetMACAddress not ready, retry %d/5", attempt + 1);
		}
		if (attempt == 5) {
			LOG_WRN("GetMACAddress failed after 5 retries, using fallback MAC");
		}
	}

	LOG_INF("esp_hosted_sdio: init complete, rx_thread started");
}

/* ── Device driver init ─────────────────────────────────────────────────── */

int esp_hosted_sdio_init(const struct device *dev)
{
	const struct esp_hosted_sdio_config *cfg = dev->config;
	struct esp_hosted_sdio_data *data = dev->data;

	if (!device_is_ready(cfg->sdhc)) {
		LOG_ERR("SDHC host not ready");
		return -ENODEV;
	}

	k_sem_init(&data->irq_sem, 0, 1);
	k_mutex_init(&data->rpc_lock);
	k_sem_init(&data->rpc_resp_sem, 0, 1);
	data->rpc_resp_ptr = NULL;
	data->initialized = false;
	data->sdio_rx_byte_count = 0;
	data->chip_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
	memset(&data->dbl_buf, 0, sizeof(data->dbl_buf));

	/* Kick off the async init thread immediately. */
	k_thread_create(&data->init_thread, esp_init_stack,
			K_THREAD_STACK_SIZEOF(esp_init_stack),
			init_thread_fn, (void *)dev, NULL, NULL,
			ESP_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&data->init_thread, "esp_init");
	return 0;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int esp_hosted_sdio_rpc_call(const struct device *dev,
			     Rpc *req, Rpc *resp,
			     k_timeout_t timeout)
{
	struct esp_hosted_sdio_data *data = dev->data;

	if (!data->initialized) {
		return -EAGAIN;
	}

	/* Koduj zapytanie do bufora TLV+pb */
	uint8_t *tx = k_malloc(ESP_TLV_HDR_LEN + ESP_RPC_PB_MAX);

	if (!tx) {
		return -ENOMEM;
	}

	/* TLV nagłówek: EP name = "RPCRsp" (6 bajtów) */
	tx[0] = ESP_TLV_EP_NAME;
	tx[1] = 0x06; tx[2] = 0x00;
	memcpy(&tx[3], "RPCRsp", 6);
	tx[9]  = ESP_TLV_DATA;
	tx[10] = 0x00; tx[11] = 0x00; /* wypełni się po encode */

	pb_ostream_t os = pb_ostream_from_buffer(&tx[ESP_TLV_HDR_LEN],
						 ESP_RPC_PB_MAX);

	if (!pb_encode(&os, Rpc_fields, req)) {
		LOG_ERR("rpc_call pb_encode: %s", PB_GET_ERROR(&os));
		k_free(tx);
		return -EINVAL;
	}

	uint16_t pb_len = (uint16_t)os.bytes_written;

	tx[10] = pb_len & 0xFF;
	tx[11] = pb_len >> 8;

	/* Tylko jedno RPC w locie naraz */
	if (k_mutex_lock(&data->rpc_lock, K_MSEC(500)) != 0) {
		k_free(tx);
		return -EBUSY;
	}

	data->rpc_resp_ptr = resp;
	k_sem_reset(&data->rpc_resp_sem);

	int ret = esp_hosted_sdio_tx(dev, ESP_SERIAL_IF, 0,
				     tx, ESP_TLV_HDR_LEN + pb_len);
	k_free(tx);

	if (ret) {
		data->rpc_resp_ptr = NULL;
		k_mutex_unlock(&data->rpc_lock);
		return ret;
	}

	/* Czekaj na odpowiedź z rx_thread */
	ret = k_sem_take(&data->rpc_resp_sem, timeout);
	data->rpc_resp_ptr = NULL;
	k_mutex_unlock(&data->rpc_lock);
	return ret ? -ETIMEDOUT : 0;
}

/**
 * @brief Start the ESP-Hosted SDIO driver.
 *
 * Runs the full card init sequence (CMD0/5/3/7, FN1, ESPInit handshake)
 * in a background thread and then starts the IRQ-driven RX loop.
 * Safe to call once after device init.
 */
int esp_hosted_sdio_start(const struct device *dev)
{
	struct esp_hosted_sdio_data *data = dev->data;

	if (data->initialized) {
		LOG_WRN("Already initialized");
		return -EALREADY;
	}

	k_thread_create(&data->init_thread, esp_init_stack,
			K_THREAD_STACK_SIZEOF(esp_init_stack),
			init_thread_fn, (void *)dev, NULL, NULL,
			ESP_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&data->init_thread, "esp_init");
	return 0;
}

/* ── Device instance (single-instance, instantiated by esp_hosted_sdio_wifi.c) */

struct esp_hosted_sdio_data esp_hosted_sdio_data_0;

const struct esp_hosted_sdio_config esp_hosted_sdio_cfg_0 = {
	.sdhc = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(0))),
};
