/*
 * Copyright (c) 2026 Krzysztof Gawrys
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WIFI_ESP_HOSTED_SDIO_H_
#define ZEPHYR_DRIVERS_WIFI_ESP_HOSTED_SDIO_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <stdint.h>
#include <stdbool.h>
#include <pb.h>
#include <esp_hosted_rpc.pb.h>

#define DT_DRV_COMPAT kg_esp_hosted_sdio

/* ── SDIO command argument helpers ─────────────────────────────────────── */
#define SD_ARG_CMD52_READ        (0 << 31)
#define SD_ARG_CMD52_WRITE       (1u << 31)
#define SD_ARG_CMD52_FUNC_SHIFT  28
#define SD_ARG_CMD52_FUNC_MASK   0x7
#define SD_ARG_CMD52_EXCHANGE    (1 << 27)
#define SD_ARG_CMD52_REG_SHIFT   9
#define SD_ARG_CMD52_REG_MASK    0x1ffff
#define SD_ARG_CMD52_DATA_SHIFT  0
#define SD_ARG_CMD52_DATA_MASK   0xff
#define SD_R5_DATA(resp)         ((resp)[0] & 0xff)

#define SD_ARG_CMD53_READ        (0 << 31)
#define SD_ARG_CMD53_WRITE       (1u << 31)
#define SD_ARG_CMD53_FUNC_SHIFT  28
#define SD_ARG_CMD53_FUNC_MASK   0x7
#define SD_ARG_CMD53_BLOCK_MODE  (1 << 27)
#define SD_ARG_CMD53_INCREMENT   (1 << 26)
#define SD_ARG_CMD53_REG_SHIFT   9
#define SD_ARG_CMD53_REG_MASK    0x1ffff
#define SD_ARG_CMD53_LENGTH_SHIFT 0
#define SD_ARG_CMD53_LENGTH_MASK  0x1ff

#define SDMMC_IO_FIXED_ADDR      BIT(31)

/* ── SDIO card commands ─────────────────────────────────────────────────── */
#define MMC_GO_IDLE_STATE        0
#define MMC_SELECT_CARD          7
#define SD_SEND_RELATIVE_ADDR    3
#define SD_IO_SEND_OP_COND       5
#define SD_IO_RW_DIRECT          52
#define SD_IO_RW_EXTENDED        53

/* ── SDIO OCR helpers ───────────────────────────────────────────────────── */
#define SD_IO_OCR_MEM_READY      (1u << 31)
#define SD_IO_OCR_NUM_FUNCTIONS(ocr) (((ocr) >> 28) & 0x7)
#define SD_IO_OCR_MEM_PRESENT    (1 << 27)
#define SD_IO_OCR_MASK           0x00fffff0
#define SD_OCR_VOL_MASK          0xFF8000
#define MMC_ARG_RCA(rca)         ((rca) << 16)
#define SD_R6_RCA(resp)          ((resp)[0] >> 16)
#define MMC_R4(resp)             ((resp)[0])

/* ── CCCR registers ─────────────────────────────────────────────────────── */
#define SD_IO_CCCR_FN_ENABLE     0x02
#define SD_IO_CCCR_FN_READY      0x03
#define SD_IO_CCCR_INT_ENABLE    0x04
#define SD_IO_CCCR_CTL           0x06
#define CCCR_CTL_RES             (1 << 3)
#define SD_IO_CCCR_BUS_WIDTH     0x07
#define SD_IO_CCCR_CARD_CAP      0x08
#define SD_IO_CCCR_BLKSIZEL      0x10
#define SD_IO_CCCR_BLKSIZEH      0x11
#define SD_IO_FBR_START          0x00100

/* ── SDIO functions ─────────────────────────────────────────────────────── */
#define SDIO_FUNC_0              0
#define SDIO_FUNC_1              1
#define FUNC1_EN_MASK            BIT(1)
#define SDIO_INIT_MAX_RETRY      10

/* ── ESP slave register map ─────────────────────────────────────────────── */
#define ESP_SLAVE_SLCHOST_BASE   0x3FF55000
#define ESP_ADDRESS_MASK         0x3FF

#define ESP_SLAVE_INT_RAW_REG    (ESP_SLAVE_SLCHOST_BASE + 0x50)
#define ESP_SLAVE_PACKET_LEN_REG (ESP_SLAVE_SLCHOST_BASE + 0x60)
#define ESP_SLAVE_INT_CLR_REG    (ESP_SLAVE_SLCHOST_BASE + 0xD4)
#define ESP_SLAVE_SCRATCH_REG_7  (ESP_SLAVE_SLCHOST_BASE + 0x8C)
#define HOST_TO_SLAVE_INTR       ESP_SLAVE_SCRATCH_REG_7

#define ESP_SLAVE_RX_NEW_PACKET_INT BIT(23)

#define REG_BUF_LEN              (ESP_SLAVE_PACKET_LEN_REG - ESP_SLAVE_INT_RAW_REG + 4)
#define PACKET_LEN_INDEX         (ESP_SLAVE_PACKET_LEN_REG - ESP_SLAVE_INT_RAW_REG)

/* ── Data path ──────────────────────────────────────────────────────────── */
#define ESP_SLAVE_CMD53_END_ADDR 0x1F800
#define ESP_SLAVE_LEN_MASK       0xFFFFF
#define ESP_BLOCK_SIZE           512
#define ESP_RX_BYTE_MAX          0x100000
#define ESP_RX_BUFFER_SIZE       1536
#define ESP_TX_BUFFER_SIZE       1536

#define H_SDIO_RX_LEN_TO_TRANSFER(x)    (((x) + 3) & (~3))
#define H_SDIO_TX_LEN_TO_TRANSFER(x)    (((x) + 3) & (~3))
#define H_SDIO_RX_BLOCKS_TO_TRANSFER(x) ((x) / ESP_BLOCK_SIZE)
#define H_SDIO_TX_BLOCKS_TO_TRANSFER(x) ((x) / ESP_BLOCK_SIZE)

/* ── ESP hosted packet interfaces ──────────────────────────────────────── */
typedef enum {
	ESP_INVALID_IF = 0,
	ESP_STA_IF,
	ESP_AP_IF,
	ESP_SERIAL_IF,
	ESP_HCI_IF,
	ESP_PRIV_IF,
	ESP_TEST_IF,
	ESP_MAX_IF,
} esp_hosted_if_type_t;

typedef enum {
	ESP_OPEN_DATA_PATH = 0,
	ESP_CLOSE_DATA_PATH,
	ESP_RESET,
	ESP_MAX_HOST_INTERRUPT,
} esp_host_interrupt_t;

#define ESP_SDIO_CONF_OFFSET     0

/* ── Packet header ──────────────────────────────────────────────────────── */
struct esp_payload_header {
	uint8_t  if_type:4;
	uint8_t  if_num:4;
	uint8_t  flags;
	uint16_t len;
	uint16_t offset;
	uint16_t checksum;
	uint16_t seq_num;
	uint8_t  reserved2;
	union {
		uint8_t reserved3;
		uint8_t hci_pkt_type;
		uint8_t priv_pkt_type;
	};
} __packed;

#define ESP_FRAME_HEADER_SIZE    sizeof(struct esp_payload_header)
#define MAX_PAYLOAD_SIZE         (ESP_RX_BUFFER_SIZE - ESP_FRAME_HEADER_SIZE)

/* ── Private event ──────────────────────────────────────────────────────── */
struct esp_priv_event {
	uint8_t event_type;
	uint8_t event_len;
	uint8_t event_data[];
} __packed;

typedef enum {
	ESP_PRIV_EVENT_INIT = 0x22,
} esp_priv_event_type_t;

typedef enum {
	ESP_PRIV_CAPABILITY    = 0x11,
	ESP_PRIV_FIRMWARE_CHIP_ID,
	ESP_PRIV_TEST_RAW_TP,
	ESP_PRIV_RX_Q_SIZE,
	ESP_PRIV_TX_Q_SIZE,
	ESP_PRIV_CAP_EXT,
	ESP_PRIV_FIRMWARE_VERSION,
} esp_priv_tag_type_t;

typedef enum {
	HOST_CAPABILITIES = 0x44,
	RCVD_ESP_FIRMWARE_CHIP_ID,
	SLV_CONFIG_TEST_RAW_TP,
	SLV_CONFIG_THROTTLE_HIGH_THRESHOLD,
	SLV_CONFIG_THROTTLE_LOW_THRESHOLD,
} slave_config_priv_tag_t;

/* ── Chip IDs ───────────────────────────────────────────────────────────── */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED 0xff
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        0x0
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      0x2
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      0x5
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      0x9
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      0xC
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      0xD
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C5      0x17

/* ── Buffer handle (RX dispatch) ────────────────────────────────────────── */
typedef struct {
	void    *buf_handle;
	uint8_t  if_type;
	uint8_t  if_num;
	uint8_t *payload;
	uint8_t  flag;
	uint16_t payload_len;
	uint16_t seq_num;
} interface_buffer_handle_t;

/* ── Double-buffer for RX ───────────────────────────────────────────────── */
typedef struct {
	uint8_t *buf[2];
	uint32_t buf_size[2];
	int      write_idx;
} esp_dbl_buf_t;

/* ── TLV header format (SERIAL_IF / RPC) ────────────────────────────────── */
/* [0x01][len_lo][len_hi]["RPCRsp"][0x02][pb_lo][pb_hi][protobuf...] */
#define ESP_TLV_EP_NAME   0x01
#define ESP_TLV_DATA      0x02
#define ESP_TLV_HDR_LEN   12   /* 3 + 6 ("RPCRsp") + 3 */
#define ESP_RPC_PB_MAX    512  /* max protobuf payload per RPC call */

/* ── Driver data ────────────────────────────────────────────────────────── */
struct esp_hosted_sdio_data {
	struct k_sem    irq_sem;
	struct k_thread rx_thread;
	struct k_thread init_thread;
	bool            initialized;
	uint32_t        sdio_rx_byte_count;
	uint8_t         chip_type;
	esp_dbl_buf_t   dbl_buf;
	/* RPC synchronization */
	struct k_mutex  rpc_lock;      /* serialises concurrent RPC callers */
	struct k_sem    rpc_resp_sem;  /* rx_thread signals response arrival */
	Rpc            *rpc_resp_ptr;  /* pointer to caller's response buffer */
	/* net_if / WiFi mgmt (Stage 3) */
	struct net_if       *iface;
	uint8_t              mac_addr[6];
	int                  state;        /* enum wifi_iface_state */
	/* Scan state — non-blocking */
	scan_result_cb_t     scan_cb;
	const struct device *scan_dev;
	struct k_work        scan_done_work;
	/* TX scratch buffer — NOT .nocache (HAL copies to DMA internally) */
	uint8_t tx_buf[ESP_TX_BUFFER_SIZE] __aligned(32);
	/* CMD53 IO scratch buffer — same constraint */
	uint8_t io_buf[ESP_BLOCK_SIZE] __aligned(32);
};

struct esp_hosted_sdio_config {
	const struct device *sdhc;
};

/* ── Global instance (single-instance driver) ───────────────────────────── */
extern struct esp_hosted_sdio_data       esp_hosted_sdio_data_0;
extern const struct esp_hosted_sdio_config esp_hosted_sdio_cfg_0;

/* ── Public API ─────────────────────────────────────────────────────────── */

/** Device init function (called by NET_DEVICE_DT_INST_DEFINE). */
int esp_hosted_sdio_init(const struct device *dev);

/** Start the full init sequence + RX thread (async). */
int esp_hosted_sdio_start(const struct device *dev);

/** Send a packet to the slave. */
int esp_hosted_sdio_tx(const struct device *dev, uint8_t if_type, uint8_t if_num,
		       uint8_t *buf, uint16_t len);

/**
 * Send an RPC request and block until the slave responds (or timeout).
 *
 * @param dev     esp_hosted_sdio device
 * @param req     request to encode (must be fully populated)
 * @param resp    buffer for decoded response (caller must zero-init before call)
 * @param timeout e.g. K_MSEC(3000)
 * @return 0 on success, -ETIMEDOUT, -EBUSY, -EINVAL or other negative errno
 */
int esp_hosted_sdio_rpc_call(const struct device *dev,
			     Rpc *req, Rpc *resp,
			     k_timeout_t timeout);

/** Get MAC address from slave and store in data->mac_addr (Stage 3). */
int esp_hosted_get_mac(const struct device *dev);

/** Forward received STA/AP Ethernet frame to net_if RX path (Stage 3). */
void esp_hosted_iface_recv(const struct device *dev, uint8_t *payload,
			   uint16_t payload_len);

/** Handle unsolicited SERIAL_IF events (StaConnected, StaDisconnected…). */
void esp_hosted_process_serial_event(const struct device *dev,
				     const uint8_t *pb_buf, uint16_t pb_len);

/** WiFi offload API struct (defined in esp_hosted_sdio_wifi.c). */
extern const struct net_wifi_mgmt_offload esp_hosted_sdio_api;

#endif /* ZEPHYR_DRIVERS_WIFI_ESP_HOSTED_SDIO_H_ */
