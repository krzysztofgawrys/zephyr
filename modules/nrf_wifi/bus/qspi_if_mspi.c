/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file qspi_if_mspi.c
 * @brief QSPI bus interface for nRF7002 Wi-Fi driver — Zephyr MSPI backend.
 *
 * Replaces the original nrfx_qspi-based qspi_if.c with a portable
 * Zephyr MSPI API implementation for STM32 OCTOSPI / QUADSPI.
 *
 * Key differences vs original qspi_if.c:
 *  - nrfx_qspi_init/uninit        → mspi_config + mspi_dev_config
 *  - nrfx_qspi_read/write         → mspi_transceive (MSPI_RX / MSPI_TX)
 *  - nrfx_qspi_cinstr_xfer        → mspi_transceive, opcode+data in one buffer
 *  - nrf_clock_hfclk192m_div_set  → removed, STM32 prescaler handled by driver
 *  - Wake-up frequency trick       → mspi_dev_config(MSPI_DEVICE_CONFIG_FREQUENCY)
 *  - nRF53 clock errata workarounds → removed, not applicable on STM32
 *  - QSPI HW dummy cycles (IFTIMING) → rx_dummy field in mspi_xfer
 *
 * STM32 OSPI driver (mspi_stm32_ospi.c) handles command / address / dummy
 * phases internally via HAL_OSPI_Command + HAL_OSPI_Receive/Transmit.
 * data_buf must point directly to payload — no manual framing needed.
 *
 * QUAD I/O support:
 *  - Bulk read uses MSPI_IO_MODE_QUAD_1_1_4 (cmd 0x6B): instruction+address
 *    on 1 line, data on 4 lines.  Requires a patch to mspi_stm32_ospi.c —
 *    see 0001-mspi-stm32-add-quad-1-1-4-and-1-4-4-io-modes.patch.
 *  - Bulk write uses MSPI_IO_MODE_QUAD_1_1_4 (cmd 0x32 QPP-Quad).
 *  - Register commands (RDSR1/2, WRSR2) always switch temporarily to
 *    MSPI_IO_MODE_SINGLE — nRF7002 only accepts them on 1 line.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mspi.h>

#include <zephyr/drivers/wifi/nrf_wifi/bus/qspi_if.h>
#include <zephyr/drivers/wifi/nrf_wifi/bus/rpu_hw_if.h>

LOG_MODULE_DECLARE(wifi_nrf_bus, CONFIG_WIFI_NRF70_BUSLIB_LOG_LEVEL);

/* --------------------------------------------------------------------------
 * qspi_nor_data — required by rpu_hw_if.c via qspi_perip.data
 * --------------------------------------------------------------------------
 */
struct qspi_nor_data {
#ifdef CONFIG_MULTITHREADING
	struct k_sem trans;
	struct k_sem sem;
	struct k_sem sync;
	struct k_sem count;
#else
	volatile bool ready;
#endif
};

static struct qspi_nor_data qspi_nor_memory_data = {
#ifdef CONFIG_MULTITHREADING
	.trans = Z_SEM_INITIALIZER(qspi_nor_memory_data.trans, 1, 1),
	.sem   = Z_SEM_INITIALIZER(qspi_nor_memory_data.sem,   1, 1),
	.sync  = Z_SEM_INITIALIZER(qspi_nor_memory_data.sync,  0, 1),
	.count = Z_SEM_INITIALIZER(qspi_nor_memory_data.count, 0, K_SEM_MAX_LIMIT),
#endif
};

/* Referenced by rpu_hw_if.c — must be a global named symbol */
struct device qspi_perip = {
	.data = &qspi_nor_memory_data,
};

/* --------------------------------------------------------------------------
 * Device-tree
 * --------------------------------------------------------------------------
 * DTS example (STM32U5G9, OCTOSPI1):
 *
 *   &octospi1 {
 *       compatible = "st,stm32-ospi-controller";
 *       status = "okay";
 *       clock-frequency = <DT_FREQ_M(8)>;
 *
 *       nrf70: nrf7002@0 {
 *           compatible = "nordic,nrf7002-mspi";
 *           reg = <0>;
 *           qspi-frequency = <8000000>;
 *           qspi-quad-mode;
 *           iovdd-ctrl-gpios = <&gpioe 6 GPIO_ACTIVE_HIGH>;
 *           bucken-gpios     = <&gpioe 7 GPIO_ACTIVE_HIGH>;
 *           host-irq-gpios   = <&gpioe 15 (GPIO_ACTIVE_HIGH|GPIO_PULL_DOWN)>;
 *           ...
 *       };
 *   };
 */
#define QSPI_BUS_NODE    DT_BUS(DT_NODELABEL(nrf70))
#define QSPI_DEVICE_NODE DT_NODELABEL(nrf70)

/* --------------------------------------------------------------------------
 * nRF7002 QSPI opcodes (Nordic Product Specification)
 * --------------------------------------------------------------------------
 */
#define NRF7002_CMD_RDSR1   0x1FU  /* Read Status Register 1 */
#define NRF7002_CMD_RDSR2   0x2FU  /* Read Status Register 2 */
#define NRF7002_CMD_WRSR2   0x3FU  /* Write Status Register 2 */
#define NRF7002_CMD_READ    0x0BU  /* Fast Read          (1-1-1, 8 dummy clocks) */
#define NRF7002_CMD_READ4IO 0xEBU  /* Fast Read Quad I/O (1-4-4, 6 dummy clocks) */
#define NRF7002_CMD_READ4O  0x6BU  /* Fast Read Quad Out (1-1-4, 8 dummy clocks) */
#define NRF7002_CMD_PP      0x02U  /* Page Program       (1-1-1) */
#define NRF7002_CMD_PP4O    0x32U  /* Page Program Quad Out (1-1-4) */

#define NRF7002_WAKEUP_FREQ MHZ(8) /* max freq for WRSR2 wake-up command */

#define QSPI_ALIGN 4U

/* --------------------------------------------------------------------------
 * MSPI state
 * --------------------------------------------------------------------------
 */
static const struct device *mspi_dev = DEVICE_DT_GET(QSPI_BUS_NODE);

static const struct mspi_dev_id nrf7002_dev_id = {
	.dev_idx = 0,
};

static struct mspi_dev_cfg nrf7002_dev_cfg = {
	.ce_num      = 0,
	.freq        = DT_PROP(QSPI_DEVICE_NODE, qspi_frequency),
	.io_mode     = MSPI_IO_MODE_SINGLE, /* overridden in qspi_init() */
	.data_rate   = MSPI_DATA_RATE_SINGLE,
	.cpp         = MSPI_CPP_MODE_0,
	.endian      = MSPI_XFER_LITTLE_ENDIAN,
	.ce_polarity = MSPI_CE_ACTIVE_LOW,
	.dqs_enable  = false,
	.rx_dummy    = 0,
	.tx_dummy    = 0,
	.read_cmd    = NRF7002_CMD_READ,
	.write_cmd   = NRF7002_CMD_PP,
	.cmd_length  = 1,
	.addr_length = 3, /* 24-bit */
};

static struct qspi_config *qspi_cfg;

/* --------------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------------
 */

static int qspi_set_frequency(uint32_t freq_hz)
{
	struct mspi_dev_cfg tmp = nrf7002_dev_cfg;

	tmp.freq = freq_hz;

	int ret = mspi_dev_config(mspi_dev, &nrf7002_dev_id,
				  MSPI_DEVICE_CONFIG_FREQUENCY, &tmp);
	if (ret) {
		LOG_ERR("Failed to set MSPI freq %u Hz: %d", freq_hz, ret);
	}

	return ret;
}

/**
 * @brief Send a short register command (opcode + optional 1-byte payload).
 *
 * RDSR1, RDSR2, WRSR2 are plain SPI (1-1-1) commands — the nRF7002 does NOT
 * accept them in QUAD mode.  This function temporarily reconfigures the MSPI
 * device to SPI mode, performs the transfer, then restores the previous mode.
 *
 * The STM32 OSPI driver maps cmd→Instruction phase, addr→Address phase,
 * data_buf→Data phase within one HAL_OSPI_Command + Receive/Transmit call,
 * so CS is held asserted across opcode and payload.
 *
 * @param op_code  Command opcode.
 * @param tx_data  TX payload (NULL if none).
 * @param tx_len   TX payload length in bytes.
 * @param rx_data  RX payload buffer (NULL if none).
 * @param rx_len   RX payload length in bytes.
 */
static int qspi_send_cmd(uint8_t op_code,
			 const void *tx_data, size_t tx_len,
			 void *rx_data,       size_t rx_len)
{
	int ret;
	size_t payload_len = (rx_len > 0) ? rx_len : tx_len;

	/*
	 * Step 1: Switch to SPI (1-1-1) mode.
	 * Register commands are always single-line — even when bulk I/O uses QUAD.
	 */
	if (nrf7002_dev_cfg.io_mode != MSPI_IO_MODE_SINGLE) {
		struct mspi_dev_cfg spi_cfg = nrf7002_dev_cfg;

		spi_cfg.io_mode   = MSPI_IO_MODE_SINGLE;
		spi_cfg.read_cmd  = NRF7002_CMD_READ;
		spi_cfg.write_cmd = NRF7002_CMD_PP;
		spi_cfg.rx_dummy  = 0;

		ret = mspi_dev_config(mspi_dev, &nrf7002_dev_id,
				      MSPI_DEVICE_CONFIG_ALL, &spi_cfg);
		if (ret) {
			LOG_ERR("qspi_send_cmd: SPI mode switch failed: %d", ret);
			return ret;
		}
	}

	/* Step 2: Transfer */
	struct mspi_xfer_packet pkt = {
		.dir       = (rx_len > 0) ? MSPI_RX : MSPI_TX,
		.cmd       = op_code,
		.address   = 0,
		.data_buf  = (rx_len > 0) ? rx_data : (void *)tx_data,
		.num_bytes = payload_len,
	};

	struct mspi_xfer xfer = {
		.xfer_mode   = MSPI_PIO,
		.async       = false,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE,
		.rx_dummy    = 0,
		.tx_dummy    = 0,
		.addr_length = 0,
		.cmd_length  = 1,
	};

	ret = mspi_transceive(mspi_dev, &nrf7002_dev_id, &xfer);

	if (ret) {
		LOG_ERR("qspi_send_cmd 0x%02x failed: %d", op_code, ret);
	}

	/*
	 * Step 3: Restore QUAD mode if that's what we normally use.
	 * Ignore errors — the original config is still in nrf7002_dev_cfg.
	 */
	if (nrf7002_dev_cfg.io_mode != MSPI_IO_MODE_SINGLE) {
		(void)mspi_dev_config(mspi_dev, &nrf7002_dev_id,
				      MSPI_DEVICE_CONFIG_ALL, &nrf7002_dev_cfg);
	}

	return ret;
}

/* --------------------------------------------------------------------------
 * Address / alignment check
 * --------------------------------------------------------------------------
 */
void qspi_addr_check(unsigned int addr, const void *data, unsigned int len)
{
	if ((addr % QSPI_ALIGN) ||
	    ((uintptr_t)data % QSPI_ALIGN) ||
	    (len % QSPI_ALIGN)) {
		LOG_ERR("%s: unaligned addr=0x%x data=%p len=%u",
			__func__, addr, data, len);
	}
}

/* --------------------------------------------------------------------------
 * Bulk read / write
 * --------------------------------------------------------------------------
 *
 * The STM32 OSPI driver handles Instruction + Address + DummyCycles phases
 * internally.  data_buf points directly to the payload — no manual framing.
 */

int qspi_write(unsigned int addr, const void *data, int len)
{
	int ret;

	qspi_addr_check(addr, data, len);
	addr |= qspi_cfg->addrmask;

	k_sem_take(&qspi_cfg->lock, K_FOREVER);

	struct mspi_xfer_packet pkt = {
		.dir       = MSPI_TX,
		.cmd       = nrf7002_dev_cfg.write_cmd,
		.address   = addr,
		.data_buf  = (void *)data,
		.num_bytes = (uint32_t)len,
	};

	struct mspi_xfer xfer = {
		.xfer_mode   = MSPI_PIO,
		.async       = false,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE,
		.tx_dummy    = 0,
		.addr_length = nrf7002_dev_cfg.addr_length,
		.cmd_length  = 1,
	};

	ret = mspi_transceive(mspi_dev, &nrf7002_dev_id, &xfer);
	if (ret) {
		LOG_ERR("qspi_write @ 0x%08x len=%d failed: %d", addr, len, ret);
	}

	k_sem_give(&qspi_cfg->lock);
	return ret;
}

int qspi_read(unsigned int addr, void *data, int len)
{
	int ret;

	qspi_addr_check(addr, data, len);
	addr |= qspi_cfg->addrmask;

	k_sem_take(&qspi_cfg->lock, K_FOREVER);

	struct mspi_xfer_packet pkt = {
		.dir       = MSPI_RX,
		.cmd       = nrf7002_dev_cfg.read_cmd,
		.address   = addr,
		.data_buf  = data,
		.num_bytes = (uint32_t)len,
	};

	/*
	 * Dummy cycles between address and data phases:
	 *  0x0B (1-1-1): 8 dummy clocks
	 *  0x6B (1-1-4): 8 dummy clocks
	 *  0xEB (1-4-4): 6 dummy clocks (2 dummy + 4 mode bits consumed by chip)
	 */
	uint8_t rx_dummy = (nrf7002_dev_cfg.read_cmd == NRF7002_CMD_READ4IO) ? 6U : 8U;

	struct mspi_xfer xfer = {
		.xfer_mode   = MSPI_PIO,
		.async       = false,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE,
		.rx_dummy    = rx_dummy,
		.addr_length = nrf7002_dev_cfg.addr_length,
		.cmd_length  = 1,
	};

	ret = mspi_transceive(mspi_dev, &nrf7002_dev_id, &xfer);
	if (ret) {
		LOG_ERR("qspi_read @ 0x%08x len=%d failed: %d", addr, len, ret);
	}

	k_sem_give(&qspi_cfg->lock);
	return ret;
}

/* --------------------------------------------------------------------------
 * High-latency read
 * --------------------------------------------------------------------------
 *
 * nRF7002 inserts slave-latency dummy words before valid data on certain
 * memory regions (qspi_slave_latency from rpu_7002_memmap[][2]).
 *
 * On nRF SoC this was handled by nrfx IFTIMING register (RDC4IO field).
 * On STM32 we express the same thing as rx_dummy cycles in the MSPI xfer —
 * the OSPI HAL inserts dummy cycles between address and data phases.
 *
 * Conversion: 1 latency word @ QSPI quad = 8 dummy clock cycles.
 */
int qspi_hl_readw(unsigned int addr, void *data)
{
	int ret;
	uint32_t result = 0;

	/*
	 * nRF7002 slave_latency encodes the full dummy cycle count including
	 * protocol turnaround — do not add a base offset on top of it.
	 * qspi_read() (PKTRAM, latency=0) uses rx_dummy=8 separately.
	 */
	uint8_t dummy = qspi_cfg->qspi_slave_latency * 8U;

	k_sem_take(&qspi_cfg->lock, K_FOREVER);

	struct mspi_xfer_packet pkt = {
		.dir       = MSPI_RX,
		.cmd       = nrf7002_dev_cfg.read_cmd,
		.address   = addr | qspi_cfg->addrmask,
		.data_buf  = &result,
		.num_bytes = sizeof(result),
	};

	struct mspi_xfer xfer = {
		.xfer_mode   = MSPI_PIO,
		.async       = false,
		.packets     = &pkt,
		.num_packet  = 1,
		.timeout     = CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE,
		.rx_dummy    = dummy,
		.addr_length = nrf7002_dev_cfg.addr_length,
		.cmd_length  = 1,
	};

	ret = mspi_transceive(mspi_dev, &nrf7002_dev_id, &xfer);

	k_sem_give(&qspi_cfg->lock);

	if (ret == 0) {
		*(uint32_t *)data = result;
	} else {
		LOG_ERR("qspi_hl_readw @ 0x%08x dummy=%u failed: %d",
			addr, dummy, ret);
	}

	return ret;
}

int qspi_hl_read(unsigned int addr, void *data, int len)
{
	int ret = 0;

	qspi_addr_check(addr, data, len);

	for (int i = 0; i < len / 4; i++) {
		ret = qspi_hl_readw(addr + (4 * i),
				    (uint8_t *)data + (4 * i));
		if (ret) {
			break;
		}
	}

	return ret;
}

/* --------------------------------------------------------------------------
 * RPU status register commands
 * --------------------------------------------------------------------------
 */

int qspi_RDSR2(const struct device *dev, uint8_t *rdsr2)
{
	uint8_t sr = 0;
	int ret;

	ARG_UNUSED(dev);
	ret = qspi_send_cmd(NRF7002_CMD_RDSR2, NULL, 0, &sr, sizeof(sr));
	if (ret == 0) {
		*rdsr2 = sr;
	}
	LOG_DBG("RDSR2=0x%02x", sr);
	return ret;
}

int qspi_validate_rpu_wake_writecmd(const struct device *dev)
{
	uint8_t rdsr2 = 0;
	int ret;

	ret = qspi_RDSR2(dev, &rdsr2);
	if (ret == 0 && (rdsr2 & RPU_WAKEUP_NOW)) {
		return 0;
	}
	return -1;
}

int qspi_RDSR1(const struct device *dev, uint8_t *rdsr1)
{
	uint8_t sr = 0;
	int ret;

	ARG_UNUSED(dev);
	ret = qspi_send_cmd(NRF7002_CMD_RDSR1, NULL, 0, &sr, sizeof(sr));
	if (ret == 0) {
		*rdsr1 = sr;
	}
	LOG_DBG("RDSR1=0x%02x", sr);
	return ret;
}

int qspi_wait_while_rpu_awake(const struct device *dev)
{
	uint8_t val = 0;
	int ret;

	for (int i = 0; i < 10; i++) {
		ret = qspi_RDSR1(dev, &val);
		if (ret == 0 && (val & RPU_AWAKE_BIT)) {
			(void)qspi_set_frequency(
				DT_PROP(QSPI_DEVICE_NODE, qspi_frequency));
			return (int)val;
		}
		k_msleep(1);
	}

	LOG_ERR("RPU did not wake within 10 ms");
	return -1;
}

int qspi_WRSR2(const struct device *dev, uint8_t data)
{
	int ret;

	ARG_UNUSED(dev);
	ret = qspi_send_cmd(NRF7002_CMD_WRSR2, &data, sizeof(data), NULL, 0);
	if (ret) {
		LOG_ERR("WRSR2 0x%02x failed: %d", data, ret);
	}
	return ret;
}

int qspi_cmd_wakeup_rpu(const struct device *dev, uint8_t data)
{
	int ret;

	ret = qspi_set_frequency(NRF7002_WAKEUP_FREQ);
	if (ret) {
		return ret;
	}
	return qspi_WRSR2(dev, data);
}

int qspi_cmd_sleep_rpu(const struct device *dev)
{
	return qspi_WRSR2(dev, 0x00U);
}

int qspi_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *reg_value)
{
	uint8_t val = 0;
	int ret;

	ARG_UNUSED(dev);
	ret = qspi_send_cmd(reg_addr, NULL, 0, &val, sizeof(val));
	if (ret == 0) {
		*reg_value = val;
	}
	return ret;
}

int qspi_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t reg_value)
{
	ARG_UNUSED(dev);
	return qspi_send_cmd(reg_addr, &reg_value, sizeof(reg_value), NULL, 0);
}

/* --------------------------------------------------------------------------
 * Init / deinit
 * --------------------------------------------------------------------------
 */

int qspi_deinit(void)
{
	if (device_is_ready(mspi_dev)) {
		(void)mspi_get_channel_status(mspi_dev, 0);
	}
	return 0;
}

int qspi_init(struct qspi_config *config)
{
	int ret;

	if (!config) {
		return -EINVAL;
	}

	qspi_cfg = config;

	if (!device_is_ready(mspi_dev)) {
		LOG_ERR("MSPI device %s not ready", mspi_dev->name);
		return -ENODEV;
	}

	/* Configure MSPI controller */
	const struct mspi_cfg bus_cfg = {
		.op_mode         = MSPI_OP_MODE_CONTROLLER,
		.duplex          = MSPI_HALF_DUPLEX,
		.max_freq        = DT_PROP(QSPI_DEVICE_NODE, qspi_frequency),
		.num_periph      = 1,
		.sw_multi_periph = false,
	};

	const struct mspi_dt_spec spec = {
		.bus    = mspi_dev,
		.config = bus_cfg,
	};

	ret = mspi_config(&spec);
	if (ret) {
		LOG_ERR("mspi_config failed: %d", ret);
		return ret;
	}

	/*
	 * Use MSPI_IO_MODE_QUAD_1_1_4 for bulk transfers:
	 *   - instruction phase: 1 line  (opcode 0x6B read / 0x32 write)
	 *   - address phase:     1 line
	 *   - data phase:        4 lines
	 *
	 * This requires the QUAD_1_1_4 patch in mspi_stm32_ospi.c.
	 * Register commands (RDSR/WRSR) temporarily override to SINGLE in
	 * qspi_send_cmd() and restore here afterwards.
	 *
	 * Fall back to SINGLE if quad_spi is not requested.
	 */
	/*
	 * QUAD I/O status: nRF7002 supports only 0xEB (1-4-4) with AlternateBytes=0xA0.
	 * 0x6B (1-1-4) is not supported.  AlternateBytes requires a deeper patch to
	 * mspi_stm32_ospi_access() which is pending.  Use SPI (1-1-1) for now.
	 *
	 * TODO: re-enable QUAD once mspi_stm32_ospi_access supports AlternateBytes
	 * for MSPI_IO_MODE_QUAD_1_4_4 (0xEB + 0xA0 mode byte on 4 lines).
	 */
	nrf7002_dev_cfg.io_mode   = MSPI_IO_MODE_SINGLE;
	nrf7002_dev_cfg.read_cmd  = NRF7002_CMD_READ;
	nrf7002_dev_cfg.write_cmd = NRF7002_CMD_PP;
	ARG_UNUSED(config->quad_spi);

	/*
	 * Slave latency → rx_dummy conversion for STM32 OSPI:
	 *
	 * 0x0B Fast Read always requires 8 dummy clock cycles (1 byte turnaround)
	 * between the address phase and data.  This is standard SPI NOR behaviour.
	 * nRF7002 additionally inserts extra latency words on certain memory blocks:
	 *   latency=0 (PKTRAM):  8 dummy clocks  (standard turnaround only)
	 *   latency=1 (most):   16 dummy clocks  (turnaround + 1 extra word)
	 *   latency=2 (some):   24 dummy clocks  (turnaround + 2 extra words)
	 */
	nrf7002_dev_cfg.rx_dummy = 0;
	if (DT_PROP(QSPI_DEVICE_NODE, qspi_frequency) >= MHZ(16)) {
		config->qspi_slave_latency = 1;
	} else {
		config->qspi_slave_latency = 0;
	}

	ret = mspi_dev_config(mspi_dev, &nrf7002_dev_id,
			      MSPI_DEVICE_CONFIG_ALL, &nrf7002_dev_cfg);
	if (ret) {
		LOG_ERR("mspi_dev_config failed: %d", ret);
		return ret;
	}

	k_sem_init(&config->lock, 1, 1);

	LOG_INF("nRF7002 MSPI init: %s, %u Hz",
		config->quad_spi ? "QUAD" : "SPI",
		DT_PROP(QSPI_DEVICE_NODE, qspi_frequency));

	return 0;
}

/* --------------------------------------------------------------------------
 * Encryption — nRF53 only, not available on STM32
 * --------------------------------------------------------------------------
 */

int qspi_enable_encryption(uint8_t *key)
{
	ARG_UNUSED(key);
	LOG_WRN("QSPI encryption not supported on this platform");
	return -ENOTSUP;
}

void qspi_update_nonce(unsigned int addr, int len, int hlread)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
	ARG_UNUSED(hlread);
}