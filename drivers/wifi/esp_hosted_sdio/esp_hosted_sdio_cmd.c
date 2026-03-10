/*
 * Copyright (c) 2026 Krzysztof Gawrys
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SDIO command layer — CMD52 / CMD53 primitives and card-init commands.
 * Ported from project src/sdio_io.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

#include "esp_hosted_sdio.h"
#include "esp_hosted_sdio_cmd.h"

LOG_MODULE_REGISTER(esp_hosted_cmd, LOG_LEVEL_INF);

/* ── CMD52 ──────────────────────────────────────────────────────────────── */

int sdio_cmd52_read(const struct device *sdhc, int func, uint32_t reg,
		    uint8_t *out)
{
	uint8_t val = 0;
	struct sdhc_command cmd = {
		.opcode = SD_IO_RW_DIRECT,
		.timeout_ms = 1000,
	};

	cmd.arg  = SD_ARG_CMD52_READ;
	cmd.arg |= (func & SD_ARG_CMD52_FUNC_MASK) << SD_ARG_CMD52_FUNC_SHIFT;
	cmd.arg |= (reg  & SD_ARG_CMD52_REG_MASK)  << SD_ARG_CMD52_REG_SHIFT;
	cmd.arg |= (val  & SD_ARG_CMD52_DATA_MASK)  << SD_ARG_CMD52_DATA_SHIFT;
	cmd.response_type = SD_RSP_TYPE_R5;

	int ret = sdhc_request(sdhc, &cmd, NULL);
	if (ret < 0) {
		return ret;
	}

	if (out) {
		*out = SD_R5_DATA(cmd.response);
	}
	return 0;
}

int sdio_cmd52_write(const struct device *sdhc, int func, uint32_t reg,
		     uint8_t val, uint8_t *out)
{
	struct sdhc_command cmd = {
		.opcode = SD_IO_RW_DIRECT,
		.timeout_ms = 1000,
	};

	cmd.arg  = SD_ARG_CMD52_WRITE | SD_ARG_CMD52_EXCHANGE;
	cmd.arg |= (func & SD_ARG_CMD52_FUNC_MASK) << SD_ARG_CMD52_FUNC_SHIFT;
	cmd.arg |= (reg  & SD_ARG_CMD52_REG_MASK)  << SD_ARG_CMD52_REG_SHIFT;
	cmd.arg |= (val  & SD_ARG_CMD52_DATA_MASK)  << SD_ARG_CMD52_DATA_SHIFT;
	cmd.response_type = SD_RSP_TYPE_R5;

	int ret = sdhc_request(sdhc, &cmd, NULL);
	if (ret < 0) {
		return ret;
	}

	if (out) {
		*out = SD_R5_DATA(cmd.response);
	}
	return 0;
}

/* ── CMD53 core ─────────────────────────────────────────────────────────── */

/*
 * io_scratch must be a 32-byte aligned buffer of at least ESP_BLOCK_SIZE bytes.
 * It is used as the DMA bounce buffer to avoid cache coherency issues.
 */
/*
 * io_scratch — zachowane w API dla kompatybilności, nieużywane.
 * STM32 SDMMC HAL zarządza własnym buforem DMA wewnętrznie i kopiuje
 * wynik do data->data, więc buf może być dowolny bufor w zwykłej pamięci
 * podręcznej (cached RAM). Nie ma wymagań na wyrównanie ani .nocache.
 */
int sdio_cmd53_rw(const struct device *sdhc, uint32_t func, uint32_t reg,
		  uint32_t arg, void *buf, size_t len,
		  uint8_t *io_scratch)
{
	ARG_UNUSED(io_scratch);

	uint32_t num_blocks = (len + ESP_BLOCK_SIZE - 1) / ESP_BLOCK_SIZE;
	uint32_t count;

	struct sdhc_command cmd = {
		.opcode = SD_IO_RW_EXTENDED,
		.response_type = SD_RSP_TYPE_R5,
		.timeout_ms = 1000,
	};

	struct sdhc_data data_cfg = {
		.block_size = ESP_BLOCK_SIZE,
		.blocks = num_blocks,
		.data = buf,
		.timeout_ms = 1000,
	};

	if (arg & SD_ARG_CMD53_BLOCK_MODE) {
		count = num_blocks;
	} else {
		data_cfg.block_size = len;
		count = (len == ESP_BLOCK_SIZE) ? 0 : len;
	}

	arg |= (func & SD_ARG_CMD53_FUNC_MASK) << SD_ARG_CMD53_FUNC_SHIFT;
	arg |= (reg  & SD_ARG_CMD53_REG_MASK)  << SD_ARG_CMD53_REG_SHIFT;
	arg |= (count & SD_ARG_CMD53_LENGTH_MASK) << SD_ARG_CMD53_LENGTH_SHIFT;
	cmd.arg = arg;

	int err = sdhc_request(sdhc, &cmd, &data_cfg);

	if (err) {
		LOG_ERR("cmd53 failed: %d (arg=0x%08x)", err, arg);
	}

	return err;
}

/* ── CMD53 byte-mode helpers ────────────────────────────────────────────── */

int sdio_cmd53_read_bytes(const struct device *sdhc, uint32_t func,
			  uint32_t addr, void *dst, size_t len,
			  uint8_t *io_scratch)
{
	uint32_t arg = SD_ARG_CMD53_READ;
	bool incr = true;

	if (addr & SDMMC_IO_FIXED_ADDR) {
		addr &= ~SDMMC_IO_FIXED_ADDR;
		incr = false;
	}
	if (incr) {
		arg |= SD_ARG_CMD53_INCREMENT;
	}

	uint8_t *p = dst;

	while (len > 0) {
		size_t aligned = len & ~3u;
		size_t chunk = aligned ? aligned : len;

		int err = sdio_cmd53_rw(sdhc, func, addr, arg, p, chunk,
					io_scratch);
		if (err) {
			return err;
		}
		p    += chunk;
		len  -= chunk;
		if (incr) {
			addr += chunk;
		}
	}
	return 0;
}

int sdio_cmd53_write_bytes(const struct device *sdhc, uint32_t func,
			   uint32_t addr, void *src, size_t len,
			   uint8_t *io_scratch)
{
	uint32_t arg = SD_ARG_CMD53_WRITE;
	bool incr = true;

	if (addr & SDMMC_IO_FIXED_ADDR) {
		addr &= ~SDMMC_IO_FIXED_ADDR;
		incr = false;
	}
	if (incr) {
		arg |= SD_ARG_CMD53_INCREMENT;
	}

	uint8_t *p = src;

	while (len > 0) {
		size_t aligned = len & ~3u;
		size_t chunk = aligned ? aligned : len;

		int err = sdio_cmd53_rw(sdhc, func, addr, arg, p, chunk,
					io_scratch);
		if (err) {
			return err;
		}
		p    += chunk;
		len  -= chunk;
		if (incr) {
			addr += chunk;
		}
	}
	return 0;
}

/* ── CMD53 block-mode helpers ───────────────────────────────────────────── */

int sdio_cmd53_read_blocks(const struct device *sdhc, uint32_t func,
			   uint32_t addr, void *dst, size_t len,
			   uint8_t *io_scratch)
{
	uint32_t arg = SD_ARG_CMD53_READ | SD_ARG_CMD53_INCREMENT |
		       SD_ARG_CMD53_BLOCK_MODE;

	if (addr & SDMMC_IO_FIXED_ADDR) {
		arg &= ~SD_ARG_CMD53_INCREMENT;
		addr &= ~SDMMC_IO_FIXED_ADDR;
	}

	return sdio_cmd53_rw(sdhc, func, addr, arg, dst, len, io_scratch);
}

int sdio_cmd53_write_blocks(const struct device *sdhc, uint32_t func,
			    uint32_t addr, const void *src, size_t len,
			    uint8_t *io_scratch)
{
	uint32_t arg = SD_ARG_CMD53_WRITE | SD_ARG_CMD53_INCREMENT |
		       SD_ARG_CMD53_BLOCK_MODE;

	if (addr & SDMMC_IO_FIXED_ADDR) {
		arg &= ~SD_ARG_CMD53_INCREMENT;
		addr &= ~SDMMC_IO_FIXED_ADDR;
	}

	return sdio_cmd53_rw(sdhc, func, addr, arg, (void *)src, len,
			     io_scratch);
}

/* ── Card init commands ─────────────────────────────────────────────────── */

int sdio_cmd0_go_idle(const struct device *sdhc)
{
	struct sdhc_command cmd = {
		.opcode = MMC_GO_IDLE_STATE,
		.arg = 0,
		.response_type = SD_RSP_TYPE_NONE,
		.timeout_ms = 1000,
		.retries = 3,
	};

	int ret = sdhc_request(sdhc, &cmd, NULL);
	if (ret < 0) {
		LOG_ERR("CMD0 failed: %d", ret);
		return ret;
	}
	k_msleep(20);
	return 0;
}

int sdio_cmd5_send_op_cond(const struct device *sdhc, uint32_t ocr,
			   uint32_t *ocr_out)
{
	struct sdhc_command cmd = {
		.opcode = SD_IO_SEND_OP_COND,
		.arg = ocr,
		.response_type = SD_RSP_TYPE_R4,
		.timeout_ms = 1000,
		.retries = 3,
	};

	for (int i = 0; i < 100; i++) {
		int ret = sdhc_request(sdhc, &cmd, NULL);
		if (ret < 0) {
			return ret;
		}
		if ((MMC_R4(cmd.response) & SD_IO_OCR_MEM_READY) || ocr == 0) {
			break;
		}
		k_msleep(10);
	}

	if (ocr_out) {
		*ocr_out = MMC_R4(cmd.response);
	}
	return 0;
}

int sdio_cmd3_send_relative_addr(const struct device *sdhc, uint16_t *rca_out)
{
	struct sdhc_command cmd = {
		.opcode = SD_SEND_RELATIVE_ADDR,
		.response_type = SD_RSP_TYPE_R6,
		.timeout_ms = 1000,
		.retries = 3,
	};

	int err = sdhc_request(sdhc, &cmd, NULL);
	if (err) {
		return err;
	}

	uint16_t rca = SD_R6_RCA(cmd.response);
	if (rca == 0) {
		err = sdhc_request(sdhc, &cmd, NULL);
		if (err) {
			return err;
		}
		rca = SD_R6_RCA(cmd.response);
	}

	*rca_out = rca;
	return 0;
}

int sdio_cmd7_select_card(const struct device *sdhc, uint32_t rca)
{
	struct sdhc_command cmd = {
		.opcode = MMC_SELECT_CARD,
		.arg = MMC_ARG_RCA(rca),
		.response_type = SD_RSP_TYPE_R1,
		.timeout_ms = 1000,
		.retries = 3,
	};

	int ret = sdhc_request(sdhc, &cmd, NULL);
	if (ret) {
		LOG_ERR("CMD7 failed: %d", ret);
	}
	return ret;
}

/* ── CCCR helpers ───────────────────────────────────────────────────────── */

int sdio_cccr_reset(const struct device *sdhc)
{
	uint8_t val = CCCR_CTL_RES;

	int ret = sdio_cmd52_write(sdhc, 0, SD_IO_CCCR_CTL,
				   CCCR_CTL_RES, &val);
	if (ret == -ETIMEDOUT) {
		/* Normal for ESP32C5 */
		return 0;
	}
	return ret;
}

int sdio_cccr_io_init(const struct device *sdhc)
{
	uint32_t ocr = 0;
	int err;

	err = sdio_cmd5_send_op_cond(sdhc, 0, &ocr);
	if (err < 0) {
		LOG_WRN("CMD5 probe: %d (not IO card?)", err);
		return 0; /* not necessarily fatal — proceed */
	}

	err = sdio_cmd5_send_op_cond(sdhc, SD_OCR_VOL_MASK, &ocr);
	if (err < 0) {
		LOG_ERR("CMD5 set voltage: %d", err);
		return err;
	}
	return 0;
}

/* ── Register helpers ───────────────────────────────────────────────────── */

int sdio_reg_read(const struct device *sdhc, uint32_t reg,
		  void *data, uint16_t size, uint8_t *io_scratch)
{
	reg &= ESP_ADDRESS_MASK;

	if (size <= 1) {
		return sdio_cmd52_read(sdhc, SDIO_FUNC_1, reg, data);
	}
	return sdio_cmd53_read_bytes(sdhc, SDIO_FUNC_1, reg, data, size,
				     io_scratch);
}

int sdio_reg_write(const struct device *sdhc, uint32_t reg,
		   void *data, uint16_t size, uint8_t *io_scratch)
{
	reg &= ESP_ADDRESS_MASK;

	if (size <= 1) {
		return sdio_cmd52_write(sdhc, SDIO_FUNC_1, reg,
					*(uint8_t *)data, NULL);
	}
	return sdio_cmd53_write_bytes(sdhc, SDIO_FUNC_1, reg, data, size,
				      io_scratch);
}

/* ── Block R/W helpers (mixed block+byte mode) ──────────────────────────── */

int esp_sdio_read_fromio(const struct device *sdhc, uint32_t func,
		     uint32_t addr, void *dst, uint16_t size,
		     uint8_t *io_scratch)
{
	uint8_t *p = dst;
	uint16_t rem = size;
	int res;

	while (rem >= ESP_BLOCK_SIZE) {
		uint16_t blocks = H_SDIO_RX_BLOCKS_TO_TRANSFER(rem);
		uint16_t chunk  = blocks * ESP_BLOCK_SIZE;

		res = sdio_cmd53_read_blocks(sdhc, func, addr, p, chunk,
					     io_scratch);
		if (res) {
			return res;
		}
		rem  -= chunk;
		p    += chunk;
		addr += chunk;
	}

	if (rem > 0) {
		res = sdio_cmd53_read_bytes(sdhc, func, addr, p, rem,
					    io_scratch);
		if (res) {
			return res;
		}
	}
	return 0;
}

int esp_sdio_write_toio(const struct device *sdhc, uint32_t func,
		    uint32_t addr, void *src, uint16_t size,
		    uint8_t *io_scratch)
{
	uint8_t *p = src;
	uint16_t rem = size;
	int res;

	while (rem >= ESP_BLOCK_SIZE) {
		uint16_t blocks = H_SDIO_TX_BLOCKS_TO_TRANSFER(rem);
		uint16_t chunk  = blocks * ESP_BLOCK_SIZE;

		res = sdio_cmd53_write_blocks(sdhc, func, addr, p, chunk,
					      io_scratch);
		if (res) {
			return res;
		}
		rem  -= chunk;
		p    += chunk;
		addr += chunk;
	}

	if (rem > 0) {
		res = sdio_cmd53_write_bytes(sdhc, func, addr, p, rem,
					     io_scratch);
		if (res) {
			return res;
		}
	}
	return 0;
}
