/*
 * Copyright (c) 2026 Krzysztof Gawrys
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SDIO command layer — all functions take the SDHC host device.
 */

#ifndef ZEPHYR_DRIVERS_WIFI_ESP_HOSTED_SDIO_CMD_H_
#define ZEPHYR_DRIVERS_WIFI_ESP_HOSTED_SDIO_CMD_H_

#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>
#include "esp_hosted_sdio.h"

/* ── CMD52 — single byte R/W ─────────────────────────────────────────── */
int sdio_cmd52_read(const struct device *sdhc, int func, uint32_t reg, uint8_t *out);
int sdio_cmd52_write(const struct device *sdhc, int func, uint32_t reg,
		     uint8_t val, uint8_t *out);

/* ── CMD53 — extended byte/block R/W ────────────────────────────────── */
int sdio_cmd53_rw(const struct device *sdhc, uint32_t func, uint32_t reg,
		  uint32_t arg, void *buf, size_t len,
		  uint8_t *io_scratch);
int sdio_cmd53_read_bytes(const struct device *sdhc, uint32_t func,
			  uint32_t addr, void *dst, size_t len,
			  uint8_t *io_scratch);
int sdio_cmd53_write_bytes(const struct device *sdhc, uint32_t func,
			   uint32_t addr, void *src, size_t len,
			   uint8_t *io_scratch);
int sdio_cmd53_read_blocks(const struct device *sdhc, uint32_t func,
			   uint32_t addr, void *dst, size_t len,
			   uint8_t *io_scratch);
int sdio_cmd53_write_blocks(const struct device *sdhc, uint32_t func,
			    uint32_t addr, const void *src, size_t len,
			    uint8_t *io_scratch);

/* ── Card init commands ──────────────────────────────────────────────── */
int sdio_cmd0_go_idle(const struct device *sdhc);
int sdio_cmd5_send_op_cond(const struct device *sdhc, uint32_t ocr,
			   uint32_t *ocr_out);
int sdio_cmd3_send_relative_addr(const struct device *sdhc, uint16_t *rca_out);
int sdio_cmd7_select_card(const struct device *sdhc, uint32_t rca);

/* ── CCCR helpers ────────────────────────────────────────────────────── */
int sdio_cccr_reset(const struct device *sdhc);
int sdio_cccr_io_init(const struct device *sdhc);   /* CMD5 x2 */

/* ── Register helpers (FN1 address space) ───────────────────────────── */
int sdio_reg_read(const struct device *sdhc, uint32_t reg,
		  void *data, uint16_t size, uint8_t *io_scratch);
int sdio_reg_write(const struct device *sdhc, uint32_t reg,
		   void *data, uint16_t size, uint8_t *io_scratch);

/* ── Block read/write helpers ───────────────────────────────────────── */
int esp_sdio_read_fromio(const struct device *sdhc, uint32_t func,
			 uint32_t addr, void *dst, uint16_t size,
			 uint8_t *io_scratch);
int esp_sdio_write_toio(const struct device *sdhc, uint32_t func,
			uint32_t addr, void *src, uint16_t size,
			uint8_t *io_scratch);

#endif /* ZEPHYR_DRIVERS_WIFI_ESP_HOSTED_SDIO_CMD_H_ */
