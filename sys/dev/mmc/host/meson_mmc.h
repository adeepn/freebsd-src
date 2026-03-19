/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 JetHome. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Register definitions for the Amlogic Meson GX SD/eMMC controller.
 *
 * The Meson GX MMC controller is a custom Amlogic IP (not DesignWare).
 * It features a built-in clock mux/divider and operates in two modes:
 *   - Non-chain mode: write directly to CMD_CFG/CMD_ARG/CMD_DAT registers
 *   - Descriptor chain mode: build a linked list of descriptors
 *
 * This driver uses non-chain mode with a DMA bounce buffer for simplicity.
 *
 * Linux reference: drivers/mmc/host/meson-gx-mmc.c
 */

#ifndef _MESON_MMC_H_
#define _MESON_MMC_H_

/* ---- Register offsets ---- */
#define	MESON_SD_EMMC_CLOCK	0x00
#define	MESON_SD_EMMC_DELAY1	0x04
#define	MESON_SD_EMMC_DELAY2	0x08
#define	MESON_SD_EMMC_ADJUST	0x08	/* V2 (GXBB/GXL) adjust register */
#define	MESON_SD_EMMC_V3_ADJUST	0x0C
#define	MESON_SD_EMMC_CALOUT	0x10
#define	MESON_SD_EMMC_START	0x40
#define	MESON_SD_EMMC_CFG	0x44
#define	MESON_SD_EMMC_STATUS	0x48
#define	MESON_SD_EMMC_IRQ_EN	0x4C
#define	MESON_SD_EMMC_CMD_CFG	0x50
#define	MESON_SD_EMMC_CMD_ARG	0x54
#define	MESON_SD_EMMC_CMD_DAT	0x58
#define	MESON_SD_EMMC_CMD_RSP	0x5C
#define	MESON_SD_EMMC_CMD_RSP1	0x60
#define	MESON_SD_EMMC_CMD_RSP2	0x64
#define	MESON_SD_EMMC_CMD_RSP3	0x68

/* ---- SD_EMMC_CLOCK fields ---- */
#define	CLK_DIV_SHIFT		0
#define	CLK_DIV_MASK		0x3F		/* 6-bit divider */
#define	CLK_DIV_MAX		63
#define	CLK_SRC_SHIFT		6
#define	CLK_SRC_MASK		(0x3 << 6)
#define	CLK_SRC_XTAL		(0 << 6)	/* 24 MHz crystal */
#define	CLK_SRC_FCLK_DIV2	(1 << 6)	/* 1 GHz (fclk_div2) */
#define	CLK_CORE_PHASE_SHIFT	8
#define	CLK_CORE_PHASE_MASK	(0x3 << 8)
#define	CLK_TX_PHASE_SHIFT	10
#define	CLK_TX_PHASE_MASK	(0x3 << 10)
#define	CLK_RX_PHASE_SHIFT	12
#define	CLK_RX_PHASE_MASK	(0x3 << 12)
#define	CLK_PHASE_0		0
#define	CLK_PHASE_90		1
#define	CLK_PHASE_180		2
#define	CLK_PHASE_270		3
#define	CLK_ALWAYS_ON		(1u << 24)	/* GXL: V2 always-on */

/* ---- SD_EMMC_ADJUST fields ---- */
#define	ADJUST_ADJ_EN		(1u << 13)
#define	ADJUST_DS_EN		(1u << 15)
#define	ADJUST_ADJ_DELAY_MASK	(0x3F << 16)

/* ---- SD_EMMC_START fields ---- */
#define	START_DESC_INIT		(1u << 0)
#define	START_DESC_BUSY		(1u << 1)
#define	START_DESC_ADDR_MASK	0xFFFFFFFC

/* ---- SD_EMMC_CFG fields ---- */
#define	CFG_BUS_WIDTH_SHIFT	0
#define	CFG_BUS_WIDTH_MASK	0x3
#define	CFG_BUS_WIDTH_1		0x0
#define	CFG_BUS_WIDTH_4		0x1
#define	CFG_BUS_WIDTH_8		0x2
#define	CFG_DDR			(1u << 2)
#define	CFG_BLK_LEN_SHIFT	4
#define	CFG_BLK_LEN_MASK	(0xF << 4)
#define	CFG_RESP_TIMEOUT_SHIFT	8
#define	CFG_RESP_TIMEOUT_MASK	(0xF << 8)
#define	CFG_RC_CC_SHIFT		12
#define	CFG_RC_CC_MASK		(0xF << 12)
#define	CFG_CLK_ALWAYS_ON	(1u << 18)
#define	CFG_CHK_DS		(1u << 20)
#define	CFG_STOP_CLOCK		(1u << 22)
#define	CFG_AUTO_CLK		(1u << 23)
#define	CFG_ERR_ABORT		(1u << 27)

/* ---- SD_EMMC_STATUS fields ---- */
#define	STATUS_DATI_SHIFT	16
#define	STATUS_DATI_MASK	(0xFF << 16)
#define	STATUS_DESC_BUSY	(1u << 30)
#define	STATUS_BUSY		(1u << 31)

/* ---- SD_EMMC_IRQ_EN / STATUS IRQ fields ---- */
#define	IRQ_RXD_ERR_MASK	0xFF		/* 8 bits for RX data CRC */
#define	IRQ_TXD_ERR		(1u << 8)
#define	IRQ_DESC_ERR		(1u << 9)
#define	IRQ_RESP_ERR		(1u << 10)
#define	IRQ_CRC_ERR		(IRQ_RXD_ERR_MASK | IRQ_TXD_ERR | \
				 IRQ_DESC_ERR | IRQ_RESP_ERR)
#define	IRQ_RESP_TIMEOUT	(1u << 11)
#define	IRQ_DESC_TIMEOUT	(1u << 12)
#define	IRQ_TIMEOUTS		(IRQ_RESP_TIMEOUT | IRQ_DESC_TIMEOUT)
#define	IRQ_END_OF_CHAIN	(1u << 13)
#define	IRQ_RESP_STATUS		(1u << 14)
#define	IRQ_SDIO		(1u << 15)
#define	IRQ_EN_MASK		(IRQ_CRC_ERR | IRQ_TIMEOUTS | \
				 IRQ_END_OF_CHAIN)
#define	IRQ_ERR_MASK		(IRQ_CRC_ERR | IRQ_TIMEOUTS)

/* ---- CMD_CFG fields (also used in sd_emmc_desc.cmd_cfg) ---- */
#define	CMD_CFG_LENGTH_SHIFT	0
#define	CMD_CFG_LENGTH_MASK	0x1FF		/* 9-bit block count or bytes */
#define	CMD_CFG_BLOCK_MODE	(1u << 9)
#define	CMD_CFG_R1B		(1u << 10)
#define	CMD_CFG_END_OF_CHAIN	(1u << 11)
#define	CMD_CFG_TIMEOUT_SHIFT	12
#define	CMD_CFG_TIMEOUT_MASK	(0xF << 12)
#define	CMD_CFG_NO_RESP		(1u << 16)
#define	CMD_CFG_NO_CMD		(1u << 17)
#define	CMD_CFG_DATA_IO		(1u << 18)
#define	CMD_CFG_DATA_WR		(1u << 19)
#define	CMD_CFG_RESP_NOCRC	(1u << 20)
#define	CMD_CFG_RESP_128	(1u << 21)
#define	CMD_CFG_RESP_NUM	(1u << 22)
#define	CMD_CFG_DATA_NUM	(1u << 23)
#define	CMD_CFG_CMD_INDEX_SHIFT	24
#define	CMD_CFG_CMD_INDEX_MASK	(0x3F << 24)
#define	CMD_CFG_ERROR		(1u << 30)
#define	CMD_CFG_OWNER		(1u << 31)

/* ---- CMD_DAT fields ---- */
#define	CMD_DATA_ADDR_MASK	0xFFFFFFFC	/* DMA address, 4-byte aligned */
#define	CMD_DATA_BIG_ENDIAN	(1u << 1)
#define	CMD_DATA_SRAM		(1u << 0)

/* ---- Descriptor structure (for descriptor chain mode) ---- */
struct meson_mmc_desc {
	uint32_t	cmd_cfg;
	uint32_t	cmd_arg;
	uint32_t	cmd_data;
	uint32_t	cmd_resp;
};

/* ---- Constants ---- */
#define	MESON_MMC_CLK_XTAL_RATE	24000000	/* 24 MHz crystal */
#define	MESON_MMC_CLK_DIV2_RATE	1000000000	/* 1 GHz fclk_div2 */

/* SD_EMMC_CFG defaults */
#define	MESON_MMC_CFG_BLK_SIZE	512
#define	MESON_MMC_CFG_RESP_TIMEOUT 256	/* In clock cycles */
#define	MESON_MMC_CFG_CMD_GAP	16	/* In clock cycles */

/* Timeouts */
#define	MESON_MMC_CMD_TIMEOUT	1024	/* ms */
#define	MESON_MMC_CMD_TIMEOUT_DATA 4096	/* ms */

/* Bounce buffer: max 511 blocks * 512 bytes = ~256KB */
#define	MESON_MMC_BOUNCE_SIZE	(512 * 512)
#define	MESON_MMC_DMA_ALIGN	8
#define	MESON_MMC_MAX_BLKCNT	511	/* CMD_CFG length field is 9-bit */

#endif /* _MESON_MMC_H_ */
