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
 * Amlogic Meson GX SD/eMMC host controller driver.
 *
 * This driver uses non-chain (bounce buffer) mode for simplicity.
 * The controller has a built-in clock mux/divider that is programmed
 * directly rather than through the FreeBSD clock framework.
 *
 * TODO: STUB -- partial implementation for MVP boot
 *
 * Current behaviour:
 *   - SD card access in SDR mode (25/50 MHz, 1/4-bit bus width)
 *   - Non-chain DMA via a coherent bounce buffer
 *   - Clock divider programmed directly in SD_EMMC_CLOCK register
 *   - Interrupt-driven command/data completion
 *
 * For a full implementation:
 *   - Descriptor chain mode for better performance (scatter-gather)
 *   - DDR mode (DDR50/DDR52) support
 *   - HS200/HS400 tuning
 *   - Proper clock framework integration (register mux + divider
 *     as clknode children of the parent clock tree)
 *   - SDIO interrupt support
 *   - Power management (runtime PM)
 *
 * Linux reference: drivers/mmc/host/meson-gx-mmc.c
 * FreeBSD template: sys/arm/allwinner/aw_mmc.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmc_fdt_helpers.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <dev/mmc/host/meson_mmc.h>

#include "mmcbr_if.h"

#define	MESON_MMC_MEMRES	0
#define	MESON_MMC_IRQRES	1

#define	MMC_READ_4(sc, reg)	bus_read_4((sc)->mem_res, (reg))
#define	MMC_WRITE_4(sc, reg, val) bus_write_4((sc)->mem_res, (reg), (val))

#define	MMC_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	MMC_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)

struct meson_mmc_softc {
	device_t		dev;
	device_t		child;		/* mmc bus child */
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*intrhand;
	struct mtx		mtx;
	struct callout		timeout_co;
	int			timeout_secs;

	struct mmc_host		host;
	struct mmc_helper	mmc_helper;
	struct mmc_request	*req;

	clk_t			clk_core;
	clk_t			clk_clkin0;
	clk_t			clk_clkin1;
	hwreset_t		rst;

	/* DMA bounce buffer */
	bus_dma_tag_t		dma_tag;
	bus_dmamap_t		dma_map;
	void			*dma_buf;
	bus_addr_t		dma_buf_phys;
	int			dma_map_err;
};

static struct resource_spec meson_mmc_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

static struct ofw_compat_data compat_data[] = {
	{ "amlogic,meson-gx-mmc",	1 },
	{ "amlogic,meson-gxbb-mmc",	1 },
	{ "amlogic,meson-axg-mmc",	1 },
	{ NULL,				0 }
};

/* Forward declarations */
static void meson_mmc_intr(void *arg);
static void meson_mmc_timeout(void *arg);
static void meson_mmc_req_done(struct meson_mmc_softc *sc);
static void meson_mmc_helper_cd_handler(device_t dev, bool present);

/* ---- Utility: compute log2 for power-of-2 values ---- */
static inline uint32_t
meson_mmc_ilog2(uint32_t val)
{
	uint32_t log = 0;

	while (val > 1) {
		val >>= 1;
		log++;
	}
	return (log);
}

/* ---- DMA bounce buffer setup ---- */

static void
meson_dma_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	struct meson_mmc_softc *sc = arg;

	if (err) {
		sc->dma_map_err = err;
		return;
	}
	sc->dma_buf_phys = segs[0].ds_addr;
}

static int
meson_mmc_setup_dma(struct meson_mmc_softc *sc)
{
	int error;

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),		/* parent */
	    MESON_MMC_DMA_ALIGN, 0,		/* align, boundary */
	    BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg */
	    MESON_MMC_BOUNCE_SIZE, 1,		/* maxsize, nsegments */
	    MESON_MMC_BOUNCE_SIZE,		/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lock, lockarg */
	    &sc->dma_tag);
	if (error)
		return (error);

	error = bus_dmamem_alloc(sc->dma_tag, &sc->dma_buf,
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &sc->dma_map);
	if (error)
		return (error);

	error = bus_dmamap_load(sc->dma_tag, sc->dma_map,
	    sc->dma_buf, MESON_MMC_BOUNCE_SIZE,
	    meson_dma_cb, sc, 0);
	if (error)
		return (error);
	if (sc->dma_map_err)
		return (sc->dma_map_err);

	return (0);
}

static void
meson_mmc_teardown_dma(struct meson_mmc_softc *sc)
{

	if (sc->dma_tag == NULL)
		return;
	bus_dmamap_unload(sc->dma_tag, sc->dma_map);
	bus_dmamem_free(sc->dma_tag, sc->dma_buf, sc->dma_map);
	bus_dma_tag_destroy(sc->dma_tag);
}

/* ---- Hardware clock programming ---- */

/*
 * TODO: STUB -- programs the controller's internal divider directly
 *
 * Current behaviour:
 *   Selects crystal (24 MHz) or fclk_div2 (1 GHz) as clock source
 *   and programs the 6-bit divider to achieve the requested rate.
 *
 * For a full implementation:
 *   - Register the internal mux + divider as clknode children in the
 *     FreeBSD clock framework, with clkin0/clkin1 as parents.
 *   - Use clk_set_freq() for rate changes.
 *   - Proper phase tuning for HS200/HS400.
 *
 * Linux reference: drivers/mmc/host/meson-gx-mmc.c meson_mmc_clk_init()
 */
static int
meson_mmc_set_clock(struct meson_mmc_softc *sc, uint32_t freq)
{
	uint32_t clk_reg, src_rate, div;

	if (freq == 0) {
		/* Stop the clock */
		clk_reg = MMC_READ_4(sc, MESON_SD_EMMC_CFG);
		clk_reg |= CFG_STOP_CLOCK;
		MMC_WRITE_4(sc, MESON_SD_EMMC_CFG, clk_reg);
		return (0);
	}

	/*
	 * Select clock source:
	 *   freq <= 12 MHz: use crystal (24 MHz), max div = 2
	 *   freq > 12 MHz:  use fclk_div2 (1 GHz)
	 */
	if (freq <= MESON_MMC_CLK_XTAL_RATE / 2) {
		src_rate = MESON_MMC_CLK_XTAL_RATE;
		clk_reg = CLK_SRC_XTAL;
	} else {
		src_rate = MESON_MMC_CLK_DIV2_RATE;
		clk_reg = CLK_SRC_FCLK_DIV2;
	}

	/* Calculate divider: output = source / div */
	div = src_rate / freq;
	if (div < 1)
		div = 1;
	if (div > CLK_DIV_MAX)
		div = CLK_DIV_MAX;

	/* Stop clock before changing */
	MMC_WRITE_4(sc, MESON_SD_EMMC_CFG,
	    MMC_READ_4(sc, MESON_SD_EMMC_CFG) | CFG_STOP_CLOCK);

	/* Program clock register: source, divider, phases, always-on */
	clk_reg |= (div & CLK_DIV_MASK);
	clk_reg |= (CLK_PHASE_180 << CLK_CORE_PHASE_SHIFT);
	clk_reg |= (CLK_PHASE_0 << CLK_TX_PHASE_SHIFT);
	clk_reg |= (CLK_PHASE_0 << CLK_RX_PHASE_SHIFT);
	clk_reg |= CLK_ALWAYS_ON;
	MMC_WRITE_4(sc, MESON_SD_EMMC_CLOCK, clk_reg);

	/* Ungate the clock */
	MMC_WRITE_4(sc, MESON_SD_EMMC_CFG,
	    MMC_READ_4(sc, MESON_SD_EMMC_CFG) & ~CFG_STOP_CLOCK);

	return (0);
}

/* ---- Hardware init ---- */

static void
meson_mmc_cfg_init(struct meson_mmc_softc *sc)
{
	uint32_t cfg;

	cfg = 0;
	cfg |= (meson_mmc_ilog2(MESON_MMC_CFG_RESP_TIMEOUT) <<
	    CFG_RESP_TIMEOUT_SHIFT) & CFG_RESP_TIMEOUT_MASK;
	cfg |= (meson_mmc_ilog2(MESON_MMC_CFG_CMD_GAP) <<
	    CFG_RC_CC_SHIFT) & CFG_RC_CC_MASK;
	cfg |= (meson_mmc_ilog2(MESON_MMC_CFG_BLK_SIZE) <<
	    CFG_BLK_LEN_SHIFT) & CFG_BLK_LEN_MASK;
	cfg |= CFG_ERR_ABORT;

	MMC_WRITE_4(sc, MESON_SD_EMMC_CFG, cfg);
}

static void
meson_mmc_hw_init(struct meson_mmc_softc *sc)
{

	/* Set up CFG register defaults */
	meson_mmc_cfg_init(sc);

	/* Set initial clock to 400 KHz for card identification */
	meson_mmc_set_clock(sc, 400000);

	/* Clear any pending status/IRQs */
	MMC_WRITE_4(sc, MESON_SD_EMMC_STATUS,
	    MMC_READ_4(sc, MESON_SD_EMMC_STATUS));

	/* Enable interrupts: errors + end-of-chain */
	MMC_WRITE_4(sc, MESON_SD_EMMC_IRQ_EN, IRQ_EN_MASK);
}

/* ---- Request handling ---- */

static void
meson_mmc_read_response(struct meson_mmc_softc *sc, struct mmc_command *cmd)
{

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = MMC_READ_4(sc, MESON_SD_EMMC_CMD_RSP3);
		cmd->resp[1] = MMC_READ_4(sc, MESON_SD_EMMC_CMD_RSP2);
		cmd->resp[2] = MMC_READ_4(sc, MESON_SD_EMMC_CMD_RSP1);
		cmd->resp[3] = MMC_READ_4(sc, MESON_SD_EMMC_CMD_RSP);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = MMC_READ_4(sc, MESON_SD_EMMC_CMD_RSP);
	}
}

static void
meson_mmc_req_done(struct meson_mmc_softc *sc)
{
	struct mmc_request *req;

	callout_stop(&sc->timeout_co);
	req = sc->req;
	sc->req = NULL;
	MMC_UNLOCK(sc);
	if (req != NULL)
		req->done(req);
}

static void
meson_mmc_timeout(void *arg)
{
	struct meson_mmc_softc *sc = arg;

	if (sc->req == NULL)
		return;

	device_printf(sc->dev, "timeout, CMD%u status=0x%08x\n",
	    sc->req->cmd->opcode,
	    MMC_READ_4(sc, MESON_SD_EMMC_STATUS));

	sc->req->cmd->error = MMC_ERR_TIMEOUT;

	/* Stop the controller */
	MMC_WRITE_4(sc, MESON_SD_EMMC_START, 0);

	meson_mmc_req_done(sc);
}

static void
meson_mmc_intr(void *arg)
{
	struct meson_mmc_softc *sc = arg;
	struct mmc_command *cmd;
	struct mmc_data *data;
	uint32_t status, irq_mask;

	MMC_LOCK(sc);

	irq_mask = IRQ_EN_MASK;
	status = MMC_READ_4(sc, MESON_SD_EMMC_STATUS) & irq_mask;
	if (status == 0) {
		MMC_UNLOCK(sc);
		return;
	}

	/* Acknowledge interrupts by writing status back */
	MMC_WRITE_4(sc, MESON_SD_EMMC_STATUS, status);

	if (sc->req == NULL) {
		device_printf(sc->dev,
		    "spurious interrupt, status=0x%08x\n", status);
		MMC_UNLOCK(sc);
		return;
	}

	cmd = sc->req->cmd;

	/* Check for errors first */
	if (status & IRQ_ERR_MASK) {
		if (status & IRQ_TIMEOUTS)
			cmd->error = MMC_ERR_TIMEOUT;
		else
			cmd->error = MMC_ERR_FAILED;

		/* Stop the controller on error */
		MMC_WRITE_4(sc, MESON_SD_EMMC_START, 0);
		meson_mmc_req_done(sc);
		return;
	}

	/* Command/data completed successfully */
	if (status & (IRQ_END_OF_CHAIN | IRQ_RESP_STATUS)) {
		meson_mmc_read_response(sc, cmd);

		data = cmd->data;
		if (data != NULL) {
			if (!(data->flags & MMC_DATA_WRITE)) {
				/* Read: copy from bounce buffer to data buf */
				memcpy(data->data, sc->dma_buf,
				    data->len);
			}
			data->xfer_len = data->len;
		}
		cmd->error = MMC_ERR_NONE;
		meson_mmc_req_done(sc);
		return;
	}

	MMC_UNLOCK(sc);
}

/* ---- mmcbr interface ---- */

static int
meson_mmc_request(device_t bus, device_t child, struct mmc_request *req)
{
	struct meson_mmc_softc *sc;
	struct mmc_command *cmd;
	uint32_t cmd_cfg, cmd_data;
	uint32_t blksz;

	sc = device_get_softc(bus);

	MMC_LOCK(sc);
	if (sc->req != NULL) {
		MMC_UNLOCK(sc);
		return (EBUSY);
	}
	sc->req = req;
	cmd = req->cmd;
	cmd->error = MMC_ERR_NONE;

	/* Build CMD_CFG */
	cmd_cfg = 0;
	cmd_cfg |= (cmd->opcode << CMD_CFG_CMD_INDEX_SHIFT) &
	    CMD_CFG_CMD_INDEX_MASK;
	cmd_cfg |= CMD_CFG_OWNER;

	/* Response type */
	if (!(cmd->flags & MMC_RSP_PRESENT))
		cmd_cfg |= CMD_CFG_NO_RESP;
	else {
		if (cmd->flags & MMC_RSP_136)
			cmd_cfg |= CMD_CFG_RESP_128;
		if (!(cmd->flags & MMC_RSP_CRC))
			cmd_cfg |= CMD_CFG_RESP_NOCRC;
		if (cmd->flags & MMC_RSP_BUSY)
			cmd_cfg |= CMD_CFG_R1B;
	}

	/* Data */
	cmd_data = 0;
	if (cmd->data != NULL) {
		cmd_cfg |= CMD_CFG_DATA_IO;

		/* Timeout for data commands */
		cmd_cfg |= (meson_mmc_ilog2(MESON_MMC_CMD_TIMEOUT_DATA) <<
		    CMD_CFG_TIMEOUT_SHIFT) & CMD_CFG_TIMEOUT_MASK;

		if (cmd->data->len > MESON_MMC_BOUNCE_SIZE) {
			device_printf(sc->dev,
			    "data too large: %d > %d\n",
			    (int)cmd->data->len, MESON_MMC_BOUNCE_SIZE);
			cmd->error = MMC_ERR_INVALID;
			meson_mmc_req_done(sc);
			return (0);
		}

		blksz = (cmd->data->len < MMC_SECTOR_SIZE) ?
		    cmd->data->len : MMC_SECTOR_SIZE;

		if (cmd->data->len > blksz) {
			/* Multi-block transfer */
			uint32_t nblocks = cmd->data->len / blksz;

			/* Update block size in CFG register */
			uint32_t cfg = MMC_READ_4(sc, MESON_SD_EMMC_CFG);
			cfg &= ~CFG_BLK_LEN_MASK;
			cfg |= (meson_mmc_ilog2(blksz) <<
			    CFG_BLK_LEN_SHIFT) & CFG_BLK_LEN_MASK;
			MMC_WRITE_4(sc, MESON_SD_EMMC_CFG, cfg);

			cmd_cfg |= CMD_CFG_BLOCK_MODE;
			cmd_cfg |= (nblocks & CMD_CFG_LENGTH_MASK);
		} else {
			/* Single-block: length is byte count */
			cmd_cfg |= (cmd->data->len & CMD_CFG_LENGTH_MASK);
		}

		if (cmd->data->flags & MMC_DATA_WRITE) {
			cmd_cfg |= CMD_CFG_DATA_WR;
			/* Copy data to bounce buffer */
			memcpy(sc->dma_buf, cmd->data->data,
			    cmd->data->len);
		}

		cmd_data = sc->dma_buf_phys & CMD_DATA_ADDR_MASK;
	} else {
		/* Command timeout */
		cmd_cfg |= (meson_mmc_ilog2(MESON_MMC_CMD_TIMEOUT) <<
		    CMD_CFG_TIMEOUT_SHIFT) & CMD_CFG_TIMEOUT_MASK;
	}

	/* Mark end of chain (single descriptor mode) */
	cmd_cfg |= CMD_CFG_END_OF_CHAIN;

	/* Stop any previous operation */
	MMC_WRITE_4(sc, MESON_SD_EMMC_START, 0);

	/*
	 * Write registers in order: CMD_CFG, CMD_DAT, CMD_RSP, then CMD_ARG.
	 * Writing CMD_ARG last triggers the command execution.
	 */
	MMC_WRITE_4(sc, MESON_SD_EMMC_CMD_CFG, cmd_cfg);
	MMC_WRITE_4(sc, MESON_SD_EMMC_CMD_DAT, cmd_data);
	MMC_WRITE_4(sc, MESON_SD_EMMC_CMD_RSP, 0);
	wmb();
	MMC_WRITE_4(sc, MESON_SD_EMMC_CMD_ARG, cmd->arg);

	/* Set timeout callout */
	callout_reset(&sc->timeout_co, sc->timeout_secs * hz,
	    meson_mmc_timeout, sc);

	MMC_UNLOCK(sc);
	return (0);
}

static int
meson_mmc_update_ios(device_t bus, device_t child)
{
	struct meson_mmc_softc *sc;
	struct mmc_ios *ios;
	uint32_t cfg;

	sc = device_get_softc(bus);
	ios = &sc->host.ios;

	/* Set bus width */
	cfg = MMC_READ_4(sc, MESON_SD_EMMC_CFG);
	cfg &= ~CFG_BUS_WIDTH_MASK;
	switch (ios->bus_width) {
	case bus_width_8:
		cfg |= CFG_BUS_WIDTH_8;
		break;
	case bus_width_4:
		cfg |= CFG_BUS_WIDTH_4;
		break;
	default:
		cfg |= CFG_BUS_WIDTH_1;
		break;
	}
	MMC_WRITE_4(sc, MESON_SD_EMMC_CFG, cfg);

	/* Set clock rate */
	if (ios->clock != 0)
		meson_mmc_set_clock(sc, ios->clock);
	else
		meson_mmc_set_clock(sc, 0);

	/* Handle power mode */
	switch (ios->power_mode) {
	case power_up:
		mmc_fdt_set_power(&sc->mmc_helper, power_up);
		break;
	case power_on:
		mmc_fdt_set_power(&sc->mmc_helper, power_on);
		break;
	case power_off:
		mmc_fdt_set_power(&sc->mmc_helper, power_off);
		break;
	}

	return (0);
}

static int
meson_mmc_get_ro(device_t bus, device_t child)
{
	struct meson_mmc_softc *sc;

	sc = device_get_softc(bus);
	return (mmc_fdt_gpio_get_readonly(&sc->mmc_helper));
}

static int
meson_mmc_acquire_host(device_t bus, device_t child)
{

	return (0);
}

static int
meson_mmc_release_host(device_t bus, device_t child)
{

	return (0);
}

/* ---- IVARs (bridge MMC child to host struct) ---- */

static int
meson_mmc_read_ivar(device_t bus, device_t child, int which,
    uintptr_t *result)
{
	struct meson_mmc_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*result = sc->host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*result = sc->host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*result = sc->host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*result = sc->host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*result = sc->host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*result = sc->host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*result = sc->host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*result = sc->host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*result = sc->host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*result = sc->host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*result = sc->host.ios.vdd;
		break;
	case MMCBR_IVAR_VCCQ:
		*result = sc->host.ios.vccq;
		break;
	case MMCBR_IVAR_CAPS:
		*result = sc->host.caps;
		break;
	case MMCBR_IVAR_TIMING:
		*result = sc->host.ios.timing;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*result = MESON_MMC_MAX_BLKCNT;
		break;
	case MMCBR_IVAR_RETUNE_REQ:
		*result = retune_req_none;
		break;
	case MMCBR_IVAR_MAX_BUSY_TIMEOUT:
		*result = 0;
		break;
	}
	return (0);
}

static int
meson_mmc_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{
	struct meson_mmc_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->host.ios.vdd = value;
		break;
	case MMCBR_IVAR_VCCQ:
		sc->host.ios.vccq = value;
		break;
	case MMCBR_IVAR_TIMING:
		sc->host.ios.timing = value;
		break;
	/* Read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return (EINVAL);
	}
	return (0);
}

/* ---- Card detect callback ---- */

static void
meson_mmc_helper_cd_handler(device_t dev, bool present)
{
	struct meson_mmc_softc *sc;

	sc = device_get_softc(dev);

	bus_topo_lock();
	if (present) {
		if (sc->child == NULL) {
			sc->child = device_add_child(dev, "mmc",
			    DEVICE_UNIT_ANY);
			if (sc->child != NULL) {
				device_set_ivars(sc->child, sc);
				device_probe_and_attach(sc->child);
			}
		}
	} else {
		if (sc->child != NULL) {
			device_delete_child(dev, sc->child);
			sc->child = NULL;
		}
	}
	bus_topo_unlock();
}

/* ---- Device interface ---- */

static int
meson_mmc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson SD/eMMC Host Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_mmc_attach(device_t dev)
{
	struct meson_mmc_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->req = NULL;
	sc->timeout_secs = 10;

	/* Allocate resources (memory + IRQ) */
	if (bus_alloc_resources(dev, meson_mmc_res_spec, &sc->mem_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	/* Set up interrupt */
	if (bus_setup_intr(dev, sc->irq_res,
	    INTR_TYPE_NET | INTR_MPSAFE, NULL, meson_mmc_intr, sc,
	    &sc->intrhand) != 0) {
		device_printf(dev, "cannot setup interrupt handler\n");
		bus_release_resources(dev, meson_mmc_res_spec, &sc->mem_res);
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "meson_mmc", MTX_DEF);
	callout_init_mtx(&sc->timeout_co, &sc->mtx, 0);

	/* Get and deassert reset */
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &sc->rst) == 0) {
		error = hwreset_deassert(sc->rst);
		if (error != 0)
			device_printf(dev,
			    "cannot deassert reset: %d\n", error);
	}

	/*
	 * TODO: STUB -- clock handling
	 *
	 * Current behaviour:
	 *   We attempt to get the "core" clock and enable it.
	 *   The "clkin0" and "clkin1" clocks (crystal and fclk_div2)
	 *   are optional -- the controller's internal divider is
	 *   programmed directly based on known fixed frequencies.
	 *
	 * For a full implementation:
	 *   - Get clkin0/clkin1, query their actual rates
	 *   - Register the controller's internal mux+divider as clock nodes
	 *   - Use clk_set_freq() for rate changes
	 */
	if (clk_get_by_ofw_name(dev, 0, "core", &sc->clk_core) == 0)
		clk_enable(sc->clk_core);
	clk_get_by_ofw_name(dev, 0, "clkin0", &sc->clk_clkin0);
	clk_get_by_ofw_name(dev, 0, "clkin1", &sc->clk_clkin1);

	/* Set up DMA bounce buffer */
	error = meson_mmc_setup_dma(sc);
	if (error != 0) {
		device_printf(dev, "cannot setup DMA: %d\n", error);
		goto fail;
	}

	/* Initialize hardware */
	meson_mmc_hw_init(sc);

	/* Set host capabilities */
	sc->host.f_min = 400000;
	sc->host.f_max = 50000000;
	sc->host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->host.caps = MMC_CAP_HSPEED | MMC_CAP_SIGNALING_330;

	/* Parse DTS for bus-width, max-frequency, etc. */
	mmc_fdt_parse(dev, 0, &sc->mmc_helper, &sc->host);

	/* Set up card detect GPIO */
	mmc_fdt_gpio_setup(dev, 0, &sc->mmc_helper,
	    meson_mmc_helper_cd_handler);

	return (0);

fail:
	callout_drain(&sc->timeout_co);
	mtx_destroy(&sc->mtx);
	bus_teardown_intr(dev, sc->irq_res, sc->intrhand);
	bus_release_resources(dev, meson_mmc_res_spec, &sc->mem_res);
	return (ENXIO);
}

static int
meson_mmc_detach(device_t dev)
{
	struct meson_mmc_softc *sc;

	sc = device_get_softc(dev);

	mmc_fdt_gpio_teardown(&sc->mmc_helper);

	callout_drain(&sc->timeout_co);
	device_delete_children(dev);
	meson_mmc_teardown_dma(sc);

	if (sc->clk_core != NULL)
		clk_disable(sc->clk_core);
	if (sc->rst != NULL)
		hwreset_assert(sc->rst);

	mtx_destroy(&sc->mtx);
	bus_teardown_intr(dev, sc->irq_res, sc->intrhand);
	bus_release_resources(dev, meson_mmc_res_spec, &sc->mem_res);

	return (0);
}

static device_method_t meson_mmc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		meson_mmc_probe),
	DEVMETHOD(device_attach,	meson_mmc_attach),
	DEVMETHOD(device_detach,	meson_mmc_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	meson_mmc_read_ivar),
	DEVMETHOD(bus_write_ivar,	meson_mmc_write_ivar),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	meson_mmc_update_ios),
	DEVMETHOD(mmcbr_request,	meson_mmc_request),
	DEVMETHOD(mmcbr_get_ro,		meson_mmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	meson_mmc_acquire_host),
	DEVMETHOD(mmcbr_release_host,	meson_mmc_release_host),

	DEVMETHOD_END
};

static driver_t meson_mmc_driver = {
	"meson_mmc",
	meson_mmc_methods,
	sizeof(struct meson_mmc_softc),
};

DRIVER_MODULE(meson_mmc, simplebus, meson_mmc_driver, NULL, NULL);
MMC_DECLARE_BRIDGE(meson_mmc);
