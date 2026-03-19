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
 * I2C controller driver for the Amlogic Meson SoC family (GXBB, GXL, AXG, etc.).
 *
 * The Meson I2C controller uses a token-based protocol: the driver builds
 * a sequence of 4-bit tokens (START, ADDR, DATA, STOP) into a pair of
 * 32-bit registers, pre-loads write data, triggers the transfer, polls
 * for completion, and extracts read data.  Up to 8 data bytes can be
 * transferred per token sequence.
 *
 * Reference: Linux drivers/i2c/busses/i2c-meson.c
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/clk.h>

#include "iicbus_if.h"

/* Register offsets */
#define	MESON_I2C_CTRL		0x00
#define	MESON_I2C_SLAVE_ADDR	0x04
#define	MESON_I2C_TOK_LIST0	0x08
#define	MESON_I2C_TOK_LIST1	0x0c
#define	MESON_I2C_WDATA0	0x10
#define	MESON_I2C_WDATA1	0x14
#define	MESON_I2C_RDATA0	0x18
#define	MESON_I2C_RDATA1	0x1c

/* CTRL register bits */
#define	MESON_I2C_CTRL_START		(1u << 0)
#define	MESON_I2C_CTRL_ACK_IGNORE	(1u << 1)
#define	MESON_I2C_CTRL_STATUS		(1u << 2)
#define	MESON_I2C_CTRL_ERROR		(1u << 3)
#define	MESON_I2C_CTRL_CLKDIV_SHIFT	12
#define	MESON_I2C_CTRL_CLKDIV_MASK	(0x3ffu << 12)
#define	MESON_I2C_CTRL_CLKDIVEXT_SHIFT	28
#define	MESON_I2C_CTRL_CLKDIVEXT_MASK	(0x3u << 28)

/* SLAVE_ADDR register bits */
#define	MESON_I2C_SLV_ADDR_MASK		0xff
#define	MESON_I2C_SLV_SDA_FILTER_SHIFT	8
#define	MESON_I2C_SLV_SDA_FILTER_MASK	(0x7u << 8)
#define	MESON_I2C_SLV_SCL_FILTER_SHIFT	11
#define	MESON_I2C_SLV_SCL_FILTER_MASK	(0x7u << 11)
#define	MESON_I2C_SLV_SCL_LOW_SHIFT	16
#define	MESON_I2C_SLV_SCL_LOW_MASK	(0xfffu << 16)
#define	MESON_I2C_SLV_SCL_LOW_EN	(1u << 28)

/* Token types (4-bit) */
#define	TOKEN_END		0
#define	TOKEN_START		1
#define	TOKEN_SLAVE_ADDR_WRITE	2
#define	TOKEN_SLAVE_ADDR_READ	3
#define	TOKEN_DATA		4
#define	TOKEN_DATA_LAST		5
#define	TOKEN_STOP		6

/* Max data bytes per token sequence */
#define	MESON_I2C_MAX_DATA	8

/* Filter delay (cycles) subtracted from clock divider */
#define	MESON_I2C_FILTER_DELAY	15

/* Timeout: 500ms in microseconds */
#define	MESON_I2C_TIMEOUT_US	500000

#define	MESON_I2C_READ(sc, reg)		\
    bus_read_4((sc)->res[0], (reg))
#define	MESON_I2C_WRITE(sc, reg, val)	\
    bus_write_4((sc)->res[0], (reg), (val))

struct meson_i2c_softc {
	device_t	dev;
	device_t	iicbus;
	struct resource	*res[1];	/* Memory only (polled, no IRQ) */
	struct mtx	mtx;
	clk_t		clk;

	/* Token state (rebuilt per chunk) */
	uint32_t	tokens[2];
	int		num_tokens;
};

static struct resource_spec meson_i2c_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0 }
};

static void
meson_i2c_add_token(struct meson_i2c_softc *sc, int token)
{
	int idx, shift;

	if (sc->num_tokens < 8) {
		idx = 0;
		shift = sc->num_tokens * 4;
	} else {
		idx = 1;
		shift = (sc->num_tokens % 8) * 4;
	}
	sc->tokens[idx] |= (token & 0xf) << shift;
	sc->num_tokens++;
}

static void
meson_i2c_reset_tokens(struct meson_i2c_softc *sc)
{

	sc->tokens[0] = 0;
	sc->tokens[1] = 0;
	sc->num_tokens = 0;
}

static void
meson_i2c_put_data(struct meson_i2c_softc *sc, const uint8_t *buf, int len)
{
	uint32_t wdata0, wdata1;
	int i;

	wdata0 = 0;
	wdata1 = 0;
	for (i = 0; i < len && i < 4; i++)
		wdata0 |= (uint32_t)buf[i] << (i * 8);
	for (; i < len && i < 8; i++)
		wdata1 |= (uint32_t)buf[i] << ((i - 4) * 8);

	MESON_I2C_WRITE(sc, MESON_I2C_WDATA0, wdata0);
	MESON_I2C_WRITE(sc, MESON_I2C_WDATA1, wdata1);
}

static void
meson_i2c_get_data(struct meson_i2c_softc *sc, uint8_t *buf, int len)
{
	uint32_t rdata0, rdata1;
	int i;

	rdata0 = MESON_I2C_READ(sc, MESON_I2C_RDATA0);
	rdata1 = MESON_I2C_READ(sc, MESON_I2C_RDATA1);

	for (i = 0; i < len && i < 4; i++)
		buf[i] = (rdata0 >> (i * 8)) & 0xff;
	for (; i < len && i < 8; i++)
		buf[i] = (rdata1 >> ((i - 4) * 8)) & 0xff;
}

/*
 * Poll for transfer completion.  Returns 0 on success, IIC error on failure.
 */
static int
meson_i2c_wait(struct meson_i2c_softc *sc)
{
	uint32_t ctrl;
	int timeout;

	for (timeout = MESON_I2C_TIMEOUT_US / 10; timeout > 0; timeout--) {
		ctrl = MESON_I2C_READ(sc, MESON_I2C_CTRL);
		if (!(ctrl & MESON_I2C_CTRL_STATUS)) {
			if (ctrl & MESON_I2C_CTRL_ERROR)
				return (IIC_ENOACK);
			return (IIC_NOERR);
		}
		DELAY(10);
	}

	return (IIC_ETIMEOUT);
}

/*
 * Transfer a single chunk (up to 8 bytes) of a message.
 * The token sequence has already been built by the caller.
 */
static int
meson_i2c_xfer_chunk(struct meson_i2c_softc *sc)
{
	uint32_t ctrl;

	/* Write token list */
	MESON_I2C_WRITE(sc, MESON_I2C_TOK_LIST0, sc->tokens[0]);
	MESON_I2C_WRITE(sc, MESON_I2C_TOK_LIST1, sc->tokens[1]);

	/* Trigger transfer: 0→1 transition on START bit */
	ctrl = MESON_I2C_READ(sc, MESON_I2C_CTRL);
	ctrl &= ~MESON_I2C_CTRL_START;
	MESON_I2C_WRITE(sc, MESON_I2C_CTRL, ctrl);
	ctrl |= MESON_I2C_CTRL_START;
	MESON_I2C_WRITE(sc, MESON_I2C_CTRL, ctrl);

	return (meson_i2c_wait(sc));
}

/*
 * Transfer a single I2C message (possibly in multiple chunks).
 */
static int
meson_i2c_xfer_msg(struct meson_i2c_softc *sc, struct iic_msg *msg, int last)
{
	int is_read, pos, count, i, error;

	is_read = (msg->flags & IIC_M_RD) != 0;

	/* Set slave address (7-bit address, shifted left by 1) */
	MESON_I2C_WRITE(sc, MESON_I2C_SLAVE_ADDR,
	    (MESON_I2C_READ(sc, MESON_I2C_SLAVE_ADDR) &
	    ~MESON_I2C_SLV_ADDR_MASK) |
	    ((msg->slave >> 1) << 1));

	for (pos = 0; pos < msg->len; ) {
		count = msg->len - pos;
		if (count > MESON_I2C_MAX_DATA)
			count = MESON_I2C_MAX_DATA;

		meson_i2c_reset_tokens(sc);

		/* START + address on first chunk */
		if (pos == 0 && !(msg->flags & IIC_M_NOSTART)) {
			meson_i2c_add_token(sc, TOKEN_START);
			meson_i2c_add_token(sc, is_read ?
			    TOKEN_SLAVE_ADDR_READ : TOKEN_SLAVE_ADDR_WRITE);
		}

		/* Data tokens */
		for (i = 0; i < count; i++) {
			if (is_read && last && pos + i == msg->len - 1)
				meson_i2c_add_token(sc, TOKEN_DATA_LAST);
			else
				meson_i2c_add_token(sc, TOKEN_DATA);
		}

		/* STOP on last chunk of last message */
		if (last && pos + count >= msg->len)
			meson_i2c_add_token(sc, TOKEN_STOP);

		/* Pre-load write data */
		if (!is_read)
			meson_i2c_put_data(sc, msg->buf + pos, count);

		/* Execute */
		error = meson_i2c_xfer_chunk(sc);
		if (error != IIC_NOERR)
			return (error);

		/* Extract read data */
		if (is_read)
			meson_i2c_get_data(sc, msg->buf + pos, count);

		pos += count;
	}

	/* Handle zero-length messages (address probe) */
	if (msg->len == 0) {
		meson_i2c_reset_tokens(sc);
		if (!(msg->flags & IIC_M_NOSTART)) {
			meson_i2c_add_token(sc, TOKEN_START);
			meson_i2c_add_token(sc, is_read ?
			    TOKEN_SLAVE_ADDR_READ : TOKEN_SLAVE_ADDR_WRITE);
		}
		if (last)
			meson_i2c_add_token(sc, TOKEN_STOP);
		error = meson_i2c_xfer_chunk(sc);
		if (error != IIC_NOERR)
			return (error);
	}

	return (IIC_NOERR);
}

static int
meson_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct meson_i2c_softc *sc;
	uint32_t ctrl;
	int i, error;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);

	/* Clear error/start bits */
	ctrl = MESON_I2C_READ(sc, MESON_I2C_CTRL);
	ctrl &= ~(MESON_I2C_CTRL_START | MESON_I2C_CTRL_ERROR);
	MESON_I2C_WRITE(sc, MESON_I2C_CTRL, ctrl);

	error = IIC_NOERR;
	for (i = 0; i < (int)nmsgs; i++) {
		error = meson_i2c_xfer_msg(sc, &msgs[i],
		    i == (int)nmsgs - 1);
		if (error != IIC_NOERR) {
			device_printf(sc->dev,
			    "xfer msg %d/%d failed: %s addr=0x%x "
			    "len=%d flags=0x%x ctrl=0x%08x slave=0x%08x\n",
			    i, nmsgs,
			    error == IIC_ENOACK ? "NACK" :
			    error == IIC_ETIMEOUT ? "TIMEOUT" : "ERROR",
			    msgs[i].slave, msgs[i].len, msgs[i].flags,
			    MESON_I2C_READ(sc, MESON_I2C_CTRL),
			    MESON_I2C_READ(sc, MESON_I2C_SLAVE_ADDR));
			break;
		}
	}

	/* Clear start bit */
	ctrl = MESON_I2C_READ(sc, MESON_I2C_CTRL);
	ctrl &= ~MESON_I2C_CTRL_START;
	MESON_I2C_WRITE(sc, MESON_I2C_CTRL, ctrl);

	mtx_unlock(&sc->mtx);
	return (error);
}

static int
meson_i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{

	return (IIC_NOERR);
}

/*
 * Set up the I2C clock divider for GXBB/AXG variants.
 * These support asymmetric SCL duty cycle (separate high/low timing).
 */
static void
meson_i2c_set_clk_div(struct meson_i2c_softc *sc, uint64_t clk_rate,
    uint32_t freq)
{
	uint32_t div_h, div_l, ctrl, slave;

	if (freq <= 100000) {
		/*
		 * Standard mode (100 kHz).
		 * Match Linux meson_gxbb_axg_i2c_set_clk_div():
		 *   div_temp = ceil(clk_rate / freq)   -- full period in clocks
		 *   div_h = ceil(div_temp / 2) - 15    -- SCL high half-period
		 *   div_l = ceil(div_temp / 4)         -- SCL low quarter-period
		 */
		uint32_t div_temp = howmany(clk_rate, freq);
		div_h = howmany(div_temp, 2) - MESON_I2C_FILTER_DELAY;
		div_l = howmany(div_temp, 4);
	} else {
		/*
		 * Fast mode (400 kHz).
		 * Match Linux: 40% high / 60% low duty cycle.
		 *   div_h = ceil(clk_rate * 2 / (freq * 5)) - 15
		 *   div_l = ceil(clk_rate * 3 / (freq * 5 * 2))
		 */
		div_h = howmany(clk_rate * 2, (uint64_t)freq * 5) -
		    MESON_I2C_FILTER_DELAY;
		div_l = howmany(clk_rate * 3, (uint64_t)freq * 5 * 2);
	}

	/* Clamp to 12 bits */
	if (div_h > 0xfff)
		div_h = 0xfff;
	if (div_l > 0xfff)
		div_l = 0xfff;

	/* div_h[9:0] → CTRL[21:12], div_h[11:10] → CTRL[29:28] */
	ctrl = MESON_I2C_READ(sc, MESON_I2C_CTRL);
	ctrl &= ~(MESON_I2C_CTRL_CLKDIV_MASK | MESON_I2C_CTRL_CLKDIVEXT_MASK);
	ctrl |= (div_h & 0x3ff) << MESON_I2C_CTRL_CLKDIV_SHIFT;
	ctrl |= ((div_h >> 10) & 0x3) << MESON_I2C_CTRL_CLKDIVEXT_SHIFT;
	MESON_I2C_WRITE(sc, MESON_I2C_CTRL, ctrl);

	/* div_l → SLAVE_ADDR[27:16], enable SCL_LOW mode */
	slave = MESON_I2C_READ(sc, MESON_I2C_SLAVE_ADDR);
	slave &= ~(MESON_I2C_SLV_SCL_LOW_MASK |
	    MESON_I2C_SLV_SDA_FILTER_MASK | MESON_I2C_SLV_SCL_FILTER_MASK);
	slave |= (div_l & 0xfff) << MESON_I2C_SLV_SCL_LOW_SHIFT;
	slave |= MESON_I2C_SLV_SCL_LOW_EN;
	MESON_I2C_WRITE(sc, MESON_I2C_SLAVE_ADDR, slave);
}

static struct ofw_compat_data meson_i2c_compat[] = {
	{ "amlogic,meson-gxbb-i2c",	1 },
	{ "amlogic,meson-axg-i2c",	1 },
	{ NULL,				0 }
};

static int meson_i2c_detach(device_t dev);

static int
meson_i2c_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, meson_i2c_compat)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson I2C Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_i2c_attach(device_t dev)
{
	struct meson_i2c_softc *sc;
	phandle_t node;
	uint64_t clk_rate;
	uint32_t bus_freq;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), "meson_i2c", MTX_DEF);

	if (bus_alloc_resources(dev, meson_i2c_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		error = ENXIO;
		goto fail;
	}

	/* Get and enable the I2C clock */
	error = clk_get_by_ofw_index(dev, node, 0, &sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot get clock\n");
		goto fail;
	}
	error = clk_enable(sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot enable clock\n");
		goto fail;
	}

	/* Get bus frequency from DTS, default 100 kHz */
	if (OF_getencprop(node, "clock-frequency", &bus_freq,
	    sizeof(bus_freq)) <= 0)
		bus_freq = 100000;

	error = clk_get_freq(sc->clk, &clk_rate);
	if (error != 0 || clk_rate == 0) {
		device_printf(dev, "cannot get clock frequency\n");
		goto fail;
	}

	/* Clear controller state */
	MESON_I2C_WRITE(sc, MESON_I2C_CTRL, 0);

	/* Set up clock divider */
	meson_i2c_set_clk_div(sc, clk_rate, bus_freq);

	/* Add iicbus child */
	sc->iicbus = device_add_child(dev, "iicbus", DEVICE_UNIT_ANY);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child\n");
		error = ENXIO;
		goto fail;
	}

	bus_attach_children(dev);
	return (0);

fail:
	meson_i2c_detach(dev);
	return (error);
}

static int
meson_i2c_detach(device_t dev)
{
	struct meson_i2c_softc *sc;
	int error;

	sc = device_get_softc(dev);

	if ((error = bus_generic_detach(dev)) != 0)
		return (error);

	if (sc->clk != NULL)
		clk_release(sc->clk);

	bus_release_resources(dev, meson_i2c_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static phandle_t
meson_i2c_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t meson_i2c_methods[] = {
	DEVMETHOD(device_probe,		meson_i2c_probe),
	DEVMETHOD(device_attach,	meson_i2c_attach),
	DEVMETHOD(device_detach,	meson_i2c_detach),

	/* OFW */
	DEVMETHOD(ofw_bus_get_node,	meson_i2c_get_node),

	/* I2C */
	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		meson_i2c_reset),
	DEVMETHOD(iicbus_transfer,	meson_i2c_transfer),

	DEVMETHOD_END
};

static driver_t meson_i2c_driver = {
	"meson_i2c",
	meson_i2c_methods,
	sizeof(struct meson_i2c_softc),
};

EARLY_DRIVER_MODULE(meson_i2c, simplebus, meson_i2c_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
EARLY_DRIVER_MODULE(ofw_iicbus, meson_i2c, ofw_iicbus_driver,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_DEPEND(meson_i2c, iicbus, 1, 1, 1);
MODULE_VERSION(meson_i2c, 1);
