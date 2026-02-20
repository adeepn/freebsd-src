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
 * Amlogic Meson clock controller — base class.
 *
 * Provides the clkdev_if methods (register I/O, locking) and a common
 * attach helper that creates a clock domain with fixed-rate clocks.
 *
 * SoC-specific drivers (meson_clk_gxbb.c, etc.) subclass this via
 * DEFINE_CLASS_1() and provide their own probe/attach with clock tables.
 *
 * TODO: STUB — core Meson clock controller framework
 *
 * Current behavior:
 *   Only supports registering fixed-rate clocks via meson_clk_attach().
 *   The clkdev_if register I/O methods (read_4, write_4, modify_4) are
 *   present but will return ENXIO when sc->res is NULL (the common case
 *   for the current stub drivers that don't need register access).
 *
 * For full implementation:
 *   - SoC-specific drivers should allocate a register resource (via
 *     syscon or direct bus_alloc_resource) and store it in sc->res
 *   - Add support for mux, divider, gate, and PLL clock types
 *   - Implement clk_set_rate()/clk_set_parent() for dynamic clocks
 *   - Add hwreset_if methods if the clock controller also provides
 *     reset functionality (as the AO clock controller does)
 *
 * Linux reference: drivers/clk/meson/ (gxbb.c, g12a.c, axg.c, etc.)
 * FreeBSD template: sys/dev/clk/allwinner/aw_ccung.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_fixed.h>

#include <dev/clk/meson/meson_clk.h>

#include "clkdev_if.h"

#if 0
#define	dprintf(format, arg...)	\
	device_printf(dev, "%s: " format, __func__, arg)
#else
#define	dprintf(format, arg...)
#endif

/*
 * clkdev_if methods.
 *
 * These provide register I/O for clock nodes that need hardware access
 * (mux, divider, gate, PLL clocks).  Fixed-rate clocks do not call
 * these methods.  In the current stub, sc->res is NULL and these
 * return ENXIO — they exist to satisfy the clkdev_if interface
 * contract and will be functional when register resources are added.
 */

static int
meson_clk_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->res == NULL)
		return (ENXIO);
	dprintf("offset=%lx write %x\n", addr, val);
	bus_write_4(sc->res, addr, val);
	return (0);
}

static int
meson_clk_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->res == NULL)
		return (ENXIO);
	*val = bus_read_4(sc->res, addr);
	dprintf("offset=%lx read %x\n", addr, *val);
	return (0);
}

static int
meson_clk_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct meson_clk_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	if (sc->res == NULL)
		return (ENXIO);
	dprintf("offset=%lx clr: %x set: %x\n", addr, clr, set);
	reg = bus_read_4(sc->res, addr);
	reg &= ~clr;
	reg |= set;
	bus_write_4(sc->res, addr, reg);
	return (0);
}

static void
meson_clk_device_lock(device_t dev)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
meson_clk_device_unlock(device_t dev)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

/*
 * Common attach helper.
 *
 * Creates a clock domain and registers an array of fixed-rate clocks.
 * SoC-specific drivers call this from their attach method.
 */
int
meson_clk_attach(device_t dev, struct clk_fixed_def *clks, int nclks)
{
	struct meson_clk_softc *sc;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		panic("Cannot create clkdom\n");

	for (i = 0; i < nclks; i++)
		clknode_fixed_register(sc->clkdom, &clks[i]);

	if (clkdom_finit(sc->clkdom) != 0)
		panic("cannot finalize clkdom initialization\n");

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	return (0);
}

static device_method_t meson_clkc_methods[] = {
	/* clkdev interface */
	DEVMETHOD(clkdev_write_4,	meson_clk_write_4),
	DEVMETHOD(clkdev_read_4,	meson_clk_read_4),
	DEVMETHOD(clkdev_modify_4,	meson_clk_modify_4),
	DEVMETHOD(clkdev_device_lock,	meson_clk_device_lock),
	DEVMETHOD(clkdev_device_unlock,	meson_clk_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(meson_clkc, meson_clkc_driver, meson_clkc_methods,
    sizeof(struct meson_clk_softc));
