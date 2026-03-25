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
 * Register I/O is provided via the parent's syscon handle (for drivers
 * on simple_mfd, e.g. the EE clock controller) or via a direct resource
 * (sc->res, for drivers that call bus_alloc_resource themselves).
 *
 * Currently supports fixed-rate, fixed-factor, and gate clocks.
 *
 * Future work:
 *   - Mux and divider clock types (for clk81 hardware readback)
 *   - PLL clock type with set_rate (for cpufreq via sys_pll)
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

#include <dev/syscon/syscon.h>

#include "clkdev_if.h"
#include "syscon_if.h"

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
 * these methods.
 *
 * Register access uses the parent's syscon (sc->syscon) when available,
 * falling back to a direct resource (sc->res).  The syscon path is used
 * by drivers on simple_mfd (EE clkc); the resource path is reserved for
 * future drivers that allocate their own register space.
 *
 * Locking: when using syscon, device_lock/unlock delegates to the
 * parent's SYSCON_DEVICE_LOCK (spin lock).  The unlocked syscon
 * accessors are used inside the locked region.
 */

static int
meson_clk_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	dprintf("offset=%lx write %x\n", addr, val);
	if (sc->syscon != NULL) {
		SYSCON_UNLOCKED_WRITE_4(sc->syscon, addr, val);
		return (0);
	}
	if (sc->res != NULL) {
		bus_write_4(sc->res, addr, val);
		return (0);
	}
	return (ENXIO);
}

static int
meson_clk_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->syscon != NULL) {
		*val = SYSCON_UNLOCKED_READ_4(sc->syscon, addr);
		dprintf("offset=%lx read %x\n", addr, *val);
		return (0);
	}
	if (sc->res != NULL) {
		*val = bus_read_4(sc->res, addr);
		dprintf("offset=%lx read %x\n", addr, *val);
		return (0);
	}
	return (ENXIO);
}

static int
meson_clk_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->syscon != NULL) {
		SYSCON_UNLOCKED_MODIFY_4(sc->syscon, addr, clr, set);
		return (0);
	}
	if (sc->res != NULL) {
		uint32_t reg;

		reg = bus_read_4(sc->res, addr);
		reg &= ~clr;
		reg |= set;
		bus_write_4(sc->res, addr, reg);
		return (0);
	}
	return (ENXIO);
}

static void
meson_clk_device_lock(device_t dev)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->syscon != NULL)
		SYSCON_DEVICE_LOCK(device_get_parent(dev));
	else
		mtx_lock(&sc->mtx);
}

static void
meson_clk_device_unlock(device_t dev)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->syscon != NULL)
		SYSCON_DEVICE_UNLOCK(device_get_parent(dev));
	else
		mtx_unlock(&sc->mtx);
}

/*
 * Common attach helper.
 *
 * Creates a clock domain and registers fixed-rate and gate clocks.
 * SoC-specific drivers call this from their attach method after
 * setting up sc->syscon or sc->res for register I/O.
 */
int
meson_clk_attach(device_t dev, struct clk_fixed_def *fixed, int nfixed,
    struct clk_gate_def *gates, int ngates)
{
	struct meson_clk_softc *sc;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		panic("Cannot create clkdom\n");

	for (i = 0; i < nfixed; i++)
		clknode_fixed_register(sc->clkdom, &fixed[i]);

	for (i = 0; i < ngates; i++)
		clknode_gate_register(sc->clkdom, &gates[i]);

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
