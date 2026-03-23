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
 * Amlogic Meson GXL MDIO mux / internal PHY power-on driver.
 *
 * The GXL SoC has an integrated 10/100 Ethernet PHY that must be
 * powered on via ETH_REG2/3/4 control registers before the DWC MAC
 * can see it on its MDIO bus.
 *
 * This driver:
 *   - Claims the "amlogic,gxl-mdio-mux" DT node
 *   - Maps the 12-byte control register space (ETH_REG2/3/4)
 *   - Performs the internal PHY power-on sequence
 *   - The generic DWC driver then finds the PHY at MDIO address 8
 *
 * No MDIO bus methods are implemented: the DWC MAC's built-in MDIO
 * controller handles all MDIO transactions.  This driver only needs
 * to power on the PHY before DWC scans the bus.
 *
 * Ordering: this device sits on the periphs simple-bus which appears
 * before the DWC MAC node in DT order, so it attaches first.
 *
 * Linux reference: drivers/net/mdio/mdio-mux-meson-gxl.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/clk.h>

/* Register offsets from node base */
#define	ETH_REG2		0x0
#define	ETH_REG3		0x4
#define	ETH_REG4		0x8

/* ETH_REG2 fields */
#define	REG2_PHYID_MASK		0x003fffff
#define	EPHY_GXL_ID		0x110181
#define	REG2_REVERSED		(1u << 28)

/* ETH_REG3 fields */
#define	REG3_ENH		(1u << 3)
#define	REG3_CFGMODE_SHIFT	4
#define	REG3_AUTOMDIX		(1u << 7)
#define	REG3_PHYADDR_SHIFT	8
#define	REG3_LEDPOL		(1u << 23)
#define	REG3_PHYMDI		(1u << 26)
#define	REG3_CLKINEN		(1u << 29)
#define	REG3_PHYIP		(1u << 30)
#define	REG3_PHYEN		(1u << 31)

/* ETH_REG4 fields */
#define	REG4_PWRUPRSTSIG	(1u << 0)

struct meson_mdio_mux_softc {
	device_t		dev;
	struct resource		*res;
	clk_t			clk_ref;
};

static struct resource_spec meson_mdio_mux_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	RESOURCE_SPEC_END
};

#define	RD4(sc, reg)		bus_read_4((sc)->res, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static void
meson_mdio_mux_enable_internal_phy(struct meson_mdio_mux_softc *sc)
{
	uint32_t val;

	/*
	 * Build REG3 value: configure internal PHY control without
	 * enabling the PHY yet (PHYEN must be set last).
	 */
	val = REG3_ENH |
	    (0x7u << REG3_CFGMODE_SHIFT) |
	    REG3_AUTOMDIX |
	    (8u << REG3_PHYADDR_SHIFT) |
	    REG3_LEDPOL |
	    REG3_PHYMDI |
	    REG3_CLKINEN |
	    REG3_PHYIP;

	/* Assert power-up reset signal */
	WR4(sc, ETH_REG4, REG4_PWRUPRSTSIG);

	/* Write PHY control config */
	WR4(sc, ETH_REG3, val);

	/* Wait for analog to settle */
	DELAY(10000);

	/*
	 * Program PHY ID.  This is arbitrary but must match the PHY
	 * driver's expected ID (Linux uses 0x01814400 which is derived
	 * from EPHY_GXL_ID via the PHY ID register encoding).
	 */
	WR4(sc, ETH_REG2, REG2_REVERSED | (EPHY_GXL_ID & REG2_PHYID_MASK));

	/* Enable the internal PHY (must be last) */
	val |= REG3_PHYEN;
	WR4(sc, ETH_REG3, val);

	/* Deassert power-up reset */
	WR4(sc, ETH_REG4, 0);

	/* Wait for PHY to power up */
	DELAY(10000);
}

static int
meson_mdio_mux_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "amlogic,gxl-mdio-mux"))
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson GXL MDIO Mux");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_mdio_mux_attach(device_t dev)
{
	struct meson_mdio_mux_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, meson_mdio_mux_res_spec,
	    &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		return (ENXIO);
	}

	/* Enable reference clock (FCLK_DIV4) */
	if (clk_get_by_ofw_name(dev, 0, "ref", &sc->clk_ref) == 0)
		clk_enable(sc->clk_ref);

	/* Power on the internal PHY */
	meson_mdio_mux_enable_internal_phy(sc);

	device_printf(dev, "internal PHY enabled (addr 8)\n");
	return (0);
}

static int
meson_mdio_mux_detach(device_t dev)
{
	struct meson_mdio_mux_softc *sc;

	sc = device_get_softc(dev);
	if (sc->clk_ref != NULL) {
		clk_disable(sc->clk_ref);
		clk_release(sc->clk_ref);
	}
	bus_release_resources(dev, meson_mdio_mux_res_spec, &sc->res);
	return (0);
}

static device_method_t meson_mdio_mux_methods[] = {
	DEVMETHOD(device_probe,		meson_mdio_mux_probe),
	DEVMETHOD(device_attach,	meson_mdio_mux_attach),
	DEVMETHOD(device_detach,	meson_mdio_mux_detach),
	DEVMETHOD_END
};

static driver_t meson_mdio_mux_driver = {
	"meson_mdio_mux",
	meson_mdio_mux_methods,
	sizeof(struct meson_mdio_mux_softc),
};

DRIVER_MODULE(meson_mdio_mux, simplebus, meson_mdio_mux_driver, 0, 0);
MODULE_VERSION(meson_mdio_mux, 1);
MODULE_DEPEND(meson_mdio_mux, ether, 1, 1, 1);
