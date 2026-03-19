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
 * Amlogic Meson GXL DWC GMAC glue driver.
 *
 * This driver provides SoC-specific configuration for the Synopsys
 * DesignWare Ethernet MAC on Amlogic Meson GXL SoCs:
 *
 *   - Configures PRG_ETH0 register for RMII mode (internal PHY)
 *   - Provides the correct MDIO clock divider for the ~166 MHz CSR clock
 *
 * Without this glue driver, the generic DWC driver uses a clock divider
 * for 25-35 MHz (DIV16), producing a ~10 MHz MDC clock that exceeds the
 * MDIO spec maximum of 2.5 MHz, causing PHY detection to fail.
 *
 * Linux reference: drivers/net/ethernet/stmicro/stmmac/dwmac-meson8b.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_media.h>

#include <machine/bus.h>

#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <dev/dwc/if_dwcvar.h>
#include <dev/dwc/dwc1000_reg.h>

#include "if_dwc_if.h"

/* PRG_ETH0 register bits (second reg range of ethmac DT node) */
#define	PRG_ETH0_RGMII_MODE		(1u << 0)
#define	PRG_ETH0_TXDLY_MASK		(3u << 5)
#define	PRG_ETH0_INVERTED_RMII_CLK	(1u << 11)
#define	PRG_ETH0_TX_AND_PHY_REF_CLK	(1u << 12)
#define	PRG_ETH0_ADJ_ENABLE		(1u << 13)
#define	PRG_ETH0_ADJ_SETUP		(1u << 14)
#define	PRG_ETH0_ADJ_DELAY		(0x1fu << 15)
#define	PRG_ETH0_ADJ_SKEW		(0x1fu << 20)

static int
meson_dwc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "amlogic,meson-gxbb-dwmac"))
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson GXL Gigabit Ethernet");

	/*
	 * Higher priority than the generic "snps,dwmac" driver (-20)
	 * since the DT node has both compatibles.
	 */
	return (BUS_PROBE_VENDOR);
}

static int
meson_dwc_init(device_t dev)
{
	struct resource *prg_eth;
	int rid;
	uint32_t val;

	/*
	 * Map PRG_ETH0 register (second memory resource in DT).
	 * This controls RGMII/RMII mode selection and clock routing.
	 */
	rid = 1;
	prg_eth = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (prg_eth == NULL) {
		device_printf(dev, "cannot map PRG_ETH0 register\n");
		return (ENXIO);
	}

	/*
	 * Configure for RMII mode with internal PHY.
	 *
	 * Linux equivalent (dwmac-meson8b.c):
	 *   meson8b_init_rgmii_delays() -> clear delay/timing bits for RMII
	 *   meson8b_set_phy_mode()      -> clear RGMII_MODE for RMII
	 *   meson8b_init_prg_eth()      -> set INVERTED_RMII_CLK, TX_AND_PHY_REF_CLK
	 */
	val = bus_read_4(prg_eth, 0);
	device_printf(dev, "PRG_ETH0 before: 0x%08x\n", val);

	val &= ~PRG_ETH0_RGMII_MODE;		/* Clear = RMII mode */
	val &= ~PRG_ETH0_TXDLY_MASK;		/* No TX delay for RMII */
	val &= ~PRG_ETH0_ADJ_ENABLE;		/* No timing adjustment */
	val &= ~PRG_ETH0_ADJ_SETUP;
	val &= ~PRG_ETH0_ADJ_DELAY;
	val &= ~PRG_ETH0_ADJ_SKEW;
	val |= PRG_ETH0_INVERTED_RMII_CLK;	/* Invert internal RMII clock */
	val |= PRG_ETH0_TX_AND_PHY_REF_CLK;	/* Enable TX_CLK & PHY_REF_CLK */
	bus_write_4(prg_eth, 0, val);

	device_printf(dev, "PRG_ETH0 after:  0x%08x\n", val);

	bus_release_resource(dev, SYS_RES_MEMORY, rid, prg_eth);

	return (0);
}

static int
meson_dwc_mii_clk(device_t dev)
{

	/*
	 * CSR clock is CLK81 ~166 MHz.  Falls in the 150-250 MHz range
	 * which requires a /102 divider to produce a ~1.6 MHz MDC clock.
	 */
	return (GMAC_MII_CLK_150_250M_DIV102);
}

static device_method_t meson_dwc_methods[] = {
	DEVMETHOD(device_probe,		meson_dwc_probe),

	DEVMETHOD(if_dwc_init,		meson_dwc_init),
	DEVMETHOD(if_dwc_mii_clk,	meson_dwc_mii_clk),

	DEVMETHOD_END
};

extern driver_t dwc_driver;

DEFINE_CLASS_1(dwc, meson_dwc_driver, meson_dwc_methods,
    sizeof(struct dwc_softc), dwc_driver);
DRIVER_MODULE(meson_dwc, simplebus, meson_dwc_driver, 0, 0);

MODULE_DEPEND(meson_dwc, dwc, 1, 1, 1);
