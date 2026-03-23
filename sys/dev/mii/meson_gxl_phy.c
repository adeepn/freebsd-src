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
 * Amlogic Meson GXL internal 10/100 Ethernet PHY driver.
 *
 * The GXL internal PHY has a configurable PHY ID (programmed via
 * ETH_REG2 in the MDIO mux block) and requires specific analog
 * register programming for the fractional PLL to work correctly.
 * Without this init, the PHY's internal clock drifts, causing the
 * Ethernet link to flap continuously.
 *
 * The PHY also has a known auto-negotiation quirk: it may report
 * aneg completion with incorrect LPA values.  A magic status bit
 * in the WOL register bank is used to detect and recover from this.
 *
 * Linux reference: drivers/net/phy/meson-gxl.c
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"

#include "miibus_if.h"

/* Indirect register access via TSTCNTL (MII reg 20) */
#define	TSTCNTL			20
#define	 TSTCNTL_READ		(1u << 15)
#define	 TSTCNTL_WRITE		(1u << 14)
#define	 TSTCNTL_BANK_SHIFT	11
#define	 TSTCNTL_BANK_MASK	(0x3u << 11)
#define	 TSTCNTL_TEST_MODE	(1u << 10)
#define	 TSTCNTL_READ_ADDR_SHIFT 5
#define	 TSTCNTL_WRITE_ADDR_MASK 0x1fu

#define	TSTREAD1		21
#define	TSTWRITE		23

/* Register banks */
#define	BANK_ANALOG_DSP		0
#define	BANK_WOL		1
#define	BANK_BIST		3

/* WOL bank registers */
#define	LPI_STATUS		0x0c
#define	 LPI_STATUS_RSV12	(1u << 12)

/* BIST bank registers */
#define	FR_PLL_CONTROL		0x1b
#define	FR_PLL_DIV0		0x1c
#define	FR_PLL_DIV1		0x1d

static int	meson_gxl_phy_probe(device_t);
static int	meson_gxl_phy_attach(device_t);
static int	meson_gxl_phy_service(struct mii_softc *,
		    struct mii_data *, int);
static void	meson_gxl_phy_status(struct mii_softc *);
static void	meson_gxl_phy_reset(struct mii_softc *);

static device_method_t meson_gxl_phy_methods[] = {
	DEVMETHOD(device_probe,		meson_gxl_phy_probe),
	DEVMETHOD(device_attach,	meson_gxl_phy_attach),
	DEVMETHOD(device_detach,	mii_phy_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD_END
};

static driver_t meson_gxl_phy_driver = {
	"meson_gxl_phy",
	meson_gxl_phy_methods,
	sizeof(struct mii_softc)
};

DRIVER_MODULE(meson_gxl_phy, miibus, meson_gxl_phy_driver, 0, 0);

static const struct mii_phydesc meson_gxl_phys[] = {
	MII_PHY_DESC(xxQUALSEMI, QS6612),
	MII_PHY_END
};

static const struct mii_phy_funcs meson_gxl_phy_funcs = {
	meson_gxl_phy_service,
	meson_gxl_phy_status,
	meson_gxl_phy_reset
};

static void
meson_gxl_open_banks(struct mii_softc *sc)
{

	/* Toggle TSTCNTL_TEST_MODE to enable bank access */
	PHY_WRITE(sc, TSTCNTL, 0);
	PHY_WRITE(sc, TSTCNTL, TSTCNTL_TEST_MODE);
	PHY_WRITE(sc, TSTCNTL, 0);
	PHY_WRITE(sc, TSTCNTL, TSTCNTL_TEST_MODE);
}

static void
meson_gxl_close_banks(struct mii_softc *sc)
{

	PHY_WRITE(sc, TSTCNTL, 0);
}

static int
meson_gxl_read_bank(struct mii_softc *sc, unsigned int bank,
    unsigned int reg)
{
	int val;

	meson_gxl_open_banks(sc);

	PHY_WRITE(sc, TSTCNTL, TSTCNTL_READ |
	    ((bank << TSTCNTL_BANK_SHIFT) & TSTCNTL_BANK_MASK) |
	    TSTCNTL_TEST_MODE |
	    ((reg << TSTCNTL_READ_ADDR_SHIFT) & (0x1fu << 5)));

	val = PHY_READ(sc, TSTREAD1);

	meson_gxl_close_banks(sc);
	return (val);
}

static void
meson_gxl_write_bank(struct mii_softc *sc, unsigned int bank,
    unsigned int reg, uint16_t value)
{

	meson_gxl_open_banks(sc);

	PHY_WRITE(sc, TSTWRITE, value);
	PHY_WRITE(sc, TSTCNTL, TSTCNTL_WRITE |
	    ((bank << TSTCNTL_BANK_SHIFT) & TSTCNTL_BANK_MASK) |
	    TSTCNTL_TEST_MODE |
	    (reg & TSTCNTL_WRITE_ADDR_MASK));

	meson_gxl_close_banks(sc);
}

/*
 * Program the fractional PLL.  Without this, the PHY's internal
 * clock is unstable and the link flaps continuously.
 */
static void
meson_gxl_config_init(struct mii_softc *sc)
{

	meson_gxl_write_bank(sc, BANK_BIST, FR_PLL_CONTROL, 0x5);
	meson_gxl_write_bank(sc, BANK_BIST, FR_PLL_DIV1, 0x029a);
	meson_gxl_write_bank(sc, BANK_BIST, FR_PLL_DIV0, 0xaaaa);
}

static int
meson_gxl_phy_probe(device_t dev)
{

	return (mii_phy_dev_probe(dev, meson_gxl_phys, BUS_PROBE_VENDOR));
}

static int
meson_gxl_phy_attach(device_t dev)
{
	struct mii_softc *sc;

	sc = device_get_softc(dev);

	mii_phy_dev_attach(dev, MIIF_NOMANPAUSE, &meson_gxl_phy_funcs, 1);

	meson_gxl_config_init(sc);

	return (0);
}

static void
meson_gxl_phy_reset(struct mii_softc *sc)
{

	mii_phy_reset(sc);
	meson_gxl_config_init(sc);
}

static int
meson_gxl_phy_service(struct mii_softc *sc, struct mii_data *mii, int cmd)
{

	switch (cmd) {
	case MII_POLLSTAT:
		break;

	case MII_MEDIACHG:
		mii_phy_setmedia(sc);
		break;

	case MII_TICK:
		if (mii_phy_tick(sc) == EJUSTRETURN)
			return (0);
		break;
	}

	/* Update the media status. */
	PHY_STATUS(sc);

	/* Callback if something changed. */
	mii_phy_update(sc, cmd);
	return (0);
}

/*
 * Check link status with auto-negotiation workaround.
 *
 * The Meson GXL internal PHY has a known bug: it may report that
 * auto-negotiation completed successfully but provide incorrect
 * link partner ability (LPA) values.  Two failure modes exist:
 *
 *   1) Early failure: LPA = 0x0001 with the link partner claiming
 *      aneg support but never acking our base page.
 *   2) Late failure: LPA looks plausible but is wrong.  A magic bit
 *      (LPI_STATUS bit 12) in the WOL register bank indicates this.
 *
 * In both cases, restarting auto-negotiation resolves the issue.
 */
static void
meson_gxl_phy_status(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	int bmsr, bmcr, anlpar, anar, expansion, wol;

	mii->mii_media_status = IFM_AVALID;
	mii->mii_media_active = IFM_ETHER;

	bmsr = PHY_READ(sc, MII_BMSR) | PHY_READ(sc, MII_BMSR);
	if (bmsr & BMSR_LINK)
		mii->mii_media_status |= IFM_ACTIVE;

	bmcr = PHY_READ(sc, MII_BMCR);
	if (bmcr & BMCR_ISO) {
		mii->mii_media_active |= IFM_NONE;
		mii->mii_media_status = 0;
		return;
	}

	if (bmcr & BMCR_LOOP)
		mii->mii_media_active |= IFM_LOOP;

	if (bmcr & BMCR_AUTOEN) {
		if ((bmsr & BMSR_ACOMP) == 0) {
			mii->mii_media_active |= IFM_NONE;
			return;
		}

		/*
		 * Aneg completed — check for the known LPA corruption.
		 */
		wol = meson_gxl_read_bank(sc, BANK_WOL, LPI_STATUS);
		anlpar = PHY_READ(sc, MII_ANLPAR);
		expansion = PHY_READ(sc, MII_ANER);

		if (!(wol & LPI_STATUS_RSV12) ||
		    ((expansion & ANER_LPAN) &&
		    !(anlpar & ANLPAR_ACK))) {
			/* LPA looks bogus — restart aneg */
			PHY_WRITE(sc, MII_BMCR,
			    BMCR_AUTOEN | BMCR_STARTNEG);
			mii->mii_media_active |= IFM_NONE;
			return;
		}

		anar = PHY_READ(sc, MII_ANAR);
		if ((anlpar & ANLPAR_TX_FD) && (anar & ANAR_TX_FD))
			mii->mii_media_active |= IFM_100_TX | IFM_FDX;
		else if (anlpar & ANLPAR_TX)
			mii->mii_media_active |= IFM_100_TX | IFM_HDX;
		else if ((anlpar & ANLPAR_10_FD) && (anar & ANAR_10_FD))
			mii->mii_media_active |= IFM_10_T | IFM_FDX;
		else if (anlpar & ANLPAR_10)
			mii->mii_media_active |= IFM_10_T | IFM_HDX;
		else
			mii->mii_media_active |= IFM_NONE;

		if ((mii->mii_media_active & IFM_FDX) != 0)
			mii->mii_media_active |=
			    mii_phy_flowstatus(sc);
	} else {
		mii->mii_media_active |=
		    (bmcr & BMCR_FDX) ? IFM_FDX : IFM_HDX;
		if (bmcr & BMCR_S100)
			mii->mii_media_active |= IFM_100_TX;
		else
			mii->mii_media_active |= IFM_10_T;
	}
}
