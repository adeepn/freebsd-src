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
 * Amlogic Meson GXBB/GXL reset controller.
 *
 * The Meson reset controller has two sets of registers:
 *   - Pulse registers at offset 0x00: writing 1 to a bit triggers an
 *     instantaneous reset pulse on the corresponding line.
 *   - Level registers at offset 0x7C (for GXBB/GXL): setting a bit
 *     holds the corresponding module in reset; clearing releases it.
 *
 * This driver uses the level registers for assert/deassert operations,
 * matching the FreeBSD hwreset_if interface semantics.
 *
 * The reset IDs correspond to bit positions across 8 register banks
 * (32 bits each, IDs 0-255).  ID mapping:
 *   Register offset = level_offset + (id / 32) * 4
 *   Bit position    = id % 32
 *
 * Polarity (active-low, level_low_reset): bit=0 means reset asserted,
 * bit=1 means deasserted (module running).  This matches the Linux
 * driver's meson8b_param.level_low_reset = true.
 *
 * Linux reference: drivers/reset/amlogic/reset-meson.c
 * FreeBSD template: sys/arm/allwinner/aw_reset.c
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

#include <dev/hwreset/hwreset.h>

#include "hwreset_if.h"

/* Level register offset from the register base for GXBB/GXL/AXG */
#define	MESON_RST_LEVEL_OFFSET	0x7C

/* Maximum number of register banks */
#define	MESON_RST_REG_COUNT	8

/* Maximum valid reset ID: 8 banks * 32 bits = 256 IDs (0-255) */
#define	MESON_RST_MAX_ID	(MESON_RST_REG_COUNT * 32)

#define	RESET_REG(id)		(MESON_RST_LEVEL_OFFSET + ((id) / 32) * 4)
#define	RESET_BIT(id)		((id) % 32)

struct meson_reset_softc {
	struct resource		*res;
	struct mtx		mtx;
};

static struct resource_spec meson_reset_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

#define	RST_READ(sc, reg)	bus_read_4((sc)->res, (reg))
#define	RST_WRITE(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static int
meson_reset_assert(device_t dev, intptr_t id, bool reset)
{
	struct meson_reset_softc *sc;
	uint32_t reg_value;

	if (id < 0 || id >= MESON_RST_MAX_ID)
		return (EINVAL);

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	reg_value = RST_READ(sc, RESET_REG(id));
	if (reset)
		reg_value &= ~(1u << RESET_BIT(id));	/* Assert: clear bit */
	else
		reg_value |= (1u << RESET_BIT(id));	/* Deassert: set bit */
	RST_WRITE(sc, RESET_REG(id), reg_value);
	mtx_unlock(&sc->mtx);

	return (0);
}

static int
meson_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct meson_reset_softc *sc;
	uint32_t reg_value;

	if (id < 0 || id >= MESON_RST_MAX_ID)
		return (EINVAL);

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	reg_value = RST_READ(sc, RESET_REG(id));
	mtx_unlock(&sc->mtx);

	/* Active-low: bit clear means reset is asserted */
	*reset = (reg_value & (1u << RESET_BIT(id))) == 0;

	return (0);
}

static int
meson_reset_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "amlogic,meson-gxbb-reset"))
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson Reset Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_reset_attach(device_t dev)
{
	struct meson_reset_softc *sc;

	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, meson_reset_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	hwreset_register_ofw_provider(dev);

	return (0);
}

static device_method_t meson_reset_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		meson_reset_probe),
	DEVMETHOD(device_attach,	meson_reset_attach),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	meson_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	meson_reset_is_asserted),

	DEVMETHOD_END
};

static driver_t meson_reset_driver = {
	"meson_reset",
	meson_reset_methods,
	sizeof(struct meson_reset_softc),
};

/*
 * The reset controller node sits inside cbus (simple-bus).
 * It must attach early so that other drivers can deassert resets
 * during their probe/attach.
 */
EARLY_DRIVER_MODULE(meson_reset, simplebus, meson_reset_driver, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(meson_reset, 1);
