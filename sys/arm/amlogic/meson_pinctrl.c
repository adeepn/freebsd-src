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
 * Amlogic Meson GXL pinctrl/GPIO stub driver.
 *
 * TODO: STUB -- relies on U-Boot initialisation
 *
 * Current behaviour:
 *   This driver registers as both a pinctrl provider and a GPIO
 *   controller but does NOT program any mux, pull, or GPIO registers.
 *   It relies entirely on U-Boot having already configured the pin
 *   muxing for UART_AO, SD/eMMC, and any other active peripherals.
 *
 *   GPIO read/write operations are stubs that return zero / succeed
 *   silently.  The pinctrl configure callback is a no-op.
 *
 * For a full implementation:
 *   - Parse pin group and function tables from meson-gxl pinctrl data
 *     (see Linux pinctrl-meson-gxl.c for the complete table).
 *   - Program mux registers to select pin functions.
 *   - Program pull/pull-enable registers for bias configuration.
 *   - Read/write GPIO data registers for actual pin I/O.
 *   - Support interrupt controller (PIC) for GPIO interrupts.
 *
 * Linux reference: drivers/pinctrl/meson/pinctrl-meson.c
 *                  drivers/pinctrl/meson/pinctrl-meson-gxl.c
 * FreeBSD template: sys/arm/allwinner/aw_gpio.c
 *
 * DTS structure (Meson GXL):
 *   The pinctrl node (e.g., pinctrl@4b0) is a child of a simple-bus
 *   and contains:
 *     - A "bank" child node with gpio-controller, #gpio-cells, reg,
 *       and gpio-ranges properties.
 *     - Multiple pin configuration children (e.g., emmc_pins, uart_a)
 *       each containing "mux" sub-nodes with groups/function properties.
 *
 *   This driver attaches to the pinctrl node and finds the bank child
 *   internally, registering itself as the GPIO provider for the bank
 *   node's phandle.
 *
 * Two instances:
 *   - periphs pinctrl: "amlogic,meson-gxl-periphs-pinctrl" (100 pins)
 *   - aobus pinctrl:   "amlogic,meson-gxl-aobus-pinctrl"   (14 pins)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include "gpio_if.h"
#include "fdt_pinctrl_if.h"

#define	MESON_GXL_PERIPHS_NPINS	100
#define	MESON_GXL_AOBUS_NPINS	14

struct meson_pinctrl_softc {
	device_t	sc_dev;
	device_t	sc_busdev;	/* GPIO bus child */
	struct mtx	sc_mtx;
	int		sc_npins;
};

static struct ofw_compat_data compat_data[] = {
	{ "amlogic,meson-gxl-periphs-pinctrl",	MESON_GXL_PERIPHS_NPINS },
	{ "amlogic,meson-gxl-aobus-pinctrl",	MESON_GXL_AOBUS_NPINS },
	{ NULL,					0 }
};

#define	MESON_PINCTRL_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	MESON_PINCTRL_UNLOCK(_sc)	mtx_unlock(&(_sc)->sc_mtx)

/* ---------- GPIO interface (stubs) ---------- */

static device_t
meson_pinctrl_get_bus(device_t dev)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_busdev);
}

static int
meson_pinctrl_pin_max(device_t dev, int *maxpin)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	*maxpin = sc->sc_npins - 1;
	return (0);
}

static int
meson_pinctrl_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	snprintf(name, GPIOMAXNAME, "pin%d", pin);
	return (0);
}

static int
meson_pinctrl_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	*caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
	    GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN;
	return (0);
}

static int
meson_pinctrl_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	/*
	 * TODO: STUB -- relies on U-Boot initialisation
	 *
	 * Current behaviour:
	 *   Returns 0 (no flags known).
	 *
	 * For a full implementation:
	 *   - Read the mux registers to determine the current function.
	 *   - Read the GPIO direction register to determine input/output.
	 *   - Read pull/pull-enable registers for bias flags.
	 */
	*flags = 0;
	return (0);
}

static int
meson_pinctrl_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	/*
	 * TODO: STUB -- relies on U-Boot initialisation
	 *
	 * Current behaviour:
	 *   Silently ignores flag changes.
	 *
	 * For a full implementation:
	 *   - Switch pin mux to GPIO function if not already.
	 *   - Program direction register for input/output.
	 *   - Program pull/pull-enable registers for bias.
	 */
	return (0);
}

static int
meson_pinctrl_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	/*
	 * TODO: STUB -- relies on U-Boot initialisation
	 *
	 * Current behaviour:
	 *   Always returns 0.
	 *
	 * For a full implementation:
	 *   - Read the GPIO input register for the appropriate bank.
	 *   - Return the bit corresponding to this pin.
	 */
	*val = 0;
	return (0);
}

static int
meson_pinctrl_pin_set(device_t dev, uint32_t pin, unsigned int val)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	/*
	 * TODO: STUB -- relies on U-Boot initialisation
	 *
	 * Current behaviour:
	 *   Silently ignores writes.
	 *
	 * For a full implementation:
	 *   - Write the GPIO output register for the appropriate bank.
	 */
	return (0);
}

static int
meson_pinctrl_pin_toggle(device_t dev, uint32_t pin)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->sc_npins)
		return (EINVAL);

	return (0);
}

static int
meson_pinctrl_map_gpios(device_t bus, phandle_t dev, phandle_t gparent,
    int gcells, pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{

	if (gcells != 2)
		return (EINVAL);
	*pin = gpios[0];
	*flags = gpios[1];
	return (0);
}

/* ---------- FDT pinctrl interface ---------- */

static int
meson_pinctrl_configure(device_t dev, phandle_t cfgxref)
{

	/*
	 * TODO: STUB -- relies on U-Boot initialisation
	 *
	 * Current behaviour:
	 *   No-op.  U-Boot has already configured pin muxing for
	 *   all active peripherals (UART_AO, SD/eMMC, I2C, etc.).
	 *
	 * For a full implementation:
	 *   - Resolve cfgxref to an FDT node.
	 *   - Iterate child nodes looking for "groups" and "function"
	 *     properties.
	 *   - Look up the group in the SoC pin table to find the
	 *     mux register offset and bit field.
	 *   - Program the mux register to select the requested function.
	 *   - Apply bias settings (bias-pull-up, bias-pull-down,
	 *     bias-disable) from DTS properties.
	 *
	 * Linux reference:
	 *   drivers/pinctrl/meson/pinctrl-meson.c: meson_pinctrl_set_mux()
	 *   drivers/pinctrl/meson/pinctrl-meson-gxl.c: GXL group/function
	 *   tables (~800 entries)
	 */
	return (0);
}

/* ---------- Device interface ---------- */

static int
meson_pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson GXL Pinctrl/GPIO");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_pinctrl_attach(device_t dev)
{
	struct meson_pinctrl_softc *sc;
	phandle_t node, child;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_npins = (int)ofw_bus_search_compatible(dev,
	    compat_data)->ocd_data;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "meson_gpio",
	    MTX_DEF);

	node = ofw_bus_get_node(dev);

	/*
	 * Register as a pinctrl provider.  Passing NULL as pinprop causes
	 * all descendant nodes to be registered via OF_device_register_xref,
	 * which covers both pin configuration nodes (e.g. emmc_pins) and
	 * the GPIO bank child node.  This allows:
	 *   - pinctrl consumers (pinctrl-0 = <&emmc_pins>) to find us
	 *   - GPIO consumers (gpios = <&gpio 42 0>) to find us via the
	 *     bank child's phandle
	 */
	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	/*
	 * Additionally register xref for any child with gpio-controller
	 * property, so that GPIO framework can resolve the bank phandle
	 * to this device even if fdt_pinctrl_register missed it.
	 */
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (OF_hasprop(child, "gpio-controller")) {
			OF_device_register_xref(
			    OF_xref_from_node(child), dev);
			break;
		}
	}

	/* Create the GPIO bus child. */
	sc->sc_busdev = gpiobus_add_bus(dev);
	if (sc->sc_busdev == NULL) {
		device_printf(dev, "cannot create gpiobus\n");
		mtx_destroy(&sc->sc_mtx);
		return (ENXIO);
	}

	bus_attach_children(dev);
	return (0);
}

static int
meson_pinctrl_detach(device_t dev)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (sc->sc_busdev != NULL)
		gpiobus_detach_bus(dev);
	mtx_destroy(&sc->sc_mtx);
	return (0);
}

static device_method_t meson_pinctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		meson_pinctrl_probe),
	DEVMETHOD(device_attach,	meson_pinctrl_attach),
	DEVMETHOD(device_detach,	meson_pinctrl_detach),

	/* GPIO interface */
	DEVMETHOD(gpio_get_bus,		meson_pinctrl_get_bus),
	DEVMETHOD(gpio_pin_max,		meson_pinctrl_pin_max),
	DEVMETHOD(gpio_pin_getname,	meson_pinctrl_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	meson_pinctrl_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	meson_pinctrl_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	meson_pinctrl_pin_setflags),
	DEVMETHOD(gpio_pin_get,		meson_pinctrl_pin_get),
	DEVMETHOD(gpio_pin_set,		meson_pinctrl_pin_set),
	DEVMETHOD(gpio_pin_toggle,	meson_pinctrl_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	meson_pinctrl_map_gpios),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),

	/* FDT pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, meson_pinctrl_configure),

	DEVMETHOD_END
};

static driver_t meson_pinctrl_driver = {
	"gpio",
	meson_pinctrl_methods,
	sizeof(struct meson_pinctrl_softc),
};

/*
 * Attach after clocks and resets (BUS_PASS_RESOURCE) but early enough
 * for MMC and other peripheral drivers to find us.
 */
EARLY_DRIVER_MODULE(meson_pinctrl, simplebus, meson_pinctrl_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_VERSION(meson_pinctrl, 1);
MODULE_DEPEND(meson_pinctrl, gpiobus, 1, 1, 1);
