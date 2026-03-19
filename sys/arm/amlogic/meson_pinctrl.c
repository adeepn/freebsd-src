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
 * Amlogic Meson GXL pinctrl/GPIO driver.
 *
 * Programs pin mux registers to select peripheral functions for pins.
 * GPIO read/write operations are still stubs that rely on U-Boot.
 *
 * The mux register table covers groups used by the JetHub J80 DTS.
 * Unknown groups are silently ignored (assumed U-Boot-configured).
 *
 * TODO: Pull/pull-enable and GPIO register access for full GPIO support.
 *
 * Linux reference: drivers/pinctrl/meson/pinctrl-meson.c
 *                  drivers/pinctrl/meson/pinctrl-meson-gxl.c
 *
 * DTS structure (Meson GXL):
 *   The pinctrl node (e.g., pinctrl@4b0) is a child of a simple-bus
 *   and contains:
 *     - A "bank" child node with gpio-controller, #gpio-cells, reg,
 *       and gpio-ranges properties.  The bank's first reg range is
 *       the mux registers (reg-names = "mux").
 *     - Multiple pin configuration children (e.g., emmc_pins, i2c_b)
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
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include "gpio_if.h"
#include "fdt_pinctrl_if.h"

#define	MESON_GXL_PERIPHS_NPINS	100
#define	MESON_GXL_AOBUS_NPINS	14

struct meson_pinctrl_softc {
	device_t		sc_dev;
	device_t		sc_busdev;	/* GPIO bus child */
	struct mtx		sc_mtx;
	int			sc_npins;
	bus_space_tag_t		sc_mux_bst;	/* Mux register mapping */
	bus_space_handle_t	sc_mux_bsh;
	bus_size_t		sc_mux_size;
	bool			sc_has_mux;
};

/*
 * Pin mux group table for GXL periphs.
 * Each entry maps a DTS group name to a mux register index and bit.
 * Register offset = reg * 4 from mux base.  Setting the bit selects
 * the peripheral function; clearing it reverts to GPIO.
 *
 * Values from Linux pinctrl-meson-gxl.c GROUP() macros.
 */
struct meson_mux_entry {
	const char	*group;
	uint8_t		reg;
	uint8_t		bit;
};

static const struct meson_mux_entry meson_gxl_periphs_mux[] = {
	/* I2C bus B (GPIODV_26/27) */
	{ "i2c_sda_b",		1,	13 },
	{ "i2c_sck_b",		1,	12 },
	/* I2C bus A (GPIODV_24/25) */
	{ "i2c_sda_a",		1,	15 },
	{ "i2c_sck_a",		1,	14 },
	/* I2C bus C (GPIODV_28/29) */
	{ "i2c_sda_c",		1,	11 },
	{ "i2c_sck_c",		1,	10 },
	/* eMMC (BOOT pins) */
	{ "emmc_nand_d07",	7,	31 },
	{ "emmc_clk",		7,	30 },
	{ "emmc_cmd",		7,	29 },
	{ "emmc_ds",		7,	28 },
	/* Ethernet LEDs */
	{ "eth_link_led",	4,	25 },
	{ "eth_act_led",	4,	24 },
	{ NULL, 0, 0 },
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

/*
 * Look up a group name in the mux table and program the mux register.
 */
static void
meson_pinctrl_set_group(struct meson_pinctrl_softc *sc, const char *name)
{
	const struct meson_mux_entry *e;
	uint32_t val;

	for (e = meson_gxl_periphs_mux; e->group != NULL; e++) {
		if (strcmp(name, e->group) == 0) {
			MESON_PINCTRL_LOCK(sc);
			val = bus_space_read_4(sc->sc_mux_bst,
			    sc->sc_mux_bsh, e->reg * 4);
			val |= (1u << e->bit);
			bus_space_write_4(sc->sc_mux_bst,
			    sc->sc_mux_bsh, e->reg * 4, val);
			MESON_PINCTRL_UNLOCK(sc);
			device_printf(sc->sc_dev,
			    "mux: %s -> reg%d bit%d\n",
			    name, e->reg, e->bit);
			return;
		}
	}
	/* Unknown group — silently ignore (U-Boot-configured). */
}

/*
 * Parse a pin configuration node's children for "groups" properties
 * and program the corresponding mux registers.
 *
 * DTS structure:
 *   i2c_b_pins: i2c_b {
 *       mux {
 *           groups = "i2c_sck_b", "i2c_sda_b";
 *           function = "i2c_b";
 *       };
 *   };
 */
static int
meson_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct meson_pinctrl_softc *sc;
	phandle_t node, child;
	char *groups;
	char *p;
	int len;

	sc = device_get_softc(dev);

	if (!sc->sc_has_mux)
		return (0);

	node = OF_node_from_xref(cfgxref);
	if (node <= 0)
		return (0);

	/*
	 * Iterate children (e.g. "mux" sub-nodes) looking for "groups".
	 */
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		len = OF_getprop_alloc(child, "groups", (void **)&groups);
		if (len <= 0)
			continue;

		/* Walk the null-terminated string list. */
		p = groups;
		while (p < groups + len) {
			meson_pinctrl_set_group(sc, p);
			p += strlen(p) + 1;
		}

		OF_prop_free(groups);
	}

	/*
	 * Also check the node itself — some DTS nodes put groups
	 * directly on the pin config node without a sub-node.
	 */
	len = OF_getprop_alloc(node, "groups", (void **)&groups);
	if (len > 0) {
		p = groups;
		while (p < groups + len) {
			meson_pinctrl_set_group(sc, p);
			p += strlen(p) + 1;
		}
		OF_prop_free(groups);
	}

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
	phandle_t node, child, bank;
	int err;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_npins = (int)ofw_bus_search_compatible(dev,
	    compat_data)->ocd_data;
	sc->sc_has_mux = false;

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "meson_gpio",
	    MTX_DEF);

	node = ofw_bus_get_node(dev);

	/*
	 * Find the bank child (gpio-controller) and map its mux registers.
	 * The bank's reg property has 4 ranges: mux, pull, pull-enable, gpio.
	 * We map the first range (index 0 = mux registers).
	 */
	bank = 0;
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (OF_hasprop(child, "gpio-controller")) {
			bank = child;
			break;
		}
	}

	if (bank != 0) {
		err = OF_decode_addr(bank, 0, &sc->sc_mux_bst,
		    &sc->sc_mux_bsh, &sc->sc_mux_size);
		if (err == 0) {
			sc->sc_has_mux = true;
			device_printf(dev, "mux registers mapped "
			    "(%lu bytes)\n",
			    (unsigned long)sc->sc_mux_size);
		} else {
			device_printf(dev, "WARNING: cannot map mux "
			    "registers (err=%d), pin mux disabled\n",
			    err);
		}
	}

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
	if (bank != 0)
		OF_device_register_xref(OF_xref_from_node(bank), dev);

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
