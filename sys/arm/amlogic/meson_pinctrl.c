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
 * Amlogic Meson pin controller and GPIO driver.
 *
 * Supports GXL (S905W/S905X) with per-SoC data tables.
 * Architecture designed for reuse with G12A, AXG, etc.
 *
 * Each instance (periphs or aobus) maps 3-4 register ranges from DTS:
 *   mux         — pin mux function selection
 *   pull        — pull direction (up/down)
 *   pull-enable — pull enable (periphs only; aobus shares with pull)
 *   gpio        — direction, output, input registers
 *
 * Linux reference: drivers/pinctrl/meson/pinctrl-meson.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/gpio.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/gpio/gpiobusvar.h>

#include "gpio_if.h"

#include <arm/amlogic/meson_pinctrl.h>
#include <arm/amlogic/meson_pinctrl_gxl.h>

/* ---- Compatibility table ---- */

static struct ofw_compat_data compat_data[] = {
	{ "amlogic,meson-gxl-periphs-pinctrl",
	    (uintptr_t)&meson_gxl_periphs_data },
	{ "amlogic,meson-gxl-aobus-pinctrl",
	    (uintptr_t)&meson_gxl_aobus_data },
	{ NULL, 0 }
};

/* ---- Internal helpers ---- */

static const struct meson_gpio_bank *
meson_get_bank(struct meson_pinctrl_softc *sc, unsigned int pin)
{
	const struct meson_pinctrl_data *data = sc->data;
	unsigned int i;

	for (i = 0; i < data->num_banks; i++) {
		if (pin >= data->banks[i].first &&
		    pin <= data->banks[i].last)
			return (&data->banks[i]);
	}
	return (NULL);
}

/*
 * Calculate register byte offset and bit position for a given pin
 * and register type within a bank.
 *
 * Linux reference: meson_calc_reg_and_bit() in pinctrl-meson.c
 */
static void
meson_calc_reg_and_bit(const struct meson_gpio_bank *bank,
    unsigned int pin, enum meson_reg_type type,
    unsigned int *reg, unsigned int *bit)
{
	const struct meson_reg_desc *desc = &bank->regs[type];
	unsigned int offset;

	offset = desc->bit + (pin - bank->first);
	*reg = (desc->reg + offset / 32) * 4;	/* byte offset */
	*bit = offset % 32;
}

static void
meson_reg_set_bit(bus_space_tag_t bst, bus_space_handle_t bsh,
    unsigned int reg, unsigned int bit, bool set)
{
	uint32_t val;

	val = bus_space_read_4(bst, bsh, reg);
	if (set)
		val |= (1u << bit);
	else
		val &= ~(1u << bit);
	bus_space_write_4(bst, bsh, reg, val);
}

static int
meson_pinctrl_find_group(struct meson_pinctrl_softc *sc, const char *name)
{
	unsigned int i;

	for (i = 0; i < sc->data->num_groups; i++) {
		if (strcmp(sc->data->groups[i].name, name) == 0)
			return (i);
	}
	return (-1);
}

/*
 * Clear all mux bits that claim the given pin, except for sel_group.
 * Pass sel_group=-1 to clear all (switch to GPIO mode).
 *
 * Linux reference: meson8_pmx_disable_other_groups()
 */
static void
meson_pinctrl_disable_other_groups(struct meson_pinctrl_softc *sc,
    unsigned int pin, int sel_group)
{
	const struct meson_pmx_group *grp;
	unsigned int i, j;

	for (i = 0; i < sc->data->num_groups; i++) {
		grp = &sc->data->groups[i];
		if (grp->is_gpio || (int)i == sel_group)
			continue;
		for (j = 0; j < grp->num_pins; j++) {
			if (grp->pins[j] == pin) {
				/* Clear this group's mux bit */
				meson_reg_set_bit(sc->mux_bst, sc->mux_bsh,
				    grp->reg * 4, grp->bit, false);
				break;
			}
		}
	}
}

static void
meson_pinctrl_set_bias(struct meson_pinctrl_softc *sc,
    unsigned int pin, int bias)
{
	const struct meson_gpio_bank *bank;
	unsigned int reg, bit;

	bank = meson_get_bank(sc, pin);
	if (bank == NULL)
		return;

	if (bias == 0) {
		/* bias-disable: clear pull-enable */
		meson_calc_reg_and_bit(bank, pin, MESON_REG_PULLEN,
		    &reg, &bit);
		meson_reg_set_bit(sc->pullen_bst, sc->pullen_bsh,
		    reg, bit, false);
	} else {
		/* Set pull direction first, then enable */
		meson_calc_reg_and_bit(bank, pin, MESON_REG_PULL,
		    &reg, &bit);
		meson_reg_set_bit(sc->pull_bst, sc->pull_bsh,
		    reg, bit, bias == 1);	/* 1 = up */
		meson_calc_reg_and_bit(bank, pin, MESON_REG_PULLEN,
		    &reg, &bit);
		meson_reg_set_bit(sc->pullen_bst, sc->pullen_bsh,
		    reg, bit, true);
	}
}

/* ---- GPIO interface ---- */

static device_t
meson_pinctrl_get_bus(device_t dev)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	return (sc->busdev);
}

static int
meson_pinctrl_pin_max(device_t dev, int *maxpin)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	*maxpin = sc->data->num_pins - 1;
	return (0);
}

static int
meson_pinctrl_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct meson_pinctrl_softc *sc;
	const struct meson_gpio_bank *bank;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	bank = meson_get_bank(sc, pin);
	if (bank != NULL)
		snprintf(name, GPIOMAXNAME, "%s_%u", bank->prefix,
		    pin - bank->first);
	else
		snprintf(name, GPIOMAXNAME, "pin%u", pin);
	return (0);
}

static int
meson_pinctrl_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	*caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |
	    GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN;
	return (0);
}

static int
meson_pinctrl_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct meson_pinctrl_softc *sc;
	const struct meson_gpio_bank *bank;
	unsigned int reg, bit;
	uint32_t val;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	bank = meson_get_bank(sc, pin);
	if (bank == NULL)
		return (EINVAL);

	MESON_PINCTRL_LOCK(sc);

	/* Direction: bit=1 means input on Meson */
	meson_calc_reg_and_bit(bank, pin, MESON_REG_DIR, &reg, &bit);
	val = bus_space_read_4(sc->gpio_bst, sc->gpio_bsh, reg);
	*flags = (val & (1u << bit)) ? GPIO_PIN_INPUT : GPIO_PIN_OUTPUT;

	/* Pull */
	meson_calc_reg_and_bit(bank, pin, MESON_REG_PULLEN, &reg, &bit);
	val = bus_space_read_4(sc->pullen_bst, sc->pullen_bsh, reg);
	if (val & (1u << bit)) {
		meson_calc_reg_and_bit(bank, pin, MESON_REG_PULL,
		    &reg, &bit);
		val = bus_space_read_4(sc->pull_bst, sc->pull_bsh, reg);
		*flags |= (val & (1u << bit)) ?
		    GPIO_PIN_PULLUP : GPIO_PIN_PULLDOWN;
	}

	MESON_PINCTRL_UNLOCK(sc);
	return (0);
}

static int
meson_pinctrl_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct meson_pinctrl_softc *sc;
	const struct meson_gpio_bank *bank;
	unsigned int reg, bit;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	bank = meson_get_bank(sc, pin);
	if (bank == NULL)
		return (EINVAL);

	MESON_PINCTRL_LOCK(sc);

	/* Switch pin to GPIO mode (clear all mux bits) */
	meson_pinctrl_disable_other_groups(sc, pin, -1);

	/* Direction */
	if (flags & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)) {
		meson_calc_reg_and_bit(bank, pin, MESON_REG_DIR,
		    &reg, &bit);
		/* Meson: DIR bit=1 → input, bit=0 → output */
		meson_reg_set_bit(sc->gpio_bst, sc->gpio_bsh,
		    reg, bit, (flags & GPIO_PIN_INPUT) != 0);
	}

	/* Pull */
	if (flags & (GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)) {
		int bias = (flags & GPIO_PIN_PULLUP) ? 1 : 2;
		meson_pinctrl_set_bias(sc, pin, bias);
	} else {
		meson_pinctrl_set_bias(sc, pin, 0);
	}

	MESON_PINCTRL_UNLOCK(sc);
	return (0);
}

static int
meson_pinctrl_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct meson_pinctrl_softc *sc;
	const struct meson_gpio_bank *bank;
	unsigned int reg, bit;
	uint32_t regval;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	bank = meson_get_bank(sc, pin);
	if (bank == NULL)
		return (EINVAL);

	MESON_PINCTRL_LOCK(sc);
	meson_calc_reg_and_bit(bank, pin, MESON_REG_IN, &reg, &bit);
	regval = bus_space_read_4(sc->gpio_bst, sc->gpio_bsh, reg);
	*val = (regval & (1u << bit)) ? 1 : 0;
	MESON_PINCTRL_UNLOCK(sc);
	return (0);
}

static int
meson_pinctrl_pin_set(device_t dev, uint32_t pin, unsigned int val)
{
	struct meson_pinctrl_softc *sc;
	const struct meson_gpio_bank *bank;
	unsigned int reg, bit;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	bank = meson_get_bank(sc, pin);
	if (bank == NULL)
		return (EINVAL);

	MESON_PINCTRL_LOCK(sc);
	meson_calc_reg_and_bit(bank, pin, MESON_REG_OUT, &reg, &bit);
	meson_reg_set_bit(sc->gpio_bst, sc->gpio_bsh, reg, bit, val != 0);
	MESON_PINCTRL_UNLOCK(sc);
	return (0);
}

static int
meson_pinctrl_pin_toggle(device_t dev, uint32_t pin)
{
	struct meson_pinctrl_softc *sc;
	const struct meson_gpio_bank *bank;
	unsigned int reg, bit;
	uint32_t val;

	sc = device_get_softc(dev);
	if (pin >= sc->data->num_pins)
		return (EINVAL);

	bank = meson_get_bank(sc, pin);
	if (bank == NULL)
		return (EINVAL);

	MESON_PINCTRL_LOCK(sc);
	meson_calc_reg_and_bit(bank, pin, MESON_REG_OUT, &reg, &bit);
	val = bus_space_read_4(sc->gpio_bst, sc->gpio_bsh, reg);
	val ^= (1u << bit);
	bus_space_write_4(sc->gpio_bst, sc->gpio_bsh, reg, val);
	MESON_PINCTRL_UNLOCK(sc);
	return (0);
}

static int
meson_pinctrl_map_gpios(device_t dev, phandle_t pnode, phandle_t gnode,
    int gcells, pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{

	if (gcells != 2)
		return (EINVAL);

	*pin = gpios[0];
	*flags = gpios[1];
	return (0);
}

/* ---- FDT pinctrl configure ---- */

/*
 * Configure a single DTS pin config node (or mux sub-node).
 * Parses "groups", "function", and bias properties, then
 * applies mux settings and bias to all pins in each group.
 */
static void
meson_pinctrl_configure_node(struct meson_pinctrl_softc *sc, phandle_t node)
{
	char *groups, *function;
	const char *p;
	int glen, flen, gi, bias;
	unsigned int j;
	const struct meson_pmx_group *grp;
	bool is_gpio_func;

	/* Parse bias */
	if (OF_hasprop(node, "bias-pull-up"))
		bias = 1;
	else if (OF_hasprop(node, "bias-pull-down"))
		bias = 2;
	else if (OF_hasprop(node, "bias-disable"))
		bias = 0;
	else
		bias = -1;

	flen = OF_getprop_alloc(node, "function", (void **)&function);
	glen = OF_getprop_alloc(node, "groups", (void **)&groups);
	if (glen <= 0) {
		if (flen > 0)
			OF_prop_free(function);
		return;
	}

	is_gpio_func = (flen > 0 &&
	    (strcmp(function, "gpio_periphs") == 0 ||
	     strcmp(function, "gpio_aobus") == 0));

	/* Walk NUL-separated groups string list */
	for (p = groups; p < groups + glen; p += strlen(p) + 1) {
		gi = meson_pinctrl_find_group(sc, p);
		if (gi < 0)
			continue;

		grp = &sc->data->groups[gi];

		MESON_PINCTRL_LOCK(sc);

		/* Clear conflicting mux bits for each pin in the group */
		for (j = 0; j < grp->num_pins; j++)
			meson_pinctrl_disable_other_groups(sc,
			    grp->pins[j], is_gpio_func ? -1 : gi);

		/* Set the mux bit (unless switching to GPIO mode) */
		if (!is_gpio_func && !grp->is_gpio && flen > 0) {
			meson_reg_set_bit(sc->mux_bst, sc->mux_bsh,
			    grp->reg * 4, grp->bit, true);
		}

		/* Apply bias to all pins in the group */
		if (bias >= 0) {
			for (j = 0; j < grp->num_pins; j++)
				meson_pinctrl_set_bias(sc, grp->pins[j],
				    bias);
		}

		MESON_PINCTRL_UNLOCK(sc);
	}

	OF_prop_free(groups);
	if (flen > 0)
		OF_prop_free(function);
}

static int
meson_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct meson_pinctrl_softc *sc;
	phandle_t node, child;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);

	/* Process the node itself */
	meson_pinctrl_configure_node(sc, node);

	/* Process children (mux-0, mux-1, etc.) */
	for (child = OF_child(node); child > 0; child = OF_peer(child))
		meson_pinctrl_configure_node(sc, child);

	return (0);
}

/* ---- Device interface ---- */

static int
meson_pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson Pin Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_pinctrl_attach(device_t dev)
{
	struct meson_pinctrl_softc *sc;
	phandle_t node, child, bank_node;
	bus_size_t sz;
	int idx;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	sc->data = (const struct meson_pinctrl_data *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	mtx_init(&sc->mtx, device_get_nameunit(dev), "meson_pinctrl",
	    MTX_DEF);

	/*
	 * Find the bank child node (gpio-controller).
	 * The DTS has register ranges on this child, not the parent.
	 */
	bank_node = 0;
	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		if (OF_hasprop(child, "gpio-controller")) {
			bank_node = child;
			break;
		}
	}
	if (bank_node == 0) {
		device_printf(dev, "no gpio-controller child found\n");
		goto fail;
	}

	/*
	 * Map named register ranges from the bank node.
	 * Periphs: "mux", "pull", "pull-enable", "gpio" (4 ranges)
	 * Aobus:   "mux", "pull", "gpio" (3 ranges, no pull-enable)
	 */
	if (ofw_bus_find_string_index(bank_node, "reg-names",
	    "mux", &idx) == 0)
		OF_decode_addr(bank_node, idx, &sc->mux_bst,
		    &sc->mux_bsh, &sz);
	else {
		device_printf(dev, "cannot find \"mux\" reg range\n");
		goto fail;
	}

	if (ofw_bus_find_string_index(bank_node, "reg-names",
	    "pull", &idx) == 0)
		OF_decode_addr(bank_node, idx, &sc->pull_bst,
		    &sc->pull_bsh, &sz);
	else {
		device_printf(dev, "cannot find \"pull\" reg range\n");
		goto fail;
	}

	if (ofw_bus_find_string_index(bank_node, "reg-names",
	    "pull-enable", &idx) == 0) {
		OF_decode_addr(bank_node, idx, &sc->pullen_bst,
		    &sc->pullen_bsh, &sz);
	} else if (sc->data->aobus) {
		/* Aobus: pull-enable shares the pull regmap */
		sc->pullen_bst = sc->pull_bst;
		sc->pullen_bsh = sc->pull_bsh;
	} else {
		device_printf(dev,
		    "cannot find \"pull-enable\" reg range\n");
		goto fail;
	}

	if (ofw_bus_find_string_index(bank_node, "reg-names",
	    "gpio", &idx) == 0)
		OF_decode_addr(bank_node, idx, &sc->gpio_bst,
		    &sc->gpio_bsh, &sz);
	else {
		device_printf(dev, "cannot find \"gpio\" reg range\n");
		goto fail;
	}

	device_printf(dev, "%s: %u pins, %u groups, %u functions, "
	    "%u banks\n", sc->data->name, sc->data->num_pins,
	    sc->data->num_groups, sc->data->num_funcs,
	    sc->data->num_banks);

	/* Register as pinctrl provider and apply DTS pin configs */
	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	/*
	 * Register as GPIO controller.
	 * Use the bank child node's xref so DTS gpio references
	 * like <&gpio GPIOX_5 0> resolve to us.
	 */
	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "cannot create gpiobus\n");
		goto fail;
	}

	return (0);

fail:
	mtx_destroy(&sc->mtx);
	return (ENXIO);
}

static int
meson_pinctrl_detach(device_t dev)
{
	struct meson_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	if (sc->busdev != NULL)
		gpiobus_detach_bus(dev);
	mtx_destroy(&sc->mtx);
	return (0);
}

/* ---- Method table ---- */

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

	/* FDT pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, meson_pinctrl_configure),

	DEVMETHOD_END
};

DEFINE_CLASS_0(gpio, meson_pinctrl_driver, meson_pinctrl_methods,
    sizeof(struct meson_pinctrl_softc));

EARLY_DRIVER_MODULE(meson_pinctrl, simplebus, meson_pinctrl_driver, NULL,
    NULL, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_VERSION(meson_pinctrl, 1);
