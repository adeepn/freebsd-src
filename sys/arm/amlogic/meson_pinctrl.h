/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 JetHome. All rights reserved.
 */

#ifndef _MESON_PINCTRL_H_
#define _MESON_PINCTRL_H_

#include <sys/types.h>

/*
 * Amlogic Meson pin controller — common data structures.
 *
 * Designed for reuse across SoC generations (GXL, G12A, AXG, etc.)
 * by keeping all SoC-specific data in separate headers included
 * from the driver source.
 *
 * Linux reference: drivers/pinctrl/meson/pinctrl-meson.h
 */

/* Register types for meson_calc_reg_and_bit() */
enum meson_reg_type {
	MESON_REG_PULLEN = 0,	/* Pull enable (1 = enabled) */
	MESON_REG_PULL,		/* Pull direction (1 = up, 0 = down) */
	MESON_REG_DIR,		/* Direction (1 = input, 0 = output) */
	MESON_REG_OUT,		/* Output value */
	MESON_REG_IN,		/* Input value (read-only) */
	MESON_NUM_REG
};

/* Per-register descriptor: word index + bit offset of first pin */
struct meson_reg_desc {
	unsigned int	reg;	/* Word index into regmap */
	unsigned int	bit;	/* Bit offset of first pin in this bank */
};

/* GPIO bank descriptor */
struct meson_gpio_bank {
	const char		*name;		/* "X", "DV", "BOOT", "AO" */
	const char		*prefix;	/* Pin name prefix: "GPIOX", "BOOT" */
	unsigned int		first;		/* First pin in flat numbering */
	unsigned int		last;		/* Last pin in flat numbering */
	struct meson_reg_desc	regs[MESON_NUM_REG];
};

/* Pin mux group descriptor */
struct meson_pmx_group {
	const char		*name;		/* "emmc_clk", "uart_tx_a" */
	const unsigned int	*pins;		/* Array of pin numbers */
	unsigned int		num_pins;
	bool			is_gpio;	/* GPIO_GROUP (no mux bit) */
	unsigned int		reg;		/* Mux register word index */
	unsigned int		bit;		/* Mux bit within that word */
};

/* Pin mux function descriptor */
struct meson_pmx_func {
	const char		*name;		/* "emmc", "i2c_b" */
	const char * const	*groups;	/* Group name strings */
	unsigned int		num_groups;
};

/* Top-level SoC-specific pinctrl descriptor */
struct meson_pinctrl_data {
	const char			*name;
	const struct meson_gpio_bank	*banks;
	unsigned int			num_banks;
	const struct meson_pmx_group	*groups;
	unsigned int			num_groups;
	const struct meson_pmx_func	*funcs;
	unsigned int			num_funcs;
	unsigned int			num_pins;
	bool				aobus;	/* pullen = pull for AO */
};

/* Driver softc */
struct meson_pinctrl_softc {
	device_t		dev;
	device_t		busdev;		/* gpiobus child */
	struct mtx		mtx;

	bus_space_tag_t		mux_bst;	/* Mux registers */
	bus_space_handle_t	mux_bsh;
	bus_space_tag_t		pull_bst;	/* Pull direction */
	bus_space_handle_t	pull_bsh;
	bus_space_tag_t		pullen_bst;	/* Pull enable */
	bus_space_handle_t	pullen_bsh;
	bus_space_tag_t		gpio_bst;	/* DIR + OUT + IN */
	bus_space_handle_t	gpio_bsh;

	const struct meson_pinctrl_data	*data;
};

/* ---- Helper macros for declaring SoC data tables ---- */

/*
 * GPIO group — one pin, no mux bit (GPIO mode = all mux bits cleared).
 * Usage: MESON_GPIO_GROUP(GPIOX_0)
 * Expands name from the pin define symbol.
 */
#define	MESON_GPIO_GROUP(gpio)						\
	{ #gpio, (const unsigned int []){ gpio }, 1, true, 0, 0 }

/*
 * Mux group — one or more pins with a mux register/bit.
 * Requires a preceding `static const unsigned int grp_pins[] = { ... };`
 * Usage: MESON_GROUP(emmc_clk, 7, 30)
 */
#define	MESON_GROUP(grp, r, b)						\
	{ #grp, grp ## _pins, nitems(grp ## _pins), false, (r), (b) }

/*
 * Function — references an array of group name strings.
 * Requires a preceding `static const char * const fn_groups[] = { ... };`
 * Usage: MESON_FUNCTION(emmc)
 */
#define	MESON_FUNCTION(fn)						\
	{ #fn, fn ## _groups, nitems(fn ## _groups) }

/*
 * Bank descriptor.
 * Arguments: name, prefix, first_pin, last_pin,
 *   pullen_reg, pullen_bit, pull_reg, pull_bit,
 *   dir_reg, dir_bit, out_reg, out_bit, in_reg, in_bit
 */
#define	MESON_BANK(n, pfx, f, l,					\
    per, peb, pr, pb, dr, db, or_, ob, ir, ib)				\
	{								\
		.name = (n), .prefix = (pfx),				\
		.first = (f), .last = (l),				\
		.regs = {						\
			[MESON_REG_PULLEN] = { (per), (peb) },		\
			[MESON_REG_PULL]   = { (pr),  (pb)  },		\
			[MESON_REG_DIR]    = { (dr),  (db)  },		\
			[MESON_REG_OUT]    = { (or_), (ob)  },		\
			[MESON_REG_IN]     = { (ir),  (ib)  },		\
		}							\
	}

#define	MESON_PINCTRL_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	MESON_PINCTRL_UNLOCK(sc)	mtx_unlock(&(sc)->mtx)

#endif /* _MESON_PINCTRL_H_ */
