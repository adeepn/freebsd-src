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
 * Amlogic Meson clock controller — shared definitions.
 *
 * This header is used by the core framework (meson_clk.c) and
 * SoC-specific clock data files (meson_clk_gxbb.c, etc.).
 */

#ifndef _DEV_CLK_MESON_CLK_H_
#define _DEV_CLK_MESON_CLK_H_

#include <dev/clk/clk.h>
#include <dev/clk/clk_fixed.h>

struct meson_clk_softc {
	device_t		dev;
	struct resource		*res;	/* May be NULL for stub drivers */
	struct clkdom		*clkdom;
	struct mtx		mtx;
};

DECLARE_CLASS(meson_clkc_driver);

int meson_clk_attach(device_t dev, struct clk_fixed_def *clks, int nclks);

/*
 * Helper macros for defining fixed-rate stub clocks.
 *
 * MESON_FIXED_RATE: Absolute frequency, no parent.
 *   Used for clocks whose frequency is hardcoded based on
 *   the typical U-Boot configuration.
 *
 * MESON_FIXED_FACTOR: Frequency derived from parent via mult/div.
 *   Used when the relationship to a parent clock should be preserved.
 */
#define	MESON_FIXED_RATE(_id, _name, _freq)		\
	{						\
		.clkdef = {				\
			.id = (_id),			\
			.name = (_name),		\
			.parent_names = NULL,		\
			.parent_cnt = 0,		\
		},					\
		.freq = (_freq),			\
	}

#define	MESON_FIXED_FACTOR(_id, _name, _pnames, _mult, _div)	\
	{							\
		.clkdef = {					\
			.id = (_id),				\
			.name = (_name),			\
			.parent_names = (_pnames),		\
			.parent_cnt = 1,			\
		},						\
		.mult = (_mult),				\
		.div = (_div),					\
	}

#endif /* _DEV_CLK_MESON_CLK_H_ */
