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
 * Amlogic Meson GXBB/GXL clock controllers — stub implementation.
 *
 * This file provides two drivers:
 *   1. EE clock controller (amlogic,gxl-clkc) — main system clocks
 *   2. AO clock controller (amlogic,meson-gx{l,}-aoclkc) — always-on domain
 *
 * Both register only fixed-rate clocks, relying on U-Boot to have
 * already configured PLLs, dividers, and gates to their standard values.
 *
 * TODO: STUB — relies on U-Boot initialization
 *
 * Current behavior:
 *   All clocks are registered as fixed-rate at their standard GXL
 *   frequencies.  No hardware registers are read or written.
 *   Clock gating (enable/disable) is not implemented — all clocks
 *   are assumed to be always enabled by U-Boot.
 *
 * For full implementation:
 *   - EE clock controller (amlogic,gxl-clkc):
 *     - Obtain register access via syscon from the parent HHI sysctrl
 *     - Implement PLL clocks: sys_pll, fixed_pll, gp0_pll, hdmi_pll
 *       using HHI_SYS_PLL_CNTL, HHI_FIXED_PLL_CNTL, etc.
 *     - Implement fixed dividers: fclk_div2..7 from fixed_pll
 *     - Implement MPEG clock selector + divider (clk81) from
 *       HHI_MPEG_CLK_CNTL register
 *     - Implement clock gates via HHI_GCLK_MPEG0/1/2 registers
 *     - Implement SD/eMMC mux+divider clocks for dynamic frequency
 *     - Add parent-child relationships for proper clock tree
 *
 *   - AO clock controller (amlogic,meson-gx-aoclkc):
 *     - Obtain register access via syscon from the parent aobus
 *     - Implement AO gate clocks via AO_RTI_GEN_CNTL_REG0
 *     - Implement AO reset functionality (#reset-cells = <1>)
 *       using AO_RTI_GEN_CNTL_REG0 reset bits
 *     - Add 32K clock tree (CEC, RTC oscillator)
 *
 * Linux reference: drivers/clk/meson/gxbb.c (~92KB, 200+ clocks)
 * Linux reference: drivers/clk/meson/gxbb-aoclk.c (~8KB)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_fixed.h>

#include <dev/clk/meson/meson_clk.h>

#include <dt-bindings/clock/gxbb-clkc.h>
#include <dt-bindings/clock/gxbb-aoclkc.h>

#ifdef __aarch64__
#include "opt_soc.h"
#endif

/* ================================================================
 * EE Clock Controller — amlogic,gxl-clkc
 *
 * Provides clocks for the EE (Everything Else) power domain.
 * The clkc node is a child of the HHI sysctrl (simple-mfd) node.
 *
 * Standard GXL clock frequencies (configured by U-Boot):
 *   fixed_pll  = 2000 MHz
 *   fclk_div2  = 1000 MHz (fixed_pll / 2)
 *   fclk_div3  =  666.67 MHz (fixed_pll / 3)
 *   fclk_div4  =  500 MHz (fixed_pll / 4)
 *   fclk_div5  =  400 MHz (fixed_pll / 5)
 *   fclk_div7  =  285.71 MHz (fixed_pll / 7)
 *   clk81      =  166.67 MHz (MPEG bus clock)
 * ================================================================ */

static struct clk_fixed_def gxl_ee_clks[] = {
	/*
	 * PLLs and fixed dividers.
	 *
	 * TODO: STUB — relies on U-Boot initialization
	 *
	 * Current behavior:
	 *   Registered as fixed-rate at the standard 2 GHz fixed_pll
	 *   output.  FCLK dividers are hardcoded to fixed_pll/N.
	 *
	 * For full implementation:
	 *   - Read HHI_FIXED_PLL_CNTL (0x40) to get actual PLL config
	 *   - Implement fclk_div2..7 as fixed-factor children of fixed_pll
	 *   - Implement sys_pll, gp0_pll, hdmi_pll with set_rate support
	 *
	 * Linux reference: drivers/clk/meson/gxbb.c, lines 50-250
	 */
	MESON_FIXED_RATE(CLKID_FIXED_PLL,  "fixed_pll",  2000000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV2,  "fclk_div2",  1000000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV3,  "fclk_div3",   666666666),
	MESON_FIXED_RATE(CLKID_FCLK_DIV4,  "fclk_div4",   500000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV5,  "fclk_div5",   400000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV7,  "fclk_div7",   285714285),

	/*
	 * MPEG bus clock (clk81) — primary peripheral bus clock.
	 *
	 * TODO: STUB — relies on U-Boot initialization
	 *
	 * Current behavior:
	 *   Registered as fixed 166.67 MHz.
	 *
	 * For full implementation:
	 *   - Read HHI_MPEG_CLK_CNTL (0x50) for mux select and divider
	 *   - Implement as mux + divider clock with fclk_div* parents
	 *   - Typically: fclk_div5 (400 MHz) / 2.4 ≈ 166.67 MHz
	 *     (actual path: fclk_div4 with specific divider settings)
	 *
	 * Linux reference: drivers/clk/meson/gxbb.c, mpeg_clk_sel/div/gate
	 */
	MESON_FIXED_RATE(CLKID_CLK81,      "clk81",       166666666),

	/*
	 * Peripheral bus gates.
	 * In the real implementation these are gate clocks on clk81,
	 * controlled by HHI_GCLK_MPEG0/1/2 registers.
	 */
	MESON_FIXED_RATE(CLKID_UART0,      "uart0",       166666666),
	MESON_FIXED_RATE(CLKID_UART1,      "uart1",       166666666),
	MESON_FIXED_RATE(CLKID_UART2,      "uart2",       166666666),
	MESON_FIXED_RATE(CLKID_ETH,        "eth",         166666666),
	MESON_FIXED_RATE(CLKID_SPICC,      "spicc",       166666666),
	MESON_FIXED_RATE(CLKID_I2C,        "i2c",         166666666),
	MESON_FIXED_RATE(CLKID_SD_EMMC_A,  "sd_emmc_a",   166666666),
	MESON_FIXED_RATE(CLKID_SD_EMMC_B,  "sd_emmc_b",   166666666),
	MESON_FIXED_RATE(CLKID_SD_EMMC_C,  "sd_emmc_c",   166666666),

	/*
	 * SD/eMMC core clocks — input to the MMC controller's clock mux.
	 *
	 * TODO: STUB — relies on U-Boot initialization
	 *
	 * Current behavior:
	 *   Reported as fixed 1 GHz (fclk_div2 frequency).  The Meson
	 *   GX MMC controller has its own internal clock register
	 *   (SD_EMMC_CLOCK at offset 0x00) with a mux that selects
	 *   between clkin0/clkin1/xtal and a divider.  The MMC driver
	 *   can use this internal divider to derive any card frequency
	 *   from the 1 GHz input.
	 *
	 * For full implementation:
	 *   - Implement SD_EMMC_*_CLK0_SEL (mux) + SD_EMMC_*_CLK0_DIV
	 *     (divider) + SD_EMMC_*_CLK0 (composite) clock nodes
	 *   - Support clk_set_rate() so the MMC driver can request
	 *     the desired frequency via the clock framework
	 *   - Parents: xtal (24 MHz), fclk_div2 (1 GHz), fclk_div3,
	 *     fclk_div5, fclk_div7
	 *
	 * Linux reference: drivers/clk/meson/gxbb.c, sd_emmc_*_clk0
	 */
	MESON_FIXED_RATE(CLKID_SD_EMMC_A_CLK0, "sd_emmc_a_clk0", 1000000000),
	MESON_FIXED_RATE(CLKID_SD_EMMC_B_CLK0, "sd_emmc_b_clk0", 1000000000),
	MESON_FIXED_RATE(CLKID_SD_EMMC_C_CLK0, "sd_emmc_c_clk0", 1000000000),
};

/* ================================================================
 * AO Clock Controller — amlogic,meson-gx{l,}-aoclkc
 *
 * Provides clocks (and resets) for the AO (Always-On) power domain.
 * The clkc_AO node is a child of aobus (simple-bus).
 *
 * AO domain clocks are gate clocks on clk81, controlled by
 * AO_RTI_GEN_CNTL_REG0.  In the stub, all are reported as
 * running at CLK81 frequency (166.67 MHz).
 *
 * Note: The AO clock controller also provides resets via
 * #reset-cells = <1>.  Reset support is not implemented in this
 * stub.  If needed, it can be added by implementing hwreset_if
 * methods in the base class or directly in this driver.
 * ================================================================ */

static struct clk_fixed_def gxl_ao_clks[] = {
	/* AO UART gates — used by uart_AO_A / uart_AO_B */
	MESON_FIXED_RATE(CLKID_AO_UART1,       "ao_uart1",       166666666),
	MESON_FIXED_RATE(CLKID_AO_UART2,       "ao_uart2",       166666666),

	/* AO CLK81 — AO domain copy of the MPEG bus clock */
	MESON_FIXED_RATE(CLKID_AO_CLK81,       "ao_clk81",       166666666),

	/* Other AO domain gates */
	MESON_FIXED_RATE(CLKID_AO_REMOTE,      "ao_remote",      166666666),
	MESON_FIXED_RATE(CLKID_AO_I2C_MASTER,  "ao_i2c_master",  166666666),
	MESON_FIXED_RATE(CLKID_AO_I2C_SLAVE,   "ao_i2c_slave",   166666666),
	MESON_FIXED_RATE(CLKID_AO_IR_BLASTER,  "ao_ir_blaster",  166666666),
};

/* ================================================================
 * EE Clock Controller Driver
 * ================================================================ */

static int
meson_gxl_clkc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "amlogic,gxl-clkc"))
		return (ENXIO);

	device_set_desc(dev, "Amlogic Meson GXL Clock Controller (stub)");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_gxl_clkc_attach(device_t dev)
{

	return (meson_clk_attach(dev, gxl_ee_clks, nitems(gxl_ee_clks)));
}

static device_method_t meson_gxl_clkc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		meson_gxl_clkc_probe),
	DEVMETHOD(device_attach,	meson_gxl_clkc_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(meson_gxl_clkc, meson_gxl_clkc_driver,
    meson_gxl_clkc_methods, sizeof(struct meson_clk_softc),
    meson_clkc_driver);

/*
 * The EE clkc node lives inside the HHI sysctrl node, which is
 * handled by the simple_mfd driver (matching "simple-mfd" compatible).
 * Children of simple_mfd are enumerated as bus devices, so we attach
 * to the simple_mfd bus.
 */
EARLY_DRIVER_MODULE(meson_gxl_clkc, simple_mfd, meson_gxl_clkc_driver,
    0, 0, BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);

/* ================================================================
 * AO Clock Controller Driver
 * ================================================================ */

static int
meson_gxl_aoclkc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "amlogic,meson-gxl-aoclkc"))
		goto match;
	if (ofw_bus_is_compatible(dev, "amlogic,meson-gx-aoclkc"))
		goto match;

	return (ENXIO);

match:
	device_set_desc(dev, "Amlogic Meson GX AO Clock Controller (stub)");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_gxl_aoclkc_attach(device_t dev)
{

	return (meson_clk_attach(dev, gxl_ao_clks, nitems(gxl_ao_clks)));
}

static device_method_t meson_gxl_aoclkc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		meson_gxl_aoclkc_probe),
	DEVMETHOD(device_attach,	meson_gxl_aoclkc_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(meson_gxl_aoclkc, meson_gxl_aoclkc_driver,
    meson_gxl_aoclkc_methods, sizeof(struct meson_clk_softc),
    meson_clkc_driver);

/*
 * The AO clkc node lives inside aobus, which is a simple-bus node
 * handled by the simplebus driver.
 */
EARLY_DRIVER_MODULE(meson_gxl_aoclkc, simplebus, meson_gxl_aoclkc_driver,
    0, 0, BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
