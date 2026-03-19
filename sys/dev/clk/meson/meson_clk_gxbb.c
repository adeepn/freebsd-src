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
 * Amlogic Meson GXBB/GXL clock controllers.
 *
 * This file provides two drivers:
 *   1. EE clock controller (amlogic,gxl-clkc) — main system clocks
 *   2. AO clock controller (amlogic,meson-gx{l,}-aoclkc) — always-on domain
 *
 * The EE controller implements:
 *   - Fixed-rate clocks for PLLs and dividers (relying on U-Boot config)
 *   - Hardware gate clocks via HHI_GCLK_MPEG0/1/2 registers
 *
 * The AO controller registers only fixed-rate clocks (gates not yet
 * implemented — the AO domain is always-on and U-Boot leaves gates open).
 *
 * TODO: STUB — PLLs and dividers rely on U-Boot initialization
 *
 * For full implementation:
 *   - EE: PLL clocks (sys_pll, fixed_pll, gp0_pll, hdmi_pll)
 *   - EE: MPEG clock mux+divider (clk81) from HHI_MPEG_CLK_CNTL
 *   - EE: SD/eMMC mux+divider clocks for dynamic frequency
 *   - AO: Gate clocks via AO_RTI_GEN_CNTL_REG0
 *   - AO: Reset functionality (#reset-cells = <1>)
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

#include "syscon_if.h"

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

/*
 * Fixed-rate clocks: PLLs, dividers, and bus clocks.
 *
 * TODO: STUB — relies on U-Boot initialization
 *
 * For full implementation:
 *   - Read HHI_FIXED_PLL_CNTL (0x40) to get actual PLL config
 *   - Implement fclk_div2..7 as fixed-factor children of fixed_pll
 *   - Implement MPEG clock mux + divider (clk81) from HHI_MPEG_CLK_CNTL
 *   - Implement SD_EMMC_*_CLK0 as composite mux+divider clocks
 */
static struct clk_fixed_def gxl_ee_fixed_clks[] = {
	MESON_FIXED_RATE(CLKID_FIXED_PLL,  "fixed_pll",  2000000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV2,  "fclk_div2",  1000000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV3,  "fclk_div3",   666666666),
	MESON_FIXED_RATE(CLKID_FCLK_DIV4,  "fclk_div4",   500000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV5,  "fclk_div5",   400000000),
	MESON_FIXED_RATE(CLKID_FCLK_DIV7,  "fclk_div7",   285714285),
	MESON_FIXED_RATE(CLKID_CLK81,      "clk81",       166666666),
};

/*
 * HHI register offsets for clock gate registers.
 * These are relative to the HHI sysctrl base (0xc883c000 on GXL).
 * Linux reference: drivers/clk/meson/gxbb.c, lines 40-43
 */
#define	HHI_GCLK_MPEG0		0x140
#define	HHI_GCLK_MPEG1		0x144
#define	HHI_GCLK_MPEG2		0x148

/*
 * HHI clock control registers for SD/eMMC controllers.
 * These contain mux + divider + gate for the controller's functional clock.
 * We only implement the gate bit; mux/divider are left at U-Boot defaults.
 * Linux reference: drivers/clk/meson/gxbb.c
 */
#define	HHI_SD_EMMC_CLK_CNTL	0x264	/* sd_emmc_a clock control */
#define	HHI_NAND_CLK_CNTL	0x25C	/* sd_emmc_b (bit 7), sd_emmc_c (bit 23) */

/*
 * Peripheral bus gate clocks.
 *
 * These are single-bit gates in HHI_GCLK_MPEG0/1/2 registers that
 * enable/disable the bus clock to each peripheral.  clk_enable()
 * sets the bit (opening the gate); clk_disable() clears it.
 *
 * Without opening the gate, register access to the peripheral
 * controller will hang the AXI bus.
 *
 * Bit positions from Linux drivers/clk/meson/gxbb.c (GXBB_PCLK macros).
 */
static struct clk_gate_def gxl_ee_gate_clks[] = {
	/* Peripheral bus gates (HHI_GCLK_MPEGx) */
	MESON_CLK_GATE(CLKID_SPICC,     "spicc",     "clk81", HHI_GCLK_MPEG0,  8),
	MESON_CLK_GATE(CLKID_I2C,       "i2c",       "clk81", HHI_GCLK_MPEG0,  9),
	MESON_CLK_GATE(CLKID_UART0,     "uart0",     "clk81", HHI_GCLK_MPEG0, 13),
	MESON_CLK_GATE(CLKID_SD_EMMC_A, "sd_emmc_a", "clk81", HHI_GCLK_MPEG0, 24),
	MESON_CLK_GATE(CLKID_SD_EMMC_B, "sd_emmc_b", "clk81", HHI_GCLK_MPEG0, 25),
	MESON_CLK_GATE(CLKID_SD_EMMC_C, "sd_emmc_c", "clk81", HHI_GCLK_MPEG0, 26),
	MESON_CLK_GATE(CLKID_ETH,       "eth",       "clk81", HHI_GCLK_MPEG1,  3),
	MESON_CLK_GATE(CLKID_UART1,     "uart1",     "clk81", HHI_GCLK_MPEG1, 16),
	MESON_CLK_GATE(CLKID_UART2,     "uart2",     "clk81", HHI_GCLK_MPEG2, 15),

	/*
	 * SD/eMMC controller functional clock gates.
	 *
	 * These gate the clock that feeds the controller IP itself.
	 * Without this clock, registers at offset >= 0x40 (CFG, STATUS,
	 * CMD) are inaccessible and reads/writes will hang the AXI bus.
	 *
	 * In Linux these are composite mux+divider+gate clocks.  We only
	 * implement the gate; the mux and divider are configured to
	 * xtal/1 (24 MHz) in meson_gxl_clkc_attach().
	 *
	 * Register layout (from Linux drivers/clk/meson/gxbb.c):
	 *   sd_emmc_a: HHI_SD_EMMC_CLK_CNTL (0x264), gate bit 7
	 *   sd_emmc_b: HHI_SD_EMMC_CLK_CNTL (0x264), gate bit 23
	 *   sd_emmc_c: HHI_NAND_CLK_CNTL    (0x25C), gate bit 7
	 */
	MESON_CLK_GATE(CLKID_SD_EMMC_A_CLK0, "sd_emmc_a_clk0", "fclk_div2",
	    HHI_SD_EMMC_CLK_CNTL,  7),
	MESON_CLK_GATE(CLKID_SD_EMMC_B_CLK0, "sd_emmc_b_clk0", "fclk_div2",
	    HHI_SD_EMMC_CLK_CNTL, 23),
	MESON_CLK_GATE(CLKID_SD_EMMC_C_CLK0, "sd_emmc_c_clk0", "fclk_div2",
	    HHI_NAND_CLK_CNTL,     7),
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

	device_set_desc(dev, "Amlogic Meson GXL Clock Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_gxl_clkc_attach(device_t dev)
{
	struct meson_clk_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);

	/* Get parent's syscon handle for HHI register access (gate clocks). */
	if (SYSCON_GET_HANDLE(dev, &sc->syscon) != 0) {
		device_printf(dev, "cannot get syscon handle\n");
		return (ENXIO);
	}

	/*
	 * Configure HHI SD/eMMC clock control registers with valid
	 * mux/divider defaults.  Our clock framework only implements
	 * the gate bit; without valid mux (source) and divider settings,
	 * enabling the gate won't produce a usable clock.
	 *
	 * Set mux to xtal (24 MHz) and divider to 1 for all controllers.
	 * Gate bits are left untouched (managed by clk_enable/disable).
	 *
	 * HHI_SD_EMMC_CLK_CNTL (0x264) layout:
	 *   sd_emmc_a: mux[11:9], gate[7], div[6:0]
	 *   sd_emmc_b: mux[27:25], gate[23], div[22:16]
	 * HHI_NAND_CLK_CNTL (0x25C) layout:
	 *   sd_emmc_c: mux[11:9], gate[7], div[6:0]
	 */
	val = SYSCON_UNLOCKED_READ_4(sc->syscon, HHI_SD_EMMC_CLK_CNTL);
	/* sd_emmc_a: clear mux[11:9] and div[6:0], keep gate[7] */
	val &= ~(0x7F | (0x7 << 9));
	val |= 1;		/* div = 1, mux = 0 (xtal) */
	/* sd_emmc_b: clear mux[27:25] and div[22:16], keep gate[23] */
	val &= ~((0x7F << 16) | (0x7 << 25));
	val |= (1 << 16);	/* div = 1, mux = 0 (xtal) */
	SYSCON_UNLOCKED_WRITE_4(sc->syscon, HHI_SD_EMMC_CLK_CNTL, val);

	/* sd_emmc_c: clear mux[11:9] and div[6:0], keep gate[7] */
	val = SYSCON_UNLOCKED_READ_4(sc->syscon, HHI_NAND_CLK_CNTL);
	val &= ~(0x7F | (0x7 << 9));
	val |= 1;		/* div = 1, mux = 0 (xtal) */
	SYSCON_UNLOCKED_WRITE_4(sc->syscon, HHI_NAND_CLK_CNTL, val);

	return (meson_clk_attach(dev, gxl_ee_fixed_clks,
	    nitems(gxl_ee_fixed_clks), gxl_ee_gate_clks,
	    nitems(gxl_ee_gate_clks)));
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

	return (meson_clk_attach(dev, gxl_ao_clks, nitems(gxl_ao_clks),
	    NULL, 0));
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
