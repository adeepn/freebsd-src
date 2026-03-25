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
 * Implemented:
 *   - EE: Fixed-rate (fixed_pll) and fixed-factor (fclk_div2..7, clk81)
 *   - EE: Gate clocks (HHI_GCLK_MPEG0/1/2, SD_EMMC functional clocks)
 *   - AO: Gate clocks (AO_RTI_GEN_CNTL_REG0) with direct resource access
 *   - AO: Reset controller (bits 16-23 of AO_RTI_GEN_CNTL_REG0)
 *
 * Future work:
 *   - sys_pll PLL clock with set_rate for cpufreq
 *   - clk81 mux+divider hardware readback (HHI_MPEG_CLK_CNTL)
 *
 * Linux reference: drivers/clk/meson/gxbb.c (~92KB, 200+ clocks)
 * Linux reference: drivers/clk/meson/gxbb-aoclk.c (~8KB)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_fixed.h>

#include <dev/clk/meson/meson_clk.h>

#include <dev/hwreset/hwreset.h>

#include "hwreset_if.h"
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
 * Clock tree: fixed_pll → fclk_div{2..7} → clk81 (→ peripheral gates).
 *
 * fixed_pll is configured by U-Boot to 2000 MHz and never changes.
 * fclk_div{2..7} are fixed-factor dividers from fixed_pll.
 * clk81 is actually a mux+divider (HHI_MPEG_CLK_CNTL), but U-Boot
 * always selects fclk_div3/4 = 166.67 MHz.  Modeled as fixed-factor
 * until hardware mux readback is implemented.
 */
static const char *fclk_div_parent[] = { "fixed_pll" };
static const char *clk81_parent[]    = { "fclk_div3" };

static struct clk_fixed_def gxl_ee_fixed_clks[] = {
	/*
	 * fixed_pll: 2000 MHz — configured by U-Boot, never changed.
	 * Parent is xtal (24 MHz) but the PLL multiplier is not modeled;
	 * we report the output frequency directly.
	 */
	MESON_FIXED_RATE(CLKID_FIXED_PLL,  "fixed_pll",  2000000000),

	/* fclk_div: fixed-factor dividers from fixed_pll */
	MESON_FIXED_FACTOR(CLKID_FCLK_DIV2, "fclk_div2", fclk_div_parent, 1, 2),
	MESON_FIXED_FACTOR(CLKID_FCLK_DIV3, "fclk_div3", fclk_div_parent, 1, 3),
	MESON_FIXED_FACTOR(CLKID_FCLK_DIV4, "fclk_div4", fclk_div_parent, 1, 4),
	MESON_FIXED_FACTOR(CLKID_FCLK_DIV5, "fclk_div5", fclk_div_parent, 1, 5),
	MESON_FIXED_FACTOR(CLKID_FCLK_DIV7, "fclk_div7", fclk_div_parent, 1, 7),

	/*
	 * clk81: MPEG bus clock, 166.67 MHz.
	 * Actually a mux+divider (HHI_MPEG_CLK_CNTL), but U-Boot
	 * always sets fclk_div3/4.  Modeled as fixed-factor for now.
	 */
	MESON_FIXED_FACTOR(CLKID_CLK81, "clk81", clk81_parent, 1, 4),
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

/*
 * AO_RTI_GEN_CNTL_REG0 register layout:
 *   Bits 0-6:  clock gate enables (1 = enabled)
 *   Bits 16-23: reset control (0 = asserted, 1 = deasserted)
 */
#define	AO_RTI_GEN_CNTL_REG0	0x40

static struct clk_gate_def gxl_ao_gate_clks[] = {
	MESON_CLK_GATE(CLKID_AO_REMOTE,      "ao_remote",      "clk81",
	    AO_RTI_GEN_CNTL_REG0, 0),
	MESON_CLK_GATE(CLKID_AO_I2C_MASTER,  "ao_i2c_master",  "clk81",
	    AO_RTI_GEN_CNTL_REG0, 1),
	MESON_CLK_GATE(CLKID_AO_I2C_SLAVE,   "ao_i2c_slave",   "clk81",
	    AO_RTI_GEN_CNTL_REG0, 2),
	MESON_CLK_GATE(CLKID_AO_UART1,       "ao_uart1",       "clk81",
	    AO_RTI_GEN_CNTL_REG0, 3),
	MESON_CLK_GATE(CLKID_AO_UART2,       "ao_uart2",       "clk81",
	    AO_RTI_GEN_CNTL_REG0, 5),
	MESON_CLK_GATE(CLKID_AO_IR_BLASTER,  "ao_ir_blaster",  "clk81",
	    AO_RTI_GEN_CNTL_REG0, 6),
};

/*
 * AO resets — bits 16-23 of AO_RTI_GEN_CNTL_REG0.
 * Reset is active-low: clear bit = assert, set bit = deassert.
 */
static const uint8_t gxl_ao_reset_bits[] = {
	[0] = 16,	/* RESET_AO_REMOTE */
	[1] = 17,	/* RESET_AO_UART1 */
	[2] = 18,	/* RESET_AO_I2C_MASTER */
	[3] = 19,	/* RESET_AO_I2C_SLAVE */
	[4] = 22,	/* RESET_AO_UART2 */
	[5] = 23,	/* RESET_AO_IR_BLASTER */
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
	device_set_desc(dev, "Amlogic Meson GX AO Clock Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
meson_gxl_aoclkc_attach(device_t dev)
{
	struct meson_clk_softc *sc;

	sc = device_get_softc(dev);

	/*
	 * AO clock controller lives inside a simple_mfd node.
	 * Use parent's syscon for register access, same as EE clock.
	 */
	if (SYSCON_GET_HANDLE(dev, &sc->syscon) != 0) {
		device_printf(dev, "cannot get syscon handle\n");
		return (ENXIO);
	}

	/* Register as reset provider for AO domain resets. */
	hwreset_register_ofw_provider(dev);

	return (meson_clk_attach(dev, NULL, 0,
	    gxl_ao_gate_clks, nitems(gxl_ao_gate_clks)));
}

/*
 * AO reset controller — hwreset_if implementation.
 *
 * Resets are controlled by bits 16-23 of AO_RTI_GEN_CNTL_REG0.
 * Active-low: clear bit = assert reset, set bit = deassert.
 */
static int
meson_gxl_aoclkc_reset_assert(device_t dev, intptr_t id, bool assert)
{
	struct meson_clk_softc *sc;
	uint32_t val, bit;

	if (id < 0 || id >= (intptr_t)nitems(gxl_ao_reset_bits))
		return (EINVAL);

	sc = device_get_softc(dev);
	bit = gxl_ao_reset_bits[id];

	SYSCON_DEVICE_LOCK(device_get_parent(dev));
	val = SYSCON_UNLOCKED_READ_4(sc->syscon, AO_RTI_GEN_CNTL_REG0);
	if (assert)
		val &= ~(1u << bit);
	else
		val |= (1u << bit);
	SYSCON_UNLOCKED_WRITE_4(sc->syscon, AO_RTI_GEN_CNTL_REG0, val);
	SYSCON_DEVICE_UNLOCK(device_get_parent(dev));

	return (0);
}

static int
meson_gxl_aoclkc_reset_is_asserted(device_t dev, intptr_t id, bool *asserted)
{
	struct meson_clk_softc *sc;
	uint32_t val, bit;

	if (id < 0 || id >= (intptr_t)nitems(gxl_ao_reset_bits))
		return (EINVAL);

	sc = device_get_softc(dev);
	bit = gxl_ao_reset_bits[id];

	val = SYSCON_UNLOCKED_READ_4(sc->syscon, AO_RTI_GEN_CNTL_REG0);
	*asserted = !(val & (1u << bit));

	return (0);
}

static device_method_t meson_gxl_aoclkc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			meson_gxl_aoclkc_probe),
	DEVMETHOD(device_attach,		meson_gxl_aoclkc_attach),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,		meson_gxl_aoclkc_reset_assert),
	DEVMETHOD(hwreset_is_asserted,		meson_gxl_aoclkc_reset_is_asserted),

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
