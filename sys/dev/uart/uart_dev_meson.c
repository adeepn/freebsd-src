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
 * UART driver for the Amlogic Meson SoC family (GXL, AXG, G12A, etc.).
 *
 * Reference: Linux drivers/tty/serial/meson_uart.c
 * FreeBSD template: sys/dev/uart/uart_dev_imx.c
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_dev_meson.h>

#include "uart_if.h"

/*
 * FIFO parameters.  The Meson UART has a 64-byte FIFO by default
 * (UART_0 on the EE domain has 128 bytes, but we use 64 as a safe
 * default).  We trigger the RX interrupt after 1 byte to minimize
 * latency, and the TX interrupt at half-empty.
 */
#define	MESON_UART_FIFOSZ	64
#define	MESON_UART_RXFIFO_LVL	1
#define	MESON_UART_TXFIFO_LVL	(MESON_UART_FIFOSZ / 2)

/* -------------------------------------------------------------------- */
/* Low-level UART interface (uart_ops)                                  */
/* -------------------------------------------------------------------- */

static int  meson_uart_probe(struct uart_bas *bas);
static void meson_uart_init(struct uart_bas *bas, int, int, int, int);
static void meson_uart_term(struct uart_bas *bas);
static void meson_uart_putc(struct uart_bas *bas, int);
static int  meson_uart_rxready(struct uart_bas *bas);
static int  meson_uart_getc(struct uart_bas *bas, struct mtx *);

static struct uart_ops uart_meson_ops = {
	.probe = meson_uart_probe,
	.init = meson_uart_init,
	.term = meson_uart_term,
	.putc = meson_uart_putc,
	.rxready = meson_uart_rxready,
	.getc = meson_uart_getc,
};

static int
meson_uart_probe(struct uart_bas *bas)
{

	return (0);
}

static void
meson_uart_init(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{
	uint32_t val;

	/* Reset TX and RX, clear errors. */
	val = uart_getreg(bas, AML_UART_CONTROL);
	val |= AML_UART_TX_RST | AML_UART_RX_RST | AML_UART_CLEAR_ERR;
	uart_setreg(bas, AML_UART_CONTROL, val);
	/* Clear the reset bits. */
	val &= ~(AML_UART_TX_RST | AML_UART_RX_RST | AML_UART_CLEAR_ERR);
	uart_setreg(bas, AML_UART_CONTROL, val);

	/* Enable TX, RX, two-wire mode (no modem signals). */
	val |= AML_UART_TX_EN | AML_UART_RX_EN | AML_UART_TWO_WIRE_EN;

	/* Data bits. */
	val &= ~AML_UART_DATA_LEN_MASK;
	switch (databits) {
	case 7:
		val |= AML_UART_DATA_LEN_7BIT;
		break;
	case 6:
		val |= AML_UART_DATA_LEN_6BIT;
		break;
	case 5:
		val |= AML_UART_DATA_LEN_5BIT;
		break;
	case 8:
	default:
		val |= AML_UART_DATA_LEN_8BIT;
		break;
	}

	/* Stop bits. */
	val &= ~AML_UART_STOP_BIT_LEN_MASK;
	if (stopbits == 2)
		val |= AML_UART_STOP_BIT_2SB;
	else
		val |= AML_UART_STOP_BIT_1SB;

	/* Parity. */
	switch (parity) {
	case UART_PARITY_EVEN:
		val |= AML_UART_PARITY_EN;
		val &= ~AML_UART_PARITY_TYPE;
		break;
	case UART_PARITY_ODD:
		val |= AML_UART_PARITY_EN;
		val |= AML_UART_PARITY_TYPE;
		break;
	case UART_PARITY_NONE:
	default:
		val &= ~AML_UART_PARITY_EN;
		break;
	}

	uart_setreg(bas, AML_UART_CONTROL, val);

	/*
	 * TODO: STUB -- relies on U-Boot baud rate configuration
	 *
	 * Current behavior:
	 *   Baud rate is only programmed if both baudrate and rclk are
	 *   provided. In practice, U-Boot has already configured 115200
	 *   baud on the console UART before we get here.
	 *
	 * For a complete implementation:
	 *   - Query the actual crystal frequency from the clock framework
	 *     (24 MHz on all known Meson GX/G12/SM1 SoCs)
	 *   - Properly handle the xtal_div2 vs xtal_div3 selection
	 *   - Support runtime baud rate changes for all UART ports
	 *
	 * Linux reference: drivers/tty/serial/meson_uart.c,
	 *   meson_uart_set_termios() and meson_uart_change_speed()
	 */
	if (baudrate > 0 && bas->rclk > 0) {
		uint32_t baud_val;

		/*
		 * Use the new baud rate register (REG5).
		 * Formula: divisor = round(xtal_clk / (xtal_div * baud)) - 1
		 * We use xtal / 3 as the default divider.
		 */
		baud_val = (bas->rclk / 3 + baudrate / 2) / baudrate - 1;
		baud_val &= AML_UART_BAUD_MASK;
		baud_val |= AML_UART_BAUD_USE | AML_UART_BAUD_XTAL;
		uart_setreg(bas, AML_UART_REG5, baud_val);
	}

	/* Set interrupt thresholds in MISC register. */
	val = (MESON_UART_TXFIFO_LVL << AML_UART_XMIT_IRQ_SHIFT) |
	    (MESON_UART_RXFIFO_LVL << AML_UART_RECV_IRQ_SHIFT);
	uart_setreg(bas, AML_UART_MISC, val);
	uart_barrier(bas);
}

static void
meson_uart_term(struct uart_bas *bas)
{

	/* Disable interrupts. */
	uint32_t val;

	val = uart_getreg(bas, AML_UART_CONTROL);
	val &= ~(AML_UART_RX_INT_EN | AML_UART_TX_INT_EN);
	uart_setreg(bas, AML_UART_CONTROL, val);
	uart_barrier(bas);
}

static void
meson_uart_putc(struct uart_bas *bas, int c)
{

	/* Wait until TX FIFO is not full. */
	while (uart_getreg(bas, AML_UART_STATUS) & AML_UART_TX_FULL)
		;
	uart_setreg(bas, AML_UART_WFIFO, c & 0xff);
	uart_barrier(bas);
}

static int
meson_uart_rxready(struct uart_bas *bas)
{

	return (!(uart_getreg(bas, AML_UART_STATUS) & AML_UART_RX_EMPTY));
}

static int
meson_uart_getc(struct uart_bas *bas, struct mtx *hwmtx)
{
	int c;

	uart_lock(hwmtx);
	/* Wait until RX FIFO is not empty. */
	while (uart_getreg(bas, AML_UART_STATUS) & AML_UART_RX_EMPTY)
		;
	c = uart_getreg(bas, AML_UART_RFIFO) & 0xff;
	uart_unlock(hwmtx);

	return (c);
}

/* -------------------------------------------------------------------- */
/* High-level UART interface (bus methods)                              */
/* -------------------------------------------------------------------- */

struct meson_uart_softc {
	struct uart_softc base;
};

static int meson_uart_bus_attach(struct uart_softc *);
static int meson_uart_bus_detach(struct uart_softc *);
static int meson_uart_bus_flush(struct uart_softc *, int);
static int meson_uart_bus_getsig(struct uart_softc *);
static int meson_uart_bus_ioctl(struct uart_softc *, int, intptr_t);
static int meson_uart_bus_ipend(struct uart_softc *);
static int meson_uart_bus_param(struct uart_softc *, int, int, int, int);
static int meson_uart_bus_probe(struct uart_softc *);
static int meson_uart_bus_receive(struct uart_softc *);
static int meson_uart_bus_setsig(struct uart_softc *, int);
static int meson_uart_bus_transmit(struct uart_softc *);
static void meson_uart_bus_grab(struct uart_softc *);
static void meson_uart_bus_ungrab(struct uart_softc *);

static kobj_method_t meson_uart_methods[] = {
	KOBJMETHOD(uart_attach,		meson_uart_bus_attach),
	KOBJMETHOD(uart_detach,		meson_uart_bus_detach),
	KOBJMETHOD(uart_flush,		meson_uart_bus_flush),
	KOBJMETHOD(uart_getsig,		meson_uart_bus_getsig),
	KOBJMETHOD(uart_ioctl,		meson_uart_bus_ioctl),
	KOBJMETHOD(uart_ipend,		meson_uart_bus_ipend),
	KOBJMETHOD(uart_param,		meson_uart_bus_param),
	KOBJMETHOD(uart_probe,		meson_uart_bus_probe),
	KOBJMETHOD(uart_receive,	meson_uart_bus_receive),
	KOBJMETHOD(uart_setsig,		meson_uart_bus_setsig),
	KOBJMETHOD(uart_transmit,	meson_uart_bus_transmit),
	KOBJMETHOD(uart_grab,		meson_uart_bus_grab),
	KOBJMETHOD(uart_ungrab,		meson_uart_bus_ungrab),
	{ 0, 0 }
};

static struct uart_class uart_meson_class = {
	"meson",
	meson_uart_methods,
	sizeof(struct meson_uart_softc),
	.uc_ops = &uart_meson_ops,
	.uc_range = AML_UART_REG5 + 4,
	.uc_rclk = 24000000,	/* 24 MHz crystal on all Meson SoCs */
	.uc_rshift = 0,
};

static struct ofw_compat_data compat_data[] = {
	{"amlogic,meson-gx-uart",	(uintptr_t)&uart_meson_class},
	{"amlogic,meson-ao-uart",	(uintptr_t)&uart_meson_class},
	{"amlogic,meson-s4-uart",	(uintptr_t)&uart_meson_class},
	{NULL,				(uintptr_t)NULL},
};
UART_FDT_CLASS_AND_DEVICE(compat_data);

static int
meson_uart_bus_attach(struct uart_softc *sc)
{
	struct uart_bas *bas;
	struct uart_devinfo *di;

	bas = &sc->sc_bas;

	/*
	 * TODO: STUB -- relies on U-Boot clock configuration
	 *
	 * Current behavior:
	 *   rclk is hardcoded to 24 MHz (the crystal oscillator frequency).
	 *   This is correct for the baud rate divisor calculation on all
	 *   known Meson GX/AXG/G12/SM1 SoCs.
	 *
	 * For a complete implementation:
	 *   - Query the "baud" clock from the clock framework via
	 *     clk_get_by_ofw_name(sc->sc_dev, 0, "baud", &clk)
	 *   - Enable the "pclk" gate clock for APB register access
	 *   - Use clk_get_freq() to get the actual clock rate
	 *
	 * Linux reference: drivers/tty/serial/meson_uart.c,
	 *   meson_uart_probe() clock setup
	 */

	if (sc->sc_sysdev != NULL) {
		di = sc->sc_sysdev;
		meson_uart_init(bas, di->baudrate, di->databits,
		    di->stopbits, di->parity);
	} else {
		meson_uart_init(bas, 115200, 8, 1, 0);
	}

	/* Enable RX interrupt, disable TX interrupt (enabled on transmit). */
	uint32_t val;
	val = uart_getreg(bas, AML_UART_CONTROL);
	val |= AML_UART_RX_INT_EN;
	val &= ~AML_UART_TX_INT_EN;
	uart_setreg(bas, AML_UART_CONTROL, val);
	uart_barrier(bas);

	return (0);
}

static int
meson_uart_bus_detach(struct uart_softc *sc)
{
	struct uart_bas *bas;

	bas = &sc->sc_bas;
	/* Disable all interrupts. */
	uint32_t val;
	val = uart_getreg(bas, AML_UART_CONTROL);
	val &= ~(AML_UART_RX_INT_EN | AML_UART_TX_INT_EN);
	uart_setreg(bas, AML_UART_CONTROL, val);
	uart_barrier(bas);

	return (0);
}

static int
meson_uart_bus_flush(struct uart_softc *sc, int what)
{
	struct uart_bas *bas;
	uint32_t val;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);

	val = uart_getreg(bas, AML_UART_CONTROL);
	if (what & UART_FLUSH_TRANSMITTER)
		val |= AML_UART_TX_RST;
	if (what & UART_FLUSH_RECEIVER)
		val |= AML_UART_RX_RST;
	uart_setreg(bas, AML_UART_CONTROL, val);

	/* Clear the reset bits. */
	val &= ~(AML_UART_TX_RST | AML_UART_RX_RST);
	uart_setreg(bas, AML_UART_CONTROL, val);
	uart_barrier(bas);

	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static int
meson_uart_bus_getsig(struct uart_softc *sc)
{

	/* Meson UART in two-wire mode has no modem signals. */
	return (0);
}

static int
meson_uart_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{
	int error;

	error = 0;
	uart_lock(sc->sc_hwmtx);
	switch (request) {
	case UART_IOCTL_BAUD:
		/*
		 * TODO: STUB -- baud rate readback not implemented
		 *
		 * Current behavior:
		 *   Returns EINVAL (unknown ioctl) since we do not
		 *   read back the baud rate from hardware.
		 *
		 * For a complete implementation:
		 *   - Read AML_UART_REG5 to extract the current divisor
		 *   - Compute baud = rclk / (xtal_div * (divisor + 1))
		 *   - Return the result in *data
		 *
		 * Linux reference: drivers/tty/serial/meson_uart.c,
		 *   meson_uart_change_speed()
		 */
		error = EINVAL;
		break;
	default:
		error = EINVAL;
		break;
	}
	uart_unlock(sc->sc_hwmtx);
	return (error);
}

static int
meson_uart_bus_ipend(struct uart_softc *sc)
{
	struct uart_bas *bas;
	uint32_t status, ctrl;
	int ipend;

	bas = &sc->sc_bas;
	ipend = 0;

	uart_lock(sc->sc_hwmtx);

	status = uart_getreg(bas, AML_UART_STATUS);
	ctrl = uart_getreg(bas, AML_UART_CONTROL);

	/*
	 * RX: if RX FIFO is not empty and RX interrupt is enabled,
	 * signal receive ready.
	 */
	if (!(status & AML_UART_RX_EMPTY) && (ctrl & AML_UART_RX_INT_EN)) {
		ipend |= SER_INT_RXREADY;
	}

	/*
	 * TX: if TX FIFO is empty (or not full) and TX interrupt is
	 * enabled, signal TX idle.  Disable TX interrupt to avoid
	 * re-entry until the next transmit call.
	 */
	if ((status & AML_UART_TX_EMPTY) && (ctrl & AML_UART_TX_INT_EN)) {
		ctrl &= ~AML_UART_TX_INT_EN;
		uart_setreg(bas, AML_UART_CONTROL, ctrl);
		uart_barrier(bas);
		ipend |= SER_INT_TXIDLE;
	}

	/* Check for framing/parity errors. */
	if (status & AML_UART_ERR) {
		/* Clear the error. */
		ctrl |= AML_UART_CLEAR_ERR;
		uart_setreg(bas, AML_UART_CONTROL, ctrl);
		ctrl &= ~AML_UART_CLEAR_ERR;
		uart_setreg(bas, AML_UART_CONTROL, ctrl);
		uart_barrier(bas);
	}

	uart_unlock(sc->sc_hwmtx);
	return (ipend);
}

static int
meson_uart_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{

	uart_lock(sc->sc_hwmtx);
	meson_uart_init(&sc->sc_bas, baudrate, databits, stopbits, parity);
	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static int
meson_uart_bus_probe(struct uart_softc *sc)
{
	int error;

	error = meson_uart_probe(&sc->sc_bas);
	if (error)
		return (error);

	sc->sc_rxfifosz = MESON_UART_FIFOSZ;
	sc->sc_txfifosz = MESON_UART_TXFIFO_LVL;

	device_set_desc(sc->sc_dev, "Amlogic Meson UART");
	return (0);
}

static int
meson_uart_bus_receive(struct uart_softc *sc)
{
	struct uart_bas *bas;
	uint32_t status;
	int c, out;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);

	status = uart_getreg(bas, AML_UART_STATUS);
	while (!(status & AML_UART_RX_EMPTY)) {
		if (uart_rx_full(sc)) {
			sc->sc_rxbuf[sc->sc_rxput] = UART_STAT_OVERRUN;
			break;
		}
		c = uart_getreg(bas, AML_UART_RFIFO);
		out = c & 0xff;
		if (status & AML_UART_FRAME_ERR)
			out |= UART_STAT_FRAMERR;
		if (status & AML_UART_PARITY_ERR)
			out |= UART_STAT_PARERR;
		uart_rx_put(sc, out);
		status = uart_getreg(bas, AML_UART_STATUS);
	}

	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static int
meson_uart_bus_setsig(struct uart_softc *sc, int sig)
{

	/* No modem signals in two-wire mode. */
	return (0);
}

static int
meson_uart_bus_transmit(struct uart_softc *sc)
{
	struct uart_bas *bas;
	uint32_t ctrl;
	int i;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);

	/* Write data to the TX FIFO. */
	for (i = 0; i < sc->sc_txdatasz; i++) {
		/* Check if TX FIFO is full before each write. */
		if (uart_getreg(bas, AML_UART_STATUS) & AML_UART_TX_FULL)
			break;
		uart_setreg(bas, AML_UART_WFIFO,
		    sc->sc_txbuf[i] & 0xff);
	}
	sc->sc_txbusy = 1;

	/* Enable TX interrupt to know when FIFO drains. */
	ctrl = uart_getreg(bas, AML_UART_CONTROL);
	ctrl |= AML_UART_TX_INT_EN;
	uart_setreg(bas, AML_UART_CONTROL, ctrl);
	uart_barrier(bas);

	uart_unlock(sc->sc_hwmtx);
	return (0);
}

static void
meson_uart_bus_grab(struct uart_softc *sc)
{
	struct uart_bas *bas;
	uint32_t ctrl;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	ctrl = uart_getreg(bas, AML_UART_CONTROL);
	ctrl &= ~AML_UART_RX_INT_EN;
	uart_setreg(bas, AML_UART_CONTROL, ctrl);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}

static void
meson_uart_bus_ungrab(struct uart_softc *sc)
{
	struct uart_bas *bas;
	uint32_t ctrl;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	ctrl = uart_getreg(bas, AML_UART_CONTROL);
	ctrl |= AML_UART_RX_INT_EN;
	uart_setreg(bas, AML_UART_CONTROL, ctrl);
	uart_barrier(bas);
	uart_unlock(sc->sc_hwmtx);
}
