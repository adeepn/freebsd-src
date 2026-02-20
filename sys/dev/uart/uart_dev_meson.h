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
 * Register definitions for the Amlogic Meson UART controller.
 *
 * Reference: Linux drivers/tty/serial/meson_uart.c
 */

#ifndef _UART_DEV_MESON_H_
#define _UART_DEV_MESON_H_

/* Register offsets */
#define	AML_UART_WFIFO		0x00	/* Write FIFO */
#define	AML_UART_RFIFO		0x04	/* Read FIFO */
#define	AML_UART_CONTROL	0x08	/* Control register */
#define	AML_UART_STATUS		0x0C	/* Status register */
#define	AML_UART_MISC		0x10	/* Misc / IRQ threshold */
#define	AML_UART_REG5		0x14	/* New baud rate register */

/* Control register bits */
#define	AML_UART_TX_EN		(1 << 12)
#define	AML_UART_RX_EN		(1 << 13)
#define	AML_UART_TWO_WIRE_EN	(1 << 15)
#define	AML_UART_STOP_BIT_LEN_SHIFT	16
#define	AML_UART_STOP_BIT_LEN_MASK	(0x3 << 16)
#define	AML_UART_STOP_BIT_1SB		(0x0 << 16)
#define	AML_UART_STOP_BIT_2SB		(0x1 << 16)
#define	AML_UART_DATA_LEN_SHIFT	20
#define	AML_UART_DATA_LEN_MASK	(0x3 << 20)
#define	AML_UART_DATA_LEN_8BIT	(0x0 << 20)
#define	AML_UART_DATA_LEN_7BIT	(0x1 << 20)
#define	AML_UART_DATA_LEN_6BIT	(0x2 << 20)
#define	AML_UART_DATA_LEN_5BIT	(0x3 << 20)
#define	AML_UART_TX_RST		(1 << 22)
#define	AML_UART_RX_RST		(1 << 23)
#define	AML_UART_CLEAR_ERR	(1 << 24)
#define	AML_UART_PARITY_EN	(1 << 26)
#define	AML_UART_RX_INT_EN	(1 << 27)
#define	AML_UART_TX_INT_EN	(1 << 28)
#define	AML_UART_PARITY_TYPE	(1 << 18)	/* 0=even, 1=odd */

/* Status register bits */
#define	AML_UART_PARITY_ERR	(1 << 16)
#define	AML_UART_FRAME_ERR	(1 << 17)
#define	AML_UART_TX_FIFO_WERR	(1 << 18)
#define	AML_UART_RX_EMPTY	(1 << 20)
#define	AML_UART_TX_FULL	(1 << 21)
#define	AML_UART_TX_EMPTY	(1 << 22)
#define	AML_UART_XMIT_BUSY	(1 << 25)
#define	AML_UART_ERR		(AML_UART_PARITY_ERR | AML_UART_FRAME_ERR)

/* RX FIFO byte count in status register (bits 7:0) */
#define	AML_UART_RX_COUNT_MASK	0x7F

/* MISC register: TX and RX interrupt threshold */
#define	AML_UART_XMIT_IRQ_SHIFT	8
#define	AML_UART_XMIT_IRQ_MASK		(0xFF << 8)
#define	AML_UART_RECV_IRQ_SHIFT		0
#define	AML_UART_RECV_IRQ_MASK		0xFF

/* REG5 (new baud rate register) bits */
#define	AML_UART_BAUD_MASK	0x7FFFFF	/* 23-bit divisor */
#define	AML_UART_BAUD_USE	(1 << 23)	/* Use new baud rate */
#define	AML_UART_BAUD_XTAL	(1 << 24)	/* Use crystal clock */
#define	AML_UART_BAUD_XTAL_DIV2 (1 << 27)	/* Crystal / 2 (else / 3) */

/* Default FIFO sizes */
#define	AML_UART_FIFO_SIZE	64
#define	AML_UART_PORT0_FIFO_SIZE 128	/* EE UART_0 has 128-byte FIFO */

/* Register access stride: Meson UART registers are 32-bit aligned */
#define	AML_UART_REG_SHIFT	2

#endif /* _UART_DEV_MESON_H_ */
