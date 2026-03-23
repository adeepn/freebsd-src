/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 JetHome. All rights reserved.
 *
 * Amlogic Meson GXL pin controller — SoC-specific data tables.
 *
 * Transcribed from Linux:
 *   drivers/pinctrl/meson/pinctrl-meson-gxl.c
 *   include/dt-bindings/gpio/meson-gxl-gpio.h
 */

#ifndef _MESON_PINCTRL_GXL_H_
#define _MESON_PINCTRL_GXL_H_

#include "meson_pinctrl.h"

/* -------------------------------------------------------------------- */
/* Pin number definitions                                                */
/* -------------------------------------------------------------------- */

/* Periphs domain (flat 0..99) */
#define	GPIOZ_0		0
#define	GPIOZ_1		1
#define	GPIOZ_2		2
#define	GPIOZ_3		3
#define	GPIOZ_4		4
#define	GPIOZ_5		5
#define	GPIOZ_6		6
#define	GPIOZ_7		7
#define	GPIOZ_8		8
#define	GPIOZ_9		9
#define	GPIOZ_10	10
#define	GPIOZ_11	11
#define	GPIOZ_12	12
#define	GPIOZ_13	13
#define	GPIOZ_14	14
#define	GPIOZ_15	15

#define	GPIOH_0		16
#define	GPIOH_1		17
#define	GPIOH_2		18
#define	GPIOH_3		19
#define	GPIOH_4		20
#define	GPIOH_5		21
#define	GPIOH_6		22
#define	GPIOH_7		23
#define	GPIOH_8		24
#define	GPIOH_9		25

#define	BOOT_0		26
#define	BOOT_1		27
#define	BOOT_2		28
#define	BOOT_3		29
#define	BOOT_4		30
#define	BOOT_5		31
#define	BOOT_6		32
#define	BOOT_7		33
#define	BOOT_8		34
#define	BOOT_9		35
#define	BOOT_10		36
#define	BOOT_11		37
#define	BOOT_12		38
#define	BOOT_13		39
#define	BOOT_14		40
#define	BOOT_15		41

#define	CARD_0		42
#define	CARD_1		43
#define	CARD_2		44
#define	CARD_3		45
#define	CARD_4		46
#define	CARD_5		47
#define	CARD_6		48

#define	GPIODV_0	49
#define	GPIODV_1	50
#define	GPIODV_2	51
#define	GPIODV_3	52
#define	GPIODV_4	53
#define	GPIODV_5	54
#define	GPIODV_6	55
#define	GPIODV_7	56
#define	GPIODV_8	57
#define	GPIODV_9	58
#define	GPIODV_10	59
#define	GPIODV_11	60
#define	GPIODV_12	61
#define	GPIODV_13	62
#define	GPIODV_14	63
#define	GPIODV_15	64
#define	GPIODV_16	65
#define	GPIODV_17	66
#define	GPIODV_18	67
#define	GPIODV_19	68
#define	GPIODV_20	69
#define	GPIODV_21	70
#define	GPIODV_22	71
#define	GPIODV_23	72
#define	GPIODV_24	73
#define	GPIODV_25	74
#define	GPIODV_26	75
#define	GPIODV_27	76
#define	GPIODV_28	77
#define	GPIODV_29	78

#define	GPIOX_0		79
#define	GPIOX_1		80
#define	GPIOX_2		81
#define	GPIOX_3		82
#define	GPIOX_4		83
#define	GPIOX_5		84
#define	GPIOX_6		85
#define	GPIOX_7		86
#define	GPIOX_8		87
#define	GPIOX_9		88
#define	GPIOX_10	89
#define	GPIOX_11	90
#define	GPIOX_12	91
#define	GPIOX_13	92
#define	GPIOX_14	93
#define	GPIOX_15	94
#define	GPIOX_16	95
#define	GPIOX_17	96
#define	GPIOX_18	97

#define	GPIOCLK_0	98
#define	GPIOCLK_1	99

/* Aobus domain (flat 0..10) */
#define	GPIOAO_0	0
#define	GPIOAO_1	1
#define	GPIOAO_2	2
#define	GPIOAO_3	3
#define	GPIOAO_4	4
#define	GPIOAO_5	5
#define	GPIOAO_6	6
#define	GPIOAO_7	7
#define	GPIOAO_8	8
#define	GPIOAO_9	9
#define	GPIO_TEST_N	10

/* -------------------------------------------------------------------- */
/* Periphs: pin arrays for mux groups                                    */
/* -------------------------------------------------------------------- */

static const unsigned int emmc_nand_d07_pins[] = {
	BOOT_0, BOOT_1, BOOT_2, BOOT_3, BOOT_4, BOOT_5, BOOT_6, BOOT_7,
};
static const unsigned int emmc_clk_pins[]	= { BOOT_8 };
static const unsigned int emmc_cmd_pins[]	= { BOOT_10 };
static const unsigned int emmc_ds_pins[]	= { BOOT_15 };

static const unsigned int nor_d_pins[]		= { BOOT_11 };
static const unsigned int nor_q_pins[]		= { BOOT_12 };
static const unsigned int nor_c_pins[]		= { BOOT_13 };
static const unsigned int nor_cs_pins[]		= { BOOT_15 };

static const unsigned int spi_mosi_pins[]	= { GPIOX_8 };
static const unsigned int spi_miso_pins[]	= { GPIOX_9 };
static const unsigned int spi_ss0_pins[]	= { GPIOX_10 };
static const unsigned int spi_sclk_pins[]	= { GPIOX_11 };

static const unsigned int sdcard_d0_pins[]	= { CARD_1 };
static const unsigned int sdcard_d1_pins[]	= { CARD_0 };
static const unsigned int sdcard_d2_pins[]	= { CARD_5 };
static const unsigned int sdcard_d3_pins[]	= { CARD_4 };
static const unsigned int sdcard_cmd_pins[]	= { CARD_3 };
static const unsigned int sdcard_clk_pins[]	= { CARD_2 };

static const unsigned int sdio_d0_pins[]	= { GPIOX_0 };
static const unsigned int sdio_d1_pins[]	= { GPIOX_1 };
static const unsigned int sdio_d2_pins[]	= { GPIOX_2 };
static const unsigned int sdio_d3_pins[]	= { GPIOX_3 };
static const unsigned int sdio_clk_pins[]	= { GPIOX_4 };
static const unsigned int sdio_cmd_pins[]	= { GPIOX_5 };
static const unsigned int sdio_irq_pins[]	= { GPIOX_7 };

static const unsigned int nand_ce0_pins[]	= { BOOT_8 };
static const unsigned int nand_ce1_pins[]	= { BOOT_9 };
static const unsigned int nand_rb0_pins[]	= { BOOT_10 };
static const unsigned int nand_ale_pins[]	= { BOOT_11 };
static const unsigned int nand_cle_pins[]	= { BOOT_12 };
static const unsigned int nand_wen_clk_pins[]	= { BOOT_13 };
static const unsigned int nand_ren_wr_pins[]	= { BOOT_14 };
static const unsigned int nand_dqs_pins[]	= { BOOT_15 };

static const unsigned int uart_tx_a_pins[]	= { GPIOX_12 };
static const unsigned int uart_rx_a_pins[]	= { GPIOX_13 };
static const unsigned int uart_cts_a_pins[]	= { GPIOX_14 };
static const unsigned int uart_rts_a_pins[]	= { GPIOX_15 };

static const unsigned int uart_tx_b_pins[]	= { GPIODV_24 };
static const unsigned int uart_rx_b_pins[]	= { GPIODV_25 };
static const unsigned int uart_cts_b_pins[]	= { GPIODV_26 };
static const unsigned int uart_rts_b_pins[]	= { GPIODV_27 };

static const unsigned int uart_tx_c_pins[]	= { GPIOX_8 };
static const unsigned int uart_rx_c_pins[]	= { GPIOX_9 };
static const unsigned int uart_cts_c_pins[]	= { GPIOX_10 };
static const unsigned int uart_rts_c_pins[]	= { GPIOX_11 };

static const unsigned int i2c_sck_a_pins[]	= { GPIODV_25 };
static const unsigned int i2c_sda_a_pins[]	= { GPIODV_24 };

static const unsigned int i2c_sck_b_pins[]	= { GPIODV_27 };
static const unsigned int i2c_sda_b_pins[]	= { GPIODV_26 };

static const unsigned int i2c_sck_c_pins[]	= { GPIODV_29 };
static const unsigned int i2c_sda_c_pins[]	= { GPIODV_28 };

static const unsigned int i2c_sck_c_dv19_pins[] = { GPIODV_19 };
static const unsigned int i2c_sda_c_dv18_pins[] = { GPIODV_18 };

static const unsigned int i2c_sck_d_pins[]	= { GPIOX_11 };
static const unsigned int i2c_sda_d_pins[]	= { GPIOX_10 };

static const unsigned int eth_mdio_pins[]	= { GPIOZ_0 };
static const unsigned int eth_mdc_pins[]	= { GPIOZ_1 };
static const unsigned int eth_clk_rx_clk_pins[] = { GPIOZ_2 };
static const unsigned int eth_rx_dv_pins[]	= { GPIOZ_3 };
static const unsigned int eth_rxd0_pins[]	= { GPIOZ_4 };
static const unsigned int eth_rxd1_pins[]	= { GPIOZ_5 };
static const unsigned int eth_rxd2_pins[]	= { GPIOZ_6 };
static const unsigned int eth_rxd3_pins[]	= { GPIOZ_7 };
static const unsigned int eth_rgmii_tx_clk_pins[] = { GPIOZ_8 };
static const unsigned int eth_tx_en_pins[]	= { GPIOZ_9 };
static const unsigned int eth_txd0_pins[]	= { GPIOZ_10 };
static const unsigned int eth_txd1_pins[]	= { GPIOZ_11 };
static const unsigned int eth_txd2_pins[]	= { GPIOZ_12 };
static const unsigned int eth_txd3_pins[]	= { GPIOZ_13 };

static const unsigned int pwm_a_pins[]		= { GPIOX_6 };

static const unsigned int pwm_b_pins[]		= { GPIODV_29 };

static const unsigned int pwm_c_pins[]		= { GPIOZ_15 };

static const unsigned int pwm_d_pins[]		= { GPIODV_28 };

static const unsigned int pwm_e_pins[]		= { GPIOX_16 };

static const unsigned int pwm_f_clk_pins[]	= { GPIOCLK_1 };
static const unsigned int pwm_f_x_pins[]	= { GPIOX_7 };

static const unsigned int hdmi_hpd_pins[]	= { GPIOH_0 };
static const unsigned int hdmi_sda_pins[]	= { GPIOH_1 };
static const unsigned int hdmi_scl_pins[]	= { GPIOH_2 };

static const unsigned int i2s_am_clk_pins[]	= { GPIOH_6 };
static const unsigned int i2s_out_ao_clk_pins[] = { GPIOH_7 };
static const unsigned int i2s_out_lr_clk_pins[] = { GPIOH_8 };
static const unsigned int i2s_out_ch01_pins[]	= { GPIOH_9 };
static const unsigned int i2s_out_ch23_z_pins[] = { GPIOZ_5 };
static const unsigned int i2s_out_ch45_z_pins[] = { GPIOZ_6 };
static const unsigned int i2s_out_ch67_z_pins[] = { GPIOZ_7 };

static const unsigned int spdif_out_h_pins[]	= { GPIOH_4 };

static const unsigned int eth_link_led_pins[]	= { GPIOZ_14 };
static const unsigned int eth_act_led_pins[]	= { GPIOZ_15 };

static const unsigned int tsin_a_d0_pins[]	= { GPIODV_0 };
static const unsigned int tsin_a_clk_pins[]	= { GPIODV_8 };
static const unsigned int tsin_a_sop_pins[]	= { GPIODV_9 };
static const unsigned int tsin_a_d_valid_pins[] = { GPIODV_10 };
static const unsigned int tsin_a_fail_pins[]	= { GPIODV_11 };
static const unsigned int tsin_a_dp_pins[] = {
	GPIODV_1, GPIODV_2, GPIODV_3, GPIODV_4,
	GPIODV_5, GPIODV_6, GPIODV_7,
};

static const unsigned int tsin_b_clk_pins[]	= { GPIOH_6 };
static const unsigned int tsin_b_d0_pins[]	= { GPIOH_7 };
static const unsigned int tsin_b_sop_pins[]	= { GPIOH_8 };
static const unsigned int tsin_b_d_valid_pins[] = { GPIOH_9 };

static const unsigned int tsin_b_fail_z4_pins[]     = { GPIOZ_4 };
static const unsigned int tsin_b_clk_z3_pins[]      = { GPIOZ_3 };
static const unsigned int tsin_b_d0_z2_pins[]       = { GPIOZ_2 };
static const unsigned int tsin_b_sop_z1_pins[]      = { GPIOZ_1 };
static const unsigned int tsin_b_d_valid_z0_pins[]  = { GPIOZ_0 };

/* -------------------------------------------------------------------- */
/* Periphs: group table                                                  */
/* -------------------------------------------------------------------- */

static const struct meson_pmx_group meson_gxl_periphs_groups[] = {
	/* GPIO groups (one per pin, no mux bit) */
	MESON_GPIO_GROUP(GPIOZ_0),
	MESON_GPIO_GROUP(GPIOZ_1),
	MESON_GPIO_GROUP(GPIOZ_2),
	MESON_GPIO_GROUP(GPIOZ_3),
	MESON_GPIO_GROUP(GPIOZ_4),
	MESON_GPIO_GROUP(GPIOZ_5),
	MESON_GPIO_GROUP(GPIOZ_6),
	MESON_GPIO_GROUP(GPIOZ_7),
	MESON_GPIO_GROUP(GPIOZ_8),
	MESON_GPIO_GROUP(GPIOZ_9),
	MESON_GPIO_GROUP(GPIOZ_10),
	MESON_GPIO_GROUP(GPIOZ_11),
	MESON_GPIO_GROUP(GPIOZ_12),
	MESON_GPIO_GROUP(GPIOZ_13),
	MESON_GPIO_GROUP(GPIOZ_14),
	MESON_GPIO_GROUP(GPIOZ_15),

	MESON_GPIO_GROUP(GPIOH_0),
	MESON_GPIO_GROUP(GPIOH_1),
	MESON_GPIO_GROUP(GPIOH_2),
	MESON_GPIO_GROUP(GPIOH_3),
	MESON_GPIO_GROUP(GPIOH_4),
	MESON_GPIO_GROUP(GPIOH_5),
	MESON_GPIO_GROUP(GPIOH_6),
	MESON_GPIO_GROUP(GPIOH_7),
	MESON_GPIO_GROUP(GPIOH_8),
	MESON_GPIO_GROUP(GPIOH_9),

	MESON_GPIO_GROUP(BOOT_0),
	MESON_GPIO_GROUP(BOOT_1),
	MESON_GPIO_GROUP(BOOT_2),
	MESON_GPIO_GROUP(BOOT_3),
	MESON_GPIO_GROUP(BOOT_4),
	MESON_GPIO_GROUP(BOOT_5),
	MESON_GPIO_GROUP(BOOT_6),
	MESON_GPIO_GROUP(BOOT_7),
	MESON_GPIO_GROUP(BOOT_8),
	MESON_GPIO_GROUP(BOOT_9),
	MESON_GPIO_GROUP(BOOT_10),
	MESON_GPIO_GROUP(BOOT_11),
	MESON_GPIO_GROUP(BOOT_12),
	MESON_GPIO_GROUP(BOOT_13),
	MESON_GPIO_GROUP(BOOT_14),
	MESON_GPIO_GROUP(BOOT_15),

	MESON_GPIO_GROUP(CARD_0),
	MESON_GPIO_GROUP(CARD_1),
	MESON_GPIO_GROUP(CARD_2),
	MESON_GPIO_GROUP(CARD_3),
	MESON_GPIO_GROUP(CARD_4),
	MESON_GPIO_GROUP(CARD_5),
	MESON_GPIO_GROUP(CARD_6),

	MESON_GPIO_GROUP(GPIODV_0),
	MESON_GPIO_GROUP(GPIODV_1),
	MESON_GPIO_GROUP(GPIODV_2),
	MESON_GPIO_GROUP(GPIODV_3),
	MESON_GPIO_GROUP(GPIODV_4),
	MESON_GPIO_GROUP(GPIODV_5),
	MESON_GPIO_GROUP(GPIODV_6),
	MESON_GPIO_GROUP(GPIODV_7),
	MESON_GPIO_GROUP(GPIODV_8),
	MESON_GPIO_GROUP(GPIODV_9),
	MESON_GPIO_GROUP(GPIODV_10),
	MESON_GPIO_GROUP(GPIODV_11),
	MESON_GPIO_GROUP(GPIODV_12),
	MESON_GPIO_GROUP(GPIODV_13),
	MESON_GPIO_GROUP(GPIODV_14),
	MESON_GPIO_GROUP(GPIODV_15),
	MESON_GPIO_GROUP(GPIODV_16),
	MESON_GPIO_GROUP(GPIODV_17),
	/* Note: GPIODV_18 omitted in Linux group table (used only via i2c_sda_c_dv18 mux) */
	MESON_GPIO_GROUP(GPIODV_19),
	MESON_GPIO_GROUP(GPIODV_20),
	MESON_GPIO_GROUP(GPIODV_21),
	MESON_GPIO_GROUP(GPIODV_22),
	MESON_GPIO_GROUP(GPIODV_23),
	MESON_GPIO_GROUP(GPIODV_24),
	MESON_GPIO_GROUP(GPIODV_25),
	MESON_GPIO_GROUP(GPIODV_26),
	MESON_GPIO_GROUP(GPIODV_27),
	MESON_GPIO_GROUP(GPIODV_28),
	MESON_GPIO_GROUP(GPIODV_29),

	MESON_GPIO_GROUP(GPIOX_0),
	MESON_GPIO_GROUP(GPIOX_1),
	MESON_GPIO_GROUP(GPIOX_2),
	MESON_GPIO_GROUP(GPIOX_3),
	MESON_GPIO_GROUP(GPIOX_4),
	MESON_GPIO_GROUP(GPIOX_5),
	MESON_GPIO_GROUP(GPIOX_6),
	MESON_GPIO_GROUP(GPIOX_7),
	MESON_GPIO_GROUP(GPIOX_8),
	MESON_GPIO_GROUP(GPIOX_9),
	MESON_GPIO_GROUP(GPIOX_10),
	MESON_GPIO_GROUP(GPIOX_11),
	MESON_GPIO_GROUP(GPIOX_12),
	MESON_GPIO_GROUP(GPIOX_13),
	MESON_GPIO_GROUP(GPIOX_14),
	MESON_GPIO_GROUP(GPIOX_15),
	MESON_GPIO_GROUP(GPIOX_16),
	MESON_GPIO_GROUP(GPIOX_17),
	MESON_GPIO_GROUP(GPIOX_18),

	MESON_GPIO_GROUP(GPIOCLK_0),
	MESON_GPIO_GROUP(GPIOCLK_1),

	MESON_GPIO_GROUP(GPIO_TEST_N),

	/* Bank X */
	MESON_GROUP(i2c_sda_d,		5,	5),
	MESON_GROUP(i2c_sck_d,		5,	4),
	MESON_GROUP(sdio_d0,		5,	31),
	MESON_GROUP(sdio_d1,		5,	30),
	MESON_GROUP(sdio_d2,		5,	29),
	MESON_GROUP(sdio_d3,		5,	28),
	MESON_GROUP(sdio_clk,		5,	27),
	MESON_GROUP(sdio_cmd,		5,	26),
	MESON_GROUP(sdio_irq,		5,	24),
	MESON_GROUP(uart_tx_a,		5,	19),
	MESON_GROUP(uart_rx_a,		5,	18),
	MESON_GROUP(uart_cts_a,		5,	17),
	MESON_GROUP(uart_rts_a,		5,	16),
	MESON_GROUP(uart_tx_c,		5,	13),
	MESON_GROUP(uart_rx_c,		5,	12),
	MESON_GROUP(uart_cts_c,		5,	11),
	MESON_GROUP(uart_rts_c,		5,	10),
	MESON_GROUP(pwm_a,		5,	25),
	MESON_GROUP(pwm_e,		5,	15),
	MESON_GROUP(pwm_f_x,		5,	14),
	MESON_GROUP(spi_mosi,		5,	3),
	MESON_GROUP(spi_miso,		5,	2),
	MESON_GROUP(spi_ss0,		5,	1),
	MESON_GROUP(spi_sclk,		5,	0),

	/* Bank Z */
	MESON_GROUP(eth_mdio,		4,	23),
	MESON_GROUP(eth_mdc,		4,	22),
	MESON_GROUP(eth_clk_rx_clk,	4,	21),
	MESON_GROUP(eth_rx_dv,		4,	20),
	MESON_GROUP(eth_rxd0,		4,	19),
	MESON_GROUP(eth_rxd1,		4,	18),
	MESON_GROUP(eth_rxd2,		4,	17),
	MESON_GROUP(eth_rxd3,		4,	16),
	MESON_GROUP(eth_rgmii_tx_clk,	4,	15),
	MESON_GROUP(eth_tx_en,		4,	14),
	MESON_GROUP(eth_txd0,		4,	13),
	MESON_GROUP(eth_txd1,		4,	12),
	MESON_GROUP(eth_txd2,		4,	11),
	MESON_GROUP(eth_txd3,		4,	10),
	MESON_GROUP(tsin_b_fail_z4,	3,	15),
	MESON_GROUP(tsin_b_clk_z3,	3,	16),
	MESON_GROUP(tsin_b_d0_z2,	3,	17),
	MESON_GROUP(tsin_b_sop_z1,	3,	18),
	MESON_GROUP(tsin_b_d_valid_z0,	3,	19),
	MESON_GROUP(pwm_c,		3,	20),
	MESON_GROUP(i2s_out_ch23_z,	3,	26),
	MESON_GROUP(i2s_out_ch45_z,	3,	25),
	MESON_GROUP(i2s_out_ch67_z,	3,	24),
	MESON_GROUP(eth_link_led,	4,	25),
	MESON_GROUP(eth_act_led,	4,	24),

	/* Bank H */
	MESON_GROUP(hdmi_hpd,		6,	31),
	MESON_GROUP(hdmi_sda,		6,	30),
	MESON_GROUP(hdmi_scl,		6,	29),
	MESON_GROUP(i2s_am_clk,	6,	26),
	MESON_GROUP(i2s_out_ao_clk,	6,	25),
	MESON_GROUP(i2s_out_lr_clk,	6,	24),
	MESON_GROUP(i2s_out_ch01,	6,	23),
	MESON_GROUP(spdif_out_h,	6,	28),
	MESON_GROUP(tsin_b_d0,		6,	17),
	MESON_GROUP(tsin_b_sop,	6,	18),
	MESON_GROUP(tsin_b_d_valid,	6,	19),
	MESON_GROUP(tsin_b_clk,	6,	20),

	/* Bank DV */
	MESON_GROUP(uart_tx_b,		2,	16),
	MESON_GROUP(uart_rx_b,		2,	15),
	MESON_GROUP(uart_cts_b,		2,	14),
	MESON_GROUP(uart_rts_b,		2,	13),
	MESON_GROUP(i2c_sda_c_dv18,	1,	17),
	MESON_GROUP(i2c_sck_c_dv19,	1,	16),
	MESON_GROUP(i2c_sda_a,		1,	15),
	MESON_GROUP(i2c_sck_a,		1,	14),
	MESON_GROUP(i2c_sda_b,		1,	13),
	MESON_GROUP(i2c_sck_b,		1,	12),
	MESON_GROUP(i2c_sda_c,		1,	11),
	MESON_GROUP(i2c_sck_c,		1,	10),
	MESON_GROUP(pwm_b,		2,	11),
	MESON_GROUP(pwm_d,		2,	12),
	MESON_GROUP(tsin_a_d0,		2,	4),
	MESON_GROUP(tsin_a_dp,		2,	3),
	MESON_GROUP(tsin_a_clk,		2,	2),
	MESON_GROUP(tsin_a_sop,		2,	1),
	MESON_GROUP(tsin_a_d_valid,	2,	0),
	MESON_GROUP(tsin_a_fail,	1,	31),

	/* Bank BOOT */
	MESON_GROUP(emmc_nand_d07,	7,	31),
	MESON_GROUP(emmc_clk,		7,	30),
	MESON_GROUP(emmc_cmd,		7,	29),
	MESON_GROUP(emmc_ds,		7,	28),
	MESON_GROUP(nor_d,		7,	13),
	MESON_GROUP(nor_q,		7,	12),
	MESON_GROUP(nor_c,		7,	11),
	MESON_GROUP(nor_cs,		7,	10),
	MESON_GROUP(nand_ce0,		7,	7),
	MESON_GROUP(nand_ce1,		7,	6),
	MESON_GROUP(nand_rb0,		7,	5),
	MESON_GROUP(nand_ale,		7,	4),
	MESON_GROUP(nand_cle,		7,	3),
	MESON_GROUP(nand_wen_clk,	7,	2),
	MESON_GROUP(nand_ren_wr,	7,	1),
	MESON_GROUP(nand_dqs,		7,	0),

	/* Bank CARD */
	MESON_GROUP(sdcard_d1,		6,	5),
	MESON_GROUP(sdcard_d0,		6,	4),
	MESON_GROUP(sdcard_d3,		6,	1),
	MESON_GROUP(sdcard_d2,		6,	0),
	MESON_GROUP(sdcard_cmd,		6,	2),
	MESON_GROUP(sdcard_clk,		6,	3),

	/* Bank CLK */
	MESON_GROUP(pwm_f_clk,		8,	30),
};

/* -------------------------------------------------------------------- */
/* Periphs: function group arrays                                        */
/* -------------------------------------------------------------------- */

static const char * const gpio_periphs_groups[] = {
	"GPIOZ_0", "GPIOZ_1", "GPIOZ_2", "GPIOZ_3", "GPIOZ_4",
	"GPIOZ_5", "GPIOZ_6", "GPIOZ_7", "GPIOZ_8", "GPIOZ_9",
	"GPIOZ_10", "GPIOZ_11", "GPIOZ_12", "GPIOZ_13", "GPIOZ_14",
	"GPIOZ_15",

	"GPIOH_0", "GPIOH_1", "GPIOH_2", "GPIOH_3", "GPIOH_4",
	"GPIOH_5", "GPIOH_6", "GPIOH_7", "GPIOH_8", "GPIOH_9",

	"BOOT_0", "BOOT_1", "BOOT_2", "BOOT_3", "BOOT_4",
	"BOOT_5", "BOOT_6", "BOOT_7", "BOOT_8", "BOOT_9",
	"BOOT_10", "BOOT_11", "BOOT_12", "BOOT_13", "BOOT_14",
	"BOOT_15",

	"CARD_0", "CARD_1", "CARD_2", "CARD_3", "CARD_4",
	"CARD_5", "CARD_6",

	"GPIODV_0", "GPIODV_1", "GPIODV_2", "GPIODV_3", "GPIODV_4",
	"GPIODV_5", "GPIODV_6", "GPIODV_7", "GPIODV_8", "GPIODV_9",
	"GPIODV_10", "GPIODV_11", "GPIODV_12", "GPIODV_13", "GPIODV_14",
	"GPIODV_15", "GPIODV_16", "GPIODV_17", "GPIODV_18", "GPIODV_19",
	"GPIODV_20", "GPIODV_21", "GPIODV_22", "GPIODV_23", "GPIODV_24",
	"GPIODV_25", "GPIODV_26", "GPIODV_27", "GPIODV_28", "GPIODV_29",

	"GPIOX_0", "GPIOX_1", "GPIOX_2", "GPIOX_3", "GPIOX_4",
	"GPIOX_5", "GPIOX_6", "GPIOX_7", "GPIOX_8", "GPIOX_9",
	"GPIOX_10", "GPIOX_11", "GPIOX_12", "GPIOX_13", "GPIOX_14",
	"GPIOX_15", "GPIOX_16", "GPIOX_17", "GPIOX_18",
};

static const char * const emmc_groups[] = {
	"emmc_nand_d07", "emmc_clk", "emmc_cmd", "emmc_ds",
};

static const char * const nor_groups[] = {
	"nor_d", "nor_q", "nor_c", "nor_cs",
};

static const char * const spi_groups[] = {
	"spi_mosi", "spi_miso", "spi_ss0", "spi_sclk",
};

static const char * const sdcard_groups[] = {
	"sdcard_d0", "sdcard_d1", "sdcard_d2", "sdcard_d3",
	"sdcard_cmd", "sdcard_clk",
};

static const char * const sdio_groups[] = {
	"sdio_d0", "sdio_d1", "sdio_d2", "sdio_d3",
	"sdio_cmd", "sdio_clk", "sdio_irq",
};

static const char * const nand_groups[] = {
	"emmc_nand_d07", "nand_ce0", "nand_ce1", "nand_rb0", "nand_ale",
	"nand_cle", "nand_wen_clk", "nand_ren_wr", "nand_dqs",
};

static const char * const uart_a_groups[] = {
	"uart_tx_a", "uart_rx_a", "uart_cts_a", "uart_rts_a",
};

static const char * const uart_b_groups[] = {
	"uart_tx_b", "uart_rx_b", "uart_cts_b", "uart_rts_b",
};

static const char * const uart_c_groups[] = {
	"uart_tx_c", "uart_rx_c", "uart_cts_c", "uart_rts_c",
};

static const char * const i2c_a_groups[] = {
	"i2c_sck_a", "i2c_sda_a",
};

static const char * const i2c_b_groups[] = {
	"i2c_sck_b", "i2c_sda_b",
};

static const char * const i2c_c_groups[] = {
	"i2c_sck_c", "i2c_sda_c", "i2c_sda_c_dv18", "i2c_sck_c_dv19",
};

static const char * const i2c_d_groups[] = {
	"i2c_sck_d", "i2c_sda_d",
};

static const char * const eth_groups[] = {
	"eth_mdio", "eth_mdc", "eth_clk_rx_clk", "eth_rx_dv",
	"eth_rxd0", "eth_rxd1", "eth_rxd2", "eth_rxd3",
	"eth_rgmii_tx_clk", "eth_tx_en",
	"eth_txd0", "eth_txd1", "eth_txd2", "eth_txd3",
};

static const char * const pwm_a_groups[] = {
	"pwm_a",
};

static const char * const pwm_b_groups[] = {
	"pwm_b",
};

static const char * const pwm_c_groups[] = {
	"pwm_c",
};

static const char * const pwm_d_groups[] = {
	"pwm_d",
};

static const char * const pwm_e_groups[] = {
	"pwm_e",
};

static const char * const pwm_f_groups[] = {
	"pwm_f_clk", "pwm_f_x",
};

static const char * const hdmi_hpd_groups[] = {
	"hdmi_hpd",
};

static const char * const hdmi_i2c_groups[] = {
	"hdmi_sda", "hdmi_scl",
};

static const char * const i2s_out_groups[] = {
	"i2s_am_clk", "i2s_out_ao_clk", "i2s_out_lr_clk",
	"i2s_out_ch01", "i2s_out_ch23_z", "i2s_out_ch45_z", "i2s_out_ch67_z",
};

static const char * const spdif_out_groups[] = {
	"spdif_out_h",
};

static const char * const eth_led_groups[] = {
	"eth_link_led", "eth_act_led",
};

static const char * const tsin_a_groups[] = {
	"tsin_a_clk", "tsin_a_sop",
	"tsin_a_d_valid", "tsin_a_d0",
	"tsin_a_dp", "tsin_a_fail",
};

static const char * const tsin_b_groups[] = {
	"tsin_b_clk", "tsin_b_sop", "tsin_b_d_valid", "tsin_b_d0",
	"tsin_b_clk_z3", "tsin_b_sop_z1", "tsin_b_d_valid_z0", "tsin_b_d0_z2",
	"tsin_b_fail_z4",
};

/* -------------------------------------------------------------------- */
/* Periphs: function table                                               */
/* -------------------------------------------------------------------- */

static const struct meson_pmx_func meson_gxl_periphs_functions[] = {
	MESON_FUNCTION(gpio_periphs),
	MESON_FUNCTION(emmc),
	MESON_FUNCTION(nor),
	MESON_FUNCTION(spi),
	MESON_FUNCTION(sdcard),
	MESON_FUNCTION(sdio),
	MESON_FUNCTION(nand),
	MESON_FUNCTION(uart_a),
	MESON_FUNCTION(uart_b),
	MESON_FUNCTION(uart_c),
	MESON_FUNCTION(i2c_a),
	MESON_FUNCTION(i2c_b),
	MESON_FUNCTION(i2c_c),
	MESON_FUNCTION(i2c_d),
	MESON_FUNCTION(eth),
	MESON_FUNCTION(pwm_a),
	MESON_FUNCTION(pwm_b),
	MESON_FUNCTION(pwm_c),
	MESON_FUNCTION(pwm_d),
	MESON_FUNCTION(pwm_e),
	MESON_FUNCTION(pwm_f),
	MESON_FUNCTION(hdmi_hpd),
	MESON_FUNCTION(hdmi_i2c),
	MESON_FUNCTION(i2s_out),
	MESON_FUNCTION(spdif_out),
	MESON_FUNCTION(eth_led),
	MESON_FUNCTION(tsin_a),
	MESON_FUNCTION(tsin_b),
};

/* -------------------------------------------------------------------- */
/* Periphs: bank table                                                   */
/* -------------------------------------------------------------------- */

static const struct meson_gpio_bank meson_gxl_periphs_banks[] = {
	/*           name    prefix     first last  pe_r pe_b p_r p_b d_r d_b  o_r  o_b  i_r  i_b */
	MESON_BANK("X",    "GPIOX",    79,   97,   4,0,  4,0,  12,0, 13,0,  14,0),
	MESON_BANK("DV",   "GPIODV",   49,   78,   0,0,  0,0,   0,0,  1,0,   2,0),
	MESON_BANK("H",    "GPIOH",    16,   25,   1,20, 1,20,  3,20, 4,20,  5,20),
	MESON_BANK("Z",    "GPIOZ",     0,   15,   3,0,  3,0,   9,0, 10,0,  11,0),
	MESON_BANK("CARD", "CARD",     42,   48,   2,20, 2,20,  6,20, 7,20,  8,20),
	MESON_BANK("BOOT", "BOOT",     26,   41,   2,0,  2,0,   6,0,  7,0,   8,0),
	MESON_BANK("CLK",  "GPIOCLK",  98,   99,   3,28, 3,28,  9,28,10,28, 11,28),
};

/* -------------------------------------------------------------------- */
/* Aobus: pin arrays for mux groups                                      */
/* -------------------------------------------------------------------- */

static const unsigned int uart_tx_ao_a_pins[]      = { GPIOAO_0 };
static const unsigned int uart_rx_ao_a_pins[]      = { GPIOAO_1 };
static const unsigned int uart_tx_ao_b_0_pins[]    = { GPIOAO_0 };
static const unsigned int uart_rx_ao_b_1_pins[]    = { GPIOAO_1 };
static const unsigned int uart_cts_ao_a_pins[]     = { GPIOAO_2 };
static const unsigned int uart_rts_ao_a_pins[]     = { GPIOAO_3 };
static const unsigned int uart_tx_ao_b_pins[]      = { GPIOAO_4 };
static const unsigned int uart_rx_ao_b_pins[]      = { GPIOAO_5 };
static const unsigned int uart_cts_ao_b_pins[]     = { GPIOAO_2 };
static const unsigned int uart_rts_ao_b_pins[]     = { GPIOAO_3 };

static const unsigned int i2c_sck_ao_pins[]        = { GPIOAO_4 };
static const unsigned int i2c_sda_ao_pins[]        = { GPIOAO_5 };
static const unsigned int i2c_slave_sck_ao_pins[]  = { GPIOAO_4 };
static const unsigned int i2c_slave_sda_ao_pins[]  = { GPIOAO_5 };

static const unsigned int remote_input_ao_pins[]   = { GPIOAO_7 };

static const unsigned int pwm_ao_a_3_pins[]        = { GPIOAO_3 };
static const unsigned int pwm_ao_a_8_pins[]        = { GPIOAO_8 };

static const unsigned int pwm_ao_b_pins[]          = { GPIOAO_9 };
static const unsigned int pwm_ao_b_6_pins[]        = { GPIOAO_6 };

static const unsigned int i2s_out_ch23_ao_pins[]   = { GPIOAO_8 };
static const unsigned int i2s_out_ch45_ao_pins[]   = { GPIOAO_9 };
static const unsigned int i2s_out_ch67_ao_pins[]   = { GPIO_TEST_N };

static const unsigned int spdif_out_ao_6_pins[]    = { GPIOAO_6 };
static const unsigned int spdif_out_ao_9_pins[]    = { GPIOAO_9 };

static const unsigned int ao_cec_pins[]            = { GPIOAO_8 };
static const unsigned int ee_cec_pins[]            = { GPIOAO_8 };

/* -------------------------------------------------------------------- */
/* Aobus: group table                                                    */
/* -------------------------------------------------------------------- */

static const struct meson_pmx_group meson_gxl_aobus_groups[] = {
	MESON_GPIO_GROUP(GPIOAO_0),
	MESON_GPIO_GROUP(GPIOAO_1),
	MESON_GPIO_GROUP(GPIOAO_2),
	MESON_GPIO_GROUP(GPIOAO_3),
	MESON_GPIO_GROUP(GPIOAO_4),
	MESON_GPIO_GROUP(GPIOAO_5),
	MESON_GPIO_GROUP(GPIOAO_6),
	MESON_GPIO_GROUP(GPIOAO_7),
	MESON_GPIO_GROUP(GPIOAO_8),
	MESON_GPIO_GROUP(GPIOAO_9),

	/* bank AO */
	MESON_GROUP(uart_tx_ao_b_0,	0,	26),
	MESON_GROUP(uart_rx_ao_b_1,	0,	25),
	MESON_GROUP(uart_tx_ao_b,	0,	24),
	MESON_GROUP(uart_rx_ao_b,	0,	23),
	MESON_GROUP(uart_tx_ao_a,	0,	12),
	MESON_GROUP(uart_rx_ao_a,	0,	11),
	MESON_GROUP(uart_cts_ao_a,	0,	10),
	MESON_GROUP(uart_rts_ao_a,	0,	9),
	MESON_GROUP(uart_cts_ao_b,	0,	8),
	MESON_GROUP(uart_rts_ao_b,	0,	7),
	MESON_GROUP(i2c_sck_ao,	0,	6),
	MESON_GROUP(i2c_sda_ao,	0,	5),
	MESON_GROUP(i2c_slave_sck_ao,	0,	2),
	MESON_GROUP(i2c_slave_sda_ao,	0,	1),
	MESON_GROUP(remote_input_ao,	0,	0),
	MESON_GROUP(pwm_ao_a_3,	0,	22),
	MESON_GROUP(pwm_ao_b_6,	0,	18),
	MESON_GROUP(pwm_ao_a_8,	0,	17),
	MESON_GROUP(pwm_ao_b,		0,	3),
	MESON_GROUP(i2s_out_ch23_ao,	1,	0),
	MESON_GROUP(i2s_out_ch45_ao,	1,	1),
	MESON_GROUP(spdif_out_ao_6,	0,	16),
	MESON_GROUP(spdif_out_ao_9,	0,	4),
	MESON_GROUP(ao_cec,		0,	15),
	MESON_GROUP(ee_cec,		0,	14),

	/* test n pin */
	MESON_GROUP(i2s_out_ch67_ao,	1,	2),
};

/* -------------------------------------------------------------------- */
/* Aobus: function group arrays                                          */
/* -------------------------------------------------------------------- */

static const char * const gpio_aobus_groups[] = {
	"GPIOAO_0", "GPIOAO_1", "GPIOAO_2", "GPIOAO_3", "GPIOAO_4",
	"GPIOAO_5", "GPIOAO_6", "GPIOAO_7", "GPIOAO_8", "GPIOAO_9",

	"GPIO_TEST_N",
};

static const char * const uart_ao_groups[] = {
	"uart_tx_ao_a", "uart_rx_ao_a", "uart_cts_ao_a", "uart_rts_ao_a",
};

static const char * const uart_ao_b_groups[] = {
	"uart_tx_ao_b", "uart_rx_ao_b", "uart_cts_ao_b", "uart_rts_ao_b",
	"uart_tx_ao_b_0", "uart_rx_ao_b_1",
};

static const char * const i2c_ao_groups[] = {
	"i2c_sck_ao", "i2c_sda_ao",
};

static const char * const i2c_slave_ao_groups[] = {
	"i2c_slave_sck_ao", "i2c_slave_sda_ao",
};

static const char * const remote_input_ao_groups[] = {
	"remote_input_ao",
};

static const char * const pwm_ao_a_groups[] = {
	"pwm_ao_a_3", "pwm_ao_a_8",
};

static const char * const pwm_ao_b_groups[] = {
	"pwm_ao_b", "pwm_ao_b_6",
};

static const char * const i2s_out_ao_groups[] = {
	"i2s_out_ch23_ao", "i2s_out_ch45_ao", "i2s_out_ch67_ao",
};

static const char * const spdif_out_ao_groups[] = {
	"spdif_out_ao_6", "spdif_out_ao_9",
};

static const char * const cec_ao_groups[] = {
	"ao_cec", "ee_cec",
};

/* -------------------------------------------------------------------- */
/* Aobus: function table                                                 */
/* -------------------------------------------------------------------- */

static const struct meson_pmx_func meson_gxl_aobus_functions[] = {
	MESON_FUNCTION(gpio_aobus),
	MESON_FUNCTION(uart_ao),
	MESON_FUNCTION(uart_ao_b),
	MESON_FUNCTION(i2c_ao),
	MESON_FUNCTION(i2c_slave_ao),
	MESON_FUNCTION(remote_input_ao),
	MESON_FUNCTION(pwm_ao_a),
	MESON_FUNCTION(pwm_ao_b),
	MESON_FUNCTION(i2s_out_ao),
	MESON_FUNCTION(spdif_out_ao),
	MESON_FUNCTION(cec_ao),
};

/* -------------------------------------------------------------------- */
/* Aobus: bank table                                                     */
/* -------------------------------------------------------------------- */

static const struct meson_gpio_bank meson_gxl_aobus_banks[] = {
	/*           name   prefix     first last  pe_r pe_b p_r p_b d_r d_b o_r  o_b i_r i_b */
	MESON_BANK("AO",  "GPIOAO",   0,    9,    0,16, 0,0,  0,0,  0,16, 1,0),
};

/* -------------------------------------------------------------------- */
/* Top-level data descriptors                                            */
/* -------------------------------------------------------------------- */

static const struct meson_pinctrl_data meson_gxl_periphs_data = {
	.name		= "periphs-banks",
	.banks		= meson_gxl_periphs_banks,
	.num_banks	= nitems(meson_gxl_periphs_banks),
	.groups		= meson_gxl_periphs_groups,
	.num_groups	= nitems(meson_gxl_periphs_groups),
	.funcs		= meson_gxl_periphs_functions,
	.num_funcs	= nitems(meson_gxl_periphs_functions),
	.num_pins	= 100,
	.aobus		= false,
};

static const struct meson_pinctrl_data meson_gxl_aobus_data = {
	.name		= "aobus-banks",
	.banks		= meson_gxl_aobus_banks,
	.num_banks	= nitems(meson_gxl_aobus_banks),
	.groups		= meson_gxl_aobus_groups,
	.num_groups	= nitems(meson_gxl_aobus_groups),
	.funcs		= meson_gxl_aobus_functions,
	.num_funcs	= nitems(meson_gxl_aobus_functions),
	.num_pins	= 11,
	.aobus		= true,
};

#endif /* _MESON_PINCTRL_GXL_H_ */
