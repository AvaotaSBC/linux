/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * ac107.h -- AC107 ALSA SoC Audio driver
 *
 * Copyright (c) 2022 Allwinnertech Ltd.
 */

#ifndef __AC107_H
#define __AC107_H

/*** AC107 Codec Register Define ***/

/* Chip Reset */
#define CHIP_AUDIO_RST		0x00

/* Power Control */
#define PWR_CTRL1		0x01
#define PWR_CTRL2		0x02

/* PLL Configure Control */
#define PLL_CTRL1		0x10
#define PLL_CTRL2		0x11
#define PLL_CTRL3		0x12
#define PLL_CTRL4		0x13
#define PLL_CTRL5		0x14
#define PLL_CTRL6		0x16
#define PLL_CTRL7		0x17
#define PLL_LOCK_CTRL		0x18

/* System Clock Control */
#define SYSCLK_CTRL		0x20
#define MOD_CLK_EN		0x21
#define MOD_RST_CTRL		0x22

/* I2S Common Control  */
#define I2S_CTRL		0x30
#define I2S_BCLK_CTRL		0x31
#define I2S_LRCK_CTRL1		0x32
#define I2S_LRCK_CTRL2		0x33
#define I2S_FMT_CTRL1		0x34
#define I2S_FMT_CTRL2		0x35
#define I2S_FMT_CTRL3		0x36

/* I2S TX Control */
#define I2S_TX_CTRL1		0x38
#define I2S_TX_CTRL2		0x39
#define I2S_TX_CTRL3		0x3A
#define I2S_TX_CHMP_CTRL1	0x3C
#define I2S_TX_CHMP_CTRL2	0x3D

/* I2S RX Control */
#define I2S_RX_CTRL1		0x50
#define I2S_RX_CTRL2		0x51
#define I2S_RX_CTRL3		0x52
#define I2S_RX_CHMP_CTRL1	0x54
#define I2S_RX_CHMP_CTRL2	0x55

/* PDM Control */
#define PDM_CTRL		0x59

/* ADC Common Control */
#define ADC_SPRC		0x60
#define ADC_DIG_EN		0x61

#define HPF_EN			0x66

/* ADC Digital Channel Volume Control */
#define ADC1_DVOL_CTRL		0x70
#define ADC2_DVOL_CTRL		0x71

/* ADC Digital Mixer Source and Gain Control */
#define ADC1_DMIX_SRC		0x76
#define ADC2_DMIX_SRC		0x77

/* ADC Digital Debug Control */
#define ADC_DIG_DEBUG		0x7F

/* IO Function and Drive Control */
#define ADC_ANA_DEBUG1		0x80
#define ADC_ANA_DEBUG2		0x81
#define I2S_PADDRV_CTRL		0x82

/* ADC1 Analog Control */
#define ANA_ADC1_CTRL1		0xA0
#define ANA_ADC1_CTRL2		0xA1
#define ANA_ADC1_CTRL3		0xA2
#define ANA_ADC1_CTRL4		0xA3
#define ANA_ADC1_CTRL5		0xA4

/* ADC2 Analog Control */
#define ANA_ADC2_CTRL1		0xA5
#define ANA_ADC2_CTRL2		0xA6
#define ANA_ADC2_CTRL3		0xA7
#define ANA_ADC2_CTRL4		0xA8
#define ANA_ADC2_CTRL5		0xA9

/* ADC Dither Control */
#define ADC_DITHER_CTRL		0xAA
#define AC107_MAX_REG		ADC_DITHER_CTRL

/*** AC107 Codec Register Bit Define ***/

/* PWR_CTRL1 */
#define VREF_ENABLE		7
#define VREF_LPMODE		6
#define VREF_FSU_DISABLE	5
#define VREF_RESCTRL		3
#define IGEN_TRIM		0

/* PWR_CTRL2 */
#define VREF_SEL		7
#define MICBIAS2_EN		6
#define MICBIAS2_VCTRL		4
#define MICBIAS1_EN		2
#define MICBIAS1_VCTRL		0

/* PLL_CTRL1 */
#define PLL_IBIAS		4
#define PLL_NDET		3
#define PLL_LOCKED_STATUS	2
#define PLL_COM_EN		1
#define PLL_EN			0

/* PLL_CTRL2 */
#define PLL_PREDIV2		5
#define PLL_PREDIV1		0

/* PLL_CTRL3 */
#define PLL_LOOPDIV_MSB		0

/* PLL_CTRL4 */
#define PLL_LOOPDIV_LSB		0

/* PLL_CTRL5 */
#define PLL_POSTDIV2		5
#define PLL_POSTDIV1		0

/* PLL_CTRL6 */
#define PLL_LDO			6
#define PLL_CP			0

/* PLL_CTRL7 */
#define PLL_CAP			6
#define PLL_RES			4

/* PLL_LOCK_CTRL */
#define SYSCLK_HOLD_TIME	4
#define LOCK_LEVEL1		2
#define LOCK_LEVEL2		1
#define PLL_LOCK_EN		0

/* SYSCLK_CTRL */
#define PLLCLK_EN		7
#define PLLCLK_SRC		4
#define SYSCLK_SRC		2
#define SYSCLK_EN		0

/* MOD_CLK_EN & MOD_RST_CTRL */
#define I2S_RST			4
#define ADC_ANALOG		2
#define ADC_DIGITAL		1
#define I2S			0

/* I2S_CTRL */
#define BCLK_IOEN		7
#define LRCK_IOEN		6
#define MCLK_IOEN		5
#define SDO_EN			4
#define TXEN			2
#define RXEN			1
#define GEN			0

/* I2S_BCLK_CTRL */
#define EDGE_TRANSFER		5
#define BCLK_POLARITY		4
#define BCLKDIV			0

/* I2S_LRCK_CTRL1 */
#define LRCK_POLARITY		4
#define LRCK_PERIODH		0

/* I2S_LRCK_CTRL2 */
#define LRCK_PERIODL		0

/* I2S_FMT_CTRL1 */
#define ENCD_FMT		7
#define ENCD_SEL		6
#define MODE_SEL		4
#define TX_OFFSET		2
#define TX_SLOT_HIZ		1
#define TX_STATE		0

/* I2S_FMT_CTRL2 */
#define SLOT_WIDTH_SEL		4
#define SAMPLE_RESOLUTION	0

/* I2S_FMT_CTRL3 */
#define TX_MLS			7
#define SEXT			5
#define SDOUT_MUTE		3
#define LRCK_WIDTH		2
#define TX_PDM			0

/* I2S_TX_CTRL1 */
#define TX_CHSEL		0

/* I2S_TX_CTRL2 */
#define TX_CH8_EN		7
#define TX_CH7_EN		6
#define TX_CH6_EN		5
#define TX_CH5_EN		4
#define TX_CH4_EN		3
#define TX_CH3_EN		2
#define TX_CH2_EN		1
#define TX_CH1_EN		0

/* I2S_TX_CTRL3 */
#define TX_CH16_EN		7
#define TX_CH15_EN		6
#define TX_CH14_EN		5
#define TX_CH13_EN		4
#define TX_CH12_EN		3
#define TX_CH11_EN		2
#define TX_CH10_EN		1
#define TX_CH9_EN		0

/*  I2S_TX_CHMP_CTRL1 */
#define TX_CH8_MAP		7
#define TX_CH7_MAP		6
#define TX_CH6_MAP		5
#define TX_CH5_MAP		4
#define TX_CH4_MAP		3
#define TX_CH3_MAP		2
#define TX_CH2_MAP		1
#define TX_CH1_MAP		0

/*  I2S_TX_CHMP_CTRL2 */
#define TX_CH16_MAP		7
#define TX_CH15_MAP		6
#define TX_CH14_MAP		5
#define TX_CH13_MAP		4
#define TX_CH12_MAP		3
#define TX_CH11_MAP		2
#define TX_CH10_MAP		1
#define TX_CH9_MAP		0

/* I2S_RX_CTRL1 */
#define RX_CHSEL		0

/* I2S_RX_CTRL2 */
#define RX_CH8_EN		7
#define RX_CH7_EN		6
#define RX_CH6_EN		5
#define RX_CH5_EN		4
#define RX_CH4_EN		3
#define RX_CH3_EN		2
#define RX_CH2_EN		1
#define RX_CH1_EN		0

/* I2S_RX_CTRL3 */
#define RX_CH16_EN		7
#define RX_CH15_EN		6
#define RX_CH14_EN		5
#define RX_CH13_EN		4
#define RX_CH12_EN		3
#define RX_CH11_EN		2
#define RX_CH10_EN		1
#define RX_CH9_EN		0

/*  I2S_RX_CHMP_CTRL1 */
#define RX_CH8_MAP		7
#define RX_CH7_MAP		6
#define RX_CH6_MAP		5
#define RX_CH5_MAP		4
#define RX_CH4_MAP		3
#define RX_CH3_MAP		2
#define RX_CH2_MAP		1
#define RX_CH1_MAP		0

/*  I2S_RX_CHMP_CTRL2 */
#define RX_CH16_MAP		7
#define RX_CH15_MAP		6
#define RX_CH14_MAP		5
#define RX_CH13_MAP		4
#define RX_CH12_MAP		3
#define RX_CH11_MAP		2
#define RX_CH10_MAP		1
#define RX_CH9_MAP		0

/* PDM_CTRL */
#define PDM_TIMING		1
#define PDM_EN			0

/* ADC_SPRC */
#define ADC_FS_I2S		0

/* ADC_DIG_EN */
#define REQ_WIDTH		4
#define REQ_EN			3
#define DG_EN			2
#define ENAD2			1
#define ENAD1			0

/* HPF_EN */
#define DIG_ADC2_HPF_EN		1
#define DIG_ADC1_HPF_EN		0

/* ADC1_DVOL_CTRL */
#define DIG_ADCL1_VOL		0

/* ADC2_DVOL_CTRL */
#define DIG_ADCL2_VOL		0

/* ADC1_DMIX_SRC */
#define ADC1_ADC2_DMXL_GC	3
#define ADC1_ADC1_DMXL_GC	2
#define ADC1_ADC2_DMXL_SRC	1
#define ADC1_ADC1_DMXL_SRC	0

/* ADC2_DMIX_SRC */
#define ADC2_ADC2_DMXL_GC	3
#define ADC2_ADC1_DMXL_GC	2
#define ADC2_ADC2_DMXL_SRC	1
#define ADC2_ADC1_DMXL_SRC	0

/* ADC_DIG_DEBUG */
#define I2S_LPB_DEBUG		3
#define ADC_PTN_SEL		0

/* ADC_ANA_DEBUG1 */
#define DMIC_CLK_PAD_SEL	4
#define DMIC_DAT_PAD_SEL	0

/* ADC_ANA_DEBUG2 */
#define DEV_ID1_PAD_SEL		4
#define DEV_ID0_PAD_SEL		0

/* I2S_PADDRV_CTRL */
#define MCLK_DRV		6
#define BCLK_DRV		4
#define LRCK_DRV		2
#define SDOUT_DRV		0

/* ANA_ADC1_CTRL1 */
#define RX1_PGA_OI_CTRL		5
#define RX1_PGA_AMP_IB_SEL	2
#define RX1_PGA_IN_VCM_CTRL	0

/* ANA_ADC1_CTRL2 */
#define RX1_PGA_OI_NM_CTRL	3
#define RX1_PGA_NMAMP_IB_SEL	0

/* ANA_ADC1_CTRL3 */
#define RX1_PGA_CTRL_RCM	5
#define RX1_PGA_GAIN_CTRL	0

/* ANA_ADC1_CTRL4 */
#define RX1_DSM_OTA_IB_SEL	5
#define RX1_DSM_COMP_IB_SEL	2
#define RX1_DSM_OTA_CTRL	0

/* ANA_ADC1_CTRL5 */
#define RX1_GLOBAL_EN		6
#define RX1_DSM_DISABLE		5
#define RX1_DSM_DEMOFF		4
#define RX1_SEL_OUT_EDGE	3
#define RX1_DSM_VRP_LPMODE	2
#define RX1_DSM_VRP_OUTCTRL	0

/* ANA_ADC2_CTRL1 */
#define RX2_PGA_OI_CTRL		5
#define RX2_PGA_AMP_IB_SEL	2
#define RX2_PGA_IN_VCM_CTRL	0

/* ANA_ADC2_CTRL2 */
#define RX2_PGA_OI_NM_CTRL	3
#define RX2_PGA_NMAMP_IB_SEL	0

/* ANA_ADC2_CTRL3 */
#define RX2_PGA_CTRL_RCM	5
#define RX2_PGA_GAIN_CTRL	0

/* ANA_ADC2_CTRL4 */
#define RX2_DSM_OTA_IB_SEL	5
#define RX2_DSM_COMP_IB_SEL	2
#define RX2_DSM_OTA_CTRL	0

/* ANA_ADC2_CTRL5 */
#define RX2_GLOBAL_EN		6
#define RX2_DSM_DISABLE		5
#define RX2_DSM_DEMOFF		4
#define RX2_SEL_OUT_EDGE	3
#define RX2_DSM_VRP_LPMODE	2
#define RX2_DSM_VRP_OUTCTRL	0

/* ADC_DITHER_CTRL */
#define DSM_DITHER_CTRL		4
#define DSM_DITHER_EN		3
#define DSM_DITHER_LVL		0

/*** Config Value ***/
enum ac107_pllclk_src {
	PLLCLK_SRC_MCLK,
	PLLCLK_SRC_BCLK,
	PLLCLK_SRC_PDMCLK,
};

enum ac107_sysclk_src {
	SYSCLK_SRC_MCLK,
	SYSCLK_SRC_BCLK,
	SYSCLK_SRC_PLL,
};

struct ac107_data {
	unsigned int tx_pin_chmap0;
	unsigned int tx_pin_chmap1;
	unsigned int ch1_dig_vol;
	unsigned int ch2_dig_vol;
	unsigned int ch1_pga_gain;
	unsigned int ch2_pga_gain;

	unsigned int codec_id;
	unsigned int ecdn_mode;

	/* TODO:
	 * 1. HPF (default enable all)
	 * 2. ADC channel digital mixer gain set (default 0dB)
	 * 3. I2S pad drive control (default level 1)
	 * 4. Mbias voltage control (default 2.1V)
	 * 5. PDM mode (default I2S mode)
	 */

	enum ac107_pllclk_src pllclk_src;
	enum ac107_sysclk_src sysclk_src;

	unsigned int pcm_bit_first:1;
	unsigned int frame_sync_width;
};

#endif
