/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner's ALSA SoC Audio driver
 *
 * Copyright (c) 2022, Dby <dby@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __SND_SUNXI_AHUB_DAM_H
#define __SND_SUNXI_AHUB_DAM_H

/* SUNXI Audio Hub registers list */
#define SUNXI_AHUB_CTL				0x00
#define SUNXI_AHUB_VER				0x04
#define SUNXI_AHUB_RST				0x08
#define SUNXI_AHUB_GAT				0x0c

#define SUNXI_AHUB_APBIF_TX_CTL(n)		(0x10 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_TX_IRQ_CTL(n)		(0x14 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_TX_IRQ_STA(n)		(0x18 + ((n) * 0x30))

#define SUNXI_AHUB_APBIF_TXFIFO_CTL(n)		(0x20 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_TXFIFO_STA(n)		(0x24 + ((n) * 0x30))

#define SUNXI_AHUB_APBIF_TXFIFO(n)		(0x30 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_TXFIFO_CNT(n)		(0x34 + ((n) * 0x30))

#define SUNXI_AHUB_APBIF_RX_CTL(n)		(0x100 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_RX_IRQ_CTL(n)		(0x104 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_RX_IRQ_STA(n)		(0x108 + ((n) * 0x30))

#define SUNXI_AHUB_APBIF_RXFIFO_CTL(n)		(0x110 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_RXFIFO_STA(n)		(0x114 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_RXFIFO_CONT(n)		(0x118 + ((n) * 0x30))

#define SUNXI_AHUB_APBIF_RXFIFO(n)		(0x120 + ((n) * 0x30))
#define SUNXI_AHUB_APBIF_RXFIFO_CNT(n)		(0x124 + ((n) * 0x30))

#define SUNXI_AHUB_I2S_CTL(n)			(0x200 + ((n) << 8))
#define SUNXI_AHUB_I2S_FMT0(n)			(0x204 + ((n) << 8))
#define SUNXI_AHUB_I2S_FMT1(n)			(0x208 + ((n) << 8))
#define SUNXI_AHUB_I2S_CLKD(n)			(0x20c + ((n) << 8))

#define SUNXI_AHUB_I2S_RXCONT(n)		(0x220 + ((n) << 8))
#define SUNXI_AHUB_I2S_CHCFG(n)			(0x224 + ((n) << 8))
#define SUNXI_AHUB_I2S_IRQ_CTL(n)		(0x228 + ((n) << 8))
#define SUNXI_AHUB_I2S_IRQ_STA(n)		(0x22C + ((n) << 8))
#define SUNXI_AHUB_I2S_OUT_SLOT(n, m)		(0x230 + ((n) << 8) + ((m) << 4))
#define SUNXI_AHUB_I2S_OUT_CHMAP0(n, m)		(0x234 + ((n) << 8) + ((m) << 4))
#define SUNXI_AHUB_I2S_OUT_CHMAP1(n, m)		(0x238 + ((n) << 8) + ((m) << 4))

#define SUNXI_AHUB_I2S_IN_SLOT(n)		(0x270 + ((n) << 8))
#define SUNXI_AHUB_I2S_IN_CHMAP0(n)		(0x274 + ((n) << 8))
#define SUNXI_AHUB_I2S_IN_CHMAP1(n)		(0x278 + ((n) << 8))
#define SUNXI_AHUB_I2S_IN_CHMAP2(n)		(0x27C + ((n) << 8))
#define SUNXI_AHUB_I2S_IN_CHMAP3(n)		(0x280 + ((n) << 8))

#define SUNXI_AHUB_DAM_CTL(n)			(0xA00 + ((n) << 7))

#define SUNXI_AHUB_DAM_RX0_SRC(n)		(0xA10 + ((n) << 7))
#define SUNXI_AHUB_DAM_RX1_SRC(n)		(0xA14 + ((n) << 7))
#define SUNXI_AHUB_DAM_RX2_SRC(n)		(0xA18 + ((n) << 7))

#define SUNXI_AHUB_DAM_MIX_CTL0(n)		(0xA30 + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL1(n)		(0xA34 + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL2(n)		(0xA38 + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL3(n)		(0xA3C + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL4(n)		(0xA40 + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL5(n)		(0xA44 + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL6(n)		(0xA48 + ((n) << 7))
#define SUNXI_AHUB_DAM_MIX_CTL7(n)		(0xA4C + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL0(n)		(0xA50 + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL1(n)		(0xA54 + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL2(n)		(0xA58 + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL3(n)		(0xA5C + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL4(n)		(0xA60 + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL5(n)		(0xA64 + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL6(n)		(0xA68 + ((n) << 7))
#define SUNXI_AHUB_DAM_GAIN_CTL7(n)		(0xA6C + ((n) << 7))

#define SUNXI_AHUB_MAX_REG			SUNXI_AHUB_DAM_GAIN_CTL7(1)

/* SUNXI_AHUB_CTL */
#define HDMI_SRC_SEL			0x04

/* SUNXI_AHUB_RST */
#define APBIF_TXDIF0_RST		31
#define APBIF_TXDIF1_RST		30
#define APBIF_TXDIF2_RST		29
#define APBIF_RXDIF0_RST		27
#define APBIF_RXDIF1_RST		26
#define APBIF_RXDIF2_RST		25
#define I2S0_RST			23
#define I2S1_RST			22
#define I2S2_RST			21
#define I2S3_RST			20
#define DAM0_RST			15
#define DAM1_RST			14

/* SUNXI_AHUB_GAT */
#define APBIF_TXDIF0_GAT		31
#define APBIF_TXDIF1_GAT		30
#define APBIF_TXDIF2_GAT		29
#define APBIF_RXDIF0_GAT		27
#define APBIF_RXDIF1_GAT		26
#define APBIF_RXDIF2_GAT		25
#define I2S0_GAT			23
#define I2S1_GAT			22
#define I2S2_GAT			21
#define I2S3_GAT			20
#define DAM0_GAT			15
#define DAM1_GAT			14

/* SUNXI_AHUB_APBIF_TX_CTL */
#define APBIF_TX_WS			16
#define APBIF_TX_CHAN_NUM		8
#define	APBIF_TX_START			4

/* SUNXI_AHUB_APBIF_TX_IRQ_CTL */
#define APBIF_TX_DRQ			3
#define APBIF_TX_OVEN			1
#define APBIF_TX_EMEN			0

/* SUNXI_AHUB_APBIF_TX_IRQ_STA */
#define APBIF_TX_OV_PEND		2
#define APBIF_TX_EM_PEND		0

/* SUNXI_AHUB_APBIF_TXFIFO_CTL */
#define APBIF_TX_FTX			12
#define APBIF_TX_LEVEL			4
#define APBIF_TX_TXIM			0

/* SUNXI_AHUB_APBIF_TXFIFO_STA */
#define APBIF_TX_EMPTY			8
#define APBIF_TX_EMCNT			0

/* SUNXI_AHUB_APBIF_RX_CTL */
#define APBIF_RX_WS			16
#define APBIF_RX_CHAN_NUM		8
#define	APBIF_RX_START			4

/* SUNXI_AHUB_APBIF_RX_IRQ_CTL */
#define APBIF_RX_DRQ			3
#define APBIF_RX_UVEN			2
#define APBIF_RX_AVEN			0

/* SUNXI_AHUB_APBIF_RX_IRQ_STA */
#define APBIF_RX_UV_PEND		2
#define APBIF_RX_AV_PEND		0

/* SUNXI_AHUB_APBIF_RXFIFO_CTL */
#define APBIF_RX_FRX			12
#define APBIF_RX_LEVEL			4
#define APBIF_RX_RXOM			0

/* SUNXI_AHUB_APBIF_RXFIFO_STA */
#define APBIF_RX_AVAIL			8
#define APBIF_RX_AVCNT			0

/* SUNXI_AHUB_APBIF_RXFIFO_CONT */
#define APBIF_RX_APBIF_TXDIF0		31
#define APBIF_RX_APBIF_TXDIF1		30
#define APBIF_RX_APBIF_TXDIF2		29
#define APBIF_RX_I2S0_TXDIF		27
#define APBIF_RX_I2S1_TXDIF		26
#define APBIF_RX_I2S2_TXDIF		25
#define APBIF_RX_I2S3_TXDIF		23
#define APBIF_RX_DAM0_TXDIF		19
#define APBIF_RX_DAM1_TXDIF		15

/* SUNXI_AHUB_I2S_CTL */
#define I2S_CTL_LOOP3			23
#define I2S_CTL_LOOP2			22
#define I2S_CTL_LOOP1			21
#define I2S_CTL_LOOP0			20
#define I2S_CTL_SDI3_EN			15
#define I2S_CTL_SDI2_EN			14
#define I2S_CTL_SDI1_EN			13
#define I2S_CTL_SDI0_EN			12
#define I2S_CTL_CLK_OUT			18
#define I2S_CTL_SDO3_EN			11
#define I2S_CTL_SDO2_EN			10
#define I2S_CTL_SDO1_EN			9
#define I2S_CTL_SDO0_EN			8
#define I2S_CTL_OUT_MUTE		6
#define I2S_CTL_MODE			4
#define I2S_CTL_TXEN			2
#define I2S_CTL_RXEN			1
#define I2S_CTL_GEN			0

/* SUNXI_AHUB_I2S_FMT0 */
#define I2S_FMT0_LRCK_WIDTH		30
#define I2S_FMT0_LRCK_POLARITY		19
#define I2S_FMT0_LRCK_PERIOD		8
#define I2S_FMT0_BCLK_POLARITY		7
#define I2S_FMT0_SR			4
#define I2S_FMT0_EDGE			3
#define I2S_FMT0_SW			0

/* SUNXI_AHUB_I2S_FMT1 */
#define I2S_FMT1_RX_LSB			7
#define I2S_FMT1_TX_LSB			6
#define I2S_FMT1_EXT			4
#define I2S_FMT1_RX_PDM			2
#define I2S_FMT1_TX_PDM			0

/* SUNXI_AHUB_I2S_CLKD */
#define I2S_CLKD_MCLK			8
#define	I2S_CLKD_BCLKDIV		4
#define I2S_CLKD_MCLKDIV		0

/* SUNXI_AHUB_I2S_RXCONT */
#define I2S_RX_APBIF_TXDIF0		31
#define I2S_RX_APBIF_TXDIF1		30
#define I2S_RX_APBIF_TXDIF2		29
#define I2S_RX_I2S0_TXDIF		27
#define I2S_RX_I2S1_TXDIF		26
#define I2S_RX_I2S2_TXDIF		25
#define I2S_RX_I2S3_TXDIF		23
#define I2S_RX_DAM0_TXDIF		19
#define I2S_RX_DAM1_TXDIF		15

/* SUNXI_AHUB_I2S_CHCFG */
#define I2S_CHCFG_HIZ			9
#define	I2S_CHCFG_TX_STATE		8
#define I2S_CHCFG_RX_CHANNUM		4
#define I2S_CHCFG_TX_CHANNUM		0

/* SUNXI_AHUB_I2S_IRQ_CTL */
#define I2S_IRQ_RXOV_EN			1
#define I2S_IRQ_TXUV_EN			0

/* SUNXI_AHUB_I2S_IRQ_STA */
#define I2S_IRQ_RXOV_PEND		1
#define I2S_IRQ_TXUV_PEND		0

/* SUNXI_AHUB_I2S_OUT_SLOT */
#define I2S_OUT_OFFSET			20
#define I2S_OUT_SLOT_NUM		16
#define I2S_OUT_SLOT_EN			0

/* SUNXI_AHUB_I2S_IN_SLOT */
#define I2S_IN_OFFSET			20
#define I2S_IN_SLOT_NUM			16

/* SUNXI_AHUB_DAM_CTL */
#define DAM_CTL_RX2_NUM			24
#define DAM_CTL_RX1_NUM			20
#define DAM_CTL_RX0_NUM			16
#define DAM_CTL_TX_NUM			8
#define DAM_CTL_RX2EN			6
#define DAM_CTL_RX1EN			5
#define DAM_CTL_RX0EN			4
#define DAM_CTL_TXEN			0

/* SUNXI_AHUB_DAM_RX##chan##_SRC */
#define DAM_RX_APBIF_TXDIF0		31
#define DAM_RX_APBIF_TXDIF1		30
#define DAM_RX_APBIF_TXDIF2		29
#define DAM_RX_I2S0_TXDIF		27
#define DAM_RX_I2S1_TXDIF		26
#define DAM_RX_I2S2_TXDIF		25
#define DAM_RX_I2S3_TXDIF		23
#define DAM_RX_DAM0_TXDIF		19
#define DAM_RX_DAM1_TXDIF		15

struct sunxi_ahub_mem {
	char *dev_name;
	struct resource *res;
	void __iomem *membase;
	struct resource *memregion;
	struct regmap *regmap;
};

struct sunxi_ahub_clk {
	struct clk *clk_pll;
	struct clk *clk_pllx4;
	struct clk *clk_module;
	struct clk *clk_bus;
	struct reset_control *clk_rst;
};

/* debug */
struct sunxi_ahub_dump {
	struct platform_device *pdev;
	const char *module_version;
	char module_name[32];
	struct snd_sunxi_dump dump;
	int show_reg_num;
	struct regmap *regmap;
};

extern int snd_sunxi_ahub_mem_get(struct sunxi_ahub_mem *mem);
extern int snd_sunxi_ahub_clk_get(struct sunxi_ahub_clk *clk);

extern void sunxi_ahub_dam_ctrl(bool enable,
				unsigned int apb_num, unsigned int tdm_num, unsigned int channels);

#endif /* __SND_SUNXI_AHUB_DAM_H */
