// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner PIPE USB3.0 PCIE Combo Phy driver
 *
 * Copyright (C) 2022 Allwinner Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>
#include <dt-bindings/phy/phy.h>
#include "sunxi-inno.h"

/* PCIE USB3 Sub-System Registers */
/* Sub-System Version Reset Register */
#define PCIE_USB3_SYS_VER		0x00

/* Sub-System PCIE Bus Gating Reset Register */
#define PCIE_COMBO_PHY_BGR		0x04
#define   PCIE_ACLK_EN			BIT(17)
#define   PCIE_HCLK_EN			BIT(16)
#define   PCIE_PERSTN			BIT(1)
#define   PCIE_PW_UP_RSTN		BIT(0)

/* Sub-System USB3 Bus Gating Reset Register */
#define USB3_COMBO_PHY_BGR		0x08
#define   USB3_ACLK_EN			BIT(17)
#define   USB3_HCLK_EN			BIT(16)
#define   USB3_RESETN			BIT(0)

/* Sub-System PCIE PHY Control Register */
#define PCIE_COMBO_PHY_CTL		0x10
#define   PHY_USE_SEL			BIT(31)	/* 0:PCIE; 1:USB3 */
#define   PHY_CLK_SEL			BIT(30) /* 0:internal clk; 1:external clk */
#define   PHY_BIST_EN			BIT(16)
#define   PHY_PIPE_SW			BIT(9)
#define   PHY_PIPE_SEL			BIT(8)  /* 0:rstn by PCIE or USB3; 1:rstn by PHY_PIPE_SW */
#define   PHY_PIPE_CLK_INVERT		BIT(4)
#define   PHY_FPGA_SYS_RSTN		BIT(1)  /* for FPGA  */
#define   PHY_RSTN			BIT(0)

/* Registers */
#define  COMBO_REG_SYSVER(comb_base_addr)	((comb_base_addr) \
							+ PCIE_USB3_SYS_VER)
#define  COMBO_REG_PCIEBGR(comb_base_addr)	((comb_base_addr) \
							+ PCIE_COMBO_PHY_BGR)
#define  COMBO_REG_USB3BGR(comb_base_addr)	((comb_base_addr) \
							+ USB3_COMBO_PHY_BGR)
#define  COMBO_REG_PHYCTRL(comb_base_addr)	((comb_base_addr) \
							+ PCIE_COMBO_PHY_CTL)

/* Sub-System Version Number */
#define  COMBO_VERSION_01				(0x10000)
#define  COMBO_VERSION_ANY				(0x0)

#define  KEY_PHY_USE_SEL				"phy_use_sel"
#define  KEY_PHY_REFCLK_SEL				"phy_refclk_sel"

enum phy_use_sel {
	PHY_USE_BY_PCIE = 0, /* PHY used by PCIE */
	PHY_USE_BY_USB3, /* PHY used by USB3 */
	PHY_USE_BY_PCIE_USB3_U2,/* PHY used by PCIE & USB3_U2 */
};

enum phy_refclk_sel {
	INTER_SIG_REF_CLK = 0, /* PHY use internal single end reference clock */
	EXTER_DIF_REF_CLK, /* PHY use external single end reference clock */
};

struct sunxi_combphy {
	struct device *dev;
	struct phy *phy;
	void __iomem *phy_ctl;  /* parse dts, control the phy mode, reset and power */
	void __iomem *phy_clk;  /* parse dts, set the phy clock */
	struct reset_control *reset;
	struct clk *clk;
	__u8 mode;
	__u32 vernum; /* version number */
	enum phy_use_sel user;
	enum phy_refclk_sel ref;
	struct notifier_block pwr_nb;
};

ATOMIC_NOTIFIER_HEAD(inno_subsys_notifier_list);
EXPORT_SYMBOL(inno_subsys_notifier_list);

/*  PCIE USB3 Sub-system Application */
static void combo_pcie_clk_set(struct sunxi_combphy *combphy, bool enable)
{
	u32 val, tmp = 0;

	val = readl(COMBO_REG_PCIEBGR(combphy->phy_ctl));
	tmp = PCIE_ACLK_EN | PCIE_HCLK_EN | PCIE_PERSTN | PCIE_PW_UP_RSTN;
	if (enable)
		val |= tmp;
	else
		val &= ~tmp;
	writel(val, COMBO_REG_PCIEBGR(combphy->phy_ctl));
}

static void combo_usb3_clk_set(struct sunxi_combphy *combphy, bool enable)
{
	u32 val, tmp = 0;

	val = readl(COMBO_REG_USB3BGR(combphy->phy_ctl));
	tmp = USB3_ACLK_EN | USB3_HCLK_EN | USB3_RESETN;
	if (enable)
		val |= tmp;
	else
		val &= ~tmp;
	writel(val, COMBO_REG_USB3BGR(combphy->phy_ctl));
}

static void combo_phy_mode_set(struct sunxi_combphy *combphy, bool enable)
{
	u32 val, tmp = 0;

	val = readl(COMBO_REG_PHYCTRL(combphy->phy_ctl));

	if (combphy->user == PHY_USE_BY_PCIE)
		tmp &= ~PHY_USE_SEL;
	else if (combphy->user == PHY_USE_BY_USB3)
		tmp |= PHY_USE_SEL;
	else if (combphy->user == PHY_USE_BY_PCIE_USB3_U2)
		tmp &= ~PHY_USE_SEL;

	if (combphy->ref == INTER_SIG_REF_CLK)
		tmp &= ~PHY_CLK_SEL;
	else if (combphy->ref == EXTER_DIF_REF_CLK)
		tmp |= PHY_CLK_SEL;

	if (enable) {
		tmp |= PHY_RSTN;
		val |= tmp;
	} else {
		tmp &= ~PHY_RSTN;
		val &= ~tmp;
	}
	writel(val, COMBO_REG_PHYCTRL(combphy->phy_ctl));
}

static u32 combo_sysver_get(struct sunxi_combphy *combphy)
{
	u32 reg;

	reg = readl(COMBO_REG_SYSVER(combphy->phy_ctl));

	return reg;
}

static void pcie_usb3_sub_system_enable(struct sunxi_combphy *combphy)
{
	combo_phy_mode_set(combphy, true);

	if (combphy->user == PHY_USE_BY_PCIE)
		combo_pcie_clk_set(combphy, true);
	else if (combphy->user == PHY_USE_BY_USB3)
		combo_usb3_clk_set(combphy, true);
	else if (combphy->user == PHY_USE_BY_PCIE_USB3_U2) {
		combo_pcie_clk_set(combphy, true);
		combo_usb3_clk_set(combphy, true);
	}

	combphy->vernum = combo_sysver_get(combphy);
}

static void pcie_usb3_sub_system_disable(struct sunxi_combphy *combphy)
{
	combo_phy_mode_set(combphy, false);

	if (combphy->user == PHY_USE_BY_PCIE)
		combo_pcie_clk_set(combphy, false);
	else if (combphy->user == PHY_USE_BY_USB3)
		combo_usb3_clk_set(combphy, false);
	else if (combphy->user == PHY_USE_BY_PCIE_USB3_U2) {
		combo_pcie_clk_set(combphy, false);
		combo_usb3_clk_set(combphy, false);
	}
}

static int pcie_usb3_sub_system_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_combphy *combphy = platform_get_drvdata(pdev);
	int ret;

	if (!IS_ERR(combphy->clk)) {
		ret = clk_prepare_enable(combphy->clk);
		if (ret)
			return ret;
	}

	ret = reset_control_deassert(combphy->reset);
	if (ret) {
		if (!IS_ERR(combphy->clk))
			clk_disable_unprepare(combphy->clk);
		return ret;
	}

	pcie_usb3_sub_system_enable(combphy);

	if (combphy->vernum == COMBO_VERSION_ANY)
		dev_err(dev, "this is unknown version number\n");

	return 0;
}

static int pcie_usb3_sub_system_exit(struct platform_device *pdev)
{
	struct sunxi_combphy *combphy = platform_get_drvdata(pdev);

	pcie_usb3_sub_system_disable(combphy);

	if (!IS_ERR(combphy->reset))
		reset_control_assert(combphy->reset);

	if (!IS_ERR(combphy->clk))
		clk_disable_unprepare(combphy->clk);

	return 0;
}

static int sunxi_inno_combophy_power_event(struct notifier_block *nb, unsigned long event, void *p)
{
	struct sunxi_combphy *combphy = container_of(nb, struct sunxi_combphy, pwr_nb);
	struct platform_device *pdev = to_platform_device(combphy->dev);

	dev_dbg(combphy->dev, "event %s\n", event ? "on" : "off");
	if (event)
		pcie_usb3_sub_system_init(pdev);
	else
		pcie_usb3_sub_system_exit(pdev);

	return NOTIFY_DONE;
}

static void sunxi_combphy_pcie_phy_enable(struct sunxi_combphy *combphy)
{
	u32 val;

	/* set the phy:
	 * bit(17): aclk enable
	 * bit(16): hclk enbale
	 * bit(1) : pcie_presetn
	 * bit(0) : pcie_power_up_rstn
	 */
	val = readl(combphy->phy_ctl + PCIE_COMBO_PHY_BGR);
	val &= (~(0x03<<0));
	val &= (~(0x03<<16));
	val |= (0x03<<0);
	val |= (0x03<<16);
	writel(val, combphy->phy_ctl + PCIE_COMBO_PHY_BGR);


	/* select phy mode, phy assert */
	val = readl(combphy->phy_ctl + PCIE_COMBO_PHY_CTL);
	val &= (~PHY_USE_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_FPGA_SYS_RSTN);
	val &= (~PHY_RSTN);
	writel(val, combphy->phy_ctl + PCIE_COMBO_PHY_CTL);

	 /* phy De-assert */
	val = readl(combphy->phy_ctl + PCIE_COMBO_PHY_CTL);
	val &= (~PHY_CLK_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_FPGA_SYS_RSTN);
	val &= (~PHY_RSTN);
	val |= PHY_RSTN;
	writel(val, combphy->phy_ctl + PCIE_COMBO_PHY_CTL);

	val = readl(combphy->phy_ctl + PCIE_COMBO_PHY_CTL);
	val &= (~PHY_CLK_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_FPGA_SYS_RSTN);
	val &= (~PHY_RSTN);
	val |= PHY_RSTN;
	val |= (PHY_FPGA_SYS_RSTN);
	writel(val, combphy->phy_ctl + PCIE_COMBO_PHY_CTL);

}

static void sunxi_combphy_usb3_phy_set(struct sunxi_combphy *combphy, bool enable)
{
	u32 val, tmp = 0;

	val = readl(combphy->phy_clk + 0x1418);
	tmp = GENMASK(17, 16);
	if (enable) {
		val &= ~tmp;
		val |= BIT(25);
	} else {
		val |= tmp;
		val &= ~BIT(25);
	}
	writel(val, combphy->phy_clk + 0x1418);

	/* reg_rx_eq_bypass[3]=1, rx_ctle_res_cal_bypass */
	val = readl(combphy->phy_clk + 0x0674);
	if (enable)
		val |= BIT(3);
	else
		val &= ~BIT(3);
	writel(val, combphy->phy_clk + 0x0674);

	/* rx_ctle_res_cal=0xf, 0x4->0xf */
	val = readl(combphy->phy_clk + 0x0704);
	tmp = GENMASK(9, 8) | BIT(11);
	if (enable)
		val |= tmp;
	else
		val &= ~tmp;
	writel(val, combphy->phy_clk + 0x0704);

	/* CDR_div_fin_gain1 */
	val = readl(combphy->phy_clk + 0x0400);
	if (enable)
		val |= BIT(4);
	else
		val &= ~BIT(4);
	writel(val, combphy->phy_clk + 0x0400);

	/* CDR_div1_fin_gain1 */
	val = readl(combphy->phy_clk + 0x0404);
	tmp = GENMASK(3, 0) | BIT(5);
	if (enable)
		val |= tmp;
	else
		val &= ~tmp;
	writel(val, combphy->phy_clk + 0x0404);

	/* CDR_div3_fin_gain1 */
	val = readl(combphy->phy_clk + 0x0408);
	if (enable)
		val |= BIT(5);
	else
		val &= ~BIT(5);
	writel(val, combphy->phy_clk + 0x0408);

	val = readl(combphy->phy_clk + 0x109c);
	if (enable)
		val |= BIT(1);
	else
		val &= ~BIT(1);
	writel(val, combphy->phy_clk + 0x109c);

	/* SSC configure */
	val = readl(combphy->phy_clk + 0x107c);
	tmp = 0x3f << 12;
	val = val & (~tmp);
	val |= ((0x1 << 12) & tmp);              /* div_N */
	writel(val, combphy->phy_clk + 0x107c);

	val = readl(combphy->phy_clk + 0x1020);
	tmp = 0x1f << 0;
	val = val & (~tmp);
	val |= ((0x6 << 0) & tmp);               /* modulation freq div */
	writel(val, combphy->phy_clk + 0x1020);

	val = readl(combphy->phy_clk + 0x1034);
	tmp = 0x7f << 16;
	val = val & (~tmp);
	val |= ((0x9 << 16) & tmp);              /* spread[6:0], 400*9=4410ppm ssc */
	writel(val, combphy->phy_clk + 0x1034);

	val = readl(combphy->phy_clk + 0x101c);
	tmp = 0x1 << 27;
	val = val & (~tmp);
	val |= ((0x1 << 27) & tmp);              /* choose downspread */

	tmp = 0x1 << 28;
	val = val & (~tmp);
	if (enable)
		val |= ((0x0 << 28) & tmp);      /* don't disable ssc = 0 */
	else
		val |= ((0x1 << 28) & tmp);      /* don't enable ssc = 1 */
	writel(val, combphy->phy_clk + 0x101c);

#ifdef SUNXI_INNO_COMMBOPHY_DEBUG
	/* TX Eye configure bypass_en */
	val = readl(combphy->phy_clk + 0x0ddc);
	if (enable)
		val |= BIT(4);                   /*  0x0ddc[4]=1 */
	else
		val &= ~BIT(4);
	writel(val, combphy->phy_clk + 0x0ddc);

	/* Leg_cur[6:0] - 7'd84 */
	val = readl(combphy->phy_clk + 0x0ddc);
	val |= ((0x54 & BIT(6)) >> 3);           /* 0x0ddc[3] */
	writel(val, combphy->phy_clk + 0x0ddc);

	val = readl(combphy->phy_clk + 0x0de0);
	val |= ((0x54 & GENMASK(5, 0)) << 2);    /* 0x0de0[7:2] */
	writel(val, combphy->phy_clk + 0x0de0);

	/* Leg_curb[5:0] - 6'd18 */
	val = readl(combphy->phy_clk + 0x0de4);
	val |= ((0x12 & GENMASK(5, 1)) >> 1);    /* 0x0de4[4:0] */
	writel(val, combphy->phy_clk + 0x0de4);

	val = readl(combphy->phy_clk + 0x0de8);
	val |= ((0x12 & BIT(0)) << 7);           /* 0x0de8[7] */
	writel(val, combphy->phy_clk + 0x0de8);

	/* Exswing_isel */
	val = readl(combphy->phy_clk + 0x0028);
	val |= (0x4 << 28);                      /* 0x28[30:28] */
	writel(val, combphy->phy_clk + 0x0028);

	/* Exswing_en */
	val = readl(combphy->phy_clk + 0x0028);
	if (enable)
		val |= BIT(31);                  /* 0x28[31]=1 */
	else
		val &= ~BIT(31);
	writel(val, combphy->phy_clk + 0x0028);
#endif
}

static void sunxi_combphy_usb3_power_set(struct sunxi_combphy *combphy, bool enable)
{
	u32 val;

	dev_dbg(combphy->dev, "set power %s\n", enable ? "on" : "off");
	val = readl(combphy->phy_clk + 0x14);
	if (enable)
		val &= ~BIT(26);
	else
		val |= BIT(26);
	writel(val, combphy->phy_clk + 0x14);

	val = readl(combphy->phy_clk + 0x0);
	if (enable)
		val &= ~BIT(10);
	else
		val |= BIT(10);
	writel(val, combphy->phy_clk + 0x0);
}

static void sunxi_combphy_pcie_phy_100M(struct sunxi_combphy *combphy)
{
	u32 val;

	val = readl(combphy->phy_clk + 0x1004);
	val &= ~(0x3<<3);
	val &= ~(0x1<<0);
	val |= (0x1<<0);
	val |= (0x1<<2);
	val |= (0x1<<4);
	writel(val, combphy->phy_clk + 0x1004);

	val = readl(combphy->phy_clk + 0x1018);
	val &= ~(0x3<<4);
	val |= (0x3<<4);
	writel(val, combphy->phy_clk + 0x1018);

	val = readl(combphy->phy_clk + 0x101c);
	val &= ~(0x0fffffff);
	writel(val, combphy->phy_clk + 0x101c);

	val = readl(combphy->phy_clk + 0x107c);
	val &= ~(0x3ffff);
	val |= (0x2<<12);
	val |= 0x32;
	writel(val, combphy->phy_clk + 0x107c);

	val = readl(combphy->phy_clk + 0x1030);
	val &= ~(0x3<<20);
	writel(val, combphy->phy_clk + 0x1030);

	val = readl(combphy->phy_clk + 0x1050);
	val &= ~(0x7<<5);
	val |= (0x1<<5);
	writel(val, combphy->phy_clk + 0x1050);

	val = readl(combphy->phy_clk + 0x1054);
	val &= ~(0x7<<5);
	val |= (0x1<<5);
	writel(val, combphy->phy_clk + 0x1054);

	val = readl(combphy->phy_clk + 0x0804);
	val &= ~(0xf<<4);
	val |= (0xc<<4);
	writel(val, combphy->phy_clk + 0x0804);

	val = readl(combphy->phy_clk + 0x109c);
	val &= ~(0x3<<8);
	val |= (0x1<<1);
	writel(val, combphy->phy_clk + 0x109c);

	writel(0x80540a0a, combphy->phy_clk + 0x1418);
}

static int sunxi_combphy_pcie_init(struct sunxi_combphy *combphy)
{
	sunxi_combphy_pcie_phy_100M(combphy);

	sunxi_combphy_pcie_phy_enable(combphy);

	return 0;
}

static int sunxi_combphy_pcie_exit(struct sunxi_combphy *combphy)
{
	u32 val;

	/* set the phy:
	 * bit(17): aclk enable
	 * bit(16): hclk enbale
	 * bit(1) : pcie_presetn
	 * bit(0) : pcie_power_up_rstn
	 */
	val = readl(combphy->phy_ctl + PCIE_COMBO_PHY_BGR);
	val &= (~(0x03<<0));
	val &= (~(0x03<<16));
	writel(val, combphy->phy_ctl + PCIE_COMBO_PHY_BGR);

	/* Assert the phy */
	val = readl(combphy->phy_ctl + PCIE_COMBO_PHY_CTL);
	val &= (~PHY_USE_SEL);
	val &= (~(0x03<<8));
	val &= (~PHY_RSTN);
	writel(val, combphy->phy_ctl + PCIE_COMBO_PHY_CTL);

	return 0;
}

static int sunxi_combphy_usb3_init(struct sunxi_combphy *combphy)
{
	sunxi_combphy_usb3_phy_set(combphy, true);

	return 0;
}

static int sunxi_combphy_usb3_exit(struct sunxi_combphy *combphy)
{
	sunxi_combphy_usb3_phy_set(combphy, false);

	return 0;
}

static int sunxi_combphy_usb3_power_on(struct sunxi_combphy *combphy)
{
	sunxi_combphy_usb3_power_set(combphy, true);

	return 0;
}

static int sunxi_combphy_usb3_power_off(struct sunxi_combphy *combphy)
{
	sunxi_combphy_usb3_power_set(combphy, false);

	return 0;
}

static int sunxi_combphy_set_mode(struct sunxi_combphy *combphy)
{
	switch (combphy->mode) {
	case PHY_TYPE_PCIE:
		sunxi_combphy_pcie_init(combphy);
		break;
	case PHY_TYPE_USB3:
		sunxi_combphy_usb3_init(combphy);
		break;
	default:
		dev_err(combphy->dev, "incompatible PHY type\n");
		return -EINVAL;
	}

	return 0;
}

static int sunxi_combphy_init(struct phy *phy)
{
	struct sunxi_combphy *combphy = phy_get_drvdata(phy);
	int ret;

	ret = sunxi_combphy_set_mode(combphy);
	if (ret) {
		dev_err(combphy->dev, "invalid number of arguments\n");
		return ret;
	}

	return ret;
}

static int sunxi_combphy_exit(struct phy *phy)
{
	struct sunxi_combphy *combphy = phy_get_drvdata(phy);

	switch (combphy->mode) {
	case PHY_TYPE_PCIE:
		sunxi_combphy_pcie_exit(combphy);
		break;
	case PHY_TYPE_USB3:
		sunxi_combphy_usb3_exit(combphy);
		break;
	default:
		dev_err(combphy->dev, "incompatible PHY type\n");
		return -EINVAL;
	}

	return 0;
}

static int sunxi_combphy_power_on(struct phy *phy)
{
	struct sunxi_combphy *combphy = phy_get_drvdata(phy);

	switch (combphy->mode) {
	case PHY_TYPE_PCIE:
		break;
	case PHY_TYPE_USB3:
		sunxi_combphy_usb3_power_on(combphy);
		break;
	default:
		dev_err(combphy->dev, "incompatible PHY type\n");
		return -EINVAL;
	}

	return 0;
}

static int sunxi_combphy_power_off(struct phy *phy)
{
	struct sunxi_combphy *combphy = phy_get_drvdata(phy);

	switch (combphy->mode) {
	case PHY_TYPE_PCIE:
		break;
	case PHY_TYPE_USB3:
		sunxi_combphy_usb3_power_off(combphy);
		break;
	default:
		dev_err(combphy->dev, "incompatible PHY type\n");
		return -EINVAL;
	}

	return 0;
}

static const struct phy_ops sunxi_combphy_ops = {
	.init = sunxi_combphy_init,
	.exit = sunxi_combphy_exit,
	.power_on = sunxi_combphy_power_on,
	.power_off = sunxi_combphy_power_off,
	.owner = THIS_MODULE,
};

static struct phy *sunxi_combphy_xlate(struct device *dev,
					  struct of_phandle_args *args)
{
	struct sunxi_combphy *combphy = dev_get_drvdata(dev);

	if (args->args_count != 1) {
		dev_err(dev, "invalid number of arguments\n");
		return ERR_PTR(-EINVAL);
	}

	if (combphy->mode != PHY_NONE && combphy->mode != args->args[0])
		dev_warn(dev, "phy type select %d overwriting type %d\n",
			 args->args[0], combphy->mode);

	combphy->mode = args->args[0];

	return combphy->phy;
}

static int sunxi_combphy_parse_dt(struct platform_device *pdev,
				     struct sunxi_combphy *combphy)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret = -1;
	struct resource *res_ctl;
	struct resource *res_clk;

	combphy->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(combphy->clk)) {
		if (PTR_ERR(combphy->clk) != -EPROBE_DEFER)
			dev_dbg(dev, "failed to get com clock\n");
	}

	combphy->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(combphy->reset))
		dev_dbg(dev, "failed to get reset control\n");

	res_ctl = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy-ctl");
	if (!res_ctl) {
		dev_err(&pdev->dev, "get phy-ctl failed\n");
		return -ENODEV;
	}

	combphy->phy_ctl = devm_ioremap_resource(&pdev->dev, res_ctl);
	if (IS_ERR(combphy->phy_ctl)) {
		dev_err(&pdev->dev, "ioremap phy-ctl failed\n");
		return PTR_ERR(combphy->phy_ctl);
	}

	res_clk = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy-clk");
	if (!res_clk) {
		dev_err(&pdev->dev, "get phy-clk failed\n");
		return -ENODEV;
	}

	combphy->phy_clk = devm_ioremap_resource(&pdev->dev, res_clk);
	if (IS_ERR(combphy->phy_clk)) {
		dev_err(&pdev->dev, "ioremap phy-clk failed\n");
		return PTR_ERR(combphy->phy_clk);
	}

	/* combo phy use sel */
	ret = of_property_read_u32(np, KEY_PHY_USE_SEL, &combphy->user);
	if (ret)
		dev_err(dev, "get phy_use_sel is fail, %d\n", ret);

	/* combo phy refclk sel */
	ret = of_property_read_u32(np, KEY_PHY_REFCLK_SEL, &combphy->ref);
	if (ret)
		dev_err(dev, "get phy_refclk_sel is fail, %d\n", ret);

	return 0;
}

static int sunxi_combphy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct sunxi_combphy *combphy;
	int ret;

	combphy = devm_kzalloc(dev, sizeof(*combphy), GFP_KERNEL);
	if (!combphy)
		return -ENOMEM;

	combphy->dev = dev;
	combphy->mode = PHY_NONE;

	ret = sunxi_combphy_parse_dt(pdev, combphy);
	if (ret) {
		dev_err(dev, "failed to parse dts of combphy\n");
		return ret;
	}

	combphy->phy = devm_phy_create(dev, NULL, &sunxi_combphy_ops);
	if (IS_ERR(combphy->phy)) {
		dev_err(dev, "failed to create combphy\n");
		return PTR_ERR(combphy->phy);
	}

	platform_set_drvdata(pdev, combphy);

	ret = pcie_usb3_sub_system_init(pdev);
	if (ret)
		dev_warn(dev, "failed to init sub system\n");

	dev_info(dev, "Sub System Version: 0x%x\n", combphy->vernum);

	phy_set_drvdata(combphy->phy, combphy);

	phy_provider = devm_of_phy_provider_register(dev, sunxi_combphy_xlate);

	if (combphy->user == PHY_USE_BY_USB3 || combphy->user == PHY_USE_BY_PCIE_USB3_U2) {
		combphy->pwr_nb.notifier_call = sunxi_inno_combophy_power_event;
		/* register inno power notifier */
		atomic_notifier_chain_register(&inno_subsys_notifier_list, &combphy->pwr_nb);

		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);
		pm_runtime_get_sync(dev);
	}

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int sunxi_combphy_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_combphy *combphy = platform_get_drvdata(pdev);
	int ret;

	ret = pcie_usb3_sub_system_exit(pdev);
	if (ret) {
		dev_err(dev, "failed to exit sub system\n");
		return ret;
	}

	if (combphy->user == PHY_USE_BY_USB3 || combphy->user == PHY_USE_BY_PCIE_USB3_U2) {
		/* unregister inno power notifier */
		atomic_notifier_chain_unregister(&inno_subsys_notifier_list, &combphy->pwr_nb);

		pm_runtime_disable(dev);
		pm_runtime_put_noidle(dev);
		pm_runtime_set_suspended(dev);
	}

	return 0;
}

static int __maybe_unused sunxi_combo_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	ret = pcie_usb3_sub_system_exit(pdev);

	if (ret) {
		dev_err(dev, "failed to suspend sub system\n");
		return ret;
	}

	return 0;
}

static int __maybe_unused sunxi_combo_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	ret = pcie_usb3_sub_system_init(pdev);
	if (ret) {
		dev_err(dev, "failed to resume sub system\n");
		return ret;
	}

	return 0;
}

static struct dev_pm_ops sunxi_combo_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sunxi_combo_suspend, sunxi_combo_resume)
};

/*
 * inno-combphy: innosilicon combo phy
 */
static const struct of_device_id sunxi_combphy_of_match[] = {
	{
		.compatible = "allwinner,inno-combphy",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sunxi_combphy_of_match);

static struct platform_driver sunxi_combphy_driver = {
	.probe	= sunxi_combphy_probe,
	.remove	= sunxi_combphy_remove,
	.driver = {
		.name = "inno-combphy",
		.of_match_table = sunxi_combphy_of_match,
		.pm = &sunxi_combo_pm_ops,
	},
};
module_platform_driver(sunxi_combphy_driver);

MODULE_DESCRIPTION("Allwinner INNO COMBOPHY driver");
MODULE_AUTHOR("songjundong@allwinnertech.com");
MODULE_VERSION("0.0.18");
MODULE_LICENSE("GPL v2");
