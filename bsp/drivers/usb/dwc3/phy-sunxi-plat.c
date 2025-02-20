// SPDX-License-Identifier: GPL-2.0+
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner sun55i(A523) USB 3.0 phy driver
 *
 * Copyright (c) 2022 Allwinner Technology Co., Ltd.
 *
 * Based on phy-sun50i-usb3.c, which is:
 *
 * Allwinner sun50i(H6) USB 3.0 phy driver
 *
 * Copyright (C) 2017 Icenowy Zheng <icenowy@aosc.io>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/of_device.h>
#include <sunxi-log.h>

/* USB Application Registers */
/* USB2.0 Interface Status and Control Register */
#define PHY_USB2_ISCR				0x00
#define   BC_ID_VALUE_STATUS			GENMASK(31, 27)
#define   USB_LINE_STATUS			GENMASK(26, 25)
#define   USB_VBUS_STATUS			BIT(24)
#define   FORCE_RID				GENMASK(23, 18)
#define   FORCE_ID				GENMASK(15, 14)
#define   FORCE_VBUS_VALID			GENMASK(13, 12)
#define   EXT_VBUS_SRC_SEL			GENMASK(11, 10)
#define   USB_WAKEUP_ENABLE			BIT(9)
#define   USB_WAKEUP_HOSC_EN			BIT(8)
/* maybe abbreviation is better */
#define   ID_MUL_VAL_C_INT_STA			BIT(7)
#define   VBUS_IN_C_DET_INT_STA			BIT(6)
#define   ID_IN_C_DET_INT_STA			BIT(5)
#define   DPDM_IN_C_DET_INT_STA			BIT(4)
#define   ID_MUL_VAL_DET_EN			BIT(3)
#define   VBUS_IN_C_DET_INT_EN			BIT(2)
#define   ID_IN_C_DET_INT_EN			BIT(1)
#define   DPDM_IN_C_DET_INT_EN			BIT(0)

/* USB2.0 PHY Control Register */
#define PHY_USB2_PHYCTL				0x10
#define   OTGDISABLE				BIT(10)
#define   DRVVBUS				BIT(9)
#define   VREGBYPASS				BIT(8)
#define   LOOPBACKENB				BIT(7)
#define   IDPULLUP				BIT(6)
#define   VBUSVLDEXT				BIT(5)
#define   VBUSVLDEXTSEL				BIT(4)
#define   SIDDQ					BIT(3)
#define   COMMONONN				BIT(2)
#define   VATESTENB				GENMASK(1, 0)

#define PHY_USB2_PHYTUNE			0x18

/* SYSCFG Registers */
/* Resister Calibration Control Register */
#define RESCAL_CTRL_REG				0x0160
#define   PCIE_USB_RES1000_0_TRIM_SEL		BIT(10)
#define   PCIE_USB_RES200_TRIM_SEL		BIT(10)
#define   USBPHY2_RES200_SEL			BIT(6)
#define   USBPHY1_RES200_TRIM_SEL		BIT(5)
#define   USBPHY1_RES200_SEL			BIT(5)
#define   USBPHY0_RES200_TRIM_SEL		BIT(4)
#define   USBPHY0_RES200_SEL			BIT(4)
#define   PHY_o_RES200_SEL(n)			(BIT(4) << n)
#define   RESCAL_MODE				BIT(2)
#define   CAL_ANA_EN				BIT(1)
#define   CAL_EN				BIT(0)
/* Resister 200ohms Manual Control Register */
#define RES200_CTRL_REG				0x0164
#define   USBPHY2_RES200_CTRL			GENMASK(21, 16)
#define   USBPHY1_RES200_CTRL			GENMASK(13, 8)
#define   USBPHY0_RES200_CTRL			GENMASK(5, 0)
#define   PHY_o_RES200_CTRL(n)			(GENMASK(5, 0) << (8 * n))
#define   PHY_o_RES200_CTRL_DEFAULT(n)		(0x33 << (8 * n))
/* Resister RES1 ohms Manual Control Register */
#define RES1_CTRL_REG				0x0168
#define   PCIE_USB_RES200_TRIM			GENMASK(15, 8)
#define   PCIE_USB_RES200_TRIM_DEFAULT		(0xC8 << 8)

/* Registers */
#define  PHY_REG_U2_ISCR(u2phy_base_addr)		((u2phy_base_addr) \
								+ PHY_USB2_ISCR)
#define  PHY_REG_U2_PHYCTL(u2phy_base_addr)		((u2phy_base_addr) \
								+ PHY_USB2_PHYCTL)
#define  PHY_REG_U2_PHYTUNE(u2phy_base_addr)		((u2phy_base_addr) \
								+ PHY_USB2_PHYTUNE)
#define  CFG_REG_RESCAL_CTRL(syscfg_base_addr)		((syscfg_base_addr) \
								+ RESCAL_CTRL_REG)
#define  CFG_REG_RES200_CTRL(syscfg_base_addr)		((syscfg_base_addr) \
								+ RES200_CTRL_REG)
#define  CFG_REG_RES1_CTRL(syscfg_base_addr)		((syscfg_base_addr) \
								+ RES1_CTRL_REG)

struct sunxi_phy_of_data {
	bool has_vbusvldext;
};

struct sunxi_phy {
	struct device *dev;
	struct phy *phy;
	void __iomem *u2;
	void __iomem *res;
	struct reset_control *reset;
	struct clk *clk;
	const struct sunxi_phy_of_data *drvdata;
	__u32 param;		/* USB2.0 PHY Tune Param */
	__u32 mode;		/* Resistance Calibration Mode */
	bool res_supported;
};

#define  PHY_TUNE_PARAM_INVALID		(0xFFFFFFFF)

enum phy_rext_mode_e {
	PHY_REXT_MODE_UNKNOWN = 0,
	PHY_REXT_MODE_V1,
	PHY_REXT_MODE_V2,
};

static const char * const phy_mode_name[] = {
	[PHY_REXT_MODE_UNKNOWN]		= "UNKNOW",
	[PHY_REXT_MODE_V1]		= "V1",
	[PHY_REXT_MODE_V2]		= "V2",
};

static void phy_rescal_set_v1(struct sunxi_phy *phy, bool enable)
{
	__u32 val = 0;
	__u32 port = 0, tmp; /* port companion enable ? */

	tmp = GENMASK(6, 4) & (~PHY_o_RES200_SEL(2));
	val = readl(CFG_REG_RESCAL_CTRL(phy->res));
	port = val & tmp;
	if (enable) {
		val &= ~CAL_EN;
		val |= PHY_o_RES200_SEL(2);
	} else {
		if (port == 0)
			val |= CAL_EN;
		val &= ~PHY_o_RES200_SEL(2);
	}
	writel(val, CFG_REG_RESCAL_CTRL(phy->res));

	val = readl(CFG_REG_RES200_CTRL(phy->res));
	if (enable)
		val &= ~PHY_o_RES200_CTRL(2);
	else
		val |= PHY_o_RES200_CTRL_DEFAULT(2);
	writel(val, CFG_REG_RES200_CTRL(phy->res));
}

static void phy_rescal_set_v2(struct sunxi_phy *phy, bool enable)
{
	__u32 val = 0;
	__u32 port = 0, tmp; /* port companion enable ? */

	tmp = GENMASK(5, 4);
	val = readl(CFG_REG_RESCAL_CTRL(phy->res));
	port = val & tmp;
	if (enable) {
		val &= ~CAL_EN;
		val |= PCIE_USB_RES200_TRIM_SEL;
	} else {
		if (port == 0)
			val |= CAL_EN;
		val &= ~PCIE_USB_RES200_TRIM_SEL;
	}
	writel(val, CFG_REG_RESCAL_CTRL(phy->res));

	val = readl(CFG_REG_RES1_CTRL(phy->res));
	if (enable)
		val &= ~PCIE_USB_RES200_TRIM;
	else
		val |= PCIE_USB_RES200_TRIM_DEFAULT;
	writel(val, CFG_REG_RES1_CTRL(phy->res));
}

static void phy_res_set(struct sunxi_phy *phy, bool enable)
{
	if (!phy->res)
		return;

	if (!phy->res_supported)
		return;

	if (phy->mode == PHY_REXT_MODE_V1)
		phy_rescal_set_v1(phy, enable);
	else if (phy->mode == PHY_REXT_MODE_V2)
		phy_rescal_set_v2(phy, enable);
	else
		sunxi_info(phy->dev, "unknown phy rext mode\n");
}

static void phy_u2_set(struct sunxi_phy *phy, bool enable)
{
	u32 val, tmp = 0;

	/* maybe it's unused, only for device */
#if IS_ENABLED(CONFIG_USB_DWC3_GADGET)
	val = readl(PHY_REG_U2_ISCR(phy->u2));
	if (enable)
		val |= FORCE_VBUS_VALID;
	else
		val &= ~FORCE_VBUS_VALID;
	writel(val, PHY_REG_U2_ISCR(phy->u2));
#endif

	val = readl(PHY_REG_U2_PHYCTL(phy->u2));
	if (phy->drvdata->has_vbusvldext)
		tmp = OTGDISABLE | VBUSVLDEXT;
	if (enable) {
		val |= tmp;
		val &= ~SIDDQ; /* write 0 to enable phy */
	} else {
		val &= ~tmp;
		val |= SIDDQ; /* write 1 to disable phy */
	}

	writel(val, PHY_REG_U2_PHYCTL(phy->u2));
}

static int sunxi_phy_init(struct phy *_phy)
{
	struct sunxi_phy *phy = phy_get_drvdata(_phy);
	int ret;

	if (!IS_ERR(phy->reset)) {
		ret = reset_control_deassert(phy->reset);
		if (ret) {
			if (!IS_ERR(phy->clk))
				clk_disable_unprepare(phy->clk);
			return ret;
		}
	}

	if (!IS_ERR(phy->clk)) {
		ret = clk_prepare_enable(phy->clk);
		if (ret)
			return ret;
	}

	phy_res_set(phy, true);
	phy_u2_set(phy, true);

	if (phy->param != PHY_TUNE_PARAM_INVALID)
		writel(phy->param, PHY_REG_U2_PHYTUNE(phy->u2));

	return 0;
}

static int sunxi_phy_exit(struct phy *_phy)
{
	struct sunxi_phy *phy = phy_get_drvdata(_phy);

	phy_u2_set(phy, false);
	phy_res_set(phy, false);

	if (!IS_ERR(phy->clk))
		clk_disable_unprepare(phy->clk);

	if (!IS_ERR(phy->reset))
		reset_control_assert(phy->reset);

	return 0;
}

static const struct phy_ops sunxi_phy_ops = {
	.init		= sunxi_phy_init,
	.exit		= sunxi_phy_exit,
	.owner		= THIS_MODULE,
};

static int phy_sunxi_plat_probe(struct platform_device *pdev)
{
	struct sunxi_phy *phy;
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	const struct sunxi_phy_of_data *data;
	struct resource *res;
	__u32 mode = 0;
	__u32 param = 0;
	int ret = 0;

	data = of_device_get_match_data(&pdev->dev);

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;
	phy->drvdata = data;

	phy->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(phy->clk)) {
		if (PTR_ERR(phy->clk) != -EPROBE_DEFER)
			sunxi_debug(dev, "failed to get phy clock\n");
	}

	phy->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(phy->reset))
		sunxi_debug(dev, "failed to get reset control\n");

	phy->u2 = devm_platform_ioremap_resource_byname(pdev, "u2_base");
	if (IS_ERR_OR_NULL(phy->u2)) {
		sunxi_debug(dev, "get u2 base failed, try to get first resource by index\n");
		phy->u2 = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(phy->u2))
			return PTR_ERR(phy->u2);
	}
	phy->dev = dev;

	/*
	 * Please don't replace this with devm_platform_ioremap_resource.
	 *
	 * devm_platform_ioremap_resource calls devm_ioremap_resource which
	 * differs from devm_ioremap by also calling devm_request_mem_region
	 * and preventing other mappings in the same area.
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "res_base");
	if (res) {
		phy->res = devm_ioremap(dev, res->start, resource_size(res));
		if (!phy->res)
			return -ENOMEM;

		phy->res_supported = !device_property_read_bool(&pdev->dev, "aw,rext_cal_bypass");

		/* get aw,rext_mode parameters from device-tree */
		ret = device_property_read_u32(&pdev->dev, "aw,rext_mode", &mode);
		if (ret) {
			phy->mode = PHY_REXT_MODE_UNKNOWN;
		} else {
			phy->mode = mode;
		}
		sunxi_info(dev, "resistance calibration supported - %s, mode - %s\n",
			   phy->res_supported ? "enabled" : "disabled", phy_mode_name[phy->mode]);
	}
	/* usb2.0 phy tune param */
	ret = device_property_read_u32(&pdev->dev, "aw,phy_tune_param", &param);
	if (ret) {
		phy->param = PHY_TUNE_PARAM_INVALID;
	} else {
		phy->param = param;
	}

	phy->phy = devm_phy_create(dev, NULL, &sunxi_phy_ops);
	if (IS_ERR(phy->phy)) {
		sunxi_err(dev, "failed to create PHY\n");
		return PTR_ERR(phy->phy);
	}

	phy_set_drvdata(phy->phy, phy);
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	sunxi_info(dev, "phy provider register success\n");

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct sunxi_phy_of_data sunxi_phy_v1_of_data = {
	.has_vbusvldext = true,
};

static const struct sunxi_phy_of_data sunxi_phy_v2_of_data = {
	.has_vbusvldext = false,
};
static const struct of_device_id phy_sunxi_plat_of_match[] = {
	{
		.compatible = "allwinner,sunxi-plat-phy",
		.data = &sunxi_phy_v1_of_data,
	},
	{
		.compatible = "allwinner,sunxi-plat-phy-v2",
		.data = &sunxi_phy_v2_of_data,
	},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, phy_sunxi_plat_of_match);

static struct platform_driver phy_sunxi_plat_driver = {
	.probe	= phy_sunxi_plat_probe,
	.driver = {
		.of_match_table	= phy_sunxi_plat_of_match,
		.name  = "sunxi-plat-phy",
	}
};
module_platform_driver(phy_sunxi_plat_driver);

MODULE_ALIAS("platform:sunxi-plat-phy");
MODULE_DESCRIPTION("Allwinner Platform USB 3.0 PHY driver");
MODULE_AUTHOR("kanghoupeng<kanghoupeng@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.3");
