/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * (C) Copyright 2010-2015
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * wangjx, 2016-9-9, create this file
 *
 * SoftWinner XHCI Driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/dma-mapping.h>

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/io.h>

#include <linux/clk.h>
#include "sunxi-hci.h"
#include "xhci.h"

#if IS_ENABLED(CONFIG_PM)
static void sunxi_xhci_resume_work(struct work_struct *work);
#endif

#define  SUNXI_XHCI_NAME	"sunxi-xhci"
static const char xhci_name[] = SUNXI_XHCI_NAME;
#define SUNXI_ALIGN_MASK		(16 - 1)

#if IS_ENABLED(CONFIG_USB_SUNXI_XHCI)
#define  SUNXI_XHCI_OF_MATCH	"allwinner,sunxi-xhci"
#else
#define  SUNXI_XHCI_OF_MATCH   "NULL"
#endif

static void sunxi_xhci_open_clock(struct sunxi_hci_hcd *sunxi_xhci);
static void sunxi_set_mode(struct sunxi_hci_hcd *sunxi_xhci, u32 mode);
static void sunxi_core_soft_reset(void __iomem *regs);
static int sunxi_core_open_phy(void __iomem *regs);

static struct sunxi_hci_hcd *g_sunxi_xhci;
static struct sunxi_hci_hcd *g_dev_data;

static ssize_t xhci_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;

	if (dev == NULL) {
		DMSG_WARN("Argment is invalid\n");
		return 0;
	}

	sunxi_xhci = dev->platform_data;
	if (sunxi_xhci == NULL) {
		DMSG_WARN("sunxi_xhci is null\n");
		return 0;
	}

	return sprintf(buf, "xhci:%d, probe:%u\n",
			sunxi_xhci->usbc_no, sunxi_xhci->probe);
}

static ssize_t xhci_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	int value = 0;
	int ret = 0;

	if (dev == NULL) {
		DMSG_WARN("Argment is invalid\n");
		return count;
	}

	sunxi_xhci = dev->platform_data;
	if (sunxi_xhci == NULL) {
		DMSG_WARN("sunxi_xhci is null\n");
		return count;
	}

	ret = kstrtoint(buf, 10, &value);
	if (ret != 0)
		return -EINVAL;
	if (value == 1)
		sunxi_usb_enable_xhci();
	else if (value == 0)
		sunxi_usb_disable_xhci();
	else
		DMSG_INFO("unknown value (%d)\n", value);

	return count;
}

static DEVICE_ATTR(xhci_enable, 0664, xhci_enable_show, xhci_enable_store);

static int xhci_host2_test_mode(void __iomem *regs, int param)
{
	int reg_value = 0;

	switch (param) {
	case TEST_J:
		DMSG_INFO("xhci_host2_test_mode: TEST_J\n");
		break;
	case TEST_K:
		DMSG_INFO("xhci_host2_test_mode: TEST_K\n");
		break;
	case TEST_SE0_NAK:
		DMSG_INFO("xhci_host2_test_mode: TEST_SE0_NAK\n");
		break;
	case TEST_PACKET:
		DMSG_INFO("xhci_host2_test_mode: TEST_PACKET\n");
		break;
	case TEST_FORCE_EN:
		DMSG_INFO("xhci_host2_test_mode: TEST_FORCE_EN\n");
		break;

	default:
		DMSG_INFO("not support test mode(%d)\n", param);
		return -1;
	}

	reg_value = USBC_Readl(regs + XHCI_OP_REGS_HCPORT1SC);
	reg_value &= ~(0x1 << 9);
	USBC_Writel(reg_value, regs + XHCI_OP_REGS_HCPORT1SC);
	msleep(20);

	reg_value = USBC_Readl(regs + XHCI_OP_REGS_HCUSBCMD);
	reg_value &= ~(0x1 << 0);
	USBC_Writel(reg_value, regs + XHCI_OP_REGS_HCUSBCMD);
	msleep(20);

	reg_value = USBC_Readl(regs + XHCI_OP_REGS_HCUSBSTS);
	reg_value &= ~(0x1 << 0);
	USBC_Writel(reg_value, regs + XHCI_OP_REGS_HCUSBSTS);
	msleep(20);
	DMSG_INFO("Halted: 0x%x, param: %d\n",
			USBC_Readl(regs + XHCI_OP_REGS_HCUSBSTS), param);

	reg_value = USBC_Readl(regs + XHCI_OP_REGS_HCPORT1PMSC);
	reg_value |= (param << 28);
	USBC_Writel(reg_value, regs + XHCI_OP_REGS_HCPORT1PMSC);
	msleep(20);
	DMSG_INFO("test_code: %x\n",
			USBC_Readl(regs + XHCI_OP_REGS_HCPORT1PMSC));

	return 0;
}

static ssize_t ed_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;

	if (dev == NULL) {
		DMSG_WARN("Argment is invalid\n");
		return 0;
	}

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL) {
		DMSG_WARN("sunxi_xhci is null\n");
		return 0;
	}

	return sprintf(buf, "USB2.0 host test mode:\n"
				"echo:\ntest_j_state\ntest_k_state\ntest_se0_nak\n"
				"test_pack\ntest_force_enable\n\n"
				"USB3.0 host test mode:\n"
				"echo:\ntest_host3\n");
}

static ssize_t ed_test_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;
	struct usb_hcd *hcd = NULL;
	u32 testmode = 0;

	if (dev == NULL) {
		DMSG_WARN("Argment is invalid\n");
		return count;
	}

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL) {
		DMSG_WARN("sunxi_xhci is null\n");
		return count;
	}

	if (dev_data->probe == 0) {
		DMSG_INFO("[%s]: is disable, can not enter test mode\n",
			dev_data->hci_name);
		return count;
	}

	hcd = dev_get_drvdata(&sunxi_xhci->pdev->dev);
	if (hcd == NULL) {
		DMSG_WARN("xhci hcd is null\n");
		return count;
	}

	/* USB3.0 test mode */
	if (!strncmp(buf, "test_host3", 10)) {
		DMSG_INFO("xhci usb3.0 host test mode\n");
		sunxi_usb_disable_xhci();
		sunxi_xhci_open_clock(dev_data);
		sunxi_core_open_phy(sunxi_xhci->regs);
		sunxi_core_soft_reset(sunxi_xhci->regs);
		sunxi_set_mode(sunxi_xhci, SUNXI_GCTL_PRTCAP_HOST);
		return count;
	}

	/* USB2.0 test mode */
	if (!strncmp(buf, "test_j_state", 12))
		testmode = TEST_J;
	else if (!strncmp(buf, "test_k_state", 12))
		testmode = TEST_K;
	else if (!strncmp(buf, "test_se0_nak", 12))
		testmode = TEST_SE0_NAK;
	else if (!strncmp(buf, "test_pack", 9))
		testmode = TEST_PACKET;
	else if (!strncmp(buf, "test_force_enable", 17))
		testmode = TEST_FORCE_EN;
	else
		testmode = 0;

	xhci_host2_test_mode(hcd->regs, testmode);

	return count;
}

static DEVICE_ATTR(ed_test, 0664, ed_test_show, ed_test_store);

/*
 * sunxi_core_soft_reset - Issues core soft reset and PHY reset
 * @sunxi_xhci: pointer to our context structure
 */
static void sunxi_core_soft_reset(void __iomem *regs)
{
	int reg = 0;

	/* Before Resetting PHY, put Core in Reset */
	reg = USBC_Readl(regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));
	reg |= SUNXI_GCTL_CORESOFTRESET;
	USBC_Writel(reg, regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));

	/* Assert USB3 PHY reset */
	reg = USBC_Readl(regs + (SUNXI_GUSB3PIPECTL(0) - SUNXI_GLOBALS_REGS_START));
	reg |= SUNXI_USB3PIPECTL_PHYSOFTRST;
	USBC_Writel(reg, regs + (SUNXI_GUSB3PIPECTL(0) - SUNXI_GLOBALS_REGS_START));

	/* Assert USB2 PHY reset */
	reg = USBC_Readl(regs + (SUNXI_GUSB2PHYCFG(0) - SUNXI_GLOBALS_REGS_START));
	reg |= SUNXI_USB2PHYCFG_PHYSOFTRST;
	USBC_Writel(reg, regs + (SUNXI_GUSB2PHYCFG(0) - SUNXI_GLOBALS_REGS_START));

	mdelay(100);

	/* Clear USB3 PHY reset */
	reg = USBC_Readl(regs + (SUNXI_GUSB3PIPECTL(0) - SUNXI_GLOBALS_REGS_START));
	reg &= ~SUNXI_USB3PIPECTL_PHYSOFTRST;
	USBC_Writel(reg, regs + (SUNXI_GUSB3PIPECTL(0) - SUNXI_GLOBALS_REGS_START));

	/* Clear USB2 PHY reset */
	reg = USBC_Readl(regs + (SUNXI_GUSB2PHYCFG(0) - SUNXI_GLOBALS_REGS_START));
	reg &= ~SUNXI_USB2PHYCFG_PHYSOFTRST;
	USBC_Writel(reg, regs + (SUNXI_GUSB2PHYCFG(0) - SUNXI_GLOBALS_REGS_START));

	mdelay(100);

	/* After PHYs are stable we can take Core out of reset state */
	reg = USBC_Readl(regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));
	reg &= ~SUNXI_GCTL_CORESOFTRESET;
	USBC_Writel(reg, regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));
}

static int sunxi_core_open_phy(void __iomem *regs)
{
	int reg_val = 0;

	reg_val = USBC_Readl(regs + (SUNXI_PHY_EXTERNAL_CONTROL - SUNXI_GLOBALS_REGS_START));
	reg_val |= SUNXI_PEC_EXTERN_VBUS;  /* Use extern vbus to phy */
	reg_val |= SUNXI_PEC_SSC_EN;  /* SSC_EN */
	reg_val |= SUNXI_PEC_REF_SSP_EN;  /* REF_SSP_EN */
	USBC_Writel(reg_val, regs + (SUNXI_PHY_EXTERNAL_CONTROL - SUNXI_GLOBALS_REGS_START));

	reg_val = USBC_Readl(regs + (SUNXI_PIPE_CLOCK_CONTROL - SUNXI_GLOBALS_REGS_START));
	reg_val |= SUNXI_PPC_PIPE_CLK_OPEN;  /* open PIPE clock */
	USBC_Writel(reg_val, regs + (SUNXI_PIPE_CLOCK_CONTROL - SUNXI_GLOBALS_REGS_START));

	reg_val = USBC_Readl(regs + (SUNXI_APP - SUNXI_GLOBALS_REGS_START));
	reg_val |= SUNXI_APP_FOCE_VBUS;  /* open PIPE clock */
	USBC_Writel(reg_val, regs + (SUNXI_APP - SUNXI_GLOBALS_REGS_START));

	/* It is set 0x0047fc87 on bare-metal. */
	USBC_Writel(0x0047fc57, regs + (SUNXI_PHY_TUNE_LOW - SUNXI_GLOBALS_REGS_START));

	reg_val = USBC_Readl(regs + (SUNXI_PHY_TUNE_HIGH - SUNXI_GLOBALS_REGS_START));
	reg_val |= SUNXI_TXVBOOSTLVL(0x7);
	reg_val |= SUNXI_LOS_BIAS(0x7);

	reg_val &= ~(SUNXI_TX_SWING_FULL(0x7f));
	reg_val |= SUNXI_TX_SWING_FULL(0x55);

	reg_val &= ~(SUNXI_TX_DEEMPH_6DB(0x3f));
	reg_val |= SUNXI_TX_DEEMPH_6DB(0x20);

	reg_val &= ~(SUNXI_TX_DEEMPH_3P5DB(0x3f));
	reg_val |= SUNXI_TX_DEEMPH_3P5DB(0x15);
	USBC_Writel(reg_val, regs + (SUNXI_PHY_TUNE_HIGH - SUNXI_GLOBALS_REGS_START));

	/* Enable USB2.0 PHY Suspend mode. */
	reg_val = USBC_Readl(regs + (SUNXI_GUSB2PHYCFG(0) - SUNXI_GLOBALS_REGS_START));
	reg_val |= SUNXI_USB2PHYCFG_SUSPHY;
	USBC_Writel(reg_val, regs + (SUNXI_GUSB2PHYCFG(0) - SUNXI_GLOBALS_REGS_START));

	/* Enable SOFITPSYNC for suspend. */
	reg_val = USBC_Readl(regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));
	reg_val |= SUNXI_GCTL_SOFITPSYNC;
	USBC_Writel(reg_val, regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));

	return 0;
}

static void sunxi_xhci_open_clock(struct sunxi_hci_hcd *sunxi_xhci)
{
	sunxi_xhci->open_clock(sunxi_xhci, 0);
}

static void sunxi_xhci_set_vbus(struct sunxi_hci_hcd *sunxi_xhci, int is_on)
{
	sunxi_xhci->set_power(sunxi_xhci, is_on);
}

static void sunxi_start_xhci(struct sunxi_hci_hcd *sunxi_xhci)
{
	sunxi_xhci->open_clock(sunxi_xhci, 1);
	sunxi_xhci->set_power(sunxi_xhci, 1);
}

static void sunxi_stop_xhci(struct sunxi_hci_hcd *sunxi_xhci)
{
	sunxi_xhci->set_power(sunxi_xhci, 0);
	sunxi_xhci->close_clock(sunxi_xhci, 0);
}

static void sunxi_set_mode(struct sunxi_hci_hcd *sunxi_xhci, u32 mode)
{
	u32 reg;

	reg = USBC_Readl(sunxi_xhci->regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));
	reg &= ~(SUNXI_GCTL_PRTCAPDIR(SUNXI_GCTL_PRTCAP_OTG));
	reg |= SUNXI_GCTL_PRTCAPDIR(mode);
	USBC_Writel(reg, sunxi_xhci->regs + (SUNXI_GLOBALS_REGS_GCTL - SUNXI_GLOBALS_REGS_START));
}

int xhci_host_init(struct sunxi_hci_hcd *sunxi_xhci)
{
	struct platform_device	*xhci;
	int			ret;

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		sunxi_err(sunxi_xhci->dev, "couldn't allocate xHCI device\n");
		ret = -ENOMEM;
		goto err0;
	}

	dma_set_coherent_mask(&xhci->dev, sunxi_xhci->dev->coherent_dma_mask);

	xhci->dev.parent	= sunxi_xhci->dev;
	xhci->dev.dma_mask	= sunxi_xhci->dev->dma_mask;
	xhci->dev.dma_parms	= sunxi_xhci->dev->dma_parms;
	xhci->dev.archdata.dma_ops	= sunxi_xhci->dev->archdata.dma_ops;
	xhci->dev.archdata.dma_coherent	= sunxi_xhci->dev->archdata.dma_coherent;

	sunxi_xhci->pdev = xhci;

	ret = platform_device_add_resources(xhci, sunxi_xhci->xhci_resources,
						XHCI_RESOURCES_NUM);
	if (ret) {
		sunxi_err(sunxi_xhci->dev, "couldn't add resources to xHCI device\n");
		goto err1;
	}

	ret = platform_device_add(xhci);
	if (ret) {
		sunxi_err(sunxi_xhci->dev, "failed to register xHCI device\n");
		goto err1;
	}

	return 0;

err1:
	platform_device_put(xhci);

err0:
	return ret;
}

void xhci_host_exit(struct sunxi_hci_hcd *sunxi_xhci)
{
	platform_device_unregister(sunxi_xhci->pdev);
}

static int sunxi_xhci_hcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	void			*mem;

	struct device		*dev = &pdev->dev;

	struct resource		*res;
	void __iomem	*regs;

	if (pdev == NULL || dev == NULL) {
		DMSG_ERR("%s, Argment is invalid\n", __func__);
		return -1;
	}

	/* if usb is disabled, can not probe */
	if (usb_disabled()) {
		DMSG_ERR("usb hcd is disabled\n");
		return -ENODEV;
	}

	mem = devm_kzalloc(dev, sizeof(*sunxi_xhci) + SUNXI_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		sunxi_err(dev, "not enough memory\n");
		return -ENOMEM;
	}
	sunxi_xhci = PTR_ALIGN(mem, SUNXI_ALIGN_MASK + 1);
	sunxi_xhci->mem = mem;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		sunxi_err(dev, "missing IRQ\n");
		return -ENODEV;
	}
	sunxi_xhci->xhci_resources[1].start = res->start;
	sunxi_xhci->xhci_resources[1].end = res->end;
	sunxi_xhci->xhci_resources[1].flags = res->flags;
	sunxi_xhci->xhci_resources[1].name = res->name;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		sunxi_err(dev, "missing memory resource\n");
		return -ENODEV;
	}
	sunxi_xhci->xhci_resources[0].start = res->start;
	sunxi_xhci->xhci_resources[0].end = sunxi_xhci->xhci_resources[0].start +
					XHCI_REGS_END;
	sunxi_xhci->xhci_resources[0].flags = res->flags;
	sunxi_xhci->xhci_resources[0].name = res->name;

	/*
	 * Request memory region but exclude xHCI regs,
	 * since it will be requested by the xhci-plat driver.
	 */
	res = devm_request_mem_region(dev, res->start + SUNXI_GLOBALS_REGS_START,
			resource_size(res) - SUNXI_GLOBALS_REGS_START,
			dev_name(dev));
	if (!res) {
		sunxi_err(dev, "can't request mem region\n");
		return -ENOMEM;
	}

	regs = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!regs) {
		sunxi_err(dev, "ioremap failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&sunxi_xhci->lock);

	sunxi_xhci->regs	= regs;
	sunxi_xhci->regs_size	= resource_size(res);
	sunxi_xhci->dev	= dev;

	dev->dma_mask	= dev->parent->dma_mask;
	dev->dma_parms	= dev->parent->dma_parms;
	dma_set_coherent_mask(dev, dev->parent->coherent_dma_mask);

	ret = init_sunxi_hci(pdev, SUNXI_USB_XHCI);
	if (ret != 0) {
		sunxi_err(&pdev->dev, "init_sunxi_hci is fail\n");
		return 0;
	}

	platform_set_drvdata(pdev, sunxi_xhci);

	sunxi_start_xhci(pdev->dev.platform_data);
	sunxi_core_open_phy(sunxi_xhci->regs);
	sunxi_set_mode(sunxi_xhci, SUNXI_GCTL_PRTCAP_HOST);

	xhci_host_init(sunxi_xhci);

	device_create_file(&pdev->dev, &dev_attr_xhci_enable);
	device_create_file(&pdev->dev, &dev_attr_ed_test);

	g_sunxi_xhci = sunxi_xhci;
	g_dev_data = pdev->dev.platform_data;

	g_dev_data->probe = 1;

#if IS_ENABLED(CONFIG_PM)
	if (!g_dev_data->wakeup_suspend)
		INIT_WORK(&g_dev_data->resume_work, sunxi_xhci_resume_work);
#endif

	return 0;
}

static int sunxi_xhci_hcd_remove(struct platform_device *pdev)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;

	if (pdev == NULL) {
		DMSG_ERR("%s, Argment is invalid\n", __func__);
		return -1;
	}

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL) {
		DMSG_ERR("%s, sunxi_xhci is null\n", __func__);
		return -1;
	}

	device_remove_file(&pdev->dev, &dev_attr_xhci_enable);

	xhci_host_exit(sunxi_xhci);
	sunxi_stop_xhci(dev_data);

	dev_data->probe = 0;

	return 0;
}

static void sunxi_xhci_hcd_shutdown(struct platform_device *pdev)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;

	if (pdev == NULL) {
		DMSG_ERR("%s, Argment is invalid\n", __func__);
		return;
	}

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL) {
		DMSG_ERR("%s, is null\n", __func__);
		return;
	}

	if (dev_data->probe == 0) {
		DMSG_INFO("%s, %s is disable, need not shutdown\n",  __func__, sunxi_xhci->hci_name);
		return;
	}

	DMSG_INFO("[%s]: xhci shutdown start\n", sunxi_xhci->hci_name);

	usb_hcd_platform_shutdown(sunxi_xhci->pdev);

	DMSG_INFO("[%s]: xhci shutdown end\n", sunxi_xhci->hci_name);

	return;
}

int sunxi_usb_disable_xhci(void)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL || dev_data == NULL) {
		DMSG_ERR("sunxi_xhci is null\n");
		return -1;
	}

	if (dev_data->probe == 0) {
		DMSG_ERR("sunxi_xhci is disable, can not disable again\n");
		return -1;
	}

	dev_data->probe = 0;

	DMSG_INFO("[%s]: sunxi_usb_disable_xhci\n", sunxi_xhci->hci_name);

	xhci_host_exit(sunxi_xhci);
	sunxi_stop_xhci(dev_data);

	return 0;
}
EXPORT_SYMBOL(sunxi_usb_disable_xhci);

int sunxi_usb_enable_xhci(void)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL || dev_data == NULL) {
		DMSG_ERR("sunxi_xhci is null\n");
		return -1;
	}

	if (dev_data->probe == 1) {
		DMSG_ERR("sunxi_xhci is already enable, can not enable again\n");
		return -1;
	}

	dev_data->probe = 1;

	DMSG_INFO("[%s]: sunxi_usb_enable_xhci\n", sunxi_xhci->hci_name);

	sunxi_start_xhci(dev_data);
	sunxi_core_open_phy(sunxi_xhci->regs);
	sunxi_set_mode(sunxi_xhci, SUNXI_GCTL_PRTCAP_HOST);

	xhci_host_init(sunxi_xhci);

	return 0;
}
EXPORT_SYMBOL(sunxi_usb_enable_xhci);

#if IS_ENABLED(CONFIG_PM)
static int sunxi_xhci_hcd_suspend(struct device *dev)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;
	struct usb_hcd *hcd = NULL;
	struct xhci_hcd *xhci = NULL;

	if (dev == NULL) {
		DMSG_ERR("%s, Argment is invalid\n", __func__);
		return 0;
	}

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL) {
		DMSG_ERR("sunxi_xhci is null\n");
		return 0;
	}

	if (dev_data->probe == 0) {
		DMSG_INFO("[%s]: is disable, need not suspend\n",
			dev_data->hci_name);
		return 0;
	}

	hcd = dev_get_drvdata(&sunxi_xhci->pdev->dev);
	if (hcd == NULL) {
		DMSG_ERR("xhci hcd is null\n");
		return 0;
	}

	xhci = hcd_to_xhci(hcd);
	if (xhci == NULL) {
		DMSG_ERR("xhci is null\n");
		return 0;
	}

	if (dev_data->wakeup_suspend) {
		DMSG_INFO("[%s]: not suspend\n", dev_data->hci_name);
	} else {
		DMSG_INFO("[%s]: sunxi_xhci_hcd_suspend\n", dev_data->hci_name);

		xhci_suspend(xhci, false);
		sunxi_stop_xhci(dev_data);

		cancel_work_sync(&dev_data->resume_work);
	}

	return 0;
}

static void sunxi_xhci_resume_work(struct work_struct *work)
{
	struct sunxi_hci_hcd *dev_data = NULL;

	dev_data = container_of(work, struct sunxi_hci_hcd, resume_work);

	/* Waiting hci to resume. */
	msleep(5000);

	sunxi_xhci_set_vbus(dev_data, 1);
}

static int sunxi_xhci_hcd_resume(struct device *dev)
{
	struct sunxi_hci_hcd *sunxi_xhci = NULL;
	struct sunxi_hci_hcd *dev_data = NULL;
	struct usb_hcd *hcd = NULL;
	struct xhci_hcd *xhci = NULL;

	if (dev == NULL) {
		DMSG_ERR("Argment is invalid\n");
		return 0;
	}

	sunxi_xhci = g_sunxi_xhci;
	dev_data = g_dev_data;
	if (sunxi_xhci == NULL) {
		DMSG_ERR("sunxi_xhci is null\n");
		return 0;
	}

	if (dev_data->probe == 0) {
		DMSG_INFO("[%s]: is disable, need not resume\n",
			dev_data->hci_name);
		return 0;
	}

	hcd = dev_get_drvdata(&sunxi_xhci->pdev->dev);
	if (hcd == NULL) {
		DMSG_ERR("xhci hcd is null\n");
		return 0;
	}

	xhci = hcd_to_xhci(hcd);
	if (xhci == NULL) {
		DMSG_ERR("xhci is null\n");
		return 0;
	}

	if (dev_data->wakeup_suspend) {
		DMSG_INFO("[%s]: controller not suspend, need not resume\n",
				dev_data->hci_name);
	} else {
		DMSG_INFO("[%s]: sunxi_xhci_hcd_resume\n", dev_data->hci_name);

		sunxi_xhci_open_clock(dev_data);
		sunxi_core_open_phy(sunxi_xhci->regs);
		sunxi_set_mode(sunxi_xhci, SUNXI_GCTL_PRTCAP_HOST);
		xhci_resume(xhci, false);

		schedule_work(&dev_data->resume_work);
	}

	return 0;
}

static const struct dev_pm_ops  xhci_pmops = {
	.suspend	= sunxi_xhci_hcd_suspend,
	.resume		= sunxi_xhci_hcd_resume,
};
#endif

static const struct of_device_id sunxi_xhci_match[] = {
	{.compatible = SUNXI_XHCI_OF_MATCH, },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_xhci_match);

static struct platform_driver sunxi_xhci_hcd_driver = {
	.probe  = sunxi_xhci_hcd_probe,
	.remove	= sunxi_xhci_hcd_remove,
	.shutdown = sunxi_xhci_hcd_shutdown,
	.driver = {
			.name	= xhci_name,
			.owner	= THIS_MODULE,
#if IS_ENABLED(CONFIG_PM)
			.pm	= &xhci_pmops,
#endif
			.of_match_table = sunxi_xhci_match,
		}
};

module_platform_driver(sunxi_xhci_hcd_driver);

MODULE_ALIAS("platform:sunxi xhci");
MODULE_AUTHOR("wangjx <wangjx@allwinnertech.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.0.2");
MODULE_DESCRIPTION("Allwinnertech Xhci Controller Driver");
