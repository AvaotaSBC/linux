// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * PCIe driver for Allwinner Soc
 *
 * Copyright (C) 2022 Allwinner Co., Ltd.
 *
 * Author: songjundong <songjundong@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>

#include "pci.h"
#include "pcie-sunxi-dma.h"
#include "pcie-sunxi.h"

#define SUNXI_PCIE_MODULE_VERSION	"1.0.8"

void sunxi_pcie_writel(u32 val, struct sunxi_pcie *pcie, u32 offset)
{
	writel(val, pcie->app_base + offset);
}

u32 sunxi_pcie_readl(struct sunxi_pcie *pcie, u32 offset)
{
	return readl(pcie->app_base + offset);
}

void sunxi_pcie_writel_dbi(struct sunxi_pcie *pci, u32 reg, u32 val)
{
	sunxi_pcie_write_dbi(pci, reg, 0x4, val);
}

u32 sunxi_pcie_readl_dbi(struct sunxi_pcie *pci, u32 reg)
{
	return sunxi_pcie_read_dbi(pci, reg, 0x4);
}

void sunxi_pcie_writew_dbi(struct sunxi_pcie *pci, u32 reg, u16 val)
{
	sunxi_pcie_write_dbi(pci, reg, 0x2, val);
}

u16 sunxi_pcie_readw_dbi(struct sunxi_pcie *pci, u32 reg)
{
	return sunxi_pcie_read_dbi(pci, reg, 0x2);
}

void sunxi_pcie_writeb_dbi(struct sunxi_pcie *pci, u32 reg, u8 val)
{
	sunxi_pcie_write_dbi(pci, reg, 0x1, val);
}

u8 sunxi_pcie_readb_dbi(struct sunxi_pcie *pci, u32 reg)
{
	return sunxi_pcie_read_dbi(pci, reg, 0x1);
}

void sunxi_pcie_dbi_ro_wr_en(struct sunxi_pcie *pci)
{
	u32 val;

	val = sunxi_pcie_readl_dbi(pci, PCIE_MISC_CONTROL_1_CFG);
	val |= (0x1 << 0);
	sunxi_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_CFG, val);
}

void sunxi_pcie_dbi_ro_wr_dis(struct sunxi_pcie *pci)
{
	u32 val;

	val = sunxi_pcie_readl_dbi(pci, PCIE_MISC_CONTROL_1_CFG);
	val &= ~(0x1 << 0);
	sunxi_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_CFG, val);
}

static void sunxi_pcie_plat_set_mode(struct sunxi_pcie *pci)
{
	u32 val;

	switch (pci->drvdata->mode) {
	case SUNXI_PCIE_EP_TYPE:
		val = sunxi_pcie_readl(pci, PCIE_LTSSM_CTRL);
		val |= ~DEVICE_TYPE_MASK;
		sunxi_pcie_writel(val, pci, PCIE_LTSSM_CTRL);
		break;
	case SUNXI_PCIE_RC_TYPE:
		val = sunxi_pcie_readl(pci, PCIE_LTSSM_CTRL);
		val |= DEVICE_TYPE_RC;
		sunxi_pcie_writel(val, pci, PCIE_LTSSM_CTRL);
		break;
	default:
		dev_err(pci->dev, "unsupported device type:%d\n", pci->drvdata->mode);
		break;
	}
}

static u8 __sunxi_pcie_find_next_cap(struct sunxi_pcie *pci, u8 cap_ptr,
						u8 cap)
{
	u8 cap_id, next_cap_ptr;
	u16 reg;

	if (!cap_ptr)
		return 0;

	reg = sunxi_pcie_readw_dbi(pci, cap_ptr);
	cap_id = (reg & CAP_ID_MASK);

	if (cap_id > PCI_CAP_ID_MAX)
		return 0;

	if (cap_id == cap)
		return cap_ptr;

	next_cap_ptr = (reg & NEXT_CAP_PTR_MASK) >> 8;
	return __sunxi_pcie_find_next_cap(pci, next_cap_ptr, cap);
}

static u8 sunxi_pcie_plat_find_capability(struct sunxi_pcie *pci, u8 cap)
{
	u8 next_cap_ptr;
	u16 reg;

	reg = sunxi_pcie_readw_dbi(pci, PCI_CAPABILITY_LIST);
	next_cap_ptr = (reg & CAP_ID_MASK);

	return __sunxi_pcie_find_next_cap(pci, next_cap_ptr, cap);
}

int sunxi_pcie_cfg_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 4) {
		*val = readl(addr);
	} else if (size == 2) {
		*val = readw(addr);
	} else if (size == 1) {
		*val = readb(addr);
	} else {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_cfg_read);

int sunxi_pcie_cfg_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_cfg_write);

void sunxi_pcie_write_dbi(struct sunxi_pcie *pci, u32 reg, size_t size, u32 val)
{
	int ret;

	ret = sunxi_pcie_cfg_write(pci->dbi_base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "Write DBI address failed\n");
}
EXPORT_SYMBOL_GPL(sunxi_pcie_write_dbi);

u32 sunxi_pcie_read_dbi(struct sunxi_pcie *pci, u32 reg, size_t size)
{
	int ret;
	u32 val;

	ret = sunxi_pcie_cfg_read(pci->dbi_base + reg, size, &val);
	if (ret)
		dev_err(pci->dev, "Read DBI address failed\n");

	return val;
}
EXPORT_SYMBOL_GPL(sunxi_pcie_read_dbi);

static void sunxi_pcie_plat_set_link_cap(struct sunxi_pcie *pci, u32 link_gen)
{
	u32 cap, ctrl2, link_speed;

	u8 offset = sunxi_pcie_plat_find_capability(pci, PCI_CAP_ID_EXP);

	cap = sunxi_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
	ctrl2 = sunxi_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCTL2);
	ctrl2 &= ~PCI_EXP_LNKCTL2_TLS;

	switch (pcie_link_speed[link_gen]) {
	case PCIE_SPEED_2_5GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_2_5GT;
		break;
	case PCIE_SPEED_5_0GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_5_0GT;
		break;
	case PCIE_SPEED_8_0GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_8_0GT;
		break;
	case PCIE_SPEED_16_0GT:
		link_speed = PCI_EXP_LNKCTL2_TLS_16_0GT;
		break;
	default:
		/* Use hardware capability */
		link_speed = FIELD_GET(PCI_EXP_LNKCAP_SLS, cap);
		ctrl2 &= ~PCI_EXP_LNKCTL2_HASD;
		break;
	}

	sunxi_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCTL2, ctrl2 | link_speed);

	cap &= ~((u32)PCI_EXP_LNKCAP_SLS);
	sunxi_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, cap | link_speed);
}

void sunxi_pcie_plat_set_rate(struct sunxi_pcie *pci)
{
	u32 val;

	sunxi_pcie_plat_set_link_cap(pci, pci->link_gen);
	/* set the number of lanes */
	val = sunxi_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_MODE_MASK;
	switch (pci->lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	default:
		dev_err(pci->dev, "num-lanes %u: invalid value\n", pci->lanes);
		return;
	}
	sunxi_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	/* set link width speed control register */
	val = sunxi_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (pci->lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	}
	sunxi_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_plat_set_rate);

static unsigned int sunxi_pcie_ep_func_conf_select(struct sunxi_pcie_ep *ep,
						u8 func_no)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_ep(ep);

	WARN_ON(func_no && !pcie->drvdata->func_offset);
	return pcie->drvdata->func_offset * func_no;
}

static const struct sunxi_pcie_ep_ops sunxi_ep_ops = {
	.func_conf_select = sunxi_pcie_ep_func_conf_select,
};

static const struct sunxi_pcie_of_data sunxi_pcie_rc_v210_of_data = {
	.mode = SUNXI_PCIE_RC_TYPE,
	.cpu_pcie_addr_quirk = true,
};

static const struct sunxi_pcie_of_data sunxi_pcie_rc_v300_of_data = {
	.mode = SUNXI_PCIE_RC_TYPE,
};

static const struct sunxi_pcie_of_data sunxi_pcie_ep_v300_of_data = {
	.mode = SUNXI_PCIE_EP_TYPE,
	.func_offset = 0x1000,
	.ops = &sunxi_ep_ops,
};

static const struct of_device_id sunxi_pcie_plat_of_match[] = {
	{
		.compatible = "allwinner,sunxi-pcie-v210-rc",
		.data = &sunxi_pcie_rc_v210_of_data,
	},
	{
		.compatible = "allwinner,sunxi-pcie-v300-rc",
		.data = &sunxi_pcie_rc_v300_of_data,
	},
	{
		.compatible = "allwinner,sunxi-pcie-v300-ep",
		.data = &sunxi_pcie_ep_v300_of_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_pcie_plat_of_match);

#if defined(CONFIG_AW_FPGA_S4) || defined(CONFIG_AW_FPGA_V7)
static inline void sunxi_pcie_writel_phy(struct sunxi_pcie *pci, u32 val, u32 reg)
{
	writel(val, pci->phy_base + reg);
}

static inline u32 sunxi_pcie_readl_phy(struct sunxi_pcie *pci, u32 reg)
{
	return readl(pci->phy_base + reg);
}
#endif

void sunxi_pcie_plat_ltssm_enable(struct sunxi_pcie *pcie)
{
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_CTRL);
	val |= PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_CTRL);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_plat_ltssm_enable);

void sunxi_pcie_plat_ltssm_disable(struct sunxi_pcie *pcie)
{
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_LTSSM_CTRL);
	val &= ~PCIE_LINK_TRAINING;
	sunxi_pcie_writel(val, pcie, PCIE_LTSSM_CTRL);
}
EXPORT_SYMBOL_GPL(sunxi_pcie_plat_ltssm_disable);

static void sunxi_pcie_plat_irqpending(struct sunxi_pcie_port *pp)
{
	struct sunxi_pcie *pcie = to_sunxi_pcie_from_pp(pp);
	u32 val;

	val = sunxi_pcie_readl(pcie, PCIE_INT_ENABLE_CLR);
	val &= ~PCIE_LINK_INT_EN;
	sunxi_pcie_writel(val, pcie, PCIE_INT_ENABLE_CLR);
}

static void sunxi_pcie_plat_set_irqmask(struct sunxi_pcie *pci)
{
	u32 val;

	val = sunxi_pcie_readl(pci, PCIE_INT_ENABLE_CLR);
	val |= PCIE_LINK_INT_EN;
	sunxi_pcie_writel(val, pci, PCIE_INT_ENABLE_CLR);
}

static int sunxi_pcie_plat_enable_power(struct sunxi_pcie *pci)
{
	struct device *dev = pci->dev;
	int ret = 0;

	if (IS_ERR_OR_NULL(pci->pcie3v3))
		return 1;

	ret = regulator_set_voltage(pci->pcie3v3, 3300000, 3300000);
	if (ret)
		dev_warn(dev, "failed to set regulator voltage\n");

	ret = regulator_enable(pci->pcie3v3);
	if (ret)
		dev_err(dev, "failed to enable pcie3v3 regulator\n");

	if (IS_ERR_OR_NULL(pci->pcie1v8))
		return 1;

	ret = regulator_set_voltage(pci->pcie1v8, 1800000, 1800000);
	if (ret)
		dev_warn(dev, "failed to set regulator voltage\n");

	ret = regulator_enable(pci->pcie1v8);
	if (ret)
		dev_err(dev, "failed to enable pcie1v8 regulator\n");

	return ret;
}

static int sunxi_pcie_plat_disable_power(struct sunxi_pcie *pci)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(pci->pcie3v3))
		return ret;

	ret = regulator_disable(pci->pcie3v3);
	if (ret)
		dev_err(pci->dev, "fail to disable pcie3v3 regulator\n");

	if (IS_ERR_OR_NULL(pci->pcie1v8))
		return ret;

	ret = regulator_disable(pci->pcie1v8);
	if (ret)
		dev_err(pci->dev, "fail to disable pcie1v8 regulator\n");

	return ret;
}

static int sunxi_pcie_plat_clk_setup(struct sunxi_pcie *pci)
{
	int ret;

	ret = clk_prepare_enable(pci->pcie_ref);
	if (ret) {
		dev_err(pci->dev, "cannot prepare/enable ref clock\n");
		return ret;
	}

	ret = clk_set_rate(pci->pcie_ref, 100000000);
	if (ret) {
		dev_err(pci->dev, "failed to set clock freq 100M!\n");
		goto err0;
	}

	ret = clk_prepare_enable(pci->pcie_aux);
	if (ret) {
		dev_err(pci->dev, "cannot prepare/enable aux clock\n");
		goto err0;
	}

	return 0;

err0:
	clk_disable_unprepare(pci->pcie_ref);

	return ret;
}

static void sunxi_pcie_plat_clk_exit(struct sunxi_pcie *pci)
{
	clk_disable_unprepare(pci->pcie_aux);
	clk_disable_unprepare(pci->pcie_ref);
}

static int sunxi_pcie_plat_clk_get(struct platform_device *pdev, struct sunxi_pcie *pci)
{
	int ret;

	pci->pcie_ref = devm_clk_get(&pdev->dev, "pclk_ref");
	if (IS_ERR(pci->pcie_ref)) {
		dev_err(&pdev->dev, "failed to clk pclk_ref\n");
		return PTR_ERR(pci->pcie_ref);
	}

	pci->pcie_per = devm_clk_get(&pdev->dev, "pclk_per");
	if (IS_ERR(pci->pcie_per)) {
		dev_err(&pdev->dev, "failed to get pclk_per\n");
		return PTR_ERR(pci->pcie_per);
	}

	ret = clk_set_parent(pci->pcie_ref, pci->pcie_per);
	if (ret) {
		dev_err(&pdev->dev, "failed to set parent\n");
		return -EINVAL;
	}

	pci->pcie_aux = devm_clk_get(&pdev->dev, "pclk_aux");
	if (IS_ERR(pci->pcie_aux)) {
		dev_err(&pdev->dev, "fail to get pclk_aux\n");
		return PTR_ERR(pci->pcie_aux);
	}

	return 0;
}

static int sunxi_pcie_plat_combo_phy_init(struct sunxi_pcie *pci)
{
	int ret;

	ret = phy_init(pci->phy);
	if (ret) {
		dev_err(pci->dev, "fail to init phy, err %d\n", ret);
		return ret;
	}

	return 0;
}

static void sunxi_pcie_plat_combo_phy_deinit(struct sunxi_pcie *pci)
{
	phy_exit(pci->phy);
}

static irqreturn_t sunxi_pcie_plat_sii_handler(int irq, void *arg)
{
	struct sunxi_pcie_port *pp = (struct sunxi_pcie_port *)arg;

	sunxi_pcie_plat_irqpending(pp);

	return IRQ_HANDLED;
}

static void sunxi_pcie_plat_dma_handle_interrupt(u32 ch, enum dma_dir dma_trx)
{
	int ret;

	ret = sunxi_pcie_dma_chan_release(ch, dma_trx);
	if (unlikely(ret < 0)) {
		pr_err("%s is error release chnl%d !\n", __func__, ch);
	}
}

#define SUNXI_PCIE_DMA_IRQ_HANDLER(name, chn, dir)				\
static irqreturn_t sunxi_pcie_##name##_irq_handler				\
						(int irq, void *arg)		\
{										\
	struct sunxi_pcie *pci = arg;						\
	union int_status sta = {0};						\
	union int_clear  clr = {0};                                             \
												  \
	sta.dword = sunxi_pcie_readl_dbi(pci, PCIE_DMA_OFFSET +					  \
					(dir ? PCIE_DMA_RD_INT_STATUS : PCIE_DMA_WR_INT_STATUS)); \
												  \
	if (sta.done & BIT(chn)) {							          \
		clr.doneclr = BIT(chn);								  \
		sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET +					  \
				(dir ? PCIE_DMA_RD_INT_CLEAR : PCIE_DMA_WR_INT_CLEAR), clr.dword);\
		sunxi_pcie_plat_dma_handle_interrupt(chn, dir);					  \
	}											  \
												  \
	if (sta.abort & BIT(chn)) {								  \
		clr.abortclr = BIT(chn);							  \
		sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET +					  \
				(dir ? PCIE_DMA_RD_INT_CLEAR : PCIE_DMA_WR_INT_CLEAR), clr.dword);\
		dev_err(pci->dev, "DMA %s channel %d is abort\n",				  \
							dir ? "read":"write", chn);		  \
	}											  \
												  \
	return IRQ_HANDLED;									  \
}

SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w0, 0, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w1, 1, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w2, 2, PCIE_DMA_WRITE)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_w3, 3, PCIE_DMA_WRITE)

SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r0, 0, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r1, 1, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r2, 2, PCIE_DMA_READ)
SUNXI_PCIE_DMA_IRQ_HANDLER(dma_r3, 3, PCIE_DMA_READ)

static void sunxi_pcie_plat_dma_read(struct sunxi_pcie *pci, struct dma_table *table)
{
	int offset = PCIE_DMA_OFFSET + table->start.chnl * 0x200;

	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_ENB,
							table->enb.dword);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_CTRL_LO,
							table->ctx_reg.ctrllo.dword);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_CTRL_HI,
							table->ctx_reg.ctrlhi.dword);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_XFERSIZE,
							table->ctx_reg.xfersize);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_SAR_LO,
							table->ctx_reg.sarptrlo);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_SAR_HI,
							table->ctx_reg.sarptrhi);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_DAR_LO,
							table->ctx_reg.darptrlo);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_DAR_HI,
							table->ctx_reg.darptrhi);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_RD_WEILO,
							table->weilo.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_DOORBELL,
							table->start.dword);
}

static void sunxi_pcie_plat_dma_write(struct sunxi_pcie *pci, struct dma_table *table)
{
	int offset = PCIE_DMA_OFFSET + table->start.chnl * 0x200;

	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_ENB,
							table->enb.dword);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_CTRL_LO,
							table->ctx_reg.ctrllo.dword);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_CTRL_HI,
							table->ctx_reg.ctrlhi.dword);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_XFERSIZE,
							table->ctx_reg.xfersize);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_SAR_LO,
							table->ctx_reg.sarptrlo);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_SAR_HI,
							table->ctx_reg.sarptrhi);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_DAR_LO,
							table->ctx_reg.darptrlo);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_DAR_HI,
							table->ctx_reg.darptrhi);
	sunxi_pcie_writel_dbi(pci, offset + PCIE_DMA_WR_WEILO,
							table->weilo.dword);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_DOORBELL,
							table->start.dword);
}

/*
 * DMA controller: I/O and Type 0 or Type 1 configuration DMA
 * transfers are not supported.
 * Transfer size: 1B - 4GB
 */
static void sunxi_pcie_plat_dma_start(struct dma_table *table, struct dma_trx_obj *obj)
{
	struct sunxi_pcie *pci = dev_get_drvdata(obj->dev);

	if (table->dir == PCIE_DMA_READ) {
		sunxi_pcie_plat_dma_read(pci, table);
	} else if (table->dir == PCIE_DMA_WRITE) {
		sunxi_pcie_plat_dma_write(pci, table);
	}
}

static int sunxi_pcie_plat_dma_config(struct dma_table *table, phys_addr_t sar_addr, phys_addr_t dar_addr,
					unsigned int size, enum dma_dir dma_trx)
{
	dma_channel_t *chn = NULL;

	table->ctx_reg.ctrllo.lie   = 0x1;
	table->ctx_reg.ctrllo.rie   = 0x0;
	table->ctx_reg.ctrllo.td    = 0x1;
	table->ctx_reg.ctrlhi.dword = 0x0;
	table->ctx_reg.xfersize = size;
	table->ctx_reg.sarptrlo = (u32)(sar_addr & 0xffffffff);
	table->ctx_reg.sarptrhi = (u32)(sar_addr >> 32);
	table->ctx_reg.darptrlo = (u32)(dar_addr & 0xffffffff);
	table->ctx_reg.darptrhi = (u32)(dar_addr >> 32);
	table->start.stop = 0x0;
	table->dir = dma_trx;

	chn = (dma_channel_t *)sunxi_pcie_dma_chan_request(dma_trx);
	if (!chn) {
		pr_err("pcie request %s channel error! \n", (dma_trx ? "DMA_READ" : "DMA_WRITE"));
		return -ENOMEM;
	}

	table->start.chnl = chn->chnl_num;

	table->weilo.dword = (PCIE_WEIGHT << (5 * chn->chnl_num));
	table->enb.enb = 0x1;
	return 0;
}

static int sunxi_pcie_plat_request_irq(struct sunxi_pcie *sunxi_pcie, struct platform_device *pdev)
{
	int irq, ret;

	irq  = platform_get_irq_byname(pdev, "sii");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq,
				sunxi_pcie_plat_sii_handler, IRQF_SHARED, "pcie-sii", &sunxi_pcie->pp);
	if (ret) {
		dev_err(&pdev->dev, "PCIe failed to request linkup IRQ\n");
		return ret;
	}

	ret = sunxi_pcie_dma_get_chan(pdev);
	if (ret)
		return -EINVAL;

	irq = platform_get_irq_byname(pdev, "edma-w0");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w0_irq_handler,
			       IRQF_SHARED, "pcie-dma-w0", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w1");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w1_irq_handler,
			       IRQF_SHARED, "pcie-dma-w1", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w2");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w2_irq_handler,
			       IRQF_SHARED, "pcie-dma-w2", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-w3");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_w3_irq_handler,
			       IRQF_SHARED, "pcie-dma-w3", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r0");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r0_irq_handler,
			       IRQF_SHARED, "pcie-dma-r0", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r1");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r1_irq_handler,
			       IRQF_SHARED, "pcie-dma-r1", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r2");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r2_irq_handler,
			       IRQF_SHARED, "pcie-dma-r2", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	irq = platform_get_irq_byname(pdev, "edma-r3");
	if (irq < 0)
		return -EINVAL;

	ret = devm_request_irq(&pdev->dev, irq, sunxi_pcie_dma_r3_irq_handler,
			       IRQF_SHARED, "pcie-dma-r3", sunxi_pcie);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCIe DMA IRQ\n");
		return ret;
	}

	return 0;
}

static int sunxi_pcie_plat_dma_init(struct sunxi_pcie *pci)
{
	pci->dma_obj = sunxi_pcie_dma_obj_probe(pci->dev);

	if (IS_ERR(pci->dma_obj)) {
		dev_err(pci->dev, "failed to prepare dma obj probe\n");
		return -EINVAL;
	}

	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_INT_MASK, 0x0);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_INT_MASK, 0x0);
	return 0;
}

static void sunxi_pcie_plat_dma_deinit(struct sunxi_pcie *pci)
{
	sunxi_pcie_dma_obj_remove(pci->dev);

	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_WR_INT_MASK, PCIE_DMA_INT_MASK);
	sunxi_pcie_writel_dbi(pci, PCIE_DMA_OFFSET + PCIE_DMA_RD_INT_MASK, PCIE_DMA_INT_MASK);
}

static int sunxi_pcie_plat_parse_dts_res(struct platform_device *pdev, struct sunxi_pcie *pci)
{
	struct sunxi_pcie_port *pp = &pci->pp;
	struct device_node *np = pp->dev->of_node;
	struct resource *dbi_res;
	int ret;

	dbi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!dbi_res) {
		dev_err(&pdev->dev, "get pcie dbi failed\n");
		return -ENODEV;
	}

	pci->dbi_base = devm_ioremap_resource(&pdev->dev, dbi_res);
	if (IS_ERR(pci->dbi_base)) {
		dev_err(&pdev->dev, "ioremap pcie dbi failed\n");
		return PTR_ERR(pci->dbi_base);
	}

	pp->dbi_base = pci->dbi_base;
	pci->app_base = pci->dbi_base + PCIE_USER_DEFINED_REGISTER;

	pci->link_gen = of_pci_get_max_link_speed(pdev->dev.of_node);
	if (pci->link_gen < 0) {
		dev_warn(&pdev->dev, "get pcie speed Gen failed\n");
		pci->link_gen = 0x1;
	}

	pci->rst_gpio = devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pci->rst_gpio))
		dev_warn(&pdev->dev, "Failed to get \"reset-gpios\"\n");
	else
		gpiod_direction_output(pci->rst_gpio, 1);

	pci->pcie3v3 = devm_regulator_get_optional(&pdev->dev, "pcie3v3");
	if (IS_ERR(pci->pcie3v3))
		dev_warn(&pdev->dev, "no pcie3v3 regulator found\n");

	pci->pcie1v8 = devm_regulator_get_optional(&pdev->dev, "pcie1v8");
	if (IS_ERR(pci->pcie1v8))
		dev_warn(&pdev->dev, "no pcie1v8 regulator found\n");

	ret = of_property_read_u32(np, "num-lanes", &pci->lanes);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse the number of lanes\n");
		return -EINVAL;
	}

	pp->cpu_pcie_addr_quirk = pci->drvdata->cpu_pcie_addr_quirk;

	ret = sunxi_pcie_plat_clk_get(pdev, pci);
	if (ret) {
		dev_err(&pdev->dev, "pcie get clk init failed\n");
		return -ENODEV;
	}

	pci->phy = devm_phy_get(pci->dev, "pcie-phy");
	if (IS_ERR(pci->phy))
		return dev_err_probe(pci->dev, PTR_ERR(pci->phy), "missing PHY\n");

	return 0;
}

static int sunxi_pcie_plat_hw_init(struct sunxi_pcie *pci)
{
	int ret;

	ret = sunxi_pcie_plat_enable_power(pci);
	if (ret)
		return ret;

	ret = sunxi_pcie_plat_clk_setup(pci);
	if (ret)
		goto err0;

	ret = sunxi_pcie_plat_combo_phy_init(pci);
	if (ret)
		goto err1;

	return 0;

err1:
	sunxi_pcie_plat_clk_exit(pci);
err0:
	sunxi_pcie_plat_disable_power(pci);

	return ret;
}

static void sunxi_pcie_plat_hw_deinit(struct sunxi_pcie *pci)
{
	sunxi_pcie_plat_combo_phy_deinit(pci);
	sunxi_pcie_plat_disable_power(pci);
	sunxi_pcie_plat_clk_exit(pci);
}

static int sunxi_pcie_plat_probe(struct platform_device *pdev)
{
	struct sunxi_pcie *pci;
	struct sunxi_pcie_port *pp;
	const struct sunxi_pcie_of_data *data;
	enum sunxi_pcie_device_mode mode;
	int ret;

	data = of_device_get_match_data(&pdev->dev);
	mode = (enum sunxi_pcie_device_mode)data->mode;

	pci = devm_kzalloc(&pdev->dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pp = &pci->pp;
	pp->dev = &pdev->dev;
	pci->dev = &pdev->dev;
	pci->drvdata = data;

	ret = sunxi_pcie_plat_parse_dts_res(pdev, pci);
	if (ret)
		return ret;

	ret = sunxi_pcie_plat_hw_init(pci);
	if (ret)
		return ret;

	sunxi_pcie_plat_set_irqmask(pci);
	platform_set_drvdata(pdev, pci);

	ret = sunxi_pcie_plat_request_irq(pci, pdev);
	if (ret)
		goto err0;

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm_runtime_get_sync failed\n");
		goto err1;
	}

	ret = sunxi_pcie_plat_dma_init(pci);
	if (ret)
		goto err2;

	if (pci->dma_obj) {
		pci->dma_obj->start_dma_trx_func  = sunxi_pcie_plat_dma_start;
		pci->dma_obj->config_dma_trx_func = sunxi_pcie_plat_dma_config;
	}

	switch (pci->drvdata->mode) {
	case SUNXI_PCIE_RC_TYPE:
		ret = sunxi_pcie_host_add_port(pci, pdev);
		break;
	case SUNXI_PCIE_EP_TYPE:
		sunxi_pcie_plat_set_mode(pci);
		pci->ep.ops = &sunxi_ep_ops;
		ret = sunxi_pcie_ep_init(pci);
		break;
	default:
		dev_err(&pdev->dev, "INVALID device type %d\n", pci->drvdata->mode);
		ret = -EINVAL;
		break;
	}

	if (ret)
		goto err3;

	dev_info(&pdev->dev, "driver version: %s\n", SUNXI_PCIE_MODULE_VERSION);

	return 0;

err3:
	sunxi_pcie_plat_dma_deinit(pci);
err2:
	pm_runtime_put(&pdev->dev);
err1:
	pm_runtime_disable(&pdev->dev);
err0:
	sunxi_pcie_plat_hw_deinit(pci);

	return ret;
}

static int sunxi_pcie_plat_remove(struct platform_device *pdev)
{
	struct sunxi_pcie *pci = platform_get_drvdata(pdev);

	sunxi_pcie_plat_hw_deinit(pci);

	pm_runtime_disable(&pdev->dev);

	pm_runtime_put(&pdev->dev);

	sunxi_pcie_plat_dma_deinit(pci);

	switch (pci->drvdata->mode) {
	case SUNXI_PCIE_RC_TYPE:
		sunxi_pcie_host_remove_port(pci);
		break;
	case SUNXI_PCIE_EP_TYPE:
		sunxi_pcie_ep_deinit(pci);
		break;
	default:
		dev_err(&pdev->dev, "unspport device type %d\n", pci->drvdata->mode);
		break;
	}

	sunxi_pcie_plat_ltssm_disable(pci);

	return 0;
}

#if IS_ENABLED(CONFIG_PM)
static int sunxi_pcie_plat_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *pci = platform_get_drvdata(pdev);

	sunxi_pcie_plat_ltssm_disable(pci);

	usleep_range(200, 300);

	sunxi_pcie_plat_hw_deinit(pci);

	return 0;
}

static int sunxi_pcie_plat_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sunxi_pcie *pci = platform_get_drvdata(pdev);
	struct sunxi_pcie_port *pp = &pci->pp;
	int ret;

	ret = sunxi_pcie_plat_hw_init(pci);
	if (ret)
		return -EINVAL;

	/* TODO */
	usleep_range(100, 300);

	switch (pci->drvdata->mode) {
	case SUNXI_PCIE_RC_TYPE:
		sunxi_pcie_plat_ltssm_disable(pci);
		sunxi_pcie_host_setup_rc(pp);

		if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->has_its) {
			phys_addr_t pa = ALIGN_DOWN(virt_to_phys(pp), SZ_4K);
			sunxi_pcie_host_wr_own_conf(pp, PCIE_MSI_ADDR_LO, 4, lower_32_bits(pa));
			sunxi_pcie_host_wr_own_conf(pp, PCIE_MSI_ADDR_HI, 4, upper_32_bits(pa));
		}

		sunxi_pcie_host_establish_link(pci);
		sunxi_pcie_host_speed_change(pci, pci->link_gen);
		break;
	case SUNXI_PCIE_EP_TYPE:
		/* TODO */
		break;
	default:
		dev_err(pci->dev, "unsupport device type %d\n", pci->drvdata->mode);
		break;
	}

	return 0;
}

static struct dev_pm_ops sunxi_pcie_plat_pm_ops = {
	.suspend = sunxi_pcie_plat_suspend,
	.resume = sunxi_pcie_plat_resume,
};
#else
static struct dev_pm_ops sunxi_pcie_plat_pm_ops;
#endif /* CONFIG_PM */

static struct platform_driver sunxi_pcie_plat_driver = {
	.driver = {
		.name	= "sunxi-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = sunxi_pcie_plat_of_match,
		.pm = &sunxi_pcie_plat_pm_ops,
	},
	.probe  = sunxi_pcie_plat_probe,
	.remove = sunxi_pcie_plat_remove,
};

module_platform_driver(sunxi_pcie_plat_driver);

MODULE_AUTHOR("songjundong <songjundong@allwinnertech.com>");
MODULE_DESCRIPTION("Allwinner PCIe controller platform driver");
MODULE_VERSION(SUNXI_PCIE_MODULE_VERSION);
MODULE_LICENSE("GPL v2");
