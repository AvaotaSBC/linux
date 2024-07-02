/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/* sunxi_tcon_v35x.c
 *
 * Copyright (C) 2023 Allwinnertech Co., Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <drm/drm_print.h>
#include <linux/component.h>

#include "sunxi_tcon_top.h"
#include "disp_al_tcon.h"

struct tcon_top_data {
	unsigned int id;
};

struct tcon_top {
	uintptr_t reg_base;
	struct clk *clk_dpss;
	struct reset_control *rst_bus_dpss;
	const struct tcon_top_data *top_data;
};

static int sunxi_tcon_top_bind(struct device *dev, struct device *master,
			       void *data)
{
	struct tcon_top *top;
	struct resource *res;
	struct platform_device *pdev = to_platform_device(dev);

	top = devm_kzalloc(dev, sizeof(*top), GFP_KERNEL);
	if (!top)
		return -ENOMEM;

	top->top_data = of_device_get_match_data(dev);
	if (!top->top_data) {
		DRM_ERROR("sunxi_tcon_top fail to get match data\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	top->reg_base = (uintptr_t)devm_ioremap_resource(dev, res);
	if (!top->reg_base) {
		DRM_ERROR("unable to map tcon top registers\n");
		return -EINVAL;
	}

	top->clk_dpss = devm_clk_get(dev, "clk_bus_dpss_top");
	if (IS_ERR(top->clk_dpss)) {
		DRM_ERROR("fail to get clk dpss_top\n");
		return -EINVAL;
	}

	top->rst_bus_dpss =
		devm_reset_control_get_shared(dev, "rst_bus_dpss_top");
	if (IS_ERR(top->rst_bus_dpss)) {
		DRM_ERROR("fail to get reset rst_bus_dpss_top\n");
		return -EINVAL;
	}

	tcon_top_set_reg_base(top->top_data->id, top->reg_base);
	dev_set_drvdata(dev, top);
	return 0;
}

static void sunxi_tcon_top_unbind(struct device *dev, struct device *master,
				  void *data)
{
}

static const struct component_ops sunxi_tcon_top_component_ops = {
	.bind = sunxi_tcon_top_bind,
	.unbind = sunxi_tcon_top_unbind,
};

static int sunxi_tcon_top_probe(struct platform_device *pdev)
{
	pm_runtime_enable(&pdev->dev);
	return component_add(&pdev->dev, &sunxi_tcon_top_component_ops);
}

static int sunxi_tcon_top_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &sunxi_tcon_top_component_ops);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

/* Note: sunxi-lcd is represented of sunxi tcon,
 * using lcd is order to be same with sunxi display driver
 */

static const struct tcon_top_data top0_data = {
	.id = 0,
};

static const struct tcon_top_data top1_data = {
	.id = 1,
};

static const struct of_device_id sunxi_tcon_top_match[] = {

	{ .compatible = "allwinner,tcon-top0", .data = &top0_data },
	{ .compatible = "allwinner,tcon-top1", .data = &top1_data },
	{},
};

struct platform_driver sunxi_tcon_top_platform_driver = {
	.probe = sunxi_tcon_top_probe,
	.remove = sunxi_tcon_top_remove,
	.driver = {
		   .name = "tcon-top",
		   .owner = THIS_MODULE,
		   .of_match_table = sunxi_tcon_top_match,
	},
};

static bool sunxi_tcon_top_node_is_tcon_top(struct device_node *node)
{
	return !!of_match_node(sunxi_tcon_top_match, node);
}

int sunxi_tcon_top_clk_enable(struct device *tcon_top)
{
	int ret;
	struct tcon_top *topif = dev_get_drvdata(tcon_top);

	if (!sunxi_tcon_top_node_is_tcon_top(tcon_top->of_node)) {
		DRM_ERROR("Device is not TCON TOP!\n");
		return -EINVAL;
	}

	ret = reset_control_deassert(topif->rst_bus_dpss);
	if (ret) {
		DRM_ERROR("reset_control_deassert for rst_bus_dpss failed!\n");
		return ret;
	}

	ret = clk_prepare_enable(topif->clk_dpss);
	if (ret != 0) {
		DRM_ERROR("fail enable topif's clock!\n");
		return ret;
	}
	pm_runtime_get_sync(tcon_top);
	return 0;
}

int sunxi_tcon_top_clk_disable(struct device *tcon_top)
{
	int ret;
	struct tcon_top *topif = dev_get_drvdata(tcon_top);

	if (!sunxi_tcon_top_node_is_tcon_top(tcon_top->of_node)) {
		DRM_ERROR("Device is not TCON TOP!\n");
		return -EINVAL;
	}

	clk_disable_unprepare(topif->clk_dpss);

	ret = reset_control_assert(topif->rst_bus_dpss);
	if (ret) {
		DRM_ERROR(
			"reset_control_deassert for rst_bus_if_top failed!\n");
		return ret;
	}
	pm_runtime_put_sync(tcon_top);

	return 0;
}

int sunxi_tcon_top_module_init(void)
{
	int ret = 0;

	DRM_INFO("%s start\n", __FUNCTION__);

	ret = platform_driver_register(&sunxi_tcon_top_platform_driver);
	if (ret) {
		DRM_ERROR("platform_driver_register failed\n");
	}

	DRM_INFO("%s ret = %d\n", __FUNCTION__, ret);
	return ret;
}

void sunxi_tcon_top_module_exit(void)
{
	DRM_INFO("[SUNXI-TCON]sunxi_tcon_module_exit\n");
	platform_driver_unregister(&sunxi_tcon_top_platform_driver);
}
