/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2024 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner's ALSA SoC Audio driver
 *
 * Copyright (c) 2024, zhouxijing <zhouxijing@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/device.h>
#include <linux/regmap.h>

#include "snd_sunxi_log.h"
#include "snd_sunxi_owa.h"

struct sunxi_owa_clk {
	/* module clk */
	struct clk *clk_owa_tx;
	struct clk *clk_owa_rx;

	struct clk *clk_bus;
	struct reset_control *clk_rst;
};

sunxi_owa_clk_t *snd_owa_clk_init(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct sunxi_owa_clk *clk = NULL;

	SND_LOG_DEBUG("\n");

	clk = kzalloc(sizeof(*clk), GFP_KERNEL);
	if (!clk) {
		SND_LOG_ERR("can't allocate sunxi_owa_clk memory\n");
		return NULL;
	}

	/* get rst clk */
	clk->clk_rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(clk->clk_rst)) {
		SND_LOG_ERR("clk rst get failed\n");
		ret =  PTR_ERR(clk->clk_rst);
		goto err_get_clk_rst;
	}

	/* get bus clk */
	clk->clk_bus = of_clk_get_by_name(np, "clk_bus_owa");
	if (IS_ERR_OR_NULL(clk->clk_bus)) {
		SND_LOG_ERR("clk bus get failed\n");
		ret = PTR_ERR(clk->clk_bus);
		goto err_get_clk_bus;
	}

	/* get owa clk */
	clk->clk_owa_tx = of_clk_get_by_name(np, "clk_owa_tx");
	if (IS_ERR_OR_NULL(clk->clk_owa_tx)) {
		SND_LOG_ERR("clk owa tx get failed\n");
		ret = PTR_ERR(clk->clk_owa_tx);
		goto err_get_clk_owa_tx;
	}
	clk->clk_owa_rx = of_clk_get_by_name(np, "clk_owa_rx");
	if (IS_ERR_OR_NULL(clk->clk_owa_rx)) {
		SND_LOG_ERR("clk owa rx get failed\n");
		ret = PTR_ERR(clk->clk_owa_rx);
		goto err_get_clk_owa_rx;
	}

	return clk;

err_get_clk_owa_rx:
	clk_put(clk->clk_owa_tx);
err_get_clk_owa_tx:
	clk_put(clk->clk_bus);
err_get_clk_bus:
err_get_clk_rst:
	kfree(clk);
	return NULL;
}

void snd_owa_clk_exit(void *clk_orig)
{
	struct sunxi_owa_clk *clk = (struct sunxi_owa_clk *)clk_orig;

	SND_LOG_DEBUG("\n");

	clk_put(clk->clk_owa_rx);
	clk_put(clk->clk_owa_tx);
	clk_put(clk->clk_bus);

	kfree(clk);
}

int snd_owa_clk_bus_enable(void *clk_orig)
{
	int ret = 0;
	struct sunxi_owa_clk *clk = (struct sunxi_owa_clk *)clk_orig;

	SND_LOG_DEBUG("\n");

	if (reset_control_deassert(clk->clk_rst)) {
		SND_LOG_ERR("clk_rst deassert failed\n");
		ret = -EINVAL;
		goto err_deassert_rst;
	}

	if (clk_prepare_enable(clk->clk_bus)) {
		SND_LOG_ERR("clk_bus enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_bus;
	}

	return 0;

err_enable_clk_bus:
	reset_control_assert(clk->clk_rst);
err_deassert_rst:
	return ret;
}

int snd_owa_clk_enable(void *clk_orig)
{
	int ret = 0;
	struct sunxi_owa_clk *clk = (struct sunxi_owa_clk *)clk_orig;

	SND_LOG_DEBUG("\n");

	if (clk_prepare_enable(clk->clk_owa_tx)) {
		SND_LOG_ERR("clk_owa_tx enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_owa_tx;
	}

	if (clk_prepare_enable(clk->clk_owa_rx)) {
		SND_LOG_ERR("clk_owa_rx enable failed\n");
		ret = -EINVAL;
		goto err_enable_clk_owa_rx;
	}

	return 0;

err_enable_clk_owa_rx:
	clk_disable_unprepare(clk->clk_owa_tx);
err_enable_clk_owa_tx:
	return ret;
}

void snd_owa_clk_bus_disable(void *clk_orig)
{
	struct sunxi_owa_clk *clk = (struct sunxi_owa_clk *)clk_orig;

	SND_LOG_DEBUG("\n");

	clk_disable_unprepare(clk->clk_bus);
	reset_control_assert(clk->clk_rst);
}

void snd_owa_clk_disable(void *clk_orig)
{
	struct sunxi_owa_clk *clk = (struct sunxi_owa_clk *)clk_orig;

	SND_LOG_DEBUG("\n");

	clk_disable_unprepare(clk->clk_owa_rx);
	clk_disable_unprepare(clk->clk_owa_tx);
}

int snd_owa_clk_rate(void *clk_orig, unsigned int freq_in, unsigned int freq_out)
{
	SND_LOG_DEBUG("\n");

	return 0;
}
