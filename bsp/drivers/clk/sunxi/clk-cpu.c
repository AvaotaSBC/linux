/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (C) 2019 Allwinnertech.
 * Author:huanghuafeng <huafenghuang@allwinnertech.com>
 *
 * base on clk/samsung/clk-cpu.c
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Adjustable factor-based clock implementation
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include "clk-cpu.h"

static int sunxi_cpuclk_pre_rate_change(struct clk_notifier_data *ndata,
		struct sunxi_cpuclk *cpuclk)
{
	int ret;
	struct clk *clk = cpuclk->clk;
	struct clk *parent = cpuclk->alt_parent;

	ret =  clk_set_parent(clk, parent);

	return ret;
}

static int sunxi_cpuclk_post_rate_change(struct clk_notifier_data *ndata,
		struct sunxi_cpuclk *cpuclk)
{
	int ret;
	struct clk *clk = cpuclk->clk;
	struct clk *parent = cpuclk->parent;
	int flags = cpuclk->periph->flags;

	/* workaround: In order to ensure the stability of CPU frequency modulation */
	if (flags & CLK_CPU_CHANGE_STABLE)
		msleep(3);
	ret =  clk_set_parent(clk, parent);

	return ret;

}

/*
 * This notifier function is called for the pre-rate and post-rate change
 * notifications of the parent clock of cpuclk.
 */
static int sunxi_cpuclk_notifier_cb(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct clk_notifier_data *ndata = data;
	struct sunxi_cpuclk *cpuclk;
	int ret;

	cpuclk = container_of(nb, struct sunxi_cpuclk, clk_nb);

	if (event == PRE_RATE_CHANGE)
		ret = sunxi_cpuclk_pre_rate_change(ndata, cpuclk);
	else if (event == POST_RATE_CHANGE)
		ret = sunxi_cpuclk_post_rate_change(ndata, cpuclk);

	return notifier_from_errno(ret);
}

struct clk *sunxi_clk_register_cpu(struct periph_init_data *pd,
	void __iomem  *base, const char *alt_parent, const char *parent)
{
	struct sunxi_cpuclk *cpuclk;
	struct clk *clk;
	struct clk_init_data init;
	int ret;

	BUG_ON((pd == NULL) && (pd->periph == NULL));

	cpuclk = kzalloc(sizeof(*cpuclk), GFP_KERNEL);
	if (!cpuclk) {
		pr_err("%s: could not allocate cpuclk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

#ifdef __SUNXI_ALL_CLK_IGNORE_UNUSED__
	pd->flags |= CLK_IGNORE_UNUSED;
#endif

	cpuclk->periph = pd->periph;
	init.name = pd->name;

	init.ops = cpuclk->periph->priv_clkops
			? cpuclk->periph->priv_clkops
			: (&sunxi_clk_periph_ops);

	init.flags = pd->flags;
	init.parent_names = pd->parent_names;
	init.num_parents = pd->num_parents;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	cpuclk->periph->hw.init = &init;
	cpuclk->periph->flags = init.flags;

	/* fix registers */
	cpuclk->periph->mux.reg = cpuclk->periph->mux.reg ? (base
			+ (unsigned long __force)cpuclk->periph->mux.reg) : NULL;

	cpuclk->periph->divider.reg = cpuclk->periph->divider.reg ? (base
			+ (unsigned long __force)cpuclk->periph->divider.reg) : NULL;

	cpuclk->periph->gate.enable = cpuclk->periph->gate.enable ? (base
			+ (unsigned long __force)cpuclk->periph->gate.enable) : NULL;

	cpuclk->periph->gate.reset = cpuclk->periph->gate.reset ? (base
			+ (unsigned long __force)cpuclk->periph->gate.reset) : NULL;

	cpuclk->periph->gate.bus = cpuclk->periph->gate.bus ? (base
			+ (unsigned long __force)cpuclk->periph->gate.bus) : NULL;

	cpuclk->periph->gate.dram = cpuclk->periph->gate.dram ? (base
			+ (unsigned long __force)cpuclk->periph->gate.dram) : NULL;

	cpuclk->clk_nb.notifier_call = sunxi_cpuclk_notifier_cb;

	cpuclk->alt_parent = __clk_lookup(alt_parent);
	if (IS_ERR(cpuclk->alt_parent)) {
		pr_err("%s: could not lookup alternate parent %s\n",
					__func__, alt_parent);
		return cpuclk->alt_parent;
	}

	cpuclk->parent = __clk_lookup(parent);
	if (IS_ERR(cpuclk->parent)) {
		pr_err("%s: could not lookup parent clock %s\n",
			__func__, parent);
		return cpuclk->parent;
	}

	ret = clk_notifier_register(cpuclk->parent, &cpuclk->clk_nb);
	if (ret) {
		pr_err("%s: failed to register clock notifier for %s\n",
			__func__, parent);
		return ERR_PTR(ret);
	}

	clk = clk_register(NULL, &cpuclk->periph->hw);
	if (IS_ERR(clk))
		return clk;

	cpuclk->clk = clk;
	return clk;
}


