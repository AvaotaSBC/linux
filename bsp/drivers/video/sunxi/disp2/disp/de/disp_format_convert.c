/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (C) 2015 Allwinnertech, z.q <zengqi@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "include.h"
#include "disp_format_convert.h"

#if defined SUPPORT_WB

#define FORMAT_MANAGER_NUM 1

static struct format_manager fmt_mgr[FORMAT_MANAGER_NUM];

struct format_manager *disp_get_format_manager(unsigned int id)
{
	return &fmt_mgr[id];
}

#if defined(__LINUX_PLAT__)
s32 disp_format_convert_finish_proc(int irq, void *parg)
#else
s32 disp_format_convert_finish_proc(void *parg)
#endif
{
	struct format_manager *mgr = (struct format_manager *)parg;

	if (mgr == NULL)
		return DISP_IRQ_RETURN;

	if (disp_al_get_eink_wb_status(mgr->disp) == 0) {
		mgr->write_back_finish = 1;
		wake_up_interruptible(&(mgr->write_back_queue));
	} else
		DE_WARN("convert err\n");
	disp_al_clear_eink_wb_interrupt(mgr->disp);

	return DISP_IRQ_RETURN;
}

static s32 disp_format_convert_enable(unsigned int id)
{
	struct format_manager *mgr = &fmt_mgr[id];
	s32 ret = -1;
	static int first = 1;

	if (mgr == NULL) {
		DE_WARN("input param is null\n");
		return -1;
	}

	ret =
	    disp_sys_register_irq(mgr->irq_num, 0,
				  disp_format_convert_finish_proc, (void *)mgr,
				  0, 0);
	if (ret != 0) {
		DE_WARN("fail to format convert irq\n");
		return ret;
	}

	disp_sys_enable_irq(mgr->irq_num);
	if (first) {
		clk_disable_unprepare(mgr->clk);
		first = 0;
	}

	ret = reset_control_deassert(mgr->rst);
	if (ret) {
		DE_WARN("reset_control_deassert for rst failed, ret=%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(mgr->clk);
	if (ret) {
		DE_WARN("clk_prepare_enable for clk failed, ret=%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(mgr->clk_parent);
	if (ret) {
		DE_WARN("clk_prepare_enable for clk_parent failed, ret=%d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(mgr->clk_bus);
	if (ret) {
		DE_WARN("clk_prepare_enable for clk_bus failed\n");
		return ret;
	}

	/* enable de clk, enable write back clk */
	disp_al_de_clk_enable(mgr->disp);
	disp_al_write_back_clk_init(mgr->disp);

	return 0;
}

static s32 disp_format_convert_disable(unsigned int id)
{
	struct format_manager *mgr = &fmt_mgr[id];
	s32 ret = -1;

	if (mgr == NULL) {
		DE_WARN("input param is null\n");
		return -1;
	}

	/* disable write back clk, disable de clk */
	disp_al_write_back_clk_exit(mgr->disp);
	ret = disp_al_de_clk_disable(mgr->disp);
	if (ret != 0)
		return ret;

	clk_disable_unprepare(mgr->clk_bus);
	clk_disable_unprepare(mgr->clk);
	ret = reset_control_assert(mgr->rst);
	if (ret)
		DE_WARN("reset_control_assert for rst failed\n");

	disp_sys_disable_irq(mgr->irq_num);
	disp_sys_unregister_irq(mgr->irq_num, disp_format_convert_finish_proc,
				(void *)mgr);

	return 0;
}

struct disp_manager_data mdata;
struct disp_layer_config_data ldata[16];

static s32 disp_format_convert_start(unsigned int id,
				     struct disp_layer_config_inner *config,
				     unsigned int layer_num,
				     struct image_format *dest)
{
	struct format_manager *mgr = &fmt_mgr[id];
#ifdef EINK_DMABUF_USED
	struct dmabuf_item *item[16] = {NULL};
	struct fb_address_transfer fb;
#endif
	s32 ret = -1, k = 0;
	u32 lay_total_num = 0;

	long timerout = (100 * HZ) / 1000;	/* 100ms */

	if ((dest == NULL) || (mgr == NULL)) {
		DE_WARN("input param is null\n");
		return -1;
	}

	if (dest->format == DISP_FORMAT_8BIT_GRAY) {
		DE_DEBUG("dest_addr = 0x%p\n", (void *)dest->addr1);

		lay_total_num = de_feat_get_num_layers(mgr->disp);
		memset((void *)&mdata, 0, sizeof(struct disp_manager_data));
		memset((void *)ldata, 0, 16 * sizeof(ldata[0]));

		mdata.flag = MANAGER_ALL_DIRTY;
		mdata.config.enable = 1;
		mdata.config.interlace = 0;
		mdata.config.blank = 0;
		mdata.config.size.x = 0;
		mdata.config.size.y = 0;
		mdata.config.size.width = dest->width;
		mdata.config.size.height = dest->height;
		mdata.config.de_freq = clk_get_rate(mgr->clk);

		for (k = 0; k < layer_num; k++) {
			ldata[k].flag = LAYER_ALL_DIRTY;

			memcpy((void *)&ldata[k].config, (void *)&config[k],
					sizeof(*config));

#ifdef EINK_DMABUF_USED
			item[k] = NULL;
			if (ldata[k].config.info.fb.fd && (true == config[k].enable)) {
				item[k] = disp_dma_map(ldata[k].config.info.fb.fd);
				if (item[k] == NULL) {
					DE_WARN("disp wb map fd %d fail\n", ldata[k].config.info.fb.fd);
					return -1;
				}

				DE_DEBUG("layer%d map ok, fd=%d, addr=0x%08x\n",
					k,
					ldata[k].config.info.fb.fd,
					item[k]->dma_addr);

				fb.format = ldata[k].config.info.fb.format;
				memcpy(fb.size, ldata[k].config.info.fb.size,
						sizeof(struct disp_rectsz) * 3);

				memcpy(fb.align, ldata[k].config.info.fb.align,
						sizeof(int) * 3);
				fb.depth = ldata[k].config.info.fb.depth;
				fb.dma_addr = item[k]->dma_addr;
				disp_set_fb_info(&fb, true);
				memcpy(ldata[k].config.info.fb.addr, fb.addr,
						sizeof(long long) * 3);
			}
#endif
		}
		disp_al_manager_apply(mgr->disp, &mdata);
		disp_al_layer_apply(mgr->disp, ldata, layer_num);
		disp_al_manager_sync(mgr->disp);
		disp_al_manager_update_regs(mgr->disp);
		disp_al_eink_wb_reset(mgr->disp);
		disp_al_eink_wb_dereset(mgr->disp);
		disp_al_set_eink_wb_param(mgr->disp, dest->width, dest->height,
				(unsigned int)dest->addr1);

		/* enable inttrrupt */
		disp_al_enable_eink_wb_interrupt(mgr->disp);
		disp_al_enable_eink_wb(mgr->disp);

#ifdef __UBOOT_PLAT__
		/* wait write back complete */
		while (disp_al_get_eink_wb_status(mgr->disp) != EWB_OK)
			mdelay(1);
#else
		timerout =
			wait_event_interruptible_timeout(mgr->write_back_queue,
					(mgr->write_back_finish == 1),
					timerout);
		mgr->write_back_finish = 0;
		if (timerout == 0) {
			DE_WARN("wait write back timeout\n");
			disp_al_disable_eink_wb_interrupt(mgr->disp);
			disp_al_disable_eink_wb(mgr->disp);
			ret = -1;
			goto EXIT;
		}

		disp_al_disable_eink_wb_interrupt(mgr->disp);
#endif
		disp_al_disable_eink_wb(mgr->disp);
		ret = 0;
	} else
		DE_WARN("src_format(), dest_format(0x%x) is not support\n",
				dest->format);

EXIT:
#ifdef EINK_DMABUF_USED
	for (k = 0; k < layer_num; k++) {
		if (NULL != item[k])
			disp_dma_unmap(item[k]);
	}
#endif
	return ret;
}

s32 disp_init_format_convert_manager(struct disp_bsp_init_para *para)
{
	s32 ret = -1;
	unsigned int i = 0;
	struct format_manager *mgr;

	for (i = 0; i < FORMAT_MANAGER_NUM; i++) {
		mgr = &fmt_mgr[i];
		memset(mgr, 0, sizeof(struct format_manager));

		mgr->disp = i;
		init_waitqueue_head(&(mgr->write_back_queue));
		mgr->write_back_finish = 0;
		mgr->irq_num = para->irq_no[DISP_MOD_DE];
		mgr->enable = disp_format_convert_enable;
		mgr->disable = disp_format_convert_disable;
		mgr->start_convert = disp_format_convert_start;
		mgr->clk = para->clk_de[i];
		mgr->clk_bus = para->clk_bus_de[i];
		mgr->rst = para->reset_bus_de[i];
		disp_al_set_eink_wb_base(i, para->reg_base[DISP_MOD_DE]);
	}

	return ret;
}

void disp_exit_format_convert_manager(void)
{
	unsigned int i = 0;
	struct format_manager *mgr = NULL;

	for (i = 0; i < FORMAT_MANAGER_NUM; i++) {
		if (mgr->disable)
			mgr->disable(i);
	}
	return;
}

#endif
