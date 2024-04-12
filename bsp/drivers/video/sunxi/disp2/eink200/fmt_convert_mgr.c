/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (C) 2019 Allwinnertech, <liulii@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "include/fmt_convert.h"

#define FORMAT_MANAGER_NUM 1

#ifdef VIRTUAL_REGISTER
static void *wb_preg_base;
static void *wb_vreg_base;
#endif

unsigned long de_dbg_reg_base;

static struct fmt_convert_manager fmt_mgr[FORMAT_MANAGER_NUM];

struct fmt_convert_manager *get_fmt_convert_mgr(unsigned int id)
{
	return &fmt_mgr[id];
}

s32 fmt_convert_finish_proc(int irq, void *parg)
{
	int ret = 0;
	struct fmt_convert_manager *mgr = (struct fmt_convert_manager *)parg;

	EINK_DEBUG_MSG("FMT INTERRUPT!\n");

	if (mgr == NULL) {
		sunxi_err(NULL, "%s:fmt mgr is NULL!\n", __func__);
		return DISP_IRQ_RETURN;
	}

	ret = wb_eink_get_status(mgr->sel);
	if (ret == 0) {
		mgr->wb_finish = 1;
		EINK_DEBUG_MSG("WB SUCCESS!\n");
		wake_up_interruptible(&(mgr->write_back_queue));
	} else
		sunxi_err(NULL, "convert err! status = 0x%x\n", ret);
	wb_eink_interrupt_clear(mgr->sel);

	return DISP_IRQ_RETURN;
}

static int fmt_convert_enable(unsigned int id)
{
	struct fmt_convert_manager *mgr = &fmt_mgr[id];
	s32 ret = -1;

	if (mgr == NULL) {
		sunxi_err(NULL, "input param is null\n");
		return -1;
	}

	if (mgr->enable_flag == true)
		return 0;
	ret = request_irq(mgr->irq_num, (irq_handler_t)fmt_convert_finish_proc, 0, "de_writeback", (void *)mgr);
	if (ret != 0) {
		sunxi_err(NULL, "fail to format convert irq\n");
		return ret;
	}

	ret = reset_control_deassert(mgr->rst_clk);
	if (ret) {
		sunxi_err(NULL, "fail deassert de mgr's clock!\n");
		return ret;
	}

	if (mgr->bus_clk) {
		ret = clk_prepare_enable(mgr->bus_clk);
	}

	if (mgr->clk) {
		ret = clk_prepare_enable(mgr->clk);
	}

	if (ret) {
		sunxi_err(NULL, "fail enable mgr's clock!\n");
		return ret;
	}

	/* enable de clk, enable write back clk */
	de_clk_enable(DE_CLK_CORE0);
	de_clk_enable(DE_CLK_WB);

	mgr->enable_flag = true;
	return 0;
}

static s32 fmt_convert_disable(unsigned int id)
{
	struct fmt_convert_manager *mgr = &fmt_mgr[id];

	if (mgr == NULL) {
		sunxi_err(NULL, "%s: input param is null\n", __func__);
		return -1;
	}

	if (mgr->enable_flag == false)
		return 0;
	/* disable write back clk, disable de clk */
	de_clk_disable(DE_CLK_WB);
	de_clk_disable(DE_CLK_CORE0);

	clk_disable(mgr->clk);
	clk_disable(mgr->bus_clk);
	reset_control_assert(mgr->rst_clk);

	free_irq(mgr->irq_num, (void *)mgr);

	mgr->enable_flag = false;
	return 0;
}

struct disp_manager_data mdata;
struct disp_layer_config_data ldata[16];

enum upd_mode fmt_auto_mode_select(struct fmt_convert_manager *mgr,
				struct eink_img *last_img,
				struct eink_img *cur_img)
{
	enum upd_mode ret  = 0;

	ret = wb_eink_auto_mode_select(mgr->sel, mgr->gray_level_cnt, last_img, cur_img);
	return ret;
}

static s32 fmt_convert_start(unsigned int id, struct disp_layer_config_inner *config,
		unsigned int layer_num, struct eink_img *last_img, struct eink_img *dest_img)
{
	struct fmt_convert_manager *mgr = &fmt_mgr[id];
	__eink_wb_config_t wbcfg;
	struct dmabuf_item *item[16] = {NULL};
	struct dmabuf_item *last_item = NULL, *cur_item = NULL;
	struct fb_address_transfer fb;
	long timerout = 0;
	s32 ret = -1, k = 0;
	unsigned long wdbg_reg_base = 0, wdbg_end_reg = 0;


#if defined(__DISP_TEMP_CODE__)
#ifdef SAVE_DE_WB_BUF
	struct dma_buf *dmabuf = NULL;
	char *user_addr = NULL;
#endif
#endif


	EINK_INFO_MSG("DE WB START!\n");
	timerout = (100 * HZ) / 1000;	/* 100ms */

	if ((dest_img == NULL) || (mgr == NULL)) {/* last img need? */
		sunxi_err(NULL, "%s:input param is null\n", __func__);
		return -1;
	}

	if (dest_img->out_fmt != EINK_Y3 &&
			dest_img->out_fmt != EINK_Y4 &&
			dest_img->out_fmt != EINK_Y5 &&
			dest_img->out_fmt != EINK_Y8) {
		sunxi_err(NULL, "%s:format %d not support!", __func__, dest_img->out_fmt);
		return -1;
	}

	last_item = eink_dma_map(last_img->fd);
	if (item == NULL) {
		sunxi_err(NULL, "[%s]wb map last img fd %d fail!\n", __func__, last_img->fd);
		return -1;
	}

	cur_item = eink_dma_map(dest_img->fd);
	if (item == NULL) {
		sunxi_err(NULL, "[%s]wb map cur img fd %d fail!\n", __func__, dest_img->fd);
		if (last_item)
			eink_dma_unmap(last_item);
		return -1;
	}
	last_img->paddr = (void *)last_item->dma_addr;
	dest_img->paddr = (void *)cur_item->dma_addr;

	EINK_DEBUG_MSG("dest_addr = 0x%p\n", (void *)dest_img->paddr);
	EINK_DEBUG_MSG("calc_win_en = %d\n", dest_img->win_calc_en);

	/* time calc debug */
	if (eink_get_print_level() == 8) {
		ktime_get_real_ts64(&mgr->stimer);
	}
	memset((void *)&mdata, 0, sizeof(struct disp_manager_data));
	memset((void *)ldata, 0, 16 * sizeof(ldata[0]));

	mdata.flag = MANAGER_ALL_DIRTY;
	mdata.config.enable = 1;
	mdata.config.interlace = 0;
	mdata.config.blank = 0;
	mdata.config.size.x = 0;
	mdata.config.size.y = 0;
	mdata.config.size.width = dest_img->size.width;
	mdata.config.size.height = dest_img->size.height;

	for (k = 0; k < layer_num; k++) {
		ldata[k].flag = LAYER_ALL_DIRTY;

		memcpy((void *)&ldata[k].config, (void *)&config[k],
				sizeof(*config));

		item[k] = NULL;
		if (ldata[k].config.info.fb.fd && (true == config[k].enable)) {

#if defined(__DISP_TEMP_CODE__)
			dmabuf = dma_buf_get(ldata[k].config.info.fb.fd);
			if (IS_ERR(dmabuf)) {
				sunxi_err(NULL, "[%s]fail to get disp fd = %d\n", __func__,
						ldata[k].config.info.fb.fd);
			}

			ret = dma_buf_begin_cpu_access(dmabuf, DMA_FROM_DEVICE);
			if (ret) {
				dma_buf_put(dmabuf);
				dmabuf = NULL;
				sunxi_err(NULL, "[%s]:[%d] dma_buf_begin_cpu_access fail\n",
						__func__, __LINE__);
			}

			user_addr = dma_buf_kmap(dmabuf, 0);
			if (user_addr == NULL) {
				dma_buf_end_cpu_access(dmabuf, DMA_FROM_DEVICE);
				dma_buf_put(dmabuf);
				dmabuf = NULL;
				sunxi_err(NULL, "[%s]fail to map disp fd\n", __func__);
			}

			/* Later, you need to add order to the img to debug */
			eink_save_current_img(user_addr,
						ldata[k].config.info.fb.size[0].width,
						ldata[k].config.info.fb.size[0].height);

			dma_buf_kunmap(dmabuf, 0, user_addr);
			dma_buf_end_cpu_access(dmabuf, DMA_FROM_DEVICE);
			dmabuf = NULL;
			user_addr = NULL;
#endif
			item[k] = eink_dma_map(ldata[k].config.info.fb.fd);
			if (item[k] == NULL) {
				sunxi_err(NULL, "eink wb map fd %d fail!\n", ldata[k].config.info.fb.fd);
				return -1;
			}

			EINK_DEBUG_MSG("layer%d map ok, fd=%d, addr=0x%08x\n", k,
					ldata[k].config.info.fb.fd,
					(unsigned int)item[k]->dma_addr);

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
	}

	if (dest_img->dither_mode) {
		if (dest_img->out_fmt == EINK_Y8)
			dest_img->dither_mode = 0; /* Y8 bypass dither */

	}
	/* de process */
	disp_al_manager_apply(mgr->sel, &mdata);
	disp_al_layer_apply(mgr->sel, ldata, layer_num);
	disp_al_manager_sync(mgr->sel);
	disp_al_manager_update_regs(mgr->sel);

	wb_eink_reset(mgr->sel);
	wb_eink_dereset(mgr->sel);

	if (dest_img->upd_mode == EINK_A2_MODE) {
		wb_eink_set_a2_mode(mgr->sel);
		if (dest_img->dither_mode == ORDERED) {
			sunxi_err(NULL, "%s:hardware not support a2 mode ORFERED dither!\n", __func__);
			ret = -1;
			goto EXIT;
		}
	}

	wb_eink_set_panel_bit(mgr->sel, mgr->panel_bit);
	if (mgr->panel_bit == 5 && mgr->gray_level_cnt == 16)
		wb_eink_set_gray_level(mgr->sel, 1);
	else
		wb_eink_set_gray_level(mgr->sel, 0);

	/* cfg wb param */
	wbcfg.frame.crop.x = 0;
	wbcfg.frame.crop.y = 0;
	wbcfg.frame.crop.width = dest_img->size.width;
	wbcfg.frame.crop.height = dest_img->size.height; /* Write it to death for the time being,
							    wait for the ic to come back and then add it to match */

	wbcfg.frame.size.width  = dest_img->size.width;
	wbcfg.frame.size.height = dest_img->size.height;
	wbcfg.frame.addr        = (unsigned long)dest_img->paddr;
	wbcfg.win_en		= dest_img->win_calc_en;

	if ((wbcfg.win_en == true) && (last_img != NULL)) {/* first time last img is NULL */
		if (last_img->out_fmt != dest_img->out_fmt) {
			sunxi_warn(NULL, "%s:calc win must be same fmt!use default screen\n", __func__);
			wbcfg.win_en = false;

			dest_img->upd_win.top = 0;
			dest_img->upd_win.left = 0;
			dest_img->upd_win.right = dest_img->size.width - 1;
			dest_img->upd_win.bottom = dest_img->size.height - 1;
		} else {

#if defined(__DISP_TEMP_CODE__)
#ifdef SAVE_DE_WB_BUF
			eink_save_last_img((char *)last_img->vaddr,
						last_img->size.width,
						last_img->size.height);
#endif
#endif
			wb_eink_set_last_img(mgr->sel, (unsigned long)last_img->paddr);/* ----------------------- */
		}
	}

	wbcfg.out_fmt		= dest_img->out_fmt;
	wbcfg.csc_std		= 2;
	wbcfg.dither_mode	= dest_img->dither_mode;

	wb_eink_set_para(mgr->sel, &wbcfg);


	/* enable inttrrupt */
	wb_eink_interrupt_enable(mgr->sel);
	wb_eink_enable(mgr->sel);

	/* for debug reg info */
	if (eink_get_print_level() == 4) {
		sunxi_info(NULL, "[EINK PRINT WB REG]:-----\n");
		wdbg_reg_base = wb_eink_get_reg_base(mgr->sel);
		EINK_INFO_MSG("reg_base = 0x%x\n", (unsigned int)wdbg_reg_base);
		wdbg_end_reg = wdbg_reg_base + 0x3ff;
		eink_print_register(wdbg_reg_base, wdbg_end_reg);

		sunxi_info(NULL, "de reg_base = 0x%x\n", (unsigned int)de_dbg_reg_base);

		sunxi_info(NULL, "[DE RTMX REG]:\n");
		wdbg_reg_base = de_dbg_reg_base + 0x100000;
		wdbg_end_reg = wdbg_reg_base + 0x20;
		eink_print_register(wdbg_reg_base, wdbg_end_reg);

		EINK_INFO_MSG("[DE RTMX APB REG]:\n");
		wdbg_reg_base = de_dbg_reg_base + 0x101000;
		wdbg_end_reg = wdbg_reg_base + 0x3ff;
		eink_print_register(wdbg_reg_base, wdbg_end_reg);

		EINK_INFO_MSG("[DE RTMX OVL0 REG]:\n");
		wdbg_reg_base = de_dbg_reg_base + 0x102000;
		wdbg_end_reg = wdbg_reg_base + 0x3ff;
		eink_print_register(wdbg_reg_base, wdbg_end_reg);
	}

	timerout =
		wait_event_interruptible_timeout(mgr->write_back_queue,
				(mgr->wb_finish == 1),
				timerout);
	mgr->wb_finish = 0;
	if (timerout == 0) {
		sunxi_err(NULL, "wait write back timeout!\n");
		wb_eink_interrupt_disable(mgr->sel);
		wb_eink_disable(mgr->sel);
		ret = -1;
		goto EXIT;
	}

	if (wbcfg.win_en) {
		wb_eink_get_upd_win(0, &dest_img->upd_win);

		if (((dest_img->upd_win.left + 1) == dest_img->size.width) &&
				((dest_img->upd_win.top + 1) == dest_img->size.height) &&
				(dest_img->upd_win.right == 0) && (dest_img->upd_win.bottom == 0)) {
			sunxi_warn(NULL, "%s:calc upd win not right\n", __func__);
			dest_img->upd_win.left = 0;
			dest_img->upd_win.top = 0;
			dest_img->upd_win.right = 0;
			dest_img->upd_win.bottom = 0;
		}
		dest_img->win_calc_en = false;
	}

	wb_eink_get_hist_val(mgr->sel, mgr->gray_level_cnt, dest_img->eink_hist);

	wb_eink_interrupt_disable(mgr->sel);
	wb_eink_disable(mgr->sel);

	ret = 0;

EXIT:
	for (k = 0; k < layer_num; k++) {
		if (item[k] != NULL)
			eink_dma_unmap(item[k]);
	}

	if (eink_get_print_level() == 8) {
		ktime_get_real_ts64(&mgr->etimer);
		sunxi_info(NULL, "take %d ms\n", get_delt_ms_timer(mgr->stimer, mgr->etimer));
	}

	return ret;
}

s32 fmt_convert_mgr_init(struct init_para *para)
{
	s32 ret = 0;
	unsigned int i = 0;
	struct fmt_convert_manager *mgr;

	for (i = 0; i < FORMAT_MANAGER_NUM; i++) {
		mgr = &fmt_mgr[i];
		memset(mgr, 0, sizeof(struct fmt_convert_manager));

		mgr->sel = i;
		init_waitqueue_head(&(mgr->write_back_queue));
		mgr->wb_finish = 0;
		mgr->panel_bit = para->panel_info.bit_num;
		mgr->gray_level_cnt = para->panel_info.gray_level_cnt;

		mgr->irq_num = para->de_irq_no;

		mgr->enable = fmt_convert_enable;
		mgr->disable = fmt_convert_disable;
		mgr->fmt_auto_mode_select = fmt_auto_mode_select;
		mgr->start_convert = fmt_convert_start;
		mgr->clk = para->de_clk;
		mgr->bus_clk = para->de_bus_clk;
		mgr->rst_clk = para->de_rst_clk;

#ifdef VIRTUAL_REGISTER
		wb_vreg_base = eink_malloc(0x03ffff, &wb_preg_base);
		wb_eink_set_reg_base(mgr->sel, wb_vreg_base);
#else
		wb_eink_set_reg_base(mgr->sel, para->de_reg_base);
#endif

		de_dbg_reg_base = (unsigned long)para->de_reg_base;

		wb_eink_set_panel_bit(mgr->sel, mgr->panel_bit);
	}

	return ret;
}

void fmt_convert_mgr_exit(void)
{
	unsigned int i = 0;
	struct fmt_convert_manager *mgr = NULL;

	for (i = 0; i < FORMAT_MANAGER_NUM; i++) {
		mgr = get_fmt_convert_mgr(i);
		if (mgr && mgr->disable)
			mgr->disable(i);
	}
	return;
}
