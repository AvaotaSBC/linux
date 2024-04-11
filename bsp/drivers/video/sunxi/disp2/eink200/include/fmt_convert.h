/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __FMT_CONVERT_H__
#define __FMT_CONVERT_H__

/* #include "../../disp/de/disp_private.h" */
/* #include "../../disp/de/include.h" */
#include "eink_sys_source.h"
#include "../lowlevel/de_wb_reg.h"


enum clk_id {
	DE_CLK_NONE = 0,
	DE_CLK_CORE0 = 1,
	DE_CLK_CORE1 = 2,
	DE_CLK_WB = 3,
};

/*
struct fmt_cvt_node {
	enum upd_pixel_fmt	format;
	enum dither_mode	dither_mode;
	unsigned int		width;
	unsigned int		height;
	unsigned long		addr;
	bool			win_calc_en;
	struct upd_win		upd_win;
	unsigned int		*eink_hist;
};
*/

struct fmt_convert_manager {
	bool			enable_flag;
	unsigned int		sel;
	unsigned int		irq_num;
	unsigned int		wb_finish;
	unsigned int		panel_bit;
	unsigned int		gray_level_cnt;
	wait_queue_head_t	write_back_queue;
	struct clk		*clk;
	struct clk		*bus_clk;
	struct reset_control	*rst_clk;
	struct timespec64	stimer;
	struct timespec64	etimer;
	int (*enable)(unsigned int id);
	int (*disable)(unsigned int id);
	enum upd_mode (*fmt_auto_mode_select)(struct fmt_convert_manager *mgr,
					struct eink_img *last_img,
					struct eink_img *cur_img);
	int (*start_convert)(unsigned int sel,
			      struct disp_layer_config_inner *config,
			      unsigned int layer_num,
			      struct eink_img *last_img,
			      struct eink_img *dest_img);
};

extern s32 de_clk_enable(u32 clk_no);
extern s32 de_clk_disable(u32 clk_no);
extern int disp_al_manager_apply(unsigned int disp, struct disp_manager_data *data);
extern int disp_al_layer_apply(unsigned int disp, struct disp_layer_config_data *data,
			unsigned int layer_num);
extern int disp_al_manager_sync(unsigned int disp);
extern int disp_al_manager_update_regs(unsigned int disp);

extern s32 disp_set_fb_info(struct fb_address_transfer *fb, bool left_eye);

/*
extern struct dmabuf_item *disp_dma_map(int fd);
extern void disp_dma_unmap(struct dmabuf_item *item);
*/

#endif
