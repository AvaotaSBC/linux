/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * isp520_reg_cfg.c
 *
 * Copyright (c) 2007-2017 Allwinnertech Co., Ltd.
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/io.h>
#include <linux/string.h>
#include "isp520_reg.h"
#include "isp520_reg_cfg.h"

#define ISP520_MAX_NUM 1
/* #define USE_DEF_PARA */

/*
 *  Load ISP register variables
 */

struct isp520_reg {
	ISP_GLOBAL_CFG0_REG_t *isp_global_cfg0;
	ISP_GLOBAL_CFG1_REG_t *isp_global_cfg1;
	ISP_GLOBAL_CFG2_REG_t *isp_global_cfg2;

	ISP_UPDATE_CTRL0_REG_t *isp_update_ctrl0;
	unsigned int *isp_load_addr0;
	unsigned int *isp_load_addr1;
	unsigned int *isp_save_addr;
	ISP_INT_BYPASS0_REG_t *isp_int_bypass0;
	ISP_INT_STATUS0_REG_t *isp_int_status0;
	ISP_INTER_STATUS0_REG_t	*isp_inter_status0;
	ISP_INTER_STATUS1_REG_t	*isp_inter_status1;
	ISP_VER_CFG_REG_t *isp_ver_cfg;
	ISP_MAX_WIDTH_REG_t *isp_max_width;
	ISP_COMP_FIFO_MAX_LAYER_REG_t *isp_comp_fifo_max_layer;
	ISP_UNCOMP_FIFO_MAX_LAYER_REG_t	*isp_uncomp_fifo_max_layer;

	ISP_WDR_CMP_BANDWIDTH_REG_t *isp_wdr_cmp_bandwidth;
	ISP_WDR_DECMP_BANDWIDTH_REG_t *isp_wdr_decmp_bandwidth;
	ISP_D3D_CMP_BANDWIDTH_REG_t *isp_d3d_cmp_bandwidth;
	ISP_D3D_DECMP_BANDWIDTH_REG_t *isp_d3d_decmp_bandwidth;

	ISP_S0_FMERR_CNT_REG_t *isp_s0_fmerr_cnt;
	ISP_S1_FMERR_CNT_REG_t *isp_s1_fmerr_cnt;
	ISP_S0_HB_CNT_REG_t *isp_s0_hb_cnt;
	ISP_S1_HB_CNT_REG_t *isp_s1_hb_cnt;
	ISP_WDR_FIFO_OVERFLOW_LINE_REG_t *isp_wdr_fifo_overflow_line;
	ISP_D3D_FIFO_OVERFLOW_LINE_REG_t *isp_d3d_fifo_overflow_line;
	unsigned int *isp_wdr_exp_addr0;
	unsigned int *isp_wdr_exp_addr1;
	ISP_SIM_CTRL_REG_t *isp_sim_ctrl;
	unsigned int *isp_d3d_rec_addr0;
	unsigned int *isp_d3d_rec_addr1;
	ISP_TOP_CTRL_REG_t *isp_top_ctrl;

	ISP_UPDATE_CTRL0_REG_t *isp_update_flag;
	ISP_S1_CFG_REG_t *isp_s1_cfg;
	ISP_MODULE_BYPASS0_REG_t *isp_module_bypass0;
	ISP_INPUT_SIZE_REG_t *isp_input_size;
	ISP_VALID_SIZE_REG_t *isp_valid_size;
	ISP_VALID_START_REG_t *isp_valid_start;

	ISP_LSC_CFG0_REG_t *isp_lsc_cfg0;

	ISP_PLTM_CFG0_REG_t *isp_pltm_cfg0;
	ISP_PLTM_CFG2_REG_t *isp_pltm_cfg2;
	ISP_PLTM_CFG3_REG_t *isp_pltm_cfg3;
};

struct isp520_reg isp_regs[ISP520_MAX_NUM];

void bsp_isp_map_reg_addr(unsigned long id, unsigned long base)
{
	isp_regs[id].isp_global_cfg0 = (ISP_GLOBAL_CFG0_REG_t *) (base + ISP_GLOBAL_CFG0_REG);
	isp_regs[id].isp_global_cfg1 = (ISP_GLOBAL_CFG1_REG_t *) (base + ISP_GLOBAL_CFG1_REG);
	isp_regs[id].isp_global_cfg2 = (ISP_GLOBAL_CFG2_REG_t *) (base + ISP_GLOBAL_CFG2_REG);

	isp_regs[id].isp_update_ctrl0 = (ISP_UPDATE_CTRL0_REG_t *) (base + ISP_UPDATE_CTRL0_REG);
	isp_regs[id].isp_load_addr0 = (unsigned int *) (base + ISP_LOAD_ADDR0_REG);
	isp_regs[id].isp_load_addr1 = (unsigned int *) (base + ISP_LOAD_ADDR1_REG);
	isp_regs[id].isp_save_addr = (unsigned int *) (base + ISP_SAVE_ADDR_REG);
	isp_regs[id].isp_int_bypass0 = (ISP_INT_BYPASS0_REG_t *) (base + ISP_INT_BYPASS0_REG);
	isp_regs[id].isp_int_status0 = (ISP_INT_STATUS0_REG_t *) (base + ISP_INT_STATUS0_REG);
	isp_regs[id].isp_inter_status0 = (ISP_INTER_STATUS0_REG_t *) (base + ISP_INTER_STATUS0_REG);
	isp_regs[id].isp_inter_status1 = (ISP_INTER_STATUS1_REG_t *) (base + ISP_INTER_STATUS1_REG);
	isp_regs[id].isp_ver_cfg = (ISP_VER_CFG_REG_t *) (base + ISP_VER_CFG_REG);
	isp_regs[id].isp_max_width = (ISP_MAX_WIDTH_REG_t *) (base + ISP_MAX_WIDTH_REG);
	isp_regs[id].isp_comp_fifo_max_layer = (ISP_COMP_FIFO_MAX_LAYER_REG_t *) (base + ISP_COMP_FIFO_MAX_LAYER_REG);
	isp_regs[id].isp_uncomp_fifo_max_layer = (ISP_UNCOMP_FIFO_MAX_LAYER_REG_t *) (base + ISP_UNCOMP_FIFO_MAX_LAYER_REG);

	isp_regs[id].isp_wdr_cmp_bandwidth = (ISP_WDR_CMP_BANDWIDTH_REG_t *) (base + ISP_WDR_CMP_BANDWIDTH_REG);
	isp_regs[id].isp_wdr_decmp_bandwidth = (ISP_WDR_DECMP_BANDWIDTH_REG_t *) (base + ISP_WDR_DECMP_BANDWIDTH_REG);
	isp_regs[id].isp_d3d_cmp_bandwidth = (ISP_D3D_CMP_BANDWIDTH_REG_t *) (base + ISP_D3D_CMP_BANDWIDTH_REG);
	isp_regs[id].isp_d3d_decmp_bandwidth = (ISP_D3D_DECMP_BANDWIDTH_REG_t *) (base + ISP_D3D_DECMP_BANDWIDTH_REG);

	isp_regs[id].isp_s0_fmerr_cnt = (ISP_S0_FMERR_CNT_REG_t *) (base + ISP_S0_FMERR_CNT_REG);
	isp_regs[id].isp_s1_fmerr_cnt = (ISP_S1_FMERR_CNT_REG_t *) (base + ISP_S1_FMERR_CNT_REG);
	isp_regs[id].isp_s0_hb_cnt = (ISP_S0_HB_CNT_REG_t *) (base + ISP_S0_HB_CNT_REG);
	isp_regs[id].isp_s1_hb_cnt = (ISP_S1_HB_CNT_REG_t *) (base + ISP_S1_HB_CNT_REG);
	isp_regs[id].isp_wdr_fifo_overflow_line = (ISP_WDR_FIFO_OVERFLOW_LINE_REG_t *) (base + ISP_WDR_FIFO_OVERFLOW_LINE_REG);
	isp_regs[id].isp_d3d_fifo_overflow_line = (ISP_D3D_FIFO_OVERFLOW_LINE_REG_t *) (base + ISP_D3D_FIFO_OVERFLOW_LINE_REG);
	isp_regs[id].isp_wdr_exp_addr0 = (unsigned int *) (base + ISP_WDR_EXP_ADDR0_REG);
	isp_regs[id].isp_wdr_exp_addr1 = (unsigned int *) (base + ISP_WDR_EXP_ADDR1_REG);
	isp_regs[id].isp_sim_ctrl = (ISP_SIM_CTRL_REG_t *) (base + ISP_SIM_CTRL_REG);
	isp_regs[id].isp_d3d_rec_addr0 = (unsigned int *) (base + ISP_D3D_REC_ADDR0_REG);
	isp_regs[id].isp_d3d_rec_addr1 = (unsigned int *) (base + ISP_D3D_REC_ADDR1_REG);
	isp_regs[id].isp_top_ctrl = (ISP_TOP_CTRL_REG_t *) (base + ISP_TOP_CTRL_REG);

#ifdef USE_DEF_PARA
	isp_regs[id].isp_s1_cfg = (ISP_S1_CFG_REG_t *) (base + ISP_S1_CFG_REG);
	isp_regs[id].isp_module_bypass0 = (ISP_MODULE_BYPASS0_REG_t *) (base + ISP_MODULE_BYPASS0_REG);
	isp_regs[id].isp_input_size = (ISP_INPUT_SIZE_REG_t *) (base + ISP_INPUT_SIZE_REG);
	isp_regs[id].isp_valid_size = (ISP_VALID_SIZE_REG_t *) (base + ISP_VALID_SIZE_REG);
	isp_regs[id].isp_valid_start = (ISP_VALID_START_REG_t *) (base + ISP_VALID_START_REG);
#endif
}

/*
 * Load DRAM Register Address
 */

void bsp_isp_map_load_dram_addr(unsigned long id, unsigned long base)
{
#ifndef USE_DEF_PARA
	isp_regs[id].isp_update_flag = (ISP_UPDATE_CTRL0_REG_t *) (base + ISP_UPDATE_CTRL0_REG);
	isp_regs[id].isp_s1_cfg = (ISP_S1_CFG_REG_t *) (base + ISP_S1_CFG_REG);
	isp_regs[id].isp_module_bypass0 = (ISP_MODULE_BYPASS0_REG_t *) (base + ISP_MODULE_BYPASS0_REG);
	isp_regs[id].isp_input_size = (ISP_INPUT_SIZE_REG_t *) (base + ISP_INPUT_SIZE_REG);
	isp_regs[id].isp_valid_size = (ISP_VALID_SIZE_REG_t *) (base + ISP_VALID_SIZE_REG);
	isp_regs[id].isp_valid_start = (ISP_VALID_START_REG_t *) (base + ISP_VALID_START_REG);
	isp_regs[id].isp_lsc_cfg0 = (ISP_LSC_CFG0_REG_t *) (base + ISP_LSC_CFG0_REG);
	isp_regs[id].isp_pltm_cfg0 = (ISP_PLTM_CFG0_REG_t *) (base + ISP_PLTM_CFG0_REG);
	isp_regs[id].isp_pltm_cfg2 = (ISP_PLTM_CFG2_REG_t *) (base + ISP_PLTM_CFG2_REG);
	isp_regs[id].isp_pltm_cfg3 = (ISP_PLTM_CFG3_REG_t *) (base + ISP_PLTM_CFG3_REG);
#endif
}

/*******isp control register which we can write directly to register*********/

void bsp_isp_enable(unsigned long id, unsigned int en)
{
	isp_regs[id].isp_global_cfg0->bits.isp_enable = en;
}

void bsp_isp_capture_start(unsigned long id)
{
	isp_regs[id].isp_global_cfg0->bits.cap_en = 1;
}

void bsp_isp_capture_stop(unsigned long id)
{
	isp_regs[id].isp_global_cfg0->bits.cap_en = 0;
}

void bsp_isp_ver_read_en(unsigned long id, unsigned int en)
{
	isp_regs[id].isp_global_cfg0->bits.isp_ver_rd_en = en;
}

void bsp_isp_set_input_fmt(unsigned long id, unsigned int fmt)
{
	isp_regs[id].isp_global_cfg0->bits.input_fmt = fmt;
}

void bsp_isp_ch_enable(unsigned long id, int ch, int enable)
{
	switch (ch) {
	case 0:
		isp_regs[id].isp_global_cfg0->bits.isp_ch0_en = enable;
		break;
	case 1:
		isp_regs[id].isp_global_cfg0->bits.isp_ch1_en = enable;
		break;
	case 2:
		isp_regs[id].isp_global_cfg0->bits.isp_ch2_en = enable;
		break;
	case 3:
		isp_regs[id].isp_global_cfg0->bits.isp_ch3_en = enable;
		break;
	default:
		isp_regs[id].isp_global_cfg0->bits.isp_ch0_en = enable;
		break;
	}
}

void bsp_isp_wdr_mode_cfg(unsigned long id, struct isp_wdr_mode_cfg *cfg)
{
	isp_regs[id].isp_global_cfg0->bits.wdr_ch_seq = cfg->wdr_ch_seq;
	isp_regs[id].isp_global_cfg0->bits.wdr_exp_seq = cfg->wdr_exp_seq;
	isp_regs[id].isp_global_cfg0->bits.wdr_mode = cfg->wdr_mode;
}

void bsp_isp_set_line_int_num(unsigned long id, unsigned int line_num)
{
	isp_regs[id].isp_global_cfg1->bits.line_int_num = line_num;
}

void bsp_isp_set_speed_mode(unsigned long id, unsigned int speed)
{
	isp_regs[id].isp_global_cfg1->bits.speed_mode = speed;
}

void bsp_isp_set_last_blank_cycle(unsigned long id, unsigned int blank)
{
	isp_regs[id].isp_global_cfg1->bits.last_blank_cycle = blank;
}

void bsp_isp_debug_output_cfg(unsigned long id, int enable, int output_sel)
{
	isp_regs[id].isp_global_cfg2->bits.debug_en = enable;
	isp_regs[id].isp_global_cfg2->bits.debug_sel = output_sel;
}

void bsp_isp_set_para_ready_mode(unsigned long id, int enable)
{
}

void bsp_isp_set_para_ready(unsigned long id, int ready)
{
#ifndef USE_DEF_PARA
	if (ready)
		isp_regs[id].isp_update_ctrl0->dwval |= S1_PARA_READY;
	else
		isp_regs[id].isp_update_ctrl0->dwval &= ~S1_PARA_READY;
	isp_regs[id].isp_update_ctrl0->bits.para_ready = ready;
#endif
}

void bsp_isp_update_table(unsigned long id, unsigned short table_update)
{
	isp_regs[id].isp_update_ctrl0->bits.linear_update = !!(table_update & LINEAR_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.lens_update = !!(table_update & LENS_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.gamma_update = !!(table_update & GAMMA_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.drc_update = !!(table_update & DRC_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.satu_update = !!(table_update & SATU_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.wdr_update = !!(table_update & WDR_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.d3d_update = !!(table_update & D3D_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.pltm_update = !!(table_update & PLTM_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.cem_update = !!(table_update & CEM_UPDATE);
	isp_regs[id].isp_update_ctrl0->bits.dehaze_update = !!(table_update & DEHAZE_UPDATE);

	isp_regs[id].isp_update_ctrl0->bits.s1_linear_update = !!(table_update & S1_LINEAR_UPDATE);
}

void bsp_isp_set_load_addr0(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_load_addr0);
}

void bsp_isp_set_load_addr1(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_load_addr1);
}

void bsp_isp_set_saved_addr(unsigned long id, unsigned long addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_save_addr);
}

void bsp_isp_set_statistics_addr(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_save_addr);
}

void bsp_isp_irq_enable(unsigned long id, unsigned int irq_flag)
{
	isp_regs[id].isp_int_bypass0->dwval |= irq_flag;
}

void bsp_isp_irq_disable(unsigned long id, unsigned int irq_flag)
{
	isp_regs[id].isp_int_bypass0->dwval &= ~irq_flag;
}

unsigned int bsp_isp_get_irq_status(unsigned long id, unsigned int flag)
{
	return isp_regs[id].isp_int_status0->dwval & flag;
}

void bsp_isp_clr_irq_status(unsigned long id, unsigned int flag)
{
	isp_regs[id].isp_int_status0->dwval = flag;
}

unsigned int bsp_isp_get_internal_status0(unsigned long id, unsigned int flag)
{
	return isp_regs[id].isp_inter_status0->dwval & flag;
}

void bsp_isp_clr_internal_status0(unsigned long id, unsigned int flag)
{
	isp_regs[id].isp_inter_status0->dwval = flag;
}

unsigned int bsp_isp_get_internal_status1(unsigned long id)
{
	return isp_regs[id].isp_inter_status1->dwval;
}

unsigned int bsp_isp_get_isp_ver(unsigned long id, unsigned int *major, unsigned int *minor)
{
	*major = isp_regs[id].isp_ver_cfg->bits.big_ver;
	*minor = isp_regs[id].isp_ver_cfg->bits.small_ver;
	return isp_regs[id].isp_ver_cfg->dwval;
}

unsigned int bsp_isp_get_max_width(unsigned long id)
{
	return isp_regs[id].isp_max_width->bits.max_width;
}

void bsp_isp_get_comp_fifo_max_layer(unsigned long id, unsigned int *wdr_fifo, unsigned int *d3d_fifo)
{
	*wdr_fifo = isp_regs[id].isp_comp_fifo_max_layer->bits.wdr_fifo_max_layer;
	*d3d_fifo = isp_regs[id].isp_comp_fifo_max_layer->bits.d3d_fifo_max_layer;
}

void bsp_isp_get_uncomp_fifo_max_layer(unsigned long id, unsigned int *wdr_fifo, unsigned int *d3d_fifo)
{
	*wdr_fifo = isp_regs[id].isp_uncomp_fifo_max_layer->bits.wdr_fifo_max_layer;
	*d3d_fifo = isp_regs[id].isp_uncomp_fifo_max_layer->bits.d3d_fifo_max_layer;
}

void bsp_isp_get_s0_ch_fmerr_cnt(unsigned long id, struct isp_size *size)
{
	size->width = isp_regs[id].isp_s0_fmerr_cnt->bits.input_width;
	size->height = isp_regs[id].isp_s0_fmerr_cnt->bits.input_height;
}

void bsp_isp_get_s1_ch_fmerr_cnt(unsigned long id, struct isp_size *size)
{
	size->width = isp_regs[id].isp_s1_fmerr_cnt->bits.input_width;
	size->height = isp_regs[id].isp_s1_fmerr_cnt->bits.input_height;
}

void bsp_isp_get_s0_ch_hb_cnt(unsigned long id, unsigned int *hb_max, unsigned int *hb_min)
{
	*hb_max = isp_regs[id].isp_s0_hb_cnt->bits.hb_max;
	*hb_min = isp_regs[id].isp_s0_hb_cnt->bits.hb_min;
}

void bsp_isp_get_s1_ch_hb_cnt(unsigned long id, unsigned int *hb_max, unsigned int *hb_min)
{
	*hb_max = isp_regs[id].isp_s1_hb_cnt->bits.hb_max;
	*hb_min = isp_regs[id].isp_s1_hb_cnt->bits.hb_min;
}

void bsp_isp_get_wdr_fifo_overflow_line(unsigned long id, unsigned int *decomp_line, unsigned int *comp_line)
{
	*decomp_line = isp_regs[id].isp_wdr_fifo_overflow_line->bits.decomp_overflow_line;
	*comp_line = isp_regs[id].isp_wdr_fifo_overflow_line->bits.comp_overflow_line;
}

void bsp_isp_get_d3d_fifo_overflow_line(unsigned long id, unsigned int *decomp_line, unsigned int *comp_line)
{
	*decomp_line = isp_regs[id].isp_d3d_fifo_overflow_line->bits.decomp_overflow_line;
	*comp_line = isp_regs[id].isp_d3d_fifo_overflow_line->bits.comp_overflow_line;
}

void bsp_isp_set_wdr_addr0(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_wdr_exp_addr0);
}

void bsp_isp_set_wdr_addr1(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_wdr_exp_addr1);
}

void bsp_isp_set_d3d_addr0(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_d3d_rec_addr0);
}

void bsp_isp_set_d3d_addr1(unsigned long id, dma_addr_t addr)
{
	writel(addr >> ISP_ADDR_BIT_R_SHIFT, isp_regs[id].isp_d3d_rec_addr1);
}

void bsp_isp_top_control(unsigned long id, int isp_num, int isp0_max_w)
{
	isp_regs[id].isp_top_ctrl->bits.isp0_max_width = isp0_max_w;
	isp_regs[id].isp_top_ctrl->bits.isp_mode = isp_num - 1;
}

void bsp_isp_set_fifo_mode(unsigned long id, unsigned int mode)
{
}

void bsp_isp_min_ddr_size(unsigned long id, unsigned int size)
{
}

void bsp_isp_fifo_raw_write(unsigned long id, unsigned int depth)
{
}

void bsp_isp_k_min_ddr_size(unsigned long id, unsigned int size)
{
}

/*******isp load register which we should write to ddr first*********/

void bsp_isp_s1_module_enable(unsigned long id, unsigned int module_flag)
{
	isp_regs[id].isp_s1_cfg->dwval |= module_flag;
}

void bsp_isp_s1_module_disable(unsigned long id, unsigned int module_flag)
{
	isp_regs[id].isp_s1_cfg->dwval &= ~module_flag;
}

void bsp_isp_module_enable(unsigned long id, unsigned int module_flag)
{
	isp_regs[id].isp_module_bypass0->dwval |= module_flag;
}

void bsp_isp_module_disable(unsigned long id, unsigned int module_flag)
{
	isp_regs[id].isp_module_bypass0->dwval &= ~module_flag;
}

void bsp_isp_set_size(unsigned long id, struct isp_size_settings *size)
{
	int i, s, rs = 12;

	/* input and output size */
	isp_regs[id].isp_input_size->bits.input_width = size->ob_black.width;
	isp_regs[id].isp_input_size->bits.input_height = size->ob_black.height;
	isp_regs[id].isp_valid_size->bits.valid_width = size->ob_valid.width;
	isp_regs[id].isp_valid_size->bits.valid_height = size->ob_valid.height;
	isp_regs[id].isp_valid_start->bits.valid_hor_start = size->ob_start.hor;
	isp_regs[id].isp_valid_start->bits.valid_ver_start = size->ob_start.ver;

	/* lsc size */
	s = (size->ob_valid.width*size->ob_valid.width + size->ob_valid.height*size->ob_valid.height)/4;
	for (i = 0; i < 20; i++) {
		if ((s / (1 << i)) < 256) {
			rs = i;
			break;
		}
	}
	isp_regs[id].isp_lsc_cfg0->bits.lsc_ct_x = size->ob_valid.width/2;
	isp_regs[id].isp_lsc_cfg0->bits.lsc_ct_y = size->ob_valid.height/2;
	isp_regs[id].isp_lsc_cfg0->bits.lsc_rs_value = rs;

	/* pltm size */
	if (size->set_cnt < 3) {
		isp_regs[id].isp_pltm_cfg0->bits.cal_en = 0;
		isp_regs[id].isp_pltm_cfg0->bits.frm_sm_en = 0;
	} else if (size->set_cnt == 3) {
		isp_regs[id].isp_pltm_cfg0->bits.cal_en = 1;
		isp_regs[id].isp_pltm_cfg0->bits.frm_sm_en = 0;
	} else if (size->set_cnt == 4) {
		isp_regs[id].isp_pltm_cfg0->bits.cal_en = 1;
		isp_regs[id].isp_pltm_cfg0->bits.frm_sm_en = 1;
	}

	isp_regs[id].isp_pltm_cfg2->bits.block_height = (size->ob_valid.height/24) - 1;
	isp_regs[id].isp_pltm_cfg2->bits.block_width = (size->ob_valid.width/32) - 1;
	isp_regs[id].isp_pltm_cfg2->bits.block_v_num = 23;
	isp_regs[id].isp_pltm_cfg2->bits.block_h_num = 31;

	isp_regs[id].isp_pltm_cfg3->bits.statistic_div = (1 << 31) / ((size->ob_valid.width/32) * (size->ob_valid.height/24) / 2);
}

unsigned int bsp_isp_load_update_flag(unsigned long id)
{
	return isp_regs[id].isp_update_flag->dwval;
}
