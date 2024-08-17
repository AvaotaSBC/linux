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
#include <linux/math64.h>
#include <linux/delay.h>

#include "dsi_v1.h"

u32 dsi_pixel_bits[4] = { 24, 24, 18, 16 };
u32 dsi_lane_den[4] = { 0x1, 0x3, 0x7, 0xf };

static s32 dsi_delay_ms(u32 ms)
{
	msleep(ms);
	return 0;
}

static s32 dsi_delay_us(u32 us)
{
	udelay(us);
	return 0;
}

s32 dsi_set_reg_base(struct sunxi_dsi_lcd *dsi, uintptr_t base)
{
	dsi->reg = (struct dsi_lcd_reg *) (uintptr_t) (base);

	return 0;
}


u32 dsi_get_reg_base(struct sunxi_dsi_lcd *dsi)
{
	return (u32) (uintptr_t) dsi->reg;
}

u32 dsi_get_start_delay(struct sunxi_dsi_lcd *dsi)
{
	return dsi->reg->dsi_basic_ctl1.bits.video_start_delay;
}

static s32 dsi_start(struct sunxi_dsi_lcd *dsi, enum __dsi_start_t func)
{
	switch (func) {
	case DSI_START_TBA:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_TBA << (4 * DSI_INST_ID_LP11)
		    | DSI_INST_ID_END << (4 * DSI_INST_ID_TBA);
		break;
	case DSI_START_HSTX:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_HSC << (4 * DSI_INST_ID_LP11)
		    | DSI_INST_ID_NOP << (4 * DSI_INST_ID_HSC)
		    | DSI_INST_ID_HSD << (4 * DSI_INST_ID_NOP)
		    | DSI_INST_ID_DLY << (4 * DSI_INST_ID_HSD)
		    | DSI_INST_ID_NOP << (4 * DSI_INST_ID_DLY)
		    | DSI_INST_ID_END << (4 * DSI_INST_ID_HSCEXIT);
		break;
	case DSI_START_LPTX:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11)
		    | DSI_INST_ID_END << (4 * DSI_INST_ID_LPDT);
		break;
	case DSI_START_LPRX:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11)
		    | DSI_INST_ID_DLY << (4 * DSI_INST_ID_LPDT)
		    | DSI_INST_ID_TBA << (4 * DSI_INST_ID_DLY)
		    | DSI_INST_ID_END << (4 * DSI_INST_ID_TBA);
		break;
	case DSI_START_HSC:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_HSC << (4 * DSI_INST_ID_LP11)
		    | DSI_INST_ID_END << (4 * DSI_INST_ID_HSC);
		break;
	case DSI_START_HSD:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_NOP << (4 * DSI_INST_ID_LP11)
		    | DSI_INST_ID_HSD << (4 * DSI_INST_ID_NOP)
		    | DSI_INST_ID_DLY << (4 * DSI_INST_ID_HSD)
		    | DSI_INST_ID_NOP << (4 * DSI_INST_ID_DLY)
		    | DSI_INST_ID_END << (4 * DSI_INST_ID_HSCEXIT);
		break;
	case DSI_START_HSD_DS:  /* lp11-(nop-deskew initial-hsd-dly-nop)----hscexit-end */
		dsi->reg->dsi_inst_jump_sel.dwval =
			DSI_INST_ID_NOP << (4 * DSI_INST_ID_LP11)
			| DSI_INST_ID_DS_1 << (4 * DSI_INST_ID_NOP)
			| DSI_INST_ID_DLY << (4 * DSI_INST_ID_HSD)
			| DSI_INST_ID_NOP << (4 * DSI_INST_ID_DLY)
			| DSI_INST_ID_END << (4 * DSI_INST_ID_HSCEXIT);
		dsi->reg->dsi_inst_jump_sel1.dwval =
			DSI_INST_ID_HSD << (4 * (DSI_INST_ID_DS_1 - 8));
		break;
	case DSI_START_HSTX_CLK_BREAK:
		dsi->reg->dsi_inst_jump_sel.dwval =
			DSI_INST_ID_NOP << (4 * DSI_INST_ID_LP11)
			| DSI_INST_ID_HSC << (4 * DSI_INST_ID_NOP)
			| DSI_INST_ID_HSD << (4 * DSI_INST_ID_HSC)
			| DSI_INST_ID_DLY << (4 * DSI_INST_ID_HSD)
			| DSI_INST_ID_HSCEXIT << (4 * DSI_INST_ID_DLY)
			| DSI_INST_ID_DLY_1 << (4 * DSI_INST_ID_HSCEXIT);
		dsi->reg->dsi_inst_jump_sel1.dwval =
			DSI_INST_ID_NOP_1 << (4 * (DSI_INST_ID_DLY_1 - 8))
			| DSI_INST_ID_NOP << (4 * (DSI_INST_ID_NOP_1 - 8))
			| DSI_INST_ID_LP11_1 << (4 * (DSI_INST_ID_HSCEXIT_1 - 8))
			| DSI_INST_ID_END  << (4 * (DSI_INST_ID_LP11_1 - 8));
		break;
	case DSI_START_HSTX_TEST:
		dsi->reg->dsi_inst_jump_sel.dwval =
			DSI_INST_ID_NOP << (4 * DSI_INST_ID_LP11)
			| DSI_INST_ID_HSC << (4 * DSI_INST_ID_NOP)
			| DSI_INST_ID_HSD << (4 * DSI_INST_ID_HSC)
			| DSI_INST_ID_DLY << (4 * DSI_INST_ID_HSD)
			| DSI_INST_ID_HSCEXIT << (4 * DSI_INST_ID_DLY)
			| DSI_INST_ID_DLY_1 << (4* DSI_INST_ID_HSCEXIT);
		dsi->reg->dsi_inst_jump_sel1.dwval =
			DSI_INST_ID_NOP_1 << (4 * (DSI_INST_ID_DLY_1 - 8))
			| DSI_INST_ID_NOP << (4 * (DSI_INST_ID_NOP_1 - 8))
			| DSI_INST_ID_LP11_1 << (4 * (DSI_INST_ID_HSCEXIT_1 - 8))
			| DSI_INST_ID_LPDT_1 << (4 * (DSI_INST_ID_LP11_1 - 8))
			| DSI_START_TBA << (4 * (DSI_INST_ID_LPDT_1 - 8))
			| DSI_INST_ID_END << (4 * DSI_START_TBA);
		break;
	case DSI_INST_TEST:
		dsi->reg->dsi_inst_jump_sel.dwval =
			DSI_INST_ID_LP11_01 << (4 * DSI_INST_ID_LP11_00)
			| DSI_INST_ID_NOP_00 << (4 * DSI_INST_ID_LP11_01)
			| DSI_INST_ID_HSC_00 << (4 * DSI_INST_ID_NOP_00)
			| DSI_INST_ID_DLY_00 << (4 * DSI_INST_ID_HSC_00)
			| DSI_INST_ID_HSD_00 << (4 * DSI_INST_ID_DLY_00)
			| DSI_INST_ID_DLY_01 << (4 * DSI_INST_ID_HSD_00)
			| DSI_INST_ID_HSCEXIT_00 << (4 * DSI_INST_ID_DLY_01)
			| DSI_INST_ID_DLY_02 << (4 * DSI_INST_ID_HSCEXIT_00);
		dsi->reg->dsi_inst_jump_sel1.dwval =
			DSI_INST_ID_LP11_02 << (4 * (DSI_INST_ID_DLY_02 - 8))
			| DSI_INST_ID_NOP_01 << (4 * (DSI_INST_ID_LP11_02 - 8))
			| DSI_INST_ID_NOP_02 << (4 * (DSI_INST_ID_NOP_01 - 8))
			| DSI_INST_ID_DLY_03 << (4 * (DSI_INST_ID_NOP_02 - 8))
			| DSI_INST_ID_LP11_03 << (4 * (DSI_INST_ID_DLY_03 - 8))
			| DSI_INST_ID_NOP_03 << (4 * (DSI_INST_ID_LP11_03 - 8))
			| DSI_INST_ID_NOP_00 << (4 * (DSI_INST_ID_NOP_03 - 8));
		break;
	default:
		dsi->reg->dsi_inst_jump_sel.dwval =
		    DSI_INST_ID_END << (4 * DSI_INST_ID_LP11);
		break;
	}
	dsi->reg->dsi_basic_ctl0.bits.inst_st = 0;
	dsi->reg->dsi_basic_ctl0.bits.inst_st = 1;
	if (func == DSI_START_HSC)
		dsi->reg->dsi_inst_func[DSI_INST_ID_LP11].bits.lane_cen =
		    (dsi->reg->dsi_pixel_ctl0.bits.pd_plug_dis) ? 0 : 1;

	return 0;
}

static void dsi_read_mode_en(struct sunxi_dsi_lcd *dsi, u32 en)
{
	dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_en = en;
	if (!en)
		dsi_start(dsi, DSI_START_HSTX);
}

u32 dsi_get_cur_line(struct sunxi_dsi_lcd *dsi)
{
	u32 curr_line = dsi->reg->dsi_debug_video0.bits.video_curr_line;
	u32 vt = dsi->reg->dsi_basic_size1.bits.vt;
	u32 vsa = dsi->reg->dsi_basic_size0.bits.vsa;
	u32 vbp = dsi->reg->dsi_basic_size0.bits.vbp;
	u32 y = dsi->reg->dsi_basic_size1.bits.vact;
	u32 vfp = vt - vsa - vbp - y;

	curr_line += vfp;
	if (curr_line > vt)
		curr_line -= vt;

	return curr_line;
}

s32 dsi_irq_enable(struct sunxi_dsi_lcd *dsi, enum __dsi_irq_id_t id)
{
	dsi->reg->dsi_gint0.bits.dsi_irq_en |= (1 << id);

	return 0;
}

s32 dsi_irq_disable(struct sunxi_dsi_lcd *dsi, enum __dsi_irq_id_t id)
{
	dsi->reg->dsi_gint0.bits.dsi_irq_en &= ~(1 << id);

	return 0;
}

u32 dsi_irq_query(struct sunxi_dsi_lcd *dsi, enum __dsi_irq_id_t id)
{
	u32 en, fl;

	en = dsi->reg->dsi_gint0.bits.dsi_irq_en;
	fl = dsi->reg->dsi_gint0.bits.dsi_irq_flag;
	if (en & fl & (1 << id)) {
		dsi->reg->dsi_gint0.bits.dsi_irq_flag |= (1 << id);
		return 1;
	} else {
		return 0;
	}
}

s32 dsi_inst_busy(struct sunxi_dsi_lcd *dsi)
{
	return dsi->reg->dsi_basic_ctl0.bits.inst_st;
}

s32 dsi_open(struct sunxi_dsi_lcd *dsi, struct disp_dsi_para *para)
{
//	dsi_irq_enable(dsi, DSI_IRQ_VIDEO_VBLK);
	dsi_start(dsi, DSI_START_HSD);
	if (para->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		dsi_start(dsi, DSI_START_HSTX_CLK_BREAK);
	return 0;
}

s32 dsi_close(struct sunxi_dsi_lcd *dsi)
{
//	dsi_irq_disable(dsi, DSI_IRQ_VIDEO_VBLK);
	dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_en = 1;
	dsi_delay_ms(30);
	/*
	 * while (dphy_dev[sel]->dphy_dbg0.bits.lptx_sta_d0 == 5);
	 * dphy_dev[sel]->dphy_ana2.bits.enp2s_cpu	= 0;
	 * dphy_dev[sel]->dphy_ana1.bits.reg_vttmode =	1;
	 */

	return 0;
}

void dsi_enable_vblank(struct sunxi_dsi_lcd *dsi, bool enable)
{
	if (enable)
		dsi_irq_enable(dsi, DSI_IRQ_VIDEO_VBLK);
	else
		dsi_irq_disable(dsi, DSI_IRQ_VIDEO_VBLK);
}

/**
 * @name       dsi_mode_switch
 * @brief      switch dsi mode between cmd and video mode
 * @param[IN]  sel: dsi module index; en:1-->video mode
 *             0--> cmd mode
 * @return     alway return 0
 */
__s32 dsi_mode_switch(struct sunxi_dsi_lcd *dsi, __u32 cmd_en, __u32 lp_en)
{
	return 0;
}

/* 0: normal; -1:under flow; */
s32 dsi_get_status(struct sunxi_dsi_lcd *dsi)
{
	if (dsi->reg->dsi_debug_inst.bits.trans_low_flag ||
	    dsi->reg->dsi_debug_inst.bits.trans_fast_flag) {
		dsi->reg->dsi_debug_inst.bits.trans_low_flag = 1;
		dsi->reg->dsi_debug_inst.bits.trans_fast_flag = 1;
		return -1;
	}
	return 0;
}

s32 dsi_tri_start(struct sunxi_dsi_lcd *dsi)
{
	dsi_start(dsi, DSI_START_HSTX);
#ifdef DEBUG
	while (dsi->reg->dsi_debug_inst.bits.curr_instru_num
	    != DSI_INST_ID_HSD)
		;
	while (dphy_dev[sel]->dphy_dbg0.bits.lptx_sta_d0 != 5)
		;
#endif

	return 0;
}

s32 dsi_dcs_wr(struct sunxi_dsi_lcd *dsi, u8 *para_p, u32 para_num)
{
	volatile u8 *p = (u8 *) dsi->reg->dsi_cmd_tx;
	u32 count = 0, i;

	while ((dsi->reg->dsi_basic_ctl0.bits.inst_st == 1)
	    && (count < 500)) {
		count++;
		dsi_delay_us(10);
	}
	if (count >= 50)
		dsi->reg->dsi_basic_ctl0.bits.inst_st = 0;

	for (i = 0; i < para_num; i++)
		*(p++) = *(para_p + i);

	dsi->reg->dsi_cmd_ctl.bits.tx_size = para_num - 1;

	dsi_start(dsi, DSI_START_LPTX);

	return 0;
}

s32 dsi_dcs_rd(struct sunxi_dsi_lcd *dsi, u8 *para_p, u32 num_p)
{
	u32 num, i;
	u32 count = 0;

	dsi_read_mode_en(dsi, 1);
	dsi_start(dsi, DSI_START_LPRX);
	while ((dsi->reg->dsi_basic_ctl0.bits.inst_st == 1)
	    && (count < 500)) {
		count++;
		dsi_delay_us(10);
	}
	if (count >= 50)
		dsi->reg->dsi_basic_ctl0.bits.inst_st = 0;

	if (dsi->reg->dsi_cmd_ctl.bits.rx_flag) {
		if (dsi->reg->dsi_cmd_ctl.bits.rx_overflow)
			return -1;
		if (dsi->reg->dsi_cmd_rx[0].bits.byte0 == DSI_DT_ACK_ERR)
			return -1;

		num = dsi->reg->dsi_cmd_ctl.bits.rx_size + 1;
		if (num >= num_p)
			num = num_p;
		else
			printk("unable to read %d data, only %d data can br read\n", num_p, num);
		for (i = 0; i < num; i++) {
			*(para_p + i) =
				*((u8 *) dsi->reg->dsi_cmd_rx + i);
		}
	}
	dsi_read_mode_en(dsi, 0);

	return 0;
}
#ifdef DEBUG
s32 dsi_dcs_rd_memory(struct sunxi_dsi_lcd *dsi, u32 *p_data, u32 length)
{
	u32 rx_cntr = length;
	u32 rx_curr;
	u32 *rx_p = p_data;
	u32 i;
	u32 start = 1;

	u8 rx_bf[32];
	u32 rx_num;

	dsi_set_max_ret_size(sel, 24);

	while (rx_cntr) {
		if (rx_cntr >= 8) {
			rx_curr = 8;
		} else {
			rx_curr = rx_cntr;
			dsi_set_max_ret_size(sel, rx_curr * 3);
		}
		rx_cntr -= rx_curr;

		if (start) {
			dsi_dcs_rd(dsi, DSI_DCS_READ_MEMORY_START, rx_bf,
				   &rx_num);
			start = 0;
		} else {
			dsi_dcs_rd(dsi, DSI_DCS_READ_MEMORY_CONTINUE, rx_bf,
				   &rx_num);
		}

		if (rx_num != rx_curr * 3)
			return -1;

		for (i = 0; i < rx_curr; i++) {
			*rx_p &= 0xff000000;
			*rx_p |= 0xff000000;
			*rx_p |= (u32) *(rx_bf + 4 + i * 3 + 0) << 16;
			*rx_p |= (u32) *(rx_bf + 4 + i * 3 + 1) << 8;
			*rx_p |= (u32) *(rx_bf + 4 + i * 3 + 2) << 0;
			rx_p++;
		}
	}
	return 0;
}

#endif
s32 dsi_clk_enable(struct sunxi_dsi_lcd *dsi, struct disp_dsi_para *para, u32 en)
{
	if (en) {
		if (para->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
			dsi_start(dsi, DSI_START_HSTX_CLK_BREAK);
		else
			dsi_start(dsi, DSI_START_HSC);
	}

	return 0;
}

static s32 dsi_basic_cfg(struct sunxi_dsi_lcd *dsi, struct disp_dsi_para *para)
{
	printk("[lowlevel] x:%d, y:%d, vt:%d, vbp:%d, vspw:%d, ht:%d, hbp:%d, hspw:%d\n",
		para->timings.x_res,
		para->timings.y_res,
		para->timings.ver_total_time,
		para->timings.ver_back_porch,
		para->timings.ver_sync_time,
		para->timings.hor_total_time,
		para->timings.hor_back_porch,
		para->timings.hor_sync_time);
	if (para->mode_flags & MIPI_DSI_MODE_COMMAND) {
		dsi->reg->dsi_basic_ctl0.bits.ecc_en = 1;
		dsi->reg->dsi_basic_ctl0.bits.crc_en = 1;
		if (para->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET)
			dsi->reg->dsi_basic_ctl0.bits.hs_eotp_en = 1;
		dsi->reg->dsi_basic_ctl1.bits.dsi_mode = 0;
		dsi->reg->dsi_trans_start.bits.trans_start_set = 10;
		dsi->reg->dsi_trans_zero.bits.hs_zero_reduce_set = 0;
	} else {
		s32 start_delay = para->timings.ver_total_time - para->timings.y_res - 10;
		u32 dsi_start_delay;

		/*
		 * put start_delay to tcon.
		 * set ready sync early to dramfreq, so set start_delay 1
		 */
		start_delay = 1;

		dsi_start_delay = start_delay;
		if (dsi_start_delay > para->timings.ver_total_time)
			dsi_start_delay -= para->timings.ver_total_time;
		if (dsi_start_delay == 0)
			dsi_start_delay = 1;

		dsi->reg->dsi_basic_ctl0.bits.ecc_en = 1;
		dsi->reg->dsi_basic_ctl0.bits.crc_en = 1;
		if (para->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET)
			dsi->reg->dsi_basic_ctl0.bits.hs_eotp_en = 1;
		dsi->reg->dsi_basic_ctl1.bits.video_start_delay =
		    dsi_start_delay;
		dsi->reg->dsi_basic_ctl1.bits.video_precision_mode_align =
		    1;
		dsi->reg->dsi_basic_ctl1.bits.video_frame_start = 1;
		dsi->reg->dsi_trans_start.bits.trans_start_set = 10;
		dsi->reg->dsi_trans_zero.bits.hs_zero_reduce_set = 0;
		dsi->reg->dsi_basic_ctl1.bits.dsi_mode = 1;

		if (para->mode_flags & MIPI_DSI_SLAVE_MODE)
			dsi->reg->dsi_basic_ctl1.bits.tri_delay = 48;

		if (para->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
			u32 line_num, edge0, edge1, sync_point = 40;

			line_num =  para->timings.hor_total_time * dsi_pixel_bits[para->format]
				/ (8 * para->lanes);
			edge1 = sync_point + (para->timings.x_res + para->timings.hor_back_porch +
					para->timings.hor_sync_time + 20) *
					dsi_pixel_bits[para->format] / (8 * para->lanes);
			edge1 = (edge1 > line_num) ? line_num : edge1;
			edge0 = edge1 + (para->timings.x_res + 40) * para->dsi_div / 8;
			edge0 = (edge0 > line_num) ? (edge0 - line_num) : 1;

			dsi->reg->dsi_basic_ctl1.bits.tri_delay =
				para->timings.hor_total_time / 10 * 2;
			dsi->reg->dsi_burst_drq.bits.drq_edge0 = edge0;
			dsi->reg->dsi_burst_drq.bits.drq_edge1 = edge1;
			dsi->reg->dsi_tcon_drq.bits.drq_mode = 1;
			dsi->reg->dsi_burst_line.bits.line_num = line_num;
			dsi->reg->dsi_burst_line.bits.line_syncpoint = sync_point;
			dsi->reg->dsi_basic_ctl.bits.video_mode_burst = 1;

		} else {
			if ((para->timings.hor_total_time - para->timings.x_res - para->timings.hor_back_porch)
			    < 21) {
				dsi->reg->dsi_tcon_drq.bits.drq_mode = 0;
			} else {
				dsi->reg->dsi_tcon_drq.bits.drq_set =
				(para->timings.hor_total_time - para->timings.x_res -
				 para->timings.hor_back_porch - para->timings.hor_sync_time -20)
				    * dsi_pixel_bits[para->format] /
				    (8 * 4);
				dsi->reg->dsi_tcon_drq.bits.drq_mode = 1;
			}
		}
	}
	dsi->reg->dsi_inst_func[DSI_INST_ID_LP11].bits.instru_mode =
	    DSI_INST_MODE_STOP;
	dsi->reg->dsi_inst_func[DSI_INST_ID_LP11].bits.lane_cen = 1;
	dsi->reg->dsi_inst_func[DSI_INST_ID_LP11].bits.lane_den =
	    dsi_lane_den[para->lanes - 1];
	dsi->reg->dsi_inst_func[DSI_INST_ID_TBA].bits.instru_mode =
	    DSI_INST_MODE_TBA;
	dsi->reg->dsi_inst_func[DSI_INST_ID_TBA].bits.lane_cen = 0;
	dsi->reg->dsi_inst_func[DSI_INST_ID_TBA].bits.lane_den = 0x1;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSC].bits.instru_mode =
	    DSI_INST_MODE_HS;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSC].bits.trans_packet =
	    DSI_INST_PACK_PIXEL;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSC].bits.lane_cen = 1;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSC].bits.lane_den = 0;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSD].bits.instru_mode =
	    DSI_INST_MODE_HS;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSD].bits.trans_packet =
	    DSI_INST_PACK_PIXEL;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSD].bits.lane_cen = 0;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSD].bits.lane_den =
	    dsi_lane_den[para->lanes - 1];
	dsi->reg->dsi_inst_func[DSI_INST_ID_LPDT].bits.instru_mode =
	    DSI_INST_MODE_ESCAPE;
	dsi->reg->dsi_inst_func[DSI_INST_ID_LPDT].bits.escape_enrty =
	    DSI_INST_ESCA_LPDT;
	dsi->reg->dsi_inst_func[DSI_INST_ID_LPDT].bits.trans_packet =
	    DSI_INST_PACK_COMMAND;
	dsi->reg->dsi_inst_func[DSI_INST_ID_LPDT].bits.lane_cen = 0;
	dsi->reg->dsi_inst_func[DSI_INST_ID_LPDT].bits.lane_den = 0x1;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSCEXIT].bits.instru_mode =
	    DSI_INST_MODE_HSCEXIT;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSCEXIT].bits.lane_cen = 1;
	dsi->reg->dsi_inst_func[DSI_INST_ID_HSCEXIT].bits.lane_den = 0;
	dsi->reg->dsi_inst_func[DSI_INST_ID_NOP].bits.instru_mode =
	    DSI_INST_MODE_STOP;
	dsi->reg->dsi_inst_func[DSI_INST_ID_NOP].bits.lane_cen = 0;
	dsi->reg->dsi_inst_func[DSI_INST_ID_NOP].bits.lane_den =
	    dsi_lane_den[para->lanes - 1];
	dsi->reg->dsi_inst_func[DSI_INST_ID_DLY].bits.instru_mode =
	    DSI_INST_MODE_NOP;
	dsi->reg->dsi_inst_func[DSI_INST_ID_DLY].bits.lane_cen = 1;
	dsi->reg->dsi_inst_func[DSI_INST_ID_DLY].bits.lane_den =
	    dsi_lane_den[para->lanes - 1];
		/* LP11 */
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LP11_1-8].bits.instru_mode =
		DSI_INST_MODE_STOP;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LP11_1-8].bits.lane_cen = 1;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LP11_1-8].bits.lane_den =
		dsi_lane_den[para->lanes - 1];
	/* clock lane HS */
	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSC_1-8].bits.instru_mode =
		DSI_INST_MODE_HS;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSC_1-8].bits.trans_packet =
		DSI_INST_PACK_PIXEL;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSC_1-8].bits.lane_cen	= 1;

	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSC_1-8].bits.lane_den	= 0;

	/* data lane initial skew calibration */

	dsi->reg->dsi_inst_func1[DSI_INST_ID_DS_1-8].bits.instru_mode =
		DSI_INST_MODE_SCINIT;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_DS_1-8].bits.trans_packet =
		DSI_INST_PACK_PIXEL;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_DS_1-8].bits.lane_cen = 0;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_DS_1-8].bits.lane_den =
		dsi_lane_den[para->lanes - 1];

	/* lane0 escape LPDT */
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LPDT_1-8].bits.instru_mode =
		DSI_INST_MODE_ESCAPE;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LPDT_1-8].bits.escape_enrty =
		DSI_INST_ESCA_LPDT;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LPDT_1-8].bits.trans_packet =
		DSI_INST_PACK_COMMAND;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LPDT_1-8].bits.lane_cen = 0;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_LPDT_1-8].bits.lane_den = 0x1;
	/* clock lane HS exit */
	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSCEXIT_1-8].bits.instru_mode =
		DSI_INST_MODE_HSCEXIT;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSCEXIT_1-8].bits.lane_cen = 1;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_HSCEXIT_1-8].bits.lane_den = 0;
	/* data lane LP11 */
	dsi->reg->dsi_inst_func1[DSI_INST_ID_NOP_1-8].bits.instru_mode =
		DSI_INST_MODE_STOP;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_NOP_1-8].bits.lane_cen	= 0;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_NOP_1-8].bits.lane_den	=
		dsi_lane_den[para->lanes - 1];
	/* all lane NOP */
	dsi->reg->dsi_inst_func1[DSI_INST_ID_DLY_1-8].bits.instru_mode =
		DSI_INST_MODE_NOP;
	dsi->reg->dsi_inst_func1[DSI_INST_ID_DLY_1-8].bits.lane_cen	= 1;

	dsi->reg->dsi_inst_func1[DSI_INST_ID_DLY_1-8].bits.lane_den	=
		dsi_lane_den[para->lanes - 1];
	dsi->reg->dsi_inst_loop_sel.dwval = 2 << (4 * DSI_INST_ID_LP11)
	    | 3 << (4 * DSI_INST_ID_DLY);
	dsi->reg->dsi_inst_loop_sel1.dwval = 2 << (4 * (DSI_INST_ID_LP11_1-8))
		| 3 << (4 * (DSI_INST_ID_DLY_1-8));
	dsi->reg->dsi_inst_loop_num.bits.loop_n0 = 50 - 1;
	dsi->reg->dsi_inst_loop_num2.bits.loop_n0 = 50 - 1;
	if (para->mode_flags & MIPI_DSI_MODE_COMMAND) {
		dsi->reg->dsi_inst_loop_num.bits.loop_n1 = 1 - 1;
		dsi->reg->dsi_inst_loop_num2.bits.loop_n1 = 1 - 1;
	} else if (para->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		int tbyteclk, tdsi_clk;
		u32 loop_n1, tmp;
		u32 hs_trail_set = 10;
		tbyteclk = 100000 / (para->timings.pixel_clk / 1000000 * 6 / 8); /* 24bpp */
		tdsi_clk = 670;  /* dsi_clk = 150mhz */
		tmp = ((para->timings.hor_front_porch * 3 - (hs_trail_set + 2 + 10 + 35 - 4)
					* tbyteclk) / tdsi_clk - 24) / 7 ;
		loop_n1 = tmp > 0 ? tmp : 0;
		dsi->reg->dsi_inst_loop_num.bits.loop_n1 = 0;
		dsi->reg->dsi_inst_loop_num2.bits.loop_n1 = 0;
/*
		dsi->reg->dsi_inst_loop_num.bits.loop_n1 =
		    (para->timings.hor_total_time - para->timings.x_res) * (150) *
		    dsi_pixel_bits[para->format] / ( para->lanes * 8 * 1000) - 50;
		dsi->reg->dsi_inst_loop_num2.bits.loop_n1 =
		    (para->timings.hor_total_time - para->timings.x_res) * (150) *
		    dsi_pixel_bits[para->format] / ( para->lanes * 8 * 1000) - 50;
*/
	} else if (para->mode_flags & MIPI_DSI_MODE_VIDEO) {
		dsi->reg->dsi_inst_loop_num.bits.loop_n1 = 50 - 1;
		dsi->reg->dsi_inst_loop_num2.bits.loop_n1 = 50 - 1;
	}

	if (para->mode_flags & MIPI_DSI_MODE_COMMAND) {
		dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_en = 1;
		dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_num =
		    para->timings.y_res;
	} else {
		dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_en = 0;
		dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_num = 1;
	}

	dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_point =
	    DSI_INST_ID_NOP;
	dsi->reg->dsi_inst_jump_cfg[0].bits.jump_cfg_to =
	    DSI_INST_ID_HSCEXIT;
	dsi->reg->dsi_debug_data.bits.test_data = 0xff;
	dsi->reg->dsi_gctl.bits.dsi_en = 1;
	return 0;
}

static s32 dsi_packet_cfg(struct sunxi_dsi_lcd *dsi, struct disp_dsi_para *para)
{
	if (para->mode_flags & MIPI_DSI_MODE_COMMAND) {
		dsi->reg->dsi_pixel_ctl0.bits.pd_plug_dis = 0;
		//dsi->reg->dsi_pixel_ph.bits.vc = panel->lcd_dsi_vc;
		dsi->reg->dsi_pixel_ph.bits.vc = 0;
		dsi->reg->dsi_pixel_ph.bits.dt = DSI_DT_DCS_LONG_WR;
		dsi->reg->dsi_pixel_ph.bits.wc = 1 +
		    para->timings.x_res * dsi_pixel_bits[para->format] / 8;
		dsi->reg->dsi_pixel_ph.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_pixel_ph.dwval);
		dsi->reg->dsi_pixel_pd.bits.pd_tran0 =
		    DSI_DCS_WRITE_MEMORY_START;
		dsi->reg->dsi_pixel_pd.bits.pd_trann =
		    DSI_DCS_WRITE_MEMORY_CONTINUE;
		dsi->reg->dsi_pixel_pf0.bits.crc_force = 0xffff;
		dsi->reg->dsi_pixel_pf1.bits.crc_init_line0 = 0xe4e9;
		dsi->reg->dsi_pixel_pf1.bits.crc_init_linen = 0xf468;
		dsi->reg->dsi_pixel_ctl0.bits.pixel_format = para->format;
	} else {
		dsi->reg->dsi_pixel_ctl0.bits.pd_plug_dis = 1;
		//dsi->reg->dsi_pixel_ph.bits.vc = panel->lcd_dsi_vc;
		dsi->reg->dsi_pixel_ph.bits.vc = 0;
		dsi->reg->dsi_pixel_ph.bits.dt =
		    DSI_DT_PIXEL_RGB888 - 0x10 * para->format;
		dsi->reg->dsi_pixel_ph.bits.wc =
		    para->timings.x_res * dsi_pixel_bits[para->format] / 8;
		dsi->reg->dsi_pixel_ph.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_pixel_ph.dwval);
		dsi->reg->dsi_pixel_pf0.bits.crc_force = 0xffff;
		dsi->reg->dsi_pixel_pf1.bits.crc_init_line0 = 0xffff;
		dsi->reg->dsi_pixel_pf1.bits.crc_init_linen = 0xffff;
		dsi->reg->dsi_pixel_ctl0.bits.pixel_format =
		    8 + para->format;
	}
	if (para->mode_flags & MIPI_DSI_MODE_VIDEO) {
		u32 dsi_hsa, dsi_hact, dsi_hbp, dsi_hfp, dsi_hblk, dsi_vblk;
		u32 tmp;
		u32 vc = 0;
		u32 x = para->timings.x_res;
		u32 y = para->timings.y_res;
		u32 vt = para->timings.ver_total_time;
		u32 vbp = para->timings.ver_back_porch;
		u32 vspw = para->timings.ver_sync_time;
		u32 ht = para->timings.hor_total_time;
		u32 hbp = para->timings.hor_back_porch;
		u32 hspw = para->timings.hor_sync_time;
		u32 format = para->format;
		u32 lane = para->lanes;

		 if (para->dual_dsi) {
			int overlap = 0;

			dsi_hsa = hspw / 2 * dsi_pixel_bits[format] / 8 - (4 + 4 + 2);
			dsi_hbp = (hbp + hspw) / 2 * dsi_pixel_bits[format] / 8 - (4 + 4 + 2);
			dsi->reg->dsi_pixel_ph.bits.wc = (para->timings.x_res / 2 + overlap) * \
				dsi_pixel_bits[para->format] / 8;

			dsi_hfp = ((ht - (hspw + hbp) - hspw) / 2 - (x / 2 + overlap)) * 3 - 6 - 6;
			dsi_hblk = (ht - hspw) / 2 * dsi_pixel_bits[format] / 8 - (4 + 4 + 2);

			if (lane == 4)
				dsi_vblk = ((ht / 2 - hspw / 2) * 3 - 10) / 2;  /* OK  */
			else
				dsi_vblk = 0;

			/* Switch to TCON to trigger DSI work */
			dsi->reg->dsi_basic_ctl.bits.start_mode = 1;
		} else {
			dsi_hsa =
			    hspw * dsi_pixel_bits[format] / 8 - 10;
			dsi_hbp = hbp * dsi_pixel_bits[format] / 8
			    - 6;
			dsi_hact = x * dsi_pixel_bits[format] / 8;
			dsi_hblk = (ht - hspw) * dsi_pixel_bits[format] / 8
			    - 10;
			dsi_hfp = dsi_hblk - (4 + dsi_hact + 2)
			    - (4 + dsi_hbp + 2);

			if (lane == 4) {
				tmp = (ht * dsi_pixel_bits[format] / 8) * vt
				    - (4 + dsi_hblk + 2);
				dsi_vblk = (lane - tmp % lane);
			} else {
				dsi_vblk = 0;
			}
			if (para->mode_flags & MIPI_DSI_SLAVE_MODE)
				dsi->reg->dsi_basic_ctl.bits.start_mode = 1;
		}

		if (para->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
			u32 hsa_hse_dis = 1, hbp_dis = 0;
			dsi_vblk = 0;
			dsi_hfp = 0;
			dsi_hact = x * dsi_pixel_bits[format] / 8;
			dsi_hsa = hsa_hse_dis ? 0 : hspw * dsi_pixel_bits[format] / 8 - 10;
			dsi_hbp = hbp_dis ? 0 : hbp * dsi_pixel_bits[format] / 8 - 14;
			dsi->reg->dsi_basic_ctl.bits.hsa_hse_dis = hsa_hse_dis;
			dsi->reg->dsi_basic_ctl.bits.hbp_dis = hbp_dis;
			if (hsa_hse_dis == 1 && hbp_dis == 1) {
				dsi_hblk = dsi_hact;
				if (lane == 4) {
					dsi->reg->dsi_basic_ctl.bits.trail_inv = 0xc;
					dsi->reg->dsi_basic_ctl.bits.trail_fill = 1;
				}
			} else if (hsa_hse_dis == 0 && hbp_dis == 1) {
				dsi_hblk = dsi_hact;
				tmp = 4 + (4 + dsi_hsa + 2) + 4 + (4 + dsi_hblk + 2);
				tmp = 4 - tmp % 4;
				dsi_hsa += tmp;
			} else if (hsa_hse_dis == 1 && hbp_dis == 0) {
				dsi_hblk = (4 + dsi_hbp + 2) + dsi_hact;
				tmp = 4 + (4 + dsi_hblk + 2);
				tmp = 4 - tmp % 4;
				dsi_hbp += tmp;
				dsi_hblk += tmp;
			} else {
				dsi_hblk = (4 + dsi_hbp + 2) + dsi_hact;
				tmp = 4 + (4 + dsi_hsa + 2) + 4 + (4 + dsi_hblk + 2);
				tmp = 4 - tmp % 4;
				dsi_hbp += tmp;
				dsi_hblk += tmp;
			}
		}
		dsi->reg->dsi_sync_hss.bits.vc = vc;
		dsi->reg->dsi_sync_hss.bits.dt = DSI_DT_HSS;
		dsi->reg->dsi_sync_hss.bits.d0 = 0;
		dsi->reg->dsi_sync_hss.bits.d1 = 0;
		dsi->reg->dsi_sync_hss.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_sync_hss.dwval);

		dsi->reg->dsi_sync_hse.bits.vc = vc;
		dsi->reg->dsi_sync_hse.bits.dt = DSI_DT_HSE;
		dsi->reg->dsi_sync_hse.bits.d0 = 0;
		dsi->reg->dsi_sync_hse.bits.d1 = 0;
		dsi->reg->dsi_sync_hse.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_sync_hse.dwval);

		dsi->reg->dsi_sync_vss.bits.vc = vc;
		dsi->reg->dsi_sync_vss.bits.dt = DSI_DT_VSS;
		dsi->reg->dsi_sync_vss.bits.d0 = 0;
		dsi->reg->dsi_sync_vss.bits.d1 = 0;
		dsi->reg->dsi_sync_vss.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_sync_vss.dwval);

		dsi->reg->dsi_sync_vse.bits.vc = vc;
		if (para->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			dsi->reg->dsi_sync_vse.bits.dt = DSI_DT_HSS;
		else
			dsi->reg->dsi_sync_vse.bits.dt = DSI_DT_VSE;
		dsi->reg->dsi_sync_vse.bits.d0 = 0;
		dsi->reg->dsi_sync_vse.bits.d1 = 0;
		dsi->reg->dsi_sync_vse.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_sync_vse.dwval);

		dsi->reg->dsi_basic_size0.bits.vsa = vspw;
		dsi->reg->dsi_basic_size0.bits.vbp = vbp;
		dsi->reg->dsi_basic_size1.bits.vact = y;
		dsi->reg->dsi_basic_size1.bits.vt = vt;
		dsi->reg->dsi_blk_hsa0.bits.vc = vc;
		dsi->reg->dsi_blk_hsa0.bits.dt = DSI_DT_BLK;
		dsi->reg->dsi_blk_hsa0.bits.wc = dsi_hsa;
		dsi->reg->dsi_blk_hsa0.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_blk_hsa0.dwval);

		dsi->reg->dsi_blk_hsa1.bits.pd = 0;
		dsi->reg->dsi_blk_hsa1.bits.pf =
		    dsi_crc_pro_pd_repeat(0, dsi_hsa);

		dsi->reg->dsi_blk_hbp0.bits.vc = vc;
		dsi->reg->dsi_blk_hbp0.bits.dt = DSI_DT_BLK;
		dsi->reg->dsi_blk_hbp0.bits.wc = dsi_hbp;
		dsi->reg->dsi_blk_hbp0.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_blk_hbp0.dwval);

		dsi->reg->dsi_blk_hbp1.bits.pd = 0;
		dsi->reg->dsi_blk_hbp1.bits.pf =
		    dsi_crc_pro_pd_repeat(0, dsi_hbp);

		dsi->reg->dsi_blk_hfp0.bits.vc = vc;
		dsi->reg->dsi_blk_hfp0.bits.dt = DSI_DT_BLK;
		dsi->reg->dsi_blk_hfp0.bits.wc = dsi_hfp;
		dsi->reg->dsi_blk_hfp0.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_blk_hfp0.dwval);

		dsi->reg->dsi_blk_hfp1.bits.pd = 0;
		dsi->reg->dsi_blk_hfp1.bits.pf =
		    dsi_crc_pro_pd_repeat(0, dsi_hfp);

		dsi->reg->dsi_blk_hblk0.bits.dt = DSI_DT_BLK;
		dsi->reg->dsi_blk_hblk0.bits.wc = dsi_hblk;
		dsi->reg->dsi_blk_hblk0.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_blk_hblk0.dwval);

		dsi->reg->dsi_blk_hblk1.bits.pd = 0;
		dsi->reg->dsi_blk_hblk1.bits.pf =
		    dsi_crc_pro_pd_repeat(0, dsi_hblk);

		dsi->reg->dsi_blk_vblk0.bits.dt = DSI_DT_BLK;
		dsi->reg->dsi_blk_vblk0.bits.wc = dsi_vblk;
		dsi->reg->dsi_blk_vblk0.bits.ecc =
		    dsi_ecc_pro(dsi->reg->dsi_blk_vblk0.dwval);

		dsi->reg->dsi_blk_vblk1.bits.pd = 0;
		dsi->reg->dsi_blk_vblk1.bits.pf =
		    dsi_crc_pro_pd_repeat(0, dsi_vblk);
	}
	return 0;
}

s32 dsi_cfg(struct sunxi_dsi_lcd *dsi, struct disp_dsi_para *dsi_para)
{
	dsi_basic_cfg(dsi, dsi_para);
	dsi_packet_cfg(dsi, dsi_para);
	return 0;
}

u8 dsi_ecc_pro(u32 dsi_ph)
{
	union dsi_ph_t ph;

	ph.bytes.byte012 = dsi_ph;

	ph.bits.bit29 =
	    ph.bits.bit10 ^ ph.bits.bit11 ^ ph.bits.bit12 ^ ph.bits.bit13 ^ ph.
	    bits.bit14 ^ ph.bits.bit15 ^ ph.bits.bit16 ^ ph.bits.bit17 ^ ph.
	    bits.bit18 ^ ph.bits.bit19 ^ ph.bits.bit21 ^ ph.bits.bit22 ^ ph.
	    bits.bit23;
	ph.bits.bit28 =
	    ph.bits.bit04 ^ ph.bits.bit05 ^ ph.bits.bit06 ^ ph.bits.bit07 ^ ph.
	    bits.bit08 ^ ph.bits.bit09 ^ ph.bits.bit16 ^ ph.bits.bit17 ^ ph.
	    bits.bit18 ^ ph.bits.bit19 ^ ph.bits.bit20 ^ ph.bits.bit22 ^ ph.
	    bits.bit23;
	ph.bits.bit27 =
	    ph.bits.bit01 ^ ph.bits.bit02 ^ ph.bits.bit03 ^ ph.bits.bit07 ^ ph.
	    bits.bit08 ^ ph.bits.bit09 ^ ph.bits.bit13 ^ ph.bits.bit14 ^ ph.
	    bits.bit15 ^ ph.bits.bit19 ^ ph.bits.bit20 ^ ph.bits.bit21 ^ ph.
	    bits.bit23;
	ph.bits.bit26 =
	    ph.bits.bit00 ^ ph.bits.bit02 ^ ph.bits.bit03 ^ ph.bits.bit05 ^ ph.
	    bits.bit06 ^ ph.bits.bit09 ^ ph.bits.bit11 ^ ph.bits.bit12 ^ ph.
	    bits.bit15 ^ ph.bits.bit18 ^ ph.bits.bit20 ^ ph.bits.bit21 ^ ph.
	    bits.bit22;
	ph.bits.bit25 =
	    ph.bits.bit00 ^ ph.bits.bit01 ^ ph.bits.bit03 ^ ph.bits.bit04 ^ ph.
	    bits.bit06 ^ ph.bits.bit08 ^ ph.bits.bit10 ^ ph.bits.bit12 ^ ph.
	    bits.bit14 ^ ph.bits.bit17 ^ ph.bits.bit20 ^ ph.bits.bit21 ^ ph.
	    bits.bit22 ^ ph.bits.bit23;
	ph.bits.bit24 =
	    ph.bits.bit00 ^ ph.bits.bit01 ^ ph.bits.bit02 ^ ph.bits.bit04 ^ ph.
	    bits.bit05 ^ ph.bits.bit07 ^ ph.bits.bit10 ^ ph.bits.bit11 ^ ph.
	    bits.bit13 ^ ph.bits.bit16 ^ ph.bits.bit20 ^ ph.bits.bit21 ^ ph.
	    bits.bit22 ^ ph.bits.bit23;
	return ph.bytes.byte3;
}

u16 dsi_crc_pro(u8 *pd_p, u32 pd_bytes)
{
	u16 gen_code = 0x8408;
	u16 byte_cntr;
	u8 bit_cntr;
	u8 curr_data;
	u16 crc = 0xffff;

	if (pd_bytes > 0) {
		for (byte_cntr = 0; byte_cntr < pd_bytes; byte_cntr++) {
			curr_data = *(pd_p + byte_cntr);
			for (bit_cntr = 0; bit_cntr < 8; bit_cntr++) {
				if (((crc & 0x0001) ^
				     ((0x0001 * curr_data) & 0x0001)) > 0)
					crc = ((crc >> 1) & 0x7fff) ^ gen_code;
				else
					crc = (crc >> 1) & 0x7fff;
				curr_data = (curr_data >> 1) & 0x7f;
			}
		}
	}
	return crc;
}

u16 dsi_crc_pro_pd_repeat(u8 pd, u32 pd_bytes)
{
	u16 gen_code = 0x8408;
	u16 byte_cntr;
	u8 bit_cntr;
	u8 curr_data;
	u16 crc = 0xffff;

	/* pd_bytes : 74 */
	if (pd_bytes > 0) {
		for (byte_cntr = 0; byte_cntr < pd_bytes; byte_cntr++) {
			curr_data = pd;
			for (bit_cntr = 0; bit_cntr < 8; bit_cntr++) {
				if (((crc & 0x0001) ^
				     ((0x0001 * curr_data) & 0x0001)) > 0)
					crc = ((crc >> 1) & 0x7fff) ^ gen_code;
				else
					crc = (crc >> 1) & 0x7fff;
				curr_data = (curr_data >> 1) & 0x7f;
			}
		}
	}
	return crc;
}
