/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/csi/csi_reg_v1.c
 *
 * Copyright (c) 2007-2017 Allwinnertech Co., Ltd.
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

/*
 * sunxi csi register read/write interface
 */

#include "csi_reg_v1.h"
#include "../utility/vfe_io.h"

#ifndef CONFIG_ARCH_SUN3IW1P1
#define ADDR_BIT_R_SHIFT 0
#define CLK_POL 0	/*0:RISING, 1:FAILING*/
#else
#define ADDR_BIT_R_SHIFT 2
#define CLK_POL 1	/*0:RISING, 1:FAILING*/
#endif
volatile void __iomem *csi_base_addr[2];
enum csi_input_fmt input_fmt;

int csi_set_base_addr(unsigned int sel, unsigned long addr)
{
	if (sel > MAX_CSI-1)
		return -1;
	csi_base_addr[sel] = (volatile void __iomem *)addr;

	return 0;
}

/* open module */
void csi_enable(unsigned int sel)
{
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_EN, 0x1);
}

void csi_disable(unsigned int sel)
{
	vfe_reg_clr(csi_base_addr[sel] + CSI_REG_EN, 0X1 << 0);
}

/* configure */
void csi_if_cfg(unsigned int sel, struct csi_if_cfg *csi_if_cfg)
{
	if (csi_if_cfg->interface == CSI_IF_CCIR656_16BIT)
		input_fmt = CSI_YUV422_16;
	else if (csi_if_cfg->interface == CSI_IF_CCIR656_1CH)
		input_fmt = CSI_CCIR656;
	else if (csi_if_cfg->interface == CSI_IF_CCIR656_2CH)
		input_fmt = CSI_CCIR656_2CH;
	else if (csi_if_cfg->interface == CSI_IF_CCIR656_4CH)
		input_fmt = CSI_CCIR656_4CH;
	vfe_reg_clr_set(csi_base_addr[sel] + CSI_REG_CONF, 0x7 << 20,
						input_fmt << 20);/*[22:20]*/
}

void csi_timing_cfg(unsigned int sel, struct csi_timing_cfg *csi_tmg_cfg)
{
	vfe_reg_clr_set(csi_base_addr[sel] + CSI_REG_CONF, 0x7,
				csi_tmg_cfg->vref << 2 |	/*[2]*/
				csi_tmg_cfg->href << 1 |	/*[1]*/
				(csi_tmg_cfg->sample == CLK_POL));/*[0]*/
}

void csi_fmt_cfg(unsigned int sel, unsigned int ch,
		struct csi_fmt_cfg *csi_fmt_cfg)
{
	vfe_reg_clr_set(csi_base_addr[sel] + CSI_REG_CONF,
				0x7 << 20 | 0xf << 16 | 0x3 << 10 | 0x3 << 8,
				csi_fmt_cfg->input_fmt << 20 |	/*[22:20]*/
				csi_fmt_cfg->output_fmt << 16 |	/*[19:16]*/
				csi_fmt_cfg->field_sel << 10 |	/*[11:10]*/
				csi_fmt_cfg->input_seq << 8);	/*[9:8]*/
	input_fmt = csi_fmt_cfg->input_fmt;
}


/* buffer */
void csi_set_buffer_address(unsigned int sel, unsigned int ch,
			enum csi_buf_sel buf, u64 addr)
{
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_BUF_0_A + ch * CSI_CH_OFF + (buf << 2), addr);
}

u64 csi_get_buffer_address(unsigned int sel, unsigned int ch,
			enum csi_buf_sel buf)
{
	u32 t;

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_BUF_0_A + ch * CSI_CH_OFF + (buf << 2));
	return t;
}

/* capture */
void csi_capture_start(unsigned int sel, unsigned int ch_total_num,
		enum csi_cap_mode csi_cap_mode)
{
	if (csi_cap_mode == CSI_VCAP)
		vfe_reg_set(csi_base_addr[sel] + CSI_REG_CTRL, 0X1 << 1);
	else
		vfe_reg_set(csi_base_addr[sel] + CSI_REG_CTRL, 0X1 << 0);
}

void csi_capture_stop(unsigned int sel, unsigned int ch_total_num,
		enum csi_cap_mode csi_cap_mode)
{
	vfe_reg_clr(csi_base_addr[sel] + CSI_REG_CTRL, 0X3);
}

void csi_capture_get_status(unsigned int sel, unsigned int ch,
		struct csi_capture_status *status)
{
	u32 t;

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_STATUS + ch * CSI_CH_OFF);
	status->picture_in_progress = t & 0x1;
	status->video_in_progress = (t >> 1) & 0x1;
}

/* size */
void csi_set_size(unsigned int sel, unsigned int ch, unsigned int length_h,
		unsigned int length_v, unsigned int buf_length_h,
		unsigned int buf_length_c)
{
	u32 t;

	switch (input_fmt) {
	case CSI_CCIR656:
	case CSI_CCIR656_2CH:
	case CSI_CCIR656_4CH:
	case CSI_YUV422_16:
	case CSI_YUV422:
		length_h = length_h*2;
		break;
	default:
		break;
	}

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_RESIZE_H + ch * CSI_CH_OFF);
	t = (t & 0x0000ffff) | (length_h << 16);
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_RESIZE_H + ch * CSI_CH_OFF, t);

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_RESIZE_V + ch * CSI_CH_OFF);
	t = (t & 0x0000ffff) | (length_v << 16);
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_RESIZE_V + ch * CSI_CH_OFF, t);

	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_BUF_LENGTH + ch * CSI_CH_OFF, buf_length_h);

}

/* offset */
void csi_set_offset(unsigned int sel, unsigned int ch, unsigned int start_h, unsigned int start_v)
{
	u32 t;

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_RESIZE_H  + ch * CSI_CH_OFF);
	t = (t & 0xffff0000) | start_h;
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_RESIZE_H + ch * CSI_CH_OFF, t);

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_RESIZE_V + ch * CSI_CH_OFF);
	t = (t & 0xffff0000) | start_v;
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_RESIZE_V + ch * CSI_CH_OFF, t);
}

/* interrupt */
void csi_int_enable(unsigned int sel, unsigned int ch, enum csi_int_sel interrupt)
{
	vfe_reg_set(csi_base_addr[sel] + CSI_REG_INT_EN + ch * CSI_CH_OFF, interrupt);
}

void csi_int_disable(unsigned int sel, unsigned int ch, enum csi_int_sel interrupt)
{
	vfe_reg_clr(csi_base_addr[sel] + CSI_REG_INT_EN + ch * CSI_CH_OFF, interrupt);
}

inline void csi_int_get_status(unsigned int sel, unsigned int ch, struct csi_int_status *status)
{
	u32 t;

	t = vfe_reg_readl(csi_base_addr[sel] + CSI_REG_INT_STATUS + ch * CSI_CH_OFF);

	status->capture_done     = t & CSI_INT_CAPTURE_DONE;
	status->frame_done       = t & CSI_INT_FRAME_DONE;
	status->buf_0_overflow   = t & CSI_INT_BUF_0_OVERFLOW;
	status->buf_1_overflow   = t & CSI_INT_BUF_1_OVERFLOW;
	status->buf_2_overflow   = t & CSI_INT_BUF_2_OVERFLOW;
	status->protection_error = t & CSI_INT_PROTECTION_ERROR;
	status->hblank_overflow  = t & CSI_INT_HBLANK_OVERFLOW;
	status->vsync_trig	 = t & CSI_INT_VSYNC_TRIG;
}

inline void csi_int_clear_status(unsigned int sel, unsigned int ch, enum csi_int_sel interrupt)
{
	vfe_reg_writel(csi_base_addr[sel] + CSI_REG_INT_STATUS + ch * CSI_CH_OFF, interrupt);
}
