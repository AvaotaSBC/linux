/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2023 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/mm.h>
#include <linux/delay.h>
#include "de_top.h"
#include "sunxi-sid.h"

#define DE_REG_OFFSET(base, index, size)	((base) + (index) * (size))

#define DE_RESET_OFFSET				(0x8000)
#define DE_CLK_OFFSET				(0x8004)
#define DE_MBUS_CLOCK_OFFSET			(0x8008)
#define DE2TCON_MUX_OFFSET			(0x8010)
#define DE_VER_CTL_OFFSET			(0x8014)
#define DE_RTWB_CTL_OFFSET			(0x8020)
#define DE_RTWB_MUX_OFFSET_V1			(0x8020)
#define DE_CH2CORE_MUX_OFFSET			(0x8024)
#define DE_PORT2CHN_MUX_OFFSET_V1		(0x8028)
#define DE_ASYNC_BRIDGE_OFFSET			(0x804c)
#define DE_BUF_DEPTH_OFFSET			(0x8050)
#define DE_DEBUG_CTL_OFFSET			(0x80E0)
#define RTMX_GLB_CTL_OFFSET			(0x8100)
#define RTMX_GLB_STS_OFFSET			(0x8104)
#define RTMX_OUT_SIZE_OFFSET			(0x8108)
#define RTMX_AUTO_CLK_OFFSET			(0x810C)
#define RTMX_RCQ_CTL_OFFSET			(0x8110)
#define RTMX_RCQ_HEADER_LADDR_OFFSET		(0x8114)

#define DE_RESERVE_CTL_OFFSET			(0x800C)
#define DE_RTWB_MUX_OFFSET_V2			(0x8024)
#define DE_VCH2CORE_MUX_OFFSET			(0x8028)
#define DE_UCH2CORE_MUX_OFFSET			(0x802C)
#define DE_PORT2CHN_MUX_OFFSET_V2		(0x8030)
#define DE_OFFLINE_CTL_OFFSET			(0x80D0)
#define DE_OFFLINE_OUTPUT_OFFSET		(0x80D4)
#define DE_OFFLINE_HIGH_ADDR_OFFSET		(0x80D8)
#define DE_OFFLINE_LOW_ADDR_OFFSET		(0x80DC)
#define DE_AHB_TIMEOUT_OFFSET			(0x80E4)
#define DE_GATING_CTL_OFFSET			(0x80E8)

#define DE_RESET_OFFSET_V21X			(0x10)
#define DE_CLK_OFFSET_V21X			(0x14)
#define DE_MBUS_CLOCK_OFFSET_V21X		(0x18)
#define RTMX_AUTO_CLK_OFFSET_V21X		(0x1C)
#define DE2TCON_MUX_OFFSET_V21X			(0x28)
#define RTMX_GLB_CTL_OFFSET_V21X		(0x100)
#define RTMX_GLB_STS_OFFSET_V21X		(0x104)
#define RTMX_OUT_SIZE_OFFSET_V21X		(0x108)
#define RTMX_RCQ_CTL_OFFSET_V21X		(0x110)
#define RTMX_RCQ_STA_OFFSET_V21X		(0x114)
#define RTMX_RCQ_HEADER_LADDR_OFFSET_V21X	(0x118)

#define DE_SCLK_GATING_OFFSET			(0x0)
#define DE_HCLK_GATING_OFFSET			(0x4)
#define DE_RESET_OFFSET_V200			(0x8)

/* global control for de200 */
#define DE_MOD_GLB_OFFSET			(0x100000)
#define DE_GLB_CTL_OFFSET			(0x0)
#define DE_GLB_DOUBLE_BUFFER_CTL_OFFSET		(0x8)
#define DE_GLB_SIZE_OFFSET			(0xC)

struct de_top_desc {
	u32 version;
	/* feature info */
	u8 support_offline;
	u8 support_channel_mux;
	u8 support_rcq_fifo;
	u8 support_mbus_reset;
	u8 support_channel_clk;
	u8 support_rcq_gate;
	/* register info
	   size: differnent reg offset
	   shift: reg bit shift */
	u32 de_reset_offset;
	u32 de_clk_offset;
	u32 disp_clk_shift;
	u32 mbus_reset_offset;
	u32 mbus_clk_offset;
	u32 glb_ctl_offset;
	u32 auto_clk_offset;
	u32 auto_clk_disp_shift;
	u32 auto_clk_disp_size;
	u32 out_size_offset;
	u32 de2tcon_mux_offset;
	u32 rcq_ctl_offset;
	u32 rcq_header_laddr_offset;
	u32 rtwb_mux_offset;
	u32 rtwb_timing_shift;
	u32 rtwb_self_wb_start_shift;
	u32 port2chn_mux_offset;
	u32 channel_clk_offset;
	u32 reserve_ctl_offset;
	u32 busy_offset;
	/* compatible ops */
	enum de_irq_state (*query_state_with_clear)(struct de_top_handle *hdl,
			    u32 disp, enum de_irq_state irq_state);
	int (*display_config)(struct de_top_handle *hdl,
			    const struct de_top_display_cfg *cfg);
	bool (*check_finish)(struct de_top_handle *hdl, u32 disp);
	int (*set_chn2core_mux)(struct de_top_handle *hdl, u32 disp,
				u32 chn_type_id, bool is_video);

};

enum de_irq_state {
	DE_IRQ_STATE_FRAME_END  = 0x1 << 0,
	DE_IRQ_STATE_ERROR      = 0,
	DE_IRQ_STATE_RCQ_FINISH = 0x1 << 2,
	DE_IRQ_STATE_RCQ_ACCEPT = 0x1 << 3,
	DE_IRQ_STATE_MASK =
		DE_IRQ_STATE_FRAME_END
		| DE_IRQ_STATE_ERROR
		| DE_IRQ_STATE_RCQ_FINISH
		| DE_IRQ_STATE_RCQ_ACCEPT,
};

enum de_irq_state_v21x {
	DE_IRQ_STATE_RCQ_FINISH_V21X = 0x1 << 0,
	DE_IRQ_STATE_RCQ_ACCEPT_V21X = 0x1 << 4,
	DE_IRQ_STATE_MASK_V21X = DE_IRQ_STATE_RCQ_FINISH_V21X |
		    DE_IRQ_STATE_RCQ_ACCEPT_V21X,
};

enum de_irq_flag {
	DE_IRQ_FLAG_FRAME_END  = 0x1 << 4,
	DE_IRQ_FLAG_ERROR      = 0,
	DE_IRQ_FLAG_RCQ_FINISH = 0x1 << 6,
	DE_IRQ_FLAG_RCQ_ACCEPT = 0x1 << 7,
	DE_IRQ_FLAG_MASK =
		DE_IRQ_FLAG_FRAME_END
		| DE_IRQ_FLAG_ERROR
		| DE_IRQ_FLAG_RCQ_FINISH
		| DE_IRQ_FLAG_RCQ_ACCEPT,
};

enum de_clk_id {
	DE_CLK_DISP0 = 0,
	DE_CLK_DISP1 = 1,
	DE_CLK_DISP2 = 2,
	DE_CLK_DISP3 = 3,
	DE_CLK_WB = 4,
	DE_CLK_NONE = 255,
};

struct offline_mode_status {
	unsigned int alloc_w;
	unsigned int alloc_h;
	unsigned long buff_size;
	dma_addr_t phy_addr;
	void *virt_addr;
};

struct de_top_private {
	enum de_rtwb_mode rtwb_mode;
	atomic_t mbus_ref_count;
	const struct de_top_desc *dsc;
	struct offline_mode_status offline;
};

extern unsigned int sunxi_get_soc_ver(void);
static enum de_irq_state de_top_query_state_with_clear_v3xx(struct de_top_handle *hdl, u32 disp, enum de_irq_state irq_state);
static enum de_irq_state de_top_query_state_with_clear_v21x(struct de_top_handle *hdl, u32 disp, enum de_irq_state irq_state);
static bool de_top_check_display_rcq_update_finish_with_clear(struct de_top_handle *hdl, u32 disp);
static bool de_top_check_display_double_buffer_update_finish(struct de_top_handle *hdl, u32 disp);
static int de_top_display_config_v2(struct de_top_handle *hdl, const struct de_top_display_cfg *cfg);
static int de_top_display_config_v1(struct de_top_handle *hdl, const struct de_top_display_cfg *cfg);
static int de_top_set_chn2core_mux_v2(struct de_top_handle *hdl, u32 disp, u32 chn_type_id, bool is_video);
static int de_top_set_chn2core_mux_v1(struct de_top_handle *hdl, u32 disp, u32 chn_type_id, bool is_video);

static struct de_top_desc de350 = {
	.version = 0x350,
	.support_channel_mux = 1,

	.de_reset_offset = DE_RESET_OFFSET,
	.de_clk_offset = DE_CLK_OFFSET,
	.disp_clk_shift = 1,
	.mbus_clk_offset = DE_MBUS_CLOCK_OFFSET,
	.glb_ctl_offset = RTMX_GLB_CTL_OFFSET,
	.auto_clk_offset = RTMX_AUTO_CLK_OFFSET,
	.auto_clk_disp_shift = 0,
	.auto_clk_disp_size = 0x40,
	.out_size_offset = RTMX_OUT_SIZE_OFFSET,
	.de2tcon_mux_offset = DE2TCON_MUX_OFFSET,
	.rcq_ctl_offset = RTMX_RCQ_CTL_OFFSET,
	.rcq_header_laddr_offset = RTMX_RCQ_HEADER_LADDR_OFFSET,
	.rtwb_mux_offset = DE_RTWB_MUX_OFFSET_V1,
	.rtwb_timing_shift = 5,
	.rtwb_self_wb_start_shift = 4,
	.port2chn_mux_offset = DE_PORT2CHN_MUX_OFFSET_V1,
	.busy_offset = RTMX_GLB_STS_OFFSET,
	.query_state_with_clear = de_top_query_state_with_clear_v3xx,
	.display_config = de_top_display_config_v2,
	.check_finish = de_top_check_display_rcq_update_finish_with_clear,
	.set_chn2core_mux = de_top_set_chn2core_mux_v1,
};

static struct de_top_desc de355 = {
	.version = 0x355,
	.support_offline = 1 << ONE_FRAME_DELAY | 1 << CURRENT_FRAME,
	.support_channel_mux = 1,
	.support_rcq_fifo = 1,
	.support_mbus_reset = 1,
	.support_channel_clk = 1,
	.de_reset_offset = DE_RESET_OFFSET,
	.de_clk_offset = DE_CLK_OFFSET,
	.disp_clk_shift = 4,
	.mbus_reset_offset = 4,
	.mbus_clk_offset = DE_MBUS_CLOCK_OFFSET,
	.glb_ctl_offset = RTMX_GLB_CTL_OFFSET,
	.auto_clk_offset = RTMX_AUTO_CLK_OFFSET,
	.auto_clk_disp_shift = 0,
	.auto_clk_disp_size = 0x40,
	.out_size_offset = RTMX_OUT_SIZE_OFFSET,
	.de2tcon_mux_offset = DE2TCON_MUX_OFFSET,
	.rcq_ctl_offset = RTMX_RCQ_CTL_OFFSET,
	.rcq_header_laddr_offset = RTMX_RCQ_HEADER_LADDR_OFFSET,
	.rtwb_mux_offset = DE_RTWB_MUX_OFFSET_V2,
	.rtwb_timing_shift = 4,
	.rtwb_self_wb_start_shift = 0,
	.port2chn_mux_offset = DE_PORT2CHN_MUX_OFFSET_V2,
	.channel_clk_offset = DE_GATING_CTL_OFFSET,
	.reserve_ctl_offset = DE_RESERVE_CTL_OFFSET,
	.busy_offset = RTMX_GLB_STS_OFFSET,
	.query_state_with_clear = de_top_query_state_with_clear_v3xx,
	.display_config = de_top_display_config_v2,
	.check_finish = de_top_check_display_rcq_update_finish_with_clear,
	.set_chn2core_mux = de_top_set_chn2core_mux_v2,
};

static struct de_top_desc de352 = {
	.version = 0x352,
	.support_offline = 1 << ONE_FRAME_DELAY | 1 << CURRENT_FRAME,
	.support_channel_mux = 1,
	.support_mbus_reset = 1,
	.support_channel_clk = 1,
	.de_reset_offset = DE_RESET_OFFSET,
	.de_clk_offset = DE_CLK_OFFSET,
	.disp_clk_shift = 4,
	.mbus_reset_offset = 4,
	.mbus_clk_offset = DE_MBUS_CLOCK_OFFSET,
	.glb_ctl_offset = RTMX_GLB_CTL_OFFSET,
	.auto_clk_offset = RTMX_AUTO_CLK_OFFSET,
	.auto_clk_disp_shift = 0,
	.auto_clk_disp_size = 0x40,
	.out_size_offset = RTMX_OUT_SIZE_OFFSET,
	.de2tcon_mux_offset = DE2TCON_MUX_OFFSET,
	.rcq_ctl_offset = RTMX_RCQ_CTL_OFFSET,
	.rcq_header_laddr_offset = RTMX_RCQ_HEADER_LADDR_OFFSET,
	.rtwb_mux_offset = DE_RTWB_MUX_OFFSET_V2,
	.rtwb_timing_shift = 4,
	.rtwb_self_wb_start_shift = 0,
	.port2chn_mux_offset = DE_PORT2CHN_MUX_OFFSET_V2,
	.channel_clk_offset = DE_GATING_CTL_OFFSET,
	.reserve_ctl_offset = DE_RESERVE_CTL_OFFSET,
	.busy_offset = RTMX_GLB_STS_OFFSET,
	.query_state_with_clear = de_top_query_state_with_clear_v3xx,
	.display_config = de_top_display_config_v2,
	.check_finish = de_top_check_display_rcq_update_finish_with_clear,
	.set_chn2core_mux = de_top_set_chn2core_mux_v2,
};

static struct de_top_desc de210 = {
	.version = 0x210,
	.support_rcq_gate = 1,
	.de_reset_offset = DE_RESET_OFFSET_V21X,
	.de_clk_offset = DE_CLK_OFFSET_V21X,
	.disp_clk_shift = 4,
	.mbus_clk_offset = DE_MBUS_CLOCK_OFFSET_V21X,
	.glb_ctl_offset = RTMX_GLB_CTL_OFFSET_V21X,
	.auto_clk_offset = RTMX_AUTO_CLK_OFFSET_V21X,
	.auto_clk_disp_shift = 4,
	.auto_clk_disp_size = 0,
	.out_size_offset = RTMX_OUT_SIZE_OFFSET_V21X,
	.de2tcon_mux_offset = DE2TCON_MUX_OFFSET_V21X,
	.rcq_ctl_offset = RTMX_RCQ_CTL_OFFSET_V21X,
	.rcq_header_laddr_offset = RTMX_RCQ_HEADER_LADDR_OFFSET_V21X,
	.query_state_with_clear = de_top_query_state_with_clear_v21x,
	.display_config = de_top_display_config_v2,
	.check_finish = de_top_check_display_rcq_update_finish_with_clear,
//TODO add wb
};

static struct de_top_desc de201 = {
	.version = 0x201,
	.display_config = de_top_display_config_v1,
	.check_finish = de_top_check_display_double_buffer_update_finish,
};

static struct de_top_desc *de_version[] = {
	&de350, &de352, &de355, &de210, &de201,
};

const struct de_top_desc *get_de_top_desc(const struct module_create_info *info)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(de_version); i++) {
		if (de_version[i]->version == info->de_version) {
			return de_version[i];
		}
	}
	return NULL;
}

static inline void __de_mbus_clk_enable(struct de_top_handle *hdl, bool en)
{
	u8 __iomem *reg_base = hdl->cinfo.de_reg_base;
	u32 reg_val;

	reg_base += hdl->private->dsc->mbus_clk_offset;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(0, 1, reg_val, en ? 1 : 0);
	writel(reg_val, reg_base);
}

static inline void __de_mbus_reset(struct de_top_handle *hdl, bool reset)
{
	u8 __iomem *reg_base = hdl->cinfo.de_reg_base + DE_MBUS_CLOCK_OFFSET;
	u32 reg_val;
	u32 shift = hdl->private->dsc->mbus_reset_offset;

	if (hdl->private->dsc->support_mbus_reset) {
		reg_val = readl(reg_base);
		reg_val = SET_BITS(shift, 1, reg_val, reset ? 0 : 1);
		writel(reg_val, reg_base);
	}
}

static int __de_mod_clk_enable(struct de_top_handle *hdl, enum de_clk_id clk_no, bool en)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base;
	u32 val = en ? 0x1 : 0x0;
	u32 reg_val;
	u32 size = hdl->private->dsc->disp_clk_shift;
	u32 shift = size * clk_no;

	reg_base = de_base + hdl->private->dsc->de_clk_offset;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(shift, 1, reg_val, val);
	writel(reg_val | 0x10000, reg_base);

	reg_base = de_base + hdl->private->dsc->de_reset_offset;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(shift, 1, reg_val, val);
	writel(reg_val | 0x10000, reg_base);

	return 0;
}

static int de_top_set_clk_enable(struct de_top_handle *hdl, enum de_clk_id clk_no, u8 en)
{
	s32 count = 0;

	__de_mod_clk_enable(hdl, clk_no, en);

	if (en)
		count = atomic_inc_return(&hdl->private->mbus_ref_count);
	else
		count = atomic_dec_return(&hdl->private->mbus_ref_count);

	if (count > 0) {
		__de_mbus_reset(hdl, 0);
		__de_mbus_clk_enable(hdl, 1);
	} else if (!count) {
		__de_mbus_reset(hdl, 1);
		__de_mbus_clk_enable(hdl, 0);
	} else {
		DRM_ERROR("invalid clk ref cnt\n");
	}

	return 0;
}

static int de_top_channel_clk_enable(struct de_top_handle *hdl, u32 chn_type_id, bool is_video)
{
	u8 __iomem *reg_base;
	u32 reg_val;
	u32 width = 1;
	u32 shift = chn_type_id + (is_video ? 0 : 8);

	if (hdl->private->dsc->support_channel_clk) {
		reg_base = hdl->cinfo.de_reg_base + hdl->private->dsc->channel_clk_offset;
		reg_val = readl(reg_base);
		reg_val = SET_BITS(shift, width, reg_val, 1);
		writel(reg_val, reg_base);
	}
	return 0;
}

static int de_top_set_de2tcon_mux(struct de_top_handle *hdl, u32 disp, u32 tcon)
{
	u8 __iomem *reg_base;
	const u32 max_disp = 3;
	u32 reg_val;
	u32 width = 4;
	u32 shift = disp << 2;
	u32 i;

	reg_base = hdl->cinfo.de_reg_base + hdl->private->dsc->de2tcon_mux_offset;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(shift, width, reg_val, tcon);

	/* set conflict tcon mux to 0xf */
	for (i = 0; i <= max_disp; i++) {
		if (i == disp)
			continue;
		shift = i << 2;
		if (GET_BITS(shift, 4, reg_val) == tcon) {
			reg_val = SET_BITS(shift, 4, reg_val, 0xf);
		}
	}
	writel(reg_val, reg_base);

	return 0;
}

static int de_top_set_chn2core_mux_v1(struct de_top_handle *hdl, u32 disp,
	u32 chn_type_id, bool is_video)
{
	u8 __iomem *reg_base = hdl->cinfo.de_reg_base + DE_CH2CORE_MUX_OFFSET;
	u32 reg_val;
	reg_val = readl(reg_base);
	chn_type_id += is_video ? 0 : 8;
	reg_val = SET_BITS(chn_type_id << 1, 2, reg_val, disp);
	writel(reg_val, reg_base);
	return 0;
}

static int de_top_set_chn2core_mux_v2(struct de_top_handle *hdl, u32 disp,
	u32 chn_type_id, bool is_video)
{
	u8 __iomem *reg_base = hdl->cinfo.de_reg_base;
	u32 reg_val;
	reg_base += is_video ? DE_VCH2CORE_MUX_OFFSET : DE_UCH2CORE_MUX_OFFSET;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(chn_type_id << 2, 4, reg_val, disp);
	writel(reg_val, reg_base);
	return 0;
}

static int de_top_set_port2chn_mux(struct de_top_handle *hdl, u32 disp, u32 port, u32 chn_type_id, bool is_video)
{
	u8 __iomem *reg_base = hdl->cinfo.de_reg_base + hdl->private->dsc->port2chn_mux_offset;
	u32 reg_val;

	reg_base = DE_REG_OFFSET(reg_base, disp, 4);
	reg_val = readl(reg_base);
	chn_type_id += is_video ? 0 : 8;
	reg_val = SET_BITS(port << 2, 4, reg_val, chn_type_id);
	writel(reg_val, reg_base);
	return 0;
}

int de_top_set_chn_mux(struct de_top_handle *hdl, u32 disp, u32 port, u32 chn_type_id, bool is_video)
{
	if (!hdl->private->dsc->support_channel_mux)
		return 0;

	de_top_channel_clk_enable(hdl, chn_type_id, is_video);
	de_top_set_port2chn_mux(hdl, disp, port, chn_type_id, is_video);
	return hdl->private->dsc->set_chn2core_mux(hdl, disp, chn_type_id, is_video);
}

static int de_top_set_rtmx_enable(struct de_top_handle *hdl, u32 disp, u8 en)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base;
	u32 reg_val;
	unsigned int ic_ver = 0;
	u32 offset;
	u32 size;
	u32 shift;

	if (hdl->private->dsc->version == 0x350) {
		ic_ver = sunxi_get_soc_ver();
		if (ic_ver < 2)
			writel(0x10, de_base + DE_ASYNC_BRIDGE_OFFSET);
		else
			writel(0x0, de_base + DE_ASYNC_BRIDGE_OFFSET);
		if (disp == 0)
			writel(0x6000, de_base + DE_REG_OFFSET(DE_BUF_DEPTH_OFFSET, disp, 0x4));
	} else if (hdl->private->dsc->version == 0x352) {
		/*
		 * sun60iw2 all linebufs have 0x3000, and the maximum size of a single de is 0x2000.
		 * here, de0 is configured as 0x2000, and de1 default is 0x800.
		 */
		if (disp == 0)
			writel(0x2000, de_base + DE_REG_OFFSET(DE_BUF_DEPTH_OFFSET, disp, 0x4));
	}
/*
	if (hdl->private->dsc->reserve_ctl_offset) {
		reg_base = de_base + DE_RESERVE_CTL_OFFSET;
		reg_val = readl(reg_base);
		reg_val = SET_BITS(8, 1, reg_val, 1);
		writel(reg_val, reg_base);
	}*/

	offset = hdl->private->dsc->glb_ctl_offset;
	reg_base = de_base + DE_REG_OFFSET(offset, disp, 0x40);
	 reg_val = readl(reg_base);
	if (en)
		reg_val |= 0x1;
	else
		reg_val &= ~0x1;
	writel(reg_val, reg_base);

	offset = hdl->private->dsc->auto_clk_offset;
	size = hdl->private->dsc->auto_clk_disp_size;
	shift = hdl->private->dsc->auto_clk_disp_shift * disp;
	reg_base = de_base + DE_REG_OFFSET(offset, disp, size);

	reg_val = readl(reg_base);
	reg_val = SET_BITS(shift, 1, reg_val, en ? 1 : 0);
	if (hdl->private->dsc->support_rcq_gate)
		reg_val = SET_BITS(16, 1, reg_val, en ? 1 : 0);
	writel(reg_val, reg_base);

	return 0;
}

static int de_top_set_out_size(struct de_top_handle *hdl, u32 disp, u32 width, u32 height)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u32 offset = hdl->private->dsc->out_size_offset;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(offset, disp, 0x40);
	u32 reg_val = ((width - 1) & 0x1FFF)
		| (((height - 1) & 0x1FFF) << 16);

	writel(reg_val, reg_base);
	return 0;
}

static int de_top_set_rcq_head(struct de_top_handle *hdl, u32 disp, u64 addr, u32 len)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u32 offset = hdl->private->dsc->rcq_header_laddr_offset;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(offset, disp, 0x40);
	u32 haddr = (u32)(addr >> 32);

	writel((u32)addr, reg_base);
	writel(haddr, reg_base + 0x4);
	writel(len, reg_base + 0x8);
	return 0;
}

int de_top_display_config(struct de_top_handle *hdl, const struct de_top_display_cfg *cfg)
{
	return hdl->private->dsc->display_config(hdl, cfg);
}

static int de_top_display_config_v1(struct de_top_handle *hdl, const struct de_top_display_cfg *cfg)
{
	unsigned int id = cfg->display_id;
	u8 __iomem *offset;
	u32 reg_val;

	/* clk & reset */
	offset = hdl->cinfo.de_reg_base + DE_SCLK_GATING_OFFSET;
	reg_val = readl(offset);
	writel(reg_val | 1 << id, offset);
	offset = hdl->cinfo.de_reg_base + DE_HCLK_GATING_OFFSET;
	reg_val = readl(offset);
	writel(reg_val | 1 << id, offset);
	offset = hdl->cinfo.de_reg_base + DE_RESET_OFFSET_V200;
	reg_val = readl(offset);
	writel(reg_val | 1 << id, offset);

	/* size */
	offset = hdl->cinfo.de_reg_base + DE_GLB_SIZE_OFFSET + DE_MOD_GLB_OFFSET;
	reg_val = ((cfg->w - 1) & 0x1FFF) |
		    (((cfg->h - 1) & 0x1FFF) << 16);
	writel(reg_val, offset);

	/* enable */
	offset = hdl->cinfo.de_reg_base + DE_GLB_CTL_OFFSET + DE_MOD_GLB_OFFSET;
	reg_val = readl(offset);
	writel(reg_val | 1, offset);
	return 0;
}

int de_top_set_double_buffer_ready(struct de_top_handle *hdl, u32 disp)
{
	u8 __iomem *offset = hdl->cinfo.de_reg_base;

	offset += DE_GLB_DOUBLE_BUFFER_CTL_OFFSET + DE_MOD_GLB_OFFSET;
	writel(1, offset);
	return 0;
}

static int de_top_display_config_v2(struct de_top_handle *hdl, const struct de_top_display_cfg *cfg)
{
	unsigned int id = cfg->display_id;
	de_top_set_clk_enable(hdl, DE_CLK_DISP0 + id, cfg->enable);
	de_top_set_rtmx_enable(hdl, id, cfg->enable);
	if (!cfg->enable)
		return 0;
	de_top_set_out_size(hdl, id, cfg->w, cfg->h);
	de_top_set_de2tcon_mux(hdl, id, cfg->device_index);
	de_top_set_rcq_head(hdl, id, cfg->rcq_header_addr, cfg->rcq_header_byte);
	return 0;
}

int de_top_get_out_size(struct de_top_handle *hdl, u32 disp, u32 *width, u32 *height)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(RTMX_OUT_SIZE_OFFSET, disp, 0x40);
	u32 reg_val = readl(reg_base);

	if (width)
		*width = (reg_val & 0x1FFF) + 1;
	if (height)
		*height = ((reg_val >> 16) & 0x1FFF) + 1;
	return 0;
}

int de_top_wb_config(struct de_top_handle *hdl, const struct de_top_wb_cfg *cfg)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base_mux = de_base + hdl->private->dsc->rtwb_mux_offset;
	u8 __iomem *reg_base_ctl = de_base + DE_RTWB_CTL_OFFSET;
	u32 reg_val;
	u32 mux = (u32)(cfg->disp * 2);
	u32 shift;

	de_top_set_clk_enable(hdl, DE_CLK_WB, cfg->enable);
	if (!cfg->enable)
		return 0;

	if (cfg->disp > 3) {
		DRM_ERROR("wb invalid disp\n");
		return -1;
	}

	shift = hdl->private->dsc->rtwb_timing_shift;
	reg_val = readl(reg_base_ctl);
	if (cfg->mode == SELF_GENERATED_TIMING) {
		reg_val = SET_BITS(shift, 1, reg_val, 1);
		writel(reg_val, reg_base_ctl);
	}

	shift = 0;
	reg_val = readl(reg_base_mux);
	if (cfg->pos == FROM_DISP)
		mux += 1;
	reg_val = SET_BITS(shift, 3, reg_val, mux);
	writel(reg_val, reg_base_mux);
	return 0;
}

static s32 de_top_pingpang_buf_free(struct de_top_handle *hdl)
{
	struct offline_mode_status *st = &hdl->private->offline;
	if (st->virt_addr && st->phy_addr) {
		sunxi_de_dma_free_coherent(hdl->cinfo.de, st->buff_size,
						st->phy_addr, st->virt_addr);
	}
	return 0;
}

static s32 de_top_pingpang_buf_alloc(struct de_top_handle *hdl, unsigned int width, unsigned int height)
{
	struct offline_mode_status *st = &hdl->private->offline;

	st->alloc_w = width;
	st->alloc_h = height;
	/* 3 is enough for rgb data, 4 for maybe 10bit data */
	st->buff_size = PAGE_ALIGN((width * height * 4 * 2));
	st->virt_addr = sunxi_de_dma_alloc_coherent(hdl->cinfo.de,
		st->buff_size, (void *)&(st->phy_addr));

	if (!st->virt_addr || !st->phy_addr) {
		DRM_ERROR("pingpang buf alloc err");
		return -1;
	}
	return 0;
}

static s32 de_top_set_offline_enable(struct de_top_handle *hdl, bool enable, enum de_offline_mode mode)
{
	struct offline_mode_status *st = &hdl->private->offline;
	u8 __iomem *reg_base;
	u32 reg_val;

	reg_base = hdl->cinfo.de_reg_base + DE_OFFLINE_LOW_ADDR_OFFSET;
	writel((u32)(st->phy_addr), reg_base);
	reg_base = hdl->cinfo.de_reg_base + DE_OFFLINE_HIGH_ADDR_OFFSET;
	writel((u32)(((u64)(st->phy_addr)) >> 32), reg_base);

	reg_base = hdl->cinfo.de_reg_base + DE_OFFLINE_CTL_OFFSET;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(0, 1, reg_val, enable);
	reg_val = SET_BITS(8, 1, reg_val, mode);
	reg_val = SET_BITS(4, 1, reg_val, 1);
	writel(reg_val, reg_base);
	return 0;
}

s32 de_top_offline_mode_config(struct de_top_handle *hdl, struct offline_cfg *cfg)
{
	struct offline_mode_status *st = &hdl->private->offline;
	if (hdl->private->dsc->support_offline) {
		if (cfg->enable && st->alloc_w * st->alloc_h < cfg->w * cfg->h) {
			if (st->phy_addr)
				de_top_pingpang_buf_free(hdl);
			de_top_pingpang_buf_alloc(hdl, cfg->w, cfg->h);
		}
		return de_top_set_offline_enable(hdl, cfg->enable, cfg->mode);
	} else if (cfg->enable) {
		DRM_ERROR("not support offline mode %d\n", cfg->mode);
		return -1;
	}

	return 0;
}

int de_top_enable_irq(struct de_top_handle *hdl, u32 disp, u32 irq_flag, u32 en)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(RTMX_GLB_CTL_OFFSET, disp, 0x40);
	u32 reg_val = readl(reg_base);

	if (en)
		reg_val |= irq_flag;
	else
		reg_val &= ~irq_flag;
	writel(reg_val, reg_base);
	return 0;
}

static unsigned int de_top_query_state_with_clear(struct de_top_handle *hdl, u32 disp, u32 irq_state)
{
	if (hdl->private->dsc->query_state_with_clear)
		return hdl->private->dsc->query_state_with_clear(hdl, disp, irq_state);
	else
		DRM_ERROR("query_state_with_clear must be set\n");
	return -1;
}

static enum de_irq_state de_top_query_state_with_clear_v3xx(struct de_top_handle *hdl, u32 disp, enum de_irq_state irq_state)
{
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(RTMX_GLB_STS_OFFSET, disp, 0x40);
	u32 reg_val = readl(reg_base);
	u32 state = reg_val & irq_state & DE_IRQ_STATE_MASK;

	reg_val &= ~DE_IRQ_STATE_MASK;
	reg_val |= state;
	writel(reg_val, reg_base); /* w1c */

	return state;
}

static enum de_irq_state de_top_query_state_with_clear_v21x(struct de_top_handle *hdl, u32 disp, enum de_irq_state irq_state)
{
	u32 reg_val;
	u32 ret = 0;
	u32 w1c = 0;
	bool check_finish = irq_state & DE_IRQ_STATE_RCQ_FINISH;
	bool check_accept = irq_state & DE_IRQ_STATE_RCQ_ACCEPT;
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(RTMX_RCQ_STA_OFFSET_V21X, disp, 0x40);

	/* not support other state flag */
	if (!check_finish && !check_accept)
		return 0;
	reg_val = readl(reg_base);
	if (check_finish && reg_val & DE_IRQ_STATE_RCQ_FINISH_V21X) {
		ret |= DE_IRQ_STATE_RCQ_FINISH;
		w1c |= DE_IRQ_STATE_RCQ_FINISH_V21X;
	}
	if (check_accept && reg_val & DE_IRQ_STATE_RCQ_FINISH) {
		ret |= DE_IRQ_STATE_RCQ_FINISH;
		w1c |= DE_IRQ_STATE_RCQ_ACCEPT_V21X;
	}
	/* be careful, v210 need writel 1 and then write 0 to clear state flag */
	writel(w1c, reg_base);
	writel(0, reg_base);
	return ret;
}

bool de_top_check_display_update_finish_with_clear(struct de_top_handle *hdl, u32 disp)
{
	return hdl->private->dsc->check_finish(hdl, disp);
}

bool de_top_check_display_rcq_update_finish_with_clear(struct de_top_handle *hdl, u32 disp)
{
	unsigned int state = de_top_query_state_with_clear(hdl, disp, DE_IRQ_STATE_RCQ_FINISH);
	bool finish = (state & DE_IRQ_STATE_RCQ_FINISH) ? true : false;
	return finish;
}

bool de_top_check_display_double_buffer_update_finish(struct de_top_handle *hdl, u32 disp)
{
/*	TODO double buffer ready bit is not readable, should wait vblank irq as finish flag.
	    besides, some platform even do NOT support double buffer actually, we should wait vblank
	    and copy regs using cpu (ahb copy).

	u8 __iomem *offset = hdl->cinfo.de_reg_base + DE_MOD_GLB_OFFSET;
	offset += DE_GLB_DOUBLE_BUFFER_CTL_OFFSET;
	return !readl(offset) & 0x1;*/
	return false;
}

int de_top_set_rcq_update(struct de_top_handle *hdl, u32 disp, bool update)
{
	u32 offset = hdl->private->dsc->rcq_ctl_offset;
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base = de_base + DE_REG_OFFSET(offset, disp, 0x40);
	u32 offset2 = DE_REG_OFFSET(hdl->private->dsc->busy_offset, disp, 0x40);
	u8 __iomem *reg_base2 = de_base + offset2;
	bool busy;
	unsigned long flag;
	int i = 0;

	/* skip busy check if calling from crash dump in interrupt or platform no need busy check */
	if (!hdl->private->dsc->busy_offset || in_interrupt()) {
		writel(update ? 1 : 0, reg_base);
		return 0;
	}

	do {
		local_irq_save(flag);
		busy = !!(readl(reg_base2) & 0x10);
		if (busy) {
			writel(update ? 1 : 0, reg_base);
			local_irq_restore(flag);
			break;
		}
		local_irq_restore(flag);
		usleep_range(5, 10);
		i++;
	} while (i < 10000);

	return 0;
}

int de_top_request_rcq_fifo_update(struct de_top_handle *hdl, u32 disp, unsigned long rcq_header_addr, unsigned int rcq_header_byte)
{
	u32 offset = hdl->private->dsc->rcq_ctl_offset;
	u8 __iomem *de_base = hdl->cinfo.de_reg_base;
	u8 __iomem *reg_base;
	u32 reg_val;
	u32 size0;

	if (!hdl->private->dsc->support_rcq_fifo) {
		DRM_ERROR("not support rcq_fifo\n");
		return -1;
	}

	reg_base = de_base + hdl->private->dsc->reserve_ctl_offset;
	reg_val = readl(reg_base);
	reg_val = SET_BITS(12, 1, reg_val, 1);
	writel(reg_val, reg_base);

	de_top_set_rcq_head(hdl, disp, rcq_header_addr, rcq_header_byte);

	size0 = readl(de_base +  DE_REG_OFFSET(offset, disp, 0x40)) /*& 0xf0000*/;
	if (((size0 & 0xf0000) >> 16) >= 1)
		return de_top_set_rcq_update(hdl, disp, 1);
	else
		return -1;
}

struct de_top_handle *de_top_create(const struct module_create_info *info)
{
	struct de_top_handle *hdl;
	if (info->id > 0)
		return NULL;
	hdl = kmalloc(sizeof(*hdl), GFP_KERNEL | __GFP_ZERO);
	memcpy(&hdl->cinfo, info, sizeof(*info));
	hdl->private = kmalloc(sizeof(*hdl->private), GFP_KERNEL | __GFP_ZERO);
	hdl->private->dsc = get_de_top_desc(info);
	return hdl;
}
