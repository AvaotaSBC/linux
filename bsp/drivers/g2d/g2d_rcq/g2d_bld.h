/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (c) 2007-2019 Allwinnertech Co., Ltd.
 * Author: zhengxiaobin <zhengxiaobin@allwinnertech.com>
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
#ifndef _G2D_BLD_H
#define _G2D_BLD_H
#include "g2d_mixer.h"
#include "g2d_rcq.h"
#include "g2d_mixer_type.h"

struct g2d_mixer_frame;
struct blender_submodule {
	struct g2d_reg_block *reg_blks;
	__u32 reg_blk_num;
	struct g2d_reg_mem_info *reg_info;
	__s32 (*destory)(struct blender_submodule *p_bld);
	__s32 (*rcq_setup)(struct blender_submodule *p_bld, u8 __iomem *base,
		  struct g2d_rcq_mem_info *p_rcq_info);
	__u32 (*get_reg_block_num)(struct blender_submodule *p_bld);
	__u32 (*get_rcq_mem_size)(struct blender_submodule *p_bld);
	__s32 (*get_reg_block)(struct blender_submodule *p_bld, struct g2d_reg_block **blks);
	struct g2d_mixer_bld_reg  *(*get_reg)(struct blender_submodule *p_bld);
	void (*set_block_dirty)(struct blender_submodule *p_bld, __u32 blk_id, __u32 dirty);
};

__s32 bld_in_set(struct blender_submodule *p_bld, __u32 sel, g2d_rect rect,
		 int premul);
__s32 bld_ck_para_set(struct blender_submodule *p_bld, g2d_ck *para,
		      __u32 flag);
__s32 bld_bk_set(struct blender_submodule *p_bld, __u32 color);
__s32 bld_out_setting(struct blender_submodule *p_bld, g2d_image_enh *p_image);
__s32 bld_cs_set(struct blender_submodule *p_bld, __u32 format);
__s32 bld_csc_reg_set(struct blender_submodule *p_bld, __u32 csc_no,
		      g2d_csc_sel csc_sel, enum color_range src_cr, enum color_range dst_cr);
__s32 bld_rop3_set(struct blender_submodule *p_bld, __u32 sel, __u32 rop3_cmd);
__s32 bld_set_rop_ctrl(struct blender_submodule *p_bld, __u32 value);
__s32 bld_rop2_set(struct blender_submodule *p_bld, __u32 rop_cmd);
struct blender_submodule *
g2d_bld_submodule_setup(struct g2d_mixer_frame *p_frame);
__s32 bld_porter_duff(struct blender_submodule *p_bld, __u32 cmd);

#endif /* End of file */
