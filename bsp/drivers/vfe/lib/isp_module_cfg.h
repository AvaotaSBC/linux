/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/lib/isp_module_cfg.h
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
 ***************************************************************************************
 *
 * isp_module_cfg.h
 *
 * Hawkview ISP - isp_module_cfg.h module
 *
 * Copyright (c) 2013 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Version		  Author         Date		    Description
 *
 *   1.0		Yang Feng	2013/11/07	    First Version
 *
 ****************************************************************************************
 */

#ifndef __ISP__MODULE__CFG__H
#define __ISP__MODULE__CFG__H

#include <linux/kernel.h>
#include "bsp_isp_comm.h"

/* For debug */
#define ISP_DGB
/* #define ISP_DGB_FL */

#ifdef ISP_DGB
#define  ISP_DBG(lev, dbg_level, x, arg...)  do { if (lev <= dbg_level) pr_debug("[ISP_DEBUG]"x, ##arg); } while (0)
#else
#define  ISP_DBG(lev, dbg_level, x, arg...)
#endif

#ifdef ISP_DGB_FL
#define  FUNCTION_LOG          do { pr_info("%s, line: %d\n", __func__, __LINE__); } while (0)
#else
#define  FUNCTION_LOG
#endif

#define ISP_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))


/* ISP module config */
/* TABLE */

#define ISP_LUT_TBL_SIZE            256
#define ISP_LENS_TBL_SIZE           256
#define ISP_GAMMA_TBL_SIZE          256
#define ISP_DRC_TBL_SIZE            256

/* sensor */
#define MAX_PIC_RESOLUTION_NUM      10

/*
 *
 *  struct isp_bndf_config - .
 *
 */
struct isp_bndf_config {
	enum isp_bndf_mode bndf_mode;
	struct isp_denoise filter_2d_coef;
};

/*
 *
 *  struct isp_drc_config - .
 *
 */
struct isp_drc_config {
	enum isp_rgb_drc_mode rgb_drc_mode;
	unsigned short drc_table[ISP_DRC_TBL_SIZE];
	unsigned short drc_table_last[ISP_DRC_TBL_SIZE];
};

/*
 *
 *  struct isp_lut_lens_gamma_config - .
 *
 */

struct isp_lut_config {
	int lut_num;
	enum isp_src lut_src;
	enum isp_lut_dpc_mode lut_dpc_mode;
	unsigned char lut_dpc_src0_table[ISP_LUT_TBL_SIZE*4];
	unsigned char lut_dpc_src1_table[ISP_LUT_TBL_SIZE*4];
};

struct isp_lens_config {
	struct isp_lsc_config lsc_cfg;
	unsigned short lens_r_table[ISP_LENS_TBL_SIZE];
	unsigned short lens_g_table[ISP_LENS_TBL_SIZE];
	unsigned short lens_b_table[ISP_LENS_TBL_SIZE];
};

struct isp_gamma_config {
	unsigned char gamma_table[ISP_GAMMA_TBL_SIZE*2];
};


/*
 *
 *  struct isp_sharp_config - .
 *
 */
struct isp_sharp_config {
	int sharp_level;
	int sharp_min_val;
	int sharp_max_val;
};

/*
 *
 *  struct isp_private_contrast_config - .
 *
 */
struct isp_private_contrast_config {
	unsigned char pri_contrast_level;
	unsigned char pri_contrast_min_val;
	unsigned char pri_contrast_max_val;
};

/*
 *
 *  struct isp_ae_config - .
 *
 */
struct isp_ae_config {
	unsigned short ae_low_bri_th;
	unsigned short ae_high_bri_th;
	struct isp_h3a_reg_win ae_reg_win;
};
/*
 *
 *  struct isp_af_config - .
 *
 */
struct isp_af_config {
	unsigned short af_sap_lim;
	struct isp_h3a_reg_win af_reg_win;
};

/*
 *
 *  struct isp_awb_config - .
 *
 */
struct isp_awb_config {
	unsigned short awb_sum_th;
	unsigned short awb_r_sat_lim;
	unsigned short awb_g_sat_lim;
	unsigned short awb_b_sat_lim;
	struct isp_wb_diff_threshold diff_th;

	struct isp_h3a_reg_win awb_reg_win;
};


/*
 *
 *  struct isp_wb_gain_config - .
 *
 */
struct isp_wb_gain_config {
	unsigned int clip_val;
	struct isp_white_balance_gain wb_gain;

};


/*
 *
 *  struct isp_hist_config - .
 *
 */
struct isp_hist_config {
	int hist_threshold;
	enum isp_src hist_src;
	enum isp_hist_mode hist_mode;
	struct isp_h3a_reg_win hist_reg_win;
};

/*
 *
 *  struct isp_hist_config - .
 *
 */
struct isp_cfa_config {
	int min_rgb;
	unsigned int dir_th;
};

/*
 *
 *  struct isp_afs_config - .
 *
 */
struct isp_afs_config {
	unsigned int inc_line;
};

/*
 *
 *  struct isp_rgb2rgb_config - .
 *
 */
struct isp_rgb2rgb_config {
	 struct isp_rgb2rgb_gain_offset color_matrix_default;
	 struct isp_rgb2rgb_gain_offset color_matrix;
};

/*
 *
 *  struct isp_gain_offset_config - .
 *
 */
struct isp_gain_offset_config {
	struct isp_bayer_gain_offset bayer_gain;
	struct isp_yuv_gain_offset yuv_gain;
};

struct isp_otf_dpc_config {
	unsigned int th_slop;
	unsigned int min_th;
	unsigned int max_th;
};

struct isp_sprite_win_config {
	struct isp_size sprite_size;
	struct coor sprite_start;
};
struct isp_cnr_config {
	unsigned short c_offset;
	unsigned short c_noise;
};
struct isp_saturation_config {
	short satu_r;
	short satu_g;
	short satu_b;
	short satu_gain;
};


/*
 *
 *  struct isp_module_config - .
 *
 */
struct isp_module_config {
	unsigned int isp_platform_id;
	unsigned int module_enable_flag;
	unsigned int isp_module_update_flags;

	unsigned int table_update;
	/* AFS config */
	struct isp_afs_config afs_cfg;
	struct isp_cfa_config cfa_cfg;
	struct isp_sharp_config sharp_cfg;
	struct isp_private_contrast_config pri_contrast_cfg;
	struct isp_bndf_config bndf_cfg;

	struct isp_drc_config drc_cfg;
	struct isp_lut_config lut_cfg;
	struct isp_lens_config lens_cfg;
	struct isp_gamma_config gamma_cfg;
	struct isp_disc_config disc_cfg;

	struct isp_rgb2rgb_config rgb2rgb_cfg;
	struct isp_ae_config ae_cfg;
	struct isp_af_config af_cfg;
	struct isp_awb_config awb_cfg;
	struct isp_hist_config hist_cfg;
	struct isp_gain_offset_config gain_offset_cfg;
	struct isp_wb_gain_config wb_gain_cfg;
	struct isp_otf_dpc_config otf_cfg;

	struct isp_sprite_win_config sprite_cfg;

	struct isp_cnr_config cnr_cfg;
	enum   isp_output_speed output_speed;
	struct isp_3d_denoise_config tdf_cfg;
	struct isp_saturation_config satu_cfg;

	/* table addr */
	void *lut_src0_table;
	void *lut_src1_table;
	void *gamma_table;
	void *lens_table;
	void *drc_table;
	void *linear_table;
	void *disc_table;
};


enum isp_features_flags {
	ISP_FEATURES_AFS               = (1 << 0),
	ISP_FEATURES_SAP               = (1 << 1),
	ISP_FEATURES_CONTRAST          = (1 << 2),
	ISP_FEATURES_BDNF              = (1 << 3),
	ISP_FEATURES_RGB_DRC           = (1 << 4),
	ISP_FEATURES_LUT_DPC           = (1 << 5),
	ISP_FEATURES_LSC               = (1 << 6),

	ISP_FEATURES_GAMMA             = (1 << 7),
	ISP_FEATURES_RGB2RGB           = (1 << 8),
	ISP_FEATURES_AE                = (1 << 9),
	ISP_FEATURES_AF                = (1 << 10),
	ISP_FEATURES_AWB               = (1 << 11),
	ISP_FEATURES_HIST              = (1 << 12),
	ISP_FEATURES_BAYER_GAIN_OFFSET = (1 << 13),
	ISP_FEATURES_WB                = (1 << 14),
	ISP_FEATURES_OTF_DPC           = (1 << 15),
	ISP_FEATURES_TG                = (1 << 16),
	ISP_FEATURES_YCbCr_DRC         = (1 << 17),
	ISP_FEATURES_OBC               = (1 << 18),
	ISP_FEATURES_CFA               = (1 << 19),
	ISP_FEATURES_SPRITE            = (1 << 20),
	ISP_FEATURES_YCBCR_GAIN_OFFSET = (1 << 21),
	ISP_FEATURES_OUTPUT_SPEED_CTRL = (1 << 22),
	ISP_FEATURES_3D_DENOISE        = (1 << 23),
	ISP_FEATURES_CNR               = (1 << 24),
	ISP_FEATURES_SATU               = (1 << 25),
	ISP_FEATURES_LINEAR		 = (1 << 26),
	ISP_FEATURES_DISC			 = (1 << 27),
	ISP_FEATURES_MAX,

	/* all possible flags raised */
	ISP_FEATURES_All = (((ISP_FEATURES_MAX - 1) << 1) - 1),
};

void isp_module_platform_init(struct isp_module_config *module_cfg);
void isp_setup_module_hw(struct isp_module_config *module_cfg);

#endif /* __ISP__MODULE__CFG__H */
