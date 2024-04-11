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

#ifndef _DE_FEAT_H_
#define _DE_FEAT_H_

#define DE_OUTPUT_TYPE_LCD   1
#define DE_OUTPUT_TYPE_TV    2
#define DE_OUTPUT_TYPE_HDMI  4
#define DE_OUTPUT_TYPE_VGA   8
#define DE_OUTPUT_TYPE_VDPO  16
#define WB_HAS_CSC

#define CVBS_PAL_WIDTH 720
#define CVBS_PAL_HEIGHT 576
#define CVBS_NTSC_WIDTH 720
#define CVBS_NTSC_HEIGHT 480

#define P2P_FB_MIN_WIDTH 704
#define P2P_FB_MAX_WIDTH 736
#define PALETTE_SIZE 256
#define SUPPORT_PQ

#if IS_ENABLED(CONFIG_ARCH_SUN50IW2)

/* features for sun50iw2 */

#define DEVICE_NUM	2
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	1
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

/* #define SUPPORT_DSI */
/* #define SUPPORT_SMBL */
#define SUPPORT_HDMI
/* #define DSI_VERSION_40 */
/* #define HAVE_DEVICE_COMMON_MODULE */
#define SUPPORT_TV
#define TV_UGLY_CLK_RATE 216000000
/* #define SUPPORT_VGA */
/* #define SUPPORT_LVDS */
/* #define LVDS_REVERT */

#if IS_ENABLED(CONFIG_AW_FPGA_V7) || IS_ENABLED(CONFIG_AW_FPGA_S4)
/*
 * TCON1_DRIVE_PANEL - General for fpga verify
 * On some platform there is no tcon0
 * At fpga period, we can use tcon1 to drive lcd pnael
 * we need to config & enable tcon1.
 * So enable this config.
 */
#define TCON1_DRIVE_PANEL
#endif

#elif defined(CONFIG_ARCH_SUN8IW11)

/* features for sun8iw11 */

#define DEVICE_NUM	4
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	1
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#define SUPPORT_DSI
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
#define SUPPORT_HDMI
#define DSI_VERSION_40
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_TV
#define TV_UGLY_CLK_RATE 216000000
#define SUPPORT_VGA
#define SUPPORT_LVDS
#define DE_WB_RESET_SHARE
/* #define LVDS_REVERT */
#define TCON_POL_CORRECT
#elif defined(CONFIG_ARCH_SUN8IW15)

/* features for sun8iw15 */

#define DEVICE_NUM	1
#define DE_NUM	1
#define CHN_NUM		4
#define VI_CHN_NUM	1
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#define SUPPORT_DSI
#define DSI_VERSION_28
#define CLK_NUM_PER_DSI 2
#define DEVICE_DSI_NUM 1
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_LVDS
/* #define DE_WB_RESET_SHARE */

/* #define SUPPORT_EINK */
/* #define EINK_PANEL_USED */
/* #define SUPPORT_WB */
/* #define EINK_DMABUF_USED */

/* #define LVDS_REVERT */

#elif defined(CONFIG_ARCH_SUN50IW10)
/* features for sun50iw10 */

#define SUPPORT_INDEPENDENT_DE
#define DEVICE_NUM	2
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	2
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#define SUPPORT_DSI
#define SUPPORT_COMBO_DPHY
#define DSI_VERSION_40
#define CLK_NUM_PER_DSI 1
#define DEVICE_DSI_NUM 1
#define SUPPORT_SMBL
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_LVDS
#define DEVICE_LVDS_NUM 2
#define SUNXI_DSI_PASSIVE_BUG
#define IOMMU_DE0_MASTOR_ID 0
#define IOMMU_DE1_MASTOR_ID 1
/* #define DE_WB_RESET_SHARE */

/* #define SUPPORT_EINK */
/* #define EINK_PANEL_USED */
/* #define SUPPORT_WB */
/* #define EINK_DMABUF_USED */

/* #define LVDS_REVERT */
#elif defined(CONFIG_ARCH_SUN8IW20) || defined(CONFIG_ARCH_SUN20IW1)
#define DEVICE_NUM      2
#define DE_NUM          2
#define CHN_NUM         2
#define VI_CHN_NUM      1
#define UI_CHN_NUM      (CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN    4
#define LAYER_NUM_PER_CHN_PER_UI_CHN    4
#define LAYER_MAX_NUM_PER_CHN 4
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_LVDS
#define DEVICE_LVDS_NUM 1
#define SUPPORT_COMBO_DPHY
#define SUPPORT_DSI
#define SUPPORT_HDMI
#define BYPASS_TCON_CEU
#define SUPPORT_TV
#define CLK_NUM_PER_DSI 1
#define DEVICE_DSI_NUM 1
#define SUPPORT_LBC
#define SUPPORT_PALETTE
#define SUPPORT_DITHER_OUTPUT
#define IOMMU_DE0_MASTOR_ID 2
#define IOMMU_DE1_MASTOR_ID 2

#define USE_CEC_DDC_PAD
#define DSI_VERSION_40
#define TV_UGLY_CLK_RATE 216000000

#elif defined(CONFIG_ARCH_SUN50IW1)

/* features for sun50iw1 */

#define DEVICE_NUM	4
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	1
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#define SUPPORT_DSI
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
#define SUPPORT_HDMI
#define DSI_VERSION_40
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_TV
#define SUPPORT_VGA
#define SUPPORT_LVDS
/* #define LVDS_REVERT */

#elif defined(CONFIG_ARCH_SUN50IW8)

/* features for sun50iw8 */

#define DEVICE_NUM      1
#define DE_NUM  1
#define CHN_NUM         2
#define VI_CHN_NUM      0
#define UI_CHN_NUM      (CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN    4
#define LAYER_NUM_PER_CHN_PER_UI_CHN    4
#define LAYER_MAX_NUM_PER_CHN 4

/* #define SUPPORT_DSI */
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
/* #define SUPPORT_HDMI */
/* #define DSI_VERSION_40 */
#define HAVE_DEVICE_COMMON_MODULE
/* #define SUPPORT_TV */
/* #define SUPPORT_VGA */
/* #define SUPPORT_LVDS */
/* #define LVDS_REVERT */


#elif defined(CONFIG_ARCH_SUN8IW12)
/* features for sun8iw12 */

#define DEVICE_NUM	2
#define DE_NUM	1
#define CHN_NUM		4
#define VI_CHN_NUM	2
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif

#define SUPPORT_DSI
#define DSI_VERSION_28
#define CLK_NUM_PER_DSI 2
#define DEVICE_DSI_NUM 1
#define SUPPORT_HDMI
/* #define DSI_VERSION_40 */
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_TV
#define TV_UGLY_CLK_RATE 216000000
/* #define SUPPORT_VGA */
#define SUPPORT_LVDS
#define DE_WB_RESET_SHARE
/* #define LVDS_REVERT */
#ifdef COFNIG_VDPO_AW_DISP2
#define SUPPORT_VDPO
#define DEVICE_VDPO_NUM 1
#endif

#elif defined(CONFIG_ARCH_SUN8IW16)
#define DEVICE_NUM	2
#define DE_NUM	1
#define CHN_NUM		4
#define VI_CHN_NUM	2
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif

#define SUPPORT_DSI
#define DSI_VERSION_28
#define CLK_NUM_PER_DSI 2
#define DEVICE_DSI_NUM 1
#define SUPPORT_HDMI
#define BYPASS_TCON_CEU
#define USE_CEC_DDC_PAD
#define WB_HAS_CSC

/* #define DSI_VERSION_40 */
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_TV
#define TV_UGLY_CLK_RATE 216000000
/* #define SUPPORT_VGA */
#define SUPPORT_LVDS
#define DE_WB_RESET_SHARE
/* #define LVDS_REVERT */

#ifdef COFNIG_VDPO_AW_DISP2
#define SUPPORT_VDPO
#define DEVICE_VDPO_NUM 1
#endif

#elif defined(CONFIG_ARCH_SUN8IW19)
#define DEVICE_NUM	1
#define DE_NUM	1
#define CHN_NUM		4 /* It is 3 in fact */
#define VI_CHN_NUM	2
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif

#define SUPPORT_DSI
#define DSI_VERSION_28
#define CLK_NUM_PER_DSI 2
#define DEVICE_DSI_NUM 1
#define BYPASS_TCON_CEU
#define USE_CEC_DDC_PAD
#define WB_HAS_CSC

/* #define DSI_VERSION_40 */
#define HAVE_DEVICE_COMMON_MODULE
/* #define SUPPORT_VGA */
#define DE_WB_RESET_SHARE
/* #define LVDS_REVERT */

#ifdef COFNIG_VDPO_AW_DISP2
#define SUPPORT_VDPO
#define DEVICE_VDPO_NUM 1
#endif

#elif defined(CONFIG_ARCH_SUN8IW6)
/* features for sun8iw6 */

#define DEVICE_NUM      2
#define DE_NUM          2
#define CHN_NUM         4
#define VI_CHN_NUM      1
#define UI_CHN_NUM      (CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN    4
#define LAYER_NUM_PER_CHN_PER_UI_CHN    4
#define LAYER_MAX_NUM_PER_CHN 4
#define SUPPORT_LVDS
#define SUPPORT_DSI
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
#define SUPPORT_HDMI
#define SUPPORT_TV
#define DSI_VERSION_28
#define CLK_NUM_PER_DSI 2
#define DE_WB_RESET_SHARE
#define LVDS_REVERT

#elif defined(CONFIG_ARCH_SUN8IW7)

#define DEVICE_NUM	2
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	1
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4
#define VEP_NUM  1
#define DE_WB_RESET_SHARE

#define SUPPORT_HDMI
#define SUPPORT_TV
#define TV_UGLY_CLK_RATE 216000000

#elif defined(CONFIG_ARCH_SUN8IW17)

/* features for sun8iw17 */

#define DEVICE_NUM	3
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	2
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_MAX_VI_NUM_PER_CHN 4
#define LAYER_MAX_UI_NUM_PER_CHN 4
#define LAYER_MAX_NUM_PER_CHN 4

#define SUPPORT_DSI
#define DEVICE_DSI_NUM 2
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
/* #define SUPPORT_HDMI */
#define DSI_VERSION_40
#define HAVE_DEVICE_COMMON_MODULE
#define SUPPORT_TV
#define TV_UGLY_CLK_RATE 216000000
/* #define SUPPORT_VGA */
#define SUPPORT_LVDS
/* #define LVDS_REVERT */

#else

/* default features */
#error "undefined platform!!!"

#define DEVICE_NUM	2
#define DE_NUM	2
#define CHN_NUM		4
#define VI_CHN_NUM	1
#define UI_CHN_NUM	(CHN_NUM - VI_CHN_NUM)
#define LAYER_NUM_PER_CHN_PER_VI_CHN	4
#define LAYER_NUM_PER_CHN_PER_UI_CHN	4
#define LAYER_MAX_NUM_PER_CHN 4

#define SUPPORT_DSI
#if IS_ENABLED(CONFIG_AW_DISP2_SUPPORT_SMBL)
#define SUPPORT_SMBL
#endif
#define SUPPORT_HDMI
#define DSI_VERSION_40
/* #define HAVE_DEVICE_COMMON_MODULE */
#define SUPPORT_TV
/* #define SUPPORT_VGA */
#define SUPPORT_LVDS
/* #define LVDS_REVERT */
#endif

#if defined(TV_UGLY_CLK_RATE)
#define TV_COMPOSITE_CLK_RATE 27000000
#endif

#ifndef CLK_NUM_PER_DSI
#define CLK_NUM_PER_DSI 1
#endif

#ifndef DEVICE_DSI_NUM
#define DEVICE_DSI_NUM 1
#endif /* endif DEVICE_DSI_NUM */

#ifndef DEVICE_LVDS_NUM
#define DEVICE_LVDS_NUM 1
#endif
/* total number of DSI clk */
#define CLK_DSI_NUM  (CLK_NUM_PER_DSI * DEVICE_DSI_NUM)

#ifndef SUPPORT_VDPO
#define DEVICE_VDPO_NUM 0
#endif

#ifndef IOMMU_DE0_MASTOR_ID
#define IOMMU_DE0_MASTOR_ID 0
#define IOMMU_DE1_MASTOR_ID 0
#endif

struct de_feat {
	const int num_screens;
	/* indicate layer manager number */
	const int num_devices;
	/* indicate timing controller number */
	const int *num_chns;
	const int *num_vi_chns;
	const int *num_layers;
	const int *is_support_vep;
	const int *is_support_smbl;
	const int *is_support_wb;
	const int *supported_output_types;
	const int *is_support_scale;
	const int *scale_line_buffer;
#if defined(SUPPORT_LBC)
	const int *is_support_lbc;
#endif

	const int num_vdpo; /* number of vdpo device */
};

int de_feat_init(void);
int de_feat_exit(void);
int de_feat_get_num_screens(void);
int de_feat_get_num_devices(void);
int de_feat_get_num_chns(unsigned int disp);
int de_feat_get_num_vi_chns(unsigned int disp);
unsigned int de_feat_get_number_of_vdpo(void);
int de_feat_get_num_ui_chns(unsigned int disp);
int de_feat_get_num_layers(unsigned int disp);
int de_feat_get_num_layers_by_chn(unsigned int disp, unsigned int chn);
int de_feat_is_support_vep(unsigned int disp);
int de_feat_is_support_vep_by_chn(unsigned int disp, unsigned int chn);
int de_feat_is_support_smbl(unsigned int disp);
int de_feat_is_supported_output_types(unsigned int disp,
				      unsigned int output_type);
int de_feat_is_support_wb(unsigned int disp);
int de_feat_is_support_scale(unsigned int disp);
int de_feat_is_support_scale_by_chn(unsigned int disp, unsigned int chn);
int de_feat_get_scale_linebuf(unsigned int disp);
int de_feat_get_tcon_index(unsigned int tcon_index);
unsigned int de_feat_get_tcon_type(unsigned int tcon_index);

#if defined(SUPPORT_LBC)
int de_feat_is_support_lbc_by_chn(unsigned int disp, unsigned int chn);
#endif

#endif
