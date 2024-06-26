/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * isp520_reg_cfg.h
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

#ifndef __ISP520__REG__CFG__H
#define __ISP520__REG__CFG__H

#define ISP_ADDR_BIT_R_SHIFT 2

#define ISP_LOAD_DRAM_SIZE			0x9f00
#define ISP_LOAD_REG_SIZE			0x1000
#define ISP_FE_TBL_SIZE				0x0600
#define ISP_BAYER_TABLE_SIZE			0x5600
#define ISP_RGB_TABLE_SIZE			0x1c00
#define ISP_YUV_TABLE_SIZE			0x1700

#define ISP_SAVE_DRAM_SIZE			0xea40
#define ISP_SAVE_REG_SIZE			0x0040
#define ISP_STATISTIC_SIZE			0xea00
#define ISP_STAT_TOTAL_SIZE			0xea40

#define ISP_S1_LC_R_TBL_SIZE			0x0200
#define ISP_S1_LC_G_TBL_SIZE			0x0200
#define ISP_S1_LC_B_TBL_SIZE			0x0200
#define ISP_S1_LC_TBL_SIZE			0x0600

#define ISP_S0_LC_R_TBL_SIZE			0x0200
#define ISP_S0_LC_G_TBL_SIZE			0x0200
#define ISP_S0_LC_B_TBL_SIZE			0x0200
#define ISP_S0_LC_TBL_SIZE			0x0600

#define ISP_WDR_GAMMA_FE_MEM_SIZE		0x2000
#define ISP_WDR_GAMMA_BE_MEM_SIZE		0x2000

#define ISP_LSC_TBL_SIZE			0x0800
#define ISP_D3D_K_3D_INCREASE_SIZE		0x0100
#define ISP_D3D_DIFF_TBL_SIZE			0x0100
#define ISP_PLTM_MERGE_H_TBL_SIZE		0x0100
#define ISP_PLTM_MERGE_V_TBL_SIZE		0x0100
#define ISP_PLTM_POW_TBL_SIZE			0x0200
#define ISP_PLTM_F_TBL_SIZE			0x0200

#define ISP_SATURATION_TBL_SIZE			0x0200
#define ISP_RGB_DRC_TBL_SIZE			0x0200
#define ISP_GAMMA_MEM_SIZE			0x1000
#define ISP_DEHAZE_PP_TBL_SIZE			0x0100
#define ISP_DEHAZE_PD_TBL_SIZE			0x0100
#define ISP_DEHAZE_TR_TBL_SIZE			0x0100
#define ISP_DEHAZE_BT_TBL_SIZE			0x0400

#define ISP_CEM_TBL0_SIZE			0x0cc0
#define ISP_CEM_TBL1_SIZE			0x0a40

#define ISP_STAT_HIST_MEM_SIZE			0x0200
#define ISP_STAT_AE_MEM_SIZE			0x4800
#define ISP_STAT_AF_MEM_SIZE			0x3c00
#define ISP_STAT_AF_IIR_ACC_SIZE		0x0c00
#define ISP_STAT_AF_FIR_ACC_SIZE		0x0c00
#define ISP_STAT_AF_IIR_CNT_SIZE		0x0c00
#define ISP_STAT_AF_FIR_CNT_SIZE		0x0c00
#define ISP_STAT_AF_HL_CNT_SIZE			0x0c00
#define ISP_STAT_AFS_SIZE			0x0200
#define ISP_STAT_AWB_RGB_SIZE			0x4800
#define ISP_STAT_AWB_CNT_SIZE			0x0c00
#define ISP_STAT_PLTM_LST_SIZE			0x0600
#define ISP_STAT_DEHAZE_DC_HIST_SIZE		0x0400
#define ISP_STAT_DEHAZE_BT_SIZE			0x0400

/*
 *  update table
 */
#define LINEAR_UPDATE		(1 << 1)
#define LENS_UPDATE		(1 << 2)
#define GAMMA_UPDATE		(1 << 3)
#define DRC_UPDATE		(1 << 4)
#define SATU_UPDATE		(1 << 5)
#define WDR_UPDATE		(1 << 6)
#define D3D_UPDATE		(1 << 7)
#define PLTM_UPDATE		(1 << 8)
#define CEM_UPDATE		(1 << 9)
#define DEHAZE_UPDATE		(1 << 11)
#define S1_PARA_READY		(1 << 12)
#define S1_LINEAR_UPDATE	(1 << 13)


#define TABLE_UPDATE_ALL 0xffffffff

/*
 *  ISP Module enable
 */
#define S1_BLC_EN	(1 << 0)
#define S1_LC_EN	(1 << 1)

#define AE_EN		(1 << 0)
#define LC_EN		(1 << 1)
#define WDR_EN		(1 << 2)
#define DPC_EN		(1 << 3)
#define D2D_EN		(1 << 4)
#define D3D_EN		(1 << 5)
#define AWB_EN		(1 << 6)
#define WB_EN		(1 << 7)
#define LSC_EN		(1 << 8)
#define BGC_EN		(1 << 9)
#define SHARP_EN	(1 << 10)
#define AF_EN		(1 << 11)
#define RGB2RGB_EN	(1 << 12)
#define RGB_DRC_EN	(1 << 13)
#define PLTM_EN		(1 << 14)
#define CEM_EN		(1 << 15)
#define AFS_EN		(1 << 16)
#define HIST_EN		(1 << 17)
#define BLC_EN		(1 << 18)
#define DG_EN		(1 << 19)
#define SO_EN		(1 << 20)
#define CTC_EN		(1 << 21)
#define CNR_EN		(1 << 23)
#define SATU_EN		(1 << 24)
#define DEHAZE_EN	(1 << 25)

#define SRC0_EN         (1 << 31)

#define ISP_MODULE_EN_ALL	(0xffffffff)

/*
 *  ISP interrupt enable
 */
#define FINISH_INT_EN		(1 << 0)
#define S0_START_INT_EN		(1 << 1)
#define S1_START_INT_EN		(1 << 2)
#define PARA_SAVE_INT_EN	(1 << 3)
#define S0_PARA_LOAD_INT_EN	(1 << 4)
#define S1_PARA_LOAD_INT_EN	(1 << 5)
#define S0_FIFO_INT_EN		(1 << 6)
#define S1_FIFO_INT_EN		(1 << 7)
#define S0_N_LINE_START_INT_EN	(1 << 8)
#define S1_N_LINE_START_INT_EN	(1 << 9)
#define S0_FRAME_ERROR_INT_EN	(1 << 10)
#define S1_FRAME_ERROR_INT_EN	(1 << 11)
#define S0_FRAME_LOST_INT_EN	(1 << 12)
#define S1_FRAME_LOST_INT_EN	(1 << 13)
#define S0_HB_SHORT_INT_EN	(1 << 14)
#define S1_HB_SHORT_INT_EN	(1 << 15)
#define SRAM_CLR_INT_EN		(1 << 16)
#define DDR_W_FINISH_INT_EN	(1 << 17)

#define ISP_IRQ_EN_ALL	0xffffffff

#define FINISH_PD		(1 << 0)
#define S0_START_PD		(1 << 1)
#define S1_START_PD		(1 << 2)
#define PARA_SAVE_PD		(1 << 3)
#define S0_PARA_LOAD_PD		(1 << 4)
#define S1_PARA_LOAD_PD		(1 << 5)
#define S0_FIFO_OF_PD		(1 << 6)
#define S1_FIFO_OF_PD		(1 << 7)
#define S0_N_LINE_START_PD	(1 << 8)
#define S1_N_LINE_START_PD	(1 << 9)
#define S0_FRAME_ERROR_PD	(1 << 10)
#define S1_FRAME_ERROR_PD	(1 << 11)
#define S0_FRAME_LOST_PD	(1 << 12)
#define S1_FRAME_LOST_PD	(1 << 13)
#define S0_HB_SHORT_PD		(1 << 14)
#define S1_HB_SHORT_PD		(1 << 15)
#define SRAM_CLR_PD		(1 << 16)
#define DDR_W_FINISH_PD		(1 << 17)
#define FIFO_VALID_ST		(1 << 31)

#define PARA_LOAD_PD		S0_PARA_LOAD_PD
#define FRAME_ERROR_PD		S0_FRAME_ERROR_PD
#define FRAME_LOST_PD		S0_FRAME_LOST_PD
#define HB_SHORT_PD		S0_HB_SHORT_PD

#define ISP_IRQ_STATUS_ALL	0xffffffff

#define D3D_W_FINISH_PD		(1 << 0)
#define WDR_W_FINISH_PD		(1 << 1)
#define S0_CIN_FIFO_OF_PD	(1 << 2)
#define DPC_FIFO_OF_PD		(1 << 3)
#define BIS_FIFO_OF_PD		(1 << 5)
#define CNR_FIFO_OF_PD		(1 << 6)
#define PLTM_FIFO_OF_PD		(1 << 7)
#define D3D_WRITE_FIFO_OF_PD	(1 << 8)
#define D3D_READ_FIFO_OF_PD	(1 << 9)
#define WDR_WRITE_FIFO_OF_PD	(1 << 11)
#define WDR_READ_FIFO_OF_PD	(1 << 13)
#define S1_CIN_FIFO_OF_PD	(1 << 15)

struct isp_wdr_mode_cfg {
	unsigned char wdr_ch_seq;
	unsigned char wdr_exp_seq;
	unsigned char wdr_mode;
};

enum isp_channel {
	ISP_CH0 = 0,
	ISP_CH1 = 1,
	ISP_CH2 = 2,
	ISP_CH3 = 3,
	ISP_MAX_CH_NUM,
};

struct isp_size {
	u32 width;
	u32 height;
};

struct coor {
	u32 hor;
	u32 ver;
};

struct isp_size_settings {
	struct coor ob_start;
	struct isp_size ob_black;
	struct isp_size ob_valid;
	u32 set_cnt;
};

enum ready_flag {
	PARA_NOT_READY = 0,
	PARA_READY = 1,
};

enum enable_flag {
	DISABLE    = 0,
	ENABLE     = 1,
};

enum isp_input_seq {
	ISP_BGGR = 4,
	ISP_RGGB = 5,
	ISP_GBRG = 6,
	ISP_GRBG = 7,
};

void bsp_isp_map_reg_addr(unsigned long id, unsigned long base);
void bsp_isp_map_load_dram_addr(unsigned long id, unsigned long base);

/*******isp control register which we can write directly to register*********/

void bsp_isp_enable(unsigned long id, unsigned int en);
void bsp_isp_capture_start(unsigned long id);
void bsp_isp_capture_stop(unsigned long id);
void bsp_isp_ver_read_en(unsigned long id, unsigned int en);
void bsp_isp_set_input_fmt(unsigned long id, unsigned int fmt);
void bsp_isp_ch_enable(unsigned long id, int ch, int enable);
void bsp_isp_wdr_mode_cfg(unsigned long id, struct isp_wdr_mode_cfg *cfg);
void bsp_isp_set_line_int_num(unsigned long id, unsigned int line_num);
void bsp_isp_set_speed_mode(unsigned long id, unsigned int speed);
void bsp_isp_set_last_blank_cycle(unsigned long id, unsigned int blank);
void bsp_isp_debug_output_cfg(unsigned long id, int enable, int output_sel);
void bsp_isp_set_para_ready_mode(unsigned long id, int enable);
void bsp_isp_set_para_ready(unsigned long id, int ready);
void bsp_isp_update_table(unsigned long id, unsigned short table_update);
void bsp_isp_set_load_addr0(unsigned long id, dma_addr_t addr);
void bsp_isp_set_load_addr1(unsigned long id, dma_addr_t addr);
void bsp_isp_set_saved_addr(unsigned long id, unsigned long addr);
void bsp_isp_set_statistics_addr(unsigned long id, dma_addr_t addr);
void bsp_isp_irq_enable(unsigned long id, unsigned int irq_flag);
void bsp_isp_irq_disable(unsigned long id, unsigned int irq_flag);
unsigned int bsp_isp_get_irq_status(unsigned long id, unsigned int flag);
void bsp_isp_clr_irq_status(unsigned long id, unsigned int flag);
unsigned int bsp_isp_get_internal_status0(unsigned long id, unsigned int flag);
void bsp_isp_clr_internal_status0(unsigned long id, unsigned int flag);
unsigned int bsp_isp_get_internal_status1(unsigned long id);
unsigned int bsp_isp_get_isp_ver(unsigned long id, unsigned int *major, unsigned int *minor);
unsigned int bsp_isp_get_max_width(unsigned long id);
void bsp_isp_get_comp_fifo_max_layer(unsigned long id, unsigned int *wdr_fifo, unsigned int *d3d_fifo);
void bsp_isp_get_uncomp_fifo_max_layer(unsigned long id, unsigned int *wdr_fifo, unsigned int *d3d_fifo);
void bsp_isp_get_s0_ch_fmerr_cnt(unsigned long id, struct isp_size *size);
void bsp_isp_get_s1_ch_fmerr_cnt(unsigned long id, struct isp_size *size);
void bsp_isp_get_s0_ch_hb_cnt(unsigned long id, unsigned int *hb_max, unsigned int *hb_min);
void bsp_isp_get_s1_ch_hb_cnt(unsigned long id, unsigned int *hb_max, unsigned int *hb_min);
void bsp_isp_get_wdr_fifo_overflow_line(unsigned long id, unsigned int *decomp_line, unsigned int *comp_line);
void bsp_isp_get_d3d_fifo_overflow_line(unsigned long id, unsigned int *decomp_line, unsigned int *comp_line);
void bsp_isp_set_wdr_addr0(unsigned long id, dma_addr_t addr);
void bsp_isp_set_wdr_addr1(unsigned long id, dma_addr_t addr);
void bsp_isp_set_d3d_addr0(unsigned long id, dma_addr_t addr);
void bsp_isp_set_d3d_addr1(unsigned long id, dma_addr_t addr);
void bsp_isp_top_control(unsigned long id, int isp_num, int isp0_max_w);
void bsp_isp_set_fifo_mode(unsigned long id, unsigned int mode);
void bsp_isp_min_ddr_size(unsigned long id, unsigned int size);
void bsp_isp_fifo_raw_write(unsigned long id, unsigned int depth);
void bsp_isp_k_min_ddr_size(unsigned long id, unsigned int size);

/*******isp load register which we should write to ddr first*********/

void bsp_isp_s1_module_enable(unsigned long id, unsigned int module_flag);
void bsp_isp_s1_module_disable(unsigned long id, unsigned int module_flag);
void bsp_isp_module_enable(unsigned long id, unsigned int module_flag);
void bsp_isp_module_disable(unsigned long id, unsigned int module_flag);
void bsp_isp_set_size(unsigned long id, struct isp_size_settings *size);
unsigned int bsp_isp_load_update_flag(unsigned long id);

#endif /* __ISP520__REG__CFG__H */
