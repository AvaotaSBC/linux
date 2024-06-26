/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * isp522_reg_cfg.h
 *
 * Copyright (c) 2007-2019 Allwinnertech Co., Ltd.
 *
 * Authors:  zheng jiangwei <zhengjiangwei@allwinnertech.com>
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

#ifndef _ISP522_REG_CFG_H_
#define _ISP522_REG_CFG_H_

#define ISP_ADDR_BIT_R_SHIFT 2

#define ISP_LOAD_DRAM_SIZE			0x13240
#define ISP_LOAD_REG_SIZE			0x1000
#define ISP_FE_TBL_SIZE				0x0600
#define ISP_BAYER_TABLE_SIZE			0x6540
#define ISP_RGB_TABLE_SIZE			0x1c00
#define ISP_YUV_TABLE_SIZE			0x1700
#define ISP_DBG_TABLE_SIZE			0x8400

#define ISP_SAVE_DRAM_SIZE			0x16e40
#define ISP_SAVE_REG_SIZE			0x0040
#define ISP_STATISTIC_SIZE			0xea00
#define ISP_STAT_TOTAL_SIZE			0xea40
#define ISP_DBG_STAT_SIZE			0x8400

#define ISP_S1_LC_R_TBL_SIZE			0x0200
#define ISP_S1_LC_G_TBL_SIZE			0x0200
#define ISP_S1_LC_B_TBL_SIZE			0x0200
#define ISP_S1_LC_TBL_SIZE			0x0600

#define ISP_S0_LC_R_TBL_SIZE			0x0200
#define ISP_S0_LC_G_TBL_SIZE			0x0200
#define ISP_S0_LC_B_TBL_SIZE			0x0200
#define ISP_S0_LC_TBL_SIZE			0x0600

#define ISP_RSC_TBL_SIZE			0x0800
#define ISP_MSC_TBL_MEM_SIZE			0x0f20

#define ISP_SATURATION_TBL_SIZE			0x0200
#define ISP_RGB_DRC_TBL_SIZE			0x0200
#define ISP_GAMMA_MEM_SIZE			0x1000

#define ISP_CEM_TBL0_SIZE			0x0cc0
#define ISP_CEM_TBL1_SIZE			0x0a40

#define ISP_STAT_HIST_MEM_SIZE			0x0200
#define ISP_STAT_AE_MEM_SIZE			0x1200
#define ISP_STAT_AF_MEM_SIZE			0x3c00
#define ISP_STAT_AF_IIR_ACC_SIZE		0x0c00
#define ISP_STAT_AF_FIR_ACC_SIZE		0x0c00
#define ISP_STAT_AF_IIR_CNT_SIZE		0x0c00
#define ISP_STAT_AF_FIR_CNT_SIZE		0x0c00
#define ISP_STAT_AF_HL_CNT_SIZE			0x0c00
#define ISP_STAT_AFS_SIZE			0x0200
#define ISP_STAT_AWB_RGB_SIZE			0x3000
#define ISP_STAT_AWB_CNT_SIZE			0x0800

#define RSC_UPDATE			(1 << 2)
#define GAMMA_UPDATE			(1 << 3)
#define DRC_UPDATE			(1 << 4)
#define SATU_UPDATE			(1 << 5)
#define WDR_UPDATE			(1 << 6) /* there is no exist in isp522 */
#define D3D_UPDATE			(1 << 7) /* there is no exist in isp522 */
#define PLTM_UPDATE			(1 << 8) /* there is no exist in isp522 */
#define CEM_UPDATE			(1 << 9)
#define MSC_UPDATE			(1 << 10)

#define TABLE_UPDATE_ALL 0xffffffff

/*
 *  ISP Module enable
 */
#define AE_EN		(1 << 0)
#define WDR_EN		(1 << 2) /* there is no exist in isp522 */
#define DPC_EN		(1 << 3)
#define D2D_EN		(1 << 4)
#define D3D_EN		(1 << 5) /* there is no exist in isp522 */
#define AWB_EN		(1 << 6)
#define WB_EN		(1 << 7)
#define LSC_EN		(1 << 8)
#define BGC_EN		(1 << 9)
#define SHARP_EN	(1 << 10)
#define AF_EN		(1 << 11)
#define RGB2RGB_EN	(1 << 12)
#define RGB_DRC_EN	(1 << 13)
#define PLTM_EN		(1 << 14) /* sthere is no exist in isp522 */
#define CEM_EN		(1 << 15)
#define AFS_EN		(1 << 16)
#define HIST_EN		(1 << 17)
#define DG_EN		(1 << 19)
#define SO_EN		(1 << 20)
#define CTC_EN		(1 << 21)
#define MSC_EN     	(1 << 22)
#define CNR_EN		(1 << 23)
#define SATU_EN		(1 << 24)
#define LCA_EN		(1 << 27)
#define GCA_EN		(1 << 28)

#define SRC0_EN         (1 << 31)

#define ISP_MODULE_EN_ALL	(0xffffffff)

/*
 *  ISP interrupt enable
 */
#define FINISH_INT_EN		(1 << 0)
#define S0_START_INT_EN		(1 << 1)
#define PARA_SAVE_INT_EN	(1 << 3)
#define S0_PARA_LOAD_INT_EN	(1 << 4)
#define S0_FIFO_INT_EN		(1 << 6)
#define S0_N_LINE_START_INT_EN	(1 << 8)
#define S0_FRAME_ERROR_INT_EN	(1 << 10)
#define S0_FRAME_LOST_INT_EN	(1 << 12)
#define S0_HB_SHORT_INT_EN	(1 << 14)
#define SRAM_CLR_INT_EN		(1 << 16)
#define DDR_W_FINISH_INT_EN	(1 << 17)
#define S0_BTYPE_ERROR_INT_EN	(1 << 18)
#define FSM_FRAME_LOST_INT_EN	(1 << 22)
#define AHB_DDR_W_INT_EN	(1 << 23)

#define ISP_IRQ_EN_ALL	0xffffffff

/*
 *  ISP interrupt status
 */
#define FINISH_PD		(1 << 0)
#define S0_START_PD		(1 << 1)
#define PARA_SAVE_PD		(1 << 3)
#define S0_PARA_LOAD_PD		(1 << 4)
#define S0_FIFO_OF_PD		(1 << 6)
#define S0_N_LINE_START_PD	(1 << 8)
#define S0_FRAME_ERROR_PD	(1 << 10)
#define S0_FRAME_LOST_PD	(1 << 12)
#define S0_HB_SHORT_PD		(1 << 14)
#define SRAM_CLR_PD		(1 << 16)
#define FSM_FRAME_LOST_PD	(1 << 22)
#define AHB_DDR_W_INT_PD	(1 << 23)

#define FIFO_VALID_ST		(1 << 31)

#define PARA_LOAD_PD		S0_PARA_LOAD_PD
#define FRAME_ERROR_PD		S0_FRAME_ERROR_PD
#define FRAME_LOST_PD		S0_FRAME_LOST_PD
#define HB_SHORT_PD		S0_HB_SHORT_PD

#define ISP_IRQ_STATUS_ALL	0xffffffff

/*
 *  ISP internal status
 */
#define S0_CIN_FIFO_OF_PD	(1 << 2)
#define BIS_FIFO_OF_PD		(1 << 5)
#define LCA_RGB_FIFO_R_EMP_PD	(1 << 16)
#define LCA_RGB_FIFO_W_FULL_PD	(1 << 17)
#define LCA_BY_FIFO_R_EMP_PD	(1 << 18)
#define LCA_BY_FIFO_W_FULL_PD	(1 << 19)

struct isp_wdr_mode_cfg {
	unsigned char wdr_ch_seq;
	unsigned char wdr_exp_seq;
	unsigned char wdr_mode;
};

struct isp_lbc_cfg {
	unsigned short line_tar_bits;
	unsigned short mb_min_bit;
	unsigned char lmtqp_min;
	bool lmtqp_en;
	bool is_lossy;
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
void bsp_isp_get_s0_ch_fmerr_cnt(unsigned long id, struct isp_size *size);
void bsp_isp_get_s0_ch_hb_cnt(unsigned long id, unsigned int *hb_max, unsigned int *hb_min);
void bsp_isp_set_fifo_mode(unsigned long id, unsigned int mode);
void bsp_isp_min_ddr_size(unsigned long id, unsigned int size);
void bsp_isp_fifo_raw_write(unsigned long id, unsigned int depth);
void bsp_isp_k_min_ddr_size(unsigned long id, unsigned int size);

/*******isp load register which we should write to ddr first*********/

void bsp_isp_module_enable(unsigned long id, unsigned int module_flag);
void bsp_isp_module_disable(unsigned long id, unsigned int module_flag);
void bsp_isp_set_size(unsigned long id, struct isp_size_settings *size);
unsigned int bsp_isp_load_update_flag(unsigned long id);

#endif /* _ISP522_REG_CFG_H_ */
