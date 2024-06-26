/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* SPDX-License-Identifier: GPL-2.0 */
 /*
  * isp522_reg.h
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

#ifndef _ISP522_REG_H_
#define _ISP522_REG_H_

#define ISP0_BASE_ADDR_OFFSET			0x0000
#define ISP1_BASE_ADDR_OFFSET			0x2000

#define ISP_GLOBAL_CFG0_REG			0x000
#define ISP_GLOBAL_CFG1_REG			0x004
#define ISP_GLOBAL_CFG2_REG			0x008
#define ISP_UPDATE_CTRL0_REG			0x020
#define ISP_LOAD_ADDR0_REG			0x030
#define ISP_LOAD_ADDR1_REG			0x034
#define ISP_SAVE_ADDR_REG			0x038
#define ISP_INT_BYPASS0_REG			0x040
#define ISP_INT_STATUS0_REG			0x048
#define ISP_INTER_STATUS0_REG			0x060
#define ISP_INTER_STATUS1_REG			0x064
#define ISP_WR_ADDR_RET_REG			0x070
#define ISP_VER_CFG_REG				0x080
#define ISP_MAX_WIDTH_REG			0x084
#define ISP_S0_FMERR_CNT_REG			0x0a0
#define ISP_S0_HB_CNT_REG			0x0b0
#define ISP_INPUT_SIZE_REG			0x0e4
#define ISP_VALID_SIZE_REG			0x0e8
#define ISP_VALID_START_REG			0x0ec
#define ISP_MODULE_STATUS_REG			0x180
#define ISP_MODULE_BYPASS0_REG			0x1a0
#define ISP_MODULE_MODE0_REG			0x1b0


typedef union {
	unsigned int dwval;
	struct {
		unsigned int isp_enable:1;
		unsigned int cap_en:1;
		unsigned int isp_ver_rd_en:1;
		unsigned int int_mode:1;
		unsigned int input_fmt:3;
		unsigned int res0:1;
		unsigned int isp_ch0_en:1;
		unsigned int isp_ch1_en:1;
		unsigned int isp_ch2_en:1;
		unsigned int isp_ch3_en:1;
		unsigned int res1:4;
		unsigned int res2:16;
	} bits;
} ISP_GLOBAL_CFG0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int line_int_num:14;
		unsigned int res0:2;
		unsigned int speed_mode:3;
		unsigned int res1:1;
		unsigned int last_blank_cycle:3;
		unsigned int res2:1;
		unsigned int res4:8;
	} bits;
} ISP_GLOBAL_CFG1_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int debug_en:1;
		unsigned int sram_clear:1;
		unsigned int pltm_sram_clr_en:1;
		unsigned int module_clr_back_door:1;
		unsigned int debug_sel:5;
		unsigned int res0:3;
		unsigned int vgm_en:1;
		unsigned int vgm_start:1;
		unsigned int vgm_mode:1;
		unsigned int res1:1;
		unsigned int d3d_cmp_ofln_sel:1;
		unsigned int d3d_uncmp_ofln_sel:2;
		unsigned int res2:9;
		unsigned int af_stat_sum_mode:1;
		unsigned int para_ready_mode:1;
		unsigned int hw_sram_clear_en:1;
		unsigned int fsm_back_door_en:1;
	} bits;
} ISP_GLOBAL_CFG2_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int fifo_dep_raw_rd:11;
		unsigned int res0:1;
		unsigned int raw_min_ddr_size:2;
		unsigned int d3d_uncmp_bandwd_sel:2;
		unsigned int fifo_dep_raw_wr:11;
		unsigned int res1:1;
		unsigned int k_min_ddr_size:2;
		unsigned int d3d_cmp_bandwd_sel:2;
	} bits;
} ISP_GLOBAL_CFG3_REG_t;


typedef union {
	unsigned int dwval;
	struct {
		unsigned int para_ready:1;
		unsigned int linear_update:1;
		unsigned int lens_update:1;
		unsigned int gamma_update:1;
		unsigned int drc_update:1;
		unsigned int satu_update:1;
		unsigned int wdr_update:1;
		unsigned int d3d_update:1;
		unsigned int pltm_update:1;
		unsigned int cem_update:1;
		unsigned int msc_update:1;
		unsigned int dehaze_update:1;
		unsigned int s1_para_ready:1;
		unsigned int s1_linear_update:1;
		unsigned int res1:16;
		unsigned int dbg_tbl_update:1;
		unsigned int dbg_stat_update:1;
	} bits;
} ISP_UPDATE_CTRL0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int finish_int_en:1;
		unsigned int s0_start_int_en:1;
		unsigned int s1_start_int_en:1;
		unsigned int para_save_int_en:1;
		unsigned int s0_para_load_int_en:1;
		unsigned int s1_para_load_int_en:1;
		unsigned int s0_fifo_int_en:1;
		unsigned int s1_fifo_int_en:1;
		unsigned int s0_n_line_start_int_en:1;
		unsigned int s1_n_line_start_int_en:1;
		unsigned int s0_frame_error_int_en:1;
		unsigned int s1_frame_error_int_en:1;
		unsigned int s0_frame_lost_int_en:1;
		unsigned int s1_frame_lost_int_en:1;
		unsigned int s0_hb_short_int_en:1;
		unsigned int s1_hb_short_int_en:1;
		unsigned int sram_clr_int_en:1;
		unsigned int ddr_w_finish_int_en:1;
		unsigned int s0_btype_error_int_en:1;
		unsigned int s1_btype_error_int_en:1;
		unsigned int addr_error_int_en:1;
		unsigned int lbc_error_int_en:1;
		unsigned int fsm_frame_lost_int_en:1;
		unsigned int res0:9;
	} bits;
} ISP_INT_BYPASS0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int finish_pd:1;
		unsigned int s0_start_pd:1;
		unsigned int s1_start_pd:1;
		unsigned int para_saved_pd:1;
		unsigned int s0_para_load_pd:1;
		unsigned int s1_para_load_pd:1;
		unsigned int s0_fifo_of_pd:1;
		unsigned int s1_fifo_of_pd:1;
		unsigned int s0_n_line_start_pd:1;
		unsigned int s1_n_line_start_pd:1;
		unsigned int s0_frame_error_pd:1;
		unsigned int s1_frame_error_pd:1;
		unsigned int s0_frame_lost_pd:1;
		unsigned int s1_frame_lost_pd:1;
		unsigned int s0_hb_short_pd:1;
		unsigned int s1_hb_short_pd:1;
		unsigned int sram_clr_pd:1;
		unsigned int ddr_w_finish_pd:1;
		unsigned int s0_btype_error_pd:1;
		unsigned int s1_btype_error_pd:1;
		unsigned int addr_error_pd:1;
		unsigned int lbc_error_pd:1;
		unsigned int fsm_frame_lost_pd:1;
		unsigned int res0:8;
		unsigned int fifo_valid_st:1;
	} bits;
} ISP_INT_STATUS0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int d3d_w_finish_pd:1;
		unsigned int wdr_w_finish_pd:1;
		unsigned int s0_cin_fifo_of_pd:1;
		unsigned int dpc_fifo_of_pd:1;
		unsigned int res0:1;
		unsigned int bis_fifo_of_pd:1;
		unsigned int cnr_fifo_of_pd:1;
		unsigned int pltm_fifo_of_pd:1;
		unsigned int d3d_write_fifo_of_pd:1;
		unsigned int d3d_read_fifo_of_pd:1;
		unsigned int res1:1;
		unsigned int wdr_write_fifo_of_pd:1;
		unsigned int res2:1;
		unsigned int wdr_read_fifo_of_pd:1;
		unsigned int res3:1;
		unsigned int s1_cin_fifo_of_pd:1;
		unsigned int lca_rgb_fifo_r_emp_pd:1;
		unsigned int lca_rgb_fifo_w_full_pd:1;
		unsigned int lca_by_fifo_r_emp_pd:1;
		unsigned int lca_by_fifo_w_full_pd:1;
		unsigned int d3d_k_fifo_w_full_pd:1;
		unsigned int d3d_raw_fifo_w_full_pd:1;
		unsigned int d3d_k_fifo_r_emp_pd:1;
		unsigned int d3d_ref_fifo_r_emp_pd:1;
		unsigned int d3d_ltf_fifo_r_emp_pd:1;
		unsigned int res4:7;
	} bits;
} ISP_INTER_STATUS0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int top_ctrl_st:11;
		unsigned int res0:5;
		unsigned int res1:16;
	} bits;
} ISP_INTER_STATUS1_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int ahb_ddr_w_addr:12;
		unsigned int res0:20;
	} bits;
} ISP_WR_ADDR_RET_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_ctrl_st:8;
		unsigned int res0:24;
	} bits;
} ISP_INTER_STATUS2_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_lbc_dec_err:5;
		unsigned int res0:3;
		unsigned int d3d_k_lbc_dec_err:5;
		unsigned int res1:3;
		unsigned int d3d_ref_lbc_dec_err:5;
		unsigned int res2:3;
		unsigned int d3d_ltf_lbc_dec_err:5;
		unsigned int res3:3;
	} bits;
} ISP_LBC_INTER_STATUS_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int small_ver:12;
		unsigned int big_ver:12;
		unsigned int res0:8;
	} bits;
} ISP_VER_CFG_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int max_width:16;
		unsigned int max_height:16;
	} bits;
} ISP_MAX_SIZE_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int d3d_k_fifo_max_layer:16;
		unsigned int d3d_raw_fifo_max_layer:16;
	} bits;
} ISP_COMP_FIFO_MAX_LAYER0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_fifo_max_layer:16;
		unsigned int res0:16;
	} bits;
} ISP_COMP_FIFO_MAX_LAYER1_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_fifo_max_layer:16;
		unsigned int d3d_ltf_fifo_max_layer:16;
	} bits;
} ISP_UNCOMP_FIFO_MAX_LAYER0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int d3d_k_fifo_max_layer:16;
		unsigned int d3d_ref_fifo_max_layer:16;
	} bits;
} ISP_UNCOMP_FIFO_MAX_LAYER1_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_cmp_bandwidth;
	} bits;
} ISP_WDR_CMP_BANDWIDTH_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_decmp_bandwidth;
	} bits;
} ISP_WDR_DECMP_BANDWIDTH_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int d3d_cmp_bandwidth;
	} bits;
} ISP_D3D_CMP_BANDWIDTH_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int d3d_decmp_bandwidth;
	} bits;
} ISP_D3D_DECMP_BANDWIDTH_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int input_width:14;
		unsigned int res0:2;
		unsigned int input_height:14;
		unsigned int res1:2;
	} bits;
} ISP_S0_FMERR_CNT_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int input_width:14;
		unsigned int res0:2;
		unsigned int input_height:14;
		unsigned int res1:2;
	} bits;
} ISP_S1_FMERR_CNT_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int hb_min:16;
		unsigned int hb_max:16;
	} bits;
} ISP_S0_HB_CNT_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int hb_min:16;
		unsigned int hb_max:16;
	} bits;
} ISP_S1_HB_CNT_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int comp_overflow_line:14;
		unsigned int res0:2;
		unsigned int decomp_overflow_line:14;
		unsigned int res1:2;
	} bits;
} ISP_WDR_FIFO_OVERFLOW_LINE_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int comp_overflow_line:14;
		unsigned int res0:2;
		unsigned int decomp_overflow_line:14;
		unsigned int res1:2;
	} bits;
} ISP_D3D_FIFO_OVERFLOW_LINE_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int wdr_dma_len:11;
		unsigned int res0:21;
	} bits;
} ISP_WDR_DMA_STRIDE_LEN_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int d3d_ref_raw_dma_len:11;
		unsigned int res0:5;
		unsigned int d3d_ref_k_dma_len:11;
		unsigned int res1:5;
	} bits;
} ISP_D3D_DMA_STRIDE_LEN_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int pltm_mid_test:1;
		unsigned int pltm_read_finish:1;
		unsigned int res0:30;
	} bits;
} ISP_SIM_CTRL_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int isp0_max_width:14;
		unsigned int res0:2;
		unsigned int isp_mode:1;
		unsigned int res1:15;
	} bits;
} ISP_TOP_CTRL_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int s1_blc_en:1;
		unsigned int s1_lc_en:1;
		unsigned int s1_dg_en:1;
		unsigned int res0:29;
	} bits;
} ISP_S1_CFG_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int ae_en:1;
		unsigned int lc_en:1;
		unsigned int wdr_en:1;
		unsigned int dpc_en:1;
		unsigned int d2d_en:1;
		unsigned int d3d_en:1;
		unsigned int awb_en:1;
		unsigned int wb_en:1;
		unsigned int lsc_en:1;
		unsigned int bgc_en:1;
		unsigned int sharp_en:1;
		unsigned int af_en:1;
		unsigned int rgb2rgb_en:1;
		unsigned int rgb_drc_en:1;
		unsigned int pltm_en:1;
		unsigned int cem_en:1;
		unsigned int afs_en:1;
		unsigned int hist_en:1;
		unsigned int blc_en:1;
		unsigned int dg_en:1;
		unsigned int so_en:1;
		unsigned int ctc_en:1;
		unsigned int res0:1;
		unsigned int cnr_en:1;
		unsigned int satu_en:1;
		unsigned int dehaze_en:1;
		unsigned int res1:6;
	} bits;
} ISP_MODULE_BYPASS0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int res0:1;
		unsigned int satu_mode:1;
		unsigned int demosaic_mode:1;
		unsigned int res1:5;
		unsigned int hist_mode:2;
		unsigned int res2:6;
		unsigned int rsc_mode:2;
		unsigned int msc_mode:3;
		unsigned int res3:11;
	} bits;
} ISP_MODULE_MODE0_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int input_width:14;
		unsigned int res0:2;
		unsigned int input_height:14;
		unsigned int res1:2;
	} bits;
} ISP_INPUT_SIZE_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int valid_width:14;
		unsigned int res0:2;
		unsigned int valid_height:14;
		unsigned int res1:2;
	} bits;
} ISP_VALID_SIZE_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int valid_hor_start:13;
		unsigned int res0:3;
		unsigned int valid_ver_start:13;
		unsigned int res1:3;
	} bits;
} ISP_VALID_START_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int isp_index:1;
		unsigned int res0:31;
	} bits;
} ISP_MODULE_STATUS_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int line_tar_bits:16;
		unsigned int lmtqp_min:4;
		unsigned int mb_min_bit:9;
		unsigned int res0:1;
		unsigned int lmtqp_en:1;
		unsigned int is_lossy:1;
	} bits;
} ISP_WDR_RAW_LBC_CTRL_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int line_tar_bits:16;
		unsigned int lmtqp_min:4;
		unsigned int mb_min_bit:9;
		unsigned int res0:1;
		unsigned int lmtqp_en:1;
		unsigned int is_lossy:1;
	} bits;
} ISP_D3D_RAW_LBC_CTRL_REG_t;

typedef union {
	unsigned int dwval;
	struct {
		unsigned int line_tar_bits:16;
		unsigned int res0:1;
		unsigned int lmtqp_min:3;
		unsigned int mb_min_bit:7;
		unsigned int res1:3;
		unsigned int lmtqp_en:1;
		unsigned int is_lossy:1;
	} bits;
} ISP_D3D_K_LBC_CTRL_REG_t;

#endif /* _ISP522_REG_H_ */
