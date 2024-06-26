/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * combo rx module
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "combo_rx_reg_i.h"

#ifndef __COMBO_RX_REG__H__
#define __COMBO_RX_REG__H__

enum combo_rx_mode_sel {
	D_PHY      = 0x2,
	SUB_LVDS   = 0x3,
	CMOS       = 0x4,
};

enum combo_rx_pix_num {
	ONE_PIXEL    = 0x0,
	TWO_PIXEL    = 0x1,
	FOUR_PIXEL   = 0x2,
	EIGHT_PIXEL  = 0x3,
};

enum mipi_inter_lane0_map {
	MIPI_IN_L0_USE_PAD_LANE0     = 0x0,
	MIPI_IN_L0_USE_PAD_LANE1     = 0x1,
	MIPI_IN_L0_USE_PAD_LANE2     = 0x2,
	MIPI_IN_L0_USE_PAD_LANE3     = 0x3,
};

enum mipi_inter_lane1_map {
	MIPI_IN_L1_USE_PAD_LANE0     = 0x0,
	MIPI_IN_L1_USE_PAD_LANE1     = 0x1,
	MIPI_IN_L1_USE_PAD_LANE2     = 0x2,
	MIPI_IN_L1_USE_PAD_LANE3     = 0x3,
};

enum mipi_inter_lane2_map {
	MIPI_IN_L2_USE_PAD_LANE0     = 0x0,
	MIPI_IN_L2_USE_PAD_LANE1     = 0x1,
	MIPI_IN_L2_USE_PAD_LANE2     = 0x2,
	MIPI_IN_L2_USE_PAD_LANE3     = 0x3,
};

enum mipi_inter_lane3_map {
	MIPI_IN_L3_USE_PAD_LANE0     = 0x0,
	MIPI_IN_L3_USE_PAD_LANE1     = 0x1,
	MIPI_IN_L3_USE_PAD_LANE2     = 0x2,
	MIPI_IN_L3_USE_PAD_LANE3     = 0x3,
};

enum lvds_bit_width {
	RAW8     = 0x0,
	RAW10    = 0x1,
	RAW12    = 0x2,
	RAW14    = 0x3,
	RAW16    = 0x4,
	YUV8     = 0x5,
	YUV10    = 0x6,
};

enum lvds_lane_num {
	LVDS_2LANE     = 0x2,
	LVDS_4LANE     = 0x4,
	LVDS_8LANE     = 0x8,
	LVDS_10LANE    = 0x0a,
	LVDS_12LANE    = 0x0c,
};

enum mipi_lane_num {
	MIPI_1LANE     = 0x0,
	MIPI_2LANE     = 0x1,
	MIPI_3LANE     = 0x2,
	MIPI_4LANE     = 0x3,

};

enum hispi_trans_mode {
	PACKETIZED_SP    = 0x0,
	STREAMING_SP     = 0x1,
};

struct lvds_ctr {
	enum lvds_bit_width lvds_bit_width;
	enum lvds_lane_num lvds_lane_num;
	unsigned int lvds_line_code_mode;/* 0:HiSPI SOF/EOF/SOL/EOL 1:SAV-EAV */
	unsigned int lvds_pix_lsb;/* 0:MSB,1:LSB */
	unsigned int lvds_wdr_lbl_sel;/* 0:normal operation 1:sync code 2:detect data */
	unsigned int lvds_sync_code_line_cnt;/* when in WDR mode,this reg can extent frame valid signal by set 1,2,3,4 */
	unsigned int lvds_wdr_fid_mode_sel;/* 0:1bit 1:2bits */
	unsigned int lvds_wdr_fid_map_en;/* bit12:FID0 bit13:FID1 bit14:FID2 bit15:FID3 */
	unsigned int lvds_wdr_fid0_map_sel;
	unsigned int lvds_wdr_fid1_map_sel;
	unsigned int lvds_wdr_fid2_map_sel;
	unsigned int lvds_wdr_fid3_map_sel;
	unsigned int lvds_code_mask;/* set 1,mask this bit,sync code ignore this bi t */
	unsigned int lvds_wdr_en_multi_ch;
	unsigned int lvds_wdr_ch0_height;
	unsigned int lvds_wdr_ch1_height;
	unsigned int lvds_wdr_ch2_height;
	unsigned int lvds_wdr_ch3_height;
};

struct mipi_ctr {
	enum mipi_lane_num mipi_lane_num;
	unsigned int mipi_msb_lsb_sel;/* PHA to controller MSB first */
	unsigned int mipi_wdr_mode_sel;/* 0:normal operation 1:sync code 2:detect data,when in DOL WDR mode,set 2 */
	unsigned int mipi_open_multi_ch;
	unsigned int mipi_ch0_height;
	unsigned int mipi_ch1_height;
	unsigned int mipi_ch2_height;
	unsigned int mipi_ch3_height;
};

struct hispi_ctr {
	enum hispi_trans_mode hispi_trans_mode;
	unsigned int hispi_wdr_en;
	unsigned int hispi_normal;/* switch lvds to  hispi normal mode */
	unsigned int hispi_wdr_eof_fild;
	unsigned int hispi_wdr_sof_fild;
	unsigned int hispi_code_mask;
};

enum lvds_inter_lane0_map {
	LVDS_LANE0_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE0     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE0     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE0     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE0     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE0     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE0     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE0     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE0     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE0     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE0     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE0     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE0     = 0xc,
};

enum lvds_inter_lane1_map {
	LVDS_LANE1_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE1     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE1     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE1     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE1     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE1     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE1     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE1     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE1     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE1     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE1     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE1     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE1     = 0xc,
};

enum lvds_inter_lane2_map {
	LVDS_LANE2_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE2     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE2     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE2     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE2     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE2     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE2     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE2     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE2     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE2     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE2     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE2     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE2     = 0xc,
};

enum lvds_inter_lane3_map {
	LVDS_LANE3_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE3     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE3     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE3     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE3     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE3     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE3     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE3     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE3     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE3     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE3     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE3     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE3     = 0xc,
};

enum lvds_inter_lane4_map {
	LVDS_LANE4_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE4     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE4     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE4     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE4     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE4     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE4     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE4     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE4     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE4     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE4     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE4     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE4     = 0xc,
};

enum lvds_inter_lane5_map {
	LVDS_LANE5_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE5     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE5     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE5     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE5     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE5     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE5     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE5     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE5     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE5     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE5     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE5     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE5     = 0xc,
};

enum lvds_inter_lane6_map {
	LVDS_LANE6_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE6     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE6     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE6     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE6     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE6     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE6     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE6     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE6     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE6     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE6     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE6     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE6     = 0xc,
};

enum lvds_inter_lane7_map {
	LVDS_LANE7_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE7     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE7     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE7     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE7     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE7     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE7     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE7     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE7     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE7     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE7     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE7     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE7     = 0xc,
};

enum lvds_inter_lane8_map {
	LVDS_LANE8_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE8     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE8     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE8     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE8     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE8     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE8     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE8     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE8     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE8     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE8     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE8     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE8     = 0xc,
};

enum lvds_inter_lane9_map {
	LVDS_LANE9_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE9     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE9     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE9     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE9     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE9     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE9     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE9     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE9     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE9     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE9     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE9     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE9     = 0xc,
};

enum lvds_inter_lane10_map {
	LVDS_LANE10_NO_USE             = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE10     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE10     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE10     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE10     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE10     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE10     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE10     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE10     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE10     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE10     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE10     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE10     = 0xc,
};

enum lvds_inter_lane11_map {
	LVDS_LANE11_NO_USE              = 0x0,
	LVDS_MAPPING_A_D0_TO_LANE11     = 0x1,
	LVDS_MAPPING_A_D1_TO_LANE11     = 0x2,
	LVDS_MAPPING_A_D2_TO_LANE11     = 0x3,
	LVDS_MAPPING_A_D3_TO_LANE11     = 0x4,
	LVDS_MAPPING_B_D0_TO_LANE11     = 0x5,
	LVDS_MAPPING_B_D1_TO_LANE11     = 0x6,
	LVDS_MAPPING_B_D2_TO_LANE11     = 0x7,
	LVDS_MAPPING_B_D3_TO_LANE11     = 0x8,
	LVDS_MAPPING_C_D0_TO_LANE11     = 0x9,
	LVDS_MAPPING_C_D1_TO_LANE11     = 0xa,
	LVDS_MAPPING_C_D2_TO_LANE11     = 0xb,
	LVDS_MAPPING_C_D3_TO_LANE11     = 0xc,
};

enum phya_a_clk_dly_set {
	A_CLK_DELAY_0     = 0x0,
	A_CLK_DELAY_1     = 0x1,
	A_CLK_DELAY_2     = 0x2,
	A_CLK_DELAY_3     = 0x3,
	A_CLK_DELAY_4     = 0x4,
	A_CLK_DELAY_5     = 0x5,
	A_CLK_DELAY_6     = 0x6,
	A_CLK_DELAY_7     = 0x7,
};

enum phya_b_clk_dly_set {
	B_CLK_DELAY_0     = 0x0,
	B_CLK_DELAY_1     = 0x1,
	B_CLK_DELAY_2     = 0x2,
	B_CLK_DELAY_3     = 0x3,
	B_CLK_DELAY_4     = 0x4,
	B_CLK_DELAY_5     = 0x5,
	B_CLK_DELAY_6     = 0x6,
	B_CLK_DELAY_7     = 0x7,
};

enum phya_c_clk_dly_set {
	C_CLK_DELAY_0     = 0x0,
	C_CLK_DELAY_1     = 0x1,
	C_CLK_DELAY_2     = 0x2,
	C_CLK_DELAY_3     = 0x3,
	C_CLK_DELAY_4     = 0x4,
	C_CLK_DELAY_5     = 0x5,
	C_CLK_DELAY_6     = 0x6,
	C_CLK_DELAY_7     = 0x7,
};

enum phya_a_d0_dly_set {
	A_D0_DELAY_0     = 0x0,
	A_D0_DELAY_1     = 0x1,
	A_D0_DELAY_2     = 0x2,
	A_D0_DELAY_3     = 0x3,
	A_D0_DELAY_4     = 0x4,
	A_D0_DELAY_5     = 0x5,
	A_D0_DELAY_6     = 0x6,
	A_D0_DELAY_7     = 0x7,
};

enum phya_b_d0_dly_set {
	B_D0_DELAY_0     = 0x0,
	B_D0_DELAY_1     = 0x1,
	B_D0_DELAY_2     = 0x2,
	B_D0_DELAY_3     = 0x3,
	B_D0_DELAY_4     = 0x4,
	B_D0_DELAY_5     = 0x5,
	B_D0_DELAY_6     = 0x6,
	B_D0_DELAY_7     = 0x7,
};

enum phya_c_d0_dly_set {
	C_D0_DELAY_0     = 0x0,
	C_D0_DELAY_1     = 0x1,
	C_D0_DELAY_2     = 0x2,
	C_D0_DELAY_3     = 0x3,
	C_D0_DELAY_4     = 0x4,
	C_D0_DELAY_5     = 0x5,
	C_D0_DELAY_6     = 0x6,
	C_D0_DELAY_7     = 0x7,
};

enum phya_a_d1_dly_set {
	A_D1_DELAY_0     = 0x0,
	A_D1_DELAY_1     = 0x1,
	A_D1_DELAY_2     = 0x2,
	A_D1_DELAY_3     = 0x3,
	A_D1_DELAY_4     = 0x4,
	A_D1_DELAY_5     = 0x5,
	A_D1_DELAY_6     = 0x6,
	A_D1_DELAY_7     = 0x7,
};

enum phya_b_d1_dly_set {
	B_D1_DELAY_0     = 0x0,
	B_D1_DELAY_1     = 0x1,
	B_D1_DELAY_2     = 0x2,
	B_D1_DELAY_3     = 0x3,
	B_D1_DELAY_4     = 0x4,
	B_D1_DELAY_5     = 0x5,
	B_D1_DELAY_6     = 0x6,
	B_D1_DELAY_7     = 0x7,
};

enum phya_c_d1_dly_set {
	C_D1_DELAY_0     = 0x0,
	C_D1_DELAY_1     = 0x1,
	C_D1_DELAY_2     = 0x2,
	C_D1_DELAY_3     = 0x3,
	C_D1_DELAY_4     = 0x4,
	C_D1_DELAY_5     = 0x5,
	C_D1_DELAY_6     = 0x6,
	C_D1_DELAY_7     = 0x7,
};

enum phya_a_d2_dly_set {
	A_D2_DELAY_0     = 0x0,
	A_D2_DELAY_1     = 0x1,
	A_D2_DELAY_2     = 0x2,
	A_D2_DELAY_3     = 0x3,
	A_D2_DELAY_4     = 0x4,
	A_D2_DELAY_5     = 0x5,
	A_D2_DELAY_6     = 0x6,
	A_D2_DELAY_7     = 0x7,
};

enum phya_b_d2_dly_set {
	B_D2_DELAY_0     = 0x0,
	B_D2_DELAY_1     = 0x1,
	B_D2_DELAY_2     = 0x2,
	B_D2_DELAY_3     = 0x3,
	B_D2_DELAY_4     = 0x4,
	B_D2_DELAY_5     = 0x5,
	B_D2_DELAY_6     = 0x6,
	B_D2_DELAY_7     = 0x7,
};

enum phya_c_d2_dly_set {
	C_D2_DELAY_0     = 0x0,
	C_D2_DELAY_1     = 0x1,
	C_D2_DELAY_2     = 0x2,
	C_D2_DELAY_3     = 0x3,
	C_D2_DELAY_4     = 0x4,
	C_D2_DELAY_5     = 0x5,
	C_D2_DELAY_6     = 0x6,
	C_D2_DELAY_7     = 0x7,
};

enum phya_a_d3_dly_set {
	A_D3_DELAY_0     = 0x0,
	A_D3_DELAY_1     = 0x1,
	A_D3_DELAY_2     = 0x2,
	A_D3_DELAY_3     = 0x3,
	A_D3_DELAY_4     = 0x4,
	A_D3_DELAY_5     = 0x5,
	A_D3_DELAY_6     = 0x6,
	A_D3_DELAY_7     = 0x7,
};

enum phya_b_d3_dly_set {
	B_D3_DELAY_0     = 0x0,
	B_D3_DELAY_1     = 0x1,
	B_D3_DELAY_2     = 0x2,
	B_D3_DELAY_3     = 0x3,
	B_D3_DELAY_4     = 0x4,
	B_D3_DELAY_5     = 0x5,
	B_D3_DELAY_6     = 0x6,
	B_D3_DELAY_7     = 0x7,
};

enum phya_c_d3_dly_set {
	C_D3_DELAY_0     = 0x0,
	C_D3_DELAY_1     = 0x1,
	C_D3_DELAY_2     = 0x2,
	C_D3_DELAY_3     = 0x3,
	C_D3_DELAY_4     = 0x4,
	C_D3_DELAY_5     = 0x5,
	C_D3_DELAY_6     = 0x6,
	C_D3_DELAY_7     = 0x7,
};

struct phya_signal_dly_ctr {
	enum phya_a_clk_dly_set a_clk_dly;
	enum phya_b_clk_dly_set b_clk_dly;
	enum phya_c_clk_dly_set c_clk_dly;

	enum phya_a_d0_dly_set a_d0_dly;
	enum phya_b_d0_dly_set b_d0_dly;
	enum phya_c_d0_dly_set c_d0_dly;

	enum phya_a_d1_dly_set a_d1_dly;
	enum phya_b_d1_dly_set b_d1_dly;
	enum phya_c_d1_dly_set c_d1_dly;

	enum phya_a_d2_dly_set a_d2_dly;
	enum phya_b_d2_dly_set b_d2_dly;
	enum phya_c_d2_dly_set c_d2_dly;

	enum phya_a_d3_dly_set a_d3_dly;
	enum phya_b_d3_dly_set b_d3_dly;
	enum phya_c_d3_dly_set c_d3_dly;
};


struct combo_lane_map {
	enum lvds_inter_lane0_map lvds_lane0;
	enum lvds_inter_lane1_map lvds_lane1;
	enum lvds_inter_lane2_map lvds_lane2;
	enum lvds_inter_lane3_map lvds_lane3;
	enum lvds_inter_lane4_map lvds_lane4;
	enum lvds_inter_lane5_map lvds_lane5;
	enum lvds_inter_lane6_map lvds_lane6;
	enum lvds_inter_lane7_map lvds_lane7;
	enum lvds_inter_lane8_map lvds_lane8;
	enum lvds_inter_lane9_map lvds_lane9;
	enum lvds_inter_lane10_map lvds_lane10;
	enum lvds_inter_lane11_map lvds_lane11;
};

struct mipi_lane_map {
	enum mipi_inter_lane0_map mipi_lane0;
	enum mipi_inter_lane1_map mipi_lane1;
	enum mipi_inter_lane2_map mipi_lane2;
	enum mipi_inter_lane3_map mipi_lane3;
};

struct lane_code {
	unsigned int  high_bit;
	unsigned int  low_bit;
};

struct combo_sync_code {
	struct lane_code lane_sof[12];
	struct lane_code lane_eof[12];
	struct lane_code lane_sol[12];
	struct lane_code lane_eol[12];
};

int cmb_rx_set_base_addr(unsigned int sel, unsigned long addr);

void cmb_rx_phya_a_d0_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_b_d0_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_c_d0_en(unsigned int sel, unsigned int en);

void cmb_rx_phya_a_d1_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_b_d1_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_c_d1_en(unsigned int sel, unsigned int en);

void cmb_rx_phya_a_d2_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_b_d2_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_c_d2_en(unsigned int sel, unsigned int en);

void cmb_rx_phya_a_d3_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_b_d3_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_c_d3_en(unsigned int sel, unsigned int en);

void cmb_rx_phya_a_ck_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_b_ck_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_c_ck_en(unsigned int sel, unsigned int en);
void cmb_rx_te_auto_disable(unsigned int sel, unsigned int en);

void cmb_rx_phya_signal_dly_en(unsigned int sel, unsigned int en);
void cmb_rx_phya_signal_dly_ctr(unsigned int sel, struct phya_signal_dly_ctr *phya_signal_dly);

void cmb_rx_phya_config(unsigned int sel);
void cmb_rx_phya_ck_mode(unsigned int sel, unsigned int mode);
void cmb_rx_phya_ck_pol(unsigned int sel, unsigned int pol);
void cmb_rx_phya_offset(unsigned int sel, unsigned int offset);

void cmb_rx_enable(unsigned int sel);

void cmb_rx_disable(unsigned int sel);

void cmb_rx_mode_sel(unsigned int sel, enum combo_rx_mode_sel mode);

void cmb_rx_app_pixel_out(unsigned int sel, enum combo_rx_pix_num pix_num);

void cmb_rx_mipi_ctr(unsigned int sel, struct mipi_ctr *mipi_ctr);
void cmb_rx_mipi_stl_time(unsigned int sel, unsigned char time_hs);
void cmb_rx_mipi_dphy_mapping(unsigned int sel, struct mipi_lane_map *mipi_map);

void cmb_rx_mipi_csi2_status(unsigned int sel, unsigned int *mipi_csi2_status);

void cmb_rx_mipi_csi2_data_id(unsigned int sel, unsigned int *mipi_csi2_data_id);

void cmb_rx_mipi_csi2_word_cnt(unsigned int sel, unsigned int *mipi_csi2_word_cnt);

void cmb_rx_mipi_csi2_ecc_val(unsigned int sel, unsigned int *mipi_csi2_ecc_val);

void cmb_rx_mipi_csi2_line_lentgh(unsigned int sel, unsigned int *mipi_csi2_line_lentgh);

void cmb_rx_mipi_csi2_rcv_cnt(unsigned int sel, unsigned int *mipi_csi2_rcv_cnt);

void cmb_rx_mipi_csi2_ecc_err_cnt(unsigned int sel, unsigned int *mipi_csi2_ecc_err_cnt);

void cmb_rx_mipi_csi2_check_sum_err_cnt(unsigned int sel, unsigned int *mipi_csi2_check_sum_err_cnt);

void cmb_rx_lvds_ctr(unsigned int sel, struct lvds_ctr *lvds_ctr);

void cmb_rx_lvds_mapping(unsigned int sel, struct combo_lane_map *lvds_map);

void cmb_rx_lvds_sync_code(unsigned int sel, struct combo_sync_code *lvds_sync_code);

void cmb_rx_hispi_ctr(unsigned int sel, struct hispi_ctr *hispi_ctr);

#endif /* __COMBO_REG__H__ */
