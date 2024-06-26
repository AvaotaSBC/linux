/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 *
 * Copyright (c) 2007-2019Allwinnertech Co., Ltd.
 *
 * Authors:  Zheng Zequn <zequnzheng@allwinnertech.com>
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

#ifndef __CSIC__TDM__REG__H__
#define __CSIC__TDM__REG__H__

#include <media/sunxi_camera_v2.h>
#include <linux/types.h>
#include <media/v4l2-mediabus.h>
#include "tdm_reg_i.h"

#define TDM_RX_NUM   2

enum tdm_int_sel {
	RX_FRM_LOST_INT_EN = 0X1,
	RX_FRM_ERR_INT_EN = 0X2,
	RX_BTYPE_ERR_INT_EN = 0X4,
	RX_BUF_FULL_INT_EN = 0X8,
	RX_COMP_ERR_INT_EN = 0X10,
	RX_HB_SHORT_INT_EN = 0X20,
	RX_FIFO_FULL_INT_EN = 0X40,
	RX0_FRM_DONE_INT_EN = 0X10000,
	RX1_TDM_DINE_INT_EN = 0X20000,
	TDM_INT_ALL = 0X307F,
};

enum min_ddr_size_sel {
	DDRSIZE_256b = 0,
	DDRSIZE_512b,
	DDRSIZE_1024b,
	DDRSIZE_2048b,
};

enum input_image_type_sel {
	INPUTTPYE_8BIT = 0,
	INPUTTPYE_10BIT,
	INPUTTPYE_12BIT,
};

enum tdm_input_fmt {
	BAYER_BGGR = 0x4,
	BAYER_RGGB,
	BAYER_GBRG,
	BAYER_GRBG,
};

struct tdm_int_status {
	bool rx_frm_lost;
	bool rx_frm_err;
	bool rx_btype_err;
	bool rx_buf_full;
	bool rx_comp_err;
	bool rx_hb_short;
	bool rx_fifo_full;
	bool rx0_frm_done;
	bool rx1_frm_done;
};

int csic_tdm_set_base_addr(unsigned int sel, unsigned long addr);
void csic_tdm_top_enable(unsigned int sel);
void csic_tdm_top_disable(unsigned int sel);
void csic_tdm_enable(unsigned int sel);
void csic_tdm_disable(unsigned int sel);
void csic_tdm_fifo_max_layer_en(unsigned int sel, unsigned int en);
void csic_tdm_int_enable(unsigned int sel, enum tdm_int_sel interrupt);
void csic_tdm_int_disable(unsigned int sel, enum tdm_int_sel interrupt);
void csic_tdm_int_get_status(unsigned int sel, struct tdm_int_status *status);
void csic_tdm_int_clear_status(unsigned int sel, enum tdm_int_sel interrupt);
unsigned int csic_tdm_internal_get_status0(unsigned int sel, unsigned int status);
void csic_tdm_internal_clear_status0(unsigned int sel, unsigned int status);
unsigned int csic_tdm_internal_get_status1(unsigned int sel, unsigned int status);
void csic_tdm_internal_clear_status1(unsigned int sel, unsigned int status);

void csic_tdm_tx_cap_enable(unsigned int sel);
void csic_tdm_tx_cap_disable(unsigned int sel);
void csic_tdm_omode(unsigned int sel, unsigned int mode);
void csic_tdm_set_hblank(unsigned int sel, unsigned int hblank);
void csic_tdm_set_bblank_fe(unsigned int sel, unsigned int bblank_fe);
void csic_tdm_set_bblank_be(unsigned int sel, unsigned int bblank_be);

void csic_tdm_rx_enable(unsigned int sel, unsigned int ch);
void csic_tdm_rx_disable(unsigned int sel, unsigned int ch);
void csic_tdm_rx_cap_enable(unsigned int sel, unsigned int ch);
void csic_tdm_rx_cap_disable(unsigned int sel, unsigned int ch);
void csic_tdm_rx_set_buf_num(unsigned int sel, unsigned int ch, unsigned int num);
void csic_tdm_rx_ch0_en(unsigned int sel, unsigned int ch, unsigned int en);
void csic_tdm_rx_set_min_ddr_size(unsigned int sel, unsigned int ch, enum min_ddr_size_sel ddr_size);
void csic_tdm_rx_input_bit(unsigned int sel, unsigned int ch, enum input_image_type_sel input_tpye);
void csic_tdm_rx_input_size(unsigned int sel, unsigned int ch, unsigned int width, unsigned int height);
void csic_tdm_rx_set_address(unsigned int sel, unsigned int ch, unsigned long address);
void csic_tdm_rx_get_size(unsigned int sel, unsigned int ch, unsigned int *width, unsigned int *heigth);
void csic_tdm_rx_get_hblank(unsigned int sel, unsigned int ch, unsigned int *hb_min, unsigned int *hb_max);

#endif /* __CSIC__TDM__REG__H__ */
