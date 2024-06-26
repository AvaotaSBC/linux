/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 *
 * Copyright (c) 2007-2019 Allwinnertech Co., Ltd.
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

#include <linux/kernel.h>
#include "tdm_reg.h"

#include "../../utility/vin_io.h"
#include "../../platform/platform_cfg.h"

volatile void __iomem *csic_tdm_base[VIN_MAX_TDM];

#define TDM_ADDR_BIT_R_SHIFT 2

int csic_tdm_set_base_addr(unsigned int sel, unsigned long addr)
{
	if (sel > VIN_MAX_TDM - 1)
		return -1;
	csic_tdm_base[sel] = (volatile void __iomem *)addr;

	return 0;
}

/*
 * function about tdm top registers
 */
void csic_tdm_top_enable(unsigned int sel)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TDM_GOLBAL_CFG0_REG_OFF,
			TDM_TOP_EN_MASK, 1 << TDM_TOP_EN);
}

void csic_tdm_top_disable(unsigned int sel)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TDM_GOLBAL_CFG0_REG_OFF,
			TDM_TOP_EN_MASK, 0 << TDM_TOP_EN);
}

void csic_tdm_enable(unsigned int sel)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TDM_GOLBAL_CFG0_REG_OFF,
			TDM_EN_MASK, 1 << TDM_EN);
}

void csic_tdm_fifo_max_layer_en(unsigned int sel, unsigned int en)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TDM_GOLBAL_CFG0_REG_OFF,
			TDM_RX_FIFO_MAX_LAYER_EN_MASK, 1 << TDM_RX_FIFO_MAX_LAYER_EN);
}

void csic_tdm_disable(unsigned int sel)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TDM_GOLBAL_CFG0_REG_OFF,
			TDM_EN_MASK, 0 << TDM_EN);
}

void csic_tdm_int_enable(unsigned int sel,	enum tdm_int_sel interrupt)
{
	vin_reg_set(csic_tdm_base[sel] + TDM_INT_BYPASS0_REG_OFF, interrupt);
}

void csic_tdm_int_disable(unsigned int sel, enum tdm_int_sel interrupt)
{
	vin_reg_clr(csic_tdm_base[sel] + TDM_INT_BYPASS0_REG_OFF, interrupt);
}

void csic_tdm_int_get_status(unsigned int sel, struct tdm_int_status *status)
{
	unsigned int reg_val = vin_reg_readl(csic_tdm_base[sel] + TDM_INT_STATUS0_REG_OFF);

	status->rx_frm_lost = (reg_val & RX_FRM_LOST_PD_MASK) >> RX_FRM_LOST_PD;
	status->rx_frm_err = (reg_val & RX_FRM_ERR_PD_MASK) >> RX_FRM_ERR_PD;
	status->rx_btype_err = (reg_val & RX_BTYPE_ERR_PD_MASK) >> RX_BTYPE_ERR_PD;
	status->rx_buf_full = (reg_val & RX_BUF_FULL_PD_MASK) >> RX_BUF_FULL_PD;
	status->rx_comp_err = (reg_val & RX_COMP_ERR_PD_MASK) >> RX_COMP_ERR_PD;
	status->rx_hb_short = (reg_val & RX_HB_SHORT_PD_MASK) >> RX_HB_SHORT_PD;
	status->rx_fifo_full = (reg_val & RX_FIFO_FULL_PD_MASK) >> RX_FIFO_FULL_PD;
	status->rx0_frm_done = (reg_val & RX0_FRM_DONE_PD_MASK) >> RX0_FRM_DONE_PD;
	status->rx1_frm_done = (reg_val & RX1_FRM_DONE_PD_MASK) >> RX1_FRM_DONE_PD;
}

void csic_tdm_int_clear_status(unsigned int sel, enum tdm_int_sel interrupt)
{
	vin_reg_writel(csic_tdm_base[sel] + TDM_INT_STATUS0_REG_OFF, interrupt);
}

unsigned int csic_tdm_internal_get_status0(unsigned int sel, unsigned int status)
{
	return (vin_reg_readl(csic_tdm_base[sel] + TDM_INTERNAL_STATUS0_REG_OFF)) & status;
}

void csic_tdm_internal_clear_status0(unsigned int sel, unsigned int status)
{
	vin_reg_writel(csic_tdm_base[sel] + TDM_INTERNAL_STATUS0_REG_OFF, status);
}

unsigned int csic_tdm_internal_get_status1(unsigned int sel, unsigned int status)
{
	return (vin_reg_readl(csic_tdm_base[sel] + TDM_INTERNAL_STATUS1_REG_OFF)) & status;
}

void csic_tdm_internal_clear_status1(unsigned int sel, unsigned int status)
{
	vin_reg_writel(csic_tdm_base[sel] + TDM_INTERNAL_STATUS1_REG_OFF, status);
}

/*
 * function about tdm tx registers
 */
void csic_tdm_tx_cap_enable(unsigned int sel)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_TX_OFFSET + TDM_TX_CFG0_REG_OFF,
				TDM_TX_CAP_EN_MASK, 1 << TDM_TX_CAP_EN);
}

void csic_tdm_tx_cap_disable(unsigned int sel)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_TX_OFFSET + TDM_TX_CFG0_REG_OFF,
				TDM_TX_CAP_EN_MASK, 0 << TDM_TX_CAP_EN);
}

void csic_tdm_omode(unsigned int sel, unsigned int mode)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_TX_OFFSET + TDM_TX_CFG0_REG_OFF,
				TDM_TX_OMODE_MASK, mode << TDM_TX_OMODE);
}

void csic_tdm_set_hblank(unsigned int sel, unsigned int hblank)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_TX_OFFSET + TDM_TX_CFG1_REG_OFF,
				TDM_TX_H_BLANK_MASK, hblank << TDM_TX_H_BLANK);
}

void csic_tdm_set_bblank_fe(unsigned int sel, unsigned int bblank_fe)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_TX_OFFSET + TDM_TX_CFG2_REG_OFF,
				TDM_TX_V_BLANK_FE_MASK, bblank_fe << TDM_TX_V_BLANK_FE);
}

void csic_tdm_set_bblank_be(unsigned int sel, unsigned int bblank_be)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_TX_OFFSET + TDM_TX_CFG2_REG_OFF,
				TDM_TX_V_BLANK_BE_MASK, bblank_be << TDM_TX_V_BLANK_BE);
}

/*
 * function about tdm rx registers
 */
void csic_tdm_rx_enable(unsigned int sel, unsigned int ch)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_EN_MASK, 1 << TDM_RX_EN);
}

void csic_tdm_rx_disable(unsigned int sel, unsigned int ch)
{
		vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_EN_MASK, 0 << TDM_RX_EN);
}

void csic_tdm_rx_cap_enable(unsigned int sel, unsigned int ch)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_CAP_EN_MASK, 1 << TDM_RX_CAP_EN);
}

void csic_tdm_rx_cap_disable(unsigned int sel, unsigned int ch)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_CAP_EN_MASK, 0 << TDM_RX_CAP_EN);
}

void csic_tdm_rx_set_buf_num(unsigned int sel, unsigned int ch, unsigned int num)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_BUF_NUM_MASK, num << TDM_RX_BUF_NUM);
}

void csic_tdm_rx_ch0_en(unsigned int sel, unsigned int ch, unsigned int en)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_CH0_EN_MASK, en << TDM_RX_CH0_EN);
}

void csic_tdm_rx_set_min_ddr_size(unsigned int sel, unsigned int ch, enum min_ddr_size_sel ddr_size)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_RX_MIN_DDR_SIZE_MASK, ddr_size << TDM_RX_MIN_DDR_SIZE);
}

void csic_tdm_rx_input_bit(unsigned int sel, unsigned int ch, enum input_image_type_sel input_tpye)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG0_REG_OFF,
					TDM_INPUT_BIT_MASK, input_tpye << TDM_INPUT_BIT);
}

void csic_tdm_rx_input_size(unsigned int sel, unsigned int ch, unsigned int width, unsigned int height)
{
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG1_REG_OFF,
					TDM_RX_WIDTH_MASK, width << TDM_RX_WIDTH);
	vin_reg_clr_set(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG1_REG_OFF,
					TDM_RX_HEIGHT_MASK, height << TDM_RX_HEIGHT);
}

void csic_tdm_rx_set_address(unsigned int sel, unsigned int ch, unsigned long address)
{
	vin_reg_writel(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_CFG2_REG_OFF,
					address >> TDM_ADDR_BIT_R_SHIFT);
}

void csic_tdm_rx_get_size(unsigned int sel, unsigned int ch, unsigned int *width, unsigned int *heigth)
{
	unsigned int regval;

	regval = vin_reg_readl(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_FRAME_ERR_REG_OFF);
	*width = regval & 0x3fff;
	*heigth = (regval >> 16) & 0x3fff;
}

void csic_tdm_rx_get_hblank(unsigned int sel, unsigned int ch, unsigned int *hb_min, unsigned int *hb_max)
{
	unsigned int regval;

	regval = vin_reg_readl(csic_tdm_base[sel] + TMD_RX0_OFFSET + ch*AMONG_RX_OFFSET + TDM_RX_HB_SHORT_REG_OFF);
	*hb_max = regval & 0xffff;
	*hb_min = (regval >> 16) & 0xffff;
}
