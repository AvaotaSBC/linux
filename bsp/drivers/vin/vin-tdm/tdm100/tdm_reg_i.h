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

#ifndef __CSIC__TDM__REG__I__H__
#define __CSIC__TDM__REG__I__H__

/*
 * Detail information of registers
 */
/*
 * tdm top registers
 */
#define TMD_BASE_ADDR				0x02108000

#define TDM_GOLBAL_CFG0_REG_OFF			0X000
#define TDM_TOP_EN				0
#define TDM_TOP_EN_MASK				(0X1 << TDM_TOP_EN)
#define TDM_EN					1
#define TDM_EN_MASK				(0X1 << TDM_EN)
#define VGM_EN					3
#define VGM_EN_MASK				(0X1 << VGM_EN)
#define TDM_RX_FIFO_MAX_LAYER_EN		4
#define TDM_RX_FIFO_MAX_LAYER_EN_MASK		(0X1 << TDM_RX_FIFO_MAX_LAYER_EN)
#define MODULE_CLK_BACK_DOOR			27
#define MODULE_CLK_BACK_DOOR_MASK		(0X1 << MODULE_CLK_BACK_DOOR)

#define TDM_INT_BYPASS0_REG_OFF			0X010

#define TDM_INT_STATUS0_REG_OFF			0X018
#define RX_FRM_LOST_PD				0
#define RX_FRM_LOST_PD_MASK			(0X1 << RX_FRM_LOST_PD)
#define RX_FRM_ERR_PD				1
#define RX_FRM_ERR_PD_MASK			(0X1 << RX_FRM_ERR_PD)
#define RX_BTYPE_ERR_PD				2
#define RX_BTYPE_ERR_PD_MASK			(0X1 << RX_BTYPE_ERR_PD)
#define RX_BUF_FULL_PD				3
#define RX_BUF_FULL_PD_MASK			(0X1 << RX_BUF_FULL_PD)
#define RX_COMP_ERR_PD				4
#define RX_COMP_ERR_PD_MASK			(0X1 << RX_COMP_ERR_PD)
#define RX_HB_SHORT_PD				5
#define RX_HB_SHORT_PD_MASK			(0X1 << RX_HB_SHORT_PD)
#define RX_FIFO_FULL_PD				6
#define RX_FIFO_FULL_PD_MASK			(0X1 << RX_FIFO_FULL_PD)
#define RX0_FRM_DONE_PD				16
#define RX0_FRM_DONE_PD_MASK			(0X1 << RX0_FRM_DONE_PD)
#define RX1_FRM_DONE_PD				17
#define RX1_FRM_DONE_PD_MASK			(0X1 << RX1_FRM_DONE_PD)

#define TDM_INTERNAL_STATUS0_REG_OFF		0X020
#define RX0_FRM_LOST_PD				(1 << 0)
#define RX1_FRM_LOST_PD				(1 << 1)
#define RX0_FRM_ERR_PD				(1 << 8)
#define RX1_FRM_ERR_PD				(1 << 9)
#define RX0_BTYPE_ERR_PD			(1 << 16)
#define RX1_BTYPE_ERR_PD			(1 << 17)
#define RX0_BUF_FULL_PD				(1 << 24)
#define RX1_BUF_FULL_PD				(1 << 25)

#define TDM_INTERNAL_STATUS1_REG_OFF		0X024
#define RX0_COMP_ERR_PD				(1 << 0)
#define RX1_COMP_ERR_PD				(1 << 1)
#define RX0_HB_SHORT_PD				(1 << 8)
#define RX1_HB_SHORT_PD				(1 << 9)
#define RX0_FIFO_FULL_PD			(1 << 16)
#define RX1_FIFO_FULL_PD			(1 << 17)

/*
 * tdm tx registers
 */
#define TMD_TX_OFFSET				0x0a0

#define TDM_TX_CFG0_REG_OFF			0X000
#define TDM_TX_CAP_EN				0
#define TDM_TX_CAP_EN_MASK			(0X1 << TDM_TX_CAP_EN)
#define TDM_TX_OMODE				1
#define TDM_TX_OMODE_MASK			(0X1 << TDM_TX_OMODE)

#define TDM_TX_CFG1_REG_OFF			0X004
#define TDM_TX_H_BLANK				0
#define TDM_TX_H_BLANK_MASK			(0X3FFF << TDM_TX_H_BLANK)

#define TDM_TX_CFG2_REG_OFF			0X008
#define TDM_TX_V_BLANK_FE			0
#define TDM_TX_V_BLANK_FE_MASK			(0X3FFF << TDM_TX_V_BLANK_FE)
#define TDM_TX_V_BLANK_BE			16
#define TDM_TX_V_BLANK_BE_MASK			(0X3FFF << TDM_TX_V_BLANK_BE)

/*
 * tdm rx registers
 */
#define TMD_RX0_OFFSET				0x100
#define TMD_RX1_OFFSET				0x140
#define AMONG_RX_OFFSET				0x40

#define TDM_RX_CFG0_REG_OFF			0X000
#define TDM_RX_EN				0
#define TDM_RX_EN_MASK				(0X1 << TDM_RX_EN)
#define TDM_RX_CAP_EN				1
#define TDM_RX_CAP_EN_MASK			(0X1 << TDM_RX_CAP_EN)
#define TDM_RX_ABD_EN				2
#define TDM_RX_ABD_EN_MASK			(0X1 << TDM_RX_ABD_EN)
#define TDM_RX_BUF_NUM				8
#define TDM_RX_BUF_NUM_MASK			(0XF << TDM_RX_BUF_NUM)
#define TDM_RX_CH0_EN				16
#define TDM_RX_CH0_EN_MASK			(0X1 << TDM_RX_CH0_EN)
#define TDM_RX_MIN_DDR_SIZE			24
#define TDM_RX_MIN_DDR_SIZE_MASK		(0X3 << TDM_RX_MIN_DDR_SIZE)
#define TDM_INPUT_BIT				28
#define TDM_INPUT_BIT_MASK			(0X3 << TDM_INPUT_BIT)

#define TDM_RX_CFG1_REG_OFF			0X004
#define TDM_RX_WIDTH				0
#define TDM_RX_WIDTH_MASK			(0X3FFF << TDM_RX_WIDTH)
#define TDM_RX_HEIGHT				16
#define TDM_RX_HEIGHT_MASK			(0X3FFF << TDM_RX_HEIGHT)

#define TDM_RX_CFG2_REG_OFF			0X010
#define TDM_RX_ADDR				0
#define TDM_RX_ADDR_MASK			(0XFFFFFFFF << TDM_RX_ADDR)

#define TDM_RX_FRAME_ERR_REG_OFF		0X020
#define TDM_RX_HB_SHORT_REG_OFF			0X024

#endif /* __CSIC__TDM__REG__I__H__ */
