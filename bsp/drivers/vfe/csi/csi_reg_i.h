/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/csi/csi_reg_i.h
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
 * csi_reg_i.h
 *
 * Hawkview ISP - csi_reg_i.h module
 *
 * Copyright (c) 2014 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Version		  Author         Date				Description
 *
 *   2.0		  Yang Feng	2014/07/15	      Second Version
 *
 ****************************************************************************************
 */
#ifndef __CSI__REG__I__H__
#define __CSI__REG__I__H__

#define u32 unsigned int
/*
 *Detail information of registers
 */
#define CSI_EN_REG_OFF			0X000
#define CSI_EN_REG_CSI_EN		0
#define CSI_EN_REG_PTN_GEN_EN		1
#define CSI_EN_REG_CLK_CNT		2
#define CSI_EN_REG_CLK_CNT_SPL		3
#define CSI_EN_REG_PTN_START		4
#define CSI_EN_REG_RES0			5
#define CSI_EN_REG_VER_EN		30
#define CSI_EN_REG_RES1			31

#define CSI_IF_CFG_REG_OFF			0X004
#define CSI_IF_CFG_REG_CSI_IF			0
#define CSI_IF_CFG_REG_CSI_IF_MASK		(0X1F<<CSI_IF_CFG_REG_CSI_IF)
#define CSI_IF_CFG_REG_RES0			5
#define CSI_IF_CFG_REG_MIPI_IF			7
#define CSI_IF_CFG_REG_MIPI_IF_MASK		(0X1 << CSI_IF_CFG_REG_MIPI_IF)
#define CSI_IF_CFG_REG_IF_DATA_WIDTH		8
#define CSI_IF_CFG_REG_IF_DATA_WIDTH_MASK	(0X3<<CSI_IF_CFG_REG_IF_DATA_WIDTH)
#define CSI_IF_CFG_REG_IF_BUS_SEQ		10
#define CSI_IF_CFG_REG_RES1			12
#define CSI_IF_CFG_REG_CLK_POL		16
#define CSI_IF_CFG_REG_CLK_POL_MASK		(0X1<<CSI_IF_CFG_REG_CLK_POL)
#define CSI_IF_CFG_REG_HREF_POL			17
#define CSI_IF_CFG_REG_HREF_POL_MASK		(0X1<<CSI_IF_CFG_REG_HREF_POL)
#define CSI_IF_CFG_REG_VREF_POL		18
#define CSI_IF_CFG_REG_VREF_POL_MASK		(0X1<<CSI_IF_CFG_REG_VREF_POL)
#define CSI_IF_CFG_REG_FIELD			19
#define CSI_IF_CFG_REG_FIELD_MASK		(0X1<<CSI_IF_CFG_REG_FIELD)
#define CSI_IF_CFG_REG_FPS_DS		20
#define CSI_IF_CFG_REG_SRC_TYPE		21
#define CSI_IF_CFG_REG_SRC_TYPE_MASK		(0X1<<CSI_IF_CFG_REG_SRC_TYPE)
#define CSI_IF_CFG_REG_DST			22
#define CSI_IF_CFG_REG_RES2			23

#define CSI_CAP_REG_OFF			0X008
#define CSI_CAP_REG_CH0_SCAP_ON			0
#define CSI_CAP_REG_CH0_VCAP_ON		1
#define CSI_CAP_REG_RES0			2
#define CSI_CAP_REG_CH1_SCAP_ON		8
#define CSI_CAP_REG_CH1_VCAP_ON		9
#define CSI_CAP_REG_RES1			10
#define CSI_CAP_REG_CH2_SCAP_ON		16
#define CSI_CAP_REG_CH2_VCAP_ON		17
#define CSI_CAP_REG_RES2			18
#define CSI_CAP_REG_CH3_SCAP_ON		24
#define CSI_CAP_REG_CH3_VCAP_ON		25
#define CSI_CAP_REG_RES3			26

#define CSI_SYNC_CNT_REG_OFF			0X00C
#define CSI_SYNC_CNT_REG_SYNC_CNT		0
#define CSI_SYNC_CNT_REG_RES0		24

#define CSI_FIFO_THRS_REG_OFF		0X010
#define CSI_FIFO_THRS_REG_FIFO_THRS		0
#define CSI_FIFO_THRS_REG_RES0		12

#define CSI_PTN_LEN_REG_OFF			0X030
#define CSI_PTN_LEN_REG_PTN_LEN			0

#define CSI_PTN_ADDR_REG_OFF			0X034
#define CSI_PTN_ADDR_REG_PTN_ADDR		0

#define CSI_VER_REG_OFF			0X03C
#define CSI_VER_REG_VER				0

#define CSI_CH_CFG_REG_OFF			0X044
#define CSI_CH_CFG_REG_RES0			0
#define CSI_CH_CFG_REG_INPUT_SEQ		8
#define CSI_CH_CFG_REG_INPUT_SEQ_MASK		(0X3<<CSI_CH_CFG_REG_INPUT_SEQ)
#define CSI_CH_CFG_REG_FIELD_SEL		10
#define CSI_CH_CFG_REG_FIELD_SEL_MASK		(0X3<<CSI_CH_CFG_REG_FIELD_SEL)
#define CSI_CH_CFG_REG_HFLIP_EN		12
#define CSI_CH_CFG_REG_VFLIP_EN		13
#define CSI_CH_CFG_REG_RES1			14
#define CSI_CH_CFG_REG_OUTPUT_FMT		16
#define CSI_CH_CFG_REG_OUTPUT_FMT_MASK		(0XF<<CSI_CH_CFG_REG_OUTPUT_FMT)

#define CSI_CH_CFG_REG_INPUT_FMT		20
#define CSI_CH_CFG_REG_INPUT_FMT_MASK		(0XF<<CSI_CH_CFG_REG_INPUT_FMT)

#define CSI_CH_CFG_REG_PAD_VAL		24

#define CSI_CH_SCALE_REG_OFF			0X04C
#define CSI_CH_SCALE_REG_QUART_EN		0
#define CSI_CH_SCALE_REG_RES0		1

#define CSI_CH_F0_BUFA_REG_OFF		0X050
#define CSI_CH_F0_BUFA_REG_C0F0_BUFA		0

#define CSI_CH_F1_BUFA_REG_OFF		0X058
#define CSI_CH_F1_BUFA_REG_C0F1_BUFA		0

#define CSI_CH_F2_BUFA_REG_OFF		0X060
#define CSI_CH_F2_BUFA_REG_C0F2_BUFA		0

#define CSI_CH_STA_REG_OFF			0X06C
#define CSI_CH_STA_REG_SCAP_STA			0
#define CSI_CH_STA_REG_VCAP_STA		1
#define CSI_CH_STA_REG_FIELD_STA		2
#define CSI_CH_STA_REG_RES0			3

#define CSI_CH_INT_EN_REG_OFF		0X070
#define CSI_CH_INT_EN_REG_CD_INT_EN		0
#define CSI_CH_INT_EN_REG_FD_INT_EN		1
#define CSI_CH_INT_EN_REG_FIFO0_OF_INT_EN	2
#define CSI_CH_INT_EN_REG_FIFO1_OF_INT_EN	3
#define CSI_CH_INT_EN_REG_FIFO2_OF_INT_EN	4
#define CSI_CH_INT_EN_REG_PRTC_ERR_INT_EN	5
#define CSI_CH_INT_EN_REG_HB_OF_INT_EN	6
#define CSI_CH_INT_EN_REG_VS_INT_EN		7
#define CSI_CH_INT_EN_REG_RES0		8

#define CSI_CH_INT_STA_REG_OFF		0X074
#define CSI_CH_INT_STA_REG_CD_PD		0
#define CSI_CH_INT_STA_REG_FD_PD		1
#define CSI_CH_INT_STA_REG_FIFO0_OF_PD	2
#define CSI_CH_INT_STA_REG_FIFO1_OF_PD	3
#define CSI_CH_INT_STA_REG_FIFO2_OF_PD	4
#define CSI_CH_INT_STA_REG_PRTC_ERR_PD	5
#define CSI_CH_INT_STA_REG_HB_OF_PD		6
#define CSI_CH_INT_STA_REG_VS_PD		7
#define CSI_CH_INT_STA_REG_RES0		8

#define CSI_CH_HSIZE_REG_OFF			0X080
#define CSI_CH_HSIZE_REG_HOR_START		0
#define CSI_CH_HSIZE_REG_HOR_START_MASK		(0X1FFF<<CSI_CH_HSIZE_REG_HOR_START)
#define CSI_CH_HSIZE_REG_RES0		13
#define CSI_CH_HSIZE_REG_HOR_LEN		16
#define CSI_CH_HSIZE_REG_HOR_LEN_MASK		(0X1FFF<<CSI_CH_HSIZE_REG_HOR_LEN)
#define CSI_CH_HSIZE_REG_RES1		29

#define CSI_CH_VSIZE_REG_OFF			0X084
#define CSI_CH_VSIZE_REG_VER_START		0
#define CSI_CH_VSIZE_REG_VER_START_MASK		(0X1FFF<<CSI_CH_VSIZE_REG_VER_START)
#define CSI_CH_VSIZE_REG_RES0		13
#define CSI_CH_VSIZE_REG_VER_LEN		16
#define CSI_CH_VSIZE_REG_VER_LEN_MASK		(0X1FFF<<CSI_CH_VSIZE_REG_VER_LEN)

#define CSI_CH_VSIZE_REG_RES1		29

#define CSI_CH_BUF_LEN_REG_OFF		0X088
#define CSI_CH_BUF_LEN_REG_BUF_LEN		0
#define CSI_CH_BUF_LEN_REG_BUF_LEN_MASK		(0X1FFF<<CSI_CH_BUF_LEN_REG_BUF_LEN)
#define CSI_CH_BUF_LEN_REG_RES0		13
#define CSI_CH_BUF_LEN_REG_BUF_LEN_C		16
#define CSI_CH_BUF_LEN_REG_BUF_LEN_C_MASK	(0X1FFF<<CSI_CH_BUF_LEN_REG_BUF_LEN_C)


#define CSI_CH_BUF_LEN_REG_RES1		29

#define CSI_CH_FLIP_SIZE_REG_OFF		0X08C
#define CSI_CH_FLIP_SIZE_REG_VALID_LEN		0
#define CSI_CH_FLIP_SIZE_REG_RES0		13
#define CSI_CH_FLIP_SIZE_REG_VER_LEN		16
#define CSI_CH_FLIP_SIZE_REG_RES1		29

#define CSI_CH_FRM_CLK_CNT_REG_OFF		0X090
#define CSI_CH_FRM_CLK_CNT_REG_FRM_CLK_CNT	0
#define CSI_CH_FRM_CLK_CNT_REG_RES0		24

#define CSI_CH_ACC_ITNL_CLK_CNT_REG_OFF	0X094
#define CSI_CH_ACC_ITNL_CLK_CNT_REG_ITNL_CLK_CNT	0
#define CSI_CH_ACC_ITNL_CLK_CNT_REG_ACC_CLK_CNT		24

#define CSI_CH_OFF					(0x0100)

#endif /* __CSI__REG__I__H__ */
