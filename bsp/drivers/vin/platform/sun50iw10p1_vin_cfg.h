/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
 /*
  * Hawkview ISP - sun8iw19p1_vin_cfg.h module
  *
  * Copyright (c) 2019 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
  *
  * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#ifndef _SUN50IW10P1_VIN_CFG_H_
#define _SUN50IW10P1_VIN_CFG_H_

#define CSI_CCU_REGS_BASE			0x02000000
#define CSI_TOP_REGS_BASE			0x02000800

#define CSI0_REGS_BASE				0x02001000
#define CSI1_REGS_BASE				0x02002000

#define COMBO_REGS_BASE				0x0200a000

#define ISP0_REGS_BASE				0x02100000
#define ISP1_REGS_BASE				0x02102000

#define TDM_REGS_BASE				0x02108000

#define VIPP0_REGS_BASE				0x02110000
#define VIPP1_REGS_BASE				0x02110400
#define VIPP2_REGS_BASE				0x02110800
#define VIPP3_REGS_BASE				0x02110c00

#define CSI_DMA0_REG_BASE			0x02009000
#define CSI_DMA1_REG_BASE			0x02009200
#define CSI_DMA2_REG_BASE			0x02009400
#define CSI_DMA3_REG_BASE			0x02009600

#define GPIO_REGS_VBASE				0x0300b000

#define VIN_MAX_DEV			4
#define VIN_MAX_CSI			2
#define VIN_MAX_TDM			1
#define VIN_MAX_CCI			2
#define VIN_MAX_MIPI			2
#define VIN_MAX_ISP			2
#define VIN_MAX_SCALER			4

#define MAX_CH_NUM			4



#endif /* _SUN50IW10P1_VIN_CFG_H_ */
