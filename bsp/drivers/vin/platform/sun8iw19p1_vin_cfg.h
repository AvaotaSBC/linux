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

#ifndef _SUN8IW19P1_VIN_CFG_H_
#define _SUN8IW19P1_VIN_CFG_H_

#define CSI_CCU_REGS_BASE			0x06600000
#define CSI_TOP_REGS_BASE			0x06600800

#define CSI0_REGS_BASE				0x06601000
#define CSI1_REGS_BASE				0x06602000

#define ISP_REGS_BASE				0x02100000

#define VIPP0_REGS_BASE				0x02101000
#define VIPP1_REGS_BASE				0x02101400
#define VIPP2_REGS_BASE				0x02101800
#define VIPP3_REGS_BASE				0x02101c00

#define CSI_DMA0_REG_BASE			0x06609000
#define CSI_DMA1_REG_BASE			0x06609200
#define CSI_DMA2_REG_BASE			0x06609400
#define CSI_DMA3_REG_BASE			0x06609600

#define GPIO_REGS_VBASE				0x0300b000

#define VIN_MAX_DEV			4
#define VIN_MAX_CSI			2
#define VIN_MAX_CCI			2
#define VIN_MAX_TDM			0
#define VIN_MAX_MIPI			1
#define VIN_MAX_ISP			4
#define VIN_MAX_SCALER			4

#define MAX_CH_NUM			4

#endif /* _SUN8IW19P1_VIN_CFG_H_ */
