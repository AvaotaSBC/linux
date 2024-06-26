/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/csi_cci/cci_platform_drv.h
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
 * cci_platform_drv.h
 *
 * Hawkview ISP - cci_platform_drv.h module
 *
 * Copyright (c) 2014 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Version		  Author         Date		    Description
 *
 *   2.0		  Yang Feng	2014/06/23	      Second Version
 *
 ****************************************************************************************
 */
#ifndef _CCI_PLATFORM_DRV_H_
#define _CCI_PLATFORM_DRV_H_

#include "../platform_cfg.h"

struct cci_platform_data {
	unsigned int cci_sel;
};

struct cci_dev {
	unsigned int  cci_sel;
	struct platform_device  *pdev;
	unsigned int id;
	spinlock_t slock;
	int irq;
	wait_queue_head_t   wait;

	void __iomem      *base;
};

#endif /*_CCI_PLATFORM_DRV_H_*/
