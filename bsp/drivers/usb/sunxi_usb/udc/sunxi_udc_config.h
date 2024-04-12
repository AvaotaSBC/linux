/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * (C) Copyright 2010-2015
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * javen, 2010-3-3, create this file
 *
 * usb udc config.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef __SUNXI_UDC_CONFIG_H__
#define __SUNXI_UDC_CONFIG_H__

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>

#define  SW_UDC_DOUBLE_FIFO       /* double FIFO          */

/**
 * we previously deleted the "dma_flag" variable for GKI, which led to the
 * loss of the flag that distinguishes some gadget modules to use DMA.
 * Therefore, UDC does not provide DMA transmission.
 */
#define  SW_UDC_DMA

/**
 * only SUN8IW5 and later ic support inner dma,
 * former ic(eg. SUN8IW1, SUN8IW3, SUN8IW2 etc) use outer dma.
 */
#ifdef SW_UDC_DMA
#define  SW_UDC_DMA_INNER
#endif

#define  SW_UDC_HS_TO_FS          /* support HS to FS */
#define  SW_UDC_DEBUG

/* #define DMSG_DGB */
/* sw udc debug print */
#ifdef DMSG_DGB
#define DMSG_DBG_UDC	DMSG_MSG
#else
#define DMSG_DBG_UDC(...)
#endif

#include  "../include/sunxi_usb_config.h"

#endif /* __SUNXI_UDC_CONFIG_H__ */
