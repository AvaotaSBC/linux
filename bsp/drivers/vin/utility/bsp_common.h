/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
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

#ifndef __BSP_COMMON__H__
#define __BSP_COMMON__H__

#include <linux/videodev2.h>
#include <media/v4l2-mediabus.h>

enum bus_pixeltype {
	BUS_FMT_RGB565,
	BUS_FMT_RGB888,
	BUS_FMT_Y_U_V,
	BUS_FMT_YY_YUYV,
	BUS_FMT_YY_YVYU,
	BUS_FMT_YY_UYVY,
	BUS_FMT_YY_VYUY,
	BUS_FMT_YUYV,
	BUS_FMT_YVYU,
	BUS_FMT_UYVY,
	BUS_FMT_VYUY,
	BUS_FMT_SBGGR,
	BUS_FMT_SGBRG,
	BUS_FMT_SGRBG,
	BUS_FMT_SRGGB,
};

enum pixel_fmt_type {
	RGB565,
	RGB888,
	PRGB888,
	YUV422_INTLVD,
	YUV422_PL,
	YUV422_SPL,
	YUV422_MB,
	YUV420_PL,
	YUV420_SPL,
	YUV420_MB,
	BAYER_RGB,
};

enum bit_width {
	W_1BIT,
	W_2BIT,
	W_4BIT,
	W_6BIT,
	W_8BIT,
	W_10BIT,
	W_12BIT,
	W_14BIT,
	W_16BIT,
	W_20BIT,
	W_24BIT,
	W_32BIT,
};

extern enum bus_pixeltype find_bus_type(u32 code);
extern enum bit_width find_bus_width(u32 code);
extern enum bit_width find_bus_precision(u32 code);
extern enum pixel_fmt_type find_pixel_fmt_type(unsigned int pix_fmt);

#endif /* __BSP_COMMON__H__ */
