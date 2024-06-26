/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/bsp_common.c
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
 * sunxi video front end common function
 * Author:raymonxiu
 */

#include "bsp_common.h"

enum bus_pixeltype find_bus_type(enum bus_pixelcode code)
{
	switch (code) {
	case BUS_FMT_BGR565_2X8_BE:
	case BUS_FMT_BGR565_2X8_LE:
	case BUS_FMT_RGB565_2X8_BE:
	case BUS_FMT_RGB565_2X8_LE:
	case BUS_FMT_RGB565_16X1:
		return BUS_FMT_RGB565;
	case BUS_FMT_UYVY8_2X8:
	case BUS_FMT_UYVY8_1X16:
	case BUS_FMT_UYVY8_16X1:
	case BUS_FMT_UYVY10_20X1:
		return BUS_FMT_UYVY;
	case BUS_FMT_VYUY8_2X8:
	case BUS_FMT_VYUY8_1X16:
		return BUS_FMT_VYUY;
	case BUS_FMT_YUYV8_2X8:
	case BUS_FMT_YUYV10_2X10:
	case BUS_FMT_YUYV8_1X16:
	case BUS_FMT_YUYV10_1X20:
		return BUS_FMT_YUYV;
	case BUS_FMT_YVYU8_2X8:
	case BUS_FMT_YVYU10_2X10:
	case BUS_FMT_YVYU8_1X16:
	case BUS_FMT_YVYU10_1X20:
		return BUS_FMT_YVYU;
	case BUS_FMT_SBGGR8_1X8:
	case BUS_FMT_SBGGR10_DPCM8_1X8:
	case BUS_FMT_SBGGR10_1X10:
	case BUS_FMT_SBGGR12_1X12:
	case BUS_FMT_SBGGR8_8X1:
	case BUS_FMT_SBGGR10_10X1:
	case BUS_FMT_SBGGR12_12X1:
		return BUS_FMT_SBGGR;
	case BUS_FMT_SGBRG8_1X8:
	case BUS_FMT_SGBRG10_DPCM8_1X8:
	case BUS_FMT_SGBRG10_1X10:
	case BUS_FMT_SGBRG12_1X12:
	case BUS_FMT_SGBRG8_8X1:
	case BUS_FMT_SGBRG10_10X1:
	case BUS_FMT_SGBRG12_12X1:
		return BUS_FMT_SGBRG;
	case BUS_FMT_SGRBG8_1X8:
	case BUS_FMT_SGRBG10_DPCM8_1X8:
	case BUS_FMT_SGRBG10_1X10:
	case BUS_FMT_SGRBG12_1X12:
	case BUS_FMT_SGRBG8_8X1:
	case BUS_FMT_SGRBG10_10X1:
	case BUS_FMT_SGRBG12_12X1:
		return BUS_FMT_SGRBG;
	case BUS_FMT_SRGGB8_1X8:
	case BUS_FMT_SRGGB10_DPCM8_1X8:
	case BUS_FMT_SRGGB10_1X10:
	case BUS_FMT_SRGGB12_1X12:
	case BUS_FMT_SRGGB8_8X1:
	case BUS_FMT_SRGGB10_10X1:
	case BUS_FMT_SRGGB12_12X1:
		return BUS_FMT_SRGGB;
	case BUS_FMT_YUV8_1X24:
		return BUS_FMT_Y_U_V;
	case BUS_FMT_YY8_UYVY8_12X1:
	case BUS_FMT_YY10_UYVY10_15X1:
		return BUS_FMT_YY_UYVY;
	default:
		return BUS_FMT_UYVY;
	}
}

enum bit_width find_bus_width(enum bus_pixelcode code)
{
	switch (code) {
	case BUS_FMT_RGB565_16X1:
	case BUS_FMT_UYVY8_16X1:
	case BUS_FMT_UYVY10_20X1:
	case BUS_FMT_SBGGR8_8X1:
	case BUS_FMT_SBGGR10_10X1:
	case BUS_FMT_SBGGR12_12X1:
	case BUS_FMT_SGBRG8_8X1:
	case BUS_FMT_SGBRG10_10X1:
	case BUS_FMT_SGBRG12_12X1:
	case BUS_FMT_SGRBG8_8X1:
	case BUS_FMT_SGRBG10_10X1:
	case BUS_FMT_SGRBG12_12X1:
	case BUS_FMT_SRGGB8_8X1:
	case BUS_FMT_SRGGB10_10X1:
	case BUS_FMT_SRGGB12_12X1:
	case BUS_FMT_YY8_UYVY8_12X1:
	case BUS_FMT_YY10_UYVY10_15X1:
		return W_1BIT;
	case BUS_FMT_BGR565_2X8_BE:
	case BUS_FMT_BGR565_2X8_LE:
	case BUS_FMT_RGB565_2X8_BE:
	case BUS_FMT_RGB565_2X8_LE:
	case BUS_FMT_UYVY8_2X8:
	case BUS_FMT_VYUY8_2X8:
	case BUS_FMT_YUYV8_2X8:
	case BUS_FMT_YVYU8_2X8:
	case BUS_FMT_SBGGR8_1X8:
	case BUS_FMT_SBGGR10_DPCM8_1X8:
	case BUS_FMT_SGBRG8_1X8:
	case BUS_FMT_SGBRG10_DPCM8_1X8:
	case BUS_FMT_SGRBG8_1X8:
	case BUS_FMT_SGRBG10_DPCM8_1X8:
	case BUS_FMT_SRGGB8_1X8:
	case BUS_FMT_SRGGB10_DPCM8_1X8:
		return W_8BIT;
	case BUS_FMT_YUYV10_2X10:
	case BUS_FMT_YVYU10_2X10:
	case BUS_FMT_SBGGR10_1X10:
	case BUS_FMT_SGBRG10_1X10:
	case BUS_FMT_SGRBG10_1X10:
	case BUS_FMT_SRGGB10_1X10:
		return W_10BIT;
	case BUS_FMT_SBGGR12_1X12:
	case BUS_FMT_SGBRG12_1X12:
	case BUS_FMT_SGRBG12_1X12:
	case BUS_FMT_SRGGB12_1X12:
		return W_12BIT;
	case BUS_FMT_UYVY8_1X16:
	case BUS_FMT_VYUY8_1X16:
	case BUS_FMT_YUYV8_1X16:
	case BUS_FMT_YVYU8_1X16:
		return W_16BIT;
	case BUS_FMT_YVYU10_1X20:
	case BUS_FMT_YUYV10_1X20:
		return W_20BIT;
	case BUS_FMT_YUV8_1X24:
		return W_24BIT;
	default:
		return W_8BIT;
	}
}

enum bit_width find_bus_precision(enum bus_pixelcode code)
{
	switch (code) {
	case BUS_FMT_RGB565_16X1:
	case BUS_FMT_BGR565_2X8_BE:
	case BUS_FMT_BGR565_2X8_LE:
	case BUS_FMT_RGB565_2X8_BE:
	case BUS_FMT_RGB565_2X8_LE:
	case BUS_FMT_SBGGR8_8X1:
	case BUS_FMT_SGBRG8_8X1:
	case BUS_FMT_SGRBG8_8X1:
	case BUS_FMT_SRGGB8_8X1:
	case BUS_FMT_SBGGR8_1X8:
	case BUS_FMT_SGBRG8_1X8:
	case BUS_FMT_SGRBG8_1X8:
	case BUS_FMT_SRGGB8_1X8:
	case BUS_FMT_SBGGR10_DPCM8_1X8:
	case BUS_FMT_SGBRG10_DPCM8_1X8:
	case BUS_FMT_SGRBG10_DPCM8_1X8:
	case BUS_FMT_SRGGB10_DPCM8_1X8:
	case BUS_FMT_UYVY8_16X1:
	case BUS_FMT_UYVY8_2X8:
	case BUS_FMT_VYUY8_2X8:
	case BUS_FMT_YUYV8_2X8:
	case BUS_FMT_YVYU8_2X8:
	case BUS_FMT_UYVY8_1X16:
	case BUS_FMT_VYUY8_1X16:
	case BUS_FMT_YUYV8_1X16:
	case BUS_FMT_YVYU8_1X16:
	case BUS_FMT_YUV8_1X24:
	case BUS_FMT_YY8_UYVY8_12X1:
		return W_8BIT;
	case BUS_FMT_SBGGR10_10X1:
	case BUS_FMT_SGBRG10_10X1:
	case BUS_FMT_SGRBG10_10X1:
	case BUS_FMT_SRGGB10_10X1:
	case BUS_FMT_SBGGR10_1X10:
	case BUS_FMT_SGBRG10_1X10:
	case BUS_FMT_SGRBG10_1X10:
	case BUS_FMT_SRGGB10_1X10:
	case BUS_FMT_YUYV10_2X10:
	case BUS_FMT_YVYU10_2X10:
	case BUS_FMT_YVYU10_1X20:
	case BUS_FMT_YUYV10_1X20:
	case BUS_FMT_UYVY10_20X1:
	case BUS_FMT_YY10_UYVY10_15X1:
		return W_10BIT;
	case BUS_FMT_SBGGR12_12X1:
	case BUS_FMT_SGBRG12_12X1:
	case BUS_FMT_SGRBG12_12X1:
	case BUS_FMT_SRGGB12_12X1:
	case BUS_FMT_SBGGR12_1X12:
	case BUS_FMT_SGBRG12_1X12:
	case BUS_FMT_SGRBG12_1X12:
	case BUS_FMT_SRGGB12_1X12:
		return W_12BIT;
	default:
		return W_8BIT;
	}
}

enum pixel_fmt_type find_pixel_fmt_type(enum pixel_fmt code)
{
	switch (code) {
	case PIX_FMT_RGB565:
		return RGB565;
	case PIX_FMT_RGB888:
		return RGB888;
	case PIX_FMT_PRGB888:
		return PRGB888;
	case PIX_FMT_YUYV:
	case PIX_FMT_YVYU:
	case PIX_FMT_UYVY:
	case PIX_FMT_VYUY:
		return YUV422_INTLVD;
	case PIX_FMT_YUV422P_8:
	case PIX_FMT_YVU422P_8:
		return YUV422_PL;
	case PIX_FMT_YUV420P_8:
	case PIX_FMT_YVU420P_8:
		return YUV420_PL;
	case PIX_FMT_YUV420SP_8:
	case PIX_FMT_YVU420SP_8:
	case PIX_FMT_YUV420SP_10:
	case PIX_FMT_YVU420SP_10:
		return YUV420_SPL;
	case PIX_FMT_YUV422SP_8:
	case PIX_FMT_YVU422SP_8:
	case PIX_FMT_YUV422SP_10:
	case PIX_FMT_YVU422SP_10:
		return YUV422_SPL;
	case PIX_FMT_YUV420MB_8:
	case PIX_FMT_YVU420MB_8:
		return YUV420_MB;
	case PIX_FMT_YUV422MB_8:
	case PIX_FMT_YVU422MB_8:
		return YUV422_MB;
	case PIX_FMT_SBGGR_8:
	case PIX_FMT_SGBRG_8:
	case PIX_FMT_SGRBG_8:
	case PIX_FMT_SRGGB_8:
	case PIX_FMT_SBGGR_10:
	case PIX_FMT_SGBRG_10:
	case PIX_FMT_SGRBG_10:
	case PIX_FMT_SRGGB_10:
	case PIX_FMT_SBGGR_12:
	case PIX_FMT_SGBRG_12:
	case PIX_FMT_SGRBG_12:
	case PIX_FMT_SRGGB_12:
		return BAYER_RGB;
	default:
		return BAYER_RGB;
	}
}

enum pixel_fmt pix_fmt_v4l2_to_common(unsigned int pix_fmt)
{
	switch (pix_fmt) {
	case V4L2_PIX_FMT_RGB565:
		return PIX_FMT_RGB565;
	case V4L2_PIX_FMT_RGB24:
		return PIX_FMT_RGB888;
	case V4L2_PIX_FMT_RGB32:
		return PIX_FMT_PRGB888;
	case V4L2_PIX_FMT_YUYV:
		return PIX_FMT_YUYV;
	case V4L2_PIX_FMT_YVYU:
		return PIX_FMT_YVYU;
	case V4L2_PIX_FMT_UYVY:
		return PIX_FMT_UYVY;
	case V4L2_PIX_FMT_VYUY:
		return PIX_FMT_VYUY;
	case V4L2_PIX_FMT_YUV422P:
		return PIX_FMT_YUV422P_8;
	case V4L2_PIX_FMT_YUV420:
		return PIX_FMT_YUV420P_8;
	case V4L2_PIX_FMT_YVU420:
		return PIX_FMT_YVU420P_8;
	case V4L2_PIX_FMT_NV12:
		return PIX_FMT_YUV420SP_8;
	case V4L2_PIX_FMT_NV21:
		return PIX_FMT_YVU420SP_8;
	case V4L2_PIX_FMT_NV16:
		return PIX_FMT_YUV422SP_8;
	case V4L2_PIX_FMT_NV61:
		return PIX_FMT_YVU422SP_8;
	case V4L2_PIX_FMT_SBGGR8:
		return PIX_FMT_SBGGR_8;
	case V4L2_PIX_FMT_SGBRG8:
		return PIX_FMT_SGBRG_8;
	case V4L2_PIX_FMT_SGRBG8:
		return PIX_FMT_SGRBG_8;
	case V4L2_PIX_FMT_SRGGB8:
		return PIX_FMT_SRGGB_8;
	case V4L2_PIX_FMT_SBGGR10:
		return PIX_FMT_SBGGR_10;
	case V4L2_PIX_FMT_SGBRG10:
		return PIX_FMT_SGBRG_10;
	case V4L2_PIX_FMT_SGRBG10:
		return PIX_FMT_SGRBG_10;
	case V4L2_PIX_FMT_SRGGB10:
		return PIX_FMT_SRGGB_10;
	case V4L2_PIX_FMT_SBGGR12:
		return PIX_FMT_SBGGR_12;
	case V4L2_PIX_FMT_SGBRG12:
		return PIX_FMT_SGBRG_12;
	case V4L2_PIX_FMT_SGRBG12:
		return PIX_FMT_SGRBG_12;
	case V4L2_PIX_FMT_SRGGB12:
		return PIX_FMT_SRGGB_12;
#ifdef CONFIG_ARCH_SUN3IW1P1
	case V4L2_PIX_FMT_HM12:
		return PIX_FMT_YUV420MB_8;
#endif
	default:
		return PIX_FMT_SBGGR_8;
	}
}

enum field field_fmt_v4l2_to_common(enum v4l2_field field)
{
	return (enum field)field;
}
