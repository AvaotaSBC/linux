/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/bsp_common.h
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
 * sunxi video front end common header
 * included by vfe and csi/mipi/isp bsp
 * Author:raymonxiu
 */

#ifndef __BSP_COMMON__H__
#define __BSP_COMMON__H__

#include <linux/videodev2.h>
/* #include <media/v4l2-mediabus.h> */

enum field {
	FIELD_ANY           = 0,	/* driver can choose from none, top, bottom, interlaced depending on whatever it thinks is approximate */
	FIELD_NONE          = 1,	/* this device has no fields ... */
	FIELD_TOP           = 2,	/* top field only */
	FIELD_BOTTOM        = 3,	/* bottom field only */
	FIELD_INTERLACED    = 4,	/* both fields interlaced */
	FIELD_SEQ_TB        = 5,	/* both fields sequential into one buffer, top-bottom order */
	FIELD_SEQ_BT        = 6,	/* same as above + bottom-top order */
	FIELD_ALTERNATE     = 7,	/* both fields alternating into separate buffers */
	FIELD_INTERLACED_TB = 8,	/* both fields interlaced, top field first and the top field transmitted first */
	FIELD_INTERLACED_BT = 9,	/* both fields interlaced, top field first and the bottom field is transmitted first */
};

enum bus_pixelcode {
	BUS_FMT_FIXED = 0x0001,

	/* RGB - next is 0x1009 */
	BUS_FMT_RGB444_2X8_PADHI_BE = 0x1001,
	BUS_FMT_RGB444_2X8_PADHI_LE = 0x1002,
	BUS_FMT_RGB555_2X8_PADHI_BE = 0x1003,
	BUS_FMT_RGB555_2X8_PADHI_LE = 0x1004,
	BUS_FMT_BGR565_2X8_BE = 0x1005,
	BUS_FMT_BGR565_2X8_LE = 0x1006,
	BUS_FMT_RGB565_2X8_BE = 0x1007,
	BUS_FMT_RGB565_2X8_LE = 0x1008,

	BUS_FMT_RGB565_16X1 = 0x1009,
	BUS_FMT_RGB888_24X1 = 0x100a,

	/* YUV (including grey) - next is 0x2014 */
	BUS_FMT_Y8_1X8 = 0x2001,
	BUS_FMT_UYVY8_1_5X8 = 0x2002,
	BUS_FMT_VYUY8_1_5X8 = 0x2003,
	BUS_FMT_YUYV8_1_5X8 = 0x2004,
	BUS_FMT_YVYU8_1_5X8 = 0x2005,
	BUS_FMT_UYVY8_2X8 = 0x2006,
	BUS_FMT_VYUY8_2X8 = 0x2007,
	BUS_FMT_YUYV8_2X8 = 0x2008,
	BUS_FMT_YVYU8_2X8 = 0x2009,
	BUS_FMT_Y10_1X10 = 0x200a,
	BUS_FMT_YUYV10_2X10 = 0x200b,
	BUS_FMT_YVYU10_2X10 = 0x200c,
	BUS_FMT_Y12_1X12 = 0x2013,
	BUS_FMT_UYVY8_1X16 = 0x200f,
	BUS_FMT_VYUY8_1X16 = 0x2010,
	BUS_FMT_YUYV8_1X16 = 0x2011,
	BUS_FMT_YVYU8_1X16 = 0x2012,
	BUS_FMT_YUYV10_1X20 = 0x200d,
	BUS_FMT_YVYU10_1X20 = 0x200e,

	BUS_FMT_YUV8_1X24 = 0x2014,
	BUS_FMT_UYVY8_16X1 = 0x2015,
	BUS_FMT_UYVY10_20X1 = 0x2016,
	BUS_FMT_YY8_UYVY8_12X1 = 0x2018,
	BUS_FMT_YY10_UYVY10_15X1 = 0x2019,

	/* Bayer - next is 0x3015 */
	BUS_FMT_SBGGR8_1X8 = 0x3001,
	BUS_FMT_SGBRG8_1X8 = 0x3013,
	BUS_FMT_SGRBG8_1X8 = 0x3002,
	BUS_FMT_SRGGB8_1X8 = 0x3014,
	BUS_FMT_SBGGR10_DPCM8_1X8 = 0x300b,
	BUS_FMT_SGBRG10_DPCM8_1X8 = 0x300c,
	BUS_FMT_SGRBG10_DPCM8_1X8 = 0x3009,
	BUS_FMT_SRGGB10_DPCM8_1X8 = 0x300d,
	BUS_FMT_SBGGR10_2X8_PADHI_BE = 0x3003,
	BUS_FMT_SBGGR10_2X8_PADHI_LE = 0x3004,
	BUS_FMT_SBGGR10_2X8_PADLO_BE = 0x3005,
	BUS_FMT_SBGGR10_2X8_PADLO_LE = 0x3006,
	BUS_FMT_SBGGR10_1X10 = 0x3007,
	BUS_FMT_SGBRG10_1X10 = 0x300e,
	BUS_FMT_SGRBG10_1X10 = 0x300a,
	BUS_FMT_SRGGB10_1X10 = 0x300f,
	BUS_FMT_SBGGR12_1X12 = 0x3008,
	BUS_FMT_SGBRG12_1X12 = 0x3010,
	BUS_FMT_SGRBG12_1X12 = 0x3011,
	BUS_FMT_SRGGB12_1X12 = 0x3012,

	BUS_FMT_SBGGR8_8X1 = 0x3015,
	BUS_FMT_SGBRG8_8X1 = 0x3016,
	BUS_FMT_SGRBG8_8X1 = 0x3017,
	BUS_FMT_SRGGB8_8X1 = 0x3018,
	BUS_FMT_SBGGR10_10X1 = 0x3019,
	BUS_FMT_SGBRG10_10X1 = 0x301a,
	BUS_FMT_SGRBG10_10X1 = 0x301b,
	BUS_FMT_SRGGB10_10X1 = 0x301c,
	BUS_FMT_SBGGR12_12X1 = 0x301d,
	BUS_FMT_SGBRG12_12X1 = 0x301e,
	BUS_FMT_SGRBG12_12X1 = 0x301f,
	BUS_FMT_SRGGB12_12X1 = 0x3020,

	/* JPEG compressed formats - next is 0x4002 */
	BUS_FMT_JPEG_1X8 = 0x4001,
};

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

enum pixel_fmt {
	PIX_FMT_RGB565 = 0x0000,
	PIX_FMT_RGB888,
	PIX_FMT_PRGB888,
	PIX_FMT_YUYV = 0x1000,
	PIX_FMT_YVYU,
	PIX_FMT_UYVY,
	PIX_FMT_VYUY,
	PIX_FMT_YUV422P_8 = 0x2000,
	PIX_FMT_YVU422P_8,
	PIX_FMT_YUV420P_8,
	PIX_FMT_YVU420P_8,
	PIX_FMT_YUV420SP_8 = 0x3000,
	PIX_FMT_YVU420SP_8,
	PIX_FMT_YUV422SP_8,
	PIX_FMT_YVU422SP_8,
	PIX_FMT_YUV420SP_10,
	PIX_FMT_YVU420SP_10,
	PIX_FMT_YUV422SP_10,
	PIX_FMT_YVU422SP_10,
	PIX_FMT_YUV420MB_8 = 0x4000,
	PIX_FMT_YVU420MB_8,
	PIX_FMT_YUV422MB_8,
	PIX_FMT_YVU422MB_8,
	PIX_FMT_SBGGR_8 = 0x5000,
	PIX_FMT_SGBRG_8,
	PIX_FMT_SGRBG_8,
	PIX_FMT_SRGGB_8,
	PIX_FMT_SBGGR_10,
	PIX_FMT_SGBRG_10,
	PIX_FMT_SGRBG_10,
	PIX_FMT_SRGGB_10,
	PIX_FMT_SBGGR_12,
	PIX_FMT_SGBRG_12,
	PIX_FMT_SGRBG_12,
	PIX_FMT_SRGGB_12,
	PIX_FMT_NONE = 0xffff,
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

extern enum bus_pixeltype   find_bus_type(enum bus_pixelcode code);
extern enum bit_width find_bus_width(enum bus_pixelcode code);
extern enum bit_width find_bus_precision(enum bus_pixelcode code);
extern enum pixel_fmt_type  find_pixel_fmt_type(enum pixel_fmt code);
extern enum pixel_fmt pix_fmt_v4l2_to_common(unsigned int pix_fmt);
extern enum field field_fmt_v4l2_to_common(enum v4l2_field field);

#endif /* __BSP_COMMON__H__ */
