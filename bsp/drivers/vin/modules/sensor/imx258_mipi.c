/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for imx258 Raw cameras.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *    Liang WeiJie <liangweijie@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "../../utility/vin_log.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>

#include "camera.h"
#include "sensor_helper.h"

MODULE_AUTHOR("lwj@1205");
MODULE_DESCRIPTION("A low-level driver for IMX258 sensors");
MODULE_LICENSE("GPL");

#define MCLK              (12*1000*1000)
#define V4L2_IDENT_SENSOR  0x0258

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30

/*
 * The imx258 i2c address
 */
#define I2C_ADDR 0x34

#define SENSOR_NUM 0x2
#define SENSOR_NAME "imx258_mipi"
#define SENSOR_NAME_2 "imx258_mipi_2"
#define WIN_H_START (0x0000)
#define WIN_H_END 	(0x106F)
#define WIN_V_START (0x0000)
#define WIN_V_END 	(0x0C2F)
/*
 * The default register settings
 */

static struct regval_list sensor_default_regs[] = {
	/* External Clock Settings */
	{0x0136, 0x0C},/* 12MHZ MCLK */
	{0x0137, 0x00},
	/* Global Settings */
	{0x3051, 0x00},
	{0x6B11, 0xCF},
	{0x7FF0, 0x08},
	{0x7FF1, 0x0F},
	{0x7FF2, 0x08},
	{0x7FF3, 0x1B},
	{0x7FF4, 0x23},
	{0x7FF5, 0x60},
	{0x7FF6, 0x00},
	{0x7FF7, 0x01},
	{0x7FF8, 0x00},
	{0x7FF9, 0x78},
	{0x7FFA, 0x01},
	{0x7FFB, 0x00},
	{0x7FFC, 0x00},
	{0x7FFD, 0x00},
	{0x7FFE, 0x00},
	{0x7FFF, 0x03},
	{0x7F76, 0x03},
	{0x7F77, 0xFE},
	{0x7FA8, 0x03},
	{0x7FA9, 0xFE},
	{0x7B24, 0x81},
	{0x7B25, 0x01},
	{0x6564, 0x07},
	{0x6B0D, 0x41},
	{0x653D, 0x04},
	{0x6B05, 0x8C},
	{0x6B06, 0xF9},
	{0x6B08, 0x65},
	{0x6B09, 0xFC},
	{0x6B0A, 0xCF},
	{0x6B0B, 0xD2},
	{0x6700, 0x0E},
	{0x6707, 0x0E},
	{0x5F04, 0x00},
	{0x5F05, 0xED},
	/* Image Quality Adjusment Settings */
	/* Defect Pixel Correction settings */
	{0x94C7, 0xFF},
	{0x94C8, 0xFF},
	{0x94C9, 0xFF},
	{0x95C7, 0xFF},
	{0x95C8, 0xFF},
	{0x95C9, 0xFF},
	{0x94C4, 0x3F},
	{0x94C5, 0x3F},
	{0x94C6, 0x3F},
	{0x95C4, 0x3F},
	{0x95C5, 0x3F},
	{0x95C6, 0x3F},
	{0x94C1, 0x02},
	{0x94C2, 0x02},
	{0x94C3, 0x02},
	{0x95C1, 0x02},
	{0x95C2, 0x02},
	{0x95C3, 0x02},
	{0x94BE, 0x0C},
	{0x94BF, 0x0C},
	{0x94C0, 0x0C},
	{0x95BE, 0x0C},
	{0x95BF, 0x0C},
	{0x95C0, 0x0C},
	{0x94D0, 0x74},
	{0x94D1, 0x74},
	{0x94D2, 0x74},
	{0x95D0, 0x74},
	{0x95D1, 0x74},
	{0x95D2, 0x74},
	{0x94CD, 0x2E},
	{0x94CE, 0x2E},
	{0x94CF, 0x2E},
	{0x95CD, 0x2E},
	{0x95CE, 0x2E},
	{0x95CF, 0x2E},
	{0x94CA, 0x4C},
	{0x94CB, 0x4C},
	{0x94CC, 0x4C},
	{0x95CA, 0x4C},
	{0x95CB, 0x4C},
	{0x95CC, 0x4C},
	{0x900E, 0x32},
	{0x94E2, 0xFF},
	{0x94E3, 0xFF},
	{0x94E4, 0xFF},
	{0x95E2, 0xFF},
	{0x95E3, 0xFF},
	{0x95E4, 0xFF},
	{0x94DF, 0x6E},
	{0x94E0, 0x6E},
	{0x94E1, 0x6E},
	{0x95DF, 0x6E},
	{0x95E0, 0x6E},
	{0x95E1, 0x6E},
	{0x7FCC, 0x01},
	{0x7B78, 0x00},
	{0x9401, 0x35},
	{0x9403, 0x23},
	{0x9405, 0x23},
	{0x9406, 0x00},
	{0x9407, 0x31},
	{0x9408, 0x00},
	{0x9409, 0x1B},
	{0x940A, 0x00},
	{0x940B, 0x15},
	{0x940D, 0x3F},
	{0x940F, 0x3F},
	{0x9411, 0x3F},
	{0x9413, 0x64},
	{0x9415, 0x64},
	{0x9417, 0x64},
	{0x941D, 0x34},
	{0x941F, 0x01},
	{0x9421, 0x01},
	{0x9423, 0x01},
	{0x9425, 0x23},
	{0x9427, 0x23},
	{0x9429, 0x23},
	{0x942B, 0x2F},
	{0x942D, 0x1A},
	{0x942F, 0x14},
	{0x9431, 0x3F},
	{0x9433, 0x3F},
	{0x9435, 0x3F},
	{0x9437, 0x6B},
	{0x9439, 0x7C},
	{0x943B, 0x81},
	{0x9443, 0x0F},
	{0x9445, 0x0F},
	{0x9447, 0x0F},
	{0x9449, 0x0F},
	{0x944B, 0x0F},
	{0x944D, 0x0F},
	{0x944F, 0x1E},
	{0x9451, 0x0F},
	{0x9453, 0x0B},
	{0x9455, 0x28},
	{0x9457, 0x13},
	{0x9459, 0x0C},
	{0x945D, 0x00},
	{0x945E, 0x00},
	{0x945F, 0x00},
	{0x946D, 0x00},
	{0x946F, 0x10},
	{0x9471, 0x10},
	{0x9473, 0x40},
	{0x9475, 0x2E},
	{0x9477, 0x10},
	{0x9478, 0x0A},
	{0x947B, 0xE0},
	{0x947C, 0xE0},
	{0x947D, 0xE0},
	{0x947E, 0xE0},
	{0x947F, 0xE0},
	{0x9480, 0xE0},
	{0x9483, 0x14},
	{0x9485, 0x14},
	{0x9487, 0x14},
	{0x9501, 0x35},
	{0x9503, 0x14},
	{0x9505, 0x14},
	{0x9507, 0x31},
	{0x9509, 0x1B},
	{0x950B, 0x15},
	{0x950D, 0x1E},
	{0x950F, 0x1E},
	{0x9511, 0x1E},
	{0x9513, 0x64},
	{0x9515, 0x64},
	{0x9517, 0x64},
	{0x951D, 0x34},
	{0x951F, 0x01},
	{0x9521, 0x01},
	{0x9523, 0x01},
	{0x9525, 0x14},
	{0x9527, 0x14},
	{0x9529, 0x14},
	{0x952B, 0x2F},
	{0x952D, 0x1A},
	{0x952F, 0x14},
	{0x9531, 0x1E},
	{0x9533, 0x1E},
	{0x9535, 0x1E},
	{0x9537, 0x6B},
	{0x9539, 0x7C},
	{0x953B, 0x81},
	{0x9543, 0x0F},
	{0x9545, 0x0F},
	{0x9547, 0x0F},
	{0x9549, 0x0F},
	{0x954B, 0x0F},
	{0x954D, 0x0F},
	{0x954F, 0x15},
	{0x9551, 0x0B},
	{0x9553, 0x08},
	{0x9555, 0x1C},
	{0x9557, 0x0D},
	{0x9559, 0x08},
	{0x955D, 0x00},
	{0x955E, 0x00},
	{0x955F, 0x00},
	{0x956D, 0x00},
	{0x956F, 0x10},
	{0x9571, 0x10},
	{0x9573, 0x40},
	{0x9575, 0x2E},
	{0x9577, 0x10},
	{0x9578, 0x0A},
	{0x957B, 0xE0},
	{0x957C, 0xE0},
	{0x957D, 0xE0},
	{0x957E, 0xE0},
	{0x957F, 0xE0},
	{0x9580, 0xE0},
	{0x9583, 0x14},
	{0x9585, 0x14},
	{0x9587, 0x14},
	{0x7F78, 0x00},
	{0x7F89, 0x00},
	{0x7F93, 0x00},
	{0x924B, 0x1B},
	{0x924C, 0x0A},
	{0x9304, 0x04},
	{0x9315, 0x04},
	{0x9250, 0x50},
	{0x9251, 0x3C},
	{0x9252, 0x14},
	{0x3031, 0x00},
	/* Output Format Settings */
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	/* Clock Settings */
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0x20},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030D, 0x02},
	{0x030E, 0x00},
	{0x030F, 0x60},
	{0x0310, 0x01},
	{0x0820, 0x14},
	{0x0821, 0x40},
	{0x0822, 0x00},
	{0x0823, 0x00},
	/* clock adjustment setting */
	{0x4648, 0x7F},
	{0x7420, 0x00},
	{0x7421, 0x1C},
	{0x7422, 0x00},
	{0x7423, 0xD7},
	{0x9104, 0x00},
	/* Gain Settings */
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	/* AF Window */
	{0x7BCD, 0x00},
	/* Added Settings IQ */
	{0x94DC, 0x20},
	{0x94DD, 0x20},
	{0x94DE, 0x20},
	{0x95DC, 0x20},
	{0x95DD, 0x20},
	{0x95DE, 0x20},
	{0x7FB0, 0x00},
	{0x9010, 0x3E},
	{0x9419, 0x50},
	{0x941B, 0x50},
	{0x9519, 0x50},
	{0x951B, 0x50},
	/* Added Settings mode */
	{0x3030, 0x01},
	{0x3032, 0x01},
	{0x0220, 0x00},
	{0x0108, 0x03},
};
static struct regval_list sensor_4K_30fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD3},
	{0x030F, 0xD3},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x0C},
	{0x0341, 0x4A},
	/* ROI Settings */
	{0x0344, WIN_H_START>>8},    /* X_ADD_STA */
	{0x0345, WIN_H_START&0xFF},    /* X_ADD_STA */
	{0x0346, WIN_V_START>>8},
	{0x0347, WIN_V_START&0xFF},
	{0x0348, WIN_H_END>>8},    /* X_ADD_END */
	{0x0349, WIN_H_END&0xFF},    /* X_ADD_END */
	{0x034A, WIN_V_END>>8},
	{0x034B, WIN_V_END&0xFF},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	/* Digital Image Size Settings */
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},    /* IMAGE_WIDTH  X */
	{0x040D, 0x00},    /* IMAGE_WIDTH  X */
	{0x040E, 0x08},
	{0x040F, 0x70},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x10},    /* X */
	{0x034D, 0x00},    /* X */
	{0x034E, 0x08},
	{0x034F, 0x70},
	/* Integration Time Settings */
	{0x0202, 0x0C},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x10},
	{0x0821, 0x68},
	/* streaming setting */
	{0x0100, 0x01},
};
#if 1
static struct regval_list sensor_4k_20fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD3},
	{0x030F, 0xD3},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x12},
	{0x0341, 0x70},
	/* ROI Settings */
	{0x0344, WIN_H_START>>8},
	{0x0345, WIN_H_START&0xFF},
	{0x0346, WIN_V_START>>8},
	{0x0347, WIN_V_START&0xFF},
	{0x0348, WIN_H_END>>8},
	{0x0349, WIN_H_END&0xFF},
	{0x034A, WIN_V_END>>8},
	{0x034B, WIN_V_END&0xFF},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	/* Digital Image Size Settings */
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},
	{0x040D, 0x00},
	{0x040E, 0x08},
	{0x040F, 0x70},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x10},
	{0x034D, 0x00},
	{0x034E, 0x08},
	{0x034F, 0x70},
	/* Integration Time Settings */
	{0x0202, 0x0C},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x10},
	{0x0821, 0x68},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_1080P_30fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD3},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x0C},
	{0x0341, 0x4A},
	/* ROI Settings */
	{0x0344, 0x00},	 /* X_ADD_STA */
	{0x0345, 0x00},    /* X_ADD_STA */
	{0x0346, 0x01},
	{0x0347, 0xD8},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},	 /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},	 /* IMAGE_WIDTH	X */
	{0x040D, 0x00},	 /* IMAGE_WIDTH	X */
	{0x040E, 0x04},
	{0x040F, 0x40},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x07},	 /* X */
	{0x034D, 0x80},	 /* X */
	{0x034E, 0x04},
	{0x034F, 0x38},
	/* Integration Time Settings */
	{0x0202, 0x0C},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_1080P_60fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD8},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x06},
	{0x0341, 0x46},
	/* ROI Settings */
	{0x0344, 0x00},	 /* X_ADD_STA */
	{0x0345, 0x00},    /* X_ADD_STA */
	{0x0346, 0x01},
	{0x0347, 0xD8},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},	 /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},	 /* IMAGE_WIDTH	X */
	{0x040D, 0x00},	 /* IMAGE_WIDTH	X */
	{0x040E, 0x04},
	{0x040F, 0x40},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x07},	 /* X */
	{0x034D, 0x80},	 /* X */
	{0x034E, 0x04},
	{0x034F, 0x38},
	/* Integration Time Settings */
	{0x0202, 0x03},
	{0x0203, 0x8E},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_1080P_80fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD8},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x04},
	{0x0341, 0xB0},
	/* ROI Settings */
	{0x0344, 0x00},   /* X_ADD_STA */
	{0x0345, 0x00},	/* X_ADD_STA */
	{0x0346, 0x01},
	{0x0347, 0xD8},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},    /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},    /* IMAGE_WIDTH  X */
	{0x040D, 0x00},    /* IMAGE_WIDTH  X */
	{0x040E, 0x04},
	{0x040F, 0x40},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x07},    /* X */
	{0x034D, 0x80},    /* X */
	{0x034E, 0x04},
	{0x034F, 0x38},
	/* Integration Time Settings */
	{0x0202, 0x04},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_1080P_100fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xF6},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x04},
	{0x0341, 0x4C},
	/* ROI Settings */
	{0x0344, 0x00},   /* X_ADD_STA */
	{0x0345, 0x00},	/* X_ADD_STA */
	{0x0346, 0x01},
	{0x0347, 0xD8},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},    /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x12},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},    /* IMAGE_WIDTH  X */
	{0x040D, 0x00},    /* IMAGE_WIDTH  X */
	{0x040E, 0x04},
	{0x040F, 0x40},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x07},    /* X */
	{0x034D, 0x80},    /* X */
	{0x034E, 0x04},
	{0x034F, 0x38},
	/* Integration Time Settings */
	{0x0202, 0x04},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_720P_50fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD8},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x07},
	{0x0341, 0x88},
	/* ROI Settings */
	{0x0344, 0x00},	 /* X_ADD_STA */
	{0x0345, 0x00},    /* X_ADD_STA */
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},	 /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x14},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x30},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},	 /* IMAGE_WIDTH	X */
	{0x040D, 0x00},	 /* IMAGE_WIDTH	X */
	{0x040E, 0x02},
	{0x040F, 0xD0},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x05},	 /* X */
	{0x034D, 0x00},	 /* X */
	{0x034E, 0x02},
	{0x034F, 0xD0},
	/* Integration Time Settings */
	{0x0202, 0x07},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_720P_60fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD8},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x06},
	{0x0341, 0x46},
	/* ROI Settings */
	{0x0344, 0x00},	 /* X_ADD_STA */
	{0x0345, 0x00},    /* X_ADD_STA */
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},	 /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x14},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x30},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},	 /* IMAGE_WIDTH	X */
	{0x040D, 0x00},	 /* IMAGE_WIDTH	X */
	{0x040E, 0x02},
	{0x040F, 0xD0},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x05},	 /* X */
	{0x034D, 0x00},	 /* X */
	{0x034E, 0x02},
	{0x034F, 0xD0},
	/* Integration Time Settings */
	{0x0202, 0x06},
	{0x0203, 0x00},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
static struct regval_list sensor_720P_120fps_regs[] = {
	/* streaming setting */
	{0x0100, 0x00},
	/* Clock Settings */
	{0x0307, 0xD8},
	{0x030F, 0x78},
	{0x0310, 0x01},
	/* Line Length Settings */
	{0x0342, 0x14},
	{0x0343, 0xE8},
	/* Frame Length Settings */
	{0x0340, 0x03},
	{0x0341, 0x23},
	/* ROI Settings */
	{0x0344, 0x00},	 /* X_ADD_STA */
	{0x0345, 0x00},    /* X_ADD_STA */
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x10},    /* X_ADD_END */
	{0x0349, 0x0F},	 /* X_ADD_END */
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	/* Analog Image Size Settings */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x14},
	/* Digital Image Size Settings */
	{0x0401, 0x01},
	{0x0404, 0x00},
	{0x0405, 0x30},
	{0x0409, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},	 /* IMAGE_WIDTH	X */
	{0x040D, 0x00},	 /* IMAGE_WIDTH	X */
	{0x040E, 0x02},
	{0x040F, 0xD0},
	{0x3038, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x10},
	{0x300D, 0x00},
	/* Output Size Settings */
	{0x034C, 0x05},	 /* X */
	{0x034D, 0x00},	 /* X */
	{0x034E, 0x02},
	{0x034F, 0xD0},
	/* Integration Time Settings */
	{0x0202, 0x03},
	{0x0203, 0x20},
	/* MIPI Global Timing Settings */
	{0x080A, 0x00},
	{0x080B, 119},
	{0x080C, 0x00},
	{0x080D, 55},
	{0x080E, 0x00},
	{0x080F, 103},
	{0x0810, 0x00},
	{0x0811, 55},
	{0x0812, 0x00},
	{0x0813, 55},
	{0x0814, 0x00},
	{0x0815, 55},
	{0x0816, 0x00},
	{0x0817, 223},
	{0x0818, 0x00},
	{0x0819, 47},
	{0x0820, 0x0B},
	{0x0821, 0x40},
	/* streaming setting */
	{0x0100, 0x01},
};
#endif
/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */

static struct regval_list sensor_fmt_raw[] = {

};

static int sensor_get_temp(struct v4l2_subdev *sd,  struct sensor_temp *temp)
{
	data_type rdval = 0;
	sensor_read(sd, 0x013a, &rdval);
	temp->temp = rdval;
	/* sensor_print("tempture: %d.\n", rdval); */
	/* printk("sensor name %s tempture: %d.\n", sd->name,rdval); */
	return 0;
}

/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	*value = info->exp;
	sensor_dbg("sensor_get_exposure = %d\n", info->exp);
	return 0;
}

static int imx258_sensor_vts;
static int frame_count;

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	data_type explow, exphigh;
	struct sensor_info *info = to_state(sd);

	exp_val = (exp_val + 8) >> 4;
	exphigh = (unsigned char)((0xff00 & exp_val) >> 8);
	explow = (unsigned char)((0x00ff & exp_val));

	sensor_write(sd, 0x0203, explow);
	sensor_write(sd, 0x0202, exphigh);

	sensor_dbg("sensor_set_exp = %d line Done!\n", exp_val);

	info->exp = exp_val;
	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	*value = info->gain;
	sensor_dbg("sensor_get_gain = %d\n", info->gain);
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	data_type gainlow = 0;
	data_type gainhigh = 0;
	long gaindigi = 0;
	int gainana = 0;

	if (gain_val <= 128) {
		gainana = 512 - 8192/gain_val;
		gaindigi = 256;
	} else {
		gainana = 448;
		gaindigi = gain_val * 2;
	}

	gainlow = (unsigned char)(gainana&0xff);
	gainhigh = (unsigned char)((gainana>>8)&0xff);

	sensor_write(sd, 0x0205, gainlow);
	sensor_write(sd, 0x0204, gainhigh);

	sensor_write(sd, 0x020F, (unsigned char)(gaindigi&0xff));
	sensor_write(sd, 0x020E, (unsigned char)(gaindigi>>8));
	sensor_write(sd, 0x0211, (unsigned char)(gaindigi&0xff));
	sensor_write(sd, 0x0210, (unsigned char)(gaindigi>>8));
	sensor_write(sd, 0x0213, (unsigned char)(gaindigi&0xff));
	sensor_write(sd, 0x0212, (unsigned char)(gaindigi>>8));
	sensor_write(sd, 0x0215, (unsigned char)(gaindigi&0xff));
	sensor_write(sd, 0x0214, (unsigned char)(gaindigi>>8));

	info->gain = gain_val;

	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
				struct sensor_exp_gain *exp_gain)
{
	int exp_val = 0, gain_val = 0, shutter = 0, frame_length = 0;
	struct sensor_info *info = to_state(sd);

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	shutter = exp_val>>4;
	if (shutter > imx258_sensor_vts-4)
		frame_length = shutter + 4;
	else
		frame_length = imx258_sensor_vts;

	sensor_write(sd, 0x0104, 0x01);
	sensor_s_exp(sd, exp_val);
	sensor_s_gain(sd, gain_val);
	sensor_write(sd, 0x0104, 0x00);
	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret;
	data_type rdval;

	ret = sensor_read(sd, 0x0100, &rdval);
	if (ret != 0)
		return ret;

	if (on_off == STBY_ON)
		ret = sensor_write(sd, 0x0100, rdval & 0xfe);
	else
		ret = sensor_write(sd, 0x0100, rdval | 0x01);

	return ret;
}

/*
 * Stuff that knows about the sensor.
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;

	switch (on) {
	case STBY_ON:
		sensor_dbg("STBY_ON!\n");
		cci_lock(sd);
		ret = sensor_s_sw_stby(sd, STBY_ON);
		if (ret < 0)
			sensor_err("soft stby falied!\n");
		usleep_range(10000, 12000);
		cci_unlock(sd);
		break;
	case STBY_OFF:
		sensor_dbg("STBY_OFF!\n");
		cci_lock(sd);
		usleep_range(10000, 12000);
		ret = sensor_s_sw_stby(sd, STBY_OFF);
		if (ret < 0) {
			sensor_err("soft stby off falied!\n");
		}
		cci_unlock(sd);
		break;
	case PWR_ON:
		sensor_print("PWR_ON!\n");
		/* printk("%s %d %s\n", __FILE__, __LINE__, __func__); */
		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_set_status(sd, POWER_EN, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, IOVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_set_pmu_channel(sd, DVDD, ON);
		usleep_range(7000, 8000);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		/* printk("%s %d %s\n", __FILE__, __LINE__, __func__); */
		break;
	case PWR_OFF:
		sensor_print("PWR_OFF!\n");
		cci_lock(sd);
		/* printk("%s %d %s\n", __FILE__, __LINE__, __func__); */
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_set_mclk(sd, OFF);
		vin_set_pmu_channel(sd, AFVDD, OFF);
		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		vin_gpio_set_status(sd, PWDN, 0);
		vin_gpio_set_status(sd, POWER_EN, 0);
		cci_unlock(sd);
//		printk("%s %d %s\n", __FILE__, __LINE__, __func__);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch (val) {
	case 0:
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type rdval = 0;

	sensor_read(sd, 0x0016, &rdval);
	sensor_print("%s read value1 is 0x%x\n", __func__, rdval);
	sensor_read(sd, 0x0017, &rdval);
	sensor_print("%s read value2 is 0x%x\n", __func__, rdval);

	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	sensor_dbg("sensor_init\n");

	/* Make sure it is a target sensor */
	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = 4208;
	info->height = 3120;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->exp = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30;	/* 30fps */

	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins != NULL) {
			memcpy(arg, info->current_wins,
				sizeof(*info->current_wins));
			ret = 0;
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		ret = 0;
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		ret = sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	case VIDIOC_VIN_SENSOR_GET_TEMP:
	    sensor_get_temp(sd, (struct sensor_temp *)arg);
	    break;
	default:
		return -EINVAL;
	}
	return ret;
}

/*
 * Store information about the video data format.
 */
static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer",
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {
#if 1
	{
		.width		= 4096,
		.height 	= 2160,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 4720,
		.pclk		= 506400000,
		.mipi_bps	= 1050*1000*1000,
		.fps_fixed	= 20,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (3146 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_4k_20fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_4k_20fps_regs),
		.set_size	= NULL,
		.top_clk = 432*1000*1000,
		.isp_clk = 420*1000*1000,

	},
#endif
	{
		.width		= 4096,
		.height 	= 2160,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 3146,
		.pclk		= 506400000,
		.mipi_bps	= 1050*1000*1000,
		.fps_fixed	= 30,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (3146 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_4K_30fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_4K_30fps_regs),
		.set_size	= NULL,
		.top_clk = 432*1000*1000,
		.isp_clk = 420*1000*1000,

	},
#if 1
	{
		.width		= 1920,
		.height 	= 1080,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 1606,
		.pclk		= 516000000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 60,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (1606 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_1080P_60fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_1080P_60fps_regs),
		.set_size	= NULL,
	}, {
		.width		= 1920,
		.height 	= 1080,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 1200,
		.pclk		= 516000000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 80,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (1100 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_1080P_80fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_1080P_80fps_regs),
		.set_size	= NULL,
		.top_clk = 384*1000*1000,
		.isp_clk = 384*1000*1000,

	}, {
		.width		= 1920,
		.height		= 1080,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 1100,
		.pclk		= 516000000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 100,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (1100 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_1080P_100fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_1080P_100fps_regs),
		.set_size	= NULL,
		.top_clk = 384*1000*1000,
		.isp_clk = 384*1000*1000,
	}, {
		.width		= 1920,
		.height		= 1080,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 3146,
		.pclk		= 506400000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 30,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (3146 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_1080P_30fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_1080P_30fps_regs),
		.set_size	= NULL,
	}, {
		.width		= 1280,
		.height		= 720,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 1606,
		.pclk		= 516000000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 60,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (1606 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_720P_60fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_720P_60fps_regs),
		.set_size	= NULL,
	}, {
		.width		= 1280,
		.height		= 720,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 1928,
		.pclk		= 516000000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 50,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (1928 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_720P_50fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_720P_50fps_regs),
		.set_size	= NULL,
	}, {
		.width		= 1280,
		.height		= 720,
		.hoffset	= 0,
		.voffset	= 0,
		.hts		= 5352,
		.vts		= 803,
		.pclk		= 516000000,
		.mipi_bps	= 720*1000*1000,
		.fps_fixed	= 120,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (803 - 4) << 4,
		.gain_min = 1 << 4,
		.gain_max = 1440 << 4,
		.regs		= sensor_720P_120fps_regs,
		.regs_size	= ARRAY_SIZE(sensor_720P_120fps_regs),
		.set_size	= NULL,
		.top_clk = 384*1000*1000,
		.isp_clk = 384*1000*1000,

	},
#endif
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd, struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->flags = 0 | V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}
/*
static int sensor_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls.
	/* see include/linux/videodev2.h for details
	switch (qc->id) {
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, 1 * 16, 128 * 16 - 1, 1, 16);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 1, 65536 * 16, 1, 1);
	}
	return -EINVAL;
}
*/
static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info = container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info = container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static int sensor_reg_init(struct sensor_info *info)
{
	short offset_x = 0;
	int ret = 0;
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;
	int *buf_src = NULL;

	buf_src = (int *)kzalloc(sizeof(int), GFP_KERNEL);

	ret = sensor_write_array(sd, sensor_default_regs, ARRAY_SIZE(sensor_default_regs));
	if (ret < 0) {
		sensor_err("write sensor_default_regs error\n");
		return ret;
	}

	if (!strncmp(sd->name, SENSOR_NAME, sizeof(SENSOR_NAME))) {/* master */
		sensor_write(sd, 0x5A5C, 0x01);
		sensor_write(sd, 0x5A5D, 0x01);
		sensor_write(sd, 0x4635, 0x01);
		sensor_write(sd, 0x5490, 0x04);
		sensor_write(sd, 0x5492, 0x04);
		sensor_write(sd, 0x5493, 0x40);
	} else {/* slave */
		sensor_write(sd, 0x5A5C, 0x00);
		sensor_write(sd, 0x5A5D, 0x00);
		sensor_write(sd, 0x4635, 0x03);
		sensor_write(sd, 0x5490, 0x00);
		sensor_write(sd, 0x5492, 0x08);
		sensor_write(sd, 0x5493, 0x00);
	}
	sensor_dbg("sensor_reg_init\n");

	sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

	if (wsize->regs) {
		sensor_write_array(sd, wsize->regs, wsize->regs_size);
	}

	if (wsize->set_size) {
		wsize->set_size(sd);
	}
	sensor_write(sd, 0x0138, 0x1);
	frame_count = 0;
	offset_x = 0;

	printk("offset_x = %d\n", offset_x);
	sensor_write(sd, 0x0104, 0x01);
	if (wsize->width == wsize->height) {
		if (offset_x > 0) {
			sensor_write(sd, 0x0348, (WIN_H_END+offset_x)>>8);
			sensor_write(sd, 0x0349, (WIN_H_END+offset_x)&0xff);
			sensor_write(sd, 0x0345, (WIN_H_START+offset_x)&0xff);
			sensor_write(sd, 0x0344, (WIN_H_START+offset_x)>>8);
		} else {
			sensor_write(sd, 0x0344, (WIN_H_START+offset_x)>>8);
			sensor_write(sd, 0x0345, (WIN_H_START+offset_x)&0xff);
			sensor_write(sd, 0x0349, (WIN_H_END+offset_x)&0xff);
			sensor_write(sd, 0x0348, (WIN_H_END+offset_x)>>8);
		}
	}
	sensor_write(sd, 0x0104, 0x00);
	info->width = wsize->width;
	info->height = wsize->height;
	imx258_sensor_vts = wsize->vts;

	sensor_print("s_fmt set width = %d, height = %d\n", wsize->width, wsize->height);

	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

/*	printk("%s %d %s\n", __FILE__, __LINE__, __func__); */
	vin_gpio_set_status(sd, SM_VS, 3);
	vin_gpio_set_status(sd, SM_HS, 3);

	sensor_print("%s on = %d, %d*%d %x\n", __func__, enable,
		     info->current_wins->width,
		     info->current_wins->height, info->fmt->mbus_code);

	if (!enable)
		return 0;

	/* printk("%s %d %s\n", __FILE__, __LINE__, __func__); */
	return sensor_reg_init(info);
}

/* ----------------------------------------------------------------------- */
static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
//	.g_ctrl = sensor_g_ctrl,
//	.s_ctrl = sensor_s_ctrl,
//	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.enum_frame_interval = sensor_enum_frame_interval,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv[] = {
	{
		.name = SENSOR_NAME,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}, {
		.name = SENSOR_NAME_2,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}
};
static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 2);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600,
			      256 * 1600, 1, 1 * 1600);
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 0,
			      65536 * 16, 1, 0);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}

	sd->ctrl_handler = handler;

	return ret;
}

static int sensor_dev_id;

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	int i;

	printk("%s %d %s \n", __FILE__, __LINE__, __func__);
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;

	if (client) {
		for (i = 0; i < SENSOR_NUM; i++) {
			if (!strcmp(cci_drv[i].name, client->name))
				break;
		}
		cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv[i]);
	} else {
		cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv[sensor_dev_id++]);
	}
	sensor_init_controls(sd, &sensor_ctrl_ops);
	mutex_init(&info->lock);

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET3 | MIPI_NORMAL_MODE;
	info->af_first_flag = 1;
	info->exp = 0;
	info->gain = 0;

	return 0;
}
static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	int i;

	if (client) {
		for (i = 0; i < SENSOR_NUM; i++) {
			if (!strcmp(cci_drv[i].name, client->name))
				break;
		}
		sd = cci_dev_remove_helper(client, &cci_drv[i]);
	} else {
		sd = cci_dev_remove_helper(client, &cci_drv[sensor_dev_id++]);
	}

	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

static const struct i2c_device_id sensor_id_2[] = {
	{SENSOR_NAME_2, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);
MODULE_DEVICE_TABLE(i2c, sensor_id_2);

static struct i2c_driver sensor_driver[] = {
	{
		.driver = {
			   .owner = THIS_MODULE,
			   .name = SENSOR_NAME,
			   },
		.probe = sensor_probe,
		.remove = sensor_remove,
		.id_table = sensor_id,
	}, {
		.driver = {
			   .owner = THIS_MODULE,
			   .name = SENSOR_NAME_2,
			   },
		.probe = sensor_probe,
		.remove = sensor_remove,
		.id_table = sensor_id_2,
	},
};
static __init int init_sensor(void)
{
	int i, ret = 0;

	sensor_dev_id = 0;

	printk("%s %d %s \n", __FILE__, __LINE__, __func__);
	for (i = 0; i < SENSOR_NUM; i++)
		ret = cci_dev_init_helper(&sensor_driver[i]);

	return ret;
}

static __exit void exit_sensor(void)
{
	int i;

	sensor_dev_id = 0;

	for (i = 0; i < SENSOR_NUM; i++)
		cci_dev_exit_helper(&sensor_driver[i]);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
