/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for imx415 Raw cameras.
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

MODULE_AUTHOR("cwh");
MODULE_DESCRIPTION("A low-level driver for IMX415 sensors");
MODULE_LICENSE("GPL");

#define MCLK              (27*1000*1000)
#define V4L2_IDENT_SENSOR 0x0415


/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 30

/*
 * The IMX477 i2c address
 */
#define I2C_ADDR 0x34

#define SENSOR_NUM 0x2
#define SENSOR_NAME "imx415_mipi"
#define SENSOR_NAME_2 "imx415_mipi_2"

/*
 * The default register settings
 */
static struct regval_list sensor_default_regs[] = {

};
static struct regval_list sensor_12bit_30fps_regs[] = {
/* All pixel - 891Mbps - 27MHZ */
	{0x3000, 0x01},/* stanby                  */
	{0x3001, 0x00},
	{0x3002, 0x01},/* XMSTA start             */
	{0x3003, 0x00},/* XMASTER SELECT MASTER   */
	{0x3008, 0x5D},
	{0x3009, 0x00},
	{0x300A, 0x42},
	{0x300B, 0xA0},
	{0x301C, 0x00},
	{0x301D, 0x08},
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xCA},/* VMAX */
	{0x3025, 0x08},
	{0x3026, 0x00},
	{0x3028, 0x4C},/* HMAX */
	{0x3029, 0x04},/*    */
	{0x302C, 0x00},/* DOL mode 0:normal 1:dol                                                   */
	{0x302D, 0x00},/* wdsel 0:normal exposure 1:dol_2  2:dol_3 3:multiple exposure 4frame       */
	{0x3030, 0x00},/*                                                                          */
	{0x3031, 0x01},/* adbit  0:10bit ,1 :12bit                                                  */
	{0x3032, 0x01},/* mdbit 0:10bit 1:12bit                                                     */
	{0x3033, 0x05},
	{0x3040, 0x00},/* window cropping                                                           */
	{0x3041, 0x00},
	{0x3042, 0x18},
	{0x3043, 0x0F},
	{0x3044, 0x18},
	{0x3045, 0x0F},
	{0x3046, 0x20},
	{0x3047, 0x11},
	{0x3050, 0x08},/* shr0        */
	{0x3051, 0x00},
	{0x3052, 0x00},
	{0x3054, 0x19},/* shr1        */
	{0x3055, 0x00},/*            */
	{0x3056, 0x00},/*            */
	{0x3058, 0x3E},/* shr2        */
	{0x3059, 0x00},/*            */
	{0x305E, 0x00},/*            */
	{0x3060, 0x25},/* rhs1        */
	{0x3061, 0x00},/*            */
	{0x3062, 0x00},/*            */
	{0x3064, 0x4A},/* rhs2        */
	{0x3065, 0x00},/*            */
	{0x3066, 0x00},/*            */
	{0x3090, 0x00},/* gain_pcg_0  */
	{0x3091, 0x00},/*            */
	{0x3092, 0x00},/* gain_pcg_1  */
	{0x3093, 0x00},/*            */
	{0x3094, 0x00},/* gain_pcg_2  */
	{0x3095, 0x00},
	{0x30C0, 0x2A},
	{0x30C1, 0x00},
	{0x30CC, 0x00},
	{0x30CD, 0x00},
	{0x30CF, 0x00},
	{0x30D9, 0x06},
	{0x30DA, 0x02},
	{0x30E2, 0x32},/* BLV */
	{0x30E3, 0x00},
	{0x3115, 0x00},
	{0x3116, 0x23},
	{0x3118, 0xC6},
	{0x3119, 0x00},
	{0x311A, 0xE7},
	{0x311B, 0x00},
	{0x311E, 0x23},
	{0x3260, 0x01},
	{0x32C8, 0x01},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x03},/* ADBIT 00h:10bit  03h:12bit */
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4000, 0x10},
	{0x4001, 0x03},
	{0x4004, 0xC0},
	{0x4005, 0x06},
	{0x400C, 0x00},
	{0x4018, 0x7F},
	{0x4019, 0x00},
	{0x401A, 0x37},
	{0x401B, 0x00},
	{0x401C, 0x37},
	{0x401D, 0x00},
	{0x401E, 0xF7},
	{0x401F, 0x00},
	{0x4020, 0x3F},
	{0x4021, 0x00},
	{0x4022, 0x6F},
	{0x4023, 0x00},
	{0x4024, 0x3F},
	{0x4025, 0x00},
	{0x4026, 0x5F},
	{0x4027, 0x00},
	{0x4028, 0x2F},
	{0x4029, 0x00},
	{0x4074, 0x01},
	{0x3000, 0x00},/* operation */
	{0x3002, 0x00},
};

static struct regval_list sensor_10bit_30fps_regs[] = {
/* ALL PIXEL  - 720Mbps - 24MHZ */
	{0x3000, 0x01},/* stanby */
	{0x3001, 0x00},
	{0x3008, 0x54},
	{0x300A, 0x3B},
	{0x3024, 0xCB},/* VMAX */
	{0x3025, 0x08},
	{0x3026, 0x00},
	{0x3028, 0x2A},/* HMAX */
	{0x3029, 0x04},
	{0x3031, 0x00},
	{0x3032, 0x00},
	{0x3033, 0x09},
	{0x3050, 0x08},
	{0x30C1, 0x00},
	{0x3116, 0x23},
	{0x3118, 0xB4},
	{0x311A, 0xFC},
	{0x311E, 0x23},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x00},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4004, 0x00},
	{0x4005, 0x06},
	{0x400C, 0x00},
	{0x4018, 0x6F},
	{0x401A, 0x2F},
	{0x401C, 0x2F},
	{0x401E, 0xBF},
	{0x401F, 0x00},
	{0x4020, 0x2F},
	{0x4022, 0x57},
	{0x4024, 0x2F},
	{0x4026, 0x4F},
	{0x4028, 0x27},
	{0x4074, 0x01},
	{0x3000, 0x00}, /* operation */
	{0x3002, 0x00},
};
static struct regval_list sensor_10bit_60fps_regs[] = {

/* All Pixel - 1440Mbps - 24MHZ */
	{0x3000, 0x01},/*
	{0x3001, 0x00},/*
	{0x3008, 0x54},/* BCWAIT_TIME[9:0] */
	{0x300A, 0x3B},/* CPWAIT_TIME[9:0] */
	{0x3024, 0xCB},/* VMAX[19:0]      */
	{0x3025, 0x08},
	{0x3028, 0x15},/* HMAX[15:0]      */
	{0x3029, 0x02},/*                 */
	{0x3031, 0x00},/* ADBIT[1:0]      */
	{0x3032, 0x00},/* MDBIT           */
	{0x3033, 0x08},/* SYS_MODE[3:0]   */
	{0x3050, 0x08},/* SHR0[19:0]      */
	{0x30C1, 0x00},/* XVS_DRV[1:0]    */
	{0x3116, 0x23},/* INCKSEL2[7:0]   */
	{0x3118, 0xB4},/* INCKSEL3[10:0]  */
	{0x311A, 0xFC},/* INCKSEL4[10:0]  */
	{0x311E, 0x23},/* INCKSEL5[7:0]   */
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x00},/* ADBIT1[7:0] */
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4004, 0x00},/* TXCLKESC_FREQ[15:0]  */
	{0x4005, 0x06},/*                      */
	{0x4018, 0x9F},/* TCLKPOST[15:0]       */
	{0x401A, 0x57},/* TCLKPREPARE[15:0]    */
	{0x401C, 0x57},/* TCLKTRAIL[15:0]      */
	{0x401E, 0x87},/* TCLKZERO[15:0]       */
	{0x4020, 0x5F},/* THSPREPARE[15:0]     */
	{0x4022, 0xA7},/* THSZERO[15:0]        */
	{0x4024, 0x5F},/* THSTRAIL[15:0]       */
	{0x4026, 0x97},/* THSEXIT[15:0]        */
	{0x4028, 0x4F},/* TLPX[15:0]           */
	{0x3000, 0x00},/* operation             */
	{0x3002, 0x00},
};
static struct regval_list sensor_12bit_60fps_regs[] = {
	/* All Pixel - 1782Mbps - 27MHz */
	{0x3000, 0x01},
	{0x3001, 0x00},
	{0x3008, 0x5D}, /* BCWAIT_TIME[9:0] */
	{0x300A, 0x42}, /* CPWAIT_TIME[9:0] */
	{0x3050, 0x08}, /* SHR0[19:0] */
	{0x3024, 0xCA}, /*  VMAX[19:0] */
	{0x3025, 0x08},
	{0x3026, 0x00},
	{0x3028, 0x26}, /* HMAX[15:0] */
	{0x3029, 0x02},
	{0x30C1, 0x00}, /* XVS_DRV[1:0]  */
	{0x3116, 0x23}, /* INCKSEL2[7:0] */
	{0x3118, 0xC6}, /* INCKSEL3[10:0] */
	{0x311A, 0xE7}, /* INCKSEL4[10:0] */
	{0x311E, 0x23}, /* INCKSEL5[7:0] */
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4004, 0xC0}, /* TXCLKESC_FREQ[15:0] */
	{0x4005, 0x06},
	{0x3000, 0x00}, /* operation */
	{0x3002, 0x00},
};
static struct regval_list sensor_10bit_90fps_regs[] = {
	/* All Pixel - 23/* Mbps - 27MHz */
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3008, 0x5D}, /* BCWAIT_TIME[9:0] */
	{0x300A, 0x42}, /* CPWAIT_TIME[9:0] */
	{0x3024, 0xCE}, /* VMAX             */
	{0x3025, 0x08},
	{0x3026, 0x00},
	{0x3028, 0x6E}, /* HMAX            */
	{0x3029, 0x01},
	{0x3031, 0x00}, /* ADBIT[1:0]      */
	{0x3032, 0x00}, /* MDBIT           */
	{0x3033, 0x00}, /* SYS_MODE[3:0]   */
	{0x3050, 0x08}, /* SHR0[19:0]      */
	{0x30C1, 0x00}, /* XVS_DRV[1:0]    */
	{0x3116, 0x23}, /* INCKSEL2[7:0]   */
	{0x3118, 0x08}, /* INCKSEL3[10:0]  */
	{0x3119, 0x01},
	{0x311A, 0xE7}, /* INCKSEL4[10:0]  */
	{0x311E, 0x23}, /* INCKSEL5[7:0]   */
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x00}, /* ADBIT1[7:0] */
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4004, 0xC0}, /* TXCLKESC_FREQ[15:0] */
	{0x4005, 0x06},
	{0x4018, 0xE7}, /* TCLKPOST[15:0]     */
	{0x401A, 0x8F}, /* TCLKPREPARE[15:0]  */
	{0x401C, 0x8F}, /* TCLKTRAIL[15:0]    */
	{0x401E, 0x7F}, /* TCLKZERO[15:0]     */
	{0x401F, 0x02},
	{0x4020, 0x97}, /* THSPREPARE[15:0]   */
	{0x4022, 0x0F}, /* THSZERO[15:0]      */
	{0x4023, 0x01},
	{0x4024, 0x97}, /* THSTRAIL[15:0]     */
	{0x4026, 0xF7}, /* THSEXIT[15:0]      */
	{0x4028, 0x7F}, /* TLPX[15:0]         */
	{0x3000, 0x00}, /* operation           */
	{0x3002, 0x00},
};
/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */

static struct regval_list sensor_fmt_raw[] = {

};


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
static int imx415_mipi_sensor_vts;
static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	data_type explow, exphigh, expmid;
	int exptime;
	struct sensor_info *info = to_state(sd);
	if (exp_val) {
		exptime = imx415_mipi_sensor_vts - (exp_val >> 4);
		exphigh = (unsigned char)((0x00f0000 & exptime) >> 16);
		expmid =  (unsigned char)((0x000ff00 & exptime) >> 8);
		explow =  (unsigned char)((0x00000ff & exptime));
		sensor_write(sd, 0x3050, explow);
		sensor_write(sd, 0x3051, expmid);
		sensor_write(sd, 0x3052, exphigh);
		sensor_dbg("sensor_set_exp = %d %d line Done!\n", exp_val, exptime);
	} else {
			sensor_write(sd, 0x3050, 0x00);
			sensor_write(sd, 0x3051, 0x00);
			sensor_write(sd, 0x3052, 0x66);
		}

	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	*value = info->gain;
	sensor_dbg("sensor_get_gain = %d\n", info->gain);
	return 0;
}

unsigned char gain2db[497] = {
	0,   2,   3,	 5,   6,   8,	9,  11,  12,  13,  14,	15,  16,  17,
	18,  19,  20,	21,  22,  23,  23,  24,  25,  26,  27,	27,  28,  29,
	29,  30,  31,	31,  32,  32,  33,  34,  34,  35,  35,	36,  36,  37,
	37,  38,  38,	39,  39,  40,  40,  41,  41,  41,  42,	42,  43,  43,
	44,  44,  44,	45,  45,  45,  46,  46,  47,  47,  47,	48,  48,  48,
	49,  49,  49,	50,  50,  50,  51,  51,  51,  52,  52,	52,  52,  53,
	53,  53,  54,	54,  54,  54,  55,  55,  55,  56,  56,	56,  56,  57,
	57,  57,  57,	58,  58,  58,  58,  59,  59,  59,  59,	60,  60,  60,
	60,  60,  61,	61,  61,  61,  62,  62,  62,  62,  62,	63,  63,  63,
	63,  63,  64,	64,  64,  64,  64,  65,  65,  65,  65,	65,  66,  66,
	66,  66,  66,	66,  67,  67,  67,  67,  67,  68,  68,	68,  68,  68,
	68,  69,  69,	69,  69,  69,  69,  70,  70,  70,  70,	70,  70,  71,
	71,  71,  71,	71,  71,  71,  72,  72,  72,  72,  72,	72,  73,  73,
	73,  73,  73,	73,  73,  74,  74,  74,  74,  74,  74,	74,  75,  75,
	75,  75,  75,	75,  75,  75,  76,  76,  76,  76,  76,	76,  76,  77,
	77,  77,  77,	77,  77,  77,  77,  78,  78,  78,  78,	78,  78,  78,
	78,  79,  79,	79,  79,  79,  79,  79,  79,  79,  80,	80,  80,  80,
	80,  80,  80,	80,  80,  81,  81,  81,  81,  81,  81,	81,  81,  81,
	82,  82,  82,	82,  82,  82,  82,  82,  82,  83,  83,	83,  83,  83,
	83,  83,  83,	83,  83,  84,  84,  84,  84,  84,  84,	84,  84,  84,
	84,  85,  85,	85,  85,  85,  85,  85,  85,  85,  85,	86,  86,  86,
	86,  86,  86,	86,  86,  86,  86,  86,  87,  87,  87,	87,  87,  87,
	87,  87,  87,	87,  87,  88,  88,  88,  88,  88,  88,	88,  88,  88,
	88,  88,  88,	89,  89,  89,  89,  89,  89,  89,  89,	89,  89,  89,
	89,  90,  90,	90,  90,  90,  90,  90,  90,  90,  90,	90,  90,  91,
	91,  91,  91,	91,  91,  91,  91,  91,  91,  91,  91,	91,  92,  92,
	92,  92,  92,	92,  92,  92,  92,  92,  92,  92,  92,	93,  93,  93,
	93,  93,  93,	93,  93,  93,  93,  93,  93,  93,  93,	94,  94,  94,
	94,  94,  94,	94,  94,  94,  94,  94,  94,  94,  94,	95,  95,  95,
	95,  95,  95,	95,  95,  95,  95,  95,  95,  95,  95,	95,  96,  96,
	96,  96,  96,	96,  96,  96,  96,  96,  96,  96,  96,	96,  96,  97,
	97,  97,  97,	97,  97,  97,  97,  97,  97,  97,  97,	97,  97,  97,
	97,  98,  98,	98,  98,  98,  98,  98,  98,  98,  98,	98,  98,  98,
	98,  98,  98,	99,  99,  99,  99,  99,  99,  99,  99,	99,  99,  99,
	99,  99,  99,	99,  99,  99, 100, 100, 100, 100, 100, 100, 100, 100,
	100, 100, 100, 100, 100, 100, 100,
};
static char gain_mode_buf = 0x02;
static unsigned int count;

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	int ret;
	data_type rdval;
	if (gain_val < 1 * 16)
		gain_val = 16;

	if (gain_val < 32 * 16) {
		sensor_write(sd, 0x3090, gain2db[gain_val - 16]);
		sensor_write(sd, 0x3091, 0);
	} else if (gain_val < 1024 * 16) {
		sensor_write(sd, 0x3090, gain2db[(gain_val>>5) - 16] + 100);
		sensor_write(sd, 0x3091, 0);
	} else {
		sensor_write(sd, 0x3090, (gain2db[(gain_val>>10) - 16] + 200) && 0xff);
		sensor_write(sd, 0x3091, (gain2db[(gain_val>>10) - 16] + 200) >> 8);
	}

	sensor_dbg("sensor_set_gain = %d, Done!\n", gain_val);
	info->gain = gain_val;

	return 0;
}


static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	int shutter, frame_length;
	struct sensor_info *info = to_state(sd);
	int exp_val, gain_val;
	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	if (gain_val < 1*16)
		gain_val = 16;
	if (gain_val > 352*16 - 1)
		gain_val = 352*16 - 1;

	if (exp_val > 0xfffff)
		exp_val = 0xfffff;

	shutter = exp_val / 16;
	if (shutter > imx415_mipi_sensor_vts - 4)
		frame_length = shutter + 4;
	else
		frame_length = imx415_mipi_sensor_vts;

	sensor_write(sd, 0x0341, (frame_length & 0xff));
	sensor_write(sd, 0x0340, (frame_length >> 8));
	sensor_write(sd, 0x3001, 0x01);
	sensor_s_exp(sd, exp_val);
	sensor_s_gain(sd, gain_val);
	sensor_write(sd, 0x3001, 0x00);
	sensor_dbg("sensor_set_gain exp = %d, %d Done!\n", gain_val, exp_val);

	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret;
	data_type rdval;

	ret = sensor_read(sd, 0x3000, &rdval);
	if (ret != 0)
		return ret;

	if (on_off == STBY_ON)
		ret = sensor_write(sd, 0x0300, rdval & 0xfe);
	else
		ret = sensor_write(sd, 0x0300, rdval | 0x01);
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
		usleep_range(1000, 1200);
		cci_unlock(sd);
		break;
	case STBY_OFF:
		sensor_dbg("STBY_OFF!\n");
		cci_lock(sd);
		usleep_range(1000, 1200);
		ret = sensor_s_sw_stby(sd, STBY_OFF);
		if (ret < 0)
			sensor_err("soft stby off falied!\n");
		cci_unlock(sd);
		break;
	case PWR_ON:
		sensor_dbg("PWR_ON!\n");
		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_set_status(sd, POWER_EN, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, IOVDD, ON);
		usleep_range(2000, 2200);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_set_pmu_channel(sd, DVDD, ON);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		usleep_range(100, 120);
		vin_set_mclk(sd, ON);
		usleep_range(100, 120);
		vin_set_mclk_freq(sd, MCLK);
		usleep_range(3000, 3200);
		cci_unlock(sd);
		break;
	case PWR_OFF:
		sensor_dbg("PWR_OFF!\n");
		cci_lock(sd);
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
	int cnt = 0;
	sensor_read(sd, 0x3008, &rdval);
	sensor_print("%s read value is 0x%x\n", __func__, rdval);
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
	info->width = 3840;
	info->height = 2160;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;

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
		.mbus_code = MEDIA_BUS_FMT_SGBRG12_1X12,
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
/*
	{  /* 3840*2160 30fps 10bit
	 .width = 3840,
	 .height = 2160,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 1066,
	 .vts = 2251,
	 .pclk = 72 * 1000 * 1000,
	 .mipi_bps = 720 * 1000 * 1000,
	 .fps_fixed = 30,
	 .bin_factor = 1,
	 .intg_min = 8 << 4,
	 .intg_max = (2251 - 4) << 4,
	 .gain_min = 1<<4,
	 .gain_max = 5631<<4,
	 .regs = sensor_10bit_30fps_regs,
	 .regs_size = ARRAY_SIZE(sensor_10bit_30fps_regs),
	 .set_size = NULL,

	 },
	*/
	  #if 1
	 {  /* 30fps 12bit */
	 .width = 3840,
	 .height = 2160,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 1100,
	 .vts = 2250,
	 .pclk = 74 * 1000 * 1000,
	 .mipi_bps = 891 * 1000 * 1000,
	 .fps_fixed = 30,
	 .bin_factor = 1,
	 .intg_min = 8<<4,
	 .intg_max = (2250 - 4) << 4,
	 .gain_min = 1<<4,
	 .gain_max = 5631<<4,
	 .regs = sensor_12bit_30fps_regs,
	 .regs_size = ARRAY_SIZE(sensor_12bit_30fps_regs),
	 .set_size = NULL,

	 },
	 #endif
	 /*
	 {  /* 60fps 10bit
	 .width = 1920,
	 .height = 1080,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 533,
	 .vts = 2251,
	 .pclk = 72 * 1000 * 1000,
	 .mipi_bps = 1440 * 1000 * 1000,
	 .fps_fixed = 60,
	 .bin_factor = 1,
	 .intg_min = 8 << 4,
	 .intg_max = (2251 - 4) << 4,
	 .gain_min = 1<<4,
	 .gain_max = 5631<<4,
	 .regs = sensor_10bit_60fps_regs,
	 .regs_size = ARRAY_SIZE(sensor_10bit_60fps_regs),
	 .set_size = NULL,
	 },
	 */
	/*
	 {  /* 60fps 10bit
	 .width = 3840,
	 .height = 2160,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 533,
	 .vts = 2251,
	 .pclk = 72 * 1000 * 1000,
	 .mipi_bps = 1440 * 1000 * 1000,
	 .fps_fixed = 60,
	 .bin_factor = 1,
	 .intg_min = 8 << 4,
	 .intg_max = (2251 - 4) << 4,
	 .gain_min = 1<<4,
	 .gain_max = 5631<<4,
	 .regs = sensor_10bit_60fps_regs,
	 .regs_size = ARRAY_SIZE(sensor_10bit_60fps_regs),
	 .set_size = NULL,

	 },
	  */
	 /*
	  {  /* 60fps 12bit
	 .width = 3840,
	 .height = 2160,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 550,
	 .vts = 2250,
	 .pclk = 74 * 1000 * 1000,
	 .mipi_bps = 1440 * 1000 * 1000,
	 .fps_fixed = 60,
	 .bin_factor = 1,
	 .intg_min = 8 << 4,
	 .intg_max = (2250 - 4) << 4,
	 .gain_min = 1<<4,
	 .gain_max = 5631<<4,
	 .regs = sensor_12bit_60fps_regs,
	 .regs_size = ARRAY_SIZE(sensor_12bit_60fps_regs),
	 .set_size = NULL,

	 },
	 */
	 /*
	{  /* 90fps 10bit
	 .width = 3840,
	 .height = 2160,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 366,
	 .vts = 2254,
	 .pclk = 74 * 1000 * 1000,
	 .mipi_bps = 2376 * 1000 * 1000,
	 .fps_fixed = 90,
	 .bin_factor = 1,
	 .intg_min = 8 << 4,
	 .intg_max = (2254 - 4) << 4,
	 .gain_min = 1<<4,
	 .gain_max = 5631<<4,
	 .regs = sensor_10bit_90fps_regs,
	 .regs_size = ARRAY_SIZE(sensor_10bit_90fps_regs),
	 .set_size = NULL,
	 },
	 */
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	struct sensor_info *info = to_state(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	if (info->isp_wdr_mode == ISP_DOL_WDR_MODE)
		cfg->flags = 0 | V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNEL_0 | V4L2_MBUS_CSI2_CHANNEL_1;
	else
		cfg->flags = 0 | V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
			container_of(ctrl->handler, struct sensor_info, handler);
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
	struct sensor_info *info =
			container_of(ctrl->handler, struct sensor_info, handler);
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
	int ret;
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;
	data_type rdval = 0;
	/* struct sensor_exp_gain exp_gain; */

	ret = sensor_write_array(sd, sensor_default_regs,
				 ARRAY_SIZE(sensor_default_regs));
	if (ret < 0) {
		sensor_err("write sensor_default_regs error\n");
		return ret;
	}

	sensor_dbg("sensor_reg_init\n");

	sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

	if (wsize->regs)
		sensor_write_array(sd, wsize->regs, wsize->regs_size);

	if (wsize->set_size)
		wsize->set_size(sd);

	info->width = wsize->width;
	info->height = wsize->height;
	imx415_mipi_sensor_vts = wsize->vts;

	/* exp_gain.exp_val = 12480; */
	/* exp_gain.gain_val = 48; */
	/* sensor_s_exp_gain(sd, &exp_gain); */

	sensor_print("s_fmt set width = %d, height = %d\n", wsize->width,
		     wsize->height);

	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_print("%s on = %d, %d*%d %x\n", __func__, enable,
		     info->current_wins->width,
		     info->current_wins->height, info->fmt->mbus_code);
	if (!enable) {
		vin_gpio_set_status(sd, SM_HS, 0);
		vin_gpio_set_status(sd, SM_VS, 0);
			return 0;
	} else {
		vin_gpio_set_status(sd, SM_VS, 3);
		vin_gpio_set_status(sd, SM_HS, 3);
	}

	return sensor_reg_init(info);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
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

	  ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600,
				256 * 1600, 1, 1 * 1600);
	  if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

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
	info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET2 | MIPI_NORMAL_MODE;
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
