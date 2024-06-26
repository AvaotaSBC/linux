/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for sp2305 Raw cameras.
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

MODULE_AUTHOR("lwj");
MODULE_DESCRIPTION("A low-level driver for sp2305 sensors");
MODULE_LICENSE("GPL");

#define MCLK              (27*1000*1000)
#define V4L2_IDENT_SENSOR  0x2735

//define the registers
#define EXP_HIGH		0xff
#define EXP_MID			0x03
#define EXP_LOW			0x04
#define GAIN_HIGH		0xff
#define GAIN_LOW		0x24
/*
 * Our nominal (default) frame rate.
 */
#define ID_REG_HIGH		0x02
#define ID_REG_LOW		0x03
#define ID_VAL_HIGH		((V4L2_IDENT_SENSOR) >> 8)
#define ID_VAL_LOW		((V4L2_IDENT_SENSOR) & 0xff)
#define SENSOR_FRAME_RATE 20

/*
 * The sp2305 i2c address
 */
#define I2C_ADDR 0x78	/* 0x7A 0x7B 0x79 */

#define SENSOR_NUM 0x2
#define SENSOR_NAME "sp2305_mipi"
#define SENSOR_NAME_2 "sp2305_mipi_2"

static int flip_status = 0x03;

/*
 * The default register settings
 */

static struct regval_list sensor_default_regs[] = {

};

static struct regval_list sensor_1080p20_regs[] = {
	{0xfd, 0x00},
	/* {0x20,0x00}, */
	/* {0xff,0x03}, */
	{0xfd, 0x00},
	{0x2f, 0x5b}, /*  21;10;yc */
	{0x34, 0x00},
	{0x30, 0x15},
	{0x33, 0x01},
	{0x35, 0x20},
	{0xfd, 0x01},
	{0x0d, 0x10}, /*  enable manual VTS */
	{0x0e, 0x07},
	{0x0f, 0xed},
	{0x30, 0x00},
	{0x03, 0x01},
	{0x04, 0x8f},
	{0x01, 0x01},
	{0x09, 0x00},
	{0x0a, 0x20}, /*  4f? */
	{0x06, 0x0a},
	{0x24, 0x10},
	{0x01, 0x01},
	{0xfb, 0x73},
	{0x01, 0x01},
	{0xfd, 0x01},
	{0x1a, 0x6b},
	{0x1c, 0xea},
	{0x16, 0x0c},
	{0x21, 0x00},
	{0x11, 0x63}, /*  e8? */
	{0x19, 0xc3},
	{0x26, 0xda},
	{0x29, 0x01},
	{0x33, 0x6f},
	{0x2a, 0xd2},
	{0x2c, 0x40},
	{0xd0, 0x02},
	{0xd1, 0x01},
	{0xd2, 0x20},
	{0xd3, 0x04},
	{0xd4, 0x2a}, /*  a4 */
	{0x50, 0x00},
	{0x51, 0x2c},
	{0x52, 0x29},
	{0x53, 0x00},
	{0x55, 0x44},
	{0x58, 0x29},
	{0x5a, 0x00},
	{0x5b, 0x00},
	{0x5d, 0x00},
	{0x64, 0x2f},
	{0x66, 0x62},
	{0x68, 0x5b},
	{0x75, 0x46},
	{0x76, 0xf0},
	{0x77, 0x4f},
	{0x78, 0xef},
	{0x72, 0xcf},
	{0x73, 0x36},
	{0x7d, 0x0d},
	{0x7e, 0x0d},
	{0x8a, 0x77},
	{0x8b, 0x77},
	{0xfd, 0x01},
	{0xb1, 0x83}, /* ;DPHY enable 8b */
	{0xb3, 0x0b}, /* ;0b;09;1d */
	{0xb4, 0x14}, /* ;MIPI PLL enable;14;35;36 */
	{0x9d, 0x40}, /* ;mipi hs dc level 40/03/55 */
	{0xa1, 0x05}, /* ;speed/03 */
	{0x94, 0x44}, /* ;dphy time */
	{0x95, 0x33}, /* ;dphy time */
	{0x96, 0x1f}, /* ;dphy time */
	{0x98, 0x45}, /* ;dphy time */
	{0x9c, 0x10}, /* ;dphy time */
	{0xb5, 0x70}, /* ;30 */
	/* {0xa0,0x01},/* ;mipi enable */
	{0x25, 0xe0},
	{0x20, 0x7b},
	{0x8f, 0x88},
	{0x91, 0x40},
	{0xfd, 0x01},
	{0xfd, 0x02},
	{0x5e, 0x03},
	{0xa1, 0x04},
	{0xa3, 0x40},
	{0xa5, 0x02},
	{0xa7, 0xc4},
	{0xfd, 0x01},
	{0x86, 0x77}, /*  78 */
	{0x89, 0x77}, /*  78 */
	{0x87, 0x6b}, /*  74 */
	{0x88, 0x6b}, /*  74 */
	{0xfc, 0xe0},
	{0xfe, 0xe0},
	{0xf0, 0x10}, /*  40 */
	{0xf1, 0x10}, /*  40 */
	{0xf2, 0x10}, /*  40 */
	{0xf3, 0x10}, /*  40 */
	{0xfd, 0x02}, /*  crop to 1920x1080 */
	{0xa0, 0x00}, /* 	;Image vertical start MSB3bits */
	{0xa1, 0x08}, /* 	;Image vertical start LSB8bits */
	{0xa2, 0x04}, /* 	;image vertical size  MSB8bits */
	{0xa3, 0x38}, /* 	;image vertical size  LSB8bits */
	{0xa4, 0x00}, //
	{0xa5, 0x04}, /* 	;H start 8Lsb, keep center */
	{0xa6, 0x03}, //
	{0xa7, 0xc0}, /* 	;Half H size Lsb8bits */
	{0xfd, 0x01}, //
	{0x8e, 0x07}, //
	{0x8f, 0x80}, /* 	;MIPI column number */
	{0x90, 0x04}, /* 	;MIPI row number */
	{0x91, 0x38}, //

	{0xfd, 0x01}, /*  mirror/flip */
	{0x3f, 0x03}, /*  2 I2c_arrary mirror/flip */
	{0xf8, 0x02},
	{0x01, 0x01},
	{0xfd, 0x02},
	{0x62, 0x48},
	{0x63, 0x04},
	{0xfd, 0x01},

	{0xfd, 0x03},
	{0xc0, 0x01}, /* 	;enable transfer OTP BP information */
	{0xfd, 0x04},
	{0x22, 0x14},
	{0x23, 0x14}, /* 	;enhance normal and dummy BPC */

	{0xfd, 0x01}, //
	{0x06, 0xe0}, /* 	;insert dummy line , the frame rate is 30.01. */
	{0x01, 0x01}, /* 	; */
	{0xa0, 0x01}, /* ;mipi enable */

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

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	struct sensor_info *info = to_state(sd);
	/* data_type reg_0x03, reg_0x04; */
	int tmp_exp_val = (exp_val+8)>>4;
	/* int eRet; */

	sensor_write(sd, 0x03, (tmp_exp_val >> 8) & 0xFF);
	sensor_write(sd, 0x04, (tmp_exp_val & 0xFF));

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

	if (gain_val == info->gain) {
		return 0;
	}

	sensor_write(sd, 0x38, ((gain_val) >> 8) & 0xFF);
	sensor_write(sd, 0x24, ((gain_val) & 0xFF));

	info->gain = gain_val;

	return 0;
}

static int sp2305_sensor_vts;

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	int exp_val, gain_val;
	struct sensor_info *info = to_state(sd);

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	if (gain_val < 1 * 16)
		gain_val = 16;

	if (exp_val > 0xfffff)
		exp_val = 0xfffff;

	sensor_write(sd, 0xfd, 0x01); /*  page 1 */
	sensor_write(sd, 0x01, 0x00); /*  frame sync disable */
	sensor_s_exp(sd, exp_val);
	sensor_s_gain(sd, gain_val);
	sensor_write(sd, 0x01, 0x01); /* frame sync enable */
	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_get_fmt_mbus_core(struct v4l2_subdev *sd, int *code)
{
	struct sensor_info *info = to_state(sd);

	/* sensor_read(sd, 0x3f, &get_value); */
	switch (flip_status) {
	case 0x00:
		*code =
		    MEDIA_BUS_FMT_SBGGR10_1X10;
		break;
	case 0x01:
		*code =
		    MEDIA_BUS_FMT_SGBRG10_1X10;
		break;
	case 0x02:
		*code =
		    MEDIA_BUS_FMT_SGRBG10_1X10;
		break;
	case 0x03:
		*code =
		    MEDIA_BUS_FMT_SRGGB10_1X10;
		break;
	default:
		*code = info->fmt->mbus_code;
	}
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int enable)
{
	data_type set_value;

	sensor_write(sd, 0xfd, 0x01); /* page 1 */
	sensor_dbg("into set sensor hfilp the value:%d \n", enable);
	if (!(enable == 0 || enable == 1))
		return -1;

	/* sensor_read(sd, 0x3f, &get_value); */
	if (enable)
		set_value =
		    flip_status & 0xfe;
	else
		set_value =
		    flip_status | 0x01;

	sensor_write(sd, 0x3f, set_value);
	flip_status = set_value;

	sensor_write(sd, 0xf8, 0x02);
	sensor_write(sd, 0x01, 0x01);
	sensor_write(sd, 0xfd, 0x02);
	sensor_write(sd, 0x62, 0x48);
	sensor_write(sd, 0x63, 0x04);
	sensor_write(sd, 0xfd, 0x01);

	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int enable)
{
	data_type set_value;

	sensor_write(sd, 0xfd, 0x01);
	sensor_dbg("into set sensor vfilp the value:%d \n", enable);
	if (!(enable == 0 || enable == 1))
		return -1;

	/* sensor_read(sd, 0x3f, &get_value); */
	if (enable)
		set_value =
		    flip_status & 0xfd;
	else
		set_value =
		    flip_status | 0x02;

	sensor_write(sd, 0x3f, set_value);
	flip_status = set_value;

	sensor_write(sd, 0xf8, 0x00);
	sensor_write(sd, 0xf8, 0x02);
	sensor_write(sd, 0x01, 0x01);
	sensor_write(sd, 0xfd, 0x02);
	sensor_write(sd, 0x62, 0x01);
	sensor_write(sd, 0x63, 0x00);
	sensor_write(sd, 0xfd, 0x01);

	return 0;
}

static int sensor_s_fps(struct v4l2_subdev *sd, struct sensor_fps *fps)
{
	/* data_type rdval1, rdval2, rdval3; */
	struct sensor_info *info = to_state(sd);
	struct sensor_win_size *wsize = info->current_wins;

	sp2305_sensor_vts = wsize->pclk / fps->fps / wsize->hts;

	/* sensor_write(sd, 0x302d, 1);
	sensor_read(sd, 0x30f8, &rdval1);
	sensor_read(sd, 0x30f9, &rdval2);
	sensor_read(sd, 0x30fa, &rdval3);

	sensor_dbg("sp2305_sensor_svr: %d, vts: %d.\n", sp2305_sensor_svr,
	(rdval1 | (rdval2<<8) | (rdval3<<16))); */
	return 0;
}

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret;
	data_type rdval;

	sensor_write(sd, 0xfd, 0x00); /* page 0 */
	ret = sensor_read(sd, 0x36, &rdval);
	if (ret != 0)
		return ret;

	if (on_off == STBY_ON)
		ret = sensor_write(sd, 0x36, rdval | 0x01);
	else
		ret = sensor_write(sd, 0x36, rdval & 0xfe);
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
		vin_set_pmu_channel(sd, DVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_set_mclk_freq(sd, MCLK);
		usleep_range(30000, 32000);
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
		usleep_range(1000, 1200);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	unsigned int sensor_id = 0;
	data_type rdval;
	int cnt = 0;

	return 0;

	sensor_read(sd, 0x02, &rdval);
	sensor_id |= (rdval << 8);
	sensor_read(sd, 0x03, &rdval);
	sensor_id |= (rdval);
	sensor_print("V4L2_IDENT_SENSOR = 0x%x\n", sensor_id);

	while ((sensor_id != V4L2_IDENT_SENSOR) && (cnt < 5)) {
		sensor_read(sd, 0x02, &rdval);
		sensor_id |= (rdval << 8);
		sensor_read(sd, 0x03, &rdval);
		sensor_id |= (rdval);
		sensor_print("retry = %d, V4L2_IDENT_SENSOR = %x\n", cnt,
			     sensor_id);
		cnt++;
	}
	if (sensor_id != V4L2_IDENT_SENSOR)
		return -ENODEV;

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
	info->width = 1920;
	info->height = 1080;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->exp = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30; /* 30fps */

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
	case VIDIOC_VIN_SENSOR_SET_FPS:
		ret = sensor_s_fps(sd, (struct sensor_fps *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	case VIDIOC_VIN_GET_SENSOR_CODE:
		sensor_get_fmt_mbus_core(sd, (int *)arg);
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
	{.desc = "Raw RGB Bayer",
	.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.regs = sensor_fmt_raw,
	.regs_size = ARRAY_SIZE(sensor_fmt_raw),
	.bpp = 1},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {

	{
		.width = 1920,
		.height = 1080,
		.hoffset = 0,
		.voffset = 0,
		.hts = 2420,
		.vts = 1664,
		.pclk = 80000000,
		.mipi_bps = 378000000,
		.fps_fixed = 20,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = 1660 << 4,
		.gain_min = 1 << 4,
		.gain_max = 31 << 4,
		.regs = sensor_1080p20_regs,
		.regs_size = ARRAY_SIZE(sensor_1080p20_regs),
		.set_size = NULL,
		.top_clk = 300 * 1000 * 1000,
		.isp_clk = 300 * 1000 * 1000,

	},

};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	struct sensor_info *info = to_state(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	if (info->isp_wdr_mode == ISP_DOL_WDR_MODE)
		cfg->flags = 0 | V4L2_MBUS_CSI2_2_LANE |
			     V4L2_MBUS_CSI2_CHANNEL_0 |
			     V4L2_MBUS_CSI2_CHANNEL_1;
	else
		cfg->flags =
		    0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
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
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->val);
	}
	return -EINVAL;
}

static int sensor_reg_init(struct sensor_info *info)
{
	int ret;
	/* data_type rdval_l, rdval_h; */
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;

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
	sp2305_sensor_vts = wsize->vts;
	flip_status = 0x03;

	sensor_dbg("s_fmt set width = %d, height = %d\n", wsize->width,
	       wsize->height);

	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_dbg("%s on = %d, %d*%d fps: %d code: %x\n", __func__, enable,
		   info->current_wins->width, info->current_wins->height,
		   info->current_wins->fps_fixed, info->fmt->mbus_code);

	if (!enable)
		return 0;

	return sensor_reg_init(info);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.try_ctrl = sensor_try_ctrl,
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
		.name = SENSOR_NAME, .addr_width = CCI_BITS_8, .data_width = CCI_BITS_8,
	}, {
		.name = SENSOR_NAME_2,
		.addr_width = CCI_BITS_8,
		.data_width = CCI_BITS_8,
	}
};

static int sensor_init_controls(struct v4l2_subdev *sd,
				const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 4);

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600,
				 256 * 1600, 1, 1 * 1600);

	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 1, 65536 * 16,
				 1, 1);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

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
		cci_dev_probe_helper(sd, client, &sensor_ops,
				     &cci_drv[sensor_dev_id++]);
	}

	sensor_init_controls(sd, &sensor_ctrl_ops);

	mutex_init(&info->lock);

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->combo_mode =
	    CMB_TERMINAL_RES | CMB_PHYA_OFFSET2 | MIPI_NORMAL_MODE;
	info->time_hs = 0x23;
	info->stream_seq = MIPI_BEFORE_SENSOR;
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
