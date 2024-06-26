/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for nvp6134 cameras.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "../../../utility/vin_log.h"
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include "../camera.h"
#include "../sensor_helper.h"
#include "csi_dev_nvp6134.h"

struct v4l2_subdev *gl_sd;

MODULE_AUTHOR("zw");
MODULE_DESCRIPTION("A low-level driver for bt1120 sensors");
MODULE_LICENSE("GPL");

#define MCLK (24 * 1000 * 1000)
#define CLK_POL V4L2_MBUS_PCLK_SAMPLE_FALLING
#define V4L2_IDENT_SENSOR 0x00c8

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 30

/*
 * The TW2866 sits on i2c with ID 0x50
 */
#define I2C_ADDR 0x60
#define SENSOR_NAME "nvp6134"

/*
static struct regval_list read_reg[] = {
};

*/

/*
static struct regval_list reg_d1_1ch[] = {

};
*/

static struct regval_list sensor_720p_30fps_regs[] = {

};

/*
static struct regval_list reg_d1_2ch[] = {
};
*/

/*
static struct regval_list reg_d1_4ch[] = {
};
*/

/*
static struct regval_list reg_cif_4ch[] = {
};
*/

/* #define NVP6134_DUMP_EN */
#ifdef NVP6134_DUMP_EN
static int powerOn;
extern void dump_bank(int bank);
static void register_dump_func(struct work_struct *ws);
struct timer_list reg_dump;
static struct workqueue_struct *nvp6134_wq;
static void register_dump_func(struct work_struct *);
static DECLARE_DELAYED_WORK(nvp6134_dwq, register_dump_func);
static void register_dump_func(struct work_struct *ws)
{
	if (powerOn == 1) {
		dump_bank(0);
		dump_bank(1);
		dump_bank(5);
	}
	queue_delayed_work(nvp6134_wq, &nvp6134_dwq, msecs_to_jiffies(5000));
}

#endif

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	if (on_off)
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
	else
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
	return 0;
}

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
	case STBY_ON:
		sensor_print("CSI_SUBDEV_STBY_ON!\n");
		sensor_s_sw_stby(sd, ON);
		break;
	case STBY_OFF:
		sensor_print("CSI_SUBDEV_STBY_OFF!\n");
		sensor_s_sw_stby(sd, OFF);
		break;
	case PWR_ON:
#ifdef NVP6134_DUMP_EN
		powerOn = 1;
#endif
		sensor_print("CSI_SUBDEV_PWR_ON!\n");
		cci_lock(sd);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		vin_set_pmu_channel(sd, IOVDD, ON);
		/* vin_set_pmu_channel(sd, DVDD, ON); */
		/* vin_gpio_set_status(sd, VDD3V3_EN, CSI_GPIO_HIGH); */
		/* vin_gpio_set_status(sd, V5_EN, CSI_GPIO_HIGH); */
		/* vin_gpio_set_status(sd, CCDVDD_EN, CSI_GPIO_HIGH); */
		vin_gpio_set_status(sd, RESET, CSI_GPIO_HIGH);
		vin_gpio_set_status(sd, POWER_EN, CSI_GPIO_HIGH);
		/* vin_gpio_write(sd, VDD3V3_EN, CSI_GPIO_HIGH); */
		/* vin_gpio_write(sd, V5_EN, CSI_GPIO_HIGH); */
		/* vin_gpio_write(sd, CCDVDD_EN, CSI_GPIO_HIGH); */
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		usleep_range(1000, 1200);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(10, 12);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(5000, 10000);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		cci_unlock(sd);
		break;
	case PWR_OFF:
#ifdef NVP6134_DUMP_EN
		powerOn = 0;
#endif
		sensor_print("CSI_SUBDEV_PWR_OFF!\n");
		cci_lock(sd);
		vin_set_mclk(sd, OFF);
		usleep_range(1000, 1200);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		/* vin_gpio_write(sd, VDD3V3_EN, CSI_GPIO_LOW); */
		/* vin_gpio_write(sd, V5_EN, CSI_GPIO_LOW); */
		/* vin_gpio_write(sd, CCDVDD_EN, CSI_GPIO_LOW); */
		vin_set_pmu_channel(sd, IOVDD, OFF);
		/* vin_set_pmu_channel(sd, DVDD, OFF); */

		cci_unlock(sd);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
	usleep_range(5000, 6000);
	vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
	usleep_range(5000, 6000);
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type rdval;

	gpio_i2c_write(0x62, 0xFF, 0x00);
	rdval = gpio_i2c_read(0x62, 0xf4);
	sensor_print("sensor id = 0x%x\n", rdval);
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	/* Make sure it is a target sensor */
	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = 1280; /* VGA_WIDTH; */
	info->height = 720; /* VGA_HEIGHT; */
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30; /* 30fps */

	info->preview_first_flag = 1;
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
	.desc = "BT656 4CH",
#ifdef AHD_1080P_1CH
	.mbus_code = MEDIA_BUS_FMT_UYVY8_2X8,
#else
	.mbus_code = MEDIA_BUS_FMT_VYUY8_1X16, /* MEDIA_BUS_FMT_YVYU8_1X16, */
#endif
	.regs = NULL,
	.regs_size = 0,
	.bpp = 2,
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size sensor_win_sizes[] = {
	{
#ifdef AHD_1080P_1CH
	.width = 1920,
	.height = 1080,
#else
	.width = 1280,
	.height = 720,
#endif
	.hoffset = 0,
	.voffset = 0,
	.regs = sensor_720p_30fps_regs,
	.regs_size = ARRAY_SIZE(sensor_720p_30fps_regs),
	.set_size = NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_BT656;
#ifdef AHD_1080P_1CH
	cfg->flags = CLK_POL | CSI_CH_0;
#else
	cfg->flags = CLK_POL | CSI_CH_0 | CSI_CH_1 | CSI_CH_2 | CSI_CH_3;
#endif
	return 0;
}

static int sensor_reg_init(struct sensor_info *info)
{
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;

	sensor_init_hardware(0);

	sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

	if (wsize->regs)
		sensor_write_array(sd, wsize->regs, wsize->regs_size);

	if (wsize->set_size)
		wsize->set_size(sd);

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_print("%s on = %d, %d*%d %x\n", __func__, enable,
		     info->current_wins->width, info->current_wins->height,
		     info->fmt->mbus_code);

	if (!enable)
		return 0;
	return sensor_reg_init(info);
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.enum_frame_interval = sensor_enum_frame_interval,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_8,
	.data_width = CCI_BITS_8,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct sensor_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	gl_sd = &info->sd;
	cci_dev_probe_helper(gl_sd, client, &sensor_ops, &cci_drv);
	mutex_init(&info->lock);
#if IS_ENABLED(CONFIG_SAME_I2C)
	info->sensor_i2c_addr = I2C_ADDR >> 1;
#endif
	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;

#ifdef NVP6134_DUMP_EN
	nvp6134_wq = create_singlethread_workqueue("nvp6134_work_queue");
	queue_delayed_work(nvp6134_wq, &nvp6134_dwq, 10 * HZ);
#endif

	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
#ifdef NVP6134_DUMP_EN
	cancel_work_sync(&nvp6134_dwq);
#endif
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
