/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for lt6911c cameras and TVI Coax protocol.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Chenjinkun <chenjinkun@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "../../utility/vin_log.h"
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <linux/io.h>
#include "camera.h"
#include "sensor_helper.h"

MODULE_AUTHOR("CJk");
MODULE_DESCRIPTION("A low-level driver for lt6911c mipi chip for TVI sensor");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

/*define module timing*/
#define MCLK              (27*1000*1000)
#define V4L2_IDENT_SENSOR  0x1605

/*
 * The lt6911c i2c address
 */
#define I2C_ADDR  0x56

/*static struct delayed_work sensor_s_ae_ratio_work;*/
static bool restart;

/* Keep device-local detection state without changing the shared VIN ABI. */
struct lt6911_state {
 struct sensor_info info;
 struct mutex io_lock;
 bool reset_fixed;
 int power_switch;
 u32 refclk_hz;
 int input_error;
 bool stopping;
 bool irq_requested;
 bool streaming; /* protected by io_lock */
 atomic_t irq_pending;
 unsigned long probe_after;
 int pending_error;
 u32 pending_width, pending_height, pending_fps;
 unsigned int pending_count;
 unsigned int settle_attempts;
 unsigned long burst_deadline;
 bool probing; /* protected by detect_lock */
 unsigned int probe_count, probe_failures;
 atomic_t irq_count;
 struct v4l2_dv_timings last_sample;
 int last_read_error;
};
static struct lt6911_state *lt_state(struct sensor_info *info)
{
 return container_of(info, struct lt6911_state, info);
}
/* Transitional boot option for a board whose installed DT still names RESET. */
static bool reset_tied_high;
module_param(reset_tied_high, bool, 0444);
MODULE_PARM_DESC(reset_tied_high, "RESET is externally held high; never drive reset GPIO");
static bool lt_reset_tied_high(struct v4l2_subdev *sd)
{
 return reset_tied_high || lt_state(to_state(sd))->reset_fixed;
}

/* Avoid reprogramming timing meters while valid frames are streaming. */
static unsigned int poll_interval_ms = 10000;
module_param(poll_interval_ms, uint, 0644);
MODULE_PARM_DESC(poll_interval_ms, "Unlocked recovery / streaming watchdog interval, 500..60000 ms");

static unsigned int refclk_hz = 27000000;
module_param(refclk_hz, uint, 0444);
MODULE_PARM_DESC(refclk_hz, "Module reference crystal frequency in Hz (24 or 27 MHz)");

#define SENSOR_NAME "lt6911c_mipi"

static const struct v4l2_dv_timings_cap lt6911c_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(
			640, 3840,       /* min/max width */
			480, 2160,       /* min/max height */
			0, 400000000,    /* min/max pixelclock*/
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,  /* Supported standards */
			V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_INTERLACED |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)    /* capabilities */
};

/*
 * The default register settings
 *
 */
static struct regval_list sensor_default_regs[] = {
};

static int sensor_ii2_en(struct v4l2_subdev *sd, unsigned int en)
{
	int ret = sensor_write(sd, 0xff, 0x80);
	if (ret < 0)
		return ret;
	return sensor_write(sd, 0xee, en ? 0x01 : 0x00);
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	return 0;
}


static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret = 0;
	return ret;
}

/*
 * Stuff that knows about the sensor.
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	ret = 0;
	switch (on) {
	case STBY_ON:
		sensor_dbg("STBY_ON!\n");
		cci_lock(sd);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vin_set_mclk(sd, OFF);
		cci_unlock(sd);
		break;
	case STBY_OFF:
		sensor_dbg("STBY_OFF!\n");
		cci_lock(sd);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		ret = sensor_s_sw_stby(sd, CSI_GPIO_LOW);
		if (ret < 0)
			sensor_err("soft stby off falied!\n");
		usleep_range(20000, 22000);

		break;
	case PWR_ON:
		if (lt_reset_tied_high(sd)) break;
		sensor_dbg("PWR_ON!\n");
		cci_lock(sd);

		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);

		cci_unlock(sd);

		break;
	case PWR_OFF:
		if (lt_reset_tied_high(sd)) break;
		sensor_dbg("PWR_OFF1123\n");
		cci_lock(sd);

		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);

		usleep_range(1000, 1200);
		cci_unlock(sd);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	if (lt_reset_tied_high(sd)) return 0;
	switch (val) {
	case 0:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);//HIGH
		usleep_range(30000, 32000);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(60000, 62000);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
 struct sensor_info *info = to_state(sd);
 data_type hi = 0, lo = 0;
 int ret = -ENODEV, restore_ret, i;
 mutex_lock(&lt_state(info)->io_lock);
 ret = sensor_ii2_en(sd, 1);
 if (ret < 0) goto done;
 for (i = 0; i < 5; ++i) {
  ret = sensor_write(sd, 0xff, 0xa0);
  if (!ret) ret = sensor_read(sd, 0x00, &hi);
  if (!ret) ret = sensor_read(sd, 0x01, &lo);
  if (!ret && ((hi << 8) | lo) == V4L2_IDENT_SENSOR) break;
  if (!ret) ret = -ENODEV;
  usleep_range(10000, 12000);
 }
done:
 restore_ret = sensor_ii2_en(sd, 0);
 if (restore_ret < 0) {
  sensor_err("failed to disable external I2C access: %d\n", restore_ret);
  if (!ret) ret = restore_ret;
 }
 mutex_unlock(&lt_state(info)->io_lock);
 return ret;
}

static unsigned int fps_calc(const struct v4l2_bt_timings *t)
{
	if (!V4L2_DV_BT_FRAME_HEIGHT(t) || !V4L2_DV_BT_FRAME_WIDTH(t))
		return 0;

	return DIV_ROUND_CLOSEST((unsigned int)t->pixelclock,
			V4L2_DV_BT_FRAME_HEIGHT(t) * V4L2_DV_BT_FRAME_WIDTH(t));
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	sensor_dbg("sensor_init\n");
	restart = 0;


	/*Make sure it is a target sensor */
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
	info->tpf.numerator = 1;
	info->tpf.denominator = 25;

	return 0;
}

static int lt6911c_get_detected_timings(struct v4l2_subdev *sd, struct v4l2_dv_timings *timings)
{
	//struct sensor_info *info = to_state(sd);
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 hact, vact, htotal, vtotal;
	u32 hbp, hs, hfp, vbp, vs, vfp;
	u32 pixel_clock, halt_pix_clk, fps;
	int ret, restore_ret;
	data_type clk_h = 0, clk_m = 0, clk_l = 0;
	data_type value = 0, val_h = 0, val_l = 0;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	ret = sensor_write(sd, 0xff, 0x80);
	if (ret < 0) goto read_error;
	ret = sensor_write(sd, 0xee, 0x01);
	if (ret < 0) goto read_error;

	ret = sensor_write(sd, 0xff, 0xa0);
	if (ret < 0) goto read_error;
	ret = sensor_write(sd, 0x34, 0x21);
	if (ret < 0) goto read_error;
	usleep_range(10000, 10100);
	ret = sensor_write(sd, 0xff, 0xb8);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0xb1, &clk_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0xb2, &clk_m);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0xb3, &clk_l);
	if (ret < 0) goto read_error;
	halt_pix_clk = (((clk_h & 0xf) << 16) | (clk_m << 8) | clk_l);
	/* The frequency meter is scaled to a 27 MHz reference by the legacy driver. */
	pixel_clock = div_u64((u64)halt_pix_clk * 2000 * lt_state(to_state(sd))->refclk_hz,
			      27000000);

	ret = sensor_write(sd, 0xff, 0xd2);
	if (ret < 0) goto read_error;
	ret = sensor_write(sd, 0x83, 0x11);
	if (ret < 0) goto read_error;
	usleep_range(10000, 10100);
	ret = sensor_read(sd, 0x89, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x8a, &val_l);
	if (ret < 0) goto read_error;
	htotal = ((val_h << 8) | val_l) * 2;
	ret = sensor_read(sd, 0x9e, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x9f, &val_l);
	if (ret < 0) goto read_error;
	vtotal = (val_h << 8) | val_l;
	ret = sensor_read(sd, 0x8b, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x8c, &val_l);
	if (ret < 0) goto read_error;
	hact = ((val_h << 8) | val_l) * 2;
	ret = sensor_read(sd, 0x96, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x97, &val_l);
	if (ret < 0) goto read_error;
	vact = (val_h << 8) | val_l;
	ret = sensor_read(sd, 0x94, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x95, &val_l);
	if (ret < 0) goto read_error;
	hs = ((val_h << 8) | val_l) * 2;
	ret = sensor_read(sd, 0x86, &value);
	if (ret < 0) goto read_error;
	vs = value;
	ret = sensor_read(sd, 0x9c, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x9d, &val_l);
	if (ret < 0) goto read_error;
	hfp = ((val_h << 8) | val_l) * 2;
	ret = sensor_read(sd, 0x88, &value);
	if (ret < 0) goto read_error;
	vfp = value;
	ret = sensor_read(sd, 0x98, &val_h);
	if (ret < 0) goto read_error;
	ret = sensor_read(sd, 0x99, &val_l);
	if (ret < 0) goto read_error;
	hbp = ((val_h << 8) | val_l) * 2;
	ret = sensor_read(sd, 0x87, &value);
	if (ret < 0) goto read_error;
	vbp = value;

	ret = sensor_ii2_en(sd, 0);
	if (ret < 0) goto read_error;

	timings->type = V4L2_DV_BT_656_1120;
	bt->interlaced = V4L2_DV_PROGRESSIVE;
	bt->width = hact;
	bt->height = vact;
	bt->vsync = vs;
	bt->hsync = hs;
	bt->pixelclock = pixel_clock;
	bt->hfrontporch = hfp;
	bt->vfrontporch = vfp;
	bt->hbackporch = hbp;
	bt->vbackporch = vbp;
	fps = fps_calc(bt);

	sensor_dbg("act:%dx%d, total:%dx%d, pixclk:%d, fps:%d\n",
			hact, vact, htotal, vtotal, pixel_clock, fps);
	sensor_dbg("hfp:%d, hs:%d, hbp:%d, vfp:%d, vs:%d, vbp:%d\n",
			hfp, hs, hbp, vfp, vs, vbp);

	return 0;
read_error:
	/* Retry restoring the gate even if the original transfer failed. */
	restore_ret = sensor_ii2_en(sd, 0);
	if (restore_ret < 0)
		sensor_err("failed to disable external I2C access: %d\n", restore_ret);
	memset(timings, 0, sizeof(*timings));
	return ret;
}


static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);
	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins != NULL) {
			memcpy(arg, info->current_wins,
			       sizeof(struct sensor_win_size));
			ret = 0;
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct sensor_format_struct sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,  //  MEDIA_BUS_FMT_UYVY8_2X8
		.regs 		= sensor_default_regs,
		.regs_size = ARRAY_SIZE(sensor_default_regs),
		.bpp		= 2,
	}
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
static struct sensor_win_size sensor_win_sizes[] = {
	{
	  .width = 720,
	  .height = 480,
	  .hoffset = 0,
	  .voffset = 0,
	  .fps_fixed = 60,
	  .regs = sensor_default_regs,
	  .regs_size = ARRAY_SIZE(sensor_default_regs),
	  .set_size = NULL,
	},
	{
	  .width = 720,
	  .height = 576,
	  .hoffset = 0,
	  .voffset = 0,
	  .fps_fixed = 50,
	  .regs = sensor_default_regs,
	  .regs_size = ARRAY_SIZE(sensor_default_regs),
	  .set_size = NULL,
	},
	{
	  .width = 1280,
	  .height = 720,
	  .hoffset = 0,
	  .voffset = 0,
	  .fps_fixed = 50,
	  .regs = sensor_default_regs,
	  .regs_size = ARRAY_SIZE(sensor_default_regs),
	  .set_size = NULL,
	},
	{
	  .width = 1280,
	  .height = 720,
	  .hoffset = 0,
	  .voffset = 0,
	  .fps_fixed = 60,
	  .regs = sensor_default_regs,
	  .regs_size = ARRAY_SIZE(sensor_default_regs),
	  .set_size = NULL,
	},
	{
	 .width = 1920,
	 .height = 1080,
	 .hoffset = 0,
	 .voffset = 0,
	 .fps_fixed = 50,
	 .regs = sensor_default_regs,
	 .regs_size = ARRAY_SIZE(sensor_default_regs),
	 .set_size = NULL,
	},
	{
	 .width = 1920,
	 .height = 1080,
	 .hoffset = 0,
	 .voffset = 0,
	 .fps_fixed = 60,
	 .regs = sensor_default_regs,
	 .regs_size = ARRAY_SIZE(sensor_default_regs),
	 .set_size = NULL,
	},
	{
	 .width = 3840,
	 .height = 2160,
	 .hoffset = 0,
	 .voffset = 0,
	 .fps_fixed = 30,
	 .regs = sensor_default_regs,
	 .regs_size = ARRAY_SIZE(sensor_default_regs),
	 .set_size = NULL,
	},
};
#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	/* The BSP VIN interprets this field as lane and virtual-channel flags. */
	cfg->bus.mipi_csi2.num_data_lanes = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0;
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

	return 0;
}

static int sensor_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct sensor_info *info = to_state(sd);

	if (!timings)
		return -EINVAL;

	//v4l2_print_dv_timings(sd->name, "s_dv_timings: ", timings, false);

	if (v4l2_match_dv_timings(&info->timings, timings, 0, false)) {
		sensor_err("%s: timings no change\n", __func__);
		return 0;
	}

	if (!v4l2_valid_dv_timings(timings,
				&lt6911c_timings_cap, NULL, NULL)) {
		sensor_err("%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	sensor_print("%s timings is get form input dp size, cannot set by users\n", sd->name);

	return 0;
}

static int sensor_query_dv_timings(struct v4l2_subdev *sd,
 struct v4l2_dv_timings *timings)
{
 struct sensor_info *info = to_state(sd);
 int ret;
 if (!timings) return -EINVAL;
 mutex_lock(&info->sensor_indet.detect_lock);
 *timings = info->timings;
 ret = lt_state(info)->input_error;
 if (!ret && (lt_state(info)->probing || lt_state(info)->pending_count ||
              atomic_read(&lt_state(info)->irq_pending)))
  ret = -ENOLCK;
 mutex_unlock(&info->sensor_indet.detect_lock);
 return ret;
}

static int sensor_g_dv_timings(struct v4l2_subdev *sd,
 struct v4l2_dv_timings *timings)
{
 return sensor_query_dv_timings(sd, timings);
}

static int sensor_enum_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings,
			&lt6911c_timings_cap, NULL, NULL);
}

static int sensor_dv_timings_cap(struct v4l2_subdev *sd,
				struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;

	*cap = lt6911c_timings_cap;

	return 0;
}

static int sensor_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		sensor_dbg("sensor subscribe event -- V4L2_EVENT_SOURCE_CHANGE\n");
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		sensor_dbg("sensor subscribe event -- V4L2_EVENT_CTRL\n");
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static int sensor_reg_init(struct sensor_info *info)
{
	int ret;
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;

	ret = sensor_write_array(sd, sensor_default_regs,
					ARRAY_SIZE(sensor_default_regs));

	sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);
	if (ret < 0) {
		sensor_err("write sensor_default_regs error\n");
		return ret;
	}
	if (wsize->regs)
		sensor_write_array(sd, wsize->regs, wsize->regs_size);

	if (wsize->set_size)
		wsize->set_size(sd);

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	sensor_dbg("s_fmt set width = %d, height = %d\n", wsize->width,
		      wsize->height);

	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
 struct sensor_info *info = to_state(sd);
 struct lt6911_state *state = lt_state(info);
 struct sensor_indetect *det = &info->sensor_indet;
 int ret = 0;

 mutex_lock(&state->io_lock);
 if (enable) {
  mutex_lock(&det->detect_lock);
  ret = state->input_error;
  if (!ret && (state->probing || state->pending_count ||
               atomic_read(&state->irq_pending)))
   ret = -ENOLCK;
  mutex_unlock(&det->detect_lock);
  if (!ret) ret = sensor_reg_init(info);
  if (!ret) state->streaming = true;
 } else {
  state->streaming = false;
  /* Allow the rest of the VIN pipeline to finish STREAMOFF before probing. */
  state->probe_after = jiffies + msecs_to_jiffies(120);
  mutex_lock(&det->detect_lock);
  state->pending_count = 0;
  state->settle_attempts = 0;
  mutex_unlock(&det->detect_lock);
 }
 mutex_unlock(&state->io_lock);
 if (!enable && !READ_ONCE(state->stopping))
  mod_delayed_work(system_wq, &det->sensor_work, msecs_to_jiffies(120));
 if (ret != -ENOLINK && ret != -ENOLCK)
  sensor_print("%s on=%d result=%d\n", __func__, enable, ret);
 return ret;
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.try_ctrl = sensor_try_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.subscribe_event = sensor_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = sensor_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_dv_timings = sensor_s_dv_timings,
	.g_dv_timings = sensor_g_dv_timings,
	.query_dv_timings = sensor_query_dv_timings,
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_dv_timings = sensor_enum_dv_timings,
	.dv_timings_cap = sensor_dv_timings_cap,
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

static bool sensor_detect_res(u32 width, u32 height, u32 fps)
{
	u32 i;

	for (i = 0; i < N_WIN_SIZES; i++) {
		if ((sensor_win_sizes[i].width == width) &&
			(sensor_win_sizes[i].height == height) &&
			(sensor_win_sizes[i].fps_fixed == fps)) {
			break;
		}
	}

	if (i == N_WIN_SIZES) {
		sensor_err("%s:do not support res wxh@fps: %dx%d@%dfps\n", __func__,
				width, height, fps);
		return false;
	} else {
		return true;
	}
}

static void sensor_det_work(struct work_struct *work)
{
 struct delayed_work *dw = to_delayed_work(work);
 struct sensor_indetect *det = container_of(dw, struct sensor_indetect, sensor_work);
 struct sensor_info *info = container_of(det, struct sensor_info, sensor_indet);
 struct lt6911_state *state = lt_state(info);
 struct v4l2_dv_timings timings;
 struct v4l2_event event = { .type = V4L2_EVENT_SOURCE_CHANGE };
 int ret, ctrl_value;
 bool changed, confirming = false, input_event = false;
 unsigned int delay_ms = 0;
 unsigned long now;
 u32 width, height, fps;
 if (READ_ONCE(state->stopping)) return;
 mutex_lock(&state->io_lock);
 if (state->streaming) {
  /* Avoid touching the timing meters while valid frames are flowing.
   * After an input change there are no valid frames to preserve, so keep
   * probing even if userspace has not stopped the capture stream yet. */
  mutex_lock(&det->detect_lock);
  if (atomic_xchg(&state->irq_pending, 0)) {
   input_event = true;
   state->pending_count = 0;
   sensor_print("HDMI event: rechecking input during capture\n");
  }
  if (!state->input_error && !input_event) {
   delay_ms = clamp_val(READ_ONCE(poll_interval_ms), 500, 60000);
   mutex_unlock(&det->detect_lock);
   mutex_unlock(&state->io_lock);
   goto reschedule;
  }
  mutex_unlock(&det->detect_lock);
 }
 now = jiffies;
 if (time_before(now, state->probe_after)) {
  delay_ms = max_t(unsigned int, 1,
   jiffies_to_msecs(state->probe_after - now));
  mutex_unlock(&state->io_lock);
  goto reschedule;
 }
 /* Consume only the event which starts this probe. An IRQ during I2C access
  * remains pending and its queued work must not be cancelled on success. */
 mutex_lock(&det->detect_lock);
 state->probing = true;
 if (time_after_eq(jiffies, state->burst_deadline))
  state->settle_attempts = 0;
 if (atomic_xchg(&state->irq_pending, 0))
  state->pending_count = 0;
 mutex_unlock(&det->detect_lock);
 state->settle_attempts++;
 state->probe_count++;
 ret = lt6911c_get_detected_timings(&info->sd, &timings);
 mutex_lock(&det->detect_lock);
 state->last_sample = timings;
 state->last_read_error = ret;
 /* With no HDMI input this firmware leaves small, non-zero meter residue. */
 if (!ret && (timings.bt.width < lt6911c_timings_cap.bt.min_width ||
              timings.bt.height < lt6911c_timings_cap.bt.min_height ||
              timings.bt.pixelclock < 10000000))
  ret = -ENOLINK;
 if (!ret && !v4l2_valid_dv_timings(&timings, &lt6911c_timings_cap, NULL, NULL))
  ret = -ERANGE;
 if (ret) state->probe_failures++;
 /* Ignore small clock-meter jitter; dimensions/rate changes still notify. */
 changed = state->input_error != ret ||
  (!ret && (info->timings.bt.width != timings.bt.width ||
            info->timings.bt.height != timings.bt.height ||
            fps_calc(&info->timings.bt) != fps_calc(&timings.bt)));
 /* Require two matching observations before publishing a transition. */
 width = ret ? 0 : timings.bt.width;
 height = ret ? 0 : timings.bt.height;
 fps = ret ? 0 : fps_calc(&timings.bt);
 if (changed) {
  if (state->pending_error == ret && state->pending_width == width &&
      state->pending_height == height && state->pending_fps == fps)
   state->pending_count++;
  else {
   state->pending_error = ret;
   state->pending_width = width;
   state->pending_height = height;
   state->pending_fps = fps;
   state->pending_count = 1;
  }
  confirming = state->pending_count < 2;
  if (confirming) changed = false;
 } else {
  state->pending_count = 0;
 }
 /* An IRQ arriving while reading must be checked, not called settled. Keep
  * a burst count across IRQs and unchanged samples: the meter itself may
  * provoke another event. A quiet second starts a fresh burst. */
 if (atomic_read(&state->irq_pending)) confirming = true;
 state->burst_deadline = jiffies + msecs_to_jiffies(1000);
 if (state->settle_attempts >= 6) {
  if (!ret) state->probe_failures++;
  ret = -ENOLCK;
  confirming = false;
  changed = state->input_error != ret || info->timings.bt.width != 0;
  state->probe_after = jiffies + msecs_to_jiffies(
   clamp_val(READ_ONCE(poll_interval_ms), 500, 60000));
  state->settle_attempts = 0;
 }
 if (!confirming) {
  state->input_error = ret;
  if (ret) memset(&timings, 0, sizeof(timings));
  info->timings = timings;
  state->pending_count = 0;
 }
 state->probing = false;
 delay_ms = confirming ? 100 : ret ?
  clamp_val(READ_ONCE(poll_interval_ms), 500, 60000) : 0;
 if (!confirming && !ret)
  sensor_print("HDMI detection settled: probes=%u failures=%u irq=%d; waiting for event\n",
    state->probe_count, state->probe_failures, atomic_read(&state->irq_count));
 mutex_unlock(&det->detect_lock);
 mutex_unlock(&state->io_lock);
 if (changed) {
  ctrl_value = ret ? HOTPLUT_DP_OUT :
   (sensor_detect_res(timings.bt.width, timings.bt.height, fps_calc(&timings.bt)) ?
     HOTPLUT_DP_IN : HOTPLUT_DP_NOSUPPRT);
  if (det->ctrl_hotplug) v4l2_ctrl_s_ctrl(det->ctrl_hotplug, ctrl_value);
  event.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION;
  v4l2_subdev_notify_event(&info->sd, &event);
  sensor_print("HDMI input %s: %ux%u@%u error=%d\n", ret ? "unavailable" : "locked",
    timings.bt.width, timings.bt.height, fps_calc(&timings.bt), ret);
 }
reschedule:
 /* A locked idle receiver is event-driven. While streaming, the watchdog
  * checks for missed IRQs without touching timing registers. A receiver
  * with lost input continues probing until the signal is valid again. */
 if (delay_ms && !READ_ONCE(state->stopping))
  schedule_delayed_work(&det->sensor_work, msecs_to_jiffies(delay_ms));
}

static irqreturn_t sensor_det_irq_func(int irq, void *priv)
{
 struct sensor_info *info = to_state((struct v4l2_subdev *)priv);
 if (!READ_ONCE(lt_state(info)->stopping)) {
  atomic_inc(&lt_state(info)->irq_count);
  atomic_set(&lt_state(info)->irq_pending, 1);
  mod_delayed_work(system_wq, &info->sensor_indet.sensor_work, msecs_to_jiffies(120));
 }
 return IRQ_HANDLED;
}

static void sensor_det_exit(struct v4l2_subdev *sd)
{
 struct sensor_info *info = to_state(sd);
 struct sensor_indetect *det = &info->sensor_indet;
 struct lt6911_state *state = lt_state(info);
 WRITE_ONCE(state->stopping, true);
 /* Stop the producer before cancelling work. dev_id must match request_irq. */
 if (state->irq_requested) {
  free_irq(det->hotplug_det_irq, sd);
  state->irq_requested = false;
 }
 cancel_delayed_work_sync(&det->sensor_work);
 if (gpio_is_valid(det->hotplug_det_gpio.gpio)) gpio_free(det->hotplug_det_gpio.gpio);
 if (gpio_is_valid(det->reset_gpio.gpio)) gpio_free(det->reset_gpio.gpio);
 if (gpio_is_valid(det->power_gpio.gpio)) gpio_free(det->power_gpio.gpio);
 if (gpio_is_valid(state->power_switch)) gpio_free(state->power_switch);
 det->hotplug_det_gpio.gpio = det->reset_gpio.gpio = det->power_gpio.gpio = -1;
 state->power_switch = -1;
}

static int lt_request_gpio(struct device_node *np, const char *name,
 unsigned long flags, int *owned)
{
 int gpio, ret;
 /* Fixed supply and fixed-high reset do not require GPIO resources. */
 if (!of_find_property(np, name, NULL)) return 0;
 gpio = of_get_named_gpio(np, name, 0);
 if (gpio < 0) return gpio;
 ret = gpio_request_one(gpio, flags, name);
 if (!ret) *owned = gpio;
 return ret;
}

static int sensor_det_init(struct v4l2_subdev *sd)
{
 struct sensor_info *info = to_state(sd);
 struct sensor_indetect *det = &info->sensor_indet;
 struct lt6911_state *state = lt_state(info);
 struct device_node *np;
 int ret;
 np = of_find_node_by_name(NULL, "sensor_detect");
 if (!np) return -EINVAL;
 state->refclk_hz = refclk_hz;
 of_property_read_u32(np, "lontium,refclk-frequency", &state->refclk_hz);
 if (state->refclk_hz != 24000000 && state->refclk_hz != 27000000) {
  ret = -EINVAL; goto out;
 }
 ret = lt_request_gpio(np, "power_gpios", GPIOF_OUT_INIT_HIGH, &det->power_gpio.gpio);
 if (ret) goto out;
 ret = lt_request_gpio(np, "power1_gpios", GPIOF_OUT_INIT_HIGH, &state->power_switch);
 if (ret) goto out;
 if (!lt_reset_tied_high(sd)) {
  ret = lt_request_gpio(np, "reset_gpios", GPIOF_OUT_INIT_LOW, &det->reset_gpio.gpio);
  if (ret) goto out;
  if (gpio_is_valid(det->reset_gpio.gpio)) {
   usleep_range(1000, 1200);
   gpio_set_value(det->reset_gpio.gpio, 1);
  }
 }
 ret = lt_request_gpio(np, "hotplug_gpios", GPIOF_IN, &det->hotplug_det_gpio.gpio);
 if (ret) goto out;
 if (!gpio_is_valid(det->hotplug_det_gpio.gpio)) { ret = -EINVAL; goto out; }
 det->hotplug_det_irq = gpio_to_irq(det->hotplug_det_gpio.gpio);
 if (det->hotplug_det_irq < 0) { ret = det->hotplug_det_irq; goto out; }
 /* Keep both edges until this module firmware's event pulse polarity is
  * established; in particular, do not discard a real unplug falling edge. */
 ret = request_irq(det->hotplug_det_irq, sensor_det_irq_func,
   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "lt6911-hdmi", sd);
 if (!ret) {
  state->irq_requested = true;
  sensor_print("HDMI detection gpio=%d irq=%d reset=%s refclk=%u\n",
    det->hotplug_det_gpio.gpio, det->hotplug_det_irq,
    gpio_is_valid(det->reset_gpio.gpio) ? "gpio" : "fixed-high", state->refclk_hz);
 }
out:
 of_node_put(np);
 return ret;
}

static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 3);

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600,
			      256 * 1600, 1, 1 * 1600);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 0,
			      65536 * 16, 1, 0);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	info->sensor_indet.ctrl_hotplug = v4l2_ctrl_new_std(handler, ops, V4L2_CID_DV_TX_HOTPLUG,
				HOTPLUT_DP_OUT, HOTPLUT_DP_MAX, 0, HOTPLUT_DP_OUT);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}

	sd->ctrl_handler = handler;

	return ret;

}

static ssize_t online_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct sensor_info *info = to_state(sd);
	struct lt6911_state *state = lt_state(info);
	bool online;

	mutex_lock(&info->sensor_indet.detect_lock);
	online = !state->input_error && !state->probing &&
		 !state->pending_count && !atomic_read(&state->irq_pending);
	mutex_unlock(&info->sensor_indet.detect_lock);
	return sysfs_emit(buf, "0x%x\n", online);
}
static DEVICE_ATTR_RO(online);

static ssize_t signal_debug_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
 struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
 struct sensor_info *info = to_state(sd);
 struct lt6911_state *state = lt_state(info);
 struct v4l2_bt_timings bt;
 unsigned int probes, failures;
 int input_error, read_error, irq_count, gpio_level;

 mutex_lock(&info->sensor_indet.detect_lock);
 bt = state->last_sample.bt;
 input_error = state->input_error;
 read_error = state->last_read_error;
 probes = state->probe_count;
 failures = state->probe_failures;
 irq_count = atomic_read(&state->irq_count);
 mutex_unlock(&info->sensor_indet.detect_lock);
 gpio_level = gpio_get_value_cansleep(info->sensor_indet.hotplug_det_gpio.gpio);

 return sysfs_emit(buf,
   "input_error=%d read_error=%d gpio=%d irqs=%d probes=%u failures=%u raw=%ux%u pixelclock=%llu\n",
   input_error, read_error, gpio_level, irq_count, probes, failures,
   bt.width, bt.height, (unsigned long long)bt.pixelclock);
}
static DEVICE_ATTR_RO(signal_debug);

static int sensor_probe(struct i2c_client *client)
{

	struct v4l2_subdev *sd;
	struct sensor_info *info;
	struct device_node *np;
	int ret;

	info = kzalloc(sizeof(struct lt6911_state), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	np = of_find_node_by_name(NULL, "sensor_detect");
	lt_state(info)->reset_fixed = np &&
		of_property_read_bool(np, "lontium,reset-tied-high");
	of_node_put(np);
	sd = &info->sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	mutex_init(&info->sensor_indet.detect_lock);
	mutex_init(&lt_state(info)->io_lock);
 atomic_set(&lt_state(info)->irq_pending, 0);
 atomic_set(&lt_state(info)->irq_count, 0);
 lt_state(info)->probe_after = jiffies;
 INIT_DELAYED_WORK(&info->sensor_indet.sensor_work, sensor_det_work);
 info->sensor_indet.hotplug_det_gpio.gpio = -1;
 info->sensor_indet.reset_gpio.gpio = -1;
 info->sensor_indet.power_gpio.gpio = -1;
 lt_state(info)->power_switch = -1;
 lt_state(info)->input_error = -ENOLINK;
 ret = sensor_init_controls(sd, &sensor_ctrl_ops);
 if (ret) goto fail_controls;
	mutex_init(&info->lock);
	restart = 0;

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->combo_mode = CMB_TERMINAL_RES | CMB_PHYA_OFFSET2 | MIPI_NORMAL_MODE;
	info->time_hs = 0x18;
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->af_first_flag = 1;
	info->exp = 0;
	info->gain = 0;
	info->not_detect_use_count = 1;
#if IS_ENABLED(CONFIG_SAME_I2C)
	info->sensor_i2c_addr = I2C_ADDR >> 1;
#endif

	ret = sensor_det_init(sd);
 if (ret) {
  sensor_det_exit(sd);
  v4l2_ctrl_handler_free(&info->handler);
  goto fail_controls;
 }
 ret = device_create_file(&client->dev, &dev_attr_online);
 if (ret) {
  sensor_det_exit(sd);
  v4l2_ctrl_handler_free(&info->handler);
  goto fail_controls;
 }
 ret = device_create_file(&client->dev, &dev_attr_signal_debug);
 if (ret) {
  device_remove_file(&client->dev, &dev_attr_online);
  sensor_det_exit(sd);
  v4l2_ctrl_handler_free(&info->handler);
  goto fail_controls;
 }
 schedule_delayed_work(&info->sensor_indet.sensor_work, msecs_to_jiffies(500));
 return 0;
fail_controls:
 cci_dev_remove_helper(client, &cci_drv);
 kfree(info);
 return ret;
}

static void sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	device_remove_file(&client->dev, &dev_attr_signal_debug);
	device_remove_file(&client->dev, &dev_attr_online);
	if (client)
		sensor_det_exit(i2c_get_clientdata(client));
	else
		sensor_det_exit(cci_drv.sd);

	sd = cci_dev_remove_helper(client, &cci_drv);
	v4l2_ctrl_handler_free(&to_state(sd)->handler);
	kfree(to_state(sd));

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
