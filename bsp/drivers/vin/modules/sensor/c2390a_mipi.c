/* SPDX-License-Identifier: GPL-2.0-or-later */

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

MODULE_AUTHOR("zsm");
MODULE_DESCRIPTION("A low-level driver for C2390A sensors");
MODULE_LICENSE("GPL");

#define MCLK              (24*1000*1000)
#define V4L2_IDENT_SENSOR 0x0203

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE 20

/*
 * The SC2232 i2c address
 */
#define I2C_ADDR 0x6c

#define SENSOR_NUM 0x2
#define SENSOR_NAME "C2390A_mipi"
#define SENSOR_NAME_2 "C2390A_mipi_2"
#define SENSOR_MAXGAIN     (32<<4)


/*
 * The default register settings
 */

static struct regval_list sensor_default_regs[] = {

};

static struct regval_list sensor_1080p20_regs[] = {
	{0x340f, 0x15},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0100, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},

	{0x0103, 0x01},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x3000, 0x80},
	{0x3080, 0x01},
	{0x3081, 0x14},
	{0x3082, 0x01},
	{0x3083, 0x4b},
	{0x3087, 0xd0},
	{0x3089, 0x10},
	{0x3180, 0x10},
	{0x3182, 0x30},
	{0x3183, 0x10},
	{0x3184, 0x20},
	{0x3185, 0xc0},
	{0x3189, 0x50},
	{0x3c03, 0x00},
	{0x3f8c, 0x00},
	{0x320f, 0x48},
	{0x3023, 0x00},
	{0x3d00, 0x33},
	{0x3c9d, 0x01},
	{0x3f08, 0x00},
	{0x0309, 0x5e},
	{0x0303, 0x01},
	{0x0304, 0x01},
	{0x0307, 0x56},
	{0x3508, 0x00},
	{0x3509, 0xcc},
	{0x3292, 0x28},
	{0x350a, 0x22},
	{0x3209, 0x05},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x3209, 0x04},
	{0x3108, 0xcd},
	{0x3109, 0x7c},
	{0x310a, 0x42},
	{0x310b, 0x02},
	{0x3112, 0x55}, /* max65 */
	{0x3113, 0x00},
	{0x3114, 0xc0},
	{0x3115, 0x10},
	{0x3905, 0x01},
	{0x3980, 0x01},
	{0x3881, 0x04},
	{0x3882, 0x15},
	{0x328b, 0x03},
	{0x328c, 0x00},
	{0x3981, 0x57},
	{0x3180, 0x10},
	{0x3213, 0x00},
	{0x3205, 0x40},
	{0x3208, 0x8d},
	{0x3210, 0x12},
	{0x3211, 0x40},
	{0x3212, 0x50},
	{0x3215, 0xc0},
	{0x3216, 0x70},
	{0x3217, 0x08},
	{0x3218, 0x20},
	{0x321a, 0x80},
	{0x321b, 0x00},
	{0x321c, 0x1a},
	{0x321e, 0x00},
	{0x3223, 0x20},
	{0x3224, 0x88},
	{0x3225, 0x00},
	{0x3226, 0x08},
	{0x3227, 0x00},
	{0x3228, 0x00},
	{0x3229, 0x08},
	{0x322a, 0x00},
	{0x322b, 0x44},
	{0x308a, 0x00},
	{0x308b, 0x00},
	{0x3280, 0x06},
	{0x3281, 0x30},
	{0x3282, 0x08},
	{0x3283, 0x51},
	{0x3284, 0x0d},
	{0x3285, 0x41},
	{0x3286, 0x3b},
	{0x3287, 0x07},
	{0x3288, 0x0b},
	{0x3289, 0x00},
	{0x328a, 0x08},
	{0x328d, 0x06},
	{0x328e, 0x10},
	{0x328f, 0x0d},
	{0x3290, 0x10},
	{0x3291, 0x00},
	{0x3292, 0x28},
	{0x3293, 0x00},
	{0x3216, 0x50},
	{0x3217, 0x04},
	{0x3205, 0x20},
	{0x3215, 0x50},
	{0x3223, 0x10},
	{0x3280, 0x06},
	{0x3282, 0x0a},
	{0x3283, 0x50},
	{0x308b, 0x05},
	{0x3184, 0x20},
	{0x3185, 0xc0},
	{0x3189, 0x50},
	{0x3280, 0x86},
	{0x0003, 0x00},
	{0x3280, 0x06},
	{0x0383, 0x01},
	{0x0387, 0x01},
	{0x0340, 0x06},/* for 20fps vts 1656 */
	{0x0341, 0x78},
	{0x0342, 0x08},/* hts 2076 */
	{0x0343, 0x1c},
	{0x034c, 0x07},
	{0x034d, 0x80},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x3b80, 0x46},
	{0x3b81, 0x10},
	{0x3b82, 0x10},
	{0x3b83, 0x10},
	{0x3b84, 0x04},
	{0x3b85, 0x30},
	{0x3b86, 0x80},
	{0x3b86, 0x80},
	{0x3021, 0x11},
	{0x3022, 0xa2},
	{0x3209, 0x04},
	{0x3584, 0x12},
	{0x3805, 0x05},
	{0x3806, 0x03},
	{0x3807, 0x03},
	{0x3808, 0x0c},
	{0x3809, 0x64},
	{0x380a, 0x5b},
	{0x380b, 0xe6},
	{0x3500, 0x10},
	{0x308c, 0x2c},
	{0x308d, 0x31},
	{0x3403, 0x00},
	{0x3407, 0x02},
	{0x3410, 0x04},
	{0x3411, 0x18},
	{0x3414, 0x02},
	{0x3415, 0x02},
	{0xe000, 0x32},
	{0xe001, 0x85},
	{0xe002, 0x41},
	{0xe003, 0x3b},
	{0xe004, 0x81},
	{0xe005, 0x10},
	{0xe030, 0x32},
	{0xe031, 0x85},
	{0xe032, 0x48},
	{0xe033, 0x3b},
	{0xe034, 0x81},
	{0xe035, 0x50},
	{0xe120, 0x35},
	{0xe121, 0x00},
	{0xe122, 0x02},
	{0xe123, 0x03},
	{0xe124, 0x09},
	{0xe125, 0x2e},
	{0x3500, 0x00},
	{0x3a87, 0x02},
	{0x3a88, 0x08},
	{0x3a89, 0x30},
	{0x3a8a, 0x01},
	{0x3a8b, 0x90},
	{0x3a80, 0x88},
	{0x3a81, 0x02},
	{0x3d03, 0x90},
	{0x3d04, 0xff},
	{0x3d05, 0x24},
	{0x3d06, 0x00},
	{0x3d07, 0x00},
	{0x3d09, 0x78},
	{0x3d0a, 0xff},
	{0x3d0d, 0x10},
	{0x3009, 0x05}, /* 08 for 0101=0x00 */
	{0x300b, 0x08}, /* 08 for 0101=0x00 */
	{0x0101, 0x01}, /* mirror&flip */
	{0x034b, 0x47},
	{0x0202, 0x00},
	{0x0203, 0x50},

	{0x3500, 0x10},/* add by James 20170629 group access for night mode */
	{0x3401, 0x01},/* modify by lujie 20190729 */
	{0x3405, 0x02},
	{0xe00c, 0x32},
	{0xe00d, 0x8e},
	{0xe00e, 0x20},
	{0xe00f, 0x02},
	{0xe010, 0x05},
	{0xe011, 0x10},
	{0x3500, 0x00},
	{0x3290, 0x00},
	{0x0400, 0x20},
	{0x0401, 0x02},/* get temperature */

	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x3500, 0x00},
	{0x0309, 0x56},
	{0x0100, 0x01},
	{0x0003, 0x00},
	{0x0003, 0x00},
	{0x0003, 0x00},

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

static int sensor_g_temperature(struct v4l2_subdev *sd,
				struct sensor_temp *temp)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval = 0;

	sensor_read(sd, 0x0405, &rdval);
	temp->temp = (int)rdval;
	sensor_dbg("sensor_get_temperature = %d\n", temp->temp);

	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	*value = info->exp;
	sensor_dbg("sensor_get_exposure = %d\n", info->exp);
	return 0;
}

static int sc2232_sensor_vts;
static int shutter_delay = 1;
static int shutter_delay_cnt;
static int fps_change_flag;

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	data_type explow, exphigh;
	struct sensor_info *info = to_state(sd);

	exphigh = (unsigned char)(0xff & (exp_val >> 8));
	explow = (unsigned char)(0xff & (exp_val << 0));

	sensor_write(sd, 0x0203, explow);
	sensor_write(sd, 0x0202, exphigh);

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

	data_type again = 0;
	data_type again_H = 0;
	data_type again_L = 0;
	data_type night_mode = 0x20;
	int gain_lvl = gain_val;

	if (gain_lvl < 16) {
		gain_lvl = 16;
	} else if (gain_lvl > SENSOR_MAXGAIN * 16) {
		gain_lvl = SENSOR_MAXGAIN * 16;
	}

	if (gain_lvl <= 16 * 16) {
		again_H = (gain_lvl / 16) - 1;
		again_L = gain_lvl % 16;
		again = (again_H << 4) + again_L;
		night_mode = 0x20; /* 20 */
	} else {
		again_H = (gain_lvl / 2 / 16) - 1;
		again_L = gain_lvl / 2 % 16;
		again = (again_H << 4) + again_L;
		night_mode = 0x10; /* 10 */
	}
	sensor_write(sd, 0xe00e, (unsigned char)night_mode);
	sensor_write(sd, 0xe011, (unsigned char)again);

	info->gain = gain_val;
	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	struct sensor_info *info = to_state(sd);
	int exp_val, gain_val;

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	if (gain_val < 1 * 16)
		gain_val = 16;
	if (exp_val > 0xfffff)
		exp_val = 0xfffff;
	if (fps_change_flag) {
		if (shutter_delay_cnt == shutter_delay) {
			/* sensor_write(sd, 0x320f, sc2232_sensor_vts /
			(sc2232_sensor_svr + 1) & 0xFF);
			sensor_write(sd, 0x320e, sc2232_sensor_vts /
			(sc2232_sensor_svr + 1) >> 8 & 0xFF);
			sensor_write(sd, 0x302d, 0);
			shutter_delay_cnt = 0; */
			fps_change_flag = 0;
		} else
			shutter_delay_cnt++;
	}

	/* sensor_write(sd, 0x3812, 0x00); */
	sensor_s_exp(sd, exp_val);
	sensor_s_gain(sd, gain_val);
	sensor_write(sd, 0x340f, 0x11);

	sensor_dbg("sensor_set_gain exp = %d, %d Done!\n", gain_val, exp_val);

	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_s_fps(struct v4l2_subdev *sd, struct sensor_fps *fps)
{
	/* data_type rdval1, rdval2, rdval3; */
	struct sensor_info *info = to_state(sd);
	struct sensor_win_size *wsize = info->current_wins;

	sc2232_sensor_vts = wsize->pclk / fps->fps / wsize->hts;
	fps_change_flag = 1;

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
		ret = sensor_write(sd, 0x0100, rdval&0x00);
	else
		ret = sensor_write(sd, 0x0100, rdval|0x01);
	return ret;
}

/*
 *set && get sensor flip
 */
static int sensor_get_fmt_mbus_core(struct v4l2_subdev *sd, int *code)
{
	struct sensor_info *info = to_state(sd);
	data_type get_value;

	sensor_read(sd, 0x0101, &get_value);
	switch (get_value) {
	case 0x00:
		*code = MEDIA_BUS_FMT_SGBRG10_1X10;
		break;
	case 0x01:
		*code = MEDIA_BUS_FMT_SBGGR10_1X10;
		break;
	case 0x10:
		*code = MEDIA_BUS_FMT_SRGGB10_1X10;
		break;
	case 0x11:
		*code = MEDIA_BUS_FMT_SGRBG10_1X10;
		break;
	default:
		*code = info->fmt->mbus_code;
	}
	return 0;
}

data_type sensor_flip_status;

static int sensor_s_hflip(struct v4l2_subdev *sd, int enable)
{
	data_type get_value;
	data_type set_value = 0;

	if (!(enable == 0 || enable == 1))
		return -1;

	/* sensor_read(sd, 0x0101, &get_value); */
	get_value = sensor_flip_status;
	if (enable)
		set_value = get_value ^ 0x01;

	sensor_flip_status = set_value;
	sensor_write(sd, 0x0101, set_value);
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int enable)
{
	data_type get_value;
	data_type set_value = 0;

	if (!(enable == 0 || enable == 1))
		return -1;

	/* sensor_read(sd, 0x17, &get_value); */
	get_value = sensor_flip_status;
	if (enable)
		set_value = get_value ^ 0x02;

	sensor_flip_status = set_value;
	sensor_write(sd, 0x0101, set_value);
	return 0;
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
		/* vin_gpio_set_status(sd, PWDN, 1); */
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_set_status(sd, POWER_EN, 1);
		/* vin_gpio_write(sd, RESET, CSI_GPIO_HIGH); */
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		/* vin_gpio_write(sd, PWDN, CSI_GPIO_LOW); */
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_set_pmu_channel(sd, IOVDD, ON);
		vin_set_pmu_channel(sd, DVDD, ON);

		usleep_range(10000, 12000);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		/* vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH); */
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
		/* vin_gpio_set_status(sd, PWDN, 1); */
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		/* vin_gpio_write(sd, PWDN, CSI_GPIO_LOW); */
		vin_set_mclk(sd, OFF);
		vin_set_pmu_channel(sd, AFVDD, OFF);
		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		/* vin_gpio_set_status(sd, PWDN, 0); */
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
	unsigned int SENSOR_ID = 0;
	data_type rdval;
	int cnt = 0;

	sensor_print("%s: start\n", __func__);

	sensor_read(sd, 0x0000, &rdval);
	SENSOR_ID |= (rdval << 8);
	sensor_read(sd, 0x0001, &rdval);
	SENSOR_ID |= (rdval);
	sensor_print("V4L2_IDENT_SENSOR = 0x%x\n", SENSOR_ID);

	while ((SENSOR_ID != V4L2_IDENT_SENSOR) && (cnt < 5)) {
		sensor_read(sd, 0x0000, &rdval);
		SENSOR_ID |= (rdval << 8);
		sensor_read(sd, 0x0001, &rdval);
		SENSOR_ID |= (rdval);
		sensor_print("retry = %d, V4L2_IDENT_SENSOR = %x\n",
			cnt, SENSOR_ID);
		cnt++;
		}
	if (SENSOR_ID != V4L2_IDENT_SENSOR)
		return -ENODEV;

	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	sensor_print("sensor_init, start sensor_detect\n");

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
	info->tpf.denominator = 20;	/* 30fps */

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
	case VIDIOC_VIN_SENSOR_GET_TEMP:
		sensor_g_temperature(sd, (struct sensor_temp *)arg);
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
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
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

	{
	 .width = 1920,
	 .height = 1080,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 2076,
	 .vts = 1656,
	 .pclk = 68800000,
	 .mipi_bps = 344000000,
	 .fps_fixed = 20,
	 .bin_factor = 1,
	 .intg_min = 2,
	 .intg_max = 1656,
	 .gain_min = 1 << 4,
	 .gain_max = 32 << 4,
	 .regs = sensor_1080p20_regs,
	 .regs_size = ARRAY_SIZE(sensor_1080p20_regs),
	 .set_size = NULL,
	 },
/*
	{
	 .width = 1920,
	 .height = 1080,
	 .hoffset = 0,
	 .voffset = 0,
	 .hts = 2400,
	 .vts = 1125,
	 .pclk = 67 * 5000 * 1000,
	 .mipi_bps = 337 * 5000 * 1000,
	 .fps_fixed = 25,
	 .bin_factor = 1,
	 .intg_min = 3 << 4,
	 .intg_max = (1125 - 2) << 4,
	 .gain_min = 1 << 4,
	 .gain_max = 1440 << 4,
	 .regs = sensor_1080p25_regs,
	 .regs_size = ARRAY_SIZE(sensor_1080p25_regs),
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

	sensor_flip_status = 0x01;

	if (wsize->regs)
		sensor_write_array(sd, wsize->regs, wsize->regs_size);

	if (wsize->set_size)
		wsize->set_size(sd);

	info->width = wsize->width;
	info->height = wsize->height;
	sc2232_sensor_vts = wsize->vts;

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
		.name = SENSOR_NAME,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}, {
		.name = SENSOR_NAME_2,
		.addr_width = CCI_BITS_16,
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

	v4l2_ctrl_handler_init(handler, 2);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 1600, 256 * 1600, 1,
			  1 * 1600);
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

	sensor_print("%s: start\n", __func__);

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
	info->combo_mode = CMB_PHYA_OFFSET2 | MIPI_NORMAL_MODE;
	/* info->time_hs = 0x23; */
	info->stream_seq = MIPI_BEFORE_SENSOR;
	info->af_first_flag = 1;
	info->exp = 0;
	info->gain = 0;

	sensor_print("%s: end\n", __func__);
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

	sensor_print("%s: ok\n", __func__);
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
