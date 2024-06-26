/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * A V4L2 driver for gc1034_mipi Raw cameras.
 *
 */
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

MODULE_AUTHOR("pcw");
MODULE_DESCRIPTION("A low-level driver for gc1034_mipi Raw sensors");
MODULE_LICENSE("GPL");

/* for internal driver debug */
#define DEV_DBG_EN      0
#if (DEV_DBG_EN == 1)
#define vfe_dev_dbg(x, arg...) pr_debug("[GC1034 Raw]"x, ##arg)
#else
#define vfe_dev_dbg(x, arg...)
#endif
#define vfe_dev_err(x, arg...) pr_err("[GC1034 Raw]"x, ##arg)
#define vfe_dev_print(x, arg...) pr_info("[GC1034 Raw]"x, ##arg)

/* define module timing */
#define MCLK			(24*1000*1000)
#define V4L2_IDENT_SENSOR	0x1034

#define ID_REG_HIGH		0xf0
#define ID_REG_LOW		0xf1
#define ID_VAL_HIGH		((V4L2_IDENT_SENSOR) >> 8)
#define ID_VAL_LOW		((V4L2_IDENT_SENSOR) & 0xff)

#define ANALOG_GAIN_1 64	/* 1.00x */
#define ANALOG_GAIN_2 91	/* 1.42x */
#define ANALOG_GAIN_3 127	/* 1.99x */
#define ANALOG_GAIN_4 182	/* 2.85x */
#define ANALOG_GAIN_5 258	/* 4.03x */
#define ANALOG_GAIN_6 369	/* 5.77x */
#define ANALOG_GAIN_7 516	/* 8.06x */
#define ANALOG_GAIN_8 738	/* 11.53x */
#define ANALOG_GAIN_9 1032	/* 16.12x */

/*
 * Our nominal (default) frame rate.
 */

#define SENSOR_FRAME_RATE	30


/*
 * The gc1034 i2c address
 */
#define I2C_ADDR 0x42
#define SENSOR_NAME "gc1034_mipi"

/* static struct delayed_work sensor_s_ae_ratio_work; */
static struct v4l2_subdev *glb_sd;


/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;	/* coming later */

struct cfg_array {
	/* coming later */
	struct regval_list *regs;
	int size;
};

static int
LOG_ERR_RET(int x)
{
	int ret;

	ret = x;
	if (ret < 0)
		vfe_dev_err("error at %s\n", __func__);
	return ret;
}

static inline struct sensor_info *
to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}

/*
 * The default register settings
 *
 */

/*static struct regval_list sensor_default_regs[] = {
};*/

/* 720P@30fps */
static struct regval_list sensor_720p_regs[] = {
	/*
	 *****************************************************
	 ********************   SYS   ***********************
	 *****************************************************
	 */
	{0xf2, 0x00},
	{0xf6, 0x00},
	{0xfc, 0x04},
	{0xf7, 0x01},
	{0xf8, 0x0c},/* 0c=25fps */
	{0xf9, 0x06},
	{0xfa, 0x80},
	{0xfc, 0x0e},
	/*
	 ****************************************************
	 ***************   ANALOG & CISCTL      ************
	 ****************************************************
	 */
	{0xfe, 0x00},
	{0x03, 0x00}, /* 0x02 */
	{0x04, 0xa6}, /* 9xa6 */
	{0x05, 0x02}, /* HB */
	{0x06, 0x07},
	{0x07, 0x00}, /* HB */
	{0x08, 0x0a},
	{0x09, 0x00},
	{0x0a, 0x04}, /* row start */
	{0x0b, 0x00},
	{0x0c, 0x00}, /* col start */
	{0x0d, 0x02},
	{0x0e, 0xd4}, /* height 724 */
	{0x0f, 0x05},
	{0x10, 0x08}, /* width 1288 */
	{0x17, 0xc0},
	{0x18, 0x02},
	{0x19, 0x08},
	{0x1a, 0x18},
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x21, 0x30},
	{0x23, 0xf8},
	{0x25, 0x10},
	{0x28, 0x20},
	{0x34, 0x08}, /* data low */
	{0x3c, 0x10},
	{0x3d, 0x0e},
	{0xcc, 0x8e},
	{0xcd, 0x9a},
	{0xcf, 0x70},
	{0xd0, 0xab},
	{0xd1, 0xc5},
	{0xd2, 0xed}, /* data high */
	{0xd8, 0x3c}, /* dacin offset */
	{0xd9, 0x7a},
	{0xda, 0x12},
	{0xdb, 0x50},
	{0xde, 0x0c},
	{0xe3, 0x60},
	{0xe4, 0x78},
	{0xfe, 0x01},
	{0xe3, 0x01},
	{0xe6, 0x10}, /* ramps offset */
	/*
	 *****************************************************
	 **********************  ISP   **********************
	 *****************************************************
	 */
	{0xfe, 0x01},
	{0x80, 0x50},
	{0x88, 0x73},
	{0x89, 0x03},
	{0x90, 0x01},
	{0x92, 0x02}, /* crop win 2<=y<=4 */
	{0x94, 0x03}, /* crop win 2<=x<=5 */
	{0x95, 0x02}, /* crop win height */
	{0x96, 0xd0},
	{0x97, 0x05}, /* crop win width */
	{0x98, 0x00},
	/*
	 *****************************************************
	 **********************  BLK   **********************
	 *****************************************************
	 */
	{0xfe, 0x01},
	{0x40, 0x22},
	{0x43, 0x03},
	{0x4e, 0x3c},
	{0x4f, 0x00},
	{0x60, 0x00},
	{0x61, 0x80},
	/*
	 *****************************************************
	 **********************  GAIN   **********************
	 *****************************************************
	 */
	{0xfe, 0x01},
	{0xb0, 0x48},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb6, 0x00},
	{0xfe, 0x02},
	{0x01, 0x00},
	{0x02, 0x01},
	{0x03, 0x02},
	{0x04, 0x03},
	{0x05, 0x04},
	{0x06, 0x05},
	{0x07, 0x06},
	{0x08, 0x0e},
	{0x09, 0x16},
	{0x0a, 0x1e},
	{0x0b, 0x36},
	{0x0c, 0x3e},
	{0x0d, 0x56},
	{0xfe, 0x02},
	{0xb0, 0x00}, /* col_gain[11:8] */
	{0xb1, 0x00},
	{0xb2, 0x00},
	{0xb3, 0x11},
	{0xb4, 0x22},
	{0xb5, 0x54},
	{0xb6, 0xb8},
	{0xb7, 0x60},
	{0xb9, 0x00}, /* col_gain[12] */
	{0xba, 0xc0},
	{0xc0, 0x20}, /* col_gain[7:0] */
	{0xc1, 0x2d},
	{0xc2, 0x40},
	{0xc3, 0x5b},
	{0xc4, 0x80},
	{0xc5, 0xb5},
	{0xc6, 0x00},
	{0xc7, 0x6a},
	{0xc8, 0x00},
	{0xc9, 0xd4},
	{0xca, 0x00},
	{0xcb, 0xa8},
	{0xcc, 0x00},
	{0xcd, 0x50},
	{0xce, 0x00},
	{0xcf, 0xa1},
	/*
	 *****************************************************
	 **********************  DARKSUN   ******************
	 *****************************************************
	 */
	{0xfe, 0x02},
	{0x54, 0xf7},
	{0x55, 0xf0},
	{0x56, 0x00},
	{0x57, 0x00},
	{0x58, 0x00},
	{0x5a, 0x04},
	/*
	 *****************************************************
	 **********************  DD   ***********************
	 *****************************************************
	 */
	{0xfe, 0x04},
	{0x81, 0x8a},
	/*
	 *****************************************************
	 **********************  MIPI   *********************
	 *****************************************************
	 */
	{0xfe, 0x03},
	{0x01, 0x03},
	{0x02, 0x11},
	{0x03, 0x90},
	{0x10, 0x90},
	{0x11, 0x2b},
	{0x12, 0x40},
	{0x13, 0x06},

	{0x15, 0x00},
	{0x21, 0x02},
	{0x22, 0x02},
	{0x23, 0x08},
	{0x24, 0x02},
	{0x25, 0x10},
	{0x26, 0x04},
	{0x29, 0x03},
	{0x2a, 0x02},
	{0x2b, 0x04},
	{0xfe, 0x00},

};

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */

static struct regval_list sensor_fmt_raw[] = {
};

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval;

	LOG_ERR_RET(sensor_write(sd, 0xfe, 0x00));
	LOG_ERR_RET(sensor_read(sd, 0x17, &rdval));

	*value = rdval & 0x01;

	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval;

	if (info->hflip == value)
	return 0;

	LOG_ERR_RET(sensor_write(sd, 0xfe, 0x00));
	LOG_ERR_RET(sensor_read(sd, 0x17, &rdval));

	switch (value) {
	case 0:
		rdval &= 0xfe;
		break;
	case 1:
		rdval |= 0x01;
		break;
	default:
		return -EINVAL;
	}

	LOG_ERR_RET(sensor_write(sd, 0x17, rdval));

	usleep_range(10000, 12000);
	info->hflip = value;
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval;

	LOG_ERR_RET(sensor_write(sd, 0xfe, 0x00));
	LOG_ERR_RET(sensor_read(sd, 0x17, &rdval));

	*value = (rdval >> 1) & 0x01;

	info->vflip = *value;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);
	data_type rdval;

	if (info->vflip == value)
		return 0;

	LOG_ERR_RET(sensor_write(sd, 0xfe, 0x00));
	LOG_ERR_RET(sensor_read(sd, 0x17, &rdval));

	switch (value) {
	case 0:
		rdval &= 0xfd;
		break;
	case 1:
		rdval |= 0x02;
		break;
	default:
		return -EINVAL;
	}


	LOG_ERR_RET(sensor_write(sd, 0x17, rdval));

	usleep_range(10000, 12000);
	info->vflip = value;
	return 0;
}

static int
sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->exp;
	vfe_dev_dbg("sensor_get_exposure = %d\n", info->exp);
	return 0;
}

static int
sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned char explow, expmid, exphigh;
	struct sensor_info *info = to_state(sd);

	if (exp_val > 0xfffff)
		exp_val = 0xfffff;
	if (exp_val < 0x40)
		exp_val = 0x40;

	sensor_write(sd, 0xfe, 0x00);

	exphigh = 0;
	expmid = (unsigned char)((0x0ff000 & exp_val) >> 12);
	explow = (unsigned char)((0x000ff0 & exp_val) >> 4);

	sensor_write(sd, 0x03, expmid);
	sensor_write(sd, 0x04, explow);

	info->exp = exp_val;
	return 0;
}


static int
sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->gain;
	vfe_dev_dbg("sensor_get_gain = %d\n", info->gain);

	return 0;
}

static int
sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	unsigned char tmp;
	struct sensor_info *info = to_state(sd);

	gain_val = gain_val * 4;

	sensor_write(sd, 0xfe, 0x01);
	sensor_write(sd, 0xb1, 0x01);
	sensor_write(sd, 0xb2, 0x00);

	if (gain_val < 0x40) {
		gain_val = 0x40;
	} else if ((gain_val >= ANALOG_GAIN_1) && (gain_val < ANALOG_GAIN_2)) {
		sensor_write(sd, 0xb6, 0x00);
		tmp = gain_val;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_2) && (gain_val < ANALOG_GAIN_3)) {
		sensor_write(sd, 0xb6, 0x01);
		tmp = 64 * gain_val / ANALOG_GAIN_2;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_3) && (gain_val < ANALOG_GAIN_4)) {
		sensor_write(sd, 0xb6, 0x02);
		tmp = 64 * gain_val / ANALOG_GAIN_3;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_4) && (gain_val < ANALOG_GAIN_5)) {
		sensor_write(sd, 0xb6, 0x03);
		tmp = 64 * gain_val / ANALOG_GAIN_4;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_5) && (gain_val < ANALOG_GAIN_6)) {
		sensor_write(sd, 0xb6, 0x04);
		tmp = 64 * gain_val / ANALOG_GAIN_5;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_6) && (gain_val < ANALOG_GAIN_7)) {
		sensor_write(sd, 0xb6, 0x05);
		tmp = 64 * gain_val / ANALOG_GAIN_6;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_7) && (gain_val < ANALOG_GAIN_8)) {
		sensor_write(sd, 0xb6, 0x06);
		tmp = 64 * gain_val / ANALOG_GAIN_7;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if ((gain_val >= ANALOG_GAIN_8) && (gain_val < ANALOG_GAIN_9)) {
		sensor_write(sd, 0xb6, 0x07);
		tmp = 64 * gain_val / ANALOG_GAIN_8;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	} else if (gain_val >= ANALOG_GAIN_9) {
		sensor_write(sd, 0xb6, 0x08);
		tmp = 64 * gain_val / ANALOG_GAIN_9;
		sensor_write(sd, 0xb1, tmp >> 6);
		sensor_write(sd, 0xb2, (tmp << 2) & 0xfc);
	}

	sensor_write(sd, 0xfe, 0x00);

	info->gain = gain_val;

	return 0;
}

static int gc1034_mipi_sensor_vts;
static int
sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *exp_gain)
{
	struct sensor_info *info = to_state(sd);
	int exp_val, gain_val;

	exp_val = exp_gain->exp_val;
	gain_val = exp_gain->gain_val;

	if (gain_val < 1 * 16)
		gain_val = 16;
	if (gain_val > 64 * 16 - 1)
		gain_val = 64 * 16 - 1;

	if (exp_val < 0x40)
		exp_val = 0x40;
	if (exp_val > 0xfffff)
		exp_val = 0xfffff;
	sensor_s_exp(sd, exp_val);
	sensor_s_gain(sd, gain_val);

	vfe_dev_dbg("sensor_s_exp_gain gain_val = %d, exp_val = %d\n",
		    gain_val, exp_val);

	info->exp = exp_val;
	info->gain = gain_val;

	return 0;
}


/*
 * Stuff that knows about the sensor.
 */

static int
sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret;

	ret = 0;
	switch (on) {
	case CSI_SUBDEV_STBY_ON:
		vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
		usleep_range(10000, 12000);
		cci_lock(sd);
		vfe_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		cci_unlock(sd);
		vfe_set_mclk(sd, OFF);
		break;
	case CSI_SUBDEV_STBY_OFF:
		vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");
		cci_lock(sd);
		vfe_set_mclk_freq(sd, MCLK);
		vfe_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vfe_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		cci_unlock(sd);
		break;
	case CSI_SUBDEV_PWR_ON:
		vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
		cci_lock(sd);

		/* set the gpio to output */
		vfe_gpio_set_status(sd, PWDN, 1);
		vfe_gpio_set_status(sd, RESET, 1);
		vfe_gpio_set_status(sd, POWER_EN, 1);

		vfe_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vfe_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vfe_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		usleep_range(1000, 1200);

		/* power supply */
		vfe_set_pmu_channel(sd, IOVDD, ON);
		vfe_set_pmu_channel(sd, AVDD, ON);
		vfe_set_pmu_channel(sd, DVDD, ON);
		vfe_set_pmu_channel(sd, AFVDD, ON);
		usleep_range(7000, 8000);

		vfe_set_mclk_freq(sd, MCLK);
		vfe_set_mclk(sd, ON);
		usleep_range(10000, 12000);

		vfe_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		vfe_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		usleep_range(10000, 12000);

		cci_unlock(sd);
		break;
	case CSI_SUBDEV_PWR_OFF:

		vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
		cci_lock(sd);
		vfe_set_mclk(sd, OFF);

		/* set the gpio to output */
		vfe_gpio_set_status(sd, PWDN, 1);
		vfe_gpio_set_status(sd, RESET, 1);
		vfe_gpio_set_status(sd, POWER_EN, 1);

		vfe_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vfe_gpio_write(sd, PWDN, CSI_GPIO_LOW);

		/* power supply */
		vfe_set_pmu_channel(sd, IOVDD, OFF);
		vfe_set_pmu_channel(sd, AVDD, OFF);
		vfe_set_pmu_channel(sd, DVDD, OFF);
		vfe_set_pmu_channel(sd, AFVDD, OFF);
		usleep_range(10000, 12000);

		vfe_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);

		vfe_gpio_set_status(sd, PWDN, 0);
		vfe_gpio_set_status(sd, RESET, 0);
		vfe_gpio_set_status(sd, POWER_EN, 0);
		cci_unlock(sd);

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int
sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch (val) {
	case 0:
		vfe_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);
		break;
	case 1:
		vfe_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int
sensor_detect(struct v4l2_subdev *sd)
{
	data_type rdval;

	vfe_dev_dbg("sensor_detect!\n");

	LOG_ERR_RET(sensor_read(sd, ID_REG_HIGH, &rdval));
	vfe_dev_dbg("***sensor gc1034,read 0xf0 is 0x%x***\n", rdval);
	if (rdval != ID_VAL_HIGH)
		return -ENODEV;

	LOG_ERR_RET(sensor_read(sd, ID_REG_LOW, &rdval));
	vfe_dev_dbg("***sensor gc1034,read 0xf1 is 0x%x***\n", rdval);
	if (rdval != ID_VAL_LOW)
		return -ENODEV;

	return 0;
}

static int
sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_init\n");

	/*Make sure it is a target sensor */
	ret = sensor_detect(sd);
	if (ret) {
		vfe_dev_err("chip found is not an target chip.\n");
		return ret;
	}

	vfe_get_standby_mode(sd, &info->stby_mode);

	if ((info->stby_mode == HW_STBY || info->stby_mode == SW_STBY)
	    && info->init_first_flag == 0) {
		vfe_dev_print("stby_mode and init_first_flag = 0\n");
		return 0;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = HD720_WIDTH;
	info->height = HD720_HEIGHT;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30;	/* 30fps */

	return ret;

	return 0;
}

static long
sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
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
			vfe_dev_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		break;
	case ISP_SET_EXP_GAIN:
		ret = sensor_s_exp_gain(sd, (struct sensor_exp_gain *) arg);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}


/*
 * Store information about the video data format.
 */
static struct sensor_format_struct {
	__u8 *desc;
	/* __u32 pixelformat; */
	u32 mbus_code;
	struct regval_list *regs;
	int regs_size;
	int bpp;			/* Bytes per pixel */
} sensor_formats[] = {
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
	/* 720P */
	{
		.width = HD720_WIDTH,
		.height = HD720_HEIGHT,
		.hoffset = 0,
		.voffset = 0,
		.hts = 1726,
		.vts = 750,
		.pclk = 39 * 1000 * 1000,
		.mipi_bps = 312 * 1000 * 1000,
		.fps_fixed = 30,
		.bin_factor = 2,
		.intg_min = 1 << 4,
		.intg_max = 764 << 4,
		.gain_min = 1 << 4,
		.gain_max = 16 << 4,
		.regs = sensor_720p_regs,
		.regs_size = ARRAY_SIZE(sensor_720p_regs),
		.set_size = NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int
sensor_enum_code(struct v4l2_subdev *sd,
		 struct v4l2_subdev_pad_config *cfg,
		 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= N_FMTS)
		return -EINVAL;

	code->code = sensor_formats[code->index].mbus_code;

	return 0;
}

static int
sensor_enum_frame_size(struct v4l2_subdev *sd,
		       struct v4l2_subdev_pad_config *cfg,
		       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > N_WIN_SIZES - 1)
		return -EINVAL;

	fse->min_width = sensor_win_sizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = sensor_win_sizes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int
sensor_try_fmt_internal(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt,
			struct sensor_format_struct **ret_fmt,
			struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)
			break;

	if (index >= N_FMTS)
		return -EINVAL;

	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;

	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;

	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;			/* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;
	fmt->height = wsize->height;
	info->current_wins = wsize;

	return 0;
}

static int
sensor_get_fmt(struct v4l2_subdev *sd,
	       struct v4l2_subdev_pad_config *cfg,
	       struct v4l2_subdev_format *fmat)
{
	struct v4l2_mbus_framefmt *fmt = &fmat->format;

	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

static int
sensor_g_mbus_config(struct v4l2_subdev *sd, struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = 0 | V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0;

	return 0;
}


/*
 * Set a format.
 */
static int
sensor_set_fmt(struct v4l2_subdev *sd,
	       struct v4l2_subdev_pad_config *cfg,
	       struct v4l2_subdev_format *fmat)
{
	int ret;
	struct v4l2_mbus_framefmt *fmt = &fmat->format;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_set_fmt\n");

	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;

	LOG_ERR_RET(sensor_write_array
		    (sd, sensor_fmt->regs, sensor_fmt->regs_size));

	if (wsize->regs)
		LOG_ERR_RET(sensor_write_array(sd, wsize->regs,
							wsize->regs_size));

	if (wsize->set_size)
		LOG_ERR_RET(wsize->set_size(sd));

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	info->exp = 0;
	info->gain = 0;
	gc1034_mipi_sensor_vts = wsize->vts;

	vfe_dev_print("s_fmt set width = %d, height = %d\n", wsize->width,
		      wsize->height);

	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int
sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = info->capture_mode;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = SENSOR_FRAME_RATE;

	return 0;
}

static int
sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("sensor_s_parm\n");

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (info->tpf.numerator == 0)
		return -EINVAL;

	info->capture_mode = cp->capturemode;

	return 0;
}

static int
sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->val);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->val);
	}
	return -EINVAL;
}

static int
sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->val);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->val);
	}

	return -EINVAL;
}

/* ---------------------------------------------------------------- */

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_parm = sensor_s_parm,
	.g_parm = sensor_g_parm,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
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

static const struct v4l2_ctrl_config sensor_custom_ctrls[] = {
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "frame rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 15,
		.max = 120,
		.step = 1,
		.def = 120,
	},
};

static int
sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;
	int i;

	v4l2_ctrl_handler_init(handler, 4 + ARRAY_SIZE(sensor_custom_ctrls));

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctrl =
		v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN,
				  1 * 16, 64 * 16 - 1, 1, 1 * 16);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	ctrl =
		v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE,
							0, 65536 * 16, 1, 0);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	for (i = 0; i < ARRAY_SIZE(sensor_custom_ctrls); i++)
		v4l2_ctrl_new_custom(handler, &sensor_custom_ctrls[i], NULL);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}

	sd->ctrl_handler = handler;

	return ret;
}

static int
sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	glb_sd = sd;
	sensor_init_controls(sd, &sensor_ctrl_ops);
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);

	info->fmt = &sensor_formats[0];
	info->af_first_flag = 1;
	info->init_first_flag = 1;

	return 0;
}

static int
sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
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

static __init int
init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void
exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
