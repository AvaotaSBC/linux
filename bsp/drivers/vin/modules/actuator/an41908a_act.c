/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for imx274 Raw cameras.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *    Liang WeiJie <liangweijie@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
#include <linux/gpio.h>
#include <sunxi-gpio.h>
#include <linux/spi/spi.h>
#include "../../platform/platform_cfg.h"
#include "actuator.h"

MODULE_AUTHOR("lwj");
MODULE_DESCRIPTION("A low-level driver for AN41908A");
MODULE_LICENSE("GPL");

#define LENS_SUNNY 1

#if VIN_FALSE
#define LENSDRV_FOCUS_STEPS_REG 0x24 /* define AB motor to Focus */
#define LENSDRV_ZOOM_STEPS_REG  0x29 /* define CD motor to Zoom */
#else
#define LENSDRV_FOCUS_STEPS_REG 0x29 /* define AB motor to Focus */
#define LENSDRV_ZOOM_STEPS_REG  0x24 /* define CD motor to Zoom */
#endif


#define SUNXI_ACT_NAME "an41908a_act"

extern void v4l2_spi_subdev_init(struct v4l2_subdev *sd, struct spi_device *spi,
		const struct v4l2_subdev_ops *ops);

static int act_spi_read_byte(struct spi_device *spi, loff_t from,
			size_t len, unsigned short *buf)
{
	struct spi_transfer t[2];
	struct spi_message m;
	unsigned char command[1] = {0};
	unsigned char data[2];

	spi_message_init(&m);
	memset(t, 0, sizeof(t));
	command[0] = from | 0x40;
	t[0].tx_buf = command;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = data;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	spi_sync(spi, &m);

	*buf = data[1] * 256 + data[0];

	return 0;
}

static int act_spi_write_byte(struct spi_device *spi, loff_t to,
			size_t len, const unsigned short buf)
{
	struct spi_transfer t;
	struct spi_message m;
	unsigned char command[3] = {0};

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	command[0] = to & 0x3F;
	command[1] = buf & 0xff;
	command[2] = (buf >> 8) & 0xff;

	t.tx_buf = command;
	t.len = 3;
	spi_message_add_tail(&t, &m);

	spi_sync(spi, &m);

	return 0;
}

static int act_spi_read(struct v4l2_subdev *sd, unsigned char reg,
				unsigned short *val)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	loff_t from = reg;

	return act_spi_read_byte(spi, from, 2, val);
}

static int act_spi_write(struct v4l2_subdev *sd, unsigned char reg,
				unsigned short val)
{
	struct spi_device *spi = v4l2_get_subdevdata(sd);
	loff_t to = reg;

	return act_spi_write_byte(spi, to, 3, val);
}

/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

#define VD_IS GPIOH(9)
#define VD_FZ GPIOH(10)

int lens_set_pulse(int gpio)
{
	if (gpio_direction_output(gpio, 0)) {
		act_err("gpio%d set 0 err!", gpio);
		return -1;
	}
	usleep_range(1000, 1200);
	if (gpio_direction_output(gpio, 1)) {
		act_err("gpio%d set 1 err!", gpio);
		return -1;
	}
	usleep_range(1000, 1200);
	if (gpio_direction_output(gpio, 0)) {
		act_err("gpio%d set 0 err!", gpio);
		return -1;
	}
	usleep_range(1000, 1200);
	return 0;
}

void lens_iris_move(struct v4l2_subdev *sd, uint iris_tgt)
{
	/* iris_tgt range:0x0000~0x03FF */
	if (iris_tgt > 0x03ff)
		return;

	act_spi_write(sd, 0x00, iris_tgt);

	lens_set_pulse(VD_IS);

	act_dbg("%s target 0x%x\n", __func__, iris_tgt);
}

void lens_focus_move(struct v4l2_subdev *sd, bool dir, uint fc_step)
{
	unsigned short backup_data = 0;

	/* more than 255step */
	if (fc_step > 0x00FF)
		return;

	act_spi_read(sd, LENSDRV_FOCUS_STEPS_REG, &backup_data);
	backup_data &= 0xFE00;
	backup_data |= fc_step;
	if (dir == 1) {
		/* Add directiion bit (CCWCWAB) to registor:0x24 */
		backup_data |= 0x0100;
		act_spi_write(sd, LENSDRV_FOCUS_STEPS_REG, backup_data);
	} else {
		act_spi_write(sd, LENSDRV_FOCUS_STEPS_REG, backup_data);
	}

	lens_set_pulse(VD_FZ);

	act_dbg("%s dir %d, step 0x%x\n", __func__, dir, fc_step);
}

void lens_zoom_move(struct v4l2_subdev *sd, bool dir, uint zoom_step)
{
	unsigned short backup_data = 0;

	/* more than 255step */
	if (zoom_step > 0x00FF)
		return;

	act_spi_read(sd, LENSDRV_ZOOM_STEPS_REG, &backup_data);
	backup_data &= 0xFE00;
	backup_data |= zoom_step;
	if (dir == 1) {
		/* Add directiion bit (CCWCWAB) to registor:0x29 */
		backup_data |= 0x0100;
		act_spi_write(sd, LENSDRV_ZOOM_STEPS_REG, backup_data);
	} else {
		act_spi_write(sd, LENSDRV_ZOOM_STEPS_REG, backup_data);
	}

	lens_set_pulse(VD_FZ);

	act_dbg("%s dir %d, step 0x%x\n", __func__, dir, zoom_step);
}

static int act_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int act_reset(struct v4l2_subdev *sd, u32 val)
{
	return 0;
}

static int act_detect(struct v4l2_subdev *sd)
{
	unsigned short rd_val;

	act_spi_write(sd, 0x0E, 0x0C00);
	act_spi_read(sd, 0x0E, &rd_val);
	act_dbg("%s 0x%x!!!\n", __func__, rd_val);

	act_spi_write(sd, 0x0E, 0x0D00);
	act_spi_read(sd, 0x0E, &rd_val);
	act_dbg("%s 0x%x!!!\n", __func__, rd_val);

	return 0;
}

static int act_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;

	act_dbg("act_init\n");

	/* Make sure it is a target sensor */
	ret = act_detect(sd);
	if (ret) {
		act_err("chip found is not an target chip.\n");
		return ret;
	}

#if LENS_SUNNY /* Zoom use AB chanel */
	act_spi_write(sd, 0x20, 0x5c0a);
	act_spi_write(sd, 0x21, 0x0005);
	act_spi_write(sd, 0x22, 0x0003);
	act_spi_write(sd, 0x23, 0xC8C8);
	act_spi_write(sd, 0x24, 0x0400);
	act_spi_write(sd, 0x25, 0x0502);
#else
	act_spi_write(sd, 0x20, 0x5c0a);
	act_spi_write(sd, 0x21, 0x0000);
	act_spi_write(sd, 0x22, 0x1603);
	act_spi_write(sd, 0x23, 0xC8C8);
	act_spi_write(sd, 0x24, 0x0400);
	act_spi_write(sd, 0x25, 0x0160);
#endif

	act_spi_write(sd, 0x27, 0x1603);
	act_spi_write(sd, 0x28, 0xC8C8);
	act_spi_write(sd, 0x29, 0x0400);
	act_spi_write(sd, 0x2A, 0x0400);

#if LENS_SUNNY
	act_spi_write(sd, 0x00, 0x0000);  /* Set Iris Target */
	act_spi_write(sd, 0x01, 0x808A);
	act_spi_write(sd, 0x02, 0x66F0);
	act_spi_write(sd, 0x03, 0x0E10);
	act_spi_write(sd, 0x04, 0x7E20);
	act_spi_write(sd, 0x05, 0x0A04);
	act_spi_write(sd, 0x0A, 0x0000);
	act_spi_write(sd, 0x0B, 0x0400);
	act_spi_write(sd, 0x0E, 0x0C00);
#else
	act_spi_write(sd, 0x00, 0x0000);
	act_spi_write(sd, 0x01, 0x6600);
	act_spi_write(sd, 0x02, 0x5400);
	act_spi_write(sd, 0x03, 0x0E10);
	act_spi_write(sd, 0x04, 0x8437);
	act_spi_write(sd, 0x05, 0x0104);
	act_spi_write(sd, 0x0A, 0x0042);
	act_spi_write(sd, 0x0B, 0x0400);
	act_spi_write(sd, 0x0E, 0x0D00);
#endif

	lens_iris_move(sd, 0x02ff);

	lens_focus_move(sd, 0, 128);

	lens_zoom_move(sd, 0, 32);

	return 0;
}

static long act_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;

	switch (cmd) {
	default:
		return -EINVAL;
	}
	return ret;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops act_core_ops = {
	.reset = act_reset,
	.init = act_init,
	.s_power = act_power,
	.ioctl = act_ioctl,
};

static const struct v4l2_subdev_ops act_ops = {
	.core = &act_core_ops,
};

/* ----------------------------------------------------------------------- */
static int act_registered(struct v4l2_subdev *sd)
{
	return act_init(sd, 0);
}

static const struct v4l2_subdev_internal_ops act_internal_ops = {
	.registered = act_registered,
};

static int act_probe(struct spi_device *spi)
{
	struct v4l2_subdev *sd;

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (sd == NULL)
		return -ENOMEM;

	v4l2_spi_subdev_init(sd, spi, &act_ops);

	snprintf(sd->name, sizeof(sd->name), "%s", SUNXI_ACT_NAME);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &act_internal_ops;

	sd->entity.function = MEDIA_ENT_F_LENS;
	media_entity_pads_init(&sd->entity, 0, NULL);

	if (gpio_request(VD_IS, NULL)) {
		act_err("request gpio%d is failed\n", VD_IS);
		return -1;
	}
	if (gpio_request(VD_FZ, NULL)) {
		act_err("request gpio%d is failed\n", VD_FZ);
		return -1;
	}
	if (gpio_direction_output(VD_IS, 1)) {
		act_err("gpio%d set 0 err!", VD_IS);
		return -1;
	}
	if (gpio_direction_output(VD_FZ, 1)) {
		act_err("gpio%d set 0 err!", VD_FZ);
		return -1;
	}

	lens_set_pulse(VD_IS);
	lens_set_pulse(VD_FZ);

	return 0;
}
static int act_remove(struct spi_device *spi)
{
	struct v4l2_subdev *sd;

	gpio_free(VD_IS);
	gpio_free(VD_FZ);

	sd = spi_get_drvdata(spi);
	v4l2_device_unregister_subdev(sd);
	kfree(sd);
	return 0;
}

static const struct spi_device_id act_id[] = {
	{SUNXI_ACT_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(spi, act_id);

static struct spi_driver act_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SUNXI_ACT_NAME,
		   },
	.probe = act_probe,
	.remove = act_remove,
	.id_table = act_id,
};
static __init int init_sensor(void)
{
#if IS_ENABLED(CONFIG_SPI)
	return spi_register_driver(&act_driver);
#endif
}

static __exit void exit_sensor(void)
{
	spi_unregister_driver(&act_driver);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
