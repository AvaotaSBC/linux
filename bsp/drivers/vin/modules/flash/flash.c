/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * flash subdev driver module
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "../../utility/vin_log.h"
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <media/sunxi_camera_v2.h>

#include "../../platform/platform_cfg.h"
#include "../../vin-video/vin_core.h"

#include "flash.h"

#if IS_ENABLED(CONFIG_FLASH_MODULE)
static LIST_HEAD(flash_drv_list);

#define FLASH_MODULE_NAME "vin_flash"

#define FLASH_EN_POL 1
#define FLASH_MODE_POL 1

static int flash_power_flag;

int io_set_flash_ctrl(struct v4l2_subdev *sd, enum sunxi_flash_ctrl ctrl)
{
	int ret = 0;
	unsigned int flash_en, flash_dis, flash_mode, torch_mode;
	struct flash_dev_info *fls_info = NULL;
	struct flash_dev *flash = (sd == NULL) ? NULL : v4l2_get_subdevdata(sd);

	if (!flash)
		return 0;

	fls_info = &flash->fl_info;

	flash_en = (fls_info->en_pol != 0) ? 1 : 0;
	flash_dis = !flash_en;
	flash_mode = (fls_info->fl_mode_pol != 0) ? 1 : 0;
	torch_mode = !flash_mode;

	if (fls_info->flash_driver_ic == FLASH_RELATING) {
		switch (ctrl) {
		case SW_CTRL_FLASH_OFF:
			vin_log(VIN_LOG_FLASH, "FLASH_RELATING SW_CTRL_FLASH_OFF\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			vin_gpio_set_status(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_EN, flash_dis);
			ret |= vin_gpio_write(sd, FLASH_MODE, torch_mode);
			break;
		case SW_CTRL_FLASH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_RELATING SW_CTRL_FLASH_ON\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			vin_gpio_set_status(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_MODE, flash_mode);
			ret |= vin_gpio_write(sd, FLASH_EN, flash_en);
			break;
		case SW_CTRL_TORCH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_RELATING SW_CTRL_TORCH_ON\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			vin_gpio_set_status(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_MODE, torch_mode);
			ret |= vin_gpio_write(sd, FLASH_EN, flash_en);
			break;
		default:
			return -EINVAL;
		}
	} else if (fls_info->flash_driver_ic == FLASH_EN_INDEPEND) {
		switch (ctrl) {
		case SW_CTRL_FLASH_OFF:
			vin_log(VIN_LOG_FLASH, "FLASH_EN_INDEPEND SW_CTRL_FLASH_OFF\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			vin_gpio_set_status(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_EN, 0);
			ret |= vin_gpio_write(sd, FLASH_MODE, 0);
			break;
		case SW_CTRL_FLASH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_EN_INDEPEND SW_CTRL_FLASH_ON\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			vin_gpio_set_status(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_EN, 1);
			break;
		case SW_CTRL_TORCH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_EN_INDEPEND SW_CTRL_TORCH_ON\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			vin_gpio_set_status(sd, FLASH_MODE, 1);
			ret |= vin_gpio_write(sd, FLASH_MODE, 0);
			ret |= vin_gpio_write(sd, FLASH_EN, 1);
			break;
		default:
			return -EINVAL;
		}
	} else if (fls_info->flash_driver_ic == FLASH_VIRTUAL) {
	switch (ctrl) {
		case SW_CTRL_FLASH_OFF:
			vin_log(VIN_LOG_FLASH, "FLASH_VIRTUAL SW_CTRL_FLASH_OFF\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			/* vin_gpio_set_status(sd, FLASH_MODE, 1); */
			ret |= vin_gpio_write(sd, FLASH_EN, 0);
			/* ret |= vin_gpio_write(sd, FLASH_MODE, 0); */
			break;
		case SW_CTRL_FLASH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_VIRTUAL SW_CTRL_FLASH_ON\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			/* vin_gpio_set_status(sd, FLASH_MODE, 1); */
			ret |= vin_gpio_write(sd, FLASH_EN, 1);
			/* ret |= vin_gpio_write(sd, FLASH_MODE, 0); */
			break;
		case SW_CTRL_TORCH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_VIRTUAL SW_CTRL_TORCH_ON\n");
			vin_gpio_set_status(sd, FLASH_EN, 1);
			/* vin_gpio_set_status(sd, FLASH_MODE, 1); */
			ret |= vin_gpio_write(sd, FLASH_EN, 1);
			/* ret |= vin_gpio_write(sd, FLASH_MODE, 0); */
			break;
		default:
			return -EINVAL;
	}
	} else {
		switch (ctrl) {
		case SW_CTRL_FLASH_OFF:
			vin_log(VIN_LOG_FLASH, "FLASH_POWER SW_CTRL_FLASH_OFF\n");
			if (flash_power_flag == 1) {
				vin_set_pmu_channel(sd, FLVDD, OFF);
				flash_power_flag--;
			}
			break;
		case SW_CTRL_FLASH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_POWER SW_CTRL_FLASH_ON\n");
			if (flash_power_flag == 0) {
				vin_set_pmu_channel(sd, FLVDD, ON);
				flash_power_flag++;
			}
			break;
		case SW_CTRL_TORCH_ON:
			vin_log(VIN_LOG_FLASH, "FLASH_POWER SW_CTRL_TORCH_ON\n");
			if (flash_power_flag == 0) {
				vin_set_pmu_channel(sd, FLVDD, ON);
				flash_power_flag++;
			}
			break;
		default:
			return -EINVAL;
		}
	}
	if (ret != 0) {
		vin_log(VIN_LOG_FLASH, "flash set ctrl fail, force shut off\n");
		ret |= vin_gpio_write(sd, FLASH_EN, flash_dis);
		ret |= vin_gpio_write(sd, FLASH_MODE, torch_mode);
	}
	return ret;
}

int sunxi_flash_check_to_start(struct v4l2_subdev *sd,
			       enum sunxi_flash_ctrl ctrl)
{
	struct modules_config *modules = sd ? sd_to_modules(sd) : NULL;
	struct flash_dev *flash = sd ? v4l2_get_subdevdata(sd) : NULL;
	struct v4l2_subdev *sensor = NULL;
	unsigned int flag = 0, to_flash;

	if (!flash)
		return 0;

	if (flash->fl_info.flash_mode == V4L2_FLASH_LED_MODE_FLASH) {
		to_flash = 1;
	} else if (flash->fl_info.flash_mode == V4L2_FLASH_LED_MODE_AUTO) {
		if (!modules)
			return 0;
		sensor = modules->modules.sensor[modules->sensors.valid_idx].sd;
		v4l2_subdev_call(sensor, core, ioctl, GET_FLASH_FLAG, &flag);
		if (flag)
			to_flash = 1;
		else
			to_flash = 0;
	} else {
		to_flash = 0;
	}

	if (to_flash)
		io_set_flash_ctrl(sd, ctrl);

	return 0;
}

int sunxi_flash_stop(struct v4l2_subdev *sd)
{
	struct flash_dev *flash = (sd == NULL) ? NULL : v4l2_get_subdevdata(sd);

	if (!flash)
		return 0;

	if (flash->fl_info.flash_mode != V4L2_FLASH_LED_MODE_NONE)
		io_set_flash_ctrl(sd, SW_CTRL_FLASH_OFF);
	return 0;
}

static int config_flash_mode(struct v4l2_subdev *sd,
			     enum v4l2_flash_led_mode mode,
			     struct flash_dev_info *fls_info)
{
	if (fls_info == NULL) {
		vin_err("camera flash not support!\n");
		return -1;
	}
	if ((fls_info->light_src != 0x01) && (fls_info->light_src != 0x02)
	    && (fls_info->light_src != 0x10)) {
		vin_err("unsupported light source, force LEDx1\n");
		fls_info->light_src = 0x01;
	}
	fls_info->flash_mode = mode;
	if (mode == V4L2_FLASH_LED_MODE_TORCH)
		io_set_flash_ctrl(sd, SW_CTRL_TORCH_ON);
	else if (mode == V4L2_FLASH_LED_MODE_NONE)
		io_set_flash_ctrl(sd, SW_CTRL_FLASH_OFF);

	return 0;
}

static int sunxi_flash_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct flash_dev *flash =
			container_of(ctrl->handler, struct flash_dev, handler);

	if (!flash)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		ctrl->val = flash->fl_info.flash_mode;
		return 0;
	default:
		break;
	}
	return -EINVAL;
}

static int sunxi_flash_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct flash_dev *flash =
			container_of(ctrl->handler, struct flash_dev, handler);
	struct v4l2_subdev *sd = &flash->subdev;


	if (!flash)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		return config_flash_mode(sd, ctrl->val, &flash->fl_info);
	case V4L2_CID_FLASH_LED_MODE_V1:
		return 0;
	default:
		break;
	}
	return -EINVAL;
}
static const struct v4l2_ctrl_ops sunxi_flash_ctrl_ops = {
	.g_volatile_ctrl = sunxi_flash_g_ctrl,
	.s_ctrl = sunxi_flash_s_ctrl,
};

static const struct v4l2_ctrl_config custom_ctrls[] = {
	{
		.ops = &sunxi_flash_ctrl_ops,
		.id = V4L2_CID_FLASH_LED_MODE_V1,
		.name = "VIN Flash ctrl1",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = 2,
		.def = 0,
		.menu_skip_mask = 0x0,
		.qmenu = flash_led_mode_v1,
		.flags = 0,
		.step = 0,
	},
};

static int sunxi_flash_controls_init(struct v4l2_subdev *sd)
{
	struct flash_dev *flash = container_of(sd, struct flash_dev, subdev);
	struct v4l2_ctrl_handler *handler = &flash->handler;
	int i, ret = 0;

	v4l2_ctrl_handler_init(handler, 1 + ARRAY_SIZE(custom_ctrls));
	v4l2_ctrl_new_std_menu(handler, &sunxi_flash_ctrl_ops,
					V4L2_CID_FLASH_LED_MODE, V4L2_FLASH_LED_MODE_RED_EYE,
					0, V4L2_FLASH_LED_MODE_NONE);

	for (i = 0; i < ARRAY_SIZE(custom_ctrls); i++)
		v4l2_ctrl_new_custom(handler, &custom_ctrls[i], NULL);

	if (handler->error) {
		ret =  handler->error;
		v4l2_ctrl_handler_free(handler);
	}

	sd->ctrl_handler = handler;

	return ret;
}
static int sunxi_flash_subdev_init(struct flash_dev *flash)
{
	struct v4l2_subdev *sd = &flash->subdev;

	snprintf(sd->name, sizeof(sd->name), "sunxi_flash.%u", flash->id);
	sd->entity.name = sd->name;

	sunxi_flash_controls_init(sd);
	flash->fl_info.dev_if = 0;
	flash->fl_info.en_pol = FLASH_EN_POL;
	flash->fl_info.fl_mode_pol = FLASH_MODE_POL;
	flash->fl_info.light_src = 0x01;
	flash->fl_info.flash_intensity = 400;
	flash->fl_info.flash_level = 0x01;
	flash->fl_info.torch_intensity = 200;
	flash->fl_info.torch_level = 0x01;
	flash->fl_info.timeout_counter = 300 * 1000;

	config_flash_mode(sd, V4L2_FLASH_LED_MODE_NONE, &flash->fl_info);

	v4l2_set_subdevdata(sd, flash);

	media_entity_pads_init(&sd->entity, 0, NULL);
	sd->entity.function = MEDIA_ENT_F_FLASH;

	return 0;
}

static int flash_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct flash_dev *flash = NULL;
	char property_name[32] = { 0 };
	int ret = 0;

	if (np == NULL) {
		vin_err("flash failed to get of node\n");
		return -ENODEV;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash) {
		ret = -ENOMEM;
		vin_err("sunxi flash kzalloc failed!\n");
		goto ekzalloc;
	}

	of_property_read_u32(np, "device_id", &pdev->id);
	if (pdev->id < 0) {
		vin_err("flash failed to get device id\n");
		ret = -EINVAL;
		goto freedev;
	}
	sprintf(property_name, "flash%d_type", pdev->id);
	ret = of_property_read_u32(np, property_name,
				 &flash->fl_info.flash_driver_ic);
	if (ret) {
		flash->fl_info.flash_driver_ic = 0;
		vin_warn("fetch %s from device_tree failed\n", property_name);
		return -EINVAL;
	}

	flash->id = pdev->id;
	flash->pdev = pdev;
	list_add_tail(&flash->flash_list, &flash_drv_list);

	sunxi_flash_subdev_init(flash);
	platform_set_drvdata(pdev, flash);
	vin_log(VIN_LOG_FLASH, "flash%d probe end!\n", flash->id);

	return 0;
freedev:
	kfree(flash);
ekzalloc:
	vin_err("flash probe err!\n");
	return ret;
}

static int flash_remove(struct platform_device *pdev)
{
	struct flash_dev *flash = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &flash->subdev;

	platform_set_drvdata(pdev, NULL);
	v4l2_device_unregister_subdev(sd);
	v4l2_set_subdevdata(sd, NULL);
	list_del(&flash->flash_list);

	kfree(flash);
	return 0;
}

static const struct of_device_id sunxi_flash_match[] = {
	{.compatible = "allwinner,sunxi-flash",},
	{},
};

static struct platform_driver flash_platform_driver = {
	.probe = flash_probe,
	.remove = flash_remove,
	.driver = {
		   .name = FLASH_MODULE_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = sunxi_flash_match,
		   },
};
struct v4l2_subdev *sunxi_flash_get_subdev(int id)
{
	struct flash_dev *flash;

	list_for_each_entry(flash, &flash_drv_list, flash_list) {
		if (flash->id == id) {
			flash->use_cnt++;
			return &flash->subdev;
		}
	}
	return NULL;
}

int sunxi_flash_platform_register(void)
{
	return platform_driver_register(&flash_platform_driver);
}

void sunxi_flash_platform_unregister(void)
{
	platform_driver_unregister(&flash_platform_driver);
	vin_log(VIN_LOG_FLASH, "flash_exit end\n");
}
#else
int io_set_flash_ctrl(struct v4l2_subdev *sd, enum sunxi_flash_ctrl ctrl)
{
	return 0;
}

int sunxi_flash_check_to_start(struct v4l2_subdev *sd,
			       enum sunxi_flash_ctrl ctrl)
{
	return 0;
}

int sunxi_flash_stop(struct v4l2_subdev *sd)
{
	return 0;
}

struct v4l2_subdev *sunxi_flash_get_subdev(int id)
{
	return NULL;
}

int sunxi_flash_platform_register(void)
{
	return 0;
}

void sunxi_flash_platform_unregister(void)
{
}
#endif
