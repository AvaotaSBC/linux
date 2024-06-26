/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * for modules (sensor/actuator/flash) power supply helper.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *	Yang Feng <yangfeng@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "vin_log.h"
#include <linux/device.h>
#include <linux/module.h>

#include "../vin-video/vin_core.h"
#include "vin_os.h"
#include "vin_supply.h"
#include "../platform/platform_cfg.h"
#include "../vin-csi/sunxi_csi.h"
#include "../vin-cci/cci_helper.h"
#include "../modules/sensor/sensor_helper.h"
#include "../vin.h"

extern struct sensor_helper_dev *glb_sensor_helper[VIN_MAX_CSI];

/*
 * called by subdev in power on/off sequency
 */
struct modules_config *sd_to_modules(struct v4l2_subdev *sd)
{
	struct vin_md *vind = dev_get_drvdata(sd->v4l2_dev->dev);
	struct modules_config *module = NULL;
	int i, j;

	for (i = 0; i < VIN_MAX_DEV; i++) {
		module = &vind->modules[i];

		for (j = 0; j < MAX_DETECT_NUM; j++) {
			if (!strcmp(module->sensors.inst[j].cam_name, sd->name))
				return module;

			if ((sd == module->modules.sensor[j].sd) ||
			    (sd == module->modules.act[j].sd) ||
			    (sd == module->modules.flash.sd))
				return module;
		}
	}
	vin_err("%s cannot find the match modules\n", sd->name);
	return NULL;
}
EXPORT_SYMBOL_GPL(sd_to_modules);

#ifndef FPGA_VER
static int find_power_pmic(struct v4l2_subdev *sd, struct vin_power *power, enum pmic_channel pmic_ch)
{
	int i, j;
	struct modules_config *module = sd_to_modules(sd);

	if (!module) {
		vin_err("%s get module fail\n", __func__);
		return -1;
	}

	if (module->sensors.use_sensor_list == 1) {
		for (i = 0; i < MAX_DETECT_NUM; i++) {
			for (j = 0; j < VIN_MAX_CSI; j++) {
				if (!glb_sensor_helper[j])
					continue;
				if (!strcmp(glb_sensor_helper[j]->name, module->sensors.inst[i].cam_name)) {
					if (!glb_sensor_helper[j]->pmic[pmic_ch])
						return -1;
					power[pmic_ch].pmic = glb_sensor_helper[j]->pmic[pmic_ch];
					return 0;
				}
			}
		}
		vin_err("sensor defined in dts need be defined sensor list init file\n");
	} else {
		for (i = 0; i < VIN_MAX_CSI; i++) {
			if (glb_sensor_helper[i] && (!strcmp(glb_sensor_helper[i]->name, sd->name))) {
				if (!glb_sensor_helper[i]->pmic[pmic_ch])
					return -1;
				power[pmic_ch].pmic = glb_sensor_helper[i]->pmic[pmic_ch];
				return 0;
			}
		}
	}

	power[pmic_ch].pmic = NULL;
	vin_err("%s cannot find the match sensor_helper\n", sd->name);
	return -1;
}
#endif

/*
 *enable/disable pmic channel
 */
int vin_set_pmu_channel(struct v4l2_subdev *sd, enum pmic_channel pmic_ch,
			enum on_off on_off)
{
	int ret = 0;

#ifndef FPGA_VER
	struct modules_config *modules = sd_to_modules(sd);
	static int def_vol[MAX_POW_NUM] = {3300000, 3300000, 1800000,
					3300000, 3300000, 3300000};
	struct vin_power *power = NULL;

	if (modules == NULL)
		return -1;

	power = &modules->sensors.power[0];
	if (on_off == OFF) {
		if (power[pmic_ch].pmic == NULL)
			return 0;
		ret = regulator_disable(power[pmic_ch].pmic);
		ret = regulator_set_voltage(power[pmic_ch].pmic,
					  0, def_vol[pmic_ch]);
		power[pmic_ch].pmic = NULL;
		vin_log(VIN_LOG_POWER, "regulator_is already disabled\n");
	} else {
		ret = find_power_pmic(sd, power, pmic_ch);
		if (ret)
			return ret;

		ret = regulator_set_voltage(power[pmic_ch].pmic,
					  power[pmic_ch].power_vol,
					  def_vol[pmic_ch]);
		vin_log(VIN_LOG_POWER, "set regulator %s = %d,return %x\n",
			power[pmic_ch].power_str, power[pmic_ch].power_vol, ret);
		ret = regulator_enable(power[pmic_ch].pmic);
	}
#endif
	return ret;
}
EXPORT_SYMBOL_GPL(vin_set_pmu_channel);

/*
 *set pmic vol, use in camera driver power on.
 */
int vin_set_pmu_vol(struct v4l2_subdev *sd, enum pmic_channel pmic_ch,
			unsigned int vol)
{
	int ret = 0;

#ifndef FPGA_VER
	struct modules_config *modules = sd_to_modules(sd);
	struct vin_power *power = NULL;
	char *pmic_name[MAX_POW_NUM] = {"IOVDD", "AVDD", "DVDD", "AFVDD", "FLVDD", "CAMERAVDD"};

	if (modules == NULL)
		return -1;

	power = &modules->sensors.power[0];
	ret = find_power_pmic(sd, power, pmic_ch);
	if (ret)
		return ret;

	power[pmic_ch].power_vol = vol;
	vin_print("set regulator %s = %d\n",
			pmic_name[pmic_ch], power[pmic_ch].power_vol);

#endif
	return ret;
}
EXPORT_SYMBOL_GPL(vin_set_pmu_vol);

/*
 *enable/disable master clock
 */
int vin_set_mclk(struct v4l2_subdev *sd, enum on_off on_off)
{
	struct vin_md *vind = dev_get_drvdata(sd->v4l2_dev->dev);
	struct modules_config *modules = sd_to_modules(sd);
	struct vin_mclk_info *mclk = NULL;
	__maybe_unused struct vin_power *power = NULL;
	__maybe_unused unsigned int mclk_mode;
	char pin_name[20] = "";
	int mclk_id = 0;

	if (modules == NULL)
		return -1;

	if (modules->sensors.mclk_id == -1)
		mclk_id = modules->sensors.csi_sel;
	else
		mclk_id = modules->sensors.mclk_id;
	if (mclk_id < 0) {
		vin_err("get mclk id failed\n");
		return -1;
	}

	mclk = &vind->mclk[mclk_id];

	switch (on_off) {
	case ON:
		csi_cci_init_helper(modules->sensors.sensor_bus_sel);
		sprintf(pin_name, "mclk%d-default", mclk_id);
		break;
	case OFF:
		csi_cci_exit_helper(modules->sensors.sensor_bus_sel);
		sprintf(pin_name, "mclk%d-sleep", mclk_id);
		break;
	default:
		return -1;
	}
#ifndef FPGA_VER

	if (on_off && mclk->use_count++ > 0)
		return 0;
	else if (!on_off && (mclk->use_count == 0 || --mclk->use_count > 0))
		return 0;

	switch (on_off) {
	case ON:
		vin_log(VIN_LOG_POWER, "sensor mclk on, use_count %d!\n", mclk->use_count);
		if (mclk->mclk) {
			if (clk_prepare_enable(mclk->mclk)) {
				vin_err("csi master clock enable error\n");
				return -1;
			}
		} else {
			vin_err("csi master%d clock is null\n", mclk_id);
			return -1;
		}
		break;
	case OFF:
		vin_log(VIN_LOG_POWER, "sensor mclk off, use_count %d!\n", mclk->use_count);
		if (mclk->mclk) {
			clk_disable_unprepare(mclk->mclk);
		} else {
			vin_err("csi master%d clock is null\n", mclk_id);
			return -1;
		}
		break;
	default:
		return -1;
	}
	mutex_lock(&vind->mclk_pin_lock);
	mclk->pin = devm_pinctrl_get_select(&vind->pdev->dev, pin_name);
	if (IS_ERR_OR_NULL(mclk->pin)) {
		mutex_unlock(&vind->mclk_pin_lock);
		vin_err("mclk%d request pin handle failed!\n", mclk_id);
		return -EINVAL;
	}
	mutex_unlock(&vind->mclk_pin_lock);
	if (on_off) {
		power = &modules->sensors.power[0];
		if (power[IOVDD].power_vol && (power[IOVDD].power_vol <= 1800000))
			mclk_mode = MCLK_POWER_VOLTAGE_1800;
		else
			mclk_mode = MCLK_POWER_VOLTAGE_3300;
		vin_log(VIN_LOG_POWER, "IOVDD power vol is %d, mclk mode is %d\n", power[IOVDD].power_vol, mclk_mode);
	}
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(vin_set_mclk);

/*
 *set frequency of master clock
 */
int vin_set_mclk_freq(struct v4l2_subdev *sd, unsigned long freq)
{
#ifndef FPGA_VER
	struct vin_md *vind = dev_get_drvdata(sd->v4l2_dev->dev);
	struct modules_config *modules = sd_to_modules(sd);
	struct clk *mclk_src = NULL;
	int mclk_id = 0;

	if (modules == NULL)
		return -1;

	if (modules->sensors.mclk_id == -1)
		mclk_id = modules->sensors.csi_sel;
	else
		mclk_id = modules->sensors.mclk_id;
	if (mclk_id < 0) {
		vin_err("get mclk id failed\n");
		return -1;
	}

	if (freq == 24000000 || freq == 12000000 || freq == 6000000) {
		if (vind->mclk[mclk_id].clk_24m) {
			mclk_src = vind->mclk[mclk_id].clk_24m;
		} else {
			vin_err("csi master clock 24M source is null\n");
			return -1;
		}
	} else {
		if (vind->mclk[mclk_id].clk_pll) {
			mclk_src = vind->mclk[mclk_id].clk_pll;
		} else {
			vin_err("csi master clock pll source is null\n");
			return -1;
		}
	}

	if (vind->mclk[mclk_id].mclk) {
		if (clk_set_parent(vind->mclk[mclk_id].mclk, mclk_src)) {
			vin_err("set mclk%d source failed!\n", mclk_id);
			return -1;
		}
		if (clk_set_rate(vind->mclk[mclk_id].mclk, freq)) {
			vin_err("set csi master%d clock error\n", mclk_id);
			return -1;
		}
		vin_log(VIN_LOG_POWER, "mclk%d set rate %ld, get rate %ld\n", mclk_id,
			freq, clk_get_rate(vind->mclk[mclk_id].mclk));
	} else {
		vin_err("csi master clock is null\n");
		return -1;
	}
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(vin_set_mclk_freq);

int vin_set_sync_mclk(struct v4l2_subdev *sd, int id, unsigned long freq, enum on_off on_off)
{
	struct vin_md *vind = dev_get_drvdata(sd->v4l2_dev->dev);
	struct modules_config *modules = sd_to_modules(sd);
	struct vin_mclk_info *mclk = NULL;
	struct clk *mclk_src = NULL;

	if (modules == NULL)
		return -1;

	if (id < 0) {
		vin_err("get mclk id failed\n");
		return -1;
	}

	mclk = &vind->mclk[id];

	if (on_off && mclk->use_count++ > 0)
		return 0;
	else if (!on_off && (mclk->use_count == 0 || --mclk->use_count > 0))
		return 0;

	switch (on_off) {
	case ON:
		vin_log(VIN_LOG_POWER, "sensor mclk on, use_count %d!\n", mclk->use_count);
		if (freq == 24000000 || freq == 12000000 || freq == 6000000) {
			if (mclk->clk_24m) {
				mclk_src = mclk->clk_24m;
			} else {
				vin_err("mclk%d 24M source is null\n", id);
				return -1;
			}
		} else {
			if (mclk->clk_pll) {
				mclk_src =  mclk->clk_pll;
			} else {
				vin_err("mclk%d pll source is null\n", id);
				return -1;
			}
		}

		if (mclk->mclk) {
			if (clk_set_parent(mclk->mclk, mclk_src)) {
				vin_err("set mclk%d source failed!\n", id);
				return -1;
			}
			if (clk_set_rate(mclk->mclk, freq)) {
				vin_err("set mclk%d error\n", id);
				return -1;
			}
			vin_log(VIN_LOG_POWER, "mclk%d set rate %ld, get rate %ld\n", id,
				freq, clk_get_rate(vind->mclk[id].mclk));
			if (clk_prepare_enable(mclk->mclk)) {
				vin_err("mclk%d enable error\n", id);
				return -1;
			}
		} else {
			vin_err("mclk%d is null\n", id);
			return -1;
		}
		break;
	case OFF:
		vin_log(VIN_LOG_POWER, "sensor mclk off, use_count %d!\n", mclk->use_count);
		if (mclk->mclk) {
			clk_disable_unprepare(mclk->mclk);
		} else {
			vin_err("mclk%d is null\n", id);
			return -1;
		}
		break;
	default:
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(vin_set_sync_mclk);

/*
 *set the gpio io status
 */
int vin_gpio_write(struct v4l2_subdev *sd, enum gpio_type gpio_id,
		   unsigned int out_value)
{
#ifndef FPGA_VER
	int force_value_flag = 1;
	struct modules_config *modules = sd_to_modules(sd);
	int gpio;

	if (modules == NULL)
		return -1;

	gpio = modules->sensors.gpio[gpio_id];

	if (gpio < 0)
		return -1;
	return os_gpio_write(gpio, out_value, force_value_flag);
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(vin_gpio_write);

/*
 *set the gpio io status
 */
int vin_gpio_set_status(struct v4l2_subdev *sd, enum gpio_type gpio_id,
			unsigned int status)
{
#ifndef FPGA_VER
	struct modules_config *modules = sd_to_modules(sd);
	int gc;

	if (modules == NULL)
		return -1;

	gc = modules->sensors.gpio[gpio_id];

	if (gc < 0)
		return 0;

	vin_log(VIN_LOG_POWER, "[%s] pin%d, set status %d\n", __func__, gc, status);

	if (status == 1)
		gpio_direction_output(gc, 0);
	else
		gpio_direction_input(gc);

#endif
	return 0;
}
EXPORT_SYMBOL_GPL(vin_gpio_set_status);

MODULE_AUTHOR("raymonxiu");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Video front end subdev for sunxi");
