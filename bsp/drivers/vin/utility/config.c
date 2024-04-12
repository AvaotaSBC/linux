/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * config.c for device tree and sensor list parser.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 * Yang Feng <yangfeng@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "vin_log.h"
#include "config.h"
#include "../platform/platform_cfg.h"
#include "../modules/sensor-list/sensor_list.h"

#ifndef FPGA_VER
static int get_value_int(struct device_node *np, const char *name,
			  u32 *value)
{
	int ret;

	ret = of_property_read_u32(np, name, value);
	if (ret) {
		*value = 0;
		vin_log(VIN_LOG_CONFIG, "fetch %s from device_tree failed\n", name);
		return -EINVAL;
	}
	vin_log(VIN_LOG_CONFIG, "%s = %x\n", name, *value);
	return 0;
}
static int get_value_string(struct device_node *np, const char *name,
			    char *string)
{
	int ret;
	const char *const_str;

	ret = of_property_read_string(np, name, &const_str);
	if (ret) {
		strcpy(string, "");
		vin_log(VIN_LOG_CONFIG, "fetch %s from device_tree failed\n", name);
		return -EINVAL;
	}
	strcpy(string, const_str);
	vin_log(VIN_LOG_CONFIG, "%s = %s\n", name, string);
	return 0;
}

static int get_gpio_info(struct device_node *np, const char *name,
			 int *gpio)
{
	int gnum;
	enum of_gpio_flags gc;

	gnum = of_get_named_gpio_flags(np, name, 0, &gc);
	if (!gpio_is_valid(gnum)) {
		vin_log(VIN_LOG_CONFIG, "fetch %s from device_tree failed\n", name);
		return -ENODEV;
	}
	*gpio = gnum;
	vin_log(VIN_LOG_CONFIG, "fetch %s gpio = %d\n", name, *gpio);

	return 0;
}

static int get_mname(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_string(np, name, sc->inst[0].cam_name);
}
static int get_twi_addr(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->inst[0].cam_addr);
}

static int get_twi_cci_spi(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->sensor_bus_type);
}

static int get_twi_id(struct device_node *np, const char *name,
		      struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->sensor_bus_sel);
}
static int get_mclk_id(struct device_node *np, const char *name,
		      struct sensor_list *sc)
{
	if (get_value_int(np, name, &sc->mclk_id))
		sc->mclk_id = -1;
	return 0;
}
static int get_pos(struct device_node *np, const char *name,
		   struct sensor_list *sc)
{
	return get_value_string(np, name, sc->sensor_pos);
}
static int get_isp_used(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->inst[0].is_isp_used);
}
static int get_fmt(struct device_node *np, const char *name,
		   struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->inst[0].is_bayer_raw);
}
static int get_vflip(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->inst[0].vflip);
}
static int get_hflip(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->inst[0].hflip);
}
static int get_cameravdd(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_string(np, name, sc->power[CAMERAVDD].power_str);
}
static int get_cameravdd_vol(struct device_node *np, const char *name,
			 struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->power[CAMERAVDD].power_vol);
}
static int get_iovdd(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_string(np, name, sc->power[IOVDD].power_str);
}
static int get_iovdd_vol(struct device_node *np, const char *name,
			 struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->power[IOVDD].power_vol);
}
static int get_avdd(struct device_node *np, const char *name,
		    struct sensor_list *sc)
{
	return get_value_string(np, name, sc->power[AVDD].power_str);
}
static int get_avdd_vol(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->power[AVDD].power_vol);
}
static int get_dvdd(struct device_node *np, const char *name,
		    struct sensor_list *sc)
{
	return get_value_string(np, name, sc->power[DVDD].power_str);
}
static int get_dvdd_vol(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->power[DVDD].power_vol);
}

#if IS_ENABLED(CONFIG_ACTUATOR_MODULE)
static int get_afvdd(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_string(np, name, sc->power[AFVDD].power_str);
}
static int get_afvdd_vol(struct device_node *np, const char *name,
			 struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->power[AFVDD].power_vol);
}
#endif

static int get_power_en(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[POWER_EN]);
}
static int get_reset(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[RESET]);
}
static int get_pwdn(struct device_node *np, const char *name,
		    struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[PWDN]);
}
static int get_sm_hs(struct device_node *np, const char *name,
		    struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[SM_HS]);
}
static int get_sm_vs(struct device_node *np, const char *name,
		    struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[SM_VS]);
}
#endif

#if IS_ENABLED(CONFIG_FLASH_MODULE)
static int get_flash_en(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[FLASH_EN]);
}
static int get_flash_mode(struct device_node *np, const char *name,
			  struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[FLASH_MODE]);
}
static int get_flvdd(struct device_node *np, const char *name,
		     struct sensor_list *sc)
{
	return get_value_string(np, name, sc->power[FLVDD].power_str);
}
static int get_flvdd_vol(struct device_node *np, const char *name,
			 struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->power[FLVDD].power_vol);
}
#endif

#if IS_ENABLED(CONFIG_ACTUATOR_MODULE)
static int get_act_bus_type(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->act_bus_type);
}

static int get_act_bus_sel(struct device_node *np, const char *name,
		      struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->act_bus_sel);
}

static int get_act_separate(struct device_node *np, const char *name,
		      struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->act_separate);
}

static int get_af_pwdn(struct device_node *np, const char *name,
		       struct sensor_list *sc)
{
	return get_gpio_info(np, name, &sc->gpio[AF_PWDN]);
}
static int get_act_name(struct device_node *np, const char *name,
			struct sensor_list *sc)
{
	return get_value_string(np, name, sc->inst[0].act_name);
}
static int get_act_slave(struct device_node *np, const char *name,
			 struct sensor_list *sc)
{
	return get_value_int(np, name, &sc->inst[0].act_addr);
}
#endif

struct FetchFunArr {
	char *sub;
	int flag;
	int (*fun)(struct device_node *, const char *,
		    struct sensor_list *);
};

#ifndef FPGA_VER
static struct FetchFunArr fetch_camera[] = {
	{"mname", 0, get_mname,},
	{"twi_addr", 0, get_twi_addr,},
	{"twi_cci_spi", 1, get_twi_cci_spi,},
	{"twi_cci_id", 1, get_twi_id,},
	{"mclk_id", 1, get_mclk_id,},
	{"pos", 1, get_pos,},
	{"isp_used", 1, get_isp_used,},
	{"fmt", 1, get_fmt,},
	{"vflip", 1, get_vflip,},
	{"hflip", 1, get_hflip,},
	{"cameravdd", 1, get_cameravdd,},
	{"cameravdd_vol", 1, get_cameravdd_vol},
	{"iovdd", 1, get_iovdd,},
	{"iovdd_vol", 1, get_iovdd_vol},
	{"avdd", 1, get_avdd,},
	{"avdd_vol", 1, get_avdd_vol,},
	{"dvdd", 1, get_dvdd,},
	{"dvdd_vol", 1, get_dvdd_vol,},
	{"power_en", 1, get_power_en,},
	{"reset", 1, get_reset,},
	{"pwdn", 1, get_pwdn,},
	{"sm_hs", 1, get_sm_hs,},
	{"sm_vs", 1, get_sm_vs,},
};
#endif

#if IS_ENABLED(CONFIG_FLASH_MODULE)
static struct FetchFunArr fetch_flash[] = {
	{"en", 1, get_flash_en,},
	{"mode", 1, get_flash_mode,},
	{"flvdd", 1, get_flvdd,},
	{"flvdd_vol", 1, get_flvdd_vol,},
};
#endif

#if IS_ENABLED(CONFIG_ACTUATOR_MODULE)
static struct FetchFunArr fetch_actuator[] = {
	{"name", 0, get_act_name,},
	{"slave", 0, get_act_slave,},
	{"separate", 1, get_act_separate,},
	{"twi_cci_spi", 1, get_act_bus_type,},
	{"twi_cci_id", 1, get_act_bus_sel,},
	{"af_pwdn", 1, get_af_pwdn,},
	{"afvdd", 1, get_afvdd,},
	{"afvdd_vol", 1, get_afvdd_vol,},
};
#endif

int parse_modules_from_device_tree(struct vin_md *vind)
{
#ifdef FPGA_VER
	unsigned int i, j;
	struct modules_config *module;
	struct sensor_list *sensors;
	struct sensor_instance *inst;
	unsigned int sensor_uses = 1; /* 1/2 mean use one/two sensor */
	struct sensor_list sensors_def[2] = {
		{
		.used = 1,
		.csi_sel = 0,
		.device_sel = 0,
		.sensor_bus_sel = 0,
		.power_set = 1,
		.detect_num = 1,
		.sensor_pos = "rear",
		.power = {
			  [IOVDD] = {NULL, 2800000, ""},
			  [AVDD] = {NULL, 2800000, ""},
			  [DVDD] = {NULL, 1500000, ""},
			  [AFVDD] = {NULL, 2800000, ""},
			  [FLVDD] = {NULL, 3300000, ""},
			  },
		.gpio = {
			 [RESET] = {GPIOE(14), 1, 0, 1, 0,},
			 [PWDN] = {GPIOE(15), 1, 0, 1, 0,},
			 [POWER_EN] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 [FLASH_EN] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 [FLASH_MODE] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 [AF_PWDN] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 },
		.inst = {
				[0] = {
				       .cam_name = "ar0238",
				       .cam_addr = 0x20,
				       .cam_type = 0,
				       .is_isp_used = 1,
				       .is_bayer_raw = 1,
				       .vflip = 0,
				       .hflip = 0,
				       .act_name = "ad5820_act",
				       .act_addr = 0x18,
				       .isp_cfg_name = "",
				       },
			},
		}, {
		.used = 1,
		.csi_sel = 1,
		.device_sel = 1,
		.sensor_bus_sel = 1,
		.power_set = 1,
		.detect_num = 1,
		.sensor_pos = "front",
		.power = {
			  [IOVDD] = {NULL, 2800000, ""},
			  [AVDD] = {NULL, 2800000, ""},
			  [DVDD] = {NULL, 1500000, ""},
			  [AFVDD] = {NULL, 2800000, ""},
			  [FLVDD] = {NULL, 3300000, ""},
			  },
		.gpio = {
			 [RESET] = {GPIOE(14), 1, 0, 1, 0,},
			 [PWDN] = {GPIOE(15), 1, 0, 1, 0,},
			 [POWER_EN] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 [FLASH_EN] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 [FLASH_MODE] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 [AF_PWDN] = {GPIO_INDEX_INVALID, 0, 0, 0, 0,},
			 },
		.inst = {
				[0] = {
				       .cam_name = "ar0238_2",
				       .cam_addr = 0x20,
				       .cam_type = 0,
				       .is_isp_used = 1,
				       .is_bayer_raw = 1,
				       .vflip = 0,
				       .hflip = 0,
				       .act_name = "ad5820_act",
				       .act_addr = 0x18,
				       .isp_cfg_name = "",
				       },
			},
		}
	};

	for (i = 0; i < sensor_uses; i++) {
		module = &vind->modules[i];
		sensors = &module->sensors;
		inst = &sensors->inst[0];

		sensors->use_sensor_list = 0;
		sensors->sensor_bus_sel = sensors_def[i].sensor_bus_sel;
		/* when insmod without parm */
		if (inst->cam_addr == 0xff) {
			strcpy(inst->cam_name, sensors_def[i].inst[0].cam_name);
			strcpy(inst->isp_cfg_name, sensors_def[i].inst[0].cam_name);
			inst->cam_addr = sensors_def[i].inst[0].cam_addr;
		}
		inst->is_isp_used = sensors_def[i].inst[0].is_isp_used;
		inst->is_bayer_raw = sensors_def[i].inst[0].is_bayer_raw;
		inst->vflip = sensors_def[i].inst[0].vflip;
		inst->hflip = sensors_def[i].inst[0].hflip;
		for (j = 0; j < MAX_POW_NUM; j++) {
			strcpy(sensors->power[j].power_str,
			       sensors_def[i].power[j].power_str);
			sensors->power[j].power_vol = sensors_def[i].power[j].power_vol;
		}
		module->flash_used = 0;
		module->act_used = 0;
		/* when insmod without parm */
		if (inst->act_addr == 0xff) {
			strcpy(inst->act_name, sensors_def[i].inst[0].act_name);
			inst->act_addr = sensors_def[i].inst[0].act_addr;
		}
/*
		for (j = 0; j < MAX_GPIO_NUM; j++) {
			sensors->gpio[j].gpio = sensors_def[i].gpio[j].gpio;
			sensors->gpio[j].mul_sel = sensors_def[i].gpio[j].mul_sel;
			sensors->gpio[j].pull = sensors_def[i].gpio[j].pull;
			sensors->gpio[j].drv_level = sensors_def[i].gpio[j].drv_level;
			sensors->gpio[j].data = sensors_def[i].gpio[j].data;
		}
*/
		sensors->detect_num = sensors_def[i].detect_num;
		vin_log(VIN_LOG_CONFIG, "vin cci_sel is %d\n", sensors->sensor_bus_sel);
	}
#else
	int i = 0, j = 0, idx;
	struct device_node *parent = vind->pdev->dev.of_node;
	struct device_node *cam = NULL, *child;
	char property[32] = { 0 };
	struct modules_config *module = NULL;
	struct sensor_list *sensors = NULL;
	const char *device_type;
	int ret;
	__maybe_unused  int size = 0;
	__maybe_unused const __be32 *list;

	for_each_available_child_of_node(parent, child) {
		if (!strcmp(child->name, "sensor")) {
			cam = child;

			ret = of_property_read_string(cam, "device_type", &device_type);
			if (ret) {
				vin_err("%s get sensor device_type failed!\n", __func__);
				continue;
			}

			sscanf(device_type, "sensor%d", &i);
			module = &vind->modules[i];
			sensors = &module->sensors;
		} else
			continue;

		/* when insmod without parm */
		if (!strcmp(sensors->inst[0].cam_name, "")) {
			fetch_camera[0].flag = 1;
			fetch_camera[1].flag = 1;
		}
		for (j = 0; j < ARRAY_SIZE(fetch_camera); j++) {
			if (!fetch_camera[j].flag)
				continue;

			sprintf(property, "%s%d_%s",
				cam->name, i, fetch_camera[j].sub);
			fetch_camera[j].fun(cam,
				property, sensors);
		}

#if IS_ENABLED(CONFIG_ACTUATOR_MODULE)
		/* get actuator node */
		sprintf(property, "%s", "act_handle");
		list = of_get_property(cam, property, &size);
		if ((!list) || (size == 0)) {
			vin_log(VIN_LOG_CONFIG, "missing %s property in node %s\n",
				property, cam->name);
			module->act_used = 0;
		} else {
			struct device_node *act = of_find_node_by_phandle(be32_to_cpup(list));
			if (!act) {
				vin_warn("%s invalid phandle\n", property);
			} else if (of_device_is_available(act)) {
				module->act_used = 1;
				/* when insmod without parm */
				if (!strcmp(sensors->inst[0].act_name, "")) {
					fetch_actuator[0].flag = 1;
					fetch_actuator[1].flag = 1;
				}
				for (j = 0; j < ARRAY_SIZE(fetch_actuator); j++) {
					if (!fetch_actuator[j].flag)
						continue;
					sprintf(property, "%s%d_%s", act->name,
						i, fetch_actuator[j].sub);
					fetch_actuator[j].fun(act,
							property, sensors);
				}
			}
		}

		if (!sensors->act_separate) {
			sensors->act_bus_sel = sensors->sensor_bus_sel;
			sensors->act_bus_type = sensors->sensor_bus_type;
		}
#else
		module->act_used = 0;
#endif

#if IS_ENABLED(CONFIG_FLASH_MODULE)
		/* get flash node */
		sprintf(property, "%s", "flash_handle");
		list = of_get_property(cam, property, &size);
		if ((!list) || (size == 0)) {
			vin_log(VIN_LOG_CONFIG, "missing %s property in node %s\n",
				property, cam->name);
			module->flash_used = 0;
		} else {
			struct device_node *flash = of_find_node_by_phandle(be32_to_cpup(list));
			if (!flash) {
				vin_warn("%s invalid phandle\n", property);
			} else if (of_device_is_available(flash)) {
				module->flash_used = 1;
				for (j = 0; j < ARRAY_SIZE(fetch_flash); j++) {
					if (!fetch_flash[j].flag)
						continue;

					sprintf(property, "%s%d_%s", flash->name,
						i, fetch_flash[j].sub);
					fetch_flash[j].fun(flash,
							property, sensors);
				}
				get_value_int(flash, "device_id",
						&module->modules.flash.id);
			}
		}
#else
		module->flash_used = 0;
#endif
		sensors->detect_num = 1;
	}

#if IS_ENABLED(CONFIG_SENSOR_LIST)
	for_each_available_child_of_node(parent, child) {
		if (strcmp(child->name, "sensor_list")) {
			continue;
		}
		cam = child;
		ret = of_property_read_string(cam, "device_type", &device_type);
		if (ret) {
			vin_err("%s get sensor_list device_type failed!\n", __func__);
			continue;
		}

		sscanf(device_type, "sensor_list%d", &i);
		module = &vind->modules[i];
		sensors = &module->sensors;

		sensor_list_get_parms(sensors, cam, i);
	}
#endif

	for_each_available_child_of_node(parent, child) {
		if (strcmp(child->name, "vinc"))
			continue;
		ret = of_property_read_string(child, "device_type", &device_type);
		if (ret) {
			vin_err("%s get sensor device_type failed!\n", __func__);
			continue;
		}
		sscanf(device_type, "vinc%d", &idx);
		sprintf(property, "%s%d_rear_sensor_sel", child->name, idx);
		if (get_value_int(child, property, &i))
			i = 0;
		module = &vind->modules[i];
		sensors = &module->sensors;
		sprintf(property, "%s%d_csi_sel", child->name, idx);
		get_value_int(child, property, &sensors->csi_sel);

		if (sensors->use_sensor_list == 0xff) {
			sprintf(property, "%s%d_sensor_list", child->name, idx);
			get_value_int(child, property, &sensors->use_sensor_list);
#ifndef CONFIG_SENSOR_LIST_MODULE
			sensors->use_sensor_list = 0;
#endif
		}

		sprintf(property, "%s%d_front_sensor_sel", child->name, idx);
		if (get_value_int(child, property, &i))
			i = 1;
		module = &vind->modules[i];
		sensors = &module->sensors;
		sprintf(property, "%s%d_csi_sel", child->name, idx);
		get_value_int(child, property, &sensors->csi_sel);

		if (sensors->use_sensor_list == 0xff) {
			sprintf(property, "%s%d_sensor_list", child->name, idx);
			get_value_int(child, property, &sensors->use_sensor_list);
#ifndef CONFIG_SENSOR_LIST_MODULE
			sensors->use_sensor_list = 0;
#endif
		}
	}
#endif
	return 0;
}
