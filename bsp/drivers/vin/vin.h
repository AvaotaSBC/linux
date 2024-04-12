/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * vin.h for all v4l2 subdev manage
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

#ifndef _VIN_H_
#define _VIN_H_

#include <media/media-device.h>
#include <media/media-entity.h>
#include <linux/reset.h>

#include "vin-video/vin_video.h"
#include "vin-video/vin_core.h"
#include "vin-csi/sunxi_csi.h"
#include "vin-isp/sunxi_isp.h"
#include "vin-vipp/sunxi_scaler.h"
#include "vin-mipi/sunxi_mipi.h"
#include "vin-tdm/vin_tdm.h"
#include "platform/platform_cfg.h"
#include "top_reg.h"

enum {
	VIN_TOP_CLK = 0,
	VIN_TOP_CLK_SRC,
	VIN_MAX_CLK,
};

enum {
	VIN_MIPI_CLK = 0,
	VIN_MIPI_CLK_SRC,
	VIN_MIPI_MAX_CLK,
};

enum {
	VIN_ISP_CLK = 0,
	VIN_ISP_CLK_SRC,
	VIN_ISP_MAX_CLK,
};

enum {
	VIN_CSI_RET = 0,
	VIN_ISP_RET,
	VIN_MAX_RET,
};

enum {
	VIN_CSI_BUS_CLK = 0,
	VIN_CSI_MBUS_CLK,
	VIN_ISP_MBUS_CLK,
	VIN_ISP_BUS_CLK,
	VIN_MAX_BUS_CLK,
};

#define VIN_CLK_RATE (432*1000*1000)
#define ISP_CLK_RATE (300*1000*1000)

#define NO_VALID_SENSOR (-1)

#if IS_ENABLED(CONFIG_RV_RUN_CAR_REVERSE)
#define VIN_STATUS_PACKET_MAGIC 0x10244025

enum vin_control {
	CONTROL_BY_RTOS,
	CONTROL_BY_ARM,
	CONTROL_BY_NONE,
};

enum vinc_status_packet_type {
	RV_VIN_START,
	RV_VIN_START_ACK,
	RV_VIN_STOP,
	RV_VIN_STOP_ACK,

	ARM_VIN_START,
	ARM_VIN_START_ACK,
	ARM_VIN_STOP,
	ARM_VIN_STOP_ACK,

	ARM_RPMSG_READY,
};

struct rpmsg_vinc {
	enum vin_control control;
};

struct vinc_status_packet {
	u32 magic;
	u32 type;
};

int vinc_status_rpmsg_send(enum vinc_status_packet_type type,
	struct rpmsg_vinc *rpmsg);

int vin_rpmsg_notify_ready(void);
#endif

struct vin_valid_sensor {
	struct v4l2_subdev *sd;
	char *name;
};

struct vin_csi_info {
	struct v4l2_subdev *sd;
	int id;
};

struct vin_mipi_info {
	struct v4l2_subdev *sd;
	int id;
};

struct vin_cci_info {
	struct v4l2_subdev *sd;
	int id;
};

struct vin_isp_info {
	struct v4l2_subdev *sd;
	int id;
};

struct vin_tdm_rx_info{
	struct v4l2_subdev *sd;
	int id;
};

struct vin_tdm_info {
	struct vin_tdm_rx_info tdm_rx[TDM_RX_NUM];
	int id;
};

struct vin_stat_info {
	struct v4l2_subdev *sd;
	int id;
};

struct vin_scaler_info {
	struct v4l2_subdev *sd;
	int id;
};

struct vin_clk_info {
	struct clk *clock;
	int use_count;
	unsigned long frequency;
};

struct vin_mclk_info {
	struct clk *mclk;
	struct clk *clk_24m;
	struct clk *clk_pll;
	struct pinctrl *pin;
	int use_count;
	unsigned long frequency;
};

enum vin_sub_device_regulator {
	ENUM_IOVDD,
	ENUM_AVDD,
	ENUM_DVDD,
	ENUM_AFVDD,
	ENUM_FLVDD,
	ENUM_CAMERAVDD,
	ENUM_MAX_REGU,
};

struct vin_power {
	struct regulator *pmic;
	int power_vol;
	char power_str[32];
};
struct sensor_instance {
	char cam_name[I2C_NAME_SIZE];
	int cam_addr;
	int cam_type;
	int is_isp_used;
	int is_bayer_raw;
	int vflip;
	int hflip;
	int act_addr;
	int act_used;
	char act_name[I2C_NAME_SIZE];
	char isp_cfg_name[I2C_NAME_SIZE];
};

struct sensor_list {
	int use_sensor_list;
	int used;
	int csi_sel;
	int device_sel;
	int mclk_id;
	int sensor_bus_sel;
	int sensor_bus_type;
	int act_bus_sel;
	int act_bus_type;
	int act_separate;
	int power_set;
	int detect_num;
	char sensor_pos[32];
	int valid_idx;
	struct vin_power power[ENUM_MAX_REGU];
	int gpio[MAX_GPIO_NUM];
	struct sensor_instance inst[MAX_DETECT_NUM];
};

enum module_type {
	VIN_MODULE_TYPE_I2C,
	VIN_MODULE_TYPE_CCI,
	VIN_MODULE_TYPE_SPI,
	VIN_MODULE_TYPE_GPIO,
	VIN_MODULE_TYPE_MAX,
};

struct vin_act_info {
	struct v4l2_subdev *sd;
	enum module_type type;
	int id;
};

struct vin_flash_info {
	struct v4l2_subdev *sd;
	enum module_type type;
	int id;
};

struct vin_sensor_info {
	struct v4l2_subdev *sd;
	enum module_type type;
	int id;
};

struct vin_module_info {
	struct vin_act_info act[MAX_DETECT_NUM];
	struct vin_flash_info flash;
	struct vin_sensor_info sensor[MAX_DETECT_NUM];
	int id;
};

struct modules_config {
	struct vin_module_info modules;
	struct sensor_list sensors;
	int flash_used;
	int act_used;
};

struct vin_md {
	struct vin_csi_info csi[VIN_MAX_CSI];
	struct vin_mipi_info mipi[VIN_MAX_MIPI];
	struct vin_cci_info cci[VIN_MAX_CCI];
	struct vin_isp_info isp[VIN_MAX_ISP];
	struct vin_tdm_info tdm[VIN_MAX_TDM];
	struct vin_stat_info stat[VIN_MAX_ISP];
	struct vin_scaler_info scaler[VIN_MAX_SCALER];
	struct vin_core *vinc[VIN_MAX_DEV];
	struct vin_clk_info clk[VIN_MAX_CLK];
	struct vin_clk_info mipi_clk[VIN_MIPI_MAX_CLK];
	struct vin_mclk_info mclk[VIN_MAX_CCI];
	struct vin_clk_info isp_clk[VIN_ISP_MAX_CLK];
	struct reset_control *clk_reset[VIN_MAX_RET];
	struct clk *bus_clk[VIN_MAX_BUS_CLK];
	struct modules_config modules[VIN_MAX_DEV];
	struct csic_feature_list csic_fl;
	struct csic_version csic_ver;
	unsigned int isp_ver_major;
	unsigned int isp_ver_minor;
	unsigned int is_empty;
	unsigned int id;
	unsigned int irq;
	int use_count;
	int bridge_en_count;
	void __iomem *base;
	void __iomem *ccu_base;
	void __iomem *cmb_top_base;
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;
	bool user_subdev_api;
	spinlock_t slock;
	struct mutex mclk_pin_lock;
	unsigned int isp_bd_tatol;
	unsigned int csi_bd_tatol;
#if IS_ENABLED(CONFIG_ARCH_SUN55IW3)
	struct regulator *vin_pinctrl[VIN_MAX_PINCTRL];
	unsigned int dram_dfs_time;
#endif
	struct bk_intpool_cfg bk_intpool;
	bool sensor_power_on;
	bool clk_en;
};

static inline struct vin_md *entity_to_vin_mdev(struct media_entity *me)
{
	return me->graph_obj.mdev == NULL ? NULL :
		container_of(me->graph_obj.mdev, struct vin_md, media_dev);
}

/*
 * Media pipeline operations to be called from within the video
 * node when it is the last entity of the pipeline. Implemented
 * by corresponding media device driver.
 */

struct vin_pipeline_ops {
	int (*open)(struct vin_pipeline *p, struct media_entity *me,
			  bool resume);
	int (*close)(struct vin_pipeline *p);
	int (*set_stream)(struct vin_pipeline *p, int state);
};

#define vin_pipeline_call(f, op, p, args...)				\
	(((f)->pipeline_ops && (f)->pipeline_ops->op) ? \
			    (f)->pipeline_ops->op((p), ##args) : -ENOIOCTLCMD)


#endif /* _VIN_H_ */
