/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2023 Allwinnertech Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_property.h>
#include <drm/drm_panel.h>
#include <drm/drm_edid.h>
#include <sunxi-log.h>
#include <linux/component.h>
#include <linux/phy/phy.h>
#include <sound/hdmi-codec.h>

#include "sunxi_device/sunxi_edp.h"
#include "sunxi_device/sunxi_tcon.h"
#include "sunxi_drm_intf.h"
#include "sunxi_drm_crtc.h"
#include "sunxi_drm_drv.h"

#if IS_ENABLED(CONFIG_EXTCON)
#include <linux/version.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include "../drivers/extcon/extcon.h"
#endif

#define DEFAULT_COLORSPACE_MODE HDMI_COLORSPACE_RGB
#define CEA_BASIC_AUDIO_MASK     (1 << 6) /* Version3 */
#define CEA_YCC444_MASK	         (1 << 5) /* Version3 */
#define CEA_YCC422_MASK	         (1 << 4) /* Version3 */

#define SUNXI_EDP_CODEC_DRV_NAME "sunxi-snd-codec-av"

u32 loglevel_debug;

struct edp_debug {
	unsigned long aux_i2c_addr;
	unsigned long aux_i2c_len;
	unsigned long aux_read_start;
	unsigned long aux_read_end;
	u32 aux_write_start;
	u32 aux_write_len;
	u32 aux_write_val[16];
	u32 aux_write_val_before[16];
	u32 lane_debug_en;
	u32 hpd_mask;
	u32 hpd_mask_pre;

	/* resource lock, power&clk won't release when edp enable fail if not 0 */
	u32 edp_res_lock;

	/* bypass training for some signal test case */
	u32 bypass_training;
};

struct sunxi_drm_edp {
	struct sunxi_drm_device sdrm;
	struct drm_display_mode mode;
	struct task_struct *edp_task;
	bool bound;

	u32 enable;
	u32 irq;
	void __iomem *base_addr;
	void __iomem *top_addr;
	dev_t devid;
	struct cdev *edp_cdev;
	struct class *edp_class;
	struct device *edp_class_dev;
	struct device *dev;
	struct platform_device *audio_pdev;
	struct phy *dp_phy;
	struct phy *aux_phy;
	struct phy *combo_phy;
	struct clk *clk_bus;
	struct clk *clk;
	struct clk *clk_24m;
	struct regulator *vdd_regulator;
	struct regulator *vcc_regulator;
	struct reset_control *rst_bus;
	struct sunxi_edp_output_desc *desc;

	/* drm property */
	struct drm_property *colorspace_property;

	bool hpd_state;
	bool hpd_state_now;
	bool dpcd_parsed;
	bool use_dpcd;
	/*FIXME:TODO: optimize relate code*/
	bool fps_limit_60;
	/*end FIXME*/
	bool use_debug_para;
	bool is_enabled;

	bool sw_enable;
	bool boot_enabled;

	struct edp_tx_core edp_core;
	struct sunxi_edp_hw_desc edp_hw;
	struct edp_tx_cap source_cap;
	struct edp_rx_cap sink_cap;
	struct edp_debug edp_debug;
	struct sunxi_dp_hdcp hdcp;
#if IS_ENABLED(CONFIG_EXTCON)
	struct extcon_dev *extcon_edp;
#endif
};

struct sunxi_edp_connector_state {
	struct drm_connector_state state;
	int color_format;
	int color_depth;
	int lane_cnt;
	int lane_rate;
};

#define to_drm_edp_connector_state(s) container_of(s, struct sunxi_edp_connector_state, state)

struct sunxi_edp_output_desc {
	int connector_type;
	int hw_id;
	s32 (*bind)(struct sunxi_drm_edp *drm_edp);
	s32 (*unbind)(struct sunxi_drm_edp *drm_edp);
	s32 (*enable_early)(struct sunxi_drm_edp *drm_edp);
	s32 (*enable_early_sw)(struct sunxi_drm_edp *drm_edp);
	s32 (*enable)(struct sunxi_drm_edp *drm_edp);
	s32 (*enable_sw)(struct sunxi_drm_edp *drm_edp);
	s32 (*disable)(struct sunxi_drm_edp *drm_edp);
	s32 (*plugin)(struct sunxi_drm_edp *drm_edp);
	s32 (*plugout)(struct sunxi_drm_edp *drm_edp);
	s32 (*runtime_suspend)(struct sunxi_drm_edp *drm_edp);
	s32 (*runtime_resume)(struct sunxi_drm_edp *drm_edp);
	s32 (*suspend)(struct sunxi_drm_edp *drm_edp);
	s32 (*resume)(struct sunxi_drm_edp *drm_edp);
	void (*soft_reset)(struct sunxi_drm_edp *drm_edp);
};


static const struct drm_prop_enum_list colorspace_mode_names[] = {
	{ HDMI_COLORSPACE_RGB, "rgb" },
	{ HDMI_COLORSPACE_YUV422, "yuv422" },
	{ HDMI_COLORSPACE_YUV444, "yuv444" },
};

static const struct drm_display_mode edp_standard_modes[] = {
	/* dmt: 0x55 - 1280x720@60Hz */
	{ DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
		   1430, 1650, 0, 720, 725, 730, 750, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
	/* cea: 19 - 1280x720@50Hz 16:9 */
	{ DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250, 1280, 1720,
		   1760, 1980, 0, 720, 725, 730, 750, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
	/* dmt: 0x52 - 1920x1080@60Hz */
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
		   2052, 2200, 0, 1080, 1084, 1089, 1125, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
	/* cea: 31 - 1920x1080@50Hz 16:9 */
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2448,
		   2492, 2640, 0, 1080, 1084, 1089, 1125, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
	/* dmt: 0x4d - 2560x1600@60Hz */
	{ DRM_MODE("2560x1600", DRM_MODE_TYPE_DRIVER, 348500, 2560, 2752,
		   3032, 3504, 0, 1600, 1603, 1609, 1658, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
	/* cea: 3 - 720x480@60Hz 16:9 */
	{ DRM_MODE("720x480", DRM_MODE_TYPE_DRIVER, 27000, 720, 736,
		   798, 858, 0, 480, 489, 495, 525, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },

};


static inline struct sunxi_drm_edp *
drm_connector_to_sunxi_drm_edp(struct drm_connector *connector)
{
	struct sunxi_drm_device *sdrm = container_of(connector, struct sunxi_drm_device, connector);

	return container_of(sdrm, struct sunxi_drm_edp, sdrm);
}

static inline struct sunxi_drm_edp *
drm_encoder_to_sunxi_drm_edp(struct drm_encoder *encoder)
{
	struct sunxi_drm_device *sdrm = container_of(encoder, struct sunxi_drm_device, encoder);

	return container_of(sdrm, struct sunxi_drm_edp, sdrm);
}

static int __parse_dump_str(const char *buf, size_t size,
				unsigned long *start, unsigned long *end)
{
	char *ptr = NULL;
	char *ptr2 = (char *)buf;
	int ret = 0, times = 0;

	/* Support single address mode, some time it haven't ',' */
next:

	/*Default dump only one register(*start =*end).
	If ptr is not NULL, we will cover the default value of end.*/
	if (times == 1)
		*start = *end;

	if (!ptr2 || (ptr2 - buf) >= size)
		goto out;

	ptr = ptr2;
	ptr2 = strnchr(ptr, size - (ptr - buf), ',');
	if (ptr2) {
		*ptr2 = '\0';
		ptr2++;
	}

	ptr = strim(ptr);
	if (!strlen(ptr))
		goto next;

	ret = kstrtoul(ptr, 16, end);
	if (!ret) {
		times++;
		goto next;
	} else
	EDP_ERR("String syntax errors: \"%s\"\n", ptr);

out:
	return ret;
}

#if IS_ENABLED(CONFIG_EXTCON)
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 20, 0)
#define EXTCON_EDP_TYPE EXTCON_DISP_DP
#else
#define EXTCON_EDP_TYPE EXTCON_DISP_EDP
#endif

#define EXTCON_DP_TYPE EXTCON_DISP_DP

static char edp_extcon_name[20];
static const u32 edp_cable[] = {
	EXTCON_EDP_TYPE,
	EXTCON_NONE,
};

static const u32 dp_cable[] = {
	EXTCON_DP_TYPE,
	EXTCON_NONE,
};

s32 edp_report_hpd_work(struct sunxi_drm_edp *drm_edp, u32 hpd)
{
	struct sunxi_edp_output_desc *desc = drm_edp->desc;

	if (drm_edp->hpd_state == hpd)
		return -1;

	switch (hpd) {

	case EDP_HPD_PLUGIN:
		EDP_DRV_DBG("set EDP EXTCON to 1\n");
		if (desc->connector_type == DRM_MODE_CONNECTOR_eDP)
			extcon_set_state_sync(drm_edp->extcon_edp, EXTCON_EDP_TYPE,
					      EDP_HPD_PLUGIN);
		else
			extcon_set_state_sync(drm_edp->extcon_edp, EXTCON_DP_TYPE,
				      EDP_HPD_PLUGIN);

		break;
	case EDP_HPD_PLUGOUT:
	default:
		EDP_DRV_DBG("set EDP EXTCON to 0\n");
		if (desc->connector_type == DRM_MODE_CONNECTOR_eDP)
			extcon_set_state_sync(drm_edp->extcon_edp, EXTCON_EDP_TYPE,
					      EDP_HPD_PLUGOUT);
		else
			extcon_set_state_sync(drm_edp->extcon_edp, EXTCON_DP_TYPE,
					      EDP_HPD_PLUGOUT);
		break;
	}

	return RET_OK;
}

#else
s32 edp_report_hpd_work(struct sunxi_drm_edp *drm_edp, u32 hpd)
{
	EDP_WRN("You need to enable CONFIG_EXTCON!\n");
	return RET_OK;
}
#endif


irqreturn_t drm_edp_irq_handler(int irq, void *dev_data)
{
	struct device *dev = dev_data;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	int ret;

	EDP_DRV_DBG("edp irq handler!\n");
	ret = edp_hw_get_hotplug_state(edp_hw);
	if (ret >= 0)
		drm_edp->hpd_state_now =  ret ? true : false;

	/* TODO: such as aux reply error */

	/* any other special irq event */
	edp_hw_irq_handler(edp_hw, &drm_edp->edp_core);

	/* CP_IRQ */
//	if (edp_get_irq_vector() & DP_CP_IRQ)
//		sunxi_dp_hdcp_irq_handler();

	return IRQ_HANDLED;
}

void edp_soft_reset(struct sunxi_drm_edp *drm_edp)
{
	struct sunxi_edp_output_desc *desc = drm_edp->desc;

	if (desc->soft_reset)
		desc->soft_reset(drm_edp);
}

static void edp_hotplugin_proc(struct sunxi_drm_edp *drm_edp)
{
	struct sunxi_edp_output_desc *desc = drm_edp->desc;

	if (desc->plugin)
		desc->plugin(drm_edp);

	drm_kms_helper_hotplug_event(drm_edp->sdrm.drm_dev);
}

static void edp_hotplugout_proc(struct sunxi_drm_edp *drm_edp)
{
	struct sunxi_edp_output_desc *desc = drm_edp->desc;

	if (desc->plugout)
		desc->plugout(drm_edp);

	drm_kms_helper_hotplug_event(drm_edp->sdrm.drm_dev);
}

static void edp_hpd_mask_proc(struct sunxi_drm_edp *drm_edp, u32 hpd_mask)
{
	switch (hpd_mask) {
	/* force plugout */
	case 0x10:
		edp_report_hpd_work(drm_edp, EDP_HPD_PLUGOUT);
		edp_hotplugout_proc(drm_edp);
		drm_edp->hpd_state = false;
		break;
	case 0x110:
	case 0x1010:
		edp_hotplugout_proc(drm_edp);
		drm_edp->hpd_state = false;
		break;
	/* force plugin */
	case 0x11:
		edp_report_hpd_work(drm_edp, EDP_HPD_PLUGIN);
		edp_hotplugin_proc(drm_edp);
		drm_edp->hpd_state = true;
		break;
	case 0x111:
	case 0x1011:
		edp_hotplugin_proc(drm_edp);
		drm_edp->hpd_state = true;
		break;
	default:
		EDP_ERR("Unknown hpd mask!\n");
		break;
	}
}

s32 edp_read_dpcd(struct sunxi_edp_hw_desc *edp_hw, char *dpcd_rx_buf)
{
	u32 i = 0;
	u32 block = 16;
	s32 ret = 0;

	for (i = 0; i < 576 / block; i++) {
		ret = edp_hw_aux_read(edp_hw, DPCD_0000H + i * block,
				block, (char *)(dpcd_rx_buf) + (i * block));
		if (ret < 0)
			return ret;
	}

	return 0;
}

s32 edp_read_dpcd_extended(struct sunxi_edp_hw_desc *edp_hw, char *dpcd_ext_rx_buf)
{
	u32 block = 16;

	return edp_hw_aux_read(edp_hw, DPCD_2200H, block, dpcd_ext_rx_buf);
}

void edp_parse_dpcd(struct sunxi_drm_edp *drm_edp, char *dpcd_rx_buf)
{
	struct edp_rx_cap *sink_cap;
	struct edp_tx_core *edp_core;

	sink_cap = &drm_edp->sink_cap;
	edp_core = &drm_edp->edp_core;

	/*
	 * Sometimes aux return all 0 data when edp panel/DisplayPort
	 * is not connect, although the aux didn't return fail. We pick
	 * to most important dpcd register as judgement because they will
	 * never set to 0 in DP spec
	 */
	if ((dpcd_rx_buf[0] == 0x00) || dpcd_rx_buf[1] == 0x00) {
		EDP_ERR("dpcd read all 0, read dpcd fail!\n");
		return;
	}

	sink_cap->dpcd_rev = 10 + ((dpcd_rx_buf[0] >> 4) & 0x0f);
	EDP_DRV_DBG("DPCD version:1.%d\n", sink_cap->dpcd_rev % 10);

	if (dpcd_rx_buf[1] == 0x06) {
		sink_cap->max_rate = BIT_RATE_1G62;
		EDP_DRV_DBG("sink max bit rate:1.62Gbps\n");
	} else if (dpcd_rx_buf[1] == 0x0a) {
		sink_cap->max_rate = BIT_RATE_2G7;
		EDP_DRV_DBG("sink max bit rate:2.7Gbps\n");
	} else if (dpcd_rx_buf[1] == 0x14) {
		sink_cap->max_rate = BIT_RATE_5G4;
		EDP_DRV_DBG("sink max bit rate:5.4Gbps\n");
	} else if (dpcd_rx_buf[1] == 0x1e) {
		sink_cap->max_rate = BIT_RATE_8G1;
		EDP_DRV_DBG("sink max bit rate:8.1Gbps\n");
	}

	sink_cap->max_lane = dpcd_rx_buf[2] & EDP_DPCD_MAX_LANE_MASK;
	EDP_DRV_DBG("sink max lane count:%d\n", sink_cap->max_lane);

	if (dpcd_rx_buf[2] & EDP_DPCD_ENHANCE_FRAME_MASK) {
		sink_cap->enhance_frame_support = true;
		EDP_DRV_DBG("enhanced mode:support\n");
	} else {
		sink_cap->enhance_frame_support = false;
		EDP_DRV_DBG("enhanced mode:not support\n");
	}

	if (dpcd_rx_buf[2] & EDP_DPCD_TPS3_MASK) {
		sink_cap->tps3_support = true;
		EDP_DRV_DBG("TPS3: support\n");
	} else {
		sink_cap->tps3_support = false;
		EDP_DRV_DBG("TPS3: not support\n");
	}

	if (dpcd_rx_buf[3] & EDP_DPCD_FAST_TRAIN_MASK) {
		sink_cap->fast_train_support = true;
		EDP_DRV_DBG("fast training: support\n");
	} else {
		sink_cap->fast_train_support = false;
		EDP_DRV_DBG("fast training: not support\n");
	}

	if (dpcd_rx_buf[5] & EDP_DPCD_DOWNSTREAM_PORT_MASK) {
		sink_cap->downstream_port_support = true;
		EDP_DRV_DBG("downstream port:present\n");
	} else {
		sink_cap->downstream_port_support = false;
		EDP_DRV_DBG("downstream port:not present\n");
	}

	sink_cap->downstream_port_type = (dpcd_rx_buf[5] & EDP_DPCD_DOWNSTREAM_PORT_TYPE_MASK);

	sink_cap->downstream_port_cnt = (dpcd_rx_buf[7] & EDP_DPCD_DOWNSTREAM_PORT_CNT_MASK);

	if (dpcd_rx_buf[8] & EDP_DPCD_LOCAL_EDID_MASK) {
		sink_cap->local_edid_support = true;
		EDP_DRV_DBG("ReceiverPort0 Capability_0:Has a local EDID\n");
	} else {
		sink_cap->local_edid_support = false;
		EDP_DRV_DBG("ReceiverPort0 Capability_0:Not has a local EDID\n");
	}

	/* eDP_CONFIGURATION_CAP */
	/* Always reads 0x00 for external receivers */
	if (dpcd_rx_buf[0x0d] != 0) {
		sink_cap->is_edp_device = true;
		EDP_DRV_DBG("Sink device is eDP receiver!\n");
		if (dpcd_rx_buf[0x0d] & EDP_DPCD_ASSR_MASK)
			sink_cap->assr_support = true;
		else
			sink_cap->assr_support = false;

		if (dpcd_rx_buf[0x0d] & EDP_DPCD_FRAME_CHANGE_MASK)
			sink_cap->framing_change_support = true;
		else
			sink_cap->framing_change_support = false;

	} else {
		sink_cap->is_edp_device = false;
		EDP_DRV_DBG("Sink device is external receiver!\n");
	}

	switch (dpcd_rx_buf[0x0e]) {
	case 0x00:
		/*Link Status/Adjust Request read interval during CR*/
		/*phase --- 100us*/
		edp_core->interval_CR = 100;
		/*Link Status/Adjust Request read interval during EQ*/
		/*phase --- 400us*/
		edp_core->interval_EQ = 400;

		break;
	case 0x01:
		edp_core->interval_CR = 4000;
		edp_core->interval_EQ = 4000;
		break;
	case 0x02:
		edp_core->interval_CR = 8000;
		edp_core->interval_EQ = 8000;
		break;
	case 0x03:
		edp_core->interval_CR = 12000;
		edp_core->interval_EQ = 12000;
		break;
	case 0x04:
		edp_core->interval_CR = 16000;
		edp_core->interval_EQ = 16000;
		break;
	default:
		edp_core->interval_CR = 100;
		edp_core->interval_EQ = 400;
	}

	drm_edp->dpcd_parsed = true;
}

s32 edp_running_thread(void *parg)
{
	struct sunxi_drm_edp *drm_edp = NULL;
	struct edp_debug *edp_debug;
	struct sunxi_edp_hw_desc *edp_hw = NULL;;

	if (!parg) {
		EDP_ERR("NUll ndl\n");
		return -1;
	}

	drm_edp = (struct sunxi_drm_edp *)parg;
	edp_debug = &drm_edp->edp_debug;
	edp_hw = &drm_edp->edp_hw;

	while (1) {
		if (kthread_should_stop())
			break;

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(200);

		/* soft reset controller when error occurs, such as ESD */
		if (edp_hw_check_controller_error(edp_hw))
			edp_soft_reset(drm_edp);

		if (edp_debug->hpd_mask & 0x1000)
			continue;

		if (edp_debug->hpd_mask & 0x10) {
			if (edp_debug->hpd_mask != edp_debug->hpd_mask_pre) {
				edp_debug->hpd_mask_pre = edp_debug->hpd_mask;
				edp_hpd_mask_proc(drm_edp, edp_debug->hpd_mask);
			}
			continue;
		} else {
			if (drm_edp->hpd_state_now != drm_edp->hpd_state) {
				if (!drm_edp->hpd_state_now)
					edp_hotplugout_proc(drm_edp);
				else
					edp_hotplugin_proc(drm_edp);

				edp_report_hpd_work(drm_edp, drm_edp->hpd_state_now);
				drm_edp->hpd_state = drm_edp->hpd_state_now;
				EDP_DRV_DBG("hpd = %d\n", drm_edp->hpd_state_now);
			}
		}
	}
	return RET_OK;
}

s32 edp_kthread_start(struct sunxi_drm_edp *drm_edp)
{
	s32 err = 0;

	if (!drm_edp->edp_task) {
		drm_edp->edp_task = kthread_create(edp_running_thread,
					      drm_edp, "edp detect");
		if (IS_ERR(drm_edp->edp_task)) {
			err = PTR_ERR(drm_edp->edp_task);
			drm_edp->edp_task = NULL;
			return err;
		}
		EDP_DRV_DBG("edp_task is ok!\n");
		wake_up_process(drm_edp->edp_task);
	}
	return RET_OK;
}

s32 edp_kthread_stop(struct sunxi_drm_edp *drm_edp)
{
	if (drm_edp->edp_task) {
		kthread_stop(drm_edp->edp_task);
		drm_edp->edp_task = NULL;
	}
	return RET_OK;
}

static s32 edp_clk_enable(struct sunxi_drm_edp *drm_edp, bool en)
{
	s32 ret = -1;

	if (en) {
		if (!IS_ERR_OR_NULL(drm_edp->rst_bus)) {
			ret = reset_control_deassert(drm_edp->rst_bus);
			if (ret) {
				EDP_ERR("deassert reset failed\n");
				return ret;
			}
		}

		if (!IS_ERR_OR_NULL(drm_edp->clk_bus)) {
			ret = clk_prepare_enable(drm_edp->clk_bus);
			if (ret) {
				EDP_ERR("enable clk_bus failed\n");
				return ret;
			}
		}

		if (!IS_ERR_OR_NULL(drm_edp->clk)) {
			ret = clk_prepare_enable(drm_edp->clk);
			if (ret) {
				EDP_ERR("enable clk failed\n");
				return ret;
			}
		}

		if (!IS_ERR_OR_NULL(drm_edp->clk_24m)) {
			ret = clk_prepare_enable(drm_edp->clk_24m);
			if (ret) {
				EDP_ERR("enable clk_24m failed\n");
				return ret;
			}
		}
	} else {
		if (!IS_ERR_OR_NULL(drm_edp->clk_24m))
			clk_disable_unprepare(drm_edp->clk_24m);

		if (!IS_ERR_OR_NULL(drm_edp->clk))
			clk_disable_unprepare(drm_edp->clk);

		if (!IS_ERR_OR_NULL(drm_edp->clk_bus))
			clk_disable_unprepare(drm_edp->clk_bus);

		if (!IS_ERR_OR_NULL(drm_edp->rst_bus)) {
			ret = reset_control_assert(drm_edp->rst_bus);
			if (ret) {
				EDP_ERR("assert reset failed\n");
				return ret;
			}
		}
	}

	return ret;
}

static void edp_phy_enable(struct sunxi_drm_edp *drm_edp, bool en)
{
	if (en) {
		if (drm_edp->aux_phy)
			phy_power_on(drm_edp->aux_phy);

		if (drm_edp->combo_phy)
			phy_power_on(drm_edp->combo_phy);

		if (drm_edp->dp_phy)
			phy_power_on(drm_edp->dp_phy);
	} else {
		if (drm_edp->aux_phy)
			phy_power_off(drm_edp->aux_phy);

		if (drm_edp->combo_phy)
			phy_power_off(drm_edp->combo_phy);

		if (drm_edp->dp_phy)
			phy_power_off(drm_edp->dp_phy);
	}
}

static void edp_phy_init(struct sunxi_drm_edp *drm_edp)
{
	// the phy has only one user such as aux/dp,
	// we can init it when edp driver need.

	if (drm_edp->aux_phy)
		phy_init(drm_edp->aux_phy);

	/*
	 * we can use 4lane as default, phy para will update
	 * when training start
	 */
	if (drm_edp->combo_phy)
//		phy_set_mode_ext(drm_edp->combo_phy, PHY_MODE_DP, DP_2LANE | USB);
		phy_set_mode(drm_edp->combo_phy, PHY_MODE_DP);

	if (drm_edp->dp_phy) {
		phy_init(drm_edp->dp_phy);
		phy_set_mode(drm_edp->dp_phy, PHY_MODE_DP);
	}

	//FIXME: if need to judge typec-dp out's case, when should we update
	//combo phy para, in typec-switch driver or in this edp driver?
}

static void edp_phy_exit(struct sunxi_drm_edp *drm_edp)
{
	if (drm_edp->aux_phy)
		phy_exit(drm_edp->aux_phy);

	if (drm_edp->dp_phy)
		phy_exit(drm_edp->dp_phy);

	// ignore combo phy, leave it until other module configure it
}

/* FIXME: we need to think about that: typec-dp out's 2LNAE+USB case,
 * how DisplayPort's report its DPCD capbility? if it report
 * 4lane max support, how to compatible with 2LANE+USB case?
 * when should we update the lane para and phy para? */
void edp_update_capacity(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct edp_rx_cap *sink_cap;
	struct edp_tx_cap *src_cap;
	struct edp_lane_para *lane_para;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->lane_para;
	sink_cap = &drm_edp->sink_cap;
	src_cap = &drm_edp->source_cap;

	if (drm_edp->dpcd_parsed) {
		lane_para->lane_cnt = min(src_cap->max_lane, sink_cap->max_lane);
		lane_para->bit_rate = min(src_cap->max_rate, sink_cap->max_rate);
	}
}

void sink_cap_reset(struct sunxi_drm_edp *drm_edp)
{
	struct edp_rx_cap *sink_cap;

	sink_cap = &drm_edp->sink_cap;

	memset(sink_cap, 0, sizeof(struct edp_rx_cap));
}

s32 edid_to_sink_info(struct sunxi_drm_edp *drm_edp, struct edid *edid)
{
	struct edp_rx_cap *sink_cap;
	const u8 *cea;
	s32 start, end;
	int ext_index = 0;

	sink_cap = &drm_edp->sink_cap;

	sink_cap->mfg_week = edid->mfg_week;
	sink_cap->mfg_year = edid->mfg_year;
	sink_cap->edid_ver = edid->version;
	sink_cap->edid_rev = edid->revision;

	/*14H: edid input info*/
	sink_cap->input_type = (edid->input & DRM_EDID_INPUT_DIGITAL);

	/*digital input*/
	if (sink_cap->input_type) {
		switch (edid->input & DRM_EDID_DIGITAL_DEPTH_MASK) {
		case DRM_EDID_DIGITAL_DEPTH_6:
			sink_cap->bit_depth = 6;
			break;
		case DRM_EDID_DIGITAL_DEPTH_8:
			sink_cap->bit_depth = 8;
			break;
		case DRM_EDID_DIGITAL_DEPTH_10:
			sink_cap->bit_depth = 10;
			break;
		case DRM_EDID_DIGITAL_DEPTH_12:
			sink_cap->bit_depth = 12;
			break;
		case DRM_EDID_DIGITAL_DEPTH_14:
			sink_cap->bit_depth = 14;
			break;
		case DRM_EDID_DIGITAL_DEPTH_16:
			sink_cap->bit_depth = 16;
			break;
		case DRM_EDID_DIGITAL_DEPTH_UNDEF:
		default:
			sink_cap->bit_depth = 0;
			break;
		}

		sink_cap->video_interface = edid->input & DRM_EDID_DIGITAL_TYPE_MASK;

		switch (edid->features & DRM_EDID_FEATURE_COLOR_MASK) {
		case DRM_EDID_FEATURE_RGB_YCRCB444:
			sink_cap->Ycc444_support = true;
			sink_cap->Ycc422_support = false;
			break;
		case DRM_EDID_FEATURE_RGB_YCRCB422:
			sink_cap->Ycc444_support = false;
			sink_cap->Ycc422_support = true;
			break;
		case DRM_EDID_FEATURE_RGB_YCRCB:
			sink_cap->Ycc444_support = true;
			sink_cap->Ycc422_support = true;
			break;
		default:
			sink_cap->Ycc444_support = false;
			sink_cap->Ycc422_support = false;
			break;
		}
	}


	sink_cap->width_cm = edid->width_cm;
	sink_cap->height_cm = edid->height_cm;

	cea = sunxi_drm_find_edid_extension(edid, CEA_EXT, &ext_index);

	if (cea) {
		if (cea[0] != CEA_EXT) {
			EDP_ERR("wrong CEA tag\n");
			return RET_OK;
		}

		if (cea[1] < 3) {
			EDP_ERR("wrong CEA revision\n");
			return RET_OK;
		}

		if (edp_edid_cea_db_offsets(cea, &start, &end)) {
			EDP_ERR("invalid data block offsets\n");
			return RET_OK;
		}

		sink_cap->audio_support = (cea[3] & CEA_BASIC_AUDIO_MASK) ?
					true : false;
		sink_cap->Ycc444_support = (cea[3] & CEA_YCC444_MASK) ?
					true : false;
		sink_cap->Ycc422_support = (cea[3] & CEA_YCC422_MASK) ?
					true : false;
	} else
		EDP_DRV_DBG("no CEA Extension found\n");

	return RET_OK;
}

s32 edp_parse_edid(struct sunxi_drm_edp *drm_edp, struct edid *edid)
{
	s32 ret;
	ret = edid_to_sink_info(drm_edp, edid);
	if (ret < 0)
		return ret;

	return ret;
}

s32 edp_debug_mode_parse(struct device *dev)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug = &drm_edp->edp_debug;
	s32 value = 0;
	s32 ret = -1;
	const char *str;

	if (drm_edp->desc->connector_type == DRM_MODE_CONNECTOR_eDP)
		drm_edp->edp_core.controller_mode = 0;
	else
		drm_edp->edp_core.controller_mode = 1;

	ret = of_property_read_string(dev->of_node, "force_mode", &str);
	if (ret == 0) {
		if (strcmp(str, "dp") == 0)
			drm_edp->edp_core.controller_mode = 1;
		else if (strcmp(str, "edp") == 0)
			drm_edp->edp_core.controller_mode = 0;
	}

	ret = of_property_read_u32(dev->of_node, "edp_training_param_type", &value);
	if (!ret) {
		if (value >= 3)
			EDP_WRN("edp_training_param_type out of range!\n");
		else
			drm_edp->edp_core.training_param_type = value;
	} else
		drm_edp->edp_core.training_param_type = 0;

	/* FIXME: limit some senior panel support 90/120 hz, default enable. we seem can not handle
	 * dual screen with different fps well */
	ret = of_property_read_u32(dev->of_node, "fps_limit_60", &value);
	if (!ret)
		drm_edp->fps_limit_60 = value ? true : false;
	else
		drm_edp->fps_limit_60 = true;

	edp_debug->bypass_training = 0;

	return RET_OK;
}

s32 edp_misc_parse(struct device *dev)
{
	s32 ret = -1;
	s32  value = 1;
	struct edp_tx_core *edp_core;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);

	edp_core = &drm_edp->edp_core;

	ret = of_property_read_u32(dev->of_node, "edp_ssc_en", &value);
	if (!ret)
		edp_core->ssc_en = value;
	else
		edp_core->ssc_en = 0;

	/*
	 * ssc modulation mode select
	 * 0: center mode
	 * 1: downspread mode
	 *
	 * */
	ret = of_property_read_u32(dev->of_node, "edp_ssc_mode", &value);
	if (!ret) {
		if ((value != 0) && (value != 1)) {
			EDP_WRN("Out of range, select from: 0-center_mode 1-downspread_mode\n");
			edp_core->ssc_mode = 0;
		} else
			edp_core->ssc_mode = value;
	} else
		edp_core->ssc_mode = 0;

	ret = of_property_read_u32(dev->of_node, "edp_psr_en", &value);
	if (!ret)
		edp_core->psr_en = value;
	else
		edp_core->psr_en = 0;

	ret = of_property_read_u32(dev->of_node, "edp_pixel_mode", &value);
	if (!ret)
		edp_core->pixel_mode = value;
	else
		edp_core->pixel_mode = 1;

	return RET_OK;
}


s32 edp_lane_para_parse(struct device *dev)
{
	s32 ret = -1;
	s32 value = 1;
	u32 i = 0;
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	u32 prop_val[4];
	u32 prop_len = 4;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->lane_para;

	ret = of_property_read_u32(dev->of_node, "edp_lane_rate", &value);
	if (!ret) {
		switch (value) {
		case 0:
			lane_para->bit_rate = BIT_RATE_1G62;
			break;
		case 1:
			lane_para->bit_rate = BIT_RATE_2G7;
			break;
		case 2:
			lane_para->bit_rate = BIT_RATE_5G4;
			break;
		default:
			EDP_ERR("edp_rate out of range!\n");
			return RET_FAIL;
		}
	}

	ret = of_property_read_u32(dev->of_node, "edp_lane_cnt", &value);
	if (!ret) {
		if ((value <= 0) || (value == 3) || (value > 4)) {
			EDP_ERR("edp_lane_cnt out of range!\n");
			return RET_FAIL;
		}
		lane_para->lane_cnt = value;
	}

	ret = of_property_read_u32(dev->of_node, "edp_colordepth", &value);
	if (!ret)
		lane_para->colordepth = value;
	else
		/* default use 8-bit */
		lane_para->colordepth = 8;

	ret = of_property_read_u32(dev->of_node, "edp_color_fmt", &value);
	if (!ret)
		lane_para->color_fmt = value;
	else
		/* default use RGB */
		lane_para->color_fmt = DISP_CSC_TYPE_RGB;

	if (lane_para->color_fmt == DISP_CSC_TYPE_RGB)
		lane_para->bpp = 3 * lane_para->colordepth;
	else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV444)
		lane_para->bpp = 3 * lane_para->colordepth;
	else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV422)
		lane_para->bpp = 2 * lane_para->colordepth;
	else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV420)
		lane_para->bpp = 3 * lane_para->colordepth / 2;

	ret = of_property_read_u32_array(dev->of_node, "lane_invert", prop_val, prop_len);
	if (ret == 0) {
		for (i = 0; i < 4; i++)
			lane_para->lane_invert[i] = prop_val[i];
	} else {
		for (i = 0; i < 4; i++)
			lane_para->lane_invert[i] = 0;
	}

	ret = of_property_read_u32_array(dev->of_node, "lane_remap", prop_val, prop_len);
	if (ret == 0) {
		for (i = 0; i < 4; i++)
			lane_para->lane_remap[i] = prop_val[i];
	} else {
		for (i = 0; i < 4; i++)
			lane_para->lane_remap[i] = i;
	}

	return RET_OK;
}

static ssize_t dpcd_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	int i = 0, ret = 0;
	char dpcd_rx_buf[576];
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	memset(&dpcd_rx_buf[0], 0, sizeof(dpcd_rx_buf));
	ret = edp_read_dpcd(edp_hw, &dpcd_rx_buf[0]);
	if (ret < 0)
		EDP_WRN("fail to read edp dpcd!\n");

	for (i = 0; i < sizeof(dpcd_rx_buf); i++) {
		if ((i % 0x10) == 0)
			count += sprintf(buf + count, "\n%02x:", i);
		count += sprintf(buf + count, "  %02x", dpcd_rx_buf[i]);
	}

	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t resource_lock_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug = &drm_edp->edp_debug;

	edp_debug->edp_res_lock = simple_strtoul(buf, NULL, 0);

	return count;
}

static ssize_t resource_lock_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug = &drm_edp->edp_debug;

	count += sprintf(buf + count, "resource_lock = %d\n", edp_debug->edp_res_lock);

	return count;
}

static ssize_t bypass_training_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug = &drm_edp->edp_debug;

	edp_debug->bypass_training = simple_strtoul(buf, NULL, 0);

	return count;
}

static ssize_t bypass_training_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug = &drm_edp->edp_debug;

	count += sprintf(buf + count, "bypass_training = %d\n", edp_debug->bypass_training);

	return count;
}

static ssize_t colorbar_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	int val;

	val = simple_strtoul(buf, NULL, 0);
	if (val < 0)
		val = 0;

	if (val > 7)
		val = 7;

	sunxi_tcon_show_pattern(drm_edp->sdrm.tcon_dev, val);
	return count;
}

static ssize_t colorbar_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;

	count += sprintf(buf + count, "0:DE  1:colorbar  2:grayscale  3:black_and_white\n");
	count += sprintf(buf + count, "4:all-0  5:all-1  6:reserved  7:gridding\n");

	return count;
}

static ssize_t loglevel_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	loglevel_debug = simple_strtoul(buf, NULL, 0);

	return count;
}

static ssize_t loglevel_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	if (!loglevel_debug)
		count += sprintf(buf + count, "0:NONE  1:EDP_DRV  2:EDP_CORE  4:EDP_LOW  8:EDP_EDID\n");
	else
		count += sprintf(buf + count, "loglevel_debug = %d\n", loglevel_debug);

	return count;
}

static ssize_t lane_debug_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	struct edp_lane_para *debug_lane_para;
	struct edp_lane_para *backup_lane_para;
	struct edp_debug *edp_debug;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->lane_para;
	debug_lane_para = &edp_core->debug_lane_para;
	backup_lane_para = &edp_core->backup_lane_para;

	edp_debug = &drm_edp->edp_debug;


	if (!strncmp(buf, "1", 1)) {
		/* only the debug_en turns 0 to 1 should restore lane config */
		if (edp_debug->lane_debug_en != 1) {
			edp_debug->lane_debug_en = 1;
			memcpy(backup_lane_para, lane_para, sizeof(struct edp_lane_para));
		}
		memcpy(lane_para, debug_lane_para, sizeof(struct edp_lane_para));
		drm_edp->use_debug_para = true;
	} else if (!strncmp(buf, "0", 1)) {
		if (edp_debug->lane_debug_en == 1) {
			edp_debug->lane_debug_en = 0;
			memcpy(lane_para, backup_lane_para, sizeof(struct edp_lane_para));
			memset(backup_lane_para, 0, sizeof(struct edp_lane_para));
			drm_edp->use_debug_para = false;
		} else {
			/* return if debug_en is already 0 */
			return count;
		}
	} else {
		EDP_WRN("Syntax error, only '0' or '1' support!\n");
		return count;
	}

	/* FIXME:TODO: need to trigger atomic_disable and enable */

	return count;
}

static ssize_t lane_debug_en_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	u32 count = 0;

	edp_debug = &drm_edp->edp_debug;

	count += sprintf(buf + count, "lane_debug_en: %d\n", edp_debug->lane_debug_en);

	return count;
}

static ssize_t lane_invert_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	u8 *separator = NULL;
	u32 i, value;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->debug_lane_para;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		for (i = 0; i < 4; i++) {
			if (i == 0) {
				value = simple_strtoul(buf, NULL, 0);
				lane_para->lane_invert[i] = value;
				continue;
			} else {
				value = simple_strtoul(separator + 1, NULL, 0);
				lane_para->lane_invert[i] = value;
			}

			separator = strchr(separator + 1, ' ');
			if (separator == NULL)
				break;
		}
	}

	return count;
}

static ssize_t lane_invert_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	count += sprintf(buf + count, "echo [ch0_invert ch1_invert ch2_invert ch3_invert] > lane_invert_debug\n");
	count += sprintf(buf + count, "eg: echo 0 0 0 0  > lane_invert_debug\n");

	return count;
}

static ssize_t lane_remap_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	u8 *separator = NULL;
	u32 i, value;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->debug_lane_para;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		for (i = 0; i < 4; i++) {
			if (i == 0) {
				value = simple_strtoul(buf, NULL, 0);
				lane_para->lane_remap[i] = value;
				continue;
			} else {
				value = simple_strtoul(separator + 1, NULL, 0);
				lane_para->lane_remap[i] = value;
			}

			lane_para->lane_remap[i] = value;

			separator = strchr(separator + 1, ' ');
			if (separator == NULL)
				break;
		}
	}

	return count;
}

static ssize_t lane_remap_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	count += sprintf(buf + count, "echo [ch0_remap ch1_remap ch2_remap ch3_remap] > lane_remap_debug\n");
	count += sprintf(buf + count, "eg: echo 0 1 2 3  > lane_remap_debug\n");

	return count;
}

static ssize_t lane_fmt_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	u8 *separator = NULL;
	u32 color_fmt = 0;
	u32 color_bits = 0;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->debug_lane_para;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		color_fmt = simple_strtoul(buf, NULL, 0);
		color_bits = simple_strtoul(separator + 1, NULL, 0);

		if (color_fmt > 2) {
			EDP_WRN("color format should select from: 0:RGB  1:YUV444  2:YUV422\n");
			return count;
		}

		if ((color_bits != 6) && (color_bits != 8) && \
		    (color_bits != 10) && (color_bits != 12) && (color_bits != 16)) {
			EDP_WRN("lane rate should select from: 6/8/10/12/16\n");
			return count;
		}

		lane_para->color_fmt = color_fmt;
		lane_para->colordepth = color_bits;

		if (lane_para->color_fmt == DISP_CSC_TYPE_RGB)
			lane_para->bpp = 3 * lane_para->colordepth;
		else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV444)
			lane_para->bpp = 3 * lane_para->colordepth;
		else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV422)
			lane_para->bpp = 2 * lane_para->colordepth;
		else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV420)
			lane_para->bpp = 3 * lane_para->colordepth / 2;
	}

	return count;
}

static ssize_t lane_fmt_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	count += sprintf(buf + count, "echo [coolor_fmt color_bits] > lane_fmt_debug\n");
	count += sprintf(buf + count, "eg: echo 0 8 > lane_fmt_debug\n");

	return count;
}

static ssize_t lane_cfg_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	u8 *separator = NULL;
	u32 lane_cnt = 0;
	u32 lane_rate = 0;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->debug_lane_para;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		lane_cnt = simple_strtoul(buf, NULL, 0);
		lane_rate = simple_strtoul(separator + 1, NULL, 0);

		if ((lane_cnt != 1) && (lane_cnt != 2) && (lane_cnt != 4)) {
			EDP_WRN("lane cnt should select from: 1/2/4\n");
			return count;
		}

		if ((lane_rate != 162) && (lane_rate != 270) && (lane_rate != 540)) {
			EDP_WRN("lane rate should select from: 162/270/540\n");
			return count;
		}

		lane_para->lane_cnt = lane_cnt;
		lane_para->bit_rate = (u64)lane_rate * 10000000;
	}

	return count;
}

static ssize_t lane_cfg_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	count += sprintf(buf + count, "echo [lane_cnt lane_rate] > lane_cfg_debug\n");
	count += sprintf(buf + count, "eg: echo 4 270 > lane_cfg_debug\n");

	return count;
}

static ssize_t edid_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct edp_tx_core *edp_core = NULL;
	struct edid *edid = NULL;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_drm_device *sdrm = &drm_edp->sdrm;

	if (!strncmp(buf, "1", 1)) {
		edp_core = &drm_edp->edp_core;

		/* FIXME:TODO: add edid correct when edid_corrupt occur */
		edid = drm_do_get_edid(&sdrm->connector, edp_get_edid_block, &drm_edp->edp_hw);
		if (edid == NULL) {
			EDP_WRN("fail to read edid\n");
			return count;
		}

		edp_parse_edid(drm_edp, edid);
	} else {
		EDP_WRN("syntax error, try 'echo 1 > edid'!\n");
	}

	/* release the previous one edid ptr */
	if (edp_core->edid)
		edp_edid_put(edp_core->edid);
	edp_core->edid = edid;

	return count;
}

static ssize_t edid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edp_tx_core *edp_core;
	struct edid *edid;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	const u8 *cea;
	u32 count = 0;
	s32 i, edid_lenth;
	int ext_index = 0;

	if (!drm_edp->hpd_state) {
		count += sprintf(buf + count, "[EDP] error: sink is plugout!\n");
		return count;
	}

	edp_core = &drm_edp->edp_core;
	edid = edp_core->edid;

	if (!edid) {
		count += sprintf(buf + count, "[EDP] error: edid read uncorrectly!\
				 Try to read edid manaually by 'echo 1 > edid'\n");
		return count;
	}

	cea = sunxi_drm_find_edid_extension(edid, CEA_EXT, &ext_index);
	if (!cea)
		edid_lenth = 128;
	 else
		edid_lenth = 256;


	for (i = 0; i < edid_lenth; i++) {
		if ((i % 0x8) == 0)
			count += sprintf(buf + count, "\n%02x:", i);
		count += sprintf(buf + count, "  %02x", *((char *)(edid) + i));
	}

	count += sprintf(buf + count, "\n");

	return count;
}


static ssize_t hotplug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);

	if (drm_edp->hpd_state)
		count += sprintf(buf + count, "HPD State: Plugin\n");
	else
		count += sprintf(buf + count, "HPD State: Plugout\n");

	return count;
}

static ssize_t sink_info_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edp_rx_cap *sink_cap;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	u32 count = 0;

	if (!drm_edp->hpd_state) {
		count += sprintf(buf + count, "[EDP] error: sink is plugout!\n");
		return count;
	}

	sink_cap = &drm_edp->sink_cap;

	if (!sink_cap) {
		count += sprintf(buf + count, "[EDP] eDP sink capacity unknown\n");
		return count;
	}

	count += sprintf(buf + count, "[Capacity Info]\n");
	count += sprintf(buf + count, "dpcd_rev: %d\n", sink_cap->dpcd_rev);
	count += sprintf(buf + count, "is_edp_device: %s\n", sink_cap->is_edp_device ? "Yes" : "No");
	count += sprintf(buf + count, "max_bit_rate: %lld\n", sink_cap->max_rate);
	count += sprintf(buf + count, "max_lane_cnt: %d\n", sink_cap->max_lane);
	count += sprintf(buf + count, "tps3_support: %s\n", sink_cap->tps3_support ? "Yes" : "No");
	count += sprintf(buf + count, "fast_link_train_support: %s\n", sink_cap->fast_train_support ? "Yes" : "No");
	count += sprintf(buf + count, "downstream_port_support: %s\n", sink_cap->downstream_port_support ? "Yes" : "No");

	if (sink_cap->downstream_port_support) {
		switch (sink_cap->downstream_port_type) {
		case 0:
			count += sprintf(buf + count, "downstream_port_type: DP\n");
			break;
		case 1:
			count += sprintf(buf + count, "downstream_port_type: VGA\n");
			break;
		case 2:
			count += sprintf(buf + count, "downstream_port_type: DVI/HDMI/DP++\n");
			break;
		case 3:
		default:
			count += sprintf(buf + count, "downstream_port_type: others\n");
			break;
		}

		count += sprintf(buf + count, "downstream_port_cnt: %d\n", sink_cap->downstream_port_cnt);
	} else {
		count += sprintf(buf + count, "downstream_port_type: NULL\n");
		count += sprintf(buf + count, "downstream_port_cnt: NULL\n");
	}

	count += sprintf(buf + count, "local_edid_support: %s\n", sink_cap->local_edid_support ? "Yes" : "No");
	count += sprintf(buf + count, "assr_support: %s\n", sink_cap->assr_support ? "Yes" : "No");
	count += sprintf(buf + count, "enhance_frame_support: %s\n", sink_cap->enhance_frame_support ? "Yes" : "No");
	count += sprintf(buf + count, "\n");

	/*edid info*/
	count += sprintf(buf + count, "[Edid Info]\n");
	count += sprintf(buf + count, "mfg_year: %d\n", sink_cap->mfg_year + 1990);
	count += sprintf(buf + count, "mfg_week: %d\n", sink_cap->mfg_week);
	count += sprintf(buf + count, "edid_ver: %d\n", sink_cap->edid_ver);
	count += sprintf(buf + count, "edid_rev: %d\n", sink_cap->edid_rev);
	count += sprintf(buf + count, "width_cm: %d\n", sink_cap->width_cm);
	count += sprintf(buf + count, "height_cm: %d\n", sink_cap->height_cm);
	count += sprintf(buf + count, "input_type: %s\n", sink_cap->input_type ? "Digital" : "Analog");
	count += sprintf(buf + count, "input_bit_depth: %d\n", sink_cap->bit_depth);

	switch (sink_cap->video_interface) {
	case DRM_EDID_DIGITAL_TYPE_UNDEF:
		count += sprintf(buf + count, "sink_video_interface: Undefined\n");
		break;
	case DRM_EDID_DIGITAL_TYPE_DVI:
		count += sprintf(buf + count, "sink_video_interface: DVI\n");
		break;
	case DRM_EDID_DIGITAL_TYPE_HDMI_A:
		count += sprintf(buf + count, "sink_video_interface: HDMIa\n");
		break;
	case DRM_EDID_DIGITAL_TYPE_HDMI_B:
		count += sprintf(buf + count, "sink_video_interface: HDMIb\n");
		break;
	case DRM_EDID_DIGITAL_TYPE_MDDI:
		count += sprintf(buf + count, "sink_video_interface: MDDI\n");
		break;
	case DRM_EDID_DIGITAL_TYPE_DP:
		count += sprintf(buf + count, "sink_video_interface: DP/eDP\n");
		break;
	}

	count += sprintf(buf + count, "Ycc444_support: %s\n", sink_cap->Ycc444_support ? "Yes" : "No");
	count += sprintf(buf + count, "Ycc422_support: %s\n", sink_cap->Ycc422_support ? "Yes" : "No");
	count += sprintf(buf + count, "Ycc420_support: %s\n", sink_cap->Ycc420_support ? "Yes" : "No");
	count += sprintf(buf + count, "audio_support: %s\n", sink_cap->audio_support ? "Yes" : "No");

	return count;
}

static ssize_t source_info_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edp_tx_core *edp_core;
	struct edp_tx_cap *src_cap;
	struct edp_lane_para tmp_lane_para;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	u32 count = 0;
	u32 valid_symbol;
	char color_fmt[10];

	edp_core = &drm_edp->edp_core;
	src_cap = &drm_edp->source_cap;

	memset(&tmp_lane_para, 0, sizeof(struct edp_lane_para));

	count += sprintf(buf + count, "[Capacity Info]\n");
	count += sprintf(buf + count, "Max lane Support: %d\n",\
			 src_cap->max_lane);
	count += sprintf(buf + count, "Max Rate Support: %lld\n",\
			 src_cap->max_rate);
	count += sprintf(buf + count, "TPS3 Support: %s\n",\
			 src_cap->tps3_support ? "Yes" : "No");
	count += sprintf(buf + count, "Fast Train Support: %s\n",\
			 src_cap->fast_train_support ? "Yes" : "No");
	count += sprintf(buf + count, "Audio Support: %s\n",\
			 src_cap->audio_support ? "Yes" : "No");
	count += sprintf(buf + count, "SSC Support: %s\n",\
			 src_cap->ssc_support ? "Yes" : "No");
	count += sprintf(buf + count, "ASSR Support: %s\n",\
			 src_cap->assr_support ? "Yes" : "No");
	count += sprintf(buf + count, "PSR Support: %s\n",\
			 src_cap->psr_support ? "Yes" : "No");
	count += sprintf(buf + count, "MST Support: %s\n",\
			 src_cap->mst_support ? "Yes" : "No");
	count += sprintf(buf + count, "Enhance Frame Support: %s\n\n",\
			 src_cap->enhance_frame_support ? "Yes" : "No");


	edp_hw_get_lane_para(edp_hw, &tmp_lane_para);

	count += sprintf(buf + count, "[Link Info]\n");
	count += sprintf(buf + count, "Pixel Mode: %d\n",\
			 edp_hw_get_pixel_mode(edp_hw));
	count += sprintf(buf + count, "Bit Rate: %lld\n",\
			 tmp_lane_para.bit_rate);
	count += sprintf(buf + count, "Lane Count: %d\n",\
			 tmp_lane_para.lane_cnt);
	count += sprintf(buf + count, "Pixel Clock: %d\n",\
			 edp_hw_get_pixclk(edp_hw));
	count += sprintf(buf + count, "TU Size: %d\n",\
			 edp_hw_get_tu_size(edp_hw));
	valid_symbol = edp_hw_get_valid_symbol_per_tu(edp_hw);
	count += sprintf(buf + count, "Valid Symbol Per TU: %d.%d\n",\
			 valid_symbol / 10, valid_symbol % 10);

	switch (edp_hw_get_color_fmt(edp_hw)) {
	case RGB_6BIT:
		snprintf(color_fmt, sizeof(color_fmt), "RGB_6BIT");
		break;
	case RGB_8BIT:
		snprintf(color_fmt, sizeof(color_fmt), "RGB_8BIT");
		break;
	case RGB_10BIT:
		snprintf(color_fmt, sizeof(color_fmt), "RGB_10BIT");
		break;
	case RGB_12BIT:
		snprintf(color_fmt, sizeof(color_fmt), "RGB_12BIT");
		break;
	case RGB_16BIT:
		snprintf(color_fmt, sizeof(color_fmt), "RGB_16BIT");
		break;
	case YCBCR444_8BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR444_8BIT");
		break;
	case YCBCR444_10BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR444_10BIT");
		break;
	case YCBCR444_12BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR444_12BIT");
		break;
	case YCBCR444_16BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR444_16BIT");
		break;
	case YCBCR422_8BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR422_8BIT");
		break;
	case YCBCR422_10BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR422_10BIT");
		break;
	case YCBCR422_12BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR422_12BIT");
		break;
	case YCBCR422_16BIT:
		snprintf(color_fmt, sizeof(color_fmt), "YCBCR422_16BIT");
		break;
	default:
		snprintf(color_fmt, sizeof(color_fmt), "Unknown");
		break;
	}
	count += sprintf(buf + count, "Color Format: %s\n", color_fmt);

	if (src_cap->ssc_support) {
		if (edp_core->ssc_en) {
			count += sprintf(buf + count, "Ssc En: %s\n",\
				edp_hw_ssc_is_enabled(edp_hw) ? "Enable" : "Disable");
			if (edp_hw_ssc_is_enabled(edp_hw))
				count += sprintf(buf + count, "Ssc Mode: %s\n", \
					 edp_hw_ssc_get_mode(edp_hw) ? "Downspread" : "Center");
		}
	}

	if (src_cap->psr_support) {
		if (edp_core->psr_en)
			count += sprintf(buf + count, "Psr En: %s\n",\
					 edp_hw_psr_is_enabled(edp_hw) ? "Enable" : "Disable");
	}

	count += sprintf(buf + count, "\n");

	count += sprintf(buf + count, "[Training Info]\n");
	count += sprintf(buf + count, "Voltage Swing0: Level-%d\n",\
			 tmp_lane_para.lane_sw[0]);
	count += sprintf(buf + count, "Pre Emphasis0:  Level-%d\n",\
			 tmp_lane_para.lane_pre[0]);
	count += sprintf(buf + count, "Voltage Swing1: Level-%d\n",\
			 tmp_lane_para.lane_sw[1]);
	count += sprintf(buf + count, "Pre Emphasis1:  Level-%d\n",\
			 tmp_lane_para.lane_pre[1]);
	count += sprintf(buf + count, "Voltage Swing2: Level-%d\n",\
			 tmp_lane_para.lane_sw[2]);
	count += sprintf(buf + count, "Pre Emphasis2:  Level-%d\n",\
			 tmp_lane_para.lane_pre[2]);
	count += sprintf(buf + count, "Voltage Swing3: Level-%d\n",\
			 tmp_lane_para.lane_sw[3]);
	count += sprintf(buf + count, "Pre Emphasis3:  Level-%d\n",\
			 tmp_lane_para.lane_pre[3]);

	count += sprintf(buf + count, "\n");

	count += sprintf(buf + count, "[Audio Info]\n");
	if (!edp_hw_audio_is_enabled(edp_hw)) {
		count += sprintf(buf + count, "Audio En: Disable\n");
		return count;
	}

	count += sprintf(buf + count, "Audio En: Enable\n");
	count += sprintf(buf + count, "Audio Interface: %s\n", edp_hw_get_audio_if(edp_hw) ?\
				 "I2S" : "SPDIF");
	count += sprintf(buf + count, "Mute: %s\n", edp_hw_audio_is_mute(edp_hw) ?\
				 "Yes" : "No");
	count += sprintf(buf + count, "Channel Count: %d\n",\
			 edp_hw_get_audio_chn_cnt(edp_hw));
	count += sprintf(buf + count, "Data Width: %d bits\n",\
			 edp_hw_get_audio_data_width(edp_hw));

	return count;
}

static ssize_t aux_read_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	u32 i, times = 0, show_cnt = 0;
	unsigned long start_reg = 0;
	unsigned long end_reg = 0;
	unsigned long len;
	char tmp_rx_buf[256];
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_debug = &drm_edp->edp_debug;

	if ((edp_debug->aux_read_start == 0) \
		 && (edp_debug->aux_read_end) == 0)
		return sprintf(buf, "%s\n", "echo [0x(start_addr), 0x(end_addr)] > aux_read");


	start_reg = edp_debug->aux_read_start;
	end_reg = edp_debug->aux_read_end;

	if (end_reg < start_reg) {
		return sprintf(buf, "%s,%d err, addresss syntax error!\n", __func__, __LINE__);
	}

	if (start_reg > 0x70000) {
		return sprintf(buf, "%s,%d err, addresss out of range define in eDP spec!\n", __func__, __LINE__);
	}

	len = end_reg - start_reg;
	if (len > 256)
		return sprintf(buf, "%s,%d err, out of length, length should <= 256!\n", __func__, __LINE__);

	memset(tmp_rx_buf, 0, sizeof(tmp_rx_buf));

	if (edp_hw_aux_read(edp_hw, start_reg, len, &tmp_rx_buf[0]) < 0)
		return sprintf(buf, "aux read fail!\n");

	show_cnt += sprintf(buf, "[AUX_READ] Addr:0x%04lx   Lenth:%ld", start_reg, len + 1);
	if ((start_reg % 0x8) == 0)
		times = 1;

	for (i = 0; i < (len + 1); i++) {
		if ((times == 0) && (((start_reg + i) % 0x8) != 0)) {
			show_cnt += sprintf(buf + show_cnt, "\n0x%04lx:", (start_reg + i));
			times = 1;
		}

		if (((start_reg + i) % 0x8) == 0)
			show_cnt += sprintf(buf + show_cnt, "\n0x%04lx:", (start_reg + i));
		show_cnt += sprintf(buf + show_cnt, "  0x%02x", tmp_rx_buf[i]);
	}

	show_cnt += sprintf(buf + show_cnt, "\n");

	return show_cnt;


}

static ssize_t phy_read_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	unsigned long start_reg = 0;
	unsigned long end_reg = 0;

	if (__parse_dump_str(buf, count, &start_reg, &end_reg)) {
		EDP_WRN("%s,%d err, invalid para!\n", __func__, __LINE__);
		return count;
	}

	if (end_reg < start_reg) {
		EDP_WRN("%s,%d err, end address should larger than start address!\n", __func__, __LINE__);
		return count;
	}

	edp_debug = &drm_edp->edp_debug;

	edp_debug->aux_read_start = start_reg;
	edp_debug->aux_read_end = end_reg;

	return count;
}


static ssize_t phy_read_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	u32 i, times = 0, show_cnt = 0;
	unsigned long start_reg = 0;
	unsigned long end_reg = 0;
	unsigned long len;
	char tmp_rx_buf[256];
	void __iomem *phy;

	edp_debug = &drm_edp->edp_debug;

	if ((edp_debug->aux_read_start == 0) \
		 && (edp_debug->aux_read_end) == 0)
		return sprintf(buf, "%s\n", "echo [0x(start_addr), 0x(end_addr)] > aux_read");


	start_reg = edp_debug->aux_read_start;
	end_reg = edp_debug->aux_read_end;

	if (end_reg < start_reg) {
		return sprintf(buf, "%s,%d err, addresss syntax error!\n", __func__, __LINE__);
	}

	len = end_reg - start_reg;
	phy = ioremap(start_reg, len * 2);

	memset(tmp_rx_buf, 0, 256);

	for (i = 0; i < len; i++) {
		tmp_rx_buf[i] = readw(phy + i * 2);
	}

	show_cnt += sprintf(buf, "[PHY_READ] Addr:0x%04lx   Lenth:%ld", start_reg, len + 1);
	if ((start_reg % 0x10) == 0)
		times = 1;

	for (i = 0; i < (len + 1); i++) {
		if ((times == 0) && (((start_reg + i * 2) % 0x10) != 0)) {
			show_cnt += sprintf(buf + show_cnt, "\n0x%04lx:", (start_reg + i * 2));
			times = 1;
		}

		if (((start_reg + i * 2) % 0x10) == 0)
			show_cnt += sprintf(buf + show_cnt, "\n0x%04lx:", (start_reg + i * 2));
		show_cnt += sprintf(buf + show_cnt, "  0x%02x", tmp_rx_buf[i]);
	}

	show_cnt += sprintf(buf + show_cnt, "\n");
	iounmap(phy);

	return show_cnt;

}


static ssize_t phy_write_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 value = 0;
	u8 *separator = NULL;
	u32 reg_addr = 0;
	void __iomem *phy;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		reg_addr = simple_strtoul(buf, NULL, 0);
		value = simple_strtoul(separator + 1, NULL, 0);

		phy = ioremap(reg_addr, 0x10);
		writew(value, phy);
		iounmap(phy);
	}

	return count;
}


static ssize_t aux_read_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	unsigned long start_reg = 0;
	unsigned long end_reg = 0;

	if (__parse_dump_str(buf, count, &start_reg, &end_reg)) {
		EDP_WRN("%s,%d err, invalid para!\n", __func__, __LINE__);
		return count;
	}

	if (end_reg < start_reg) {
		EDP_WRN("%s,%d err, end address should larger than start address!\n", __func__, __LINE__);
		return count;
	}

	if (start_reg > 0x70000) {
		EDP_WRN("%s,%d err, addresss out of range define in eDP spec!\n", __func__, __LINE__);
		return count;
	}

	edp_debug = &drm_edp->edp_debug;

	edp_debug->aux_read_start = start_reg;
	edp_debug->aux_read_end = end_reg;

	return count;
}

static ssize_t aux_write_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	char tmp_rx_buf[16];
	u8 regval_after[16], i;
	u32 show_cnt = 0;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_debug = &drm_edp->edp_debug;

	if (edp_debug->aux_write_len == 0)
		return sprintf(buf, "%s\n", "echo [0x(address) 0x(value)] > aux_write");

	memset(tmp_rx_buf, 0, sizeof(tmp_rx_buf));
	edp_hw_aux_read(edp_hw, edp_debug->aux_write_start, edp_debug->aux_write_len, tmp_rx_buf);
	show_cnt += sprintf(buf, "[AUX_WRITE]  Lenth:%d\n", edp_debug->aux_write_len);

	for (i = 0; i < edp_debug->aux_write_len; i++) {
		regval_after[i] = tmp_rx_buf[i];

		show_cnt += sprintf(buf + show_cnt, \
			    "[0x%02x]:val_before:0x%02x  val_wants:0x%02x  val_after:0x%02x\n", \
				edp_debug->aux_write_start + i, edp_debug->aux_write_val_before[i], \
				edp_debug->aux_write_val[i], regval_after[i]);
	}

	return show_cnt;

}

static ssize_t aux_write_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	u8 value = 0, i = 0;
	u8 *separator = NULL;
	char tmp_tx_buf[16];
	char tmp_rx_buf[16];
	u32 reg_addr = 0;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		edp_debug = &drm_edp->edp_debug;

		memset(tmp_tx_buf, 0, sizeof(tmp_tx_buf));
		memset(tmp_rx_buf, 0, sizeof(tmp_rx_buf));
		reg_addr = simple_strtoul(buf, NULL, 0);
		edp_debug->aux_write_start = reg_addr;
		edp_debug->aux_write_len = 0;

		for (i = 0; i < 16; i++) {
			value = simple_strtoul(separator + 1, NULL, 0);

			tmp_tx_buf[i] = value;

			edp_debug->aux_write_val[i] = value;
			edp_debug->aux_write_len++;

			separator = strchr(separator + 1, ' ');
			if (separator == NULL)
				break;
		}

		edp_hw_aux_read(edp_hw, reg_addr, edp_debug->aux_write_len, tmp_rx_buf);
		for (i = 0; i < edp_debug->aux_write_len; i++) {
			edp_debug->aux_write_val_before[i] = tmp_rx_buf[i];
		}

		mdelay(1);
		edp_hw_aux_write(edp_hw, reg_addr, edp_debug->aux_write_len, &tmp_tx_buf[0]);

	}

	return count;

}

static ssize_t aux_i2c_read_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	u32 i, show_cnt = 0;
	unsigned long i2c_addr = 0;
	unsigned long len;
	char tmp_rx_buf[256];
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_debug = &drm_edp->edp_debug;

	memset(tmp_rx_buf, 0, sizeof(tmp_rx_buf));

	if ((edp_debug->aux_i2c_addr == 0) || (edp_debug->aux_i2c_len) == 0)
		return sprintf(buf, "%s\n", "echo [0x(i2c_addr), 0x(lenth)] > aux_i2c_read");

	i2c_addr = edp_debug->aux_i2c_addr;
	len = edp_debug->aux_i2c_len;

#if IS_ENABLED(CONFIG_AW_DRM_INNO_EDP13)
	edp_hw_aux_i2c_write(edp_hw, i2c_addr, 0, 1, &tmp_rx_buf[0]);
#endif

	show_cnt += sprintf(buf, "[AUX_I2C_READ] I2C_Addr:0x%04lx   Lenth:%ld", i2c_addr, len);

	if (edp_hw_aux_i2c_read(edp_hw, i2c_addr, 0, len, &tmp_rx_buf[0]) < 0)
		return sprintf(buf, "aux_i2c_read fail!\n");

	for (i = 0; i < len; i++) {
		if ((i % 0x8) == 0)
			show_cnt += sprintf(buf + show_cnt, "\n0x%02x:", i);
		show_cnt += sprintf(buf + show_cnt, "  0x%02x", tmp_rx_buf[i]);
	}


	show_cnt += sprintf(buf + show_cnt, "\n");

	return show_cnt;


}

static ssize_t aux_i2c_read_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;
	unsigned long i2c_addr = 0;
	unsigned long lenth = 0;

	if (__parse_dump_str(buf, count, &i2c_addr, &lenth)) {
		EDP_WRN("%s err, invalid param, line:%d!\n", __func__, __LINE__);
		return count;
	}

	if ((lenth <= 0) || (lenth > 256)) {
		EDP_WRN("%s aux i2c read lenth should between 0 and 256!\n", __func__);
		return count;
	}

	edp_debug = &drm_edp->edp_debug;

	edp_debug->aux_i2c_addr = i2c_addr;
	edp_debug->aux_i2c_len = lenth;

	return count;
}

static ssize_t lane_config_now_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	u32 count = 0;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->lane_para;

	if (drm_edp->use_debug_para)
		count += sprintf(buf + count, "lane_para_use: debug\n");
	else
		count += sprintf(buf + count, "lane_para_use: user\n");

	count += sprintf(buf + count, "bit_rate:    %lld\n", lane_para->bit_rate);
	count += sprintf(buf + count, "lane_cnt:    %d\n", lane_para->lane_cnt);
	count += sprintf(buf + count, "color depth: %d\n", lane_para->colordepth);
	count += sprintf(buf + count, "color fmt:   %d\n", lane_para->color_fmt);
	count += sprintf(buf + count, "bpp:         %d\n", lane_para->bpp);
	count += sprintf(buf + count, "ch_invert[0]:%d\n", lane_para->lane_invert[0]);
	count += sprintf(buf + count, "ch_invert[1]:%d\n", lane_para->lane_invert[1]);
	count += sprintf(buf + count, "ch_invert[2]:%d\n", lane_para->lane_invert[2]);
	count += sprintf(buf + count, "ch_invert[3]:%d\n", lane_para->lane_invert[3]);
	count += sprintf(buf + count, "ch_remap[0]: %d\n", lane_para->lane_remap[0]);
	count += sprintf(buf + count, "ch_remap[1]: %d\n", lane_para->lane_remap[1]);
	count += sprintf(buf + count, "ch_remap[2]: %d\n", lane_para->lane_remap[2]);
	count += sprintf(buf + count, "ch_remap[3]: %d\n", lane_para->lane_remap[3]);
	return count;
}

static ssize_t lane_config_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edp_tx_core *edp_core;
	struct edp_lane_para *lane_para;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	u32 count = 0;

	edp_core = &drm_edp->edp_core;
	lane_para = &edp_core->debug_lane_para;

	count += sprintf(buf + count, "bit_rate:    %lld\n", lane_para->bit_rate);
	count += sprintf(buf + count, "lane_cnt:    %d\n", lane_para->lane_cnt);
	count += sprintf(buf + count, "color depth: %d\n", lane_para->colordepth);
	count += sprintf(buf + count, "color fmt:   %d\n", lane_para->color_fmt);
	count += sprintf(buf + count, "bpp:         %d\n", lane_para->bpp);
	count += sprintf(buf + count, "ch_invert[0]:%d\n", lane_para->lane_invert[0]);
	count += sprintf(buf + count, "ch_invert[1]:%d\n", lane_para->lane_invert[1]);
	count += sprintf(buf + count, "ch_invert[2]:%d\n", lane_para->lane_invert[2]);
	count += sprintf(buf + count, "ch_invert[3]:%d\n", lane_para->lane_invert[3]);
	count += sprintf(buf + count, "ch_remap[0]: %d\n", lane_para->lane_remap[0]);
	count += sprintf(buf + count, "ch_remap[1]: %d\n", lane_para->lane_remap[1]);
	count += sprintf(buf + count, "ch_remap[2]: %d\n", lane_para->lane_remap[2]);
	count += sprintf(buf + count, "ch_remap[3]: %d\n", lane_para->lane_remap[3]);

	return count;
}

static ssize_t timings_now_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edp_tx_core *edp_core;
	struct disp_video_timings *timings;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	u32 count = 0;
	u32 fps;

	edp_core = &drm_edp->edp_core;
	timings = &edp_core->timings;

	fps = (timings->pixel_clk / timings->hor_total_time) / timings->ver_total_time;
	count += sprintf(buf + count, "fps: %d\n", fps);
	count += sprintf(buf + count, "vic: %d\n", timings->vic);
	count += sprintf(buf + count, "tv_mode: %d\n", timings->tv_mode);
	count += sprintf(buf + count, "pixel_clk: %d\n", timings->pixel_clk);
	count += sprintf(buf + count, "pixel_repeat: %d\n", timings->pixel_repeat);
	count += sprintf(buf + count, "x_res: %d\n", timings->x_res);
	count += sprintf(buf + count, "y_res: %d\n", timings->y_res);
	count += sprintf(buf + count, "hor_total_time: %d\n", timings->hor_total_time);
	count += sprintf(buf + count, "hor_back_porch: %d\n", timings->hor_back_porch);
	count += sprintf(buf + count, "hor_front_porch: %d\n", timings->hor_front_porch);
	count += sprintf(buf + count, "hor_sync_time: %d\n", timings->hor_sync_time);
	count += sprintf(buf + count, "ver_total_time: %d\n", timings->ver_total_time);
	count += sprintf(buf + count, "ver_back_porch: %d\n", timings->ver_back_porch);
	count += sprintf(buf + count, "ver_front_porch: %d\n", timings->ver_front_porch);
	count += sprintf(buf + count, "ver_sync_time: %d\n", timings->ver_sync_time);
	count += sprintf(buf + count, "hor_sync_polarity: %d\n", timings->hor_sync_polarity);
	count += sprintf(buf + count, "ver_sync_polarity: %d\n", timings->ver_sync_polarity);
	count += sprintf(buf + count, "b_interlace: %d\n", timings->b_interlace);
	count += sprintf(buf + count, "trd_mode: %d\n", timings->trd_mode);
	count += sprintf(buf + count, "dclk_rate_set: %ld\n", timings->dclk_rate_set);
	count += sprintf(buf + count, "frame_period: %lld\n", timings->frame_period);
	count += sprintf(buf + count, "start_delay: %d\n", timings->start_delay);

	return count;
}

static ssize_t mode_list_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct drm_connector *connector = &drm_edp->sdrm.connector;
	struct drm_display_mode *mode;
	struct disp_video_timings timings;
	u32 fps;

	if (!connector)
		return count;

	if (list_empty(&connector->modes))
		return count;

	count += sprintf(buf + count, "[width x height]@fps  pclk");
	count += sprintf(buf + count, "x ht hbp hfp hsw y vt vbp vfp vsw hpol vpol type\n");
	list_for_each_entry(mode, &connector->modes, head) {

		drm_mode_to_sunxi_video_timings(mode, &timings);
		fps = (timings.pixel_clk / timings.hor_total_time) / timings.ver_total_time;
		count += sprintf(buf + count, "[%s]@%d     %d     ", mode->name, fps, timings.pixel_clk);


		count += sprintf(buf + count, "%d ", timings.x_res);
		count += sprintf(buf + count, "%d ", timings.hor_total_time);
		count += sprintf(buf + count, "%d ", timings.hor_back_porch);
		count += sprintf(buf + count, "%d ", timings.hor_front_porch);
		count += sprintf(buf + count, "%d ", timings.hor_sync_time);
		count += sprintf(buf + count, "%d ", timings.y_res);
		count += sprintf(buf + count, "%d ", timings.ver_total_time);
		count += sprintf(buf + count, "%d ", timings.ver_back_porch);
		count += sprintf(buf + count, "%d ", timings.ver_front_porch);
		count += sprintf(buf + count, "%d ", timings.ver_sync_time);
		count += sprintf(buf + count, "%d ", timings.hor_sync_polarity);
		count += sprintf(buf + count, "%d ", timings.ver_sync_polarity);

		if (mode->type & DRM_MODE_TYPE_BUILTIN)
			count += sprintf(buf + count, "BUILDIN ");
		if (mode->type & DRM_MODE_TYPE_PREFERRED)
			count += sprintf(buf + count, "PREFERED ");
		if (mode->type & DRM_MODE_TYPE_DEFAULT)
			count += sprintf(buf + count, "DEFAULT ");
		if (mode->type & DRM_MODE_TYPE_USERDEF)
			count += sprintf(buf + count, "USERDEF ");
		if (mode->type & DRM_MODE_TYPE_DRIVER)
			count += sprintf(buf + count, "DRIVER ");
		count += sprintf(buf + count, "\n");
	}

	return count;
}

static ssize_t hpd_mask_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;

	edp_debug = &drm_edp->edp_debug;

	return sprintf(buf, "0x%x\n", edp_debug->hpd_mask);
}

static ssize_t hpd_mask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long val;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_debug *edp_debug;

	if (count < 1)
		return -EINVAL;

	err = kstrtoul(buf, 16, &val);
	if (err) {
		EDP_WRN("%s syntax error!\n", __func__);
		return count;
	}

	if ((val != 0x0) && (val != 0x10) && (val != 0x11) && (val != 0x110) && \
	    (val != 0x111) && (val != 0x1010) && (val != 0x1011)) {
		EDP_WRN("unavailable param, select from: 0x0/0x10/0x11/0x110/0x111/0x1010/0x1011 !\n");
		return count;
	}

	edp_debug = &drm_edp->edp_debug;

	edp_debug->hpd_mask = val;

	return count;
}

static ssize_t training_param_type_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);

	return sprintf(buf, "training_param_type: %d\n", drm_edp->edp_core.training_param_type);
}

static ssize_t training_param_type_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long val;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);

	if (count < 1)
		return -EINVAL;

	err = kstrtoul(buf, 16, &val);
	if (err) {
		EDP_WRN("%s syntax error!\n", __func__);
		return count;
	}

	if ((val != 0) && (val != 1) && (val != 2)) {
		EDP_WRN("training param type out of range, select from: 0/1/2\n");
		return count;
	}

	drm_edp->edp_core.training_param_type = val;

	return count;
}

static ssize_t pattern_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u32 pattern_debug = 0;
	u32 test_lane_cnt;
	u8 *separator = NULL;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		pattern_debug = simple_strtoul(buf, NULL, 0);
		test_lane_cnt = simple_strtoul(separator + 1, NULL, 0);
	}

	edp_hw_set_pattern(edp_hw, pattern_debug, test_lane_cnt);

	return count;
}

static ssize_t pattern_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	s32 pattern = 0;
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	pattern = edp_hw_get_pattern(edp_hw);
	switch (pattern) {
	case PATTERN_NONE:
		count += sprintf(buf + count, "PATTERN_NONE\n");
		break;
	case TPS1:
		count += sprintf(buf + count, "TPS1\n");
		break;
	case TPS2:
		count += sprintf(buf + count, "TPS2\n");
		break;
	case TPS3:
		count += sprintf(buf + count, "TPS3\n");
		break;
	case TPS4:
		count += sprintf(buf + count, "TPS4\n");
		break;
	case PRBS7:
		count += sprintf(buf + count, "PRBS7\n");
		break;
	case D10_2:
		count += sprintf(buf + count, "D10_2\n");
		break;
	case PATTERN_80BIT:
		count += sprintf(buf + count, "80BIT\n");
		break;
	case HBR2_EYE:
		count += sprintf(buf + count, "HBR2_EYE\n");
		break;
	case LINK_QUALITY_PATTERN:
		count += sprintf(buf + count, "LINK_QUALITY_PATTERN\n");
		break;
	case CP2520_PATTERN2:
		count += sprintf(buf + count, "CP2520_PATTERN2\n");
		break;
	case CP2520_PATTERN3:
		count += sprintf(buf + count, "CP2520_PATTERN3\n");
		break;
	case SYMBOL_MEASURE_PATTERN:
		count += sprintf(buf + count, "SYMBOL_MEASURE_PATTERN\n");
		break;
	default:
		count += sprintf(buf + count, "reversed\n");
		break;

	}

	return count;
}

static ssize_t ssc_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct edp_tx_core *edp_core;
	u8 *separator = NULL;
	u32 ssc_en = 0;
	u32 ssc_mode = 0;

	edp_core = &drm_edp->edp_core;

	separator = strchr(buf, ' ');
	if (separator == NULL) {
		EDP_WRN("%s,%d err, syntax error!\n", __func__, __LINE__);
	} else {
		ssc_en = simple_strtoul(buf, NULL, 0);
		ssc_mode = simple_strtoul(separator + 1, NULL, 0);

		if (ssc_mode > 1) {
			EDP_WRN("ssc mode should select from: 0/1\n");
			return count;
		}

		if (ssc_en == 0)
			edp_core->ssc_en = 0;
		else
			edp_core->ssc_en = 1;

		edp_core->ssc_mode = ssc_mode;
	}

	return count;
}

static ssize_t ssc_debug_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u32 count = 0;
	count += sprintf(buf + count, "echo [ssc_en ssc_mode] > ssc_debug\n");
	count += sprintf(buf + count, "eg: 0-Center_Mode   1-Downspread_Mode\n");

	return count;
}

static DEVICE_ATTR(dpcd, 0664, dpcd_show, NULL);
static DEVICE_ATTR(edid, 0664, edid_show, edid_store);
static DEVICE_ATTR(lane_debug_en, 0664, lane_debug_en_show, lane_debug_en_store);
static DEVICE_ATTR(lane_invert_debug, 0664, lane_invert_debug_show, lane_invert_debug_store);
static DEVICE_ATTR(lane_remap_debug, 0664, lane_remap_debug_show, lane_remap_debug_store);
static DEVICE_ATTR(lane_fmt_debug, 0664, lane_fmt_debug_show, lane_fmt_debug_store);
static DEVICE_ATTR(lane_cfg_debug, 0664, lane_cfg_debug_show, lane_cfg_debug_store);
static DEVICE_ATTR(resource_lock, 0664, resource_lock_show, resource_lock_store);
static DEVICE_ATTR(bypass_training, 0664, bypass_training_show, bypass_training_store);
static DEVICE_ATTR(loglevel_debug, 0664, loglevel_debug_show, loglevel_debug_store);
static DEVICE_ATTR(hotplug, 0664, hotplug_show, NULL);
static DEVICE_ATTR(hpd_mask, 0664, hpd_mask_show, hpd_mask_store);
static DEVICE_ATTR(sink_info, 0664, sink_info_show, NULL);
static DEVICE_ATTR(source_info, 0664, source_info_show, NULL);
static DEVICE_ATTR(phy_read, 0664, phy_read_show, phy_read_store);
static DEVICE_ATTR(phy_write, 0664, NULL, phy_write_store);
static DEVICE_ATTR(aux_read, 0664, aux_read_show, aux_read_store);
static DEVICE_ATTR(aux_write, 0664, aux_write_show, aux_write_store);
static DEVICE_ATTR(aux_i2c_read, 0664, aux_i2c_read_show, aux_i2c_read_store);
static DEVICE_ATTR(lane_config_now, 0664, lane_config_now_show, NULL);
static DEVICE_ATTR(lane_config_debug, 0664, lane_config_debug_show, NULL);
static DEVICE_ATTR(timings_now, 0664, timings_now_show, NULL);
static DEVICE_ATTR(mode_list, 0664, mode_list_show, NULL);
static DEVICE_ATTR(training_param_type, 0664, training_param_type_show, training_param_type_store);
static DEVICE_ATTR(ssc_debug, 0664, ssc_debug_show, ssc_debug_store);
static DEVICE_ATTR(pattern_debug, 0664, pattern_debug_show, pattern_debug_store);
static DEVICE_ATTR(colorbar, 0664, colorbar_show, colorbar_store);

static struct attribute *edp_attributes[] = {
	&dev_attr_dpcd.attr,
	&dev_attr_edid.attr,
	&dev_attr_lane_debug_en.attr,
	&dev_attr_lane_remap_debug.attr,
	&dev_attr_lane_invert_debug.attr,
	&dev_attr_lane_fmt_debug.attr,
	&dev_attr_lane_cfg_debug.attr,
	&dev_attr_resource_lock.attr,
	&dev_attr_bypass_training.attr,
	&dev_attr_loglevel_debug.attr,
	&dev_attr_hotplug.attr,
	&dev_attr_hpd_mask.attr,
	&dev_attr_sink_info.attr,
	&dev_attr_source_info.attr,
	&dev_attr_phy_read.attr,
	&dev_attr_phy_write.attr,
	&dev_attr_aux_read.attr,
	&dev_attr_aux_write.attr,
	&dev_attr_aux_i2c_read.attr,
	&dev_attr_lane_config_now.attr,
	&dev_attr_lane_config_debug.attr,
	&dev_attr_timings_now.attr,
	&dev_attr_mode_list.attr,
	&dev_attr_training_param_type.attr,
	&dev_attr_ssc_debug.attr,
	&dev_attr_pattern_debug.attr,
	&dev_attr_colorbar.attr,
	NULL
};

static struct attribute_group edp_attribute_group = {
	.name = "attr",
	.attrs = edp_attributes
};




static s32 edp_open(struct inode *inode, struct file *filp)
{
	return -EINVAL;
}

static s32 edp_release(struct inode *inode, struct file *filp)
{
	return -EINVAL;
}

static ssize_t edp_read(struct file *file, char __user *buf,
						size_t count,
						loff_t *ppos)
{
	return -EINVAL;
}

static ssize_t edp_write(struct file *file, const char __user *buf,
						size_t count,
						loff_t *ppos)
{
	return -EINVAL;
}

static s32 edp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static long edp_ioctl(struct file *filp, u32 cmd, unsigned long arg)
{
	return -EINVAL;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long edp_compat_ioctl(struct file *filp, u32 cmd,
						unsigned long arg)
{
	return -EINVAL;
}
#endif

static const struct file_operations edp_fops = {
	.owner		= THIS_MODULE,
	.open		= edp_open,
	.release	= edp_release,
	.write		= edp_write,
	.read		= edp_read,
	.unlocked_ioctl	= edp_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl	= edp_compat_ioctl,
#endif
	.mmap		= edp_mmap,
};


void sunxi_edp_get_source_capacity(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_cap *src_cap;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	src_cap = &drm_edp->source_cap;

	src_cap->max_lane = edp_source_get_max_lane(edp_hw);
	src_cap->max_rate = edp_source_get_max_rate(edp_hw);
	src_cap->tps3_support = edp_source_support_tps3(edp_hw);
	src_cap->audio_support = edp_source_support_audio(edp_hw);
	src_cap->fast_train_support = edp_source_support_fast_training(edp_hw);
	src_cap->ssc_support = edp_source_support_ssc(edp_hw);
	src_cap->psr_support = edp_source_support_psr(edp_hw);
	src_cap->psr2_support = edp_source_support_psr2(edp_hw);
	src_cap->assr_support = edp_source_support_assr(edp_hw);
	src_cap->mst_support = edp_source_support_mst(edp_hw);
	src_cap->fec_support = edp_source_support_fec(edp_hw);
	src_cap->hdcp1x_support = edp_source_support_hdcp1x(edp_hw);
	src_cap->hdcp2x_support = edp_source_support_hdcp2x(edp_hw);
	src_cap->hardware_hdcp1x_support = edp_source_support_hardware_hdcp1x(edp_hw);
	src_cap->hardware_hdcp2x_support = edp_source_support_hardware_hdcp2x(edp_hw);
	src_cap->enhance_frame_support = edp_source_support_enhance_frame(edp_hw);
	src_cap->lane_remap_support = edp_source_support_lane_remap(edp_hw);
	src_cap->lane_invert_support = edp_source_support_lane_invert(edp_hw);

	if (edp_source_support_hdcp1x(edp_hw) ||
		    edp_source_support_hdcp2x(edp_hw)) {
		sunxi_dp_hdcp_init(&drm_edp->hdcp, edp_hw);
	}
}

int sunxi_edp_init_hardware(struct sunxi_drm_edp *drm_edp)
{
	int ret = RET_OK;

	sunxi_edp_hw_callback_init(&drm_edp->edp_hw);
	sunxi_edp_get_source_capacity(drm_edp);

	if (drm_edp->desc->bind) {
		ret = drm_edp->desc->bind(drm_edp);
		if (ret)
			EDP_ERR("edp desc bind failed!\n");
	}

	return ret;
}

void sunxi_edp_smooth_display_process(struct sunxi_drm_edp *drm_edp)
{
	char dpcd_rx_buf[576];
	struct edid *edid = NULL;
	int ret;

	/* update edid and dpcd for connector_get_modes during smooth display */
	//drm_edp->is_enabled = true;
	drm_edp->hpd_state = true;

	memset(&dpcd_rx_buf[0], 0, sizeof(dpcd_rx_buf));
	ret = edp_read_dpcd(&drm_edp->edp_hw, &dpcd_rx_buf[0]);
	if (ret < 0)
		EDP_WRN("fail to read edp dpcd!\n");
	else
		edp_parse_dpcd(drm_edp, &dpcd_rx_buf[0]);

	/* update lane config to adjust source-sink capacity */
	edp_update_capacity(drm_edp);

	edid = drm_do_get_edid(&drm_edp->sdrm.connector, edp_get_edid_block, &drm_edp->edp_hw);
	if (edid == NULL)
		EDP_WRN("fail to read edid\n");
	else {
		edp_parse_edid(drm_edp, edid);
		/* release previous one to avoid dangling pointer */
		if (drm_edp->edp_core.edid)
			edp_edid_put(drm_edp->edp_core.edid);
		drm_edp->edp_core.edid = edid;
	}
}

int edp_parse_dts(struct device *dev)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	int ret = 0;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	drm_edp->base_addr = of_iomap(dev->of_node, 0);
	if (!drm_edp->base_addr) {
		EDP_DEV_ERR(dev, "fail to get addr for edp!\n");
		ret = RET_FAIL;
		goto OUT;
	} else
		edp_hw_set_reg_base(edp_hw, drm_edp->base_addr);

	drm_edp->top_addr = of_iomap(dev->of_node, 1);
	if (!drm_edp->top_addr)
		EDP_DBG("Maybe there is no top addr for edp!\n");
	else
		edp_hw_set_top_base(edp_hw, drm_edp->top_addr);

	drm_edp->irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!drm_edp->irq) {
		EDP_DEV_ERR(dev, "edp parse irq fail!\n");
		ret = RET_FAIL;
		goto ERR_IOMAP;
	}

	drm_edp->clk_bus = devm_clk_get(dev, "clk_bus_edp");
	if (IS_ERR_OR_NULL(drm_edp->clk_bus)) {
		EDP_DEV_ERR(dev, "fail to get clk bus for edp!\n");
		ret = RET_FAIL;
		goto ERR_IOMAP;
	}

	drm_edp->clk = devm_clk_get(dev, "clk_edp");
	if (IS_ERR_OR_NULL(drm_edp->clk)) {
		EDP_DEV_ERR(dev, "fail to get clk for edp!\n");
		ret = RET_FAIL;
		goto ERR_CLK_BUS;
	}

	drm_edp->clk_24m = devm_clk_get(dev, "clk_24m_edp");
	if (IS_ERR_OR_NULL(drm_edp->clk_24m)) {
		EDP_DRV_DBG("24M clock for edp is not need or missing!\n");
	}

	drm_edp->rst_bus = devm_reset_control_get(dev, "rst_bus_edp");
	if (IS_ERR_OR_NULL(drm_edp->rst_bus)) {
		EDP_DEV_ERR(dev, "fail to get rst for edp!\n");
		ret = RET_FAIL;
		goto ERR_CLK;
	}

	/*parse power resource*/
	drm_edp->vdd_regulator = regulator_get(dev, "vdd-edp");
	if (IS_ERR_OR_NULL(drm_edp->vdd_regulator))
		EDP_DRV_DBG("vdd_edp is null or no need!\n");

	drm_edp->vcc_regulator = regulator_get(dev, "vcc-edp");
	if (IS_ERR_OR_NULL(drm_edp->vcc_regulator))
		EDP_DRV_DBG("vcc_edp is null or no need!\n");

	// sync phy handle to exp_tx_core because phy para need
	// update during training procedure
	drm_edp->dp_phy = devm_phy_get(dev, "dp-phy");
	if (IS_ERR_OR_NULL(drm_edp->dp_phy)) {
		DRM_INFO("edp's dp-phy not setting, maybe not used!\n");
		drm_edp->dp_phy = NULL;
	} else
		drm_edp->edp_core.dp_phy = drm_edp->dp_phy;

	drm_edp->aux_phy = devm_phy_get(dev, "aux-phy");
	if (IS_ERR_OR_NULL(drm_edp->aux_phy)) {
		DRM_INFO("edp's aux-phy not setting, maybe not used!\n");
		drm_edp->aux_phy = NULL;
	} else
		drm_edp->edp_core.aux_phy = drm_edp->aux_phy;

	drm_edp->combo_phy = devm_phy_get(dev, "combo-phy");
	if (IS_ERR_OR_NULL(drm_edp->combo_phy)) {
		DRM_INFO("edp's combo-phy not setting, maybe not used!\n");
		drm_edp->combo_phy = NULL;
	} else
		drm_edp->edp_core.combo_phy = drm_edp->combo_phy;

	ret = edp_debug_mode_parse(dev);
	if (ret != 0)
		goto ERR_CLK;

	ret = edp_misc_parse(dev);
	if (ret != 0)
		goto ERR_CLK;

	ret = edp_lane_para_parse(dev);
	if (ret != 0)
		goto ERR_CLK;

	return RET_OK;

ERR_CLK:
	if (drm_edp->clk_24m)
		clk_put(drm_edp->clk_24m);
	clk_put(drm_edp->clk);
ERR_CLK_BUS:
	clk_put(drm_edp->clk_bus);
ERR_IOMAP:
	if (drm_edp->base_addr)
		iounmap((char __iomem *)drm_edp->base_addr);
OUT:
	return ret;

}

static int populate_mode_from_default_timings(struct drm_connector *connector)
{
	int mode_num = 0;
	unsigned int i;
	struct sunxi_drm_edp *drm_edp =
		drm_connector_to_sunxi_drm_edp(connector);

	for (i = 0; i < ARRAY_SIZE(edp_standard_modes); i++) {
		struct drm_display_mode *mode =
			drm_mode_duplicate(drm_edp->sdrm.drm_dev,
					&edp_standard_modes[i]);
		if (!mode)
			continue;

		/* the first mode is the preferred mode */
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
		mode_num++;
	}

	return mode_num;
}

static int sunxi_edp_connector_get_modes(struct drm_connector *connector)
{
	struct sunxi_drm_edp *drm_edp = drm_connector_to_sunxi_drm_edp(connector);
	struct edid *edid = NULL;
	int mode_num = 0;

	if (drm_edp->desc->connector_type == DRM_MODE_CONNECTOR_eDP) {
		/*
		 * Open edp power temporarily, for get panel timings,
		 * poweroff again if done, for ensuring energy save
		 */
		if (!drm_edp->is_enabled && !drm_edp->boot_enabled) {
			drm_panel_prepare(drm_edp->sdrm.panel);
			if (drm_edp->desc->enable_early)
				drm_edp->desc->enable_early(drm_edp);

			if (drm_edp->edp_core.edid != NULL)
				drm_connector_update_edid_property(connector, drm_edp->edp_core.edid);
			else
				drm_connector_update_edid_property(connector, NULL);

			/*
			 * thought it looks silly, but it can remind us to support
			 * aux_ep framework in panel, which can use aux chanel
			 * flexiable in panel driver
			 */
			mode_num += drm_panel_get_modes(drm_edp->sdrm.panel, connector);
			if (drm_edp->desc->disable)
				drm_edp->desc->disable(drm_edp);
			drm_panel_unprepare(drm_edp->sdrm.panel);
		} else {
			if (drm_edp->edp_core.edid != NULL)
				drm_connector_update_edid_property(connector, drm_edp->edp_core.edid);
			else
				drm_connector_update_edid_property(connector, NULL);

			/*
			 * thought it looks silly, but it can remind us to support
			 * aux_ep framework in panel, which can use aux chanel
			 * flexiable in panel driver
			 */
			mode_num += drm_panel_get_modes(drm_edp->sdrm.panel, connector);
		}
	} else {
		edid = drm_do_get_edid(&drm_edp->sdrm.connector, edp_get_edid_block, &drm_edp->edp_hw);
		if (edid != NULL) {
			drm_connector_update_edid_property(connector, edid);
			mode_num += drm_add_edid_modes(connector, edid);

			/* release edid to avoid dangling pointer */
			edp_edid_put(edid);
		}

		if (!mode_num)
			mode_num += populate_mode_from_default_timings(connector);
	}

	return mode_num;
}

static enum drm_mode_status
sunxi_edp_connector_mode_valid(struct drm_connector *connector,
			      struct drm_display_mode *mode)
{
	struct sunxi_drm_edp *drm_edp
		= drm_connector_to_sunxi_drm_edp(connector);
	struct disp_video_timings timings;
	s32 ret = 0;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	drm_mode_to_sunxi_video_timings(mode, &timings);

	ret = edp_hw_query_lane_capability(edp_hw, &drm_edp->edp_core, &timings);
	if (ret) {
		DRM_ERROR("Mode[%s] is out of edp lane's capability!\n", mode->name);
		return MODE_BAD;
	}

	return MODE_OK;
}

static enum drm_connector_status
drm_edp_connector_detect(struct drm_connector *connector, bool force)
{
	struct sunxi_drm_edp *drm_edp
		= drm_connector_to_sunxi_drm_edp(connector);

	if (!drm_edp)
		return connector_status_unknown;

	if (drm_edp->desc->connector_type == DRM_MODE_CONNECTOR_eDP) {
		//FIXME
		/* for such case: fb_client need detect and enable plugin
		 * connector, but edp's controller init in atomic_enable */
		if (!drm_edp->is_enabled)
			return connector_status_connected;
	}

	return drm_edp->hpd_state ? connector_status_connected :
		connector_status_disconnected;
}


static void drm_edp_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static int drm_edp_connector_set_property(struct drm_connector *connector,
				struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t val)
{
	struct sunxi_drm_edp *drm_edp
		= drm_connector_to_sunxi_drm_edp(connector);
	struct sunxi_drm_private *private = to_sunxi_drm_private(drm_edp->sdrm.drm_dev);
	struct sunxi_edp_connector_state *estate = to_drm_edp_connector_state(state);

	if (property == private->prop_color_format) {
		estate->color_format = val;
		return 0;
	} else if (property == private->prop_color_depth) {
		estate->color_depth = val;
		return 0;
	}

	DRM_ERROR("failed to set edp connector property:%s\n", property->name);
	return -EINVAL;
}

static int drm_edp_connector_get_property(struct drm_connector *connector,
				const struct drm_connector_state *state,
				struct drm_property *property,
				uint64_t *val)
{
	struct sunxi_drm_edp *drm_edp
		= drm_connector_to_sunxi_drm_edp(connector);
	struct sunxi_drm_private *private = to_sunxi_drm_private(drm_edp->sdrm.drm_dev);
	struct sunxi_edp_connector_state *estate = to_drm_edp_connector_state(state);

	if (property == private->prop_color_format) {
		*val = estate->color_format;
		return 0;
	} else if (property == private->prop_color_depth) {
		*val = estate->color_depth;
		return 0;
	}

	DRM_ERROR("failed to get edp connector property:%s\n", property->name);
	return -EINVAL;
}

static struct drm_connector_state *drm_edp_connector_duplicate_state(struct drm_connector *connector)
{
	struct sunxi_edp_connector_state *new_estate, *cur_estate;

	if (WARN_ON(!connector->state))
		return NULL;

	cur_estate = to_drm_edp_connector_state(connector->state);
	new_estate = kzalloc(sizeof(*new_estate), GFP_KERNEL);
	if (!new_estate)
		return NULL;
	memcpy(new_estate, cur_estate, sizeof(*new_estate));

	__drm_atomic_helper_connector_duplicate_state(connector, &new_estate->state);
	return &new_estate->state;

}

static void drm_edp_connector_destroy_state(struct drm_connector *connector, struct drm_connector_state *state)
{
	struct sunxi_edp_connector_state *estate = to_drm_edp_connector_state(state);

	__drm_atomic_helper_connector_destroy_state(&estate->state);
	kfree(estate);
}

static void drm_edp_connector_state_reset(struct drm_connector *connector)
{
	struct sunxi_edp_connector_state *estate = NULL;

	if (connector->state)
		drm_edp_connector_destroy_state(connector, connector->state);

	estate = kzalloc(sizeof(*estate), GFP_KERNEL);

	__drm_atomic_helper_connector_reset(connector, &estate->state);
	estate->color_format = DISP_CSC_TYPE_RGB;
	estate->color_depth = DISP_DATA_8BITS;
}

static const struct drm_connector_funcs sunxi_edp_connector_funcs = {
	.detect = drm_edp_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_edp_connector_destroy,
	.reset = drm_edp_connector_state_reset,
	.atomic_duplicate_state = drm_edp_connector_duplicate_state,
	.atomic_destroy_state = drm_edp_connector_destroy_state,
	.atomic_set_property = drm_edp_connector_set_property,
	.atomic_get_property = drm_edp_connector_get_property,
};

static const struct drm_connector_helper_funcs
	sunxi_edp_connector_helper_funcs = {
		.get_modes = sunxi_edp_connector_get_modes,
		.mode_valid = sunxi_edp_connector_mode_valid,
	};

static bool sunxi_edp_fifo_check(void *data)
{
	struct sunxi_drm_edp *drm_edp = (struct sunxi_drm_edp *)data;
	return sunxi_tcon_check_fifo_status(drm_edp->sdrm.tcon_dev);
}

static void sunxi_edp_enable_vblank(bool enable, void *data)
{
	struct sunxi_drm_edp *drm_edp = (struct sunxi_drm_edp *)data;

	sunxi_tcon_enable_vblank(drm_edp->sdrm.tcon_dev, enable);
}

int sunxi_edp_get_current_line(void *data)
{
	struct sunxi_drm_edp *drm_edp = (struct sunxi_drm_edp *)data;
	return sunxi_tcon_get_current_line(drm_edp->sdrm.tcon_dev);
}

static bool sunxi_edp_is_sync_time_enough(void *data)
{
	struct sunxi_drm_edp *drm_edp = (struct sunxi_drm_edp *)data;
	return sunxi_tcon_is_sync_time_enough(drm_edp->sdrm.tcon_dev);
}

void sunxi_edp_encoder_atomic_disable(struct drm_encoder *encoder,
				      struct drm_atomic_state *state)
{
	struct sunxi_drm_edp *drm_edp = drm_encoder_to_sunxi_drm_edp(encoder);

	if (drm_edp->desc->disable)
		drm_edp->desc->disable(drm_edp);

	if (drm_edp->sdrm.panel) {
		drm_panel_disable(drm_edp->sdrm.panel);
		drm_panel_unprepare(drm_edp->sdrm.panel);
	}
	sunxi_tcon_mode_exit(drm_edp->sdrm.tcon_dev);
	drm_edp->is_enabled = false;
	DRM_DEBUG_DRIVER("%s finish\n", __FUNCTION__);
}

void sunxi_edp_encoder_atomic_enable(struct drm_encoder *encoder,
				     struct drm_atomic_state *state)
{
	struct drm_crtc *crtc = encoder->crtc;
	struct sunxi_drm_edp *drm_edp = drm_encoder_to_sunxi_drm_edp(encoder);
	int de_id = sunxi_drm_crtc_get_hw_id(crtc);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct sunxi_crtc_state *scrtc_state = to_sunxi_crtc_state(crtc_state);
	struct edp_tx_core *edp_core = &drm_edp->edp_core;
	struct drm_connector *connector = &drm_edp->sdrm.connector;
	struct drm_connector_state *conn_state = connector->state;
	struct sunxi_edp_connector_state *estate = to_drm_edp_connector_state(conn_state);
	struct edp_lane_para *lane_para = &drm_edp->edp_core.lane_para;
	struct disp_output_config disp_cfg;
	int fps = 0;

	/* update color_format/color_depth from property */
	if (drm_edp->desc->connector_type == DRM_MODE_CONNECTOR_DisplayPort) {
		lane_para->color_fmt = estate->color_format;
		switch (estate->color_depth) {
		case DISP_DATA_8BITS:
			lane_para->colordepth = 8;
			break;
		case DISP_DATA_10BITS:
			lane_para->colordepth = 10;
			break;
		case DISP_DATA_12BITS:
			lane_para->colordepth = 12;
			break;
		case DISP_DATA_16BITS:
			lane_para->colordepth = 16;
			break;
		}

		if (lane_para->color_fmt == DISP_CSC_TYPE_RGB)
			lane_para->bpp = 3 * lane_para->colordepth;
		else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV444)
			lane_para->bpp = 3 * lane_para->colordepth;
		else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV422)
			lane_para->bpp = 2 * lane_para->colordepth;
		else if (lane_para->color_fmt == DISP_CSC_TYPE_YUV420)
			lane_para->bpp = 3 * lane_para->colordepth / 2;
	}

	/* parse timings from drm_video_mode */
	drm_mode_to_sunxi_video_timings(&drm_edp->mode, &edp_core->timings);

	fps = edp_core->timings.pixel_clk / edp_core->timings.hor_total_time
		/ edp_core->timings.ver_total_time;
	if (fps > 65 && drm_edp->fps_limit_60) {
		EDP_INFO("eDP/DP screen's timings:[%dx%d@%dHz], limit to [%dx%d@60Hz] now!\n",
			 edp_core->timings.x_res, edp_core->timings.y_res, fps,
			 edp_core->timings.x_res, edp_core->timings.y_res);
		edp_core->timings.pixel_clk =
			edp_core->timings.x_res *edp_core->timings.y_res * 60;
	}

	memset(&disp_cfg, 0, sizeof(struct disp_output_config));
	memcpy(&disp_cfg.timing,
	       &drm_edp->edp_core.timings, sizeof(struct disp_video_timings));
	disp_cfg.type = INTERFACE_EDP;
	disp_cfg.de_id = de_id;
	disp_cfg.irq_handler = sunxi_crtc_event_proc;
	disp_cfg.irq_data = scrtc_state->base.crtc;
	disp_cfg.sw_enable = drm_edp->sw_enable;
	disp_cfg.pixel_mode = edp_core->pixel_mode;

	sunxi_tcon_mode_init(drm_edp->sdrm.tcon_dev, &disp_cfg);

	if (drm_edp->sdrm.panel)
		drm_panel_prepare(drm_edp->sdrm.panel);

	/* FIXME:TODO: consider cancel enable_early? make dp and edp unity?  */
	if (drm_edp->sw_enable) {
		if (drm_edp->desc->enable_early_sw) {
			if (drm_edp->desc->enable_early_sw(drm_edp))
				goto FAIL;
		}

		if (drm_edp->desc->enable_sw) {
			if (drm_edp->desc->enable_sw(drm_edp))
				goto FAIL;

		}
	} else {
		if (drm_edp->desc->enable_early) {
			if (drm_edp->desc->enable_early(drm_edp))
				goto FAIL;
		}

		if (drm_edp->desc->enable) {
			if (drm_edp->desc->enable(drm_edp))
				goto FAIL;
		}

		/* usually for backlight enable */
		if (drm_edp->sdrm.panel)
			drm_panel_enable(drm_edp->sdrm.panel);
	}

	DRM_DEBUG_DRIVER("%s finish, sw = %d\n", __FUNCTION__, drm_edp->sw_enable);
	drm_edp->is_enabled = true;
	return;
FAIL:
	DRM_DEBUG_DRIVER("%s fail!\n", __FUNCTION__);
	drm_edp->is_enabled = false;
}

int sunxi_edp_encoder_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct sunxi_crtc_state *scrtc_state = to_sunxi_crtc_state(crtc_state);
	struct sunxi_drm_edp *drm_edp = drm_encoder_to_sunxi_drm_edp(encoder);
	struct sunxi_edp_connector_state *new_estate = to_drm_edp_connector_state(conn_state);

	if ((drm_edp->edp_core.lane_para.color_fmt != new_estate->color_format) ||
		(drm_edp->edp_core.lane_para.colordepth != new_estate->color_depth))
		crtc_state->connectors_changed = true;

	scrtc_state->tcon_id = drm_edp->sdrm.tcon_id;
	scrtc_state->enable_vblank = sunxi_edp_enable_vblank;
	scrtc_state->check_status = sunxi_edp_fifo_check;
	scrtc_state->is_sync_time_enough = sunxi_edp_is_sync_time_enough;
	scrtc_state->get_cur_line = sunxi_edp_get_current_line;
	scrtc_state->output_dev_data = drm_edp;
	if (conn_state->crtc) {
		drm_edp->sw_enable = sunxi_drm_check_if_need_sw_enable(conn_state->connector);
		scrtc_state->sw_enable = drm_edp->sw_enable;
	}

	DRM_DEBUG_DRIVER("%s finish\n", __FUNCTION__);
	return 0;
}


static void sunxi_edp_encoder_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adj_mode)
{
	struct sunxi_drm_edp *drm_edp = drm_encoder_to_sunxi_drm_edp(encoder);

	drm_mode_copy(&drm_edp->mode, adj_mode);

}

static const struct drm_encoder_helper_funcs sunxi_edp_encoder_helper_funcs = {
	.atomic_disable = sunxi_edp_encoder_atomic_disable,
	.atomic_enable = sunxi_edp_encoder_atomic_enable,
	.atomic_check = sunxi_edp_encoder_atomic_check,
	.mode_set = sunxi_edp_encoder_mode_set,
};

static void drm_edp_connector_init_property (struct drm_device *drm,
				struct drm_connector *connector)
{
	struct sunxi_drm_private *private = to_sunxi_drm_private(drm);

	/* edp colorspace property */
	drm_object_attach_property(&connector->base, private->prop_color_format, DISP_CSC_TYPE_RGB);

	/* edp colordepth property */
	drm_object_attach_property(&connector->base, private->prop_color_depth, DISP_DATA_8BITS);
}

int sunxi_edp_init_drm(struct sunxi_drm_edp *drm_edp)
{
	int ret = RET_OK;
	struct sunxi_drm_device *sdrm = &drm_edp->sdrm;

	drm_encoder_helper_add(&sdrm->encoder, &sunxi_edp_encoder_helper_funcs);
	ret = drm_simple_encoder_init(sdrm->drm_dev, &sdrm->encoder, DRM_MODE_ENCODER_TMDS);
	if (ret) {
		DRM_ERROR("Couldn't initialise the encoder for tcon %d\n", drm_edp->sdrm.tcon_id);
		return ret;
	}

	sdrm->encoder.possible_crtcs =
		drm_of_find_possible_crtcs(sdrm->drm_dev, sdrm->tcon_dev->of_node);

	drm_connector_helper_add(&sdrm->connector,
				 &sunxi_edp_connector_helper_funcs);

	sdrm->connector.polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(sdrm->drm_dev, &sdrm->connector,
				 &sunxi_edp_connector_funcs,
				 drm_edp->desc->connector_type);
	if (ret) {
		drm_encoder_cleanup(&sdrm->encoder);
		DRM_ERROR("Couldn't initialise the connector for tcon %d\n", drm_edp->sdrm.tcon_id);
		return ret;
	}

	drm_edp_connector_init_property(sdrm->drm_dev, &sdrm->connector);

	drm_connector_attach_encoder(&sdrm->connector, &sdrm->encoder);

	return ret;
}


s32 drm_edp_output_bind(struct sunxi_drm_edp *drm_edp)
{

	edp_hw_init_early(&drm_edp->edp_hw);
#if IS_ENABLED(CONFIG_EXTCON)
	snprintf(edp_extcon_name, sizeof(edp_extcon_name), "drm-edp");
	drm_edp->extcon_edp = devm_extcon_dev_allocate(drm_edp->dev, edp_cable);
	if (IS_ERR_OR_NULL(drm_edp->extcon_edp)) {
		EDP_ERR("devm_extcon_dev_allocate fail\n");
		return RET_FAIL;
	}
	devm_extcon_dev_register(drm_edp->dev, drm_edp->extcon_edp);
	drm_edp->extcon_edp->name = edp_extcon_name;
#endif

	return RET_OK;
}

s32 drm_edp_output_unbind(struct sunxi_drm_edp *drm_edp)
{
	return RET_OK;
}

s32 drm_edp_output_enable_early_sw(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct edid *edid;
	//struct edp_debug *edp_debug = &drm_edp->edp_debug;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	char dpcd_rx_buf[576];
	s32 ret = RET_FAIL;

	pm_runtime_get_sync(drm_edp->dev);

	edp_core = &drm_edp->edp_core;
	if (drm_edp->vdd_regulator) {
		ret = regulator_enable(drm_edp->vdd_regulator);
		if (ret) {
			EDP_ERR("vdd-edp enable failed!\n");
			goto OUT;
		}
	}

	if (drm_edp->vcc_regulator) {
		ret = regulator_enable(drm_edp->vcc_regulator);
		if (ret) {
			EDP_ERR("vcc-edp enable failed!\n");
			goto OUT;
		}
	}

	ret = edp_clk_enable(drm_edp, true);
	if (ret) {
		EDP_ERR("edp edp_clk_enable fail!!\n");
		goto OUT;
	}

	edp_phy_init(drm_edp);
	edp_phy_enable(drm_edp, true);

	memset(&dpcd_rx_buf[0], 0, sizeof(dpcd_rx_buf));
	ret = edp_read_dpcd(edp_hw, &dpcd_rx_buf[0]);
	if (ret < 0)
		EDP_WRN("fail to read edp dpcd!\n");
	else
		edp_parse_dpcd(drm_edp, &dpcd_rx_buf[0]);

	edid = drm_do_get_edid(&drm_edp->sdrm.connector, edp_get_edid_block, &drm_edp->edp_hw);
	if (edid == NULL)
		EDP_WRN("fail to read edid\n");
	else {
		edp_parse_edid(drm_edp, edid);
		edp_core->edid = edid;
	}

	/* update lane config to adjust source-sink capacity */
	edp_update_capacity(drm_edp);

OUT:
	return ret;
}

s32 drm_edp_output_enable_early(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct edid *edid;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	char dpcd_rx_buf[576];
	s32 ret = RET_FAIL;

	pm_runtime_get_sync(drm_edp->dev);

	edp_core = &drm_edp->edp_core;
	if (drm_edp->vdd_regulator) {
		ret = regulator_enable(drm_edp->vdd_regulator);
		if (ret) {
			EDP_ERR("vdd-edp enable failed!\n");
			goto OUT;
		}
	}

	if (drm_edp->vcc_regulator) {
		ret = regulator_enable(drm_edp->vcc_regulator);
		if (ret) {
			EDP_ERR("vcc-edp enable failed!\n");
			goto OUT;
		}
	}

	ret = edp_clk_enable(drm_edp, true);
	if (ret) {
		EDP_ERR("edp edp_clk_enable fail!!\n");
		goto OUT;
	}

	edp_phy_init(drm_edp);
	edp_phy_enable(drm_edp, true);

	edp_hw_controller_init(edp_hw, edp_core);

	memset(&dpcd_rx_buf[0], 0, sizeof(dpcd_rx_buf));
	ret = edp_read_dpcd(edp_hw, &dpcd_rx_buf[0]);
	if (ret < 0)
		EDP_WRN("fail to read edp dpcd!\n");
	else
		edp_parse_dpcd(drm_edp, &dpcd_rx_buf[0]);

	edid = drm_do_get_edid(&drm_edp->sdrm.connector, edp_get_edid_block, &drm_edp->edp_hw);
	if (edid == NULL)
		EDP_WRN("fail to read edid\n");
	else {
		edp_parse_edid(drm_edp, edid);
		edp_core->edid = edid;
	}

	/* update lane config to adjust source-sink capacity */
	edp_update_capacity(drm_edp);

OUT:
	return ret;
}

s32 drm_edp_output_enable(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core = &drm_edp->edp_core;
	struct edp_debug *edp_debug = &drm_edp->edp_debug;
	struct edp_rx_cap *sink_cap = &drm_edp->sink_cap;
	struct edp_tx_cap *src_cap = &drm_edp->source_cap;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	s32 ret = RET_FAIL;

	if (!edp_core->timings.pixel_clk) {
		EDP_ERR("timing is not set by edid or dts, please check!\n");
		//goto ERR_PHY;
		goto OUT;
	}

	clk_set_rate(drm_edp->clk, edp_core->timings.pixel_clk / edp_core->pixel_mode);
	ret = edp_hw_enable(edp_hw, edp_core);
	if (ret) {
		EDP_ERR("edp core enable failed!\n");
		//goto ERR_PHY;
		goto OUT;
	}

	if (src_cap->ssc_support) {
		edp_hw_ssc_enable(edp_hw, edp_core->ssc_en ? true : false);
		if (edp_core->ssc_en)
			edp_hw_ssc_set_mode(edp_hw, edp_core->ssc_mode);
	}

	// edp need assr enable
	// note: DP should and must not use ASSR, otherwise panel shows nothing
	if (!edp_core->controller_mode && sink_cap->assr_support && src_cap->assr_support) {
		ret = edp_hw_assr_enable(edp_hw, true);
		if (ret < 0)
			goto OUT;
	}

	if (src_cap->enhance_frame_support && sink_cap->enhance_frame_support)
		edp_hw_enhance_frame_enable(edp_hw, true);

	/* set color space, color depth */
	ret = edp_hw_set_video_format(edp_hw, edp_core);
	if (ret < 0)
		goto OUT;

	/* set specific timings */
	ret = edp_hw_set_video_timings(edp_hw, &edp_core->timings);
	if (ret < 0)
		goto OUT;

	/* set transfer unit */
	ret = edp_hw_set_transfer_config(edp_hw, edp_core);
	if (ret < 0)
		goto OUT;

	if (!edp_debug->bypass_training) {
		ret = edp_main_link_setup(edp_hw, edp_core);
		if (ret < 0)
			goto OUT;
	}

	edp_hw_link_start(edp_hw);

//	if (dprx_hdcp2_capable(&drm_edp->hdcp))
//		sunxi_dp_hdcp2_enable(&drm_edp->hdcp);
	if (drm_edp->use_dpcd && edp_source_support_hdcp1x(edp_hw) && dprx_hdcp1_capable(&drm_edp->hdcp))
		sunxi_dp_hdcp1_enable(&drm_edp->hdcp);

OUT:
	return ret;

}

s32 drm_edp_output_disable(struct sunxi_drm_edp *drm_edp)
{
	s32 ret = 0;
	struct edp_tx_core *edp_core = &drm_edp->edp_core;
	struct edid *edid;
	struct edp_rx_cap *sink_cap = &drm_edp->sink_cap;
	struct edp_tx_cap *src_cap = &drm_edp->source_cap;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	ret = edp_hw_disable(edp_hw, edp_core);
	if (ret) {
		EDP_ERR("edp core disable failed!\n");
		return ret;
	}

	if (!edp_core->controller_mode && sink_cap->assr_support && src_cap->assr_support)
		edp_hw_assr_enable(edp_hw, false);

	if (src_cap->enhance_frame_support && sink_cap->enhance_frame_support)
		edp_hw_enhance_frame_enable(edp_hw, false);

	edp_phy_enable(drm_edp, false);
	edp_phy_exit(drm_edp);

	ret = edp_clk_enable(drm_edp, false);
	if (ret) {
		EDP_ERR("edp clk disaable fail!!\n");
		return ret;
	}

	if (drm_edp->vcc_regulator)
		regulator_disable(drm_edp->vcc_regulator);

	if (drm_edp->vdd_regulator)
		regulator_disable(drm_edp->vdd_regulator);

	edid = edp_core->edid;
	if (edid) {
		edp_edid_put(edid);
		edp_core->edid = NULL;
		drm_connector_update_edid_property(&drm_edp->sdrm.connector, NULL);
	}
	drm_edp->dpcd_parsed = false;

	sink_cap_reset(drm_edp);

	pm_runtime_put_sync(drm_edp->dev);

	return ret;
}

void drm_edp_output_soft_reset(struct sunxi_drm_edp *drm_edp)
{
	if (drm_edp->is_enabled) {
		drm_edp_output_disable(drm_edp);
		msleep(1);
		drm_edp_output_enable_early(drm_edp);
		drm_edp_output_enable(drm_edp);
	}
}


/* edp's hpd line is useless in some case, so plugin/plugout is not neccessary */
struct sunxi_edp_output_desc drm_edp_output = {
	.hw_id			= 0,
	.connector_type		= DRM_MODE_CONNECTOR_eDP,
	.bind			= drm_edp_output_bind,
	.unbind			= drm_edp_output_unbind,
	.enable_early		= drm_edp_output_enable_early,
	.enable_early_sw	= drm_edp_output_enable_early_sw,
	.enable			= drm_edp_output_enable,
	.disable		= drm_edp_output_disable,
	.soft_reset     	= drm_edp_output_soft_reset,
};

s32 drm_dp_output_bind(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	s32 ret;

	pm_runtime_get_sync(drm_edp->dev);

	edp_core = &drm_edp->edp_core;
	if (drm_edp->vdd_regulator) {
		ret = regulator_enable(drm_edp->vdd_regulator);
		if (ret) {
			EDP_ERR("vdd-edp enable failed!\n");
			goto OUT;
		}
	}

	if (drm_edp->vcc_regulator) {
		ret = regulator_enable(drm_edp->vcc_regulator);
		if (ret) {
			EDP_ERR("vcc-edp enable failed!\n");
			goto ERR_VCC;
		}
	}

	ret = edp_clk_enable(drm_edp, true);
	if (ret) {
		EDP_ERR("edp_clk_enable fail!!\n");
		goto ERR_CLK;
	}

	edp_phy_init(drm_edp);
	edp_phy_enable(drm_edp, true);

	edp_hw_init_early(edp_hw);

	edp_hw_controller_init(edp_hw, edp_core);

#if IS_ENABLED(CONFIG_EXTCON)
	snprintf(edp_extcon_name, sizeof(edp_extcon_name), "drm-dp");
	drm_edp->extcon_edp = devm_extcon_dev_allocate(drm_edp->dev, dp_cable);
	if (IS_ERR_OR_NULL(drm_edp->extcon_edp)) {
		EDP_ERR("devm_extcon_dev_allocate fail\n");
		goto ERR_PHY;
	}
	devm_extcon_dev_register(drm_edp->dev, drm_edp->extcon_edp);
	drm_edp->extcon_edp->name = edp_extcon_name;
#endif

	return ret;

#if IS_ENABLED(CONFIG_EXTCON)
ERR_PHY:
	edp_phy_enable(drm_edp, false);
	edp_phy_exit(drm_edp);
	edp_clk_enable(drm_edp, false);
#endif
ERR_CLK:
	if (drm_edp->vcc_regulator)
		regulator_disable(drm_edp->vcc_regulator);
ERR_VCC:
	if (drm_edp->vdd_regulator)
		regulator_disable(drm_edp->vdd_regulator);
OUT:
	pm_runtime_put_sync(drm_edp->dev);

	return ret;
}

s32 drm_dp_output_unbind(struct sunxi_drm_edp *drm_edp)
{
	s32 ret = 0;

	edp_phy_enable(drm_edp, false);
	edp_phy_exit(drm_edp);

	ret = edp_clk_enable(drm_edp, false);
	if (ret) {
		EDP_ERR("edp clk disaable fail!!\n");
		return ret;
	}

	if (drm_edp->vcc_regulator)
		regulator_disable(drm_edp->vcc_regulator);

	if (drm_edp->vdd_regulator)
		regulator_disable(drm_edp->vdd_regulator);

	pm_runtime_put_sync(drm_edp->dev);

	return ret;
}

s32 drm_dp_output_enable(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core = &drm_edp->edp_core;
	struct edp_debug *edp_debug = &drm_edp->edp_debug;
	struct edp_tx_cap *src_cap = &drm_edp->source_cap;
	struct edp_rx_cap *sink_cap = &drm_edp->sink_cap;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	s32 ret = 0;

	if (edp_debug->hpd_mask) {
		if ((edp_debug->hpd_mask == 0x10) || (edp_debug->hpd_mask == 0x110)\
		    || (edp_debug->hpd_mask == 0x1010)) {
			EDP_ERR("sink device unconnect virtual!\n");
			return RET_FAIL;
		}

	} else {
		if (!drm_edp->hpd_state_now) {
			EDP_ERR("sink device unconnect!\n");
			return RET_FAIL;
		}
	}

	if (!edp_core->timings.pixel_clk) {
		EDP_ERR("timing is not set by edid or dts, please check!\n");
		return RET_FAIL;
	}
	clk_set_rate(drm_edp->clk, edp_core->timings.pixel_clk / edp_core->pixel_mode);
	ret = edp_hw_enable(edp_hw, edp_core);
	if (ret) {
		EDP_ERR("edp core enable failed!\n");
		return RET_FAIL;
	}

	if (src_cap->ssc_support) {
		edp_hw_ssc_enable(edp_hw, edp_core->ssc_en ? true : false);
		if (edp_core->ssc_en)
			edp_hw_ssc_set_mode(edp_hw, edp_core->ssc_mode);
	}

	if (src_cap->enhance_frame_support && sink_cap->enhance_frame_support)
		edp_hw_enhance_frame_enable(edp_hw, true);

	/* set color space, color depth */
	ret = edp_hw_set_video_format(edp_hw, edp_core);
	if (ret < 0)
		return RET_FAIL;

	/* set specific timings */
	ret = edp_hw_set_video_timings(edp_hw, &edp_core->timings);
	if (ret < 0)
		return RET_FAIL;

	/* set transfer unit */
	ret = edp_hw_set_transfer_config(edp_hw, edp_core);
	if (ret < 0)
		return RET_FAIL;

	if (!edp_debug->bypass_training) {
		ret = edp_main_link_setup(edp_hw, edp_core);
		if (ret < 0)
			return RET_FAIL;
	}

	edp_hw_link_start(edp_hw);

	return RET_OK;
}

s32 drm_dp_output_disable(struct sunxi_drm_edp *drm_edp)
{
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	struct edp_tx_cap *src_cap = &drm_edp->source_cap;
	struct edp_rx_cap *sink_cap = &drm_edp->sink_cap;

	if (src_cap->enhance_frame_support && sink_cap->enhance_frame_support)
		edp_hw_enhance_frame_enable(edp_hw, false);

	return edp_hw_link_stop(edp_hw);
}

s32 drm_dp_output_plugin(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct edid *edid;
	char dpcd_rx_buf[576];
	char dpcd_ext_rx_buf[32];
	s32 ret = 0;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_core = &drm_edp->edp_core;

	memset(&dpcd_rx_buf[0], 0, sizeof(dpcd_rx_buf));
	ret = edp_read_dpcd(edp_hw, &dpcd_rx_buf[0]);
	if (ret < 0)
		EDP_WRN("fail to read edp dpcd!\n");
	else
		edp_parse_dpcd(drm_edp, &dpcd_rx_buf[0]);

	/* DP CTS 1.2 Core Rev 1.1, 4.2.2.2 */
	memset(&dpcd_ext_rx_buf[0], 0, sizeof(dpcd_ext_rx_buf));
	ret = edp_read_dpcd_extended(edp_hw, &dpcd_ext_rx_buf[0]);

	/* update lane config to adjust source-sink capacity */
	edp_update_capacity(drm_edp);

	/* DP CTS 1.2 Core Rev 1.1, 4.2.2.8 */
	/* TODO, code for branch device detection */

	edid = drm_do_get_edid(&drm_edp->sdrm.connector, edp_get_edid_block, &drm_edp->edp_hw);
	if (edid == NULL)
		EDP_WRN("fail to read edid\n");
	else {
		edp_parse_edid(drm_edp, edid);
		edp_core->edid = edid;
	}

	return RET_OK;
}

s32 drm_dp_output_plugout(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct edid *edid;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_core = &drm_edp->edp_core;
	edid = edp_core->edid;

	if (edid) {
		edp_edid_put(edid);
		edp_core->edid = NULL;
		drm_connector_update_edid_property(&drm_edp->sdrm.connector, NULL);
	}
	drm_edp->dpcd_parsed = false;

	sink_cap_reset(drm_edp);

	return edp_hw_link_stop(edp_hw);
}

s32 drm_dp_output_runtime_suspend(struct sunxi_drm_edp *drm_edp)
{
	s32 ret = 0;

	edp_phy_enable(drm_edp, false);
	edp_phy_exit(drm_edp);

	ret = edp_clk_enable(drm_edp, false);
	if (ret) {
		EDP_ERR("edp clk disaable fail!!\n");
		return ret;
	}

	if (drm_edp->vcc_regulator)
		regulator_disable(drm_edp->vcc_regulator);

	if (drm_edp->vdd_regulator)
		regulator_disable(drm_edp->vdd_regulator);

	return ret;
}

s32 drm_dp_output_runtime_resume(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core;
	struct edp_debug *edp_debug = &drm_edp->edp_debug;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	s32 ret;

	edp_core = &drm_edp->edp_core;
	if (drm_edp->vdd_regulator) {
		ret = regulator_enable(drm_edp->vdd_regulator);
		if (ret) {
			EDP_ERR("vdd-edp enable failed!\n");
			return ret;
		}
	}

	if (drm_edp->vcc_regulator) {
		ret = regulator_enable(drm_edp->vcc_regulator);
		if (ret) {
			EDP_ERR("vcc-edp enable failed!\n");
			goto ERR_VCC;
		}
	}

	ret = edp_clk_enable(drm_edp, true);
	if (ret) {
		EDP_ERR("edp_clk_enable fail!!\n");
		goto ERR_CLK;
	}

	edp_phy_init(drm_edp);
	edp_phy_enable(drm_edp, true);

	edp_hw_controller_init(edp_hw, edp_core);
	usleep_range(50, 100);

	if (edp_debug->hpd_mask) {
		if (edp_debug->hpd_mask & 0x10 && (edp_debug->hpd_mask != edp_debug->hpd_mask_pre)) {
			edp_debug->hpd_mask_pre = edp_debug->hpd_mask;
			edp_hpd_mask_proc(drm_edp, edp_debug->hpd_mask);
		}
	} else {
		if (!drm_edp->hpd_state_now)
			edp_hotplugout_proc(drm_edp);
		else
			edp_hotplugin_proc(drm_edp);

		EDP_DRV_DBG("drm-edp-hpd = %d\n", drm_edp->hpd_state_now);
		edp_report_hpd_work(drm_edp, drm_edp->hpd_state_now);
		drm_edp->hpd_state = drm_edp->hpd_state_now;
	}

	return ret;

ERR_CLK:
	if (drm_edp->vcc_regulator && !edp_debug->edp_res_lock)
		regulator_disable(drm_edp->vcc_regulator);
ERR_VCC:
	if (drm_edp->vdd_regulator && !edp_debug->edp_res_lock)
		regulator_disable(drm_edp->vdd_regulator);

	return ret;
}

void drm_dp_output_soft_reset(struct sunxi_drm_edp *drm_edp)
{
	struct edp_tx_core *edp_core = &drm_edp->edp_core;
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	s32 ret;

	/* when use as DP, it is not need to judge if dp enabled,
	 * because the resource is alread enabled in probe procedure
	 */
	drm_dp_output_plugout(drm_edp);
	ret = edp_clk_enable(drm_edp, false);

	if (drm_edp->vcc_regulator)
		regulator_disable(drm_edp->vcc_regulator);

	if (drm_edp->vdd_regulator)
		regulator_disable(drm_edp->vdd_regulator);

	edp_report_hpd_work(drm_edp, EDP_HPD_PLUGOUT);
	drm_edp->hpd_state = false;
	msleep(500);

	if (drm_edp->vdd_regulator)
		ret = regulator_enable(drm_edp->vdd_regulator);

	if (drm_edp->vcc_regulator)
		ret = regulator_enable(drm_edp->vcc_regulator);

	ret = edp_clk_enable(drm_edp, true);

	edp_hw_controller_init(edp_hw, edp_core);
}

struct sunxi_edp_output_desc drm_dp_output = {
	.hw_id			= 0,
	.connector_type		= DRM_MODE_CONNECTOR_DisplayPort,
	.bind			= drm_dp_output_bind,
	.unbind			= drm_dp_output_unbind,
	.enable			= drm_dp_output_enable,
	.disable		= drm_dp_output_disable,
	.plugin			= drm_dp_output_plugin,
	.plugout		= drm_dp_output_plugout,
	.runtime_suspend	= drm_dp_output_runtime_suspend,
	.runtime_resume		= drm_dp_output_runtime_resume,
	.soft_reset		= drm_dp_output_soft_reset,
};


static const struct of_device_id drm_edp_match[] = {
	{ .compatible = "allwinner,drm-edp", .data = &drm_edp_output },
	{ .compatible = "allwinner,drm-dp", .data = &drm_dp_output },
	{},
};

static struct device *drm_edp_of_get_tcon(struct device *edp_dev)
{
	struct device_node *node = edp_dev->of_node;
	struct device_node *tcon_tv_node;
	struct device_node *edp_in_tcon;
	struct platform_device *pdev = NULL;
	struct device *tcon_tv_dev = NULL;;

	edp_in_tcon = of_graph_get_endpoint_by_regs(node, 0, 0);
	if (!edp_in_tcon) {
		DRM_ERROR("endpoint edp_in_tcon not fount\n");
		return NULL;
	}

	tcon_tv_node = of_graph_get_remote_port_parent(edp_in_tcon);
	if (!tcon_tv_node) {
		DRM_ERROR("node tcon_tv not fount\n");
		tcon_tv_dev = NULL;
		goto EDP_EP_PUT;
	}

	pdev = of_find_device_by_node(tcon_tv_node);
	if (!pdev) {
		DRM_ERROR(" tcon_tv platform device not fount\n");
		tcon_tv_dev = NULL;
		goto TCON_EP_PUT;
	}

	tcon_tv_dev = &pdev->dev;
	platform_device_put(pdev);

TCON_EP_PUT:
	of_node_put(tcon_tv_node);
EDP_EP_PUT:
	of_node_put(edp_in_tcon);

	return tcon_tv_dev;

}

static struct device_node *drm_edp_of_get_panel_node(struct device *edp_dev)
{
	struct device_node *node = edp_dev->of_node;
	struct device_node *panel_node;
	struct device_node *edp_out_panel;

	edp_out_panel = of_graph_get_endpoint_by_regs(node, 1, 0);
	if (!edp_out_panel) {
		DRM_ERROR("endpoint edp_out_panel not fount\n");
		return NULL;
	}

	panel_node = of_graph_get_remote_port_parent(edp_out_panel);
	if (!panel_node) {
		DRM_ERROR("remote endpoint for edp panel not fount\n");
		goto NODE_PUT;
	}

NODE_PUT:
	of_node_put(edp_out_panel);

	return panel_node;
}

int sunxi_edp_init_sysfs(struct sunxi_drm_edp *drm_edp)
{
	int ret = RET_OK;
	char edp_class_dev_name[10];

	/*Create and add a character device*/
	snprintf(edp_class_dev_name, sizeof(edp_class_dev_name), "edp");
	alloc_chrdev_region(&drm_edp->devid, 0, 1, edp_class_dev_name);/*corely for device number*/
	drm_edp->edp_cdev = cdev_alloc();
	cdev_init(drm_edp->edp_cdev, &edp_fops);
	drm_edp->edp_cdev->owner = THIS_MODULE;
	ret = cdev_add(drm_edp->edp_cdev, drm_edp->devid, 1);/*/proc/device/edp*/
	if (ret) {
		EDP_ERR("edp cdev_add fail!\n");
		ret = RET_FAIL;
		goto OUT;
	}

	/*Create a path: sys/class/edp*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 19, 0)
	drm_edp->edp_class = class_create(THIS_MODULE, edp_class_dev_name);
#else
	drm_edp->edp_class = class_create(edp_class_dev_name);
#endif
	if (IS_ERR(drm_edp->edp_class)) {
		EDP_ERR("edp class_create fail\n");
		ret = RET_FAIL;
		goto ERR_CDEV;
	}

	/*Create a path "sys/class/edp/edp"*/
	drm_edp->edp_class_dev = device_create(drm_edp->edp_class, NULL, drm_edp->devid, drm_edp, edp_class_dev_name);
	if (IS_ERR(drm_edp->edp_class_dev)) {
		EDP_ERR("edp device_create fail\n");
		ret = RET_FAIL;
		goto ERR_CLASS;
	}

	/*Create a path: sys/class/edp/edp/attr*/
	ret = sysfs_create_group(&drm_edp->edp_class_dev->kobj, &edp_attribute_group);
	if (ret) {
		EDP_ERR("edp sysfs_create_group failed!\n");
		ret = RET_FAIL;
		goto ERR_DEV;
	}

	return RET_OK;

ERR_DEV:
	device_destroy(drm_edp->edp_class, drm_edp->devid);
ERR_CLASS:
	class_destroy(drm_edp->edp_class);
ERR_CDEV:
	cdev_del(drm_edp->edp_cdev);
OUT:
	return ret;
}

static int sunxi_edp_audio_hw_params(struct device *dev, void *data,
				  struct hdmi_codec_daifmt *daifmt,
				  struct hdmi_codec_params *params)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;
	int channels = params->channels;
	int sample_rate = params->sample_rate;
	int sample_width = params->sample_width;
	int interface = daifmt->fmt;

	edp_hw_audio_config(edp_hw, interface, channels,
			    sample_width, sample_rate);

	return 0;
}

static int sunxi_edp_audio_mute_stream(struct device *dev, void *data,
				     bool enable, int direction)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_hw_audio_mute(edp_hw, enable, direction);

	return 0;
}

static int sunxi_edp_audio_startup(struct device *dev, void *data)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_hw_audio_enable(edp_hw);

	return 0;
}

static void sunxi_edp_audio_shutdown(struct device *dev, void *data)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_edp_hw_desc *edp_hw = &drm_edp->edp_hw;

	edp_hw_audio_disable(edp_hw);
}

static const struct hdmi_codec_ops sunxi_edp_audio_codec_ops = {
	.hw_params = sunxi_edp_audio_hw_params,
	.audio_startup = sunxi_edp_audio_startup,
	.audio_shutdown = sunxi_edp_audio_shutdown,
	.mute_stream = sunxi_edp_audio_mute_stream,
};

static int sunxi_edp_register_audio_driver(struct device *dev)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct hdmi_codec_pdata codec_data = {
		.ops = &sunxi_edp_audio_codec_ops,
		.max_i2s_channels = edp_hw_audio_get_max_channel(&drm_edp->edp_hw),
		.i2s = 1,
		.spdif = 1,
		.data = drm_edp,
	};

	drm_edp->audio_pdev = platform_device_register_data(dev,
							   SUNXI_EDP_CODEC_DRV_NAME,
							   PLATFORM_DEVID_AUTO,
							   &codec_data,
							   sizeof(codec_data));

	return PTR_ERR_OR_ZERO(drm_edp->audio_pdev);
}

static int sunxi_drm_edp_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct sunxi_drm_edp *drm_edp;
	struct sunxi_drm_device *sdrm;
	struct drm_panel *drm_panel;
	struct device_node *panel_np;
	struct drm_device *drm = (struct drm_device *)data;
	struct device *tcon_tv_dev = NULL;
	struct edp_tx_cap *src_cap;

	const struct of_device_id *match;
	int ret, tcon_id, conn_type, conn_id;

	drm_edp = kzalloc(sizeof(*drm_edp), GFP_KERNEL);
	if (!drm_edp)
		return -ENOMEM;

	match = of_match_device(drm_edp_match, dev);
	if (!match) {
		EDP_DEV_ERR(dev, "Unable to match OF ID\n");
		return -ENODEV;
	}
	drm_edp->desc = (struct sunxi_edp_output_desc *)match->data;

	/* ensure panel driver is load when use as edp */
	if (drm_edp->desc->connector_type == DRM_MODE_CONNECTOR_eDP) {
		panel_np = drm_edp_of_get_panel_node(dev);
		if (panel_np) {
			drm_panel = of_drm_find_panel(panel_np);
			of_node_put(panel_np);
			if (IS_ERR(drm_panel)) {
				DRM_ERROR("edp's panel driver maybe not registered yet!\n");
				return PTR_ERR(drm_panel);
			}
			drm_edp->sdrm.panel = drm_panel;
		} else {
			DRM_ERROR("panel not found for eDp output used!\n");
			return -EINVAL;
		}
	}

	dev_set_drvdata(dev, drm_edp);

	/* ensure ppu open before bind, bind may need to configure controller */
	pm_runtime_enable(dev);

	ret = edp_parse_dts(dev);
	if (ret < 0)
		goto OUT;

	/* get tcon dev, tcon may need to be init before edp init */
	tcon_tv_dev = drm_edp_of_get_tcon(dev);
	if (tcon_tv_dev == NULL) {
		DRM_ERROR("tcon_tv for edp not found!\n");
		ret = RET_FAIL;
		goto OUT;
	}
	tcon_id = sunxi_tcon_of_get_id(tcon_tv_dev);

	conn_type = drm_edp->desc->connector_type;
	conn_id = drm_edp->desc->hw_id;

	drm_edp->boot_enabled = sunxi_drm_check_device_boot_enabled(drm, conn_type, conn_id);
	drm_edp->dev = dev;

	sdrm = &drm_edp->sdrm;
	sdrm->tcon_dev = tcon_tv_dev;
	sdrm->tcon_id = tcon_id;
	sdrm->drm_dev = drm;
	sdrm->hw_id = drm_edp->desc->hw_id;

	ret = sunxi_edp_init_hardware(drm_edp);
	if (ret) {
		DRM_ERROR("edp init hardware fail!\n");
		goto OUT;
	}

	ret = sunxi_edp_init_drm(drm_edp);
	if (ret) {
		DRM_ERROR("edp init drm fail!\n");
		goto OUT;
	}

	if (drm_edp->boot_enabled)
		sunxi_edp_smooth_display_process(drm_edp);

	/* thread should run after drm encoder/connector ready */
	ret = edp_kthread_start(drm_edp);
	if (ret) {
		DRM_ERROR("edp_kthread_start fail!\n");
		goto OUT;
	}

	ret = sunxi_edp_init_sysfs(drm_edp);
	if (ret) {
		DRM_ERROR("edp init sysfs fail!\n");
		goto OUT;
	}

	ret = devm_request_irq(dev, drm_edp->irq, drm_edp_irq_handler, 0, "edp_irq", dev);
	if (ret) {
		EDP_DEV_ERR(dev, "request irq fail: %d\n", ret);
		goto OUT;
	}

	src_cap = &drm_edp->source_cap;
	if (src_cap->audio_support)
		sunxi_edp_register_audio_driver(dev);

	EDP_DRV_DBG("edp bind finish!\n");
	return RET_OK;

OUT:
	pm_runtime_disable(dev);
	return ret;
}

static void sunxi_drm_edp_unbind(struct device *dev, struct device *master,
			      void *data)
{
	struct sunxi_drm_edp *drm_edp = dev_get_drvdata(dev);
	struct sunxi_drm_device *sdrm = &drm_edp->sdrm;
	struct sunxi_edp_output_desc *desc = drm_edp->desc;
	int ret = 0;

	edp_kthread_stop(drm_edp);

	drm_connector_cleanup(&sdrm->connector);
	drm_encoder_cleanup(&sdrm->encoder);

	if (desc->unbind)
		ret = desc->unbind(drm_edp);

	sysfs_remove_group(&drm_edp->edp_class_dev->kobj, &edp_attribute_group);
	device_destroy(drm_edp->edp_class, drm_edp->devid);

	class_destroy(drm_edp->edp_class);
	cdev_del(drm_edp->edp_cdev);
	pm_runtime_disable(drm_edp->dev);
	kfree(drm_edp);
}

static const struct component_ops sunxi_drm_edp_component_ops = {
	.bind = sunxi_drm_edp_bind,
	.unbind = sunxi_drm_edp_unbind,
};

static int drm_edp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct drm_panel *drm_panel;
	struct device_node *panel_np;
	const struct of_device_id *match;
	struct sunxi_edp_output_desc *desc;

	match = of_match_device(drm_edp_match, &pdev->dev);
	if (!match) {
		EDP_DEV_ERR(&pdev->dev, "Unable to match OF ID\n");
		return -ENODEV;
	}

	desc = (struct sunxi_edp_output_desc *)match->data;
	if (desc->connector_type == DRM_MODE_CONNECTOR_eDP) {
		panel_np = drm_edp_of_get_panel_node(&pdev->dev);
		if (panel_np) {
			drm_panel = of_drm_find_panel(panel_np);
			of_node_put(panel_np);
			if (IS_ERR(drm_panel)) {
				DRM_ERROR("edp's panel driver maybe not registered yet!\n");
				return -EPROBE_DEFER;
			}
		}
	}

	ret = component_add(&pdev->dev, &sunxi_drm_edp_component_ops);
	if (ret < 0) {
		DRM_ERROR("failed to add component edp\n");
	}

	return ret;
}

int drm_edp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &sunxi_drm_edp_component_ops);

	return 0;
}


struct platform_driver sunxi_drm_edp_platform_driver = {
	.probe = drm_edp_probe,
	.remove = drm_edp_remove,
	.driver = {
		   .name = "drm_edp",
		   .owner = THIS_MODULE,
		   .of_match_table = drm_edp_match,
	},
};
/*End of File*/
