// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Onsemi FSA4480(BCT4480) USB Type-C Analog Audio Switch driver
 *
 * Copyright (C) 2023-2024 Allwinner Technology Co., Ltd.
 * Copyright (C) 2021-2022 Linaro Ltd.
 * Copyright (C) 2018-2020 The Linux Foundation
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>

#define FSA4480_DEVICE_ID	0x00
#define FSA4480_SWITCH_ENABLE	0x04
#define FSA4480_SWITCH_SELECT	0x05
#define FSA4480_SWITCH_STATUS1	0x07
#define FSA4480_SLOW_L		0x08
#define FSA4480_SLOW_R		0x09
#define FSA4480_SLOW_MIC	0x0a
#define FSA4480_SLOW_SENSE	0x0b
#define FSA4480_SLOW_GND	0x0c
#define FSA4480_DELAY_L_R	0x0d
#define FSA4480_DELAY_L_MIC	0x0e
#define FSA4480_DELAY_L_SENSE	0x0f
#define FSA4480_DELAY_L_AGND	0x10
#define FSA4480_FUNCTION_ENABLE	0x12
#define FSA4480_AUDIO_JACK_STATUS	0x17
#define FSA4480_RESET		0x1e
#define FSA4480_MAX_REGISTER	0x1f

#define FSA4480_VENDOR_ID	GENMASK(7, 6)
#define FSA4480_VERSION_ID	GENMASK(5, 3)
#define FSA4480_REV_ID		GENMASK(2, 0)

#define FSA4480_ENABLE_DEVICE	BIT(7)
#define FSA4480_ENABLE_SBU	GENMASK(6, 5)
#define FSA4480_ENABLE_USB	GENMASK(4, 3)
#define FSA4480_ENABLE_SENSE	BIT(2)
#define FSA4480_ENABLE_MIC	BIT(1)
#define FSA4480_ENABLE_AGND	BIT(0)

#define FSA4480_SEL_SBU_REVERSE	GENMASK(6, 5)
#define FSA4480_SEL_USB		GENMASK(4, 3)
#define FSA4480_SEL_SENSE	BIT(2)
#define FSA4480_SEL_MIC		BIT(1)
#define FSA4480_SEL_AGND	BIT(0)

#define FSA4480_ENABLE_AUTO_JACK_DETECT	BIT(0)

/* *
 * 3: 4 pole SBU2 to MIC, SBU1 to Audio Ground
 * 2: 4 pole SBU1 to MIC, SBU2 to Audio Ground
 * 1: 3 pole
 * 0: No Audio Accessory
 * */
#define FSA4480_AUDIO_JACK_MASK	GENMASK(3, 0)

/* Vendor Device ID */
enum vendor_id {
	/* FSA4480_DEVICE_ID = 0x09, */
	BCT4480_DEVICE_ID = 0x09,
	ASW5480_DEVICE_ID = 0x59,
};

struct fsa4480 {
	struct i2c_client *client;

	/* used to serialize concurrent change requests */
	struct mutex lock;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	struct typec_switch *sw;
	struct typec_mux *mux;
#else
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
#endif
	struct extcon_dev *edev;

	struct regmap *regmap;

	enum typec_orientation orientation;
	unsigned long mode;
	unsigned int svid;

	u8 cur_enable;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_MAX_REGISTER,
	/* Accesses only done under fsa4480->lock */
	.disable_locking = true,
};

static const unsigned int fsa4480_usb_extcon_cable[] = {
	EXTCON_JACK_HEADPHONE,
	EXTCON_NONE,
};

static inline const char *to_vendor_name(int id)
{
	if (id == BCT4480_DEVICE_ID)
		return "BCT4480";
	else if (id == ASW5480_DEVICE_ID)
		return "ASW5480";
	else
		return "FSA4480";
}

static int fsa4480_set(struct fsa4480 *fsa)
{
	bool reverse = (fsa->orientation == TYPEC_ORIENTATION_REVERSE);
	u8 enable = FSA4480_ENABLE_DEVICE;
	u8 sel = 0;

	/* USB Mode */
	if (fsa->mode < TYPEC_STATE_MODAL ||
	    (!fsa->svid && (fsa->mode == TYPEC_MODE_USB2 ||
			    fsa->mode == TYPEC_MODE_USB3))) {
		enable |= FSA4480_ENABLE_USB;
		sel = FSA4480_SEL_USB;
	} else if (fsa->svid) {
		switch (fsa->mode) {
		/* DP Only */
		case TYPEC_DP_STATE_C:
		case TYPEC_DP_STATE_E:
			enable |= FSA4480_ENABLE_SBU;
			if (reverse)
				sel = FSA4480_SEL_SBU_REVERSE;
			break;

		/* DP + USB */
		case TYPEC_DP_STATE_D:
		case TYPEC_DP_STATE_F:
			enable |= FSA4480_ENABLE_USB | FSA4480_ENABLE_SBU;
			sel = FSA4480_SEL_USB;
			if (reverse)
				sel |= FSA4480_SEL_SBU_REVERSE;
			break;

		default:
			return -EOPNOTSUPP;
		}
	} else if (fsa->mode == TYPEC_MODE_AUDIO) {
		/* Audio Accessory Mode, setup to auto Jack Detection */
		enable |= FSA4480_ENABLE_USB | FSA4480_ENABLE_AGND;
	} else
		return -EOPNOTSUPP;

	if (fsa->cur_enable & FSA4480_ENABLE_SBU) {
		/* Disable SBU output while re-configuring the switch */
		regmap_write(fsa->regmap, FSA4480_SWITCH_ENABLE,
			     fsa->cur_enable & ~FSA4480_ENABLE_SBU);

		/* 35us to allow the SBU switch to turn off */
		usleep_range(35, 1000);
	}

	regmap_write(fsa->regmap, FSA4480_SWITCH_SELECT, sel);
	regmap_write(fsa->regmap, FSA4480_SWITCH_ENABLE, enable);

	/* Start AUDIO JACK DETECTION to setup MIC, AGND & Sense muxes */
	if (enable & FSA4480_ENABLE_AGND)
		regmap_write(fsa->regmap, FSA4480_FUNCTION_ENABLE,
			     FSA4480_ENABLE_AUTO_JACK_DETECT);

	if (enable & FSA4480_ENABLE_SBU) {
		/* 15us to allow the SBU switch to turn on again */
		usleep_range(15, 1000);
	}

	fsa->cur_enable = enable;

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
static int fsa4480_switch_set(struct typec_switch *sw,
			      enum typec_orientation orientation)
#else
static int fsa4480_switch_set(struct typec_switch_dev *sw,
			      enum typec_orientation orientation)
#endif
{
	struct fsa4480 *fsa = typec_switch_get_drvdata(sw);
	int ret = 0;

	mutex_lock(&fsa->lock);

	dev_info(&fsa->client->dev, "set sw conf %d -> %d [%s]\n",
		 fsa->orientation, orientation,
		 orientation == TYPEC_ORIENTATION_REVERSE ? "REVERSE" : "NORMAL");

	if (fsa->orientation != orientation) {
		fsa->orientation = orientation;

		ret = fsa4480_set(fsa);
	}

	mutex_unlock(&fsa->lock);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
static int fsa4480_mux_set(struct typec_mux *mux, struct typec_mux_state *state)
#else
static int fsa4480_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
#endif
{
	struct fsa4480 *fsa = typec_mux_get_drvdata(mux);
	union extcon_property_value property;
	int val = 0;
	int ret = 0;

	mutex_lock(&fsa->lock);

	dev_info(&fsa->client->dev, "set mux conf %lu -> %lu [%s]\n",
		 fsa->mode, state->mode,
		 state->alt ? "Altmode" : state->mode == TYPEC_MODE_AUDIO ? "Audio" : "USB");

	if (fsa->mode != state->mode) {
		fsa->mode = state->mode;

		if (state->alt)
			fsa->svid = state->alt->svid;
		else
			fsa->svid = 0; // No SVID

		ret = fsa4480_set(fsa);
	}

	if (fsa->mode == TYPEC_MODE_AUDIO) {
		/* 2000us to allow the report audio jack status */
		usleep_range(2000, 2500);
		ret = regmap_read(fsa->regmap, FSA4480_AUDIO_JACK_STATUS, &val);
		if (ret || !val)
			dev_err(&fsa->client->dev, "delay 2000us for audio jack status, ret = %d, val = %d\n", ret, val);

		property.intval = val & FSA4480_AUDIO_JACK_MASK;
		// extcon_set_property(fsa->edev, EXTCON_JACK_HEADPHONE, EXTCON_PROP_JACK_POLE, property);

		extcon_set_state_sync(fsa->edev, EXTCON_JACK_HEADPHONE, true);
	} else {
		extcon_set_state_sync(fsa->edev, EXTCON_JACK_HEADPHONE, false);
	}

	mutex_unlock(&fsa->lock);

	return ret;
}

static void fsa4480_extcon_state_init(struct fsa4480 *fsa)
{
	if (fsa->mode == TYPEC_MODE_AUDIO)
		extcon_set_state_sync(fsa->edev, EXTCON_JACK_HEADPHONE, true);
	else
		extcon_set_state_sync(fsa->edev, EXTCON_JACK_HEADPHONE, false);

	// extcon_set_property_capability(fsa->edev, EXTCON_JACK_HEADPHONE, EXTCON_PROP_JACK_POLE);
}

static int fsa4480_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = { };
	struct fsa4480 *fsa;
	int val = 0;
	int ret;

	fsa = devm_kzalloc(dev, sizeof(*fsa), GFP_KERNEL);
	if (!fsa)
		return -ENOMEM;

	fsa->client = client;
	mutex_init(&fsa->lock);

	fsa->regmap = devm_regmap_init_i2c(client, &fsa4480_regmap_config);
	if (IS_ERR(fsa->regmap))
		return dev_err_probe(dev, PTR_ERR(fsa->regmap), "failed to initialize regmap\n");

	ret = regmap_read(fsa->regmap, FSA4480_DEVICE_ID, &val);
	if (ret || !val)
		return dev_err_probe(dev, -ENODEV, "FSA4480 not found\n");

	dev_info(dev, "Found %s v%lu.%lu (Vendor ID = %lu)\n", to_vendor_name(val),
		 FIELD_GET(FSA4480_VERSION_ID, val),
		 FIELD_GET(FSA4480_REV_ID, val),
		 FIELD_GET(FSA4480_VENDOR_ID, val));

	/* Safe mode */
	fsa->cur_enable = FSA4480_ENABLE_DEVICE | FSA4480_ENABLE_USB;
	fsa->mode = TYPEC_STATE_SAFE;
	fsa->orientation = TYPEC_ORIENTATION_NONE;

	/* set default settings */
	regmap_write(fsa->regmap, FSA4480_SLOW_L, 0x00);
	regmap_write(fsa->regmap, FSA4480_SLOW_R, 0x00);
	regmap_write(fsa->regmap, FSA4480_SLOW_MIC, 0x00);
	regmap_write(fsa->regmap, FSA4480_SLOW_SENSE, 0x00);
	regmap_write(fsa->regmap, FSA4480_SLOW_GND, 0x00);
	regmap_write(fsa->regmap, FSA4480_DELAY_L_R, 0x00);
	regmap_write(fsa->regmap, FSA4480_DELAY_L_MIC, 0x00);
	regmap_write(fsa->regmap, FSA4480_DELAY_L_SENSE, 0x00);
	regmap_write(fsa->regmap, FSA4480_DELAY_L_AGND, 0x09);
	regmap_write(fsa->regmap, FSA4480_SWITCH_SELECT, FSA4480_SEL_USB);
	regmap_write(fsa->regmap, FSA4480_SWITCH_ENABLE, fsa->cur_enable);

	sw_desc.drvdata = fsa;
	sw_desc.fwnode = dev_fwnode(dev);
	sw_desc.set = fsa4480_switch_set;

	fsa->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(fsa->sw))
		return dev_err_probe(dev, PTR_ERR(fsa->sw), "failed to register typec switch\n");

	mux_desc.drvdata = fsa;
	mux_desc.fwnode = dev_fwnode(dev);
	mux_desc.set = fsa4480_mux_set;

	fsa->mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(fsa->mux)) {
		typec_switch_unregister(fsa->sw);
		return dev_err_probe(dev, PTR_ERR(fsa->mux), "failed to register typec mux\n");
	}

	fsa->edev = devm_extcon_dev_allocate(dev, fsa4480_usb_extcon_cable);
	if (IS_ERR(fsa->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, fsa->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}
	fsa4480_extcon_state_init(fsa);

	i2c_set_clientdata(client, fsa);
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static int fsa4480_remove(struct i2c_client *client)
{
	struct fsa4480 *fsa = i2c_get_clientdata(client);

	typec_mux_unregister(fsa->mux);
	typec_switch_unregister(fsa->sw);
	return 0;
}
#else
static void fsa4480_remove(struct i2c_client *client)
{
	struct fsa4480 *fsa = i2c_get_clientdata(client);

	typec_mux_unregister(fsa->mux);
	typec_switch_unregister(fsa->sw);
}
#endif

static const struct i2c_device_id fsa4480_table[] = {
	{ "fsa4480" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fsa4480_table);

static const struct of_device_id fsa4480_of_table[] = {
	{ .compatible = "fcs,fsa4480" },
	{ }
};
MODULE_DEVICE_TABLE(of, fsa4480_of_table);

static struct i2c_driver fsa4480_driver = {
	.driver = {
		.name = "fsa4480",
		.of_match_table = fsa4480_of_table,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
	.probe_new	= fsa4480_probe,
#else
	.probe		= fsa4480_probe,
#endif
	.remove		= fsa4480_remove,
	.id_table	= fsa4480_table,
};
module_i2c_driver(fsa4480_driver);

MODULE_ALIAS("platform:fsa4480-i2c-driver");
MODULE_DESCRIPTION("ON Semiconductor FSA4480 driver");
MODULE_AUTHOR("kanghoupeng<kanghoupeng@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");