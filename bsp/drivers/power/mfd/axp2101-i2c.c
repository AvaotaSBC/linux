/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * I2C driver for the X-Powers' Power Management ICs
 *
 * AXP20x typically comprises an adaptive USB-Compatible PWM charger, BUCK DC-DC
 * converters, LDOs, multiple 12-bit ADCs of voltage, current and temperature
 * as well as configurable GPIOs.
 *
 * This driver supports the I2C variants.
 *
 * Copyright (C) 2014 Carlo Caione
 *
 * Author: Carlo Caione <carlo@caione.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <power/axp2101.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>

static int axp20x_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct axp20x_dev *axp20x;
	int ret;

	axp20x = devm_kzalloc(&i2c->dev, sizeof(*axp20x), GFP_KERNEL);
	if (!axp20x)
		return -ENOMEM;

	axp20x->dev = &i2c->dev;
	axp20x->irq = i2c->irq;
	dev_set_drvdata(axp20x->dev, axp20x);

	ret = axp20x_match_device(axp20x);
	if (ret)
		return ret;

	axp20x->regmap = devm_regmap_init_i2c(i2c, axp20x->regmap_cfg);
	if (IS_ERR(axp20x->regmap)) {
		ret = PTR_ERR(axp20x->regmap);
		PMIC_DEV_ERR(&i2c->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	return axp20x_device_probe(axp20x);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int axp20x_i2c_remove(struct i2c_client *i2c)
{
	struct axp20x_dev *axp20x = i2c_get_clientdata(i2c);
	return axp20x_device_remove(axp20x);
}
#else
static void axp20x_i2c_remove(struct i2c_client *i2c)
{
	struct axp20x_dev *axp20x = i2c_get_clientdata(i2c);

	axp20x_device_remove(axp20x);
}
#endif

static void axp20x_i2c_shutdown(struct i2c_client *i2c)
{
	struct axp20x_dev *axp20x = i2c_get_clientdata(i2c);

	axp20x_device_shutdown(axp20x);

	return;
}

static const struct of_device_id axp20x_i2c_of_match[] = {
	{ .compatible = "x-powers,axp152", .data = (void *)AXP152_ID },
	{ .compatible = "x-powers,axp202", .data = (void *)AXP202_ID },
	{ .compatible = "x-powers,axp209", .data = (void *)AXP209_ID },
	{ .compatible = "x-powers,axp221", .data = (void *)AXP221_ID },
	{ .compatible = "x-powers,axp2101", .data = (void *)AXP2101_ID },
	{ .compatible = "x-powers,axp15", .data = (void *)AXP15_ID },
	{ .compatible = "x-powers,axp1530", .data = (void *)AXP1530_ID },
	{ .compatible = "x-powers,axp858", .data = (void *)AXP858_ID },
	{ .compatible = "x-powers,axp803", .data = (void *)AXP803_ID },
	{ .compatible = "x-powers,axp2202", .data = (void *)AXP2202_ID },
	{ .compatible = "x-powers,axp806", .data = (void *)AXP806_ID },
	{ .compatible = "x-powers,axp8191", .data = (void *)AXP8191_ID },
	{ },
};
MODULE_DEVICE_TABLE(of, axp20x_i2c_of_match);

/*
 * This is useless for OF-enabled devices, but it is needed by I2C subsystem
 */
static const struct i2c_device_id axp20x_i2c_id[] = {
	{ },
};
MODULE_DEVICE_TABLE(i2c, axp20x_i2c_id);

static const struct acpi_device_id axp20x_i2c_acpi_match[] = {
	{
		.id = "INT33F4",
		.driver_data = AXP288_ID,
	},
	{ },
};
MODULE_DEVICE_TABLE(acpi, axp20x_i2c_acpi_match);

static struct i2c_driver axp2101_i2c_driver = {
	.driver = {
		.name	= "axp20x-i2c",
		.of_match_table	= of_match_ptr(axp20x_i2c_of_match),
		.acpi_match_table = ACPI_PTR(axp20x_i2c_acpi_match),
	},
	.probe		= axp20x_i2c_probe,
	.remove		= axp20x_i2c_remove,
	.shutdown	= axp20x_i2c_shutdown,
	.id_table	= axp20x_i2c_id,
};

static int __init axp2101_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&axp2101_i2c_driver);
	if (ret != 0) {
		PMIC_ERR("axp2101 i2c registration failed %d\n", ret);
		return ret;
	}

	return 0;
}
subsys_initcall(axp2101_i2c_init);

static void __exit axp2101_i2c_exit(void)
{
	i2c_del_driver(&axp2101_i2c_driver);
}
module_exit(axp2101_i2c_exit);

MODULE_DESCRIPTION("PMIC MFD I2C driver for AXP20X");
MODULE_AUTHOR("Carlo Caione <carlo@caione.org>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");