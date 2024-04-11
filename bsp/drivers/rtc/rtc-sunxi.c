/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * An RTC driver for Sunxi Platform of Allwinner SoC
 *
 * Copyright (c) 2020, Martin <wuyan@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

/* #define DEBUG */	/* Enable dev_dbg */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/version.h>
#include "rtc-sunxi.h"
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 10, 149)
#include <linux/panic_notifier.h>
#endif

#define LOSC_CTRL_REG			0x00
#define RTC_DAY_ACCESS(day_access_bit)		BIT(day_access_bit)  /* 1: the DAY setting operation is in progress */
#define RTC_HHMMSS_ACCESS(hhmmss_access_bit)		BIT(hhmmss_access_bit)  /* 1: the HH-MM-SS setting operation is in progress */

#define LOSC_AUTO_SWT_STA_REG		0x0004
#define LOSC_STATUS			BIT(0)

#define RTC_DAY_REG			0x0010
#define RTC_HHMMSS_REG			0x0014

#define ALARM0_DAY_REG			0x0020
#define ALARM0_HHMMSS_REG		0x0024

#define ALARM0_ENABLE_REG		0x0028
#define ALARM0_ENABLE			BIT(0)

#define ALARM0_IRQ_EN_REG		0x002c
#define ALARM0_IRQ_EN			BIT(0)

#define ALARM0_IRQ_STA_REG		0x0030
#define ALARM0_IRQ_PEND			BIT(0)

#define ALARM0_CONFIG_REG		0x0050
#define ALARM0_OUTPUT_EN		BIT(0)

#define GET_VAL_FROM_REG(reg, mask, shift)	(((reg) & ((mask) << (shift))) >> (shift))
#define PUT_VAL_IN_REG(val, mask, shift)	(((val) & (mask)) << (shift))

#define GET_DAY_FROM_REG(x)		GET_VAL_FROM_REG(x, 0x1f, 0)
#define GET_MON_FROM_REG(x)		GET_VAL_FROM_REG(x, 0x0f, 8)
#define GET_YEAR_FROM_REG(x, d)		GET_VAL_FROM_REG(x, (d)->year_mask, (d)->year_shift)

#define GET_SEC_FROM_REG(x)		GET_VAL_FROM_REG(x, 0x3f, 0)
#define GET_MIN_FROM_REG(x)		GET_VAL_FROM_REG(x, 0x3f, 8)
#define GET_HOUR_FROM_REG(x)		GET_VAL_FROM_REG(x, 0x1f, 16)

#define PUT_DAY_IN_REG(x)		PUT_VAL_IN_REG(x, 0x1f, 0)
#define PUT_MON_IN_REG(x)		PUT_VAL_IN_REG(x, 0x0f, 8)
#define PUT_YEAR_IN_REG(x, d)		PUT_VAL_IN_REG(x, (d)->year_mask, (d)->year_shift)
#define PUT_LEAP_IN_REG(is_leap, shift)	PUT_VAL_IN_REG(is_leap, 0x01, shift)

#define PUT_SEC_IN_REG(x)		PUT_VAL_IN_REG(x, 0x3f, 0)
#define PUT_MIN_IN_REG(x)		PUT_VAL_IN_REG(x, 0x3f, 8)
#define PUT_HOUR_IN_REG(x)		PUT_VAL_IN_REG(x, 0x1f, 16)

#define SEC_IN_MIN			(60)
#define SEC_IN_HOUR			(60 * SEC_IN_MIN)
#define SEC_IN_DAY			(24 * SEC_IN_HOUR)
#define XO_CTRL_REG			0x160
#define XO_CTRL_MASK			0x0F000000
#define XO_ICTRL_SHIFT			24

/*
 * NOTE: To know the valid range of each member in 'struct rtc_time', see 'struct tm' in include/linux/time.h
 *       See also rtc_valid_tm() in drivers/rtc/lib.c.
 */

/*
 * @tm_year:   the year in kernel. See '(struct rtc_time).tm_year' and '(struct tm).tm_year',
 *	       offset to 1900. @tm_year=0 means year 1900
 * @real_year: the year in human world.
 *	       for example 1900
 * @hw_year:   the year in hardware register, offset to @hw_year_base.
 *	       @hw_year=0 means year @hw_year_base
 * @hw_year_base: the base of @hw_year, for example 1970.
 *		  let's say @hw_year=30 && @hw_year_base=1970, then @real_year should be 2000.
 */
#define TM_YEAR_BASE					(1900)
#define TM_YEAR_TO_HW_YEAR(tm_year, hw_year_base)	((tm_year) + TM_YEAR_BASE - (hw_year_base))
#define HW_YEAR_TO_TM_YEAR(hw_year, hw_year_base)	((hw_year) + (hw_year_base) - TM_YEAR_BASE)
#define TM_YEAR_TO_REAL_YEAR(tm_year)			((tm_year) + TM_YEAR_BASE)
#define REAL_YEAR_TO_TM_YEAR(real_year)			((real_year) - TM_YEAR_BASE)
#define HW_YEAR_TO_REAL_YEAR(hw_year, hw_year_base)	((hw_year) + (hw_year_base))
#define REAL_YEAR_TO_HW_YEAR(real_year, hw_year_base)	((real_year) - (hw_year_base))

/*
 * @tm_mon:   the month in kernel. See '(struct rtc_time).tm_mon' and '(struct tm).tm_mon', In the range 0 to 11
 * @real_mon: the month in human world, in the range 1 to 12
 * @hw_mon:   the month in hardware register, in the range 1 to 12
 */
#define TM_MON_TO_REAL_MON(tm_mon)	((tm_mon) + 1)
#define REAL_MON_TO_TM_MON(real_mon)	((real_mon) - 1)
#define TM_MON_TO_HW_MON(tm_mon)	((tm_mon) + 1)
#define HW_MON_TO_TM_MON(hw_mon)	((hw_mon) - 1)
#define REAL_MON_TO_HW_MON(real_mon)	(real_mon)
#define HW_MON_TO_REAL_MON(hw_mon)	(hw_mon)

/* Only valid when the "type == SUNXI_RTC_TYPE_A" */
/*
 * @hw_day:   the day in hardware register, in the range TYPE_A_HW_DAY_MIN to TYPE_A_HW_DAY_MAX.
 *	      let's say @hw_year_base=1970, then @hw_day=TYPE_A_HW_DAY_MIN means 1970-01-01.
 * @real_day: the day in human world. starts from 1 (TYPE_A_REAL_DAY_MIN).
 *	      It's the number of day since the first day of @hw_year_base.
 *	      let's say @hw_year_base=1970, then @real_day=1 means 1970-01-01
 */
#define TYPE_A_HW_DAY_MIN			(1)
/*
 * AW1855 RTC SPEC said the DAY register starts from 0, but actually it's 1.
 * The fact is, RTC_DAY_REG starts from 1, but ALARM0_DAY_REG starts from 0.
 * See errata__fix_alarm_day_reg_default_value() for more information.
 */
#define TYPE_A_HW_DAY_MAX			(65535)
#define TYPE_A_REAL_DAY_MIN			(1)
#define TYPE_A_HW_DAY_TO_REAL_DAY(hw_day)	((hw_day) + (TYPE_A_REAL_DAY_MIN - TYPE_A_HW_DAY_MIN))
#define TYPE_A_REAL_DAY_TO_HW_DAY(real_day)	((real_day) - (TYPE_A_REAL_DAY_MIN - TYPE_A_HW_DAY_MIN))
#define TYPE_A_REAL_YEAR_MAX(min_year)		((min_year) + (TYPE_A_HW_DAY_MAX - TYPE_A_HW_DAY_MIN + 1) / 366 - 1)
/* '366' and '-1' make the result smaller, so that the time to be set won't exceed the hardware capability */

/* Only valid when the "type == SUNXI_RTC_TYPE_B" */
#define TYPE_B_HW_YEAR_MAX		(127)
#define TYPE_B_REAL_YEAR_MAX(min_year)	((min_year) + TYPE_B_HW_YEAR_MAX)
#define TYPE_B_HW_SECONDS_MAX			0xFFFFFFFF
#define RTC_YYMMDD_REG				0x0010
#define ALARM0_COUNTER_REG			0x0020
#define ALARM0_CUR_VAL_REG			0x0024

/* when gpr base address is same with rtc, set gpr_base_address to GPR_BASEADDR_SAMEWITH_RTC */
#define GPR_BASEADDR_SAMEWITH_RTC	0x0

static inline void rtc_reg_write(struct sunxi_rtc_dev *chip, u32 reg, u32 val)
{
	writel(val, chip->base + reg);
}
static inline u32 rtc_reg_read(struct sunxi_rtc_dev *chip, u32 reg)
{
	return readl(chip->base + reg);
}

static short month_days[2][13] = {
	{0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
	{0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},  /* Leap year */
};

/*
 * Convert real @day based on @year_base-01-01 to real @yyyy+@mm+@dd.
 * @day: The number of day from @year_base-01-01. The minimum value is TYPE_A_REAL_DAY_MIN.
 *       For example, if @year_base=1970, then @day=TYPE_A_REAL_DAY_MIN means 1970-01-01.
 *       Note that 0 is invalid.
 * @year_base: the base year of @day, for example 1970.
 * @yyyy: real year (output), for example 2000
 * @mm: real month (output), 1 ~ 12
 * @dd: real date (output),  1 ~ 31
 */
static int real_day_to_real_ymd(u32 day, u32 year_base, u32 *yyyy, u32 *mm, u32 *dd)
{
	u32 y, m, d, leap;

	if (day < TYPE_A_REAL_DAY_MIN) {
		printk("real_day_to_real_ymd(): Invalid argument\n");
		*yyyy = *mm = *dd = 0;
		return -EINVAL;
	}

	/* Calc year */
	for (y = year_base; day > (is_leap_year(y) ? 366 : 365); y++)
		day -= (is_leap_year(y) ? 366 : 365);

	/* Calc month */
	leap = is_leap_year(y);
	for (m = 1; day > month_days[leap][m]; m++)
		day -= month_days[leap][m];

	/* Calc date */
	d = day;

	/* Return result */
	*yyyy = y;
	*mm = m;
	*dd = d;

	return 0;
}

/*
 * Convert real @yyyy+@mm+@dd to @day based on @year_base-01-01.
 * Arguments are the same as real_day_to_real_ymd()
 */
static int real_ymd_to_real_day(u32 yyyy, u32 mm, u32 dd, u32 year_base, u32 *day)
{
	u32 leap = is_leap_year(yyyy);
	u32 i;

	*day = 0;  /* 0 is invalid */

	if (yyyy < year_base || mm < 1 || mm > 12 || dd < 1 || dd > month_days[leap][mm]) {
		printk("real_ymd_to_real_day(): Invalid argument\n");
		return -EINVAL;
	}

	for (i = year_base; i < yyyy; i++)
		*day += is_leap_year(i) ? 366 : 365;
	for (i = 1; i < mm; i++)
		*day += month_days[leap][i];
	*day += dd;

	return 0;
}

/* Convert register value @hw_day and @hw_hhmmss to @tm */
static int hw_day_hhmmss_to_tm(const struct sunxi_rtc_data *data,
				u32 hw_day, u32 hw_hhmmss, struct rtc_time *tm)
{
	int err;
	u32 real_year, real_mon, real_mday;
	u32 real_day = TYPE_A_HW_DAY_TO_REAL_DAY(hw_day);

	err = real_day_to_real_ymd(real_day, data->min_year, &real_year, &real_mon, &real_mday);
	if (err)
		return err;

	tm->tm_year = REAL_YEAR_TO_TM_YEAR(real_year);
	tm->tm_mon  = REAL_MON_TO_TM_MON(real_mon);
	tm->tm_mday = real_mday;
	tm->tm_hour = GET_HOUR_FROM_REG(hw_hhmmss);
	tm->tm_min  = GET_MIN_FROM_REG(hw_hhmmss);
	tm->tm_sec  = GET_SEC_FROM_REG(hw_hhmmss);

	return 0;
}

/* Convert @tm to register value @hw_day and @hw_hhmmss */
static int tm_to_hw_day_hhmmss(const struct sunxi_rtc_data *data,
			const struct rtc_time *tm, u32 *hw_day, u32 *hw_hhmmss)
{
	int err;
	u32 real_year, real_mon, real_mday;
	u32 real_day;

	real_year = TM_YEAR_TO_REAL_YEAR(tm->tm_year);
	real_mon = TM_MON_TO_REAL_MON(tm->tm_mon);
	real_mday = tm->tm_mday;
	err = real_ymd_to_real_day(real_year, real_mon, real_mday, data->min_year, &real_day);
	if (err)
		return err;
	*hw_day = TYPE_A_REAL_DAY_TO_HW_DAY(real_day);
	*hw_hhmmss = PUT_HOUR_IN_REG(tm->tm_hour) |
		     PUT_MIN_IN_REG(tm->tm_min) |
		     PUT_SEC_IN_REG(tm->tm_sec);
	return 0;
}

/* Convert register value @hw_yymmdd and @hw_hhmmss to @tm */
static void hw_yymmdd_hhmmss_to_tm(const struct sunxi_rtc_data *data,
				u32 hw_yymmdd, u32 hw_hhmmss, struct rtc_time *tm)
{
	u32 hw_year = GET_YEAR_FROM_REG(hw_yymmdd, data);
	u32 hw_mon  = GET_MON_FROM_REG(hw_yymmdd);

	tm->tm_year = HW_YEAR_TO_TM_YEAR(hw_year, data->min_year);
	tm->tm_mon  = HW_MON_TO_TM_MON(hw_mon);
	tm->tm_mday = GET_DAY_FROM_REG(hw_yymmdd);
	tm->tm_hour = GET_HOUR_FROM_REG(hw_hhmmss);
	tm->tm_min  = GET_MIN_FROM_REG(hw_hhmmss);
	tm->tm_sec  = GET_SEC_FROM_REG(hw_hhmmss);
}

/* Convert @tm to register value @hw_yymmdd and @hw_hhmmss */
static void tm_to_hw_yymmdd_hhmmss(const struct sunxi_rtc_data *data,
			const struct rtc_time *tm, u32 *hw_yymmdd, u32 *hw_hhmmss)
{
	u32 hw_year = TM_YEAR_TO_HW_YEAR(tm->tm_year, data->min_year);
	u32 hw_mon  = TM_MON_TO_HW_MON(tm->tm_mon);
	u32 real_year = TM_YEAR_TO_REAL_YEAR(tm->tm_year);

	*hw_yymmdd = PUT_LEAP_IN_REG(is_leap_year(real_year), data->leap_shift) |
		     PUT_YEAR_IN_REG(hw_year, data) |
		     PUT_MON_IN_REG(hw_mon) |
		     PUT_DAY_IN_REG(tm->tm_mday);
	*hw_hhmmss = PUT_HOUR_IN_REG(tm->tm_hour) |
		     PUT_MIN_IN_REG(tm->tm_min) |
		     PUT_SEC_IN_REG(tm->tm_sec);
}

/* Wait until the @bits in @reg are cleared, or else return -ETIMEDOUT */
static int wait_bits_cleared(struct sunxi_rtc_dev *chip, int reg,
			  unsigned int bits, unsigned int ms_timeout)
{
	const unsigned long timeout = jiffies + msecs_to_jiffies(ms_timeout);
	u32 val;

	do {
		val = rtc_reg_read(chip, reg);
		if ((val & bits) == 0) {
			/*
			 * For sunxi chips ahead of sun8iw16p1's platform, we must wait at least 2ms
			 * after the access bits of LOSC_CTRL_REG's HMS/YMD have been cleared.
			 */
			if ((reg == LOSC_CTRL_REG) &&
			    ((bits == RTC_HHMMSS_ACCESS(chip->data->hhmmss_access_bit)) ||
					(bits == RTC_DAY_ACCESS(chip->data->day_access_bit))))
				msleep(2);
			return 0;
		}
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static __maybe_unused void show_tm(struct device *dev, struct rtc_time *tm)
{
	dev_info(dev, "%04d-%02d-%02d %02d:%02d:%02d\n",
		TM_YEAR_TO_REAL_YEAR(tm->tm_year),
		TM_MON_TO_REAL_MON(tm->tm_mon), tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
}

static int sunxi_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	u32 hw_day, hw_hhmmss, hw_yymmdd;
	u32 val;
	int err;

	val = rtc_reg_read(chip, LOSC_AUTO_SWT_STA_REG);
	if ((val & LOSC_STATUS) == 0)
		dev_warn(dev, "Warning: Using internal RC 16M clock source. Time may be inaccurate!\n");

	switch (chip->data->type) {
	case SUNXI_RTC_TYPE_A:
		do {  /* read again in case it changes */
			hw_day = rtc_reg_read(chip, RTC_DAY_REG);
			hw_hhmmss = rtc_reg_read(chip, RTC_HHMMSS_REG);
		} while ((hw_day != rtc_reg_read(chip, RTC_DAY_REG)) ||
				(hw_hhmmss != rtc_reg_read(chip, RTC_HHMMSS_REG)));
		err = hw_day_hhmmss_to_tm(chip->data, hw_day, hw_hhmmss, tm);
		if (err)
			return err;
		break;
	case SUNXI_RTC_TYPE_B:
		do {  /* read again in case it changes */
			hw_yymmdd = rtc_reg_read(chip, RTC_YYMMDD_REG);
			hw_hhmmss = rtc_reg_read(chip, RTC_HHMMSS_REG);
		} while ((hw_yymmdd != rtc_reg_read(chip, RTC_YYMMDD_REG)) ||
			(hw_hhmmss != rtc_reg_read(chip, RTC_HHMMSS_REG)));
		hw_yymmdd_hhmmss_to_tm(chip->data, hw_yymmdd, hw_hhmmss, tm);
		break;
	default:
		dev_err(dev, "Invalid chip->data->type = %d\n", chip->data->type);
		return -EINVAL;
	}

	err = rtc_valid_tm(tm);
	if (err) {
		dev_err(dev, "sunxi_rtc_gettime(): Invalid rtc_time: %ptR\n", tm);
		return err;
	}

	dev_dbg(dev, "%s(): %ptR\n", __func__, tm);
	return err;
}

static int sunxi_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	u32 hw_day, hw_hhmmss;
	int real_year;
	int err;

	dev_dbg(dev, "%s(): %ptR\n", __func__, tm);

	err = rtc_valid_tm(tm);
	if (err) {
		dev_err(dev, "sunxi_rtc_settime(): Invalid rtc_time: %ptR\n", tm);
		return err;
	}

	real_year = TM_YEAR_TO_REAL_YEAR(tm->tm_year);
	if (rtc_valid_tm(tm)
	     || real_year < chip->data->min_year
	     || real_year > chip->data->max_year) {
		dev_err(dev, "Invalid year %d. Should be in range %d ~ %d\n",
			real_year, chip->data->min_year, chip->data->max_year);
		return -EINVAL;
	}

	switch (chip->data->type) {
	case SUNXI_RTC_TYPE_A:
		err = tm_to_hw_day_hhmmss(chip->data, tm, &hw_day, &hw_hhmmss);
		if (err)
			return err;
		break;
	case SUNXI_RTC_TYPE_B:
		tm_to_hw_yymmdd_hhmmss(chip->data, tm, &hw_day, &hw_hhmmss);
		break;
	default:
		dev_err(dev, "Invalid chip->data->type = %d\n", chip->data->type);
		return -EINVAL;
	}

	/* Before writting RTC_HHMMSS_REG, we should check the RTC_HHMMSS_ACCESS bit */
	err = wait_bits_cleared(chip, LOSC_CTRL_REG, RTC_HHMMSS_ACCESS(chip->data->hhmmss_access_bit), 50);
	if (err) {
		dev_err(dev, "sunxi_rtc_settime(): wait_bits_cleared() timeout (1)\n");
		return err;
	}
	rtc_reg_write(chip, RTC_HHMMSS_REG, hw_hhmmss);
	err = wait_bits_cleared(chip, LOSC_CTRL_REG, RTC_HHMMSS_ACCESS(chip->data->hhmmss_access_bit), 50);
	if (err) {
		dev_err(dev, "sunxi_rtc_settime(): wait_bits_cleared() timeout (2)\n");
		return err;
	}
	/* Before writting RTC_DAY_REG, we should check the RTC_DAY_ACCESS bit */
	err = wait_bits_cleared(chip, LOSC_CTRL_REG, RTC_DAY_ACCESS(chip->data->day_access_bit), 50);
	if (err) {
		dev_err(dev, "sunxi_rtc_settime(): wait_bits_cleared() timeout (3)\n");
		return err;
	}
	rtc_reg_write(chip, RTC_DAY_REG, hw_day);
	err = wait_bits_cleared(chip, LOSC_CTRL_REG, RTC_DAY_ACCESS(chip->data->day_access_bit), 50);
	if (err) {
		dev_err(dev, "sunxi_rtc_settime(): wait_bits_cleared() timeout (4)\n");
		return err;
	}

	return 0;
}

extern bool alarm0_is_enabled(struct sunxi_rtc_dev *chip);
extern void alarm0_ctrl(struct sunxi_rtc_dev *chip, bool en);

/*
 * The RTC has two DAY registers: RTC_DAY_REG and ALARM0_DAY_REG.
 * RTC_DAY_REG starts from 1, but ALARM0_DAY_REG starts from 0.
 * If RTC power losts, ALARM0_DAY_REG will resets to it's default value 0.
 * When RTC is registered into the framework, sunxi_rtc_getalarm() will be called.
 * And if ALARM0_DAY_REG has a value of 0, hw_day_hhmmss_to_tm() will throw an error,
 * which causes the probe failure.
 * So we fix it here: Forcely modify the default value of ALARM0_DAY_REG from 0 to 1.
 */
static void errata__fix_alarm_day_reg_default_value(struct device *dev)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);

	if (rtc_reg_read(chip, ALARM0_DAY_REG) == 0) {
		dev_info(dev, "%s(): ALARM0_DAY_REG=0, set it to 1\n", __func__);
		rtc_reg_write(chip, ALARM0_DAY_REG, 1);
	}
}

static int sunxi_rtc_getalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;
	u32 hw_day, hw_hhmmss;
	int err;
	u32 target_second, current_second;
	time64_t target_time64;

	switch (chip->data->type) {
	case SUNXI_RTC_TYPE_A:
		hw_day = rtc_reg_read(chip, ALARM0_DAY_REG);
		hw_hhmmss = rtc_reg_read(chip, ALARM0_HHMMSS_REG);
		err = hw_day_hhmmss_to_tm(chip->data, hw_day, hw_hhmmss, tm);
		if (err)
			return err;
		break;
	case SUNXI_RTC_TYPE_B:
		target_second = rtc_reg_read(chip, ALARM0_COUNTER_REG);
		current_second = rtc_reg_read(chip, ALARM0_CUR_VAL_REG);
		if (current_second > target_second) {
			/* Alarm has expired, clear this alarm */
			alarm0_ctrl(chip, wkalrm->enabled);
			goto out;
		}
		err = sunxi_rtc_gettime(dev, tm);
		if (err) {
			dev_err(dev, "Error getting time in getalarm\n");
			return err;
		}
		target_time64 = rtc_tm_to_time64(tm) + target_second - current_second;
		rtc_time64_to_tm(target_time64, tm);
		break;
	default:
		dev_err(dev, "Invalid chip->data->type = %d\n", chip->data->type);
		return -EINVAL;
	}

	err = rtc_valid_tm(tm);
	if (err)
		dev_err(dev, "sunxi_rtc_getalarm(): Invalid rtc_time: %ptR\n", tm);

out:
	wkalrm->enabled = alarm0_is_enabled(chip);

	dev_dbg(dev, "%s(): %ptR\n", __func__, tm);
	return 0;
}

static int sunxi_rtc_setalarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;
	u32 hw_day, hw_hhmmss;
	int err;
	struct rtc_time tm_now;
	time64_t sec_diff;

	dev_dbg(dev, "%s(): %ptR\n", __func__, tm);

	err = rtc_valid_tm(tm);
	if (err) {
		dev_err(dev, "sunxi_rtc_setalarm(): Invalid rtc_time: %ptR\n", tm);
		return err;
	}

	/* @TODO: check that tm should be later than current time? */
	switch (chip->data->type) {
	case SUNXI_RTC_TYPE_A:
		err = tm_to_hw_day_hhmmss(chip->data, tm, &hw_day, &hw_hhmmss);
		if (err)
			return err;
		rtc_reg_write(chip, ALARM0_DAY_REG, hw_day);
		rtc_reg_write(chip, ALARM0_HHMMSS_REG, hw_hhmmss);
		break;
	case SUNXI_RTC_TYPE_B:
		err = sunxi_rtc_gettime(dev, &tm_now);
		if (err) {
			dev_err(dev, "sunxi_rtc_setalarm(): Failed to get time\n");
			return err;
		}
		sec_diff = rtc_tm_sub(tm, &tm_now);
		if (sec_diff <= 0) {
			dev_err(dev, "sunxi_rtc_setalarm(): Cannot set alarm in the past\n");
			return -EINVAL;
		} else if (sec_diff > TYPE_B_HW_SECONDS_MAX) {
			dev_err(dev, "sunxi_rtc_setalarm(): Time must be in the range\n");
			return -EINVAL;
		}
		rtc_reg_write(chip, ALARM0_COUNTER_REG, sec_diff);
		break;
	default:
		dev_err(dev, "Invalid chip->data->type = %d\n", chip->data->type);
		return -EINVAL;
	}

	alarm0_ctrl(chip, wkalrm->enabled);

	return 0;
}

/* Alarm0 Interrupt Sevice Routine */
static irqreturn_t sunxi_rtc_alarm0_isr(int irq, void *id)
{
	struct sunxi_rtc_dev *chip = (struct sunxi_rtc_dev *)id;
	u32 val;

	val = rtc_reg_read(chip, ALARM0_IRQ_STA_REG);
	if (val & ALARM0_IRQ_PEND) {
		val |= ALARM0_IRQ_PEND;
		rtc_reg_write(chip, ALARM0_IRQ_STA_REG, val);  /* clear the interrupt */
		rtc_update_irq(chip->rtc, 1, RTC_AF | RTC_IRQF);  /* Report Alarm interrupt */
		dev_dbg(chip->dev, "%s(): IRQ_HANDLED\n", __func__);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

bool alarm0_is_enabled(struct sunxi_rtc_dev *chip)
{
	u32 val1, val2;

	val1 = rtc_reg_read(chip, ALARM0_ENABLE_REG);
	val2 = rtc_reg_read(chip, ALARM0_IRQ_EN_REG);

	return (val1 & ALARM0_ENABLE) && (val2 & ALARM0_IRQ_EN);
}

void alarm0_ctrl(struct sunxi_rtc_dev *chip, bool en)
{
	u32 val;

	if (en) {
		/* enable alarm0 */
		val = rtc_reg_read(chip, ALARM0_ENABLE_REG);
		val |= ALARM0_ENABLE;
		rtc_reg_write(chip, ALARM0_ENABLE_REG, val);
		/* enable the interrupt */
		val = rtc_reg_read(chip, ALARM0_IRQ_EN_REG);
		val |= ALARM0_IRQ_EN;
		rtc_reg_write(chip, ALARM0_IRQ_EN_REG, val);
		dev_dbg(chip->dev, "%s(): enabled\n", __func__);
	} else {
		/* disable alarm0 */
		val = rtc_reg_read(chip, ALARM0_ENABLE_REG);
		val &= ~ALARM0_ENABLE;
		rtc_reg_write(chip, ALARM0_ENABLE_REG, val);
		/* disable the interrupt */
		val = rtc_reg_read(chip, ALARM0_IRQ_EN_REG);
		val &= ~ALARM0_IRQ_EN;
		rtc_reg_write(chip, ALARM0_IRQ_EN_REG, val);
		/* clear the interrupt */
		val = rtc_reg_read(chip, ALARM0_IRQ_STA_REG);
		val |= ALARM0_IRQ_PEND;
		rtc_reg_write(chip, ALARM0_IRQ_STA_REG, val);
		dev_dbg(chip->dev, "%s(): disabled\n", __func__);
	}
}

static __maybe_unused void alarm0_reset(struct sunxi_rtc_dev *chip)
{
	struct rtc_wkalrm wkalrm;

	wkalrm.time.tm_year = HW_YEAR_TO_TM_YEAR(0, chip->data->min_year);
	wkalrm.time.tm_mon = 0;
	wkalrm.time.tm_mday = 1;
	wkalrm.time.tm_hour = 0;
	wkalrm.time.tm_min = 0;
	wkalrm.time.tm_sec = 0;
	wkalrm.enabled = false;

	sunxi_rtc_setalarm(chip->dev, &wkalrm);
}

static void alarm0_output_ctrl(struct sunxi_rtc_dev *chip, bool en)
{
	u32 val;

	/*
	 * Alarm's output control:
	 * When the alarm expires in poweroff state (while SoC pin RESET# is asserted),
	 * the SoC pin 'AP-NMI#' will ouput LOW to PMIC, so that the system is powered up.
	 */

	if (en) {
		/*
		 * Release 'AP-NMI#' (LOW -> HIGH) by clearing alarm IRQ pending status.
		 * Some newer SoCs (such as A100) don't need this -- 'AP-NMI#' will be
		 * released automatically after SoC pin RESET# is de-asserted.
		 */
		/* We don't have to do this here... sunxi_rtc_alarm0_isr() will do
		val = rtc_reg_read(chip, ALARM0_IRQ_STA_REG);
		val |= ALARM0_IRQ_PEND;
		rtc_reg_write(chip, ALARM0_IRQ_STA_REG, val);
		*/

		/* Enable alarm output */
		val = rtc_reg_read(chip, ALARM0_CONFIG_REG);
		val |= ALARM0_OUTPUT_EN;
		rtc_reg_write(chip, ALARM0_CONFIG_REG, val);

		dev_dbg(chip->dev, "%s(): enabled\n", __func__);
	} else {
		/* Disable alarm output */
		val = rtc_reg_read(chip, ALARM0_CONFIG_REG);
		val &= ~ALARM0_OUTPUT_EN;
		rtc_reg_write(chip, ALARM0_CONFIG_REG, val);

		dev_dbg(chip->dev, "%s(): disabled\n", __func__);
	}
}

static int sunxi_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);

	alarm0_ctrl(chip, enabled);

	return 0;
}

static const struct rtc_class_ops sunxi_rtc_ops = {
	.read_time	  = sunxi_rtc_gettime,
	.set_time	  = sunxi_rtc_settime,
	.read_alarm	  = sunxi_rtc_getalarm,
	.set_alarm	  = sunxi_rtc_setalarm,
	.alarm_irq_enable = sunxi_rtc_alarm_irq_enable
};

static struct sunxi_rtc_data sunxi_rtc_v200_data = {
	.type = SUNXI_RTC_TYPE_A,
	.min_year   = 1970,
	.max_year   = TYPE_A_REAL_YEAR_MAX(1970),
	.gpr_offset = 0x100,
	.gpr_len    = 8,
	.has_dcxo_ictrl = false,
	.gpr_base_address = GPR_BASEADDR_SAMEWITH_RTC,
	.day_access_bit = 7,
	.hhmmss_access_bit = 8,
};

static struct sunxi_rtc_data sunxi_rtc_v201_data = {
	.type = SUNXI_RTC_TYPE_A,
	.min_year   = 1970,
	.max_year   = TYPE_A_REAL_YEAR_MAX(1970),
	.gpr_offset = 0x100,
	.gpr_len    = 8,
	.has_dcxo_ictrl = true,
	.dcxo_ictrl_val = 0xf,
	.gpr_base_address = GPR_BASEADDR_SAMEWITH_RTC,
	.day_access_bit = 7,
	.hhmmss_access_bit = 8,
};

static struct sunxi_rtc_data sunxi_rtc_v202_data = {
	.type = SUNXI_RTC_TYPE_B,
	.min_year   = 1970,
	.max_year   = TYPE_B_REAL_YEAR_MAX(1970),
	.gpr_offset = 0x100,
	.gpr_len    = 8,
	.has_dcxo_ictrl = false,
	.dcxo_ictrl_val = 0xf,
	.year_mask = 0x7F,
	.year_shift = 16,
	.leap_shift = 23,
	.gpr_base_address = GPR_BASEADDR_SAMEWITH_RTC,
	.day_access_bit = 7,
	.hhmmss_access_bit  = 8,
};

static struct sunxi_rtc_data sunxi_rtc_v203_data = {
	.type = SUNXI_RTC_TYPE_B,
	.min_year   = 1970,
	.max_year   = TYPE_B_REAL_YEAR_MAX(2098),
	.has_dcxo_ictrl = false,
	.year_mask = 0xFF,
	.year_shift = 16,
	.leap_shift = 24,
	.gpr_offset = 0x0,
	.gpr_len    = 8,
	.gpr_base_address = 0x4a000200,
	.day_access_bit = 0,
	.hhmmss_access_bit = 1,
};

static const struct of_device_id sunxi_rtc_dt_ids[] = {
	{.compatible = "allwinner,rtc-v200",        .data = &sunxi_rtc_v200_data},
	{.compatible = "allwinner,rtc-v201",        .data = &sunxi_rtc_v201_data},
	{.compatible = "allwinner,rtc-v202",        .data = &sunxi_rtc_v202_data},
	{.compatible = "allwinner,rtc-v203",        .data = &sunxi_rtc_v203_data},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sunxi_rtc_dt_ids);

/* These sysfs files are only for RTC test */
static ssize_t min_year_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct sunxi_rtc_dev *chip = platform_get_drvdata(pdev);
	return scnprintf(buf, PAGE_SIZE, "%u \n", chip->data->min_year);
}
static ssize_t max_year_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct sunxi_rtc_dev *chip = platform_get_drvdata(pdev);
	return scnprintf(buf, PAGE_SIZE, "%u \n", chip->data->max_year);
}

static ssize_t dcxo_ictrl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct sunxi_rtc_dev *chip = platform_get_drvdata(pdev);

	val = rtc_reg_read(chip, XO_CTRL_REG);
	val = (val & XO_CTRL_MASK) >> XO_ICTRL_SHIFT;

	return sprintf(buf, "0x%x\n", val);

}

static ssize_t dcxo_ictrl_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u32 val, temp, ret;
	char str[5]; /* the value rage is 0x0~0xf, so the len is set to 5 */
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct sunxi_rtc_dev *chip = platform_get_drvdata(pdev);

	if (size >= ARRAY_SIZE(str)) {
		dev_err(dev, "parameter is too long\n");
		return -EINVAL;
	}

	ret = kstrtou32(buf, 16, &val);
	if (ret) {
		dev_err(dev, "invalid para!\n");
		return -EINVAL;
	}

	temp = rtc_reg_read(chip, XO_CTRL_REG);
	temp = (temp & (~XO_CTRL_MASK)) | (val << XO_ICTRL_SHIFT);
	rtc_reg_write(chip, XO_CTRL_REG, temp);

	return size;

}

static struct device_attribute min_year_attr =
	__ATTR(min_year, S_IRUGO, min_year_show, NULL);
static struct device_attribute max_year_attr =
	__ATTR(max_year, S_IRUGO, max_year_show, NULL);
static struct device_attribute dcxo_ictrl_attr =
	__ATTR(dcxo_ictrl, 0664, dcxo_ictrl_show, dcxo_ictrl_store);

static int rtc_registers_access_check(struct sunxi_rtc_dev *chip)
{
	struct device *dev = chip->dev;
	void __iomem *addr = chip->base + chip->data->gpr_offset + 0 * 0x04;  /* The first General Purpose Register */
	u32 val_backup;
	u32 val_w = 0x12345678;
	u32 val_r;

	val_backup = readl(addr);
	writel(val_w, addr);
	val_r = readl(addr);
	if (val_r != val_w) {
		dev_err(dev, "%s(): FAILED: Expect 0x%x but got 0x%x\n", __func__, val_w, val_r);
		writel(val_backup, addr);
		return -EINVAL;
	}
	writel(val_backup, addr);
	return 0;
}

/* Inform user about the alarm-irq with sysfs node 'alarm_in_booting' */
static int alarm_in_booting;
module_param(alarm_in_booting, int, S_IRUGO | S_IWUSR);
static int sunxi_rtc_poweroff_alarm(struct sunxi_rtc_dev *chip)
{
#if IS_ENABLED(CONFIG_AW_RTC_POWEROFF_ALARM)
	int err;
	bool alarm0_en;
	bool alarm0_irq_pending;
	bool alarm0_expired;
	struct rtc_wkalrm wkalrm;
	struct rtc_time now;

	alarm0_en = alarm0_is_enabled(chip);
	alarm0_irq_pending = rtc_reg_read(chip, ALARM0_IRQ_STA_REG) & ALARM0_IRQ_PEND;

	/* @WARNING: Driver callbacks are used before RTC is registered! Take care of this... */
	err = sunxi_rtc_getalarm(chip->dev, &wkalrm);
	if (err) {
		dev_err(chip->dev, "%s(): get alarm failed\n", __func__);
		return err;
	}
	err = sunxi_rtc_gettime(chip->dev, &now);
	if (err) {
		dev_err(chip->dev, "%s(): get time failed\n", __func__);
		return err;
	}
	alarm0_expired = (rtc_tm_sub(&now, &(wkalrm.time)) >= 0);

	if (alarm0_en && alarm0_irq_pending && alarm0_expired)
		alarm_in_booting = 1;

	dev_dbg(chip->dev, "%s(): alarm0_en=%d, alarm0_irq_pending=%d, alarm0_expired=%d, alarm=%ptR, now=%ptR\n",
		 __func__, alarm0_en, alarm0_irq_pending, alarm0_expired, &(wkalrm.time), &now);

	alarm0_output_ctrl(chip, true);

	return 0;
#else
	/* This will clear the alarm that was set before. We should not do this? */
	/* alarm0_reset(chip); */

	alarm0_output_ctrl(chip, false);

	return 0;
#endif
}

#ifdef CONFIG_AW_FAKE_POWEROFF

static void __iomem *fake_poweroff_rtc_addr;

static void poweroff_deal(void *data)
{
	if (fake_poweroff_rtc_addr) {
		writel(NORMAL_POWEROFF_RTC_FLAG, fake_poweroff_rtc_addr);
	}
}

#endif /* #ifdef CONFIG_AW_FAKE_POWEROFF */

#if IS_ENABLED(CONFIG_AW_RTC_REBOOT_FLAG)

static void __iomem *gpr_cur_addr;  /* Currently Used General Purpose Register's Address */
static void __iomem *boot_reasonbase;

struct str_num_pair {
	const char *str;
	u32 num;
};

enum {
	SUNXI_RTC_BOOT_REASON_NONE = 0,
	SUNXI_RTC_BOOT_REASON_COLD_BOOT,
	SUNXI_RTC_BOOT_REASON_HOT_REBOOT,
	SUNXI_RTC_BOOT_REASON_PANIC_REBOOT,
	SUNXI_RTC_BOOT_REASON_MAX,
};

#define REBOOT_PARAM_LEN_MAX    32  /* Max length of user input from sysfs (including '\0') */
static const struct str_num_pair str_flag_table[] = {
	{ "debug",		0x59 },
	{ "efex",		0x5A },
	{ "boot-resignature",	0x5B },
	{ "recovery",		0x5C },
	{ "boot-recovery",	0x5C },
	{ "sysrecovery",	0x5D },
	{ "usb-recovery",	0x5E },
	{ "bootloader",		0x5F },
	{ "uboot",		0x60 },
};

static unsigned int str2flag(const char *str)
{
	unsigned int flag = 0;  /* default value is 0 */
	int i;

	for (i = 0; i < ARRAY_SIZE(str_flag_table); i++) {
		if (!strcmp(str, str_flag_table[i].str)) {
			flag = str_flag_table[i].num;
			break;
		}
	}

	return flag;
}

static void reboot_deal(void *data)
{
	unsigned int flag;
	const char *str = (const char *)data;

	if (!data) {
		pr_info("%s(): empty arg\n", __func__);
		return;
	}

	flag = str2flag(str);
	if (flag == 0) {
		pr_info("%s(): unkown arg '%s'\n", __func__, str);
		return;
	}

	pr_info("%s(): store flag '%s' (0x%x) in RTC General-Purpose-Register\n", __func__, str, flag);
	writel(flag, gpr_cur_addr);
	return;
}

static int sunxi_rtc_bootreason_set(u32 value)
{
	if (((value & 0xffff) >= SUNXI_RTC_BOOT_REASON_MAX) ||
		IS_ERR_OR_NULL(boot_reasonbase)) {
		pr_err("invalid reason or reasonbase NULL\n");
		return -1;
	}

	writel(value, boot_reasonbase);

	if (value != readl(boot_reasonbase))
		pr_err("set bootreason flag fail\n");

	return 0;
}

static int sunxi_rtc_bootreason_get(void)
{
	u32 value;

	if (IS_ERR_OR_NULL(boot_reasonbase)) {
		pr_err("reasonbase NULL\n");
		return -1;
	}

	value = readl(boot_reasonbase);

	return (value & 0xffff);
}

static int sunxi_rtc_bootreason_init(void)
{
	unsigned int bootreason = 0;
	bootreason = sunxi_rtc_bootreason_get();

	if ((bootreason & 0xffff) == SUNXI_RTC_BOOT_REASON_NONE) {
		/* none --> cold */
		pr_info(" Saving SoC boot-reason: COLD-BOOT\n");
		sunxi_rtc_bootreason_set(SUNXI_RTC_BOOT_REASON_COLD_BOOT);
	} else if ((bootreason & 0xffff) == SUNXI_RTC_BOOT_REASON_COLD_BOOT) {
		/* cold --> hot */
		pr_info(" Saving SoC boot-reason: HOT-BOOT\n");
		sunxi_rtc_bootreason_set(SUNXI_RTC_BOOT_REASON_HOT_REBOOT);
	} else if ((bootreason & 0xffff) < SUNXI_RTC_BOOT_REASON_MAX) {
		/* others, remain */
		pr_info(" Saving SoC boot-reason: OTHERS\n");
		sunxi_rtc_bootreason_set(bootreason);
	} else {
		/* error --> hot */
		pr_info("reason large than max, fix to hot reboot, save boot reason\n");
		sunxi_rtc_bootreason_set(SUNXI_RTC_BOOT_REASON_HOT_REBOOT);
	}

	return 0;
}

static int reboot_callback(struct notifier_block *this,
	unsigned long code, void *data)
{
	u32 ret;
	ret = sunxi_rtc_bootreason_set(SUNXI_RTC_BOOT_REASON_HOT_REBOOT);
	if (ret)
		pr_err("rtc set bootreason hot reboot fail\n");
	reboot_deal(data);
	#ifdef CONFIG_AW_FAKE_POWEROFF
	switch (code) {
	case SYS_RESTART:
		/* enter the restart workflow, include fake poweroff function */
		if (fake_poweroff_rtc_addr)
			writel(FAKE_POWEROFF_RTC_FLAG, fake_poweroff_rtc_addr);
		break;
	case SYS_HALT:
		break;
	case SYS_POWER_OFF:
		/* enter the normal poweroff workflow */
		poweroff_deal(data);
		break;
	default:
		break;
	}
	#endif

	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_callback,
};

static int panic_callback(struct notifier_block *nb,
				      unsigned long code, void *unused)
{
	u32 ret;

	ret = sunxi_rtc_bootreason_set(SUNXI_RTC_BOOT_REASON_PANIC_REBOOT);
	if (ret)
		pr_err("rtc set bootreason panic reboot fail\n");

	return NOTIFY_DONE;
}

static struct notifier_block panic_notifier = {
	.notifier_call = panic_callback,
};

static ssize_t flag_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int flag;
	flag = readl(gpr_cur_addr);
	return sprintf(buf, "0x%x\n", flag);
}

static ssize_t flag_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int flag;
	char str[REBOOT_PARAM_LEN_MAX];

	/* 'buf' contains an extra '\n', we need to remove it here */
	if (size >= ARRAY_SIZE(str)) {
		dev_err(dev, "parameter is too long\n");
		return size;
	}
	strncpy(str, buf, size - 1);
	str[size - 1] = '\0';

	flag = str2flag(str);
	if (flag == 0) {
		dev_err(dev, "unkown arg '%s'\n", str);
		return size;
	}
	dev_dbg(dev, "store flag '%s' (0x%x) in RTC General-Purpose-Register\n", str, flag);
	writel(flag, gpr_cur_addr);

	return size;
}

static struct device_attribute flag_attr =
	__ATTR(flag, 0664, flag_show, flag_store);

static ssize_t bootreason_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int rtc_bootreason = 0;

	rtc_bootreason = sunxi_rtc_bootreason_get();

	return sprintf(buf, "0x%x\n", rtc_bootreason);
}

static ssize_t bootreason_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int rtc_bootreason = 0;
	int ret;
	char rtc_str[30] = {0};

	if (size >= 30) {
		pr_err("parameters too long\n");
		return size;
	}

	snprintf(rtc_str, size, "%s", buf);
	ret = kstrtoul(rtc_str, 0, (unsigned long *)&rtc_bootreason);
	if (ret >= SUNXI_RTC_BOOT_REASON_MAX)
		pr_warn("unknown reason %s", buf);
	else
		pr_info("store rtc reason 0x%x\n", rtc_bootreason);

	/* write the data to reg */
	sunxi_rtc_bootreason_set(rtc_bootreason);

	return size;
}

static struct device_attribute bootreason_attr =
	__ATTR(bootreason, 0664, bootreason_show, bootreason_store);

static int sunxi_rtc_reboot_flag_setup(struct sunxi_rtc_dev *chip)
{
	struct device *dev = chip->dev;
	int err;
	u32 value;
	const char *name = "gpr_cur_pos";  /* A dts property indicating where to store the reboot flag */
	u32 gpr_bootcount = 0;

	#ifdef CONFIG_AW_FAKE_POWEROFF
	u32 gpr_fake_poweroff = 0;
	#endif

	err = of_property_read_u32(dev->of_node, name, &value);
	if (err) {
		dev_err(dev, "Fail to read dts property '%s'\n", name);
		return err;
	}
	dev_dbg(dev, "Read dts property '%s' = 0x%x\n", name, value);
	if (value >= chip->data->gpr_len) {
		dev_err(dev, "dts property '%s' is out of range!\n", name);
		return -EINVAL;
	}
	gpr_cur_addr = chip->gpr_base + chip->data->gpr_offset + value * 0x4;

	err = of_property_read_u32(dev->of_node,
					"gpr_bootcount_pos", &gpr_bootcount);
	if (err) {
		dev_err(dev, "Fail to read dts property 'gpr_bootcount_pos'\n");
		boot_reasonbase = NULL;
	} else {
		boot_reasonbase = chip->gpr_base + chip->data->gpr_offset + gpr_bootcount * 0x4;
	}

#ifdef CONFIG_AW_FAKE_POWEROFF
	err = of_property_read_u32(dev->of_node,
					"gpr_fake_poweroff_pos", &gpr_fake_poweroff);
	if (err) {
		dev_err(dev, "Fail to read dts property 'gpr_fake_poweroff_pos'\n");
		fake_poweroff_rtc_addr = NULL;
	} else {
		fake_poweroff_rtc_addr = chip->gpr_base + chip->data->gpr_offset + gpr_fake_poweroff * 0x4;
	}
#endif

	/* When rebooting, reboot_notifier.notifier_call will be called.
	 * If you pass a parameter to 'reboot' like this: $ reboot param
	 * - For Android: reboot_notifier.notifier_call(..., data=param).
	 * - For busybox: reboot_notifier.notifier_call(..., data=NULL). i.e. param will be lost.
	 */
	err = register_reboot_notifier(&reboot_notifier);
	if (err) {
		dev_err(dev, "register reboot notifier error %d\n", err);
		return err;
	}

	err = atomic_notifier_chain_register(&panic_notifier_list, &panic_notifier);
	if (err) {
		dev_err(dev, "register panic notifier error %d\n", err);
		goto fail;
	}

	/* Export sunxi-rtc-flag to sysfs:
	 * Android's reboot can pass parameters to kernel but busybox's reboot can not.
	 * Here we export rtc-flag to sysfs, so the following command can be used as
	 * an alternative of "reboot efex":
	 * $ echo efex > /sys/devices/platform/soc@2900000/7000000.rtc/flag; reboot
	 */
	err = device_create_file(dev, &flag_attr);
	if (err) {
		dev_err(dev, "device_create_file() failed\n");
		/* unregister_reboot_notifier(&reboot_notifier); */
		/* We don't need to rollback here:
		 * 'reboot_notifier' and 'flag_attr' does not rely on each other.
		 * Even one of them cannot work, the other one could still be useful.
		 */
		return err;
	}

	err = device_create_file(dev, &bootreason_attr);
	if (err) {
		dev_err(dev, "device_create_file() failed\n");
		return err;
	}

	sunxi_rtc_bootreason_init();

	return 0;

fail:
	unregister_reboot_notifier(&reboot_notifier);

	return err;
}

static void sunxi_rtc_reboot_flag_destroy(struct sunxi_rtc_dev *chip)
{
	struct device *dev = chip->dev;

	device_remove_file(dev, &bootreason_attr);
	device_remove_file(dev, &flag_attr);
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &panic_notifier);
	unregister_reboot_notifier(&reboot_notifier);

	gpr_cur_addr = NULL;
	boot_reasonbase = NULL;

#ifdef CONFIG_AW_FAKE_POWEROFF
	fake_poweroff_rtc_addr = NULL;
#endif
}

#else  /* CONFIG_AW_RTC_REBOOT_FLAG */
#define sunxi_rtc_reboot_flag_setup(chip)       (0)
#define sunxi_rtc_reboot_flag_destroy(chip)
#endif  /* CONFIG_AW_RTC_REBOOT_FLAG */

static int sunxi_rtc_probe(struct platform_device *pdev)
{
	struct sunxi_rtc_dev *chip;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id;
	struct resource *res;
	int err;
	u32 val;

	dev_dbg(dev, "%s(): BEGIN\n", __func__);

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	of_id = of_match_device(sunxi_rtc_dt_ids, dev);
	if (!of_id) {
		dev_err(dev, "of_match_device() failed\n");
		return -EINVAL;
	}
	chip->data = (struct sunxi_rtc_data *)(of_id->data);

	platform_set_drvdata(pdev, chip);
	chip->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Fail to get IORESOURCE_MEM\n");
		return -EINVAL;
	}
	chip->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(chip->base)) {
		dev_err(dev, "Fail to map IO resource\n");
		return PTR_ERR(chip->base);
	}

	/* FIXME: Move the reset and clk logic to a separated function */
	/* Release the reset */
	chip->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(chip->reset)) {
		dev_err(dev, "reset_control_get() failed\n");
		/* Some SoCs have no resets... Let's continue */
		/*
		err = PTR_ERR(chip->reset);
		goto err1;
		*/
	} else {
		err = reset_control_deassert(chip->reset);
		if (err) {
			dev_err(dev, "reset_control_deassert() failed\n");
			goto err1;
		}
	}

	/* Enable BUS clock */
	chip->clk_bus = devm_clk_get(dev, "r-ahb-rtc");
	if (IS_ERR(chip->clk_bus)) {
		dev_err(dev, "Fail to get clock 'r-ahb-rtc'\n");
		err = PTR_ERR(chip->clk_bus);
		goto err2;
	}
	err = clk_prepare_enable(chip->clk_bus);
	if (err) {
		dev_err(dev, "Cannot enable clock 'r-ahb-rtc'\n");
		goto err2;
	}

	/* Enable RTC clock */
	chip->clk = devm_clk_get(dev, "rtc-1k");
	if (IS_ERR(chip->clk)) {
		dev_warn(dev, "Fail to get clock 'rtc-1k'\n");
		/* Some SoCs have no such clock, like sun8iw11. Let's continue */
	} else {
		err = clk_prepare_enable(chip->clk);
		if (err) {
			dev_err(dev, "Cannot enable clock 'rtc-1k'\n");
			goto err3;
		}
	}

	/* Enable RTC SPI clock */
	chip->clk_spi = devm_clk_get(dev, "rtc-spi");
	if (IS_ERR(chip->clk_spi))
		dev_warn(dev, "Fail to get clock 'rtc-spi'\n");
		/* Some SoCs have no such clock. Let's continue */
	else {
		err = clk_prepare_enable(chip->clk_spi);
		if (err) {
			dev_err(dev, "Cannot enable clock 'rtc-spi'\n");
			goto err4;
		}
	}

	/* Check whether the gpr base address is consistent with rtc base address,
	 * if it is(gpr_base_address == 0), chip->gpr_base = chip->base,
	 * and if not, ioremap gpr_base_address to chip->gpr_base.
	 */
	if (chip->data->gpr_base_address) {
		chip->gpr_base = ioremap(chip->data->gpr_base_address, 0x20);
	} else {
		chip->gpr_base = chip->base;
	}

	/* Now that the clks and resets are ready. We should be able to read/write registers.
	 * Let's ensure this before any access to the registers...Because we always have troubles here.
	 */
	err = rtc_registers_access_check(chip);
	if (err) {
		goto err5;
	}

	err = sunxi_rtc_reboot_flag_setup(chip);
	if (err) {
		dev_err(dev, "sunxi_rtc_reboot_flag_setup() failed\n");
		goto err5;
	}

	chip->gpr_only = of_property_read_bool(np, "gpr-only");
	if (chip->gpr_only)
		goto out;

	/* We must ensure the error value of ALARM0_DAY_REG is fixed before rtc_register_device().
	 * Because ALARM0_DAY_REG will be read during registration.
	 */
	errata__fix_alarm_day_reg_default_value(chip->dev);

	/*
	 * sunxi_rtc_poweroff_alarm() must be placed before IRQ setting up.
	 * Since we'll read the IRQ pending status in sunxi_rtc_poweroff_alarm(),
	 * we don't want the IRQ to be handled and cleared before this.
	 */
	sunxi_rtc_poweroff_alarm(chip);

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0) {
		err = chip->irq;
		goto err6;
	}
	err = devm_request_irq(dev, chip->irq, sunxi_rtc_alarm0_isr, 0,
				dev_name(dev), chip);
	if (err) {
		dev_err(dev, "Could not request IRQ\n");
		goto err6;
	}

	if (of_property_read_bool(np, "wakeup-source")) {
		device_init_wakeup(dev, true);
		dev_pm_set_wake_irq(dev, chip->irq);
	}

	/* Register RTC device */
	chip->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(chip->rtc))  {
		dev_err(dev, "Unable to allocate RTC device\n");
		err = PTR_ERR(chip->rtc);
		goto err6;
	}
	chip->rtc->ops = &sunxi_rtc_ops;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	err = devm_rtc_register_device(chip->rtc);
#else
	err = rtc_register_device(chip->rtc);
#endif
	if (err) {
		dev_err(dev, "Unable to register RTC device\n");
		goto err6;
	}

	/* Change XO_CTRL_REG bit[24:27] value to dcxo_ictrl_val for
	 * adjust current value to reduce power consumption
	 */
	if (chip->data->has_dcxo_ictrl) {
		val = rtc_reg_read(chip, XO_CTRL_REG);
		val |= (chip->data->dcxo_ictrl_val << XO_ICTRL_SHIFT);
		rtc_reg_write(chip, XO_CTRL_REG, val);
		device_create_file(dev, &dcxo_ictrl_attr);
	}

	device_create_file(dev, &min_year_attr);
	device_create_file(dev, &max_year_attr);

out:
	dev_info(dev, "sunxi rtc probed\n");
	return 0;

err6:
	sunxi_rtc_reboot_flag_destroy(chip);
err5:
	if (!IS_ERR(chip->clk_spi))
		clk_disable_unprepare(chip->clk_spi);
err4:
	clk_disable_unprepare(chip->clk);
err3:
	clk_disable_unprepare(chip->clk_bus);
err2:
	if (!IS_ERR(chip->reset))
		reset_control_assert(chip->reset);
err1:
	return err;
}

static int sunxi_rtc_remove(struct platform_device *pdev)
{
	struct sunxi_rtc_dev *chip = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (!chip->gpr_only) {
		device_remove_file(dev, &max_year_attr);
		device_remove_file(dev, &min_year_attr);
		if (chip->data->has_dcxo_ictrl)
			device_remove_file(dev, &dcxo_ictrl_attr);
	}
	sunxi_rtc_reboot_flag_destroy(chip);
	clk_disable_unprepare(chip->clk_spi);
	clk_disable_unprepare(chip->clk);
	clk_disable_unprepare(chip->clk_bus);
	reset_control_assert(chip->reset);

	return 0;
}

static struct platform_driver sunxi_rtc_driver = {
	.probe    = sunxi_rtc_probe,
	.remove   = sunxi_rtc_remove,
	.driver   = {
		.name  = "sunxi-rtc",
		.owner = THIS_MODULE,
		.of_match_table = sunxi_rtc_dt_ids,
	},
};

static int __init sunxi_rtc_init(void)
{
	int err;

	err = platform_driver_register(&sunxi_rtc_driver);
	if (err)
		pr_err("register sunxi rtc failed\n");

	return err;
}
module_init(sunxi_rtc_init);

static void __exit sunxi_rtc_exit(void)
{
	platform_driver_unregister(&sunxi_rtc_driver);
}
module_exit(sunxi_rtc_exit);

MODULE_DESCRIPTION("sunxi RTC driver");
MODULE_AUTHOR("Martin <wuyan@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.2.0");
