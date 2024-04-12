/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * The driver of SUNXI SecuritySystem controller.
 *
 * Copyright (C) 2013 Allwinner.
 *
 * Mintow <duanmintao@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/acpi.h>
#include <linux/crypto.h>
#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <crypto/internal/rng.h>

#include "sunxi_ce_cdev.h"

#define SUNXI_TRNG_DEV_NAME "sunxi_trng"
#define SUNXI_TRNG_MAX_LEN  32

struct sunxi_hwrng {
	struct hwrng rng;
	s8      name[8];
};

int ce_get_hardware_random(u8 *dst, u32 rng_len, u32 flag);

static int sunxi_trng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	int currsize = 0;
	u32 ret = -1;
	u8 tmp[SUNXI_TRNG_MAX_LEN];

	memset(tmp, 0x0, sizeof(tmp));

	do {
		ret = ce_get_hardware_random(tmp, SUNXI_TRNG_MAX_LEN, 1);
		if (ret) {
			SS_ERR("do_rng_crypto: %d!\n", ret);
			return ret;
		}

		if ((max - currsize) < SUNXI_TRNG_MAX_LEN) {
			memcpy((buf + currsize), tmp, (max - currsize));
			return max;
		} else {
			memcpy((buf + currsize), tmp, SUNXI_TRNG_MAX_LEN);
			currsize = currsize + SUNXI_TRNG_MAX_LEN;
		}

	} while (currsize < max);

	return currsize;
}

int sunxi_register_hwrng(sunxi_ce_cdev_t *ce_dev)
{
	int ret;

	ce_dev->hwrng = devm_kzalloc(ce_dev->pdevice, sizeof(struct sunxi_hwrng), GFP_KERNEL);
	if (!ce_dev->hwrng) {
		SS_ERR("failed to devm_kzalloc: %d!\n", ret);
		return -ENOMEM;
	}

	ce_dev->hwrng->rng.name = SUNXI_TRNG_DEV_NAME;
	ce_dev->hwrng->rng.read = sunxi_trng_read;
	ce_dev->hwrng->rng.quality = 512;
	ret = devm_hwrng_register(ce_dev->pdevice, &ce_dev->hwrng->rng);
	if (ret) {
		SS_ERR("failed to register hwrng: %d!\n", ret);
		return ret;
	}

	return 0;
}
