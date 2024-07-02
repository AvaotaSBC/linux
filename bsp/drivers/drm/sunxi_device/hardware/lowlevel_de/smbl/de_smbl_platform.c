/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2023 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "de_smbl_platform.h"

#define SMBL_OFST_V2X		0x1B0000
#define SMBL_OFST_V35X		0xA0000

struct de_version_smbl {
	unsigned int version;
	unsigned int smbl_cnt;
	struct de_smbl_dsc *smbls;
};

static struct de_smbl_dsc de201_smbls[] = {
	{
		.id = 0,
		.support_csc = true,
		.reg_offset = SMBL_OFST_V2X,
	},
};

static struct de_version_smbl de201 = {
	.version = 0x201,
	.smbl_cnt = ARRAY_SIZE(de201_smbls),
	.smbls = de201_smbls,
};

static struct de_version_smbl *de_version[] = {
	&de201,
};

const struct de_smbl_dsc *get_smbl_dsc(struct module_create_info *info)
{
	int i, j;
	for (i = 0; i < ARRAY_SIZE(de_version); i++) {
		if (de_version[i]->version == info->de_version) {
			for (j = 0; j < de_version[i]->smbl_cnt; j++) {
				if (de_version[i]->smbls[j].id == info->id)
					return &de_version[i]->smbls[j];
			}
		}
	}
	return NULL;
}

