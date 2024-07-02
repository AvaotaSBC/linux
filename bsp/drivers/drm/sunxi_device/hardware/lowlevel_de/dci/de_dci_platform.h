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

#ifndef _DE_SNR_PLATFORM_H_
#define _DE_SNR_PLATFORM_H_

#include "de_base.h"

struct de_version_dci {
	unsigned int version;
	unsigned int dci_cnt;
	struct de_dci_desc **dci;
};

struct de_dci_desc {
	char name[32];
	unsigned int id;
	unsigned int reg_offset;
};

/* input info from channel */
const struct de_dci_desc *get_dci_desc(struct module_create_info *info);

#endif
