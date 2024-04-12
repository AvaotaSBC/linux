/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright(c) 2019-2025 Allwinnertech Co., Ltd.
 *      http://www.allwinnertech.com
 *
 *
 * sunxi dump header file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _INCLUDE_LINUX_SUNXI_DUMP_H
#define _INCLUDE_LINUX_SUNXI_DUMP_H

int sunxi_dump_group_dump(void);
int sunxi_set_crashdump_mode(void);
#endif
