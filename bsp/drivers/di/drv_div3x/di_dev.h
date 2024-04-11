/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (c) 2007-2018 Allwinnertech Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _DI_DEV_H_
#define _DI_DEV_H_

#ifdef CONFIG_ARCH_SUN50IW9
#define USE_DI300
#endif

#ifdef CONFIG_ARCH_SUN55IW3
#define USE_DI300
#endif

#ifdef USE_DI300
#include "lowlevel_v3x/di300.h"
#endif

#endif /* #ifndef _DI_DEV_H_ */
