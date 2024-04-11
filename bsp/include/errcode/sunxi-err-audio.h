/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner's errcode for audio
 *
 * Copyright (c) 2023, zhouxijing <zhouxijing@allwinnertech.com>
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

#ifndef __SUNXI_ERR_AUDIO_H__
#define __SUNXI_ERR_AUDIO_H__

enum sunxi_err_audio_func {
	MOD_INIT = 0x0,
	PLAT_PROBE,
	MEM_INIT,
	CLK_INIT,
	PIN_INIT,
	COMP_PROBE,
	UCFMT_CB,
	HW_PARAMS,
	DAIFMT_SET,
	DAIFMT_GET,
	CLK_SET,
	DUMP_STORE,
	DUMP_SHOW,
	DUMP_VER,
	HW_IRQ,

	E_SWARG_END = 0xFFFF,
};

enum sunxi_err_audio {
	E_I2S_HW_IRQ	       = E_I2S_HW_ERR0	   | E_USER(HW_IRQ),
	E_I2S_SWDEP_MEM_INIT   = E_I2S_SW_DEP_ERR0 | E_USER(MEM_INIT),
	E_I2S_SWDEP_CLK_INIT   = E_I2S_SW_DEP_ERR0 | E_USER(CLK_INIT),
	E_I2S_SWDEP_PIN_INIT   = E_I2S_SW_DEP_ERR0 | E_USER(PIN_INIT),
	E_I2S_SWSYS_MOD_INIT   = E_I2S_SW_SYS_ERR0 | E_USER(MOD_INIT),
	E_I2S_SWSYS_PLAT_PROBE = E_I2S_SW_SYS_ERR0 | E_USER(PLAT_PROBE),
	E_I2S_SWSYS_COMP_PROBE = E_I2S_SW_SYS_ERR0 | E_USER(COMP_PROBE),
	E_I2S_SWARG_UCFMT_CB   = E_I2S_SW_ARG_ERR0 | E_USER(UCFMT_CB),
	E_I2S_SWARG_HW_PARAMS  = E_I2S_SW_ARG_ERR0 | E_USER(HW_PARAMS),
	E_I2S_SWARG_DAIFMT_SET = E_I2S_SW_ARG_ERR0 | E_USER(DAIFMT_SET),
	E_I2S_SWARG_DAIFMT_GET = E_I2S_SW_ARG_ERR0 | E_USER(DAIFMT_GET),
	E_I2S_SWARG_CLK_SET    = E_I2S_SW_ARG_ERR0 | E_USER(CLK_SET),
	E_I2S_SWARG_DUMP_STORE = E_I2S_SW_ARG_ERR0 | E_USER(DUMP_STORE),
	E_I2S_SWARG_DUMP_SHOW  = E_I2S_SW_ARG_ERR0 | E_USER(DUMP_SHOW),
	E_I2S_SWARG_DUMP_VER   = E_I2S_SW_ARG_ERR0 | E_USER(DUMP_VER),
};

#endif
