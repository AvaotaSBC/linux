/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/**
 *	All Winner Tech, All Right Reserved. 2014-2015 Copyright (c)
 *
 *	File name   :       de_wb.h
 *
 *	Description :       display engine 2.0 wbc basic function declaration
 *
 *	History     :       2014/03/03   wangxuan   initial version
 *	                    2014/04/02   wangxuan   change the register
 *	                                            operation from bits to word
 */

#ifndef __DE_WB_H__
#define __DE_WB_H__

#include "de_rtmx.h"

enum wb_irq_flag {
	WB_IRQ_FLAG_INTR = 1 << 0,
	WB_IRQ_FLAG_MASK = WB_IRQ_FLAG_INTR,
};

enum wb_irq_state {
	WB_IRQ_STATE_PROC_END = 1 << 0,
	WB_IRQ_STATE_FINISH   = 1 << 4,
	WB_IRQ_STATE_OVERFLOW = 1 << 5,
	WB_IRQ_STATE_TIMEOUT  = 1 << 6,
	WB_IRQ_STATE_MASK =
		WB_IRQ_STATE_PROC_END
		| WB_IRQ_STATE_FINISH
		| WB_IRQ_STATE_OVERFLOW
		| WB_IRQ_STATE_TIMEOUT,
};



s32 wb_ebios_set_reg_base(u32 sel, uintptr_t base);
uintptr_t wb_ebios_get_reg_base(u32 sel);
s32 wb_ebios_init(struct disp_bsp_init_para *para);
s32 wb_ebios_writeback_enable(u32 sel, bool en);
s32 wb_ebios_set_para(u32 sel, struct disp_capture_config *cfg);
s32 wb_ebios_apply(u32 sel, struct disp_capture_config *cfg);
s32 wb_ebios_update_regs(u32 sel);
s32 wb_ebios_reset(u32 sel);
s32 wb_ebios_dereset(u32 sel);
u32 wb_ebios_get_status(u32 sel);
s32 wb_ebios_enableint(u32 sel);
s32 wb_ebios_disableint(u32 sel);
u32 wb_ebios_queryint(u32 sel);
u32 wb_ebios_clearint(u32 sel);
s32 wb_input_select(u32 sel);

#endif
