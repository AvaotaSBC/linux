/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/csi_cci/bsp_cci.h
 *
 * Copyright (c) 2007-2017 Allwinnertech Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * csi cci header
 * included by bsp_cci.c
 * Author:raymonxiu
 */

#ifndef __BSP_CCI__H__
#define __BSP_CCI__H__

#include "csi_cci_reg.h"
#define CCI_IRQ
struct cci_tx_mode {
	struct cci_tx_buf tx_buf_mode;
	struct cci_tx_trig tx_trig_mode;
	unsigned int tx_trig_line_cnt;
};

struct cci_msg {
	struct cci_bus_fmt bus_fmt;
	unsigned char *pkt_buf;
	unsigned char pkt_num;
};

int bsp_csi_cci_set_base_addr(unsigned int sel, unsigned long addr);
void bsp_csi_cci_init(unsigned int sel);
void bsp_csi_cci_exit(unsigned int sel);
void bsp_csi_cci_init_helper(unsigned int sel);
void bsp_cci_set_tx_mode(unsigned int sel, struct cci_tx_mode *tx_mode);
void bsp_cci_tx_single(unsigned int sel, struct cci_msg *msg);
void bsp_cci_tx_repeat(unsigned int sel, struct cci_msg *msg);
void bsp_cci_int_enable(unsigned int sel, enum cci_int_sel interrupt);
void bsp_cci_int_disable(unsigned int sel, enum cci_int_sel interrupt);
void bsp_cci_bus_error_process(unsigned int sel);
int bsp_cci_irq_process(unsigned int sel);

int bsp_cci_tx_start_wait_done(unsigned int sel, struct cci_msg *msg);

void CCI_INLINE_FUNC bsp_cci_int_get_status(unsigned int sel, struct cci_int_status *status);
void CCI_INLINE_FUNC bsp_cci_int_clear_status(unsigned int sel, enum cci_int_sel interrupt);
enum cci_bus_status CCI_INLINE_FUNC bsp_cci_get_bus_status(unsigned int sel);


int cci_wr_8_8(unsigned int sel, unsigned char reg, unsigned char data, unsigned char slv);
int cci_wr_16_8(unsigned int sel, unsigned short reg, unsigned char data, unsigned char slv);
int cci_wr_16_16(unsigned int sel, unsigned short reg, unsigned short data, unsigned char slv);
int cci_wr_8_16(unsigned int sel, unsigned char reg, unsigned short data, unsigned char slv);
int cci_wr_0_16(unsigned int sel, unsigned short data, unsigned char slv);
int cci_rd_8_8(unsigned int sel, unsigned char reg, unsigned char *data, unsigned char slv);
int cci_rd_16_8(unsigned int sel, unsigned short reg, unsigned char *data, unsigned char slv);
int cci_rd_16_16(unsigned int sel, unsigned short reg, unsigned short *data, unsigned char slv);
int cci_rd_8_16(unsigned int sel, unsigned char reg, unsigned short *data, unsigned char slv);
int cci_rd_0_16(unsigned int sel, unsigned short *data, unsigned char slv);
int cci_wr_a16_d8_continuous(unsigned int sel, unsigned short reg, unsigned char *data, unsigned char slv, int size);

#endif /* __BSP_CCI__H__ */
