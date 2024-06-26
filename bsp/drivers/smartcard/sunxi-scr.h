/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (C) 2016 Allwinner.
 * fuzhaoke <fuzhaoke@allwinnertech.com>
 *
 * SUNXI SCR Register Definition
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __SUNXI_SCR_H__
#define __SUNXI_SCR_H__

#include "smartcard.h"
#include "sunxi-scr-user.h"

#define SCR_MODULE_NAME "smartcard"

/* smart card registers */
#define SCR_CSR_OFF                     (0x000)
#define SCR_INTEN_OFF                   (0x004)
#define SCR_INTST_OFF                   (0x008)
#define SCR_FCSR_OFF                    (0x00c)
#define SCR_FCNT_OFF                    (0x010)
#define SCR_RPT_OFF                     (0x014)
#define SCR_DIV_OFF                     (0x018)
#define SCR_LTIM_OFF                    (0x01c)
#define SCR_CTIM_OFF                    (0x020)
#define SCR_LCTL_OFF                    (0x030)
#define SCR_FSM_OFF                     (0x03c)
#define SCR_FIFO_OFF                    (0x100)

/* smart card interrupt status */
#define SCR_INTSTA_DEACT                (0x1<<23)
#define SCR_INTSTA_ACT                  (0x1<<22)
#define SCR_INTSTA_INS                  (0x1<<21)
#define SCR_INTSTA_REM                  (0x1<<20)
#define SCR_INTSTA_ATRDONE              (0x1<<19)
#define SCR_INTSTA_ATRFAIL              (0x1<<18)
#define SCR_INTSTA_CHTO                 (0x1<<17)  /*Character Timout */
#define SCR_INTSTA_CLOCK                (0x1<<16)
#define SCR_INTSTA_RXPERR               (0x1<<12)
#define SCR_INTSTA_RXDONE               (0x1<<11)
#define SCR_INTSTA_RXFTH                (0x1<<10)
#define SCR_INTSTA_RXFFULL              (0x1<<9)
#define SCR_INTSTA_TXPERR               (0x1<<4)
#define SCR_INTSTA_TXDONE               (0x1<<3)
#define SCR_INTSTA_TXFTH                (0x1<<2)
#define SCR_INTSTA_TXFEMPTY             (0x1<<1)
#define SCR_INTSTA_TXFDONE              (0x1<<0)


#define SCR_BUF_SIZE			256
#define SCR_FIFO_DEPTH                  8  /* just half of hw fifo size */
#define SCR_RX_TRANSMIT_NOYET		0
#define SCR_RX_TRANSMIT_TMOUT		1  /* time out */

enum scr_atr_status {
	SCR_ATR_RESP_INVALID,
	SCR_ATR_RESP_FAIL,
	SCR_ATR_RESP_AGAIN,
	SCR_ATR_RESP_OK,
};



struct sunxi_scr {
	void __iomem *reg_base;
	struct clk *scr_clk;
	struct clk *scr_clk_source;
	struct platform_device *scr_device;
	struct pinctrl *scr_pinctrl;
	struct resource *mem_res;
	uint32_t clk_freq;
	uint32_t irq_no;
	spinlock_t rx_lock;
	int32_t open_cnt;	/* support multi_process */
	bool suspended;
	/* smart card register parameters */
	uint32_t inten_bm;	/* interrupt enable bit map */
	uint32_t txfifo_thh;	/* txfifo threshold */
	uint32_t rxfifo_thh;	/* rxfifo threahold */
	uint32_t tx_repeat;	/* tx repeat */
	uint32_t rx_repeat;	/* rx repeat */
	uint32_t scclk_div;	/* scclk divisor */
	uint32_t baud_div;	/* baud divisor */
	uint8_t act_time;	/* active/deactive time, in scclk cycles */
	uint8_t rst_time;	/* reset time, in scclk cycles */
	uint8_t atr_time;	/* ATR limit time, in scclk cycles */
	uint32_t guard_time;	/* gaurd time, in ETUs */
	uint32_t chlimit_time;	/* character limit time, in ETUs */

	/* some necessary flags */
	volatile uint8_t atr_resp;
	volatile uint8_t rx_transmit_status;

	struct scr_card_para card_para;
	struct scr_atr scr_atr_des;
	struct smc_atr_para smc_atr_para;
	struct smc_pps_para smc_pps_para;

	wait_queue_head_t scr_poll;
	struct timer_list poll_timer;
	bool card_in;
	bool card_last;
};

#endif
