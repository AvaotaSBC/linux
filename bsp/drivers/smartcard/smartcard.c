/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (C) 2016 Allwinner.
 * fuzhaoke <fuzhaoke@allwinnertech.com>
 *
 * Decode for ISO7816 smart card
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include "smartcard.h"

void smartcard_ta1_decode(struct smc_atr_para *psmc_atr, uint8_t ta1)
{
	pr_debug("%s: enter!!\n", __func__);

	switch ((ta1>>4)&0xf) {
	case 0x0:
		psmc_atr->FMAX = 4;
		psmc_atr->F = 372;
		break;
	case 0x1:
		psmc_atr->FMAX = 5;
		psmc_atr->F = 372;
		break;
	case 0x2:
		psmc_atr->FMAX = 6;
		psmc_atr->F = 558;
		break;
	case 0x3:
		psmc_atr->FMAX = 8;
		psmc_atr->F = 744;
		break;
	case 0x4:
		psmc_atr->FMAX = 12;
		psmc_atr->F = 1116;
		break;
	case 0x5:
		psmc_atr->FMAX = 16;
		psmc_atr->F = 1488;
		break;
	case 0x6:
		psmc_atr->FMAX = 20;
		psmc_atr->F = 1860;
		break;
	case 0x9:
		psmc_atr->FMAX = 5;
		psmc_atr->F = 512;
		break;
	case 0xA:
		psmc_atr->FMAX = 7;
		psmc_atr->F = 768;
		break;
	case 0xB:
		psmc_atr->FMAX = 10;
		psmc_atr->F = 1024;
		break;
	case 0xC:
		psmc_atr->FMAX = 15;
		psmc_atr->F = 1536;
		break;
	case 0xD:
		psmc_atr->FMAX = 20;
		psmc_atr->F = 2048;
		break;
	default:  /* 0x7/0x8/0xE/0xF */
		psmc_atr->FMAX = 4;
		psmc_atr->F = 372;
		pr_err("Unsupport ta1 = 0x%x\n", ta1);
		break;
	}

	switch (ta1&0xf) {
	case 0x1:
		psmc_atr->D = 1;
		break;
	case 0x2:
		psmc_atr->D = 2;
		break;
	case 0x3:
		psmc_atr->D = 4;
		break;
	case 0x4:
		psmc_atr->D = 8;
		break;
	case 0x5:
		psmc_atr->D = 16;
		break;
	case 0x6:
		psmc_atr->D = 32;
		break;
	case 0x8:
		psmc_atr->D = 12;
		break;
	case 0x9:
		psmc_atr->D = 20;
		break;
	default: /* 0x0/0x7/0xA/0xB/0xC/0xD/0xE/0xF */
		psmc_atr->D = 1;
		pr_err("Unsupport ta1 = 0x%x\n", ta1);
		break;
	}
}

void smartcard_tb1_decode(struct smc_atr_para *psmc_atr, uint8_t tb1)
{
	pr_debug("%s: enter!!\n", __func__);

	switch ((tb1>>5)&0x3) {
	case 0:
		psmc_atr->I = 25;
		break;
	case 1:
		psmc_atr->I = 50;
		break;
	case 2:
		psmc_atr->I = 100;
		break;
	default:
		psmc_atr->I = 50;
	}

	if (((tb1&0x1f) > 4) && ((tb1&0x1f) < 26))
		psmc_atr->P = (tb1&0x1f); /* 5~25 in Volts */
	else if (0 == (tb1&0x1f))
		psmc_atr->P = 0;
	else
		psmc_atr->P = 5;
}

/* ATR data format: max 33 byte(include TS)
 * [TS|T0|TA1|TB1|TC1|TD1|TA2|TB2|TC2|TD2|...|T1|T2|........|TK|TCK]
 *       |-------- max 16 byte --------------|-- max 15 byte --|verify
 * |--------------------- max 33 byte -------------------------|
 * TS: Initial character
 *	0x3b:direct convention, 0x3f:inverse convention
 * T0: Format character
 *	[bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0]
 *	|------- Y1 --------|-------- K --------|
 *	Y1: indicator for the presence of the interface character
 *		TA1 is transmitted when bit[4] = 1
 *		TB1 is transmitted when bit[5] = 1
 *		TC1 is transmitted when bit[6] = 1
 *		TD1 is transmitted when bit[7] = 1
 *	K: history number
 * TAi: Interface character
 *	code FI,DI
 * TBi: Interface character
 *	code II,PI
 * TCi: Interface character
 *	code N
 * TDi: Interface character
 *	code Yi+1, T
 *	[bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0]
 *	|------ Yi + 1 -----|-------- T --------|
 *	Yi + 1: indicator for the presence of the interface character
 *		TAi+1 is transmitted when bit[4] = 1
 *		TBi+1 is transmitted when bit[5] = 1
 *		TCi+1 is transmitted when bit[6] = 1
 *		TDi+1 is transmitted when bit[7] = 1
 *	T: Protocol type for subsequent transmission
 *		T=0: character transmit
 *		T=1: block transmit
 *		T=other: reserved
 * T1~TK: history byte, information of the card like manufacture name
 * TCK: verify character, make sure the data is correctly
*/
void smartcard_atr_decode(struct smc_atr_para *psmc_atr, struct smc_pps_para *psmc_pps,
			  uint8_t *pdata, uint8_t with_ts)
{
	uint8_t index = 0;
	uint8_t temp;
	uint8_t i;

	pr_debug("%s: Enter...\n", __func__);

	psmc_pps->ppss = 0xff;
	psmc_pps->pps0 = 0;

	if (with_ts) {
		psmc_atr->TS = pdata[0]; /* TS */
		index++;
	}
	temp = pdata[index]; /* T0 */
	index++;
	psmc_atr->TK_NUM = temp & 0xf;

	/* TA1 */
	if (temp & 0x10) {
		smartcard_ta1_decode(psmc_atr, pdata[index]);
		psmc_pps->pps0 |= 0x1<<4;
		psmc_pps->pps1 = pdata[index];
		index++;
	}

	/* TB1 */
	if (temp & 0x20) {
		smartcard_tb1_decode(psmc_atr, pdata[index]);
		index++;
	}

	/* TC1 */
	if (temp & 0x40) {
		psmc_atr->N = pdata[index] & 0xff;
		index++;
	}

	/* TD1 */
	if (temp & 0x80) {
		pr_debug("%s: TD1 parse 0x%x !!\n", __func__, pdata[index]);
		temp = pdata[index];
		psmc_atr->T = temp & 0xf;
		psmc_pps->pps0 |= temp & 0xf;

		/* Adjust Guard Time */
		if (psmc_atr->N == 0xff) {
			if (psmc_atr->T == 1)
				psmc_atr->N = 1;
			else
				psmc_atr->N = 2;
		}
		index++;
	} else {
		if (psmc_atr->N == 0xff)
			psmc_atr->N = 2;
		goto rx_tk;
	}

	/* TA2 */
	if (temp & 0x10) {
		pr_debug("TA2 Exist!!\n");
		index++;
	}

	/* TB2 */
	if (temp & 0x20) {
		pr_debug("TB2 Exist!!\n");
		index++;
	}

	/* TC2 */
	if (temp & 0x40) {
		pr_debug("TC2 Exist!!\n");
		index++;
	}

	/* TD2 */
	if (temp & 0x80) {
		pr_debug("TD2 Exist!!\n");
		temp = pdata[index];
		index++;
	} else {
		goto rx_tk;
	}

	/* TA3 */
	if (temp & 0x10) {
		pr_debug("TA3 Exist!!\n");
		index++;
	}

	/* TB3 */
	if (temp & 0x20) {
		pr_debug("TB3 Exist!!\n");
		index++;
	}

	/* TC3 */
	if (temp & 0x40) {
		pr_debug("TC3 Exist!!\n");
		index++;
	}

	/* TD3 */
	if (temp & 0x80) {
		pr_debug("TD3 Exist!!\n");
		temp = pdata[index];
		index++;
	} else {
		goto rx_tk;
	}

	/* TA4 */
	if (temp & 0x10) {
		pr_debug("TA4 Exist!!\n");
		index++;
	}

	/* TB4 */
	if (temp & 0x20) {
		pr_debug("TB4 Exist!!\n");
		index++;
	}

	/* TC4 */
	if (temp & 0x40) {
		pr_debug("TC4 Exist!!\n");
		index++;
	}

	/* TD4 */
	if (temp & 0x80) {
		pr_debug("TD4 Exist!!\n");
		temp = pdata[index];
		index++;
	}

rx_tk:
	for (i = 0; i < (psmc_atr->TK_NUM); i++)
		psmc_atr->TK[i] = pdata[index++];

	psmc_pps->pck = psmc_pps->ppss;
	psmc_pps->pck ^= psmc_pps->pps0;
	if (psmc_pps->pps0&(0x1<<4))
		psmc_pps->pck ^= psmc_pps->pps1;
	if (psmc_pps->pps0&(0x1<<5))
		psmc_pps->pck ^= psmc_pps->pps2;
	if (psmc_pps->pps0&(0x1<<6))
		psmc_pps->pck ^= psmc_pps->pps3;
}
