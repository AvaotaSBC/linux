/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * The interface function of controlling the SS register.
 *
 * Copyright (C) 2013 Allwinner.
 *
 * Mintow <duanmintao@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "../sunxi_ce.h"
#include "sunxi_ce_reg.h"

static int gs_ss_osc_prev_state;

inline u32 ss_readl(u32 offset)
{
	return readl(ss_membase() + offset);
}

inline void ss_writel(u32 offset, u32 val)
{
	writel(val, ss_membase() + offset);
}

u32 ss_reg_rd(u32 offset)
{
	return ss_readl(offset);
}

void ss_reg_wr(u32 offset, u32 val)
{
	ss_writel(offset, val);
}

void ss_keyselect_set(int select)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~SS_REG_CTL_KEY_SELECT_MASK;
	val |= select << SS_REG_CTL_KEY_SELECT_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_keysize_set(int size)
{
	int val = ss_readl(SS_REG_CTL);
	int type = SS_AES_KEY_SIZE_128;

	switch (size) {
	case AES_KEYSIZE_192:
		type = SS_AES_KEY_SIZE_192;
		break;
	case AES_KEYSIZE_256:
		type = SS_AES_KEY_SIZE_256;
		break;
	default:
/*		type = SS_AES_KEY_SIZE_128; */
		break;
	}

	val &= ~(SS_REG_CTL_KEY_SIZE_MASK);
	val |= (type << SS_REG_CTL_KEY_SIZE_SHIFT);
	ss_writel(SS_REG_CTL, val);
}

void ss_flow_enable(int flow)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~(SS_STREM1_SELECT|SS_STREM0_SELECT);
	if (flow == 0)
		val |= SS_STREM0_SELECT;
	else
		val |= SS_STREM1_SELECT;
	ss_writel(SS_REG_CTL, val);
}

void ss_flow_mode_set(int mode)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~(SS_REG_CTL_FLOW_MODE_MASK);
	val |= mode;
	ss_writel(SS_REG_CTL, val);
}

void ss_pending_clear(int flow)
{
	int val = ss_readl(SS_REG_ISR);

	val &= ~(SS_REG_ICR_FLOW0_PENDING_MASK|SS_REG_ICR_FLOW1_PENDING_MASK);
	val |= SS_FLOW_PENDING << flow;
	ss_writel(SS_REG_ISR, val);
}

int ss_pending_get(void)
{
	return ss_readl(SS_REG_ISR);
}

void ss_irq_enable(int flow)
{
	int val = ss_readl(SS_REG_ICR);

	val |= SS_FLOW_END_INT_ENABLE << flow;
	ss_writel(SS_REG_ICR, val);
}

void ss_irq_disable(int flow)
{
	int val = ss_readl(SS_REG_ICR);

	val &= ~(SS_FLOW_END_INT_ENABLE << flow);
	ss_writel(SS_REG_ICR, val);
}

/* key: phsical address. */
void ss_key_set(char *key, int size)
{
	int i = 0;
	int key_sel = CE_KEY_SELECT_INPUT;
	struct {
		int type;
		char desc[AES_MIN_KEY_SIZE];
	} key_select[] = {
		{CE_KEY_SELECT_SSK,			CE_KS_SSK},
		{CE_KEY_SELECT_HUK,			CE_KS_HUK},
		{CE_KEY_SELECT_RSSK,		CE_KS_RSSK},
		{CE_KEY_SELECT_INTERNAL_0,	CE_KS_INTERNAL_0},
		{CE_KEY_SELECT_INTERNAL_1,	CE_KS_INTERNAL_1},
		{CE_KEY_SELECT_INTERNAL_2,	CE_KS_INTERNAL_2},
		{CE_KEY_SELECT_INTERNAL_3,	CE_KS_INTERNAL_3},
		{CE_KEY_SELECT_INTERNAL_4,	CE_KS_INTERNAL_4},
		{CE_KEY_SELECT_INTERNAL_5,	CE_KS_INTERNAL_5},
		{CE_KEY_SELECT_INTERNAL_6,	CE_KS_INTERNAL_6},
		{CE_KEY_SELECT_INTERNAL_7,	CE_KS_INTERNAL_7},
		{CE_KEY_SELECT_INPUT, ""}
	};

	while (key_select[i].type != CE_KEY_SELECT_INPUT) {
		if (strncasecmp(key, key_select[i].desc, AES_MIN_KEY_SIZE) == 0) {
			key_sel = key_select[i].type;
			memset(key, 0, size);
			break;
		}
		i++;
	}
	SS_DBG("The key select: %d\n", key_sel);

	ss_keyselect_set(key_sel);
	ss_keysize_set(size);
	ss_writel(SS_REG_KEY_L, virt_to_phys(key));
}

/* iv: phsical address. */
void ss_iv_set(char *iv, int size)
{
	ss_writel(SS_REG_IV_L, virt_to_phys(iv));
}

void ss_cntsize_set(int size)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~SS_REG_CTL_CTR_SIZE_MASK;
	val |= size << SS_REG_CTL_CTR_SIZE_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_cnt_set(char *cnt, int size)
{
	ss_cntsize_set(SS_CTR_SIZE_128);
	ss_writel(SS_REG_IV_L, virt_to_phys(cnt));
}

void ss_cnt_get(int flow, char *cnt, int size)
{
	int i;
	int *val = (int *)cnt;
	int base = SS_REG_CNT_BASE;

	if (flow == 1)
		base = SS_REG_FLOW1_CNT_BASE;

	for (i = 0; i < size/4; i++, val++)
		*val = ss_readl(base + i*4);
}

void ss_md_get(char *dst, char *src, int size)
{
	memcpy(dst, src, size);
}

void ss_iv_mode_set(int mode)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~SS_REG_CTL_IV_MODE_MASK;
	val |= mode << SS_REG_CTL_IV_MODE_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_cts_last(void)
{
	int val = ss_readl(SS_REG_CTL);

	val |= SS_REG_CTL_AES_CTS_LAST;
	ss_writel(SS_REG_CTL, val);
}

void ss_method_set(int dir, int type)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~(SS_REG_CTL_OP_DIR_MASK|SS_REG_CTL_METHOD_MASK);
	val |= dir << SS_REG_CTL_OP_DIR_SHIFT;
	val |= type << SS_REG_CTL_METHOD_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_aes_mode_set(int mode)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~SS_REG_CTL_OP_MODE_MASK;
	val |= mode << SS_REG_CTL_OP_MODE_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_rng_mode_set(int mode)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~SS_REG_CTL_PRNG_MODE_MASK;
	val |= mode << SS_REG_CTL_PRNG_MODE_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_trng_osc_enable(void)
{
	int val = readl(SS_TRNG_OSC_ADDR);

	gs_ss_osc_prev_state = 1;
	if (val & 1)
		return;

	val |= 1;
	writel(val, SS_TRNG_OSC_ADDR);
}

void ss_trng_osc_disable(void)
{
	int val = 0;

	if (gs_ss_osc_prev_state == 1)
		return;

	val = readl(SS_TRNG_OSC_ADDR);
	val &= ~1;
	writel(val, SS_TRNG_OSC_ADDR);
}

void ss_sha_final(void)
{
	/* unsupported. */
}

void ss_check_sha_end(void)
{
	/* unsupported. */
}

void ss_rsa_width_set(int size)
{
	int val = ss_readl(SS_REG_CTL);
	int width_type = SS_RSA_PUB_MODULUS_WIDTH_512;

	switch (size*8) {
	case 512:
		width_type = SS_RSA_PUB_MODULUS_WIDTH_512;
		break;
	case 1024:
		width_type = SS_RSA_PUB_MODULUS_WIDTH_1024;
		break;
	case 2048:
		width_type = SS_RSA_PUB_MODULUS_WIDTH_2048;
		break;
	case 3072:
		width_type = SS_RSA_PUB_MODULUS_WIDTH_3072;
		break;
	default:
		break;
	}

	val &= ~SS_REG_CTL_RSA_PM_WIDTH_MASK;
	val |= width_type<<SS_REG_CTL_RSA_PM_WIDTH_SHIFT;
	ss_writel(SS_REG_CTL, val);
}

void ss_ctrl_start(void)
{
	int val = ss_readl(SS_REG_CTL);

	val |= 1<<27;
	val |= SS_CTL_START;
	ss_writel(SS_REG_CTL, val);
}

void ss_ctrl_stop(void)
{
	int val = ss_readl(SS_REG_CTL);

	val &= ~SS_CTL_START;
	ss_writel(SS_REG_CTL, val);
}

void ss_wait_idle(void)
{
	while ((ss_readl(SS_REG_CTL) & SS_REG_CTL_IDLE_MASK) == SS_CTL_BUSY) {
		/* SS_DBG("Need wait for the hardware.\n"); */
		msleep(20);
	}
}

void ss_data_src_set(int addr)
{
	ss_writel(SS_REG_DATA_SRC_H, 0);
	ss_writel(SS_REG_DATA_SRC_L, addr);
}

void ss_data_dst_set(int addr)
{
	ss_writel(SS_REG_DATA_DST_H, 0);
	ss_writel(SS_REG_DATA_DST_L, addr);
}

void ss_data_len_set(int len)
{
	ss_writel(SS_REG_DATA_LEN, len);
}

int ss_reg_print(char *buf, int len)
{
	return snprintf(buf, len,
		"The SS control register:\n"
		"[CTL] 0x%02x = 0x%08x\n"
		"[ICR] 0x%02x = 0x%08x, [ISR] 0x%02x = 0x%08x\n"
		"[KEY_L] 0x%02x = 0x%08x, [KEY_H] 0x%02x = 0x%08x\n"
		"[IV_L]  0x%02x = 0x%08x, [IV_H]  0x%02x = 0x%08x\n"
		"[DATA_SRC_L] 0x%02x = 0x%08x, [DATA_SRC_H] 0x%02x = 0x%08x\n"
		"[DATA_DST_L] 0x%02x = 0x%08x, [DATA_DST_H] 0x%02x = 0x%08x\n"
		"[DATA_LEN] 0x%02x = 0x%08x\n"
		"[CNT0-3] 0x%02x = 0x%08x, 0x%08x, 0x%08x, 0x%08x\n"
		"[CNT4-7] 0x%02x = 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
		SS_REG_CTL, ss_readl(SS_REG_CTL),
		SS_REG_ICR, ss_readl(SS_REG_ICR),
		SS_REG_ISR, ss_readl(SS_REG_ISR),
		SS_REG_KEY_L, ss_readl(SS_REG_KEY_L),
		SS_REG_KEY_H, ss_readl(SS_REG_KEY_H),
		SS_REG_IV_L, ss_readl(SS_REG_IV_L),
		SS_REG_IV_H, ss_readl(SS_REG_IV_H),
		SS_REG_DATA_SRC_L, ss_readl(SS_REG_DATA_SRC_L),
		SS_REG_DATA_SRC_H, ss_readl(SS_REG_DATA_SRC_H),
		SS_REG_DATA_DST_L, ss_readl(SS_REG_DATA_DST_L),
		SS_REG_DATA_DST_H, ss_readl(SS_REG_DATA_DST_H),
		SS_REG_DATA_LEN, ss_readl(SS_REG_DATA_LEN),
		SS_REG_CNT(0),
		ss_readl(SS_REG_CNT(0)), ss_readl(SS_REG_CNT(1)),
		ss_readl(SS_REG_CNT(2)), ss_readl(SS_REG_CNT(3)),
		SS_REG_FLOW1_CNT(0),
		ss_readl(SS_REG_FLOW1_CNT(0)), ss_readl(SS_REG_FLOW1_CNT(1)),
		ss_readl(SS_REG_FLOW1_CNT(2)), ss_readl(SS_REG_FLOW1_CNT(3))
		);
}
