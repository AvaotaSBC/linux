/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * The interface function of controlling the SS register.
 *
 * Copyright (C) 2018 Allwinner.
 *
 *<xupengliu@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "sunxi_ce_reg.h"

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

void ss_keyselect_set(int select, ce_task_desc_t *task)
{
	task->sym_ctl |= select << CE_SYM_CTL_KEY_SELECT_SHIFT;
}

void ss_keysize_set(int size, ce_task_desc_t *task)
{
	int type = CE_AES_KEY_SIZE_128;

	switch (size) {
	case AES_KEYSIZE_192:
		type = CE_AES_KEY_SIZE_192;
		break;
	case AES_KEYSIZE_256:
		type = CE_AES_KEY_SIZE_256;
		break;
	default:
		break;
	}

	task->sym_ctl |= (type << CE_SYM_CTL_KEY_SIZE_SHIFT);
}

void ce_task_addr_set(u8 *vir_addr, phys_addr_t phy_addr, u8 *dst)
{
	phys_addr_t phy;

	if (phy_addr) {
		phy = phy_addr;
	} else {
		phy = virt_to_phys(vir_addr);
	}

	if (phy > 0xffffffff) {
		phy = phy & CE_ADDR_MASK;
		memcpy(dst, (void *)&phy, 5);
	} else {
		memcpy(dst, (void *)&phy, 4);
	}
}

phys_addr_t ce_task_addr_get(u8 *dst)
{
	phys_addr_t phy;
	if (dst[4]) {
		phy = (*((u64 *)dst)) & CE_ADDR_MASK;
	} else {
		phy = *((u32 *)dst);
	}

	return phy;
}
void ce_task_data_len_set(u32 len, u8 *dst)
{
	dst[0] = len & 0xff;
	dst[1] = (len >> 8) & 0xff;
	dst[2] = (len >> 16) & 0xff;
	dst[3] = (len >> 24) & 0xff;
	dst[4] = 0;
}

/* key: phsical address. */
void ss_key_set(char *key, int size, ce_task_desc_t *task)
{
	int i = 0;
	int key_sel = CE_KEY_SELECT_INPUT;
	struct {
		int type;
		char desc[AES_MIN_KEY_SIZE];
	} keys[] = {
		{CE_KEY_SELECT_SSK,		   CE_KS_SSK},
		{CE_KEY_SELECT_HUK,		   CE_KS_HUK},
		{CE_KEY_SELECT_RSSK,	   CE_KS_RSSK},

		{CE_KEY_SELECT_INTERNAL_0, CE_KS_INTERNAL_0},
		{CE_KEY_SELECT_INTERNAL_1, CE_KS_INTERNAL_1},
		{CE_KEY_SELECT_INTERNAL_2, CE_KS_INTERNAL_2},
		{CE_KEY_SELECT_INTERNAL_3, CE_KS_INTERNAL_3},
		{CE_KEY_SELECT_INTERNAL_4, CE_KS_INTERNAL_4},
		{CE_KEY_SELECT_INTERNAL_5, CE_KS_INTERNAL_5},
		{CE_KEY_SELECT_INTERNAL_6, CE_KS_INTERNAL_6},
		{CE_KEY_SELECT_INTERNAL_7, CE_KS_INTERNAL_7},
		{CE_KEY_SELECT_INPUT, ""} };

	while (keys[i].type != CE_KEY_SELECT_INPUT) {
		if (strncasecmp(key, keys[i].desc, AES_MIN_KEY_SIZE) == 0) {
			key_sel = keys[i].type;
			memset(key, 0, size);
			break;
		}
		i++;
	}
	SS_DBG("The key select: %d\n", key_sel);

	ss_keyselect_set(key_sel, task);
	ss_keysize_set(size, task);

	ce_task_addr_set(key, 0, task->key_addr);
}

void ss_pending_clear(int flow)
{
	int val = 0;

	switch (flow) {
	case (0):
		val = CE_CHAN_PENDING << 0;
		break;
	case (1):
		val = CE_CHAN_PENDING << 2;
		break;
	case (2):
		val = CE_CHAN_PENDING << 4;
		break;
	case (3):
		val = CE_CHAN_PENDING << 6;
		break;

	default:
		break;
	}
	ss_writel(CE_REG_ISR, val);
}

int ss_pending_get(void)
{
	return ss_readl(CE_REG_ISR);
}

void ss_irq_enable(int flow)
{
	int val = ss_readl(CE_REG_ICR);

	val |= CE_CHAN_INT_ENABLE << flow;
	ss_writel(CE_REG_ICR, val);
}

void ss_irq_disable(int flow)
{
	int val = ss_readl(CE_REG_ICR);

	val &= ~(CE_CHAN_INT_ENABLE << flow);
	ss_writel(CE_REG_ICR, val);
}

void ss_md_get(char *dst, char *src, int size)
{
	memcpy(dst, src, size);
}

/* iv: phsical address. */
void ss_iv_set(char *iv, int size, ce_task_desc_t *task)
{
	ce_task_addr_set(iv, 0, task->iv_addr);
}

void ce_iv_phyaddr_set(dma_addr_t phy_addr, ce_task_desc_t *task)
{
	ce_task_addr_set(0, phy_addr, task->iv_addr);
}

void ss_iv_mode_set(int mode, ce_task_desc_t *task)
{
	task->comm_ctl |= mode << CE_COMM_CTL_IV_MODE_SHIFT;
}

void ss_cntsize_set(int size, ce_task_desc_t *task)
{
	task->sym_ctl |= size << CE_SYM_CTL_CTR_SIZE_SHIFT;
}

void ss_cnt_set(char *cnt, int size, ce_task_desc_t *task)
{
	ce_task_addr_set(cnt, 0, task->ctr_addr);
	ss_cntsize_set(CE_CTR_SIZE_128, task);
}

void ss_gcm_cnt_set(char *cnt, int size, ce_task_desc_t *task)
{
	ce_task_addr_set(cnt, 0, task->ctr_addr);
	ss_cntsize_set(CE_CTR_SIZE_32, task);
}

void ss_cts_last(ce_task_desc_t *task)
{
	task->sym_ctl |= CE_SYM_CTL_AES_CTS_LAST;
}

void ss_tag_len_set(u32 len, ce_task_desc_t *task)
{
	task->comm_ctl |= len << CE_COM_CTL_GCM_TAGLEN_SHIFT;
}

void ss_gcm_iv_mode(ce_task_desc_t *task, int iv_mode)
{
	task->sym_ctl |= iv_mode << CE_SYM_CTL_GCM_IV_MODE_SHIFT;
}

void ss_gcm_reserve_set(ce_task_desc_t *task, int iv_len, int aad_len, int pt_len)
{
	task->reserved[0] = iv_len * 8;
	task->reserved[1] = aad_len * 8;
	task->reserved[2] = pt_len * 8;
}

void ss_gcm_src_config(ce_scatter_t *sg, u32 addr, u32 len)
{
	ce_task_addr_set(0, addr, sg->src_addr);
	sg->src_len = len;
	/* src->addr = addr >> 2; */
	/* src->len = len >> 2; */
}

void ss_xts_first(ce_task_desc_t *task)
{
	task->sym_ctl |= CE_SYM_CTL_AES_XTS_FIRST;
}

void ss_xts_last(ce_task_desc_t *task)
{
	task->sym_ctl |= CE_SYM_CTL_AES_XTS_LAST;
}

void ss_method_set(int dir, int type, ce_task_desc_t *task)
{
	task->comm_ctl |= dir << CE_COMM_CTL_OP_DIR_SHIFT;
	task->comm_ctl |= type << CE_COMM_CTL_METHOD_SHIFT;
}

void ss_aes_mode_set(int mode, ce_task_desc_t *task)
{
	task->sym_ctl |= mode << CE_SYM_CTL_OP_MODE_SHIFT;
}

void ss_aead_mode_set(int mode, ce_task_desc_t *task)
{
	task->sym_ctl |= mode << CE_SYM_CTL_OP_MODE_SHIFT;
	task->sym_ctl |= 1 << 2;  /* set gcm mode counter width */
}

void ss_cfb_bitwidth_set(int bitwidth, ce_task_desc_t *task)
{
	int val = 0;

	switch (bitwidth) {
	case 1:
		val = CE_CFB_WIDTH_1;
		break;
	case 8:
		val = CE_CFB_WIDTH_8;
		break;
	case 64:
		val = CE_CFB_WIDTH_64;
		break;
	case 128:
		val = CE_CFB_WIDTH_128;
		break;
	default:
		break;
	}
	task->sym_ctl |= val << CE_SYM_CTL_CFB_WIDTH_SHIFT;
}

void ss_ctr_bitwidth_set(int bitwidth, ce_task_desc_t *task)
{
	int val = 0;

	switch (bitwidth) {
	case 16:
		val = CE_CTR_SIZE_16;
		break;
	case 32:
		val = CE_CTR_SIZE_32;
		break;
	case 64:
		val = CE_CTR_SIZE_64;
		break;
	case 128:
		val = CE_CTR_SIZE_128;
		break;
	default:
		break;
	}

	task->sym_ctl |= val << CE_SYM_CTL_CTR_SIZE_SHIFT;
}

void ss_sha_final(void)
{
	/* unsupported. */
}

void ss_check_sha_end(void)
{
	/* unsupported. */
}

void ss_rsa_width_set(int size, ce_task_desc_t *task)
{
	task->asym_ctl |= DIV_ROUND_UP(size, 4);
}

void ss_rsa_op_mode_set(int mode, ce_task_desc_t *task)
{
	/* Consider !2 as M_EXP, for compatible with the previous SOC. */
	if (mode == CE_RSA_OP_M_MUL)
		task->asym_ctl |= CE_RSA_OP_M_MUL<<CE_ASYM_CTL_RSA_OP_SHIFT;
	else
		task->asym_ctl |= CE_RSA_OP_M_EXP<<CE_ASYM_CTL_RSA_OP_SHIFT;
}

void ss_ecc_width_set(int size, ce_task_desc_t *task)
{
	task->asym_ctl |= DIV_ROUND_UP(size, 4);
}

void ss_ecc_op_mode_set(int mode, ce_task_desc_t *task)
{
	task->asym_ctl |= mode<<CE_ASYM_CTL_RSA_OP_SHIFT;
}

void ss_ctrl_start(ce_task_desc_t *task, int type, int mode)
{
	phys_addr_t task_phy;

	if (task->task_phy_addr) {
		task_phy = task->task_phy_addr;
	} else {
		task_phy = virt_to_phys(task);
	}

	if (task_phy > 0xffffffff) {
		ss_writel(CE_REG_TSK0, task_phy & 0xffffffff);
		ss_writel(CE_REG_TSK1, ((unsigned long long)task_phy >> 32));
	} else {
		ss_writel(CE_REG_TSK0, task_phy);
	}

	if (type == SS_METHOD_RAES)
		ss_writel(CE_REG_TLR, 0x1 << CE_REG_TLR_RAES_TYPE_SHIFT);
	else if (CE_METHOD_IS_AES(type) && (mode == SS_AES_MODE_XTS))
		ss_writel(CE_REG_TLR, 0x1 << CE_REG_TLR_RAES_TYPE_SHIFT);
	else if (CE_METHOD_IS_AES(type) && (mode != SS_AES_MODE_XTS))
		ss_writel(CE_REG_TLR, 0x1 << CE_REG_TLR_SYMM_TYPE_SHIFT);
	else
		ss_writel(CE_REG_TLR, 0x1 << CE_REG_TLR_ASYM_TYPE_SHIFT);
}

void ss_ctrl_hash_start(ce_new_task_desc_t *task, int type, int mode)
{
	phys_addr_t task_phy;

	if (task->task_phy_addr) {
		task_phy = task->task_phy_addr;
	} else {
		task_phy = virt_to_phys(task);
	}

	if (task_phy > 0xffffffff) {
		ss_writel(CE_REG_TSK0, task_phy & 0xffffffff);
		ss_writel(CE_REG_TSK1, ((unsigned long long)task_phy >> 32));
	} else {
		ss_writel(CE_REG_TSK0, task_phy);
	}

	ss_writel(CE_REG_TLR, 0x1 << CE_REG_TLR_HASH_RBG_TYPE_SHIFT);
}

void ss_ctrl_stop(void)
{
	/* unsupported */
}

int ss_flow_err(int flow)
{
	return ss_readl(CE_REG_ERR) & CE_REG_ESR_CHAN_MASK(flow);
}

void ss_wait_idle(void)
{
#ifdef SS_SUPPORT_CE_V3_1
	while ((ss_readl(CE_REG_TSR) & CE_REG_TSR_BUSY_MASK) ==
		CE_REG_TSR_BUSY) {
		SS_DBG("Need wait for the hardware.\n");
		msleep(20);
	}
#else
	while ((ss_readl(CE_REG_TSR) & 0xff) != 0x0) {
		SS_DBG("Need wait for the hardware.\n");
		msleep(20);
	}
#endif
}

void ss_data_len_set(int len, ce_task_desc_t *task)
{
	task->data_len = len;
}

/* key: phsical address. */
void ss_rng_key_set(char *key, int size, ce_new_task_desc_t *task)
{
	ce_task_addr_set(key, 0, task->key_addr);
}

/* iv: phsical address. */
void ss_hash_iv_set(char *iv, int size, ce_new_task_desc_t *task)
{
	ce_task_addr_set(iv, 0, task->iv_addr);
}

void ss_hash_iv_mode_set(int mode, ce_new_task_desc_t *task)
{
	task->comm_ctl |= mode << CE_CTL_IV_MODE_SHIFT;
}

void ss_hmac_sha1_last(ce_new_task_desc_t *task)
{
	task->comm_ctl |= CE_CTL_HMAC_SHA1_LAST;
}

void ss_hmac_method_set(int type, ce_new_task_desc_t *task)
{
	task->main_cmd |= type << CE_CMD_HASH_METHOD_SHIFT;
	task->main_cmd |= 0x1 << CE_CMD_HMAC_METHOD_SHIFT;
}

void ss_hash_method_set(int type, ce_new_task_desc_t *task)
{
	task->main_cmd |= type << CE_CMD_HASH_METHOD_SHIFT;
}

void ss_hash_cmd_set(int channel_id, ce_new_task_desc_t *task)
{
	task->comm_ctl |=
	    ((channel_id << CHN) | (0x1 << LPKG) | (0x0 << DLAV) | (0x1 << IE));
}

void ss_rng_method_set(int hash_type, int type, ce_new_task_desc_t *task)
{
	task->main_cmd |= (type << CE_CMD_RNG_METHOD_SHIFT);
	task->main_cmd |= (hash_type << CE_CMD_HASH_METHOD_SHIFT);

	task->comm_ctl |=
	    ((0x1 << LPKG) | (0x0 << DLAV) | (0x1 << IE));

	if (type == SS_METHOD_DRBG) {
		task->comm_ctl |= CE_CTL_HMAC_SHA1_LAST;
		/* for drbg, need set sub_cmd bit[28:16] */
		task->main_cmd |= (2 << 16 | 0 << 20 | BIT(28));
	}
}

void ss_hash_rng_ctrl_start(ce_new_task_desc_t *task)
{
	phys_addr_t task_phy;

	if (task->task_phy_addr) {
		task_phy = task->task_phy_addr;
	} else {
		task_phy = virt_to_phys(task);
	}

	if (task_phy > 0xffffffff) {
		ss_writel(CE_REG_TSK0, task_phy & 0xffffffff);
		ss_writel(CE_REG_TSK1, ((unsigned long long)task_phy >> 32));
	} else {
		ss_writel(CE_REG_TSK0, task_phy);
	}

	ss_writel(CE_REG_TLR, 0x1 << CE_REG_TLR_HASH_RBG_TYPE_SHIFT);
	task->task_phy_addr = task_phy;
}

void ss_hash_data_len_set(int len, ce_new_task_desc_t *task)
{
	task->comm_ctl |= 0 << 13;
	ce_task_data_len_set(len, task->data_len);
}

int ss_reg_print(char *buf, int len)
{
	return snprintf(buf, len,
		"The SS control register:\n"
		"[TSK0] 0x%02x = 0x%08x\n"
		"[TSK1] 0x%02x = 0x%08x\n"
#ifdef SS_SUPPORT_CE_V3_1
		"[CTL] 0x%02x = 0x%08x\n"
#endif
		"[ICR] 0x%02x = 0x%08x, [ISR] 0x%02x = 0x%08x\n"
		"[TLR] 0x%02x = 0x%08x\n"
		"[TSR] 0x%02x = 0x%08x\n"
		"[ERR] 0x%02x = 0x%08x\n"
#ifdef SS_SUPPORT_CE_V3_1
		"[CSS] 0x%02x = 0x%08x, [CDS] 0x%02x = 0x%08x\n"
#endif
		"[CSA] 0x%02x = 0x%08x, [CDA] 0x%02x = 0x%08x\n"
#ifdef SS_SUPPORT_CE_V3_1
		"[TPR] 0x%02x = 0x%08x\n"
#else
		"[HCSA] 0x%02x = 0x%08x\n"
		"[HCDA] 0x%02x = 0x%08x\n"
		"[ACSA] 0x%02x = 0x%08x\n"
		"[ACDA] 0x%02x = 0x%08x\n"
		"[XCSA] 0x%02x = 0x%08x\n"
		"[XCDA] 0x%02x = 0x%08x\n"
		"[VER] 0x%02x = 0x%08x\n"
#endif
		,
		CE_REG_TSK0, ss_readl(CE_REG_TSK0),
		CE_REG_TSK1, ss_readl(CE_REG_TSK1),
#ifdef SS_SUPPORT_CE_V3_1
		CE_REG_CTL, ss_readl(CE_REG_CTL),
#endif
		CE_REG_ICR, ss_readl(CE_REG_ICR),
		CE_REG_ISR, ss_readl(CE_REG_ISR),
		CE_REG_TLR, ss_readl(CE_REG_TLR),
		CE_REG_TSR, ss_readl(CE_REG_TSR),
		CE_REG_ERR, ss_readl(CE_REG_ERR),
#ifdef SS_SUPPORT_CE_V3_1
		CE_REG_CSS, ss_readl(CE_REG_CSS),
		CE_REG_CDS, ss_readl(CE_REG_CDS),
#endif
		CE_REG_CSA, ss_readl(CE_REG_CSA),
		CE_REG_CDA, ss_readl(CE_REG_CDA)
#ifdef SS_SUPPORT_CE_V3_1
		,
		CE_REG_TPR, ss_readl(CE_REG_TPR)
#else
		,
		CE_REG_HCSA, ss_readl(CE_REG_HCSA),
		CE_REG_HCDA, ss_readl(CE_REG_HCDA),
		CE_REG_ACSA, ss_readl(CE_REG_ACSA),
		CE_REG_ACDA, ss_readl(CE_REG_ACDA),
		CE_REG_XCSA, ss_readl(CE_REG_XCSA),
		CE_REG_XCDA, ss_readl(CE_REG_XCDA),
		CE_REG_VER, ss_readl(CE_REG_VER)
#endif
		);
}


void ce_reg_print(void)
{
		pr_err("The SS control register:\n");
		pr_err("[TSK] 0x%02x = 0x%08x\n", CE_REG_TSK0, ss_readl(CE_REG_TSK0));
		pr_err("[TSK] 0x%02x = 0x%08x\n", CE_REG_TSK1, ss_readl(CE_REG_TSK1));
#ifdef SS_SUPPORT_CE_V3_1
		pr_err("[CTL] 0x%02x = 0x%08x\n", CE_REG_CTL, ss_readl(CE_REG_CTL));
#endif
		pr_err("[ICR] 0x%02x = 0x%08x\n", CE_REG_ICR, ss_readl(CE_REG_ICR));
		pr_err("[ISR] 0x%02x = 0x%08x\n", CE_REG_ISR, ss_readl(CE_REG_ISR));
		pr_err("[TLR] 0x%02x = 0x%08x\n", CE_REG_TLR, ss_readl(CE_REG_TLR));
		pr_err("[TSR] 0x%02x = 0x%08x\n", CE_REG_TSR, ss_readl(CE_REG_TSR));
		pr_err("[ERR] 0x%02x = 0x%08x\n", CE_REG_ERR, ss_readl(CE_REG_ERR));
#ifdef SS_SUPPORT_CE_V3_1
		pr_err("[CSS] 0x%02x = 0x%08x\n", CE_REG_CSS, ss_readl(CE_REG_CSS));
		pr_err("[CDS] 0x%02x = 0x%08x\n", CE_REG_CDS, ss_readl(CE_REG_CDS));
#endif
		pr_err("[CSA] 0x%02x = 0x%08x\n", CE_REG_CSA, ss_readl(CE_REG_CSA));
		pr_err("[CDA] 0x%02x = 0x%08x\n", CE_REG_CDA, ss_readl(CE_REG_CDA));
#ifdef SS_SUPPORT_CE_V3_1
		pr_err("[TPR] 0x%02x = 0x%08x\n", CE_REG_TPR, ss_readl(CE_REG_TPR));
#else
		pr_err("[HCSA] 0x%02x = 0x%08x\n", CE_REG_HCSA, ss_readl(CE_REG_HCSA));
		pr_err("[HCDA] 0x%02x = 0x%08x\n", CE_REG_HCDA, ss_readl(CE_REG_HCDA));
		pr_err("[ACSA] 0x%02x = 0x%08x\n", CE_REG_ACSA, ss_readl(CE_REG_ACSA));
		pr_err("[ACDA] 0x%02x = 0x%08x\n", CE_REG_ACDA, ss_readl(CE_REG_ACDA));
		pr_err("[XCSA] 0x%02x = 0x%08x\n", CE_REG_XCSA, ss_readl(CE_REG_XCSA));
		pr_err("[XCDA] 0x%02x = 0x%08x\n", CE_REG_XCDA, ss_readl(CE_REG_XCDA));
		pr_err("[VER] 0x%02x = 0x%08x\n", CE_REG_VER, ss_readl(CE_REG_VER));
#endif

}
