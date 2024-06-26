/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * The register macro of SUNXI SecuritySystem controller.
 *
 * Copyright (C) 2013 Allwinner.
 *
 * Mintow <duanmintao@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _SUNXI_SECURITY_SYSTEM_REG_H_
#define _SUNXI_SECURITY_SYSTEM_REG_H_

#include <mach/platform.h>

#define SS_REG_CTL			0x00
#define SS_REG_ICR			0x04
#define SS_REG_ISR			0x08
#define SS_REG_KEY_L		0x10
#define SS_REG_KEY_H		0x14
#define SS_REG_IV_L			0x18
#define SS_REG_IV_H			0x1C
#define SS_REG_DATA_SRC_L	0x20
#define SS_REG_DATA_SRC_H	0x24
#define SS_REG_DATA_DST_L	0x28
#define SS_REG_DATA_DST_H	0x2C
#define SS_REG_DATA_LEN		0x30

#define SS_REG_CNT_BASE			0x34
#define SS_REG_CNT(n)			(SS_REG_CNT_BASE + 4*n)
#define SS_REG_CNT_NUM			4
#define SS_REG_FLOW1_CNT_BASE	0x48
#define SS_REG_FLOW1_CNT(n)		(SS_REG_FLOW1_CNT_BASE + 4*n)

#define SS_STREM1_SELECT		BIT(31)
#define SS_STREM0_SELECT		BIT(30)

#define SS_CTL_BUSY				0
#define SS_CTL_IDLE				1
#define SS_REG_CTL_IDLE_SHIFT	29
#define SS_REG_CTL_IDLE_MASK	(0x1 << SS_REG_CTL_IDLE_SHIFT)

#define SS_FLOW_MODE_NON_CONTINUE	0
#define SS_FLOW_MODE_CONINUE		1
#define SS_REG_CTL_FLOW_MODE_SHIFT	28
#define SS_REG_CTL_FLOW_MODE_MASK	(0x1 << SS_REG_CTL_FLOW_MODE_SHIFT)

#define SS_DMA_CONSISENT_SEND_END	0
#define SS_DMA_CONSISENT_WAIT_END	1
#define SS_REG_CTL_DMA_CON_SHIFT	27
#define SS_REG_CTL_DMA_CON_MASK		(0x1 << SS_REG_CTL_DMA_CON_SHIFT)

#define SS_KEY_SELECT_INPUT			0
#define SS_KEY_SELECT_INTER0		8
#define SS_REG_CTL_KEY_SELECT_SHIFT	23
#define SS_REG_CTL_KEY_SELECT_MASK	(0xF << SS_REG_CTL_KEY_SELECT_SHIFT)

#define SS_CFG_GENERATE_POLY_NONE	0
#define SS_CFG_GENERATE_POLY		1
#define SS_REG_CTL_POLY_SHIFT		19
#define SS_REG_CTL_POLY_MASK		(0x1 << SS_REG_CTL_POLY_SHIFT)

#define SS_RNG_MODE_ONESHOT			0
#define SS_RNG_MODE_CONTINUE		1
#define SS_REG_CTL_PRNG_MODE_SHIFT	18
#define SS_REG_CTL_PRNG_MODE_MASK	(1 << SS_REG_CTL_PRNG_MODE_SHIFT)

#define SS_IV_MODE_CONSTANT			0
#define SS_IV_MODE_ARBITRARY		1
#define SS_REG_CTL_IV_MODE_SHIFT	17
#define SS_REG_CTL_IV_MODE_MASK		(1 << SS_REG_CTL_IV_MODE_SHIFT)

#define SS_REG_CTL_AES_CTS_LAST		BIT(16)

#define SS_REG_WRITABLE				0
#define SS_REG_UNWRITABLE			1
#define SS_REG_CTL_CFG_VALID_SHIFT	15
#define SS_REG_CTL_CFG_VALID_MASK	(0x1 << SS_REG_CTL_CFG_VALID_SHIFT)

#define SS_AES_MODE_ECB				0
#define SS_AES_MODE_CBC				1
#define SS_AES_MODE_CTR				2
#define SS_AES_MODE_CTS				3
#define SS_REG_CTL_OP_MODE_SHIFT	13
#define SS_REG_CTL_OP_MODE_MASK		(0x3 << SS_REG_CTL_OP_MODE_SHIFT)

#define SS_CTR_SIZE_16				0
#define SS_CTR_SIZE_32				1
#define SS_CTR_SIZE_64				2
#define SS_CTR_SIZE_128				3
#define SS_REG_CTL_CTR_SIZE_SHIFT	11
#define SS_REG_CTL_CTR_SIZE_MASK	(0x3 << SS_REG_CTL_CTR_SIZE_SHIFT)

#define CE_RSA_OP_M_EXP		1 /* modular exponentiation */
#define SS_RSA_PUB_MODULUS_WIDTH_512	0
#define SS_RSA_PUB_MODULUS_WIDTH_1024	1
#define SS_RSA_PUB_MODULUS_WIDTH_2048	2
#define SS_RSA_PUB_MODULUS_WIDTH_3072	3
#define SS_REG_CTL_RSA_PM_WIDTH_SHIFT	9
#define SS_REG_CTL_RSA_PM_WIDTH_MASK	(0x3 << SS_REG_CTL_RSA_PM_WIDTH_SHIFT)

#define SS_AES_KEY_SIZE_128         0
#define SS_AES_KEY_SIZE_192         1
#define SS_AES_KEY_SIZE_256         2
#define SS_REG_CTL_KEY_SIZE_SHIFT   7
#define SS_REG_CTL_KEY_SIZE_MASK    (0x3 << SS_REG_CTL_KEY_SIZE_SHIFT)

#define SS_DIR_ENCRYPT              0
#define SS_DIR_DECRYPT              1
#define SS_REG_CTL_OP_DIR_SHIFT     6
#define SS_REG_CTL_OP_DIR_MASK      (0x1 << SS_REG_CTL_OP_DIR_SHIFT)

#define SS_METHOD_AES			0
#define SS_METHOD_DES			1
#define SS_METHOD_3DES			2
#define SS_METHOD_MD5			3
#define SS_METHOD_PRNG			4
#define SS_METHOD_TRNG			5
#define SS_METHOD_SHA1			6
#define SS_METHOD_SHA224		7
#define SS_METHOD_SHA256		8
#define SS_METHOD_RSA			9
#define SS_METHOD_CRC32			10
#define SS_REG_CTL_METHOD_SHIFT	2
#define SS_REG_CTL_METHOD_MASK	(0xF << SS_REG_CTL_METHOD_SHIFT)

#define SS_METHOD_IS_HASH(type) ((type == SS_METHOD_MD5) \
				|| (type == SS_METHOD_SHA1) \
				|| (type == SS_METHOD_SHA224) \
				|| (type == SS_METHOD_SHA256))

#define SS_CTL_START			1
#define SS_REG_CTL_START_MASK	0x1

#define SS_FLOW_END_INT_DISABLE		0
#define SS_FLOW_END_INT_ENABLE		1
#define SS_REG_ICR_FLOW1_INT_SHIFT	1
#define SS_REG_ICR_FLOW1_INT_MASK	(1 << SS_REG_ICR_FLOW1_INT_SHIFT)
#define SS_REG_ICR_FLOW0_INT_SHIFT	0
#define SS_REG_ICR_FLOW0_INT_MASK	(1 << SS_REG_ICR_FLOW0_INT_SHIFT)

#define SS_FLOW_NO_PENDING				0
#define SS_FLOW_PENDING					1
#define SS_REG_ICR_FLOW1_PENDING_SHIFT	1
#define SS_REG_ICR_FLOW1_PENDING_MASK	(1 << SS_REG_ICR_FLOW1_PENDING_SHIFT)
#define SS_REG_ICR_FLOW0_PENDING_SHIFT	0
#define SS_REG_ICR_FLOW0_PENDING_MASK	(1 << SS_REG_ICR_FLOW0_PENDING_SHIFT)

#ifdef CONFIG_ARCH_SUN8IW6
#define SS_TRNG_OSC_ADDR	((void __iomem *)IO_ADDRESS(0x01f01400 + 0x1f4))
#endif

#define SS_SEED_SIZE			24

#define CE_KEY_SELECT_INPUT			0
#define CE_KEY_SELECT_SSK			1
#define CE_KEY_SELECT_HUK			2
#define CE_KEY_SELECT_RSSK			3
#define CE_KEY_SELECT_INTERNAL_0	8
#define CE_KEY_SELECT_INTERNAL_1	9
#define CE_KEY_SELECT_INTERNAL_2	10
#define CE_KEY_SELECT_INTERNAL_3	11
#define CE_KEY_SELECT_INTERNAL_4	12
#define CE_KEY_SELECT_INTERNAL_5	13
#define CE_KEY_SELECT_INTERNAL_6	14
#define CE_KEY_SELECT_INTERNAL_7	15

/* The identification string to indicate the key source. */
#define CE_KS_SSK			"KEY_SEL_SSK"
#define CE_KS_HUK			"KEY_SEL_HUK"
#define CE_KS_RSSK			"KEY_SEL_RSSK"
#define CE_KS_INTERNAL_0	"KEY_SEL_INTRA_0"
#define CE_KS_INTERNAL_1	"KEY_SEL_INTRA_1"
#define CE_KS_INTERNAL_2	"KEY_SEL_INTRA_2"
#define CE_KS_INTERNAL_3	"KEY_SEL_INTRA_3"
#define CE_KS_INTERNAL_4	"KEY_SEL_INTRA_4"
#define CE_KS_INTERNAL_5	"KEY_SEL_INTRA_5"
#define CE_KS_INTERNAL_6	"KEY_SEL_INTRA_6"
#define CE_KS_INTERNAL_7	"KEY_SEL_INTRA_7"


/* Function declaration */

u32 ss_reg_rd(u32 offset);
void ss_reg_wr(u32 offset, u32 val);

void ss_keyselect_set(int select);
void ss_keysize_set(int size);
void ss_key_set(char *key, int size);

void ss_fifo_init(void);
void ss_flow_enable(int flow);
void ss_flow_mode_set(int mode);

int ss_pending_get(void);
void ss_pending_clear(int flow);
void ss_irq_enable(int flow);
void ss_irq_disable(int flow);

void ss_iv_set(char *iv, int size);
void ss_cntsize_set(int size);
void ss_cnt_set(char *cnt, int size);
void ss_cnt_get(int flow, char *cnt, int size);

void ss_md_get(char *dst, char *src, int size);
void ss_sha_final(void);
void ss_check_sha_end(void);

void ss_iv_mode_set(int mode);

void ss_rsa_width_set(int size);

void ss_cts_last(void);

void ss_method_set(int dir, int type);
void ss_aes_mode_set(int mode);
void ss_rng_mode_set(int mode);
void ss_trng_osc_enable(void);
void ss_trng_osc_disable(void);

void ss_wait_idle(void);
void ss_ctrl_start(void);
void ss_ctrl_stop(void);

void ss_data_src_set(int addr);
void ss_data_dst_set(int addr);
void ss_data_len_set(int len);

int ss_reg_print(char *buf, int len);

#endif /* end of _SUNXI_SECURITY_SYSTEM_REG_H_ */
