/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * The register macro of SUNXI SecuritySystem controller.
 *
 * Copyright (C) 2018 Allwinner.
 *
 * <xupengliu@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _SUNXI_SECURITY_SYSTEM_REG_H_
#define _SUNXI_SECURITY_SYSTEM_REG_H_
#include "../sunxi_ce_cdev.h"

/* CE: Crypto Engine, start using CE from sun8iw7/sun8iw9 */
#define CE_REG_TSK			0x00
#define CE_REG_ICR			0x08
#define CE_REG_ISR			0x0C
#define CE_REG_TLR			0x10
#define CE_REG_TSR			0x14
#define CE_REG_ERR			0x18

#define CE_REG_DRL			0x1c

#define CE_REG_CSA			0x24
#define CE_REG_CDA			0x28
#define CE_REG_HCSA			0x34
#define CE_REG_HCDA			0x38
#define CE_REG_ACSA			0x44
#define CE_REG_ACDA			0x48
#define CE_REG_XCSA			0x54
#define CE_REG_XCDA			0x58
#define CE_REG_VER			0x90

#define CE_CHAN_INT_ENABLE		1


#define CE_REG_TLR_METHOD_TYPE_SHIFT		8

#define CE_REG_ESR_ERR_UNSUPPORT	0
#define CE_REG_ESR_ERR_LEN			1
#define CE_REG_ESR_ERR_KEYSRAM		2

#define CE_REG_ESR_ERR_ADDR			5


#define CE_REG_ESR_CHAN_SHIFT		8
#define CE_REG_ESR_CHAN_MASK(flow)	(0xFF << (CE_REG_ESR_CHAN_SHIFT*flow))

/* About the hash/RBG control word */
#define CE_CTL_CHAN_MASK	0

#define CE_CHAN_PENDING	0x3

#define CE_REG_TLR_SYMM_TYPE_SHIFT		0
#define CE_REG_TLR_HASH_RBG_TYPE_SHIFT	1
#define CE_REG_TLR_ASYM_TYPE_SHIFT		2
#define CE_REG_TLR_RAES_TYPE_SHIFT		3

#define CE_CMD_HASH_METHOD_SHIFT	0
#define CE_CMD_RNG_METHOD_SHIFT	8
#define CE_CMD_HMAC_METHOD_SHIFT	4

#define CE_CTL_IV_MODE_SHIFT	8
#define CE_CTL_HMAC_SHA1_LAST	BIT(12)

#define CE_CTL_IE_SHIFT		16
#define CE_CTL_IE_MASK 	(0x1 << CE_CTL_IE_SHIFT)

#define SS_METHOD_MD5				0
#define SS_METHOD_SHA1			1
#define SS_METHOD_SHA224			2
#define SS_METHOD_SHA256			3
#define SS_METHOD_SHA384			4
#define SS_METHOD_SHA512			5
#define SS_METHOD_SM3				6

#define SS_METHOD_PRNG			1
#define SS_METHOD_TRNG			2
#define SS_METHOD_DRBG			3

/* About the common control word */
#define CE_COMM_CTL_TASK_INT_SHIFT	31
#define CE_COMM_CTL_TASK_INT_MASK	(0x1 << CE_COMM_CTL_TASK_INT_SHIFT)

#define CE_CBC_MAC_LEN_SHIFT		17

#define CE_HASH_IV_DEFAULT			0
#define CE_HASH_IV_INPUT			1
#define CE_COMM_CTL_IV_MODE_SHIFT	16

#define CE_HMAC_SHA1_LAST			BIT(15)

#define SS_DIR_ENCRYPT				0
#define SS_DIR_DECRYPT				1
#define CE_COMM_CTL_OP_DIR_SHIFT	8

#define SS_METHOD_AES				0x0
#define SS_METHOD_DES				0x1
#define SS_METHOD_3DES			0x2
#define SS_METHOD_SM4				0x3

#define SS_METHOD_HMAC_SHA1		0x10	/* to distinguish hmac,
										but this is not in hardware in fact */
#define SS_METHOD_HMAC_SHA256	0x11

#define SS_METHOD_RSA				0x20
#define SS_METHOD_DH				SS_METHOD_RSA
#define SS_METHOD_ECC				0x21
#define SS_METHOD_SM2				0x22
#define SS_METHOD_RAES				0x30	/* XTS mode */

#define CE_COMM_CTL_METHOD_SHIFT		0
#define CE_COMM_CTL_METHOD_MASK		0x7F

#define CE_METHOD_IS_HASH(type) ((type == SS_METHOD_MD5) \
				|| (type == SS_METHOD_SHA1) \
				|| (type == SS_METHOD_SHA224) \
				|| (type == SS_METHOD_SHA256) \
				|| (type == SS_METHOD_SHA384) \
				|| (type == SS_METHOD_SHA512) \
				|| (type == SS_METHOD_SM3))

#define CE_METHOD_IS_AES(type) ((type == SS_METHOD_AES) \
				|| (type == SS_METHOD_DES) \
				|| (type == SS_METHOD_3DES) \
				|| (type == SS_METHOD_SM4))

#define CE_METHOD_IS_HMAC(type) ((type == SS_METHOD_HMAC_SHA1) \
				|| (type == SS_METHOD_HMAC_SHA256))

/* About the symmetric control word */

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
#define CE_SYM_CTL_KEY_SELECT_SHIFT	20

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

#define CE_CFB_WIDTH_1				0
#define CE_CFB_WIDTH_8				1
#define CE_CFB_WIDTH_64				2
#define CE_CFB_WIDTH_128			3

#define CE_SYM_CTL_CFB_WIDTH_SHIFT	18
#define CE_SYM_CTL_GCM_IV_MODE_SHIFT	4

#define CE_SYM_CTL_AES_CTS_LAST		BIT(16)

#define CE_SYM_CTL_AES_XTS_LAST		BIT(13)
#define CE_SYM_CTL_AES_XTS_FIRST	BIT(12)
#define CE_COM_CTL_GCM_TAGLEN_SHIFT	17

#define TAG_START       48
#define IV_SIZE_START   64
#define AAD_SIZE_START  72
#define PT_SIZE_START   80

#define SS_AES_MODE_ECB				0
#define SS_AES_MODE_CBC				1
#define SS_AES_MODE_CTR				2
#define SS_AES_MODE_CTS				3
#define SS_AES_MODE_OFB				4
#define SS_AES_MODE_CFB				5
#define SS_AES_MODE_CBC_MAC			6
#define SS_AES_MODE_OCB				7
#define SS_AES_MODE_GCM				8
#define SS_AES_MODE_XTS				9

#define CE_SYM_CTL_OP_MODE_SHIFT	8

#define CE_CTR_SIZE_16				0
#define CE_CTR_SIZE_32				1
#define CE_CTR_SIZE_64				2
#define CE_CTR_SIZE_128				3
#define CE_SYM_CTL_CTR_SIZE_SHIFT	2

#define CE_AES_KEY_SIZE_128			0
#define CE_AES_KEY_SIZE_192			1
#define CE_AES_KEY_SIZE_256			2
#define CE_SYM_CTL_KEY_SIZE_SHIFT	0

#define CE_IS_AES_MODE(type, mode, M) (CE_METHOD_IS_AES(type) \
					&& (mode == SS_AES_MODE_##M))

/* About the asymmetric control word */
#define CE_RSA_OP_M_EXP		0 /* modular exponentiation */

#define CE_RSA_OP_M_ADD                 1 /* modular add */
#define CE_RSA_OP_M_MINUS               2 /* modular minus */
#define CE_RSA_OP_M_MUL                 3 /* modular multiplication */

#define CE_ASYM_CTL_RSA_OP_SHIFT		16

#define CE_ECC_OP_POINT_ADD             0 /* point add */
#define CE_ECC_OP_POINT_DBL             1 /* point double */
#define CE_ECC_OP_POINT_MUL             2 /* point multiplication */
#define CE_ECC_OP_POINT_VER             3 /* point verification */
#define CE_ECC_OP_ENC                   4 /* encryption */
#define CE_ECC_OP_DEC                   5 /* decryption */
#define CE_ECC_OP_SIGN                  6 /* sign */
#define CE_ECC_OP_VERIFY                7 /* verification */

#define SS_SEED_SIZE			24


/* Function declaration */

u32 ss_reg_rd(u32 offset);
void ss_reg_wr(u32 offset, u32 val);

void ss_key_set(char *key, int size, ce_task_desc_t *task);

int ss_pending_get(void);
void ss_pending_clear(int flow);
void ss_irq_enable(int flow);
void ss_irq_disable(int flow);

void ss_iv_set(char *iv, int size, ce_task_desc_t *task);
void ss_iv_mode_set(int mode, ce_task_desc_t *task);

void ss_cnt_set(char *cnt, int size, ce_task_desc_t *task);
void ss_cnt_get(int flow, char *cnt, int size);

void ss_md_get(char *dst, char *src, int size);
void ss_sha_final(void);
void ss_check_sha_end(void);

void ss_rsa_width_set(int size, ce_task_desc_t *task);
void ss_rsa_op_mode_set(int mode, ce_task_desc_t *task);

void ss_ecc_width_set(int size, ce_task_desc_t *task);
void ss_ecc_op_mode_set(int mode, ce_task_desc_t *task);

void ss_cts_last(ce_task_desc_t *task);

void ss_xts_first(ce_task_desc_t *task);
void ss_xts_last(ce_task_desc_t *task);

void ss_method_set(int dir, int type, ce_task_desc_t *task);

void ss_aes_mode_set(int mode, ce_task_desc_t *task);

void ss_aead_mode_set(int mode, ce_task_desc_t *task);
void ss_tag_len_set(u32 len, ce_task_desc_t *task);
void ss_gcm_src_config(ce_scatter_t *src, u32 addr, u32 len);
void ss_gcm_reserve_set(ce_task_desc_t *task, int iv_len, int aad_len, int pt_len);
void ss_gcm_cnt_set(char *cnt, int size, ce_task_desc_t *task);
void ss_gcm_iv_mode(ce_task_desc_t *task, int iv_mode);

void ss_cfb_bitwidth_set(int bitwidth, ce_task_desc_t *task);

void ss_wait_idle(void);
void ss_ctrl_start(ce_task_desc_t *task, int type, int mode);
void ss_ctrl_stop(void);
int ss_flow_err(int flow);

void ss_data_len_set(int len, ce_task_desc_t *task);

int ss_reg_print(char *buf, int len);
void ss_keyselect_set(int select, ce_task_desc_t *task);
void ss_keysize_set(int size, ce_task_desc_t *task);

void ss_rng_key_set(char *key, int size, ce_new_task_desc_t *task);
void ss_hash_iv_set(char *iv, int size, ce_new_task_desc_t *task);
void ss_hash_iv_mode_set(int mode, ce_new_task_desc_t *task);
void ss_hmac_sha1_last(ce_new_task_desc_t *task);
void ss_hmac_method_set(int type, ce_new_task_desc_t *task);
void ss_hash_method_set(int type, ce_new_task_desc_t *task);
void ss_rng_method_set(int hash_type, int type, ce_new_task_desc_t *task);
void ss_hash_rng_ctrl_start(ce_new_task_desc_t *task);
void ss_hash_data_len_set(int len, ce_new_task_desc_t *task);

#endif /* end of _SUNXI_SECURITY_SYSTEM_REG_H_ */
