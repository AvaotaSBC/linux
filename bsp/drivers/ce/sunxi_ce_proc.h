/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Declare the function interface of SUNXI SS process.
 *
 * Copyright (C) 2014 Allwinner.
 *
 * Mintow <duanmintao@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _SUNXI_SECURITY_SYSTEM_PROC_H_
#define _SUNXI_SECURITY_SYSTEM_PROC_H_

#include <crypto/aes.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/sha3.h>
//#include <crypto/algapi.h>
#include <crypto/skcipher.h>
#include <linux/scatterlist.h>
#include "sunxi_ce_cdev.h"

#define SRC_FLAG	(0x0)
#define DST_FLAG	(0x1)
/* Inner functions declaration, defined in vx/sunxi_ss_proc.c */

int ss_aes_key_valid(struct crypto_tfm *tfm, int len);
int ss_aes_one_req(sunxi_ce_cdev_t *sss, struct skcipher_request *req);

#ifdef SS_GCM_MODE_ENABLE
int ss_aead_crypt(struct aead_request *req, int dir, int method, int mode);
int ss_aead_one_req(sunxi_ce_cdev_t *sss, struct aead_request *req);
#endif

#ifdef SS_DRBG_MODE_ENABLE
int ss_drbg_get_random(struct crypto_rng *tfm, const u8 *src, u32 slen,
				u8 *rdata, u32 dlen, u32 mode);
int ss_drbg_sha1_get_random(struct crypto_rng *tfm, const u8 *src,
				unsigned int slen, u8 *dst, unsigned int dlen);
int ss_drbg_sha256_get_random(struct crypto_rng *tfm, const u8 *src,
				unsigned int slen, u8 *dst, unsigned int dlen);
int ss_drbg_sha512_get_random(struct crypto_rng *tfm, const u8 *src,
				unsigned int slen, u8 *dst, unsigned int dlen);
#endif

int ss_rng_get_random(struct crypto_rng *tfm, u8 *rdata, u32 dlen, u32 trng);

u32 ss_hash_start(ss_hash_ctx_t *ctx,
		ss_aes_req_ctx_t *req_ctx, u32 len, u32 last);

irqreturn_t sunxi_ss_irq_handler(int irq, void *dev_id);

/* defined in sunxi_ss_proc_comm.c */

void ss_print_hex(char *_data, int _len, void *_addr);
#ifdef SS_SCATTER_ENABLE
void ss_print_task_info(ce_task_desc_t *task);
#endif
int ss_sg_cnt(struct scatterlist *sg, int total);

int ss_prng_get_random(struct crypto_rng *tfm, const u8 *src, unsigned int slen,
		u8 *dst, unsigned int dlen);

int ce_trng_get_random(u8 *buf, u32 rng_len, u32 flag);
#ifdef SS_TRNG_ENABLE
int ss_trng_get_random(struct crypto_rng *tfm, const u8 *src, unsigned int slen,
		u8 *dst, unsigned int dlen);
#endif
#ifdef SS_TRNG_POSTPROCESS_ENABLE
void ss_trng_postprocess(u8 *out, u32 outlen, u8 *in, u32 inlen);
#endif

int ss_aes_crypt(struct skcipher_request *req, int dir, int method, int mode);

void ss_hash_swap(char *data, int len);
int ss_hash_blk_size(int type);
void ss_hash_padding_sg_prepare(struct scatterlist *last, int total);
int ss_hash_update(struct ahash_request *req);
int ss_hash_final(struct ahash_request *req);
int ss_hash_finup(struct ahash_request *req);
int ss_hash_digest(struct ahash_request *req);

#endif /* end of _SUNXI_SECURITY_SYSTEM_PROC_H_ */
