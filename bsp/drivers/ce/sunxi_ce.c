/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * The driver of SUNXI SecuritySystem controller.
 *
 * Copyright (C) 2013 Allwinner.
 *
 * Mintow <duanmintao@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/des.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/rng.h>

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/reset.h>
#include "sunxi_ce_cdev.h"
#include "sunxi_ce_proc.h"

#ifdef SS_SUPPORT_CE_V5
#include "v5/sunxi_ce_reg.h"
#elif defined(SS_SUPPORT_CE_V4)
#include "v4/sunxi_ce_reg.h"
#else
#include "v3/sunxi_ce_reg.h"
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

static const struct of_device_id sunxi_ss_of_match[] = {
		{.compatible = "allwinner,sunxi-ce",},
		{},
};
MODULE_DEVICE_TABLE(of, sunxi_ss_of_match);
#endif

sunxi_ce_cdev_t *ss_dev;

static DEFINE_MUTEX(ss_lock);

void ss_dev_lock(void)
{
	mutex_lock(&ss_lock);
}

void ss_dev_unlock(void)
{
	mutex_unlock(&ss_lock);
}

void __iomem *ss_membase(void)
{
	return ss_dev->base_addr;
}

void print_hex(void *_data, int _len)
{
	int i;
	unsigned char *data = (unsigned char *)_data;

	pr_err("-------------------- The valid len = %d ----------------------- \n", _len);
	for (i = 0; i < (_len + 7) / 8; i++) {
		pr_err("0x%08X: %02X %02X %02X %02X %02X %02X %02X %02X \n", i * 8,
			data[i*8+0], data[i*8+1], data[i*8+2], data[i*8+3],
			data[i*8+4], data[i*8+5], data[i*8+6], data[i*8+7]);
	}
	pr_err("-------------------------------------------------------------- \n");

}

void ss_reset(void)
{
	SS_ENTER();
	reset_control_assert(ss_dev->reset);
	reset_control_deassert(ss_dev->reset);
}

#ifdef SS_RSA_CLK_ENABLE
void ss_clk_set(u32 rate)
{
}
#endif

static int ss_aes_key_is_weak(const u8 *key, unsigned int keylen)
{
	s32 i;
	u8 tmp = key[0];

	for (i = 0; i < keylen; i++)
		if (tmp != key[i])
			return 0;

	SS_ERR("The key is weak!\n");
	return 1;
}

#ifdef SS_GCM_MODE_ENABLE
static int sunxi_aes_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
				unsigned int keylen)
{
	ss_aead_ctx_t *ctx = crypto_aead_ctx(tfm);

	if (keylen != AES_KEYSIZE_256 &&
	    keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_128) {
		/* crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN); */
		return -EINVAL;
	}

	memcpy(ctx->key, key, keylen);
	ctx->key_size = keylen;

	return 0;
}

static int sunxi_aes_gcm_setauthsize(struct crypto_aead *tfm,
				     unsigned int authsize)
{
	/* Same as crypto_gcm_authsize() from crypto/gcm.c */
	switch (authsize) {
	case 4:
	case 8:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif  /* SS_GCM_MODE_ENABLE */

static int ss_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
				unsigned int keylen)
{
	int ret = 0;
	ss_aes_ctx_t *ctx = crypto_skcipher_ctx(tfm);

	SS_DBG("keylen = %d\n", keylen);
	if (ctx->comm.flags & SS_FLAG_NEW_KEY) {
		SS_ERR("The key has already update.\n");
		return -EBUSY;
	}

	ret = ss_aes_key_valid(crypto_skcipher_tfm(tfm), keylen);
	if (ret != 0)
		return ret;

	if (ss_aes_key_is_weak(key, keylen)) {
		crypto_skcipher_tfm(tfm)->crt_flags
					|= CRYPTO_TFM_REQ_FORBID_WEAK_KEYS;
		/* testmgr.c need this, but we don't want to support it. */
/*		return -EINVAL; */
	}

	ctx->key_size = keylen;
	memcpy(ctx->key, key, keylen);
	if (keylen < AES_KEYSIZE_256)
		memset(&ctx->key[keylen], 0, AES_KEYSIZE_256 - keylen);

	ctx->comm.flags |= SS_FLAG_NEW_KEY;
	return 0;
}

static int ss_aes_ecb_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_ECB);
}

static int ss_aes_ecb_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_ECB);
}

static int ss_aes_cbc_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CBC);
}

static int ss_aes_cbc_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CBC);
}

static int ss_aes_ctr_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CTR);
}

static int ss_aes_ctr_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CTR);
}

static int ss_aes_cts_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CTS);
}

static int ss_aes_cts_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CTS);
}

#ifdef SS_XTS_MODE_ENABLE
static int ss_aes_xts_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
			SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_XTS);
}

static int ss_aes_xts_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
			SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_XTS);
}
#endif

static int ss_aes_ofb_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_OFB);
}

static int ss_aes_ofb_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_OFB);
}

static int ss_aes_cfb1_encrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 1;
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb1_decrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 1;
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb8_encrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 8;
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb8_decrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 8;
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb64_encrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 64;
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb64_decrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 64;
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb128_encrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 128;
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

static int ss_aes_cfb128_decrypt(struct skcipher_request *req)
{
	ss_aes_req_ctx_t *req_ctx = skcipher_request_ctx(req);

	req_ctx->bitwidth = 128;
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_CFB);
}

#ifdef SS_GCM_MODE_ENABLE
static int sunxi_aes_gcm_encrypt(struct aead_request *req)
{
	return ss_aead_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_AES, SS_AES_MODE_GCM);
}

static int sunxi_aes_gcm_decrypt(struct aead_request *req)
{
	return ss_aead_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_AES, SS_AES_MODE_GCM);
}
#endif  /* SS_GCM_MODE_ENABLE */

static int ss_des_ecb_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_DES, SS_AES_MODE_ECB);
}

static int ss_des_ecb_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_DES, SS_AES_MODE_ECB);
}

static int ss_des_cbc_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_DES, SS_AES_MODE_CBC);
}

static int ss_des_cbc_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_DES, SS_AES_MODE_CBC);
}

static int ss_des3_ecb_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_3DES, SS_AES_MODE_ECB);
}

static int ss_des3_ecb_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_3DES, SS_AES_MODE_ECB);
}

static int ss_des3_cbc_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_ENCRYPT, SS_METHOD_3DES, SS_AES_MODE_CBC);
}

static int ss_des3_cbc_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req,
		SS_DIR_DECRYPT, SS_METHOD_3DES, SS_AES_MODE_CBC);
}

#ifdef SS_RSA_ENABLE
static int ss_rsa_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_ENCRYPT,
		SS_METHOD_RSA, CE_RSA_OP_M_EXP);
}

static int ss_rsa_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_DECRYPT,
		SS_METHOD_RSA, CE_RSA_OP_M_EXP);
}

#ifdef SS_DH_ENABLE
static int ss_dh_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_ENCRYPT,
		SS_METHOD_DH, CE_RSA_OP_M_EXP);
}

static int ss_dh_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_DECRYPT,
		SS_METHOD_DH, CE_RSA_OP_M_EXP);
}
#endif

#ifdef SS_ECC_ENABLE
static int ss_ecdh_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_ENCRYPT, SS_METHOD_ECC,
				CE_ECC_OP_POINT_MUL);
}

static int ss_ecdh_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_DECRYPT, SS_METHOD_ECC,
				CE_ECC_OP_POINT_MUL);
}

static int ss_ecc_sign_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_ENCRYPT, SS_METHOD_ECC, CE_ECC_OP_SIGN);
}

static int ss_ecc_sign_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_DECRYPT, SS_METHOD_ECC, CE_ECC_OP_SIGN);
}

static int ss_ecc_verify_encrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_ENCRYPT, SS_METHOD_RSA,
				CE_RSA_OP_M_MUL);
}

static int ss_ecc_verify_decrypt(struct skcipher_request *req)
{
	return ss_aes_crypt(req, SS_DIR_DECRYPT, SS_METHOD_RSA,
				CE_RSA_OP_M_MUL);
}
#endif
#endif

int ss_rng_reset(struct crypto_rng *tfm, const u8 *seed, u32 slen)
{
	int len = slen > SS_PRNG_SEED_LEN ? SS_PRNG_SEED_LEN : slen;
	ss_aes_ctx_t *ctx = crypto_rng_ctx(tfm);

	SS_DBG("Seed len: %d/%d, flags = %#x\n", len, slen, ctx->comm.flags);
	ctx->key_size = len;
	memset(ctx->key, 0, SS_PRNG_SEED_LEN);
	memcpy(ctx->key, seed, len);
	ctx->comm.flags |= SS_FLAG_NEW_KEY;

	return 0;
}

#ifdef SS_DRBG_MODE_ENABLE
int ss_drbg_reset(struct crypto_rng *tfm, const u8 *seed, u32 slen)
{
	int len = slen > SS_PRNG_SEED_LEN ? SS_PRNG_SEED_LEN : slen;
	ss_drbg_ctx_t *ctx = crypto_rng_ctx(tfm);

	SS_DBG("Seed len: %d/%d, flags = %#x\n", len, slen, ctx->comm.flags);
	ctx->person_size = len;
	memset(ctx->person, 0, SS_PRNG_SEED_LEN);
	memcpy(ctx->person, seed, slen);
	ctx->comm.flags |= SS_FLAG_NEW_KEY;

	return 0;
}

void ss_drbg_set_ent(struct crypto_rng *tfm, const u8 *entropy, u32 entropy_len)
{
	int len = entropy_len > SS_PRNG_SEED_LEN ? SS_PRNG_SEED_LEN : entropy_len;
	ss_drbg_ctx_t *ctx = crypto_rng_ctx(tfm);

	SS_DBG("Seed len: %d / %d, flags = %#x\n", len, entropy_len, ctx->comm.flags);
	ctx->entropt_size = entropy_len;
	memset(ctx->entropt, 0, SS_PRNG_SEED_LEN);
	memcpy(ctx->entropt, entropy, len);
	ctx->comm.flags |= SS_FLAG_NEW_KEY;
}
#endif

int ss_flow_request(ss_comm_ctx_t *comm)
{
	int i;
	unsigned long flags = 0;

	spin_lock_irqsave(&ss_dev->lock, flags);
	for (i = 0; i < SS_FLOW_NUM; i++) {
		if (ss_dev->flows[i].available == SS_FLOW_AVAILABLE) {
			comm->flow = i;
			ss_dev->flows[i].available = SS_FLOW_UNAVAILABLE;
			SS_DBG("The flow %d is available.\n", i);
			break;
		}
	}
	spin_unlock_irqrestore(&ss_dev->lock, flags);

	if (i == SS_FLOW_NUM) {
		SS_ERR("Failed to get an available flow.\n");
		i = -EINVAL;
	}
	return i;
}

void ss_flow_release(ss_comm_ctx_t *comm)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&ss_dev->lock, flags);
	ss_dev->flows[comm->flow].available = SS_FLOW_AVAILABLE;
	spin_unlock_irqrestore(&ss_dev->lock, flags);
}

#ifdef SS_GCM_MODE_ENABLE
static int sunxi_aes_gcm_init(struct crypto_aead *tfm)
{
	if (ss_flow_request(crypto_aead_ctx(tfm)) < 0)
		return -EINVAL;

	crypto_aead_set_reqsize(tfm, sizeof(ss_aes_req_ctx_t));
	SS_DBG("reqsize = %d\n", tfm->reqsize);

	return 0;
}

static void sunxi_aes_gcm_exit(struct crypto_aead *tfm)
{
	SS_ENTER();
	ss_flow_release(crypto_aead_ctx(tfm));
}
#endif  /* SS_GCM_MODE_ENABLE */

static int sunxi_ss_skcipher_init(struct crypto_skcipher *tfm)
{
	if (ss_flow_request(crypto_skcipher_ctx(tfm)) < 0)
		return -EINVAL;

	crypto_skcipher_set_reqsize(tfm, sizeof(ss_aes_req_ctx_t));
	SS_DBG("reqsize = %d\n", tfm->reqsize);

	return 0;
}

static void sunxi_ss_skcipher_exit(struct crypto_skcipher *tfm)
{
	SS_ENTER();
	ss_flow_release(crypto_skcipher_ctx(tfm));
}

static void sunxi_ss_cra_exit(struct crypto_tfm *tfm)
{
	SS_ENTER();
	ss_flow_release(crypto_tfm_ctx(tfm));
}

static int sunxi_ss_cra_rng_init(struct crypto_tfm *tfm)
{
	if (ss_flow_request(crypto_tfm_ctx(tfm)) < 0)
		return -EINVAL;

	return 0;
}

static int sunxi_ss_cra_hash_init(struct crypto_tfm *tfm)
{
	if (ss_flow_request(crypto_tfm_ctx(tfm)) < 0)
		return -EINVAL;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				sizeof(ss_aes_req_ctx_t));

	SS_DBG("reqsize = %zu\n", sizeof(ss_aes_req_ctx_t));
	return 0;
}

static int ss_hash_init(struct ahash_request *req, int type, int size, char *iv)
{
	ss_aes_req_ctx_t *req_ctx = ahash_request_ctx(req);
	ss_hash_ctx_t *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));

	SS_DBG("Method: %d\n", type);

	memset(req_ctx, 0, sizeof(ss_aes_req_ctx_t));
	req_ctx->type = type;

	ctx->md_size = size;
	memcpy(ctx->md, iv, size);

	ctx->cnt = 0;
	ctx->npackets = 0;
	memset(ctx->pad, 0, SS_HASH_PAD_SIZE);
	return 0;
}

static int ss_md5_init(struct ahash_request *req)
{
	int iv[MD5_DIGEST_SIZE/4] = {SHA1_H0, SHA1_H1, SHA1_H2, SHA1_H3};

	return ss_hash_init(req, SS_METHOD_MD5, MD5_DIGEST_SIZE, (char *)iv);
}

static int ss_sha1_init(struct ahash_request *req)
{
	int iv[SHA1_DIGEST_SIZE/4] = {
			SHA1_H0, SHA1_H1, SHA1_H2, SHA1_H3, SHA1_H4};

#ifdef SS_SHA_SWAP_PRE_ENABLE
#ifdef SS_SHA_NO_SWAP_IV4
	ss_hash_swap((char *)iv, SHA1_DIGEST_SIZE - 4);
#else
	ss_hash_swap((char *)iv, SHA1_DIGEST_SIZE);
#endif
#endif

	return ss_hash_init(req, SS_METHOD_SHA1, SHA1_DIGEST_SIZE, (char *)iv);
}

#ifdef SS_SHA224_ENABLE
static int ss_sha224_init(struct ahash_request *req)
{
	int iv[SHA256_DIGEST_SIZE/4] = {
			SHA224_H0, SHA224_H1, SHA224_H2, SHA224_H3,
			SHA224_H4, SHA224_H5, SHA224_H6, SHA224_H7};

#ifdef SS_SHA_SWAP_PRE_ENABLE
	ss_hash_swap((char *)iv, SHA256_DIGEST_SIZE);
#endif

	return ss_hash_init(req, SS_METHOD_SHA224, SHA256_DIGEST_SIZE, (char *)iv);
}
#endif

#ifdef SS_SHA256_ENABLE
static int ss_sha256_init(struct ahash_request *req)
{
	int iv[SHA256_DIGEST_SIZE/4] = {
			SHA256_H0, SHA256_H1, SHA256_H2, SHA256_H3,
			SHA256_H4, SHA256_H5, SHA256_H6, SHA256_H7};

#ifdef SS_SHA_SWAP_PRE_ENABLE
	ss_hash_swap((char *)iv, SHA256_DIGEST_SIZE);
#endif

	return ss_hash_init(req, SS_METHOD_SHA256, SHA256_DIGEST_SIZE, (char *)iv);
}
#endif

#define GET_U64_HIGH(data64)	(int)(data64 >> 32)
#define GET_U64_LOW(data64)		(int)(data64 & 0xFFFFFFFF)

#ifdef SS_SHA384_ENABLE
static int ss_sha384_init(struct ahash_request *req)
{
	int iv[SHA512_DIGEST_SIZE/4] = {
			GET_U64_HIGH(SHA384_H0), GET_U64_LOW(SHA384_H0),
			GET_U64_HIGH(SHA384_H1), GET_U64_LOW(SHA384_H1),
			GET_U64_HIGH(SHA384_H2), GET_U64_LOW(SHA384_H2),
			GET_U64_HIGH(SHA384_H3), GET_U64_LOW(SHA384_H3),
			GET_U64_HIGH(SHA384_H4), GET_U64_LOW(SHA384_H4),
			GET_U64_HIGH(SHA384_H5), GET_U64_LOW(SHA384_H5),
			GET_U64_HIGH(SHA384_H6), GET_U64_LOW(SHA384_H6),
			GET_U64_HIGH(SHA384_H7), GET_U64_LOW(SHA384_H7)};

#ifdef SS_SHA_SWAP_PRE_ENABLE
	ss_hash_swap((char *)iv, SHA512_DIGEST_SIZE);
#endif

	return ss_hash_init(req, SS_METHOD_SHA384, SHA512_DIGEST_SIZE, (char *)iv);
}
#endif

#ifdef SS_SHA512_ENABLE
static int ss_sha512_init(struct ahash_request *req)
{
	int iv[SHA512_DIGEST_SIZE/4] = {
			GET_U64_HIGH(SHA512_H0), GET_U64_LOW(SHA512_H0),
			GET_U64_HIGH(SHA512_H1), GET_U64_LOW(SHA512_H1),
			GET_U64_HIGH(SHA512_H2), GET_U64_LOW(SHA512_H2),
			GET_U64_HIGH(SHA512_H3), GET_U64_LOW(SHA512_H3),
			GET_U64_HIGH(SHA512_H4), GET_U64_LOW(SHA512_H4),
			GET_U64_HIGH(SHA512_H5), GET_U64_LOW(SHA512_H5),
			GET_U64_HIGH(SHA512_H6), GET_U64_LOW(SHA512_H6),
			GET_U64_HIGH(SHA512_H7), GET_U64_LOW(SHA512_H7)};

#ifdef SS_SHA_SWAP_PRE_ENABLE
	ss_hash_swap((char *)iv, SHA512_DIGEST_SIZE);
#endif

	return ss_hash_init(req, SS_METHOD_SHA512, SHA512_DIGEST_SIZE, (char *)iv);
}
#endif

#ifdef SS_HMAC_ENABLE
static int sunxi_ss_hmac_sha1_init(struct ahash_request *req)
{
	ss_aes_req_ctx_t *req_ctx = ahash_request_ctx(req);
	int ret;

	ret = ss_sha1_init(req);

	/* req_ctx->type has already set in function ss_sha1_init(), here set it again to HMAC mode */
	req_ctx->type = SS_METHOD_HMAC_SHA1;
	return ret;
}

static int sunxi_ss_hmac_sha256_init(struct ahash_request *req)
{
	ss_aes_req_ctx_t *req_ctx = ahash_request_ctx(req);
	int ret;

	ret = ss_sha256_init(req);

	/* req_ctx->type has already set in function ss_sha256_init(), here set it again to HMAC mode */
	req_ctx->type = SS_METHOD_HMAC_SHA256;
	return ret;
}

static int sunxi_ss_hmac_hash_setkey(struct crypto_ahash *tfm, const u8 *key, unsigned int keylen)
{
	int ret = 0;
	ss_hash_ctx_t *ctx = crypto_ahash_ctx(tfm);

	SS_DBG("keylen = %d\n", keylen);
	if (ctx->comm.flags & SS_FLAG_NEW_KEY) {
		SS_ERR("The key has already update.\n");
		return -EBUSY;
	}

	ret = ss_aes_key_valid(crypto_ahash_tfm(tfm), keylen);
	if (ret != 0)
		return ret;

	if (keylen > sizeof(ctx->key))
		return -ENOMEM;

	memcpy(ctx->key, key, keylen);
	ctx->key_size = keylen;
	ctx->comm.flags |= SS_FLAG_NEW_KEY;

	return 0;
}

static int sunxi_ss_hmac_hash_export(struct ahash_request *req, void *out)
{
	ss_aes_req_ctx_t *req_ctx = ahash_request_ctx(req);

	memcpy(out, req_ctx, sizeof(*req_ctx));
	return 0;
}

static int sunxi_ss_hmac_hash_import(struct ahash_request *req, const void *in)
{
	ss_aes_req_ctx_t *req_ctx = ahash_request_ctx(req);

	memcpy(req_ctx, in, sizeof(*req_ctx));
	return 0;
}

#endif

#define DES_MIN_KEY_SIZE	DES_KEY_SIZE
#define DES_MAX_KEY_SIZE	DES_KEY_SIZE
#define DES3_MIN_KEY_SIZE	DES3_EDE_KEY_SIZE
#define DES3_MAX_KEY_SIZE	DES3_EDE_KEY_SIZE

#define DECLARE_SS_AES_ALG(utype, ltype, lmode, block_size, iv_size) \
{ \
	.type = ALG_TYPE_CIPHER, \
	.alg.skcipher = { \
		.base.cra_name	      = #lmode"("#ltype")", \
		.base.cra_driver_name = "ss-"#lmode"-"#ltype, \
		.base.cra_flags	      = CRYPTO_ALG_ASYNC, \
		.base.cra_blocksize   = block_size, \
		.base.cra_alignmask   = 3, \
		.setkey      = ss_aes_setkey, \
		.encrypt     = ss_##ltype##_##lmode##_encrypt, \
		.decrypt     = ss_##ltype##_##lmode##_decrypt, \
		.min_keysize = utype##_MIN_KEY_SIZE, \
		.max_keysize = utype##_MAX_KEY_SIZE, \
		.ivsize	     = iv_size, \
	} \
}

#ifdef SS_XTS_MODE_ENABLE
#define DECLARE_SS_AES_XTS_ALG(utype, ltype, lmode, block_size, iv_size) \
{ \
	.type = ALG_TYPE_CIPHER, \
	.alg.skcipher = { \
		.base.cra_name	      = #lmode"("#ltype")", \
		.base.cra_driver_name = "ss-"#lmode"-"#ltype, \
		.base.cra_flags	      = CRYPTO_ALG_ASYNC, \
		.base.cra_blocksize   = block_size, \
		.base.cra_alignmask   = 3, \
		.setkey      = ss_aes_setkey, \
		.encrypt     = ss_##ltype##_##lmode##_encrypt, \
		.decrypt     = ss_##ltype##_##lmode##_decrypt, \
		.min_keysize = utype##_MAX_KEY_SIZE, \
		.max_keysize = utype##_MAX_KEY_SIZE * 2, \
		.ivsize	     = iv_size, \
	} \
}
#endif

#ifdef SS_RSA_ENABLE
#define DECLARE_SS_ASYM_ALG(type, bitwidth, key_size, iv_size) \
{ \
	.alg.asym = { \
		.set_pub_key  = ss_asym_setpubkey, \
		.set_priv_key = ss_asym_setprivkey, \
		.encrypt      = ss_##type##_encrypt, \
		.decrypt      = ss_##type##_decrypt, \
		.max_keysize  = key_size, \
		.ivsize       = iv_size, \
		.reqsize      = 64, \
		.base.cra_name	 = #type"("#bitwidth")", \
		.base.cra_driver_name = "ss-"#type"-"#bitwidth, \
		.base.cra_blocksize	 = key_size%AES_BLOCK_SIZE == 0 ? AES_BLOCK_SIZE : 4, \
		.base.cra_alignmask	 = key_size%AES_BLOCK_SIZE == 0 ? 31 : 3, \
	}, \
}

#ifndef SS_SUPPORT_CE_V3_2
#define DECLARE_SS_RSA_ALG(type, bitwidth) \
		DECLARE_SS_ASYM_ALG(type, bitwidth, (bitwidth/8), (bitwidth/8))
#else
#define DECLARE_SS_RSA_ALG(type, bitwidth) \
		DECLARE_SS_ASYM_ALG(type, bitwidth, (bitwidth/8), 0)
#endif
#define DECLARE_SS_DH_ALG(type, bitwidth) DECLARE_SS_RSA_ALG(type, bitwidth)
#endif

#ifdef SS_GCM_MODE_ENABLE
static struct aead_alg sunxi_aes_gcm_alg = {
	.setkey		= sunxi_aes_gcm_setkey,
	.setauthsize	= sunxi_aes_gcm_setauthsize,
	.encrypt	= sunxi_aes_gcm_encrypt,
	.decrypt	= sunxi_aes_gcm_decrypt,
	.init		= sunxi_aes_gcm_init,
	.exit		= sunxi_aes_gcm_exit,
	.ivsize		= AES_MIN_KEY_SIZE,
	.maxauthsize	= AES_BLOCK_SIZE,

	.base = {
		.cra_name		= "gcm(aes)",
		.cra_driver_name	= "ss-gcm-aes",
		.cra_priority		= SS_ALG_PRIORITY,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(ss_aead_ctx_t),
		.cra_alignmask		= 31,
		.cra_module		= THIS_MODULE,
	},
};
#endif  /* SS_GCM_MODE_ENABLE */

static struct sunxi_crypto_tmp sunxi_ss_algs[] = {
	DECLARE_SS_AES_ALG(AES, aes, ecb, AES_BLOCK_SIZE, 0),
	DECLARE_SS_AES_ALG(AES, aes, cbc, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(AES, aes, ctr, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(AES, aes, cts, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
#ifdef SS_XTS_MODE_ENABLE
	DECLARE_SS_AES_XTS_ALG(AES, aes, xts, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
#endif
	DECLARE_SS_AES_ALG(AES, aes, ofb, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(AES, aes, cfb1, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(AES, aes, cfb8, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(AES, aes, cfb64, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(AES, aes, cfb128, AES_BLOCK_SIZE, AES_MIN_KEY_SIZE),
	DECLARE_SS_AES_ALG(DES, des, ecb, DES_BLOCK_SIZE, 0),
	DECLARE_SS_AES_ALG(DES, des, cbc, DES_BLOCK_SIZE, DES_KEY_SIZE),
	DECLARE_SS_AES_ALG(DES3, des3, ecb, DES3_EDE_BLOCK_SIZE, 0),
	DECLARE_SS_AES_ALG(DES3, des3, cbc, DES3_EDE_BLOCK_SIZE, DES_KEY_SIZE),
};

#ifdef SS_RSA_ENABLE
static struct akcipher_alg sunxi_ss_algs_asym[] = {
#ifdef SS_RSA512_ENABLE
	DECLARE_SS_RSA_ALG(rsa, 512),
#endif
#ifdef SS_RSA1024_ENABLE
	DECLARE_SS_RSA_ALG(rsa, 1024),
#endif
#ifdef SS_RSA2048_ENABLE
	DECLARE_SS_RSA_ALG(rsa, 2048),
#endif
#ifdef SS_RSA3072_ENABLE
	DECLARE_SS_RSA_ALG(rsa, 3072),
#endif
#ifdef SS_RSA4096_ENABLE
	DECLARE_SS_RSA_ALG(rsa, 4096),
#endif
#ifdef SS_DH512_ENABLE
	DECLARE_SS_DH_ALG(dh, 512),
#endif
#ifdef SS_DH1024_ENABLE
	DECLARE_SS_DH_ALG(dh, 1024),
#endif
#ifdef SS_DH2048_ENABLE
	DECLARE_SS_DH_ALG(dh, 2048),
#endif
#ifdef SS_DH3072_ENABLE
	DECLARE_SS_DH_ALG(dh, 3072),
#endif
#ifdef SS_DH4096_ENABLE
	DECLARE_SS_DH_ALG(dh, 4096),
#endif

#ifdef SS_ECC_ENABLE
#ifndef SS_SUPPORT_CE_V3_2
	DECLARE_SS_ASYM_ALG(ecdh, 160, 160/8, 160/8),
	DECLARE_SS_ASYM_ALG(ecdh, 224, 224/8, 224/8),
	DECLARE_SS_ASYM_ALG(ecdh, 256, 256/8, 256/8),
	DECLARE_SS_ASYM_ALG(ecdh, 521, ((521+31)/32)*4, ((521+31)/32)*4),
	DECLARE_SS_ASYM_ALG(ecc_sign, 160, 160/8, (160/8)*2),
	DECLARE_SS_ASYM_ALG(ecc_sign, 224, 224/8, (224/8)*2),
	DECLARE_SS_ASYM_ALG(ecc_sign, 256, 256/8, (256/8)*2),
	DECLARE_SS_ASYM_ALG(ecc_sign, 521, ((521+31)/32)*4, ((521+31)/32)*4*2),
#else
	DECLARE_SS_ASYM_ALG(ecdh, 160, 160/8, 0),
	DECLARE_SS_ASYM_ALG(ecdh, 224, 224/8, 0),
	DECLARE_SS_ASYM_ALG(ecdh, 256, 256/8, 0),
	DECLARE_SS_ASYM_ALG(ecdh, 521, ((521+31)/32)*4, 0),
	DECLARE_SS_ASYM_ALG(ecc_sign, 160, 160/8, 0),
	DECLARE_SS_ASYM_ALG(ecc_sign, 224, 224/8, 0),
	DECLARE_SS_ASYM_ALG(ecc_sign, 256, 256/8, 0),
	DECLARE_SS_ASYM_ALG(ecc_sign, 521, ((521+31)/32)*4, 0),
#endif
	DECLARE_SS_RSA_ALG(ecc_verify, 512),
	DECLARE_SS_RSA_ALG(ecc_verify, 1024),
#endif
};
#endif

#ifdef SS_HMAC_ENABLE
static struct sunxi_crypto_tmp sunxi_ss_algs_hmac_hash[] = {
{
	.type					= ALG_TYPE_HASH,
	.alg.hash.init				= sunxi_ss_hmac_sha1_init,
	.alg.hash.halg.digestsize		= SHA1_DIGEST_SIZE,
	.alg.hash.halg.statesize		= sizeof(struct sha1_state),
	.alg.hash.halg.base.cra_driver_name	= "ss-hmac-sha1",
	.alg.hash.halg.base.cra_name		= "hmac(sha1)",
	.alg.hash.halg.base.cra_driver_name	= "ss-hmac-sha1",
	.alg.hash.halg.base.cra_blocksize	= SHA1_BLOCK_SIZE,
},
{
	.type					= ALG_TYPE_HASH,
	.alg.hash.init				= sunxi_ss_hmac_sha256_init,
	.alg.hash.halg.digestsize		= SHA256_DIGEST_SIZE,
	.alg.hash.halg.statesize		= sizeof(struct sha256_state),
	.alg.hash.halg.base.cra_driver_name	= "ss-hmac-sha256",
	.alg.hash.halg.base.cra_name		= "hmac(sha256)",
	.alg.hash.halg.base.cra_driver_name	= "ss-hmac-sha256",
	.alg.hash.halg.base.cra_blocksize	= SHA256_BLOCK_SIZE,
},
};

static void sunxi_ss_hmac_ahash_alg_init(struct ahash_alg *alg)
{
	alg->update = ss_hash_update;
	alg->final  = ss_hash_final;
	alg->finup  = ss_hash_finup;
	alg->digest = ss_hash_digest;
	alg->setkey = sunxi_ss_hmac_hash_setkey;
	alg->export = sunxi_ss_hmac_hash_export;
	alg->import = sunxi_ss_hmac_hash_import;

	alg->halg.base.cra_priority = SS_ALG_PRIORITY;
	alg->halg.base.cra_flags = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC;
	alg->halg.base.cra_ctxsize = sizeof(ss_hash_ctx_t);
	alg->halg.base.cra_module = THIS_MODULE;
	alg->halg.base.cra_init	= sunxi_ss_cra_hash_init;
	alg->halg.base.cra_exit	= sunxi_ss_cra_exit;
}
#endif

#define DECLARE_SS_RNG_ALG(ltype) \
{ \
	.type = ALG_TYPE_RNG, \
	.alg.rng = { \
		.generate = ss_##ltype##_get_random, \
		.seed	  = ss_rng_reset, \
		.seedsize = SS_SEED_SIZE, \
		.base	  = { \
			.cra_name = #ltype, \
			.cra_driver_name = "ss-"#ltype, \
			.cra_flags = CRYPTO_ALG_TYPE_RNG, \
			.cra_priority = SS_ALG_PRIORITY, \
			.cra_ctxsize = sizeof(ss_aes_ctx_t), \
			.cra_module = THIS_MODULE, \
			.cra_init = sunxi_ss_cra_rng_init, \
			.cra_exit = sunxi_ss_cra_exit, \
		} \
	} \
}

#ifdef SS_DRBG_MODE_ENABLE
#define DECLARE_SS_DRBG_ALG(ltype) \
{ \
	.type = ALG_TYPE_RNG, \
	.alg.rng = { \
		.generate = ss_drbg_##ltype##_get_random, \
		.seed	  = ss_drbg_reset, \
		.set_ent  = ss_drbg_set_ent, \
		.seedsize = SS_SEED_SIZE, \
		.base	  = { \
			.cra_name = "drbg-"#ltype, \
			.cra_driver_name = "ss-drbg-"#ltype, \
			.cra_flags = CRYPTO_ALG_TYPE_RNG, \
			.cra_priority = SS_ALG_PRIORITY, \
			.cra_ctxsize = sizeof(ss_drbg_ctx_t), \
			.cra_module = THIS_MODULE, \
			.cra_init = sunxi_ss_cra_rng_init, \
			.cra_exit = sunxi_ss_cra_exit, \
		} \
	} \
}
#endif

static struct sunxi_crypto_tmp sunxi_ss_algs_rng[] = {
#ifdef SS_TRNG_ENABLE
	DECLARE_SS_RNG_ALG(trng),
#endif
	DECLARE_SS_RNG_ALG(prng),
#ifdef SS_DRBG_MODE_ENABLE
	DECLARE_SS_DRBG_ALG(sha1),
	DECLARE_SS_DRBG_ALG(sha256),
	DECLARE_SS_DRBG_ALG(sha512),
#endif
};

#define MD5_BLOCK_SIZE	MD5_HMAC_BLOCK_SIZE
#define sha224_state   sha256_state
#define sha384_state   sha512_state
#define DECLARE_SS_AHASH_ALG(ltype, utype) \
	{ \
		.type = ALG_TYPE_HASH, \
		.alg.hash = { \
			.init		= ss_##ltype##_init, \
			.update		= ss_hash_update, \
			.final		= ss_hash_final, \
			.finup		= ss_hash_finup, \
			.digest		= ss_hash_digest, \
			.halg = { \
				.digestsize = utype##_DIGEST_SIZE, \
				.statesize = sizeof(struct ltype##_state), \
				.base	= { \
					.cra_name        = #ltype, \
					.cra_driver_name = "ss-"#ltype, \
					.cra_priority = SS_ALG_PRIORITY, \
					.cra_flags  = CRYPTO_ALG_TYPE_AHASH|CRYPTO_ALG_ASYNC, \
					.cra_blocksize	 = utype##_BLOCK_SIZE, \
					.cra_ctxsize	 = sizeof(ss_hash_ctx_t), \
					.cra_alignmask	 = 3, \
					.cra_module	 = THIS_MODULE, \
					.cra_init	 = sunxi_ss_cra_hash_init, \
					.cra_exit	 = sunxi_ss_cra_exit, \
				} \
			} \
		} \
	}

static struct sunxi_crypto_tmp sunxi_ss_algs_hash[] = {
	DECLARE_SS_AHASH_ALG(md5, MD5),
	DECLARE_SS_AHASH_ALG(sha1, SHA1),
#ifdef SS_SHA224_ENABLE
	DECLARE_SS_AHASH_ALG(sha224, SHA224),
#endif
#ifdef SS_SHA256_ENABLE
	DECLARE_SS_AHASH_ALG(sha256, SHA256),
#endif
#ifdef SS_SHA384_ENABLE
	DECLARE_SS_AHASH_ALG(sha384, SHA384),
#endif
#ifdef SS_SHA512_ENABLE
	DECLARE_SS_AHASH_ALG(sha512, SHA512),
#endif
};

/* Requeset the resource: IRQ, mem */
static int sunxi_ss_res_request(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *pnode = pdev->dev.of_node;
	sunxi_ce_cdev_t *sss = platform_get_drvdata(pdev);

	sss->irq = irq_of_parse_and_map(pnode, SS_RES_INDEX);
	if (sss->irq == 0) {
		SS_ERR("Failed to get the SS IRQ.\n");
		return -EINVAL;
	}

	ret = request_irq(sss->irq,
			sunxi_ss_irq_handler, 0, sss->dev_name, sss);
	if (ret != 0) {
		SS_ERR("Cannot request IRQ\n");
		return ret;
	}

#ifdef CONFIG_OF
	sss->base_addr = of_iomap(pnode, SS_RES_INDEX);
	if (sss->base_addr == NULL) {
		SS_ERR("Unable to remap IO\n");
		return -ENXIO;
	}
#endif

	return 0;
}

/* Release the resource: IRQ, mem */
static int sunxi_ss_res_release(sunxi_ce_cdev_t *sss)
{
	iounmap(sss->base_addr);

	free_irq(sss->irq, sss);
	return 0;
}

static int sunxi_get_ce_clk(sunxi_ce_cdev_t *sss)
{
	struct platform_device *pdev = sss->pdev;

	if (sss->suspend == 1) {
		return 0;
	}

	sss->pclk = devm_clk_get(&pdev->dev, "clk_src");
	if (IS_ERR(sss->pclk)) {
		SS_ERR("Fail to get src clk\n");
		return PTR_ERR(sss->pclk);
	}
	sss->ce_clk = devm_clk_get(&pdev->dev, "ce_clk");
	if (IS_ERR(sss->ce_clk)) {
		SS_ERR("Fail to get module clk\n");
		return PTR_ERR(sss->ce_clk);
	}

	sss->bus_clk = devm_clk_get(&pdev->dev, "bus_ce");
	if (IS_ERR(sss->bus_clk)) {
		SS_ERR("Fail to get bus_ce clk\n");
		return PTR_ERR(sss->bus_clk);
	}

	sss->mbus_clk = devm_clk_get(&pdev->dev, "mbus_ce");
	if (IS_ERR(sss->mbus_clk)) {
		SS_ERR("Fail to get mbus clk\n");
		return PTR_ERR(sss->mbus_clk);
	}

	sss->ce_sys_clk = devm_clk_get(&pdev->dev, "ce_sys_clk");
	if (IS_ERR(sss->ce_sys_clk))
		SS_ERR("Fail to get ce_sys clk\n");

	sss->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(sss->reset)) {
		SS_ERR("Fail to get reset clk\n");
		return PTR_ERR(sss->reset);
	}

	return 0;
}

static int sunxi_ss_hw_init(sunxi_ce_cdev_t *sss)
{
	struct device_node *pnode = sss->pdev->dev.of_node;

	if (sunxi_get_ce_clk(sss) != 0) {
		return -EINVAL;
	}

	/* deassert ce reset */
	if (reset_control_deassert(sss->reset)) {
		SS_ERR("Couldn't deassert reset\n");
		return -EBUSY;
	}
	/* enable ce gating */
	if (clk_prepare_enable(sss->bus_clk)) {
		SS_ERR("Couldn't enable bus gating\n");
		return -EBUSY;
	}

#ifdef SS_RSA_CLK_ENABLE
	if (of_property_read_u32_array(pnode, "clock-frequency",
		&sss->gen_clkrate, 2)) {
#else
	if (of_property_read_u32(pnode, "clock-frequency", &sss->gen_clkrate)) {
#endif
		SS_ERR("Unable to get clock-frequency.\n");
		return -EINVAL;
	}
	SS_DBG("The clk freq: %d, %d\n", sss->gen_clkrate, sss->rsa_clkrate);

	SS_DBG("SS ce_clk%luMHz, pclk %luMHz\n", clk_get_rate(sss->ce_clk)/1000000,
			clk_get_rate(sss->pclk)/1000000);

	/* enable ce clock */
	if (clk_prepare_enable(sss->ce_clk)) {
		SS_ERR("Couldn't enable module clock\n");
		return -EBUSY;
	}

	/* enable ce mbus_clock */
	if (clk_prepare_enable(sss->mbus_clk)) {
		SS_ERR("Couldn't enable ce mbus clock\n");
		return -EBUSY;
	}

	/* enable ce sys clk */
	if (!IS_ERR_OR_NULL(sss->ce_sys_clk)) {
		if (clk_prepare_enable(sss->ce_sys_clk))
			SS_ERR("Couldn't enable ce sys clk\n");
	}

	return 0;
}

static int sunxi_ss_hw_exit(sunxi_ce_cdev_t *sss)
{
	if (!IS_ERR_OR_NULL(sss->ce_sys_clk))
		clk_disable_unprepare(sss->ce_sys_clk);
	if (!IS_ERR_OR_NULL(sss->mbus_clk))
		clk_disable_unprepare(sss->mbus_clk);
	if (!IS_ERR_OR_NULL(sss->ce_clk))
		clk_disable_unprepare(sss->ce_clk);
	if (!IS_ERR_OR_NULL(sss->bus_clk))
		clk_disable_unprepare(sss->bus_clk);

	if (!IS_ERR_OR_NULL(sss->reset))
		reset_control_assert(sss->reset);

	return 0;
}

static int sunxi_ss_alg_register(void)
{
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs); i++) {
		INIT_LIST_HEAD(&sunxi_ss_algs[i].alg.skcipher.base.cra_list);
		SS_DBG("Add %s...\n", sunxi_ss_algs[i].alg.skcipher.base.cra_name);
		sunxi_ss_algs[i].alg.skcipher.base.cra_priority = SS_ALG_PRIORITY;
		sunxi_ss_algs[i].alg.skcipher.base.cra_ctxsize = sizeof(ss_aes_ctx_t);
		sunxi_ss_algs[i].alg.skcipher.base.cra_module = THIS_MODULE;
		sunxi_ss_algs[i].alg.skcipher.exit = sunxi_ss_skcipher_exit;
		sunxi_ss_algs[i].alg.skcipher.init = sunxi_ss_skcipher_init;

		ret = crypto_register_skcipher(&sunxi_ss_algs[i].alg.skcipher);
		if (ret != 0) {
			SS_ERR("crypto_register_alg(%s) failed! return %d\n",
				sunxi_ss_algs[i].alg.skcipher.base.cra_name, ret);
			return ret;
		}
	}

#ifdef SS_HMAC_ENABLE
	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs_hmac_hash); i++) {
		SS_DBG("Add %s...\n", sunxi_ss_algs_hmac_hash[i].alg.hash.halg.base.cra_name);
		sunxi_ss_hmac_ahash_alg_init(&sunxi_ss_algs_hmac_hash[i].alg.hash);
		ret = crypto_register_ahash(&sunxi_ss_algs_hmac_hash[i].alg.hash);
		if (ret) {
			SS_ERR("crypto_register_alg(%s) failed! return %d\n",
					sunxi_ss_algs_hmac_hash[i].alg.hash.halg.base.cra_name, ret);
			return ret;
		}
	}
#endif

	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs_rng); i++) {
		SS_DBG("Add %s...\n", sunxi_ss_algs_rng[i].alg.rng.base.cra_name);
		ret = crypto_register_rng(&sunxi_ss_algs_rng[i].alg.rng);
		if (ret != 0) {
			SS_ERR("crypto_register_rng(%s) failed! return %d\n",
				sunxi_ss_algs_rng[i].alg.rng.base.cra_name, ret);
			return ret;
		}
	}

#if IS_ENABLED(CONFIG_AW_HWRNG_DRIVER)
	ret = sunxi_register_hwrng(ss_dev);
	if (ret < 0) {
		SS_ERR("sunxi_register_hwrng failed, return %d\n", ret);
		return ret;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs_hash); i++) {
		SS_DBG("Add %s...\n", sunxi_ss_algs_hash[i].alg.hash.halg.base.cra_name);
		ret = crypto_register_ahash(&sunxi_ss_algs_hash[i].alg.hash);
		if (ret != 0) {
			SS_ERR("crypto_register_ahash(%s) failed! return %d\n",
				sunxi_ss_algs_hash[i].alg.hash.halg.base.cra_name, ret);
			return ret;
		}
	}

#ifdef SS_GCM_MODE_ENABLE
	ret = crypto_register_aead(&sunxi_aes_gcm_alg);
	if (ret != 0) {
		SS_ERR("crypto_register_aead(%s) failed! return %d\n",
			sunxi_aes_gcm_alg.base.cra_name, ret);
		return ret;
	}
#endif  /* SS_GCM_MODE_ENABLE */

	return 0;
}

static void sunxi_ss_alg_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs); i++)
		crypto_unregister_skcipher(&sunxi_ss_algs[i].alg.skcipher);

#ifdef SS_HMAC_ENABLE
	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs_hmac_hash); i++)
		crypto_unregister_ahash(&sunxi_ss_algs_hmac_hash[i].alg.hash);
#endif

	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs_rng); i++)
		crypto_unregister_rng(&sunxi_ss_algs_rng[i].alg.rng);

	for (i = 0; i < ARRAY_SIZE(sunxi_ss_algs_hash); i++)
		crypto_unregister_ahash(&sunxi_ss_algs_hash[i].alg.hash);
}

static ssize_t sunxi_ss_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	sunxi_ce_cdev_t *sss = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE,
		"pdev->id   = %d\n"
		"pdev->name = %s\n"
		"pdev->num_resources = %u\n"
		"pdev->resource.irq = %d\n"
		"SS module clk rate = %ld Mhz\n"
		"IO membase = 0x%px\n",
		pdev->id, pdev->name, pdev->num_resources,
		sss->irq,
		(clk_get_rate(sss->ce_clk)/1000000), sss->base_addr);
}
static struct device_attribute sunxi_ss_info_attr =
	__ATTR(info, S_IRUGO, sunxi_ss_info_show, NULL);

static ssize_t sunxi_ss_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	struct platform_device *pdev =
			container_of(dev, struct platform_device, dev);
	sunxi_ce_cdev_t *sss = platform_get_drvdata(pdev);
	static char const *avail[] = {"Available", "Unavailable"};

	if (sss == NULL)
		return scnprintf(buf, PAGE_SIZE, "%s\n", "sunxi_ss is NULL!");

	buf[0] = 0;
	for (i = 0; i < SS_FLOW_NUM; i++) {
		snprintf(buf+strlen(buf), PAGE_SIZE-strlen(buf),
			"The flow %d state: %s\n"
			, i, avail[sss->flows[i].available]
		);
	}

	return strlen(buf)
		+ ss_reg_print(buf + strlen(buf), PAGE_SIZE - strlen(buf));
}

static struct device_attribute sunxi_ss_status_attr =
	__ATTR(status, S_IRUGO, sunxi_ss_status_show, NULL);

static void sunxi_ss_sysfs_create(struct platform_device *_pdev)
{
	device_create_file(&_pdev->dev, &sunxi_ss_info_attr);
	device_create_file(&_pdev->dev, &sunxi_ss_status_attr);
}

static void sunxi_ss_sysfs_remove(struct platform_device *_pdev)
{
	device_remove_file(&_pdev->dev, &sunxi_ss_info_attr);
	device_remove_file(&_pdev->dev, &sunxi_ss_status_attr);
}

static u64 sunxi_ss_dma_mask = DMA_BIT_MASK(64);

static int sunxi_ss_probe(struct platform_device *pdev)
{
	int ret = 0;
	sunxi_ce_cdev_t *sss = NULL;

	sss = devm_kzalloc(&pdev->dev, sizeof(sunxi_ce_cdev_t), GFP_KERNEL);
	if (sss == NULL) {
		SS_ERR("Unable to allocate sunxi_ce_cdev_t\n");
		return -ENOMEM;
	}

#ifdef TASK_DMA_POOL
	sss->task_pool = dma_pool_create("task_pool", &pdev->dev,
			sizeof(struct ce_task_desc), 4, 0);
	if (sss->task_pool == NULL)
		return -ENOMEM;
#endif

#ifdef CONFIG_OF
	pdev->dev.dma_mask = &sunxi_ss_dma_mask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
#endif

	sss->pdevice = &pdev->dev;
	snprintf(sss->dev_name, sizeof(sss->dev_name), SUNXI_SS_DEV_NAME);
	platform_set_drvdata(pdev, sss);

	ret = sunxi_ss_res_request(pdev);
	if (ret != 0)
		goto err0;

	sss->pdev = pdev;

	ret = sunxi_ss_hw_init(sss);
	if (ret != 0) {
		SS_ERR("SS hw init failed!\n");
		goto err1;
	}

	ss_dev = sss;
	ret = sunxi_ss_alg_register();
	if (ret != 0) {
		SS_ERR("sunxi_ss_alg_register() failed! return %d\n", ret);
		goto err2;
	}

	sunxi_ss_sysfs_create(pdev);

	SS_DBG("SS is inited, base 0x%px, irq %d!\n", sss->base_addr, sss->irq);
	return 0;

err2:
	sunxi_ss_hw_exit(sss);
err1:
	sunxi_ss_res_release(sss);
err0:
	platform_set_drvdata(pdev, NULL);
#ifdef SS_SCATTER_ENABLE
	if (sss->task_pool)
		dma_pool_destroy(sss->task_pool);
#endif
	return ret;
}

static int sunxi_ss_remove(struct platform_device *pdev)
{
	sunxi_ce_cdev_t *sss = platform_get_drvdata(pdev);

	ss_wait_idle();
	sunxi_ss_sysfs_remove(pdev);

	sunxi_ss_alg_unregister();
	sunxi_ss_hw_exit(sss);
	sunxi_ss_res_release(sss);

#ifdef SS_SCATTER_ENABLE
	if (sss->task_pool)
		dma_pool_destroy(sss->task_pool);
#endif
	platform_set_drvdata(pdev, NULL);
	ss_dev = NULL;
	return 0;
}

#ifdef CONFIG_PM
static int sunxi_ss_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	sunxi_ce_cdev_t *sss = platform_get_drvdata(pdev);
	unsigned long flags = 0;

	SS_ENTER();

	/* Wait for the completion of SS operation. */
	ss_dev_lock();

	spin_lock_irqsave(&ss_dev->lock, flags);
	sss->suspend = 1;
	spin_unlock_irqrestore(&sss->lock, flags);

	sunxi_ss_hw_exit(sss);
	ss_dev_unlock();

	return 0;
}

static int sunxi_ss_resume(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	sunxi_ce_cdev_t *sss = platform_get_drvdata(pdev);
	unsigned long flags = 0;

	SS_ENTER();
	ret = sunxi_ss_hw_init(sss);
	spin_lock_irqsave(&ss_dev->lock, flags);
	sss->suspend = 0;
	spin_unlock_irqrestore(&sss->lock, flags);

	return ret;
}

static const struct dev_pm_ops sunxi_ss_dev_pm_ops = {
	.suspend = sunxi_ss_suspend,
	.resume  = sunxi_ss_resume,
};

#define SUNXI_SS_DEV_PM_OPS (&sunxi_ss_dev_pm_ops)
#else
#define SUNXI_SS_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

static struct platform_driver sunxi_ss_driver = {
	.probe   = sunxi_ss_probe,
	.remove  = sunxi_ss_remove,
	.driver = {
	.name	= SUNXI_SS_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm		= SUNXI_SS_DEV_PM_OPS,
		.of_match_table = sunxi_ss_of_match,
	},
};

static int __init sunxi_ss_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&sunxi_ss_driver);
	if (ret < 0) {
		SS_ERR("platform_driver_register() failed, return %d\n", ret);
		return ret;
	}

	return ret;
}

static void __exit sunxi_ss_exit(void)
{
	platform_driver_unregister(&sunxi_ss_driver);
}

module_init(sunxi_ss_init);
module_exit(sunxi_ss_exit);

MODULE_AUTHOR("mintow");
MODULE_VERSION("1.0.7");
MODULE_DESCRIPTION("SUNXI SS Controller Driver");
MODULE_ALIAS("platform:"SUNXI_SS_DEV_NAME);
MODULE_LICENSE("GPL");
