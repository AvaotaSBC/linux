// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

//#define pr_fmt(fmt) "sunxi-spinand-phy: " fmt
#define SUNXI_MODNAME "sunxi-spinand-phy"
#include <sunxi-log.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/aw-spinand.h>
#include <linux/types.h>
#include <linux/delay.h>

#include "../sunxi-spinand.h"
#include "physic.h"

static int aw_spinand_chip_wait(struct aw_spinand_chip *chip, u8 *status)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	struct aw_spinand_chip_ops *ops = chip->ops;
	u8 s = 0;
	int ret;

	do {
		ret = ops->read_status(chip, &s);
		if (ret)
			return ret;

		if (!(s & STATUS_BUSY))
			goto out;
	} while (time_before(jiffies, timeout));

	/*
	 * Extra read, just in case the STATUS_READY bit has changed
	 * since our last check
	 */
	ret = ops->read_status(chip, &s);
	if (ret)
		return ret;

out:
	if (status)
		*status = s;
	return s & STATUS_BUSY ? -ETIMEDOUT : 0;
}

static int aw_spinand_chip_reset(struct aw_spinand_chip *chip)
{
	int ret;
	unsigned char txbuf[1];

	txbuf[0] = SPI_NAND_RESET;

	ret = spi_write(chip->spi, txbuf, 1);
	if (ret)
		return ret;

	return aw_spinand_chip_wait(chip, NULL);
}

static int aw_spinand_chip_read_id(struct aw_spinand_chip *chip, void *id,
		int len, int dummy)
{
	int ret;
	unsigned char txbuf[2] = {SPI_NAND_RDID, 0x00};

	ret = aw_spinand_chip_reset(chip);
	if (ret)
		return ret;

	/* some spinand readid conmand should follow one byte dummy */
	if (dummy)
		return spi_write_then_read(chip->spi, txbuf, 2, id, len);
	else
		return spi_write_then_read(chip->spi, txbuf, 1, id, len);
}

static int aw_spinand_chip_read_reg(struct aw_spinand_chip *chip, u8 cmd,
		u8 reg, u8 *val)
{
	unsigned char txbuf[2];

	txbuf[0] = cmd;
	txbuf[1] = reg;

	return spi_write_then_read(chip->spi, txbuf, 2, val, 1);
}

static int aw_spinand_chip_read_reg_atomic(struct aw_spinand_chip *chip, u8 cmd,
		u8 reg, u8 *val)
{
	unsigned char txbuf[2];

	txbuf[0] = cmd;
	txbuf[1] = reg;

	return aw_spi_write_then_read_atomic(chip->spi, txbuf, 2, val, 1);
}

static int aw_spinand_chip_write_reg(struct aw_spinand_chip *chip, u8 cmd,
		u8 reg, u8 val)
{
	unsigned char txbuf[3];

	txbuf[0] = cmd;
	txbuf[1] = reg;
	txbuf[2] = val;

	return spi_write(chip->spi, txbuf, 3);
}

static int aw_spinand_chip_read_status(struct aw_spinand_chip *chip,
		u8 *status)
{
	return aw_spinand_chip_read_reg(chip, SPI_NAND_GETSR, REG_STATUS,
			status);
}

static int aw_spinand_chip_read_status_atomic(struct aw_spinand_chip *chip,
		u8 *status)
{
	return aw_spinand_chip_read_reg_atomic(chip, SPI_NAND_GETSR, REG_STATUS,
			status);
}

static int aw_spinand_chip_wait_atomic(struct aw_spinand_chip *chip, u8 *status)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	u8 s = 0;
	int ret;

	do {
		ret = aw_spinand_chip_read_status_atomic(chip, &s);
		if (ret)
			return ret;

		if (!(s & STATUS_BUSY))
			goto out;
	} while (time_before(jiffies, timeout));

	/*
	 * Extra read, just in case the STATUS_READY bit has changed
	 * since our last check
	 */
	/* ret = ops->read_status(chip, &s); */
	ret = aw_spinand_chip_read_status_atomic(chip, &s);
	if (ret)
		return ret;

out:
	if (status)
		*status = s;
	return s & STATUS_BUSY ? -ETIMEDOUT : 0;
}

static int aw_spinand_chip_get_block_lock(struct aw_spinand_chip *chip,
		u8 *reg_val)
{
	return aw_spinand_chip_read_reg(chip, SPI_NAND_GETSR, REG_BLOCK_LOCK,
			reg_val);
}

static int aw_spinand_chip_set_block_lock(struct aw_spinand_chip *chip,
		u8 reg_val)
{
	return aw_spinand_chip_write_reg(chip, SPI_NAND_SETSR, REG_BLOCK_LOCK,
			reg_val);
}

static int aw_spinand_chip_get_otp(struct aw_spinand_chip *chip, u8 *reg_val)
{
	return aw_spinand_chip_read_reg(chip, SPI_NAND_GETSR, REG_CFG, reg_val);
}

static int aw_spinand_chip_set_otp(struct aw_spinand_chip *chip, u8 reg_val)
{
	return aw_spinand_chip_write_reg(chip, SPI_NAND_SETSR, REG_CFG, reg_val);
}

static int aw_spinand_chip_get_driver_level(struct aw_spinand_chip *chip,
		u8 *reg_val)
{
	return aw_spinand_chip_read_reg(chip, SPI_NAND_GETSR, REG_DRV, reg_val);
}

static int aw_spinand_chip_set_driver_level(struct aw_spinand_chip *chip,
		u8 reg_val)
{
	return aw_spinand_chip_write_reg(chip, SPI_NAND_SETSR, REG_CFG, reg_val);
}

static int aw_spinand_chip_write_enable(struct aw_spinand_chip *chip)
{
	unsigned char txbuf[1];

	txbuf[0] = SPI_NAND_WREN;
	return spi_write(chip->spi, txbuf, 1);
}

static int aw_spinand_chip_write_enable_atomic(struct aw_spinand_chip *chip)
{
	unsigned char txbuf[1];

	txbuf[0] = SPI_NAND_WREN;
	return aw_spi_write_atomic(chip->spi, txbuf, 1);
}

/**
 * addr_to_req: convert data address to request for spinand
 *
 * @chip: spinand chip
 * @req: the request to save
 * @addr: the address
 *
 * Note:
 * If simulate multiplane, the request converted based on super page/block.
 */
int addr_to_req(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req, unsigned int addr)
{
	struct aw_spinand_info *info = chip->info;
	unsigned int addrtmp = addr;
	unsigned int block_size, page_size;

	if (addr > info->total_size(chip)) {
		sunxi_err(NULL, "over size: %u > %u\n", addr, info->total_size(chip));
		return -EOVERFLOW;
	}

	/*
	 * For historical reasons
	 * The SYS partition uses a superblock layout,
	 * while the other physical partitions do not.
	 * Added judgment to enable MTD devices to access the entire Flash
	 */
	if (addrtmp >= get_sys_part_offset()) {
		block_size = info->block_size(chip);
		page_size = info->page_size(chip);
	} else {
		block_size = info->phy_block_size(chip);
		page_size = info->phy_page_size(chip);
	}

	req->block = addrtmp / block_size;
	addrtmp = addrtmp & (block_size - 1);
	req->page = addrtmp / page_size;
	req->pageoff = addrtmp & (page_size - 1);
	sunxi_debug(NULL, "addr 0x%x to blk %u page %u pageoff %u\n",
			addr, req->block, req->page, req->pageoff);
	return 0;
}
EXPORT_SYMBOL(addr_to_req);

/**
 * req_to_paddr: convert request for spinand to page address
 *
 * @chip: spinand chip
 * @req: the request to save
 * @addr: the address
 *
 * Note:
 * If simulate multiplane, the request converted based on super page/block.
 * It is only used to get the page address to send to spinand.
 */
static unsigned int req_to_paddr(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;

	return req->block * pinfo->PageCntPerBlk + req->page;
}

static int aw_spinand_chip_erase_single_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	int ret;
	unsigned char txbuf[4];
	unsigned int paddr, pmax;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;
	u8 status = 0;

	if (likely(req->type != AW_SPINAND_MTD_REQ_PANIC))
		ret = aw_spinand_chip_write_enable(chip);
	else
		ret = aw_spinand_chip_write_enable_atomic(chip);
	if (ret)
		return ret;

	pmax = pinfo->DieCntPerChip * pinfo->BlkCntPerDie * pinfo->PageCntPerBlk;
	paddr = req_to_paddr(chip, req);
	if (paddr >= pmax)
		return -EOVERFLOW;

	txbuf[0] = SPI_NAND_BE;
	txbuf[1] = (paddr >> 16) & 0xFF;
	txbuf[2] = (paddr >> 8) & 0xFF;
	txbuf[3] = paddr & 0xFF;

	if (likely(req->type != AW_SPINAND_MTD_REQ_PANIC))
		ret = spi_write(chip->spi, txbuf, 4);
	else
		ret = aw_spi_write_atomic(chip->spi, txbuf, 4);

	if (ret)
		return ret;

	ret = aw_spinand_chip_wait(chip, &status);
	if (!ret && (status & STATUS_ERASE_FAILED))
		ret = -EIO;

	return ret;
}

/* the request must bases on single physical page/block */
static inline int aw_spinand_chip_write_to_cache(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	return chip->cache->write_to_cache(chip, req);
}

static int aw_spinand_chip_program(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	unsigned char txbuf[4];
	unsigned int paddr, pmax;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;

	pmax = pinfo->DieCntPerChip * pinfo->BlkCntPerDie * pinfo->PageCntPerBlk;
	paddr = req_to_paddr(chip, req);
	if (paddr >= pmax)
		return -EOVERFLOW;

	txbuf[0] = SPI_NAND_PE;
	txbuf[1] = (paddr >> 16) & 0xFF;
	txbuf[2] = (paddr >> 8) & 0xFF;
	txbuf[3] = paddr & 0xFF;

	if (likely(req->type != AW_SPINAND_MTD_REQ_PANIC))
		return spi_write(chip->spi, txbuf, 4);
	else
		return aw_spi_write_atomic(chip->spi, txbuf, 4);
}

static int aw_spinand_chip_write_single_page(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	int ret;
	struct aw_spinand_info *info = chip->info;
	struct aw_spinand_phy_info *pinfo = info->phy_info;
	u8 status = 0;

	aw_spinand_reqdump(pr_debug, "do single write", req);
	BUG_ON(req->pageoff + req->datalen > chip->info->phy_page_size(chip));

	if (req->page >= pinfo->PageCntPerBlk) {
		sunxi_err(NULL, "page %u over max pages per blk %u\n", req->page,
				pinfo->PageCntPerBlk);
		return -EOVERFLOW;
	}

	if (req->block >= pinfo->BlkCntPerDie) {
		sunxi_err(NULL, "block %u over max blocks per die %u\n", req->block,
				pinfo->BlkCntPerDie);
		return -EOVERFLOW;
	}


	if (likely(req->type != AW_SPINAND_MTD_REQ_PANIC))
		ret = aw_spinand_chip_write_enable(chip);
	else
		ret = aw_spinand_chip_write_enable_atomic(chip);
	if (ret)
		return ret;

	ret = aw_spinand_chip_write_to_cache(chip, req);
	if (ret)
		return ret;

	ret = aw_spinand_chip_program(chip, req);
	if (ret)
		return ret;

	if (likely(req->type != AW_SPINAND_MTD_REQ_PANIC))
		ret = aw_spinand_chip_wait(chip, &status);
	else
		ret = aw_spinand_chip_wait_atomic(chip, &status);

	if (!ret && (status & STATUS_PROG_FAILED))
		ret = -EIO;

	return ret;
}

static int aw_spinand_chip_load_page(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	unsigned char txbuf[4];
	unsigned int paddr, pmax;
	struct aw_spinand_info *info = chip->info;

	WARN_ON(!req->datalen && !req->ooblen);

	pmax = info->total_size(chip) / info->phy_page_size(chip);
	paddr = req_to_paddr(chip, req);
	if (paddr >= pmax)
		return -EOVERFLOW;

	txbuf[0] = SPI_NAND_PAGE_READ;
	txbuf[1] = (paddr >> 16) & 0xFF;
	txbuf[2] = (paddr >> 8) & 0xFF;
	txbuf[3] = paddr & 0xFF;

	if (likely(req->type != AW_SPINAND_MTD_REQ_PANIC))
		return spi_write(chip->spi, txbuf, 4);
	else
		return aw_spi_write_atomic(chip->spi, txbuf, 4);
}

static inline int aw_spinand_chip_read_from_cache(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	return chip->cache->read_from_cache(chip, req);
}

static int aw_spinand_chip_check_ecc(struct aw_spinand_chip *chip, u8 status)
{
	int ret;
	struct aw_spinand_ecc *ecc = chip->ecc;
	struct aw_spinand_chip_ops *ops = chip->ops;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;

	/* to get real ecc status */
	if (pinfo->EccFlag & HAS_EXT_ECC_STATUS) {
		/* extern ecc status should not shift */
		ret = ops->read_reg(chip, SPI_NAND_READ_INT_ECCSTATUS, 0,
				&status);
		if (ret)
			return ret;
	} else {
		if (pinfo->ecc_status_shift)
			status = status >> pinfo->ecc_status_shift;
		else
			status = status >> STATUS_ECC_SHIFT;
	}
	if (status && pinfo->EccFlag & HAS_EXT_ECC_SE01) {
		u8 ext_status;

		ret = ops->read_reg(chip, SPI_NAND_GETSR, GD_REG_EXT_ECC_STATUS,
				&ext_status);
		if (pinfo->ecc_status_shift)
			ext_status = (ext_status >> pinfo->ecc_status_shift) & 0x03;
		else
			ext_status = (ext_status >> STATUS_ECC_SHIFT) & 0x03;
		status = (status << 2) | ext_status;
	}

	return ecc->check_ecc(pinfo->EccType, status);
}

static int aw_spinand_chip_read_single_page(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	int ret;
	u8 status = 0;
	struct aw_spinand_cache *cache = chip->cache;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;

	aw_spinand_reqdump(pr_debug, "do single read", req);
	BUG_ON(req->pageoff + req->datalen > chip->info->phy_page_size(chip));

	if (req->page >= pinfo->PageCntPerBlk) {
		sunxi_err(NULL, "page %u over max pages per blk %u\n", req->page,
				pinfo->PageCntPerBlk);
		return -EOVERFLOW;
	}

	if (req->block >= pinfo->BlkCntPerDie) {
		sunxi_err(NULL, "block %u over max blocks per die %u\n", req->block,
				pinfo->BlkCntPerDie);
		return -EOVERFLOW;
	}

	/* If the cache already has the data before, just copy them to req */
	if (cache->match_cache(chip, req)) {
		sunxi_debug(NULL, "cache match request blk %u page %u, no need to send to spinand\n",
				req->block, req->page);
		return cache->copy_from_cache(chip, req);
	}

	ret = aw_spinand_chip_load_page(chip, req);
	if (ret)
		return ret;

	ret = aw_spinand_chip_wait(chip, &status);
	if (ret)
		return ret;

	ret = aw_spinand_chip_read_from_cache(chip, req);
	if (ret)
		return ret;

	return aw_spinand_chip_check_ecc(chip, status);
}

static int _aw_spinand_chip_isbad_single_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;
	int page_addr[2] = {0, -1};
	unsigned char oob[AW_OOB_SIZE_PER_PHY_PAGE] = {0xFF};
	int i;

	switch (pinfo->BadBlockFlag & BAD_BLK_FLAG_MARK) {
	case BAD_BLK_FLAG_FRIST_1_PAGE:
		/*
		 * the bad block flag is in the first page, same as the logical
		 * information, just read 1 page is ok
		 */
		page_addr[0] = 0;
		break;
	case BAD_BLK_FLAG_FIRST_2_PAGE:
		/*
		 * the bad block flag is in the first page or the second page,
		 * need read the first page and the second page
		 */
		page_addr[1] = 1;
		break;
	case BAD_BLK_FLAG_LAST_1_PAGE:
		/*
		 * the bad block flag is in the last page, need read the first
		 * page and the last page
		 */
		page_addr[1] = pinfo->PageCntPerBlk - 1;
		break;
	case BAD_BLK_FLAG_LAST_2_PAGE:
		/*
		 * the bad block flag is in the last 2 page, so, need read the
		 * first page, the last page and the last-1 page
		 */
		page_addr[1] = pinfo->PageCntPerBlk - 2;
		break;
	default:
		break;
	}

	for (i = 0; i < ARRAY_SIZE(page_addr); i++) {
		struct aw_spinand_chip_request tmp = {0};
		int ret;

		if (page_addr[i] == -1)
			break;

		tmp.block = req->block;
		tmp.page = (unsigned int)page_addr[i];
		tmp.ooblen = AW_OOB_SIZE_PER_PHY_PAGE;
		tmp.oobbuf = oob;
		tmp.mode = AW_SPINAND_MTD_OPS_RAW;
		ret = aw_spinand_chip_read_single_page(chip, &tmp);
		if (ret < 0)
			ret = aw_spinand_chip_read_single_page(chip, &tmp);
		/* ignore ECC_ERROR and ECC_LIMIT */
		if (ret < 0 || oob[0] != 0xFF)
			return true;
	}
	return false;
}

static int aw_spinand_chip_isbad_single_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	struct aw_spinand_bbt *bbt = chip->bbt;
	int ret;

	ret = bbt->is_badblock(chip, req->block);
	if (ret == NOT_MARKED) {
		ret = _aw_spinand_chip_isbad_single_block(chip, req);
		if (ret < 0)
			return ret;
		if (ret == true)
			bbt->mark_badblock(chip, req->block, true);
		else
			bbt->mark_badblock(chip, req->block, false);

		if (ret == true)
			sunxi_info(NULL, "phy blk %d is bad\n", req->block);
	}
	return ret;
}

static int aw_spinand_chip_markbad_single_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	int ret;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;
	struct aw_spinand_bbt *bbt = chip->bbt;
	unsigned char oob[AW_OOB_SIZE_PER_PHY_PAGE];
	struct aw_spinand_chip_request tmp = {0};

	ret = aw_spinand_chip_isbad_single_block(chip, req);
	if (ret == true)
		return 0;

	ret = aw_spinand_chip_erase_single_block(chip, req);
	if (ret)
		sunxi_err(NULL, "erase phy blk %d before markbad failed with %d back\n",
				req->block, ret);

	bbt->mark_badblock(chip, req->block, true);

	memset(oob, 0, AW_OOB_SIZE_PER_PHY_PAGE);
	tmp.block = req->block;
	tmp.oobbuf = oob;
	tmp.ooblen = AW_OOB_SIZE_PER_PHY_PAGE;
	tmp.mode = AW_SPINAND_MTD_OPS_RAW;

	/* write bad flag on the first page */
	tmp.page = 0;
	ret = aw_spinand_chip_write_single_page(chip, &tmp);
	if (ret) {
		sunxi_err(NULL, "mark phy blk %u page %d as bad failed with %d back\n",
				tmp.block, tmp.page, ret);
		return ret;
	}

	/* write bad flag on the last page */
	tmp.page = pinfo->PageCntPerBlk - 1;
	ret = aw_spinand_chip_write_single_page(chip, &tmp);
	if (ret) {
		sunxi_err(NULL, "mark phy blk %u page %d as bad failed with %d back\n",
				tmp.block, tmp.page, ret);
		return ret;
	}
	return 0;
}

static int aw_spinand_chip_copy_single_block(struct aw_spinand_chip *chip,
		unsigned int from_blk, unsigned int to_blk)
{
	struct aw_spinand_info *info = chip->info;
	struct aw_spinand_phy_info *pinfo = info->phy_info;
	struct aw_spinand_chip_request req = {0};
	int i, ret = -ENOMEM;

	req.datalen = info->phy_page_size(chip);
	req.databuf = kmalloc(req.datalen, GFP_KERNEL);
	if (!req.databuf)
		return ret;

	req.ooblen = info->phy_oob_size(chip);
	req.oobbuf = kmalloc(req.ooblen, GFP_KERNEL);
	if (!req.oobbuf)
		goto free_databuf;

	req.block = to_blk;
	ret = aw_spinand_chip_erase_single_block(chip, &req);
	if (ret)
		goto free_oobbuf;

	for (i = 0; i < pinfo->PageCntPerBlk; i++) {
		req.page = i;

		req.block = from_blk;
		ret = aw_spinand_chip_read_single_page(chip, &req);
		if (ret)
			goto free_oobbuf;

		req.block = to_blk;
		ret = aw_spinand_chip_write_single_page(chip, &req);
		if (ret)
			goto free_oobbuf;
	}

	ret = 0;
free_oobbuf:
	kfree(req.oobbuf);
free_databuf:
	kfree(req.databuf);
	return ret;
}

#if IS_ENABLED(CONFIG_SIMULATE_MULTIPLANE)

#if IS_ENABLED(CONFIG_AW_SPINAND_MTD_OOB_RAW_SPARE)
static void super_to_phy(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super,
		struct aw_spinand_chip_request *phy)
{
	struct aw_spinand_info *info = chip->info;
	unsigned int phy_page_size, phy_oob_size;

	phy_page_size = info->phy_page_size(chip);
	phy_oob_size = info->phy_oob_size(chip);

	phy->databuf = super->databuf;
	phy->oobbuf = super->oobbuf;
	phy->dataleft = super->datalen;
	phy->oobleft = super->ooblen;
	phy->pageoff = super->pageoff & (phy_page_size - 1);
	phy->ooblen = min(phy_oob_size, super->ooblen);
	phy->datalen = min(phy_page_size - phy->pageoff, phy->dataleft);
	phy->page = super->page;
	if (super->pageoff >= phy_page_size)
		phy->block = super->block * 2 + 1;
	else
		phy->block = super->block * 2;

		phy->type = super->type;
}
#else
static void super_to_phy(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super,
		struct aw_spinand_chip_request *phy)
{
	struct aw_spinand_info *info = chip->info;
	unsigned int phy_page_size, phy_oob_size;

	phy_page_size = info->phy_page_size(chip);
	phy_oob_size = info->phy_oob_size(chip);

	phy->databuf = super->databuf;
	phy->oobbuf = super->oobbuf;
	phy->dataleft = super->datalen;
	phy->oobleft = super->ooblen;
	phy->pageoff = super->pageoff & (phy_page_size - 1);
	phy->ooblen = min3(phy_oob_size, super->ooblen,
				(unsigned int)AW_OOB_SIZE_PER_PHY_PAGE);
	phy->datalen = min(phy_page_size - phy->pageoff, phy->dataleft);
	phy->page = super->page;
	if (super->pageoff >= phy_page_size)
		phy->block = super->block * 2 + 1;
	else
		phy->block = super->block * 2;

	phy->type = super->type;
}
#endif

void aw_spinand_chip_super_init(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super,
		struct aw_spinand_chip_request *phy)
{
	super_to_phy(chip, super, phy);
}

int aw_spinand_chip_super_end(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	if (req->dataleft || req->oobleft)
		return false;
	return true;
}

void aw_spinand_chip_super_next(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *req)
{
	struct aw_spinand_info *info = chip->info;
	unsigned int phy_page_size, phy_oob_size;
	const char *str;

	if (req->dataleft < req->datalen) {
		str = "[phy] dataleft < datalen";
		goto bug;
	}

	if (req->oobleft < req->ooblen) {
		str = "[phy] oobleft < ooblen";
		goto bug;
	}

	phy_page_size = info->phy_page_size(chip);
	phy_oob_size = info->phy_oob_size(chip);

	req->databuf += req->datalen;
	req->dataleft -= req->datalen;
	req->oobbuf += req->ooblen;
	req->oobleft -= req->ooblen;
	req->datalen = min(req->dataleft, phy_page_size);
	req->ooblen = min(req->oobleft, phy_oob_size);
	/* next page have no offset */
	req->pageoff = 0;
	req->block++;

	/*
	 * The super page (4K) just cut to 2 phyical page(2K), so,
	 * aw_spinand_chip_for_each_single maximum can only loop for twice.
	 */
	if ((req->dataleft || req->oobleft) && !(req->block % 2)) {
		str = "[phy] over loop twice";
		goto bug;
	}

	return;
bug:
	aw_spinand_reqdump(pr_err, str, req);
	WARN_ON(1);
	return;
}

#define aw_spinand_chip_for_each_single(chip, super, phy)		\
	for (aw_spinand_chip_super_init(chip, super, phy);		\
		!aw_spinand_chip_super_end(chip, phy);			\
		aw_spinand_chip_super_next(chip, phy))

static int aw_spinand_chip_isbad_super_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super)
{
	struct aw_spinand_chip_request phy = {0};
	int ret;

	super_to_phy(chip, super, &phy);
	if (unlikely(phy.block % 2)) {
		sunxi_err(NULL, "unaligned block %d\n", phy.block);
		return -EINVAL;
	}

	/* check the 1st block */
	ret = aw_spinand_chip_isbad_single_block(chip, &phy);
	if (ret)
		return ret;

	/* check the 2st block */
	phy.block++;
	ret = aw_spinand_chip_isbad_single_block(chip, &phy);
	if (ret)
		return ret;

	return false;
}

static int aw_spinand_chip_markbad_super_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super)
{
	struct aw_spinand_chip_request phy = {0};
	int ret;

	super_to_phy(chip, super, &phy);
	if (unlikely(phy.block % 2)) {
		sunxi_err(NULL, "unaligned block %d\n", phy.block);
		return -EINVAL;
	}

	/* mark the 1st block */
	ret = aw_spinand_chip_markbad_single_block(chip, &phy);

	/* check the 2st block */
	phy.block++;
	ret &= aw_spinand_chip_markbad_single_block(chip, &phy);
	if (ret)
		return ret;

	return 0;
}

static int aw_spinand_chip_erase_super_block(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super)
{
	struct aw_spinand_chip_request phy = {0};
	int ret;

	super_to_phy(chip, super, &phy);
	if (unlikely(phy.block % 2)) {
		sunxi_err(NULL, "unaligned block %d\n", phy.block);
		return -EINVAL;
	}

	/* erase the 1st block */
	ret = aw_spinand_chip_erase_single_block(chip, &phy);
	if (ret)
		return ret;

	/* check the 2st block */
	phy.block++;
	ret = aw_spinand_chip_erase_single_block(chip, &phy);
	if (ret)
		return ret;

	return 0;
}

static int aw_spinand_chip_write_super_page(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super)
{
	struct aw_spinand_chip_request phy = {0};
	int ret;

	aw_spinand_chip_for_each_single(chip, super, &phy) {
		ret = aw_spinand_chip_write_single_page(chip, &phy);
		if (ret)
			return ret;
	}
	return 0;
}


static int aw_spinand_chip_read_super_page(struct aw_spinand_chip *chip,
		struct aw_spinand_chip_request *super)
{
	struct aw_spinand_chip_request phy = {0};
	int ret, limit = 0;

	aw_spinand_chip_for_each_single(chip, super, &phy) {
		ret = aw_spinand_chip_read_single_page(chip, &phy);
		if (ret < 0)
			return ret;
		if (ret == ECC_LIMIT) {
			sunxi_debug(NULL, "ecc limit: phy block: %u page: %u\n",
					phy.block, phy.page);
			limit = ECC_LIMIT;
			continue;
		} else if (ret == ECC_ERR) {
			sunxi_err(NULL, "ecc err: phy block: %u page: %u\n",
					phy.block, phy.page);
			return ret;
		}
		/* else ECC_GOOD */
	}
	return limit;
}
#endif

static struct aw_spinand_chip_ops spinand_ops = {
	.get_block_lock = aw_spinand_chip_get_block_lock,
	.set_block_lock = aw_spinand_chip_set_block_lock,
	.get_otp = aw_spinand_chip_get_otp,
	.set_otp = aw_spinand_chip_set_otp,
	.get_driver_level = aw_spinand_chip_get_driver_level,
	.set_driver_level = aw_spinand_chip_set_driver_level,
	.reset = aw_spinand_chip_reset,
	.read_status = aw_spinand_chip_read_status,
	.read_id = aw_spinand_chip_read_id,
	.write_reg = aw_spinand_chip_write_reg,
	.read_reg = aw_spinand_chip_read_reg,
#if IS_ENABLED(CONFIG_SIMULATE_MULTIPLANE)
	.is_bad = aw_spinand_chip_isbad_super_block,
	.mark_bad = aw_spinand_chip_markbad_super_block,
	.erase_block = aw_spinand_chip_erase_super_block,
	.write_page = aw_spinand_chip_write_super_page,
	.read_page = aw_spinand_chip_read_super_page,
#else
	.is_bad = aw_spinand_chip_isbad_single_block,
	.mark_bad = aw_spinand_chip_markbad_single_block,
	.erase_block = aw_spinand_chip_erase_single_block,
	.write_page = aw_spinand_chip_write_single_page,
	.read_page = aw_spinand_chip_read_single_page,
#endif
	.phy_is_bad = aw_spinand_chip_isbad_single_block,
	.phy_mark_bad = aw_spinand_chip_markbad_single_block,
	.phy_erase_block = aw_spinand_chip_erase_single_block,
	.phy_write_page = aw_spinand_chip_write_single_page,
	.phy_read_page = aw_spinand_chip_read_single_page,
	.phy_copy_block = aw_spinand_chip_copy_single_block,
};

int aw_spinand_chip_ops_init(struct aw_spinand_chip *chip)
{
	chip->ops = &spinand_ops;
	return 0;
}
