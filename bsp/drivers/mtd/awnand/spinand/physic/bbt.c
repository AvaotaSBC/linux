// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* bad block table */
//#define pr_fmt(fmt) "sunxi-spinand-phy: " fmt
#define SUNXI_MODNAME "sunxi-spinand-phy"
#include <sunxi-log.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/aw-spinand.h>
#include <linux/types.h>
#include <linux/bitops.h>

#include <linux/delay.h>

#include "physic.h"

static int aw_spinand_bbt_mark_badblock(struct aw_spinand_chip *chip,
		unsigned int blknum, bool badblk)
{
	struct aw_spinand_bbt *bbt = chip->bbt;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;
	unsigned int blkcnt = pinfo->DieCntPerChip * pinfo->BlkCntPerDie;

	if (blknum > blkcnt)
		return -EOVERFLOW;

	if (badblk == true)
		set_bit(blknum, bbt->bitmap);
	set_bit(blknum, bbt->en_bitmap);
	sunxi_debug(NULL, "bbt: mark blk %u as %s\n", blknum, badblk ? "bad" : "good");
	return 0;
}

static int aw_spinand_bbt_is_badblock(struct aw_spinand_chip *chip,
		unsigned int blknum)
{
	struct aw_spinand_bbt *bbt = chip->bbt;
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;
	unsigned int blkcnt = pinfo->DieCntPerChip * pinfo->BlkCntPerDie;

	if (blknum > blkcnt)
		return -EOVERFLOW;

	if (!test_bit(blknum, bbt->en_bitmap))
		return NOT_MARKED;

	sunxi_debug(NULL, "bbt: blk %u is %s\n", blknum,
			test_bit(blknum, bbt->bitmap) ? "bad" : "good");
	if (test_bit(blknum, bbt->bitmap))
		return BADBLOCK;
	else
		return NON_BADBLOCK;
}

struct aw_spinand_bbt aw_spinand_bbt = {
	.mark_badblock = aw_spinand_bbt_mark_badblock,
	.is_badblock = aw_spinand_bbt_is_badblock,
};

int aw_spinand_chip_bbt_init(struct aw_spinand_chip *chip)
{
	struct aw_spinand_phy_info *pinfo = chip->info->phy_info;
	unsigned int blkcnt = pinfo->DieCntPerChip * pinfo->BlkCntPerDie;
	unsigned long longcnt = BITS_TO_LONGS(blkcnt);
	struct aw_spinand_bbt *bbt = &aw_spinand_bbt;

	bbt->bitmap = kzalloc(longcnt * sizeof(long) * 2, GFP_KERNEL);
	if (!bbt->bitmap)
		return -ENOMEM;
	bbt->en_bitmap = bbt->bitmap + longcnt;
	chip->bbt = bbt;
	return 0;
}

void aw_spinand_chip_bbt_exit(struct aw_spinand_chip *chip)
{
	struct aw_spinand_bbt *bbt = chip->bbt;

	if (bbt && bbt->bitmap)
		kfree(bbt->bitmap);
	bbt->bitmap = NULL;
}
