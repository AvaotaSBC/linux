/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * sunxi-nftl-core.c for  sunxi spi nand base on nftl.
 *
 * Copyright (C) 2019 Allwinner.
 * SPDX-License-Identifier: GPL-2.0
 */

//#define pr_fmt(fmt) "sunxi-spinand: " fmt
#define SUNXI_MODNAME "sunxi-spinand"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <sunxi-sid.h>
#include "sunxi-spinand.h"

static int ubootblks = -1;
module_param(ubootblks, int, 0400);

void aw_spinand_uboot_blknum(struct aw_spinand *spinand, unsigned int *start,
		unsigned int *end)
{
	struct aw_spinand_chip *chip = spinand_to_chip(spinand);
	struct aw_spinand_info *info = chip->info;
	unsigned int blksize = info->phy_block_size(chip);
	unsigned int pagecnt = blksize / info->phy_page_size(chip);
	unsigned int _start, _end;

	/* small nand:block size < 1MB;  reserve 4M for uboot */
	if (blksize <= SZ_128K) {
		_start = UBOOT_START_BLOCK_SMALLNAND;
		_end = _start + 32;
	} else if (blksize <= SZ_256K) {
		_start = UBOOT_START_BLOCK_SMALLNAND;
		_end = _start + 16;
	} else if (blksize <= SZ_512K) {
		_start = UBOOT_START_BLOCK_SMALLNAND;
		_end = _start + 8;
	} else if (blksize <= SZ_1M && pagecnt <= 128) {
		_start = UBOOT_START_BLOCK_SMALLNAND;
		_end = _start + 4;
		/* big nand;  reserve at least 20M for uboot */
	} else if (blksize <= SZ_1M && pagecnt > 128) {
		_start = UBOOT_START_BLOCK_BIGNAND;
		_end = _start + 20;
	} else if (blksize <= SZ_2M) {
		_start = UBOOT_START_BLOCK_BIGNAND;
		_end = _start + 10;
	} else {
		_start = UBOOT_START_BLOCK_BIGNAND;
		_end = _start + 8;
	}

	if (ubootblks > 0)
		_end = _start + ubootblks;
	else
		/* update parameter to /sys */
		ubootblks = _end - _start;

	if (start)
		*start = _start;
	if (end)
		*end = _end;
}
EXPORT_SYMBOL(aw_spinand_uboot_blknum);
