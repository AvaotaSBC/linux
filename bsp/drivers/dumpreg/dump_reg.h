/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * dump registers head file
 *
 * (C) Copyright 2015-2018
 * Reuuimlla Technology Co., Ltd. <www.reuuimllatech.com>
 * Liugang <liugang@reuuimllatech.com>
 * Xiafeng <xiafeng@allwinnertech.com>
 * Martin  <wuyan@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */
#ifndef _DUMP_REG_H_
#define _DUMP_REG_H_

/* BROM/SRAM/peripheral-registers space */
#define SUNXI_IO_PHYS_START		(0x00000000UL)
#if IS_ENABLED(CONFIG_ARCH_RV32I)
#define SUNXI_IO_PHYS_END		(0x7fffffffUL)
#else
#define SUNXI_IO_PHYS_END		(0x09020FFFUL)
#endif

/* DRAM space (Only map the first 1GB) */
#define SUNXI_PLAT_PHYS_START		(0x40000000UL)
#define SUNXI_PLAT_PHYS_END		(SUNXI_PLAT_PHYS_START + SZ_1G - 1)

#if IS_ENABLED(CONFIG_ARM64) || IS_ENABLED(CONFIG_ARCH_RV64I)
/* Virtual address space 1 */
#define SUNXI_IOMEM_START		(0xffffff8000000000UL)
#define SUNXI_IOMEM_END			(SUNXI_IOMEM_START + SZ_2G - 1)
/* Virtual address space 2 */
#define SUNXI_MEM_PHYS_START		(0xffffffc000000000UL)
#define SUNXI_MEM_PHYS_END		(SUNXI_MEM_PHYS_START + SZ_2G - 1)
/* Print format */
#define PRINT_ADDR_FMT			"0x%016lx"
#define CMP_PRINT_FMT                   "reg                 expect      actual      mask        result\n"
#define WR_PRINT_FMT                    "reg                 to_write    after_write\n"
#elif IS_ENABLED(CONFIG_ARM) || IS_ENABLED(CONFIG_ARCH_RV32I)
/* Virtual address space 2 */
#define SUNXI_MEM_PHYS_START		PAGE_OFFSET
#define SUNXI_MEM_PHYS_END		(SUNXI_MEM_PHYS_START + SZ_1G - 1)
/* Print format */
#define PRINT_ADDR_FMT			"0x%08lx"
#define CMP_PRINT_FMT                   "reg         expect      actual      mask        result\n"
#define WR_PRINT_FMT                    "reg         to_write    after_write\n"
#endif

/* Item count */
#define MAX_COMPARE_ITEM		64
#define MAX_WRITE_ITEM			64

struct dump_addr {
	/* User specified address. Maybe physical or virtual address */
	unsigned long uaddr_start;
	unsigned long uaddr_end;
	/* Virtual address */
	void __iomem *vaddr_start;
};

struct dump_struct {
	unsigned long addr_start;
	unsigned long addr_end;
	/* some registers' operate method maybe different */
	void __iomem *(*remap)(unsigned long paddr, size_t size);
	void (*unmap)(void __iomem *vaddr);
	void __iomem *(*get_vaddr)(struct dump_addr *dump_addr, unsigned long uaddr);
	 u32 (*read)(void __iomem *vaddr);
	void (*write)(u32 val, void __iomem *vaddr);
};

/**
 * compare_item - reg compare item struct
 * @reg_addr:	reg address.
 * @val_expect: expected value, provided by caller.
 * @val_mask:   mask value, provided by caller. only mask bits will be compared.
 */
struct compare_item {
	unsigned long reg_addr;
	u32	val_expect;
	u32	val_mask;
};

/**
 * compare_group - reg compare group struct
 * @num:	pitem element count. cannot exceed MAX_COMPARE_ITEM.
 * @pitem:	items that will be compared, provided by caller.
 */
struct compare_group {
	u32	num;
	u32	reserve;
	struct compare_item *pitem;
};

/**
 * write_item - reg write item struct
 * @reg_addr:	reg address.
 * @val:	value to write
 */
struct write_item {
	unsigned long reg_addr;
	u32	val;
	u32	reserve;
};

/**
 * write_group - reg write group struct
 * @num:	pitem element count. cannot exceed MAX_WRITE_ITEM.
 * @pitem:	items that will be write, provided by caller.
 */
struct write_group {
	u32	num;
	u32	reserve;
	struct write_item *pitem;
};

const struct dump_struct dump_table[4];

#endif /* _DUMP_REG_H_ */
