/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Earlyprintk support.
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Catalin Marinas <catalin.marinas@arm.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/io.h>

#include <linux/amba/serial.h>
#include <linux/serial_reg.h>

#include <asm/fixmap.h>

#define UART_USR       31      /* I/O: uart status Register */
#define UART_USR_NF    0x02    /* Tansmit fifo not full */

static void __iomem *early_base;
static void (*printch)(char ch);

/*
 * 8250/16550 (8-bit aligned registers) single character TX.
 */
static void uart8250_8bit_printch(char ch)
{
	while (!(readb_relaxed(early_base + UART_LSR) & UART_LSR_THRE))
		;
	writeb_relaxed(ch, early_base + UART_TX);
}

/*
 * 8250/16550 (32-bit aligned registers) single character TX.
 */
static void uart8250_32bit_printch(char ch)
{
	while (!(readl_relaxed(early_base + (UART_LSR << 2)) & UART_LSR_THRE))
		;
	writel_relaxed(ch, early_base + (UART_TX << 2));
}

/*
 * sunxi-uart single character TX.
 */
static void sunxi_uart_printch(char ch)
{

	while (!(readl_relaxed(early_base + (UART_USR << 2)) & UART_USR_NF))
		;

	writel_relaxed(ch, early_base + (UART_TX << 2));
}

struct earlycon_match {
	const char *name;
	void (*printch)(char ch);
};

static const struct earlycon_match earlycon_match[] __initconst = {
	{ .name = "uart8250-8bit", .printch = uart8250_8bit_printch, },
	{ .name = "uart8250-32bit", .printch = uart8250_32bit_printch, },
	{ .name = "sunxi-uart", .printch = sunxi_uart_printch, },
	{}
};

static void early_write(struct console *con, const char *s, unsigned n)
{
	while (n-- > 0) {
		if (*s == '\n')
			printch('\r');
		printch(*s);
		s++;
	}
}

static struct console early_console_dev = {
	.name =		"earlycon",
	.write =	early_write,
	.flags =	CON_PRINTBUFFER | CON_BOOT,
	.index =	-1,
};

struct console *early_console;

/*
 * Parse earlyprintk=... parameter in the format:
 *
 *   <name>[,<addr>][,<options>]
 *
 * and register the early console. It is assumed that the UART has been
 * initialised by the bootloader already.
 */
static int __init setup_early_printk(char *buf)
{
	const struct earlycon_match *match = earlycon_match;
	phys_addr_t paddr = 0;
	char *e;

	if (!buf) {
		pr_warn("No earlyprintk arguments passed.\n");
		return 0;
	}

	while (match->name) {
		size_t len = strlen(match->name);
		if (!strncmp(buf, match->name, len)) {
			buf += len;
			break;
		}
		match++;
	}
	if (!match->name) {
		pr_warn("Unknown earlyprintk arguments: %s\n", buf);
		return 0;
	}

	/* I/O address */
	if (!strncmp(buf, ",0x", 3)) {
		paddr = simple_strtoul(buf + 1, &e, 16);
	}
	/* no options parsing yet */

	if (paddr) {
		set_fixmap_io(FIX_EARLYCON_MEM_BASE, paddr);
		early_base = (void __iomem *)(fix_to_virt(FIX_EARLYCON_MEM_BASE)
						|(paddr & (PAGE_SIZE - 1)));
	}
	printch = match->printch;
	early_console = &early_console_dev;
	register_console(&early_console_dev);

	return 0;
}

early_param("earlyprintk", setup_early_printk);
