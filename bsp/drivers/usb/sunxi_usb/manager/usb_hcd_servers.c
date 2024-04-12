/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * (C) Copyright 2010-2015
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * javen, 2011-4-14, create this file
 *
 * usb host contoller driver, service function set.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include  "../include/sunxi_usb_config.h"
#include  "usb_hcd_servers.h"

#if IS_ENABLED(CONFIG_ARCH_SUN50I) \
			|| IS_ENABLED(CONFIG_ARCH_SUN8IW8) \
			|| IS_ENABLED(CONFIG_ARCH_SUN8IW10) \
			|| IS_ENABLED(CONFIG_ARCH_SUN8IW11)
#define HCI0_USBC_NO    0
#define HCI1_USBC_NO    1
#define HCI2_USBC_NO    2
#define HCI3_USBC_NO    3
#else
#define HCI0_USBC_NO    1
#define HCI1_USBC_NO    2
#define HCI2_USBC_NO    3
#define HCI3_USBC_NO    4
#endif

int sunxi_usb_disable_hcd(__u32 usbc_no)
{
#ifndef SUNXI_USB_FPGA
	if (usbc_no == 0) {
#if IS_ENABLED(CONFIG_ARCH_SUN8IW6)
	#if IS_ENABLED(CONFIG_USB_SUNXI_HCD0)
		sunxi_usb_disable_hcd0();
	#endif
#else
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI0)
		sunxi_usb_disable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI0)
		sunxi_usb_disable_ohci(usbc_no);
	#endif
#endif
	} else if (usbc_no == HCI0_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI0)
		sunxi_usb_disable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI0)
		sunxi_usb_disable_ohci(usbc_no);
	#endif
	} else if (usbc_no == HCI1_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI1)
		sunxi_usb_disable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI1)
		sunxi_usb_disable_ohci(usbc_no);
	#endif
	} else if (usbc_no == HCI2_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI2)
		sunxi_usb_disable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI2)
		sunxi_usb_disable_ohci(usbc_no);
	#endif
	} else if (usbc_no == HCI3_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI3)
		sunxi_usb_disable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI3)
		sunxi_usb_disable_ohci(usbc_no);
	#endif
	} else {
		DMSG_PANIC("ERR: unknown usbc_no(%d)\n", usbc_no);
		return -1;
	}
#endif
	return 0;
}
EXPORT_SYMBOL(sunxi_usb_disable_hcd);

int sunxi_usb_enable_hcd(__u32 usbc_no)
{
#ifndef SUNXI_USB_FPGA
	if (usbc_no == 0) {
#if IS_ENABLED(CONFIG_ARCH_SUN8IW6)
	#if IS_ENABLED(CONFIG_USB_SUNXI_HCD0)
		sunxi_usb_enable_hcd0();
	#endif
#else
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI0)
		sunxi_usb_enable_ehci(usbc_no);
	#endif
	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI0)
		sunxi_usb_enable_ohci(usbc_no);
	#endif
#endif
	} else if (usbc_no == HCI0_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI0)
		sunxi_usb_enable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI0)
		sunxi_usb_enable_ohci(usbc_no);
	#endif
	} else if (usbc_no == HCI1_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI1)
		sunxi_usb_enable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI1)
		sunxi_usb_enable_ohci(usbc_no);
	#endif
	} else if (usbc_no == HCI2_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI2)
		sunxi_usb_enable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI2)
		sunxi_usb_enable_ohci(usbc_no);
	#endif
	} else if (usbc_no == HCI3_USBC_NO) {
	#if IS_ENABLED(CONFIG_USB_SUNXI_EHCI3)
		sunxi_usb_enable_ehci(usbc_no);
	#endif

	#if IS_ENABLED(CONFIG_USB_SUNXI_OHCI3)
		sunxi_usb_enable_ohci(usbc_no);
	#endif
	} else {
		DMSG_PANIC("ERR: unknown usbc_no(%d)\n", usbc_no);
		return -1;
	}
#endif
	return 0;
}
EXPORT_SYMBOL(sunxi_usb_enable_hcd);
