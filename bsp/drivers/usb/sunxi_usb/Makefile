# SPDX-License-Identifier: GPL-2.0
# Makefile for SUNXI USB2.0 Dual Role Controller Driver.
#
CFLAGS_sunxi_usb_trace.o	:= -I$(src)

sunxi_usbc-objs := manager/usb_manager.o \
			manager/usbc0_platform.o \
			manager/usb_hcd_servers.o \
			manager/usb_hw_scan.o \
			manager/usb_msg_center.o

ifeq ($(CONFIG_USB_SUNXI_HCD0),y)
        sunxi_usbc-objs +=hcd/core/sunxi_hcd_core.o \
                                        hcd/core/sunxi_hcd_host.o \
                                        hcd/core/sunxi_hcd_virt_hub.o \
                                        hcd/core/sunxi_hcd_debug.o \
                                        hcd/core/sunxi_hcd_dma.o \
                                        usbc/usbc_host.o

        sunxi_usb_hcd0-objs := hcd/hcd0/sunxi_hcd0.o
endif

obj-$(CONFIG_USB_SUNXI_HCD0) += sunxi_usb_hcd0.o
obj-$(CONFIG_USB_SUNXI_USB_MANAGER) += sunxi_usbc.o

sunxi_usb_udc-objs :=  udc/sunxi_udc.o \
                    udc/sunxi_udc_board.o \
                    udc/sunxi_udc_debug.o \
                    udc/sunxi_udc_dma.o \
                    usbc/usbc.o \
                    usbc/usbc_dev.o \
                    usbc/usbc_phy.o \
                    misc/sunxi_usb_debug.o \
		    sunxi_usb_trace.o

obj-$(CONFIG_USB_SUNXI_UDC0) += sunxi_usb_udc.o

ccflags-y += -DDYNAMIC_DEBUG_MODULE
