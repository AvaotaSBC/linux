/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * (C) Copyright 2010-2015
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * javen, 2010-3-3, create this file
 *
 * udc dma ops.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#ifndef __SUNXI_UDC_DMA_H__
#define __SUNXI_UDC_DMA_H__

#ifdef SW_UDC_DMA
#define  is_udc_support_dma()		1
#else
#define  is_udc_support_dma()		0
#endif

/* dma channel total */
#if IS_ENABLED(CONFIG_ARCH_SUN60IW2)
#define DMA_CHAN_TOTAL			(16)
#else
#define DMA_CHAN_TOTAL			(8)
#endif
typedef int *dm_hdl_t;

/* define dma channel struct */
typedef struct {
	u32		used;		/* 1 used, 0 unuse */
	u32		channel_num;
	u32		ep_num;		/* the ep's index in sunxi_udc.ep[] */
	void __iomem    *reg_base;	/* regs base addr */
	spinlock_t	lock;		/* dma channel lock */
} dma_channel_t;

extern dma_channel_t dma_chnl[DMA_CHAN_TOTAL];

/* dma config information */
struct dma_config_t {
	bool		dma_addr_ext_enable;
	spinlock_t	lock;		/* dma channel lock */
	u32		dma_num;
	u32		dma_working;
	u32		dma_en;
	u32		dma_bst_len;
	u32		dma_dir;
	u32		dma_for_ep;
	u32		dma_sdram_str_addr;
	u32		dma_bc;
	u32		dma_residual_bc;
	u32		dma_sdram_str_addr_ext;
	u32		dma_wordaddr_bypass;
};

extern int g_dma_debug;

void sunxi_udc_switch_bus_to_dma(struct sunxi_udc_ep *ep, u32 is_tx);
void sunxi_udc_switch_bus_to_pio(struct sunxi_udc_ep *ep, __u32 is_tx);

void sunxi_udc_enable_dma_channel_irq(struct sunxi_udc_ep *ep);
void sunxi_udc_disable_dma_channel_irq(struct sunxi_udc_ep *ep);
void sunxi_dma_set_config(dm_hdl_t dma_hdl, struct dma_config_t *pcfg);
dm_hdl_t sunxi_udc_dma_request(void);
int sunxi_udc_dma_release(dm_hdl_t dma_hdl);
int sunxi_udc_dma_chan_disable(dm_hdl_t dma_hdl);
void sunxi_udc_dma_set_config(struct sunxi_udc_ep *ep,
		struct sunxi_udc_request *req, __u64 buff_addr, __u32 len);
void sunxi_udc_dma_start(struct sunxi_udc_ep *ep,
		void __iomem  *fifo, __u64 buffer, __u32 len);
void sunxi_udc_dma_stop(struct sunxi_udc_ep *ep);
__u32 sunxi_udc_dma_transmit_length(struct sunxi_udc_ep *ep);
__u32 sunxi_udc_dma_is_busy(struct sunxi_udc_ep *ep);
void sunxi_udc_dma_completion(struct sunxi_udc *dev,
		struct sunxi_udc_ep *ep, struct sunxi_udc_request *req);

__s32 sunxi_udc_dma_probe(struct sunxi_udc *dev);
__s32 sunxi_udc_dma_remove(struct sunxi_udc *dev);

#endif /* __SUNXI_UDC_DMA_H__ */
