/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs display driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _DISP_PRIVATE_H_
#define _DISP_PRIVATE_H_

#include "disp_features.h"
#if defined(DE_VERSION_V33X) || defined(CONFIG_ARCH_SUN50IW9)
#include "./lowlevel_v33x/disp_al.h"
#elif defined(DE_VERSION_V35X) || defined(CONFIG_ARCH_SUN55IW3)
#include "./lowlevel_v35x/disp_al.h"
#elif defined(DE_VERSION_V21X)
#include "./lowlevel_v21x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN50IW1)
#include "./lowlevel_sun50iw1/disp_al.h"
#elif defined(CONFIG_ARCH_SUN50IW2)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN50IW8)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW10)
#include "./lowlevel_sun8iw10/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW11)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW12) || defined(CONFIG_ARCH_SUN8IW16)\
	|| defined(CONFIG_ARCH_SUN8IW19) || defined(CONFIG_ARCH_SUN8IW20)\
	|| defined(CONFIG_ARCH_SUN20IW1)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW15)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN50IW10)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW6)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW7) || defined(CONFIG_ARCH_SUN8IW17)
#include "./lowlevel_v2x/disp_al.h"
#elif defined(CONFIG_ARCH_SUN8IW8)
#include "./lowlevel_sun8iw8/disp_al.h"
#elif defined(CONFIG_ARCH_SUN50IW3) || defined(CONFIG_ARCH_SUN50IW6)
#include "./lowlevel_v3x/disp_al.h"
#else
#error "undefined platform!!!"
#endif

/**
 * layer identify
 */
struct disp_layer_id {
	unsigned int disp;
	unsigned int channel;
	unsigned int layer_id;
	unsigned int type; /* 1:layer,2:trd,4:atw */
};

struct disp_layer_address {
	dma_addr_t dma_addr;
	dma_addr_t atw_addr;
	dma_addr_t trd_addr;
	struct disp_layer_id lyr_id;
};

struct dmabuf_item {
	struct list_head list;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	unsigned long long id;
	struct disp_layer_id lyr_id;
	unsigned int unref;
	bool force_set_layer_buffer;
	struct afbc_header afbc_head;
};
/* fb_addrss_transfer - framebuffer address transfer
 *
 * @format: pixel format
 * @size: size for each plane
 * @align: align for each plane
 * @depth: depth perception for stereo image
 * @dma_addr: the start addrss of this buffer
 *
 * @addr[out]: address for each plane
 * @trd_addr[out]: address for each plane of right eye buffer
 */
struct fb_address_transfer {
	enum disp_pixel_format format;
	struct disp_rectsz size[3];
	unsigned int align[3];
	int depth;
	dma_addr_t dma_addr;
	unsigned long long addr[3];
	unsigned long long trd_right_addr[3];
};

/* disp_format_attr - display format attribute
 *
 * @format: pixel format
 * @bits: bits of each component
 * @hor_rsample_u: reciprocal of horizontal sample rate
 * @hor_rsample_v: reciprocal of horizontal sample rate
 * @ver_rsample_u: reciprocal of vertical sample rate
 * @hor_rsample_v: reciprocal of vertical sample rate
 * @uvc: 1: u & v component combined
 * @interleave: 0: progressive, 1: interleave
 * @factor & div: bytes of pixel = factor / div (bytes)
 *
 * @addr[out]: address for each plane
 * @trd_addr[out]: address for each plane of right eye buffer
 */
struct disp_format_attr {
	enum disp_pixel_format format;
	unsigned int bits;
	unsigned int hor_rsample_u;
	unsigned int hor_rsample_v;
	unsigned int ver_rsample_u;
	unsigned int ver_rsample_v;
	unsigned int uvc;
	unsigned int interleave;
	unsigned int factor;
	unsigned int div;
};

struct disp_irq_info {
	u32 sel; /* select id of disp or wb */
	u32 irq_flag;
	u32 irq_type;
	void *ptr;
	s32 (*irq_handler)(u32 sel, u32 irq_flag, void *ptr);
};

extern int disp_lcd_get_device_count(void);
extern struct disp_device *disp_get_lcd(u32 dev_index);

extern struct disp_device *disp_get_edp(u32 disp);

extern struct disp_device *disp_get_hdmi(u32 disp);

extern struct disp_manager *disp_get_layer_manager(u32 disp);

extern struct disp_layer *disp_get_layer(u32 disp, u32 chn, u32 layer_id);
extern struct disp_layer *disp_get_layer_1(u32 disp, u32 layer_id);
extern struct disp_smbl *disp_get_smbl(u32 disp);
extern struct disp_enhance *disp_get_enhance(u32 disp);
extern struct disp_capture *disp_get_capture(u32 disp);

extern s32 disp_delay_ms(u32 ms);
extern s32 disp_delay_us(u32 us);
extern s32 disp_init_lcd(struct disp_bsp_init_para *para);
extern s32 disp_exit_lcd(void);
extern s32 disp_init_hdmi(struct disp_bsp_init_para *para);
extern s32 disp_exit_hdmi(void);
extern s32 disp_init_tv(void);	/* (struct disp_bsp_init_para * para); */
extern s32 disp_exit_tv(void);	/* (struct disp_bsp_init_para * para); */
extern s32 disp_exit_vdpo(void);
extern s32 disp_tv_set_func(struct disp_device *ptv, struct disp_tv_func *func);
extern s32 disp_init_tv_para(struct disp_bsp_init_para *para);
extern s32 disp_exit_tv_para(void);
extern s32 disp_tv_set_hpd(struct disp_device *ptv, u32 state);
extern s32 disp_init_vga(void);
extern s32 disp_exit_vga(void);
extern s32 disp_init_vdpo(struct disp_bsp_init_para *para);
extern s32 disp_init_edp(struct disp_bsp_init_para *para);

extern s32 disp_init_feat(struct disp_feat_init *feat_init);
extern s32 disp_exit_feat(void);
extern s32 disp_init_mgr(struct disp_bsp_init_para *para);
extern s32 disp_exit_mgr(void);
extern s32 disp_init_enhance(struct disp_bsp_init_para *para);
extern s32 disp_exit_enhance(void);
extern s32 disp_init_smbl(struct disp_bsp_init_para *para);
extern s32 disp_exit_smbl(void);
extern s32 disp_init_capture(struct disp_bsp_init_para *para);
extern s32 disp_exit_capture(void);

#if IS_ENABLED(CONFIG_EINK_PANEL_USED)
extern s32 disp_init_eink(struct disp_bsp_init_para *para);
extern s32 disp_exit_eink(void);
extern s32 write_edma(struct disp_eink_manager *manager);
extern s32 disp_init_format_convert_manager(struct disp_bsp_init_para *para);
extern void disp_exit_format_convert_manager(void);

extern struct disp_eink_manager *disp_get_eink_manager(unsigned int disp);
extern int eink_display_one_frame(struct disp_eink_manager *manager);
#endif
extern void sync_event_proc(u32 disp, bool timeout);

#include "disp_device.h"

u32 dump_layer_config(struct disp_layer_config_data *data);
void *disp_vmap(unsigned long phys_addr, unsigned long size);
void disp_vunmap(const void *vaddr);

struct dmabuf_item *disp_dma_map(int fd);
void disp_dma_unmap(struct dmabuf_item *item);
s32 disp_set_fb_info(struct fb_address_transfer *fb, bool left_eye);
s32 disp_set_fb_base_on_depth(struct fb_address_transfer *fb);
extern s32 disp_init_rotation_sw(struct disp_bsp_init_para *para);
extern struct disp_rotation_sw *disp_get_rotation_sw(u32 disp);

s32 disp_init_irq_util(u32 irq_no);
s32 disp_register_irq(u32 id, struct disp_irq_info *irq_info);
s32 disp_unregister_irq(u32 id, struct disp_irq_info *irq_info);

#endif
