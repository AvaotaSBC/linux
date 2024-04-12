/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Copyright (c) 2007-2018 Allwinnertech Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _DI_CLIENT_H_
#define _DI_CLIENT_H_

#include "sunxi_di.h"
#include "di_utils.h"

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>

#define DI_CLIENT_CNT_MAX 32

#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
#define DI_IN_FB_NUM_MAX 6
#else
#define DI_IN_FB_NUM_MAX 3
#endif

#define DI_OUT_FB_NUM_MAX 3
#define DI_FB_NUM_MAX (DI_IN_FB_NUM_MAX + DI_OUT_FB_NUM_MAX)

enum di_mode {
	DI_MODE_INVALID = 0,
	DI_MODE_60HZ,
	DI_MODE_30HZ,
	DI_MODE_BOB,
	DI_MODE_WEAVE,
	DI_MODE_TNR, /* only tnr */
};

enum {
	DI_PROC_STATE_IDLE = 0,
	DI_PROC_STATE_FINISH = DI_PROC_STATE_IDLE,
	DI_PROC_STATE_WAIT2START,
	DI_PROC_STATE_2START,
	DI_PROC_STATE_WAIT4FINISH,
	DI_PROC_STATE_FINISH_ERR,
};

enum {
	DI_DETECT_INTERLACE = 0,
	DI_DETECT_PROGRESSIVE = 1,
};

/* buffer for md flags */
struct di_md_buf {
	u32 dir;
	u32 w_bit;
	u32 w_stride;
	u32 h;
	struct di_mapped_buf *mbuf[2];
};

/* for process fb */
struct di_dma_fb {
	struct di_fb *fb;
	struct di_dma_item *dma_item;
};

#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
struct di_client {
	struct list_head node;
	const char *name;

	/* user setting para */
	struct di_timeout_ns timeout;
	struct di_size video_size;
	struct di_dit_mode dit_mode;
	struct di_tnr_mode tnr_mode;
	struct di_fmd_enable fmd_en;
	/* inner setting para */
	enum di_mode mode;
	bool md_en;
	bool tnr_en;
	bool vof_buf_en;
	struct di_rect md_out_crop;
	struct di_rect dit_out_crop;
	struct di_rect tnr_out_crop;
	struct di_rect fmd_out_crop;
	struct di_rect dit_demo_crop;
	struct di_rect tnr_demo_crop;
	u32 vof_blk_size_sel;

	struct di_process_fb_arg fb_arg;

	/* dma fb resources pointers */
	struct di_dma_fb *in_fb0;
	struct di_dma_fb *in_fb0_nf; /* _nf, next field */
	struct di_dma_fb *in_fb1;
	struct di_dma_fb *in_fb1_nf; /* _nf, next field */
	struct di_dma_fb *in_fb2;
	struct di_dma_fb *in_fb2_nf; /* _nf, next field */
	struct di_dma_fb *out_dit_fb0;
	struct di_dma_fb *out_dit_fb1;
	struct di_dma_fb *out_tnr_fb0;
	struct di_dma_fb *dma_di;
	struct di_dma_fb *dma_di_nf; /* _nf, next field */
	struct di_dma_fb *dma_p;
	struct di_dma_fb *dma_p_nf; /* _nf, next field */
	struct di_dma_fb *dma_c;
	struct di_dma_fb *dma_c_nf; /* _nf, next field */
	struct di_dma_fb *di_w0;
	struct di_dma_fb *di_w1;
	struct di_dma_fb *tnr_w;
	/* dma fb resources. it is the memory that the above di_dma_fb* member point to */
	struct di_dma_fb fb_pool[DI_FB_NUM_MAX];

	struct di_md_buf md_buf;

	/* runtime context */
	wait_queue_head_t wait;
	atomic_t wait_con;
	u64 proc_fb_seqno;
	bool apply_fixed_para;
	bool para_checked;
	bool unreset;

	u8 di_detect_result;
	u64 interlace_detected_counts;
	u64 lastest_interlace_detected_frame;
	u64 interlace_detected_counts_exceed_first_p_frame;
	u64 progressive_detected_counts;
	u64 lastest_progressive_detected_frame;
	u64 progressive_detected_first_frame;

	/* dev_cdata must be at last!!! */
	uintptr_t dev_cdata;
};
#else
struct di_client {
	struct list_head node;
	const char *name;

	/* user setting para */
	struct di_timeout_ns timeout;
	struct di_size video_size;
	struct di_dit_mode dit_mode;
	struct di_tnr_mode tnr_mode;
	struct di_fmd_enable fmd_en;
	/* inner setting para */
	enum di_mode mode;
	bool md_en;
	bool tnr_en;
	bool vof_buf_en;
	struct di_rect md_out_crop;
	struct di_rect dit_out_crop;
	struct di_rect tnr_out_crop;
	struct di_rect fmd_out_crop;
	struct di_rect dit_demo_crop;
	struct di_rect tnr_demo_crop;
	u32 vof_blk_size_sel;

	struct di_process_fb_arg fb_arg;

	/* dma fb resources pointers */
	struct di_dma_fb *in_fb0;
	struct di_dma_fb *in_fb1;
	struct di_dma_fb *in_fb2;
	struct di_dma_fb *out_dit_fb0;
	struct di_dma_fb *out_dit_fb1;
	struct di_dma_fb *out_tnr_fb0;
	struct di_dma_fb *dma_di;
	struct di_dma_fb *dma_p;
	struct di_dma_fb *dma_c;
	struct di_dma_fb *di_w0;
	struct di_dma_fb *di_w1;
	struct di_dma_fb *tnr_w;
	/* dma fb resources. it is the memory that the above di_dma_fb* member point to */
	struct di_dma_fb fb_pool[DI_FB_NUM_MAX];

	struct di_md_buf md_buf;

	/* runtime context */
	wait_queue_head_t wait;
	atomic_t wait_con;
	u64 proc_fb_seqno;
	bool apply_fixed_para;
	bool para_checked;
	bool unreset;

	u8 di_detect_result;
	u64 interlace_detected_counts;
	u64 lastest_interlace_detected_frame;
	u64 interlace_detected_counts_exceed_first_p_frame;
	u64 progressive_detected_counts;
	u64 lastest_progressive_detected_frame;
	u64 progressive_detected_first_frame;

	/* dev_cdata must be at last!!! */
	uintptr_t dev_cdata;
};
#endif

void *di_client_create(const char *name);
void di_client_destroy(void *c);

int di_client_mem_request(struct di_client *c, void *data);
int di_client_mem_release(struct di_client *c, void *data);

int di_client_get_version(struct di_client *c, void *data);
int di_client_reset(struct di_client *c, void *data);
int di_client_check_para(struct di_client *c, void *data);
int di_client_set_timeout(struct di_client *c, void *data);
int di_client_set_video_size(struct di_client *c, void *data);
int di_client_set_video_crop(struct di_client *c, void *data);
int di_client_set_demo_crop(struct di_client *c, void *data);
int di_client_set_dit_mode(struct di_client *c, void *data);
int di_client_set_tnr_mode(struct di_client *c, void *data);
int di_client_set_fmd_enable(struct di_client *c, void *data);
int di_client_process_fb(struct di_client *c, void *data);
int di_client_get_tnrpara(struct di_client *c, void *data);
int di_client_set_tnrpara(struct di_client *c, void *data);

#endif /* ifndef _DI_CLIENT_H_ */
