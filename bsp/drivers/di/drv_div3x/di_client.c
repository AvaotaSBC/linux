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

#include "di_debug.h"
#include "di_client.h"
#include "di_dev.h"
#include "di_driver.h"
#include "di_utils.h"
#include "sunxi_di.h"
#include <linux/kernel.h>
#include <linux/slab.h>

#define DI_TNR_BUF_ALIGN_LEN 16
#define DI_MD_BUF_ALIGN_LEN 32

static int di_client_alloc_mbuf(struct di_mapped_buf **mbuf, u32 size)
{
	struct di_mapped_buf *p = *mbuf;

	if (p != NULL) {
		u32 size_alloced = PAGE_ALIGN(size);

		if (p->size_alloced != size_alloced) {
			di_dma_buf_unmap_free(p);
			p = NULL;
			*mbuf = NULL;
		} else {
			p->size_request = size;
		}
	}
	if (p == NULL) {
		p = di_dma_buf_alloc_map(size);
		if (p == NULL)
			return -ENOMEM;
	}
	memset((void *)p->vir_addr, 0, p->size_alloced);
	*mbuf = p;

	return 0;
}

static int di_client_setup_md_buf(struct di_client *c)
{
	int ret = 0;
	u32 w_bit = c->video_size.width * 2;
	/* each byte of in/out_flag save MD flag of 4 pixel
	 * and stride is request for align with 128
	 * w_stride = (c->video_size.width + (128-1)) >>2 ) & ~128;
	 */
	u32 w_stride = ALIGN(w_bit, DI_MD_BUF_ALIGN_LEN * 8) / 8;
	u32 h = c->video_size.height;
	u32 size = h * w_stride;

	//TODO check
	if (c->mode == DI_MODE_TNR) {
		size = c->video_size.width * c->video_size.height;
		DI_DEBUG("%s [%dx%d] w_stride=%d w_stride*h=%d  w*h=%d size=%d", __func__,
			 c->video_size.width, c->video_size.height, w_stride, w_stride*h,
			 c->video_size.width * c->video_size.height, size);
	}

	ret = di_client_alloc_mbuf(&(c->md_buf.mbuf[0]), size);
	if (ret)
		return ret;
	ret = di_client_alloc_mbuf(&(c->md_buf.mbuf[1]), size);
	if (ret) {
		if (c->md_buf.mbuf[0]) {
			di_dma_buf_unmap_free(c->md_buf.mbuf[0]);
			c->md_buf.mbuf[0] = NULL;
		}
		return ret;
	}

	c->md_buf.dir = 0;
	c->md_buf.w_bit = w_bit;
	c->md_buf.w_stride = w_stride;
	c->md_buf.h = h;

	return 0;
}

int di_client_reset(struct di_client *c, void *data)
{
	u32 i;

	DI_DEBUG("%s: %s\n", c->name, __func__);

	c->para_checked = false;
	c->unreset = false;
	c->proc_fb_seqno = 0;
	c->di_detect_result = 0;
	c->interlace_detected_counts = 0;
	c->lastest_interlace_detected_frame = 0;
	c->interlace_detected_counts_exceed_first_p_frame = 0;
	c->progressive_detected_counts = 0;
	c->progressive_detected_first_frame = 0;
	c->lastest_progressive_detected_frame = 0;
	c->apply_fixed_para = true;
	atomic_set(&c->wait_con, DI_PROC_STATE_IDLE);

	c->in_fb0 = c->in_fb1 = c->in_fb2 =
#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
	c->in_fb0_nf = c->in_fb1_nf = c->in_fb2_nf =
#endif
		c->out_dit_fb0 = c->out_dit_fb1 = c->out_tnr_fb0 =
		c->dma_di = c->dma_p = c->dma_c =
		c->di_w0 = c->di_w1 = c->tnr_w = NULL;
	for (i = 0; i < ARRAY_SIZE(c->fb_pool); i++) {
		if (c->fb_pool[i].dma_item) {
			di_dma_buf_self_unmap(c->fb_pool[i].dma_item);
			c->fb_pool[i].dma_item = NULL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(c->md_buf.mbuf); i++) {
		if (c->md_buf.mbuf[i]) {
			di_dma_buf_unmap_free(c->md_buf.mbuf[i]);
			c->md_buf.mbuf[i] = NULL;
		}
	}

	di_dev_reset_cdata((void *)c->dev_cdata);

	return 0;
}

/* checking and correcting di_client para before running proccess_fb */
#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
int di_client_check_para(struct di_client *c, void *data)
{
	DI_DEBUG("%s: %s\n", c->name, __func__);

	if (c->para_checked == true)
		return 0;

	if (c->unreset == true) {
		DI_ERR("%s: do reset before setting, then check\n", c->name);
		return -EINVAL;
	}

	c->proc_fb_seqno = 0;
	c->di_detect_result = 0;
	c->interlace_detected_counts = 0;
	c->lastest_interlace_detected_frame = 0;
	c->interlace_detected_counts_exceed_first_p_frame = 0;
	c->progressive_detected_counts = 0;
	c->progressive_detected_first_frame = 0;
	c->lastest_progressive_detected_frame = 0;
	c->apply_fixed_para = true;

	if ((c->video_size.height == 0)
		|| ((c->video_size.height & 0x1) != 0)
		|| (c->video_size.width == 0)
		|| ((c->video_size.width & 0x1) != 0)) {
		DI_ERR("%s: invalid size W(%d)xH(%d)\n",
			c->name, c->video_size.height, c->video_size.width);
		goto err_out;
	}

	if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_MOTION)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_2FRAME)
		/* && (c->tnr_mode.mode != DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en != 0) */) {
		DI_DEBUG("%s: this is 60hz mode\n", c->name);
		c->mode = DI_MODE_60HZ;
		c->md_en = true;
		if (c->tnr_mode.mode)
			c->tnr_en = true;
		else
			c->tnr_en = false;
		c->vof_buf_en = true;
		c->dma_di = c->in_fb0 = &c->fb_pool[0];
		c->dma_di_nf = c->in_fb0_nf = &c->fb_pool[1];
		c->dma_p = c->in_fb1 = &c->fb_pool[2];
		c->dma_p_nf = c->in_fb1_nf = &c->fb_pool[3];
		c->dma_c = c->in_fb2 = &c->fb_pool[4];
		c->dma_c_nf = c->in_fb2_nf = &c->fb_pool[5];
		c->di_w0 = c->out_dit_fb0 = &c->fb_pool[6];
		c->di_w1 = c->out_dit_fb1 = &c->fb_pool[7];
		c->tnr_w = c->out_tnr_fb0 = &c->fb_pool[8];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_MOTION)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_1FRAME)
		/* && (c->tnr_mode.mode != DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en != 0) */) {
		DI_DEBUG("%s: this is 30hz mode\n", c->name);
		c->mode = DI_MODE_30HZ;
		c->md_en = true;
		if (c->tnr_mode.mode)
			c->tnr_en = true;
		else
			c->tnr_en = false;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->dma_p_nf = c->in_fb0_nf = &c->fb_pool[1];
		c->dma_c = c->in_fb1 = &c->fb_pool[2];
		c->dma_c_nf = c->in_fb1_nf = &c->fb_pool[3];
		c->di_w1 = c->out_dit_fb0 = &c->fb_pool[4];
		c->tnr_w = c->out_tnr_fb0 = &c->fb_pool[5];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_BOB)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_1FRAME)
		&& (c->tnr_mode.mode == DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en == 0)) {
		DI_DEBUG("%s: this is bob mode\n", c->name);
		c->mode = DI_MODE_BOB;
		c->md_en = false;
		c->tnr_en = false;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->di_w1 = c->out_dit_fb0 = &c->fb_pool[1];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_WEAVE)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_1FRAME)
		&& (c->tnr_mode.mode == DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en == 0)) {
		DI_DEBUG("%s: this is weave mode\n", c->name);
		c->mode = DI_MODE_WEAVE;
		c->md_en = false;
		c->tnr_en = false;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->di_w1 = c->out_dit_fb0 = &c->fb_pool[1];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_INVALID)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_0FRAME)
		&& (c->tnr_mode.mode != DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en == 0)) {
		DI_DEBUG("%s: this is only-tnr mode\n", c->name);
		c->mode = DI_MODE_TNR;
		c->md_en = true;
		c->tnr_en = true;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->dma_c = c->in_fb1 = &c->fb_pool[1];
		c->tnr_w = c->out_tnr_fb0 = &c->fb_pool[2];
	} else {
		DI_ERR("%s: wrong paras:\n  "
			"dit_mode:%d,%d; tnr:%d,%d; fmd_en:%d\n",
			c->name,
			c->dit_mode.intp_mode, c->dit_mode.out_frame_mode,
			c->tnr_mode.mode, c->tnr_mode.level, c->fmd_en.en);
		goto err_out;
	}

	if (c->md_en)
		if (di_client_setup_md_buf(c))
			goto err_out;

	atomic_set(&c->wait_con, DI_PROC_STATE_IDLE);
	di_dev_reset_cdata((void *)c->dev_cdata);

	c->para_checked = true;
	c->unreset = true;

	return 0;

err_out:
	c->para_checked = false;
	return -EINVAL;
}
#else
int di_client_check_para(struct di_client *c, void *data)
{
	DI_DEBUG("%s: %s\n", c->name, __func__);

	if (c->para_checked == true)
		return 0;

	if (c->unreset == true) {
		DI_ERR("%s: do reset before setting, then check\n", c->name);
		return -EINVAL;
	}

	c->proc_fb_seqno = 0;
	c->di_detect_result = 0;
	c->interlace_detected_counts = 0;
	c->lastest_interlace_detected_frame = 0;
	c->interlace_detected_counts_exceed_first_p_frame = 0;
	c->progressive_detected_counts = 0;
	c->progressive_detected_first_frame = 0;
	c->lastest_progressive_detected_frame = 0;
	c->apply_fixed_para = true;

	if ((c->video_size.height == 0)
		|| ((c->video_size.height & 0x1) != 0)
		|| (c->video_size.width == 0)
		|| ((c->video_size.width & 0x1) != 0)) {
		DI_ERR("%s: invalid size W(%d)xH(%d)\n",
			c->name, c->video_size.height, c->video_size.width);
		goto err_out;
	}

	if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_MOTION)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_2FRAME)
		/* && (c->tnr_mode.mode != DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en != 0) */) {
		DI_DEBUG("%s: this is 60hz mode\n", c->name);
		c->mode = DI_MODE_60HZ;
		c->md_en = true;
		if (c->tnr_mode.mode)
			c->tnr_en = true;
		else
			c->tnr_en = false;
		c->vof_buf_en = true;
		c->dma_di = c->in_fb0 = &c->fb_pool[0];
		c->dma_p = c->in_fb1 = &c->fb_pool[1];
		c->dma_c = c->in_fb2 = &c->fb_pool[2];
		c->di_w0 = c->out_dit_fb0 = &c->fb_pool[3];
		c->di_w1 = c->out_dit_fb1 = &c->fb_pool[4];
		c->tnr_w = c->out_tnr_fb0 = &c->fb_pool[5];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_MOTION)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_1FRAME)
		/* && (c->tnr_mode.mode != DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en != 0) */) {
		DI_DEBUG("%s: this is 30hz mode\n", c->name);
		c->mode = DI_MODE_30HZ;
		c->md_en = true;
		if (c->tnr_mode.mode)
			c->tnr_en = true;
		else
			c->tnr_en = false;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->dma_c = c->in_fb1 = &c->fb_pool[1];
		c->di_w1 = c->out_dit_fb0 = &c->fb_pool[2];
		c->tnr_w = c->out_tnr_fb0 = &c->fb_pool[3];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_BOB)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_1FRAME)
		&& (c->tnr_mode.mode == DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en == 0)) {
		DI_DEBUG("%s: this is bob mode\n", c->name);
		c->mode = DI_MODE_BOB;
		c->md_en = false;
		c->tnr_en = false;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->di_w1 = c->out_dit_fb0 = &c->fb_pool[1];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_WEAVE)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_1FRAME)
		&& (c->tnr_mode.mode == DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en == 0)) {
		DI_DEBUG("%s: this is weave mode\n", c->name);
		c->mode = DI_MODE_WEAVE;
		c->md_en = false;
		c->tnr_en = false;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->di_w1 = c->out_dit_fb0 = &c->fb_pool[1];
	} else if ((c->dit_mode.intp_mode == DI_DIT_INTP_MODE_INVALID)
		&& (c->dit_mode.out_frame_mode == DI_DIT_OUT_0FRAME)
		&& (c->tnr_mode.mode != DI_TNR_MODE_INVALID)
		&& (c->fmd_en.en == 0)) {
		DI_DEBUG("%s: this is only-tnr mode\n", c->name);
		c->mode = DI_MODE_TNR;
		c->md_en = true;
		c->tnr_en = true;
		c->vof_buf_en = false;
		c->dma_p = c->in_fb0 = &c->fb_pool[0];
		c->dma_c = c->in_fb1 = &c->fb_pool[1];
		c->tnr_w = c->out_tnr_fb0 = &c->fb_pool[2];
	} else {
		DI_ERR("%s: wrong paras:\n  "
			"dit_mode:%d,%d; tnr:%d,%d; fmd_en:%d\n",
			c->name,
			c->dit_mode.intp_mode, c->dit_mode.out_frame_mode,
			c->tnr_mode.mode, c->tnr_mode.level, c->fmd_en.en);
		goto err_out;
	}

	if (c->md_en)
		if (di_client_setup_md_buf(c))
			goto err_out;

	atomic_set(&c->wait_con, DI_PROC_STATE_IDLE);
	di_dev_reset_cdata((void *)c->dev_cdata);

	c->para_checked = true;
	c->unreset = true;

	return 0;

err_out:
	c->para_checked = false;
	return -EINVAL;
}
#endif

static bool di_client_check_fb_arg(struct di_client *c,
	struct di_process_fb_arg *fb_arg)
{
	DI_DEBUG("%s pulldown[%s] topFieldFirst[%s] baseFiled[%s]\n",
		fb_arg->is_interlace ? "Interlace" : "P",
		fb_arg->is_pulldown ? "Y" : "N",
		fb_arg->top_field_first ? "Y" : "N",
		fb_arg->base_field ? "TOP" : "BOTTOM");
	DI_DEBUG("out fb0 info:format:%s dma_buf_fd:%d "
	"buf:(y_addr:0x%llx)(cb_addr:0x%llx)(cr_addr:0x%llx)(ystride:%d)(cstride:%d) "
	"size:(%dx%d)\n",
	di_format_to_string(fb_arg->out_dit_fb0.format), fb_arg->out_dit_fb0.dma_buf_fd,
	fb_arg->out_dit_fb0.buf.y_addr, fb_arg->out_dit_fb0.buf.cb_addr, fb_arg->out_dit_fb0.buf.cr_addr,
	fb_arg->out_dit_fb0.buf.ystride, fb_arg->out_dit_fb0.buf.cstride,
	fb_arg->out_dit_fb0.size.width, fb_arg->out_dit_fb0.size.height);

	DI_DEBUG("out fb1 info:format:%s dma_buf_fd:%d "
	"buf:(y_addr:0x%llx)(cb_addr:0x%llx)(cr_addr:0x%llx)(ystride:%d)(cstride:%d)"
	"size:(%dx%d)\n",
	di_format_to_string(fb_arg->out_dit_fb1.format), fb_arg->out_dit_fb1.dma_buf_fd,
	fb_arg->out_dit_fb1.buf.y_addr, fb_arg->out_dit_fb1.buf.cb_addr, fb_arg->out_dit_fb1.buf.cr_addr,
	fb_arg->out_dit_fb1.buf.ystride, fb_arg->out_dit_fb1.buf.cstride,
	fb_arg->out_dit_fb1.size.width, fb_arg->out_dit_fb1.size.height);

	DI_DEBUG("out tnr info:format:%s dma_buf_fd:%d "
	"buf:(y_addr:0x%llx)(cb_addr:0x%llx)(cr_addr:0x%llx)(ystride:%d)(cstride:%d)"
	"size:(%dx%d)\n",
	di_format_to_string(fb_arg->out_tnr_fb0.format), fb_arg->out_tnr_fb0.dma_buf_fd,
	fb_arg->out_tnr_fb0.buf.y_addr, fb_arg->out_tnr_fb0.buf.cb_addr, fb_arg->out_tnr_fb0.buf.cr_addr,
	fb_arg->out_tnr_fb0.buf.ystride, fb_arg->out_tnr_fb0.buf.cstride,
	fb_arg->out_tnr_fb0.size.width, fb_arg->out_tnr_fb0.size.height);

	/* TODO: add more check ? */
	return true;
}

static int di_client_get_fb(struct di_client *c,
	struct di_dma_fb *dma_fb, struct di_fb *fb,
	enum dma_data_direction dir)
{
#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
	DI_DEBUG("%s: type:%s buf_addr[0x%llx,0x%llx,0x%llx],"
		"buf_stride[%d,%d],fmt=%s,fd=%d,size[%d,%d]\n",
		c->name,
		fb->field_type ? (fb->field_type == DI_FIELD_TYPE_TOP_FIELD ? "top field" : "bottom field") : "combine field",
		fb->buf.y_addr, fb->buf.cb_addr,
		fb->buf.cr_addr,
		fb->buf.ystride, fb->buf.cstride,
		di_format_to_string(fb->format),
		fb->dma_buf_fd,
		fb->size.width, fb->size.height);
#else
	DI_DEBUG("%s: buf_addr[0x%llx,0x%llx,0x%llx],"
		"buf_stride[%d,%d],fmt=%s,fd=%d,size[%d,%d]\n",
		c->name,
		fb->buf.y_addr, fb->buf.cb_addr,
		fb->buf.cr_addr,
		fb->buf.ystride, fb->buf.cstride,
		di_format_to_string(fb->format),
		fb->dma_buf_fd,
		fb->size.width, fb->size.height);
#endif
	if (dma_fb->dma_item != NULL) {
		di_dma_buf_self_unmap(dma_fb->dma_item);
		dma_fb->dma_item = NULL;
	}
	if (fb->dma_buf_fd >= 0) {
		dma_fb->dma_item = di_dma_buf_self_map(fb->dma_buf_fd, dir);
		if (dma_fb->dma_item == NULL) {
			DI_ERR("%s: %s,%d\n", c->name, __func__, __LINE__);
			return -EINVAL;
		}
		fb->buf.y_addr +=
			(u64)(dma_fb->dma_item->dma_addr);
		if (fb->buf.cb_addr)
			fb->buf.cb_addr += (u64)(dma_fb->dma_item->dma_addr);
		if (fb->buf.cr_addr)
			fb->buf.cr_addr += (u64)(dma_fb->dma_item->dma_addr);
		DI_DEBUG("%s:dma_addr=0x%llx,yuv[0x%llx,0x%llx,0x%llx]\n",
			c->name, (u64)(dma_fb->dma_item->dma_addr),
			fb->buf.y_addr, fb->buf.cb_addr, fb->buf.cr_addr);
	} else {
		DI_DEBUG("%s: On phy_addr_buf method\n", c->name);
	}
	dma_fb->fb = fb;

	return 0;
}

static int di_client_get_fbs(struct di_client *c)
{
	struct di_process_fb_arg *fb_arg = &c->fb_arg;

	if ((c->in_fb0 != NULL)
		&& di_client_get_fb(c, c->in_fb0,
			&fb_arg->in_fb0, DMA_TO_DEVICE))
		return -EINVAL;
#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
	if ((c->in_fb0_nf != NULL) && (c->in_fb0 != NULL)
		&& c->in_fb0->fb
		&& c->in_fb0->fb->field_type
		&& di_client_get_fb(c, c->in_fb0_nf,
			&fb_arg->in_fb0_nf, DMA_TO_DEVICE))
		return -EINVAL;
#endif
	if ((c->in_fb1 != NULL)
		&& di_client_get_fb(c, c->in_fb1,
			&fb_arg->in_fb1, DMA_TO_DEVICE))
		return -EINVAL;
#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
	if ((c->in_fb1_nf != NULL) && (c->in_fb1 != NULL)
		&& c->in_fb1->fb
		&& c->in_fb1->fb->field_type
		&& di_client_get_fb(c, c->in_fb1_nf,
			&fb_arg->in_fb1_nf, DMA_TO_DEVICE))
		return -EINVAL;
#endif
	if ((c->in_fb2 != NULL)
		&& di_client_get_fb(c, c->in_fb2,
			&fb_arg->in_fb2, DMA_TO_DEVICE))
		return -EINVAL;
#if IS_ENABLED(CONFIG_SUNXI_DI_SINGEL_FILE)
	if ((c->in_fb2_nf != NULL) && (c->in_fb2 != NULL)
		&& c->in_fb2->fb
		&& c->in_fb2->fb->field_type
		&& di_client_get_fb(c, c->in_fb2_nf,
			&fb_arg->in_fb2_nf, DMA_TO_DEVICE))
		return -EINVAL;
#endif
	if ((c->out_dit_fb0 != NULL)
		&& di_client_get_fb(c, c->out_dit_fb0,
			&fb_arg->out_dit_fb0, DMA_FROM_DEVICE))
		return -EINVAL;
	if ((c->out_dit_fb1 != NULL)
		&& di_client_get_fb(c, c->out_dit_fb1,
			&fb_arg->out_dit_fb1, DMA_FROM_DEVICE))
		return -EINVAL;

	if ((c->out_tnr_fb0 != NULL)
		&& (c->tnr_mode.mode > 0)) {
		if (di_client_get_fb(c, c->out_tnr_fb0,
				&fb_arg->out_tnr_fb0, DMA_FROM_DEVICE))
			return -EINVAL;

		/* NOTE: when use tnr, the output format must be planner */
		if (di_format_get_planar_num(c->out_tnr_fb0->fb->format) != 3) {
			DI_ERR("%s: invalid %s(%d) for out_tnr_fb0\n", c->name,
				di_format_to_string(c->out_tnr_fb0->fb->format),
				c->out_tnr_fb0->fb->format);
			return -EINVAL;
		}

		/* NOTE: when use tnr, the output format must be planner */
		if ((c->out_dit_fb0 != NULL)
			&& (di_format_get_planar_num(
				c->out_dit_fb0->fb->format) != 3)) {
			DI_ERR("%s: invalid %s(%d) for out_dit_fb0\n", c->name,
				di_format_to_string(c->out_dit_fb0->fb->format),
				c->out_dit_fb0->fb->format);
			return -EINVAL;
		}

		/* NOTE: when use tnr, the output format must be planner */
		if ((c->out_dit_fb1 != NULL)
			&& (di_format_get_planar_num(
				c->out_dit_fb1->fb->format) != 3)) {
			DI_ERR("%s: invalid %s(%d) for out_dit_fb1\n", c->name,
				di_format_to_string(c->out_dit_fb1->fb->format),
				c->out_dit_fb1->fb->format);
			return -EINVAL;
		}
	}

	return 0;
}

static int di_client_put_fbs(struct di_client *c)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(c->fb_pool); i++) {
		struct di_dma_fb *fb = &(c->fb_pool[i]);

		fb->fb = NULL;
		if (fb->dma_item != NULL) {
			di_dma_buf_self_unmap(fb->dma_item);
			fb->dma_item = NULL;
		}
	}
	return 0;
}

int di_client_process_fb(struct di_client *c, void *data)
{
	int ret = 0;
	ktime_t time;
	unsigned long long t0 = 0, t1 = 0, t2 = 0, t3 = 0;
	struct di_process_fb_arg *fb_arg = (struct di_process_fb_arg *)data;

	time = ktime_get();
	t0 = ktime_to_us(time);
	if (c->para_checked == false) {
		DI_ERR("%s: para unchecked\n", c->name);
		return -EINVAL;
	}

	if (!di_client_check_fb_arg(c, fb_arg)) {
		DI_ERR("%s: check_fb_arg fail\n", c->name);
		return -EINVAL;
	}
	memcpy((void *)&c->fb_arg, fb_arg, sizeof(c->fb_arg));

	ret = di_client_get_fbs(c);

	time = ktime_get();
	t1 = ktime_to_us(time);

	if (!ret)
		ret = di_drv_process_fb(c);

	if ((c->mode == DI_MODE_60HZ
		|| c->mode == DI_MODE_30HZ
		|| c->mode == DI_MODE_WEAVE)
		&& c->di_detect_result == DI_DETECT_PROGRESSIVE) {
		DI_DEBUG("di detect result:progressive, di_mode:%s\n",
					c->mode == DI_MODE_60HZ ? "60HZ" :
					c->mode == DI_MODE_30HZ ? "30HZ" : "weave");
		if (!c->progressive_detected_counts)
			c->progressive_detected_first_frame = c->proc_fb_seqno;
		if (c->progressive_detected_counts == 0xffffffffffffffff)
			c->interlace_detected_counts = 0;
		c->progressive_detected_counts++;
		c->lastest_progressive_detected_frame = c->proc_fb_seqno;
		ret = FB_PROCESS_ERROR_INTERLACE_TYPE;
	} else {
		DI_DEBUG("di detect result:interlace, di_mode:%s\n",
					c->mode == DI_MODE_60HZ ? "60HZ" :
					c->mode == DI_MODE_30HZ ? "30HZ" :
					c->mode == DI_MODE_BOB ? "bob" :
					c->mode == DI_MODE_TNR ? "tnr only" :
					"weave");
		c->interlace_detected_counts++;
		c->lastest_interlace_detected_frame = c->proc_fb_seqno;
		if (c->progressive_detected_counts
			&& (c->interlace_detected_counts
				> c->progressive_detected_counts))
			c->interlace_detected_counts_exceed_first_p_frame++;
	}

	time = ktime_get();
	t2 = ktime_to_us(time);

	di_client_put_fbs(c);

	time = ktime_get();
	t3 = ktime_to_us(time);

	DI_DEBUG("total:%lluus     t0~t1:%lluus  t1~t2:%lluus  t2~t3:%lluus\n",
				(t3 - t0),
				(t1 - t0),
				(t2 - t1),
				(t3 - t2));
	return ret;
}

int di_client_set_video_size(struct di_client *c, void *data)
{
	struct di_size *size = (struct di_size *)data;

	DI_DEBUG("%s: video_size[%d x %d]\n", c->name,
		size->width, size->height);

	c->video_size.width = size->width;
	c->video_size.height = size->height;
	return 0;
}

int di_client_set_video_crop(struct di_client *c, void *data)
{
	struct di_rect *rect = (struct di_rect *)data;

	DI_DEBUG("%s: video_crop:(%u, %u)  (%u, %u)\n", c->name,
		rect->left, rect->top, rect->right, rect->bottom);

	c->dit_out_crop.left = rect->left;
	c->dit_out_crop.top = rect->top;
	c->dit_out_crop.right = rect->right;
	c->dit_out_crop.bottom = rect->bottom;

	memcpy((void *)&c->md_out_crop, (void *)&c->dit_out_crop,
		sizeof(c->md_out_crop));
	memcpy((void *)&c->fmd_out_crop, (void *)&c->dit_out_crop,
		sizeof(c->fmd_out_crop));
	memcpy((void *)&c->tnr_out_crop, (void *)&c->dit_out_crop,
		sizeof(c->tnr_out_crop));
	memcpy((void *)&c->dit_demo_crop, (void *)&c->dit_out_crop,
		sizeof(c->dit_demo_crop));
	memcpy((void *)&c->tnr_demo_crop, (void *)&c->dit_out_crop,
		sizeof(c->tnr_demo_crop));

	return 0;
}

int di_client_set_demo_crop(struct di_client *c, void *data)
{
	struct di_demo_crop_arg *demo_arg = (struct di_demo_crop_arg *)data;

	DI_DEBUG("%s: demo crop: dit:(%u, %u)  (%u, %u) "
		"tnr: (%u, %u)  (%u, %u)\n", c->name,
		demo_arg->dit_demo.left, demo_arg->dit_demo.top,
		demo_arg->dit_demo.right, demo_arg->dit_demo.bottom,
		demo_arg->tnr_demo.left, demo_arg->tnr_demo.top,
		demo_arg->tnr_demo.right, demo_arg->tnr_demo.bottom);

	demo_arg->dit_demo.left = demo_arg->dit_demo.left
		- (demo_arg->dit_demo.left % 4);
	demo_arg->dit_demo.top = demo_arg->dit_demo.top
		- (demo_arg->dit_demo.top % 4);
	demo_arg->dit_demo.right = demo_arg->dit_demo.right
		- (demo_arg->dit_demo.right % 4);
	demo_arg->dit_demo.bottom = demo_arg->dit_demo.bottom
		- (demo_arg->dit_demo.bottom % 4);

	demo_arg->tnr_demo.left = demo_arg->tnr_demo.left
		- (demo_arg->tnr_demo.left % 4);
	demo_arg->tnr_demo.top = demo_arg->tnr_demo.top
		- (demo_arg->tnr_demo.top % 4);
	demo_arg->tnr_demo.right = demo_arg->tnr_demo.right
		- (demo_arg->tnr_demo.right % 4);
	demo_arg->tnr_demo.bottom = demo_arg->tnr_demo.bottom
		- (demo_arg->tnr_demo.bottom % 4);

	memcpy((void *)&c->dit_demo_crop, (void *)&demo_arg->dit_demo,
		sizeof(c->dit_demo_crop));
	memcpy((void *)&c->tnr_demo_crop, (void *)&demo_arg->tnr_demo,
		sizeof(c->tnr_demo_crop));

	return 0;
}

int di_client_set_dit_mode(struct di_client *c, void *data)
{
	struct di_dit_mode *mode = (struct di_dit_mode *)data;

	DI_DEBUG("%s: dit_mode: intp_mode=%d, out_frame_mode=%d\n",
		c->name, mode->intp_mode, mode->out_frame_mode);

	c->dit_mode.intp_mode = mode->intp_mode;
	c->dit_mode.out_frame_mode = mode->out_frame_mode;
	return 0;
}

int di_client_set_tnr_mode(struct di_client *c, void *data)
{
	struct di_tnr_mode *mode = (struct di_tnr_mode *)data;

	DI_DEBUG("%s: tnr_mode: mode=%d, level=%d\n",
		c->name, mode->mode, mode->level);

	c->tnr_mode.mode = mode->mode;
	c->tnr_mode.level = mode->level;
	return 0;
}

int di_client_set_fmd_enable(struct di_client *c, void *data)
{
	struct di_fmd_enable *en = (struct di_fmd_enable *)data;

	DI_DEBUG("%s: fmd_en: en=%d\n", c->name, en->en);

	c->fmd_en.en = en->en;
	return 0;
}

int di_client_get_version(struct di_client *c, void *data)
{
	struct di_version *version = (struct di_version *)data;
	return di_drv_get_version(version);
}

int di_client_set_timeout(struct di_client *c, void *data)
{
	struct di_timeout_ns *timeout = (struct di_timeout_ns *)data;
	DI_DEBUG("%s:wait4start=%llu,wait4finish=%llu\n",
		c->name, timeout->wait4start, timeout->wait4finish);
	if (timeout->wait4start > 0)
		c->timeout.wait4start = timeout->wait4start;
	if (timeout->wait4finish > 0)
		c->timeout.wait4finish = timeout->wait4finish;

	return 0;
}

void *di_client_create(const char *name)
{
	struct di_client *client;

	if (!name) {
		DI_ERR("%s: Name cannot be null\n", __func__);
		return NULL;
	}

	client = kzalloc(sizeof(*client) + di_dev_get_cdata_size(),
		GFP_KERNEL);
	if (client == NULL) {
		DI_ERR("kzalloc for client%s fail\n", name);
		return NULL;
	}

	client->name = kstrdup(name, GFP_KERNEL);
	if (client->name == NULL) {
		kfree(client);
		DI_ERR("kstrdup for name(%s) fail\n", name);
		return NULL;
	}

	INIT_LIST_HEAD(&client->node);

	client->timeout.wait4start = 3 * 1000000000UL;
	client->timeout.wait4finish = 3 * 1000000000UL;

	init_waitqueue_head(&client->wait);
	atomic_set(&client->wait_con, DI_PROC_STATE_IDLE);

	client->apply_fixed_para = true;

	client->dev_cdata = (uintptr_t)(
		(char *)&client->dev_cdata + sizeof(client->dev_cdata));

	if (di_drv_client_inc(client)) {
		kfree(client);
		return NULL;
	}

	return client;
}
EXPORT_SYMBOL_GPL(di_client_create);

void di_client_destroy(void *client)
{
	struct di_client *c = (struct di_client *)client;
	u32 i;

	if (!di_drv_is_valid_client(c)) {
		DI_ERR("%s, invalid client(%p)\n", __func__, c);
		return;
	}
	di_drv_client_dec(c);

	for (i = 0; i < ARRAY_SIZE(c->fb_pool); i++)
		if (c->fb_pool[i].dma_item)
			di_dma_buf_self_unmap(c->fb_pool[i].dma_item);

	for (i = 0; i < ARRAY_SIZE(c->md_buf.mbuf); i++)
		if (c->md_buf.mbuf[i])
			di_dma_buf_unmap_free(c->md_buf.mbuf[i]);

	di_dev_reset_cdata((void *)c->dev_cdata);

	kfree(c->name);
	kfree(c);
}
EXPORT_SYMBOL_GPL(di_client_destroy);

int di_client_mem_request(struct di_client *c, void *data)
{
	struct di_mem_arg *mem = (struct di_mem_arg *)data;
	int handle;

	handle = di_mem_request(mem->size, &mem->phys_addr);
	if (handle < 0)
		return -1;

	mem->handle = (unsigned int)handle;

	return 0;
}
EXPORT_SYMBOL_GPL(di_client_mem_request);

int di_client_mem_release(struct di_client *c, void *data)
{
	struct di_mem_arg *mem = (struct di_mem_arg *)data;

	return mem->handle = di_mem_release(mem->handle);
}
EXPORT_SYMBOL_GPL(di_client_mem_release);

int di_client_get_tnrpara(__maybe_unused struct di_client *c, void *data)
{
	struct tnr_module_param_t *para = (struct tnr_module_param_t *)data;
	int ret = -1;

	ret = di_dev_get_tnrpara(para);
	if (ret != 0)
		DI_ERR("%s, di_dev_get_tnrpara process fail\n", __func__);

	return ret;
}

int di_client_set_tnrpara(__maybe_unused struct di_client *c, void *data)
{
	struct tnr_module_param_t *para = (struct tnr_module_param_t *)data;
	int ret = -1;

	ret = di_dev_set_tnrpara(para);
	if (ret != 0)
		DI_ERR("%s, di_dev_set_tnrpara process fail\n", __func__);

	return ret;
}
