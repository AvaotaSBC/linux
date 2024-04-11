/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * A V4L2 driver for nvp6324 cameras and AHD Coax protocol.
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Li Huiyu <lihuiyu@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _JAGUAR1_VIDEO_
#define _JAGUAR1_VIDEO_

#include "jaguar1_common.h"


#define USE_1080P   1
#define USE_720P    2

/* ===============================================
 * APP -> DRV
 * =============================================== */
typedef struct _video_input_init {
	unsigned char ch;
	unsigned char format;
	unsigned char dist;
	unsigned char input;
	unsigned char val;
	unsigned char interface;
} video_input_init;

typedef struct _video_init_all {
	video_input_init ch_param[4];
} video_init_all;

typedef struct _video_output_init {
	unsigned char format;
	unsigned char port;
	unsigned char out_ch;
	unsigned char interface;
} video_output_init;

typedef struct _video_video_loss_s {
	unsigned char devnum;
	unsigned char videoloss;
	unsigned char reserve2;
} video_video_loss_s;

void vd_jaguar1_init_set(void *p_param);
void vd_jaguar1_vo_ch_seq_set(void *p_param);
void vd_jaguar1_eq_set(void *p_param);
void vd_jaguar1_sw_reset(void *p_param);
void vd_jaguar1_get_novideo(video_video_loss_s *vidloss);

void current_bank_set(unsigned char bank);
unsigned char current_bank_get(void);
void vd_register_set(int dev, unsigned char bank, unsigned char addr, unsigned char val, int pos, int size);
void reg_val_print_flag_set(int set);
#endif
/********************************************************************
 *  End of file
 ********************************************************************/
