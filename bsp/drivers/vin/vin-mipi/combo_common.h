/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */

/* SPDX-License-Identifier: GPL-2.0 */
/*
 * combo common header file
 *
 * Copyright (c) 2017 by Allwinnertech Co., Ltd.  http://www.allwinnertech.com
 *
 * Authors:  Zhao Wei <zhaowei@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __COMBO_COMMON__H__
#define __COMBO_COMMON__H__

#include "combo_rx/combo_rx_reg.h"

/*
 * The combo interface.
 */

#define V4L2_MBUS_SUBLVDS			7
#define V4L2_MBUS_HISPI				8

#define V4L2_MBUS_SUBLVDS_1_LANE		(1 << 0)
#define V4L2_MBUS_SUBLVDS_2_LANE		(1 << 1)
#define V4L2_MBUS_SUBLVDS_3_LANE		(1 << 2)
#define V4L2_MBUS_SUBLVDS_4_LANE		(1 << 3)
#define V4L2_MBUS_SUBLVDS_5_LANE		(1 << 4)
#define V4L2_MBUS_SUBLVDS_6_LANE		(1 << 5)
#define V4L2_MBUS_SUBLVDS_7_LANE		(1 << 6)
#define V4L2_MBUS_SUBLVDS_8_LANE		(1 << 7)
#define V4L2_MBUS_SUBLVDS_9_LANE		(1 << 8)
#define V4L2_MBUS_SUBLVDS_10_LANE		(1 << 9)
#define V4L2_MBUS_SUBLVDS_11_LANE		(1 << 10)
#define V4L2_MBUS_SUBLVDS_12_LANE		(1 << 11)

/* flag to open combo terminal resistance */
#define CMB_TERMINAL_RES	(0x80)

/* flag of phya offset */
#define CMB_PHYA_OFFSET0	(0x00)
#define CMB_PHYA_OFFSET1	(0x10)
#define CMB_PHYA_OFFSET2	(0x20)
#define CMB_PHYA_OFFSET3	(0x30)

enum combo_mipi_mode {
	MIPI_NORMAL_MODE,
	MIPI_VC_WDR_MODE,
	MIPI_DOL_WDR_MODE,
};

enum combo_lvds_mode {
	LVDS_NORMAL_MODE,
	LVDS_4CODE_WDR_MODE,
	LVDS_5CODE_WDR_MODE,
};

enum combo_hispi_mode {
	HISPI_NORMAL_MODE,
	HISPI_WDR_MODE,
};

enum isp_wdr_mode {
	ISP_NORMAL_MODE = 0,
	ISP_DOL_WDR_MODE,
	ISP_COMANDING_MODE,
	ISP_SEHDR_MODE,
	ISP_3FDOL_WDR_MODE,

};

enum sensor_lp_mode {
	SENSOR_LP_CONTINUOUS,
	SENSOR_LP_DISCONTINUOUS,
};

struct combo_wdr_cfg {
	unsigned int line_code_mode;/* 0:HiSPI SOF/EOF/SOL/EOL 1:SAV-EAV */
	unsigned int pix_lsb;/* 0:MSB,1:LSB */
	unsigned int line_cnt;/* when in WDR mode,this reg can extent frame valid signal by set 1,2,3,4 */

	unsigned int wdr_fid_mode_sel;/* 0:1bit 1:2bits */
	unsigned int wdr_fid_map_en;/* bit12:FID0 bit13:FID1 bit14:FID2 bit15:FID3 */
	unsigned int wdr_fid0_map_sel;
	unsigned int wdr_fid1_map_sel;
	unsigned int wdr_fid2_map_sel;
	unsigned int wdr_fid3_map_sel;

	unsigned int wdr_en_multi_ch;
	unsigned int wdr_ch0_height;
	unsigned int wdr_ch1_height;
	unsigned int wdr_ch2_height;
	unsigned int wdr_ch3_height;

	unsigned int wdr_eof_fild;
	unsigned int wdr_sof_fild;
	unsigned int code_mask;
};

#endif /* __COMBO_COMMON__H__ */
