/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * linux-4.9/drivers/media/platform/sunxi-vfe/mipi_csi/bsp_mipi_csi.h
 *
 * Copyright (c) 2007-2017 Allwinnertech Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * sunxi mipi csi bsp header file
 * Author:raymonxiu
*/

#ifndef __MIPI__CSI__H__
#define __MIPI__CSI__H__

#include "protocol.h"
#include "../bsp_common.h"
#define MAX_MIPI  1
#define MAX_MIPI_CH 4

/* enum field { */
/* FIELD_ANY           = 0, */ /*driver can choose from none, */
/* top, bottom, interlaced */
/* depending on whatever it thinks */
/* is approximate ... */
/* FIELD_NONE          = 1, */ /* this device has no fields ... */
/* FIELD_TOP           = 2, */ /* top field only */
/* FIELD_BOTTOM        = 3, */ /* bottom field only */
/* FIELD_INTERLACED    = 4, */ /* both fields interlaced */
/* FIELD_SEQ_TB        = 5, */ /* both fields sequential into one */
/* buffer, top-bottom order */
/* FIELD_SEQ_BT        = 6, */ /* same as above + bottom-top order */
/* FIELD_ALTERNATE     = 7, */ /* both fields alternating into */
/* separate buffers */
/* FIELD_INTERLACED_TB = 8, */ /* both fields interlaced, top field */
/* first and the top field is */
/* transmitted first */
/* FIELD_INTERLACED_BT = 9, */ /* both fields interlaced, top field */
/* first and the bottom field is */
/* transmitted first */
/* }; */

struct mipi_para {
  unsigned int        auto_check_bps;
  unsigned int        bps;
  unsigned int        dphy_freq;
  unsigned int        lane_num;
  unsigned int        total_rx_ch;
};

struct mipi_fmt {
  enum field          field[MAX_MIPI_CH];
  enum pkt_fmt        packet_fmt[MAX_MIPI_CH];
  unsigned int        vc[MAX_MIPI_CH];
};
extern void bsp_mipi_csi_set_version(unsigned int sel, unsigned int ver);
extern int  bsp_mipi_csi_set_base_addr(unsigned int sel, unsigned long addr_base);
extern int  bsp_mipi_dphy_set_base_addr(unsigned int sel, unsigned long addr_base);
extern void bsp_mipi_csi_dphy_init(unsigned int sel);
extern void bsp_mipi_csi_dphy_exit(unsigned int sel);
extern void bsp_mipi_csi_dphy_enable(unsigned int sel);
extern void bsp_mipi_csi_dphy_disable(unsigned int sel);
extern void bsp_mipi_csi_protocol_enable(unsigned int sel);
extern void bsp_mipi_csi_protocol_disable(unsigned int sel);
extern void bsp_mipi_csi_set_para(unsigned int sel, struct mipi_para *para);
extern void bsp_mipi_csi_set_fmt(unsigned int sel, unsigned int total_rx_ch, struct mipi_fmt *fmt);
#endif  /* __MIPI__CSI__H__ */
