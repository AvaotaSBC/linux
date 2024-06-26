/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * A V4L2 driver for nvp6134 cameras and AHD Coax protocol.
 *
 * Copyright (C) 2016 	NEXTCHIP Inc. All rights reserved.
 * Description	: communicate between Decoder and ISP
 * 				  get information(AGC, motion, FPS) of ISP
 * 				  set NRT/RT
 * 				  upgrade Firmware of ISP
 * Authors:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ACP_NVP6134_H__
#define __ACP_NVP6134_H__

/********************************************************************
 *  define and enum
 ********************************************************************/
/* init ACP buffer flag */
#define ACP_INIT_ON 0x00
#define ACP_INIT_OFF 0x01

/* common ACP define */
#define ACP_COMMON_ON 0x00
#define ACP_COMMON_OFF 0x01

/* receive maximum line */
#define ACP_RECV_MAX_LINE_8 0x00
#define ACP_RECV_MAX_LINE_4 0x01

/* max channel number */
#define MAX_CHANNEL_NUM 16

/* ACP command status */
#define ACP_CAM_STAT 0x55
#define ACP_REG_WR 0x60
#define ACP_REG_RD 0x61
#define ACP_MODE_ID 0x60

/* for baud rate */
#define ACP_PACKET_MODE 0x0B
#define ACP_AHD2_FHD_D0 0x10
#define ACP_AHD2_PEL_BAUD 0x02
#define ACP_AHD2_PEL_LINE 0x07
#define ACP_AHD2_PEL_SYNC 0x0D
#define ACP_AHD2_PEL_EVEN 0x2F
#define ACP_AHD2_FHD_BAUD 0x00
#define ACP_AHD2_FHD_LINE 0x03
#define ACP_AHD2_FHD_LINES 0x05
#define ACP_AHD2_FHD_BYTE 0x0A
#define ACP_AHD2_FHD_MODE 0x0B
#define ACP_AHD2_FHD_OUT 0x09
#define ACP_CLR_REG 0x3A

/********************************************************************
 *  structure
 ********************************************************************/
/* ACP structure, this structure shared with application */
typedef struct _nvp6134_acp_ {
	unsigned char
	    ch_recvmaxline[MAX_CHANNEL_NUM]; /* receive max */
					     /* line(ACP_RECV_MAX_LINE_8, */
					     /* ACP_RECV_MAX_LINE_4) */

} nvp6134_acp;

/********************************************************************
 *  external api
 ********************************************************************/
void init_acp(unsigned char ch);
void acp_each_setting(unsigned char ch);
void acp_read(nvp6134_input_videofmt *pvideoacp, unsigned char ch);
unsigned char acp_isp_read(unsigned char ch, unsigned int reg_addr);
void acp_isp_write(unsigned char ch, unsigned int reg_addr,
		   unsigned char reg_data);
void acp_read_byline(const unsigned char header, unsigned char ch,
		     unsigned char line, unsigned char *acpheader);
void init_acp_reg_rd(unsigned char ch);
unsigned char read_acp_status(unsigned char ch);
void get_acp_reg_rd(unsigned char ch, unsigned char bank, unsigned char addr);
void acp_reg_rx_clear(unsigned char ch);
void acp_set_baudrate(unsigned char ch);

#endif /* End of __ACP_NVP6134_H__ */

/********************************************************************
 *  End of file
 ********************************************************************/
