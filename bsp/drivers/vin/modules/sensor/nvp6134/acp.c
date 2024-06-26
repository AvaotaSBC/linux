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

#include "video.h"
#include "acp.h"

/*******************************************************************************
	* extern variable
	*******************************************************************************/
extern int chip_id[4];			 /* Chip ID */
extern unsigned int nvp6134_cnt;	 /* Chip count */
extern unsigned char ch_mode_status[16]; /* Video format each channel */
extern unsigned char ch_vfmt_status[16]; /* NTSC(0x00), PAL(0x01) */
extern unsigned int nvp6134_iic_addr[4]; /* Slave address of Chip */

/*******************************************************************************
	* internal variable
	*******************************************************************************/
unsigned char ACP_RX_D0 = 0x50; /* Default read start address */
nvp6134_acp s_acp;		/* ACP manager structure */

/*******************************************************************************
	* internal functions
	*******************************************************************************/
/* void 			init_acp(unsigned char ch);
void			acp_reg_rx_clear(unsigned char ch);
void 			get_acp_reg_rd(unsigned char ch, unsigned char
bank, unsigned char addr);
void 			init_acp_reg_rd(unsigned char ch);
unsigned char	read_acp_status(unsigned char ch);
void 			acp_set_baudrate(unsigned char ch);
*/

/*******************************************************************************
	*
	*
	*
	*  Internal Functions
	*
	*
	*
	*******************************************************************************/
/*******************************************************************************
*	Description		: initialize acp register for read
*	Argurments		: ch(channel ID)
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void init_acp_reg_rd(unsigned char ch)
{
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINES + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_MODE_ID + ((ch % 2) * 0x80), ACP_REG_RD);
}

/*******************************************************************************
*	Description		: read acp status(header value)
*	Argurments		: ch(channel ID),
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
unsigned char read_acp_status(unsigned char ch)
{
	unsigned char val;

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));

	val = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0x50 + ((ch % 2) * 0x80));

	return val;
}

/*******************************************************************************
*	Description		: ?
*	Argurments		: ch(channel ID),
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void get_acp_reg_rd(unsigned char ch, unsigned char bank, unsigned char addr)
{
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_D0 + (ch % 2) * 0x80, ACP_REG_RD);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_D0 + 1 + (ch % 2) * 0x80, bank);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_D0 + 2 + (ch % 2) * 0x80, addr);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_D0 + 3 + (ch % 2) * 0x80, 0x00); /* Dummy */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_OUT + (ch % 2) * 0x80, 0x08);
	msleep(100);
}

/*******************************************************************************
*	Description		: ?
*	Argurments		: ch(channel ID),
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void acp_reg_rx_clear(unsigned char ch)
{
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_CLR_REG + ((ch % 2) * 0x80), 0x01);
	/* msleep(10); */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_CLR_REG + ((ch % 2) * 0x80), 0x00);
	/* msleep(100); */
}

/*******************************************************************************
*	Description		: Initialize ACP each CHIP ID
*	Argurments		: ch(channel ID)
*	Return value	: void
*	Modify			:
*	warning			: Now, The Chip ID of NVP6134 and NVP6134B
*is 0x90
*******************************************************************************/
void init_acp(unsigned char ch)
{
	acp_reg_rx_clear(ch);
}

/*******************************************************************************
*	Description		: set each channel's baud rate of coax
*	Argurments		: ch(channel ID)
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void acp_set_baudrate(unsigned char ch)
{
	/* set baud rate */

	if ((ch_mode_status[ch] < NVP6134_VI_720P_2530)) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x11);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_BAUD + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x1B : 0x1B);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_LINE + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x0E : 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_PACKET_MODE + ((ch % 2) * 0x80), 0x06);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_SYNC + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x20 : 0xd4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_SYNC + 1 + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x06 : 0x05);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_EVEN + ((ch % 2) * 0x80), 0x01);

	/* printk(">>>>> DRV[%s:%d] NVP6134_VI_CVBS COAXIAL PROTOCOL IS
	*/
	/* SETTING....\n", __func__, __LINE__ ); */
	}
#ifdef AHD_PELCO_16BIT
	else if (ch_mode_status[ch] == NVP6134_VI_720P_2530 ||
	ch_mode_status[ch] == NVP6134_VI_HDEX) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x11);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_BAUD + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x10 : 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_LINE + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x0D : 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_PACKET_MODE + ((ch % 2) * 0x80), 0x06);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_SYNC + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x00 : 0x00);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_SYNC + 1 + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x00 : 0x00);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_EVEN + ((ch % 2) * 0x80), 0x00);

	/* printk(">>>>> DRV[%s:%d] NVP6134_VI_720P_2530 COAXIAL */
	/* PROTOCOL IS SETTING....\n", __func__, __LINE__ ); */
	}
#else
	else if (ch_mode_status[ch] == NVP6134_VI_720P_2530 ||
	ch_mode_status[ch] == NVP6134_VI_HDEX) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x11);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_BAUD + ((ch % 2) * 0x80), 0x15);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINE + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x0D : 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0D) + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x35 : 0x30);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80), 0x00);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINES + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_BYTE + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_MODE + ((ch % 2) * 0x80), 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_EVEN + ((ch % 2) * 0x80), 0x00);
	/* printk(">>>>> DRV[%s:%d] NVP6134_VI_720P_2530 COAXIAL */
	/* PROTOCOL IS SETTING....\n", __func__, __LINE__ ); */
	}
#endif
	else if (ch_mode_status[ch] == NVP6134_VI_720P_5060) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x01);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_BAUD + ((ch % 2) * 0x80), 0x1A);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINE + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x0D : 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0D) + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x16 : 0x20);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80), 0x00);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINES + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_BYTE + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_MODE + ((ch % 2) * 0x80), 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_EVEN + ((ch % 2) * 0x80), 0x00);
	/* printk(">>>>> DRV[%s:%d] NVP6134_VI_720P_5060 COAXIAL */
	/* PROTOCOL IS SETTING....\n", __func__, __LINE__ ); */
	} else if (ch_mode_status[ch] == NVP6134_VI_1080P_2530 ||
	ch_mode_status[ch] == NVP6134_VI_1080P_NRT) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x11);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
#if VIN_FALSE
	gpio_i2c_write(nvp6134_iic_addr[ch/4], ACP_AHD2_FHD_BAUD+((ch%2)*0x80), 0x27);
	gpio_i2c_write(nvp6134_iic_addr[ch/4], ACP_AHD2_FHD_LINE+((ch%2)*0x80), ch_vfmt_status[ch] == PAL ? 0x0E : 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch/4], (0x0D)+((ch%2)*0x80), ch_vfmt_status[ch] == PAL ? 0xB4 : 0xBB);
	gpio_i2c_write(nvp6134_iic_addr[ch/4], (0x0E)+((ch%2)*0x80), ch_vfmt_status[ch] == PAL ? 0x00 : 0x00);
#else
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_BAUD + ((ch % 2) * 0x80), 0x24);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINE + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x0E : 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0D) + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x84 : 0xBB);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL ? 0x00 : 0x00);
#endif

	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINES + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_BYTE + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_MODE + ((ch % 2) * 0x80), 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_PEL_EVEN + ((ch % 2) * 0x80), 0x00);

	/* printk(">>>>> DRV[%s:%d] NVP6124_VI_1080P_2530, */
	/* NVP6134_VI_1080P_NRT COAXIAL PROTOCOL IS SETTING....\n", */
	/* __func__, __LINE__ ); */
	} else if (ch_mode_status[ch] == NVP6134_VI_3M_NRT ||
	ch_mode_status[ch] == NVP6134_VI_3M ||
	ch_mode_status[ch] == NVP6134_VI_5M_NRT) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x01);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7D, 0x80);

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	if (ch_mode_status[ch] == NVP6134_VI_5M_NRT)
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x00 + ((ch % 2) * 0x80), 0x2C);
	else
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x00 + ((ch % 2) * 0x80), 0x33);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x03 + ((ch % 2) * 0x80), 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x05 + ((ch % 2) * 0x80), 0x07);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0D + ((ch % 2) * 0x80), 0x30);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80), 0x01);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0A + ((ch % 2) * 0x80), 0x07);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0B + ((ch % 2) * 0x80), 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x2F + ((ch % 2) * 0x80), 0x00);
	} else if (ch_mode_status[ch] == NVP6134_VI_4M ||
	ch_mode_status[ch] == NVP6134_VI_4M_NRT) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x11);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7D, 0x80);

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	if (ch_mode_status[ch] == NVP6134_VI_4M_NRT)
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x00 + ((ch % 2) * 0x80), 0x2C);
	else
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x00 + ((ch % 2) * 0x80), 0x34);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x03 + ((ch % 2) * 0x80), 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x05 + ((ch % 2) * 0x80), 0x03);
	if (ch_mode_status[ch] == NVP6134_VI_4M_NRT) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0D + ((ch % 2) * 0x80), 0x60);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80), 0x01);
	} else {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0D + ((ch % 2) * 0x80), 0x00);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80), 0x00);
	}
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0A + ((ch % 2) * 0x80), 0x07);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0B + ((ch % 2) * 0x80), 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x2F + ((ch % 2) * 0x80), 0x00);

	/* printk(">>>>> DRV[%s:%d] NVP6134_VI_4M / NVP6134_VI_4M_NRT */
	/* COAXIAL PROTOCOL IS SETTING....\n", __func__, __LINE__ ); */
	} else if (ch_mode_status[ch] == NVP6134_VI_5M_20P) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C, 0x01);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7D, 0x80);

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x00 + ((ch % 2) * 0x80), 0x35);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x03 + ((ch % 2) * 0x80), 0x0E);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x05 + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0D + ((ch % 2) * 0x80), 0x88);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	(0x0E) + ((ch % 2) * 0x80), 0x00);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0A + ((ch % 2) * 0x80), 0x07);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x0B + ((ch % 2) * 0x80), 0x10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x2F + ((ch % 2) * 0x80), 0x00);

	/* printk(">>>>> DRV[%s:%d] NVP6134_VI_5M_20P COAXIAL PROTOCOL
	*/
	/* IS SETTING....\n", __func__, __LINE__ ); */
	} else {
	printk(">>>>> DRV[%s:%d] COAXIAL MODE NOT RIGHT...\n", __func__,
	__LINE__);
	}
}

/*******************************************************************************
	*
	*
	*
	*  External Functions
	*
	*
	*
	*******************************************************************************/
/*******************************************************************************
*	Description		: set each acp
*	Argurments		: ch(channel ID),
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void acp_each_setting(unsigned char ch)
{
	int vidmode = 0;
	/* unsigned char val, val1; */

	/* mpp setting */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x01); /*   - set band(1) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xA8 + ch % 4,
	(ch % 4) < 2 ? 0x01 : 0x02);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xBC + ch % 4,
	(ch % 2) == 0 ? 0x07 : 0x0F);

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x05 + ch % 4); /*   - set bank(5) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x2F,
	0x00); /*  (+) - internal MPP, HVF(Horizontal Vertical */
	/*  Field sync inversion option) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x30,
	0x00); /* H sync start position */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x31,
	0x43); /* H sync start position */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x32,
	0xa2); /* H sync end position */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7C,
	0x11); /* RX coax Input selection */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x7D, 0x80); /* RX threshold */

	/* set baud rate each format - TX */
	acp_set_baudrate(ch);

	/* a-cp setting */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
	0x03 + ((ch % 4) / 2)); /*   - set bank */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x05 + ((ch % 2) * 0x80),
	0x07); /* TX active Max line(8 line) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x60 + ((ch % 2) * 0x80),
	0x55); /* RX [coax header value] */

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x0b + ((ch % 2) * 0x80),
	0x10); /* change coaxial mode */

	vidmode = ch_mode_status[ch];
	if (vidmode == NVP6134_VI_1080P_2530 ||
	vidmode == NVP6134_VI_720P_5060 ||
	vidmode == NVP6134_VI_720P_2530 || vidmode == NVP6134_VI_HDEX ||
	vidmode == NVP6134_VI_3M_NRT || vidmode == NVP6134_VI_3M ||
	vidmode == NVP6134_VI_4M_NRT || vidmode == NVP6134_VI_4M ||
	vidmode == NVP6134_VI_5M_NRT) {
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x66 + ((ch % 2) * 0x80),
	0x80); /* RX Auto duty */
	if (vidmode == NVP6134_VI_3M_NRT || vidmode == NVP6134_VI_3M ||
	vidmode == NVP6134_VI_4M_NRT || vidmode == NVP6134_VI_4M ||
	vidmode == NVP6134_VI_5M_NRT) {
	gpio_i2c_write(
	nvp6134_iic_addr[ch / 4], 0x62 + ((ch % 2) * 0x80),
	0x06); /* RX receive start line (Change AHD mode) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x68 + ((ch % 2) * 0x80),
	0x70); /* RX(Receive max line - 8 line,
	high-4bits) */
	} else { /* 720P, 1080P */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x62 + ((ch % 2) * 0x80),
	ch_vfmt_status[ch] == PAL
	? 0x05
	: 0x06); /* RX receive start line */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	0x68 + ((ch % 2) * 0x80),
	0x70); /* RX(Receive max line - 8 line,
	high-4bits) */
	}
	}

	/*********************************************************************
	* recognize our code
	*********************************************************************/

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x63 + ((ch % 2) * 0x80),
	0x01); /* RX device(module) ON(1), OFF(0) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x64 + ((ch % 2) * 0x80),
	0x00); /* Delay count */
	gpio_i2c_write(
	nvp6134_iic_addr[ch / 4], 0x67 + ((ch % 2) * 0x80),
	0x01); /* RX(1:interrupt enable), (0:interrupt disable) */

	acp_reg_rx_clear(ch); /* reset */
}

/*******************************************************************************
*	Description		: read acp data of ISP
*	Argurments		: ch(channel ID), reg_addr(high[1byte]:bank,
*low[1byte]:register)
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
unsigned char acp_isp_read(unsigned char ch, unsigned int reg_addr)
{
	unsigned int data;
	unsigned char bank;
	unsigned char addr;

	bank = (reg_addr >> 8) & 0xFF;
	addr = reg_addr & 0xFF;

	init_acp_reg_rd(ch);
	get_acp_reg_rd(ch, bank, addr);

	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));
	data = gpio_i2c_read(nvp6134_iic_addr[ch / 4],
	ACP_RX_D0 + 3 + ((ch % 2) * 0x80));

	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_OUT + ((ch % 2) * 0x80), 0x00);
	acp_reg_rx_clear(ch);
	printk("acp_isp_read ch = %d, reg_addr = %x, reg_data = %x\n", ch,
	reg_addr, data);

	return data;
}

/*******************************************************************************
*	Description		: write data to ISP
*	Argurments		: ch(channel ID),reg_addr(high[1byte]:bank,
*low[1byte]:register)
*					  reg_data(data)
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void acp_isp_write(unsigned char ch, unsigned int reg_addr,
	unsigned char reg_data)
{
	unsigned char bankaddr = 0x00;
	unsigned char device_id = 0x00;

	/* set coax RX device ID */
	bankaddr = (reg_addr >> 8) & 0xFF;
	if (bankaddr >= 0xB0 && bankaddr <= 0xB4) {
	device_id = 0x55;
	} else {
	device_id = ACP_REG_WR;
	}

	/* write data to isp */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_LINES + ((ch % 2) * 0x80), 0x03);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_MODE_ID + ((ch % 2) * 0x80), ACP_REG_WR);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x10 + ((ch % 2) * 0x80),
	ACP_REG_WR); /* data1(#define ACP_AHD2_FHD_D0	0x10) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x11 + ((ch % 2) * 0x80),
	(reg_addr >> 8) & 0xFF); /* data2(bank) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x12 + ((ch % 2) * 0x80),
	reg_addr & 0xFF); /* data3(address) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x13 + ((ch % 2) * 0x80),
	reg_data); /* data4(Don't care) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x60 + ((ch % 2) * 0x80),
	device_id); /* data4(DEVICE ID) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x09 + ((ch % 2) * 0x80),
	0x08); /*   - pulse on(trigger) */
	msleep(140); /* sleep to recognize NRT(15fps) signal for ISP  (M) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + ((ch % 4) / 2));
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x09 + ((ch % 2) * 0x80),
	0x10); /* reset - pulse off */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x09 + ((ch % 2) * 0x80),
	0x00); /*   - pulse off */
	printk("____acp_isp_write ch = %d, reg_addr = %x, reg_data = %x\n", ch,
	reg_addr, reg_data);
}

/*******************************************************************************
*	Description		: read data from ISP
*	Argurments		: pvideoacp( channel's read information),
*ch(channel)
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void acp_read(nvp6134_input_videofmt *pvideoacp, unsigned char ch)
{
	unsigned int buf[16];
	unsigned char val, i;

	/*
	* check status and set/get information
	*/
	val = read_acp_status(ch);
	if (val == ACP_CAM_STAT) {
	for (i = 0; i < 8; i++) {
	buf[i] =
	gpio_i2c_read(nvp6134_iic_addr[ch / 4],
	ACP_RX_D0 + ((ch % 2) * 0x80) + i);
	pvideoacp->getacpdata[ch][i] = buf[i];
	}

/*
	/* for Debuging message [ISP<->DECODER]
	if (buf[7] == 0x00)
		printk(">>>>> DRV[%s:%d] CH:%d, ACP_CAM_STATUS[STATUS MODE] = ", __func__, __LINE__, ch);
	else if (buf[7] == 0x01)
		printk(">>>>> DRV[%s:%d] CH:%d, ACP_CAM_STATUS[MOTION INFO] = ", __func__, __LINE__, ch);
	else if (buf[7] == 0x02)
		printk(">>>>> DRV[%s:%d] CH:%d, ACP_CAM_STATUS[FIRM UPGRADE] = ", __func__, __LINE__, ch);
	else if (buf[7] == 0x03)
		printk(">>>>> DRV[%s:%d] CH:%d, ACP_CAM_STATUS[FRAME RATE] = ", __func__, __LINE__, ch);
	printk("[%02x %02x %02x %02x %02x %02x %02x %02x]\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
*/
	} else if (val == ACP_REG_WR) {
	for (i = 0; i < 4; i++) {
	buf[i] =
	gpio_i2c_read(nvp6134_iic_addr[ch / 4],
	ACP_RX_D0 + ((ch % 2) * 0x80) + i);
	pvideoacp->getacpdata[ch][i] = buf[i];
	}
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_OUT + ((ch % 2) * 0x80), 0x00);
	/* printk(">>>>> DRV[%s:%d] CH:%D, ACP_Write = 0x%02x 0x%02x */
	/* 0x%02x 0x%02x\n", __func__, __LINE__, ch, buf[0], buf[1], */
	/* buf[2]. buf[3]); */
	} else if (val == ACP_REG_RD) {
	for (i = 0; i < 4; i++) {
	buf[i] =
	gpio_i2c_read(nvp6134_iic_addr[ch / 4],
	ACP_RX_D0 + ((ch % 2) * 0x80) + i);
	pvideoacp->getacpdata[ch][i] = buf[i];
	}
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_OUT + ((ch % 2) * 0x80), 0x00);
	/* printk(">>>>> DRV[%s:%d] CH:%d, ACP_REG_RD, ACP_Read = 0x%02x
	*/
	/* 0x%02x 0x%02x 0x%02x\n", __func__, __LINE__, ch, buf[0], */
	/* buf[1], buf[2], buf[3]); */
	} else {
	for (i = 0; i < 8; i++) {
	pvideoacp->getacpdata[ch][i] = 0x00;
	}
	gpio_i2c_write(nvp6134_iic_addr[ch / 4],
	ACP_AHD2_FHD_OUT + ((ch % 2) * 0x80), 0x00);
	/* printk(">>>>> DRV[%s:%d] CH:%d, ACP_RX_Error!!!!\n", */
	/* __func__, __LINE__, ch ); */
	}
	acp_reg_rx_clear(ch);
}

void acp_read_byline(const unsigned char header, unsigned char ch,
	unsigned char line, unsigned char *acpheader)
{
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x03 + (ch % 4) / 2);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x60 + ((ch % 2) * 0x80),
	header); /* header */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x62 + ((ch % 2) * 0x80),
	line); /* line */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x63 + ((ch % 2) * 0x80),
	0x01); /* common on */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x64 + ((ch % 2) * 0x80),
	0x00); /* Delay count */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x66 + ((ch % 2) * 0x80),
	0x80);
	gpio_i2c_write(
	nvp6134_iic_addr[ch / 4], 0x67 + ((ch % 2) * 0x80),
	0x01); /* RX(1:interrupt enable), (0:interrupt disable) */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x68 + ((ch % 2) * 0x80),
	0x70); /* [7:4] read number */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x69 + ((ch % 2) * 0x80),
	0x80);
	msleep(10);
	acpheader[0] =
	gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0x50 + ((ch % 2) * 0x80));
	msleep(10);
	acpheader[1] =
	gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0x51 + ((ch % 2) * 0x80));
	printk("DRV[%s %d]>>>>header = %x %x\n", __FUNCTION__, __LINE__,
	acpheader[0], acpheader[1]);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x3A + ((ch % 2) * 0x80),
	0x01); /* clear status */
	msleep(10);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x3A + ((ch % 2) * 0x80),
	0x00);
	msleep(10);
}

/*******************************************************************************
*	End of file
*******************************************************************************/
