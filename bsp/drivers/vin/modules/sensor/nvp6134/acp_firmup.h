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

#ifndef __ACP_FIRMWARE_UPDATE_H__
#define __ACP_FIRMWARE_UPDATE_H__

/********************************************************************************
*                              DEFINE
*********************************************************************************/
#define ONE_PACKET_MAX_SIZE 139

#define ACP_FIRMWARE_UP_START 0x00
#define ACP_FIRMWARE_UP_STOP 0x01

/********************************************************************************
*                              ENUMERATION
*********************************************************************************/

/********************************************************************************
*                              STRUCTURE-external
*********************************************************************************/
/* Firmware update(TX) managemnet structure */
typedef struct __firmware_update_manager__ {
	int firmware_status[16]; /* firmware status */
	int curvidmode[16];      /* now, firmware video mode */
	int curvideofmt[16];     /* now, firmware video format */

} firmware_update_manager, *pfirmware_update_manager;

/********************************************************************************
*                             EXTERN FUNCTIONS
*********************************************************************************/
extern int acp_dvr_ispossible_update(void *p_param);
extern int acp_dvr_firmware_update(void *p_param);
extern int acp_dvr_end_command(int send_success, void *p_param);
extern int acp_dvr_check_ispstatus(void *p_param);
extern int acp_dvr_start_command(void *p_param);

extern void acp_dvr_set_firmupstatus(int ch, int flag);
extern int acp_dvr_get_firmupstatus(int ch);
extern int acp_dvr_set_curvideomode(int ch, int curvidmode, int vfmt);
extern int acp_dvr_get_curvideomode(int ch, int *pcurvidmode, int *pvfmt);

#endif /* __ACP_FIRMWARE_UPDATE_H__ */

/*******************************************************************************
*	End of file
*******************************************************************************/
