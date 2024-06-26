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
 * published by the Free Software Foundation. */


#ifndef __COMMON_H__
#define __COMMON_H__

#ifdef HI_GPIO_I2C
#include "gpio_i2c.h"
#endif

#ifdef HI_I2C
#include "hi_i2c.h"
#undef I2C_INTERNAL
#endif

/* #ifdef HI_GPIO_I2C */
/* #define  I2CReadByte   gpio_i2c_read */
/* #define  I2CWriteByte  gpio_i2c_write */
/* */
/* #else */
/* unsigned char __I2CReadByte8(unsigned char devaddress, unsigned char */
/* address); */
/* void __I2CWriteByte8(unsigned char devaddress, unsigned char address, */
/* unsigned char data); */
/* */
/* #define  gpio_i2c_read   __I2CReadByte8 */
/* #define  gpio_i2c_write  __I2CWriteByte8 */
/* */
/* #endif */

/* device address define */
#define NVP6134_R0_ID 0x91
#define NVP6134B_R0_ID                                                         \
	0x90 /* 6134B AND 6134C USES THE SAME CHIPID,DIFF IN REV_ID */
#define NVP6134B_REV_ID 0x00
#define NVP6134C_REV_ID 0x01
#define CH_PER_CHIP 4

#define NTSC 0x00
#define PAL 0x01

#define AHD_PELCO_16BIT

#define PELCO_CMD_RESET 0
#define PELCO_CMD_SET 1
#define PELCO_CMD_UP 2
#define PELCO_CMD_DOWN 3
#define PELCO_CMD_LEFT 4
#define PELCO_CMD_RIGHT 5
#define PELCO_CMD_OSD 6
#define PELCO_CMD_IRIS_OPEN 7
#define PELCO_CMD_IRIS_CLOSE 8
#define PELCO_CMD_FOCUS_NEAR 9
#define PELCO_CMD_FOCUS_FAR 10
#define PELCO_CMD_ZOOM_WIDE 11
#define PELCO_CMD_ZOOM_TELE 12
#define PELCO_CMD_SCAN_SR 13
#define PELCO_CMD_SCAN_ST 14
#define PELCO_CMD_PRESET1 15
#define PELCO_CMD_PRESET2 16
#define PELCO_CMD_PRESET3 17
#define PELCO_CMD_PTN1_SR 18
#define PELCO_CMD_PTN1_ST 19
#define PELCO_CMD_PTN2_SR 20
#define PELCO_CMD_PTN2_ST 21
#define PELCO_CMD_PTN3_SR 22
#define PELCO_CMD_PTN3_ST 23
#define PELCO_CMD_RUN 24

/* other command */
#define EXC_CMD_RESET 0
#define EXC_CMD_SET 1
#define EXC_CMD_UP 2
#define EXC_CMD_DOWN 3
#define EXC_CMD_LEFT 4
#define EXC_CMD_RIGHT 5
#define EXC_CMD_OSD 6
#define EXC_CMD_IRIS_OPEN 7
#define EXC_CMD_IRIS_CLOSE 8
#define EXC_CMD_FOCUS_NEAR 9
#define EXC_CMD_FOCUS_FAR 10
#define EXC_CMD_ZOOM_WIDE 11
#define EXC_CMD_ZOOM_TELE 12
#define EXC_CMD_SCAN_SR 13
#define EXC_CMD_SCAN_ST 14
#define EXC_CMD_PRESET1 15
#define EXC_CMD_PRESET2 16
#define EXC_CMD_PRESET3 17
#define EXC_CMD_PTN1_SR 18
#define EXC_CMD_PTN1_ST 19
#define EXC_CMD_PTN2_SR 20
#define EXC_CMD_PTN2_ST 21
#define EXC_CMD_PTN3_SR 22
#define EXC_CMD_PTN3_ST 23
#define EXC_CMD_RUN 24

#define EXT_CMD_RESET 0
#define EXT_CMD_SET 1
#define EXT_CMD_UP 2
#define EXT_CMD_DOWN 3
#define EXT_CMD_LEFT 4
#define EXT_CMD_RIGHT 5
#define EXT_CMD_OSD 6
#define EXT_CMD_IRIS_OPEN 7
#define EXT_CMD_IRIS_CLOSE 8
#define EXT_CMD_FOCUS_NEAR 9
#define EXT_CMD_FOCUS_FAR 10
#define EXT_CMD_ZOOM_WIDE 11
#define EXT_CMD_ZOOM_TELE 12
#define EXT_CMD_SCAN_SR 13
#define EXT_CMD_SCAN_ST 14
#define EXT_CMD_PRESET1 15
#define EXT_CMD_PRESET2 16
#define EXT_CMD_PRESET3 17
#define EXT_CMD_PTN1_SR 18
#define EXT_CMD_PTN1_ST 19
#define EXT_CMD_PTN2_SR 20
#define EXT_CMD_PTN2_ST 21
#define EXT_CMD_PTN3_SR 22
#define EXT_CMD_PTN3_ST 23
#define EXT_CMD_RUN 24
#define EXT_CMD_VER_SWITCH 25

#define SET_ALL_CH 0xff

/* FIXME HI3520 Register */
#define VIU_CH_CTRL 0x08
#define VIU_ANC0_START 0x9c
#define VIU_ANC0_SIZE 0xa0
#define VIU_ANC1_START 0xa4
#define VIU_ANC1_SIZE 0xa8
#define VIU_BLANK_DATA_ADDR 0xac

#define IOC_VDEC_SET_VIDEO_MODE 0x07
#define IOC_VDEC_GET_INPUT_VIDEO_FMT 0x08
#define IOC_VDEC_GET_VIDEO_LOSS 0x09
#define IOC_VDEC_SET_SYNC 0x0A
#define IOC_VDEC_SET_EQUALIZER 0x0B
#define IOC_VDEC_GET_DRIVERVER 0x0C
#define IOC_VDEC_PTZ_ACP_READ 0x0D
#define IOC_VDEC_SET_BRIGHTNESS 0x0E
#define IOC_VDEC_SET_CONTRAST 0x0F
#define IOC_VDEC_SET_HUE 0x10
#define IOC_VDEC_SET_SATURATION 0x11
#define IOC_VDEC_SET_SHARPNESS 0x12
#define IOC_VDEC_SET_CHNMODE 0x13
#define IOC_VDEC_SET_OUTPORTMODE 0x14
#define IOC_VDEC_PTZ_CHANNEL_SEL 0x20
#define IOC_VDEC_PTZ_PELCO_INIT 0x21
#define IOC_VDEC_PTZ_PELCO_RESET 0x22
#define IOC_VDEC_PTZ_PELCO_SET 0x23
#define IOC_VDEC_PTZ_PELCO_UP 0x24
#define IOC_VDEC_PTZ_PELCO_DOWN 0x25
#define IOC_VDEC_PTZ_PELCO_LEFT 0x26
#define IOC_VDEC_PTZ_PELCO_RIGHT 0x27
#define IOC_VDEC_PTZ_PELCO_OSD 0x28
#define IOC_VDEC_PTZ_PELCO_IRIS_OPEN 0x29
#define IOC_VDEC_PTZ_PELCO_IRIS_CLOSE 0x2a
#define IOC_VDEC_PTZ_PELCO_FOCUS_NEAR 0x2b
#define IOC_VDEC_PTZ_PELCO_FOCUS_FAR 0x2c
#define IOC_VDEC_PTZ_PELCO_ZOOM_WIDE 0x2d
#define IOC_VDEC_PTZ_PELCO_ZOOM_TELE 0x2e
#define IOC_VDEC_ACP_WRITE 0x2f

#define IOC_VDEC_INIT_MOTION 0x40
#define IOC_VDEC_ENABLE_MOTION 0x41
#define IOC_VDEC_DISABLE_MOTION 0x42
#define IOC_VDEC_SET_MOTION_AREA 0x43
#define IOC_VDEC_GET_MOTION_INFO 0x44
#define IOC_VDEC_SET_MOTION_DISPLAY 0x45
#define IOC_VDEC_SET_MOTION_SENS 0x46

#define IOC_AUDIO_SET_CHNNUM 0x80
#define IOC_AUDIO_SET_SAMPLE_RATE 0x81
#define IOC_AUDIO_SET_BITWIDTH 0x82
#define IOC_VDEC_SET_I2C 0x83

#define IOC_VDEC_ACP_POSSIBLE_FIRMUP 0xA0 /* by Andy(2016-06-26) */
#define IOC_VDEC_ACP_CHECK_ISPSTATUS 0xA1 /* by Andy(2016-07-12) */
#define IOC_VDEC_ACP_START_FIRMUP 0xA2    /* by Andy(2016-07-12) */
#define IOC_VDEC_ACP_FIRMUP 0xA3	  /* by Andy(2016-06-26) */
#define IOC_VDEC_ACP_FIRMUP_END 0xA4      /* by Andy(2016-06-26) */

typedef struct _nvp6134_video_mode {
	unsigned int chip;
	unsigned int mode;
	unsigned char vformat[16];
	unsigned char chmode[16];
} nvp6134_video_mode;

typedef struct _nvp6134_chn_mode {
	unsigned char ch;
	unsigned char vformat;
	unsigned char chmode;
} nvp6134_chn_mode;

typedef struct _nvp6134_opt_mode {
	unsigned char chipsel;
	unsigned char portsel;
	unsigned char portmode;
	unsigned char chid;
} nvp6134_opt_mode;

typedef struct _nvp6134_input_videofmt {
	unsigned int inputvideofmt[16];
	unsigned int getvideofmt[16];
	unsigned int geteqstage[16];
	unsigned int getacpdata[16][8];
} nvp6134_input_videofmt;

typedef struct _nvp6124_i2c_mode {
	unsigned char flag; /* 0: read, 1 : write */
	unsigned char slaveaddr;
	unsigned char bank;
	unsigned char address;
	unsigned char data;
} nvp6124_i2c_mode;

typedef struct _nvp6134_video_adjust {
	unsigned int ch;
	unsigned int value;
} nvp6134_video_adjust;

typedef struct _nvp6134_motion_area {
	unsigned char ch;
	int m_info[12];
} nvp6134_motion_area;

typedef struct _nvp6134_audio_playback {
	unsigned int chip;
	unsigned int ch;
} nvp6134_audio_playback;

typedef struct _nvp6134_audio_da_mute {
	unsigned int chip;
} nvp6134_audio_da_mute;

typedef struct _nvp6134_audio_da_volume {
	unsigned int chip;
	unsigned int volume;
} nvp6134_audio_da_volume;

typedef struct _nvp6134_audio_format {
	unsigned char format;    /* 0:i2s; 1:dsp */
	unsigned char mode;      /* 0:slave 1:master */
	unsigned char dspformat; /* 0:dsp;1:ssp */
	unsigned char clkdir;    /* 0:inverted;1:non-inverted */
	unsigned char chn_num;   /* 2,4,8,16 */
	unsigned char bitrate;   /* 0:256fs 1:384fs invalid for nvp6114 2:320fs */
	unsigned char precision; /* 0:16bit;1:8bit */
	unsigned char samplerate; /* 0:8kHZ;1:16kHZ; 2:32kHZ */
} nvp6134_audio_format;

/* by Andy(2016-06-26) */
typedef struct __file_information {
	unsigned int channel;
	unsigned char filename[64];
	unsigned char filePullname[64 + 32];
	unsigned int filesize;
	unsigned int filechecksum; /* (sum of file&0x0000FFFFF) */
	unsigned int
	    currentpacketnum; /* current packet sequnce number(0,1,2........) */
	unsigned int filepacketnum; /* file packet number = (total */
				    /* size/128bytes), if remain exist, file */
				    /* packet number++ */
	unsigned char onepacketbuf[128 + 32];

	unsigned int currentFileOffset; /* Current file offset */
	unsigned int readsize;		/* currnet read size */

	unsigned int ispossiblefirmup[16]; /* is it possible to update firmware? */
	int result;

	int appstatus[16]; /* Application status */

} FIRMWARE_UP_FILE_INFO, *PFIRMWARE_UP_FILE_INFO;

#define NVP6134_IOC_MAGIC 'n'

#define NVP6134_SET_AUDIO_PLAYBACK                                             \
	_IOW(NVP6134_IOC_MAGIC, 0x21, nvp6134_audio_playback)
#define NVP6134_SET_AUDIO_DA_MUTE                                              \
	_IOW(NVP6134_IOC_MAGIC, 0x22, nvp6134_audio_da_mute)
#define NVP6134_SET_AUDIO_DA_VOLUME                                            \
	_IOW(NVP6134_IOC_MAGIC, 0x23, nvp6134_audio_da_volume)
/* set record format */
#define NVP6134_SET_AUDIO_R_FORMAT                                             \
	_IOW(NVP6134_IOC_MAGIC, 0x24, nvp6134_audio_format)
/* set playback format */
#define NVP6134_SET_AUDIO_PB_FORMAT                                            \
	_IOW(NVP6134_IOC_MAGIC, 0x25, nvp6134_audio_format)

#define SET_BIT(data, bit) ((data) |= (1 << (bit)))
#define CLE_BIT(data, bit) ((data) &= (~(1 << (bit))))

#endif
