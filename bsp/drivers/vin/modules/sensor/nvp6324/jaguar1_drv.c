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

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/string.h>
#include <linux/list.h>
#include <asm/delay.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "jaguar1_common.h"
#include "jaguar1_video.h"
#include "jaguar1_coax_protocol.h"
#include "jaguar1_motion.h"
#include "jaguar1_ioctl.h"
#include "jaguar1_video_eq.h"
#include "jaguar1_mipi.h"



#define I2C_0       (0)
#define I2C_1       (1)
#define I2C_2       (2)
#define I2C_3       (3)

#define JAGUAR1_4PORT_R0_ID 0xB0
#define JAGUAR1_2PORT_R0_ID 0xA0
#define JAGUAR1_1PORT_R0_ID 0xA2
#define AFE_NVP6134E_R0_ID 	0x80

#define JAGUAR1_4PORT_REV_ID 0x00
#define JAGUAR1_2PORT_REV_ID 0x00
#define JAGUAR1_1PORT_REV_ID 0x00

static int chip_id[4];
static int rev_id[4];
static int jaguar1_cnt;
unsigned int jaguar1_i2c_addr[4] = {0x60, 0x62, 0x64, 0x66};

struct semaphore jaguar1_lock;
struct i2c_client *jaguar1_client;
static struct i2c_board_info hi_info = {
	I2C_BOARD_INFO("jaguar1", 0x60),
};
decoder_get_information_str decoder_inform;

static void vd_pattern_enable(void)
{
	gpio_i2c_write(0x60, 0xFF, 0x00);
	gpio_i2c_write(0x60, 0x1C, 0x1A);
	gpio_i2c_write(0x60, 0x1D, 0x1A);
	gpio_i2c_write(0x60, 0x1E, 0x1A);
	gpio_i2c_write(0x60, 0x1F, 0x1A);

	gpio_i2c_write(0x60, 0xFF, 0x05);
	gpio_i2c_write(0x60, 0x6A, 0x80);
	gpio_i2c_write(0x60, 0xFF, 0x06);
	gpio_i2c_write(0x60, 0x6A, 0x80);
	gpio_i2c_write(0x60, 0xFF, 0x07);
	gpio_i2c_write(0x60, 0x6A, 0x80);
	gpio_i2c_write(0x60, 0xFF, 0x08);
	gpio_i2c_write(0x60, 0x6A, 0x80);
}

/*******************************************************************************
 *	Description		: Sample function - for select video format
 *	Argurments		: int dev_num(i2c_address array's num)
 *	Return value	: void
 *	Modify			:
 *	warning			:
 *******************************************************************************/
#ifdef FMT_SETTING_SAMPLE
static void set_default_video_fmt(int dev_num)
{
	int i;
	video_input_init  video_val;

/*
	for (i = 0; i < 4 ; i++) {
		video_val.ch = i;

		/* select video format, include struct'vd_vi_init_list' in jaguar1_video_table.h
		 *  ex > AHD20_1080P_30P / AHD20_720P_25P_EX_Btype / AHD20_SD_H960_2EX_Btype_NT
		video_val.format = AHD20_720P_30P_EX_Btype;

		/* select analog input type, SINGLE_ENDED or DIFFERENTIAL
		video_val.input = SINGLE_ENDED;

		/* select decoder to soc interface
		video_val.interface = MIPI;

		/* run video setting
		vd_jaguar1_init_set(&video_val);

		/* run video format setting for mipi/arbiter
		mipi_video_format_set(&video_val);
	}
	arb_init(dev_num);
	disable_parallel(dev_num);
*/
/*
	video_val.ch = 0;
	video_val.format = AHD20_1080P_30P;
	video_val.input = SINGLE_ENDED;
	vd_jaguar1_init_set(&video_val);

	mipi_video_format_set(&video_val);

	arb_init(dev_num);
*/
}
#endif

/*******************************************************************************
 *	Description		: Check ID
 *	Argurments		: dec(slave address)
 *	Return value	: Device ID
 *	Modify			:
 *	warning			:
 *******************************************************************************/
static void vd_set_all(video_init_all *param)
{
	int i, dev_num = 0;
	video_input_init  video_val[4];

/*
	for (i = 0; i < 4; i++) {
		printk("[DRV || %s] ch%d / fmt:%d / input:%d / interface:%d\n", __func__
				, param->ch_param[i].ch
				, param->ch_param[i].format
				, param->ch_param[i].input
				, param->ch_param[i].interface);
	}
*/
	mipi_datatype_set(VD_DATA_TYPE_YUV422);
	mipi_tx_init(dev_num);

	for (i = 0; i < 4; i++) {
		video_val[i].ch = param->ch_param[i].ch;
		video_val[i].format = param->ch_param[i].format;
		video_val[i].input = param->ch_param[i].input;
		video_val[i].interface = param->ch_param[i].interface;

		vd_jaguar1_init_set(&video_val[i]);
		mipi_video_format_set(&video_val[i]);
	}
	arb_init(dev_num);
	disable_parallel(dev_num);
	/* vd_pattern_enable(); */
}

/*******************************************************************************
 *	Description		: Check ID
 *	Argurments		: dec(slave address)
 *	Return value	: Device ID
 *	Modify			:
 *	warning			:
 *******************************************************************************/
static int check_id(unsigned int dec)
{
	int ret;
	gpio_i2c_write(dec, 0xFF, 0x00);
	ret = gpio_i2c_read(dec, 0xf4);
	return ret;
}

/*******************************************************************************
 *	Description		: Get rev ID
 *	Argurments		: dec(slave address)
 *	Return value	: rev ID
 *	Modify			:
 *	warning			:
 *******************************************************************************/
static int check_rev(unsigned int dec)
{
	int ret;
	gpio_i2c_write(dec, 0xFF, 0x00);
	ret = gpio_i2c_read(dec, 0xf5);
	return ret;
}

/*******************************************************************************
 *	Description		: Check decoder count
 *	Argurments		: void
 *	Return value	: (total chip count - 1) or -1(not found any chip)
 *	Modify			:
 *	warning			:
 *******************************************************************************/
int check_decoder_count(void)
{
	int chip, i;
	int ret = -1;

	for (chip = 0; chip < 4; chip++) {
		chip_id[chip] = check_id(jaguar1_i2c_addr[chip]);
		rev_id[chip]  = check_rev(jaguar1_i2c_addr[chip]);
		if ((chip_id[chip] != JAGUAR1_4PORT_R0_ID)  	&&
				(chip_id[chip] != JAGUAR1_2PORT_R0_ID) 		&&
				(chip_id[chip] != JAGUAR1_1PORT_R0_ID)		&&
				(chip_id[chip] != AFE_NVP6134E_R0_ID)
		  ) {
			printk("Device ID Error... %x, Chip Count:[%d]\n", chip_id[chip], chip);
			jaguar1_i2c_addr[chip] = 0xFF;
			chip_id[chip] = 0xFF;
		} else {
			printk("Device (0x%x) ID OK... %x , Chip Count:[%d]\n", jaguar1_i2c_addr[chip], chip_id[chip], chip);
			printk("Device (0x%x) REV %x\n", jaguar1_i2c_addr[chip], rev_id[chip]);
			jaguar1_i2c_addr[jaguar1_cnt] = jaguar1_i2c_addr[chip];

			if (jaguar1_cnt < chip) {
				jaguar1_i2c_addr[chip] = 0xFF;
			}

			chip_id[jaguar1_cnt] = chip_id[chip];
			rev_id[jaguar1_cnt]  = rev_id[chip];

			jaguar1_cnt++;
		}

		if ((chip == 3) && (jaguar1_cnt < chip)) {
			for (i = jaguar1_cnt; i < 4; i++) {
				chip_id[i] = 0xff;
				rev_id[i]  = 0xff;
			}
		}
	}
	printk("Chip Count = %d\n", jaguar1_cnt);
	printk("Address [0x%x][0x%x][0x%x][0x%x]\n", jaguar1_i2c_addr[0], jaguar1_i2c_addr[1], jaguar1_i2c_addr[2], jaguar1_i2c_addr[3]);
	printk("Chip Id [0x%x][0x%x][0x%x][0x%x]\n", chip_id[0], chip_id[1], chip_id[2], chip_id[3]);
	printk("Rev Id  [0x%x][0x%x][0x%x][0x%x]\n", rev_id[0], rev_id[1], rev_id[2], rev_id[3]);

	for (i = 0; i < 4; i++) {
		decoder_inform.chip_id[i] = chip_id[i];
		decoder_inform.chip_rev[i] = rev_id[i];
		decoder_inform.chip_addr[i] = jaguar1_i2c_addr[i];
	}
	decoder_inform.Total_Chip_Cnt = jaguar1_cnt;
	ret = jaguar1_cnt;

	return ret;
}

/*******************************************************************************
 *	Description		: Video decoder initial
 *	Argurments		: void
 *	Return value	: void
 *	Modify			:
 *	warning			:
 *******************************************************************************/
void video_decoder_init(void)
{
	int ii = 0;

	gpio_i2c_write(jaguar1_i2c_addr[0], 0xff, 0x04);

	for (ii = 0; ii < 36; ii++) {
		gpio_i2c_write(jaguar1_i2c_addr[0], 0xa0 + ii, 0x24);
	}

	gpio_i2c_write(jaguar1_i2c_addr[0], 0xff, 0x01);
	for (ii = 0; ii < 4; ii++) {
		gpio_i2c_write(jaguar1_i2c_addr[0], 0xcc + ii, 0x64);
	}

}
/*
/*******************************************************************************
 *	Description		: Driver open
 *	Argurments		:
 *	Return value	:
 *	Modify			:
 *	warning			:
 *******************************************************************************
int jaguar1_open(struct inode *inode, struct file *file)
{
	printk("[DRV] Jaguar1 Driver Open\n");
	printk("[DRV] Jaguar1 Driver Ver::%s\n", DRIVER_VER);
	return 0;
}

/*******************************************************************************
 *	Description		: Driver close
 *	Argurments		:
 *	Return value	:
 *	Modify			:
 *	warning			:
 *******************************************************************************
int jaguar1_close(struct inode *inode, struct file *file)
{
	printk("[DRV] Jaguar1 Driver Close\n");
	return 0;
}

/*******************************************************************************
 *	Description		: Driver IOCTL function
 *	Argurments		:
 *	Return value	:
 *	Modify			:
 *	warning			:
 *******************************************************************************
long jaguar1_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int cpy2usr_ret;
	unsigned int __user *argp = (unsigned int __user *)arg;

	/* AllVideo Variable
	video_init_all all_vd_val;

	/* Video Variable
	video_input_init  video_val;
	video_output_init vo_seq_set;
	video_equalizer_info_s video_eq;
	video_video_loss_s vidloss;

	/* Coaxial Protocol Variable
	NC_VD_COAX_STR           coax_val;
	NC_VD_COAX_BANK_DUMP_STR coax_bank_dump;
	FIRMWARE_UP_FILE_INFO    coax_fw_val;
	NC_VD_COAX_TEST_STR      coax_test_val;

	/* Motion Variable
	motion_mode motion_set;

	down(&jaguar1_lock);

	switch (cmd) {
		/*===============================================================================================
		 * Set All - for MIPI Interface
		 *===============================================================================================
	case IOC_VDEC_INIT_ALL:
			if (copy_from_user(&all_vd_val, argp, sizeof(video_init_all)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			vd_set_all(&all_vd_val);
			break;
		/*===============================================================================================
		 * Video Initialize
		 *===============================================================================================
	case IOC_VDEC_INPUT_INIT:
			if (copy_from_user(&video_val, argp, sizeof(video_input_init)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			vd_jaguar1_init_set(&video_val);
			break;
	case IOC_VDEC_OUTPUT_SEQ_SET:
			if (copy_from_user(&vo_seq_set, argp, sizeof(video_output_init)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			vd_jaguar1_vo_ch_seq_set(&vo_seq_set);
			break;
	case IOC_VDEC_VIDEO_EQ_SET:
			if (copy_from_user(&video_eq, argp, sizeof(video_equalizer_info_s)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			video_input_eq_val_set(&video_eq);
			break;
	case IOC_VDEC_VIDEO_SW_RESET:
			if (copy_from_user(&video_val, argp, sizeof(video_input_init)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			vd_jaguar1_sw_reset(&video_val);
			break;
	case IOC_VDEC_VIDEO_EQ_CABLE_SET:
			if (copy_from_user(&video_eq, argp, sizeof(video_equalizer_info_s)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			video_input_eq_cable_set(&video_eq);
			break;
	case IOC_VDEC_VIDEO_EQ_ANALOG_INPUT_SET:
			if (copy_from_user(&video_eq, argp, sizeof(video_equalizer_info_s)))
				printk("IOC_VDEC_INPUT_INIT error\n");
			video_input_eq_analog_input_set(&video_eq);
			break;
	case IOC_VDEC_VIDEO_GET_VIDEO_LOSS:
			if (copy_from_user(&vidloss, argp, sizeof(video_video_loss_s)))
				printk("IOC_VDEC_VIDEO_GET_VIDEO_LOSS error\n");
			vd_jaguar1_get_novideo(&vidloss);
			cpy2usr_ret = copy_to_user(argp, &vidloss, sizeof(video_video_loss_s));
			break;
			/*===============================================================================================
			 * Coaxial Protocol
			 *===============================================================================================
	case IOC_VDEC_COAX_TX_INIT:   /* SK_CHANGE 170703
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk("IOC_VDEC_COAX_TX_INIT error\n");
			coax_tx_init(&coax_val);
			break;
	case IOC_VDEC_COAX_TX_16BIT_INIT:   /* SK_CHANGE 170703
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk("IOC_VDEC_COAX_TX_INIT error\n");
			coax_tx_16bit_init(&coax_val);
			break;
	case IOC_VDEC_COAX_TX_CMD_SEND: /* SK_CHANGE 170703
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_TX_CMD_SEND error\n");
			coax_tx_cmd_send(&coax_val);
			break;
	case IOC_VDEC_COAX_TX_16BIT_CMD_SEND: /* SK_CHANGE 170703
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_TX_CMD_SEND error\n");
			coax_tx_16bit_cmd_send(&coax_val);
			break;
	case IOC_VDEC_COAX_TX_CVI_NEW_CMD_SEND: /* SK_CHANGE 170703
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_TX_CMD_SEND error\n");
			coax_tx_cvi_new_cmd_send(&coax_val);
			break;
	case IOC_VDEC_COAX_RX_INIT:
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_RX_INIT error\n");
			coax_rx_init(&coax_val);
			break;
	case IOC_VDEC_COAX_RX_DATA_READ:
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_RX_DATA_READ error\n");
			coax_rx_data_get(&coax_val);
			cpy2usr_ret = copy_to_user(argp, &coax_val, sizeof(NC_VD_COAX_STR));
			break;
	case IOC_VDEC_COAX_RX_BUF_CLEAR:
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_RX_BUF_CLEAR error\n");
			coax_rx_buffer_clear(&coax_val);
			break;
	case IOC_VDEC_COAX_RX_DEINIT:
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk("IOC_VDEC_COAX_RX_DEINIT error\n");
			coax_rx_deinit(&coax_val);
			break;
	case IOC_VDEC_COAX_BANK_DUMP_GET:
			if (copy_from_user(&coax_bank_dump, argp, sizeof(NC_VD_COAX_BANK_DUMP_STR)))
				printk("IOC_VDEC_COAX_BANK_DUMP_GET error\n");
			coax_test_Bank_dump_get(&coax_bank_dump);
			cpy2usr_ret = copy_to_user(argp, &coax_bank_dump, sizeof(NC_VD_COAX_BANK_DUMP_STR));
			break;
	case IOC_VDEC_COAX_RX_DETECTION_READ:
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_RX_DATA_READ error\n");
			coax_acp_rx_detect_get(&coax_val);
			cpy2usr_ret = copy_to_user(argp, &coax_val, sizeof(NC_VD_COAX_STR));
			break;
			/*===============================================================================================
			 * Coaxial Protocol. Function
			 *===============================================================================================
	case IOC_VDEC_COAX_RT_NRT_MODE_CHANGE_SET:
			if (copy_from_user(&coax_val, argp, sizeof(NC_VD_COAX_STR)))
				printk(" IOC_VDEC_COAX_SHOT_SET error\n");
			coax_option_rt_nrt_mode_change_set(&coax_val);
			cpy2usr_ret = copy_to_user(argp, &coax_val, sizeof(NC_VD_COAX_STR));
			break;
			/*===============================================================================================
			 * Coaxial Protocol FW Update
			 *===============================================================================================
	case IOC_VDEC_COAX_FW_ACP_HEADER_GET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_READY_CMD_SET error\n");
			coax_fw_ready_header_check_from_isp_recv(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_READY_CMD_SET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_READY_CMD_SET error\n");
			coax_fw_ready_cmd_to_isp_send(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_READY_ACK_GET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_READY_ISP_STATUS_GET error\n");
			coax_fw_ready_cmd_ack_from_isp_recv(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_START_CMD_SET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_START_CMD_SET error\n");
			coax_fw_start_cmd_to_isp_send(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_START_ACK_GET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_START_CMD_SET error\n");
			coax_fw_start_cmd_ack_from_isp_recv(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_SEND_DATA_SET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_START_CMD_SET error\n");
			coax_fw_one_packet_data_to_isp_send(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_SEND_ACK_GET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_START_CMD_SET error\n");
			coax_fw_one_packet_data_ack_from_isp_recv(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_END_CMD_SET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_START_CMD_SET error\n");
			coax_fw_end_cmd_to_isp_send(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
	case IOC_VDEC_COAX_FW_END_ACK_GET:
			if (copy_from_user(&coax_fw_val, argp, sizeof(FIRMWARE_UP_FILE_INFO)))
				printk("IOC_VDEC_COAX_FW_START_CMD_SET error\n");
			coax_fw_end_cmd_ack_from_isp_recv(&coax_fw_val);
			cpy2usr_ret = copy_to_user(argp, &coax_fw_val, sizeof(FIRMWARE_UP_FILE_INFO));
			break;
			/*===============================================================================================
			 * Test Function
			 *===============================================================================================
	case IOC_VDEC_COAX_TEST_TX_INIT_DATA_READ:
			if (copy_from_user(&coax_test_val, argp, sizeof(NC_VD_COAX_TEST_STR)))
				printk("IOC_VDEC_COAX_INIT_SET error\n");
			coax_test_tx_init_read(&coax_test_val);
			cpy2usr_ret = copy_to_user(argp, &coax_test_val, sizeof(NC_VD_COAX_TEST_STR));
			break;
	case IOC_VDEC_COAX_TEST_DATA_SET:
			if (copy_from_user(&coax_test_val, argp, sizeof(NC_VD_COAX_TEST_STR)))
				printk("IOC_VDEC_COAX_TEST_DATA_SET error\n");
			coax_test_data_set(&coax_test_val);
			break;
	case IOC_VDEC_COAX_TEST_DATA_READ:
			if (copy_from_user(&coax_test_val, argp, sizeof(NC_VD_COAX_TEST_STR)))
				printk("IOC_VDEC_COAX_TEST_DATA_SET error\n");
			coax_test_data_get(&coax_test_val);
			cpy2usr_ret = copy_to_user(argp, &coax_test_val, sizeof(NC_VD_COAX_TEST_STR));
			break;
			/*===============================================================================================
			 * Motion
			 *===============================================================================================
	case IOC_VDEC_MOTION_DETECTION_GET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_SET error\n");
			motion_detection_get(&motion_set);
			cpy2usr_ret = copy_to_user(argp, &motion_set, sizeof(motion_mode));
			break;
	case IOC_VDEC_MOTION_SET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_SET error\n");
			motion_onoff_set(&motion_set);
			break;
	case IOC_VDEC_MOTION_PIXEL_SET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_Pixel_SET error\n");
			motion_pixel_onoff_set(&motion_set);
			break;
	case IOC_VDEC_MOTION_PIXEL_GET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_Pixel_SET error\n");
			motion_pixel_onoff_get(&motion_set);
			cpy2usr_ret = copy_to_user(argp, &motion_set, sizeof(motion_mode));
			break;
	case IOC_VDEC_MOTION_ALL_PIXEL_SET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_Pixel_SET error\n");
			motion_pixel_all_onoff_set(&motion_set);
			break;
	case IOC_VDEC_MOTION_TSEN_SET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_TSEN_SET error\n");
			motion_tsen_set(&motion_set);
			break;
	case IOC_VDEC_MOTION_PSEN_SET:
			if (copy_from_user(&motion_set, argp, sizeof(motion_set)))
				printk("IOC_VDEC_MOTION_PSEN_SET error\n");
			motion_psen_set(&motion_set);
			break;
	}

	up(&jaguar1_lock);

	return 0;
}
*/

/*******************************************************************************
 *	End of file
 *******************************************************************************/
