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

#include <stdio.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "strfunc.h"

int g_fd_i2c = -1;


/*************************************************************************************************
 * IMX6 DD Test Program
 * Bank Read
 **************************************************************************************************/
int _I2C_Open(int port_num)
{
	int i2cfile = (-1);

	char devpath[64];

	sprintf(devpath, "/dev/i2c-%d", port_num);
	/* For sensor I2C speed up */
	i2cfile = open(devpath, O_RDWR);
	printf("i2cfile::%d\n", i2cfile);
	return i2cfile;
}



int _I2C_SendWordEx(int fd_i2c, unsigned int DevAddr, unsigned int reg, unsigned int DataWord, int SizeOfRegWord, int SizeOfDataWord)
{
	unsigned int i2c_Addr = DevAddr;
	int err = 0;
	int ret = 0;

	struct i2c_msg I2CMsg[2];
	struct i2c_rdwr_ioctl_data I2CIOCTLData;

	if (fd_i2c < 0) {
		printf("I2C Send : open error\n");
		return (-1);
	}

	printf("I2C Send : send line(%d) [%x][%x][%x]\n", __LINE__, DevAddr, reg, DataWord);
	unsigned char tmpWrbuf[4 + 4];
	int Tlen;
	unsigned char *pT = &tmpWrbuf[0];

	Tlen = SizeOfRegWord;
	/* MSB First */
	while (Tlen--) {
		unsigned char dat = (reg >> (Tlen * 8)) & 0xFF;
		/* unsigned char dat = ( reg >> ( (SizeOfRegWord-1-Tlen) * 8 ) ) & 0xFF; */
		*pT++ = dat;
	}

	Tlen = SizeOfDataWord;
#if VIN_FALSE /* MSB First */
	while (Tlen--) {
		unsigned char dat = (DataWord >> (Tlen * 8)) & 0xFF;
		/* unsigned char dat = ( DataWord >> ( (SizeOfDataWord-1-Tlen) * 8 ) ) & 0xFF; */
		*pT++ = dat;
	}
#else /* LSB First */
	while (Tlen--) {
		/* unsigned char dat = ( DataWord >> ( Tlen * 8 ) ) & 0xFF;
		unsigned char dat = ( DataWord >> ( (SizeOfDataWord-1-Tlen) * 8 ) ) & 0xFF; */
		*pT++ = DataWord & 0xFF;
		DataWord >>= 8;
	}
#endif

	int WriteLen = SizeOfRegWord + SizeOfDataWord;
#if 1
	I2CMsg[0].addr = DevAddr>>1;
#else
	I2CMsg[0].addr = DevAddr;
#endif
	I2CMsg[0].flags = 0; /*  Send */
	I2CMsg[0].len = WriteLen;
	I2CMsg[0].buf = (char *)tmpWrbuf;

	I2CIOCTLData.msgs = I2CMsg;
	I2CIOCTLData.nmsgs = 1;
	printf("I2C Send : send line(%d) [%x][%x][%x]\n", __LINE__, DevAddr, reg, I2CMsg[0].buf);
	err = ioctl(fd_i2c, I2C_RDWR, &I2CIOCTLData);
	if (err < 0) {
		printf("I2C Send : send error(%d) [%x][%x][%x]\n", err, DevAddr, reg, DataWord);
		printf("ERROR:%d=%s\n", errno, strerror(errno));
		ret = (-1);
	}
	return ret;
}

int _I2C_ReadWordEx(int fd_i2c, unsigned int DevAddr, unsigned int reg, int SizeOfRegWord, int SizeOfDataWord)
{
	unsigned int i2c_Addr = DevAddr;
	int err = 0;

	struct i2c_msg I2CMsg[2];
	struct i2c_rdwr_ioctl_data I2CIOCTLData;

	int RetData = 0;

	if (fd_i2c < 0) {
		printf("I2C Read_3 : open error\n");
		return (-1);
	}

	/* if(i2c_core==3) */
	/* printf("XAPPIF : I2C Read : [%x][%x]\n",DevAddr,reg ); */

	unsigned char tmpRegWrbuf[4 + 4];

	unsigned char *pT = &tmpRegWrbuf[0];
	int Tlen = SizeOfRegWord;

	/* MSB First */
	while (Tlen--) {
		unsigned char dat = (reg >> (Tlen * 8)) & 0xFF;
		*pT++ = dat;
	}

	I2CMsg[0].addr = DevAddr>>1;
	I2CMsg[0].flags = 0; /* Send */
	I2CMsg[0].len = SizeOfRegWord;
	I2CMsg[0].buf = (char *)&tmpRegWrbuf;

	I2CMsg[1].addr = DevAddr>>1;
	I2CMsg[1].flags = I2C_M_RD; /* Read */
	I2CMsg[1].len = SizeOfDataWord;
	I2CMsg[1].buf = (char *)&RetData;

	I2CIOCTLData.msgs = I2CMsg;
	I2CIOCTLData.nmsgs = 2;
	err = ioctl(fd_i2c, I2C_RDWR, &I2CIOCTLData);
	if (err < 0) {
		printf("I2C Read : read error(%d) [%x][%x]\n", err, DevAddr, reg);
		return (-1);
	}

/*
	unsigned char RetSwap = 0;
	Tlen = SizeOfDataWord;

	pT = &RetSwap;
	/* MSB First
	while (Tlen--) {
		unsigned char dat = (RetData >> (Tlen * 8)) & 0xFF;
		*pT++ = dat;
	}
	RetData = RetSwap;
*/
	return RetData;
}

#define _IMX274_RD(addr, reg) _I2C_ReadWordEx(g_fd_i2c, addr, reg, 1, 1)
#define _IMX274_WR(addr, reg, data) _I2C_SendWordEx(g_fd_i2c, addr, reg, data, 1, 1)

int main(int argc, char *argv[])
{
	int ret;
	unsigned int device_addr, reg_addr, reg_value;

	if (argc < 3) {
		printf("usage: dec <device_addr> <reg_addr> <data_value>\n");
		return -1;
	}

	g_fd_i2c = _I2C_Open(1);
	if (g_fd_i2c < 0) {
		printf("Open imx6 dev error!\n");
		return -1;
	}


	if (StrToNumber(argv[1], &device_addr)) {
		close(g_fd_i2c);
		return 0;
	}
	if (StrToNumber(argv[2], &reg_addr)) {
		close(g_fd_i2c);
		return 0;
	}

	if (argc == 3) {
		ret = _IMX274_RD(device_addr, reg_addr);
		if (ret < 0) {
			printf("CMD_I2C_READ error!\n");
			close(g_fd_i2c);
			return -1;
		}
		printf("value =  0x%x\n", ret);
	} else if (argc == 4) {
		if (StrToNumber(argv[3], &reg_value)) {
			close(g_fd_i2c);
			return 0;
		}
		printf("dev_addr:0x%02x; reg_addr:0x%02x; reg_value:0x%02x\n", device_addr, reg_addr, reg_value);
		ret = _IMX274_WR(device_addr, reg_addr, reg_value);
	}

	close(g_fd_i2c);

	return ret;
}
