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

#include "../../../utility/vin_log.h"
#include "common.h"
#include "video.h"

#include "eq.h"
#include "eq_common.h"
#include "eq_recovery.h"

/*******************************************************************************
 * extern variable
 *******************************************************************************/
extern unsigned int nvp6134_iic_addr[4];
extern unsigned char ch_mode_status[16];
extern unsigned char ch_vfmt_status[16];
extern unsigned int nvp6134_cnt;
extern unsigned int g_slp_ahd[16];

/*******************************************************************************
 * internal variable
 *******************************************************************************/
static volatile unsigned int min_acc[MAX_CHANNEL_NUM] = {
	0xFFFF,
};
static volatile unsigned int max_acc[MAX_CHANNEL_NUM] = {
	0x0000,
};
static volatile unsigned int min_ypstage[MAX_CHANNEL_NUM] = {
	0xFFFF,
};
static volatile unsigned int max_ypstage[MAX_CHANNEL_NUM] = {
	0x0000,
};
static volatile unsigned int min_ymstage[MAX_CHANNEL_NUM] = {
	0xFFFF,
};
static volatile unsigned int max_ymstage[MAX_CHANNEL_NUM] = {
	0x0000,
};

static volatile unsigned int min_fr_ac_min[MAX_CHANNEL_NUM] = {
	0xFFFF,
};
static volatile unsigned int max_fr_ac_min[MAX_CHANNEL_NUM] = {
	0x0000,
};
static volatile unsigned int min_fr_ac_max[MAX_CHANNEL_NUM] = {
	0xFFFF,
};
static volatile unsigned int max_fr_ac_max[MAX_CHANNEL_NUM] = {
	0x0000,
};
static volatile unsigned int min_dc[MAX_CHANNEL_NUM] = {
	0xFFFF,
};
static volatile unsigned int max_dc[MAX_CHANNEL_NUM] = {
	0x0000,
};

static unsigned int eq_chk_cnt[MAX_CHANNEL_NUM] = {
	0,
};

static int bInitEQ = EQ_INIT_OFF; /* EQ initialize(structure) */
nvp6134_equalizer s_eq;		  /* EQ manager structure */
nvp6134_equalizer s_eq_bak;       /* EQ manager structure */

unsigned char video_fmt_debounce_eq(unsigned char ch, unsigned char value)
{
	unsigned char tmp, buf[2] = {0, 0};
	unsigned char reg_vfmt_5C;
	unsigned char reg_vfmt_F2 = 0;
	unsigned char reg_vfmt_F3 = 0;
	unsigned char reg_vfmt_F4 = 0;
	unsigned char reg_vfmt_F5 = 0;
	int i;

	for (i = 0; i < 2; i++) {
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
		tmp = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF0);
		reg_vfmt_F4 = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF4);
		reg_vfmt_F5 = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF5);

		if (tmp == 0x6F) /* AHD 3M detection */ {
			tmp = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF3);
			reg_vfmt_F2 =
			    gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF2);
		} else if ((reg_vfmt_F5 == 0x06) &&
			   ((reg_vfmt_F4 == 0x30) ||
			    (reg_vfmt_F4 == 0x31))) /* TVI  3M vcnt 0xF0 == 0xFF */ {
			tmp = 0x64; /* EXT  3M 18P */
		}

		if (tmp == 0x7F) /* AHD 5M detection */ {
			reg_vfmt_F2 =
			    gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF2);
			reg_vfmt_F3 =
			    gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF3);

			if (reg_vfmt_F3 == 0x04) /* 5M 12_5P */
				tmp = 0xA0;
			else if (reg_vfmt_F3 == 0x02) /* 5M 20P */
				tmp = 0xA1;
		}

		buf[i] = nvp6134_vfmt_convert(tmp, reg_vfmt_F2);

		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + (ch % 4));
		reg_vfmt_5C = (gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0x5C));
		if ((reg_vfmt_5C == 0x20) || (reg_vfmt_5C == 0x21) ||
		    (reg_vfmt_5C == 0x22) || (reg_vfmt_5C == 0x23) ||
		    (reg_vfmt_5C == 0x30) || (reg_vfmt_5C == 0x31)) {
			buf[i] = nvp6134_vfmt_convert(reg_vfmt_5C, reg_vfmt_F2);
		}

		else {
			if (isItAHDmode(buf[i])) {
				buf[i] = trans_ahd_to_chd(buf[i]);
			}
		}
		msleep(10);
	}
	/* printk("video_fmt_debounce ch[%d] */
	/* buf[0][%x],buf[1][%x],buf[2][%x]\n", ch, buf[0], buf[1], buf[2]); */
	tmp = value;
	if ((tmp == buf[0]) && (tmp == buf[1]))
		return tmp;
	else
		return buf[1];
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
*	Description		: EQ configuration value
*	Argurments		:
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
void eq_init_each_format(unsigned char ch, int mode, const unsigned char vfmt)
{
	/* turn off Analog by-pass */
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
	gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x59, 0x11);

	s_eq.ch_stage[ch] = 0; /* Set default stage to 0. */

	switch (mode) {
	case NVP6134_VI_720H:
	case NVP6134_VI_960H:
	case NVP6134_VI_1280H:
	case NVP6134_VI_1440H:
	case NVP6134_VI_1920H:
	case NVP6134_VI_960H2EX:
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x01, 0x00);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x58, 0x00);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x59, 0x00);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x5B, 0x03);

		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x00);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x00 + ch % 4, 0x00);
		/* printk(">>>>> DRV : CH:%d, EQ init, NVP6134_VI_SD, */
		/* NVP6134_VI_SD - conf\n", ch); */
		break;
	case NVP6134_VI_720P_2530: /* HD AHD  @ 30P */
	case NVP6134_VI_720P_5060: /* HD AHD  @ 25P */
	case NVP6134_VI_HDEX:      /* HD AHD EX */
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC0,
			       0x16); /* TX_PAT_STR */
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC1,
			       0x11); /* ACC_GAIN_STS_SEL & TX_PAT_WIDTH */
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC8,
			       0x04); /* SLOPE_VALUE_2S */

		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
			       ch % 4 < 2 ? 0x0A : 0x0B);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4],
			       ch % 2 == 0 ? 0x74 : 0xF4,
			       0x02); /* chX_EQ_SRC_SEL */
		/* printk(">>>>> DRV : CH:%d, EQ init, NVP6134_VI_720P_2530, */
		/* NVP6134_VI_720P_5060 - conf\n", ch); */
		break;
	case NVP6134_VI_3M_NRT:
	case NVP6134_VI_3M:
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC0,
			       0x17); /* change TX_PATTERN START */
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC1, 0x13);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC8, 0x04);

		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
			       ch % 4 < 2 ? 0x0A : 0x0B);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4],
			       ch % 2 == 0 ? 0x74 : 0xF4, 0x02);
		/* printk(">>>>> DRV : CH:%d, EQ init, NVP6134_VI_3M_NRT, */
		/* NVP6134_VI_3M - conf\n", ch); */
		break;
	case NVP6134_VI_1080P_2530: /* FHD AHD */
	case NVP6134_VI_5M_NRT:
	case NVP6134_VI_4M:
	case NVP6134_VI_4M_NRT:
	case NVP6134_VI_5M_20P:
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + ch % 4);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC0, 0x17);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC1, 0x13);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xC8, 0x04);

		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
			       ch % 4 < 2 ? 0x0A : 0x0B);
		gpio_i2c_write(nvp6134_iic_addr[ch / 4],
			       ch % 2 == 0 ? 0x74 : 0xF4, 0x02);
		/* printk(">>>>> DRV : CH:%d, EQ init, NVP6134_VI_1080P_2530 - */
		/* conf\n", ch); */
		break;
	}
}

/*******************************************************************************
*	Description		: Adjust Equalizer(EQ)
*	Argurments		: ch: channel information
*	Return value	: void
*	Modify			:
*	warning			:
*******************************************************************************/
int nvp6134_set_equalizer(unsigned char ch)
{
	unsigned int acc_gain_status[16], y_plus_slope[16], y_minus_slope[16];
	unsigned char vloss;
	unsigned char vidformat;
	/* unsigned char agc_lock; */
	unsigned char agc_ckcnt = 0;
	unsigned char reg_vfmt_F2 = 0;
	unsigned char reg_vfmt_F3 = 0;
	unsigned char reg_vfmt_F4 = 0;
	unsigned char reg_vfmt_F5 = 0;
	/* int i; */

	/*
	 * Initialization structure
	 */
	if (bInitEQ == EQ_INIT_OFF) {
		memset(&s_eq, 0x00, sizeof(nvp6134_equalizer));
		memset(&s_eq_bak, 0x00, sizeof(nvp6134_equalizer));
		bInitEQ = EQ_INIT_ON;
	}

	/*
	 * exception
	 * skip under NVP6134_VI_720P_2530 mode and over NVP6134_VI_BUTT
	 */
	if (ch_mode_status[ch] >= NVP6134_VI_BUTT) {
		s_eq.ch_previdmode[ch] = 0xFF;
		s_eq.ch_curvidmode[ch] = 0xFF;
		s_eq.ch_previdon[ch] = EQ_VIDEO_OFF;
		s_eq.ch_curvidon[ch] = EQ_VIDEO_OFF;
		s_eq.ch_previdformat[ch] =
		    0xFF; /* pre video format for Auto detection value */
		s_eq.ch_curvidformat[ch] =
		    0xFF; /* current video format for Auto detection value */
		s_eq.ch_vfmt_status[ch] =
		    PAL; /* default(PAL) : NTSC(0), PAL(1)ch_vfmt_status */
		return 0;
	}

	/*
	 * check and set video loss and video on/off information to buffer
	 */
	if (1) {
		/* get video format(video loss), 0:videoloss, 1:video on */
		gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF, 0x05 + (ch % 4));
		vidformat = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF0);
		reg_vfmt_F4 = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF4);
		reg_vfmt_F5 = gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF5);
		if (vidformat == 0x6F) {
			vidformat =
			    gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF3);
		} else if ((reg_vfmt_F5 == 0x06) &&
			   ((reg_vfmt_F4 == 0x30) ||
			    (reg_vfmt_F4 ==
			     0x31))) /* TVI  3M vcnt  0x0F == 0xFF */ {
			vidformat = 0x93; /* EXT  3M 18P */
		}

		if (vidformat == 0x7F) /* AHD 5M detection */ {
			reg_vfmt_F2 =
			    gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF2);
			reg_vfmt_F3 =
			    gpio_i2c_read(nvp6134_iic_addr[ch / 4], 0xF3);

			if (reg_vfmt_F3 == 0x04) /* 5M 12_5P */
				vidformat = 0xA0;
			else if (reg_vfmt_F3 == 0x02) /* 5M 20P */
				vidformat = 0xA1;
		}
		vidformat = video_fmt_debounce_eq(ch, vidformat);

		if (vidformat == 0xFF)
			vloss = 0;
		else
			vloss = 1;

		/* get agc locking information(signal)*/
		/* gpio_i2c_write(nvp6134_iic_addr[ch/4], 0xFF, 0x00); */
		/* agc_lock = gpio_i2c_read(nvp6134_iic_addr[ch/4],0xE0); */

		/* after checking agc locking(signal) and video loss, save Video
		 * ON information to buffer */
		if ((vloss == 1) && (nvp6134_GetAgcLockStatus(ch) == 0x01)) {
			s_eq.ch_curvidon[ch] = EQ_VIDEO_ON;
			s_eq.ch_curvidmode[ch] = ch_mode_status[ch];
			s_eq.ch_curvidformat[ch] = vidformat;
			s_eq.ch_vfmt_status[ch] =
			    ch_vfmt_status[ch]; /* NTSC/PAL */
			if (nvp6134_GetFSCLockStatus(ch) == 0 &&
			    (ch_mode_status[ch] != NVP6134_VI_1080P_NOVIDEO)) {
				nvp6134_ResetFSCLock(ch);
			}
		} else {
			/* These are default value of NO video */
			s_eq.ch_curvidmode[ch] = 0xFF;
			s_eq.ch_curvidon[ch] = EQ_VIDEO_OFF;
			s_eq.ch_curvidformat[ch] = 0xFF;
			s_eq.ch_vfmt_status[ch] = PAL;
			s_eq.ch_stage[ch] = 0;
			g_slp_ahd[ch] = 0;
			eq_chk_cnt[ch] = 0;
			/* set default value for distinguishing between EXC and
			 * AHD */
			gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
				       0x05 + (ch % 4));
			gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x5C, 0x00);
			gpio_i2c_write(
			    nvp6134_iic_addr[ch / 4], 0xB8,
			    0xB9); /* recover this for video loss detection */
			nvp6134_hide_ch(ch);
		}
	}

	if ((1 == s_eq_bak.ch_man_eq_flag[ch]) ||
	    ((0 == s_eq_bak.ch_stage[ch]) &&
	     (2 == s_eq_bak.ch_man_eq_flag[ch]))) {
		s_eq.ch_stage[ch] = 0;
	}
	/*
	 * it only want to set EQ(algorithm) once when there is the video
	 * signal.
	 *    - PRE(video off) != CUR(video on)
	 *    - PRE(video format) != CUR(Video ON)
	 */
	if ((s_eq.ch_curvidon[ch] != EQ_VIDEO_OFF) &&
	    (s_eq.ch_curvidmode[ch] != NVP6134_VI_1080P_NOVIDEO)) {
		if ((s_eq.ch_previdon[ch] != s_eq.ch_curvidon[ch]) ||
		    (s_eq.ch_previdmode[ch] != s_eq.ch_curvidmode[ch]) ||
		    (s_eq.ch_stage[ch] == 0)) {

			/* AHD */
			{
				/* AHD */
				acc_gain_status[ch] = GetAccGain(ch);
				y_plus_slope[ch] = GetYPlusSlope(ch);
				y_minus_slope[ch] = GetYMinusSlope(ch);

				if (acc_gain_status[ch] < min_acc[ch])
					min_acc[ch] = acc_gain_status[ch];
				else if (acc_gain_status[ch] > max_acc[ch])
					max_acc[ch] = acc_gain_status[ch];

				if (y_plus_slope[ch] < min_ypstage[ch])
					min_ypstage[ch] = y_plus_slope[ch];
				else if (y_plus_slope[ch] > max_ypstage[ch])
					max_ypstage[ch] = y_plus_slope[ch];

				if (y_minus_slope[ch] < min_ymstage[ch])
					min_ymstage[ch] = y_minus_slope[ch];
				else if (y_minus_slope[ch] > max_ymstage[ch])
					max_ymstage[ch] = y_minus_slope[ch];

				/* printk("---------------DRV: CH:%d, AFHD_AHD */
				/* --------------------\n", ch); */
				/* printk("gain_status 	cur=%3d min=%3d */
				/* max=%3d\n",acc_gain_status[ch], min_acc[ch], */
				/* max_acc[ch]); */
				/* printk("y_plus_slope    cur=%3d min=%3d */
				/* max=%3d\n",y_plus_slope[ch],min_ypstage[ch], */
				/* max_ypstage[ch]); */
				/* printk("y_minus_slope   cur=%3d min=%3d */
				/* max=%3d\n",y_minus_slope[ch],min_ymstage[ch], */
				/* max_ymstage[ch]); */

				/* To recovery FSC Locking Miss Situation -
				 * Btype is AHD, CHD */
				__eq_recovery_Btype(ch, s_eq.ch_curvidmode[ch],
						    s_eq.ch_vfmt_status[ch],
						    acc_gain_status[ch],
						    y_minus_slope[ch],
						    y_plus_slope[ch]);

				/* save distance information to buffer for
				 * display */
				s_eq.acc_gain[ch] = acc_gain_status[ch];
				s_eq.y_plus_slope[ch] = y_plus_slope[ch];
				s_eq.y_minus_slope[ch] = y_minus_slope[ch];

				if (s_eq.ch_curvidmode[ch] ==
					NVP6134_VI_720P_2530 ||
				    s_eq.ch_curvidmode[ch] == NVP6134_VI_HDEX) {
					if ((y_plus_slope[ch] == 0) &&
					    (y_minus_slope[ch] == 0) &&
					    (g_slp_ahd[ch] == 0))
						g_slp_ahd[ch] = 1;
				}
				if (g_slp_ahd[ch] == 1)
					s_eq.ch_stage[ch] = eq_get_stage(
					    ch, s_eq.ch_curvidmode[ch],
					    acc_gain_status[ch], 0 /* Not used */,
					    0 /* Not used */,
					    s_eq.ch_vfmt_status[ch]);
				else
					s_eq.ch_stage[ch] = eq_get_stage(
					    ch, s_eq.ch_curvidmode[ch],
					    y_plus_slope[ch], y_minus_slope[ch],
					    0 /* Not used */,
					    s_eq.ch_vfmt_status[ch]);

				/* manual set eq stage */
				if (1 == s_eq_bak.ch_man_eq_flag[ch])
					s_eq.ch_stage[ch] =
					    s_eq_bak.ch_stage[ch];

				/* adjust EQ depend on distance */
				eq_adjust_eqvalue(ch, s_eq.ch_curvidmode[ch],
						  s_eq.ch_vfmt_status[ch],
						  s_eq.ch_stage[ch]);

				y_plus_slope[ch] = GetYPlusSlope(ch);
				y_minus_slope[ch] = GetYMinusSlope(ch);
				g_slp_ahd[ch] = 0;
			}

			/* after getting EQ value, Analog filter bypass option
			 * is off */
			gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
				       0x05 + (ch % 4));
			gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0x59, 0x00);
			gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xB8, 0x38);
/* printk(">>>>> DRV : getting EQ value, Analog filter bypass option is off\n" */
/* ); */
#if 1
			msleep(35);
			while ((nvp6134_GetAgcLockStatus(ch) == 0) &&
			       ((agc_ckcnt++) < 10)) {
				gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xFF,
					       0x05 + (ch % 4));
				gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xB8,
					       0xB8);
				msleep(10);
				gpio_i2c_write(nvp6134_iic_addr[ch / 4], 0xB8,
					       0x38);
				msleep(35);
				printk(">>>>> DRV : agc re-init cnt = %d\n",
				       agc_ckcnt);
			}
#endif
			nvp6134_show_ch(ch);
		} else {
			{
				/* AHD */
				s_eq.cur_acc_gain[ch] = GetAccGain(ch);
				s_eq.cur_y_plus_slope[ch] = GetYPlusSlope(ch);
				s_eq.cur_y_minus_slope[ch] = GetYMinusSlope(ch);
			}
		}
	}

	/*
	 * save current status to pre buffer(video on/off, video format)
	 */
	if (1) {
		s_eq.ch_previdon[ch] = s_eq.ch_curvidon[ch];
		s_eq.ch_previdmode[ch] = s_eq.ch_curvidmode[ch];
		s_eq.ch_previdformat[ch] = s_eq.ch_curvidformat[ch];
	}

	return 0;
}

/*******************************************************************************
*	End of file
*******************************************************************************/
