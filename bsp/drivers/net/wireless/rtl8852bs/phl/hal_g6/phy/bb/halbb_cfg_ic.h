/******************************************************************************
 *
 * Copyright(c) 2007 - 2020  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/
#ifndef __HALBB_CFG_IC_H__
#define __HALBB_CFG_IC_H__

#include "halbb_ic_sw_info.h"


#ifdef CONFIG_RTL8852A
	//#define BB_8852A_CAV_SUPPORT /*CAV*/
	#define BB_8852A_2_SUPPORT /*> CBV*/
#endif

#if defined(CONFIG_RTL8852B) || defined(CONFIG_RTL8852BP)
	#define BB_8852B_SUPPORT
#endif

#if defined(CONFIG_RTL8852C) || defined(CONFIG_RTL8852D)
	#define BB_8852C_SUPPORT
	#ifdef CONFIG_RTL8852D
	#define BB_8852D_SUPPORT
	#endif
#endif

#if defined(CONFIG_RTL8192XB) || defined(CONFIG_RTL8832BR)
	#define BB_8192XB_SUPPORT
#endif

#ifdef CONFIG_RTL8851B
	#define BB_8851B_SUPPORT
	#define HALBB_DUAL_SAR_LMT_TB_8851B
#endif

#if (HLABB_CODE_BASE_NUM == 32) //will be removed when (022+032) branch phase out
	#ifdef CONFIG_RTL8922A
	#define BB_1115_SUPPORT
	#define BB_1115_DVLP_SPF
	#define BB_1115_DVLP_SPF_2

	#define BB_8922_DVLP_SPF
	#define BB_8922_DVLP_SPF_2
	#endif
#else
#ifdef CONFIG_RTL8922A
	//#define BB_8922A_SUPPORT
#endif
	#define BB_8922A_DVLP_SPF_2
#endif


#endif

