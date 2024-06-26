/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs g2d driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


#ifndef __G2D_MIXER_REGS_H
#define __G2D_MIXER_REGS_H

/* module base addr */
/*
 *#define G2D_TOP        (0x00000 + G2D_BASE)
 *#define G2D_MIXER      (0x00100 + G2D_BASE)
 *#define G2D_BLD        (0x00400 + G2D_BASE)
 *#define G2D_V0         (0x00800 + G2D_BASE)
 *#define G2D_UI0        (0x01000 + G2D_BASE)
 *#define G2D_UI1        (0x01800 + G2D_BASE)
 *#define G2D_UI2        (0x02000 + G2D_BASE)
 *#define G2D_WB         (0x03000 + G2D_BASE)
 *#define G2D_VSU        (0x08000 + G2D_BASE)
 *#define G2D_ROT        (0x28000 + G2D_BASE)
 *#define G2D_GSU        (0x30000 + G2D_BASE)
 */
#define G2D_TOP        (0x00000)
#define G2D_MIXER      (0x00100)
#define G2D_BLD        (0x00400)
#define G2D_V0         (0x00800)
#define G2D_UI0        (0x01000)
#define G2D_UI1        (0x01800)
#define G2D_UI2        (0x02000)
#define G2D_WB         (0x03000)
#define G2D_VSU        (0x08000)
#define G2D_ROT        (0x28000)
#define G2D_GSU        (0x30000)

/* register offset */
/* TOP register */
#define G2D_SCLK_GATE  (0x00 + G2D_TOP)
#define G2D_HCLK_GATE  (0x04 + G2D_TOP)
#define G2D_AHB_RESET  (0x08 + G2D_TOP)
#define G2D_SCLK_DIV   (0x0C + G2D_TOP)
#define G2D_IP_VERSION (0x10 + G2D_TOP)

/* MIXER GLB register */
#define G2D_MIXER_CTL  (0x00 + G2D_MIXER)
#define G2D_MIXER_INT  (0x04 + G2D_MIXER)
#define G2D_MIXER_CLK  (0x08 + G2D_MIXER)

/* LAY VIDEO register */
#define V0_ATTCTL      (0x00 + G2D_V0)
#define V0_MBSIZE      (0x04 + G2D_V0)
#define V0_COOR        (0x08 + G2D_V0)
#define V0_PITCH0      (0x0C + G2D_V0)
#define V0_PITCH1      (0x10 + G2D_V0)
#define V0_PITCH2      (0x14 + G2D_V0)
#define V0_LADD0       (0x18 + G2D_V0)
#define V0_LADD1       (0x1C + G2D_V0)
#define V0_LADD2       (0x20 + G2D_V0)
#define V0_FILLC       (0x24 + G2D_V0)
#define V0_HADD        (0x28 + G2D_V0)
#define V0_SIZE        (0x2C + G2D_V0)
#define V0_HDS_CTL0    (0x30 + G2D_V0)
#define V0_HDS_CTL1    (0x34 + G2D_V0)
#define V0_VDS_CTL0    (0x38 + G2D_V0)
#define V0_VDS_CTL1    (0x3C + G2D_V0)

/* LAY0 UI register */
#define UI0_ATTR       (0x00 + G2D_UI0)
#define UI0_MBSIZE     (0x04 + G2D_UI0)
#define UI0_COOR       (0x08 + G2D_UI0)
#define UI0_PITCH      (0x0C + G2D_UI0)
#define UI0_LADD       (0x10 + G2D_UI0)
#define UI0_FILLC      (0x14 + G2D_UI0)
#define UI0_HADD       (0x18 + G2D_UI0)
#define UI0_SIZE       (0x1C + G2D_UI0)

/* LAY1 UI register */
#define UI1_ATTR       (0x00 + G2D_UI1)
#define UI1_MBSIZE     (0x04 + G2D_UI1)
#define UI1_COOR       (0x08 + G2D_UI1)
#define UI1_PITCH      (0x0C + G2D_UI1)
#define UI1_LADD       (0x10 + G2D_UI1)
#define UI1_FILLC      (0x14 + G2D_UI1)
#define UI1_HADD       (0x18 + G2D_UI1)
#define UI1_SIZE       (0x1C + G2D_UI1)

/* LAY2 UI register */
#define UI2_ATTR       (0x00 + G2D_UI2)
#define UI2_MBSIZE     (0x04 + G2D_UI2)
#define UI2_COOR       (0x08 + G2D_UI2)
#define UI2_PITCH      (0x0C + G2D_UI2)
#define UI2_LADD       (0x10 + G2D_UI2)
#define UI2_FILLC      (0x14 + G2D_UI2)
#define UI2_HADD       (0x18 + G2D_UI2)
#define UI2_SIZE       (0x1C + G2D_UI2)

/* VSU register */
#define VS_CTRL           (0x000 + G2D_VSU)
#define VS_OUT_SIZE       (0x040 + G2D_VSU)
#define VS_GLB_ALPHA      (0x044 + G2D_VSU)
#define VS_Y_SIZE         (0x080 + G2D_VSU)
#define VS_Y_HSTEP        (0x088 + G2D_VSU)
#define VS_Y_VSTEP        (0x08C + G2D_VSU)
#define VS_Y_HPHASE       (0x090 + G2D_VSU)
#define VS_Y_VPHASE0      (0x098 + G2D_VSU)
#define VS_C_SIZE         (0x0C0 + G2D_VSU)
#define VS_C_HSTEP        (0x0C8 + G2D_VSU)
#define VS_C_VSTEP        (0x0CC + G2D_VSU)
#define VS_C_HPHASE       (0x0D0 + G2D_VSU)
#define VS_C_VPHASE0      (0x0D8 + G2D_VSU)
#define VS_Y_HCOEF0       (0x200 + G2D_VSU)
#define VS_Y_VCOEF0       (0x300 + G2D_VSU)
#define VS_C_HCOEF0       (0x400 + G2D_VSU)

/* BLD register */
#define BLD_EN_CTL         (0x000 + G2D_BLD)
#define BLD_FILLC0         (0x010 + G2D_BLD)
#define BLD_FILLC1         (0x014 + G2D_BLD)
#define BLD_CH_ISIZE0      (0x020 + G2D_BLD)
#define BLD_CH_ISIZE1      (0x024 + G2D_BLD)
#define BLD_CH_OFFSET0     (0x030 + G2D_BLD)
#define BLD_CH_OFFSET1     (0x034 + G2D_BLD)
#define BLD_PREMUL_CTL     (0x040 + G2D_BLD)
#define BLD_BK_COLOR       (0x044 + G2D_BLD)
#define BLD_SIZE           (0x048 + G2D_BLD)
#define BLD_CTL            (0x04C + G2D_BLD)
#define BLD_KEY_CTL        (0x050 + G2D_BLD)
#define BLD_KEY_CON        (0x054 + G2D_BLD)
#define BLD_KEY_MAX        (0x058 + G2D_BLD)
#define BLD_KEY_MIN        (0x05C + G2D_BLD)
#define BLD_OUT_COLOR      (0x060 + G2D_BLD)
#define ROP_CTL            (0x080 + G2D_BLD)
#define ROP_INDEX0         (0x084 + G2D_BLD)
#define ROP_INDEX1         (0x088 + G2D_BLD)
#define BLD_CSC_CTL        (0x100 + G2D_BLD)
#define BLD_CSC0_COEF00    (0x110 + G2D_BLD)
#define BLD_CSC0_COEF01    (0x114 + G2D_BLD)
#define BLD_CSC0_COEF02    (0x118 + G2D_BLD)
#define BLD_CSC0_CONST0    (0x11C + G2D_BLD)
#define BLD_CSC0_COEF10    (0x120 + G2D_BLD)
#define BLD_CSC0_COEF11    (0x124 + G2D_BLD)
#define BLD_CSC0_COEF12    (0x128 + G2D_BLD)
#define BLD_CSC0_CONST1    (0x12C + G2D_BLD)
#define BLD_CSC0_COEF20    (0x130 + G2D_BLD)
#define BLD_CSC0_COEF21    (0x134 + G2D_BLD)
#define BLD_CSC0_COEF22    (0x138 + G2D_BLD)
#define BLD_CSC0_CONST2    (0x13C + G2D_BLD)
#define BLD_CSC1_COEF00    (0x140 + G2D_BLD)
#define BLD_CSC1_COEF01    (0x144 + G2D_BLD)
#define BLD_CSC1_COEF02    (0x148 + G2D_BLD)
#define BLD_CSC1_CONST0    (0x14C + G2D_BLD)
#define BLD_CSC1_COEF10    (0x150 + G2D_BLD)
#define BLD_CSC1_COEF11    (0x154 + G2D_BLD)
#define BLD_CSC1_COEF12    (0x158 + G2D_BLD)
#define BLD_CSC1_CONST1    (0x15C + G2D_BLD)
#define BLD_CSC1_COEF20    (0x160 + G2D_BLD)
#define BLD_CSC1_COEF21    (0x164 + G2D_BLD)
#define BLD_CSC1_COEF22    (0x168 + G2D_BLD)
#define BLD_CSC1_CONST2    (0x16C + G2D_BLD)
#define BLD_CSC2_COEF00    (0x170 + G2D_BLD)
#define BLD_CSC2_COEF01    (0x174 + G2D_BLD)
#define BLD_CSC2_COEF02    (0x178 + G2D_BLD)
#define BLD_CSC2_CONST0    (0x17C + G2D_BLD)
#define BLD_CSC2_COEF10    (0x180 + G2D_BLD)
#define BLD_CSC2_COEF11    (0x184 + G2D_BLD)
#define BLD_CSC2_COEF12    (0x188 + G2D_BLD)
#define BLD_CSC2_CONST1    (0x18C + G2D_BLD)
#define BLD_CSC2_COEF20    (0x190 + G2D_BLD)
#define BLD_CSC2_COEF21    (0x194 + G2D_BLD)
#define BLD_CSC2_COEF22    (0x198 + G2D_BLD)
#define BLD_CSC2_CONST2    (0x19C + G2D_BLD)

/* WB register */
#define WB_ATT             (0x00 + G2D_WB)
#define WB_SIZE            (0x04 + G2D_WB)
#define WB_PITCH0          (0x08 + G2D_WB)
#define WB_PITCH1          (0x0C + G2D_WB)
#define WB_PITCH2          (0x10 + G2D_WB)
#define WB_LADD0           (0x14 + G2D_WB)
#define WB_HADD0           (0x18 + G2D_WB)
#define WB_LADD1           (0x1C + G2D_WB)
#define WB_HADD1           (0x20 + G2D_WB)
#define WB_LADD2           (0x24 + G2D_WB)
#define WB_HADD2           (0x28 + G2D_WB)

/* Rotate register */
#define ROT_CTL            (0x00 + G2D_ROT)
#define ROT_INT            (0x04 + G2D_ROT)
#define ROT_TIMEOUT        (0x08 + G2D_ROT)
#define ROT_IFMT           (0x20 + G2D_ROT)
#define ROT_ISIZE          (0x24 + G2D_ROT)
#define ROT_IPITCH0        (0x30 + G2D_ROT)
#define ROT_IPITCH1        (0x34 + G2D_ROT)
#define ROT_IPITCH2        (0x38 + G2D_ROT)
#define ROT_ILADD0         (0x40 + G2D_ROT)
#define ROT_IHADD0         (0x44 + G2D_ROT)
#define ROT_ILADD1         (0x48 + G2D_ROT)
#define ROT_IHADD1         (0x4C + G2D_ROT)
#define ROT_ILADD2         (0x50 + G2D_ROT)
#define ROT_IHADD2         (0x54 + G2D_ROT)
#define ROT_OSIZE          (0x84 + G2D_ROT)
#define ROT_OPITCH0        (0x90 + G2D_ROT)
#define ROT_OPITCH1        (0x94 + G2D_ROT)
#define ROT_OPITCH2        (0x98 + G2D_ROT)
#define ROT_OLADD0         (0xA0 + G2D_ROT)
#define ROT_OHADD0         (0xA4 + G2D_ROT)
#define ROT_OLADD1         (0xA8 + G2D_ROT)
#define ROT_OHADD1         (0xAC + G2D_ROT)
#define ROT_OLADD2         (0xB0 + G2D_ROT)
#define ROT_OHADD2         (0xB4 + G2D_ROT)

/* #define write_wvalue(addr, data) m_usbwordwrite32(  addr, data ) */
/* #define write_wvalue(addr, v) put_wvalue(addr, v) */
/* #define read_wvalue(addr) get_wvalue(addr) */

/* byte input */
#define get_bvalue(n)	(*((volatile __u8 *)(n)))
/* byte output */
#define put_bvalue(n, c)	(*((volatile __u8 *)(n)) = (c))
/* half word input */
#define get_hvalue(n)	(*((volatile __u16 *)(n)))
/* half word output */
#define put_hvalue(n, c)	(*((volatile __u16 *)(n)) = (c))
/* word input */
#define get_wvalue(n)	(*((volatile __u32 *)(n)))
/* word output */
#define put_wvalue(n, c)	(*((volatile __u32 *)(n)) = (c))

#endif /*
 */
