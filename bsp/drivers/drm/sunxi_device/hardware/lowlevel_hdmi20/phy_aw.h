/* SPDX-License-Identifier: GPL-2.0-or-later */
/*******************************************************************************
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 ******************************************************************************/
#ifndef AW_PHY_H_
#define AW_PHY_H_

#define AW_PHY_TIMEOUT	1000
#define PHY_REG_OFFSET	0x10000

/* allwinner phy register offset */
#define HDMI_PHY_CTL0				0x40
#define HDMI_PHY_CTL1				0x44
#define HDMI_PHY_CTL2				0x48
#define HDMI_PHY_CTL3				0x4C
#define HDMI_PHY_CTL4				0x50
#define HDMI_PHY_CTL5				0x54
#define HDMI_PLL_CTL0				0x58
#define HDMI_PLL_CTL1				0x5C
#define HDMI_AFIFO_CFG				0x60
#define HDMI_MODULATOR_CFG0			0x64
#define HDMI_MODULATOR_CFG1			0x68
#define HDMI_PHY_INDEB_CTRL			0x6C
#define HDMI_PHY_INDBG_TXD0			0x70
#define HDMI_PHY_INDBG_TXD1			0x74
#define HDMI_PHY_INDBG_TXD2			0x78
#define HDMI_PHY_INDBG_TXD3			0x7C
#define HDMI_PHY_PLL_STS			0x80
#define HDMI_PRBS_CTL				0x84
#define HDMI_PRBS_SEED_GEN			0x88
#define HDMI_PRBS_SEED_CHK			0x8C
#define HDMI_PRBS_SEED_NUM			0x90
#define HDMI_PRBS_CYCLE_NUM			0x94
#define HDMI_PHY_PLL_ODLY_CFG			0x98
#define HDMI_PHY_CTL6				0x9C
#define HDMI_PHY_CTL7				0xA0

typedef union {
	u32 dwval;
	struct {
		u32 sda_en		:1;    /* Default: 0; */
		u32 scl_en		:1;    /* Default: 0; */
		u32 hpd_en		:1;    /* Default: 0; */
		u32 res0		:1;    /* Default: 0; */
		u32 reg_ck_sel		:1;    /* Default: 1; */
		u32 reg_ck_test_sel	:1;    /* Default: 1; */
		u32 reg_csmps		:2;    /* Default: 0; */
		u32 reg_den		:4;    /* Default: F; */
		u32 reg_plr		:4;    /* Default: 0; */
		u32 enck		:1;    /* Default: 1; */
		u32 enldo_fs		:1;    /* Default: 1; */
		u32 enldo		:1;    /* Default: 1; */
		u32 res1		:1;    /* Default: 1; */
		u32 enbi		:4;    /* Default: F; */
		u32 entx		:4;    /* Default: F; */
		u32 async_fifo_autosync_disable	:1;    /* Default: 0; */
		u32 async_fifo_workc_enable	:1;    /* Default: 1; */
		u32 phy_pll_lock_mode		:1;    /* Default: 1; */
		u32 phy_pll_lock_mode_man	:1;    /* Default: 1; */
	} bits;
} HDMI_PHY_CTL0_t;      /* ===========================    0x0040 */

typedef union {
	u32 dwval;
	struct {
		u32 reg_sp2_0				:   4  ;    /* Default: 0; */
		u32 reg_sp2_1				:   4  ;    /* Default: 0; */
		u32 reg_sp2_2				:   4  ;    /* Default: 0; */
		u32 reg_sp2_3				:   4  ;    /* Default: 0; */
		u32 reg_bst0				:   2  ;    /* Default: 3; */
		u32 reg_bst1				:   2  ;    /* Default: 3; */
		u32 reg_bst2				:   2  ;    /* Default: 3; */
		u32 res0				:   2  ;    /* Default: 0; */
		u32 reg_svr				:   2  ;    /* Default: 2; */
		u32 reg_swi				:   1  ;    /* Default: 0; */
		u32 res_scktmds				:   1  ;    /* Default: 0; */
		u32 res_res_s				:   2  ;    /* Default: 3; */
		u32 phy_rxsense_mode			:   1  ;    /* Default: 0; */
		u32 res_rxsense_mode_man		:   1  ;    /* Default: 0; */
	} bits;
} HDMI_PHY_CTL1_t;      /* =====================================================    0x0044 */

typedef union {
	u32 dwval;
	struct {
		u32 reg_p2opt				:   4  ;    /* Default: 0; */
		u32 reg_sp1_0				:   5  ;    /* Default: 0; */
		u32 reg_sp1_1				:   5  ;    /* Default: 0; */
		u32 reg_sp1_2				:   5  ;    /* Default: 0; */
		u32 reg_sp1_3				:   5  ;    /* Default: 0; */
		u32 reg_resdi				:   6  ;    /* Default: 18; */
		u32 phy_hpdo_mode			:   1  ;    /* Default: 0; */
		u32 phy_hpdo_mode_man			:   1  ;    /* Default: 0; */
	} bits;
} HDMI_PHY_CTL2_t;      /* =====================================================    0x0048 */



typedef union {
	u32 dwval;
	struct {
		u32 reg_mc0				:   4  ;    /* Default: F; */
		u32 reg_mc1				:   4  ;    /* Default: F; */
		u32 reg_mc2				:   4  ;    /* Default: F; */
		u32 reg_mc3				:   4  ;    /* Default: F; */
		u32 reg_p2_0				:   4  ;    /* Default: F; */
		u32 reg_p2_1				:   4  ;    /* Default: F; */
		u32 reg_p2_2				:   4  ;    /* Default: F; */
		u32 reg_p2_3				:   4  ;    /* Default: F; */
	} bits;
} HDMI_PHY_CTL3_t;      /* =====================================================    0x004C */



typedef union {
	u32 dwval;
	struct {
		u32 reg_p1_0				:   5  ;    /* Default: 0x10; */
		u32 res0				:   3  ;    /* Default: 0; */
		u32 reg_p1_1				:   5  ;    /* Default: 0x10; */
		u32 res1				:   3  ;    /* Default: 0; */
		u32 reg_p1_2				:   5  ;    /* Default: 0x10; */
		u32 res2				:   3  ;    /* Default: 0; */
		u32 reg_p1_3				:   5  ;    /* Default: 0x10; */
		u32 reg_slv				:   3  ;    /* Default: 0; */
	} bits;
} HDMI_PHY_CTL4_t;      /* =====================================================    0x0050 */

typedef union {
	u32 dwval;
	struct {
		u32 encalog				:   1  ;    /* Default: 0x1; */
		u32 enib				:   1  ;    /* Default: 0x1; */
		u32 res0				:   2  ;    /* Default: 0; */
		u32 enp2s				:   4  ;    /* Default: 0xF; */
		u32 enrcal				:   1  ;    /* Default: 0x1; */
		u32 enres				:   1  ;    /* Default: 1; */
		u32 enresck				:   1  ;    /* Default: 1; */
		u32 reg_calsw				:   1  ;    /* Default: 0; */
		u32 reg_ckpdlyopt			:   1  ;    /* Default: 0; */
		u32 res1				:   3  ;    /* Default: 0; */
		u32 reg_p1opt				:   4  ;    /* Default: 0; */
		u32 res2				:  12  ;    /* Default: 0; */
	} bits;
} HDMI_PHY_CTL5_t;      /* =====================================================    0x0054 */

typedef union {
	u32 dwval;
	struct {
		u32 prop_cntrl				:   3  ;    /* Default: 0x7; */
		u32 res0				:   1  ;    /* Default: 0; */
		u32 gmp_cntrl				:   2  ;    /* Default: 1; */
		u32 n_cntrl				:   2  ;    /* Default: 0; */
		u32 vcorange				:   1  ;    /* Default: 0; */
		u32 sdrven				:   1  ;    /* Default: 0; */
		u32 divx1				:   1  ;    /* Default: 0; */
		u32 res1				:   1  ;    /* Default: 0; */
		u32 div_pre				:   4  ;    /* Default: 0; */
		u32 div2_cktmds				:   1  ;    /* Default: 1; */
		u32 div2_ckbit				:   1  ;    /* Default: 1; */
		u32 cutfb				:   1  ;    /* Default: 0; */
		u32 res2				:   1  ;    /* Default: 0; */
		u32 clr_dpth				:   2  ;    /* Default: 0; */
		u32 bypass_clrdpth			:   1  ;    /* Default: 0; */
		u32 bcr					:   1  ;    /* Default: 0; */
		u32 slv					:   3  ;    /* Default: 4; */
		u32 res3				:   1  ;    /* Default: 0; */
		u32 envbs				:   1  ;    /* Default: 0; */
		u32 bypass_ppll				:   1  ;    /* Default: 0; */
		u32 cko_sel				:   2  ;    /* Default: 0; */
	} bits;
} HDMI_PLL_CTL0_t;      /* =====================================================    0x0058 */



typedef union {
	u32 dwval;
	struct {
		u32 int_cntrl				:   3  ;    /* Default: 0x0; */
		u32 res0				:   1  ;    /* Default: 0; */
		u32 ref_cntrl				:   2  ;    /* Default: 3; */
		u32 gear_shift				:   1  ;    /* Default: 0; */
		u32 fast_tech				:   1  ;    /* Default: 0; */
		u32 drv_ana				:   1  ;    /* Default: 1; */
		u32 sckfb				:   1  ;    /* Default: 0; */
		u32 sckref				:   1  ;    /* Default: 0; */
		u32 reset				:   1  ;    /* Default: 0; */
		u32 pwron				:   1  ;    /* Default: 0; */
		u32 res1				:   3  ;    /* Default: 0; */
		u32 pixel_rep				:   2  ;    /* Default: 0; */
		u32 sdm_en				:   1  ;    /* Default: 0; */
		u32 pcnt_en				:   1  ;    /* Default: 0; */
		u32 pcnt_n				:   8  ;    /* Default: 0xE; */
		u32 res2				:   3  ;    /* Default: 0; */
		u32 ctrl_modle_clksrc			:   1  ;    /* Default: 0; */
	} bits;
} HDMI_PLL_CTL1_t;      /* =====================================================    0x005C */

typedef union {
	u32 dwval;
	struct {
		u32 hdmi_afifo_error	:   1  ;    /* Default: 0x0; */
		u32 hdmi_afifo_error_det	:   1  ;    /* Default: 0x0; */
		u32 res0			:  30  ;    /* Default: 0; */
	} bits;
} HDMI_AFIFO_CFG_t;      /* =====================================================    0x0060 */

typedef union {
	u32 dwval;
	struct {
		u32 fnpll_mash_en		:   1  ;    /* Default: 0x0; */
		u32 fnpll_mash_mod		:   2  ;    /* Default: 0x0; */
		u32 fnpll_mash_stp		:   9  ;    /* Default: 0x0; */
		u32 fnpll_mash_m12		:   1  ;    /* Default: 0x0; */
		u32 fnpll_mash_frq		:   2  ;    /* Default: 0x0; */
		u32 fnpll_mash_bot		:  17  ;    /* Default: 0x0; */
	} bits;
} HDMI_MODULATOR_CFG0_t;      /* =====================================================    0x0064 */

typedef union {
	u32 dwval;
	struct {
		u32 fnpll_mash_dth		:   1  ;    /* Default: 0x0; */
		u32 fnpll_mash_fen		:   1  ;    /* Default: 0x0; */
		u32 fnpll_mash_frc		:  17  ;    /* Default: 0x0; */
		u32 fnpll_mash_fnv		:   8  ;    /* Default: 0x0; */
		u32 res0			:   5  ;    /* Default: 0x0; */
	} bits;
} HDMI_MODULATOR_CFG1_t;      /* =====================================================    0x0068 */

typedef union {
	u32 dwval;
	struct {
		u32 txdata_debugmode		:   2  ;    /* Default: 0x0; */
		u32 res0			:  14  ;    /* Default: 0x0; */
		u32 ceci_debug			:   1  ;    /* Default: 0x0; */
		u32 ceci_debugmode		:   1  ;    /* Default: 0x0; */
		u32 res1			:   2  ;    /* Default: 0x0; */
		u32 sdai_debug			:   1  ;    /* Default: 0x0; */
		u32 sdai_debugmode		:   1  ;    /* Default: 0x0; */
		u32 res2			:   2  ;    /* Default: 0x0; */
		u32 scli_debug			:   1  ;    /* Default: 0x0; */
		u32 scli_debugmode		:   1  ;    /* Default: 0x0; */
		u32 res3			:   2  ;    /* Default: 0x0; */
		u32 hpdi_debug			:   1  ;    /* Default: 0x0; */
		u32 hpdi_debugmode		:   1  ;    /* Default: 0x0; */
		u32 res4			:   2  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_INDBG_CTRL_t;      /* ==================================================    0x006C */

typedef union {
	u32 dwval;
	struct {
		u32 txdata0_debug_data	    :  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_INDBG_TXD0_t;      /* ==================================================    0x0070 */

typedef union {
	u32 dwval;
	struct {
		u32 txdata1_debug_data	    :  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_INDBG_TXD1_t;      /* ==================================================    0x0074 */

typedef union {
	u32 dwval;
	struct {
		u32 txdata2_debug_data	    :  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_INDBG_TXD2_t;      /* ==================================================    0x0078 */

typedef union {
	u32 dwval;
	struct {
		u32 txdata3_debug_data	    :  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_INDBG_TXD3_t;      /* ==================================================    0x007C */

typedef union {
	u32 dwval;
	struct {
		u32 tx_ready_dly_status	:   1  ;    /* Default: 0x0; */
		u32 rxsense_dly_status		:   1  ;    /* Default: 0x0; */
		u32 res0			:   2  ;    /* Default: 0x0; */
		u32 pll_lock_status		:   1  ;    /* Default: 0x0; */
		u32 res1			:   3  ;    /* Default: 0x0; */
		u32 phy_resdo2d_status		:   6  ;    /* Default: 0x0; */
		u32 res2			:   2  ;    /* Default: 0x0; */
		u32 phy_rcalend2d_status	:   1  ;    /* Default: 0x0; */
		u32 phy_cout2d_status		:   1  ;    /* Default: 0x0; */
		u32 res3			:   2  ;    /* Default: 0x0; */
		u32 phy_ceco_status		:   1  ;    /* Default: 0x0; */
		u32 phy_sdao_status		:   1  ;    /* Default: 0x0; */
		u32 phy_sclo_status		:   1  ;    /* Default: 0x0; */
		u32 phy_hpdo_status		:   1  ;    /* Default: 0x0; */
		u32 phy_cdetn_status		:   3  ;    /* Default: 0x0; */
		u32 phy_cdetnck_status		:   1  ;    /* Default: 0x0; */
		u32 phy_cdetp_status		:   3  ;    /* Default: 0x0; */
		u32 phy_cdetpck_status		:   1  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_PLL_STS_t;      /* =====================================================    0x0080 */

typedef union {
	u32 dwval;
	struct {
		u32 prbs_en				:   1  ;    /* Default: 0x0; */
		u32 prbs_start				:   1  ;    /* Default: 0x0; */
		u32 prbs_seq_gen			:   1  ;    /* Default: 0x0; */
		u32 prbs_seq_chk			:   1  ;    /* Default: 0x0; */
		u32 prbs_mode				:   4  ;    /* Default: 0x0; */
		u32 prbs_type				:   2  ;    /* Default: 0x0; */
		u32 prbs_clk_pol			:   1  ;    /* Default: 0x0; */
		u32 res0				:  21  ;    /* Default: 0x0; */
	} bits;
} HDMI_PRBS_CTL_t;      /* =====================================================    0x0084 */

typedef union {
	u32 dwval;
	struct {
		u32 prbs_seed_gen			:  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PRBS_SEED_GEN_t;      /* =================================================    0x0088 */

typedef union {
	u32 dwval;
	struct {
		u32 prbs_seed_chk			:  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PRBS_SEED_CHK_t;      /* =================================================    0x008C */

typedef union {
	u32 dwval;
	struct {
		u32 prbs_seed_num			:  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PRBS_SEED_NUM_t;      /* =================================================    0x0090 */

typedef union {
	u32 dwval;
	struct {
		u32 prbs_cycle_num			:  32  ;    /* Default: 0x0; */
	} bits;
} HDMI_PRBS_CYCLE_NUM_t;      /* =================================================    0x0094 */

typedef union {
	u32 dwval;
	struct {
		u32	tx_ready_dly_count		:  15  ;    /* Default: 0x0; */
		u32	tx_ready_dly_reset		:   1  ;    /* Default: 0x0; */
		u32	rxsense_dly_count		:  15  ;    /* Default: 0x0; */
		u32	rxsense_dly_reset		:   1  ;    /* Default: 0x0; */
	} bits;
} HDMI_PHY_PLL_ODLY_CFG_t;      /* =================================================    0x0098 */

typedef union {
	u32 dwval;
	struct {
		u32	clk_greate0_340m		:  10  ;    /* Default: 0x3FF; */
		u32	clk_greate1_340m		:  10  ;    /* Default: 0x3FF; */
		u32	clk_greate2_340m		:  10  ;    /* Default: 0x3FF; */
		u32	en_ckdat				:   1  ;    /* Default: 0x3FF; */
		u32	switch_clkch_data_corresponding	:   1  ;    /* Default: 0x3FF; */
	} bits;
} HDMI_PHY_CTL6_t;      /* =========================================================    0x009C */

typedef union {
	u32 dwval;
	struct {
		u32	clk_greate3_340m		:  10  ;    /* Default: 0x0; */
		u32	res0				:   2  ;    /* Default: 0x3FF; */
		u32	clk_low_340m			:  10  ;    /* Default: 0x3FF; */
		u32	res1				:  10  ;    /* Default: 0x3FF; */
	} bits;
} HDMI_PHY_CTL7_t;      /* =========================================================    0x00A0 */

struct __aw_phy_reg_t {
	u32 res[16];		/* 0x0 ~ 0x3c */
	HDMI_PHY_CTL0_t		phy_ctl0; /* 0x0040 */
	HDMI_PHY_CTL1_t		phy_ctl1; /* 0x0044 */
	HDMI_PHY_CTL2_t		phy_ctl2; /* 0x0048 */
	HDMI_PHY_CTL3_t		phy_ctl3; /* 0x004c */
	HDMI_PHY_CTL4_t		phy_ctl4; /* 0x0050 */
	HDMI_PHY_CTL5_t		phy_ctl5; /* 0x0054 */
	HDMI_PLL_CTL0_t		pll_ctl0; /* 0x0058 */
	HDMI_PLL_CTL1_t		pll_ctl1; /* 0x005c */
	HDMI_AFIFO_CFG_t	afifo_cfg; /* 0x0060 */
	HDMI_MODULATOR_CFG0_t	modulator_cfg0; /* 0x0064 */
	HDMI_MODULATOR_CFG1_t	modulator_cfg1; /* 0x0068 */
	HDMI_PHY_INDBG_CTRL_t	phy_indbg_ctrl;	/* 0x006c */
	HDMI_PHY_INDBG_TXD0_t	phy_indbg_txd0; /* 0x0070 */
	HDMI_PHY_INDBG_TXD1_t	phy_indbg_txd1; /* 0x0074 */
	HDMI_PHY_INDBG_TXD2_t	phy_indbg_txd2; /* 0x0078 */
	HDMI_PHY_INDBG_TXD3_t	phy_indbg_txd3; /* 0x007c */
	HDMI_PHY_PLL_STS_t	phy_pll_sts; /* 0x0080 */
	HDMI_PRBS_CTL_t		prbs_ctl;	/* 0x0084 */
	HDMI_PRBS_SEED_GEN_t	prbs_seed_gen;	/* 0x0088 */
	HDMI_PRBS_SEED_CHK_t	prbs_seed_chk;	/* 0x008c */
	HDMI_PRBS_SEED_NUM_t	prbs_seed_num;	/* 0x0090 */
	HDMI_PRBS_CYCLE_NUM_t	prbs_cycle_num;	/* 0x0094 */
	HDMI_PHY_PLL_ODLY_CFG_t	phy_pll_odly_cfg; /* 0x0098 */
	HDMI_PHY_CTL6_t		phy_ctl6;	/* 0x009c */
	HDMI_PHY_CTL7_t		phy_ctl7;	/* 0x00A0 */
};

struct aw_phy_params {
	u32			clock;/* phy clock: unit:kHZ */
	dw_pixel_repetition_t	pixel;
	dw_color_depth_t		color;
	u32			phy_ctl1;
	u32			phy_ctl2;
	u32			phy_ctl3;
	u32			phy_ctl4;
	u32			pll_ctl0;
	u32			pll_ctl1;
};

int aw_phy_write(u8 addr, void *data);

int aw_phy_read(u8 addr, void *data);

void aw_phy_set_reg_base(uintptr_t base);

uintptr_t aw_phy_get_reg_base(void);

int aw_phy_reset(void);

int aw_phy_resume(void);

int aw_phy_config(void);

int aw_phy_init(void);

#endif	/* AW_PHY_H_ */
