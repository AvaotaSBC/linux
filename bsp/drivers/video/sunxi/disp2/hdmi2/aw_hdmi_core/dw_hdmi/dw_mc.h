/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright(c) 2020 - 2023 Allwinner Technology Co.,Ltd. All rights reserved. */
/*
 * Allwinner SoCs hdmi2.0 driver.
 *
 * Copyright (C) 2016 Allwinner.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


#ifndef _DW_MC_H_
#define _DW_MC_H_

#include "dw_dev.h"

#define ALL_IRQ_MASK 	0xff

typedef enum irq_sources {
	IRQ_AUDIO_PACKETS = 1,
	IRQ_OTHER_PACKETS,
	IRQ_PACKETS_OVERFLOW,
	IRQ_AUDIO_SAMPLER,
	IRQ_PHY,
	IRQ_I2C_DDC,
	IRQ_CEC,
	IRQ_VIDEO_PACKETIZER,
	IRQ_I2C_PHY,
	IRQ_AUDIO_DMA,
} irq_sources_t;

/*****************************************************************************
 *                                                                           *
 *                         Main Controller Registers                         *
 *                                                                           *
 *****************************************************************************/

/* Main Controller Synchronous Clock Domain Disable Register */
#define MC_CLKDIS  0x00010004
#define MC_CLKDIS_PIXELCLK_DISABLE_MASK  0x00000001 /* Pixel clock synchronous disable signal */
#define MC_CLKDIS_TMDSCLK_DISABLE_MASK  0x00000002 /* TMDS clock synchronous disable signal */
#define MC_CLKDIS_PREPCLK_DISABLE_MASK  0x00000004 /* Pixel Repetition clock synchronous disable signal */
#define MC_CLKDIS_AUDCLK_DISABLE_MASK  0x00000008 /* Audio Sampler clock synchronous disable signal */
#define MC_CLKDIS_CSCCLK_DISABLE_MASK  0x00000010 /* Color Space Converter clock synchronous disable signal */
#define MC_CLKDIS_CECCLK_DISABLE_MASK  0x00000020 /* CEC Engine clock synchronous disable signal */
#define MC_CLKDIS_HDCPCLK_DISABLE_MASK  0x00000040 /* HDCP clock synchronous disable signal */

/* Main Controller Software Reset Register Main controller software reset request per clock domain */
#define MC_SWRSTZREQ  0x00010008
#define MC_SWRSTZREQ_PIXELSWRST_REQ_MASK  0x00000001 /* Pixel software reset request */
#define MC_SWRSTZREQ_TMDSSWRST_REQ_MASK  0x00000002 /* TMDS software reset request */
#define MC_SWRSTZREQ_PREPSWRST_REQ_MASK  0x00000004 /* Pixel Repetition software reset request */
#define MC_SWRSTZREQ_II2SSWRST_REQ_MASK  0x00000008 /* I2S audio software reset request */
#define MC_SWRSTZREQ_ISPDIFSWRST_REQ_MASK  0x00000010 /* SPDIF audio software reset request */
#define MC_SWRSTZREQ_CECSWRST_REQ_MASK  0x00000040 /* CEC software reset request */
#define MC_SWRSTZREQ_IGPASWRST_REQ_MASK  0x00000080 /* GPAUD interface soft reset request */

/* Main Controller HDCP Bypass Control Register */
#define MC_OPCTRL  0x0001000C
#define MC_OPCTRL_HDCP_BLOCK_BYP_MASK  0x00000001 /* Block HDCP bypass mechanism - 1'b0: This is the default value */

/* Main Controller Feed Through Control Register */
#define MC_FLOWCTRL  0x00010010
#define MC_FLOWCTRL_FEED_THROUGH_OFF_MASK  0x00000001 /* Video path Feed Through enable bit: - 1b: Color Space Converter is in the video data path */

/* Main Controller PHY Reset Register */
#define MC_PHYRSTZ  0x00010014
#define MC_PHYRSTZ_PHYRSTZ_MASK  0x00000001 /* HDMI Source PHY active low reset control for PHY GEN1, active high reset control for PHY GEN2 */

/* Main Controller Clock Present Register */
#define MC_LOCKONCLOCK  0x00010018
#define MC_LOCKONCLOCK_CECCLK_MASK  0x00000001 /* CEC clock status */
#define MC_LOCKONCLOCK_AUDIOSPDIFCLK_MASK  0x00000004 /* SPDIF clock status */
#define MC_LOCKONCLOCK_I2SCLK_MASK  0x00000008 /* I2S clock status */
#define MC_LOCKONCLOCK_PREPCLK_MASK  0x00000010 /* Pixel Repetition clock status */
#define MC_LOCKONCLOCK_TCLK_MASK  0x00000020 /* TMDS clock status */
#define MC_LOCKONCLOCK_PCLK_MASK  0x00000040 /* Pixel clock status */
#define MC_LOCKONCLOCK_IGPACLK_MASK  0x00000080 /* GPAUD interface clock status */


/*****************************************************************************
 *                                                                           *
 *                            Interrupt Registers                            *
 *                                                                           *
 *****************************************************************************/

/* Frame Composer Interrupt Status Register 0 (Packet Interrupts) */
#define IH_FC_STAT0  0x00000400
#define IH_FC_STAT0_NULL_MASK  0x00000001 /* Active after successful transmission of an Null packet */
#define IH_FC_STAT0_ACR_MASK  0x00000002 /* Active after successful transmission of an Audio Clock Regeneration (N/CTS transmission) packet */
#define IH_FC_STAT0_AUDS_MASK  0x00000004 /* Active after successful transmission of an Audio Sample packet */
#define IH_FC_STAT0_NVBI_MASK  0x00000008 /* Active after successful transmission of an NTSC VBI packet */
#define IH_FC_STAT0_MAS_MASK  0x00000010 /* Active after successful transmission of an MultiStream Audio packet */
#define IH_FC_STAT0_HBR_MASK  0x00000020 /* Active after successful transmission of an Audio HBR packet */
#define IH_FC_STAT0_ACP_MASK  0x00000040 /* Active after successful transmission of an Audio Content Protection packet */
#define IH_FC_STAT0_AUDI_MASK  0x00000080 /* Active after successful transmission of an Audio InfoFrame packet */

/* Frame Composer Interrupt Status Register 1 (Packet Interrupts) */
#define IH_FC_STAT1  0x00000404
#define IH_FC_STAT1_GCP_MASK  0x00000001 /* Active after successful transmission of an General Control Packet */
#define IH_FC_STAT1_AVI_MASK  0x00000002 /* Active after successful transmission of an AVI InfoFrame packet */
#define IH_FC_STAT1_AMP_MASK  0x00000004 /* Active after successful transmission of an Audio Metadata packet */
#define IH_FC_STAT1_SPD_MASK  0x00000008 /* Active after successful transmission of an Source Product Descriptor InfoFrame packet */
#define IH_FC_STAT1_VSD_MASK  0x00000010 /* Active after successful transmission of an Vendor Specific Data InfoFrame packet */
#define IH_FC_STAT1_ISCR2_MASK  0x00000020 /* Active after successful transmission of an International Standard Recording Code 2 packet */
#define IH_FC_STAT1_ISCR1_MASK  0x00000040 /* Active after successful transmission of an International Standard Recording Code 1 packet */
#define IH_FC_STAT1_GMD_MASK  0x00000080 /* Active after successful transmission of an Gamut metadata packet */

/* Frame Composer Interrupt Status Register 2 (Packet Queue Overflow Interrupts) */
#define IH_FC_STAT2  0x00000408
#define IH_FC_STAT2_HIGHPRIORITY_OVERFLOW_MASK  0x00000001 /* Frame Composer high priority packet queue descriptor overflow indication */
#define IH_FC_STAT2_LOWPRIORITY_OVERFLOW_MASK  0x00000002 /* Frame Composer low priority packet queue descriptor overflow indication */

/* Audio Sampler Interrupt Status Register (FIFO Threshold, Underflow and Overflow Interrupts) */
#define IH_AS_STAT0  0x0000040C
#define IH_AS_STAT0_AUD_FIFO_OVERFLOW_MASK  0x00000001 /* Audio Sampler audio FIFO full indication */
#define IH_AS_STAT0_AUD_FIFO_UNDERFLOW_MASK  0x00000002 /* Audio Sampler audio FIFO empty indication */
#define IH_AS_STAT0_AUD_FIFO_UNDERFLOW_THR_MASK  0x00000004 /* Audio Sampler audio FIFO empty threshold (four samples) indication for the legacy HBR audio interface */
#define IH_AS_STAT0_FIFO_OVERRUN_MASK  0x00000008 /* Indicates an overrun on the audio FIFO */

/* PHY Interface Interrupt Status Register (RXSENSE, PLL Lock and HPD Interrupts) */
#define IH_PHY_STAT0  0x00000410
#define IH_PHY_STAT0_HPD_MASK  0x00000001 /* HDMI Hot Plug Detect indication */
#define IH_PHY_STAT0_TX_PHY_LOCK_MASK  0x00000002 /* TX PHY PLL lock indication */
#define IH_PHY_STAT0_RX_SENSE_0_MASK  0x00000004 /* TX PHY RX_SENSE indication for driver 0 */
#define IH_PHY_STAT0_RX_SENSE_1_MASK  0x00000008 /* TX PHY RX_SENSE indication for driver 1 */
#define IH_PHY_STAT0_RX_SENSE_2_MASK  0x00000010 /* TX PHY RX_SENSE indication for driver 2 */
#define IH_PHY_STAT0_RX_SENSE_3_MASK  0x00000020 /* TX PHY RX_SENSE indication for driver 3 */

/* E-DDC I2C Master Interrupt Status Register (Done and Error Interrupts) */
#define IH_I2CM_STAT0  0x00000414
#define IH_I2CM_STAT0_I2CMASTERERROR_MASK  0x00000001 /* I2C Master error indication */
#define IH_I2CM_STAT0_I2CMASTERDONE_MASK  0x00000002 /* I2C Master done indication */
#define IH_I2CM_STAT0_SCDC_READREQ_MASK  0x00000004 /* I2C Master SCDC read request indication */

/* CEC Interrupt Status Register (Functional Operation Interrupts) */
#define IH_CEC_STAT0  0x00000418
#define IH_CEC_STAT0_DONE_MASK  0x00000001 /* CEC Done Indication */
#define IH_CEC_STAT0_EOM_MASK  0x00000002 /* CEC End of Message Indication */
#define IH_CEC_STAT0_NACK_MASK  0x00000004 /* CEC Not Acknowledge indication */
#define IH_CEC_STAT0_ARB_LOST_MASK  0x00000008 /* CEC Arbitration Lost indication */
#define IH_CEC_STAT0_ERROR_INITIATOR_MASK  0x00000010 /* CEC Error Initiator indication */
#define IH_CEC_STAT0_ERROR_FOLLOW_MASK  0x00000020 /* CEC Error Follow indication */
#define IH_CEC_STAT0_WAKEUP_MASK  0x00000040 /* CEC Wake-up indication */

/* Video Packetizer Interrupt Status Register (FIFO Full and Empty Interrupts) */
#define IH_VP_STAT0  0x0000041C
#define IH_VP_STAT0_FIFOEMPTYBYP_MASK  0x00000001 /* Video Packetizer 8 bit bypass FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLBYP_MASK  0x00000002 /* Video Packetizer 8 bit bypass FIFO full interrupt */
#define IH_VP_STAT0_FIFOEMPTYREMAP_MASK  0x00000004 /* Video Packetizer pixel YCC 422 re-mapper FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLREMAP_MASK  0x00000008 /* Video Packetizer pixel YCC 422 re-mapper FIFO full interrupt */
#define IH_VP_STAT0_FIFOEMPTYPP_MASK  0x00000010 /* Video Packetizer pixel packing FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLPP_MASK  0x00000020 /* Video Packetizer pixel packing FIFO full interrupt */
#define IH_VP_STAT0_FIFOEMPTYREPET_MASK  0x00000040 /* Video Packetizer pixel repeater FIFO empty interrupt */
#define IH_VP_STAT0_FIFOFULLREPET_MASK  0x00000080 /* Video Packetizer pixel repeater FIFO full interrupt */

/* PHY GEN2 I2C Master Interrupt Status Register (Done and Error Interrupts) */
#define IH_I2CMPHY_STAT0  0x00000420
#define IH_I2CMPHY_STAT0_I2CMPHYERROR_MASK  0x00000001 /* I2C Master PHY error indication */
#define IH_I2CMPHY_STAT0_I2CMPHYDONE_MASK  0x00000002 /* I2C Master PHY done indication */

/* DMA - not supported in this build */
#define IH_AHBDMAAUD_STAT0  0x00000424

/* Interruption Handler Decode Assist Register */
#define IH_DECODE  0x000005C0
#define IH_DECODE_IH_AHBDMAAUD_STAT0_MASK  0x00000001 /* Interruption active at the ih_ahbdmaaud_stat0 register */
#define IH_DECODE_IH_CEC_STAT0_MASK  0x00000002 /* Interruption active at the ih_cec_stat0 register */
#define IH_DECODE_IH_I2CM_STAT0_MASK  0x00000004 /* Interruption active at the ih_i2cm_stat0 register */
#define IH_DECODE_IH_PHY_MASK  0x00000008 /* Interruption active at the ih_phy_stat0 or ih_i2cmphy_stat0 register */
#define IH_DECODE_IH_AS_STAT0_MASK  0x00000010 /* Interruption active at the ih_as_stat0 register */
#define IH_DECODE_IH_FC_STAT2_VP_MASK  0x00000020 /* Interruption active at the ih_fc_stat2 or ih_vp_stat0 register */
#define IH_DECODE_IH_FC_STAT1_MASK  0x00000040 /* Interruption active at the ih_fc_stat1 register */
#define IH_DECODE_IH_FC_STAT0_MASK  0x00000080 /* Interruption active at the ih_fc_stat0 register */

/* Frame Composer Interrupt Mute Control Register 0 */
#define IH_MUTE_FC_STAT0  0x00000600
#define IH_MUTE_FC_STAT0_NULL_MASK  0x00000001 /* When set to 1, mutes ih_fc_stat0[0] */
#define IH_MUTE_FC_STAT0_ACR_MASK  0x00000002 /* When set to 1, mutes ih_fc_stat0[1] */
#define IH_MUTE_FC_STAT0_AUDS_MASK  0x00000004 /* When set to 1, mutes ih_fc_stat0[2] */
#define IH_MUTE_FC_STAT0_NVBI_MASK  0x00000008 /* When set to 1, mutes ih_fc_stat0[3] */
#define IH_MUTE_FC_STAT0_MAS_MASK  0x00000010 /* When set to 1, mutes ih_fc_stat0[4] */
#define IH_MUTE_FC_STAT0_HBR_MASK  0x00000020 /* When set to 1, mutes ih_fc_stat0[5] */
#define IH_MUTE_FC_STAT0_ACP_MASK  0x00000040 /* When set to 1, mutes ih_fc_stat0[6] */
#define IH_MUTE_FC_STAT0_AUDI_MASK  0x00000080 /* When set to 1, mutes ih_fc_stat0[7] */

/* Frame Composer Interrupt Mute Control Register 1 */
#define IH_MUTE_FC_STAT1  0x00000604
#define IH_MUTE_FC_STAT1_GCP_MASK  0x00000001 /* When set to 1, mutes ih_fc_stat1[0] */
#define IH_MUTE_FC_STAT1_AVI_MASK  0x00000002 /* When set to 1, mutes ih_fc_stat1[1] */
#define IH_MUTE_FC_STAT1_AMP_MASK  0x00000004 /* When set to 1, mutes ih_fc_stat1[2] */
#define IH_MUTE_FC_STAT1_SPD_MASK  0x00000008 /* When set to 1, mutes ih_fc_stat1[3] */
#define IH_MUTE_FC_STAT1_VSD_MASK  0x00000010 /* When set to 1, mutes ih_fc_stat1[4] */
#define IH_MUTE_FC_STAT1_ISCR2_MASK  0x00000020 /* When set to 1, mutes ih_fc_stat1[5] */
#define IH_MUTE_FC_STAT1_ISCR1_MASK  0x00000040 /* When set to 1, mutes ih_fc_stat1[6] */
#define IH_MUTE_FC_STAT1_GMD_MASK  0x00000080 /* When set to 1, mutes ih_fc_stat1[7] */

/* Frame Composer Interrupt Mute Control Register 2 */
#define IH_MUTE_FC_STAT2  0x00000608
#define IH_MUTE_FC_STAT2_HIGHPRIORITY_OVERFLOW_MASK  0x00000001 /* When set to 1, mutes ih_fc_stat2[0] */
#define IH_MUTE_FC_STAT2_LOWPRIORITY_OVERFLOW_MASK  0x00000002 /* When set to 1, mutes ih_fc_stat2[1] */

/* Audio Sampler Interrupt Mute Control Register */
#define IH_MUTE_AS_STAT0  0x0000060C
#define IH_MUTE_AS_STAT0_AUD_FIFO_OVERFLOW_MASK  0x00000001 /* When set to 1, mutes ih_as_stat0[0] */
#define IH_MUTE_AS_STAT0_AUD_FIFO_UNDERFLOW_MASK  0x00000002 /* When set to 1, mutes ih_as_stat0[1] */
#define IH_MUTE_AS_STAT0_AUD_FIFO_UNDERFLOW_THR_MASK  0x00000004 /* When set to 1, mutes ih_as_stat0[2] */
#define IH_MUTE_AS_STAT0_FIFO_OVERRUN_MASK  0x00000008 /* When set to 1, mutes ih_as_stat0[3] */

/* PHY Interface Interrupt Mute Control Register */
#define IH_MUTE_PHY_STAT0  0x00000610
#define IH_MUTE_PHY_STAT0_HPD_MASK  0x00000001 /* When set to 1, mutes ih_phy_stat0[0] */
#define IH_MUTE_PHY_STAT0_TX_PHY_LOCK_MASK  0x00000002 /* When set to 1, mutes ih_phy_stat0[1] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_0_MASK  0x00000004 /* When set to 1, mutes ih_phy_stat0[2] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_1_MASK  0x00000008 /* When set to 1, mutes ih_phy_stat0[3] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_2_MASK  0x00000010 /* When set to 1, mutes ih_phy_stat0[4] */
#define IH_MUTE_PHY_STAT0_RX_SENSE_3_MASK  0x00000020 /* When set to 1, mutes ih_phy_stat0[5] */

/* E-DDC I2C Master Interrupt Mute Control Register */
#define IH_MUTE_I2CM_STAT0  0x00000614
#define IH_MUTE_I2CM_STAT0_I2CMASTERERROR_MASK  0x00000001 /* When set to 1, mutes ih_i2cm_stat0[0] */
#define IH_MUTE_I2CM_STAT0_I2CMASTERDONE_MASK  0x00000002 /* When set to 1, mutes ih_i2cm_stat0[1] */
#define IH_MUTE_I2CM_STAT0_SCDC_READREQ_MASK  0x00000004 /* When set to 1, mutes ih_i2cm_stat0[2] */

/* CEC Interrupt Mute Control Register */
#define IH_MUTE_CEC_STAT0  0x00000618
#define IH_MUTE_CEC_STAT0_DONE_MASK  0x00000001 /* When set to 1, mutes ih_cec_stat0[0] */
#define IH_MUTE_CEC_STAT0_EOM_MASK  0x00000002 /* When set to 1, mutes ih_cec_stat0[1] */
#define IH_MUTE_CEC_STAT0_NACK_MASK  0x00000004 /* When set to 1, mutes ih_cec_stat0[2] */
#define IH_MUTE_CEC_STAT0_ARB_LOST_MASK  0x00000008 /* When set to 1, mutes ih_cec_stat0[3] */
#define IH_MUTE_CEC_STAT0_ERROR_INITIATOR_MASK  0x00000010 /* When set to 1, mutes ih_cec_stat0[4] */
#define IH_MUTE_CEC_STAT0_ERROR_FOLLOW_MASK  0x00000020 /* When set to 1, mutes ih_cec_stat0[5] */
#define IH_MUTE_CEC_STAT0_WAKEUP_MASK  0x00000040 /* When set to 1, mutes ih_cec_stat0[6] */

/* Video Packetizer Interrupt Mute Control Register */
#define IH_MUTE_VP_STAT0  0x0000061C
#define IH_MUTE_VP_STAT0_FIFOEMPTYBYP_MASK  0x00000001 /* When set to 1, mutes ih_vp_stat0[0] */
#define IH_MUTE_VP_STAT0_FIFOFULLBYP_MASK  0x00000002 /* When set to 1, mutes ih_vp_stat0[1] */
#define IH_MUTE_VP_STAT0_FIFOEMPTYREMAP_MASK  0x00000004 /* When set to 1, mutes ih_vp_stat0[2] */
#define IH_MUTE_VP_STAT0_FIFOFULLREMAP_MASK  0x00000008 /* When set to 1, mutes ih_vp_stat0[3] */
#define IH_MUTE_VP_STAT0_FIFOEMPTYPP_MASK  0x00000010 /* When set to 1, mutes ih_vp_stat0[4] */
#define IH_MUTE_VP_STAT0_FIFOFULLPP_MASK  0x00000020 /* When set to 1, mutes ih_vp_stat0[5] */
#define IH_MUTE_VP_STAT0_FIFOEMPTYREPET_MASK  0x00000040 /* When set to 1, mutes ih_vp_stat0[6] */
#define IH_MUTE_VP_STAT0_FIFOFULLREPET_MASK  0x00000080 /* When set to 1, mutes ih_vp_stat0[7] */

/* PHY GEN2 I2C Master Interrupt Mute Control Register */
#define IH_MUTE_I2CMPHY_STAT0  0x00000620
#define IH_MUTE_I2CMPHY_STAT0_I2CMPHYERROR_MASK  0x00000001 /* When set to 1, mutes ih_i2cmphy_stat0[0] */
#define IH_MUTE_I2CMPHY_STAT0_I2CMPHYDONE_MASK  0x00000002 /* When set to 1, mutes ih_i2cmphy_stat0[1] */

/* AHB Audio DMA Interrupt Mute Control Register */
#define IH_MUTE_AHBDMAAUD_STAT0  0x00000624
#define IH_MUTE_AHBDMAAUD_STAT0_INTBUFFEMPTY_MASK  0x00000001 /* When set to 1, mutes ih_ahbdmaaud_stat0[0] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTBUFFULL_MASK  0x00000002 /* en set to 1, mutes ih_ahbdmaaud_stat0[1] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTDONE_MASK  0x00000004 /* When set to 1, mutes ih_ahbdmaaud_stat0[2] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTINTERTRYSPLIT_MASK  0x00000008 /* When set to 1, mutes ih_ahbdmaaud_stat0[3] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTLOSTOWNERSHIP_MASK  0x00000010 /* When set to 1, mutes ih_ahbdmaaud_stat0[4] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTERROR_MASK  0x00000020 /* When set to 1, mutes ih_ahbdmaaud_stat0[5] */
#define IH_MUTE_AHBDMAAUD_STAT0_INTBUFFOVERRUN_MASK  0x00000040 /* When set to 1, mutes ih_ahbdmaaud_stat0[6] */

/* Global Interrupt Mute Control Register */
#define IH_MUTE  0x000007FC
#define IH_MUTE_MUTE_ALL_INTERRUPT_MASK  0x00000001 /* When set to 1, mutes the main interrupt line (where all interrupts are ORed) */
#define IH_MUTE_MUTE_WAKEUP_INTERRUPT_MASK  0x00000002 /* When set to 1, mutes the main interrupt output port */

/**
 * @desc: main control set hdcp clock
*/
void dw_mc_hdcp_clock_disable(dw_hdmi_dev_t *dev, u8 bit);

/**
 * @desc: main control set cec clock
*/
void dw_mc_cec_clock_enable(dw_hdmi_dev_t *dev, u8 bit);

/**
 * @desc: main control set audio sample clock
*/
void dw_mc_audio_sampler_clock_enable(dw_hdmi_dev_t *dev, u8 bit);

/**
 * @desc: main control reset i2s
*/
void dw_mc_audio_i2s_reset(dw_hdmi_dev_t *dev, u8 bit);

/**
 * @desc: main control reset tmds clock
*/
void dw_mc_tmds_clock_reset(dw_hdmi_dev_t *dev, u8 bit);

/**
 * @desc: main control set phy
*/
void dw_mc_phy_reset(dw_hdmi_dev_t *dev, u8 bit);

/**
 * @desc: main control all clock enable
*/
void dw_mc_all_clock_enable(dw_hdmi_dev_t *dev);

/**
 * @desc: main control all clock disable
*/
void dw_mc_all_clock_disable(dw_hdmi_dev_t *dev);

/**
 * @desc: main control all clock standby
*/
void dw_mc_all_clock_standby(dw_hdmi_dev_t *dev);

/**
 * @desc: main control get i2c master irq
*/
u32 dw_mc_get_i2cm_irq_mask(dw_hdmi_dev_t *dev);

/**
 * @desc: main control set irq mute by source
*/
int dw_mc_irq_mute_source(dw_hdmi_dev_t *dev, irq_sources_t irq_source);

/**
 * @desc: main control set irq unmute by source
*/
int dw_mc_irq_unmute_source(dw_hdmi_dev_t *dev, irq_sources_t irq_source);

/**
 * @desc: main control mute all irq
*/
void dw_mc_irq_all_mute(dw_hdmi_dev_t *dev);

/**
 * @desc: main control unmute all irq
*/
void dw_mc_irq_all_unmute(dw_hdmi_dev_t *dev);

/**
 * @desc: main control mask all irq
*/
void dw_mc_irq_mask_all(dw_hdmi_dev_t *dev);

#endif	/* _DW_MC_H_ */
