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
#include <linux/delay.h>
#include "dw_avp.h"
#include "dw_fc.h"

#define FC_GMD_PB_SIZE			28
#define FC_ACP_TX		       (0)
#define FC_ISRC1_TX		       (1)
#define FC_ISRC2_TX		       (2)
#define FC_SPD_TX		       (4)
#define FC_VSD_TX		       (3)

channel_count_t channel_cnt[] = {
	{0x00, 1}, {0x01, 2}, {0x02, 2}, {0x04, 2}, {0x03, 3}, {0x05, 3},
	{0x06, 3}, {0x08, 3}, {0x14, 3}, {0x07, 4}, {0x09, 4}, {0x0A, 4},
	{0x0C, 4}, {0x15, 4}, {0x16, 4}, {0x18, 4}, {0x0B, 5}, {0x0D, 5},
	{0x0E, 5}, {0x10, 5}, {0x17, 5}, {0x19, 5}, {0x1A, 5}, {0x1C, 5},
	{0x20, 5}, {0x22, 5}, {0x24, 5}, {0x26, 5}, {0x0F, 6}, {0x11, 6},
	{0x12, 6}, {0x1B, 6}, {0x1D, 6}, {0x1E, 6}, {0x21, 6}, {0x23, 6},
	{0x25, 6}, {0x27, 6}, {0x28, 6}, {0x2A, 6}, {0x2C, 6}, {0x2E, 6},
	{0x30, 6}, {0x13, 7}, {0x1F, 7}, {0x29, 7}, {0x2B, 7}, {0x2D, 7},
	{0x2F, 7}, {0x31, 7}, {0, 0},
};

/* sampling frequency: unit:Hz */
iec_params_t iec_original_sampling_freq_values[] = {
	{{.frequency = 44100}, 0xF},
	{{.frequency = 88200}, 0x7},
	{{.frequency = 22050}, 0xB},
	{{.frequency = 176400}, 0x3},
	{{.frequency = 48000}, 0xD},
	{{.frequency = 96000}, 0x5},
	{{.frequency = 24000}, 0x9},
	{{.frequency = 192000}, 0x1},
	{{.frequency =  8000}, 0x6},
	{{.frequency = 11025}, 0xA},
	{{.frequency = 12000}, 0x2},
	{{.frequency = 32000}, 0xC},
	{{.frequency = 16000}, 0x8},
	{{.frequency = 0},      0x0}
};

iec_params_t iec_sampling_freq_values[] = {
	{{.frequency = 22050}, 0x4},
	{{.frequency = 44100}, 0x0},
	{{.frequency = 88200}, 0x8},
	{{.frequency = 176400}, 0xC},
	{{.frequency = 24000}, 0x6},
	{{.frequency = 48000}, 0x2},
	{{.frequency = 96000}, 0xA},
	{{.frequency = 192000}, 0xE},
	{{.frequency = 32000}, 0x3},
	{{.frequency = 768000}, 0x9},
	{{.frequency = 0},      0x0}
};

iec_params_t iec_word_length[] = {
	{{.sample_size = 16}, 0x2},
	{{.sample_size = 17}, 0xC},
	{{.sample_size = 18}, 0x4},
	{{.sample_size = 19}, 0x8},
	{{.sample_size = 20}, 0x3},
	{{.sample_size = 21}, 0xD},
	{{.sample_size = 22}, 0x5},
	{{.sample_size = 23}, 0x9},
	{{.sample_size = 24}, 0xB},
	{{.sample_size = 0},  0x0}
};

/**
 * @param params pointer to the audio parameters structure
 * @return number of audio channels transmitted -1
 */
static u8 _audio_sw_get_channel_count(struct dw_audio_s *params)
{
	int i = 0;

	for (i = 0; channel_cnt[i].channel_count != 0; i++) {
		if (channel_cnt[i].channel_allocation ==
					params->mChannelAllocation) {
			return channel_cnt[i].channel_count;
		}
	}

	return 0;
}

/**
 * @param params pointer to the audio parameters structure
 */
static u8 _audio_sw_get_iec_original_sampling_freq(struct dw_audio_s *params)
{
	int i = 0;

	for (i = 0; iec_original_sampling_freq_values[i].iec.frequency != 0; i++) {
		if (params->mSamplingFrequency ==
			iec_original_sampling_freq_values[i].iec.frequency) {
			u8 value = iec_original_sampling_freq_values[i].value;
			return value;
		}
	}

	return 0x0;
}

/**
 * @param params pointer to the audio parameters structure
 */
static u8 _audio_sw_get_iec_sampling_freq(struct dw_audio_s *params)
{
	int i = 0;

	for (i = 0; iec_sampling_freq_values[i].iec.frequency != 0; i++) {
		if (params->mSamplingFrequency ==
			iec_sampling_freq_values[i].iec.frequency) {
			u8 value = iec_sampling_freq_values[i].value;
			return value;
		}
	}

	return 0x1;
}

/**
 * @param params pointer to the audio parameters structure
 */
static u8 _audio_sw_get_iec_word_length(struct dw_audio_s *params)
{
	int i = 0;

	for (i = 0; iec_word_length[i].iec.sample_size != 0; i++) {
		if (params->mSampleSize == iec_word_length[i].iec.sample_size)
			return iec_word_length[i].value;
	}

	return 0x0;
}

/**
 * return if channel is enabled or not using the user's channel allocation
 * code
 * @param params pointer to the audio parameters structure
 * @param channel in question -1
 * @return 1 if channel is to be enabled, 0 otherwise
 */
static u8 _audio_sw_check_channel_en(struct dw_audio_s *params, u8 channel)
{
	switch (channel) {
	case 0:
	case 1:
		audio_log("channal %d is enable\n", channel);
		return 1;
	case 2:
		audio_log("channal %d is enable\n", channel);
		return params->mChannelAllocation & BIT(0);
	case 3:
		audio_log("channal %d is enable\n", channel);
		return (params->mChannelAllocation & BIT(1)) >> 1;
	case 4:
		if (((params->mChannelAllocation > 0x03) &&
			(params->mChannelAllocation < 0x14)) ||
			((params->mChannelAllocation > 0x17) &&
			(params->mChannelAllocation < 0x20))) {
			audio_log("channal %d is enable\n", channel);
			return 1;
		} else {
			return 0;
		}
	case 5:
		if (((params->mChannelAllocation > 0x07) &&
			(params->mChannelAllocation < 0x14)) ||
			((params->mChannelAllocation > 0x1C) &&
			(params->mChannelAllocation < 0x20))) {
			audio_log("channal %d is enable\n", channel);
			return 1;
		} else {
			return 0;
		}
	case 6:
		if ((params->mChannelAllocation > 0x0B) &&
			(params->mChannelAllocation < 0x20)) {
			audio_log("channal %d is enable\n", channel);
			return 1;
		} else {
			return 0;
		}
	case 7:
		audio_log("channal %d is enable\n", channel);
		return (params->mChannelAllocation & BIT(4)) >> 4;
	default:
		return 0;
	}
}

static void _dw_fc_audio_set_channel_right(u8 value, u8 channel)
{
	log_trace2(value, channel);
	if (channel == 0)
		dw_write_mask(FC_AUDSCHNL3, FC_AUDSCHNL3_OIEC_CHANNELNUMCR0_MASK, value);
	else if (channel == 1)
		dw_write_mask(FC_AUDSCHNL3, FC_AUDSCHNL3_OIEC_CHANNELNUMCR1_MASK, value);
	else if (channel == 2)
		dw_write_mask(FC_AUDSCHNL4, FC_AUDSCHNL4_OIEC_CHANNELNUMCR2_MASK, value);
	else if (channel == 3)
		dw_write_mask(FC_AUDSCHNL4, FC_AUDSCHNL4_OIEC_CHANNELNUMCR3_MASK, value);
	else
		hdmi_err("set channel %d right is invalid!!!", channel);
}

static void _dw_fc_audio_set_channel_left(u8 value, unsigned channel)
{
	log_trace2(value, channel);
	if (channel == 0)
		dw_write_mask(FC_AUDSCHNL5, FC_AUDSCHNL5_OIEC_CHANNELNUMCL0_MASK, value);
	else if (channel == 1)
		dw_write_mask(FC_AUDSCHNL5, FC_AUDSCHNL5_OIEC_CHANNELNUMCL1_MASK, value);
	else if (channel == 2)
		dw_write_mask(FC_AUDSCHNL6, FC_AUDSCHNL6_OIEC_CHANNELNUMCL2_MASK, value);
	else if (channel == 3)
		dw_write_mask(FC_AUDSCHNL6, FC_AUDSCHNL6_OIEC_CHANNELNUMCL3_MASK, value);
	else
		hdmi_err("set channel %d left is invalid!!!", channel);
}

void dw_fc_audio_force(u8 bit)
{
	log_trace1(bit);
	dw_write_mask(FC_DBGFORCE, FC_DBGFORCE_FORCEAUDIO_MASK, bit);
}

void dw_fc_audio_set_mute(u8 state)
{
	dw_write_mask(FC_AUDSCONF, FC_AUDSCONF_AUD_PACKET_SAMPFLT_MASK,
			state ? 0xF : 0x0);
}

u8 dw_fc_audio_get_mute(void)
{
	return dw_read_mask(FC_AUDSCONF, FC_AUDSCONF_AUD_PACKET_SAMPFLT_MASK);
}

u8 dw_fc_audio_get_packet_layout(void)
{
	log_trace();
	return dw_read_mask(FC_AUDSCONF, FC_AUDSCONF_AUD_PACKET_LAYOUT_MASK);
}

u8 dw_fc_audio_get_channel_count(void)
{
	log_trace();
	return dw_read_mask(FC_AUDICONF0, FC_AUDICONF0_CC_MASK);
}

void dw_fc_audio_packet_config(struct dw_audio_s *audio)
{
	u8 channel_count = _audio_sw_get_channel_count(audio);
	u8 fs_value = 0;

	log_trace();

	dw_write_mask(FC_AUDICONF0, FC_AUDICONF0_CC_MASK, channel_count);

	dw_write(FC_AUDICONF2, audio->mChannelAllocation);

	dw_write_mask(FC_AUDICONF3, FC_AUDICONF3_LSV_MASK,
			audio->mLevelShiftValue);

	dw_write_mask(FC_AUDICONF3, FC_AUDICONF3_DM_INH_MASK,
			(audio->mDownMixInhibitFlag ? 1 : 0));

	audio_log("Audio packet:\n");
	audio_log(" - channel count = %d\n", channel_count);
	audio_log(" - channel allocation = %d\n", audio->mChannelAllocation);
	audio_log(" - level shift = %d\n", audio->mLevelShiftValue);

	if ((audio->mCodingType == DW_AUD_CODING_ONE_BIT_AUDIO) ||
			(audio->mCodingType == DW_AUD_CODING_DST)) {
		u32 freq = audio->mSamplingFrequency;

		/* Audio InfoFrame sample frequency when OBA or DST */
		if (freq == 32000)
			fs_value = 1;
		else if (freq == 44100)
			fs_value = 2;
		else if (freq == 48000)
			fs_value = 3;
		else if (freq == 88200)
			fs_value = 4;
		else if (freq == 96000)
			fs_value = 5;
		else if (freq == 176400)
			fs_value = 6;
		else if (freq == 192000)
			fs_value = 7;
		else
			fs_value = 0;
	} else {
		/* otherwise refer to stream header (0) */
		fs_value = 0;
	}
	audio_log(" - freq number = %d\n", fs_value);
	dw_write_mask(FC_AUDICONF1, FC_AUDICONF1_SF_MASK, fs_value);

	/* for HDMI refer to stream header  (0) */
	dw_write_mask(FC_AUDICONF0, FC_AUDICONF0_CT_MASK, 0x0);

	/* for HDMI refer to stream header  (0) */
	dw_write_mask(FC_AUDICONF1, FC_AUDICONF1_SS_MASK, 0x0);
}

void dw_fc_audio_sample_config(struct dw_audio_s *audio)
{
	int i = 0;
	u8 channel_count = _audio_sw_get_channel_count(audio);
	u8 data = 0;

	/* More than 2 channels => layout 1 else layout 0 */
	if ((channel_count + 1) > 2)
		dw_write_mask(FC_AUDSCONF, FC_AUDSCONF_AUD_PACKET_LAYOUT_MASK, 0x1);
	else
		dw_write_mask(FC_AUDSCONF, FC_AUDSCONF_AUD_PACKET_LAYOUT_MASK, 0x0);

	/* iec validity and user bits (IEC 60958-1) */
	for (i = 0; i < 4; i++) {
		/* _audio_sw_check_channel_en considers left as 1 channel and
		 * right as another (+1), hence the x2 factor in the following */
		/* validity bit is 0 when reliable, which is !IsChannelEn */
		u8 channel_enable = _audio_sw_check_channel_en(audio, (2 * i));
		if (i < 4)
			dw_write_mask(FC_AUDSV, (1 << (4 + i)), (!channel_enable));
		else
			hdmi_err("invalid channel number\n");

		channel_enable = _audio_sw_check_channel_en(audio, (2 * i) + 1);
		if (i < 4)
			dw_write_mask(FC_AUDSV, (1 << i), !channel_enable);
		else
			hdmi_err("invalid channel number: %d", i);

		if (i < 4)
			dw_write_mask(FC_AUDSU, (1 << (4 + i)), 0x1);
		else
			hdmi_err("invalid channel number: %d\n", i);

		if (i < 4)
			dw_write_mask(FC_AUDSU, (1 << i), 0x1);
		else
			hdmi_err("invalid channel number: %d\n", i);

	}

	if (audio->mCodingType != DW_AUD_CODING_PCM)
		return;

	/* IEC - not needed if non-linear PCM */
	dw_write_mask(FC_AUDSCHNL0,
			FC_AUDSCHNL0_OIEC_CGMSA_MASK, audio->mIecCgmsA);

	dw_write_mask(FC_AUDSCHNL0,
			FC_AUDSCHNL0_OIEC_COPYRIGHT_MASK, audio->mIecCopyright ? 0 : 1);

	dw_write(FC_AUDSCHNL1, audio->mIecCategoryCode);

	dw_write_mask(FC_AUDSCHNL2,
				FC_AUDSCHNL2_OIEC_PCMAUDIOMODE_MASK, audio->mIecPcmMode);

	dw_write_mask(FC_AUDSCHNL2,
			FC_AUDSCHNL2_OIEC_SOURCENUMBER_MASK, audio->mIecSourceNumber);

	for (i = 0; i < 4; i++) {	/* 0, 1, 2, 3 */
		_dw_fc_audio_set_channel_left(2 * i + 1, i);	/* 1, 3, 5, 7 */
		_dw_fc_audio_set_channel_right(2 * (i + 1), i);	/* 2, 4, 6, 8 */
	}

	dw_write_mask(FC_AUDSCHNL7,
			FC_AUDSCHNL7_OIEC_CLKACCURACY_MASK, audio->mIecClockAccuracy);

	data = _audio_sw_get_iec_sampling_freq(audio);
	dw_write_mask(FC_AUDSCHNL7, FC_AUDSCHNL7_OIEC_SAMPFREQ_MASK, data);

	data = _audio_sw_get_iec_original_sampling_freq(audio);
	dw_write_mask(FC_AUDSCHNL8, FC_AUDSCHNL8_OIEC_ORIGSAMPFREQ_MASK, data);

	data = _audio_sw_get_iec_word_length(audio);
	dw_write_mask(FC_AUDSCHNL8, FC_AUDSCHNL8_OIEC_WORDLENGTH_MASK, data);
}

u32 dw_fc_audio_get_sample_freq(void)
{
	int i = 0;
	u32 hw_freq = dw_read_mask(FC_AUDSCHNL7, FC_AUDSCHNL7_OIEC_SAMPFREQ_MASK);

	for (i = 0; iec_sampling_freq_values[i].iec.frequency != 0; i++) {
		if (hw_freq == iec_sampling_freq_values[i].value) {
			return iec_sampling_freq_values[i].iec.frequency;
		}
	}

	/* Not indicated */
	return 0;
}

u8 dw_fc_audio_get_word_length(void)
{
	int i = 0;
	u32 hw_data = dw_read_mask(FC_AUDSCHNL8, FC_AUDSCHNL8_OIEC_WORDLENGTH_MASK);

	for (i = 0; iec_word_length[i].iec.sample_size != 0; i++) {
		if (hw_data == iec_word_length[i].value)
			return iec_word_length[i].iec.sample_size;
	}

	return 0x0;
}

u32 dw_fc_video_get_hactive(void)
{
	u32 value = 0;
	value = dw_read_mask(FC_INHACTIV1, FC_INHACTIV1_H_IN_ACTIV_MASK |
		FC_INHACTIV1_H_IN_ACTIV_12_MASK) << 8;
	value |= dw_read(FC_INHACTIV0);
	return value;
}

void _dw_fc_video_set_hactive(u16 value)
{
	log_trace1(value);
	/* 12-bit width */
	dw_write((FC_INHACTIV0), (u8) (value));
	dw_write_mask(FC_INHACTIV1, FC_INHACTIV1_H_IN_ACTIV_MASK |
			FC_INHACTIV1_H_IN_ACTIV_12_MASK, (u8)(value >> 8));
}

void _dw_fc_video_set_hblank(u16 value)
{
	log_trace1(value);
	/* 10-bit width */
	dw_write((FC_INHBLANK0), (u8) (value));
	dw_write_mask(FC_INHBLANK1, FC_INHBLANK1_H_IN_BLANK_MASK |
			FC_INHBLANK1_H_IN_BLANK_12_MASK, (u8)(value >> 8));
}

u32 dw_fc_video_get_vactive(void)
{
	u32 value = 0;
	value = dw_read_mask(FC_INVACTIV1, FC_INVACTIV1_V_IN_ACTIV_MASK |
		FC_INVACTIV1_V_IN_ACTIV_12_11_MASK) << 8;
	value |= dw_read(FC_INVACTIV0);
	return value;
}

/* Setting Frame Composer Input Video HSync Front Porch */
void _dw_fc_video_set_hsync_edge_delay(u16 value)
{
	log_trace1(value);
	/* 11-bit width */
	dw_write((FC_HSYNCINDELAY0), (u8) (value));
	dw_write_mask(FC_HSYNCINDELAY1, FC_HSYNCINDELAY1_H_IN_DELAY_MASK |
			FC_HSYNCINDELAY1_H_IN_DELAY_12_MASK, (u8)(value >> 8));
}

void _dw_fc_video_set_hsync_pluse_width(u16 value)
{
	log_trace1(value);
	/* 9-bit width */
	dw_write((FC_HSYNCINWIDTH0), (u8) (value));
	dw_write_mask(FC_HSYNCINWIDTH1, FC_HSYNCINWIDTH1_H_IN_WIDTH_MASK,
			(u8)(value >> 8));
}

void _dw_fc_video_set_preamble_filter(u8 value, u8 channel)
{
	log_trace1(value);
	if (channel == 0)
		dw_write((FC_CH0PREAM), value);
	else if (channel == 1)
		dw_write_mask(FC_CH1PREAM,
				FC_CH1PREAM_CH1_PREAMBLE_FILTER_MASK, value);
	else if (channel == 2)
		dw_write_mask(FC_CH2PREAM,
				FC_CH2PREAM_CH2_PREAMBLE_FILTER_MASK, value);
	else
		hdmi_err("invalid channel number: %d\n", channel);
}

void _dw_fc_video_force(u8 bit)
{
	log_trace1(bit);

	/* avoid glitches */
	if (bit != 0) {
		dw_write(FC_DBGTMDS2, bit ? 0x00 : 0x00);	/* R */
		dw_write(FC_DBGTMDS1, bit ? 0x00 : 0x00);	/* G */
		dw_write(FC_DBGTMDS0, bit ? 0xFF : 0x00);	/* B */
		dw_write_mask(FC_DBGFORCE,
				FC_DBGFORCE_FORCEVIDEO_MASK, 1);
	} else {
		dw_write_mask(FC_DBGFORCE,
				FC_DBGFORCE_FORCEVIDEO_MASK, 0);
		dw_write(FC_DBGTMDS2, bit ? 0x00 : 0x00);	/* R */
		dw_write(FC_DBGTMDS1, bit ? 0x00 : 0x00);	/* G */
		dw_write(FC_DBGTMDS0, bit ? 0xFF : 0x00);	/* B */
	}
}

void dw_fc_video_force_value(u8 bit, u32 value)
{
	if (bit) {
		/* enable force frame composer video output */
		dw_write(FC_DBGTMDS2, (u8)((value >> 16) & 0xFF)); /* R */
		dw_write(FC_DBGTMDS1, (u8)((value >> 8)  & 0xFF)); /* G */
		dw_write(FC_DBGTMDS0, (u8)((value >> 0)  & 0xFF)); /* B */
	}
	dw_write_mask(FC_DBGFORCE, FC_DBGFORCE_FORCEVIDEO_MASK, bit ? 0x1 : 0x0);
}

void dw_fc_video_set_hdcp_keepout(u8 bit)
{
	log_trace1(bit);
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_HDCP_KEEPOUT_MASK, bit);
}

void dw_fc_video_set_tmds_mode(u8 bit)
{
	log_trace1(bit);
	/* 1: HDMI; 0: DVI */
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_DVI_MODEZ_MASK, bit);
}

u8 dw_fc_video_get_tmds_mode(void)
{
	log_trace();
	/* 1: HDMI; 0: DVI */
	return dw_read_mask(FC_INVIDCONF, FC_INVIDCONF_DVI_MODEZ_MASK);
}

void dw_fc_video_set_scramble(u8 state)
{
	dw_write_mask(FC_SCRAMBLER_CTRL,
			FC_SCRAMBLER_CTRL_SCRAMBLER_ON_MASK, state);
}

u8 dw_fc_video_get_scramble(void)
{
	return dw_read_mask(FC_SCRAMBLER_CTRL,
			FC_SCRAMBLER_CTRL_SCRAMBLER_ON_MASK);
}

u8 dw_fc_video_get_hsync_polarity(void)
{
	return dw_read_mask(FC_INVIDCONF, FC_INVIDCONF_HSYNC_IN_POLARITY_MASK);
}

u8 dw_fc_video_get_vsync_polarity(void)
{
	return dw_read_mask(FC_INVIDCONF, FC_INVIDCONF_VSYNC_IN_POLARITY_MASK);
}

int dw_fc_video_config(struct dw_video_s *video)
{
	const dw_dtd_t *dtd = &video->mDtd;
	u8  interlaced = dtd->mInterlaced;
	u16 vactive = dtd->mVActive;
	u16 hactive = dtd->mHActive;
	u16 hblank  = dtd->mHBlanking;
	u16 hsync = dtd->mHSyncPulseWidth;
	u16 hsync_delay = dtd->mHSyncOffset;
	u16 i = 0;

	log_trace();

	dtd = &video->mDtd;
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_VSYNC_IN_POLARITY_MASK,
			dtd->mVSyncPolarity);
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_HSYNC_IN_POLARITY_MASK,
			dtd->mHSyncPolarity);
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_DE_IN_POLARITY_MASK, 0x1);

	dw_fc_video_set_tmds_mode(video->mHdmi);

	if (video->mHdmiVideoFormat == 2 && video->m3dStructure == 0) {
		if (interlaced)
			vactive = (dtd->mVActive << 2) + 3 * dtd->mVBlanking + 2;
		else
			vactive = (dtd->mVActive << 1) + dtd->mVBlanking;
		interlaced = 0;
	}
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_R_V_BLANK_IN_OSC_MASK, interlaced);
	dw_write_mask(FC_INVIDCONF, FC_INVIDCONF_IN_I_P_MASK, interlaced);
	dw_write_mask(FC_INVACTIV0, FC_INVACTIV0_V_IN_ACTIV_MASK,
			(u8)(vactive >> 0));
	dw_write_mask(FC_INVACTIV1, FC_INVACTIV1_V_IN_ACTIV_HIGHT_MASK,
			(u8)(vactive >> 8));

	if (video->mEncodingOut == DW_COLOR_FORMAT_YCC420) {
		hactive = dtd->mHActive / 2;
		hblank  = dtd->mHBlanking / 2;
		hsync   = dtd->mHSyncPulseWidth / 2;
		hsync_delay = dtd->mHSyncOffset / 2;
	}

	_dw_fc_video_set_hactive(hactive);
	_dw_fc_video_set_hblank(hblank);
	_dw_fc_video_set_hsync_pluse_width(hsync);
	_dw_fc_video_set_hsync_edge_delay(hsync_delay);

	dw_write_mask(FC_INVBLANK, FC_INVBLANK_V_IN_BLANK_MASK,
			(u8)dtd->mVBlanking);
	dw_write_mask(FC_VSYNCINDELAY, FC_VSYNCINDELAY_V_IN_DELAY_MASK,
			(u8)dtd->mVSyncOffset);
	dw_write_mask(FC_VSYNCINWIDTH, FC_VSYNCINWIDTH_V_IN_WIDTH_MASK,
			(u8)dtd->mVSyncPulseWidth);

	dw_write(FC_CTRLDUR, 12);
	dw_write(FC_EXCTRLDUR, 32);

	/* spacing < 256^2 * config / tmdsClock, spacing <= 50ms
	 * worst case: tmdsClock == 25MHz => config <= 19
	 */
	dw_write(FC_EXCTRLSPAC, 0x1);

	for (i = 0; i < 3; i++)
		_dw_fc_video_set_preamble_filter((i + 1) * 11, i);

	dw_write_mask(FC_PRCONF, FC_PRCONF_INCOMING_PR_FACTOR_MASK,
			(dtd->mPixelRepetitionInput + 1));

	return true;
}

void _dw_gamut_set_content(const u8 *content, u8 length)
{
	u8 i = 0;

	log_trace1(content[0]);
	if (length > (FC_GMD_PB_SIZE)) {
		length = (FC_GMD_PB_SIZE);
		hdmi_wrn("gamut content truncated");
	}

	for (i = 0; i < length; i++)
		dw_write(FC_GMD_PB0 + (i*4), content[i]);
}

void _dw_gamut_set_enable_tx(u8 enable)
{
	log_trace1(enable);
	if (enable)
		enable = 1; /* ensure value is 1 */
	dw_write_mask(FC_GMD_EN, FC_GMD_EN_GMDENABLETX_MASK, enable);
}

void _dw_gamut_config(void)
{
	/* P0 */
	dw_write_mask(FC_GMD_HB, FC_GMD_HB_GMDGBD_PROFILE_MASK, 0x0);

	/* P0 */
	dw_write_mask(FC_GMD_CONF, FC_GMD_CONF_GMDPACKETSINFRAME_MASK, 0x1);

	dw_write_mask(FC_GMD_CONF, FC_GMD_CONF_GMDPACKETLINESPACING_MASK, 0x1);
}

void _dw_gamut_packet_config(const u8 *gbdContent, u8 length)
{
	u8 temp = 0x0;
	_dw_gamut_set_enable_tx(1);
	/* sequential */
	temp = (u8)(dw_read(FC_GMD_STAT) & 0xF);
	dw_write_mask(FC_GMD_HB,
			FC_GMD_HB_GMDAFFECTED_GAMUT_SEQ_NUM_MASK, (temp + 1) % 16);

	_dw_gamut_set_content(gbdContent, length);

	/* set next_field to 1 */
	dw_write_mask(FC_GMD_UP, FC_GMD_UP_GMDUPDATEPACKET_MASK, 0x1);
}

void _dw_avi_set_color_metry(u8 value)
{
	log_trace1(value);
	dw_write_mask(FC_AVICONF1, FC_AVICONF1_COLORIMETRY_MASK, value);
}

void _dw_avi_set_active_aspect_ratio_valid(u8 valid)
{
	log_trace1(valid);
	dw_write_mask(FC_AVICONF0,
			FC_AVICONF0_ACTIVE_FORMAT_PRESENT_MASK, valid);
}

void _dw_avi_set_active_format_aspect_ratio(u8 value)
{
	log_trace1(value);
	dw_write_mask(FC_AVICONF1,
			FC_AVICONF1_ACTIVE_ASPECT_RATIO_MASK, value);
}

void _dw_avi_set_extend_color_metry(u8 extColor)
{
	log_trace1(extColor);
	dw_write_mask(FC_AVICONF2,
			FC_AVICONF2_EXTENDED_COLORIMETRY_MASK, extColor);
}

void _dw_avi_config(struct dw_video_s *videoParams)
{
	u8 temp = 0;
	u16 endTop = 0;
	u16 startBottom = 0;
	u16 endLeft = 0;
	u16 startRight = 0;
	dw_dtd_t *dtd = &videoParams->mDtd;

	log_trace();

	dw_write_mask(FC_AVICONF0,
			FC_AVICONF0_RGC_YCC_INDICATION_MASK, videoParams->mEncodingOut);

	_dw_avi_set_active_format_aspect_ratio(0x8);

	dw_avi_set_scan_info(videoParams->mScanInfo);

	temp = 0;
	if ((dtd->mHImageSize != 0 || dtd->mVImageSize != 0)
			&& (dtd->mHImageSize > dtd->mVImageSize)) {
		/* 16:9 or 4:3 */
		u8 pic = (dtd->mHImageSize * 10) % dtd->mVImageSize;
		temp = (pic > 5) ? 2 : 1;
	}
	dw_write_mask(FC_AVICONF1,
			FC_AVICONF1_PICTURE_ASPECT_RATIO_MASK, temp);

	dw_write_mask(FC_AVICONF2,
			FC_AVICONF2_IT_CONTENT_MASK, (videoParams->mItContent ? 1 : 0));

	dw_avi_set_quantization_range(videoParams->mRgbQuantizationRange);

	dw_write_mask(FC_AVICONF2,
			FC_AVICONF2_NON_UNIFORM_PICTURE_SCALING_MASK, videoParams->mNonUniformScaling);

	dw_write(FC_AVIVID, videoParams->mCea_code);

	if (videoParams->mColorimetry == DW_METRY_EXTENDED) {
		/* ext colorimetry valid */
		if (videoParams->mExtColorimetry != (u8) (-1)) {
			_dw_avi_set_extend_color_metry(videoParams->mExtColorimetry);
			_dw_avi_set_color_metry(videoParams->mColorimetry);/* EXT-3 */
		} else {
			_dw_avi_set_extend_color_metry(0);
			_dw_avi_set_color_metry(0);	/* No Data */
		}
	} else {
		_dw_avi_set_extend_color_metry(0);
		/* NODATA-0/ 601-1/ 709-2/ EXT-3 */
		_dw_avi_set_color_metry(videoParams->mColorimetry);
	}
	if (videoParams->mActiveFormatAspectRatio != 0) {
		_dw_avi_set_active_format_aspect_ratio(videoParams->mActiveFormatAspectRatio);
		_dw_avi_set_active_aspect_ratio_valid(1);
	} else {
		_dw_avi_set_active_format_aspect_ratio(0);
		_dw_avi_set_active_aspect_ratio_valid(0);
	}

	temp = 0x0;
	if (videoParams->mEndTopBar != (u16) (-1) ||
			videoParams->mStartBottomBar != (u16) (-1)) {

		if (videoParams->mEndTopBar != (u16) (-1))
			endTop = videoParams->mEndTopBar;
		if (videoParams->mStartBottomBar != (u16) (-1))
			startBottom = videoParams->mStartBottomBar;

		dw_write(FC_AVIETB0, (u8) (endTop));
		dw_write(FC_AVIETB1, (u8) (endTop >> 8));

		dw_write(FC_AVISBB0, (u8) (startBottom));
		dw_write(FC_AVISBB1, (u8) (startBottom >> 8));

		temp = 0x1;
	}
	dw_write_mask(FC_AVICONF0,
		FC_AVICONF0_BAR_INFORMATION_MASK & 0x8, temp);

	temp = 0x0;
	if (videoParams->mEndLeftBar != (u16) (-1) ||
			videoParams->mStartRightBar != (u16) (-1)) {
		if (videoParams->mEndLeftBar != (u16) (-1))
			endLeft = videoParams->mEndLeftBar;

		if (videoParams->mStartRightBar != (u16) (-1))
			startRight = videoParams->mStartRightBar;

		dw_write(FC_AVIELB0, (u8) (endLeft));
		dw_write(FC_AVIELB1, (u8) (endLeft >> 8));
		dw_write(FC_AVISRB0, (u8) (startRight));
		dw_write(FC_AVISRB1, (u8) (startRight >> 8));
		temp = 0x1;
	}
	dw_write_mask(FC_AVICONF0,
		FC_AVICONF0_BAR_INFORMATION_MASK & 0x4, temp);

	temp = (dtd->mPixelRepetitionInput + 1) *
				(videoParams->mPixelRepetitionFactor + 1) - 1;
	dw_write_mask(FC_PRCONF, FC_PRCONF_OUTPUT_PR_FACTOR_MASK, temp);
}

u8 dw_avi_get_colori_metry(void)
{
	u8 colorimetry = 0;
	log_trace();
	colorimetry = (u8)dw_read_mask(FC_AVICONF1, FC_AVICONF1_COLORIMETRY_MASK);
	if (colorimetry == 3)
		return (colorimetry + dw_read_mask(FC_AVICONF2,
					FC_AVICONF2_EXTENDED_COLORIMETRY_MASK));
	return colorimetry;
}

void dw_avi_set_colori_metry(u8 metry, u8 ex_metry)
{
	if (ex_metry || (metry == DW_METRY_EXTENDED)) {
		_dw_avi_set_extend_color_metry(ex_metry);
		_dw_avi_set_color_metry(DW_METRY_EXTENDED);
	} else {
		_dw_avi_set_extend_color_metry(0);
		_dw_avi_set_color_metry(metry);
	}
	_dw_gamut_set_enable_tx(0);
}

void dw_avi_set_scan_info(u8 value)
{
	log_trace1(value);
	dw_write_mask(FC_AVICONF0, FC_AVICONF0_SCAN_INFORMATION_MASK, value);
}

u8 dw_avi_get_rgb_ycc(void)
{
	log_trace();
	return dw_read_mask(FC_AVICONF0, FC_AVICONF0_RGC_YCC_INDICATION_MASK);
}

u8 dw_avi_get_video_code(void)
{
	log_trace();
	return dw_read_mask(FC_AVIVID, FC_AVIVID_FC_AVIVID_MASK);
}

void dw_avi_set_video_code(u8 data)
{
	log_trace();
	return dw_write_mask(FC_AVIVID, FC_AVIVID_FC_AVIVID_MASK, data);
}

void dw_avi_set_aspect_ratio(u8 value)
{
	if (value) {
		_dw_avi_set_active_format_aspect_ratio(value);
		_dw_avi_set_active_aspect_ratio_valid(1);
	} else {
		_dw_avi_set_active_format_aspect_ratio(0);
		_dw_avi_set_active_aspect_ratio_valid(0);
	}
}

void dw_avi_set_quantization_range(u8 range)
{
	log_trace1(range);
	dw_write_mask(FC_AVICONF2, FC_AVICONF2_QUANTIZATION_RANGE_MASK, range);
}

void _dw_spd_set_vendor_name(const u8 *data, u8 length)
{
	u8 i = 0;

	log_trace();
	for (i = 0; i < length; i++)
		dw_write(FC_SPDVENDORNAME0 + (i*4), data[i]);
}

void _dw_spd_set_product_name(const u8 *data, u8 length)
{
	u8 i = 0;
	log_trace();
	for (i = 0; i < length; i++)
		dw_write(FC_SPDPRODUCTNAME0 + (i*4), data[i]);
}

/*
 * Configure the Vendor Payload to be carried by the InfoFrame
 * @param info array
 * @param length of the array
 * @return 0 when successful and 1 on error
 */
u8 _dw_vsi_set_vendor_payload(const u8 *data, unsigned short length)
{
	const unsigned short size = 24;
	unsigned i = 0;

	log_trace();
	if (data == 0) {
		hdmi_err("invalid parameter\n");
		return -1;
	}
	if (length > size) {
		length = size;
		hdmi_err("vendor payload truncated\n");
	}
	for (i = 0; i < length; i++)
		dw_write((FC_VSDPAYLOAD0 + (i*4)), data[i]);

	return 0;
}

void _dw_vsi_enable(u8 enable)
{
	dw_write_mask(FC_PACKET_TX_EN, FC_PACKET_TX_EN_AUT_TX_EN_MASK, enable);
}

void _dw_packets_auto_send(u8 enable, u8 mask)
{
	log_trace2(enable, mask);
	dw_write_mask(FC_DATAUTO0, (1 << mask), (enable ? 1 : 0));
}

void _dw_packets_manual_send(u8 mask)
{
	log_trace1(mask);
	dw_write_mask(FC_DATMAN, (1 << mask), 1);
}

void _dw_packets_disable_all(void)
{
	uint32_t value = (uint32_t)(~(BIT(FC_ACP_TX) | BIT(FC_ISRC1_TX) |
			BIT(FC_ISRC2_TX) | BIT(FC_SPD_TX) | BIT(FC_VSD_TX)));

	log_trace();
	dw_write(FC_DATAUTO0, value & dw_read(FC_DATAUTO0));
}

void _dw_packets_metadata_config(void)
{
	dw_write_mask(FC_DATAUTO1,
			FC_DATAUTO1_AUTO_FRAME_INTERPOLATION_MASK, 0x1);
	dw_write_mask(FC_DATAUTO2,
			FC_DATAUTO2_AUTO_FRAME_PACKETS_MASK, 0x1);
	dw_write_mask(FC_DATAUTO2,
			FC_DATAUTO2_AUTO_LINE_SPACING_MASK, 0x1);
}

/**
 * Configure Colorimetry packets
 * @param dev Device structure
 * @param video Video information structure
 */
void _dw_gamut_colorimetry_config(struct dw_video_s *video)
{
	u8 gamut_metadata[28] = {0};
	int gdb_color_space = 0;

	_dw_gamut_set_enable_tx(0);

	if (video->mColorimetry == DW_METRY_EXTENDED) {
		if (video->mExtColorimetry == DW_METRY_EXT_XV_YCC601) {
			gdb_color_space = 1;
		} else if (video->mExtColorimetry == DW_METRY_EXT_XV_YCC709) {
			gdb_color_space = 2;
			video_log("xv ycc709\n");
		} else if (video->mExtColorimetry == DW_METRY_EXT_S_YCC601) {
			gdb_color_space = 3;
		} else if (video->mExtColorimetry == DW_METRY_EXT_ADOBE_YCC601) {
			gdb_color_space = 3;
		} else if (video->mExtColorimetry == DW_METRY_EXT_ADOBE_RGB) {
			gdb_color_space = 3;
		}

		if (video->mColorimetryDataBlock == 0x1) {
			gamut_metadata[0] = (1 << 7) | gdb_color_space;
			_dw_gamut_packet_config(gamut_metadata,
					(sizeof(gamut_metadata) / sizeof(u8)));
		}
	}
}

/**
 * Configure Vendor Specific InfoFrames.
 * @param dev Device structure
 * @param oui Vendor Organisational Unique Identifier 24 bit IEEE
 * Registration Identifier
 * @param payload Vendor Specific Info Payload
 * @param length of the payload array
 * @param autoSend Start send Vendor Specific InfoFrame automatically
 */
int _dw_packet_vsi_config(u32 oui, const u8 *payload, u8 length, u8 autoSend)
{
	log_trace();
	_dw_packets_auto_send(0, FC_VSD_TX);/* prevent sending half the info. */

	dw_write((FC_VSDIEEEID0), oui);
	dw_write((FC_VSDIEEEID1), oui >> 8);
	dw_write((FC_VSDIEEEID2), oui >> 16);

	if (_dw_vsi_set_vendor_payload(payload, length))
		return false;	/* DEFINE ERROR */

	if (autoSend)
		_dw_packets_auto_send(autoSend, FC_VSD_TX);
	else
		_dw_packets_manual_send(FC_VSD_TX);

	return true;
}

int _dw_packet_spd_config(fc_spd_info_t *spd_data)
{
	const unsigned short pSize = 8;
	const unsigned short vSize = 16;

	log_trace();

	if (spd_data == NULL) {
		hdmi_err("Improper argument: spd_data\n");
		return false;
	}

	_dw_packets_auto_send(0, FC_SPD_TX);/* prevent sending half the info. */

	if (spd_data->vName == 0) {
		hdmi_err("invalid parameter\n");
		return false;
	}
	if (spd_data->vLength > vSize) {
		spd_data->vLength = vSize;
		hdmi_err("vendor name truncated\n");
	}
	if (spd_data->pName == 0) {
		hdmi_err("invalid parameter\n");
		return false;
	}
	if (spd_data->pLength > pSize) {
		spd_data->pLength = pSize;
		video_log("product name truncated\n");
	}

	_dw_spd_set_vendor_name(spd_data->vName, spd_data->vLength);
	_dw_spd_set_product_name(spd_data->pName, spd_data->pLength);
	dw_write(FC_SPDDEVICEINF, spd_data->code);

	if (spd_data->autoSend)
		_dw_packets_auto_send(spd_data->autoSend, FC_SPD_TX);
	else
		_dw_packets_manual_send(FC_SPD_TX);

	return true;
}

void dw_fc_force_output(int enable)
{
	log_trace1(enable);
	dw_fc_audio_force(0);
	_dw_fc_video_force((u8)enable);
}

void dw_gcp_set_avmute(u8 enable)
{
	log_trace1(enable);
	dw_write_mask(FC_GCP, FC_GCP_SET_AVMUTE_MASK, (enable ? 1 : 0));
	dw_write_mask(FC_GCP, FC_GCP_CLEAR_AVMUTE_MASK, (enable ? 0 : 1));
}

u8 dw_gcp_get_avmute(void)
{
	return dw_read_mask(FC_GCP, FC_GCP_SET_AVMUTE_MASK);
}

void dw_drm_packet_clear(dw_fc_drm_pb_t *pb)
{
	if (pb) {
		pb->r_x = 0;
		pb->r_y = 0;
		pb->g_x = 0;
		pb->g_y = 0;
		pb->b_x = 0;
		pb->b_y = 0;
		pb->w_x = 0;
		pb->w_y = 0;
		pb->luma_max = 0;
		pb->luma_min = 0;
		pb->mcll = 0;
		pb->mfll = 0;
	}
}

int dw_drm_packet_filling_data(dw_fc_drm_pb_t *data)
{
	data->r_x      = 0x33c2;
	data->r_y      = 0x86c4;
	data->g_x      = 0x1d4c;
	data->g_y      = 0x0bb8;
	data->b_x      = 0x84d0;
	data->b_y      = 0x3e80;
	data->w_x      = 0x3d13;
	data->w_y      = 0x4042;
	data->luma_max = 0x03e8;
	data->luma_min = 0x1;
	data->mcll     = 0x03e8;
	data->mfll     = 0x0190;

	return 0;
}

void dw_drm_packet_up(dw_fc_drm_pb_t *pb)
{
	int timeout = 10;
	u32 status = 0;

	log_trace();
	/* Configure Dynamic Range and Mastering infoFrame */
	if (pb != 0) {
		dw_write(FC_DRM_PB0, pb->eotf & 0x07);
		dw_write(FC_DRM_PB1, pb->metadata & 0x07);
		dw_write(FC_DRM_PB2, (pb->r_x >> 0) & 0xff);
		dw_write(FC_DRM_PB3, (pb->r_x >> 8) & 0xff);
		dw_write(FC_DRM_PB4, (pb->r_y >> 0) & 0xff);
		dw_write(FC_DRM_PB5, (pb->r_y >> 8) & 0xff);
		dw_write(FC_DRM_PB6, (pb->g_x >> 0) & 0xff);
		dw_write(FC_DRM_PB7, (pb->g_x >> 8) & 0xff);
		dw_write(FC_DRM_PB8, (pb->g_y >> 0) & 0xff);
		dw_write(FC_DRM_PB9, (pb->g_y >> 8) & 0xff);
		dw_write(FC_DRM_PB10, (pb->b_x >> 0) & 0xff);
		dw_write(FC_DRM_PB11, (pb->b_x >> 8) & 0xff);
		dw_write(FC_DRM_PB12, (pb->b_y >> 0) & 0xff);
		dw_write(FC_DRM_PB13, (pb->b_y >> 8) & 0xff);
		dw_write(FC_DRM_PB14, (pb->w_x >> 0) & 0xff);
		dw_write(FC_DRM_PB15, (pb->w_x >> 8) & 0xff);
		dw_write(FC_DRM_PB16, (pb->w_y >> 0) & 0xff);
		dw_write(FC_DRM_PB17, (pb->w_y >> 8) & 0xff);
		dw_write(FC_DRM_PB18, (pb->luma_max >> 0) & 0xff);
		dw_write(FC_DRM_PB19, (pb->luma_max >> 8) & 0xff);
		dw_write(FC_DRM_PB20, (pb->luma_min >> 0) & 0xff);
		dw_write(FC_DRM_PB21, (pb->luma_min >> 8) & 0xff);
		dw_write(FC_DRM_PB22, (pb->mcll >> 0) & 0xff);
		dw_write(FC_DRM_PB23, (pb->mcll >> 8) & 0xff);
		dw_write(FC_DRM_PB24, (pb->mfll >> 0) & 0xff);
		dw_write(FC_DRM_PB25, (pb->mfll >> 8) & 0xff);
	 }
	dw_write_mask(FC_DRM_HB0, FC_DRM_UP_FC_DRM_HB_MASK, 0x01);
	dw_write_mask(FC_DRM_HB1, FC_DRM_UP_FC_DRM_HB_MASK, 26);
	dw_write_mask(FC_PACKET_TX_EN, FC_PACKET_TX_EN_DRM_TX_EN_MASK, 0x1);
	do {
		udelay(10);
		status = dw_read_mask(FC_DRM_UP, FC_DRM_UP_DRMPACKETUPDATE_MASK);
	} while (status && (timeout--));
	dw_write_mask(FC_DRM_UP,  FC_DRM_UP_DRMPACKETUPDATE_MASK, 0x1);
}

void dw_drm_packet_disabled(void)
{
	log_trace();
	dw_write_mask(FC_PACKET_TX_EN, FC_PACKET_TX_EN_DRM_TX_EN_MASK, 0x0);
}

/*
* get vsif data
* data[0]: hdmi_format filed in vsif
* data[1]: hdmi_vic or 3d strcture filed in vsif
*/
void dw_vsif_get_hdmi_vic(u8 *data)
{
	data[0] = dw_read(FC_VSDPAYLOAD0);
	data[1] = dw_read(FC_VSDPAYLOAD0 + 0x4);
}

/*
* set vsif data
* data[0]: hdmi_format filed in vsif
* data[1]: hdmi_vic or 3d strcture filed in vsif
*/
void dw_vsif_set_hdmi_vic(u8 *data)
{
	 dw_write(FC_VSDPAYLOAD0, data[0]);
	 dw_write(FC_VSDPAYLOAD0 + 0x4, data[1]);
}

/* packets configure is the same as infoframe configure */
int dw_infoframe_packet(struct dw_video_s *video, struct dw_product_s *prod)
{
	u32 oui = 0;
	u8 struct_3d = 0;
	u8 data[4];
	u8 *vendor_payload = prod->mVendorPayload;
	u8 payload_length = prod->mVendorPayloadLength;

	log_trace();

	if (video->mHdmi != DW_TMDS_MODE_HDMI) {
		hdmi_inf("packet not config when dvi mode\n");
		return true;
	}

	if (video->mHdmiVideoFormat == 2) {
		struct_3d = video->m3dStructure;
		video_log("3D packets configure\n");

		/* frame packing || tab || sbs */
		if ((struct_3d == 0) || (struct_3d == 6) || (struct_3d == 8)) {
			data[0] = video->mHdmiVideoFormat << 5; /* PB4 */
			data[1] = struct_3d << 4; /* PB5 */
			data[2] = video->m3dExtData << 4;
			data[3] = 0;
			/* HDMI Licensing, LLC */
			_dw_packet_vsi_config(0x000C03, data, sizeof(data), 1);
			_dw_vsi_enable(0x1);
		} else {
			hdmi_err("3d structure not supported %d\n", struct_3d);
			return false;
		}
	} else if ((video->mHdmiVideoFormat == 0x1) || (video->mHdmiVideoFormat == 0x0)) {
		if (prod != 0) {
			fc_spd_info_t spd_data;

			spd_data.vName    = prod->mVendorName;
			spd_data.vLength  = prod->mVendorNameLength;
			spd_data.pName    = prod->mProductName;
			spd_data.pLength  = prod->mProductNameLength;
			spd_data.code     = prod->mSourceType;
			spd_data.autoSend = 1;

			oui = prod->mOUI;
			_dw_packet_spd_config(&spd_data);
			_dw_packet_vsi_config(oui, vendor_payload,
					payload_length, 1);
			_dw_vsi_enable(0x1);
		} else {
				video_log("No product info provided: not configured\n");
		}
	} else {
		hdmi_err("unknow video format %d\n", video->mHdmiVideoFormat);
		_dw_vsi_enable(0x0);
	}

	_dw_packets_metadata_config();

	/* default phase 1 = true */
	dw_write_mask(FC_GCP, FC_GCP_DEFAULT_PHASE_MASK,
			((video->mPixelPackingDefaultPhase == 1) ? 1 : 0));

	_dw_gamut_config();

	_dw_avi_config(video);

	/* * Colorimetry */
	_dw_gamut_colorimetry_config(video);

	if (video->mHdr) {
		video_log("Is HDR video format\n");
		dw_drm_packet_up(video->pb);
	} else {
		dw_drm_packet_disabled();
	}

	return true;
}
