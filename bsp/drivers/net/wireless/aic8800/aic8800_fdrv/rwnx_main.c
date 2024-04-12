/**
 ******************************************************************************
 *
 * @file rwnx_main.c
 *
 * @brief Entry point of the RWNX driver
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/inetdevice.h>
#include <net/cfg80211.h>
#include <net/ip.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <net/netlink.h>
#include <linux/wireless.h>
#include <linux/if_arp.h>
#include <linux/ctype.h>
#include <linux/random.h>
#include "rwnx_defs.h"
#include "rwnx_dini.h"
#include "rwnx_msg_tx.h"
#include "reg_access.h"
#include "hal_desc.h"
#include "rwnx_debugfs.h"
#include "rwnx_cfgfile.h"
#include "rwnx_irqs.h"
#include "rwnx_radar.h"
#include "rwnx_version.h"
#ifdef CONFIG_RWNX_BFMER
#include "rwnx_bfmer.h"
#endif //(CONFIG_RWNX_BFMER)
#include "rwnx_tdls.h"
#include "rwnx_events.h"
#include "rwnx_compat.h"
#include "rwnx_version.h"
#include "rwnx_main.h"
#include "aicwf_txrxif.h"
#ifdef AICWF_SDIO_SUPPORT
#include "aicwf_sdio.h"
#endif
#ifdef AICWF_USB_SUPPORT
#include "aicwf_usb.h"
#endif
#include "aic_bsp_export.h"
#include "rwnx_wakelock.h"

#define RW_DRV_DESCRIPTION  "RivieraWaves 11nac driver for Linux cfg80211"
#define RW_DRV_COPYRIGHT    "Copyright(c) 2015-2017 RivieraWaves"
#define RW_DRV_AUTHOR       "RivieraWaves S.A.S"

#define RWNX_PRINT_CFM_ERR(req) \
		printk(KERN_CRIT "%s: Status Error(%d)\n", #req, (&req##_cfm)->status)

#define RWNX_HT_CAPABILITIES                                    \
{                                                               \
	.ht_supported   = true,                                     \
	.cap            = 0,                                        \
	.ampdu_factor   = IEEE80211_HT_MAX_AMPDU_64K,               \
	.ampdu_density  = IEEE80211_HT_MPDU_DENSITY_16,             \
	.mcs        = {                                             \
		.rx_mask = { 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, },        \
		.rx_highest = cpu_to_le16(65),                          \
		.tx_params = IEEE80211_HT_MCS_TX_DEFINED,               \
	},                                                          \
}

#define RWNX_VHT_CAPABILITIES                                   \
{                                                               \
	.vht_supported = false,                                     \
	.cap       =                                                \
	  (7 << IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT),\
	.vht_mcs       = {                                          \
		.rx_mcs_map = cpu_to_le16(                              \
					  IEEE80211_VHT_MCS_SUPPORT_0_9    << 0  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 2  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 4  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 6  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 8  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 10 |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 12 |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 14),  \
		.tx_mcs_map = cpu_to_le16(                              \
					  IEEE80211_VHT_MCS_SUPPORT_0_9    << 0  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 2  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 4  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 6  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 8  |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 10 |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 12 |  \
					  IEEE80211_VHT_MCS_NOT_SUPPORTED  << 14),  \
	}                                                           \
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0) || defined(CONFIG_HE_FOR_OLD_KERNEL)
#define RWNX_HE_CAPABILITIES                                    \
{                                                               \
	.has_he = false,                                            \
	.he_cap_elem = {                                            \
		.mac_cap_info[0] = 0,                                   \
		.mac_cap_info[1] = 0,                                   \
		.mac_cap_info[2] = 0,                                   \
		.mac_cap_info[3] = 0,                                   \
		.mac_cap_info[4] = 0,                                   \
		.mac_cap_info[5] = 0,                                   \
		.phy_cap_info[0] = 0,                                   \
		.phy_cap_info[1] = 0,                                   \
		.phy_cap_info[2] = 0,                                   \
		.phy_cap_info[3] = 0,                                   \
		.phy_cap_info[4] = 0,                                   \
		.phy_cap_info[5] = 0,                                   \
		.phy_cap_info[6] = 0,                                   \
		.phy_cap_info[7] = 0,                                   \
		.phy_cap_info[8] = 0,                                   \
		.phy_cap_info[9] = 0,                                   \
		.phy_cap_info[10] = 0,                                  \
	},                                                          \
	.he_mcs_nss_supp = {                                        \
		.rx_mcs_80 = cpu_to_le16(0xfffa),                       \
		.tx_mcs_80 = cpu_to_le16(0xfffa),                       \
		.rx_mcs_160 = cpu_to_le16(0xffff),                      \
		.tx_mcs_160 = cpu_to_le16(0xffff),                      \
		.rx_mcs_80p80 = cpu_to_le16(0xffff),                    \
		.tx_mcs_80p80 = cpu_to_le16(0xffff),                    \
	},                                                          \
	.ppe_thres = {0x08, 0x1c, 0x07},                            \
}
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
#define RWNX_HE_CAPABILITIES                                    \
{                                                               \
	.has_he = false,                                            \
	.he_cap_elem = {                                            \
		.mac_cap_info[0] = 0,                                   \
		.mac_cap_info[1] = 0,                                   \
		.mac_cap_info[2] = 0,                                   \
		.mac_cap_info[3] = 0,                                   \
		.mac_cap_info[4] = 0,                                   \
		.phy_cap_info[0] = 0,                                   \
		.phy_cap_info[1] = 0,                                   \
		.phy_cap_info[2] = 0,                                   \
		.phy_cap_info[3] = 0,                                   \
		.phy_cap_info[4] = 0,                                   \
		.phy_cap_info[5] = 0,                                   \
		.phy_cap_info[6] = 0,                                   \
		.phy_cap_info[7] = 0,                                   \
		.phy_cap_info[8] = 0,                                   \
	},                                                          \
	.he_mcs_nss_supp = {                                        \
		.rx_mcs_80 = cpu_to_le16(0xfffa),                       \
		.tx_mcs_80 = cpu_to_le16(0xfffa),                       \
		.rx_mcs_160 = cpu_to_le16(0xffff),                      \
		.tx_mcs_160 = cpu_to_le16(0xffff),                      \
		.rx_mcs_80p80 = cpu_to_le16(0xffff),                    \
		.tx_mcs_80p80 = cpu_to_le16(0xffff),                    \
	},                                                          \
	.ppe_thres = {0x08, 0x1c, 0x07},                            \
}
#endif
#endif

#define RATE(_bitrate, _hw_rate, _flags) {      \
	.bitrate    = (_bitrate),                   \
	.flags      = (_flags),                     \
	.hw_value   = (_hw_rate),                   \
}

#define CHAN(_freq) {                           \
	.center_freq    = (_freq),                  \
	.max_power  = 30, /* FIXME */               \
}

static struct ieee80211_rate rwnx_ratetable[] = {
	RATE(10,  0x00, 0),
	RATE(20,  0x01, IEEE80211_RATE_SHORT_PREAMBLE),
	RATE(55,  0x02, IEEE80211_RATE_SHORT_PREAMBLE),
	RATE(110, 0x03, IEEE80211_RATE_SHORT_PREAMBLE),
	RATE(60,  0x04, 0),
	RATE(90,  0x05, 0),
	RATE(120, 0x06, 0),
	RATE(180, 0x07, 0),
	RATE(240, 0x08, 0),
	RATE(360, 0x09, 0),
	RATE(480, 0x0A, 0),
	RATE(540, 0x0B, 0),
};

/* The channels indexes here are not used anymore */
static struct ieee80211_channel rwnx_2ghz_channels[] = {
	CHAN(2412),
	CHAN(2417),
	CHAN(2422),
	CHAN(2427),
	CHAN(2432),
	CHAN(2437),
	CHAN(2442),
	CHAN(2447),
	CHAN(2452),
	CHAN(2457),
	CHAN(2462),
	CHAN(2467),
	CHAN(2472),
	CHAN(2484),
	// Extra channels defined only to be used for PHY measures.
	// Enabled only if custregd and custchan parameters are set
	CHAN(2390),
	CHAN(2400),
	CHAN(2410),
	CHAN(2420),
	CHAN(2430),
	CHAN(2440),
	CHAN(2450),
	CHAN(2460),
	CHAN(2470),
	CHAN(2480),
	CHAN(2490),
	CHAN(2500),
	CHAN(2510),
};

static struct ieee80211_channel rwnx_5ghz_channels[] = {
	CHAN(5180),             // 36 -   20MHz
	CHAN(5200),             // 40 -   20MHz
	CHAN(5220),             // 44 -   20MHz
	CHAN(5240),             // 48 -   20MHz
	CHAN(5260),             // 52 -   20MHz
	CHAN(5280),             // 56 -   20MHz
	CHAN(5300),             // 60 -   20MHz
	CHAN(5320),             // 64 -   20MHz
	CHAN(5500),             // 100 -  20MHz
	CHAN(5520),             // 104 -  20MHz
	CHAN(5540),             // 108 -  20MHz
	CHAN(5560),             // 112 -  20MHz
	CHAN(5580),             // 116 -  20MHz
	CHAN(5600),             // 120 -  20MHz
	CHAN(5620),             // 124 -  20MHz
	CHAN(5640),             // 128 -  20MHz
	CHAN(5660),             // 132 -  20MHz
	CHAN(5680),             // 136 -  20MHz
	CHAN(5700),             // 140 -  20MHz
	CHAN(5720),             // 144 -  20MHz
	CHAN(5745),             // 149 -  20MHz
	CHAN(5765),             // 153 -  20MHz
	CHAN(5785),             // 157 -  20MHz
	CHAN(5805),             // 161 -  20MHz
	CHAN(5825),             // 165 -  20MHz
	// Extra channels defined only to be used for PHY measures.
	// Enabled only if custregd and custchan parameters are set
	CHAN(5190),
	CHAN(5210),
	CHAN(5230),
	CHAN(5250),
	CHAN(5270),
	CHAN(5290),
	CHAN(5310),
	CHAN(5330),
	CHAN(5340),
	CHAN(5350),
	CHAN(5360),
	CHAN(5370),
	CHAN(5380),
	CHAN(5390),
	CHAN(5400),
	CHAN(5410),
	CHAN(5420),
	CHAN(5430),
	CHAN(5440),
	CHAN(5450),
	CHAN(5460),
	CHAN(5470),
	CHAN(5480),
	CHAN(5490),
	CHAN(5510),
	CHAN(5530),
	CHAN(5550),
	CHAN(5570),
	CHAN(5590),
	CHAN(5610),
	CHAN(5630),
	CHAN(5650),
	CHAN(5670),
	CHAN(5690),
	CHAN(5710),
	CHAN(5730),
	CHAN(5750),
	CHAN(5760),
	CHAN(5770),
	CHAN(5780),
	CHAN(5790),
	CHAN(5800),
	CHAN(5810),
	CHAN(5820),
	CHAN(5830),
	CHAN(5840),
	CHAN(5850),
	CHAN(5860),
	CHAN(5870),
	CHAN(5880),
	CHAN(5890),
	CHAN(5900),
	CHAN(5910),
	CHAN(5920),
	CHAN(5930),
	CHAN(5940),
	CHAN(5950),
	CHAN(5960),
	CHAN(5970),
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)) || defined(CONFIG_HE_FOR_OLD_KERNEL)
struct ieee80211_sband_iftype_data rwnx_he_capa = {
	.types_mask = BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP),
	.he_cap = RWNX_HE_CAPABILITIES,
};
#endif

static struct ieee80211_supported_band rwnx_band_2GHz = {
	.channels   = rwnx_2ghz_channels,
	.n_channels = ARRAY_SIZE(rwnx_2ghz_channels) - 13, // -13 to exclude extra channels
	.bitrates   = rwnx_ratetable,
	.n_bitrates = ARRAY_SIZE(rwnx_ratetable),
	.ht_cap     = RWNX_HT_CAPABILITIES,
	.vht_cap    = RWNX_VHT_CAPABILITIES,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	.iftype_data = &rwnx_he_capa,
	.n_iftype_data = 1,
#endif
};

static struct ieee80211_supported_band rwnx_band_5GHz = {
	.channels   = rwnx_5ghz_channels,
	.n_channels = ARRAY_SIZE(rwnx_5ghz_channels) - 59, // -59 to exclude extra channels
	.bitrates   = &rwnx_ratetable[4],
	.n_bitrates = ARRAY_SIZE(rwnx_ratetable) - 4,
	.ht_cap     = RWNX_HT_CAPABILITIES,
	.vht_cap    = RWNX_VHT_CAPABILITIES,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	.iftype_data = &rwnx_he_capa,
	.n_iftype_data = 1,
#endif
};

static struct ieee80211_iface_limit rwnx_limits[] = {
	{ .max = 1,
	  .types = BIT(NL80211_IFTYPE_STATION)},
	{ .max = 1,
	  .types = BIT(NL80211_IFTYPE_AP)},
	{ .max = 1,
	  .types = BIT(NL80211_IFTYPE_P2P_CLIENT) | BIT(NL80211_IFTYPE_P2P_GO)},
	{ .max = 1,
	  .types = BIT(NL80211_IFTYPE_P2P_DEVICE),
	}
};

static struct ieee80211_iface_limit rwnx_limits_dfs[] = {
	{ .max = NX_VIRT_DEV_MAX, .types = BIT(NL80211_IFTYPE_AP)}
};

static const struct ieee80211_iface_combination rwnx_combinations[] = {
	{
		.limits                 = rwnx_limits,
		.n_limits               = ARRAY_SIZE(rwnx_limits),
		.num_different_channels = NX_CHAN_CTXT_CNT,
		.max_interfaces         = NX_VIRT_DEV_MAX,
	},
	/* Keep this combination as the last one */
	{
		.limits                 = rwnx_limits_dfs,
		.n_limits               = ARRAY_SIZE(rwnx_limits_dfs),
		.num_different_channels = 1,
		.max_interfaces         = NX_VIRT_DEV_MAX,
		.radar_detect_widths = (BIT(NL80211_CHAN_WIDTH_20_NOHT) |
								BIT(NL80211_CHAN_WIDTH_20) |
								BIT(NL80211_CHAN_WIDTH_40) |
								BIT(NL80211_CHAN_WIDTH_80)),
	}
};

/* There isn't a lot of sense in it, but you can transmit anything you like */
static struct ieee80211_txrx_stypes
rwnx_default_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ACTION >> 4) |
			   BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			   BIT(IEEE80211_STYPE_AUTH >> 4)),
	},
	[NL80211_IFTYPE_AP] = {
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			   BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			   BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			   BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			   BIT(IEEE80211_STYPE_AUTH >> 4) |
			   BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			   BIT(IEEE80211_STYPE_ACTION >> 4)),
	},
	[NL80211_IFTYPE_AP_VLAN] = {
		/* copy AP */
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			   BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			   BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			   BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			   BIT(IEEE80211_STYPE_AUTH >> 4) |
			   BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			   BIT(IEEE80211_STYPE_ACTION >> 4)),
	},
	[NL80211_IFTYPE_P2P_CLIENT] = {
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ACTION >> 4) |
			   BIT(IEEE80211_STYPE_PROBE_REQ >> 4)),
	},
	[NL80211_IFTYPE_P2P_GO] = {
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			   BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			   BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			   BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			   BIT(IEEE80211_STYPE_AUTH >> 4) |
			   BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			   BIT(IEEE80211_STYPE_ACTION >> 4)),
	},
	[NL80211_IFTYPE_P2P_DEVICE] = {
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ACTION >> 4) |
			   BIT(IEEE80211_STYPE_PROBE_REQ >> 4)),
	},
	[NL80211_IFTYPE_MESH_POINT] = {
		.tx = 0xffff,
		.rx = (BIT(IEEE80211_STYPE_ACTION >> 4) |
			   BIT(IEEE80211_STYPE_AUTH >> 4) |
			   BIT(IEEE80211_STYPE_DEAUTH >> 4)),
	},
};


static u32 cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC, // reserved entries to enable AES-CMAC and/or SMS4
	WLAN_CIPHER_SUITE_SMS4,
	0,
};
#define NB_RESERVED_CIPHER 1;

static const int rwnx_ac2hwq[1][NL80211_NUM_ACS] = {
	{
		[NL80211_TXQ_Q_VO] = RWNX_HWQ_VO,
		[NL80211_TXQ_Q_VI] = RWNX_HWQ_VI,
		[NL80211_TXQ_Q_BE] = RWNX_HWQ_BE,
		[NL80211_TXQ_Q_BK] = RWNX_HWQ_BK
	}
};

const int rwnx_tid2hwq[IEEE80211_NUM_TIDS] = {
	RWNX_HWQ_BE,
	RWNX_HWQ_BK,
	RWNX_HWQ_BK,
	RWNX_HWQ_BE,
	RWNX_HWQ_VI,
	RWNX_HWQ_VI,
	RWNX_HWQ_VO,
	RWNX_HWQ_VO,
	/* TID_8 is used for management frames */
	RWNX_HWQ_VO,
	/* At the moment, all others TID are mapped to BE */
	RWNX_HWQ_BE,
	RWNX_HWQ_BE,
	RWNX_HWQ_BE,
	RWNX_HWQ_BE,
	RWNX_HWQ_BE,
	RWNX_HWQ_BE,
	RWNX_HWQ_BE,
};

static const int rwnx_hwq2uapsd[NL80211_NUM_ACS] = {
	[RWNX_HWQ_VO] = IEEE80211_WMM_IE_STA_QOSINFO_AC_VO,
	[RWNX_HWQ_VI] = IEEE80211_WMM_IE_STA_QOSINFO_AC_VI,
	[RWNX_HWQ_BE] = IEEE80211_WMM_IE_STA_QOSINFO_AC_BE,
	[RWNX_HWQ_BK] = IEEE80211_WMM_IE_STA_QOSINFO_AC_BK,
};

extern uint8_t scanning;


/*********************************************************************
 * helper
 *********************************************************************/
struct rwnx_sta *rwnx_get_sta(struct rwnx_hw *rwnx_hw, const u8 *mac_addr)
{
	int i;

	for (i = 0; i < NX_REMOTE_STA_MAX; i++) {
		struct rwnx_sta *sta = &rwnx_hw->sta_table[i];
		if (sta->valid && (memcmp(mac_addr, &sta->mac_addr, 6) == 0))
			return sta;
	}

	return NULL;
}

void rwnx_enable_wapi(struct rwnx_hw *rwnx_hw)
{
	cipher_suites[rwnx_hw->wiphy->n_cipher_suites] = WLAN_CIPHER_SUITE_SMS4;
	rwnx_hw->wiphy->n_cipher_suites++;
	rwnx_hw->wiphy->flags |= WIPHY_FLAG_CONTROL_PORT_PROTOCOL;
}

void rwnx_enable_mfp(struct rwnx_hw *rwnx_hw)
{
	cipher_suites[rwnx_hw->wiphy->n_cipher_suites] = WLAN_CIPHER_SUITE_AES_CMAC;
	rwnx_hw->wiphy->n_cipher_suites++;
}

u8 *rwnx_build_bcn(struct rwnx_bcn *bcn, struct cfg80211_beacon_data *new)
{
	u8 *buf, *pos;

	if (new->head) {
		u8 *head = kmalloc(new->head_len, GFP_KERNEL);

		if (!head)
			return NULL;

		if (bcn->head)
			kfree(bcn->head);

		bcn->head = head;
		bcn->head_len = new->head_len;
		memcpy(bcn->head, new->head, new->head_len);
	}
	if (new->tail) {
		u8 *tail = kmalloc(new->tail_len, GFP_KERNEL);

		if (!tail)
			return NULL;

		if (bcn->tail)
			kfree(bcn->tail);

		bcn->tail = tail;
		bcn->tail_len = new->tail_len;
		memcpy(bcn->tail, new->tail, new->tail_len);
	}

	if (!bcn->head)
		return NULL;

	bcn->tim_len = 6;
	bcn->len = bcn->head_len + bcn->tail_len + bcn->ies_len + bcn->tim_len;

	buf = kmalloc(bcn->len, GFP_KERNEL);
	if (!buf)
		return NULL;

	// Build the beacon buffer
	pos = buf;
	memcpy(pos, bcn->head, bcn->head_len);
	pos += bcn->head_len;
	*pos++ = WLAN_EID_TIM;
	*pos++ = 4;
	*pos++ = 0;
	*pos++ = bcn->dtim;
	*pos++ = 0;
	*pos++ = 0;
	if (bcn->tail) {
		memcpy(pos, bcn->tail, bcn->tail_len);
		pos += bcn->tail_len;
	}
	if (bcn->ies) {
		memcpy(pos, bcn->ies, bcn->ies_len);
	}

	return buf;
}


static void rwnx_del_bcn(struct rwnx_bcn *bcn)
{
	if (bcn->head) {
		kfree(bcn->head);
		bcn->head = NULL;
	}
	bcn->head_len = 0;

	if (bcn->tail) {
		kfree(bcn->tail);
		bcn->tail = NULL;
	}
	bcn->tail_len = 0;

	if (bcn->ies) {
		kfree(bcn->ies);
		bcn->ies = NULL;
	}
	bcn->ies_len = 0;
	bcn->tim_len = 0;
	bcn->dtim = 0;
	bcn->len = 0;
}

/**
 * Link channel ctxt to a vif and thus increments count for this context.
 */
void rwnx_chanctx_link(struct rwnx_vif *vif, u8 ch_idx,
					   struct cfg80211_chan_def *chandef)
{
	struct rwnx_chanctx *ctxt;

	if (ch_idx >= NX_CHAN_CTXT_CNT) {
		WARN(1, "Invalid channel ctxt id %d", ch_idx);
		return;
	}

	vif->ch_index = ch_idx;
	ctxt = &vif->rwnx_hw->chanctx_table[ch_idx];
	ctxt->count++;

	// For now chandef is NULL for STATION interface
	if (chandef) {
		if (!ctxt->chan_def.chan)
			ctxt->chan_def = *chandef;
		else {
			// TODO. check that chandef is the same as the one already
			// set for this ctxt
		}
	}
}

/**
 * Unlink channel ctxt from a vif and thus decrements count for this context
 */
void rwnx_chanctx_unlink(struct rwnx_vif *vif)
{
	struct rwnx_chanctx *ctxt;

	if (vif->ch_index == RWNX_CH_NOT_SET)
		return;

	ctxt = &vif->rwnx_hw->chanctx_table[vif->ch_index];

	if (ctxt->count == 0) {
		WARN(1, "Chan ctxt ref count is already 0");
	} else {
		ctxt->count--;
	}

	if (ctxt->count == 0) {
		if (vif->ch_index == vif->rwnx_hw->cur_chanctx) {
			/* If current chan ctxt is no longer linked to a vif
			   disable radar detection (no need to check if it was activated) */
			rwnx_radar_detection_enable(&vif->rwnx_hw->radar,
										RWNX_RADAR_DETECT_DISABLE,
										RWNX_RADAR_RIU);
		}
		/* set chan to null, so that if this ctxt is relinked to a vif that
		   don't have channel information, don't use wrong information */
		ctxt->chan_def.chan = NULL;
	}
	vif->ch_index = RWNX_CH_NOT_SET;
}

int rwnx_chanctx_valid(struct rwnx_hw *rwnx_hw, u8 ch_idx)
{
	if (ch_idx >= NX_CHAN_CTXT_CNT ||
		rwnx_hw->chanctx_table[ch_idx].chan_def.chan == NULL) {
		return 0;
	}

	return 1;
}

static void rwnx_del_csa(struct rwnx_vif *vif)
{
	struct rwnx_csa *csa = vif->ap.csa;

	if (!csa)
		return;

	rwnx_del_bcn(&csa->bcn);
	kfree(csa);
	vif->ap.csa = NULL;
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
static void rwnx_csa_finish(struct work_struct *ws)
{
	struct rwnx_csa *csa = container_of(ws, struct rwnx_csa, work);
	struct rwnx_vif *vif = csa->vif;
	struct rwnx_hw *rwnx_hw = vif->rwnx_hw;
	int error = csa->status;
	u8 *buf, *pos;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	buf = kmalloc(csa->bcn.len, GFP_KERNEL);
	if (!buf) {
		printk ("%s buf fail\n", __func__);
		return;
	}
	pos = buf;

	memcpy(pos, csa->bcn.head, csa->bcn.head_len);
	pos += csa->bcn.head_len;
	*pos++ = WLAN_EID_TIM;
	*pos++ = 4;
	*pos++ = 0;
	*pos++ = csa->bcn.dtim;
	*pos++ = 0;
	*pos++ = 0;
	if (csa->bcn.tail) {
		memcpy(pos, csa->bcn.tail, csa->bcn.tail_len);
		pos += csa->bcn.tail_len;
	}
	if (csa->bcn.ies) {
		memcpy(pos, csa->bcn.ies, csa->bcn.ies_len);
	}

	if (!error) {
		error = rwnx_send_bcn(rwnx_hw, buf, vif->vif_index, csa->bcn.len);
		if (error)
			return;
		error = rwnx_send_bcn_change(rwnx_hw, vif->vif_index, 0,
									 csa->bcn.len, csa->bcn.head_len,
									 csa->bcn.tim_len, NULL);
	}
	if (error) {
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0))
		cfg80211_stop_iface(rwnx_hw->wiphy, &vif->wdev, GFP_KERNEL);
		#else
		cfg80211_disconnected(vif->ndev, 0, NULL, 0, 0, GFP_KERNEL);
		#endif
	} else {
		mutex_lock(&vif->wdev.mtx);
		__acquire(&vif->wdev.mtx);
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_chanctx_unlink(vif);
		rwnx_chanctx_link(vif, csa->ch_idx, &csa->chandef);
		if (rwnx_hw->cur_chanctx == csa->ch_idx) {
			rwnx_radar_detection_enable_on_cur_channel(rwnx_hw);
			rwnx_txq_vif_start(vif, RWNX_TXQ_STOP_CHAN, rwnx_hw);
		} else
			rwnx_txq_vif_stop(vif, RWNX_TXQ_STOP_CHAN, rwnx_hw);
		spin_unlock_bh(&rwnx_hw->cb_lock);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 94)) && defined(KERNEL_AOSP)
		cfg80211_ch_switch_notify(vif->ndev, &csa->chandef, 0, 0);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)) && defined(KERNEL_AOSP)
		cfg80211_ch_switch_notify(vif->ndev, &csa->chandef, 0);
#else
		cfg80211_ch_switch_notify(vif->ndev, &csa->chandef);
#endif
		mutex_unlock(&vif->wdev.mtx);
		__release(&vif->wdev.mtx);
	}
	rwnx_del_csa(vif);
}
#endif

/**
 * rwnx_external_auth_enable - Enable external authentication on a vif
 *
 * @vif: VIF on which external authentication must be enabled
 *
 * External authentication requires to start TXQ for unknown STA in
 * order to send auth frame pusehd by user space.
 * Note: It is assumed that fw is on the correct channel.
 */
void rwnx_external_auth_enable(struct rwnx_vif *vif)
{
	vif->sta.external_auth = true;
	rwnx_txq_unk_vif_init(vif);
	rwnx_txq_start(rwnx_txq_vif_get(vif, NX_UNK_TXQ_TYPE), 0);
}

/**
 * rwnx_external_auth_disable - Disable external authentication on a vif
 *
 * @vif: VIF on which external authentication must be disabled
 */
void rwnx_external_auth_disable(struct rwnx_vif *vif)
{
	if (!vif->sta.external_auth)
		return;

	vif->sta.external_auth = false;
	rwnx_txq_unk_vif_deinit(vif);
}

/**
 * rwnx_update_mesh_power_mode -
 *
 * @vif: mesh VIF  for which power mode is updated
 *
 * Does nothing if vif is not a mesh point interface.
 * Since firmware doesn't support one power save mode per link select the
 * most "active" power mode among all mesh links.
 * Indeed as soon as we have to be active on one link we might as well be
 * active on all links.
 *
 * If there is no link then the power mode for next peer is used;
 */
void rwnx_update_mesh_power_mode(struct rwnx_vif *vif)
{
	enum nl80211_mesh_power_mode mesh_pm;
	struct rwnx_sta *sta;
	struct mesh_config mesh_conf;
	struct mesh_update_cfm cfm;
	u32 mask;

	if (RWNX_VIF_TYPE(vif) != NL80211_IFTYPE_MESH_POINT)
		return;

	if (list_empty(&vif->ap.sta_list)) {
		mesh_pm = vif->ap.next_mesh_pm;
	} else {
		mesh_pm = NL80211_MESH_POWER_DEEP_SLEEP;
		list_for_each_entry(sta, &vif->ap.sta_list, list) {
			if (sta->valid && (sta->mesh_pm < mesh_pm)) {
				mesh_pm = sta->mesh_pm;
			}
		}
	}

	if (mesh_pm == vif->ap.mesh_pm)
		return;

	mask = BIT(NL80211_MESHCONF_POWER_MODE - 1);
	mesh_conf.power_mode = mesh_pm;
	if (rwnx_send_mesh_update_req(vif->rwnx_hw, vif, mask, &mesh_conf, &cfm) ||
		cfm.status)
		return;

	vif->ap.mesh_pm = mesh_pm;
}


/*********************************************************************
 * netdev callbacks
 ********************************************************************/
/**
 * int (*ndo_open)(struct net_device *dev);
 *     This function is called when network device transistions to the up
 *     state.
 *
 * - Start FW if this is the first interface opened
 * - Add interface at fw level
 */
static int rwnx_open(struct net_device *dev)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = rwnx_vif->rwnx_hw;
	struct mm_add_if_cfm add_if_cfm;
	int error = 0;
	u8 rwnx_rx_gain = 0x0E;
	int waiting_counter = 10;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	while (test_bit(RWNX_DEV_STARTED, &rwnx_vif->drv_flags)) {
		msleep(100);
		printk("%s waiting for rwnx_close \r\n", __func__);
		waiting_counter--;
		if (waiting_counter == 0) {
			printk("%s error waiting for close time out \r\n", __func__);
			break;
		}
	}

	// Check if it is the first opened VIF
	if (rwnx_hw->vif_started == 0) {
		// Start the FW
		error = rwnx_send_start(rwnx_hw);
		if (error)
			return error;

		if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
			error = rwnx_send_dbg_mem_mask_write_req(rwnx_hw, 0x4033b300, 0xFF, rwnx_rx_gain);
			if (error) {
				return error;
			}
		}
		#ifdef CONFIG_COEX
		if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_STATION || RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_CLIENT)
			rwnx_send_coex_req(rwnx_hw, 0, 1);
		#endif

		/* Device is now started */
		set_bit(RWNX_DEV_STARTED, &rwnx_vif->drv_flags);
		atomic_set(&rwnx_vif->drv_conn_state, RWNX_DRV_STATUS_DISCONNECTED);
	}
	#ifdef CONFIG_COEX
	else if ((RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_AP || RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_GO)) {
	   rwnx_send_coex_req(rwnx_hw, 1, 0);
	}
	#endif

	if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_CLIENT || RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_GO) {
		if (!rwnx_hw->is_p2p_alive) {
			if (rwnx_hw->p2p_dev_vif && !rwnx_hw->p2p_dev_vif->up) {
				printk("add p2p dev if\n");
				error = rwnx_send_add_if(rwnx_hw, rwnx_hw->p2p_dev_vif->wdev.address,
										 RWNX_VIF_TYPE(rwnx_hw->p2p_dev_vif),
										 false, &add_if_cfm);
				if (error)
					return error;

				if (add_if_cfm.status != 0)
					return -EIO;

				/* Save the index retrieved from LMAC */
				spin_lock_bh(&rwnx_hw->cb_lock);
				rwnx_hw->p2p_dev_vif->vif_index = add_if_cfm.inst_nbr;
				rwnx_hw->p2p_dev_vif->up = true;
				rwnx_hw->vif_started++;
				rwnx_hw->vif_table[add_if_cfm.inst_nbr] = rwnx_hw->p2p_dev_vif;
				spin_unlock_bh(&rwnx_hw->cb_lock);
			}
			rwnx_hw->is_p2p_alive = 1;
			mod_timer(&rwnx_hw->p2p_alive_timer, jiffies + msecs_to_jiffies(1000));
			atomic_set(&rwnx_hw->p2p_alive_timer_count, 0);
		}
	}

	if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_AP_VLAN) {
		/* For AP_vlan use same fw and drv indexes. We ensure that this index
		   will not be used by fw for another vif by taking index >= NX_VIRT_DEV_MAX */
		add_if_cfm.inst_nbr = rwnx_vif->drv_vif_index;
		netif_tx_stop_all_queues(dev);

		/* Save the index retrieved from LMAC */
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_vif->vif_index = add_if_cfm.inst_nbr;
		rwnx_vif->up = true;
		rwnx_hw->vif_started++;
		rwnx_hw->vif_table[add_if_cfm.inst_nbr] = rwnx_vif;
		spin_unlock_bh(&rwnx_hw->cb_lock);
	} else {
		/* Forward the information to the LMAC,
		 *     p2p value not used in FMAC configuration, iftype is sufficient */
		error = rwnx_send_add_if (rwnx_hw, rwnx_vif->wdev.address, RWNX_VIF_TYPE(rwnx_vif), false, &add_if_cfm);
		if (error) {
			printk("add if fail\n");
			return error;
		}

		if (add_if_cfm.status != 0) {
			RWNX_PRINT_CFM_ERR(add_if);
			return -EIO;
		}

		/* Save the index retrieved from LMAC */
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_vif->vif_index = add_if_cfm.inst_nbr;
		rwnx_vif->up = true;
		rwnx_hw->vif_started++;
		rwnx_hw->vif_table[add_if_cfm.inst_nbr] = rwnx_vif;
		spin_unlock_bh(&rwnx_hw->cb_lock);
	}

	if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_MONITOR) {
		rwnx_hw->monitor_vif = rwnx_vif->vif_index;
		if (rwnx_vif->ch_index != RWNX_CH_NOT_SET) {
			//Configure the monitor channel
			error = rwnx_send_config_monitor_req(rwnx_hw, &rwnx_hw->chanctx_table[rwnx_vif->ch_index].chan_def, NULL);
		}
	}

	//netif_carrier_off(dev);
	netif_start_queue(dev);

	return error;
}

/**
 * int (*ndo_stop)(struct net_device *dev);
 *     This function is called when network device transistions to the down
 *     state.
 *
 * - Remove interface at fw level
 * - Reset FW if this is the last interface opened
 */
static int rwnx_close(struct net_device *dev)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = rwnx_vif->rwnx_hw;
	int waiting_counter = 20;
	int test_counter = 0;
	int ret;
#if defined(AICWF_USB_SUPPORT)
	struct aicwf_bus *bus_if = NULL;
	struct aic_usb_dev *usbdev = NULL;
	bus_if = dev_get_drvdata(rwnx_hw->dev);
	usbdev = bus_if->bus_priv.usb;
#elif defined(AICWF_SDIO_SUPPORT)
	struct aicwf_bus *bus_if = NULL;
	struct aic_sdio_dev *sdiodev = NULL;
#else
#endif

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	test_counter = waiting_counter;
	while (atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_DISCONNECTING ||
		atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_CONNECTING) {
		printk("%s wifi is connecting or disconnecting, waiting 200ms for state to stable\r\n", __func__);
		msleep(200);
		test_counter--;
		if (test_counter == 0) {
			printk("%s connecting or disconnecting, not finish\r\n", __func__);
			WARN_ON(1);
			break;
		}
	}

#if defined(AICWF_USB_SUPPORT) || defined(AICWF_SDIO_SUPPORT)
	if (scanning) {
		scanning = false;
	}
#endif

	netdev_info(dev, "CLOSE");

	rwnx_radar_cancel_cac(&rwnx_hw->radar);

	/* Abort scan request on the vif */
	if (rwnx_hw->scan_request &&
		rwnx_hw->scan_request->wdev == &rwnx_vif->wdev) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
		struct cfg80211_scan_info info = {
			.aborted = true,
		};

		cfg80211_scan_done(rwnx_hw->scan_request, &info);
#else
		cfg80211_scan_done(rwnx_hw->scan_request, true);
#endif
		rwnx_hw->scan_request = NULL;

		ret = rwnx_send_scanu_cancel_req(rwnx_hw, NULL);
		mdelay(35);//make sure firmware take affect
		if (ret) {
			printk("scanu_cancel fail\n");
			return ret;
		}
	}

	if (rwnx_hw->roc_elem && (rwnx_hw->roc_elem->wdev == &rwnx_vif->wdev)) {
		printk(KERN_CRIT "%s clear roc\n", __func__);
		/* Initialize RoC element pointer to NULL, indicate that RoC can be started */
		kfree(rwnx_hw->roc_elem);
		rwnx_hw->roc_elem = NULL;
	}

	rwnx_vif->up = false;

	if (netif_carrier_ok(dev)) {
		if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_STATION ||
			RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_CLIENT) {
			cfg80211_disconnected(dev, WLAN_REASON_DEAUTH_LEAVING,
								  NULL, 0, true, GFP_ATOMIC);
			netif_tx_stop_all_queues(dev);
			netif_carrier_off(dev);
			udelay(1000);
		} else if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_AP_VLAN) {
			netif_carrier_off(dev);
		} else {
			netdev_warn(dev, "AP not stopped when disabling interface");
		}
	}

#if defined(AICWF_USB_SUPPORT)
	if (usbdev != NULL) {
		if (usbdev->state != USB_DOWN_ST)
			rwnx_send_remove_if (rwnx_hw, rwnx_vif->vif_index, false);
	}
#endif
#if defined(AICWF_SDIO_SUPPORT)
	bus_if = dev_get_drvdata(rwnx_hw->dev);
	if (bus_if) {
		sdiodev = bus_if->bus_priv.sdio;
	}
	if (sdiodev != NULL) {
		if (sdiodev->bus_if->state != BUS_DOWN_ST) {
			if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_STATION ||
				RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_CLIENT) {
				test_counter = waiting_counter;
				if (atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_CONNECTED) {
					atomic_set(&rwnx_vif->drv_conn_state, RWNX_DRV_STATUS_DISCONNECTING);
					rwnx_send_sm_disconnect_req(rwnx_hw, rwnx_vif, 3);
					while (atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_DISCONNECTING) {
						printk("%s wifi is disconnecting, waiting 100ms for state to stable\r\n", __func__);
						msleep(100);
						test_counter--;
						if (test_counter == 0)
							break;
					}
				}
			}
			rwnx_send_remove_if (rwnx_hw, rwnx_vif->vif_index, false);
		}
	}
#endif
	/* Ensure that we won't process disconnect ind */
	spin_lock_bh(&rwnx_hw->cb_lock);

	rwnx_hw->vif_table[rwnx_vif->vif_index] = NULL;

	rwnx_chanctx_unlink(rwnx_vif);

	if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_MONITOR)
		rwnx_hw->monitor_vif = RWNX_INVALID_VIF;

	rwnx_hw->vif_started--;
	spin_unlock_bh(&rwnx_hw->cb_lock);

	if (rwnx_hw->vif_started == 0) {
	/* This also lets both ipc sides remain in sync before resetting */
#ifdef AICWF_USB_SUPPORT
		if (usbdev->bus_if->state != BUS_DOWN_ST) {
#else
		if (sdiodev->bus_if->state != BUS_DOWN_ST) {
#endif
#ifdef CONFIG_COEX
			rwnx_send_coex_req(rwnx_hw, 1, 0);
#endif
			rwnx_send_reset(rwnx_hw);
			// Set parameters to firmware
			rwnx_send_me_config_req(rwnx_hw);
			// Set channel parameters to firmware
			rwnx_send_me_chan_config_req(rwnx_hw);
		}
		clear_bit(RWNX_DEV_STARTED, &rwnx_vif->drv_flags);
	}
#ifdef CONFIG_COEX
	else {
		if (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_AP || RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_GO)
			rwnx_send_coex_req(rwnx_hw, 0, 1);
	}
#endif

	return 0;
}

#ifdef CONFIG_RFTEST
enum {
	SET_TX,
	SET_TXSTOP,
	SET_TXTONE,
	SET_RX,
	GET_RX_RESULT,
	SET_RXSTOP,
	SET_RX_METER,
	SET_POWER,
	SET_XTAL_CAP,
	SET_XTAL_CAP_FINE,
	GET_EFUSE_BLOCK,
	SET_FREQ_CAL,
	SET_FREQ_CAL_FINE,
	GET_FREQ_CAL,
	SET_MAC_ADDR,
	GET_MAC_ADDR,
	SET_BT_MAC_ADDR,
	GET_BT_MAC_ADDR,
	SET_VENDOR_INFO,
	GET_VENDOR_INFO,
	RDWR_PWRMM,
	RDWR_PWRIDX,
	RDWR_PWRLVL = RDWR_PWRIDX,
	RDWR_PWROFST,
	RDWR_DRVIBIT,
	RDWR_EFUSE_PWROFST,
	RDWR_EFUSE_DRVIBIT,
	SET_PAPR,
	SET_CAL_XTAL,
	GET_CAL_XTAL_RES,
	SET_COB_CAL,
	GET_COB_CAL_RES,
	RDWR_EFUSE_USRDATA,
	SET_NOTCH,
	RDWR_PWROFSTFINE,
	RDWR_EFUSE_PWROFSTFINE,
	RDWR_EFUSE_SDIOCFG,
	RDWR_EFUSE_USBVIDPID,
#ifdef CONFIG_USB_BT
	BT_CMD_BASE = 0x100,
	BT_RESET,
	BT_TXDH,
	BT_RXDH,
	BT_STOP,
#endif
};

typedef struct {
	u8_l chan;
	u8_l bw;
	u8_l mode;
	u8_l rate;
	u16_l length;
	u16_l tx_intv_us;
} cmd_rf_settx_t;

typedef struct {
	u8_l val;
} cmd_rf_setfreq_t;

typedef struct {
	u8_l chan;
	u8_l bw;
} cmd_rf_rx_t;

typedef struct {
	u8_l block;
} cmd_rf_getefuse_t;

typedef struct {
	u8_l dutid;
	u8_l chip_num;
	u8_l dis_xtal;
} cmd_rf_setcobcal_t;

typedef struct {
	u16_l dut_rcv_golden_num;
	u8_l golden_rcv_dut_num;
	s8_l rssi_static;
	s8_l snr_static;
	s8_l dut_rssi_static;
	u16_l reserved;
} cob_result_ptr_t;
#endif

#define CMD_MAXARGS 20

static int parse_line (char *line, char *argv[])
{
	int nargs = 0;

	while (nargs < CMD_MAXARGS) {
		/* skip any white space */
		while ((*line == ' ') || (*line == '\t')) {
			++line;
		}

		if (*line == '\0') {    /* end of line, no more args    */
			argv[nargs] = 0;
			return nargs;
		}

		/* Argument include space should be bracketed by quotation mark */
		if (*line == '\"') {
			/* Skip quotation mark */
			line++;

			/* Begin of argument string */
			argv[nargs++] = line;

			/* Until end of argument */
			while (*line && (*line != '\"')) {
				++line;
			}
		} else {
			argv[nargs++] = line;    /* begin of argument string    */

			/* find end of string */
			while (*line && (*line != ' ') && (*line != '\t')) {
				++line;
			}
		}

		if (*line == '\0') {    /* end of line, no more args    */
			argv[nargs] = 0;
			return nargs;
		}

		*line++ = '\0';         /* terminate current arg     */
	}

	printk("** Too many args (max. %d) **\n", CMD_MAXARGS);

	return nargs;
}

unsigned int command_strtoul(const char *cp, char **endp, unsigned int base)
{
	unsigned int result = 0, value, is_neg = 0;

	if (*cp == '0') {
		cp++;
		if ((*cp == 'x') && isxdigit(cp[1])) {
			base = 16;
			cp++;
		}
		if (!base) {
			base = 8;
		}
	}
	if (!base) {
		base = 10;
	}
	if (*cp == '-') {
		is_neg = 1;
		cp++;
	}
	while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp - '0' : (islower(*cp) ? toupper(*cp) : *cp) - 'A' + 10) < base) {
		result = result * base + value;
		cp++;
	}
	if (is_neg)
		result = (unsigned int)((int)result * (-1));

	if (endp)
		*endp = (char *)cp;
	return result;
}


int handle_private_cmd(struct net_device *net, char *command, u32 cmd_len)
{
	int bytes_written = 0;
	char *para = NULL;
	char *cmd = NULL;
	char *argv[CMD_MAXARGS + 1];
	int argc;
#ifdef CONFIG_RFTEST
	struct dbg_rftest_cmd_cfm cfm;
	u8_l mac_addr[6];
	cmd_rf_settx_t settx_param;
	cmd_rf_rx_t setrx_param;
	int freq;
	cmd_rf_getefuse_t getefuse_param;
	cmd_rf_setfreq_t cmd_setfreq;
#ifdef AICWF_SDIO_SUPPORT
	cmd_rf_setcobcal_t setcob_cal;
	cob_result_ptr_t *cob_result_ptr;
#endif
	u8_l ana_pwr;
	u8_l dig_pwr;
	u8_l pwr;
	u8_l xtal_cap;
	u8_l xtal_cap_fine;
	u8_l vendor_info;
	#ifdef CONFIG_USB_BT
	int bt_index;
	u8_l dh_cmd_reset[4];
	u8_l dh_cmd_txdh[18];
	u8_l dh_cmd_rxdh[17];
	u8_l dh_cmd_stop[5];
	#endif
#endif
#ifdef AICWF_SDIO_SUPPORT
	u8_l state;
#endif
	u8_l buf[2];
	s8_l freq_ = 0;
	u8_l func = 0;

	argc = parse_line(command, argv);
	if (argc == 0) {
		return -1;
	}

	do {
#ifdef CONFIG_RFTEST
	#ifdef AICWF_SDIO_SUPPORT
		struct rwnx_hw *p_rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
	#endif
	#ifdef AICWF_USB_SUPPORT
		struct rwnx_hw *p_rwnx_hw = g_rwnx_plat->usbdev->rwnx_hw;
	#endif
		if (strcasecmp(argv[0], "GET_RX_RESULT") == 0) {
			printk("get_rx_result\n");
			rwnx_send_rftest_req(p_rwnx_hw, GET_RX_RESULT, 0, NULL, &cfm);
			memcpy(command, &cfm.rftest_result[0], 8);
			bytes_written = 8;
		} else if (strcasecmp(argv[0], "SET_TX") == 0) {
			printk("set_tx\n");
			if (argc < 6) {
				printk("wrong param\n");
				break;
			}
			settx_param.chan = command_strtoul(argv[1], NULL, 10);
			settx_param.bw = command_strtoul(argv[2], NULL, 10);
			settx_param.mode = command_strtoul(argv[3], NULL, 10);
			settx_param.rate = command_strtoul(argv[4], NULL, 10);
			settx_param.length = command_strtoul(argv[5], NULL, 10);
			if (argc > 6) {
				settx_param.tx_intv_us = command_strtoul(argv[6], NULL, 10);
			} else {
				settx_param.tx_intv_us = 0;
			}
			printk("txparam:%d,%d,%d,%d,%d,%d\n", settx_param.chan, settx_param.bw,
					settx_param.mode, settx_param.rate, settx_param.length, settx_param.tx_intv_us);
			rwnx_send_rftest_req(p_rwnx_hw, SET_TX, sizeof(cmd_rf_settx_t), (u8_l *)&settx_param, NULL);
		} else if (strcasecmp(argv[0], "SET_TXSTOP") == 0) {
			printk("settx_stop\n");
			rwnx_send_rftest_req(p_rwnx_hw, SET_TXSTOP, 0, NULL, NULL);
		} else if (strcasecmp(argv[0], "SET_TXTONE") == 0) {
			printk("set_tx_tone,argc:%d\n", argc);
			if ((argc == 2) || (argc == 3)) {
				printk("argv 1:%s\n", argv[1]);
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
				if (argc == 3) {
					printk("argv 2:%s\n", argv[2]);
					freq_ = (u8_l)command_strtoul(argv[2], NULL, 10);
				} else {
					freq_ = 0;
				}
				buf[0] = func;
				buf[1] = (u8_l)freq_;
				rwnx_send_rftest_req(p_rwnx_hw, SET_TXTONE, argc - 1, buf, NULL);
			} else {
				printk("wrong args\n");
			}
		} else if (strcasecmp(argv[0], "SET_RX") == 0) {
			printk("set_rx\n");
			if (argc < 3) {
				printk("wrong param\n");
				break;
			}
			setrx_param.chan = command_strtoul(argv[1], NULL, 10);
			setrx_param.bw = command_strtoul(argv[2], NULL, 10);
			rwnx_send_rftest_req(p_rwnx_hw, SET_RX, sizeof(cmd_rf_rx_t), (u8_l *)&setrx_param, NULL);
		} else if (strcasecmp(argv[0], "SET_RXSTOP") == 0) {
			printk("set_rxstop\n");
			rwnx_send_rftest_req(p_rwnx_hw, SET_RXSTOP, 0, NULL, NULL);
		} else if (strcasecmp(argv[0], "SET_RX_METER") == 0) {
			printk("set_rx_meter\n");
			freq = (int)command_strtoul(argv[1], NULL, 10);
			rwnx_send_rftest_req(p_rwnx_hw, SET_RX_METER, sizeof(freq), (u8_l *)&freq, NULL);
		} else if (strcasecmp(argv[0], "SET_FREQ_CAL") == 0) {
			printk("set_freq_cal\n");
			if (argc < 2) {
				printk("wrong param\n");
				break;
			}
			cmd_setfreq.val = command_strtoul(argv[1], NULL, 16);
			printk("param:%x\r\n", cmd_setfreq.val);
			rwnx_send_rftest_req(p_rwnx_hw, SET_FREQ_CAL, sizeof(cmd_rf_setfreq_t), (u8_l *)&cmd_setfreq, &cfm);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "SET_FREQ_CAL_FINE") == 0) {
			printk("set_freq_cal_fine\n");
			if (argc < 2) {
				printk("wrong param\n");
				break;
			}
			cmd_setfreq.val = command_strtoul(argv[1], NULL, 16);
			printk("param:%x\r\n", cmd_setfreq.val);
			rwnx_send_rftest_req(p_rwnx_hw, SET_FREQ_CAL_FINE, sizeof(cmd_rf_setfreq_t), (u8_l *)&cmd_setfreq, &cfm);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "GET_EFUSE_BLOCK") == 0) {
			printk("get_efuse_block\n");
			if (argc < 2) {
				printk("wrong param\n");
				break;
			}
			getefuse_param.block = command_strtoul(argv[1], NULL, 10);
			rwnx_send_rftest_req(p_rwnx_hw, GET_EFUSE_BLOCK, sizeof(cmd_rf_getefuse_t), (u8_l *)&getefuse_param, &cfm);
			printk("get val=%x\r\n", cfm.rftest_result[0]);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "SET_POWER") == 0) {
			printk("set_power\n");
			ana_pwr = command_strtoul(argv[1], NULL, 16);
			dig_pwr = command_strtoul(argv[2], NULL, 16);
			pwr = (ana_pwr << 4 | dig_pwr);
			if (ana_pwr > 0xf || dig_pwr > 0xf) {
				printk("invalid param\r\n");
				break;
			}
			printk("pwr =%x\r\n", pwr);
			rwnx_send_rftest_req(p_rwnx_hw, SET_POWER, sizeof(pwr), (u8_l *)&pwr, NULL);
		} else if (strcasecmp(argv[0], "SET_XTAL_CAP") == 0) {
			printk("set_xtal_cap\n");
			if (argc < 2) {
				printk("wrong param\n");
				break;
			}
			xtal_cap = command_strtoul(argv[1], NULL, 10);
			printk("xtal_cap =%x\r\n", xtal_cap);
			rwnx_send_rftest_req(p_rwnx_hw, SET_XTAL_CAP, sizeof(xtal_cap), (u8_l *)&xtal_cap, &cfm);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "SET_XTAL_CAP_FINE") == 0) {
			printk("set_xtal_cap_fine\n");
			if (argc < 2) {
				printk("wrong param\n");
				break;
			}
			xtal_cap_fine = command_strtoul(argv[1], NULL, 10);
			printk("xtal_cap_fine =%x\r\n", xtal_cap_fine);
			rwnx_send_rftest_req(p_rwnx_hw, SET_XTAL_CAP_FINE, sizeof(xtal_cap_fine), (u8_l *)&xtal_cap_fine, &cfm);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "SET_MAC_ADDR") == 0) {
			printk("set_mac_addr\n");
			if (argc < 7) {
				printk("wrong param\n");
				break;
			}
			mac_addr[5] = command_strtoul(argv[1], NULL, 16);
			mac_addr[4] = command_strtoul(argv[2], NULL, 16);
			mac_addr[3] = command_strtoul(argv[3], NULL, 16);
			mac_addr[2] = command_strtoul(argv[4], NULL, 16);
			mac_addr[1] = command_strtoul(argv[5], NULL, 16);
			mac_addr[0] = command_strtoul(argv[6], NULL, 16);
			printk("set macaddr:%x,%x,%x,%x,%x,%x\n", mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]);
			rwnx_send_rftest_req(p_rwnx_hw, SET_MAC_ADDR, sizeof(mac_addr), (u8_l *)&mac_addr, NULL);
		} else if (strcasecmp(argv[0], "GET_MAC_ADDR") == 0) {
			printk("get mac addr\n");
			rwnx_send_rftest_req(p_rwnx_hw, GET_MAC_ADDR, 0, NULL, &cfm);
			memcpy(command, &cfm.rftest_result[0], 8);
			bytes_written = 8;
			printk("0x%x,0x%x\n", cfm.rftest_result[0], cfm.rftest_result[1]);
		} else if (strcasecmp(argv[0], "SET_BT_MAC_ADDR") == 0) {
			printk("set_bt_mac_addr\n");
			if (argc < 7) {
				printk("wrong param\n");
				break;
			}
			mac_addr[5] = command_strtoul(argv[1], NULL, 16);
			mac_addr[4] = command_strtoul(argv[2], NULL, 16);
			mac_addr[3] = command_strtoul(argv[3], NULL, 16);
			mac_addr[2] = command_strtoul(argv[4], NULL, 16);
			mac_addr[1] = command_strtoul(argv[5], NULL, 16);
			mac_addr[0] = command_strtoul(argv[6], NULL, 16);
			printk("set bt macaddr:%x,%x,%x,%x,%x,%x\n", mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]);
			rwnx_send_rftest_req(p_rwnx_hw, SET_BT_MAC_ADDR, sizeof(mac_addr), (u8_l *)&mac_addr, NULL);
		} else if (strcasecmp(argv[0], "GET_BT_MAC_ADDR") == 0) {
			printk("get bt mac addr\n");
			rwnx_send_rftest_req(p_rwnx_hw, GET_BT_MAC_ADDR, 0, NULL, &cfm);
			memcpy(command, &cfm.rftest_result[0], 8);
			bytes_written = 8;
			printk("0x%x,0x%x\n", cfm.rftest_result[0], cfm.rftest_result[1]);
		} else if (strcasecmp(argv[0], "SET_VENDOR_INFO") == 0) {
			vendor_info = command_strtoul(argv[1], NULL, 16);
			printk("set vendor info:%x\n", vendor_info);
			rwnx_send_rftest_req(p_rwnx_hw, SET_VENDOR_INFO, 1, &vendor_info, &cfm);
			memcpy(command, &cfm.rftest_result[0], 1);
			bytes_written = 1;
			printk("0x%x\n", cfm.rftest_result[0]);
		} else if (strcasecmp(argv[0], "GET_VENDOR_INFO") == 0) {
			printk("get vendor info\n");
			rwnx_send_rftest_req(p_rwnx_hw, GET_VENDOR_INFO, 0, NULL, &cfm);
			memcpy(command, &cfm.rftest_result[0], 1);
			bytes_written = 1;
			printk("0x%x\n", cfm.rftest_result[0]);
		} else if (strcasecmp(argv[0], "GET_FREQ_CAL") == 0) {
			printk("get freq cal\n");
			rwnx_send_rftest_req(p_rwnx_hw, GET_FREQ_CAL, 0, NULL, &cfm);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
			printk("cap=0x%x, cap_fine=0x%x\n", cfm.rftest_result[0] & 0x0000ffff, (cfm.rftest_result[0] >> 16) & 0x0000ffff);
		} else if (strcasecmp(argv[0], "RDWR_PWRMM") == 0) {
			printk("read/write txpwr manul mode\n");
			if (argc <= 1) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWRMM, 0, NULL, &cfm);
			} else { // write
				u8_l pwrmm = (u8_l)command_strtoul(argv[1], NULL, 16);
				pwrmm = (pwrmm) ? 1 : 0;
				printk("set pwrmm = %x\r\n", pwrmm);
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWRMM, sizeof(pwrmm), (u8_l *)&pwrmm, &cfm);
			}
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "RDWR_PWRIDX") == 0) {
			u8_l func = 0;
			printk("read/write txpwr index\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWRIDX, 0, NULL, &cfm);
			} else if (func <= 2) { // write 2.4g/5g pwr idx
				if (argc > 3) {
					u8_l type = (u8_l)command_strtoul(argv[2], NULL, 16);
					u8_l pwridx = (u8_l)command_strtoul(argv[3], NULL, 10);
					u8_l buf[3] = {func, type, pwridx};
					printk("set pwridx:[%x][%x]=%x\r\n", func, type, pwridx);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWRIDX, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
				}
			} else {
				printk("wrong func: %x\n", func);
			}
			memcpy(command, &cfm.rftest_result[0], 7);
			bytes_written = 7;
		} else if (strcasecmp(argv[0], "RDWR_PWRLVL") == 0) {
			u8_l func = 0;
			printk("read/write txpwr level\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWRLVL, 0, NULL, &cfm);
			} else if (func <= 2) { // write 2.4g/5g pwr lvl
				if (argc > 4) {
					u8_l grp = (u8_l)command_strtoul(argv[2], NULL, 16);
					u8_l idx, size;
					u8_l buf[14] = {func, grp,};
					if (argc > 12) { // set all grp
						printk("set pwrlvl %s:\n"
						"  [%x] =", (func == 1) ? "2.4g" : "5g", grp);
						if (grp == 1) { // TXPWR_LVL_GRP_11N_11AC
							size = 10;
						} else {
							size = 12;
						}
						for (idx = 0; idx < size; idx++) {
							s8_l pwrlvl = (s8_l)command_strtoul(argv[3 + idx], NULL, 10);
							buf[2 + idx] = (u8_l)pwrlvl;
							if (idx && !(idx & 0x3)) {
								printk(" ");
							}
							printk(" %2d", pwrlvl);
						}
						printk("\n");
						size += 2;
					} else { // set grp[idx]
						u8_l idx = (u8_l)command_strtoul(argv[3], NULL, 10);
						s8_l pwrlvl = (s8_l)command_strtoul(argv[4], NULL, 10);
						buf[2] = idx;
						buf[3] = (u8_l)pwrlvl;
						size = 4;
						printk("set pwrlvl %s:\n"
						"  [%x][%d] = %d\n", (func == 1) ? "2.4g" : "5g", grp, idx, pwrlvl);
					}
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWRLVL, size, buf, &cfm);
				} else {
					printk("wrong args\n");
					bytes_written = -EINVAL;
					break;
				}
			} else {
				printk("wrong func: %x\n", func);
				bytes_written = -EINVAL;
				break;
			}
			if (p_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
				memcpy(command, &cfm.rftest_result[0], 6 * 12);
				bytes_written = 6 * 12;
			} else {
				memcpy(command, &cfm.rftest_result[0], 3 * 12);
				bytes_written = 3 * 12;
			}
		} else if (strcasecmp(argv[0], "RDWR_PWROFST") == 0) {
			u8_l func = 0;
			int res_len = 0;

			printk("read/write txpwr offset\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWROFST, 0, NULL, &cfm);
			} else if (func <= 2) { // write 2.4g/5g pwr ofst
				if ((argc > 4) && (p_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)) {
					u8_l type = (u8_l)command_strtoul(argv[2], NULL, 16);
					u8_l chgrp = (u8_l)command_strtoul(argv[3], NULL, 16);
					s8_l pwrofst = (u8_l)command_strtoul(argv[4], NULL, 10);
					#ifdef AICWF_SDIO_SUPPORT
					u8_l buf[4] = {func, type, chgrp, (u8_l)pwrofst};
					#endif
					printk("set pwrofst_%s:[%x][%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", type, chgrp, pwrofst);
					#ifdef AICWF_SDIO_SUPPORT
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWROFST, sizeof(buf), buf, &cfm);
					#endif
				} else if ((argc > 3) && (p_rwnx_hw->chipid != PRODUCT_ID_AIC8800D80)) {
					u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
					s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
					u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};
					printk("set pwrofst_%s:[%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", chgrp, pwrofst);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWROFST, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
				}
			} else {
				printk("wrong func: %x\n", func);
			}
			if ((p_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
				(p_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)) { // 3 = 3 (2.4g)
				res_len = 3;
			} else if (p_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) { // 3 * 2 (2.4g) + 3 * 6 (5g)
				res_len = 3 * 3 + 3 * 6;
			} else {
				res_len = 3 + 4;
			}
			memcpy(command, &cfm.rftest_result[0], res_len);
			bytes_written = res_len;
		} else if (strcasecmp(argv[0], "RDWR_DRVIBIT") == 0) {
			u8_l func = 0;
			printk("read/write pa drv_ibit\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_DRVIBIT, 0, NULL, &cfm);
			} else if (func == 1) { // write 2.4g pa drv_ibit
				if (argc > 2) {
					u8_l ibit = (u8_l)command_strtoul(argv[2], NULL, 16);
					u8_l buf[2] = {func, ibit};
					printk("set drvibit:[%x]=%x\r\n", func, ibit);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_DRVIBIT, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
				}
			} else {
				printk("wrong func: %x\n", func);
			}
			memcpy(command, &cfm.rftest_result[0], 16);
			bytes_written = 16;
		} else if (strcasecmp(argv[0], "RDWR_EFUSE_PWROFST") == 0) {
			u8_l func = 0;
			int res_len = 0;

			printk("read/write txpwr offset into efuse\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_PWROFST, 0, NULL, &cfm);
			} else if (func <= 2) { // write 2.4g/5g pwr ofst
				if ((argc > 4) && (p_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)) {
					u8_l type = (u8_l)command_strtoul(argv[2], NULL, 16);
					u8_l chgrp = (u8_l)command_strtoul(argv[3], NULL, 16);
					s8_l pwrofst = (u8_l)command_strtoul(argv[4], NULL, 10);
					#ifdef AICWF_SDIO_SUPPORT
					u8_l buf[4] = {func, type, chgrp, (u8_l)pwrofst};
					#endif
					printk("set efuse pwrofst_%s:[%x][%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", type, chgrp, pwrofst);
					#ifdef AICWF_SDIO_SUPPORT
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_PWROFST, sizeof(buf), buf, &cfm);
					#endif
				} else if ((argc > 3) && (p_rwnx_hw->chipid != PRODUCT_ID_AIC8800D80)) {
					u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
					s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
					u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};
					printk("set efuse pwrofst_%s:[%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", chgrp, pwrofst);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_PWROFST, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
				}
			} else {
				printk("wrong func: %x\n", func);
			}
			if ((p_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
				(p_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)) { // 6 = 3 (2.4g) * 2
				res_len = 3 * 2;
			} else if (p_rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) { // 3 * 2 (2.4g) + 3 * 6 (5g)
				res_len = 3 * 3 + 3 * 6;
			} else { // 7 = 3(2.4g) + 4(5g)
				res_len = 3 + 4;
			}
			memcpy(command, &cfm.rftest_result[0], res_len);
			bytes_written = res_len;
		} else if (strcasecmp(argv[0], "RDWR_PWROFSTFINE") == 0) {
			u8_l func = 0;

			printk("read/write txpwr offset fine\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWROFSTFINE, 0, NULL, &cfm);
			} else if (func <= 2) { // write 2.4g/5g pwr ofst
				if (argc > 3) {
					u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
					s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
					u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};

					printk("set pwrofstfine:[%x][%x]=%d\r\n", func, chgrp, pwrofst);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_PWROFSTFINE, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
					bytes_written = -EINVAL;
					break;
				}
			} else {
				printk("wrong func: %x\n", func);
				bytes_written = -EINVAL;
				break;
			}
			memcpy(command, &cfm.rftest_result[0], 7);
			bytes_written = 7;
		} else if (strcasecmp(argv[0], "RDWR_EFUSE_DRVIBIT") == 0) {
			u8_l func = 0;
			printk("read/write pa drv_ibit into efuse\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_DRVIBIT, 0, NULL, &cfm);
			} else if (func == 1) { // write 2.4g pa drv_ibit
				if (argc > 2) {
				u8_l ibit = (u8_l)command_strtoul(argv[2], NULL, 16);
				u8_l buf[2] = {func, ibit};
				printk("set efuse drvibit:[%x]=%x\r\n", func, ibit);
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_DRVIBIT, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
				}
			} else {
				printk("wrong func: %x\n", func);
			}
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "RDWR_EFUSE_PWROFSTFINE") == 0) {
			u8_l func = 0;
			printk("read/write txpwr offset fine into efuse\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_PWROFSTFINE, 0, NULL, &cfm);
			} else if (func <= 2) { // write 2.4g/5g pwr ofst
				if (argc > 3) {
					u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
					s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
					u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};

					printk("set efuse pwrofstfine:[%x][%x]=%d\r\n", func, chgrp, pwrofst);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_PWROFSTFINE, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
					bytes_written = -EINVAL;
					break;
				}
			} else {
				printk("wrong func: %x\n", func);
				bytes_written = -EINVAL;
				break;
			}
			if ((p_rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
				(p_rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)) { // 6 = 3 (2.4g) * 2
				memcpy(command, &cfm.rftest_result[0], 6);
				bytes_written = 6;
			} else { // 7 = 3(2.4g) + 4(5g)
				memcpy(command, &cfm.rftest_result[0], 7);
				bytes_written = 7;
			}
		} else if (strcasecmp(argv[0], "RDWR_EFUSE_SDIOCFG") == 0) {
			u8_l func = 0;

			printk("read/write sdiocfg_bit into efuse\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_SDIOCFG, 0, NULL, &cfm);
			} else if (func == 1) { // write sdiocfg
				if (argc > 2) {
					u8_l ibit = (u8_l)command_strtoul(argv[2], NULL, 16);
					u8_l buf[2] = {func, ibit};
					printk("set efuse sdiocfg:[%x]=%x\r\n", func, ibit);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_SDIOCFG, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
					bytes_written = -EINVAL;
					break;
				}
			} else {
				printk("wrong func: %x\n", func);
				bytes_written = -EINVAL;
				break;
			}
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
		} else if (strcasecmp(argv[0], "RDWR_EFUSE_USBVIDPID") == 0) {
			u8_l func = 0;
			printk("read/write usb vid/pid into efuse\n");
			if (argc > 1) {
				func = (u8_l)command_strtoul(argv[1], NULL, 16);
			}
			if (func == 0) { // read cur
				rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_USBVIDPID, 0, NULL, &cfm);
			} else if (func == 1) { // write USB vid+pid
				if (argc > 2) {
					u32_l usb_id = (u32_l)command_strtoul(argv[2], NULL, 16);
					u8_l buf[5] = {func, (u8_l)usb_id, (u8_l)(usb_id >> 8), (u8_l)(usb_id >> 16), (u8_l)(usb_id >> 24)};
					printk("set efuse usb vid/pid:[%x]=%x\r\n", func, usb_id);
					rwnx_send_rftest_req(p_rwnx_hw, RDWR_EFUSE_USBVIDPID, sizeof(buf), buf, &cfm);
				} else {
					printk("wrong args\n");
					bytes_written = -EINVAL;
					break;
				}
			} else {
				printk("wrong func: %x\n", func);
				bytes_written = -EINVAL;
				break;
			}
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
	#ifdef CONFIG_USB_BT
		} else if (strcasecmp(argv[0], "BT_RESET") == 0) {
			if (argc == 5) {
				printk("btrf reset\n");
				for (bt_index = 0; bt_index < 4; bt_index++) {
					dh_cmd_reset[bt_index] = command_strtoul(argv[bt_index+1], NULL, 16);
					printk("0x%x ", dh_cmd_reset[bt_index]);
				}
				printk("\n");
			} else {
				printk("wrong param\n");
				break;
			}
			rwnx_send_rftest_req(p_rwnx_hw, BT_RESET, sizeof(dh_cmd_reset), (u8_l *)&dh_cmd_reset, NULL);
		} else if (strcasecmp(argv[0], "BT_TXDH") == 0) {
			if (argc == 19) {
				printk("btrf txdh\n");
				for (bt_index = 0; bt_index < 18; bt_index++) {
					dh_cmd_txdh[bt_index] = command_strtoul(argv[bt_index+1], NULL, 16);
					printk("0x%x ", dh_cmd_txdh[bt_index]);
				}
				printk("\n");
			} else {
				printk("wrong param\n");
				break;
			}
			rwnx_send_rftest_req(p_rwnx_hw, BT_TXDH, sizeof(dh_cmd_txdh), (u8_l *)&dh_cmd_txdh, NULL);
		} else if (strcasecmp(argv[0], "BT_RXDH") == 0) {
			if (argc == 18) {
				printk("btrf rxdh\n");
				for (bt_index = 0; bt_index < 17; bt_index++) {
					dh_cmd_rxdh[bt_index] = command_strtoul(argv[bt_index+1], NULL, 16);
					printk("0x%x ", dh_cmd_rxdh[bt_index]);
				}
				printk("\n");
			} else {
				printk("wrong param\n");
				break;
			}
			rwnx_send_rftest_req(p_rwnx_hw, BT_RXDH, sizeof(dh_cmd_rxdh), (u8_l *)&dh_cmd_rxdh, NULL);
		} else if (strcasecmp(argv[0], "BT_STOP") == 0) {
			if (argc == 6) {
				printk("btrf stop\n");
				for (bt_index = 0; bt_index < 5; bt_index++) {
					dh_cmd_stop[bt_index] = command_strtoul(argv[bt_index+1], NULL, 16);
					printk("0x%x ", dh_cmd_stop[bt_index]);
				}
				printk("\n");
			} else {
				printk("wrong param\n");
				break;
			}
			rwnx_send_rftest_req(p_rwnx_hw, BT_STOP, sizeof(dh_cmd_stop), (u8_l *)&dh_cmd_stop, NULL);
	#endif
	#ifdef AICWF_SDIO_SUPPORT
		} else if (strcasecmp(argv[0], "SET_PAPR") == 0) {
			printk("set papr\n");
			if (argc > 1) {
				u8_l func = (u8_l) command_strtoul(argv[1], NULL, 10);
				printk("papr %d\r\n", func);
				rwnx_send_rftest_req(g_rwnx_plat->sdiodev->rwnx_hw, SET_PAPR, sizeof(func), &func, NULL);
			} else {
				printk("wrong args\n");
			}
		} else if (strcasecmp(argv[0], "SETSUSPENDMODE") == 0 && p_rwnx_hw->cpmode != AICBSP_CPMODE_TEST) {
			u8_l setsusp_mode = command_strtoul(argv[1], NULL, 10);
			rwnx_send_me_set_lp_level(g_rwnx_plat->sdiodev->rwnx_hw, setsusp_mode);
			if (setsusp_mode == 1) {
				#if defined(CONFIG_SDIO_PWRCTRL)
				aicwf_sdio_pwr_stctl(g_rwnx_plat->sdiodev, SDIO_SLEEP_ST);
				#endif
				if (aicwf_sdio_writeb(g_rwnx_plat->sdiodev, g_rwnx_plat->sdiodev->sdio_reg.wakeup_reg, 2)) {
					sdio_err("reg:%d write failed!\n", g_rwnx_plat->sdiodev->sdio_reg.wakeup_reg);
				}
			}
			printk("set suspend mode %d\n", setsusp_mode);
		} else if (strcasecmp(argv[0], "SET_NOTCH") == 0) {
			if (argc > 1) {
				u8_l func = (u8_l) command_strtoul(argv[1], NULL, 10);
				printk("set notch %d\r\n", func);
				rwnx_send_rftest_req(g_rwnx_plat->sdiodev->rwnx_hw, SET_NOTCH, sizeof(func), &func, NULL);
			} else {
				printk("wrong args\n");
				bytes_written = -EINVAL;
				break;
			}
		} else if (strcasecmp(argv[0], "SET_COB_CAL") == 0) {
			printk("set_cob_cal\n");
			if (argc < 3) {
				printk("wrong param\n");
				bytes_written = -EINVAL;
				break;
			}
			setcob_cal.dutid = command_strtoul(argv[1], NULL, 10);
			setcob_cal.chip_num = command_strtoul(argv[2], NULL, 10);
			setcob_cal.dis_xtal = command_strtoul(argv[3], NULL, 10);
			rwnx_send_rftest_req(g_rwnx_plat->sdiodev->rwnx_hw, SET_COB_CAL, sizeof(cmd_rf_setcobcal_t), (u8_l *)&setcob_cal, NULL);
		} else if (strcasecmp(argv[0], "GET_COB_CAL_RES") == 0) {
			printk("get cob cal res\n");
			rwnx_send_rftest_req(g_rwnx_plat->sdiodev->rwnx_hw, GET_COB_CAL_RES, 0, NULL, &cfm);
			memcpy(command, &cfm.rftest_result[0], 4);
			bytes_written = 4;
			printk("cap=0x%x, cap_fine=0x%x\n", cfm.rftest_result[0] & 0x0000ffff, (cfm.rftest_result[0] >> 16) & 0x0000ffff);
		} else if (strcasecmp(argv[0], "DO_COB_TEST") == 0) {
			printk("do_cob_test\n");
			setcob_cal.dutid = 1;
			setcob_cal.chip_num = 1;
			setcob_cal.dis_xtal = 0;
			if (argc > 1) {
				setcob_cal.dis_xtal = command_strtoul(argv[1], NULL, 10);
			}
			rwnx_send_rftest_req(g_rwnx_plat->sdiodev->rwnx_hw, SET_COB_CAL, sizeof(cmd_rf_setcobcal_t), (u8_l *)&setcob_cal, NULL);
			msleep(2000);
			rwnx_send_rftest_req(g_rwnx_plat->sdiodev->rwnx_hw, GET_COB_CAL_RES, 0, NULL, &cfm);
			state = (cfm.rftest_result[0] >> 16) & 0x000000ff;
			if (!state) {
				printk("cap= 0x%x, cap_fine= 0x%x, freq_ofst= %d Hz\n",
				cfm.rftest_result[0] & 0x000000ff, (cfm.rftest_result[0] >> 8) & 0x000000ff, cfm.rftest_result[1]);
				cob_result_ptr = (cob_result_ptr_t *)&(cfm.rftest_result[2]);
				printk("golden_rcv_dut= %d , tx_rssi= %d dBm, snr = %d dB\ndut_rcv_godlden= %d , rx_rssi= %d dBm",
				cob_result_ptr->golden_rcv_dut_num, cob_result_ptr->rssi_static, cob_result_ptr->snr_static,
				cob_result_ptr->dut_rcv_golden_num, cob_result_ptr->dut_rssi_static);
				memcpy(command, &cfm.rftest_result, 16);
				bytes_written = 16;
			} else {
				printk("cob not idle\n");
				bytes_written = -EINVAL;
				break;
			}
	#endif
		} else {
			printk("wrong cmd:%s in %s\n", cmd, __func__);
		}
#endif
	} while (0);
	kfree(cmd);
	kfree(para);
	return bytes_written;
}


int android_priv_cmd(struct net_device *net, struct ifreq *ifr, int cmd)
{
#define PRIVATE_COMMAND_MAX_LEN 8192
#define PRIVATE_COMMAND_DEF_LEN 4096
	int ret = 0;
	char *command = NULL;
	int bytes_written = 0;
	android_wifi_priv_cmd priv_cmd;
	int buf_size = 0;

	///todo: add our lock
	//net_os_wake_lock(net);


/*	if (!capable(CAP_NET_ADMIN)) {
		ret = -EPERM;
		goto exit;
	}*/
	if (!ifr->ifr_data) {
		ret = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_COMPAT
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
	if (in_compat_syscall())
#else
	if (is_compat_task())
#endif
	{
		compat_android_wifi_priv_cmd compat_priv_cmd;
		if (copy_from_user(&compat_priv_cmd, ifr->ifr_data, sizeof(compat_android_wifi_priv_cmd))) {
		ret = -EFAULT;
			goto exit;
		}
		priv_cmd.buf = compat_ptr(compat_priv_cmd.buf);
		priv_cmd.used_len = compat_priv_cmd.used_len;
		priv_cmd.total_len = compat_priv_cmd.total_len;
	} else
#endif /* CONFIG_COMPAT */
	{
		if (copy_from_user(&priv_cmd, ifr->ifr_data, sizeof(android_wifi_priv_cmd))) {
		ret = -EFAULT;
			goto exit;
		}
	}
	if ((priv_cmd.total_len > PRIVATE_COMMAND_MAX_LEN) || (priv_cmd.total_len < 0)) {
		printk("%s: buf length invalid:%d\n", __FUNCTION__, priv_cmd.total_len);
		ret = -EINVAL;
		goto exit;
	}

	buf_size = max(priv_cmd.total_len, PRIVATE_COMMAND_DEF_LEN);
	command = kmalloc((buf_size + 1), GFP_KERNEL);

	if (!command) {
		printk("%s: failed to allocate memory\n", __FUNCTION__);
		ret = -ENOMEM;
		goto exit;
	}
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		goto exit;
	}
	command[priv_cmd.total_len] = '\0';

	/* outputs */
	printk("%s: Android private cmd \"%s\" on %s\n", __FUNCTION__, command, ifr->ifr_name);

	bytes_written = handle_private_cmd(net, command, priv_cmd.total_len);
	if (bytes_written >= 0) {
		if ((bytes_written == 0) && (priv_cmd.total_len > 0)) {
			command[0] = '\0';
		}
		if (bytes_written >= priv_cmd.total_len) {
			printk("%s: err. bytes_written:%d >= buf_size:%d \n",
				__FUNCTION__, bytes_written, buf_size);
			goto exit;
		}
		bytes_written++;
		priv_cmd.used_len = bytes_written;
		if (copy_to_user(priv_cmd.buf, command, bytes_written)) {
			printk("%s: failed to copy data to user buffer\n", __FUNCTION__);
			ret = -EFAULT;
		}
	} else {
		/* Propagate the error */
		ret = bytes_written;
	}

exit:
	///todo: add our unlock
	//net_os_wake_unlock(net);
	kfree(command);
	return ret;
}

#define IOCTL_HOSTAPD   (SIOCIWFIRSTPRIV+28)
#define IOCTL_WPAS      (SIOCIWFIRSTPRIV+30)

#if LINUX_VERSION_CODE <  KERNEL_VERSION(5, 15, 0)
static int rwnx_do_ioctl(struct net_device *net, struct ifreq *req, int cmd)
#else
static int rwnx_do_ioctl(struct net_device *net, struct ifreq *req, void __user *data, int cmd)
#endif
{
	int ret = 0;
	///TODO: add ioctl command handler later
	switch (cmd) {
	case IOCTL_HOSTAPD:
		printk("IOCTL_HOSTAPD\n");
		break;
	case IOCTL_WPAS:
		printk("IOCTL_WPAS\n");
		break;
	case SIOCDEVPRIVATE:
		printk("IOCTL SIOCDEVPRIVATE\n");
		break;
	case (SIOCDEVPRIVATE+1):
		ret = android_priv_cmd(net, req, cmd);
		break;
	default:
		ret = -EOPNOTSUPP;
	}
	return ret;
}

/**
 * struct net_device_stats* (*ndo_get_stats)(struct net_device *dev);
 *	Called when a user wants to get the network device usage
 *	statistics. Drivers must do one of the following:
 *	1. Define @ndo_get_stats64 to fill in a zero-initialised
 *	   rtnl_link_stats64 structure passed by the caller.
 *	2. Define @ndo_get_stats to update a net_device_stats structure
 *	   (which should normally be dev->stats) and return a pointer to
 *	   it. The structure may be changed asynchronously only if each
 *	   field is written atomically.
 *	3. Update dev->stats asynchronously and atomically, and define
 *	   neither operation.
 */
static struct net_device_stats *rwnx_get_stats(struct net_device *dev)
{
	struct rwnx_vif *vif = netdev_priv(dev);

	return &vif->net_stats;
}

/**
 * u16 (*ndo_select_queue)(struct net_device *dev, struct sk_buff *skb,
 *                         struct net_device *sb_dev);
 *	Called to decide which queue to when device supports multiple
 *	transmit queues.
 */
u16 rwnx_select_queue(struct net_device *dev, struct sk_buff *skb,
					  struct net_device *sb_dev)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	return rwnx_select_txq(rwnx_vif, skb);
}

/**
 * int (*ndo_set_mac_address)(struct net_device *dev, void *addr);
 *	This function  is called when the Media Access Control address
 *	needs to be changed. If this interface is not defined, the
 *	mac address can not be changed.
 */
static int rwnx_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sa = addr;
	int ret;
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);

	ret = eth_mac_addr(dev, sa);
	memcpy(rwnx_vif->wdev.address, dev->dev_addr, 6);

	return ret;
}

static const struct net_device_ops rwnx_netdev_ops = {
	.ndo_open               = rwnx_open,
	.ndo_stop               = rwnx_close,
#if LINUX_VERSION_CODE <  KERNEL_VERSION(5, 15, 0)
	.ndo_do_ioctl           = rwnx_do_ioctl,
#else
	.ndo_siocdevprivate     = rwnx_do_ioctl,
#endif
	.ndo_start_xmit         = rwnx_start_xmit,
	.ndo_get_stats          = rwnx_get_stats,
	.ndo_select_queue       = rwnx_select_queue,
	.ndo_set_mac_address    = rwnx_set_mac_address
//    .ndo_set_features       = rwnx_set_features,
//    .ndo_set_rx_mode        = rwnx_set_multicast_list,
};

static const struct net_device_ops rwnx_netdev_monitor_ops = {
	.ndo_open               = rwnx_open,
	.ndo_stop               = rwnx_close,
	.ndo_get_stats          = rwnx_get_stats,
	.ndo_set_mac_address    = rwnx_set_mac_address,
};

static void rwnx_netdev_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->netdev_ops = &rwnx_netdev_ops;
#if LINUX_VERSION_CODE <  KERNEL_VERSION(4, 12, 0)
	dev->destructor = free_netdev;
#else
	dev->needs_free_netdev = true;
#endif
	dev->watchdog_timeo = RWNX_TX_LIFETIME_MS;

	dev->needed_headroom = sizeof(struct rwnx_txhdr) + RWNX_SWTXHDR_ALIGN_SZ;
#ifdef CONFIG_RWNX_AMSDUS_TX
	dev->needed_headroom = max(dev->needed_headroom,
							   (unsigned short)(sizeof(struct rwnx_amsdu_txhdr)
												+ sizeof(struct ethhdr) + 4
												+ sizeof(rfc1042_header) + 2));
#endif /* CONFIG_RWNX_AMSDUS_TX */

	dev->hw_features = 0;
}

/*********************************************************************
 * Cfg80211 callbacks (and helper)
 *********************************************************************/
static struct rwnx_vif *rwnx_interface_add(struct rwnx_hw *rwnx_hw,
											   const char *name,
											   unsigned char name_assign_type,
											   enum nl80211_iftype type,
											   struct vif_params *params)
{
	struct net_device *ndev;
	struct rwnx_vif *vif;
	int min_idx, max_idx;
	int vif_idx = -1;
	int i;
	int nx_nb_ndev_txq = NX_NB_NDEV_TXQ;

#if defined(AICWF_SDIO_SUPPORT)
	if ((g_rwnx_plat->sdiodev->rwnx_hw->chipid == PRODUCT_ID_AIC8800D) ||
		((g_rwnx_plat->sdiodev->rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
		g_rwnx_plat->sdiodev->rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) && (g_rwnx_plat->sdiodev->rwnx_hw->rev < CHIP_REV_ID_U02))) {
#elif defined(AICWF_USB_SUPPORT)
	if ((g_rwnx_plat->usbdev->rwnx_hw->chipid == PRODUCT_ID_AIC8800D) ||
		((g_rwnx_plat->usbdev->rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
		g_rwnx_plat->usbdev->rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) && (g_rwnx_plat->usbdev->rwnx_hw->rev < CHIP_REV_ID_U02))) {
#endif
		nx_nb_ndev_txq = NX_NB_NDEV_TXQ_FOR_OLD_IC;
	}
	printk("rwnx_interface_add: %s, %d, %d", name, type, NL80211_IFTYPE_P2P_DEVICE);
	// Look for an available VIF
	if (type == NL80211_IFTYPE_AP_VLAN) {
		min_idx = NX_VIRT_DEV_MAX;
		max_idx = NX_VIRT_DEV_MAX + NX_REMOTE_STA_MAX;
	} else {
		min_idx = 0;
		max_idx = NX_VIRT_DEV_MAX;
	}

	for (i = min_idx; i < max_idx; i++) {
		if ((rwnx_hw->avail_idx_map) & BIT(i)) {
			vif_idx = i;
			break;
		}
	}
	if (vif_idx < 0)
		return NULL;

	#ifndef CONFIG_RWNX_MON_DATA
	list_for_each_entry(vif, &rwnx_hw->vifs, list) {
		// Check if monitor interface already exists or type is monitor
		if ((RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_MONITOR) ||
		   (type == NL80211_IFTYPE_MONITOR)) {
			wiphy_err(rwnx_hw->wiphy,
					"Monitor+Data interface support (MON_DATA) disabled\n");
			return NULL;
		}
	}
	#endif

	ndev = alloc_netdev_mqs(sizeof(*vif), name, name_assign_type,
							rwnx_netdev_setup, nx_nb_ndev_txq, 1);
	if (!ndev)
		return NULL;

	vif = netdev_priv(ndev);
	vif->key_has_add = 0;
	ndev->ieee80211_ptr = &vif->wdev;
	vif->wdev.wiphy = rwnx_hw->wiphy;
	vif->rwnx_hw = rwnx_hw;
	vif->ndev = ndev;
	vif->drv_vif_index = vif_idx;
	SET_NETDEV_DEV(ndev, wiphy_dev(vif->wdev.wiphy));
	vif->wdev.netdev = ndev;
	vif->wdev.iftype = type;
	vif->up = false;
	vif->ch_index = RWNX_CH_NOT_SET;
	memset(&vif->net_stats, 0, sizeof(vif->net_stats));
	vif->is_p2p_vif = 0;

	switch (type) {
	case NL80211_IFTYPE_STATION:
		vif->sta.ap = NULL;
		vif->sta.tdls_sta = NULL;
		vif->sta.external_auth = false;
		break;
	case NL80211_IFTYPE_P2P_CLIENT:
		vif->sta.ap = NULL;
		vif->sta.tdls_sta = NULL;
		vif->sta.external_auth = false;
		vif->is_p2p_vif = 1;
		break;
	case NL80211_IFTYPE_MESH_POINT:
		INIT_LIST_HEAD(&vif->ap.mpath_list);
		INIT_LIST_HEAD(&vif->ap.proxy_list);
		vif->ap.create_path = false;
		vif->ap.generation = 0;
		vif->ap.mesh_pm = NL80211_MESH_POWER_ACTIVE;
		vif->ap.next_mesh_pm = NL80211_MESH_POWER_ACTIVE;
		// no break
	case NL80211_IFTYPE_AP:
		INIT_LIST_HEAD(&vif->ap.sta_list);
		memset(&vif->ap.bcn, 0, sizeof(vif->ap.bcn));
		break;
	case NL80211_IFTYPE_P2P_GO:
		INIT_LIST_HEAD(&vif->ap.sta_list);
		memset(&vif->ap.bcn, 0, sizeof(vif->ap.bcn));
		vif->is_p2p_vif = 1;
		break;
	case NL80211_IFTYPE_AP_VLAN:
	{
		struct rwnx_vif *master_vif;
		bool found = false;
		list_for_each_entry(master_vif, &rwnx_hw->vifs, list) {
			if ((RWNX_VIF_TYPE(master_vif) == NL80211_IFTYPE_AP) &&
				!(!memcmp(master_vif->ndev->dev_addr, params->macaddr,
						   ETH_ALEN))) {
				 found = true;
				 break;
			}
		}

		if (!found)
			goto err;

		 vif->ap_vlan.master = master_vif;
		 vif->ap_vlan.sta_4a = NULL;
		 break;
	}
	case NL80211_IFTYPE_MONITOR:
		ndev->type = ARPHRD_IEEE80211_RADIOTAP;
		ndev->netdev_ops = &rwnx_netdev_monitor_ops;
		break;
	default:
		break;
	}

	if (type == NL80211_IFTYPE_AP_VLAN) {
		memcpy(ndev->dev_addr, params->macaddr, ETH_ALEN);
		memcpy(vif->wdev.address, params->macaddr, ETH_ALEN);
	} else {
		memcpy(ndev->dev_addr, rwnx_hw->wiphy->perm_addr, ETH_ALEN);
		ndev->dev_addr[5] ^= vif_idx;
		memcpy(vif->wdev.address, ndev->dev_addr, ETH_ALEN);
	}

	printk("interface add:%x %x %x %x %x %x\n", vif->wdev.address[0], vif->wdev.address[1], \
		vif->wdev.address[2], vif->wdev.address[3], vif->wdev.address[4], vif->wdev.address[5]);

	if (params) {
		vif->use_4addr = params->use_4addr;
		ndev->ieee80211_ptr->use_4addr = params->use_4addr;
	} else
		vif->use_4addr = false;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
	if (cfg80211_register_netdevice(ndev))
#else
	if (register_netdevice(ndev))
#endif
		goto err;

	spin_lock_bh(&rwnx_hw->cb_lock);
	list_add_tail(&vif->list, &rwnx_hw->vifs);
	spin_unlock_bh(&rwnx_hw->cb_lock);
	rwnx_hw->avail_idx_map &= ~BIT(vif_idx);

	return vif;

err:
	free_netdev(ndev);
	return NULL;
}

#define P2P_ALIVE_TIME_MS       (1*1000)
#define P2P_ALIVE_TIME_COUNT    200

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
void aicwf_p2p_alive_timeout(ulong data)
#else
void aicwf_p2p_alive_timeout(struct timer_list *t)
#endif
{
	struct rwnx_hw *rwnx_hw;
	struct rwnx_vif *rwnx_vif;
	struct rwnx_vif *rwnx_vif1, *tmp;
	u8_l p2p = 0;
	#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	rwnx_vif = (struct rwnx_vif *)data;
	rwnx_hw = rwnx_vif->rwnx_hw;
	#else
	rwnx_hw = from_timer(rwnx_hw, t, p2p_alive_timer);
	rwnx_vif = rwnx_hw->p2p_dev_vif;
	#endif

	list_for_each_entry_safe(rwnx_vif1, tmp, &rwnx_hw->vifs, list) {
		if ((rwnx_hw->avail_idx_map & BIT(rwnx_vif1->drv_vif_index)) == 0) {
			switch (RWNX_VIF_TYPE(rwnx_vif1)) {
			case NL80211_IFTYPE_P2P_CLIENT:
			case NL80211_IFTYPE_P2P_GO:
				rwnx_hw->is_p2p_alive = 1;
				p2p = 1;
				break;
			default:
				break;
			}
	   }
	}

	if (p2p)
		atomic_set(&rwnx_hw->p2p_alive_timer_count, 0);
	else
		atomic_inc(&rwnx_hw->p2p_alive_timer_count);

	if (atomic_read(&rwnx_hw->p2p_alive_timer_count) < P2P_ALIVE_TIME_COUNT) {
		mod_timer(&rwnx_hw->p2p_alive_timer,
			jiffies + msecs_to_jiffies(P2P_ALIVE_TIME_MS));
		return;
	} else
		atomic_set(&rwnx_hw->p2p_alive_timer_count, 0);

	rwnx_hw->is_p2p_alive = 0;
	if (rwnx_vif->up) {
		rwnx_send_remove_if (rwnx_hw, rwnx_vif->vif_index, true);

		/* Ensure that we won't process disconnect ind */
		spin_lock_bh(&rwnx_hw->cb_lock);

		rwnx_vif->up = false;
		rwnx_hw->vif_table[rwnx_vif->vif_index] = NULL;
		rwnx_hw->vif_started--;
		spin_unlock_bh(&rwnx_hw->cb_lock);
	}
}


/*********************************************************************
 * Cfg80211 callbacks (and helper)
 *********************************************************************/
static struct wireless_dev *rwnx_virtual_interface_add(struct rwnx_hw *rwnx_hw,
											   const char *name,
											   unsigned char name_assign_type,
											   enum nl80211_iftype type,
											   struct vif_params *params)
{
	struct wireless_dev *wdev = NULL;
	struct rwnx_vif *vif;
	int min_idx, max_idx;
	int vif_idx = -1;
	int i;

	printk("rwnx_virtual_interface_add: %d, %s\n", type, name);

	if (type == NL80211_IFTYPE_AP_VLAN) {
		min_idx = NX_VIRT_DEV_MAX;
		max_idx = NX_VIRT_DEV_MAX + NX_REMOTE_STA_MAX;
	} else {
		min_idx = 0;
		max_idx = NX_VIRT_DEV_MAX;
	}

	for (i = min_idx; i < max_idx; i++) {
		if ((rwnx_hw->avail_idx_map) & BIT(i)) {
			vif_idx = i;
			break;
		}
	}

	if (vif_idx < 0) {
		printk("virtual_interface_add %s fail\n", name);
		return NULL;
	}

	vif = kzalloc(sizeof(struct rwnx_vif), GFP_KERNEL);
	if (unlikely(!vif)) {
		printk("Could not allocate wireless device\n");
		return NULL;
	}
	wdev = &vif->wdev;
	wdev->wiphy = rwnx_hw->wiphy;
	wdev->iftype = type;

	printk("rwnx_virtual_interface_add, ifname=%s, wdev=%p, vif_idx=%d\n", name, wdev, vif_idx);

	vif->is_p2p_vif = 1;
	vif->rwnx_hw = rwnx_hw;
	vif->vif_index = vif_idx;
	vif->wdev.wiphy = rwnx_hw->wiphy;
	vif->drv_vif_index = vif_idx;
	vif->up = false;
	vif->ch_index = RWNX_CH_NOT_SET;
	memset(&vif->net_stats, 0, sizeof(vif->net_stats));
	vif->use_4addr = false;

	spin_lock_bh(&rwnx_hw->cb_lock);
	list_add_tail(&vif->list, &rwnx_hw->vifs);
	spin_unlock_bh(&rwnx_hw->cb_lock);

	if (rwnx_hw->is_p2p_alive == 0) {
		#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
		init_timer(&rwnx_hw->p2p_alive_timer);
		rwnx_hw->p2p_alive_timer.data = (unsigned long)vif;
		rwnx_hw->p2p_alive_timer.function = aicwf_p2p_alive_timeout;
		#else
		timer_setup(&rwnx_hw->p2p_alive_timer, aicwf_p2p_alive_timeout, 0);
		#endif
		rwnx_hw->is_p2p_alive = 0;
		rwnx_hw->is_p2p_connected = 0;
		rwnx_hw->p2p_dev_vif = vif;
		atomic_set(&rwnx_hw->p2p_alive_timer_count, 0);
	}
	rwnx_hw->avail_idx_map &= ~BIT(vif_idx);

	memcpy(vif->wdev.address, rwnx_hw->wiphy->perm_addr, ETH_ALEN);
	vif->wdev.address[5] ^= vif_idx;
	printk("p2p dev addr=%x %x %x %x %x %x\n", vif->wdev.address[0], vif->wdev.address[1], \
		vif->wdev.address[2], vif->wdev.address[3], vif->wdev.address[4], vif->wdev.address[5]);

	return wdev;
}

/*
 * @brief Retrieve the rwnx_sta object allocated for a given MAC address
 * and a given role.
 */
static struct rwnx_sta *rwnx_retrieve_sta(struct rwnx_hw *rwnx_hw,
										  struct rwnx_vif *rwnx_vif, u8 *addr,
										  __le16 fc, bool ap)
{
	if (ap) {
		/* only deauth, disassoc and action are bufferable MMPDUs */
		bool bufferable = ieee80211_is_deauth(fc) ||
						  ieee80211_is_disassoc(fc) ||
						  ieee80211_is_action(fc);

		/* Check if the packet is bufferable or not */
		if (bufferable) {
			/* Check if address is a broadcast or a multicast address */
			if (is_broadcast_ether_addr(addr) || is_multicast_ether_addr(addr)) {
				/* Returned STA pointer */
				struct rwnx_sta *rwnx_sta = &rwnx_hw->sta_table[rwnx_vif->ap.bcmc_index];

				if (rwnx_sta->valid)
					return rwnx_sta;
			} else {
				/* Returned STA pointer */
				struct rwnx_sta *rwnx_sta;

				/* Go through list of STAs linked with the provided VIF */
				list_for_each_entry(rwnx_sta, &rwnx_vif->ap.sta_list, list) {
					if (rwnx_sta->valid &&
						ether_addr_equal(rwnx_sta->mac_addr, addr)) {
						/* Return the found STA */
						return rwnx_sta;
					}
				}
			}
		}
	} else {
		return rwnx_vif->sta.ap;
	}

	return NULL;
}

/**
 * @add_virtual_intf: create a new virtual interface with the given name,
 *	must set the struct wireless_dev's iftype. Beware: You must create
 *	the new netdev in the wiphy's network namespace! Returns the struct
 *	wireless_dev, or an ERR_PTR. For P2P device wdevs, the driver must
 *	also set the address member in the wdev.
 */
static struct wireless_dev *rwnx_cfg80211_add_iface(struct wiphy *wiphy,
													const char *name,
													unsigned char name_assign_type,
													enum nl80211_iftype type,
													struct vif_params *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct wireless_dev *wdev;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0))
	unsigned char name_assign_type = NET_NAME_UNKNOWN;
#endif

	if (type != NL80211_IFTYPE_P2P_DEVICE) {
		struct rwnx_vif *vif = rwnx_interface_add(rwnx_hw, name, name_assign_type, type, params);
		if (!vif)
			return ERR_PTR(-EINVAL);
		return &vif->wdev;

	} else {
		wdev = rwnx_virtual_interface_add(rwnx_hw, name, name_assign_type, type, params);
		if (!wdev)
			return ERR_PTR(-EINVAL);
		return wdev;
	}
}

/**
 * @del_virtual_intf: remove the virtual interface
 */
static int rwnx_cfg80211_del_iface(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct net_device *dev = wdev->netdev;
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = container_of(wdev, struct rwnx_vif, wdev);

	printk("del_iface: %p, %x\n", wdev, wdev->address[5]);

	if (!dev || !rwnx_vif->ndev) {
		cfg80211_unregister_wdev(wdev);
		spin_lock_bh(&rwnx_hw->cb_lock);
		list_del(&rwnx_vif->list);
		spin_unlock_bh(&rwnx_hw->cb_lock);
		rwnx_hw->avail_idx_map |= BIT(rwnx_vif->drv_vif_index);
		rwnx_vif->ndev = NULL;
		kfree(rwnx_vif);
		return 0;
	}

	netdev_info(dev, "Remove Interface");

	if (dev->reg_state == NETREG_REGISTERED) {
		/* Will call rwnx_close if interface is UP */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
		cfg80211_unregister_netdevice(dev);
#else
		unregister_netdevice(dev);
#endif
	}

	spin_lock_bh(&rwnx_hw->cb_lock);
	list_del(&rwnx_vif->list);
	spin_unlock_bh(&rwnx_hw->cb_lock);
	rwnx_hw->avail_idx_map |= BIT(rwnx_vif->drv_vif_index);
	rwnx_vif->ndev = NULL;

	/* Clear the priv in adapter */
	dev->ieee80211_ptr = NULL;

	return 0;
}

/**
 * @change_virtual_intf: change type/configuration of virtual interface,
 *	keep the struct wireless_dev's iftype updated.
 */
static int rwnx_cfg80211_change_iface(struct wiphy *wiphy,
									  struct net_device *dev,
									  enum nl80211_iftype type,
									  struct vif_params *params)
{
#ifndef CONFIG_RWNX_MON_DATA
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
#endif
	struct rwnx_vif *vif = netdev_priv(dev);
	struct mm_add_if_cfm add_if_cfm;
	bool_l p2p = false;
	int ret;

	RWNX_DBG(RWNX_FN_ENTRY_STR);
	printk("change_if: %d to %d, %d, %d", vif->wdev.iftype, type, NL80211_IFTYPE_P2P_CLIENT, NL80211_IFTYPE_STATION);

#ifdef CONFIG_COEX
	if (type == NL80211_IFTYPE_AP || type == NL80211_IFTYPE_P2P_GO)
		rwnx_send_coex_req(vif->rwnx_hw, 1, 0);
	if (RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_AP || RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_P2P_GO)
		rwnx_send_coex_req(vif->rwnx_hw, 0, 1);
#endif
#ifndef CONFIG_RWNX_MON_DATA
	if ((type == NL80211_IFTYPE_MONITOR) &&
	   (RWNX_VIF_TYPE(vif) != NL80211_IFTYPE_MONITOR)) {
		struct rwnx_vif *vif_el;
		list_for_each_entry(vif_el, &rwnx_hw->vifs, list) {
			// Check if data interface already exists
			if ((vif_el != vif) &&
			   (RWNX_VIF_TYPE(vif) != NL80211_IFTYPE_MONITOR)) {
				wiphy_err(rwnx_hw->wiphy,
						"Monitor+Data interface support (MON_DATA) disabled\n");
				return -EIO;
			}
		}
	}
#endif

	// Reset to default case (i.e. not monitor)
	dev->type = ARPHRD_ETHER;
	dev->netdev_ops = &rwnx_netdev_ops;

	switch (type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		vif->sta.ap = NULL;
		vif->sta.tdls_sta = NULL;
		vif->sta.external_auth = false;
		break;
	case NL80211_IFTYPE_MESH_POINT:
		INIT_LIST_HEAD(&vif->ap.mpath_list);
		INIT_LIST_HEAD(&vif->ap.proxy_list);
		vif->ap.create_path = false;
		vif->ap.generation = 0;
		// no break
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
		INIT_LIST_HEAD(&vif->ap.sta_list);
		memset(&vif->ap.bcn, 0, sizeof(vif->ap.bcn));
		break;
	case NL80211_IFTYPE_AP_VLAN:
		return -EPERM;
	case NL80211_IFTYPE_MONITOR:
		dev->type = ARPHRD_IEEE80211_RADIOTAP;
		dev->netdev_ops = &rwnx_netdev_monitor_ops;
		break;
	default:
		break;
	}

	vif->wdev.iftype = type;
	if (params->use_4addr != -1)
		vif->use_4addr = params->use_4addr;
	if (type == NL80211_IFTYPE_P2P_CLIENT || type == NL80211_IFTYPE_P2P_GO)
		p2p = true;

	if (vif->up) {
	/* Abort scan request on the vif */
	if (vif->rwnx_hw->scan_request &&
		vif->rwnx_hw->scan_request->wdev == &vif->wdev) {
#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
			struct cfg80211_scan_info info = {
				.aborted = true,
			};

			cfg80211_scan_done(vif->rwnx_hw->scan_request, &info);
#else
			cfg80211_scan_done(vif->rwnx_hw->scan_request, true);
#endif
		ret = rwnx_send_scanu_cancel_req(vif->rwnx_hw, NULL);
		if (ret) {
			printk("scanu_cancel fail\n");
			return ret;
		}
		vif->rwnx_hw->scan_request = NULL;
#else
		ret = rwnx_send_scanu_cancel_req(vif->rwnx_hw, NULL);
		if (ret) {
			printk("scanu_cancel fail\n");
			return ret;
		}
#endif
	}
	ret = rwnx_send_remove_if(vif->rwnx_hw, vif->vif_index, false);
	if (ret) {
		printk("remove_if fail\n");
		return ret;
	}
	vif->rwnx_hw->vif_table[vif->vif_index] = NULL;
	printk("change_if from %d \n", vif->vif_index);
	ret = rwnx_send_add_if(vif->rwnx_hw, vif->wdev.address, RWNX_VIF_TYPE(vif), p2p, &add_if_cfm);
	if (ret) {
		printk("add if fail\n");
		return ret;
	}
	if (add_if_cfm.status != 0) {
		printk("add if status fail\n");
		return -EIO;
	}
		printk("change_if to %d \n", add_if_cfm.inst_nbr);
		/* Save the index retrieved from LMAC */
		spin_lock_bh(&vif->rwnx_hw->cb_lock);
		vif->vif_index = add_if_cfm.inst_nbr;
		vif->rwnx_hw->vif_table[add_if_cfm.inst_nbr] = vif;
		spin_unlock_bh(&vif->rwnx_hw->cb_lock);
	}
	return 0;
}

static int rwnx_cfgp2p_start_p2p_device(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	int ret = 0;

	//do nothing
	printk("P2P interface started\n");

	return ret;
}

static void rwnx_cfgp2p_stop_p2p_device(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	int ret = 0;
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = container_of(wdev, struct rwnx_vif, wdev);
	/* Abort scan request on the vif */
	if (rwnx_hw->scan_request &&
		rwnx_hw->scan_request->wdev == &rwnx_vif->wdev) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 8, 0)
		struct cfg80211_scan_info info = {
						.aborted = true,
				};

		cfg80211_scan_done(rwnx_hw->scan_request, &info);
#else
		cfg80211_scan_done(rwnx_hw->scan_request, true);
#endif
		rwnx_hw->scan_request = NULL;
		ret = rwnx_send_scanu_cancel_req(rwnx_hw, NULL);
		if (ret)
			printk("scanu_cancel fail\n");
	}

	if (rwnx_vif == rwnx_hw->p2p_dev_vif) {
		rwnx_hw->is_p2p_alive = 0;
		if (timer_pending(&rwnx_hw->p2p_alive_timer)) {
			del_timer_sync(&rwnx_hw->p2p_alive_timer);
		}

		if (rwnx_vif->up) {
			rwnx_send_remove_if(rwnx_hw, rwnx_vif->vif_index, true);
			/* Ensure that we won't process disconnect ind */
			spin_lock_bh(&rwnx_hw->cb_lock);
			rwnx_vif->up = false;
			rwnx_hw->vif_table[rwnx_vif->vif_index] = NULL;
			rwnx_hw->vif_started--;
			spin_unlock_bh(&rwnx_hw->cb_lock);
		}

	}

	printk("Exit. P2P interface stopped\n");

	return;
}


/**
 * @scan: Request to do a scan. If returning zero, the scan request is given
 *	the driver, and will be valid until passed to cfg80211_scan_done().
 *	For scan results, call cfg80211_inform_bss(); you can call this outside
 *	the scan/scan_done bracket too.
 */
static int rwnx_cfg80211_scan(struct wiphy *wiphy,
							  struct cfg80211_scan_request *request)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = container_of(request->wdev, struct rwnx_vif, wdev);
	int error;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	if (rwnx_hw->cpmode == AICBSP_CPMODE_TEST)
		return -EBUSY;

	if ((int)atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_CONNECTING) {
		printk("%s wifi is connecting, return it\r\n", __func__);
		return -EBUSY;
	}

	if (scanning) {
		printk("%s, is scanning, abort\n", __func__);
		return -EBUSY;
	}

#if 1
	if ((RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_STATION ||
		RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_CLIENT) &&
		rwnx_vif->sta.external_auth) {
		printk("scan about: external auth\r\n");
		return -EBUSY;
	}
#endif

	rwnx_hw->scan_request = request;
	error = rwnx_send_scanu_req(rwnx_hw, rwnx_vif, request);
	if (error)
		return error;

	return 0;
}

/**
 * @add_key: add a key with the given parameters. @mac_addr will be %NULL
 *	when adding a group key.
 */
static int rwnx_cfg80211_add_key(struct wiphy *wiphy, struct net_device *netdev,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)) && defined(KERNEL_AOSP)
								 int link_id,
#endif
								 u8 key_index, bool pairwise, const u8 *mac_addr,
								 struct key_params *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif = netdev_priv(netdev);
	int i, error = 0;
	struct mm_key_add_cfm key_add_cfm;
	u8_l cipher = 0;
	struct rwnx_sta *sta = NULL;
	struct rwnx_key *rwnx_key;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	if (mac_addr) {
		sta = rwnx_get_sta(rwnx_hw, mac_addr);
		if (!sta)
			return -EINVAL;
		rwnx_key = &sta->key;
		if (vif->wdev.iftype == NL80211_IFTYPE_STATION || vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT)
			vif->sta.paired_cipher_type = params->cipher;
	} else {
		rwnx_key = &vif->key[key_index];
		vif->key_has_add = 1;
		if (vif->wdev.iftype == NL80211_IFTYPE_STATION || vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT)
			vif->sta.group_cipher_type = params->cipher;
	}

	/* Retrieve the cipher suite selector */
	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		cipher = MAC_CIPHER_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		cipher = MAC_CIPHER_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		cipher = MAC_CIPHER_TKIP;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		cipher = MAC_CIPHER_CCMP;
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		cipher = MAC_CIPHER_BIP_CMAC_128;
		break;
	case WLAN_CIPHER_SUITE_SMS4:
	{
		// Need to reverse key order
		u8 tmp, *key = (u8 *)params->key;
		cipher = MAC_CIPHER_WPI_SMS4;
		for (i = 0; i < WPI_SUBKEY_LEN/2; i++) {
			tmp = key[i];
			key[i] = key[WPI_SUBKEY_LEN - 1 - i];
			key[WPI_SUBKEY_LEN - 1 - i] = tmp;
		}
		for (i = 0; i < WPI_SUBKEY_LEN/2; i++) {
			tmp = key[i + WPI_SUBKEY_LEN];
			key[i + WPI_SUBKEY_LEN] = key[WPI_KEY_LEN - 1 - i];
			key[WPI_KEY_LEN - 1 - i] = tmp;
		}
		break;
	}
	default:
		return -EINVAL;
	}

	error = rwnx_send_key_add(rwnx_hw, vif->vif_index,
				(sta ? sta->sta_idx : 0xFF), pairwise,
				(u8 *)params->key, params->key_len,
				key_index, cipher, &key_add_cfm);
	if (error)
		return error;

	if (key_add_cfm.status != 0) {
		RWNX_PRINT_CFM_ERR(key_add);
		return -EIO;
	}

	/* Save the index retrieved from LMAC */
	rwnx_key->hw_idx = key_add_cfm.hw_key_idx;

	return 0;
}

/**
 * @get_key: get information about the key with the given parameters.
 *	@mac_addr will be %NULL when requesting information for a group
 *	key. All pointers given to the @callback function need not be valid
 *	after it returns. This function should return an error if it is
 *	not possible to retrieve the key, -ENOENT if it doesn't exist.
 *
 */
static int rwnx_cfg80211_get_key(struct wiphy *wiphy, struct net_device *netdev,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)) && defined(KERNEL_AOSP)
								 int link_id,
#endif
								 u8 key_index, bool pairwise, const u8 *mac_addr,
								 void *cookie,
								 void (*callback)(void *cookie, struct key_params*))
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	return -1;
}


/**
 * @del_key: remove a key given the @mac_addr (%NULL for a group key)
 *	and @key_index, return -ENOENT if the key doesn't exist.
 */
static int rwnx_cfg80211_del_key(struct wiphy *wiphy, struct net_device *netdev,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)) && defined(KERNEL_AOSP)
								 int link_id,
#endif
								 u8 key_index, bool pairwise, const u8 *mac_addr)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif = netdev_priv(netdev);
	int error;
	struct rwnx_sta *sta = NULL;
	struct rwnx_key *rwnx_key;

	RWNX_DBG(RWNX_FN_ENTRY_STR);
	if (mac_addr) {
		sta = rwnx_get_sta(rwnx_hw, mac_addr);
		if (!sta)
			return -EINVAL;
		rwnx_key = &sta->key;
		if (vif->wdev.iftype == NL80211_IFTYPE_STATION || vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT)
			vif->sta.paired_cipher_type = 0xff;
	} else {
		rwnx_key = &vif->key[key_index];
		vif->key_has_add = 0;
		if (vif->wdev.iftype == NL80211_IFTYPE_STATION || vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT)
			vif->sta.group_cipher_type = 0xff;
	}

	error = rwnx_send_key_del(rwnx_hw, rwnx_key->hw_idx);

	rwnx_key->hw_idx = 0;
	return error;
}

/**
 * @set_default_key: set the default key on an interface
 */
static int rwnx_cfg80211_set_default_key(struct wiphy *wiphy,
										 struct net_device *netdev,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)) && defined(KERNEL_AOSP)
										 int link_id,
#endif
										 u8 key_index, bool unicast, bool multicast)
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	return 0;
}

/**
 * @set_default_mgmt_key: set the default management frame key on an interface
 */
static int rwnx_cfg80211_set_default_mgmt_key(struct wiphy *wiphy,
											  struct net_device *netdev,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)) && defined(KERNEL_AOSP)
											  int link_id,
#endif
											  u8 key_index)
{
	return 0;
}

/**
 * @connect: Connect to the ESS with the specified parameters. When connected,
 *	call cfg80211_connect_result() with status code %WLAN_STATUS_SUCCESS.
 *	If the connection fails for some reason, call cfg80211_connect_result()
 *	with the status from the AP.
 *	(invoked with the wireless_dev mutex held)
 */
static int rwnx_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
								 struct cfg80211_connect_params *sme)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct sm_connect_cfm sm_connect_cfm;
	int error = 0;
	int is_wep = ((sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP40) ||
			(sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP104) ||
			(sme->crypto.ciphers_pairwise[0] == WLAN_CIPHER_SUITE_WEP40) ||
			(sme->crypto.ciphers_pairwise[0] == WLAN_CIPHER_SUITE_WEP104));

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	if ((int)atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_CONNECTED) {
		printk("%s this connection is roam \r\n", __func__);
		rwnx_vif->sta.is_roam = true;
	} else {
		rwnx_vif->sta.is_roam = false;
	}

	if ((int)atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_DISCONNECTING ||
		(int)atomic_read(&rwnx_vif->drv_conn_state) == (int)RWNX_DRV_STATUS_CONNECTING) {
		printk("%s driver is disconnecting or connecting ,return it \r\n", __func__);
		return -EALREADY;
	}

	atomic_set(&rwnx_vif->drv_conn_state, (int)RWNX_DRV_STATUS_CONNECTING);

	if (is_wep) {
		if (sme->auth_type == NL80211_AUTHTYPE_AUTOMATIC) {
			if (rwnx_vif->wep_enabled && rwnx_vif->wep_auth_err) {
				if (rwnx_vif->last_auth_type == NL80211_AUTHTYPE_SHARED_KEY)
					sme->auth_type = NL80211_AUTHTYPE_OPEN_SYSTEM;
				else
					sme->auth_type = NL80211_AUTHTYPE_SHARED_KEY;
			} else {
				if ((rwnx_vif->wep_enabled && !rwnx_vif->wep_auth_err))
					sme->auth_type = rwnx_vif->last_auth_type;
				else
				sme->auth_type = NL80211_AUTHTYPE_SHARED_KEY;
			}
			printk("auto: use sme->auth_type = %d\r\n", sme->auth_type);
		} else {
			if (rwnx_vif->wep_enabled && rwnx_vif->wep_auth_err && (sme->auth_type == rwnx_vif->last_auth_type)) {
				if (sme->auth_type == NL80211_AUTHTYPE_SHARED_KEY) {
					sme->auth_type = NL80211_AUTHTYPE_OPEN_SYSTEM;
					printk("start connect, auth_type changed, shared --> open\n");
				} else if (sme->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM) {
					sme->auth_type = NL80211_AUTHTYPE_SHARED_KEY;
					printk("start connect, auth_type changed, open --> shared\n");
				}
			}
		}
	}

	/* For SHARED-KEY authentication, must install key first */
	if (sme->auth_type == NL80211_AUTHTYPE_SHARED_KEY && sme->key) {
		struct key_params key_params;
		key_params.key = sme->key;
		key_params.seq = NULL;
		key_params.key_len = sme->key_len;
		key_params.seq_len = 0;
		key_params.cipher = sme->crypto.cipher_group;
		rwnx_cfg80211_add_key(wiphy, dev,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41)) && defined(KERNEL_AOSP)
				0,
#endif
				sme->key_idx, false, NULL, &key_params);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0) || defined(CONFIG_WPA3_FOR_OLD_KERNEL)
	else if ((sme->auth_type == NL80211_AUTHTYPE_SAE) &&
			 !(sme->flags & CONNECT_REQ_EXTERNAL_AUTH_SUPPORT)) {
		netdev_err(dev, "Doesn't support SAE without external authentication\n");
		return -EINVAL;
	}
#endif

	if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT) {
		rwnx_hw->is_p2p_connected = 1;
	}

	if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_STATION || rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT) {
		rwnx_vif->sta.paired_cipher_type = 0xff;
		rwnx_vif->sta.group_cipher_type = 0xff;
	}

	/* Forward the information to the LMAC */
	error = rwnx_send_sm_connect_req(rwnx_hw, rwnx_vif, sme, &sm_connect_cfm);
	if (error)
		return error;

	// Check the status
	switch (sm_connect_cfm.status) {
	case CO_OK:
		error = 0;
		break;
	case CO_BUSY:
		error = -EINPROGRESS;
		break;
	case CO_OP_IN_PROGRESS:
		error = -EALREADY;
		break;
	default:
		error = -EIO;
		break;
	}

	return error;
}

/**
 * @disconnect: Disconnect from the BSS/ESS.
 *	(invoked with the wireless_dev mutex held)
 */
static int rwnx_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
									u16 reason_code)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	printk("%s drv_vif_index:%d disconnect reason:%d \r\n",
			__func__, rwnx_vif->drv_vif_index, reason_code);

	if (atomic_read(&rwnx_vif->drv_conn_state) == RWNX_DRV_STATUS_CONNECTING) {
		printk("%s call cfg80211_connect_result reason:%d \r\n",
				__func__, reason_code);
		msleep(500);
	}

	if (atomic_read(&rwnx_vif->drv_conn_state) == RWNX_DRV_STATUS_DISCONNECTING) {
		printk("%s wifi is disconnecting, return it:%d \r\n",
				__func__, reason_code);
		return -EBUSY;
	}

	if (atomic_read(&rwnx_vif->drv_conn_state) == RWNX_DRV_STATUS_CONNECTED) {
		atomic_set(&rwnx_vif->drv_conn_state, RWNX_DRV_STATUS_DISCONNECTING);
		return rwnx_send_sm_disconnect_req(rwnx_hw, rwnx_vif, reason_code);
	} else {
		cfg80211_connect_result(dev,  NULL, NULL, 0, NULL, 0,
				reason_code?reason_code:WLAN_STATUS_UNSPECIFIED_FAILURE, GFP_ATOMIC);
		atomic_set(&rwnx_vif->drv_conn_state, RWNX_DRV_STATUS_DISCONNECTED);
		rwnx_external_auth_disable(rwnx_vif);
		return 0;
	}
}

#ifdef CONFIG_SCHED_SCAN
static int rwnx_cfg80211_sched_scan_stop(struct wiphy *wiphy,
					   struct net_device *ndev
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
					   , u64 reqid
#endif
						)
{

	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	//struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	printk("%s enter, wiphy:%p\r\n", __func__, wiphy);

	if (rwnx_hw->scan_request) {
		printk("%s rwnx_send_scanu_cancel_req\r\n", __func__);
		return rwnx_send_scanu_cancel_req(rwnx_hw, NULL);
	} else {
		return 0;
	}
}

static int rwnx_cfg80211_sched_scan_start(struct wiphy *wiphy,
							 struct net_device *dev,
							 struct cfg80211_sched_scan_request *request)

{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct cfg80211_scan_request *scan_request = NULL;

	int ret = 0;
	int index = 0;

	printk("%s enter, wiphy:%p\r\n", __func__, wiphy);

	if (rwnx_hw->is_sched_scan || scanning) {
		printk("%s is_sched_scanning and scanning, busy", __func__);
		return -EBUSY;
	}

	scan_request = (struct cfg80211_scan_request *)kmalloc(sizeof(struct cfg80211_scan_request), GFP_KERNEL);

	scan_request->ssids = request->ssids;
	scan_request->n_channels = request->n_channels;
	scan_request->n_ssids = request->n_match_sets;
	scan_request->no_cck = false;
	scan_request->ie = request->ie;
	scan_request->ie_len = request->ie_len;
	scan_request->flags = request->flags;

	scan_request->wiphy = wiphy;
	scan_request->scan_start = request->scan_start;
	memcpy(scan_request->mac_addr, request->mac_addr, ETH_ALEN);
	memcpy(scan_request->mac_addr_mask, request->mac_addr_mask, ETH_ALEN);
	rwnx_hw->sched_scan_req = request;
	scan_request->wdev = &rwnx_vif->wdev;
	printk("%s scan_request->n_channels:%d \r\n", __func__, scan_request->n_channels);
	printk("%s scan_request->n_ssids:%d \r\n", __func__, scan_request->n_ssids);

	for (index = 0; index < scan_request->n_ssids; index++) {
		memset(scan_request->ssids[index].ssid, 0, IEEE80211_MAX_SSID_LEN);

		memcpy(scan_request->ssids[index].ssid,
			request->match_sets[index].ssid.ssid,
			IEEE80211_MAX_SSID_LEN);

		scan_request->ssids[index].ssid_len = request->match_sets[index].ssid.ssid_len;

		printk("%s request ssid:%s len:%d \r\n", __func__,
			scan_request->ssids[index].ssid, scan_request->ssids[index].ssid_len);
	}

	for (index = 0; index < scan_request->n_channels; index++) {
		scan_request->channels[index] = request->channels[index];

		printk("%s scan_request->channels[%d]:%d \r\n", __func__, index,
			scan_request->channels[index]->center_freq);

		if (scan_request->channels[index] == NULL) {
			printk("%s ERROR!!! channels is NULL", __func__);
			continue;
		}
	}

	rwnx_hw->is_sched_scan = true;

	if (scanning) {
		printk("%s scanning, about it", __func__);
		kfree(scan_request);
		return -EBUSY;
	} else {
		ret = rwnx_cfg80211_scan(wiphy, scan_request);
	}
	return ret;
}
#endif //CONFIG_SCHED_SCAN

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0) || defined(CONFIG_WPA3_FOR_OLD_KERNEL)
/**
 * @external_auth: indicates result of offloaded authentication processing from
 *     user space
 */
static int rwnx_cfg80211_external_auth(struct wiphy *wiphy, struct net_device *dev,
									   struct cfg80211_external_auth_params *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);

	if (!rwnx_vif->sta.external_auth)
		return -EINVAL;

	rwnx_external_auth_disable(rwnx_vif);
	return rwnx_send_sm_external_auth_required_rsp(rwnx_hw, rwnx_vif,
												   params->status);
}
#endif

/**
 * @add_station: Add a new station.
 */
static int rwnx_cfg80211_add_station(struct wiphy *wiphy, struct net_device *dev,
									 const u8 *mac, struct station_parameters *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct me_sta_add_cfm me_sta_add_cfm;
	int error = 0;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	WARN_ON(RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_AP_VLAN);

	/* Do not add TDLS station */
	if (params->sta_flags_set & BIT(NL80211_STA_FLAG_TDLS_PEER))
		return 0;

	/* Indicate we are in a STA addition process - This will allow handling
	 * potential PS mode change indications correctly
	 */
	rwnx_hw->adding_sta = true;

	/* Forward the information to the LMAC */
	error = rwnx_send_me_sta_add(rwnx_hw, params, mac, rwnx_vif->vif_index, &me_sta_add_cfm);
	if (error)
		return error;

	// Check the status
	switch (me_sta_add_cfm.status) {
	case CO_OK:
	{
		struct rwnx_sta *sta = &rwnx_hw->sta_table[me_sta_add_cfm.sta_idx];
		int tid;
		sta->aid = params->aid;

		sta->sta_idx = me_sta_add_cfm.sta_idx;
		sta->ch_idx = rwnx_vif->ch_index;
		sta->vif_idx = rwnx_vif->vif_index;
		sta->vlan_idx = sta->vif_idx;
		sta->qos = (params->sta_flags_set & BIT(NL80211_STA_FLAG_WME)) != 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41) && defined(KERNEL_AOSP)
		sta->ht = params->link_sta_params.ht_capa ? 1 : 0;
		sta->vht = params->link_sta_params.vht_capa ? 1 : 0;
#else
		sta->ht = params->ht_capa ? 1 : 0;
		sta->vht = params->vht_capa ? 1 : 0;
#endif
		sta->acm = 0;
		sta->key.hw_idx = 0;

		if (params->local_pm != NL80211_MESH_POWER_UNKNOWN)
			sta->mesh_pm = params->local_pm;
		else
			sta->mesh_pm = rwnx_vif->ap.next_mesh_pm;
		rwnx_update_mesh_power_mode(rwnx_vif);

		for (tid = 0; tid < NX_NB_TXQ_PER_STA; tid++) {
			int uapsd_bit = rwnx_hwq2uapsd[rwnx_tid2hwq[tid]];
			if (params->uapsd_queues & uapsd_bit)
				sta->uapsd_tids |= 1 << tid;
			else
				sta->uapsd_tids &= ~(1 << tid);
		}
		memcpy(sta->mac_addr, mac, ETH_ALEN);
#ifdef CONFIG_DEBUG_FS
		rwnx_dbgfs_register_rc_stat(rwnx_hw, sta);
#endif

		/* Ensure that we won't process PS change or channel switch ind*/
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_txq_sta_init(rwnx_hw, sta, rwnx_txq_vif_get_status(rwnx_vif));
		list_add_tail(&sta->list, &rwnx_vif->ap.sta_list);
		sta->valid = true;
		rwnx_ps_bh_enable(rwnx_hw, sta, sta->ps.active || me_sta_add_cfm.pm_state);
		spin_unlock_bh(&rwnx_hw->cb_lock);

		error = 0;

		if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_AP || rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO) {
			struct station_info sinfo;
			memset(&sinfo, 0, sizeof(struct station_info));
			sinfo.assoc_req_ies = NULL;
			sinfo.assoc_req_ies_len = 0;
			#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
			sinfo.filled |= STATION_INFO_ASSOC_REQ_IES;
			#endif
			cfg80211_new_sta(rwnx_vif->ndev, sta->mac_addr, &sinfo, GFP_KERNEL);
		}
#ifdef CONFIG_RWNX_BFMER
		if (rwnx_hw->mod_params->bfmer)
			rwnx_send_bfmer_enable(rwnx_hw, sta, params->vht_capa);

		rwnx_mu_group_sta_init(sta, params->vht_capa);
#endif /* CONFIG_RWNX_BFMER */

		#define PRINT_STA_FLAG(f)                               \
			(params->sta_flags_set & BIT(NL80211_STA_FLAG_##f) ? "["#f"]" : "")

		netdev_info(dev, "Add sta %d (%pM) flags=%s%s%s%s%s%s%s",
					sta->sta_idx, mac,
					PRINT_STA_FLAG(AUTHORIZED),
					PRINT_STA_FLAG(SHORT_PREAMBLE),
					PRINT_STA_FLAG(WME),
					PRINT_STA_FLAG(MFP),
					PRINT_STA_FLAG(AUTHENTICATED),
					PRINT_STA_FLAG(TDLS_PEER),
					PRINT_STA_FLAG(ASSOCIATED));
		#undef PRINT_STA_FLAG
		break;
	}
	default:
		error = -EBUSY;
		break;
	}

	rwnx_hw->adding_sta = false;

	return error;
}

/**
 * @del_station: Remove a station
 */
static int rwnx_cfg80211_del_station_compat(struct wiphy *wiphy,
											struct net_device *dev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
				struct station_del_parameters *params
#else
		const u8 *mac
#endif
)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_sta *cur, *tmp;
	int error = 0, found = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	const u8 *mac = NULL;
#endif
#ifdef AICWF_RX_REORDER
	struct reord_ctrl_info *reord_info, *reord_tmp;
	u8 *macaddr;
	struct aicwf_rx_priv *rx_priv;
#endif

	RWNX_DBG(RWNX_FN_ENTRY_STR);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	if (params)
		mac = params->mac;
#endif

	list_for_each_entry_safe(cur, tmp, &rwnx_vif->ap.sta_list, list) {
		if ((!mac) || (!memcmp(cur->mac_addr, mac, ETH_ALEN))) {
			netdev_info(dev, "Del sta %d (%pM)", cur->sta_idx, cur->mac_addr);
			/* Ensure that we won't process PS change ind */
			spin_lock_bh(&rwnx_hw->cb_lock);
			cur->ps.active = false;
			cur->valid = false;
			spin_unlock_bh(&rwnx_hw->cb_lock);

			if (cur->vif_idx != cur->vlan_idx) {
				struct rwnx_vif *vlan_vif;
				vlan_vif = rwnx_hw->vif_table[cur->vlan_idx];
				if (vlan_vif->up) {
					if ((RWNX_VIF_TYPE(vlan_vif) == NL80211_IFTYPE_AP_VLAN) &&
						(vlan_vif->use_4addr)) {
						vlan_vif->ap_vlan.sta_4a = NULL;
					} else {
						WARN(1, "Deleting sta belonging to VLAN other than AP_VLAN 4A");
					}
				}
			}
		if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_AP || rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO) {
				cfg80211_del_sta(rwnx_vif->ndev, cur->mac_addr, GFP_KERNEL);
			}

#ifdef AICWF_RX_REORDER
#ifdef AICWF_SDIO_SUPPORT
			rx_priv = rwnx_hw->sdiodev->rx_priv;
#else
			rx_priv = rwnx_hw->usbdev->rx_priv;
#endif
			if ((rwnx_vif->wdev.iftype == NL80211_IFTYPE_STATION) || (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT)) {
				BUG();//should be other function
			} else if ((rwnx_vif->wdev.iftype == NL80211_IFTYPE_AP) || (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO)) {
				macaddr = cur->mac_addr;
				printk("deinit:macaddr:%x,%x,%x,%x,%x,%x\r\n", macaddr[0], macaddr[1], macaddr[2], \
									   macaddr[3], macaddr[4], macaddr[5]);
				spin_lock_bh(&rx_priv->stas_reord_lock);
				list_for_each_entry_safe(reord_info, reord_tmp,
					&rx_priv->stas_reord_list, list) {
					printk("reord_mac:%x,%x,%x,%x,%x,%x\r\n", reord_info->mac_addr[0], reord_info->mac_addr[1], reord_info->mac_addr[2], \
										   reord_info->mac_addr[3], reord_info->mac_addr[4], reord_info->mac_addr[5]);
					if (!memcmp(reord_info->mac_addr, macaddr, 6)) {
						reord_deinit_sta(rx_priv, reord_info);
						break;
					}
				}
				spin_unlock_bh(&rx_priv->stas_reord_lock);
			}
#endif

			rwnx_txq_sta_deinit(rwnx_hw, cur);
			error = rwnx_send_me_sta_del(rwnx_hw, cur->sta_idx, false);
			if ((error != 0) && (error != -EPIPE))
				return error;

#ifdef CONFIG_RWNX_BFMER
			// Disable Beamformer if supported
			rwnx_bfmer_report_del(rwnx_hw, cur);
			rwnx_mu_group_sta_del(rwnx_hw, cur);
#endif /* CONFIG_RWNX_BFMER */

			list_del(&cur->list);
#ifdef CONFIG_DEBUG_FS
			rwnx_dbgfs_unregister_rc_stat(rwnx_hw, cur);
#endif
			found = true;
			break;
		}
	}

	if (!found)
		return -ENOENT;

	rwnx_update_mesh_power_mode(rwnx_vif);

	return 0;
}

void apm_staloss_work_process(struct work_struct *work)
{
	struct rwnx_hw *rwnx_hw = container_of(work, struct rwnx_hw, apmStalossWork);
	struct rwnx_sta *cur, *tmp;
	int error = 0;

#ifdef AICWF_RX_REORDER
	struct reord_ctrl_info *reord_info, *reord_tmp;
	u8 *macaddr;
	struct aicwf_rx_priv *rx_priv;
#endif
	struct rwnx_vif *rwnx_vif;
	bool_l found = false;
	const u8 *mac = rwnx_hw->sta_mac_addr;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	// Look for VIF entry
	list_for_each_entry(rwnx_vif, &rwnx_hw->vifs, list) {
		if (rwnx_vif->vif_index == rwnx_hw->apm_vif_idx) {
			found = true;
			break;
		}
	}

	printk("apm vif idx=%d, found=%d, mac addr=%pM\n", rwnx_hw->apm_vif_idx, found, mac);
	if (!found || !rwnx_vif || (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_AP && RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_P2P_GO)) {
		return;
	}

	list_for_each_entry_safe(cur, tmp, &rwnx_vif->ap.sta_list, list) {
		if ((!mac) || (!memcmp(cur->mac_addr, mac, ETH_ALEN))) {
			netdev_info(rwnx_vif->ndev, "Del sta %d (%pM)", cur->sta_idx, cur->mac_addr);
			/* Ensure that we won't process PS change ind */
			spin_lock_bh(&rwnx_hw->cb_lock);
			cur->ps.active = false;
			cur->valid = false;
			spin_unlock_bh(&rwnx_hw->cb_lock);

			if (cur->vif_idx != cur->vlan_idx) {
				struct rwnx_vif *vlan_vif;
				vlan_vif = rwnx_hw->vif_table[cur->vlan_idx];
				if (vlan_vif->up) {
					if ((RWNX_VIF_TYPE(vlan_vif) == NL80211_IFTYPE_AP_VLAN) &&
						(vlan_vif->use_4addr)) {
						vlan_vif->ap_vlan.sta_4a = NULL;
					} else {
						WARN(1, "Deleting sta belonging to VLAN other than AP_VLAN 4A");
					}
				}
			}
			/*if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_AP || rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO) {
				cfg80211_del_sta(rwnx_vif->ndev, cur->mac_addr, GFP_KERNEL);
			}*/

#ifdef AICWF_RX_REORDER
#ifdef AICWF_SDIO_SUPPORT
			rx_priv = rwnx_hw->sdiodev->rx_priv;
#else
			rx_priv = rwnx_hw->usbdev->rx_priv;
#endif
			if ((rwnx_vif->wdev.iftype == NL80211_IFTYPE_STATION) || (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT)) {
				BUG();//should be other function
			} else if ((rwnx_vif->wdev.iftype == NL80211_IFTYPE_AP) || (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO)) {
				macaddr = cur->mac_addr;
				printk("deinit:macaddr:%x,%x,%x,%x,%x,%x\r\n", macaddr[0], macaddr[1], macaddr[2], \
									   macaddr[3], macaddr[4], macaddr[5]);
				spin_lock_bh(&rx_priv->stas_reord_lock);
				list_for_each_entry_safe(reord_info, reord_tmp,
					&rx_priv->stas_reord_list, list) {
					printk("reord_mac:%x,%x,%x,%x,%x,%x\r\n", reord_info->mac_addr[0], reord_info->mac_addr[1], reord_info->mac_addr[2], \
										   reord_info->mac_addr[3], reord_info->mac_addr[4], reord_info->mac_addr[5]);
					if (!memcmp(reord_info->mac_addr, macaddr, 6)) {
						reord_deinit_sta(rx_priv, reord_info);
						break;
					}
				}
				spin_unlock_bh(&rx_priv->stas_reord_lock);
			}
#endif

			rwnx_txq_sta_deinit(rwnx_hw, cur);
			error = rwnx_send_me_sta_del(rwnx_hw, cur->sta_idx, false);
			if ((error != 0) && (error != -EPIPE))
				return;

#ifdef CONFIG_RWNX_BFMER
			// Disable Beamformer if supported
			rwnx_bfmer_report_del(rwnx_hw, cur);
			rwnx_mu_group_sta_del(rwnx_hw, cur);
#endif /* CONFIG_RWNX_BFMER */

			list_del(&cur->list);
#ifdef CONFIG_DEBUG_FS
			rwnx_dbgfs_unregister_rc_stat(rwnx_hw, cur);
#endif
			found = true;
			break;
		}
	}

	if (!found)
		return;

	rwnx_update_mesh_power_mode(rwnx_vif);
}

void apm_probe_sta_work_process(struct work_struct *work)
{
	struct apm_probe_sta *probe_sta = container_of(work, struct apm_probe_sta, apmprobestaWork);
	struct rwnx_vif *rwnx_vif = container_of(probe_sta, struct rwnx_vif, sta_probe);
	bool found = false;
	struct rwnx_sta *cur, *tmp;

	u8 *mac = rwnx_vif->sta_probe.sta_mac_addr;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	list_for_each_entry_safe(cur, tmp, &rwnx_vif->ap.sta_list, list) {
		if (!memcmp(cur->mac_addr, mac, ETH_ALEN)) {
			found = true;
			break;
		}
	}

	printk("sta %pM found = %d\n", mac, found);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0)
	if (found)
		cfg80211_probe_status(rwnx_vif->ndev, mac, (u64)rwnx_vif->sta_probe.probe_id, 1, 0, false, GFP_ATOMIC);
	else
		cfg80211_probe_status(rwnx_vif->ndev, mac, (u64)rwnx_vif->sta_probe.probe_id, 0, 0, false, GFP_ATOMIC);
#else
	if (found)
		cfg80211_probe_status(rwnx_vif->ndev, mac, (u64)rwnx_vif->sta_probe.probe_id, 1, GFP_ATOMIC);
	else
		cfg80211_probe_status(rwnx_vif->ndev, mac, (u64)rwnx_vif->sta_probe.probe_id, 0, GFP_ATOMIC);

#endif
	rwnx_vif->sta_probe.probe_id++;
}

/**
 * @change_station: Modify a given station. Note that flags changes are not much
 *	validated in cfg80211, in particular the auth/assoc/authorized flags
 *	might come to the driver in invalid combinations -- make sure to check
 *	them, also against the existing state! Drivers must call
 *	cfg80211_check_station_change() to validate the information.
 */
static int rwnx_cfg80211_change_station(struct wiphy *wiphy, struct net_device *dev,
										const u8 *mac, struct station_parameters *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif = netdev_priv(dev);
	struct rwnx_sta *sta;

	sta = rwnx_get_sta(rwnx_hw, mac);
	if (!sta) {
		/* Add the TDLS station */
		if (params->sta_flags_set & BIT(NL80211_STA_FLAG_TDLS_PEER)) {
			struct rwnx_vif *rwnx_vif = netdev_priv(dev);
			struct me_sta_add_cfm me_sta_add_cfm;
			int error = 0;

			/* Indicate we are in a STA addition process - This will allow handling
			 * potential PS mode change indications correctly
			 */
			rwnx_hw->adding_sta = true;

			/* Forward the information to the LMAC */
			error = rwnx_send_me_sta_add(rwnx_hw, params, mac, rwnx_vif->vif_index, &me_sta_add_cfm);
			if (error)
				return error;

			// Check the status
			switch (me_sta_add_cfm.status) {
			case CO_OK:
			{
				int tid;
				sta = &rwnx_hw->sta_table[me_sta_add_cfm.sta_idx];
				sta->aid = params->aid;
				sta->sta_idx = me_sta_add_cfm.sta_idx;
				sta->ch_idx = rwnx_vif->ch_index;
				sta->vif_idx = rwnx_vif->vif_index;
				sta->vlan_idx = sta->vif_idx;
				sta->qos = (params->sta_flags_set & BIT(NL80211_STA_FLAG_WME)) != 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 41) && defined(KERNEL_AOSP)
				sta->ht = params->link_sta_params.ht_capa ? 1 : 0;
				sta->vht = params->link_sta_params.vht_capa ? 1 : 0;
#else
				sta->ht = params->ht_capa ? 1 : 0;
				sta->vht = params->vht_capa ? 1 : 0;
#endif
				sta->acm = 0;
				for (tid = 0; tid < NX_NB_TXQ_PER_STA; tid++) {
					int uapsd_bit = rwnx_hwq2uapsd[rwnx_tid2hwq[tid]];
					if (params->uapsd_queues & uapsd_bit)
						sta->uapsd_tids |= 1 << tid;
					else
						sta->uapsd_tids &= ~(1 << tid);
				}
				memcpy(sta->mac_addr, mac, ETH_ALEN);
#ifdef CONFIG_DEBUG_FS
				rwnx_dbgfs_register_rc_stat(rwnx_hw, sta);
#endif
				/* Ensure that we won't process PS change or channel switch ind*/
				spin_lock_bh(&rwnx_hw->cb_lock);
				rwnx_txq_sta_init(rwnx_hw, sta, rwnx_txq_vif_get_status(rwnx_vif));
				if (rwnx_vif->tdls_status == TDLS_SETUP_RSP_TX) {
					rwnx_vif->tdls_status = TDLS_LINK_ACTIVE;
					sta->tdls.initiator = true;
					sta->tdls.active = true;
				}
				/* Set TDLS channel switch capability */
				if ((params->ext_capab[3] & WLAN_EXT_CAPA4_TDLS_CHAN_SWITCH) &&
					!rwnx_vif->tdls_chsw_prohibited)
					sta->tdls.chsw_allowed = true;
				rwnx_vif->sta.tdls_sta = sta;
				sta->valid = true;
				spin_unlock_bh(&rwnx_hw->cb_lock);
#ifdef CONFIG_RWNX_BFMER
				if (rwnx_hw->mod_params->bfmer)
					rwnx_send_bfmer_enable(rwnx_hw, sta, params->vht_capa);

				rwnx_mu_group_sta_init(sta, NULL);
#endif /* CONFIG_RWNX_BFMER */

				#define PRINT_STA_FLAG(f)                               \
					(params->sta_flags_set & BIT(NL80211_STA_FLAG_##f) ? "["#f"]" : "")

				netdev_info(dev, "Add %s TDLS sta %d (%pM) flags=%s%s%s%s%s%s%s",
							sta->tdls.initiator ? "initiator" : "responder",
							sta->sta_idx, mac,
							PRINT_STA_FLAG(AUTHORIZED),
							PRINT_STA_FLAG(SHORT_PREAMBLE),
							PRINT_STA_FLAG(WME),
							PRINT_STA_FLAG(MFP),
							PRINT_STA_FLAG(AUTHENTICATED),
							PRINT_STA_FLAG(TDLS_PEER),
							PRINT_STA_FLAG(ASSOCIATED));
				#undef PRINT_STA_FLAG

				break;
			}
			default:
				error = -EBUSY;
				break;
			}

			rwnx_hw->adding_sta = false;
		} else  {
			return -EINVAL;
		}
	}

	if (params->sta_flags_mask & BIT(NL80211_STA_FLAG_AUTHORIZED))
		rwnx_send_me_set_control_port_req(rwnx_hw,
				(params->sta_flags_set & BIT(NL80211_STA_FLAG_AUTHORIZED)) != 0,
				sta->sta_idx);

	if (RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_MESH_POINT) {
		if (params->sta_modify_mask & STATION_PARAM_APPLY_PLINK_STATE) {
			if (params->plink_state < NUM_NL80211_PLINK_STATES) {
				rwnx_send_mesh_peer_update_ntf(rwnx_hw, vif, sta->sta_idx, params->plink_state);
			}
		}

		if (params->local_pm != NL80211_MESH_POWER_UNKNOWN) {
			sta->mesh_pm = params->local_pm;
			rwnx_update_mesh_power_mode(vif);
		}
	}

	if (params->vlan) {
		uint8_t vlan_idx;

		vif = netdev_priv(params->vlan);
		vlan_idx = vif->vif_index;

		if (sta->vlan_idx != vlan_idx) {
			struct rwnx_vif *old_vif;
			old_vif = rwnx_hw->vif_table[sta->vlan_idx];
			rwnx_txq_sta_switch_vif(sta, old_vif, vif);
			sta->vlan_idx = vlan_idx;

			if ((RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_AP_VLAN) &&
				(vif->use_4addr)) {
				WARN((vif->ap_vlan.sta_4a),
					 "4A AP_VLAN interface with more than one sta");
				vif->ap_vlan.sta_4a = sta;
			}

			if ((RWNX_VIF_TYPE(old_vif) == NL80211_IFTYPE_AP_VLAN) &&
				(old_vif->use_4addr)) {
				old_vif->ap_vlan.sta_4a = NULL;
			}
		}
	}

	return 0;
}

/**
 * @start_ap: Start acting in AP mode defined by the parameters.
 */
static int rwnx_cfg80211_start_ap(struct wiphy *wiphy, struct net_device *dev,
								  struct cfg80211_ap_settings *settings)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct apm_start_cfm apm_start_cfm;
	struct rwnx_ipc_elem_var elem;
	struct rwnx_sta *sta;
	int error = 0;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	INIT_WORK(&rwnx_vif->sta_probe.apmprobestaWork, apm_probe_sta_work_process);
	rwnx_vif->sta_probe.apmprobesta_wq = create_singlethread_workqueue("apmprobe_wq");
	if (!rwnx_vif->sta_probe.apmprobesta_wq) {
		txrx_err("insufficient memory to create apmprobe_wq.\n");
		return -ENOBUFS;
	}
	if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO)
		rwnx_hw->is_p2p_connected = 1;
	/* Forward the information to the LMAC */
	error = rwnx_send_apm_start_req(rwnx_hw, rwnx_vif, settings, &apm_start_cfm, &elem);
	if (error)
		goto end;

	// Check the status
	switch (apm_start_cfm.status) {
	case CO_OK:
	{
		u8 txq_status = 0;
		rwnx_vif->ap.bcmc_index = apm_start_cfm.bcmc_idx;
		rwnx_vif->ap.flags = 0;
		sta = &rwnx_hw->sta_table[apm_start_cfm.bcmc_idx];
		sta->valid = true;
		sta->aid = 0;
		sta->sta_idx = apm_start_cfm.bcmc_idx;
		sta->ch_idx = apm_start_cfm.ch_idx;
		sta->vif_idx = rwnx_vif->vif_index;
		sta->qos = false;
		sta->acm = 0;
		sta->ps.active = false;
		rwnx_mu_group_sta_init(sta, NULL);
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_chanctx_link(rwnx_vif, apm_start_cfm.ch_idx,
						  &settings->chandef);
		if (rwnx_hw->cur_chanctx != apm_start_cfm.ch_idx) {
			txq_status = RWNX_TXQ_STOP_CHAN;
		}
		rwnx_txq_vif_init(rwnx_hw, rwnx_vif, txq_status);
		spin_unlock_bh(&rwnx_hw->cb_lock);

		netif_tx_start_all_queues(dev);
		netif_carrier_on(dev);
		error = 0;
		/* If the AP channel is already the active, we probably skip radar
		   activation on MM_CHANNEL_SWITCH_IND (unless another vif use this
		   ctxt). In anycase retest if radar detection must be activated
		 */
		if (txq_status == 0) {
			rwnx_radar_detection_enable_on_cur_channel(rwnx_hw);
		}
		break;
	}
	case CO_BUSY:
		error = -EINPROGRESS;
		break;
	case CO_OP_IN_PROGRESS:
		error = -EALREADY;
		break;
	default:
		error = -EIO;
		break;
	}

	if (error) {
		netdev_info(dev, "Failed to start AP (%d)", error);
	} else {
		netdev_info(dev, "AP started: ch=%d, bcmc_idx=%d channel=%d bw=%d",
					rwnx_vif->ch_index, rwnx_vif->ap.bcmc_index,
					((settings->chandef).chan)->center_freq,
					((settings->chandef).width));
	}

end:
	//rwnx_ipc_elem_var_deallocs(rwnx_hw, &elem);

	return error;
}


/**
 * @change_beacon: Change the beacon parameters for an access point mode
 *	interface. This should reject the call when AP mode wasn't started.
 */
static int rwnx_cfg80211_change_beacon(struct wiphy *wiphy, struct net_device *dev,
									   struct cfg80211_beacon_data *info)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif = netdev_priv(dev);
	struct rwnx_bcn *bcn = &vif->ap.bcn;
	struct rwnx_ipc_elem_var elem;
	u8 *buf;
	int error = 0;
	elem.dma_addr = 0;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	// Build the beacon
	buf = rwnx_build_bcn(bcn, info);
	if (!buf)
		return -ENOMEM;

	rwnx_send_bcn(rwnx_hw, buf, vif->vif_index, bcn->len);

	// Forward the information to the LMAC
	error = rwnx_send_bcn_change(rwnx_hw, vif->vif_index, 0,
								 bcn->len, bcn->head_len, bcn->tim_len, NULL);

	return error;
}

/**
 * * @stop_ap: Stop being an AP, including stopping beaconing.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)) && defined(KERNEL_AOSP)
static int rwnx_cfg80211_stop_ap(struct wiphy *wiphy, struct net_device *dev, unsigned int link_id)
#else
static int rwnx_cfg80211_stop_ap(struct wiphy *wiphy, struct net_device *dev)
#endif
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_sta *sta;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	netif_tx_stop_all_queues(dev);
	netif_carrier_off(dev);

	if (rwnx_vif->wdev.iftype == NL80211_IFTYPE_P2P_GO)
		rwnx_hw->is_p2p_connected = 0;
	rwnx_radar_cancel_cac(&rwnx_hw->radar);
	rwnx_send_apm_stop_req(rwnx_hw, rwnx_vif);
	spin_lock_bh(&rwnx_hw->cb_lock);
	rwnx_chanctx_unlink(rwnx_vif);
	spin_unlock_bh(&rwnx_hw->cb_lock);

	/* delete any remaining STA*/
	while (!list_empty(&rwnx_vif->ap.sta_list)) {
		rwnx_cfg80211_del_station_compat(wiphy, dev, NULL);
	}

	/* delete BC/MC STA */
	sta = &rwnx_hw->sta_table[rwnx_vif->ap.bcmc_index];
	rwnx_txq_vif_deinit(rwnx_hw, rwnx_vif);
	rwnx_del_bcn(&rwnx_vif->ap.bcn);
	rwnx_del_csa(rwnx_vif);

	flush_workqueue(rwnx_vif->sta_probe.apmprobesta_wq);
	destroy_workqueue(rwnx_vif->sta_probe.apmprobesta_wq);

	netdev_info(dev, "AP Stopped");

	return 0;
}

/**
 * @set_monitor_channel: Set the monitor mode channel for the device. If other
 *	interfaces are active this callback should reject the configuration.
 *	If no interfaces are active or the device is down, the channel should
 *	be stored for when a monitor interface becomes active.
 *
 * Also called internaly with chandef set to NULL simply to retrieve the channel
 * configured at firmware level.
 */
static int rwnx_cfg80211_set_monitor_channel(struct wiphy *wiphy,
											 struct cfg80211_chan_def *chandef)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif;
	struct me_config_monitor_cfm cfm;
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	if (rwnx_hw->monitor_vif == RWNX_INVALID_VIF)
		return -EINVAL;

	rwnx_vif = rwnx_hw->vif_table[rwnx_hw->monitor_vif];

	// Do nothing if monitor interface is already configured with the requested channel
	if (rwnx_chanctx_valid(rwnx_hw, rwnx_vif->ch_index)) {
		struct rwnx_chanctx *ctxt;
		ctxt = &rwnx_vif->rwnx_hw->chanctx_table[rwnx_vif->ch_index];
		if (chandef && cfg80211_chandef_identical(&ctxt->chan_def, chandef))
			return 0;
	}

	// Always send command to firmware. It allows to retrieve channel context index
	// and its configuration.
	if (rwnx_send_config_monitor_req(rwnx_hw, chandef, &cfm))
		return -EIO;

	// Always re-set channel context info
	rwnx_chanctx_unlink(rwnx_vif);



	// If there is also a STA interface not yet connected then monitor interface
	// will only have a channel context after the connection of the STA interface.
	if (cfm.chan_index != RWNX_CH_NOT_SET) {
		struct cfg80211_chan_def mon_chandef;

		if (rwnx_hw->vif_started > 1) {
			// In this case we just want to update the channel context index not
			// the channel configuration
			rwnx_chanctx_link(rwnx_vif, cfm.chan_index, NULL);
			return -EBUSY;
		}

		mon_chandef.chan = ieee80211_get_channel(wiphy, cfm.chan.prim20_freq);
		mon_chandef.center_freq1 = cfm.chan.center1_freq;
		mon_chandef.center_freq2 = cfm.chan.center2_freq;
		mon_chandef.width =  chnl2bw[cfm.chan.type];
		rwnx_chanctx_link(rwnx_vif, cfm.chan_index, &mon_chandef);
	}

	return 0;
}

/**
 * @probe_client: probe an associated client, must return a cookie that it
 *	later passes to cfg80211_probe_status().
 */
int rwnx_cfg80211_probe_client(struct wiphy *wiphy, struct net_device *dev,
			const u8 *peer, u64 *cookie)
{
	//struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif = netdev_priv(dev);
	struct rwnx_sta *sta = NULL;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	if ((RWNX_VIF_TYPE(vif) != NL80211_IFTYPE_AP) && (RWNX_VIF_TYPE(vif) != NL80211_IFTYPE_P2P_GO) &&
		(RWNX_VIF_TYPE(vif) != NL80211_IFTYPE_AP_VLAN))
		return -EINVAL;
	list_for_each_entry(sta, &vif->ap.sta_list, list) {
	if (sta->valid && ether_addr_equal(sta->mac_addr, peer))
		break;
	}

	if (!sta)
		return -ENOENT;


	memcpy(vif->sta_probe.sta_mac_addr, peer, 6);
	queue_work(vif->sta_probe.apmprobesta_wq, &vif->sta_probe.apmprobestaWork);

	*cookie = vif->sta_probe.probe_id;

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
/**
 * @mgmt_frame_register: Notify driver that a management frame type was
 *	registered. Note that this callback may not sleep, and cannot run
 *	concurrently with itself.
 */
void rwnx_cfg80211_mgmt_frame_register(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   u16 frame_type, bool reg)
{
}
#endif

/**
 * @set_wiphy_params: Notify that wiphy parameters have changed;
 *	@changed bitfield (see &enum wiphy_params_flags) describes which values
 *	have changed. The actual parameter values are available in
 *	struct wiphy. If returning an error, no value should be changed.
 */
static int rwnx_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	return 0;
}


/**
 * @set_tx_power: set the transmit power according to the parameters,
 *	the power passed is in mBm, to get dBm use MBM_TO_DBM(). The
 *	wdev may be %NULL if power was set for the wiphy, and will
 *	always be %NULL unless the driver supports per-vif TX power
 *	(as advertised by the nl80211 feature flag.)
 */
static int rwnx_cfg80211_set_tx_power(struct wiphy *wiphy, struct wireless_dev *wdev,
									  enum nl80211_tx_power_setting type, int mbm)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif;
	s8 pwr;
	int res = 0;

	if (type == NL80211_TX_POWER_AUTOMATIC) {
		pwr = 0x7f;
	} else {
		pwr = MBM_TO_DBM(mbm);
	}

	if (wdev) {
		vif = container_of(wdev, struct rwnx_vif, wdev);
		res = rwnx_send_set_power(rwnx_hw, vif->vif_index, pwr, NULL);
	} else {
		list_for_each_entry(vif, &rwnx_hw->vifs, list) {
			res = rwnx_send_set_power(rwnx_hw, vif->vif_index, pwr, NULL);
			if (res)
				break;
		}
	}

	return res;
}

/**
 * @set_power_mgmt: set the power save to one of those two modes:
 *  Power-save off
 *  Power-save on - Dynamic mode
 */
static int rwnx_cfg80211_set_power_mgmt(struct wiphy *wiphy,
										struct net_device *dev,
										bool enabled, int timeout)
{
	/* TODO
	 * Add handle in the feature!
	 */
	return 0;
}

static int rwnx_cfg80211_set_txq_params(struct wiphy *wiphy, struct net_device *dev,
										struct ieee80211_txq_params *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	u8 hw_queue, aifs, cwmin, cwmax;
	u32 param;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	hw_queue = rwnx_ac2hwq[0][params->ac];

	aifs  = params->aifs;
	cwmin = fls(params->cwmin);
	cwmax = fls(params->cwmax);

	/* Store queue information in general structure */
	param  = (u32) (aifs << 0);
	param |= (u32) (cwmin << 4);
	param |= (u32) (cwmax << 8);
	param |= (u32) (params->txop) << 12;

	/* Send the MM_SET_EDCA_REQ message to the FW */
	return rwnx_send_set_edca(rwnx_hw, hw_queue, param, false, rwnx_vif->vif_index);
}


/**
 * @remain_on_channel: Request the driver to remain awake on the specified
 *	channel for the specified duration to complete an off-channel
 *	operation (e.g., public action frame exchange). When the driver is
 *	ready on the requested channel, it must indicate this with an event
 *	notification by calling cfg80211_ready_on_channel().
 */
static int
rwnx_cfg80211_remain_on_channel(struct wiphy *wiphy, struct wireless_dev *wdev,
								struct ieee80211_channel *chan,
								unsigned int duration, u64 *cookie)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	//struct rwnx_vif *rwnx_vif = netdev_priv(wdev->netdev);
	struct rwnx_vif *rwnx_vif = container_of(wdev, struct rwnx_vif, wdev);
	struct rwnx_roc_elem *roc_elem;
	struct mm_add_if_cfm add_if_cfm;
	struct mm_remain_on_channel_cfm roc_cfm;
	int error;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	/* For debug purpose (use ftrace kernel option) */
#ifdef CREATE_TRACE_POINTS
	trace_roc(rwnx_vif->vif_index, chan->center_freq, duration);
#endif
	/* Check that no other RoC procedure has been launched */
	if (rwnx_hw->roc_elem) {
		msleep(2);
		if (rwnx_hw->roc_elem) {
			printk("remain_on_channel fail\n");
			return -EBUSY;
		}
	}

	printk("remain:%d,%d,%d\n", rwnx_vif->vif_index, rwnx_vif->is_p2p_vif, rwnx_hw->is_p2p_alive);
	if (rwnx_vif == rwnx_hw->p2p_dev_vif && !rwnx_vif->up) {
		if (!rwnx_hw->is_p2p_alive) {
			error = rwnx_send_add_if (rwnx_hw, rwnx_vif->wdev.address, //wdev->netdev->dev_addr,
								  RWNX_VIF_TYPE(rwnx_vif), false, &add_if_cfm);
			if (error)
				return -EIO;

			if (add_if_cfm.status != 0) {
				return -EIO;
			}

			/* Save the index retrieved from LMAC */
			spin_lock_bh(&rwnx_hw->cb_lock);
			rwnx_vif->vif_index = add_if_cfm.inst_nbr;
			rwnx_vif->up = true;
			rwnx_hw->vif_started++;
			rwnx_hw->vif_table[add_if_cfm.inst_nbr] = rwnx_vif;
			spin_unlock_bh(&rwnx_hw->cb_lock);
			rwnx_hw->is_p2p_alive = 1;
			mod_timer(&rwnx_hw->p2p_alive_timer, jiffies + msecs_to_jiffies(1000));
			atomic_set(&rwnx_hw->p2p_alive_timer_count, 0);
		} else {
			mod_timer(&rwnx_hw->p2p_alive_timer, jiffies + msecs_to_jiffies(1000));
			atomic_set(&rwnx_hw->p2p_alive_timer_count, 0);
		}
	}

	/* Allocate a temporary RoC element */
	roc_elem = kmalloc(sizeof(struct rwnx_roc_elem), GFP_KERNEL);

	/* Verify that element has well been allocated */
	if (!roc_elem) {
		return -ENOMEM;
	}

	/* Initialize the RoC information element */
	roc_elem->wdev = wdev;
	roc_elem->chan = chan;
	roc_elem->duration = duration;
	roc_elem->mgmt_roc = false;
	roc_elem->on_chan = false;

	/* Initialize the OFFCHAN TX queue to allow off-channel transmissions */
	rwnx_txq_offchan_init(rwnx_vif);

	/* Forward the information to the FMAC */
	rwnx_hw->roc_elem = roc_elem;
	error = rwnx_send_roc(rwnx_hw, rwnx_vif, chan, duration, &roc_cfm);

	/* If no error, keep all the information for handling of end of procedure */
	if (error == 0) {
		/* Set the cookie value */
		*cookie = (u64)(rwnx_hw->roc_cookie_cnt);
		if (roc_cfm.status) {
			// failed to roc
			rwnx_hw->roc_elem = NULL;
			kfree(roc_elem);
			rwnx_txq_offchan_deinit(rwnx_vif);
			return -EBUSY;
		}
	} else {
		/* Free the allocated element */
		rwnx_hw->roc_elem = NULL;
		kfree(roc_elem);
		rwnx_txq_offchan_deinit(rwnx_vif);
	}

	return error;
}

/**
 * @cancel_remain_on_channel: Cancel an on-going remain-on-channel operation.
 *	This allows the operation to be terminated prior to timeout based on
 *	the duration value.
 */
static int rwnx_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
												  struct wireless_dev *wdev,
												  u64 cookie)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
#ifdef CREATE_TRACE_POINTS
	struct rwnx_vif *rwnx_vif = container_of(wdev, struct rwnx_vif, wdev);//netdev_priv(wdev->netdev);
#endif
	RWNX_DBG(RWNX_FN_ENTRY_STR);

	/* For debug purpose (use ftrace kernel option) */
#ifdef CREATE_TRACE_POINTS
	trace_cancel_roc(rwnx_vif->vif_index);
#endif
	/* Check if a RoC procedure is pending */
	if (!rwnx_hw->roc_elem) {
		return 0;
	}

	/* Forward the information to the FMAC */
	return rwnx_send_cancel_roc(rwnx_hw);
}

#define IS_2P4GHZ(n) (n >= 2412 && n <= 2484)
#define IS_5GHZ(n) (n >= 4000 && n <= 5895)
#define DEFAULT_NOISE_FLOOR_2GHZ (-89)
#define DEFAULT_NOISE_FLOOR_5GHZ (-92)
/**
 * @dump_survey: get site survey information.
 */
static int rwnx_cfg80211_dump_survey(struct wiphy *wiphy, struct net_device *netdev,
									 int idx, struct survey_info *info)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct ieee80211_supported_band *sband;
	struct rwnx_survey_info *rwnx_survey;

	//RWNX_DBG(RWNX_FN_ENTRY_STR);

	if (idx >= ARRAY_SIZE(rwnx_hw->survey))
		return -ENOENT;

	rwnx_survey = &rwnx_hw->survey[idx];

	// Check if provided index matches with a supported 2.4GHz channel
	sband = wiphy->bands[NL80211_BAND_2GHZ];
	if (sband && idx >= sband->n_channels) {
		idx -= sband->n_channels;
		sband = NULL;
	}

	if (rwnx_hw->band_5g_support) {
		if (!sband) {
			// Check if provided index matches with a supported 5GHz channel
			sband = wiphy->bands[NL80211_BAND_5GHZ];

			if (!sband || idx >= sband->n_channels)
				return -ENOENT;
		}
	} else {
		if (!sband || idx >= sband->n_channels)
			return -ENOENT;
	}

	// Fill the survey
	info->channel = &sband->channels[idx];
	info->filled = rwnx_survey->filled;

	if (rwnx_survey->filled != 0) {
		SURVEY_TIME(info) = (u64)rwnx_survey->chan_time_ms;
		SURVEY_TIME_BUSY(info) = (u64)rwnx_survey->chan_time_busy_ms;
		//info->noise = rwnx_survey->noise_dbm;
		info->noise = ((IS_2P4GHZ(info->channel->center_freq)) ? DEFAULT_NOISE_FLOOR_2GHZ :
				(IS_5GHZ(info->channel->center_freq)) ? DEFAULT_NOISE_FLOOR_5GHZ : DEFAULT_NOISE_FLOOR_5GHZ);

		// Set the survey report as not used
		if (info->noise == 0) {
			rwnx_survey->filled = 0;
		} else {
			rwnx_survey->filled |= SURVEY_INFO_NOISE_DBM;
		}
	}

	return 0;
}

/**
 * @get_channel: Get the current operating channel for the virtual interface.
 *	For monitor interfaces, it should return %NULL unless there's a single
 *	current monitoring channel.
 */
static int rwnx_cfg80211_get_channel(struct wiphy *wiphy,
									 struct wireless_dev *wdev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0) && defined(KERNEL_AOSP)
									 unsigned int link_id,
#endif
									 struct cfg80211_chan_def *chandef)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = container_of(wdev, struct rwnx_vif, wdev);
	struct rwnx_chanctx *ctxt;

	if (!rwnx_vif->up) {
		return -ENODATA;
	}

	if (rwnx_vif->vif_index == rwnx_hw->monitor_vif) {
		//retrieve channel from firmware
		rwnx_cfg80211_set_monitor_channel(wiphy, NULL);
	}

	//Check if channel context is valid
	if (!rwnx_chanctx_valid(rwnx_hw, rwnx_vif->ch_index)) {
		return -ENODATA;
	}

	ctxt = &rwnx_hw->chanctx_table[rwnx_vif->ch_index];
	*chandef = ctxt->chan_def;

	return 0;
}

/**
 * @mgmt_tx: Transmit a management frame.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
static int rwnx_cfg80211_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
								 struct cfg80211_mgmt_tx_params *params,
								 u64 *cookie)
#else
static int rwnx_cfg80211_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
								 struct ieee80211_channel *channel, bool offchan,
								 unsigned int wait, const u8 *buf, size_t len,
							#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
								 bool no_cck,
							#endif
							#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
								 bool dont_wait_for_ack,
							#endif
								 u64 *cookie)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0) */
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = container_of(wdev, struct rwnx_vif, wdev);//netdev_priv(wdev->netdev);
	struct rwnx_sta *rwnx_sta;
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	struct ieee80211_channel *channel = params->chan;
	const u8 *buf = params->buf;
	//size_t len = params->len;
	#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0) */
	struct ieee80211_mgmt *mgmt = (void *)buf;
	bool ap = false;
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	bool offchan = false;
	#endif

	/* Check if provided VIF is an AP or a STA one */
	switch (RWNX_VIF_TYPE(rwnx_vif)) {
	case NL80211_IFTYPE_AP_VLAN:
		rwnx_vif = rwnx_vif->ap_vlan.master;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_MESH_POINT:
		ap = true;
		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
	default:
		break;
	}

	/* Get STA on which management frame has to be sent */
	rwnx_sta = rwnx_retrieve_sta(rwnx_hw, rwnx_vif, mgmt->da,
								 mgmt->frame_control, ap);
#ifdef CREATE_TRACE_POINTS
	trace_mgmt_tx((channel) ? channel->center_freq : 0,
				  rwnx_vif->vif_index, (rwnx_sta) ? rwnx_sta->sta_idx : 0xFF,
				  mgmt);
#endif
	if (ap || rwnx_sta)
		goto send_frame;

	/* Not an AP interface sending frame to unknown STA:
	 * This is allowed for external authetication */
	if (rwnx_vif->sta.external_auth && ieee80211_is_auth(mgmt->frame_control))
		goto send_frame;

	/* Otherwise ROC is needed */
	if (!channel) {
		printk("mgmt_tx fail since channel\n");
		return -EINVAL;
	}

	/* Check that a RoC is already pending */
	if (rwnx_hw->roc_elem) {
		/* Get VIF used for current ROC */
		struct rwnx_vif *rwnx_roc_vif = container_of(rwnx_hw->roc_elem->wdev, struct rwnx_vif, wdev);//netdev_priv(rwnx_hw->roc_elem->wdev->netdev);

		/* Check if RoC channel is the same than the required one */
		if ((rwnx_hw->roc_elem->chan->center_freq != channel->center_freq)
			|| (rwnx_vif->vif_index != rwnx_roc_vif->vif_index)) {
			printk("mgmt rx chan invalid: %d, %d", rwnx_hw->roc_elem->chan->center_freq, channel->center_freq);
			return -EINVAL;
		}
	} else {
		u64 cookie;
		int error;

		printk("mgmt rx remain on chan\n");

		/* Start a ROC procedure for 30ms */
		error = rwnx_cfg80211_remain_on_channel(wiphy, wdev, channel,
												30, &cookie);
		if (error) {
			printk("mgmt rx chan err\n");
			return error;
		}
		/* Need to keep in mind that RoC has been launched internally in order to
		 * avoid to call the cfg80211 callback once expired */
		rwnx_hw->roc_elem->mgmt_roc = true;
	}

	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	offchan = true;
	#endif

send_frame:
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
	return rwnx_start_mgmt_xmit(rwnx_vif, rwnx_sta, params, offchan, cookie);
	#else
	return rwnx_start_mgmt_xmit(rwnx_vif, rwnx_sta, channel, offchan, wait, buf, len, no_cck, dont_wait_for_ack, cookie);
	#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0) */
}

/**
 * @start_radar_detection: Start radar detection in the driver.
 */
static
int rwnx_cfg80211_start_radar_detection(struct wiphy *wiphy,
										struct net_device *dev,
										struct cfg80211_chan_def *chandef
									#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
										, u32 cac_time_ms
									#endif
										)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct apm_start_cac_cfm cfm;

	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	rwnx_radar_start_cac(&rwnx_hw->radar, cac_time_ms, rwnx_vif);
	#endif
	rwnx_send_apm_start_cac_req(rwnx_hw, rwnx_vif, chandef, &cfm);

	if (cfm.status == CO_OK) {
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_chanctx_link(rwnx_vif, cfm.ch_idx, chandef);
		if (rwnx_hw->cur_chanctx == rwnx_vif->ch_index)
			rwnx_radar_detection_enable(&rwnx_hw->radar,
										RWNX_RADAR_DETECT_REPORT,
										RWNX_RADAR_RIU);
		spin_unlock_bh(&rwnx_hw->cb_lock);
	} else {
		return -EIO;
	}

	return 0;
}

/**
 * @update_ft_ies: Provide updated Fast BSS Transition information to the
 *	driver. If the SME is in the driver/firmware, this information can be
 *	used in building Authentication and Reassociation Request frames.
 */
static
int rwnx_cfg80211_update_ft_ies(struct wiphy *wiphy,
							struct net_device *dev,
							struct cfg80211_update_ft_ies_params *ftie)
{
	printk("%s\n", __func__);
	return 0;
}

/**
 * @set_cqm_rssi_config: Configure connection quality monitor RSSI threshold.
 */
static
int rwnx_cfg80211_set_cqm_rssi_config(struct wiphy *wiphy,
								  struct net_device *dev,
								  int32_t rssi_thold, uint32_t rssi_hyst)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);

	return rwnx_send_cfg_rssi_req(rwnx_hw, rwnx_vif->vif_index, rssi_thold, rssi_hyst);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
/**
 *
 * @channel_switch: initiate channel-switch procedure (with CSA). Driver is
 *	responsible for veryfing if the switch is possible. Since this is
 *	inherently tricky driver may decide to disconnect an interface later
 *	with cfg80211_stop_iface(). This doesn't mean driver can accept
 *	everything. It should do it's best to verify requests and reject them
 *	as soon as possible.
 */
int rwnx_cfg80211_channel_switch (struct wiphy *wiphy,
								 struct net_device *dev,
								 struct cfg80211_csa_settings *params)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *vif = netdev_priv(dev);
	struct rwnx_bcn *bcn, *bcn_after;
	struct rwnx_csa *csa;
	u16 csa_oft[BCN_MAX_CSA_CPT];
	u8 *buf;
	int i, error = 0;


	if (vif->ap.csa)
		return -EBUSY;

	if (params->n_counter_offsets_beacon > BCN_MAX_CSA_CPT)
		return -EINVAL;

	/* Build the new beacon with CSA IE */
	bcn = &vif->ap.bcn;
	buf = rwnx_build_bcn(bcn, &params->beacon_csa);
	if (!buf)
		return -ENOMEM;

	memset(csa_oft, 0, sizeof(csa_oft));
	for (i = 0; i < params->n_counter_offsets_beacon; i++) {
		csa_oft[i] = params->counter_offsets_beacon[i] + bcn->head_len +
			bcn->tim_len;
	}

	/* If count is set to 0 (i.e anytime after this beacon) force it to 2 */
	if (params->count == 0) {
		params->count = 2;
		for (i = 0; i < params->n_counter_offsets_beacon; i++) {
			buf[csa_oft[i]] = 2;
		}
	}

	error = rwnx_send_bcn(rwnx_hw, buf, vif->vif_index, bcn->len);
	if (error) {
		goto end;
	}

	/* Build the beacon to use after CSA. It will only be sent to fw once
	   CSA is over, but do it before sending the beacon as it must be ready
	   when CSA is finished. */
	csa = kzalloc(sizeof(struct rwnx_csa), GFP_KERNEL);
	if (!csa) {
		error = -ENOMEM;
		goto end;
	}

	bcn_after = &csa->bcn;
	buf = rwnx_build_bcn(bcn_after, &params->beacon_after);
	if (!buf) {
		error = -ENOMEM;
		rwnx_del_csa(vif);
		goto end;
	}

	error = rwnx_send_bcn(rwnx_hw, buf, vif->vif_index, bcn_after->len);
	if (error) {
		goto end;
	}

	vif->ap.csa = csa;
	csa->vif = vif;
	csa->chandef = params->chandef;

	/* Send new Beacon. FW will extract channel and count from the beacon */
	error = rwnx_send_bcn_change(rwnx_hw, vif->vif_index, 0,
								 bcn->len, bcn->head_len, bcn->tim_len, csa_oft);

	if (error) {
		rwnx_del_csa(vif);
		goto end;
	} else {
		INIT_WORK(&csa->work, rwnx_csa_finish);
		rwnx_cfg80211_ch_switch_started_notify(dev
					, &csa->chandef
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
					, 0
#endif
					, params->count
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
					, false
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
					, params->block_tx
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 94)
					, 0
#endif
					);
	}

end:
	return error;
}
#endif


/*
 * @tdls_mgmt: prepare TDLS action frame packets and forward them to FW
 */
static int
rwnx_cfg80211_tdls_mgmt(struct wiphy *wiphy, struct net_device *dev,
						const u8 *peer, u8 action_code,  u8 dialog_token,
						u16 status_code, u32 peer_capability,
						bool initiator, const u8 *buf, size_t len)
{
	#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0)
	u32 peer_capability = 0;
	#endif
	#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
	bool initiator = false;
	#endif
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	int ret = 0;

	/* make sure we support TDLS */
	if (!(wiphy->flags & WIPHY_FLAG_SUPPORTS_TDLS))
		return -ENOTSUPP;

	/* make sure we are in station mode (and connected) */
	if ((RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_STATION) ||
		(!rwnx_vif->up) || (!rwnx_vif->sta.ap))
		return -ENOTSUPP;

	/* only one TDLS link is supported */
	if ((action_code == WLAN_TDLS_SETUP_REQUEST) &&
		(rwnx_vif->sta.tdls_sta) &&
		(rwnx_vif->tdls_status == TDLS_LINK_ACTIVE)) {
		printk("%s: only one TDLS link is supported!\n", __func__);
		return -ENOTSUPP;
	}

	if ((action_code == WLAN_TDLS_DISCOVERY_REQUEST) &&
		(rwnx_hw->mod_params->ps_on)) {
		printk("%s: discovery request is not supported when "
				"power-save is enabled!\n", __func__);
		return -ENOTSUPP;
	}

	switch (action_code) {
	case WLAN_TDLS_SETUP_RESPONSE:
		/* only one TDLS link is supported */
		if ((status_code == 0) &&
			(rwnx_vif->sta.tdls_sta) &&
			(rwnx_vif->tdls_status == TDLS_LINK_ACTIVE)) {
			printk("%s: only one TDLS link is supported!\n", __func__);
			status_code = WLAN_STATUS_REQUEST_DECLINED;
		}
		/* fall-through */
	case WLAN_TDLS_SETUP_REQUEST:
	case WLAN_TDLS_TEARDOWN:
	case WLAN_TDLS_DISCOVERY_REQUEST:
	case WLAN_TDLS_SETUP_CONFIRM:
	case WLAN_PUB_ACTION_TDLS_DISCOVER_RES:
		ret = rwnx_tdls_send_mgmt_packet_data(rwnx_hw, rwnx_vif, peer, action_code,
				dialog_token, status_code, peer_capability, initiator, buf, len, 0, NULL);
		break;

	default:
		printk("%s: Unknown TDLS mgmt/action frame %pM\n",
				__func__, peer);
		ret = -EOPNOTSUPP;
		break;
	}

	if (action_code == WLAN_TDLS_SETUP_REQUEST) {
		rwnx_vif->tdls_status = TDLS_SETUP_REQ_TX;
	} else if (action_code == WLAN_TDLS_SETUP_RESPONSE) {
		rwnx_vif->tdls_status = TDLS_SETUP_RSP_TX;
	} else if ((action_code == WLAN_TDLS_SETUP_CONFIRM) && (ret == CO_OK)) {
		rwnx_vif->tdls_status = TDLS_LINK_ACTIVE;
		/* Set TDLS active */
		rwnx_vif->sta.tdls_sta->tdls.active = true;
	}

	return ret;
}

/*
 * @tdls_oper: execute TDLS operation
 */
static int
rwnx_cfg80211_tdls_oper(struct wiphy *wiphy, struct net_device *dev,
		const u8 *peer, enum nl80211_tdls_operation oper)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	int error;

	if (oper != NL80211_TDLS_DISABLE_LINK)
		return 0;

	if (!rwnx_vif->sta.tdls_sta) {
		printk("%s: TDLS station %pM does not exist\n", __func__, peer);
		return -ENOLINK;
	}

	if (memcmp(rwnx_vif->sta.tdls_sta->mac_addr, peer, ETH_ALEN) == 0) {
		/* Disable Channel Switch */
		if (!rwnx_send_tdls_cancel_chan_switch_req(rwnx_hw, rwnx_vif,
												   rwnx_vif->sta.tdls_sta,
												   NULL))
			rwnx_vif->sta.tdls_sta->tdls.chsw_en = false;

		netdev_info(dev, "Del TDLS sta %d (%pM)",
				rwnx_vif->sta.tdls_sta->sta_idx,
				rwnx_vif->sta.tdls_sta->mac_addr);
		/* Ensure that we won't process PS change ind */
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_vif->sta.tdls_sta->ps.active = false;
		rwnx_vif->sta.tdls_sta->valid = false;
		spin_unlock_bh(&rwnx_hw->cb_lock);
		rwnx_txq_sta_deinit(rwnx_hw, rwnx_vif->sta.tdls_sta);
		error = rwnx_send_me_sta_del(rwnx_hw, rwnx_vif->sta.tdls_sta->sta_idx, true);
		if ((error != 0) && (error != -EPIPE))
			return error;

#ifdef CONFIG_RWNX_BFMER
			// Disable Beamformer if supported
			rwnx_bfmer_report_del(rwnx_hw, rwnx_vif->sta.tdls_sta);
			rwnx_mu_group_sta_del(rwnx_hw, rwnx_vif->sta.tdls_sta);
#endif /* CONFIG_RWNX_BFMER */

		/* Set TDLS not active */
		rwnx_vif->sta.tdls_sta->tdls.active = false;
#ifdef CONFIG_DEBUG_FS
		rwnx_dbgfs_unregister_rc_stat(rwnx_hw, rwnx_vif->sta.tdls_sta);
#endif
		// Remove TDLS station
		rwnx_vif->tdls_status = TDLS_LINK_IDLE;
		rwnx_vif->sta.tdls_sta = NULL;
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
/*
 * @tdls_channel_switch: enable TDLS channel switch
 */
static int
rwnx_cfg80211_tdls_channel_switch (struct wiphy *wiphy,
									  struct net_device *dev,
									  const u8 *addr, u8 oper_class,
									  struct cfg80211_chan_def *chandef)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_sta *rwnx_sta = rwnx_vif->sta.tdls_sta;
	struct tdls_chan_switch_cfm cfm;
	int error;

	if ((!rwnx_sta) || (memcmp(addr, rwnx_sta->mac_addr, ETH_ALEN))) {
		printk("%s: TDLS station %pM doesn't exist\n", __func__, addr);
		return -ENOLINK;
	}

	if (!rwnx_sta->tdls.chsw_allowed) {
		printk("%s: TDLS station %pM does not support TDLS channel switch\n", __func__, addr);
		return -ENOTSUPP;
	}

	error = rwnx_send_tdls_chan_switch_req(rwnx_hw, rwnx_vif, rwnx_sta,
										   rwnx_sta->tdls.initiator,
										   oper_class, chandef, &cfm);
	if (error)
		return error;

	if (!cfm.status) {
		rwnx_sta->tdls.chsw_en = true;
		return 0;
	} else {
		printk("%s: TDLS channel switch already enabled and only one is supported\n", __func__);
		return -EALREADY;
	}
}

/*
 * @tdls_cancel_channel_switch: disable TDLS channel switch
 */
static void
rwnx_cfg80211_tdls_cancel_channel_switch (struct wiphy *wiphy,
											  struct net_device *dev,
											  const u8 *addr)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_sta *rwnx_sta = rwnx_vif->sta.tdls_sta;
	struct tdls_cancel_chan_switch_cfm cfm;

	if (!rwnx_sta)
		return;

	if (!rwnx_send_tdls_cancel_chan_switch_req(rwnx_hw, rwnx_vif,
											   rwnx_sta, &cfm))
		rwnx_sta->tdls.chsw_en = false;
}
#endif /* version >= 3.19 */

/**
 * @change_bss: Modify parameters for a given BSS (mainly for AP mode).
 */
int rwnx_cfg80211_change_bss(struct wiphy *wiphy, struct net_device *dev,
							 struct bss_parameters *params)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	int res =  -EOPNOTSUPP;

	if (((RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_AP) ||
		 (RWNX_VIF_TYPE(rwnx_vif) == NL80211_IFTYPE_P2P_GO)) &&
		(params->ap_isolate > -1)) {

		if (params->ap_isolate)
			rwnx_vif->ap.flags |= RWNX_AP_ISOLATE;
		else
			rwnx_vif->ap.flags &= ~RWNX_AP_ISOLATE;

		res = 0;
	}

	return res;
}

static int rwnx_fill_station_info(struct rwnx_sta *sta, struct rwnx_vif *vif,
								  struct station_info *sinfo)
{
	struct rwnx_sta_stats *stats = &sta->stats;
	struct rx_vector_1 *rx_vect1 = &stats->last_rx.rx_vect1;
	union rwnx_rate_ctrl_info *rate_info;
	struct mm_get_sta_info_cfm cfm;

	rwnx_send_get_sta_info_req(vif->rwnx_hw, sta->sta_idx, &cfm);
	sinfo->tx_failed = cfm.txfailed;
	rate_info = (union rwnx_rate_ctrl_info *)&cfm.rate_info;
	switch (rate_info->formatModTx) {
	case FORMATMOD_NON_HT:
	case FORMATMOD_NON_HT_DUP_OFDM:
		sinfo->txrate.flags = 0;
		sinfo->txrate.legacy = tx_legrates_lut_rate[rate_info->mcsIndexTx];
		break;
	case FORMATMOD_HT_MF:
	case FORMATMOD_HT_GF:
		sinfo->txrate.flags = RATE_INFO_FLAGS_MCS;
		sinfo->txrate.mcs = rate_info->mcsIndexTx;
		if (rate_info->giAndPreTypeTx)
			sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
	case FORMATMOD_VHT:
		sinfo->txrate.flags = RATE_INFO_FLAGS_VHT_MCS;
		sinfo->txrate.mcs = rate_info->mcsIndexTx;
		if (rate_info->giAndPreTypeTx)
			sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
		break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	case FORMATMOD_HE_MU:
	case FORMATMOD_HE_SU:
	case FORMATMOD_HE_ER:
		sinfo->txrate.flags = RATE_INFO_FLAGS_HE_MCS;
		sinfo->txrate.mcs = rate_info->mcsIndexTx;
		sinfo->txrate.he_gi = rate_info->giAndPreTypeTx;
		break;
#else
	case FORMATMOD_HE_MU:
	case FORMATMOD_HE_SU:
	case FORMATMOD_HE_ER:
		sinfo->txrate.flags = RATE_INFO_FLAGS_VHT_MCS;
		if (rate_info->mcsIndexTx > 9) {
			sinfo->txrate.mcs = 9;
		} else {
			sinfo->txrate.mcs = rate_info->mcsIndexTx;
		}
		break;
#endif
	default:
		return -EINVAL;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	switch (rate_info->bwTx) {
	case PHY_CHNL_BW_20:
		sinfo->txrate.bw = RATE_INFO_BW_20;
		break;
	case PHY_CHNL_BW_40:
		sinfo->txrate.bw = RATE_INFO_BW_40;
		break;
	case PHY_CHNL_BW_80:
		sinfo->txrate.bw = RATE_INFO_BW_80;
		break;
	case PHY_CHNL_BW_160:
		sinfo->txrate.bw = RATE_INFO_BW_160;
		break;
	default:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
		sinfo->txrate.bw = RATE_INFO_BW_HE_RU;
#else
		sinfo->txrate.bw = RATE_INFO_BW_20;
#endif
		break;
	}
#endif

	sinfo->txrate.nss = 1;
	sinfo->filled |= (BIT(NL80211_STA_INFO_TX_BITRATE) | BIT(NL80211_STA_INFO_TX_FAILED));

	sinfo->inactive_time = jiffies_to_msecs(jiffies - vif->rwnx_hw->stats.last_tx);
	sinfo->rx_bytes = vif->net_stats.rx_bytes;
	sinfo->tx_bytes = vif->net_stats.tx_bytes;
	sinfo->tx_packets = vif->net_stats.tx_packets;
	sinfo->rx_packets = vif->net_stats.rx_packets;
	sinfo->signal = (s8)cfm.rssi;
	sinfo->rxrate.nss = 1;

	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
	switch (rx_vect1->ch_bw) {
	case PHY_CHNL_BW_20:
		sinfo->rxrate.bw = RATE_INFO_BW_20;
		break;
	case PHY_CHNL_BW_40:
		sinfo->rxrate.bw = RATE_INFO_BW_40;
		break;
	case PHY_CHNL_BW_80:
		sinfo->rxrate.bw = RATE_INFO_BW_80;
		break;
	case PHY_CHNL_BW_160:
		sinfo->rxrate.bw = RATE_INFO_BW_160;
		break;
	default:
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
		sinfo->rxrate.bw = RATE_INFO_BW_HE_RU;
	#else
		sinfo->rxrate.bw = RATE_INFO_BW_20;
	#endif
		break;
	}
	#endif

	switch (rx_vect1->format_mod) {
	case FORMATMOD_NON_HT:
	case FORMATMOD_NON_HT_DUP_OFDM:
		sinfo->rxrate.flags = 0;
		sinfo->rxrate.legacy = legrates_lut_rate[legrates_lut[rx_vect1->leg_rate]];
		break;
	case FORMATMOD_HT_MF:
	case FORMATMOD_HT_GF:
		sinfo->rxrate.flags = RATE_INFO_FLAGS_MCS;
		if (rx_vect1->ht.short_gi)
			sinfo->rxrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
		sinfo->rxrate.mcs = rx_vect1->ht.mcs;
		break;
	case FORMATMOD_VHT:
		sinfo->rxrate.flags = RATE_INFO_FLAGS_VHT_MCS;
		if (rx_vect1->vht.short_gi)
			sinfo->rxrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
		sinfo->rxrate.mcs = rx_vect1->vht.mcs;
		break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	case FORMATMOD_HE_MU:
		sinfo->rxrate.he_ru_alloc = rx_vect1->he.ru_size;
	case FORMATMOD_HE_SU:
	case FORMATMOD_HE_ER:
		sinfo->rxrate.flags = RATE_INFO_FLAGS_HE_MCS;
		sinfo->rxrate.mcs = rx_vect1->he.mcs;
		sinfo->rxrate.he_gi = rx_vect1->he.gi_type;
		sinfo->rxrate.he_dcm = rx_vect1->he.dcm;
		break;
#else
	//kernel not support he
	case FORMATMOD_HE_MU:
	case FORMATMOD_HE_SU:
	case FORMATMOD_HE_ER:
		sinfo->rxrate.flags = RATE_INFO_FLAGS_VHT_MCS;
		if (rx_vect1->he.mcs > 9) {
			sinfo->rxrate.mcs = 9;
		} else {
			sinfo->rxrate.mcs = rx_vect1->he.mcs;
		}
		break;
#endif
	default:
		return -EINVAL;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
	sinfo->filled |= (STATION_INFO_INACTIVE_TIME |
					 STATION_INFO_RX_BYTES64 |
					 STATION_INFO_TX_BYTES64 |
					 STATION_INFO_RX_PACKETS |
					 STATION_INFO_TX_PACKETS |
					 STATION_INFO_SIGNAL |
					 STATION_INFO_RX_BITRATE);
#else
	sinfo->filled |= (BIT(NL80211_STA_INFO_INACTIVE_TIME) |
					 BIT(NL80211_STA_INFO_RX_BYTES64)    |
					 BIT(NL80211_STA_INFO_TX_BYTES64)    |
					 BIT(NL80211_STA_INFO_RX_PACKETS)    |
					 BIT(NL80211_STA_INFO_TX_PACKETS)    |
					 BIT(NL80211_STA_INFO_SIGNAL)        |
					 BIT(NL80211_STA_INFO_RX_BITRATE));
#endif

	return 0;
}


/**
 * @get_station: get station information for the station identified by @mac
 */
static int rwnx_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
									 const u8 *mac, struct station_info *sinfo)
{
	struct rwnx_vif *vif = netdev_priv(dev);
	struct rwnx_sta *sta = NULL;

	if (RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_MONITOR)
		return -EINVAL;
	else if ((RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_STATION) ||
			 (RWNX_VIF_TYPE(vif) == NL80211_IFTYPE_P2P_CLIENT)) {
		if (vif->sta.ap && ether_addr_equal(vif->sta.ap->mac_addr, mac))
			sta = vif->sta.ap;
	} else {
		struct rwnx_sta *sta_iter;
		list_for_each_entry(sta_iter, &vif->ap.sta_list, list) {
			if (sta_iter->valid && ether_addr_equal(sta_iter->mac_addr, mac)) {
				sta = sta_iter;
				break;
			}
		}
	}

	if (sta)
		return rwnx_fill_station_info(sta, vif, sinfo);

	return -ENOENT;
}


/**
 * @dump_station: dump station callback -- resume dump at index @idx
 */
static int rwnx_cfg80211_dump_station(struct wiphy *wiphy, struct net_device *dev,
									  int idx, u8 *mac, struct station_info *sinfo)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_sta *sta_iter, *sta = NULL;
	struct mesh_peer_info_cfm peer_info_cfm;
	int i = 0;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	list_for_each_entry(sta_iter, &rwnx_vif->ap.sta_list, list) {
		if (i < idx) {
			i++;
			continue;
		}

		sta = sta_iter;
		break;
	}

	if (sta == NULL)
		return -ENOENT;

	/* Forward the information to the UMAC */
	if (rwnx_send_mesh_peer_info_req(rwnx_hw, rwnx_vif, sta->sta_idx,
									 &peer_info_cfm))
		return -ENOMEM;

	/* Copy peer MAC address */
	memcpy(mac, &sta->mac_addr, ETH_ALEN);

	/* Fill station information */
	sinfo->llid = peer_info_cfm.local_link_id;
	sinfo->plid = peer_info_cfm.peer_link_id;
	sinfo->plink_state = peer_info_cfm.link_state;
	sinfo->local_pm = peer_info_cfm.local_ps_mode;
	sinfo->peer_pm = peer_info_cfm.peer_ps_mode;
	sinfo->nonpeer_pm = peer_info_cfm.non_peer_ps_mode;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
	sinfo->filled = (STATION_INFO_LLID |
					 STATION_INFO_PLID |
					 STATION_INFO_PLINK_STATE |
					 STATION_INFO_LOCAL_PM |
					 STATION_INFO_PEER_PM |
					 STATION_INFO_NONPEER_PM);
#else
	sinfo->filled = (BIT(NL80211_STA_INFO_LLID) |
					 BIT(NL80211_STA_INFO_PLID) |
					 BIT(NL80211_STA_INFO_PLINK_STATE) |
					 BIT(NL80211_STA_INFO_LOCAL_PM) |
					 BIT(NL80211_STA_INFO_PEER_PM) |
					 BIT(NL80211_STA_INFO_NONPEER_PM));
#endif

	return 0;
}

/**
 * @add_mpath: add a fixed mesh path
 */
static int rwnx_cfg80211_add_mpath(struct wiphy *wiphy, struct net_device *dev,
								   const u8 *dst, const u8 *next_hop)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct mesh_path_update_cfm cfm;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	return rwnx_send_mesh_path_update_req(rwnx_hw, rwnx_vif, dst, next_hop, &cfm);
}

/**
 * @del_mpath: delete a given mesh path
 */
static int rwnx_cfg80211_del_mpath(struct wiphy *wiphy, struct net_device *dev,
								   const u8 *dst)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct mesh_path_update_cfm cfm;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	return rwnx_send_mesh_path_update_req(rwnx_hw, rwnx_vif, dst, NULL, &cfm);
}

/**
 * @change_mpath: change a given mesh path
 */
static int rwnx_cfg80211_change_mpath(struct wiphy *wiphy, struct net_device *dev,
									  const u8 *dst, const u8 *next_hop)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct mesh_path_update_cfm cfm;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	return rwnx_send_mesh_path_update_req(rwnx_hw, rwnx_vif, dst, next_hop, &cfm);
}

/**
 * @get_mpath: get a mesh path for the given parameters
 */
static int rwnx_cfg80211_get_mpath(struct wiphy *wiphy, struct net_device *dev,
								   u8 *dst, u8 *next_hop, struct mpath_info *pinfo)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_mesh_path *mesh_path = NULL;
	struct rwnx_mesh_path *cur;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	list_for_each_entry(cur, &rwnx_vif->ap.mpath_list, list) {
		/* Compare the path target address and the provided destination address */
		if (memcmp(dst, &cur->tgt_mac_addr, ETH_ALEN)) {
			continue;
		}

		mesh_path = cur;
		break;
	}

	if (mesh_path == NULL)
		return -ENOENT;

	/* Copy next HOP MAC address */
	if (mesh_path->p_nhop_sta)
		memcpy(next_hop, &mesh_path->p_nhop_sta->mac_addr, ETH_ALEN);

	/* Fill path information */
	pinfo->filled = 0;
	pinfo->generation = rwnx_vif->ap.generation;

	return 0;
}

/**
 * @dump_mpath: dump mesh path callback -- resume dump at index @idx
 */
static int rwnx_cfg80211_dump_mpath(struct wiphy *wiphy, struct net_device *dev,
									int idx, u8 *dst, u8 *next_hop,
									struct mpath_info *pinfo)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_mesh_path *mesh_path = NULL;
	struct rwnx_mesh_path *cur;
	int i = 0;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	list_for_each_entry(cur, &rwnx_vif->ap.mpath_list, list) {
		if (i < idx) {
			i++;
			continue;
		}

		mesh_path = cur;
		break;
	}

	if (mesh_path == NULL)
		return -ENOENT;

	/* Copy target and next hop MAC address */
	memcpy(dst, &mesh_path->tgt_mac_addr, ETH_ALEN);
	if (mesh_path->p_nhop_sta)
		memcpy(next_hop, &mesh_path->p_nhop_sta->mac_addr, ETH_ALEN);

	/* Fill path information */
	pinfo->filled = 0;
	pinfo->generation = rwnx_vif->ap.generation;

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
/**
 * @get_mpp: get a mesh proxy path for the given parameters
 */
static int rwnx_cfg80211_get_mpp(struct wiphy *wiphy, struct net_device *dev,
								 u8 *dst, u8 *mpp, struct mpath_info *pinfo)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_mesh_proxy *mesh_proxy = NULL;
	struct rwnx_mesh_proxy *cur;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	list_for_each_entry(cur, &rwnx_vif->ap.proxy_list, list) {
		if (cur->local) {
			continue;
		}

		/* Compare the path target address and the provided destination address */
		if (memcmp(dst, &cur->ext_sta_addr, ETH_ALEN)) {
			continue;
		}

		mesh_proxy = cur;
		break;
	}

	if (mesh_proxy == NULL)
		return -ENOENT;

	memcpy(mpp, &mesh_proxy->proxy_addr, ETH_ALEN);

	/* Fill path information */
	pinfo->filled = 0;
	pinfo->generation = rwnx_vif->ap.generation;

	return 0;
}

/**
 * @dump_mpp: dump mesh proxy path callback -- resume dump at index @idx
 */
static int rwnx_cfg80211_dump_mpp(struct wiphy *wiphy, struct net_device *dev,
								  int idx, u8 *dst, u8 *mpp, struct mpath_info *pinfo)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_mesh_proxy *mesh_proxy = NULL;
	struct rwnx_mesh_proxy *cur;
	int i = 0;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	list_for_each_entry(cur, &rwnx_vif->ap.proxy_list, list) {
		if (cur->local) {
			continue;
		}

		if (i < idx) {
			i++;
			continue;
		}

		mesh_proxy = cur;
		break;
	}

	if (mesh_proxy == NULL)
		return -ENOENT;

	/* Copy target MAC address */
	memcpy(dst, &mesh_proxy->ext_sta_addr, ETH_ALEN);
	memcpy(mpp, &mesh_proxy->proxy_addr, ETH_ALEN);

	/* Fill path information */
	pinfo->filled = 0;
	pinfo->generation = rwnx_vif->ap.generation;

	return 0;
}
#endif /* version >= 3.19 */

/**
 * @get_mesh_config: Get the current mesh configuration
 */
static int rwnx_cfg80211_get_mesh_config(struct wiphy *wiphy, struct net_device *dev,
										 struct mesh_config *conf)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	return 0;
}

/**
 * @update_mesh_config: Update mesh parameters on a running mesh.
 */
static int rwnx_cfg80211_update_mesh_config(struct wiphy *wiphy, struct net_device *dev,
											u32 mask, const struct mesh_config *nconf)
{
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct mesh_update_cfm cfm;
	int status;

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	if (mask & CO_BIT(NL80211_MESHCONF_POWER_MODE - 1)) {
		rwnx_vif->ap.next_mesh_pm = nconf->power_mode;

		if (!list_empty(&rwnx_vif->ap.sta_list)) {
			// If there are mesh links we don't want to update the power mode
			// It will be updated with rwnx_update_mesh_power_mode() when the
			// ps mode of a link is updated or when a new link is added/removed
			mask &= ~BIT(NL80211_MESHCONF_POWER_MODE - 1);

			if (!mask)
				return 0;
		}
	}

	status = rwnx_send_mesh_update_req(rwnx_hw, rwnx_vif, mask, nconf, &cfm);

	if (!status && (cfm.status != 0))
		status = -EINVAL;

	return status;
}

/**
 * @join_mesh: join the mesh network with the specified parameters
 * (invoked with the wireless_dev mutex held)
 */
static int rwnx_cfg80211_join_mesh(struct wiphy *wiphy, struct net_device *dev,
								   const struct mesh_config *conf, const struct mesh_setup *setup)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct mesh_start_cfm mesh_start_cfm;
	int error = 0;
	u8 txq_status = 0;
	/* STA for BC/MC traffic */
	struct rwnx_sta *sta;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	if (RWNX_VIF_TYPE(rwnx_vif) != NL80211_IFTYPE_MESH_POINT)
		return -ENOTSUPP;

	/* Forward the information to the UMAC */
	error = rwnx_send_mesh_start_req(rwnx_hw, rwnx_vif, conf, setup, &mesh_start_cfm);
	if (error) {
		return error;
	}

	/* Check the status */
	switch (mesh_start_cfm.status) {
	case CO_OK:
		rwnx_vif->ap.bcmc_index = mesh_start_cfm.bcmc_idx;
		rwnx_vif->ap.flags = 0;
		rwnx_vif->use_4addr = true;
		rwnx_vif->user_mpm = setup->user_mpm;

		sta = &rwnx_hw->sta_table[mesh_start_cfm.bcmc_idx];
		sta->valid = true;
		sta->aid = 0;
		sta->sta_idx = mesh_start_cfm.bcmc_idx;
		sta->ch_idx = mesh_start_cfm.ch_idx;
		sta->vif_idx = rwnx_vif->vif_index;
		sta->qos = true;
		sta->acm = 0;
		sta->ps.active = false;
		rwnx_mu_group_sta_init(sta, NULL);
		spin_lock_bh(&rwnx_hw->cb_lock);
		rwnx_chanctx_link(rwnx_vif, mesh_start_cfm.ch_idx,
						  (struct cfg80211_chan_def *)(&setup->chandef));
		if (rwnx_hw->cur_chanctx != mesh_start_cfm.ch_idx) {
			txq_status = RWNX_TXQ_STOP_CHAN;
		}
		rwnx_txq_vif_init(rwnx_hw, rwnx_vif, txq_status);
		spin_unlock_bh(&rwnx_hw->cb_lock);

		netif_tx_start_all_queues(dev);
		netif_carrier_on(dev);

		/* If the AP channel is already the active, we probably skip radar
		   activation on MM_CHANNEL_SWITCH_IND (unless another vif use this
		   ctxt). In anycase retest if radar detection must be activated
		 */
		if (rwnx_hw->cur_chanctx == mesh_start_cfm.ch_idx) {
			rwnx_radar_detection_enable_on_cur_channel(rwnx_hw);
		}
		break;

	case CO_BUSY:
		error = -EINPROGRESS;
		break;

	default:
		error = -EIO;
		break;
	}

	/* Print information about the operation */
	if (error) {
		netdev_info(dev, "Failed to start MP (%d)", error);
	} else {
		netdev_info(dev, "MP started: ch=%d, bcmc_idx=%d",
					rwnx_vif->ch_index, rwnx_vif->ap.bcmc_index);
	}

	return error;
}

/**
 * @leave_mesh: leave the current mesh network
 * (invoked with the wireless_dev mutex held)
 */
static int rwnx_cfg80211_leave_mesh(struct wiphy *wiphy, struct net_device *dev)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);
	struct rwnx_vif *rwnx_vif = netdev_priv(dev);
	struct mesh_stop_cfm mesh_stop_cfm;
	int error = 0;

	error = rwnx_send_mesh_stop_req(rwnx_hw, rwnx_vif, &mesh_stop_cfm);

	if (error == 0) {
		/* Check the status */
		switch (mesh_stop_cfm.status) {
		case CO_OK:
			spin_lock_bh(&rwnx_hw->cb_lock);
			rwnx_chanctx_unlink(rwnx_vif);
			rwnx_radar_cancel_cac(&rwnx_hw->radar);
			spin_unlock_bh(&rwnx_hw->cb_lock);
			/* delete BC/MC STA */
			rwnx_txq_vif_deinit(rwnx_hw, rwnx_vif);
			rwnx_del_bcn(&rwnx_vif->ap.bcn);

			netif_tx_stop_all_queues(dev);
			netif_carrier_off(dev);

			break;

		default:
			error = -EIO;
			break;
		}
	}

	if (error) {
		netdev_info(dev, "Failed to stop MP");
	} else {
		netdev_info(dev, "MP Stopped");
	}

	return 0;
}

static struct cfg80211_ops rwnx_cfg80211_ops = {
	.add_virtual_intf = rwnx_cfg80211_add_iface,
	.del_virtual_intf = rwnx_cfg80211_del_iface,
	.change_virtual_intf = rwnx_cfg80211_change_iface,
	.start_p2p_device = rwnx_cfgp2p_start_p2p_device,
	.stop_p2p_device = rwnx_cfgp2p_stop_p2p_device,
	.scan = rwnx_cfg80211_scan,
	.connect = rwnx_cfg80211_connect,
	.disconnect = rwnx_cfg80211_disconnect,
	.add_key = rwnx_cfg80211_add_key,
	.get_key = rwnx_cfg80211_get_key,
	.del_key = rwnx_cfg80211_del_key,
	.set_default_key = rwnx_cfg80211_set_default_key,
	.set_default_mgmt_key = rwnx_cfg80211_set_default_mgmt_key,
	.add_station = rwnx_cfg80211_add_station,
	.del_station = rwnx_cfg80211_del_station_compat,
	.change_station = rwnx_cfg80211_change_station,
	.mgmt_tx = rwnx_cfg80211_mgmt_tx,
	.start_ap = rwnx_cfg80211_start_ap,
	.change_beacon = rwnx_cfg80211_change_beacon,
	.stop_ap = rwnx_cfg80211_stop_ap,
	.set_monitor_channel = rwnx_cfg80211_set_monitor_channel,
	.probe_client = rwnx_cfg80211_probe_client,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0)
	.mgmt_frame_register = rwnx_cfg80211_mgmt_frame_register,
#endif
	.set_wiphy_params = rwnx_cfg80211_set_wiphy_params,
	.set_txq_params = rwnx_cfg80211_set_txq_params,
	.set_tx_power = rwnx_cfg80211_set_tx_power,
//    .get_tx_power = rwnx_cfg80211_get_tx_power,
	.set_power_mgmt = rwnx_cfg80211_set_power_mgmt,
	.get_station = rwnx_cfg80211_get_station,
	.remain_on_channel = rwnx_cfg80211_remain_on_channel,
	.cancel_remain_on_channel = rwnx_cfg80211_cancel_remain_on_channel,
	.dump_survey = rwnx_cfg80211_dump_survey,
	.get_channel = rwnx_cfg80211_get_channel,
	.start_radar_detection = rwnx_cfg80211_start_radar_detection,
	.update_ft_ies = rwnx_cfg80211_update_ft_ies,
	.set_cqm_rssi_config = rwnx_cfg80211_set_cqm_rssi_config,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	.channel_switch = rwnx_cfg80211_channel_switch,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	.tdls_channel_switch = rwnx_cfg80211_tdls_channel_switch,
	.tdls_cancel_channel_switch = rwnx_cfg80211_tdls_cancel_channel_switch,
#endif
	.tdls_mgmt = rwnx_cfg80211_tdls_mgmt,
	.tdls_oper = rwnx_cfg80211_tdls_oper,
	.change_bss = rwnx_cfg80211_change_bss,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0) || defined(CONFIG_WPA3_FOR_OLD_KERNEL)
	.external_auth = rwnx_cfg80211_external_auth,
#endif
#ifdef CONFIG_SCHED_SCAN
	.sched_scan_start = rwnx_cfg80211_sched_scan_start,
	.sched_scan_stop = rwnx_cfg80211_sched_scan_stop,
#endif
};


/*********************************************************************
 * Init/Exit functions
 *********************************************************************/
static void rwnx_wdev_unregister(struct rwnx_hw *rwnx_hw)
{
	struct rwnx_vif *rwnx_vif, *tmp;

	rtnl_lock();
	list_for_each_entry_safe(rwnx_vif, tmp, &rwnx_hw->vifs, list) {
		rwnx_cfg80211_del_iface(rwnx_hw->wiphy, &rwnx_vif->wdev);
	}
	rtnl_unlock();
}

static void rwnx_set_vers(struct rwnx_hw *rwnx_hw)
{
	u32 vers = rwnx_hw->version_cfm.version_lmac;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

	snprintf(rwnx_hw->wiphy->fw_version,
			 sizeof(rwnx_hw->wiphy->fw_version), "%d.%d.%d.%d",
			 (vers & (0xff << 24)) >> 24, (vers & (0xff << 16)) >> 16,
			 (vers & (0xff <<  8)) >>  8, (vers & (0xff <<  0)) >>  0);
}

static void rwnx_reg_notifier(struct wiphy *wiphy,
							  struct regulatory_request *request)
{
	struct rwnx_hw *rwnx_hw = wiphy_priv(wiphy);

	// For now trust all initiator
	rwnx_radar_set_domain(&rwnx_hw->radar, request->dfs_region);
	rwnx_send_me_chan_config_req(rwnx_hw);
}

static void rwnx_enable_mesh(struct rwnx_hw *rwnx_hw)
{
	struct wiphy *wiphy = rwnx_hw->wiphy;

	if (!rwnx_mod_params.mesh)
		return;

	rwnx_cfg80211_ops.get_station = rwnx_cfg80211_get_station;
	rwnx_cfg80211_ops.dump_station = rwnx_cfg80211_dump_station;
	rwnx_cfg80211_ops.add_mpath = rwnx_cfg80211_add_mpath;
	rwnx_cfg80211_ops.del_mpath = rwnx_cfg80211_del_mpath;
	rwnx_cfg80211_ops.change_mpath = rwnx_cfg80211_change_mpath;
	rwnx_cfg80211_ops.get_mpath = rwnx_cfg80211_get_mpath;
	rwnx_cfg80211_ops.dump_mpath = rwnx_cfg80211_dump_mpath;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
	rwnx_cfg80211_ops.get_mpp = rwnx_cfg80211_get_mpp;
	rwnx_cfg80211_ops.dump_mpp = rwnx_cfg80211_dump_mpp;
#endif
	rwnx_cfg80211_ops.get_mesh_config = rwnx_cfg80211_get_mesh_config;
	rwnx_cfg80211_ops.update_mesh_config = rwnx_cfg80211_update_mesh_config;
	rwnx_cfg80211_ops.join_mesh = rwnx_cfg80211_join_mesh;
	rwnx_cfg80211_ops.leave_mesh = rwnx_cfg80211_leave_mesh;

	wiphy->flags |= (WIPHY_FLAG_MESH_AUTH | WIPHY_FLAG_IBSS_RSN);
	wiphy->features |= NL80211_FEATURE_USERSPACE_MPM;
	wiphy->interface_modes |= BIT(NL80211_IFTYPE_MESH_POINT);

	rwnx_limits[0].types |= BIT(NL80211_IFTYPE_MESH_POINT);
	rwnx_limits_dfs[0].types |= BIT(NL80211_IFTYPE_MESH_POINT);
}

extern int rwnx_init_aic(struct rwnx_hw *rwnx_hw);

#if IS_ENABLED(CONFIG_AW_MACADDR_MGT) || IS_ENABLED(CONFIG_SUNXI_ADDR_MGT)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
extern int get_custom_mac_address(int fmt, char *name, char *addr);
#else
extern int get_wifi_custom_mac_address(char *addr_str);
#endif
#endif
#if IS_ENABLED(CONFIG_PM)
static const struct wiphy_wowlan_support aic_wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY | WIPHY_WOWLAN_MAGIC_PKT,
};
#endif
/**
 *
 */
extern int aicwf_vendor_init(struct wiphy *wiphy);

int rwnx_cfg80211_init(struct rwnx_plat *rwnx_plat, void **platform_data)
{
	struct rwnx_hw *rwnx_hw;
	struct rwnx_conf_file init_conf;
	int ret = 0;
	struct wiphy *wiphy;
	struct rwnx_vif *vif;
	int i;
	u8 dflt_mac[ETH_ALEN] = { 0x88, 0x00, 0x33, 0x77, 0x10, 0x99};
	u8 addr_str[20];
	struct mm_set_rf_calib_cfm cfm;
	struct mm_get_fw_version_cfm fw_version;
	u8_l mac_addr_efuse[ETH_ALEN];
	struct aicbsp_feature_t feature;
	struct mm_set_stack_start_cfm set_start_cfm;
	(void)addr_str;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

#ifdef AICWF_BSP_CTRL
	aicbsp_get_feature(&feature);
#endif
	get_random_bytes(&dflt_mac[4], 2);
	/* create a new wiphy for use with cfg80211 */
	printk("%s sizeof(struct rwnx_hw):%d \r\n", __func__, (int)sizeof(struct rwnx_hw));
	wiphy = wiphy_new(&rwnx_cfg80211_ops, sizeof(struct rwnx_hw));

	if (!wiphy) {
		dev_err(rwnx_platform_get_dev(rwnx_plat), "Failed to create new wiphy\n");
		ret = -ENOMEM;
		goto err_out;
	}

	rwnx_hw = wiphy_priv(wiphy);
	rwnx_hw->wiphy = wiphy;
	rwnx_hw->plat = rwnx_plat;
	rwnx_hw->dev = rwnx_platform_get_dev(rwnx_plat);
	rwnx_hw->chipid = feature.chipinfo->chipid;
	rwnx_hw->rev = feature.chipinfo->rev;
	rwnx_hw->subrev = feature.chipinfo->subrev;
#ifdef AICWF_SDIO_SUPPORT
	rwnx_hw->sdiodev = rwnx_plat->sdiodev;
	rwnx_plat->sdiodev->rwnx_hw = rwnx_hw;
	rwnx_hw->cmd_mgr = &rwnx_plat->sdiodev->cmd_mgr;
#else
	rwnx_hw->usbdev = rwnx_plat->usbdev;
	rwnx_plat->usbdev->rwnx_hw = rwnx_hw;
	rwnx_hw->cmd_mgr = &rwnx_plat->usbdev->cmd_mgr;
#endif
	rwnx_hw->mod_params = &rwnx_mod_params;
	rwnx_hw->tcp_pacing_shift = 7;

#ifdef CONFIG_SCHED_SCAN
	rwnx_hw->is_sched_scan = false;
#endif //CONFIG_SCHED_SCAN

	aicwf_wakeup_lock_init(rwnx_hw);
	rwnx_init_aic(rwnx_hw);
	/* set device pointer for wiphy */
	set_wiphy_dev(wiphy, rwnx_hw->dev);

	/* Create cache to allocate sw_txhdr */
	rwnx_hw->sw_txhdr_cache = KMEM_CACHE(rwnx_sw_txhdr, 0);
	if (!rwnx_hw->sw_txhdr_cache) {
		wiphy_err(wiphy, "Cannot allocate cache for sw TX header\n");
		ret = -ENOMEM;
		goto err_cache;
	}

	memcpy(init_conf.mac_addr, dflt_mac, ETH_ALEN);

	rwnx_hw->vif_started = 0;
	rwnx_hw->monitor_vif = RWNX_INVALID_VIF;
	rwnx_hw->adding_sta = false;

	rwnx_hw->scan_ie.addr = NULL;

	for (i = 0; i < NX_VIRT_DEV_MAX + NX_REMOTE_STA_MAX; i++)
		rwnx_hw->avail_idx_map |= BIT(i);

	rwnx_hwq_init(rwnx_hw);

	for (i = 0; i < NX_NB_TXQ; i++) {
		rwnx_hw->txq[i].idx = TXQ_INACTIVE;
	}

	rwnx_mu_group_init(rwnx_hw);

	/* Initialize RoC element pointer to NULL, indicate that RoC can be started */
	rwnx_hw->roc_elem = NULL;
	/* Cookie can not be 0 */
	rwnx_hw->roc_cookie_cnt = 1;

	INIT_LIST_HEAD(&rwnx_hw->vifs);
	INIT_LIST_HEAD(&rwnx_hw->defrag_list);
	spin_lock_init(&rwnx_hw->defrag_lock);
	mutex_init(&rwnx_hw->mutex);
	mutex_init(&rwnx_hw->dbgdump_elem.mutex);
	spin_lock_init(&rwnx_hw->tx_lock);
	spin_lock_init(&rwnx_hw->cb_lock);

	INIT_WORK(&rwnx_hw->apmStalossWork, apm_staloss_work_process);
	rwnx_hw->apmStaloss_wq = create_singlethread_workqueue("apmStaloss_wq");
	if (!rwnx_hw->apmStaloss_wq) {
		txrx_err("insufficient memory to create apmStaloss workqueue.\n");
		goto err_cache;
	}

	wiphy->mgmt_stypes = rwnx_default_mgmt_stypes;

	rwnx_hw->fwlog_en = feature.fwlog_en;
	rwnx_hw->cpmode   = feature.cpmode;

	rwnx_hw->irq_enable = true;
#ifdef CONFIG_OOB
	if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
		u32 memdata_temp = 0x00000006;
		int ret_oob;
		struct dbg_mem_read_cfm rd_mem_addr_cfm;
		ret_oob = rwnx_send_dbg_mem_block_write_req(rwnx_hw, 0x40504084, 4, &memdata_temp);
		if (ret_oob) {
			sdio_err("[0x40504084] write fail: %d\n", ret_oob);
			goto err_lmac_reqs;
		}

		ret_oob = rwnx_send_dbg_mem_read_req(rwnx_hw, 0x40504084, &rd_mem_addr_cfm);
		if (ret_oob) {
			sdio_err("[0x40504084] rd fail\n");
			goto err_lmac_reqs;
		}
		sdio_info("rd [0x40504084] = %x\n", rd_mem_addr_cfm.memdata);
	}
#endif

	if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D) {
		ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, feature.hwinfo < 0, feature.hwinfo, feature.fwlog_en, &set_start_cfm);
	} else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
			rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) {
		ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, 0, 0, feature.fwlog_en, &set_start_cfm);
		set_start_cfm.is_5g_support = false;
	} else {
		ret = rwnx_send_set_stack_start_req(rwnx_hw, 1, 0, CO_BIT(5), feature.fwlog_en, &set_start_cfm);
	}
	if (ret)
		goto err_lmac_reqs;

	printk("is 5g support = %d, vendor_info = 0x%02X\n", set_start_cfm.is_5g_support, set_start_cfm.vendor_info);
	rwnx_hw->band_5g_support = set_start_cfm.is_5g_support;
	rwnx_hw->vendor_info = (feature.hwinfo < 0) ? set_start_cfm.vendor_info : feature.hwinfo;

	ret = rwnx_send_get_fw_version_req(rwnx_hw, &fw_version);
	memcpy(wiphy->fw_version, fw_version.fw_version, fw_version.fw_version_len > 32 ? 32 : fw_version.fw_version_len);
	printk("Firmware Version: %s", fw_version.fw_version);

	wiphy->bands[NL80211_BAND_2GHZ] = &rwnx_band_2GHz;
	if (rwnx_hw->band_5g_support)
		wiphy->bands[NL80211_BAND_5GHZ] = &rwnx_band_5GHz;

	wiphy->interface_modes =
	BIT(NL80211_IFTYPE_STATION)     |
	BIT(NL80211_IFTYPE_AP)          |
	BIT(NL80211_IFTYPE_AP_VLAN)     |
	BIT(NL80211_IFTYPE_P2P_CLIENT)  |
	BIT(NL80211_IFTYPE_P2P_GO)      |
	BIT(NL80211_IFTYPE_P2P_DEVICE)  |
	BIT(NL80211_IFTYPE_MONITOR);

#if IS_ENABLED(CONFIG_PM)
	/* Set WoWLAN flags */
	wiphy->wowlan = &aic_wowlan_support;
#endif
	wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL |
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
		WIPHY_FLAG_HAS_CHANNEL_SWITCH |
		#endif
		WIPHY_FLAG_4ADDR_STATION |
		WIPHY_FLAG_4ADDR_AP;

	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
	wiphy->max_num_csa_counters = BCN_MAX_CSA_CPT;
	#endif

	wiphy->max_remain_on_channel_duration = rwnx_hw->mod_params->roc_dur_max;

	wiphy->features |= NL80211_FEATURE_NEED_OBSS_SCAN |
		NL80211_FEATURE_SK_TX_STATUS |
		NL80211_FEATURE_VIF_TXPOWER |
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
		NL80211_FEATURE_ACTIVE_MONITOR |
		#endif
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
		NL80211_FEATURE_AP_MODE_CHAN_WIDTH_CHANGE |
		#endif
		0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0) || defined(CONFIG_WPA3_FOR_OLD_KERNEL)
	wiphy->features |= NL80211_FEATURE_SAE;
#endif

	if (rwnx_mod_params.tdls)
		/* TDLS support */
		wiphy->features |= NL80211_FEATURE_TDLS_CHANNEL_SWITCH;

	wiphy->iface_combinations   = rwnx_combinations;
	/* -1 not to include combination with radar detection, will be re-added in
	   rwnx_handle_dynparams if supported */
	wiphy->n_iface_combinations = ARRAY_SIZE(rwnx_combinations) - 1;
	wiphy->reg_notifier = rwnx_reg_notifier;

	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	//rwnx_enable_wapi(rwnx_hw);

	wiphy->cipher_suites = cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(cipher_suites) - NB_RESERVED_CIPHER;

	rwnx_hw->ext_capa[0] = WLAN_EXT_CAPA1_EXT_CHANNEL_SWITCHING;
	rwnx_hw->ext_capa[7] = WLAN_EXT_CAPA8_OPMODE_NOTIF;

	wiphy->extended_capabilities = rwnx_hw->ext_capa;
	wiphy->extended_capabilities_mask = rwnx_hw->ext_capa;
	wiphy->extended_capabilities_len = ARRAY_SIZE(rwnx_hw->ext_capa);

#ifdef CONFIG_SCHED_SCAN
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
	wiphy->max_sched_scan_reqs = 1;
#endif
	wiphy->max_sched_scan_ssids = SCAN_SSID_MAX; // 16;
	wiphy->max_match_sets = SCAN_SSID_MAX;       // 16;
	wiphy->max_sched_scan_ie_len = 2048;
#endif // CONFIG_SCHED_SCAN

	tasklet_init(&rwnx_hw->task, rwnx_task, (unsigned long)rwnx_hw);

	//system_config(rwnx_hw);

	ret = rwnx_platform_on(rwnx_hw, NULL);
	if (ret)
		goto err_platon;
	//patch_config(rwnx_hw);

#if defined(CONFIG_START_FROM_BOOTROM)
	//ret = start_from_bootrom(rwnx_hw);
	//if (ret)
	//	goto err_lmac_reqs;

	//rf_config(rwnx_hw);
#endif
	ret = rwnx_send_rf_calib_req(rwnx_hw, &cfm);
	if (ret)
		goto err_lmac_reqs;

	ret = rwnx_send_txpwr_idx_req(rwnx_hw);
	if (ret)
		goto err_lmac_reqs;

	if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) {
		ret = rwnx_send_txpwr_ofst2x_req(rwnx_hw);
	} else {
		ret = rwnx_send_txpwr_ofst_req(rwnx_hw);
	}
	if (ret)
		goto err_lmac_reqs;

#if IS_ENABLED(CONFIG_AW_MACADDR_MGT) || IS_ENABLED(CONFIG_SUNXI_ADDR_MGT)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
	ret = get_custom_mac_address(1, "wifi", mac_addr_efuse);
#else
	ret = get_wifi_custom_mac_address(addr_str);
	if (ret >= 0) {
		sscanf(addr_str, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
				&mac_addr_efuse[0], &mac_addr_efuse[1], &mac_addr_efuse[2],
				&mac_addr_efuse[3], &mac_addr_efuse[4], &mac_addr_efuse[5]);
	}
#endif
	if (ret < 0)
#endif
	{
		ret = rwnx_send_get_macaddr_req(rwnx_hw, (struct mm_get_mac_addr_cfm *)mac_addr_efuse);
		if (ret)
			goto err_lmac_reqs;
	}

	if (mac_addr_efuse[0] | mac_addr_efuse[1] | mac_addr_efuse[2] | mac_addr_efuse[3]) {
		memcpy(init_conf.mac_addr, mac_addr_efuse, ETH_ALEN);
	}

	printk("get macaddr: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
			mac_addr_efuse[0], mac_addr_efuse[1], mac_addr_efuse[2],
			mac_addr_efuse[3], mac_addr_efuse[4], mac_addr_efuse[5]);
	memcpy(wiphy->perm_addr, init_conf.mac_addr, ETH_ALEN);

	/* Reset FW */
	ret = rwnx_send_reset(rwnx_hw);
	if (ret)
		goto err_lmac_reqs;
	ret = rwnx_send_version_req(rwnx_hw, &rwnx_hw->version_cfm);
	if (ret)
		goto err_lmac_reqs;
	rwnx_set_vers(rwnx_hw);

	ret = rwnx_handle_dynparams(rwnx_hw, rwnx_hw->wiphy);
	if (ret)
		goto err_lmac_reqs;

	rwnx_enable_mesh(rwnx_hw);
	rwnx_radar_detection_init(&rwnx_hw->radar);

	/* Set parameters to firmware */
	rwnx_send_me_config_req(rwnx_hw);

	/* Only monitor mode supported when custom channels are enabled */
	if (rwnx_mod_params.custchan) {
		rwnx_limits[0].types = BIT(NL80211_IFTYPE_MONITOR);
		rwnx_limits_dfs[0].types = BIT(NL80211_IFTYPE_MONITOR);
	}

	aicwf_vendor_init(wiphy);

	ret = wiphy_register(wiphy);
	if (ret) {
		wiphy_err(wiphy, "Could not register wiphy device\n");
		goto err_register_wiphy;
	}

	/* Update regulatory (if needed) and set channel parameters to firmware
	   (must be done after WiPHY registration) */
	rwnx_custregd(rwnx_hw, wiphy);
	rwnx_send_me_chan_config_req(rwnx_hw);
	*platform_data = rwnx_hw;

#ifdef CONFIG_DEBUG_FS
	ret = rwnx_dbgfs_register(rwnx_hw, "rwnx");
	if (ret) {
		wiphy_err(wiphy, "Failed to register debugfs entries");
		goto err_debugfs;
	}
#endif
	rtnl_lock();

	/* Add an initial station interface */
	vif = rwnx_interface_add(rwnx_hw, "wlan%d", NET_NAME_UNKNOWN,
								NL80211_IFTYPE_STATION, NULL);

	rtnl_unlock();

	if (!vif) {
		wiphy_err(wiphy, "Failed to instantiate a network device\n");
		ret = -ENOMEM;
		goto err_add_interface;
	}

	wiphy_info(wiphy, "New interface create %s", vif->ndev->name);

	return 0;

err_add_interface:
#ifdef CONFIG_DEBUG_FS
	rwnx_dbgfs_unregister(rwnx_hw);
err_debugfs:
#endif
	wiphy_unregister(rwnx_hw->wiphy);
err_register_wiphy:
err_lmac_reqs:
	printk("err_lmac_reqs\n");
	rwnx_platform_off(rwnx_hw, NULL);
err_platon:
//err_config:
	kmem_cache_destroy(rwnx_hw->sw_txhdr_cache);
err_cache:
	aicwf_wakeup_lock_deinit(rwnx_hw);
	wiphy_free(wiphy);
err_out:
	return ret;
}

/**
 *
 */
void rwnx_cfg80211_deinit(struct rwnx_hw *rwnx_hw)
{
	struct mm_set_stack_start_cfm set_start_cfm;
	struct defrag_ctrl_info *defrag_ctrl = NULL;
	(void)set_start_cfm;

	RWNX_DBG(RWNX_FN_ENTRY_STR);

#ifdef AICWF_USB_SUPPORT
	if (rwnx_hw->usbdev->bus_if->state != BUS_DOWN_ST)
#else
	if (rwnx_hw->sdiodev->bus_if->state != BUS_DOWN_ST)
#endif
		rwnx_send_set_stack_start_req(rwnx_hw, 0, 0, 0, 0, &set_start_cfm);

	rwnx_hw->irq_enable = false;
	rwnx_hw->fwlog_en = 0;
	spin_lock_bh(&rwnx_hw->defrag_lock);
	if (!list_empty(&rwnx_hw->defrag_list)) {
		list_for_each_entry(defrag_ctrl, &rwnx_hw->defrag_list, list) {
			list_del_init(&defrag_ctrl->list);
			if (timer_pending(&defrag_ctrl->defrag_timer))
				del_timer_sync(&defrag_ctrl->defrag_timer);
			dev_kfree_skb(defrag_ctrl->skb);
			kfree(defrag_ctrl);
		}
	}
	spin_unlock_bh(&rwnx_hw->defrag_lock);
#ifdef CONFIG_DEBUG_FS
	rwnx_dbgfs_unregister(rwnx_hw);
#endif
	flush_workqueue(rwnx_hw->apmStaloss_wq);
	destroy_workqueue(rwnx_hw->apmStaloss_wq);

	rwnx_wdev_unregister(rwnx_hw);
	wiphy_unregister(rwnx_hw->wiphy);
	rwnx_radar_detection_deinit(&rwnx_hw->radar);
	rwnx_platform_off(rwnx_hw, NULL);
	kmem_cache_destroy(rwnx_hw->sw_txhdr_cache);
	aicwf_wakeup_lock_deinit(rwnx_hw);
	wiphy_free(rwnx_hw->wiphy);
}

static void aicsmac_driver_register(void)
{
#ifdef AICWF_SDIO_SUPPORT
	aicwf_sdio_register();
#endif
#ifdef AICWF_USB_SUPPORT
	aicwf_usb_register();
#endif
#ifdef AICWF_PCIE_SUPPORT
	aicwf_pcie_register();
#endif
}

//static DECLARE_WORK(aicsmac_driver_work, aicsmac_driver_register);

struct completion hostif_register_done;
static int rwnx_driver_err = -1;

#define REGISTRATION_TIMEOUT                     9000

void aicwf_hostif_ready(void)
{
	rwnx_driver_err = 0;
	complete(&hostif_register_done);
}

void aicwf_hostif_fail(void)
{
	rwnx_driver_err = 1;
	complete(&hostif_register_done);
}

static int __init rwnx_mod_init(void)
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);
	rwnx_print_version();

#ifdef AICWF_BSP_CTRL
	if (aicbsp_set_subsys(AIC_WIFI, AIC_PWR_ON) < 0) {
		printk("%s, set power on fail!\n", __func__);
		return -ENODEV;
	}
#endif

	init_completion(&hostif_register_done);
	aicsmac_driver_register();

#ifdef AICWF_SDIO_SUPPORT
	if ((wait_for_completion_timeout(&hostif_register_done, msecs_to_jiffies(REGISTRATION_TIMEOUT)) == 0) || rwnx_driver_err) {
		printk("register_driver timeout or error\n");
		aicwf_sdio_exit();
		aicbsp_set_subsys(AIC_WIFI, AIC_PWR_OFF);
		return -ENODEV;
	}
#endif /* AICWF_SDIO_SUPPORT */

#ifdef AICWF_PCIE_SUPPORT
	return rwnx_platform_register_drv();
#else
	return 0;
#endif
}

/**
 *
 */
static void __exit rwnx_mod_exit(void)
{
	RWNX_DBG(RWNX_FN_ENTRY_STR);

#ifdef AICWF_PCIE_SUPPORT
	rwnx_platform_unregister_drv();
#endif

#ifdef AICWF_SDIO_SUPPORT
	aicwf_sdio_exit();
#endif

#ifdef AICWF_USB_SUPPORT
	aicwf_usb_exit();
#endif

#ifdef AICWF_BSP_CTRL
	aicbsp_set_subsys(AIC_WIFI, AIC_PWR_OFF);
#endif
}

module_init(rwnx_mod_init);
module_exit(rwnx_mod_exit);

MODULE_FIRMWARE(RWNX_CONFIG_FW_NAME);

MODULE_DESCRIPTION(RW_DRV_DESCRIPTION);
MODULE_VERSION(RWNX_VERS_MOD);
MODULE_AUTHOR(RW_DRV_COPYRIGHT " " RW_DRV_AUTHOR);
MODULE_LICENSE("GPL");

