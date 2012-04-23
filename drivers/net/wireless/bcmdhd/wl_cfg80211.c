/*
 * Linux cfg80211 driver
 *
 * Copyright (C) 1999-2011, Broadcom Corporation
 *
 *         Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: wl_cfg80211.c,v 1.1.4.1.2.14 2011/02/09 01:40:07 Exp $
 */

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <linux/kernel.h>

#include <bcmutils.h>
#include <bcmwifi.h>
#include <bcmendian.h>
#include <proto/ethernet.h>
#include <proto/802.11.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>

#include <dngl_stats.h>
#include <dhd.h>
#include <dhdioctl.h>
#include <wlioctl.h>

#include <proto/ethernet.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/wait.h>
#include <net/cfg80211.h>

#include <net/rtnetlink.h>
#include <linux/mmc/sdio_func.h>
#include <linux/firmware.h>
#include <bcmsdbus.h>

#include <wlioctl.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <wl_cfgp2p.h>

static struct sdio_func *cfg80211_sdio_func;
static struct wl_priv *wlcfg_drv_priv;

u32 wl_dbg_level = WL_DBG_ERR;

#define WL_4329_FW_FILE "brcm/bcm4329-fullmac-4-218-248-5.bin"
#define WL_4329_NVRAM_FILE "brcm/bcm4329-fullmac-4-218-248-5.txt"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAX_WAIT_TIME 1500
static s8 ioctlbuf[WLC_IOCTL_MAXLEN];

#define COEX_DHCP

#if defined(COEX_DHCP)
#define BT_DHCP_eSCO_FIX		/* use New SCO/eSCO smart YG
					 * suppression
					 */
#define BT_DHCP_USE_FLAGS		/* this flag boost wifi pkt priority
					 * to max, caution: -not fair to sco
					 */
#define BT_DHCP_OPPR_WIN_TIME	2500	/* T1 start SCO/ESCo priority
					 * suppression
					 */
#define BT_DHCP_FLAG_FORCE_TIME 5500	/* T2 turn off SCO/SCO supperesion
					 * is (timeout)
					 */
enum wl_cfg80211_btcoex_status {
	BT_DHCP_IDLE,
	BT_DHCP_START,
	BT_DHCP_OPPR_WIN,
	BT_DHCP_FLAG_FORCE_TIMEOUT
};

static int wl_cfg80211_btcoex_init(struct wl_priv *wl);
static void wl_cfg80211_btcoex_deinit(struct wl_priv *wl);
#endif

/* This is to override regulatory domains defined in cfg80211 module (reg.c)
 * By default world regulatory domain defined in reg.c puts the flags NL80211_RRF_PASSIVE_SCAN
 * and NL80211_RRF_NO_IBSS for 5GHz channels (for 36..48 and 149..165).
 * With respect to these flags, wpa_supplicant doesn't start p2p operations on 5GHz channels.
 * All the chnages in world regulatory domain are to be done here.
 */
static const struct ieee80211_regdomain brcm_regdom = {
	.n_reg_rules = 5,
	.alpha2 =  "99",
	.reg_rules = {
		/* IEEE 802.11b/g, channels 1..11 */
		REG_RULE(2412-10, 2462+10, 40, 6, 20, 0),
		/* IEEE 802.11b/g, channels 12..13. No HT40
		 * channel fits here.
		 */
		REG_RULE(2467-10, 2472+10, 20, 6, 20,
		NL80211_RRF_PASSIVE_SCAN |
		NL80211_RRF_NO_IBSS),
		/* IEEE 802.11 channel 14 - Only JP enables
		 * this and for 802.11b only
		 */
		REG_RULE(2484-10, 2484+10, 20, 6, 20,
		NL80211_RRF_PASSIVE_SCAN |
		NL80211_RRF_NO_IBSS |
		NL80211_RRF_NO_OFDM),
		/* IEEE 802.11a, channel 36..64 */
		REG_RULE(5150-10, 5350+10, 40, 6, 20, 0),
		/* IEEE 802.11a, channel 100..165 */
		REG_RULE(5470-10, 5850+10, 40, 6, 20, 0), }
};


/* Data Element Definitions */
#define WPS_ID_CONFIG_METHODS     0x1008
#define WPS_ID_REQ_TYPE           0x103A
#define WPS_ID_DEVICE_NAME        0x1011
#define WPS_ID_VERSION            0x104A
#define WPS_ID_DEVICE_PWD_ID      0x1012
#define WPS_ID_REQ_DEV_TYPE       0x106A
#define WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS 0x1053
#define WPS_ID_PRIM_DEV_TYPE      0x1054

/* Device Password ID */
#define DEV_PW_DEFAULT 0x0000
#define DEV_PW_USER_SPECIFIED 0x0001,
#define DEV_PW_MACHINE_SPECIFIED 0x0002
#define DEV_PW_REKEY 0x0003
#define DEV_PW_PUSHBUTTON 0x0004
#define DEV_PW_REGISTRAR_SPECIFIED 0x0005

/* Config Methods */
#define WPS_CONFIG_USBA 0x0001
#define WPS_CONFIG_ETHERNET 0x0002
#define WPS_CONFIG_LABEL 0x0004
#define WPS_CONFIG_DISPLAY 0x0008
#define WPS_CONFIG_EXT_NFC_TOKEN 0x0010
#define WPS_CONFIG_INT_NFC_TOKEN 0x0020
#define WPS_CONFIG_NFC_INTERFACE 0x0040
#define WPS_CONFIG_PUSHBUTTON 0x0080
#define WPS_CONFIG_KEYPAD 0x0100
#define WPS_CONFIG_VIRT_PUSHBUTTON 0x0280
#define WPS_CONFIG_PHY_PUSHBUTTON 0x0480
#define WPS_CONFIG_VIRT_DISPLAY 0x2008
#define WPS_CONFIG_PHY_DISPLAY 0x4008

/*
 * cfg80211_ops api/callback list
 */
static s32 wl_frame_get_mgmt(u16 fc, const struct ether_addr *da,
	const struct ether_addr *sa, const struct ether_addr *bssid,
	u8 **pheader, u32 *body_len, u8 *pbody);
static s32 __wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request,
	struct cfg80211_ssid *this_ssid);
static s32 wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request);
static s32 wl_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed);
static s32 wl_cfg80211_join_ibss(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_ibss_params *params);
static s32 wl_cfg80211_leave_ibss(struct wiphy *wiphy,
	struct net_device *dev);
static s32 wl_cfg80211_get_station(struct wiphy *wiphy,
	struct net_device *dev, u8 *mac,
	struct station_info *sinfo);
static s32 wl_cfg80211_set_power_mgmt(struct wiphy *wiphy,
	struct net_device *dev, bool enabled,
	s32 timeout);
static int wl_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
	u16 reason_code);
static s32 wl_cfg80211_set_tx_power(struct wiphy *wiphy,
	enum nl80211_tx_power_setting type,
	s32 dbm);
static s32 wl_cfg80211_get_tx_power(struct wiphy *wiphy, s32 *dbm);
static s32 wl_cfg80211_config_default_key(struct wiphy *wiphy,
	struct net_device *dev,
	u8 key_idx, bool unicast, bool multicast);
static s32 wl_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr,
	struct key_params *params);
static s32 wl_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr);
static s32 wl_cfg80211_get_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr,
	void *cookie, void (*callback) (void *cookie,
	struct key_params *params));
static s32 wl_cfg80211_config_default_mgmt_key(struct wiphy *wiphy,
	struct net_device *dev,	u8 key_idx);
static s32 wl_cfg80211_resume(struct wiphy *wiphy);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
static s32 wl_cfg80211_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow);
#else
static s32 wl_cfg80211_suspend(struct wiphy *wiphy);
#endif
static s32 wl_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa);
static s32 wl_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa);
static s32 wl_cfg80211_flush_pmksa(struct wiphy *wiphy,
	struct net_device *dev);
static void wl_notify_escan_complete(struct wl_priv *wl, bool aborted);
/*
 * event & event Q handlers for cfg80211 interfaces
 */
static s32 wl_create_event_handler(struct wl_priv *wl);
static void wl_destroy_event_handler(struct wl_priv *wl);
static s32 wl_event_handler(void *data);
static void wl_init_eq(struct wl_priv *wl);
static void wl_flush_eq(struct wl_priv *wl);
static unsigned long wl_lock_eq(struct wl_priv *wl);
static void wl_unlock_eq(struct wl_priv *wl, unsigned long flags);
static void wl_init_eq_lock(struct wl_priv *wl);
static void wl_init_event_handler(struct wl_priv *wl);
static struct wl_event_q *wl_deq_event(struct wl_priv *wl);
static s32 wl_enq_event(struct wl_priv *wl, struct net_device *ndev, u32 type,
	const wl_event_msg_t *msg, void *data);
static void wl_put_event(struct wl_event_q *e);
static void wl_wakeup_event(struct wl_priv *wl);
static s32 wl_notify_connect_status(struct wl_priv *wl,
	struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_roaming_status(struct wl_priv *wl,
	struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_scan_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_bss_connect_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data, bool completed);
static s32 wl_bss_roaming_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_mic_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
/*
 * register/deregister sdio function
 */
struct sdio_func *wl_cfg80211_get_sdio_func(void);
static void wl_clear_sdio_func(void);

/*
 * ioctl utilites
 */
static s32 wl_dev_bufvar_get(struct net_device *dev, s8 *name, s8 *buf,
	s32 buf_len);
static __used s32 wl_dev_bufvar_set(struct net_device *dev, s8 *name,
	s8 *buf, s32 len);
static s32 wl_dev_intvar_set(struct net_device *dev, s8 *name, s32 val);
static s32 wl_dev_intvar_get(struct net_device *dev, s8 *name,
	s32 *retval);

/*
 * cfg80211 set_wiphy_params utilities
 */
static s32 wl_set_frag(struct net_device *dev, u32 frag_threshold);
static s32 wl_set_rts(struct net_device *dev, u32 frag_threshold);
static s32 wl_set_retry(struct net_device *dev, u32 retry, bool l);

/*
 * wl profile utilities
 */
static s32 wl_update_prof(struct wl_priv *wl, const wl_event_msg_t *e,
	void *data, s32 item);
static void *wl_read_prof(struct wl_priv *wl, s32 item);
static void wl_init_prof(struct wl_priv *wl);

/*
 * cfg80211 connect utilites
 */
static s32 wl_set_wpa_version(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_auth_type(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_set_cipher(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_key_mgmt(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_set_sharedkey(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_get_assoc_ies(struct wl_priv *wl, struct net_device *ndev);
static void wl_ch_to_chanspec(int ch,
	struct wl_join_params *join_params, size_t *join_params_size);

/*
 * information element utilities
 */
static void wl_rst_ie(struct wl_priv *wl);
static __used s32 wl_add_ie(struct wl_priv *wl, u8 t, u8 l, u8 *v);
static s32 wl_mrg_ie(struct wl_priv *wl, u8 *ie_stream, u16 ie_size);
static s32 wl_cp_ie(struct wl_priv *wl, u8 *dst, u16 dst_size);
static u32 wl_get_ielen(struct wl_priv *wl);

static s32 wl_mode_to_nl80211_iftype(s32 mode);

static struct wireless_dev *wl_alloc_wdev(struct device *sdiofunc_dev);
static void wl_free_wdev(struct wl_priv *wl);

static s32 wl_inform_bss(struct wl_priv *wl);
static s32 wl_inform_single_bss(struct wl_priv *wl, struct wl_bss_info *bi);
static s32 wl_update_bss_info(struct wl_priv *wl, struct net_device *ndev);

static s32 wl_add_keyext(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, const u8 *mac_addr,
	struct key_params *params);
/*
 * key indianess swap utilities
 */
static void swap_key_from_BE(struct wl_wsec_key *key);
static void swap_key_to_BE(struct wl_wsec_key *key);

/*
 * wl_priv memory init/deinit utilities
 */
static s32 wl_init_priv_mem(struct wl_priv *wl);
static void wl_deinit_priv_mem(struct wl_priv *wl);

static void wl_delay(u32 ms);

/*
 * ibss mode utilities
 */
static bool wl_is_ibssmode(struct wl_priv *wl, struct net_device *ndev);
static __used bool wl_is_ibssstarter(struct wl_priv *wl);

/*
 * dongle up/down , default configuration utilities
 */
static bool wl_is_linkdown(struct wl_priv *wl, const wl_event_msg_t *e);
static bool wl_is_linkup(struct wl_priv *wl, const wl_event_msg_t *e, struct net_device *ndev);
static bool wl_is_nonetwork(struct wl_priv *wl, const wl_event_msg_t *e);
static void wl_link_up(struct wl_priv *wl);
static void wl_link_down(struct wl_priv *wl);
static s32 wl_dongle_mode(struct wl_priv *wl, struct net_device *ndev, s32 iftype);
static s32 __wl_cfg80211_up(struct wl_priv *wl);
static s32 __wl_cfg80211_down(struct wl_priv *wl);
static s32 wl_dongle_probecap(struct wl_priv *wl);
static void wl_init_conf(struct wl_conf *conf);
static s32 wl_dongle_add_remove_eventmsg(struct net_device *ndev, u16 event, bool add);

/*
 * dongle configuration utilities
 */
#ifndef EMBEDDED_PLATFORM
static s32 wl_dongle_country(struct net_device *ndev, u8 ccode);
static s32 wl_dongle_up(struct net_device *ndev, u32 up);
static s32 wl_dongle_power(struct net_device *ndev, u32 power_mode);
static s32 wl_dongle_glom(struct net_device *ndev, u32 glom,
	u32 dongle_align);
static s32 wl_dongle_roam(struct net_device *ndev, u32 roamvar,
	u32 bcn_timeout);
static s32 wl_dongle_scantime(struct net_device *ndev, s32 scan_assoc_time,
	s32 scan_unassoc_time);
static s32 wl_dongle_offload(struct net_device *ndev, s32 arpoe,
	s32 arp_ol);
static s32 wl_pattern_atoh(s8 *src, s8 *dst);
static s32 wl_dongle_filter(struct net_device *ndev, u32 filter_mode);
static s32 wl_update_wiphybands(struct wl_priv *wl);
#endif				/* !EMBEDDED_PLATFORM */
static __used void wl_dongle_poweron(struct wl_priv *wl);
static __used void wl_dongle_poweroff(struct wl_priv *wl);
static s32 wl_config_dongle(struct wl_priv *wl, bool need_lock);

/*
 * iscan handler
 */
static void wl_iscan_timer(unsigned long data);
static void wl_term_iscan(struct wl_priv *wl);
static s32 wl_init_scan(struct wl_priv *wl);
static s32 wl_iscan_thread(void *data);
static s32 wl_run_iscan(struct wl_iscan_ctrl *iscan, struct cfg80211_scan_request *request,
	u16 action);
static s32 wl_do_iscan(struct wl_priv *wl,  struct cfg80211_scan_request *request);
static s32 wl_wakeup_iscan(struct wl_iscan_ctrl *iscan);
static s32 wl_invoke_iscan(struct wl_priv *wl);
static s32 wl_get_iscan_results(struct wl_iscan_ctrl *iscan, u32 *status,
	struct wl_scan_results **bss_list);
static void wl_notify_iscan_complete(struct wl_iscan_ctrl *iscan, bool aborted);
static void wl_init_iscan_handler(struct wl_iscan_ctrl *iscan);
static s32 wl_iscan_done(struct wl_priv *wl);
static s32 wl_iscan_pending(struct wl_priv *wl);
static s32 wl_iscan_inprogress(struct wl_priv *wl);
static s32 wl_iscan_aborted(struct wl_priv *wl);

/*
 * fw/nvram downloading handler
 */
static void wl_init_fw(struct wl_fw_ctrl *fw);

/*
 * find most significant bit set
 */
static __used u32 wl_find_msb(u16 bit16);

/*
 * update pmklist to dongle
 */
static __used s32 wl_update_pmklist(struct net_device *dev,
	struct wl_pmk_list *pmk_list, s32 err);

/*
 * debufs support
 */
static int wl_debugfs_add_netdev_params(struct wl_priv *wl);
static void wl_debugfs_remove_netdev(struct wl_priv *wl);

/*
 * rfkill support
 */
static int wl_setup_rfkill(struct wl_priv *wl, bool setup);
static int wl_rfkill_set(void *data, bool blocked);

/*
 * Some external functions, TODO: move them to dhd_linux.h
 */
int dhd_add_monitor(char *name, struct net_device **new_ndev);
int dhd_del_monitor(struct net_device *ndev);
int dhd_monitor_init(void *dhd_pub);
int dhd_monitor_uninit(void);
int dhd_start_xmit(struct sk_buff *skb, struct net_device *net);

#define CHECK_SYS_UP(wlpriv)							\
do {									\
	if (unlikely(!wl_get_drv_status(wlpriv, READY))) {	\
		WL_INFO(("device is not ready : status (%d)\n",		\
			(int)wlpriv->status));			\
		return -EIO;						\
	}								\
} while (0)


#define IS_WPA_AKM(akm) ((akm) == RSN_AKM_NONE || \
				 (akm) == RSN_AKM_UNSPECIFIED || \
				 (akm) == RSN_AKM_PSK)


extern int dhd_wait_pend8021x(struct net_device *dev);

#if (WL_DBG_LEVEL > 0)
#define WL_DBG_ESTR_MAX	50
static s8 wl_dbg_estr[][WL_DBG_ESTR_MAX] = {
	"SET_SSID", "JOIN", "START", "AUTH", "AUTH_IND",
	"DEAUTH", "DEAUTH_IND", "ASSOC", "ASSOC_IND", "REASSOC",
	"REASSOC_IND", "DISASSOC", "DISASSOC_IND", "QUIET_START", "QUIET_END",
	"BEACON_RX", "LINK", "MIC_ERROR", "NDIS_LINK", "ROAM",
	"TXFAIL", "PMKID_CACHE", "RETROGRADE_TSF", "PRUNE", "AUTOAUTH",
	"EAPOL_MSG", "SCAN_COMPLETE", "ADDTS_IND", "DELTS_IND", "BCNSENT_IND",
	"BCNRX_MSG", "BCNLOST_MSG", "ROAM_PREP", "PFN_NET_FOUND",
	"PFN_NET_LOST",
	"RESET_COMPLETE", "JOIN_START", "ROAM_START", "ASSOC_START",
	"IBSS_ASSOC",
	"RADIO", "PSM_WATCHDOG", "WLC_E_CCX_ASSOC_START", "WLC_E_CCX_ASSOC_ABORT",
	"PROBREQ_MSG",
	"SCAN_CONFIRM_IND", "PSK_SUP", "COUNTRY_CODE_CHANGED",
	"EXCEEDED_MEDIUM_TIME", "ICV_ERROR",
	"UNICAST_DECODE_ERROR", "MULTICAST_DECODE_ERROR", "TRACE",
	"WLC_E_BTA_HCI_EVENT", "IF", "WLC_E_P2P_DISC_LISTEN_COMPLETE",
	"RSSI", "PFN_SCAN_COMPLETE", "WLC_E_EXTLOG_MSG",
	"ACTION_FRAME", "ACTION_FRAME_COMPLETE", "WLC_E_PRE_ASSOC_IND",
	"WLC_E_PRE_REASSOC_IND", "WLC_E_CHANNEL_ADOPTED", "WLC_E_AP_STARTED",
	"WLC_E_DFS_AP_STOP", "WLC_E_DFS_AP_RESUME", "WLC_E_WAI_STA_EVENT",
	"WLC_E_WAI_MSG", "WLC_E_ESCAN_RESULT", "WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE",
	"WLC_E_PROBRESP_MSG", "WLC_E_P2P_PROBREQ_MSG", "WLC_E_DCS_REQUEST", "WLC_E_FIFO_CREDIT_MAP",
	"WLC_E_ACTION_FRAME_RX", "WLC_E_WAKE_EVENT", "WLC_E_RM_COMPLETE"
};
#endif				/* WL_DBG_LEVEL */

#define CHAN2G(_channel, _freq, _flags) {			\
	.band			= IEEE80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define CHAN5G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= 5000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define RATE_TO_BASE100KBPS(rate)   (((rate) * 10) / 2)
#define RATETAB_ENT(_rateid, _flags) \
	{								\
		.bitrate	= RATE_TO_BASE100KBPS(_rateid),     \
		.hw_value	= (_rateid),			    \
		.flags	  = (_flags),			     \
	}

static struct ieee80211_rate __wl_rates[] = {
	RATETAB_ENT(WLC_RATE_1M, 0),
	RATETAB_ENT(WLC_RATE_2M, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLC_RATE_5M5, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLC_RATE_11M, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLC_RATE_6M, 0),
	RATETAB_ENT(WLC_RATE_9M, 0),
	RATETAB_ENT(WLC_RATE_12M, 0),
	RATETAB_ENT(WLC_RATE_18M, 0),
	RATETAB_ENT(WLC_RATE_24M, 0),
	RATETAB_ENT(WLC_RATE_36M, 0),
	RATETAB_ENT(WLC_RATE_48M, 0),
	RATETAB_ENT(WLC_RATE_54M, 0)
};

#define wl_a_rates		(__wl_rates + 4)
#define wl_a_rates_size	8
#define wl_g_rates		(__wl_rates + 0)
#define wl_g_rates_size	12

static struct ieee80211_channel __wl_2ghz_channels[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0)
};

static struct ieee80211_channel __wl_5ghz_a_channels[] = {
	CHAN5G(34, 0), CHAN5G(36, 0),
	CHAN5G(38, 0), CHAN5G(40, 0),
	CHAN5G(42, 0), CHAN5G(44, 0),
	CHAN5G(46, 0), CHAN5G(48, 0),
	CHAN5G(52, 0), CHAN5G(56, 0),
	CHAN5G(60, 0), CHAN5G(64, 0),
	CHAN5G(100, 0), CHAN5G(104, 0),
	CHAN5G(108, 0), CHAN5G(112, 0),
	CHAN5G(116, 0), CHAN5G(120, 0),
	CHAN5G(124, 0), CHAN5G(128, 0),
	CHAN5G(132, 0), CHAN5G(136, 0),
	CHAN5G(140, 0), CHAN5G(149, 0),
	CHAN5G(153, 0), CHAN5G(157, 0),
	CHAN5G(161, 0), CHAN5G(165, 0)
};

static struct ieee80211_supported_band __wl_band_2ghz = {
	.band = IEEE80211_BAND_2GHZ,
	.channels = __wl_2ghz_channels,
	.n_channels = ARRAY_SIZE(__wl_2ghz_channels),
	.bitrates = wl_g_rates,
	.n_bitrates = wl_g_rates_size
};

static struct ieee80211_supported_band __wl_band_5ghz_a = {
	.band = IEEE80211_BAND_5GHZ,
	.channels = __wl_5ghz_a_channels,
	.n_channels = ARRAY_SIZE(__wl_5ghz_a_channels),
	.bitrates = wl_a_rates,
	.n_bitrates = wl_a_rates_size
};

static const u32 __wl_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

/* There isn't a lot of sense in it, but you can transmit anything you like */
static const struct ieee80211_txrx_stypes
wl_cfg80211_default_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_ADHOC] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_STATION] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_AP] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		BIT(IEEE80211_STYPE_AUTH >> 4) |
		BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_AP_VLAN] = {
		/* copy AP */
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		BIT(IEEE80211_STYPE_AUTH >> 4) |
		BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_P2P_CLIENT] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_P2P_GO] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		BIT(IEEE80211_STYPE_AUTH >> 4) |
		BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		BIT(IEEE80211_STYPE_ACTION >> 4)
	}
};

static void swap_key_from_BE(struct wl_wsec_key *key)
{
	key->index = htod32(key->index);
	key->len = htod32(key->len);
	key->algo = htod32(key->algo);
	key->flags = htod32(key->flags);
	key->rxiv.hi = htod32(key->rxiv.hi);
	key->rxiv.lo = htod16(key->rxiv.lo);
	key->iv_initialized = htod32(key->iv_initialized);
}

static void swap_key_to_BE(struct wl_wsec_key *key)
{
	key->index = dtoh32(key->index);
	key->len = dtoh32(key->len);
	key->algo = dtoh32(key->algo);
	key->flags = dtoh32(key->flags);
	key->rxiv.hi = dtoh32(key->rxiv.hi);
	key->rxiv.lo = dtoh16(key->rxiv.lo);
	key->iv_initialized = dtoh32(key->iv_initialized);
}

/* For debug: Dump the contents of the encoded wps ie buffe */
static void
wl_validate_wps_ie(char *wps_ie, bool *pbc)
{
	#define WPS_IE_FIXED_LEN 6
	u16 len = (u16) wps_ie[TLV_LEN_OFF];
	u8 *subel = wps_ie+  WPS_IE_FIXED_LEN;
	u16 subelt_id;
	u16 subelt_len;
	u16 val;
	u8 *valptr = (uint8*) &val;

	WL_DBG(("wps_ie len=%d\n", len));

	len -= 4;	/* for the WPS IE's OUI, oui_type fields */

	while (len >= 4) {		/* must have attr id, attr len fields */
		valptr[0] = *subel++;
		valptr[1] = *subel++;
		subelt_id = HTON16(val);

		valptr[0] = *subel++;
		valptr[1] = *subel++;
		subelt_len = HTON16(val);

		len -= 4;			/* for the attr id, attr len fields */
		len -= subelt_len;	/* for the remaining fields in this attribute */
		WL_DBG((" subel=%p, subelt_id=0x%x subelt_len=%u\n",
			subel, subelt_id, subelt_len));

		if (subelt_id == WPS_ID_VERSION) {
			WL_DBG(("  attr WPS_ID_VERSION: %u\n", *subel));
		} else if (subelt_id == WPS_ID_REQ_TYPE) {
			WL_DBG(("  attr WPS_ID_REQ_TYPE: %u\n", *subel));
		} else if (subelt_id == WPS_ID_CONFIG_METHODS) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_CONFIG_METHODS: %x\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_DEVICE_NAME) {
			char devname[100];
			memcpy(devname, subel, subelt_len);
			devname[subelt_len] = '\0';
			WL_DBG(("  attr WPS_ID_DEVICE_NAME: %s (len %u)\n",
				devname, subelt_len));
		} else if (subelt_id == WPS_ID_DEVICE_PWD_ID) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_DEVICE_PWD_ID: %u\n", HTON16(val)));
			*pbc = (HTON16(val) == DEV_PW_PUSHBUTTON) ? true : false;
		} else if (subelt_id == WPS_ID_PRIM_DEV_TYPE) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_PRIM_DEV_TYPE: cat=%u \n", HTON16(val)));
			valptr[0] = *(subel + 6);
			valptr[1] = *(subel + 7);
			WL_DBG(("  attr WPS_ID_PRIM_DEV_TYPE: subcat=%u\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_REQ_DEV_TYPE) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_REQ_DEV_TYPE: cat=%u\n", HTON16(val)));
			valptr[0] = *(subel + 6);
			valptr[1] = *(subel + 7);
			WL_DBG(("  attr WPS_ID_REQ_DEV_TYPE: subcat=%u\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS"
				": cat=%u\n", HTON16(val)));
		} else {
			WL_DBG(("  unknown attr 0x%x\n", subelt_id));
		}

		subel += subelt_len;
	}
}

static struct net_device* wl_cfg80211_add_monitor_if(char *name)
{
	int ret = 0;
	struct net_device* ndev = NULL;

	ret = dhd_add_monitor(name, &ndev);
	WL_INFO(("wl_cfg80211_add_monitor_if net device returned: 0x%p\n", ndev));
	return ndev;
}

static struct net_device *
wl_cfg80211_add_virtual_iface(struct wiphy *wiphy, char *name,
	enum nl80211_iftype type, u32 *flags,
	struct vif_params *params)
{
	s32 err;
	s32 timeout = -1;
	s32 wlif_type = -1;
	s32 index = 0;
	s32 mode = 0;
	chanspec_t chspec;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *_ndev;
	dhd_pub_t *dhd = (dhd_pub_t *)(wl->pub);
	int (*net_attach)(dhd_pub_t *dhdp, int ifidx);
	bool rollback_lock = false;

	WL_DBG(("if name: %s, type: %d\n", name, type));
	switch (type) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		WL_ERR(("Unsupported interface type\n"));
		mode = WL_MODE_IBSS;
		return NULL;
	case NL80211_IFTYPE_MONITOR:
		return wl_cfg80211_add_monitor_if(name);
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_STATION:
		wlif_type = WL_P2P_IF_CLIENT;
		mode = WL_MODE_BSS;
		break;
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_AP:
		wlif_type = WL_P2P_IF_GO;
		mode = WL_MODE_AP;
		break;
	default:
		WL_ERR(("Unsupported interface type\n"));
		return NULL;
		break;
	}

	if (!name) {
		WL_ERR(("name is NULL\n"));
		return NULL;
	}
	if (wl->p2p_supported && (wlif_type != -1)) {
		if (wl_get_p2p_status(wl, IF_DELETING)) {
			/* wait till IF_DEL is complete
			 * release the lock for the unregister to proceed
			 */
			if (rtnl_is_locked()) {
				rtnl_unlock();
				rollback_lock = true;
			}
			WL_INFO(("%s: Released the lock and wait till IF_DEL is complete\n",
				__func__));
			timeout = wait_event_interruptible_timeout(wl->dongle_event_wait,
				(wl_get_p2p_status(wl, IF_DELETING) == false),
				msecs_to_jiffies(MAX_WAIT_TIME));

			/* put back the rtnl_lock again */
			if (rollback_lock) {
				rtnl_lock();
				rollback_lock = false;
			}
			if (timeout > 0) {
				WL_ERR(("IF DEL is Success\n"));

			} else {
				WL_ERR(("timeount < 0, return -EAGAIN\n"));
				return ERR_PTR(-EAGAIN);
			}
		}
		if (!p2p_on(wl) && strstr(name, WL_P2P_INTERFACE_PREFIX)) {
			p2p_on(wl) = true;
			wl_cfgp2p_set_firm_p2p(wl);
			wl_cfgp2p_init_discovery(wl);
		}
		memset(wl->p2p->vir_ifname, 0, IFNAMSIZ);
		strncpy(wl->p2p->vir_ifname, name, IFNAMSIZ - 1);
		wl_cfgp2p_generate_bss_mac(&dhd->mac, &wl->p2p->dev_addr, &wl->p2p->int_addr);

		/* Temporary use channel 11, in case GO will be changed with set_channel API  */
		chspec = wf_chspec_aton(WL_P2P_TEMP_CHAN);

		/* For P2P mode, use P2P-specific driver features to create the
		 * bss: "wl p2p_ifadd"
		 */
		wl_set_p2p_status(wl, IF_ADD);
		err = wl_cfgp2p_ifadd(wl, &wl->p2p->int_addr, htod32(wlif_type), chspec);

		if (unlikely(err))
			return ERR_PTR(-ENOMEM);

		timeout = wait_event_interruptible_timeout(wl->dongle_event_wait,
			(wl_get_p2p_status(wl, IF_ADD) == false),
			msecs_to_jiffies(MAX_WAIT_TIME));
		if (timeout > 0 && (!wl_get_p2p_status(wl, IF_ADD))) {

			struct wireless_dev *vwdev;
			vwdev = kzalloc(sizeof(*vwdev), GFP_KERNEL);
			if (unlikely(!vwdev)) {
				WL_ERR(("Could not allocate wireless device\n"));
				return ERR_PTR(-ENOMEM);
			}
			vwdev->wiphy = wl->wdev->wiphy;
			WL_INFO((" virtual interface(%s) is created memalloc done \n",
			wl->p2p->vir_ifname));
			index = alloc_idx_vwdev(wl);
			wl->vwdev[index] = vwdev;
			vwdev->iftype =
				(wlif_type == WL_P2P_IF_CLIENT) ? NL80211_IFTYPE_STATION
				: NL80211_IFTYPE_AP;
			_ndev =  wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION);
			_ndev->ieee80211_ptr = vwdev;
			SET_NETDEV_DEV(_ndev, wiphy_dev(vwdev->wiphy));
			vwdev->netdev = _ndev;
			wl_set_drv_status(wl, READY);
			wl->p2p->vif_created = true;
			set_mode_by_netdev(wl, _ndev, mode);
			net_attach =  wl_to_p2p_bss_private(wl, P2PAPI_BSSCFG_CONNECTION);
			if (rtnl_is_locked()) {
				rtnl_unlock();
				rollback_lock = true;
			}
			if (net_attach && !net_attach(dhd, _ndev->ifindex)) {
				WL_DBG((" virtual interface(%s) is "
					"created net attach done\n", wl->p2p->vir_ifname));
			} else {
				/* put back the rtnl_lock again */
				if (rollback_lock)
					rtnl_lock();
				goto fail;
			}
			/* put back the rtnl_lock again */
			if (rollback_lock)
				rtnl_lock();
			return _ndev;

		} else {
			wl_clr_p2p_status(wl, IF_ADD);
			WL_ERR((" virtual interface(%s) is not created \n", wl->p2p->vir_ifname));
			memset(wl->p2p->vir_ifname, '\0', IFNAMSIZ);
			wl->p2p->vif_created = false;
		}
	}
fail:
	return ERR_PTR(-ENODEV);
}

static s32
wl_cfg80211_del_virtual_iface(struct wiphy *wiphy, struct net_device *dev)
{
	struct ether_addr p2p_mac;
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 timeout = -1;
	s32 ret = 0;
	WL_DBG(("Enter\n"));
	if (wl->p2p_supported) {
		memcpy(p2p_mac.octet, wl->p2p->int_addr.octet, ETHER_ADDR_LEN);
		if (wl->p2p->vif_created) {
			if (wl_get_drv_status(wl, SCANNING)) {
				wl_cfg80211_scan_abort(wl, dev);
			}
			wldev_iovar_setint(dev, "mpc", 1);
			wl_set_p2p_status(wl, IF_DELETING);
			ret = wl_cfgp2p_ifdel(wl, &p2p_mac);
			if (ret) {
			/* Firmware could not delete the interface so we will not get WLC_E_IF
			* event for cleaning the dhd virtual nw interace
			* So lets do it here. Failures from fw will ensure the application to do
			* ifconfig <inter> down and up sequnce, which will reload the fw
			* however we should cleanup the linux network virtual interfaces
			*/
				dhd_pub_t *dhd = (dhd_pub_t *)(wl->pub);
				WL_ERR(("Firmware returned an error from p2p_ifdel\n"));
				WL_ERR(("try to remove linux virtual interface %s\n", dev->name));
				dhd_del_if(dhd->info, dhd_net2idx(dhd->info, dev));
			}

			/* Wait for any pending scan req to get aborted from the sysioc context */
			timeout = wait_event_interruptible_timeout(wl->dongle_event_wait,
				(wl_get_p2p_status(wl, IF_DELETING) == false),
				msecs_to_jiffies(MAX_WAIT_TIME));
			if (timeout > 0 && !wl_get_p2p_status(wl, IF_DELETING)) {
				WL_DBG(("IFDEL operation done\n"));
			} else {
				WL_ERR(("IFDEL didn't complete properly\n"));
			}
			ret = dhd_del_monitor(dev);
		}
	}
	return ret;
}

static s32
wl_cfg80211_change_virtual_iface(struct wiphy *wiphy, struct net_device *ndev,
	enum nl80211_iftype type, u32 *flags,
	struct vif_params *params)
{
	s32 ap = 0;
	s32 infra = 0;
	s32 err = BCME_OK;
	s32 timeout = -1;
	s32 wlif_type;
	s32 mode = 0;
	chanspec_t chspec;
	struct wl_priv *wl = wiphy_priv(wiphy);
	WL_DBG(("Enter \n"));
	switch (type) {
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		ap = 1;
		WL_ERR(("type (%d) : currently we do not support this type\n",
			type));
		break;
	case NL80211_IFTYPE_ADHOC:
		mode = WL_MODE_IBSS;
		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		mode = WL_MODE_BSS;
		infra = 1;
		break;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_P2P_GO:
		mode = WL_MODE_AP;
		ap = 1;
		break;
	default:
		return -EINVAL;
	}


	if (ap) {
		set_mode_by_netdev(wl, ndev, mode);
		if (wl->p2p_supported && wl->p2p->vif_created) {
			WL_DBG(("p2p_vif_created (%d) p2p_on (%d)\n", wl->p2p->vif_created,
			p2p_on(wl)));
			chspec = wf_chspec_aton(WL_P2P_TEMP_CHAN);
			wlif_type = ap ? WL_P2P_IF_GO : WL_P2P_IF_CLIENT;
			WL_ERR(("%s : ap (%d), infra (%d), iftype: (%d)\n",
				ndev->name, ap, infra, type));
			wl_set_p2p_status(wl, IF_CHANGING);
			wl_clr_p2p_status(wl, IF_CHANGED);
			err = wl_cfgp2p_ifchange(wl, &wl->p2p->int_addr, htod32(wlif_type), chspec);
			timeout = wait_event_interruptible_timeout(wl->dongle_event_wait,
				(wl_get_p2p_status(wl, IF_CHANGED) == true),
				msecs_to_jiffies(MAX_WAIT_TIME));
			set_mode_by_netdev(wl, ndev, mode);
			wl_clr_p2p_status(wl, IF_CHANGING);
			wl_clr_p2p_status(wl, IF_CHANGED);
		} else if (ndev == wl_to_prmry_ndev(wl) &&
			!wl_get_drv_status(wl, AP_CREATED)) {
			wl_set_drv_status(wl, AP_CREATING);
			if (!wl->ap_info &&
				!(wl->ap_info = kzalloc(sizeof(struct ap_info), GFP_KERNEL))) {
				WL_ERR(("struct ap_saved_ie allocation failed\n"));
				return -ENOMEM;
			}
		} else {
			WL_ERR(("Cannot change the interface for GO or SOFTAP\n"));
			return -EINVAL;
		}
	}

	ndev->ieee80211_ptr->iftype = type;
	return 0;
}

s32
wl_cfg80211_notify_ifadd(struct net_device *ndev, s32 idx, s32 bssidx,
int (*_net_attach)(dhd_pub_t *dhdp, int ifidx))
{
	struct wl_priv *wl = wlcfg_drv_priv;
	s32 ret = BCME_OK;
	if (!ndev) {
		WL_ERR(("net is NULL\n"));
		return 0;
	}
	if (wl->p2p_supported) {
		WL_DBG(("IF_ADD event called from dongle, old interface name: %s,"
			"new name: %s\n", ndev->name, wl->p2p->vir_ifname));
		/* Assign the net device to CONNECT BSSCFG */
		strncpy(ndev->name, wl->p2p->vir_ifname, IFNAMSIZ - 1);
		wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION) = ndev;
		wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_CONNECTION) = bssidx;
		wl_to_p2p_bss_private(wl, P2PAPI_BSSCFG_CONNECTION) = _net_attach;
		ndev->ifindex = idx;
		wl_clr_p2p_status(wl, IF_ADD);

		wake_up_interruptible(&wl->dongle_event_wait);
	}
	return ret;
}

s32
wl_cfg80211_notify_ifdel(struct net_device *ndev)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	bool rollback_lock = false;
	s32 index = 0;

	if (!ndev || !ndev->name) {
		WL_ERR(("net is NULL\n"));
		return 0;
	}

	if (p2p_is_on(wl) && wl->p2p->vif_created) {
		if (wl->scan_request) {
			/* Abort any pending scan requests */
			wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
			if (!rtnl_is_locked()) {
				rtnl_lock();
				rollback_lock = true;
			}
			WL_DBG(("ESCAN COMPLETED\n"));
			wl_notify_escan_complete(wl, true);
			if (rollback_lock)
				rtnl_unlock();
		}
		WL_ERR(("IF_DEL event called from dongle, net %x, vif name: %s\n",
			(unsigned int)ndev, wl->p2p->vir_ifname));

		memset(wl->p2p->vir_ifname, '\0', IFNAMSIZ);
		index = wl_cfgp2p_find_idx(wl, ndev);
		wl_to_p2p_bss_ndev(wl, index) = NULL;
		wl_to_p2p_bss_bssidx(wl, index) = 0;
		wl->p2p->vif_created = false;
		wl_cfgp2p_clear_management_ie(wl,
			index);
		wl_clr_p2p_status(wl, IF_DELETING);
		WL_DBG(("index : %d\n", index));

	}
	/* Wake up any waiting thread */
	wake_up_interruptible(&wl->dongle_event_wait);

	return 0;
}

s32
wl_cfg80211_is_progress_ifadd(void)
{
	s32 is_progress = 0;
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl_get_p2p_status(wl, IF_ADD))
		is_progress = 1;
	return is_progress;
}

s32
wl_cfg80211_is_progress_ifchange(void)
{
	s32 is_progress = 0;
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl_get_p2p_status(wl, IF_CHANGING))
		is_progress = 1;
	return is_progress;
}


s32
wl_cfg80211_notify_ifchange(void)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl_get_p2p_status(wl, IF_CHANGING)) {
		wl_set_p2p_status(wl, IF_CHANGED);
		wake_up_interruptible(&wl->dongle_event_wait);
	}
	return 0;
}

static void wl_scan_prep(struct wl_scan_params *params, struct cfg80211_scan_request *request)
{
	u32 n_ssids = request->n_ssids;
	u32 n_channels = request->n_channels;
	u16 channel;
	chanspec_t chanspec;
	s32 i, offset;
	char *ptr;
	wlc_ssid_t ssid;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = 0;
	params->nprobes = -1;
	params->active_time = -1;
	params->passive_time = -1;
	params->home_time = -1;
	params->channel_num = 0;
	memset(&params->ssid, 0, sizeof(wlc_ssid_t));

	WL_SCAN(("Preparing Scan request\n"));
	WL_SCAN(("nprobes=%d\n", params->nprobes));
	WL_SCAN(("active_time=%d\n", params->active_time));
	WL_SCAN(("passive_time=%d\n", params->passive_time));
	WL_SCAN(("home_time=%d\n", params->home_time));
	WL_SCAN(("scan_type=%d\n", params->scan_type));

	params->nprobes = htod32(params->nprobes);
	params->active_time = htod32(params->active_time);
	params->passive_time = htod32(params->passive_time);
	params->home_time = htod32(params->home_time);

	/* Copy channel array if applicable */
	WL_SCAN(("### List of channelspecs to scan ###\n"));
	if (n_channels > 0) {
		for (i = 0; i < n_channels; i++) {
			chanspec = 0;
			channel = ieee80211_frequency_to_channel(request->channels[i]->center_freq);
			if (request->channels[i]->band == IEEE80211_BAND_2GHZ)
				chanspec |= WL_CHANSPEC_BAND_2G;
			else
				chanspec |= WL_CHANSPEC_BAND_5G;

			if (request->channels[i]->flags & IEEE80211_CHAN_NO_HT40) {
				chanspec |= WL_CHANSPEC_BW_20;
				chanspec |= WL_CHANSPEC_CTL_SB_NONE;
			} else {
				chanspec |= WL_CHANSPEC_BW_40;
				if (request->channels[i]->flags & IEEE80211_CHAN_NO_HT40PLUS)
					chanspec |= WL_CHANSPEC_CTL_SB_LOWER;
				else
					chanspec |= WL_CHANSPEC_CTL_SB_UPPER;
			}

			params->channel_list[i] = channel;
			params->channel_list[i] &= WL_CHANSPEC_CHAN_MASK;
			params->channel_list[i] |= chanspec;
			WL_SCAN(("Chan : %d, Channel spec: %x \n",
			channel, params->channel_list[i]));
			params->channel_list[i] = htod16(params->channel_list[i]);
		}
	} else {
		WL_SCAN(("Scanning all channels\n"));
	}

	/* Copy ssid array if applicable */
	WL_SCAN(("### List of SSIDs to scan ###\n"));
	if (n_ssids > 0) {
		offset = offsetof(wl_scan_params_t, channel_list) + n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		ptr = (char*)params + offset;
		for (i = 0; i < n_ssids; i++) {
			memset(&ssid, 0, sizeof(wlc_ssid_t));
			ssid.SSID_len = request->ssids[i].ssid_len;
			memcpy(ssid.SSID, request->ssids[i].ssid, ssid.SSID_len);
			if (!ssid.SSID_len)
				WL_SCAN(("%d: Broadcast scan\n", i));
			else
				WL_SCAN(("%d: scan  for  %s size =%d\n", i,
				ssid.SSID, ssid.SSID_len));
			memcpy(ptr, &ssid, sizeof(wlc_ssid_t));
			ptr += sizeof(wlc_ssid_t);
		}
	} else {
		WL_SCAN(("Broadcast scan\n"));
	}
	/* Adding mask to channel numbers */
	params->channel_num =
	        htod32((n_ssids << WL_SCAN_PARAMS_NSSID_SHIFT) |
	               (n_channels & WL_SCAN_PARAMS_COUNT_MASK));
}

static s32
wl_run_iscan(struct wl_iscan_ctrl *iscan, struct cfg80211_scan_request *request, u16 action)
{
	u32 n_channels;
	u32 n_ssids;
	s32 params_size =
	    (WL_SCAN_PARAMS_FIXED_SIZE + offsetof(wl_iscan_params_t, params));
	struct wl_iscan_params *params;
	s32 err = 0;

	if (request != NULL) {
		n_channels = request->n_channels;
		n_ssids = request->n_ssids;
		/* Allocate space for populating ssids in wl_iscan_params struct */
		if (n_channels % 2)
			/* If n_channels is odd, add a padd of u16 */
			params_size += sizeof(u16) * (n_channels + 1);
		else
			params_size += sizeof(u16) * n_channels;

		/* Allocate space for populating ssids in wl_iscan_params struct */
		params_size += sizeof(struct wlc_ssid) * n_ssids;
	}
	params = (struct wl_iscan_params *)kzalloc(params_size, GFP_KERNEL);
	if (!params) {
		return -ENOMEM;
	}

	if (request != NULL)
		wl_scan_prep(&params->params, request);

	params->version = htod32(ISCAN_REQ_VERSION);
	params->action = htod16(action);
	params->scan_duration = htod16(0);

	if (params_size + sizeof("iscan") >= WLC_IOCTL_MEDLEN) {
		WL_ERR(("ioctl buffer length is not sufficient\n"));
		err = -ENOMEM;
		goto done;
	}
	err = wldev_iovar_setbuf(iscan->dev, "iscan", params, params_size,
		iscan->ioctl_buf, WLC_IOCTL_MEDLEN);
	if (unlikely(err)) {
		if (err == -EBUSY) {
			WL_ERR(("system busy : iscan canceled\n"));
		} else {
			WL_ERR(("error (%d)\n", err));
		}
	}
done:
	kfree(params);
	return err;
}

static s32 wl_do_iscan(struct wl_priv *wl, struct cfg80211_scan_request *request)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 passive_scan;
	s32 err = 0;

	iscan->state = WL_ISCAN_STATE_SCANING;

	passive_scan = wl->active_scan ? 0 : 1;
	err = wldev_ioctl(ndev, WLC_SET_PASSIVE_SCAN,
		&passive_scan, sizeof(passive_scan), false);
	if (unlikely(err)) {
		WL_DBG(("error (%d)\n", err));
		return err;
	}
	wl->iscan_kickstart = true;
	wl_run_iscan(iscan, request, WL_SCAN_ACTION_START);
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms * HZ / 1000);
	iscan->timer_on = 1;

	return err;
}

static s32
wl_run_escan(struct wl_priv *wl, struct net_device *ndev,
	struct cfg80211_scan_request *request, uint16 action)
{
	s32 err = BCME_OK;
	u32 n_channels;
	u32 n_ssids;
	s32 params_size = (WL_SCAN_PARAMS_FIXED_SIZE + OFFSETOF(wl_escan_params_t, params));
	wl_escan_params_t *params;
	struct cfg80211_scan_request *scan_request = wl->scan_request;
	u32 num_chans = 0;
	s32 search_state = WL_P2P_DISC_ST_SCAN;
	u32 i;
	u16 *default_chan_list = NULL;
	struct net_device *dev = NULL;
	WL_DBG(("Enter \n"));


	if (!wl->p2p_supported || ((ndev == wl_to_prmry_ndev(wl)) &&
		!p2p_scan(wl))) {
		/* LEGACY SCAN TRIGGER */
		WL_SCAN((" LEGACY E-SCAN START\n"));

		if (request != NULL) {
			n_channels = request->n_channels;
			n_ssids = request->n_ssids;
			/* Allocate space for populating ssids in wl_iscan_params struct */
			if (n_channels % 2)
				/* If n_channels is odd, add a padd of u16 */
				params_size += sizeof(u16) * (n_channels + 1);
			else
				params_size += sizeof(u16) * n_channels;

			/* Allocate space for populating ssids in wl_iscan_params struct */
			params_size += sizeof(struct wlc_ssid) * n_ssids;
		}
		params = (wl_escan_params_t *) kzalloc(params_size, GFP_KERNEL);
		if (params == NULL) {
			err = -ENOMEM;
			goto exit;
		}

		if (request != NULL)
			wl_scan_prep(&params->params, request);
		params->version = htod32(ESCAN_REQ_VERSION);
		params->action =  htod16(action);
		params->sync_id = htod16(0x1234);
		if (params_size + sizeof("escan") >= WLC_IOCTL_MEDLEN) {
			WL_ERR(("ioctl buffer length not sufficient\n"));
			kfree(params);
			err = -ENOMEM;
			goto exit;
		}
		err = wldev_iovar_setbuf(ndev, "escan", params, params_size,
			wl->escan_ioctl_buf, WLC_IOCTL_MEDLEN);
		if (unlikely(err))
			WL_ERR((" Escan set error (%d)\n", err));
		kfree(params);
	}
	else if (p2p_on(wl) && p2p_scan(wl)) {
		/* P2P SCAN TRIGGER */
		if (scan_request && scan_request->n_channels) {
			num_chans = scan_request->n_channels;
			WL_SCAN((" chann number : %d\n", num_chans));
			default_chan_list = kzalloc(num_chans * sizeof(*default_chan_list),
				GFP_KERNEL);
			if (default_chan_list == NULL) {
				WL_ERR(("channel list allocation failed \n"));
				err = -ENOMEM;
				goto exit;
			}
			for (i = 0; i < num_chans; i++)
			{
				default_chan_list[i] =
				ieee80211_frequency_to_channel(
					scan_request->channels[i]->center_freq);
			}
			if (num_chans == 3 && (
						(default_chan_list[0] == SOCIAL_CHAN_1) &&
						(default_chan_list[1] == SOCIAL_CHAN_2) &&
						(default_chan_list[2] == SOCIAL_CHAN_3))) {
				/* SOCIAL CHANNELS 1, 6, 11 */
				search_state = WL_P2P_DISC_ST_SEARCH;
				WL_INFO(("P2P SEARCH PHASE START \n"));
			} else if ((dev = wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION)) &&
				(get_mode_by_netdev(wl, dev) == WL_MODE_AP)) {
				/* If you are already a GO, then do SEARCH only */
				WL_INFO(("Already a GO. Do SEARCH Only"));
				search_state = WL_P2P_DISC_ST_SEARCH;
			} else {
				WL_INFO(("P2P SCAN STATE START \n"));
			}

		}
		err = wl_cfgp2p_escan(wl, ndev, wl->active_scan, num_chans, default_chan_list,
			search_state, action,
			wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_DEVICE));
		kfree(default_chan_list);
	}
exit:
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
	}
	return err;
}


static s32
wl_do_escan(struct wl_priv *wl, struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request)
{
	s32 err = BCME_OK;
	s32 passive_scan;
	wl_scan_results_t *results;
	WL_SCAN(("Enter \n"));

	wl->escan_info.wiphy = wiphy;
	wl->escan_info.escan_state = WL_ESCAN_STATE_SCANING;
	passive_scan = wl->active_scan ? 0 : 1;
	err = wldev_ioctl(ndev, WLC_SET_PASSIVE_SCAN,
		&passive_scan, sizeof(passive_scan), false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	results = (wl_scan_results_t *) wl->escan_info.escan_buf;
	results->version = 0;
	results->count = 0;
	results->buflen = WL_SCAN_RESULTS_FIXED_SIZE;

	err = wl_run_escan(wl, ndev, request, WL_SCAN_ACTION_START);
	return err;
}

static s32
__wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request,
	struct cfg80211_ssid *this_ssid)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct cfg80211_ssid *ssids;
	struct wl_scan_req *sr = wl_to_sr(wl);
	wpa_ie_fixed_t *wps_ie;
	s32 passive_scan;
	bool iscan_req;
	bool escan_req;
	bool spec_scan;
	bool p2p_ssid;
	s32 err = 0;
	s32 i;
	u32 wpsie_len = 0;
	u8 wpsie[IE_MAX_LEN];

	WL_DBG(("Enter wiphy (%p)\n", wiphy));
	if (unlikely(wl_get_drv_status(wl, SCANNING))) {
		WL_ERR(("Scanning already : status (%d)\n", (int)wl->status));
		return -EAGAIN;
	}
	if (unlikely(wl_get_drv_status(wl, SCAN_ABORTING))) {
		WL_ERR(("Scanning being aborted : status (%d)\n",
			(int)wl->status));
		return -EAGAIN;
	}
	if (request->n_ssids > WL_SCAN_PARAMS_SSID_MAX) {
		WL_ERR(("n_ssids > WL_SCAN_PARAMS_SSID_MAX\n"));
		return -EOPNOTSUPP;
	}

	/* Arm scan timeout timer */
	mod_timer(&wl->scan_timeout, jiffies + WL_SCAN_TIMER_INTERVAL_MS * HZ / 1000);
	iscan_req = false;
	spec_scan = false;
	if (request) {		/* scan bss */
		ssids = request->ssids;
		if (wl->iscan_on && (!ssids || !ssids->ssid_len || request->n_ssids != 1)) {
			iscan_req = true;
		} else if (wl->escan_on) {
			escan_req = true;
			p2p_ssid = false;
			for (i = 0; i < request->n_ssids; i++) {
				if (ssids[i].ssid_len && IS_P2P_SSID(ssids[i].ssid)) {
					p2p_ssid = true;
					break;
				}
			}
			if (p2p_ssid) {
				if (wl->p2p_supported) {
					/* p2p scan trigger */
					if (p2p_on(wl) == false) {
						/* p2p on at the first time */
						p2p_on(wl) = true;
						wl_cfgp2p_set_firm_p2p(wl);
					}
					p2p_scan(wl) = true;
				}
			} else {
				/* legacy scan trigger
				 * So, we have to disable p2p discovery if p2p discovery is on
				 */
				if (wl->p2p_supported) {
					p2p_scan(wl) = false;
					/* If Netdevice is not equals to primary and p2p is on
					*  , we will do p2p scan using P2PAPI_BSSCFG_DEVICE.
					*/
					if (p2p_on(wl) && (ndev != wl_to_prmry_ndev(wl)))
						p2p_scan(wl) = true;

					if (p2p_scan(wl) == false) {
						if (wl_get_p2p_status(wl, DISCOVERY_ON)) {
							err = wl_cfgp2p_discover_enable_search(wl,
							false);
							if (unlikely(err)) {
								goto scan_out;
							}

						}
					}
				}
				if (!wl->p2p_supported || !p2p_scan(wl)) {
					if (ndev == wl_to_prmry_ndev(wl)) {
						/* find the WPSIE */
						memset(wpsie, 0, sizeof(wpsie));
						if ((wps_ie = wl_cfgp2p_find_wpsie(
							(u8 *)request->ie,
							request->ie_len)) != NULL) {
							wpsie_len =
							wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
							memcpy(wpsie, wps_ie, wpsie_len);
						} else {
							wpsie_len = 0;
						}
						err = wl_cfgp2p_set_management_ie(wl, ndev, -1,
							VNDR_IE_PRBREQ_FLAG, wpsie, wpsie_len);
						if (unlikely(err)) {
							goto scan_out;
						}
					}
				}
			}
		}
	} else {		/* scan in ibss */
		/* we don't do iscan in ibss */
		ssids = this_ssid;
	}
	wl->scan_request = request;
	wl_set_drv_status(wl, SCANNING);
	if (iscan_req) {
		err = wl_do_iscan(wl, request);
		if (likely(!err))
			return err;
		else
			goto scan_out;
	} else if (escan_req) {
		if (wl->p2p_supported) {
			if (p2p_on(wl) && p2p_scan(wl)) {

				err = wl_cfgp2p_enable_discovery(wl, ndev,
				request->ie, request->ie_len);

				if (unlikely(err)) {
					goto scan_out;
				}
			}
		}
		err = wl_do_escan(wl, wiphy, ndev, request);
		if (likely(!err))
			return err;
		else
			goto scan_out;


	} else {
		memset(&sr->ssid, 0, sizeof(sr->ssid));
		sr->ssid.SSID_len =
			min_t(u8, sizeof(sr->ssid.SSID), ssids->ssid_len);
		if (sr->ssid.SSID_len) {
			memcpy(sr->ssid.SSID, ssids->ssid, sr->ssid.SSID_len);
			sr->ssid.SSID_len = htod32(sr->ssid.SSID_len);
			WL_SCAN(("Specific scan ssid=\"%s\" len=%d\n",
				sr->ssid.SSID, sr->ssid.SSID_len));
			spec_scan = true;
		} else {
			WL_SCAN(("Broadcast scan\n"));
		}
		WL_SCAN(("sr->ssid.SSID_len (%d)\n", sr->ssid.SSID_len));
		passive_scan = wl->active_scan ? 0 : 1;
		err = wldev_ioctl(ndev, WLC_SET_PASSIVE_SCAN,
			&passive_scan, sizeof(passive_scan), false);
		if (unlikely(err)) {
			WL_SCAN(("WLC_SET_PASSIVE_SCAN error (%d)\n", err));
			goto scan_out;
		}
		err = wldev_ioctl(ndev, WLC_SCAN, &sr->ssid,
			sizeof(sr->ssid), false);
		if (err) {
			if (err == -EBUSY) {
				WL_ERR(("system busy : scan for \"%s\" "
					"canceled\n", sr->ssid.SSID));
			} else {
				WL_ERR(("WLC_SCAN error (%d)\n", err));
			}
			goto scan_out;
		}
	}

	return 0;

scan_out:
	wl_clr_drv_status(wl, SCANNING);
	wl->scan_request = NULL;
	return err;
}

static s32
wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request)
{
	s32 err = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);

	WL_DBG(("Enter \n"));
	CHECK_SYS_UP(wl);

	err = __wl_cfg80211_scan(wiphy, ndev, request, NULL);
	if (unlikely(err)) {
		WL_ERR(("scan error (%d)\n", err));
		return err;
	}

	return err;
}

static s32 wl_dev_intvar_set(struct net_device *dev, s8 *name, s32 val)
{
	s8 buf[WLC_IOCTL_SMLEN];
	u32 len;
	s32 err = 0;

	val = htod32(val);
	len = bcm_mkiovar(name, (char *)(&val), sizeof(val), buf, sizeof(buf));
	BUG_ON(unlikely(!len));

	err = wldev_ioctl(dev, WLC_SET_VAR, buf, len, false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
	}

	return err;
}

static s32
wl_dev_intvar_get(struct net_device *dev, s8 *name, s32 *retval)
{
	union {
		s8 buf[WLC_IOCTL_SMLEN];
		s32 val;
	} var;
	u32 len;
	u32 data_null;
	s32 err = 0;

	len = bcm_mkiovar(name, (char *)(&data_null), 0,
		(char *)(&var), sizeof(var.buf));
	BUG_ON(unlikely(!len));
	err = wldev_ioctl(dev, WLC_GET_VAR, &var, len, false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
	}
	*retval = dtoh32(var.val);

	return err;
}

static s32 wl_set_rts(struct net_device *dev, u32 rts_threshold)
{
	s32 err = 0;

	err = wl_dev_intvar_set(dev, "rtsthresh", rts_threshold);
	if (unlikely(err)) {
		WL_ERR(("Error (%d)\n", err));
		return err;
	}
	return err;
}

static s32 wl_set_frag(struct net_device *dev, u32 frag_threshold)
{
	s32 err = 0;

	err = wl_dev_intvar_set(dev, "fragthresh", frag_threshold);
	if (unlikely(err)) {
		WL_ERR(("Error (%d)\n", err));
		return err;
	}
	return err;
}

static s32 wl_set_retry(struct net_device *dev, u32 retry, bool l)
{
	s32 err = 0;
	u32 cmd = (l ? WLC_SET_LRL : WLC_SET_SRL);

	retry = htod32(retry);
	err = wldev_ioctl(dev, cmd, &retry, sizeof(retry), false);
	if (unlikely(err)) {
		WL_ERR(("cmd (%d) , error (%d)\n", cmd, err));
		return err;
	}
	return err;
}

static s32 wl_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	struct wl_priv *wl = (struct wl_priv *)wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 err = 0;

	CHECK_SYS_UP(wl);
	if (changed & WIPHY_PARAM_RTS_THRESHOLD &&
		(wl->conf->rts_threshold != wiphy->rts_threshold)) {
		wl->conf->rts_threshold = wiphy->rts_threshold;
		err = wl_set_rts(ndev, wl->conf->rts_threshold);
		if (!err)
			return err;
	}
	if (changed & WIPHY_PARAM_FRAG_THRESHOLD &&
		(wl->conf->frag_threshold != wiphy->frag_threshold)) {
		wl->conf->frag_threshold = wiphy->frag_threshold;
		err = wl_set_frag(ndev, wl->conf->frag_threshold);
		if (!err)
			return err;
	}
	if (changed & WIPHY_PARAM_RETRY_LONG &&
		(wl->conf->retry_long != wiphy->retry_long)) {
		wl->conf->retry_long = wiphy->retry_long;
		err = wl_set_retry(ndev, wl->conf->retry_long, true);
		if (!err)
			return err;
	}
	if (changed & WIPHY_PARAM_RETRY_SHORT &&
		(wl->conf->retry_short != wiphy->retry_short)) {
		wl->conf->retry_short = wiphy->retry_short;
		err = wl_set_retry(ndev, wl->conf->retry_short, false);
		if (!err) {
			return err;
		}
	}

	return err;
}

static s32
wl_cfg80211_join_ibss(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_ibss_params *params)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct cfg80211_bss *bss;
	struct ieee80211_channel *chan;
	struct wl_join_params join_params;
	struct cfg80211_ssid ssid;
	s32 scan_retry = 0;
	s32 err = 0;
	bool rollback_lock = false;

	WL_TRACE(("In\n"));
	CHECK_SYS_UP(wl);
	if (params->bssid) {
		WL_ERR(("Invalid bssid\n"));
		return -EOPNOTSUPP;
	}
	bss = cfg80211_get_ibss(wiphy, NULL, params->ssid, params->ssid_len);
	if (!bss) {
		memcpy(ssid.ssid, params->ssid, params->ssid_len);
		ssid.ssid_len = params->ssid_len;
		do {
			if (unlikely
				(__wl_cfg80211_scan(wiphy, dev, NULL, &ssid) ==
				 -EBUSY)) {
				wl_delay(150);
			} else {
				break;
			}
		} while (++scan_retry < WL_SCAN_RETRY_MAX);
		/* to allow scan_inform to propagate to cfg80211 plane */
		if (rtnl_is_locked()) {
			rtnl_unlock();
			rollback_lock = true;
		}

		/* wait 4 secons till scan done.... */
		schedule_timeout_interruptible(4 * HZ);
		if (rollback_lock)
			rtnl_lock();
		bss = cfg80211_get_ibss(wiphy, NULL,
			params->ssid, params->ssid_len);
	}
	if (bss) {
		wl->ibss_starter = false;
		WL_DBG(("Found IBSS\n"));
	} else {
		wl->ibss_starter = true;
	}
	chan = params->channel;
	if (chan)
		wl->channel = ieee80211_frequency_to_channel(chan->center_freq);
	/*
	 * Join with specific BSSID and cached SSID
	 * If SSID is zero join based on BSSID only
	 */
	memset(&join_params, 0, sizeof(join_params));
	memcpy((void *)join_params.ssid.SSID, (void *)params->ssid,
		params->ssid_len);
	join_params.ssid.SSID_len = htod32(params->ssid_len);
	if (params->bssid)
		memcpy(&join_params.params.bssid, params->bssid,
			ETHER_ADDR_LEN);
	else
		memset(&join_params.params.bssid, 0, ETHER_ADDR_LEN);

	err = wldev_ioctl(dev, WLC_SET_SSID, &join_params,
		sizeof(join_params), false);
	if (unlikely(err)) {
		WL_ERR(("Error (%d)\n", err));
		return err;
	}
	return err;
}

static s32 wl_cfg80211_leave_ibss(struct wiphy *wiphy, struct net_device *dev)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;

	CHECK_SYS_UP(wl);
	wl_link_down(wl);

	return err;
}

static s32
wl_set_wpa_version(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 val = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1)
		val = WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED;
	else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)
		val = WPA2_AUTH_PSK| WPA2_AUTH_UNSPECIFIED;
	else
		val = WPA_AUTH_DISABLED;

	if (is_wps_conn(sme))
		val = WPA_AUTH_DISABLED;

	WL_DBG(("setting wpa_auth to 0x%0x\n", val));
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", val, bssidx);
	if (unlikely(err)) {
		WL_ERR(("set wpa_auth failed (%d)\n", err));
		return err;
	}
	sec = wl_read_prof(wl, WL_PROF_SEC);
	sec->wpa_versions = sme->crypto.wpa_versions;
	return err;
}

static s32
wl_set_auth_type(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 val = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);
	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		val = 0;
		WL_DBG(("open system\n"));
		break;
	case NL80211_AUTHTYPE_SHARED_KEY:
		val = 1;
		WL_DBG(("shared key\n"));
		break;
	case NL80211_AUTHTYPE_AUTOMATIC:
		val = 2;
		WL_DBG(("automatic\n"));
		break;
	case NL80211_AUTHTYPE_NETWORK_EAP:
		WL_DBG(("network eap\n"));
	default:
		val = 2;
		WL_ERR(("invalid auth type (%d)\n", sme->auth_type));
		break;
	}

	err = wldev_iovar_setint_bsscfg(dev, "auth", val, bssidx);
	if (unlikely(err)) {
		WL_ERR(("set auth failed (%d)\n", err));
		return err;
	}
	sec = wl_read_prof(wl, WL_PROF_SEC);
	sec->auth_type = sme->auth_type;
	return err;
}

static s32
wl_set_set_cipher(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 pval = 0;
	s32 gval = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (sme->crypto.n_ciphers_pairwise) {
		switch (sme->crypto.ciphers_pairwise[0]) {
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			pval = WEP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			pval = TKIP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			pval = AES_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			pval = AES_ENABLED;
			break;
		default:
			WL_ERR(("invalid cipher pairwise (%d)\n",
				sme->crypto.ciphers_pairwise[0]));
			return -EINVAL;
		}
	}
	if (sme->crypto.cipher_group) {
		switch (sme->crypto.cipher_group) {
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			gval = WEP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			gval = TKIP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			gval = AES_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			gval = AES_ENABLED;
			break;
		default:
			WL_ERR(("invalid cipher group (%d)\n",
				sme->crypto.cipher_group));
			return -EINVAL;
		}
	}

	WL_DBG(("pval (%d) gval (%d)\n", pval, gval));

	if (is_wps_conn(sme)) {
		err = wldev_iovar_setint_bsscfg(dev, "wsec", 4, bssidx);
	} else {
		WL_DBG((" NO, is_wps_conn, Set pval | gval to WSEC"));
		err = wldev_iovar_setint_bsscfg(dev, "wsec",
				pval | gval, bssidx);
	}
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}

	sec = wl_read_prof(wl, WL_PROF_SEC);
	sec->cipher_pairwise = sme->crypto.ciphers_pairwise[0];
	sec->cipher_group = sme->crypto.cipher_group;

	return err;
}

static s32
wl_set_key_mgmt(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 val = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (sme->crypto.n_akm_suites) {
		err = wl_dev_intvar_get(dev, "wpa_auth", &val);
		if (unlikely(err)) {
			WL_ERR(("could not get wpa_auth (%d)\n", err));
			return err;
		}
		if (val & (WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED)) {
			switch (sme->crypto.akm_suites[0]) {
			case WLAN_AKM_SUITE_8021X:
				val = WPA_AUTH_UNSPECIFIED;
				break;
			case WLAN_AKM_SUITE_PSK:
				val = WPA_AUTH_PSK;
				break;
			default:
				WL_ERR(("invalid cipher group (%d)\n",
					sme->crypto.cipher_group));
				return -EINVAL;
			}
		} else if (val & (WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED)) {
			switch (sme->crypto.akm_suites[0]) {
			case WLAN_AKM_SUITE_8021X:
				val = WPA2_AUTH_UNSPECIFIED;
				break;
			case WLAN_AKM_SUITE_PSK:
				val = WPA2_AUTH_PSK;
				break;
			default:
				WL_ERR(("invalid cipher group (%d)\n",
					sme->crypto.cipher_group));
				return -EINVAL;
			}
		}
		WL_DBG(("setting wpa_auth to %d\n", val));

		err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", val, bssidx);
		if (unlikely(err)) {
			WL_ERR(("could not set wpa_auth (%d)\n", err));
			return err;
		}
	}
	sec = wl_read_prof(wl, WL_PROF_SEC);
	sec->wpa_auth = sme->crypto.akm_suites[0];

	return err;
}

static s32
wl_set_set_sharedkey(struct net_device *dev,
	struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	struct wl_wsec_key key;
	s32 val;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("key len (%d)\n", sme->key_len));
	if (sme->key_len) {
		sec = wl_read_prof(wl, WL_PROF_SEC);
		WL_DBG(("wpa_versions 0x%x cipher_pairwise 0x%x\n",
			sec->wpa_versions, sec->cipher_pairwise));
		if (!(sec->wpa_versions & (NL80211_WPA_VERSION_1 |
			NL80211_WPA_VERSION_2)) &&
			(sec->cipher_pairwise & (WLAN_CIPHER_SUITE_WEP40 |
			WLAN_CIPHER_SUITE_WEP104))) {
			memset(&key, 0, sizeof(key));
			key.len = (u32) sme->key_len;
			key.index = (u32) sme->key_idx;
			if (unlikely(key.len > sizeof(key.data))) {
				WL_ERR(("Too long key length (%u)\n", key.len));
				return -EINVAL;
			}
			memcpy(key.data, sme->key, key.len);
			key.flags = WL_PRIMARY_KEY;
			switch (sec->cipher_pairwise) {
			case WLAN_CIPHER_SUITE_WEP40:
				key.algo = CRYPTO_ALGO_WEP1;
				break;
			case WLAN_CIPHER_SUITE_WEP104:
				key.algo = CRYPTO_ALGO_WEP128;
				break;
			default:
				WL_ERR(("Invalid algorithm (%d)\n",
					sme->crypto.ciphers_pairwise[0]));
				return -EINVAL;
			}
			/* Set the new key/index */
			WL_DBG(("key length (%d) key index (%d) algo (%d)\n",
				key.len, key.index, key.algo));
			WL_DBG(("key \"%s\"\n", key.data));
			swap_key_from_BE(&key);
			err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key),
				ioctlbuf, sizeof(ioctlbuf), bssidx);
			if (unlikely(err)) {
				WL_ERR(("WLC_SET_KEY error (%d)\n", err));
				return err;
			}
			if (sec->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM) {
				WL_DBG(("set auth_type to shared key\n"));
				val = 1;	/* shared key */
				err = wldev_iovar_setint_bsscfg(dev, "auth", val, bssidx);
				if (unlikely(err)) {
					WL_ERR(("set auth failed (%d)\n", err));
					return err;
				}
			}
		}
	}
	return err;
}

static s32
wl_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct ieee80211_channel *chan = sme->channel;
	wl_extjoin_params_t *ext_join_params;
	struct wl_join_params join_params;
	size_t join_params_size;
	s32 err = 0;
	wpa_ie_fixed_t *wpa_ie;
	wpa_ie_fixed_t *wps_ie;
	bcm_tlv_t *wpa2_ie;
	u8* wpaie  = 0;
	u32 wpaie_len = 0;
	u32 wpsie_len = 0;
	u32 chan_cnt = 0;
	u8 wpsie[IE_MAX_LEN];
	struct ether_addr bssid;

	WL_DBG(("In\n"));
	CHECK_SYS_UP(wl);

	/*
	 * Cancel ongoing scan to sync up with sme state machine of cfg80211.
	 */
	if (wl->scan_request) {
		wl_cfg80211_scan_abort(wl, dev);
	}
	/* Clean BSSID */
	bzero(&bssid, sizeof(bssid));
	wl_update_prof(wl, NULL, (void *)&bssid, WL_PROF_BSSID);

	if (IS_P2P_SSID(sme->ssid) && (dev != wl_to_prmry_ndev(wl))) {
		/* we only allow to connect using virtual interface in case of P2P */
		if (p2p_on(wl) && is_wps_conn(sme)) {
			WL_DBG(("ASSOC1 p2p index : %d sme->ie_len %d\n",
				wl_cfgp2p_find_idx(wl, dev), sme->ie_len));
			/* Have to apply WPS IE + P2P IE in assoc req frame */
			wl_cfgp2p_set_management_ie(wl, dev,
				wl_cfgp2p_find_idx(wl, dev), VNDR_IE_PRBREQ_FLAG,
				wl_to_p2p_bss_saved_ie(wl, P2PAPI_BSSCFG_DEVICE).p2p_probe_req_ie,
				wl_to_p2p_bss_saved_ie(wl,
				P2PAPI_BSSCFG_DEVICE).p2p_probe_req_ie_len);
			wl_cfgp2p_set_management_ie(wl, dev, wl_cfgp2p_find_idx(wl, dev),
				VNDR_IE_ASSOCREQ_FLAG, sme->ie, sme->ie_len);
		} else if (p2p_on(wl) && (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)) {
			/* This is the connect req after WPS is done [credentials exchanged]
			 * currently identified with WPA_VERSION_2 .
			 * Update the previously set IEs with
			 * the newly received IEs from Supplicant. This will remove the WPS IE from
			 * the Assoc Req.
			 */
			WL_DBG(("ASSOC2 p2p index : %d sme->ie_len %d\n",
				wl_cfgp2p_find_idx(wl, dev), sme->ie_len));
			wl_cfgp2p_set_management_ie(wl, dev, wl_cfgp2p_find_idx(wl, dev),
				VNDR_IE_ASSOCREQ_FLAG, sme->ie, sme->ie_len);
		}

	} else if (dev == wl_to_prmry_ndev(wl)) {
			/* find the RSN_IE */
			if ((wpa2_ie = bcm_parse_tlvs((u8 *)sme->ie, sme->ie_len,
				DOT11_MNG_RSN_ID)) != NULL) {
				WL_DBG((" WPA2 IE is found\n"));
			}
			/* find the WPA_IE */
			if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)sme->ie,
				sme->ie_len)) != NULL) {
				WL_DBG((" WPA IE is found\n"));
			}
			if (wpa_ie != NULL || wpa2_ie != NULL) {
				wpaie = (wpa_ie != NULL) ? (u8 *)wpa_ie : (u8 *)wpa2_ie;
				wpaie_len = (wpa_ie != NULL) ? wpa_ie->length : wpa2_ie->len;
				wpaie_len += WPA_RSN_IE_TAG_FIXED_LEN;
				wldev_iovar_setbuf(dev, "wpaie", wpaie, wpaie_len,
				ioctlbuf, sizeof(ioctlbuf));
			} else {
				wldev_iovar_setbuf(dev, "wpaie", NULL, 0,
				ioctlbuf, sizeof(ioctlbuf));
			}

			/* find the WPSIE */
			memset(wpsie, 0, sizeof(wpsie));
			if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)sme->ie,
				sme->ie_len)) != NULL) {
				wpsie_len = wps_ie->length +WPA_RSN_IE_TAG_FIXED_LEN;
				memcpy(wpsie, wps_ie, wpsie_len);
			} else {
				wpsie_len = 0;
			}
			err = wl_cfgp2p_set_management_ie(wl, dev, -1,
				VNDR_IE_ASSOCREQ_FLAG, wpsie, wpsie_len);
			if (unlikely(err)) {
				return err;
			}
	}
	if (unlikely(!sme->ssid)) {
		WL_ERR(("Invalid ssid\n"));
		return -EOPNOTSUPP;
	}
	if (chan) {
		wl->channel = ieee80211_frequency_to_channel(chan->center_freq);
		chan_cnt = 1;
		WL_DBG(("channel (%d), center_req (%d)\n", wl->channel,
			chan->center_freq));
	} else
		wl->channel = 0;
	WL_DBG(("ie (%p), ie_len (%zd)\n", sme->ie, sme->ie_len));
	err = wl_set_wpa_version(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid wpa_version\n"));
		return err;
	}

	err = wl_set_auth_type(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid auth type\n"));
		return err;
	}

	err = wl_set_set_cipher(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid ciper\n"));
		return err;
	}

	err = wl_set_key_mgmt(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid key mgmt\n"));
		return err;
	}

	err = wl_set_set_sharedkey(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid shared key\n"));
		return err;
	}

	/*
	 *  Join with specific BSSID and cached SSID
	 *  If SSID is zero join based on BSSID only
	 */
	join_params_size = WL_EXTJOIN_PARAMS_FIXED_SIZE +
		chan_cnt * sizeof(chanspec_t);
	ext_join_params =  (wl_extjoin_params_t*)kzalloc(join_params_size, GFP_KERNEL);
	if (ext_join_params == NULL) {
		err = -ENOMEM;
		wl_clr_drv_status(wl, CONNECTING);
		goto exit;
	}
	ext_join_params->ssid.SSID_len = min(sizeof(ext_join_params->ssid.SSID), sme->ssid_len);
	memcpy(&ext_join_params->ssid.SSID, sme->ssid, ext_join_params->ssid.SSID_len);
	ext_join_params->ssid.SSID_len = htod32(ext_join_params->ssid.SSID_len);
	/* Set up join scan parameters */
	ext_join_params->scan.scan_type = -1;
	ext_join_params->scan.nprobes = 2;
	/* increate dwell time to receive probe response
	* from target AP at a noisy air
	*/
	ext_join_params->scan.active_time = 150;
	ext_join_params->scan.passive_time = 300;
	ext_join_params->scan.home_time = -1;
	if (sme->bssid)
		memcpy(&ext_join_params->assoc.bssid, sme->bssid, ETH_ALEN);
	else
		memcpy(&ext_join_params->assoc.bssid, &ether_bcast, ETH_ALEN);
	ext_join_params->assoc.chanspec_num = chan_cnt;
	if (chan_cnt) {
		u16 channel, band, bw, ctl_sb;
		chanspec_t chspec;
		channel = wl->channel;
		band = (channel <= CH_MAX_2G_CHANNEL) ? WL_CHANSPEC_BAND_2G
			: WL_CHANSPEC_BAND_5G;
		bw = WL_CHANSPEC_BW_20;
		ctl_sb = WL_CHANSPEC_CTL_SB_NONE;
		chspec = (channel | band | bw | ctl_sb);
		ext_join_params->assoc.chanspec_list[0]  &= WL_CHANSPEC_CHAN_MASK;
		ext_join_params->assoc.chanspec_list[0] |= chspec;
		ext_join_params->assoc.chanspec_list[0] =
			htodchanspec(ext_join_params->assoc.chanspec_list[0]);
	}
	ext_join_params->assoc.chanspec_num = htod32(ext_join_params->assoc.chanspec_num);
	if (ext_join_params->ssid.SSID_len < IEEE80211_MAX_SSID_LEN) {
		WL_INFO(("ssid \"%s\", len (%d)\n", ext_join_params->ssid.SSID,
			ext_join_params->ssid.SSID_len));
	}
	wl_set_drv_status(wl, CONNECTING);
	err = wldev_iovar_setbuf_bsscfg(dev, "join", ext_join_params, join_params_size, ioctlbuf,
		sizeof(ioctlbuf), wl_cfgp2p_find_idx(wl, dev));
	kfree(ext_join_params);
	if (err) {
		wl_clr_drv_status(wl, CONNECTING);
		if (err == BCME_UNSUPPORTED) {
			WL_DBG(("join iovar is not supported\n"));
			goto set_ssid;
		} else
			WL_ERR(("error (%d)\n", err));
	} else
		goto exit;

set_ssid:
	memset(&join_params, 0, sizeof(join_params));
	join_params_size = sizeof(join_params.ssid);

	join_params.ssid.SSID_len = min(sizeof(join_params.ssid.SSID), sme->ssid_len);
	memcpy(&join_params.ssid.SSID, sme->ssid, join_params.ssid.SSID_len);
	join_params.ssid.SSID_len = htod32(join_params.ssid.SSID_len);
	wl_update_prof(wl, NULL, &join_params.ssid, WL_PROF_SSID);
	if (sme->bssid)
		memcpy(&join_params.params.bssid, sme->bssid, ETH_ALEN);
	else
		memcpy(&join_params.params.bssid, &ether_bcast, ETH_ALEN);

	wl_ch_to_chanspec(wl->channel, &join_params, &join_params_size);
	WL_DBG(("join_param_size %d\n", join_params_size));

	if (join_params.ssid.SSID_len < IEEE80211_MAX_SSID_LEN) {
		WL_INFO(("ssid \"%s\", len (%d)\n", join_params.ssid.SSID,
			join_params.ssid.SSID_len));
	}
	wl_set_drv_status(wl, CONNECTING);
	err = wldev_ioctl(dev, WLC_SET_SSID, &join_params, join_params_size, true);
	if (err) {
		WL_ERR(("error (%d)\n", err));
		wl_clr_drv_status(wl, CONNECTING);
	}
exit:
	return err;
}

static s32
wl_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
	u16 reason_code)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	scb_val_t scbval;
	bool act = false;
	s32 err = 0;
	u8 *curbssid;
	WL_ERR(("Reason %d\n", reason_code));
	CHECK_SYS_UP(wl);
	act = *(bool *) wl_read_prof(wl, WL_PROF_ACT);
	curbssid = wl_read_prof(wl, WL_PROF_BSSID);
	if (likely(act)) {
		/*
		* Cancel ongoing scan to sync up with sme state machine of cfg80211.
		*/
		if (wl->scan_request) {
			wl_cfg80211_scan_abort(wl, dev);
		}
		wl_set_drv_status(wl, DISCONNECTING);
		scbval.val = reason_code;
		memcpy(&scbval.ea, curbssid, ETHER_ADDR_LEN);
		scbval.val = htod32(scbval.val);
		err = wldev_ioctl(dev, WLC_DISASSOC, &scbval,
			sizeof(scb_val_t), true);
		if (unlikely(err)) {
			wl_clr_drv_status(wl, DISCONNECTING);
			WL_ERR(("error (%d)\n", err));
			return err;
		}
	}

	return err;
}

static s32
wl_cfg80211_set_tx_power(struct wiphy *wiphy,
	enum nl80211_tx_power_setting type, s32 dbm)
{

	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	u16 txpwrmw;
	s32 err = 0;
	s32 disable = 0;

	CHECK_SYS_UP(wl);
	switch (type) {
	case NL80211_TX_POWER_AUTOMATIC:
		break;
	case NL80211_TX_POWER_LIMITED:
		if (dbm < 0) {
			WL_ERR(("TX_POWER_LIMITTED - dbm is negative\n"));
			return -EINVAL;
		}
		break;
	case NL80211_TX_POWER_FIXED:
		if (dbm < 0) {
			WL_ERR(("TX_POWER_FIXED - dbm is negative..\n"));
			return -EINVAL;
		}
		break;
	}
	/* Make sure radio is off or on as far as software is concerned */
	disable = WL_RADIO_SW_DISABLE << 16;
	disable = htod32(disable);
	err = wldev_ioctl(ndev, WLC_SET_RADIO, &disable, sizeof(disable), true);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_RADIO error (%d)\n", err));
		return err;
	}

	if (dbm > 0xffff)
		txpwrmw = 0xffff;
	else
		txpwrmw = (u16) dbm;
	err = wl_dev_intvar_set(ndev, "qtxpower",
		(s32) (bcm_mw_to_qdbm(txpwrmw)));
	if (unlikely(err)) {
		WL_ERR(("qtxpower error (%d)\n", err));
		return err;
	}
	wl->conf->tx_power = dbm;

	return err;
}

static s32 wl_cfg80211_get_tx_power(struct wiphy *wiphy, s32 *dbm)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 txpwrdbm;
	u8 result;
	s32 err = 0;

	CHECK_SYS_UP(wl);
	err = wl_dev_intvar_get(ndev, "qtxpower", &txpwrdbm);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	result = (u8) (txpwrdbm & ~WL_TXPWR_OVERRIDE);
	*dbm = (s32) bcm_qdbm_to_mw(result);

	return err;
}

static s32
wl_cfg80211_config_default_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool unicast, bool multicast)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	u32 index;
	s32 wsec;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("key index (%d)\n", key_idx));
	CHECK_SYS_UP(wl);
	err = wldev_iovar_getint_bsscfg(dev, "wsec", &wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("WLC_GET_WSEC error (%d)\n", err));
		return err;
	}
	if (wsec & WEP_ENABLED) {
		/* Just select a new current key */
		index = (u32) key_idx;
		index = htod32(index);
		err = wldev_ioctl(dev, WLC_SET_KEY_PRIMARY, &index,
			sizeof(index), true);
		if (unlikely(err)) {
			WL_ERR(("error (%d)\n", err));
		}
	}
	return err;
}

static s32
wl_add_keyext(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, const u8 *mac_addr, struct key_params *params)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct wl_wsec_key key;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);
	s32 mode = get_mode_by_netdev(wl, dev);
	memset(&key, 0, sizeof(key));
	key.index = (u32) key_idx;

	if (!ETHER_ISMULTI(mac_addr))
		memcpy((char *)&key.ea, (void *)mac_addr, ETHER_ADDR_LEN);
	key.len = (u32) params->key_len;

	/* check for key index change */
	if (key.len == 0) {
		/* key delete */
		swap_key_from_BE(&key);
		wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key), ioctlbuf,
			sizeof(ioctlbuf), bssidx);
		if (unlikely(err)) {
			WL_ERR(("key delete error (%d)\n", err));
			return err;
		}
	} else {
		if (key.len > sizeof(key.data)) {
			WL_ERR(("Invalid key length (%d)\n", key.len));
			return -EINVAL;
		}
		WL_DBG(("Setting the key index %d\n", key.index));
		memcpy(key.data, params->key, key.len);

		if ((mode == WL_MODE_BSS) &&
			(params->cipher == WLAN_CIPHER_SUITE_TKIP)) {
			u8 keybuf[8];
			memcpy(keybuf, &key.data[24], sizeof(keybuf));
			memcpy(&key.data[24], &key.data[16], sizeof(keybuf));
			memcpy(&key.data[16], keybuf, sizeof(keybuf));
		}

		/* if IW_ENCODE_EXT_RX_SEQ_VALID set */
		if (params->seq && params->seq_len == 6) {
			/* rx iv */
			u8 *ivptr;
			ivptr = (u8 *) params->seq;
			key.rxiv.hi = (ivptr[5] << 24) | (ivptr[4] << 16) |
				(ivptr[3] << 8) | ivptr[2];
			key.rxiv.lo = (ivptr[1] << 8) | ivptr[0];
			key.iv_initialized = true;
		}

		switch (params->cipher) {
		case WLAN_CIPHER_SUITE_WEP40:
			key.algo = CRYPTO_ALGO_WEP1;
			WL_DBG(("WLAN_CIPHER_SUITE_WEP40\n"));
			break;
		case WLAN_CIPHER_SUITE_WEP104:
			key.algo = CRYPTO_ALGO_WEP128;
			WL_DBG(("WLAN_CIPHER_SUITE_WEP104\n"));
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			key.algo = CRYPTO_ALGO_TKIP;
			WL_DBG(("WLAN_CIPHER_SUITE_TKIP\n"));
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			key.algo = CRYPTO_ALGO_AES_CCM;
			WL_DBG(("WLAN_CIPHER_SUITE_AES_CMAC\n"));
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			key.algo = CRYPTO_ALGO_AES_CCM;
			WL_DBG(("WLAN_CIPHER_SUITE_CCMP\n"));
			break;
		default:
			WL_ERR(("Invalid cipher (0x%x)\n", params->cipher));
			return -EINVAL;
		}
		swap_key_from_BE(&key);
#ifdef CONFIG_WIRELESS_EXT
		dhd_wait_pend8021x(dev);
#endif
		wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key), ioctlbuf,
			sizeof(ioctlbuf), bssidx);
		if (unlikely(err)) {
			WL_ERR(("WLC_SET_KEY error (%d)\n", err));
			return err;
		}
	}
	return err;
}

static s32
wl_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr,
	struct key_params *params)
{
	struct wl_wsec_key key;
	s32 val = 0;
	s32 wsec = 0;
	s32 err = 0;
	u8 keybuf[8];
	s32 bssidx = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 mode = get_mode_by_netdev(wl, dev);
	WL_DBG(("key index (%d)\n", key_idx));
	CHECK_SYS_UP(wl);

	bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (mac_addr) {
		wl_add_keyext(wiphy, dev, key_idx, mac_addr, params);
		goto exit;
	}
	memset(&key, 0, sizeof(key));

	key.len = (u32) params->key_len;
	key.index = (u32) key_idx;

	if (unlikely(key.len > sizeof(key.data))) {
		WL_ERR(("Too long key length (%u)\n", key.len));
		return -EINVAL;
	}
	memcpy(key.data, params->key, key.len);

	key.flags = WL_PRIMARY_KEY;
	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		key.algo = CRYPTO_ALGO_WEP1;
		val = WEP_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_WEP40\n"));
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		key.algo = CRYPTO_ALGO_WEP128;
		val = WEP_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_WEP104\n"));
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		key.algo = CRYPTO_ALGO_TKIP;
		val = TKIP_ENABLED;
		/* wpa_supplicant switches the third and fourth quarters of the TKIP key */
		if (mode == WL_MODE_BSS) {
			bcopy(&key.data[24], keybuf, sizeof(keybuf));
			bcopy(&key.data[16], &key.data[24], sizeof(keybuf));
			bcopy(keybuf, &key.data[16], sizeof(keybuf));
		}
		WL_DBG(("WLAN_CIPHER_SUITE_TKIP\n"));
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		key.algo = CRYPTO_ALGO_AES_CCM;
		val = AES_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_AES_CMAC\n"));
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key.algo = CRYPTO_ALGO_AES_CCM;
		val = AES_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_CCMP\n"));
		break;
	default:
		WL_ERR(("Invalid cipher (0x%x)\n", params->cipher));
		return -EINVAL;
	}

	/* Set the new key/index */
	swap_key_from_BE(&key);
	err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key), ioctlbuf,
		sizeof(ioctlbuf), bssidx);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_KEY error (%d)\n", err));
		return err;
	}

exit:
	err = wldev_iovar_getint_bsscfg(dev, "wsec", &wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("get wsec error (%d)\n", err));
		return err;
	}

	wsec |= val;
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("set wsec error (%d)\n", err));
		return err;
	}

	return err;
}

static s32
wl_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr)
{
	struct wl_wsec_key key;
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("Enter\n"));
	CHECK_SYS_UP(wl);
	memset(&key, 0, sizeof(key));

	key.index = (u32) key_idx;
	key.flags = WL_PRIMARY_KEY;
	key.algo = CRYPTO_ALGO_OFF;

	WL_DBG(("key index (%d)\n", key_idx));
	/* Set the new key/index */
	swap_key_from_BE(&key);
	wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key), ioctlbuf,
		sizeof(ioctlbuf), bssidx);
	if (unlikely(err)) {
		if (err == -EINVAL) {
			if (key.index >= DOT11_MAX_DEFAULT_KEYS) {
				/* we ignore this key index in this case */
				WL_DBG(("invalid key index (%d)\n", key_idx));
			}
		} else {
			WL_ERR(("WLC_SET_KEY error (%d)\n", err));
		}
		return err;
	}
	return err;
}

static s32
wl_cfg80211_get_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr, void *cookie,
	void (*callback) (void *cookie, struct key_params * params))
{
	struct key_params params;
	struct wl_wsec_key key;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct wl_security *sec;
	s32 wsec;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("key index (%d)\n", key_idx));
	CHECK_SYS_UP(wl);
	memset(&key, 0, sizeof(key));
	key.index = key_idx;
	swap_key_to_BE(&key);
	memset(&params, 0, sizeof(params));
	params.key_len = (u8) min_t(u8, DOT11_MAX_KEY_SIZE, key.len);
	memcpy(params.key, key.data, params.key_len);

	wldev_iovar_getint_bsscfg(dev, "wsec", &wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("WLC_GET_WSEC error (%d)\n", err));
		return err;
	}
	switch (wsec & ~SES_OW_ENABLED) {
		case WEP_ENABLED:
			sec = wl_read_prof(wl, WL_PROF_SEC);
			if (sec->cipher_pairwise & WLAN_CIPHER_SUITE_WEP40) {
				params.cipher = WLAN_CIPHER_SUITE_WEP40;
				WL_DBG(("WLAN_CIPHER_SUITE_WEP40\n"));
			} else if (sec->cipher_pairwise & WLAN_CIPHER_SUITE_WEP104) {
				params.cipher = WLAN_CIPHER_SUITE_WEP104;
				WL_DBG(("WLAN_CIPHER_SUITE_WEP104\n"));
			}
			break;
		case TKIP_ENABLED:
			params.cipher = WLAN_CIPHER_SUITE_TKIP;
			WL_DBG(("WLAN_CIPHER_SUITE_TKIP\n"));
			break;
		case AES_ENABLED:
			params.cipher = WLAN_CIPHER_SUITE_AES_CMAC;
			WL_DBG(("WLAN_CIPHER_SUITE_AES_CMAC\n"));
			break;
		default:
			WL_ERR(("Invalid algo (0x%x)\n", wsec));
			return -EINVAL;
	}

	callback(cookie, &params);
	return err;
}

static s32
wl_cfg80211_config_default_mgmt_key(struct wiphy *wiphy,
	struct net_device *dev, u8 key_idx)
{
	WL_INFO(("Not supported\n"));
	return -EOPNOTSUPP;
}

static s32
wl_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
	u8 *mac, struct station_info *sinfo)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	scb_val_t scb_val;
	s32 rssi;
	s32 rate;
	s32 err = 0;
	sta_info_t *sta;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	s8 eabuf[ETHER_ADDR_STR_LEN];
#endif
	dhd_pub_t *dhd =  (dhd_pub_t *)(wl->pub);

	CHECK_SYS_UP(wl);
	if (get_mode_by_netdev(wl, dev) == WL_MODE_AP) {
		err = wldev_iovar_getbuf(dev, "sta_info", (struct ether_addr *)mac,
			ETHER_ADDR_LEN, ioctlbuf, sizeof(ioctlbuf));
		if (err < 0) {
			WL_ERR(("GET STA INFO failed, %d\n", err));
			return err;
		}
		sinfo->filled = STATION_INFO_INACTIVE_TIME;
		sta = (sta_info_t *)ioctlbuf;
		sta->len = dtoh16(sta->len);
		sta->cap = dtoh16(sta->cap);
		sta->flags = dtoh32(sta->flags);
		sta->idle = dtoh32(sta->idle);
		sta->in = dtoh32(sta->in);
		sinfo->inactive_time = sta->idle * 1000;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
		if (sta->flags & WL_STA_ASSOC) {
			sinfo->filled |= STATION_INFO_CONNECTED_TIME;
			sinfo->connected_time = sta->in;
		}
		WL_INFO(("STA %s : idle time : %d sec, connected time :%d ms\n",
			bcm_ether_ntoa((const struct ether_addr *)mac, eabuf), sinfo->inactive_time,
			sta->idle * 1000));
#endif
	} else if (get_mode_by_netdev(wl, dev) == WL_MODE_BSS) {
			u8 *curmacp = wl_read_prof(wl, WL_PROF_BSSID);

			if (!wl_get_drv_status(wl, CONNECTED) ||
			    (dhd_is_associated(dhd, NULL) == FALSE)) {
				WL_ERR(("NOT assoc\n"));
				err = -ENODEV;
				goto get_station_err;
			}
			if (memcmp(mac, curmacp, ETHER_ADDR_LEN)) {
				WL_ERR(("Wrong Mac address: "MACSTR" != "MACSTR"\n",
					MAC2STR(mac), MAC2STR(curmacp)));
			}

			/* Report the current tx rate */
			err = wldev_ioctl(dev, WLC_GET_RATE, &rate, sizeof(rate), false);
			if (err) {
				WL_ERR(("Could not get rate (%d)\n", err));
			} else {
				rate = dtoh32(rate);
				sinfo->filled |= STATION_INFO_TX_BITRATE;
				sinfo->txrate.legacy = rate * 5;
				WL_DBG(("Rate %d Mbps\n", (rate / 2)));
			}

			memset(&scb_val, 0, sizeof(scb_val));
			scb_val.val = 0;
			err = wldev_ioctl(dev, WLC_GET_RSSI, &scb_val,
					sizeof(scb_val_t), false);
			if (err) {
				WL_ERR(("Could not get rssi (%d)\n", err));
				goto get_station_err;
			}

			rssi = dtoh32(scb_val.val);
			sinfo->filled |= STATION_INFO_SIGNAL;
			sinfo->signal = rssi;
			WL_DBG(("RSSI %d dBm\n", rssi));

get_station_err:
		if (err) {
			/* Disconnect due to zero BSSID or error to get RSSI */
			WL_ERR(("force cfg80211_disconnected\n"));
			wl_clr_drv_status(wl, CONNECTED);
			cfg80211_disconnected(dev, 0, NULL, 0, GFP_KERNEL);
			wl_link_down(wl);
		}
	}

	return err;
}

static s32
wl_cfg80211_set_power_mgmt(struct wiphy *wiphy, struct net_device *dev,
	bool enabled, s32 timeout)
{
	s32 pm;
	s32 err = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);

	CHECK_SYS_UP(wl);
	pm = enabled ? PM_FAST : PM_OFF;
	/* Do not enable the power save after assoc if it is p2p interface */
	if (wl->p2p && wl->p2p->vif_created) {
		WL_DBG(("Do not enable the power save for p2p interfaces even after assoc\n"));
		pm = PM_OFF;
	}
	pm = htod32(pm);
	WL_DBG(("power save %s\n", (pm ? "enabled" : "disabled")));
	err = wldev_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm), true);
	if (unlikely(err)) {
		if (err == -ENODEV)
			WL_DBG(("net_device is not ready yet\n"));
		else
			WL_ERR(("error (%d)\n", err));
		return err;
	}
	return err;
}

static __used u32 wl_find_msb(u16 bit16)
{
	u32 ret = 0;

	if (bit16 & 0xff00) {
		ret += 8;
		bit16 >>= 8;
	}

	if (bit16 & 0xf0) {
		ret += 4;
		bit16 >>= 4;
	}

	if (bit16 & 0xc) {
		ret += 2;
		bit16 >>= 2;
	}

	if (bit16 & 2)
		ret += bit16 & 2;
	else if (bit16)
		ret += bit16;

	return ret;
}

static s32 wl_cfg80211_resume(struct wiphy *wiphy)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;

	if (unlikely(!wl_get_drv_status(wl, READY))) {
		WL_INFO(("device is not ready : status (%d)\n",
			(int)wl->status));
		return 0;
	}

	wl_invoke_iscan(wl);

	return err;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
static s32 wl_cfg80211_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow)
#else
static s32 wl_cfg80211_suspend(struct wiphy *wiphy)
#endif
{
#ifdef DHD_CLEAR_ON_SUSPEND
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	unsigned long flags;

	if (unlikely(!wl_get_drv_status(wl, READY))) {
		WL_INFO(("device is not ready : status (%d)\n",
			(int)wl->status));
		return 0;
	}

	wl_set_drv_status(wl, SCAN_ABORTING);
	wl_term_iscan(wl);
	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	if (wl->scan_request) {
		cfg80211_scan_done(wl->scan_request, true);
		wl->scan_request = NULL;
	}
	wl_clr_drv_status(wl, SCANNING);
	wl_clr_drv_status(wl, SCAN_ABORTING);
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
	if (wl_get_drv_status(wl, CONNECTING)) {
		wl_bss_connect_done(wl, ndev, NULL, NULL, false);
	}
#endif
	return 0;
}

static __used s32
wl_update_pmklist(struct net_device *dev, struct wl_pmk_list *pmk_list,
	s32 err)
{
	int i, j;
	struct wl_priv *wl = wlcfg_drv_priv;
	struct net_device *primary_dev = wl_to_prmry_ndev(wl);

	/* Firmware is supporting pmk list only for STA interface i.e. primary interface
	  * Refer code wlc_bsscfg.c->wlc_bsscfg_sta_init
	  * Do we really need to support PMK cache in P2P in firmware?
	*/
	if (primary_dev != dev) {
		WL_INFO(("Not supporting Flushing pmklist on virtual"
			" interfaces than primary interface\n"));
		return err;
	}

	WL_DBG(("No of elements %d\n", pmk_list->pmkids.npmkid));
	for (i = 0; i < pmk_list->pmkids.npmkid; i++) {
		WL_DBG(("PMKID[%d]: %pM =\n", i,
			&pmk_list->pmkids.pmkid[i].BSSID));
		for (j = 0; j < WPA2_PMKID_LEN; j++) {
			WL_DBG(("%02x\n", pmk_list->pmkids.pmkid[i].PMKID[j]));
		}
	}
	if (likely(!err)) {
		err = wl_dev_bufvar_set(dev, "pmkid_info", (char *)pmk_list,
			sizeof(*pmk_list));
	}

	return err;
}

static s32
wl_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;
	int i;

	CHECK_SYS_UP(wl);
	for (i = 0; i < wl->pmk_list->pmkids.npmkid; i++)
		if (!memcmp(pmksa->bssid, &wl->pmk_list->pmkids.pmkid[i].BSSID,
			ETHER_ADDR_LEN))
			break;
	if (i < WL_NUM_PMKIDS_MAX) {
		memcpy(&wl->pmk_list->pmkids.pmkid[i].BSSID, pmksa->bssid,
			ETHER_ADDR_LEN);
		memcpy(&wl->pmk_list->pmkids.pmkid[i].PMKID, pmksa->pmkid,
			WPA2_PMKID_LEN);
		if (i == wl->pmk_list->pmkids.npmkid)
			wl->pmk_list->pmkids.npmkid++;
	} else {
		err = -EINVAL;
	}
	WL_DBG(("set_pmksa,IW_PMKSA_ADD - PMKID: %pM =\n",
		&wl->pmk_list->pmkids.pmkid[wl->pmk_list->pmkids.npmkid - 1].BSSID));
	for (i = 0; i < WPA2_PMKID_LEN; i++) {
		WL_DBG(("%02x\n",
			wl->pmk_list->pmkids.pmkid[wl->pmk_list->pmkids.npmkid - 1].
			PMKID[i]));
	}

	err = wl_update_pmklist(dev, wl->pmk_list, err);

	return err;
}

static s32
wl_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct _pmkid_list pmkid;
	s32 err = 0;
	int i;

	CHECK_SYS_UP(wl);
	memcpy(&pmkid.pmkid[0].BSSID, pmksa->bssid, ETHER_ADDR_LEN);
	memcpy(&pmkid.pmkid[0].PMKID, pmksa->pmkid, WPA2_PMKID_LEN);

	WL_DBG(("del_pmksa,IW_PMKSA_REMOVE - PMKID: %pM =\n",
		&pmkid.pmkid[0].BSSID));
	for (i = 0; i < WPA2_PMKID_LEN; i++) {
		WL_DBG(("%02x\n", pmkid.pmkid[0].PMKID[i]));
	}

	for (i = 0; i < wl->pmk_list->pmkids.npmkid; i++)
		if (!memcmp
		    (pmksa->bssid, &wl->pmk_list->pmkids.pmkid[i].BSSID,
		     ETHER_ADDR_LEN))
			break;

	if ((wl->pmk_list->pmkids.npmkid > 0) &&
		(i < wl->pmk_list->pmkids.npmkid)) {
		memset(&wl->pmk_list->pmkids.pmkid[i], 0, sizeof(pmkid_t));
		for (; i < (wl->pmk_list->pmkids.npmkid - 1); i++) {
			memcpy(&wl->pmk_list->pmkids.pmkid[i].BSSID,
				&wl->pmk_list->pmkids.pmkid[i + 1].BSSID,
				ETHER_ADDR_LEN);
			memcpy(&wl->pmk_list->pmkids.pmkid[i].PMKID,
				&wl->pmk_list->pmkids.pmkid[i + 1].PMKID,
				WPA2_PMKID_LEN);
		}
		wl->pmk_list->pmkids.npmkid--;
	} else {
		err = -EINVAL;
	}

	err = wl_update_pmklist(dev, wl->pmk_list, err);

	return err;

}

static s32
wl_cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *dev)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;
	CHECK_SYS_UP(wl);
	memset(wl->pmk_list, 0, sizeof(*wl->pmk_list));
	err = wl_update_pmklist(dev, wl->pmk_list, err);
	return err;

}

wl_scan_params_t *
wl_cfg80211_scan_alloc_params(int channel, int nprobes, int *out_params_size)
{
	wl_scan_params_t *params;
	int params_size;
	int num_chans;

	*out_params_size = 0;

	/* Our scan params only need space for 1 channel and 0 ssids */
	params_size = WL_SCAN_PARAMS_FIXED_SIZE + 1 * sizeof(uint16);
	params = (wl_scan_params_t*) kzalloc(params_size, GFP_KERNEL);
	if (params == NULL) {
		WL_ERR(("%s: mem alloc failed (%d bytes)\n", __func__, params_size));
		return params;
	}
	memset(params, 0, params_size);
	params->nprobes = nprobes;

	num_chans = (channel == 0) ? 0 : 1;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = DOT11_SCANTYPE_ACTIVE;
	params->nprobes = htod32(1);
	params->active_time = htod32(-1);
	params->passive_time = htod32(-1);
	params->home_time = htod32(10);
	params->channel_list[0] = htodchanspec(channel);

	/* Our scan params have 1 channel and 0 ssids */
	params->channel_num = htod32((0 << WL_SCAN_PARAMS_NSSID_SHIFT) |
	(num_chans & WL_SCAN_PARAMS_COUNT_MASK));

	*out_params_size = params_size;	/* rtn size to the caller */
	return params;
}

s32
wl_cfg80211_scan_abort(struct wl_priv *wl, struct net_device *ndev)
{
	wl_scan_params_t *params = NULL;
	s32 params_size = 0;
	s32 err = BCME_OK;
	unsigned long flags;

	WL_DBG(("Enter\n"));

	/* Our scan params only need space for 1 channel and 0 ssids */
	params = wl_cfg80211_scan_alloc_params(-1, 0, &params_size);
	if (params == NULL) {
		WL_ERR(("scan params allocation failed \n"));
		err = -ENOMEM;
	} else {
		/* Do a scan abort to stop the driver's scan engine */
		err = wldev_ioctl(ndev, WLC_SCAN, params, params_size, true);
		if (err < 0) {
			WL_ERR(("scan abort  failed \n"));
		}
	}
	del_timer_sync(&wl->scan_timeout);
	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	if (wl->scan_request) {
		cfg80211_scan_done(wl->scan_request, true);
		wl->scan_request = NULL;
	}
	wl_clr_drv_status(wl, SCANNING);
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
	if (params)
		kfree(params);
	return err;
}

static s32
wl_cfg80211_remain_on_channel(struct wiphy *wiphy, struct net_device *dev,
	struct ieee80211_channel * channel,
	enum nl80211_channel_type channel_type,
	unsigned int duration, u64 *cookie)
{
	s32 target_channel;

	s32 err = BCME_OK;
	struct wl_priv *wl = wiphy_priv(wiphy);
	dhd_pub_t *dhd = (dhd_pub_t *)(wl->pub);
	WL_DBG(("Enter, netdev_ifidx: %d \n", dev->ifindex));
	if (likely(wl_get_drv_status(wl, SCANNING))) {
		wl_cfg80211_scan_abort(wl, dev);
	}

	target_channel = ieee80211_frequency_to_channel(channel->center_freq);
	memcpy(&wl->remain_on_chan, channel, sizeof(struct ieee80211_channel));
	wl->remain_on_chan_type = channel_type;
	wl->cache_cookie = *cookie;
	cfg80211_ready_on_channel(dev, *cookie, channel,
		channel_type, duration, GFP_KERNEL);
	if (!p2p_on(wl)) {
		wl_cfgp2p_generate_bss_mac(&dhd->mac, &wl->p2p->dev_addr, &wl->p2p->int_addr);

		/* In case of p2p_listen command, supplicant send remain_on_channel
		 * without turning on P2P
		 */

		p2p_on(wl) = true;
		err = wl_cfgp2p_enable_discovery(wl, dev, NULL, 0);

		if (unlikely(err)) {
			goto exit;
		}
	}
	if (p2p_on(wl))
		wl_cfgp2p_discover_listen(wl, target_channel, duration);


exit:
	return err;
}

static s32
wl_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy, struct net_device *dev,
	u64 cookie)
{
	s32 err = 0;
	WL_DBG((" enter ) netdev_ifidx: %d \n", dev->ifindex));
	return err;
}

static s32
wl_cfg80211_mgmt_tx(struct wiphy *wiphy, struct net_device *dev,
	struct ieee80211_channel *channel, bool offchan,
	enum nl80211_channel_type channel_type,
	bool channel_type_valid, unsigned int wait,
	const u8* buf, size_t len, u64 *cookie)
{
	wl_action_frame_t *action_frame;
	wl_af_params_t *af_params;
	wifi_p2p_ie_t *p2p_ie;
	wpa_ie_fixed_t *wps_ie;
	const struct ieee80211_mgmt *mgmt;
	struct wl_priv *wl = wiphy_priv(wiphy);
	dhd_pub_t *dhd = (dhd_pub_t *)(wl->pub);
	s32 err = BCME_OK;
	s32 bssidx = 0;
	u32 p2pie_len = 0;
	u32 wpsie_len = 0;
	u16 fc;
	bool ack = false;
	wifi_p2p_pub_act_frame_t *act_frm;
	WL_DBG(("Enter \n"));
	/* find bssidx based on ndev */
	bssidx = wl_cfgp2p_find_idx(wl, dev);
	/* cookie generation */
	*cookie = (unsigned long) buf;

	if (bssidx == -1) {

		WL_ERR(("Can not find the bssidx for dev( %p )\n", dev));
		return -ENODEV;
	}
	if (wl->p2p_supported && p2p_on(wl)) {
		wl_cfgp2p_generate_bss_mac(&dhd->mac, &wl->p2p->dev_addr, &wl->p2p->int_addr);
		/* Suspend P2P discovery search-listen to prevent it from changing the
		 * channel.
		 */
		if ((err = wl_cfgp2p_discover_enable_search(wl, false)) < 0) {
			WL_ERR(("Can not disable discovery mode\n"));
			return -EFAULT;
		}
	}

	mgmt = (const struct ieee80211_mgmt *) buf;
	fc = mgmt->frame_control;
	if (fc != IEEE80211_STYPE_ACTION) {
		if (fc == IEEE80211_STYPE_PROBE_RESP) {
			s32 ie_offset =  DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
			s32 ie_len = len - ie_offset;
			if ((p2p_ie = wl_cfgp2p_find_p2pie((u8 *)(buf + ie_offset), ie_len))
				!= NULL) {
				/* Total length of P2P Information Element */
				p2pie_len = p2p_ie->len + sizeof(p2p_ie->len) + sizeof(p2p_ie->id);
				/* Have to change p2p device address in dev_info attribute
				 * because Supplicant use primary eth0 address
				 */
			#ifdef ENABLE_DRIVER_CHANGE_IFADDR /* We are now doing this in supplicant */
				wl_cfg80211_change_ifaddr((u8 *)p2p_ie,
					&wl->p2p_dev_addr, P2P_SEID_DEV_INFO);
			#endif
			}
			if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)(buf + ie_offset), ie_len))
				!= NULL) {
				/* Order of Vendor IE is 1) WPS IE +
				 * 2) P2P IE created by supplicant
				 *  So, it is ok to find start address of WPS IE
				 *  to save IEs to firmware
				 */
				wpsie_len = wps_ie->length + sizeof(wps_ie->length) +
					sizeof(wps_ie->tag);
				wl_cfgp2p_set_management_ie(wl, dev, bssidx,
					VNDR_IE_PRBRSP_FLAG,
					(u8 *)wps_ie, wpsie_len + p2pie_len);
			}
		}
		cfg80211_mgmt_tx_status(dev, *cookie, buf, len, true, GFP_KERNEL);
		goto exit;
	} else {
	    /* Abort the dwell time of any previous off-channel action frame that may
	     * be still in effect.  Sending off-channel action frames relies on the
	     * driver's scan engine.  If a previous off-channel action frame tx is
	     * still in progress (including the dwell time), then this new action
	     * frame will not be sent out.
	     */
		wl_cfg80211_scan_abort(wl, dev);
	}
	af_params = (wl_af_params_t *) kzalloc(WL_WIFI_AF_PARAMS_SIZE, GFP_KERNEL);

	if (af_params == NULL)
	{
		WL_ERR(("unable to allocate frame\n"));
		return -ENOMEM;
	}

	action_frame = &af_params->action_frame;

	/* Add the packet Id */
	action_frame->packetId = (u32) action_frame;
	WL_DBG(("action frame %d\n", action_frame->packetId));
	/* Add BSSID */
	memcpy(&action_frame->da, &mgmt->da[0], ETHER_ADDR_LEN);
	memcpy(&af_params->BSSID, &mgmt->bssid[0], ETHER_ADDR_LEN);

	/* Add the length exepted for 802.11 header  */
	action_frame->len = len - DOT11_MGMT_HDR_LEN;
	WL_DBG(("action_frame->len: %d\n", action_frame->len));

	/* Add the channel */
	af_params->channel =
		ieee80211_frequency_to_channel(channel->center_freq);

	/* Add the dwell time
	 * Dwell time to stay off-channel to wait for a response action frame
	 * after transmitting an GO Negotiation action frame
	 */
	af_params->dwell_time = WL_DWELL_TIME;

	memcpy(action_frame->data, &buf[DOT11_MGMT_HDR_LEN], action_frame->len);

	act_frm = (wifi_p2p_pub_act_frame_t *) (action_frame->data);
	WL_DBG(("action_frame->len: %d chan %d category %d subtype %d\n",
		action_frame->len, af_params->channel,
		act_frm->category, act_frm->subtype));
	if (wl->p2p->vif_created) {
		/*
		 * To make sure to send successfully action frame, we have to turn off mpc
		 */
		if ((act_frm->subtype == P2P_PAF_GON_REQ)||
		  (act_frm->subtype == P2P_PAF_GON_RSP)) {
			wldev_iovar_setint(dev, "mpc", 0);
		} else if (act_frm->subtype == P2P_PAF_GON_CONF) {
			wldev_iovar_setint(dev, "mpc", 1);
		} else if (act_frm->subtype == P2P_PAF_DEVDIS_REQ) {
			af_params->dwell_time = WL_LONG_DWELL_TIME;
		}
	}

	ack = (wl_cfgp2p_tx_action_frame(wl, dev, af_params, bssidx)) ? false : true;
	cfg80211_mgmt_tx_status(dev, *cookie, buf, len, ack, GFP_KERNEL);

	kfree(af_params);
exit:
	return err;
}


static void
wl_cfg80211_mgmt_frame_register(struct wiphy *wiphy, struct net_device *dev,
	u16 frame_type, bool reg)
{

	WL_DBG(("%s: frame_type: %x, reg: %d\n", __func__, frame_type, reg));

	if (frame_type != (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_REQ))
		return;

	return;
}


static s32
wl_cfg80211_change_bss(struct wiphy *wiphy,
	struct net_device *dev,
	struct bss_parameters *params)
{
	if (params->use_cts_prot >= 0) {
	}

	if (params->use_short_preamble >= 0) {
	}

	if (params->use_short_slot_time >= 0) {
	}

	if (params->basic_rates) {
	}

	if (params->ap_isolate >= 0) {
	}

	if (params->ht_opmode >= 0) {
	}

	return 0;
}

static s32
wl_cfg80211_set_channel(struct wiphy *wiphy, struct net_device *dev,
	struct ieee80211_channel *chan,
	enum nl80211_channel_type channel_type)
{
	s32 channel;
	s32 err = BCME_OK;

	channel = ieee80211_frequency_to_channel(chan->center_freq);
	WL_DBG(("netdev_ifidx(%d), chan_type(%d) target channel(%d) \n",
		dev->ifindex, channel_type, channel));
	err = wldev_ioctl(dev, WLC_SET_CHANNEL, &channel, sizeof(channel), true);
	if (err < 0) {
		WL_ERR(("WLC_SET_CHANNEL error %d chip may not be supporting this channel\n", err));
	}
	return err;
}

static s32
wl_validate_wpa2ie(struct net_device *dev, bcm_tlv_t *wpa2ie, s32 bssidx)
{
	s32 len = 0;
	s32 err = BCME_OK;
	u16 auth = 0; /* d11 open authentication */
	u16 count;
	u32 wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	u8* tmp;
	wpa_suite_mcast_t *mcast;
	wpa_suite_ucast_t *ucast;
	wpa_suite_auth_key_mgmt_t *mgmt;
	if (wpa2ie == NULL)
		goto exit;

	WL_DBG(("Enter \n"));
	len =  wpa2ie->len;
	/* check the mcast cipher */
	mcast = (wpa_suite_mcast_t *)&wpa2ie->data[WPA2_VERSION_LEN];
	tmp = mcast->oui;
	switch (tmp[DOT11_OUI_LEN]) {
		case WPA_CIPHER_NONE:
			gval = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			gval = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			gval = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			gval = AES_ENABLED;
			break;
		default:
			WL_ERR(("No Security Info\n"));
			break;
	}
	len -= WPA_SUITE_LEN;
	/* check the unicast cipher */
	ucast = (wpa_suite_ucast_t *)&mcast[1];
	count = ltoh16_ua(&ucast->count);
	tmp = ucast->list[0].oui;
	switch (tmp[DOT11_OUI_LEN]) {
		case WPA_CIPHER_NONE:
			pval = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			pval = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			pval = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			pval = AES_ENABLED;
			break;
		default:
			WL_ERR(("No Security Info\n"));
	}
	/* FOR WPS , set SEC_OW_ENABLED */
	wsec = (pval | gval | SES_OW_ENABLED);
	/* check the AKM */
	mgmt = (wpa_suite_auth_key_mgmt_t *)&ucast->list[1];
	count = ltoh16_ua(&mgmt->count);
	tmp = (u8 *)&mgmt->list[0];
	switch (tmp[DOT11_OUI_LEN]) {
		case RSN_AKM_NONE:
			wpa_auth = WPA_AUTH_NONE;
			break;
		case RSN_AKM_UNSPECIFIED:
			wpa_auth = WPA2_AUTH_UNSPECIFIED;
			break;
		case RSN_AKM_PSK:
			wpa_auth = WPA2_AUTH_PSK;
			break;
		default:
			WL_ERR(("No Key Mgmt Info\n"));
	}
	/* set auth */
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	/* set wsec */
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	/* set upper-layer auth */
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}
exit:
	return 0;
}

static s32
wl_validate_wpaie(struct net_device *dev, wpa_ie_fixed_t *wpaie, s32 bssidx)
{
	wpa_suite_mcast_t *mcast;
	wpa_suite_ucast_t *ucast;
	wpa_suite_auth_key_mgmt_t *mgmt;
	u16 auth = 0; /* d11 open authentication */
	u16 count;
	s32 err = BCME_OK;
	s32 len = 0;
	u32 i;
	u32 wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	u32 tmp = 0;

	if (wpaie == NULL)
		goto exit;
	WL_DBG(("Enter \n"));
	len = wpaie->length;    /* value length */
	len -= WPA_IE_TAG_FIXED_LEN;
	/* check for multicast cipher suite */
	if (len < WPA_SUITE_LEN) {
		WL_INFO(("no multicast cipher suite\n"));
		goto exit;
	}

	/* pick up multicast cipher */
	mcast = (wpa_suite_mcast_t *)&wpaie[1];
	len -= WPA_SUITE_LEN;
	if (!bcmp(mcast->oui, WPA_OUI, WPA_OUI_LEN)) {
		if (IS_WPA_CIPHER(mcast->type)) {
			tmp = 0;
			switch (mcast->type) {
				case WPA_CIPHER_NONE:
					tmp = 0;
					break;
				case WPA_CIPHER_WEP_40:
				case WPA_CIPHER_WEP_104:
					tmp = WEP_ENABLED;
					break;
				case WPA_CIPHER_TKIP:
					tmp = TKIP_ENABLED;
					break;
				case WPA_CIPHER_AES_CCM:
					tmp = AES_ENABLED;
					break;
				default:
					WL_ERR(("No Security Info\n"));
			}
			gval |= tmp;
		}
	}
	/* Check for unicast suite(s) */
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFO(("no unicast suite\n"));
		goto exit;
	}
	/* walk thru unicast cipher list and pick up what we recognize */
	ucast = (wpa_suite_ucast_t *)&mcast[1];
	count = ltoh16_ua(&ucast->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(ucast->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_CIPHER(ucast->list[i].type)) {
				tmp = 0;
				switch (ucast->list[i].type) {
					case WPA_CIPHER_NONE:
						tmp = 0;
						break;
					case WPA_CIPHER_WEP_40:
					case WPA_CIPHER_WEP_104:
						tmp = WEP_ENABLED;
						break;
					case WPA_CIPHER_TKIP:
						tmp = TKIP_ENABLED;
						break;
					case WPA_CIPHER_AES_CCM:
						tmp = AES_ENABLED;
						break;
					default:
						WL_ERR(("No Security Info\n"));
				}
				pval |= tmp;
			}
		}
	}
	len -= (count - i) * WPA_SUITE_LEN;
	/* Check for auth key management suite(s) */
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFO((" no auth key mgmt suite\n"));
		goto exit;
	}
	/* walk thru auth management suite list and pick up what we recognize */
	mgmt = (wpa_suite_auth_key_mgmt_t *)&ucast->list[count];
	count = ltoh16_ua(&mgmt->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(mgmt->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_AKM(mgmt->list[i].type)) {
				tmp = 0;
				switch (mgmt->list[i].type) {
					case RSN_AKM_NONE:
						tmp = WPA_AUTH_NONE;
						break;
					case RSN_AKM_UNSPECIFIED:
						tmp = WPA_AUTH_UNSPECIFIED;
						break;
					case RSN_AKM_PSK:
						tmp = WPA_AUTH_PSK;
						break;
					default:
						WL_ERR(("No Key Mgmt Info\n"));
				}
				wpa_auth |= tmp;
			}
		}

	}
	/* FOR WPS , set SEC_OW_ENABLED */
	wsec = (pval | gval | SES_OW_ENABLED);
	/* set auth */
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	/* set wsec */
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	/* set upper-layer auth */
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}
exit:
	return 0;
}

static s32
wl_cfg80211_add_set_beacon(struct wiphy *wiphy, struct net_device *dev,
	struct beacon_parameters *info)
{
	s32 err = BCME_OK;
	bcm_tlv_t *ssid_ie;
	wlc_ssid_t ssid;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct wl_join_params join_params;
	wpa_ie_fixed_t *wps_ie;
	wpa_ie_fixed_t *wpa_ie;
	bcm_tlv_t *wpa2_ie;
	wifi_p2p_ie_t *p2p_ie;
	bool is_bssup = false;
	bool update_bss = false;
	bool pbc = false;
	u16 wpsie_len = 0;
	u16 p2pie_len = 0;
	u8 beacon_ie[IE_MAX_LEN];
	s32 ie_offset = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);
	s32 infra = 1;
	s32 join_params_size = 0;
	s32 ap = 0;
	WL_DBG(("interval (%d) dtim_period (%d) head_len (%d) tail_len (%d)\n",
		info->interval, info->dtim_period, info->head_len, info->tail_len));
	if (wl->p2p_supported && p2p_on(wl) &&
		(bssidx == wl_to_p2p_bss_bssidx(wl,
		P2PAPI_BSSCFG_CONNECTION))) {
		memset(beacon_ie, 0, sizeof(beacon_ie));
		/* We don't need to set beacon for P2P_GO,
		 * but need to parse ssid from beacon_parameters
		 * because there is no way to set ssid
		 */
		ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
		/* find the SSID */
		if ((ssid_ie = bcm_parse_tlvs((u8 *)&info->head[ie_offset],
			info->head_len - ie_offset,
			DOT11_MNG_SSID_ID)) != NULL) {
			memcpy(wl->p2p->ssid.SSID, ssid_ie->data, ssid_ie->len);
			wl->p2p->ssid.SSID_len = ssid_ie->len;
			WL_DBG(("SSID (%s) in Head \n", ssid_ie->data));

		} else {
			WL_ERR(("No SSID in beacon \n"));
		}

		/* find the WPSIE */
		if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)info->tail, info->tail_len)) != NULL) {
			wpsie_len = wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
			/*
			 * Should be compared with saved ie before saving it
			 */
			wl_validate_wps_ie((char *) wps_ie, &pbc);
			memcpy(beacon_ie, wps_ie, wpsie_len);
		} else {
			WL_ERR(("No WPSIE in beacon \n"));
		}


		/* find the P2PIE */
		if ((p2p_ie = wl_cfgp2p_find_p2pie((u8 *)info->tail, info->tail_len)) != NULL) {
			/* Total length of P2P Information Element */
			p2pie_len = p2p_ie->len + sizeof(p2p_ie->len) + sizeof(p2p_ie->id);
		#ifdef ENABLE_DRIVER_CHANGE_IFADDR /* We are now doing this in supplicant */
			/* Have to change device address in dev_id attribute because Supplicant
			 * use primary eth0 address
			 */
			wl_cfg80211_change_ifaddr((u8 *)p2p_ie, &wl->p2p_dev_addr, P2P_SEID_DEV_ID);
		#endif
			memcpy(&beacon_ie[wpsie_len], p2p_ie, p2pie_len);

		} else {
			WL_ERR(("No P2PIE in beacon \n"));
		}
		/* add WLC_E_PROBREQ_MSG event to respose probe_request from STA */
		wl_dongle_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
		wl_cfgp2p_set_management_ie(wl, dev, bssidx, VNDR_IE_BEACON_FLAG,
			beacon_ie, wpsie_len + p2pie_len);

		/* find the RSN_IE */
		if ((wpa2_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
			DOT11_MNG_RSN_ID)) != NULL) {
			WL_DBG((" WPA2 IE is found\n"));
		}
		is_bssup = wl_cfgp2p_bss_isup(dev, bssidx);

		if (!is_bssup && (wpa2_ie != NULL)) {
			wldev_iovar_setint(dev, "mpc", 0);
			if ((err = wl_validate_wpa2ie(dev, wpa2_ie, bssidx)) < 0) {
				WL_ERR(("WPA2 IE parsing error"));
				goto exit;
			}
			err = wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
			if (err < 0) {
				WL_ERR(("SET INFRA error %d\n", err));
				goto exit;
			}
			err = wldev_iovar_setbuf_bsscfg(dev, "ssid", &wl->p2p->ssid,
				sizeof(wl->p2p->ssid), ioctlbuf, sizeof(ioctlbuf), bssidx);
			if (err < 0) {
				WL_ERR(("GO SSID setting error %d\n", err));
				goto exit;
			}
			if ((err = wl_cfgp2p_bss(dev, bssidx, 1)) < 0) {
				WL_ERR(("GO Bring up error %d\n", err));
				goto exit;
			}
		}
	} else if (wl_get_drv_status(wl, AP_CREATING)) {
		ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
		ap = 1;
		/* find the SSID */
		if ((ssid_ie = bcm_parse_tlvs((u8 *)&info->head[ie_offset],
			info->head_len - ie_offset,
			DOT11_MNG_SSID_ID)) != NULL) {
			memset(&ssid, 0, sizeof(wlc_ssid_t));
			memcpy(ssid.SSID, ssid_ie->data, ssid_ie->len);
			WL_DBG(("SSID is (%s) in Head \n", ssid.SSID));
			ssid.SSID_len = ssid_ie->len;
			wldev_iovar_setint(dev, "mpc", 0);
			wldev_ioctl(dev, WLC_DOWN, &ap, sizeof(s32), true);
			wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
			if ((err = wldev_ioctl(dev, WLC_SET_AP, &ap, sizeof(s32), true)) < 0) {
				WL_ERR(("setting AP mode failed %d \n", err));
				return err;
			}
			/* find the RSN_IE */
			if ((wpa2_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
				DOT11_MNG_RSN_ID)) != NULL) {
				WL_DBG((" WPA2 IE is found\n"));
			}
			/* find the WPA_IE */
			if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)info->tail,
			info->tail_len)) != NULL) {
				WL_DBG((" WPA IE is found\n"));
			}
			if ((wpa_ie != NULL || wpa2_ie != NULL)) {
				if (wl_validate_wpa2ie(dev, wpa2_ie, bssidx)  < 0 ||
					wl_validate_wpaie(dev, wpa_ie, bssidx) < 0) {
					wl->ap_info->security_mode = false;
					return BCME_ERROR;
				}
				wl->ap_info->security_mode = true;
				if (wl->ap_info->rsn_ie) {
					kfree(wl->ap_info->rsn_ie);
					wl->ap_info->rsn_ie = NULL;
				}
				if (wl->ap_info->wpa_ie) {
					kfree(wl->ap_info->wpa_ie);
					wl->ap_info->wpa_ie = NULL;
				}
				if (wl->ap_info->wps_ie) {
					kfree(wl->ap_info->wps_ie);
					wl->ap_info->wps_ie = NULL;
				}
				if (wpa_ie != NULL) {
					/* WPAIE */
					wl->ap_info->rsn_ie = NULL;
					wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
				} else {
					/* RSNIE */
					wl->ap_info->wpa_ie = NULL;
					wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
				}
			} else
				wl->ap_info->security_mode = false;
			/* find the WPSIE */
			if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)info->tail,
				info->tail_len)) != NULL) {
				wpsie_len = wps_ie->length +WPA_RSN_IE_TAG_FIXED_LEN;
				/*
				* Should be compared with saved ie before saving it
				*/
				wl_validate_wps_ie((char *) wps_ie, &pbc);
				memcpy(beacon_ie, wps_ie, wpsie_len);
				wl_cfgp2p_set_management_ie(wl, dev, bssidx, VNDR_IE_BEACON_FLAG,
				beacon_ie, wpsie_len);
				wl->ap_info->wps_ie = kmemdup(wps_ie, 	wpsie_len, GFP_KERNEL);
				/* add WLC_E_PROBREQ_MSG event to respose probe_request from STA */
				wl_dongle_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
			} else {
				WL_DBG(("No WPSIE in beacon \n"));
			}
			if (info->interval) {
				if ((err = wldev_ioctl(dev, WLC_SET_BCNPRD,
					&info->interval, sizeof(s32), true)) < 0) {
					WL_ERR(("Beacon Interval Set Error, %d\n", err));
					return err;
				}
			}
			if (info->dtim_period) {
				if ((err = wldev_ioctl(dev, WLC_SET_DTIMPRD,
					&info->dtim_period, sizeof(s32), true)) < 0) {
					WL_ERR(("DTIM Interval Set Error, %d\n", err));
					return err;
				}
			}
			err = wldev_ioctl(dev, WLC_UP, &ap, sizeof(s32), true);
			if (unlikely(err)) {
				WL_ERR(("WLC_UP error (%d)\n", err));
				return err;
			}
			memset(&join_params, 0, sizeof(join_params));
			/* join parameters starts with ssid */
			join_params_size = sizeof(join_params.ssid);
			memcpy(join_params.ssid.SSID, ssid.SSID, ssid.SSID_len);
			join_params.ssid.SSID_len = htod32(ssid.SSID_len);
			/* create softap */
			if ((err = wldev_ioctl(dev, WLC_SET_SSID, &join_params,
				join_params_size, true)) == 0) {
				wl_clr_drv_status(wl, AP_CREATING);
				wl_set_drv_status(wl, AP_CREATED);
			}
		}
	} else if (wl_get_drv_status(wl, AP_CREATED)) {
		ap = 1;
		/* find the WPSIE */
		if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)info->tail, info->tail_len)) != NULL) {
			wpsie_len = wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
			/*
			 * Should be compared with saved ie before saving it
			 */
			wl_validate_wps_ie((char *) wps_ie, &pbc);
			memcpy(beacon_ie, wps_ie, wpsie_len);
			wl_cfgp2p_set_management_ie(wl, dev, bssidx, VNDR_IE_BEACON_FLAG,
			beacon_ie, wpsie_len);
			if (wl->ap_info->wps_ie &&
				memcmp(wl->ap_info->wps_ie, wps_ie, wpsie_len)) {
				WL_DBG((" WPS IE is changed\n"));
				kfree(wl->ap_info->wps_ie);
				wl->ap_info->wps_ie = kmemdup(wps_ie, 	wpsie_len, GFP_KERNEL);
				/* add WLC_E_PROBREQ_MSG event to respose probe_request from STA */
				wl_dongle_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
			} else if (wl->ap_info->wps_ie == NULL) {
				WL_DBG((" WPS IE is added\n"));
				wl->ap_info->wps_ie = kmemdup(wps_ie, 	wpsie_len, GFP_KERNEL);
				/* add WLC_E_PROBREQ_MSG event to respose probe_request from STA */
				wl_dongle_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
			}
			/* find the RSN_IE */
			if ((wpa2_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
				DOT11_MNG_RSN_ID)) != NULL) {
				WL_DBG((" WPA2 IE is found\n"));
			}
			/* find the WPA_IE */
			if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)info->tail,
				info->tail_len)) != NULL) {
				WL_DBG((" WPA IE is found\n"));
			}
			if ((wpa_ie != NULL || wpa2_ie != NULL)) {
				if (!wl->ap_info->security_mode) {
					/* change from open mode to security mode */
					update_bss = true;
					if (wpa_ie != NULL) {
						wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
					} else {
						wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
					}
				} else if (wl->ap_info->wpa_ie) {
					/* change from WPA mode to WPA2 mode */
					if (wpa2_ie != NULL) {
						update_bss = true;
						kfree(wl->ap_info->wpa_ie);
						wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
						wl->ap_info->wpa_ie = NULL;
					}
					else if (memcmp(wl->ap_info->wpa_ie,
						wpa_ie, wpa_ie->length +
						WPA_RSN_IE_TAG_FIXED_LEN)) {
						kfree(wl->ap_info->wpa_ie);
						update_bss = true;
						wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
						wl->ap_info->rsn_ie = NULL;
					}
				} else {
					/* change from WPA2 mode to WPA mode */
					if (wpa_ie != NULL) {
						update_bss = true;
						kfree(wl->ap_info->rsn_ie);
						wl->ap_info->rsn_ie = NULL;
						wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
					} else if (memcmp(wl->ap_info->rsn_ie,
						wpa2_ie, wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN)) {
						update_bss = true;
						kfree(wl->ap_info->rsn_ie);
						wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
						wl->ap_info->wpa_ie = NULL;
					}
				}
				if (update_bss) {
					wl->ap_info->security_mode = true;
					wl_cfgp2p_bss(dev, bssidx, 0);
					if (wl_validate_wpa2ie(dev, wpa2_ie, bssidx)  < 0 ||
						wl_validate_wpaie(dev, wpa_ie, bssidx) < 0) {
						return BCME_ERROR;
					}
					wl_cfgp2p_bss(dev, bssidx, 1);
				}
			}
		} else {
			WL_ERR(("No WPSIE in beacon \n"));
		}
	}
exit:
	if (err)
		wldev_iovar_setint(dev, "mpc", 1);
	return err;
}

static struct cfg80211_ops wl_cfg80211_ops = {
	.add_virtual_intf = wl_cfg80211_add_virtual_iface,
	.del_virtual_intf = wl_cfg80211_del_virtual_iface,
	.change_virtual_intf = wl_cfg80211_change_virtual_iface,
	.scan = wl_cfg80211_scan,
	.set_wiphy_params = wl_cfg80211_set_wiphy_params,
	.join_ibss = wl_cfg80211_join_ibss,
	.leave_ibss = wl_cfg80211_leave_ibss,
	.get_station = wl_cfg80211_get_station,
	.set_tx_power = wl_cfg80211_set_tx_power,
	.get_tx_power = wl_cfg80211_get_tx_power,
	.add_key = wl_cfg80211_add_key,
	.del_key = wl_cfg80211_del_key,
	.get_key = wl_cfg80211_get_key,
	.set_default_key = wl_cfg80211_config_default_key,
	.set_default_mgmt_key = wl_cfg80211_config_default_mgmt_key,
	.set_power_mgmt = wl_cfg80211_set_power_mgmt,
	.connect = wl_cfg80211_connect,
	.disconnect = wl_cfg80211_disconnect,
	.suspend = wl_cfg80211_suspend,
	.resume = wl_cfg80211_resume,
	.set_pmksa = wl_cfg80211_set_pmksa,
	.del_pmksa = wl_cfg80211_del_pmksa,
	.flush_pmksa = wl_cfg80211_flush_pmksa,
	.remain_on_channel = wl_cfg80211_remain_on_channel,
	.cancel_remain_on_channel = wl_cfg80211_cancel_remain_on_channel,
	.mgmt_tx = wl_cfg80211_mgmt_tx,
	.mgmt_frame_register = wl_cfg80211_mgmt_frame_register,
	.change_bss = wl_cfg80211_change_bss,
	.set_channel = wl_cfg80211_set_channel,
	.set_beacon = wl_cfg80211_add_set_beacon,
	.add_beacon = wl_cfg80211_add_set_beacon,
};

static s32 wl_mode_to_nl80211_iftype(s32 mode)
{
	s32 err = 0;

	switch (mode) {
	case WL_MODE_BSS:
		return NL80211_IFTYPE_STATION;
	case WL_MODE_IBSS:
		return NL80211_IFTYPE_ADHOC;
	case WL_MODE_AP:
		return NL80211_IFTYPE_AP;
	default:
		return NL80211_IFTYPE_UNSPECIFIED;
	}

	return err;
}

static struct wireless_dev *wl_alloc_wdev(struct device *sdiofunc_dev)
{
	struct wireless_dev *wdev;
	s32 err = 0;
	wdev = kzalloc(sizeof(*wdev), GFP_KERNEL);
	if (unlikely(!wdev)) {
		WL_ERR(("Could not allocate wireless device\n"));
		return ERR_PTR(-ENOMEM);
	}
	wdev->wiphy =
	    wiphy_new(&wl_cfg80211_ops, sizeof(struct wl_priv));
	if (unlikely(!wdev->wiphy)) {
		WL_ERR(("Couldn not allocate wiphy device\n"));
		err = -ENOMEM;
		goto wiphy_new_out;
	}
	set_wiphy_dev(wdev->wiphy, sdiofunc_dev);
	wdev->wiphy->max_scan_ie_len = WL_SCAN_IE_LEN_MAX;
	/* Report  how many SSIDs Driver can support per Scan request */
	wdev->wiphy->max_scan_ssids = WL_SCAN_PARAMS_SSID_MAX;
	wdev->wiphy->max_num_pmkids = WL_NUM_PMKIDS_MAX;
	wdev->wiphy->interface_modes =
	    BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_ADHOC)
	    | BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_MONITOR);

	wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &__wl_band_2ghz;
	wdev->wiphy->bands[IEEE80211_BAND_5GHZ] = &__wl_band_5ghz_a;
	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wdev->wiphy->cipher_suites = __wl_cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(__wl_cipher_suites);
	wdev->wiphy->max_remain_on_channel_duration = 5000;
	wdev->wiphy->mgmt_stypes = wl_cfg80211_default_mgmt_stypes;
#ifndef WL_POWERSAVE_DISABLED
	wdev->wiphy->flags |= WIPHY_FLAG_PS_ON_BY_DEFAULT;
#else
	wdev->wiphy->flags &= ~WIPHY_FLAG_PS_ON_BY_DEFAULT;
#endif				/* !WL_POWERSAVE_DISABLED */
	wdev->wiphy->flags |= WIPHY_FLAG_NETNS_OK |
		WIPHY_FLAG_4ADDR_AP |
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 39)
		WIPHY_FLAG_SUPPORTS_SEPARATE_DEFAULT_KEYS |
#endif
		WIPHY_FLAG_4ADDR_STATION;

	WL_DBG(("Registering custom regulatory)\n"));
	wdev->wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;
	wiphy_apply_custom_regulatory(wdev->wiphy, &brcm_regdom);
	/* Now we can register wiphy with cfg80211 module */
	err = wiphy_register(wdev->wiphy);
	if (unlikely(err < 0)) {
		WL_ERR(("Couldn not register wiphy device (%d)\n", err));
		goto wiphy_register_out;
	}
	return wdev;

wiphy_register_out:
	wiphy_free(wdev->wiphy);

wiphy_new_out:
	kfree(wdev);

	return ERR_PTR(err);
}

static void wl_free_wdev(struct wl_priv *wl)
{
	int i;
	struct wireless_dev *wdev = wl->wdev;

	if (unlikely(!wdev)) {
		WL_ERR(("wdev is invalid\n"));
		return;
	}

	for (i = 0; i < VWDEV_CNT; i++) {
		if ((wl->vwdev[i] != NULL)) {
			kfree(wl->vwdev[i]);
			wl->vwdev[i] = NULL;
		}
	}
	wiphy_unregister(wdev->wiphy);
	wdev->wiphy->dev.parent = NULL;
	wiphy_free(wdev->wiphy);
	kfree(wdev);
}

static s32 wl_inform_bss(struct wl_priv *wl)
{
	struct wl_scan_results *bss_list;
	struct wl_bss_info *bi = NULL;	/* must be initialized */
	s32 err = 0;
	s32 i;

	bss_list = wl->bss_list;
	WL_DBG(("scanned AP count (%d)\n", bss_list->count));
	bi = next_bss(bss_list, bi);
	for_each_bss(bss_list, bi, i) {
		err = wl_inform_single_bss(wl, bi);
		if (unlikely(err))
			break;
	}
	return err;
}

static s32 wl_inform_single_bss(struct wl_priv *wl, struct wl_bss_info *bi)
{
	struct wiphy *wiphy = wiphy_from_scan(wl);
	struct ieee80211_mgmt *mgmt;
	struct ieee80211_channel *channel;
	struct ieee80211_supported_band *band;
	struct wl_cfg80211_bss_info *notif_bss_info;
	struct wl_scan_req *sr = wl_to_sr(wl);
	struct beacon_proberesp *beacon_proberesp;
	s32 mgmt_type;
	s32 signal;
	u32 freq;
	s32 err = 0;

	if (unlikely(dtoh32(bi->length) > WL_BSS_INFO_MAX)) {
		WL_DBG(("Beacon is larger than buffer. Discarding\n"));
		return err;
	}
	notif_bss_info = kzalloc(sizeof(*notif_bss_info) + sizeof(*mgmt)
		- sizeof(u8) + WL_BSS_INFO_MAX, GFP_KERNEL);
	if (unlikely(!notif_bss_info)) {
		WL_ERR(("notif_bss_info alloc failed\n"));
		return -ENOMEM;
	}
	mgmt = (struct ieee80211_mgmt *)notif_bss_info->frame_buf;
	notif_bss_info->channel =
		bi->ctl_ch ? bi->ctl_ch : CHSPEC_CHANNEL(bi->chanspec);

	if (notif_bss_info->channel <= CH_MAX_2G_CHANNEL)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];
	notif_bss_info->rssi = dtoh16(bi->RSSI);
	memcpy(mgmt->bssid, &bi->BSSID, ETHER_ADDR_LEN);
	mgmt_type = wl->active_scan ?
		IEEE80211_STYPE_PROBE_RESP : IEEE80211_STYPE_BEACON;
	if (!memcmp(bi->SSID, sr->ssid.SSID, bi->SSID_len)) {
	    mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | mgmt_type);
	}
	beacon_proberesp = wl->active_scan ?
		(struct beacon_proberesp *)&mgmt->u.probe_resp :
		(struct beacon_proberesp *)&mgmt->u.beacon;
	beacon_proberesp->timestamp = 0;
	beacon_proberesp->beacon_int = cpu_to_le16(bi->beacon_period);
	beacon_proberesp->capab_info = cpu_to_le16(bi->capability);
	wl_rst_ie(wl);

	wl_mrg_ie(wl, ((u8 *) bi) + bi->ie_offset, bi->ie_length);
	wl_cp_ie(wl, beacon_proberesp->variable, WL_BSS_INFO_MAX -
		offsetof(struct wl_cfg80211_bss_info, frame_buf));
	notif_bss_info->frame_len = offsetof(struct ieee80211_mgmt,
		u.beacon.variable) + wl_get_ielen(wl);
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
	freq = ieee80211_channel_to_frequency(notif_bss_info->channel);
#else
	freq = ieee80211_channel_to_frequency(notif_bss_info->channel, band->band);
#endif
	channel = ieee80211_get_channel(wiphy, freq);

	WL_DBG(("SSID : \"%s\", rssi %d, channel %d, capability : 0x04%x, bssid %pM"
			"mgmt_type %d frame_len %d\n", bi->SSID,
			notif_bss_info->rssi, notif_bss_info->channel,
			mgmt->u.beacon.capab_info, &bi->BSSID, mgmt_type,
			notif_bss_info->frame_len));

	signal = notif_bss_info->rssi * 100;

	if (unlikely(!cfg80211_inform_bss_frame(wiphy, channel, mgmt,
		le16_to_cpu(notif_bss_info->frame_len),
		signal, GFP_KERNEL))) {
		WL_ERR(("cfg80211_inform_bss_frame error\n"));
		kfree(notif_bss_info);
		return -EINVAL;
	}
	kfree(notif_bss_info);

	return err;
}

static bool wl_is_linkup(struct wl_priv *wl, const wl_event_msg_t *e, struct net_device *ndev)
{
	u32 event = ntoh32(e->event_type);
	u32 status =  ntoh32(e->status);
	u16 flags = ntoh16(e->flags);

	WL_DBG(("event %d, status %d\n", event, status));
	if (event == WLC_E_SET_SSID) {
		if (status == WLC_E_STATUS_SUCCESS) {
			if (!wl_is_ibssmode(wl, ndev))
				return true;
		}
	} else if (event == WLC_E_LINK) {
		if (flags & WLC_EVENT_MSG_LINK)
			return true;
	}

	WL_DBG(("wl_is_linkup false\n"));
	return false;
}

static bool wl_is_linkdown(struct wl_priv *wl, const wl_event_msg_t *e)
{
	u32 event = ntoh32(e->event_type);
	u16 flags = ntoh16(e->flags);

	if (event == WLC_E_DEAUTH_IND ||
	event == WLC_E_DISASSOC_IND ||
	event == WLC_E_DISASSOC ||
	event == WLC_E_DEAUTH) {
		return true;
	} else if (event == WLC_E_LINK) {
		if (!(flags & WLC_EVENT_MSG_LINK))
			return true;
	}

	return false;
}

static bool wl_is_nonetwork(struct wl_priv *wl, const wl_event_msg_t *e)
{
	u32 event = ntoh32(e->event_type);
	u32 status = ntoh32(e->status);

	if (event == WLC_E_LINK && status == WLC_E_STATUS_NO_NETWORKS)
		return true;
	if (event == WLC_E_SET_SSID && status != WLC_E_STATUS_SUCCESS)
		return true;

	return false;
}

static s32
wl_notify_connect_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	bool act;
	bool isfree = false;
	s32 err = 0;
	s32 freq;
	s32 channel;
	u8 body[200];
	u32 event = ntoh32(e->event_type);
	u32 reason = ntoh32(e->reason);
	u32 len = ntoh32(e->datalen);
	u16 fc = 0;
	u8 *mgmt_frame;
	u8 bsscfgidx = e->bsscfgidx;
	struct ieee80211_supported_band *band;
	struct ether_addr da;
	struct ether_addr bssid;
	struct wiphy *wiphy = wl_to_wiphy(wl);
	channel_info_t ci;

	memset(body, 0, sizeof(body));
	memset(&bssid, 0, ETHER_ADDR_LEN);
	WL_DBG(("Enter \n"));

	if (get_mode_by_netdev(wl, ndev) == WL_MODE_AP) {
		memcpy(body, data, len);
		wldev_iovar_getbuf_bsscfg(ndev, "cur_etheraddr",
		NULL, 0, ioctlbuf, sizeof(ioctlbuf), bsscfgidx);
		memcpy(da.octet, ioctlbuf, ETHER_ADDR_LEN);
		err = wldev_ioctl(ndev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN, false);
		switch (event) {
			case WLC_E_ASSOC_IND:
				fc = FC_ASSOC_REQ;
				break;
			case WLC_E_REASSOC_IND:
				fc = FC_REASSOC_REQ;
				break;
			case WLC_E_DISASSOC_IND:
				fc = FC_DISASSOC;
				break;
			case WLC_E_DEAUTH_IND:
				fc = FC_DISASSOC;
				break;
			case WLC_E_DEAUTH:
				fc = FC_DISASSOC;
				break;
			default:
				fc = 0;
				goto exit;
		}
		if ((err = wldev_ioctl(ndev, WLC_GET_CHANNEL, &ci, sizeof(ci), false)))
			return err;

		channel = dtoh32(ci.hw_channel);
		if (channel <= CH_MAX_2G_CHANNEL)
			band = wiphy->bands[IEEE80211_BAND_2GHZ];
		else
			band = wiphy->bands[IEEE80211_BAND_5GHZ];

#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
		freq = ieee80211_channel_to_frequency(channel);
#else
		freq = ieee80211_channel_to_frequency(channel, band->band);
#endif

		err = wl_frame_get_mgmt(fc, &da, &e->addr, &bssid,
		&mgmt_frame, &len, body);
		if (err < 0)
				goto exit;
		isfree = true;

		if (event == WLC_E_ASSOC_IND && reason == DOT11_SC_SUCCESS) {
			cfg80211_rx_mgmt(ndev, freq, mgmt_frame, len, GFP_ATOMIC);
		} else if (event == WLC_E_DISASSOC_IND) {
			cfg80211_rx_mgmt(ndev, freq, mgmt_frame, len, GFP_ATOMIC);
		} else if ((event == WLC_E_DEAUTH_IND) || (event == WLC_E_DEAUTH)) {
			cfg80211_rx_mgmt(ndev, freq, mgmt_frame, len, GFP_ATOMIC);
		}

	} else {
		WL_DBG(("wl_notify_connect_status : event %d status : %d \n",
		ntoh32(e->event_type), ntoh32(e->status)));
		if (wl_is_linkup(wl, e, ndev)) {
			wl_link_up(wl);
			act = true;
			wl_update_prof(wl, e, &act, WL_PROF_ACT);
			wl_update_prof(wl, NULL, (void *)(e->addr.octet), WL_PROF_BSSID);
			if (wl_is_ibssmode(wl, ndev)) {
				printk("cfg80211_ibss_joined\n");
				cfg80211_ibss_joined(ndev, (s8 *)&e->addr,
					GFP_KERNEL);
				WL_DBG(("joined in IBSS network\n"));
			} else {
				if (!wl_get_drv_status(wl, DISCONNECTING)) {
					printk("wl_bss_connect_done succeeded status=(0x%x)\n",
						(int)wl->status);
					wl_bss_connect_done(wl, ndev, e, data, true);
					WL_DBG(("joined in BSS network \"%s\"\n",
					((struct wlc_ssid *)
					 wl_read_prof(wl, WL_PROF_SSID))->SSID));
				}
			}

		} else if (wl_is_linkdown(wl, e)) {
			if (wl->scan_request) {
				del_timer_sync(&wl->scan_timeout);
				if (wl->escan_on) {
					wl_notify_escan_complete(wl, true);
				} else
					wl_iscan_aborted(wl);
			}
			if (wl_get_drv_status(wl, CONNECTED)) {
				scb_val_t scbval;
				u8 *curbssid = wl_read_prof(wl, WL_PROF_BSSID);
				printk("link down, call cfg80211_disconnected\n");
				wl_clr_drv_status(wl, CONNECTED);
				/* To make sure disconnect, explictly send dissassoc
				*  for BSSID 00:00:00:00:00:00 issue
				*/
				scbval.val = WLAN_REASON_DEAUTH_LEAVING;

				memcpy(&scbval.ea, curbssid, ETHER_ADDR_LEN);
				scbval.val = htod32(scbval.val);
				wldev_ioctl(ndev, WLC_DISASSOC, &scbval,
					sizeof(scb_val_t), true);
				cfg80211_disconnected(ndev, 0, NULL, 0, GFP_KERNEL);
				wl_link_down(wl);
				wl_init_prof(wl);
			} else if (wl_get_drv_status(wl, CONNECTING)) {
				printk("link down, during connecting\n");
				wl_bss_connect_done(wl, ndev, e, data, false);
			}
			wl_clr_drv_status(wl, DISCONNECTING);

		} else if (wl_is_nonetwork(wl, e)) {
			printk("connect failed event=%d e->status 0x%x\n",
				event, (int)ntoh32(e->status));
			/* Clean up any pending scan request */
			if (wl->scan_request) {
				del_timer_sync(&wl->scan_timeout);
				if (wl->escan_on) {
					wl_notify_escan_complete(wl, true);
				} else
					wl_iscan_aborted(wl);
			}
			if (wl_get_drv_status(wl, CONNECTING))
				wl_bss_connect_done(wl, ndev, e, data, false);
		} else {
			printk("%s nothing\n", __FUNCTION__);
		}
	}
exit:
	if (isfree)
		kfree(mgmt_frame);
	return err;
}

static s32
wl_notify_roaming_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	bool act;
	s32 err = 0;
	u32 event = be32_to_cpu(e->event_type);
	u32 status = be32_to_cpu(e->status);
	WL_DBG(("Enter \n"));
	if (event == WLC_E_ROAM && status == WLC_E_STATUS_SUCCESS) {
		if (wl_get_drv_status(wl, CONNECTED))
			wl_bss_roaming_done(wl, ndev, e, data);
		else
			wl_bss_connect_done(wl, ndev, e, data, true);
		act = true;
		wl_update_prof(wl, e, &act, WL_PROF_ACT);
		wl_update_prof(wl, NULL, (void *)(e->addr.octet), WL_PROF_BSSID);
	}
	return err;
}

static __used s32
wl_dev_bufvar_set(struct net_device *dev, s8 *name, s8 *buf, s32 len)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	u32 buflen;

	buflen = bcm_mkiovar(name, buf, len, wl->ioctl_buf, WL_IOCTL_LEN_MAX);
	BUG_ON(unlikely(!buflen));

	return wldev_ioctl(dev, WLC_SET_VAR, wl->ioctl_buf, buflen, true);
}

static s32
wl_dev_bufvar_get(struct net_device *dev, s8 *name, s8 *buf,
	s32 buf_len)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	u32 len;
	s32 err = 0;

	len = bcm_mkiovar(name, NULL, 0, wl->ioctl_buf, WL_IOCTL_LEN_MAX);
	BUG_ON(unlikely(!len));
	err = wldev_ioctl(dev, WLC_GET_VAR, (void *)wl->ioctl_buf,
		WL_IOCTL_LEN_MAX, false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	memcpy(buf, wl->ioctl_buf, buf_len);

	return err;
}

static s32 wl_get_assoc_ies(struct wl_priv *wl, struct net_device *ndev)
{
	wl_assoc_info_t assoc_info;
	struct wl_connect_info *conn_info = wl_to_conn(wl);
	s32 err = 0;

	WL_DBG(("Enter \n"));
	err = wl_dev_bufvar_get(ndev, "assoc_info", wl->extra_buf,
		WL_ASSOC_INFO_MAX);
	if (unlikely(err)) {
		WL_ERR(("could not get assoc info (%d)\n", err));
		return err;
	}
	memcpy(&assoc_info, wl->extra_buf, sizeof(wl_assoc_info_t));
	assoc_info.req_len = htod32(assoc_info.req_len);
	assoc_info.resp_len = htod32(assoc_info.resp_len);
	assoc_info.flags = htod32(assoc_info.flags);
	if (conn_info->req_ie_len) {
		conn_info->req_ie_len = 0;
		bzero(conn_info->req_ie, sizeof(conn_info->req_ie));
	}
	if (conn_info->resp_ie_len) {
		conn_info->resp_ie_len = 0;
		bzero(conn_info->resp_ie, sizeof(conn_info->resp_ie));
	}
	if (assoc_info.req_len) {
		err = wl_dev_bufvar_get(ndev, "assoc_req_ies", wl->extra_buf,
			WL_ASSOC_INFO_MAX);
		if (unlikely(err)) {
			WL_ERR(("could not get assoc req (%d)\n", err));
			return err;
		}
		conn_info->req_ie_len = assoc_info.req_len - sizeof(struct dot11_assoc_req);
		if (assoc_info.flags & WLC_ASSOC_REQ_IS_REASSOC) {
			conn_info->req_ie_len -= ETHER_ADDR_LEN;
		}
		if (conn_info->req_ie_len <= MAX_REQ_LINE)
			memcpy(conn_info->req_ie, wl->extra_buf, conn_info->req_ie_len);
		else {
			WL_ERR(("%s IE size %d above max %d size \n",
				__FUNCTION__, conn_info->req_ie_len, MAX_REQ_LINE));
			return err;
		}
	} else {
		conn_info->req_ie_len = 0;
	}
	if (assoc_info.resp_len) {
		err = wl_dev_bufvar_get(ndev, "assoc_resp_ies", wl->extra_buf,
			WL_ASSOC_INFO_MAX);
		if (unlikely(err)) {
			WL_ERR(("could not get assoc resp (%d)\n", err));
			return err;
		}
		conn_info->resp_ie_len = assoc_info.resp_len -sizeof(struct dot11_assoc_resp);
		if (conn_info->resp_ie_len <= MAX_REQ_LINE)
			memcpy(conn_info->resp_ie, wl->extra_buf, conn_info->resp_ie_len);
		else {
			WL_ERR(("%s IE size %d above max %d size \n",
				__FUNCTION__, conn_info->resp_ie_len, MAX_REQ_LINE));
			return err;
		}
	} else {
		conn_info->resp_ie_len = 0;
	}
	WL_DBG(("req len (%d) resp len (%d)\n", conn_info->req_ie_len,
		conn_info->resp_ie_len));

	return err;
}

static void wl_ch_to_chanspec(int ch, struct wl_join_params *join_params,
        size_t *join_params_size)
{
	chanspec_t chanspec = 0;

	if (ch != 0) {
		join_params->params.chanspec_num = 1;
		join_params->params.chanspec_list[0] = ch;

		if (join_params->params.chanspec_list[0] <= CH_MAX_2G_CHANNEL)
			chanspec |= WL_CHANSPEC_BAND_2G;
		else
			chanspec |= WL_CHANSPEC_BAND_5G;

		chanspec |= WL_CHANSPEC_BW_20;
		chanspec |= WL_CHANSPEC_CTL_SB_NONE;

		*join_params_size += WL_ASSOC_PARAMS_FIXED_SIZE +
			join_params->params.chanspec_num * sizeof(chanspec_t);

		join_params->params.chanspec_list[0]  &= WL_CHANSPEC_CHAN_MASK;
		join_params->params.chanspec_list[0] |= chanspec;
		join_params->params.chanspec_list[0] =
			htodchanspec(join_params->params.chanspec_list[0]);

		join_params->params.chanspec_num =
			htod32(join_params->params.chanspec_num);

		WL_DBG(("%s  join_params->params.chanspec_list[0]= %X\n",
			__FUNCTION__, join_params->params.chanspec_list[0]));

	}
}

static s32 wl_update_bss_info(struct wl_priv *wl, struct net_device *ndev)
{
	struct cfg80211_bss *bss;
	struct wl_bss_info *bi;
	struct wlc_ssid *ssid;
	struct bcm_tlv *tim;
	u16 beacon_interval;
	u8 dtim_period;
	size_t ie_len;
	u8 *ie;
	u8 *curbssid;
	s32 err = 0;
	struct wiphy *wiphy;
	wiphy = wl_to_wiphy(wl);

	if (wl_is_ibssmode(wl, ndev))
		return err;

	ssid = (struct wlc_ssid *)wl_read_prof(wl, WL_PROF_SSID);
	curbssid = wl_read_prof(wl, WL_PROF_BSSID);
	bss = cfg80211_get_bss(wiphy, NULL, curbssid,
		ssid->SSID, ssid->SSID_len, WLAN_CAPABILITY_ESS,
		WLAN_CAPABILITY_ESS);

	mutex_lock(&wl->usr_sync);
	if (unlikely(!bss)) {
		WL_DBG(("Could not find the AP\n"));
		*(u32 *) wl->extra_buf = htod32(WL_EXTRA_BUF_MAX);
		err = wldev_ioctl(ndev, WLC_GET_BSS_INFO,
			wl->extra_buf, WL_EXTRA_BUF_MAX, false);
		if (unlikely(err)) {
			WL_ERR(("Could not get bss info %d\n", err));
			goto update_bss_info_out;
		}
		bi = (struct wl_bss_info *)(wl->extra_buf + 4);
		if (memcmp(bi->BSSID.octet, curbssid, ETHER_ADDR_LEN)) {
			err = -EIO;
			goto update_bss_info_out;
		}
		err = wl_inform_single_bss(wl, bi);
		if (unlikely(err))
			goto update_bss_info_out;

		ie = ((u8 *)bi) + bi->ie_offset;
		ie_len = bi->ie_length;
		beacon_interval = cpu_to_le16(bi->beacon_period);
	} else {
		WL_DBG(("Found the AP in the list - BSSID %pM\n", bss->bssid));
		ie = bss->information_elements;
		ie_len = bss->len_information_elements;
		beacon_interval = bss->beacon_interval;
		cfg80211_put_bss(bss);
	}

	tim = bcm_parse_tlvs(ie, ie_len, WLAN_EID_TIM);
	if (tim) {
		dtim_period = tim->data[1];
	} else {
		/*
		* active scan was done so we could not get dtim
		* information out of probe response.
		* so we speficially query dtim information to dongle.
		*/
		err = wldev_ioctl(ndev, WLC_GET_DTIMPRD,
			&dtim_period, sizeof(dtim_period), false);
		if (unlikely(err)) {
			WL_ERR(("WLC_GET_DTIMPRD error (%d)\n", err));
			goto update_bss_info_out;
		}
	}

	wl_update_prof(wl, NULL, &beacon_interval, WL_PROF_BEACONINT);
	wl_update_prof(wl, NULL, &dtim_period, WL_PROF_DTIMPERIOD);

update_bss_info_out:
	mutex_unlock(&wl->usr_sync);
	return err;
}

static s32
wl_bss_roaming_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	struct wl_connect_info *conn_info = wl_to_conn(wl);
	s32 err = 0;
	u8 *curbssid;

	wl_get_assoc_ies(wl, ndev);
	wl_update_prof(wl, NULL, (void *)(e->addr.octet), WL_PROF_BSSID);
	curbssid = wl_read_prof(wl, WL_PROF_BSSID);
	wl_update_bss_info(wl, ndev);
	wl_update_pmklist(ndev, wl->pmk_list, err);
	cfg80211_roamed(ndev,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
		NULL,
#endif
		curbssid,
		conn_info->req_ie, conn_info->req_ie_len,
		conn_info->resp_ie, conn_info->resp_ie_len, GFP_KERNEL);
	WL_DBG(("Report roaming result\n"));

	wl_set_drv_status(wl, CONNECTED);

	return err;
}

static s32
wl_bss_connect_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data, bool completed)
{
	struct wl_connect_info *conn_info = wl_to_conn(wl);
	s32 err = 0;
	u8 *curbssid = wl_read_prof(wl, WL_PROF_BSSID);
	WL_DBG((" enter\n"));
	if (wl->scan_request) {
		wl_cfg80211_scan_abort(wl, ndev);
	}
	if (wl_get_drv_status(wl, CONNECTING)) {
		wl_clr_drv_status(wl, CONNECTING);
		if (completed) {
			wl_get_assoc_ies(wl, ndev);
			wl_update_prof(wl, NULL, (void *)(e->addr.octet), WL_PROF_BSSID);
			curbssid = wl_read_prof(wl, WL_PROF_BSSID);
			wl_update_bss_info(wl, ndev);
			wl_update_pmklist(ndev, wl->pmk_list, err);
			wl_set_drv_status(wl, CONNECTED);
		}
		cfg80211_connect_result(ndev,
			curbssid,
			conn_info->req_ie,
			conn_info->req_ie_len,
			conn_info->resp_ie,
			conn_info->resp_ie_len,
			completed ? WLAN_STATUS_SUCCESS : WLAN_STATUS_AUTH_TIMEOUT,
			GFP_KERNEL);
		if (completed)
			WL_INFO(("Report connect result - connection succeeded\n"));
		else
			WL_ERR(("Report connect result - connection failed\n"));
	}
	return err;
}

static s32
wl_notify_mic_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	u16 flags = ntoh16(e->flags);
	enum nl80211_key_type key_type;

	mutex_lock(&wl->usr_sync);
	if (flags & WLC_EVENT_MSG_GROUP)
		key_type = NL80211_KEYTYPE_GROUP;
	else
		key_type = NL80211_KEYTYPE_PAIRWISE;

	cfg80211_michael_mic_failure(ndev, (u8 *)&e->addr, key_type, -1,
		NULL, GFP_KERNEL);
	mutex_unlock(&wl->usr_sync);

	return 0;
}

static s32
wl_notify_scan_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	struct channel_info channel_inform;
	struct wl_scan_results *bss_list;
	u32 len = WL_SCAN_BUF_MAX;
	s32 err = 0;
	unsigned long flags;

	WL_DBG(("Enter \n"));
	if (wl->iscan_on && wl->iscan_kickstart)
		return wl_wakeup_iscan(wl_to_iscan(wl));

	mutex_lock(&wl->usr_sync);
	wl_clr_drv_status(wl, SCANNING);
	err = wldev_ioctl(ndev, WLC_GET_CHANNEL, &channel_inform,
		sizeof(channel_inform), false);
	if (unlikely(err)) {
		WL_ERR(("scan busy (%d)\n", err));
		goto scan_done_out;
	}
	channel_inform.scan_channel = dtoh32(channel_inform.scan_channel);
	if (unlikely(channel_inform.scan_channel)) {

		WL_DBG(("channel_inform.scan_channel (%d)\n",
			channel_inform.scan_channel));
	}
	wl->bss_list = wl->scan_results;
	bss_list = wl->bss_list;
	memset(bss_list, 0, len);
	bss_list->buflen = htod32(len);
	err = wldev_ioctl(ndev, WLC_SCAN_RESULTS, bss_list, len, false);
	if (unlikely(err)) {
		WL_ERR(("%s Scan_results error (%d)\n", ndev->name, err));
		err = -EINVAL;
		goto scan_done_out;
	}
	bss_list->buflen = dtoh32(bss_list->buflen);
	bss_list->version = dtoh32(bss_list->version);
	bss_list->count = dtoh32(bss_list->count);

	err = wl_inform_bss(wl);

scan_done_out:
	del_timer_sync(&wl->scan_timeout);
	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	if (wl->scan_request) {
		WL_DBG(("cfg80211_scan_done\n"));
		cfg80211_scan_done(wl->scan_request, false);
		wl->scan_request = NULL;
	}
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
	mutex_unlock(&wl->usr_sync);
	return err;
}
static s32
wl_frame_get_mgmt(u16 fc, const struct ether_addr *da,
	const struct ether_addr *sa, const struct ether_addr *bssid,
	u8 **pheader, u32 *body_len, u8 *pbody)
{
	struct dot11_management_header *hdr;
	u32 totlen = 0;
	s32 err = 0;
	u8 *offset;
	u32 prebody_len = *body_len;
	switch (fc) {
		case FC_ASSOC_REQ:
			/* capability , listen interval */
			totlen = DOT11_ASSOC_REQ_FIXED_LEN;
			*body_len += DOT11_ASSOC_REQ_FIXED_LEN;
			break;

		case FC_REASSOC_REQ:
			/* capability, listen inteval, ap address */
			totlen = DOT11_REASSOC_REQ_FIXED_LEN;
			*body_len += DOT11_REASSOC_REQ_FIXED_LEN;
			break;
	}
	totlen += DOT11_MGMT_HDR_LEN + prebody_len;
	*pheader = kzalloc(totlen, GFP_KERNEL);
	if (*pheader == NULL) {
		WL_ERR(("memory alloc failed \n"));
		return -ENOMEM;
	}
	hdr = (struct dot11_management_header *) (*pheader);
	hdr->fc = htol16(fc);
	hdr->durid = 0;
	hdr->seq = 0;
	offset = (u8*)(hdr + 1) + (totlen - DOT11_MGMT_HDR_LEN - prebody_len);
	bcopy((const char*)da, (u8*)&hdr->da, ETHER_ADDR_LEN);
	bcopy((const char*)sa, (u8*)&hdr->sa, ETHER_ADDR_LEN);
	bcopy((const char*)bssid, (u8*)&hdr->bssid, ETHER_ADDR_LEN);
	bcopy((const char*)pbody, offset, prebody_len);
	*body_len = totlen;
	return err;
}
static s32
wl_notify_rx_mgmt_frame(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	struct ieee80211_supported_band *band;
	struct wiphy *wiphy = wl_to_wiphy(wl);
	struct ether_addr da;
	struct ether_addr bssid;
	bool isfree = false;
	s32 err = 0;
	s32 freq;
	wifi_p2p_pub_act_frame_t *act_frm;
	wl_event_rx_frame_data_t *rxframe =
		(wl_event_rx_frame_data_t*)data;
	u32 event = ntoh32(e->event_type);
	u8 *mgmt_frame;
	u8 bsscfgidx = e->bsscfgidx;
	u32 mgmt_frame_len = ntoh32(e->datalen) - sizeof(wl_event_rx_frame_data_t);
	u16 channel = ((ntoh16(rxframe->channel) & WL_CHANSPEC_CHAN_MASK));

	memset(&bssid, 0, ETHER_ADDR_LEN);
	if (channel <= CH_MAX_2G_CHANNEL)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];

#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
	freq = ieee80211_channel_to_frequency(channel);
#else
	freq = ieee80211_channel_to_frequency(channel, band->band);
#endif
	if (event == WLC_E_ACTION_FRAME_RX) {
		wldev_iovar_getbuf_bsscfg(ndev, "cur_etheraddr",
		NULL, 0, ioctlbuf, sizeof(ioctlbuf), bsscfgidx);

		wldev_ioctl(ndev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN, false);
		memcpy(da.octet, ioctlbuf, ETHER_ADDR_LEN);
		err = wl_frame_get_mgmt(FC_ACTION, &da, &e->addr, &bssid,
			&mgmt_frame, &mgmt_frame_len,
			(u8 *)((wl_event_rx_frame_data_t *)rxframe + 1));
		if (err < 0) {
			WL_ERR(("%s: Error in receiving action frame len %d channel %d freq %d\n",
				__func__, mgmt_frame_len, channel, freq));
			goto exit;
		}
		isfree = true;
		act_frm =
			(wifi_p2p_pub_act_frame_t *) (&mgmt_frame[DOT11_MGMT_HDR_LEN]);
		/*
		 * After complete GO Negotiation, roll back to mpc mode
		 */
		 if (act_frm->subtype == P2P_PAF_GON_CONF) {
			wldev_iovar_setint(ndev, "mpc", 1);
		}
	} else {
		mgmt_frame = (u8 *)((wl_event_rx_frame_data_t *)rxframe + 1);
	}

	cfg80211_rx_mgmt(ndev, freq, mgmt_frame, mgmt_frame_len, GFP_ATOMIC);

	WL_DBG(("%s: mgmt_frame_len (%d) , e->datalen (%d), channel (%d), freq (%d)\n", __func__,
		mgmt_frame_len, ntoh32(e->datalen), channel, freq));

	if (isfree)
		kfree(mgmt_frame);
exit:
	return 0;
}

static void wl_init_conf(struct wl_conf *conf)
{
	s32 i = 0;
	WL_DBG(("Enter \n"));
	for (i = 0; i <= VWDEV_CNT; i++) {
		conf->mode[i].type = -1;
		conf->mode[i].ndev = NULL;
	}
	conf->frag_threshold = (u32)-1;
	conf->rts_threshold = (u32)-1;
	conf->retry_short = (u32)-1;
	conf->retry_long = (u32)-1;
	conf->tx_power = -1;
}

static void wl_init_prof(struct wl_priv *wl)
{
	unsigned long flags;

	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	memset(wl->profile, 0, sizeof(struct wl_profile));
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
}

static void wl_init_event_handler(struct wl_priv *wl)
{
	memset(wl->evt_handler, 0, sizeof(wl->evt_handler));

	wl->evt_handler[WLC_E_SCAN_COMPLETE] = wl_notify_scan_status;
	wl->evt_handler[WLC_E_LINK] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_DEAUTH_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_DEAUTH] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_DISASSOC_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_ASSOC_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_REASSOC_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_ROAM] = wl_notify_roaming_status;
	wl->evt_handler[WLC_E_MIC_ERROR] = wl_notify_mic_status;
	wl->evt_handler[WLC_E_SET_SSID] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_ACTION_FRAME_RX] = wl_notify_rx_mgmt_frame;
	wl->evt_handler[WLC_E_PROBREQ_MSG] = wl_notify_rx_mgmt_frame;
	wl->evt_handler[WLC_E_P2P_PROBREQ_MSG] = wl_notify_rx_mgmt_frame;
	wl->evt_handler[WLC_E_P2P_DISC_LISTEN_COMPLETE] = wl_cfgp2p_listen_complete;
	wl->evt_handler[WLC_E_ACTION_FRAME_COMPLETE] = wl_cfgp2p_action_tx_complete;
	wl->evt_handler[WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE] = wl_cfgp2p_action_tx_complete;

}

static s32 wl_init_priv_mem(struct wl_priv *wl)
{
	WL_DBG(("Enter \n"));
	wl->scan_results = (void *)kzalloc(WL_SCAN_BUF_MAX, GFP_KERNEL);
	if (unlikely(!wl->scan_results)) {
		WL_ERR(("Scan results alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->conf = (void *)kzalloc(sizeof(*wl->conf), GFP_KERNEL);
	if (unlikely(!wl->conf)) {
		WL_ERR(("wl_conf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->profile = (void *)kzalloc(sizeof(*wl->profile), GFP_KERNEL);
	if (unlikely(!wl->profile)) {
		WL_ERR(("wl_profile alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->bss_info = (void *)kzalloc(WL_BSS_INFO_MAX, GFP_KERNEL);
	if (unlikely(!wl->bss_info)) {
		WL_ERR(("Bss information alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->scan_req_int =
	    (void *)kzalloc(sizeof(*wl->scan_req_int), GFP_KERNEL);
	if (unlikely(!wl->scan_req_int)) {
		WL_ERR(("Scan req alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->ioctl_buf = (void *)kzalloc(WL_IOCTL_LEN_MAX, GFP_KERNEL);
	if (unlikely(!wl->ioctl_buf)) {
		WL_ERR(("Ioctl buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->escan_ioctl_buf = (void *)kzalloc(WLC_IOCTL_MAXLEN, GFP_KERNEL);
	if (unlikely(!wl->escan_ioctl_buf)) {
		WL_ERR(("Ioctl buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->extra_buf = (void *)kzalloc(WL_EXTRA_BUF_MAX, GFP_KERNEL);
	if (unlikely(!wl->extra_buf)) {
		WL_ERR(("Extra buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->iscan = (void *)kzalloc(sizeof(*wl->iscan), GFP_KERNEL);
	if (unlikely(!wl->iscan)) {
		WL_ERR(("Iscan buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->fw = (void *)kzalloc(sizeof(*wl->fw), GFP_KERNEL);
	if (unlikely(!wl->fw)) {
		WL_ERR(("fw object alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->pmk_list = (void *)kzalloc(sizeof(*wl->pmk_list), GFP_KERNEL);
	if (unlikely(!wl->pmk_list)) {
		WL_ERR(("pmk list alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->sta_info = (void *)kzalloc(sizeof(*wl->sta_info), GFP_KERNEL);
	if (unlikely(!wl->sta_info)) {
		WL_ERR(("sta info  alloc failed\n"));
		goto init_priv_mem_out;
	}
	return 0;

init_priv_mem_out:
	wl_deinit_priv_mem(wl);

	return -ENOMEM;
}

static void wl_deinit_priv_mem(struct wl_priv *wl)
{
	kfree(wl->scan_results);
	wl->scan_results = NULL;
	kfree(wl->bss_info);
	wl->bss_info = NULL;
	kfree(wl->conf);
	wl->conf = NULL;
	kfree(wl->profile);
	wl->profile = NULL;
	kfree(wl->scan_req_int);
	wl->scan_req_int = NULL;
	kfree(wl->ioctl_buf);
	wl->ioctl_buf = NULL;
	kfree(wl->escan_ioctl_buf);
	wl->escan_ioctl_buf = NULL;
	kfree(wl->extra_buf);
	wl->extra_buf = NULL;
	kfree(wl->iscan);
	wl->iscan = NULL;
	kfree(wl->fw);
	wl->fw = NULL;
	kfree(wl->pmk_list);
	wl->pmk_list = NULL;
	kfree(wl->sta_info);
	wl->sta_info = NULL;
	if (wl->ap_info) {
		kfree(wl->ap_info->wpa_ie);
		kfree(wl->ap_info->rsn_ie);
		kfree(wl->ap_info->wps_ie);
		kfree(wl->ap_info);
		wl->ap_info = NULL;
	}
}

static s32 wl_create_event_handler(struct wl_priv *wl)
{
	int ret = 0;
	WL_DBG(("Enter \n"));

	wl->event_tsk.thr_pid = DHD_PID_KT_INVALID;
	PROC_START(wl_event_handler, wl, &wl->event_tsk, 0);
	if (wl->event_tsk.thr_pid < 0)
		ret = -ENOMEM;
	return ret;
}

static void wl_destroy_event_handler(struct wl_priv *wl)
{
	if (wl->event_tsk.thr_pid >= 0)
		PROC_STOP(&wl->event_tsk);
}

static void wl_term_iscan(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	WL_TRACE(("In\n"));
	if (wl->iscan_on && iscan->tsk) {
		iscan->state = WL_ISCAN_STATE_IDLE;
		WL_INFO(("SIGTERM\n"));
		send_sig(SIGTERM, iscan->tsk, 1);
		WL_DBG(("kthread_stop\n"));
		kthread_stop(iscan->tsk);
		iscan->tsk = NULL;
	}
}

static void wl_notify_iscan_complete(struct wl_iscan_ctrl *iscan, bool aborted)
{
	struct wl_priv *wl = iscan_to_wl(iscan);
	unsigned long flags;

	WL_DBG(("Enter \n"));
	if (unlikely(!wl_get_drv_status(wl, SCANNING))) {
		wl_clr_drv_status(wl, SCANNING);
		WL_ERR(("Scan complete while device not scanning\n"));
		return;
	}
	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	wl_clr_drv_status(wl, SCANNING);
	if (likely(wl->scan_request)) {
		cfg80211_scan_done(wl->scan_request, aborted);
		wl->scan_request = NULL;
	}
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
	wl->iscan_kickstart = false;
}

static s32 wl_wakeup_iscan(struct wl_iscan_ctrl *iscan)
{
	if (likely(iscan->state != WL_ISCAN_STATE_IDLE)) {
		WL_DBG(("wake up iscan\n"));
		up(&iscan->sync);
		return 0;
	}

	return -EIO;
}

static s32
wl_get_iscan_results(struct wl_iscan_ctrl *iscan, u32 *status,
	struct wl_scan_results **bss_list)
{
	struct wl_iscan_results list;
	struct wl_scan_results *results;
	struct wl_iscan_results *list_buf;
	s32 err = 0;

	WL_DBG(("Enter \n"));
	memset(iscan->scan_buf, 0, WL_ISCAN_BUF_MAX);
	list_buf = (struct wl_iscan_results *)iscan->scan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WL_ISCAN_BUF_MAX);
	err = wldev_iovar_getbuf(iscan->dev, "iscanresults", &list,
		WL_ISCAN_RESULTS_FIXED_SIZE, iscan->scan_buf,
		WL_ISCAN_BUF_MAX);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	results->count = dtoh32(results->count);
	WL_DBG(("results->count = %d\n", results->count));
	WL_DBG(("results->buflen = %d\n", results->buflen));
	*status = dtoh32(list_buf->status);
	*bss_list = results;

	return err;
}

static s32 wl_iscan_done(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	iscan->state = WL_ISCAN_STATE_IDLE;
	mutex_lock(&wl->usr_sync);
	wl_inform_bss(wl);
	wl_notify_iscan_complete(iscan, false);
	mutex_unlock(&wl->usr_sync);

	return err;
}

static s32 wl_iscan_pending(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	/* Reschedule the timer */
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms * HZ / 1000);
	iscan->timer_on = 1;

	return err;
}

static s32 wl_iscan_inprogress(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	mutex_lock(&wl->usr_sync);
	wl_inform_bss(wl);
	wl_run_iscan(iscan, NULL, WL_SCAN_ACTION_CONTINUE);
	mutex_unlock(&wl->usr_sync);
	/* Reschedule the timer */
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms * HZ / 1000);
	iscan->timer_on = 1;

	return err;
}

static s32 wl_iscan_aborted(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	iscan->state = WL_ISCAN_STATE_IDLE;
	mutex_lock(&wl->usr_sync);
	wl_notify_iscan_complete(iscan, true);
	mutex_unlock(&wl->usr_sync);

	return err;
}

static s32 wl_iscan_thread(void *data)
{
	struct sched_param param = {.sched_priority = MAX_RT_PRIO - 1 };
	struct wl_iscan_ctrl *iscan = (struct wl_iscan_ctrl *)data;
	struct wl_priv *wl = iscan_to_wl(iscan);
	u32 status;
	int err = 0;

	sched_setscheduler(current, SCHED_FIFO, &param);
	allow_signal(SIGTERM);
	status = WL_SCAN_RESULTS_PARTIAL;
	while (likely(!down_interruptible(&iscan->sync))) {
		if (kthread_should_stop())
			break;
		if (iscan->timer_on) {
			del_timer_sync(&iscan->timer);
			iscan->timer_on = 0;
		}
		mutex_lock(&wl->usr_sync);
		err = wl_get_iscan_results(iscan, &status, &wl->bss_list);
		if (unlikely(err)) {
			status = WL_SCAN_RESULTS_ABORTED;
			WL_ERR(("Abort iscan\n"));
		}
		mutex_unlock(&wl->usr_sync);
		iscan->iscan_handler[status] (wl);
	}
	if (iscan->timer_on) {
		del_timer_sync(&iscan->timer);
		iscan->timer_on = 0;
	}
	WL_DBG(("%s was terminated\n", __func__));

	return 0;
}

static void wl_scan_timeout(unsigned long data)
{
	struct wl_priv *wl = (struct wl_priv *)data;

	if (wl->scan_request) {
		WL_ERR(("timer expired\n"));
		if (wl->escan_on)
			wl_notify_escan_complete(wl, true);
		else
			wl_notify_iscan_complete(wl_to_iscan(wl), true);
	}
}

static void wl_iscan_timer(unsigned long data)
{
	struct wl_iscan_ctrl *iscan = (struct wl_iscan_ctrl *)data;

	if (iscan) {
		iscan->timer_on = 0;
		WL_DBG(("timer expired\n"));
		wl_wakeup_iscan(iscan);
	}
}

static s32 wl_invoke_iscan(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	int err = 0;

	if (wl->iscan_on && !iscan->tsk) {
		iscan->state = WL_ISCAN_STATE_IDLE;
		sema_init(&iscan->sync, 0);
		iscan->tsk = kthread_run(wl_iscan_thread, iscan, "wl_iscan");
		if (IS_ERR(iscan->tsk)) {
			WL_ERR(("Could not create iscan thread\n"));
			iscan->tsk = NULL;
			return -ENOMEM;
		}
	}

	return err;
}

static void wl_init_iscan_handler(struct wl_iscan_ctrl *iscan)
{
	memset(iscan->iscan_handler, 0, sizeof(iscan->iscan_handler));
	iscan->iscan_handler[WL_SCAN_RESULTS_SUCCESS] = wl_iscan_done;
	iscan->iscan_handler[WL_SCAN_RESULTS_PARTIAL] = wl_iscan_inprogress;
	iscan->iscan_handler[WL_SCAN_RESULTS_PENDING] = wl_iscan_pending;
	iscan->iscan_handler[WL_SCAN_RESULTS_ABORTED] = wl_iscan_aborted;
	iscan->iscan_handler[WL_SCAN_RESULTS_NO_MEM] = wl_iscan_aborted;
}

static void wl_notify_escan_complete(struct wl_priv *wl, bool aborted)
{
	unsigned long flags;

	WL_DBG(("Enter \n"));
	wl_clr_drv_status(wl, SCANNING);
	if (wl->p2p_supported && p2p_on(wl))
		wl_clr_p2p_status(wl, SCANNING);

	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	if (likely(wl->scan_request)) {
		cfg80211_scan_done(wl->scan_request, aborted);
		wl->scan_request = NULL;
	}
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
}

static s32 wl_escan_handler(struct wl_priv *wl,
	struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	s32 err = BCME_OK;
	s32 status = ntoh32(e->status);
	wl_bss_info_t *bi;
	wl_escan_result_t *escan_result;
	wl_bss_info_t *bss = NULL;
	wl_scan_results_t *list;
	u32 bi_length;
	u32 i;
	WL_DBG((" enter event type : %d, status : %d \n",
		ntoh32(e->event_type), ntoh32(e->status)));
	if (!wl->escan_on &&
		!wl_get_drv_status(wl, SCANNING)) {
		WL_ERR(("escan is not ready \n"));
		return err;
	}

	if (status == WLC_E_STATUS_PARTIAL) {
		WL_INFO(("WLC_E_STATUS_PARTIAL \n"));
		escan_result = (wl_escan_result_t *) data;
		if (!escan_result) {
			WL_ERR(("Invalid escan result (NULL pointer)\n"));
			goto exit;
		}
		if (dtoh16(escan_result->bss_count) != 1) {
			WL_ERR(("Invalid bss_count %d: ignoring\n", escan_result->bss_count));
			goto exit;
		}
		bi = escan_result->bss_info;
		if (!bi) {
			WL_ERR(("Invalid escan bss info (NULL pointer)\n"));
			goto exit;
		}
		bi_length = dtoh32(bi->length);
		if (bi_length != (dtoh32(escan_result->buflen) - WL_ESCAN_RESULTS_FIXED_SIZE)) {
			WL_ERR(("Invalid bss_info length %d: ignoring\n", bi_length));
			goto exit;
		}
		list = (wl_scan_results_t *)wl->escan_info.escan_buf;
		if (bi_length > ESCAN_BUF_SIZE - list->buflen) {
			WL_ERR(("Buffer is too small: ignoring\n"));
			goto exit;
		}
#define WLC_BSS_RSSI_ON_CHANNEL 0x0002
		for (i = 0; i < list->count; i++) {
			bss = bss ? (wl_bss_info_t *)((uintptr)bss + dtoh32(bss->length))
				: list->bss_info;

			if (!bcmp(&bi->BSSID, &bss->BSSID, ETHER_ADDR_LEN) &&
				CHSPEC_BAND(bi->chanspec) == CHSPEC_BAND(bss->chanspec) &&
				bi->SSID_len == bss->SSID_len &&
				!bcmp(bi->SSID, bss->SSID, bi->SSID_len)) {
				if ((bss->flags & WLC_BSS_RSSI_ON_CHANNEL) ==
					(bi->flags & WLC_BSS_RSSI_ON_CHANNEL)) {
					/* preserve max RSSI if the measurements are
					 * both on-channel or both off-channel
					 */
					bss->RSSI = MAX(bss->RSSI, bi->RSSI);
				} else if ((bss->flags & WLC_BSS_RSSI_ON_CHANNEL) &&
					(bi->flags & WLC_BSS_RSSI_ON_CHANNEL) == 0) {
					/* preserve the on-channel rssi measurement
					 * if the new measurement is off channel
					 */
					bss->RSSI = bi->RSSI;
					bss->flags |= WLC_BSS_RSSI_ON_CHANNEL;
				}

				goto exit;
			}
		}
		memcpy(&(wl->escan_info.escan_buf[list->buflen]), bi, bi_length);
		list->version = dtoh32(bi->version);
		list->buflen += bi_length;
		list->count++;

	}
	else if (status == WLC_E_STATUS_SUCCESS) {
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
		if (likely(wl->scan_request)) {
			mutex_lock(&wl->usr_sync);
			del_timer_sync(&wl->scan_timeout);
			WL_INFO(("ESCAN COMPLETED\n"));
			wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
			wl_inform_bss(wl);
			wl_notify_escan_complete(wl, false);
			mutex_unlock(&wl->usr_sync);
		}
	}
	else if (status == WLC_E_STATUS_ABORT) {
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
		if (likely(wl->scan_request)) {
			mutex_lock(&wl->usr_sync);
			del_timer_sync(&wl->scan_timeout);
			WL_INFO(("ESCAN ABORTED\n"));
			wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
			wl_inform_bss(wl);
			wl_notify_escan_complete(wl, true);
			mutex_unlock(&wl->usr_sync);
		}
	}
	else {
		WL_ERR(("unexpected Escan Event %d : abort\n", status));
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
		if (likely(wl->scan_request)) {
			mutex_lock(&wl->usr_sync);
			del_timer_sync(&wl->scan_timeout);
			wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
			wl_inform_bss(wl);
			wl_notify_escan_complete(wl, true);
			mutex_unlock(&wl->usr_sync);
		}
	}
exit:
	return err;
}

static s32 wl_init_scan(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	int err = 0;

	if (wl->iscan_on) {
		iscan->dev = wl_to_prmry_ndev(wl);
		iscan->state = WL_ISCAN_STATE_IDLE;
		wl_init_iscan_handler(iscan);
		iscan->timer_ms = WL_ISCAN_TIMER_INTERVAL_MS;
		init_timer(&iscan->timer);
		iscan->timer.data = (unsigned long) iscan;
		iscan->timer.function = wl_iscan_timer;
		sema_init(&iscan->sync, 0);
		iscan->tsk = kthread_run(wl_iscan_thread, iscan, "wl_iscan");
		if (IS_ERR(iscan->tsk)) {
			WL_ERR(("Could not create iscan thread\n"));
			iscan->tsk = NULL;
			return -ENOMEM;
		}
		iscan->data = wl;
	} else if (wl->escan_on) {
		wl->evt_handler[WLC_E_ESCAN_RESULT] = wl_escan_handler;
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
	}
	/* Init scan_timeout timer */
	init_timer(&wl->scan_timeout);
	wl->scan_timeout.data = (unsigned long) wl;
	wl->scan_timeout.function = wl_scan_timeout;

	return err;
}

static void wl_init_fw(struct wl_fw_ctrl *fw)
{
	fw->status = 0;
}

static s32 wl_init_priv(struct wl_priv *wl)
{
	struct wiphy *wiphy = wl_to_wiphy(wl);
	s32 err = 0;
	s32 i = 0;

	wl->scan_request = NULL;
	wl->pwr_save = !!(wiphy->flags & WIPHY_FLAG_PS_ON_BY_DEFAULT);
	wl->iscan_on = false;
	wl->escan_on = true;
	wl->roam_on = false;
	wl->iscan_kickstart = false;
	wl->active_scan = true;
	wl->dongle_up = false;
	wl->rf_blocked = false;

	for (i = 0; i < VWDEV_CNT; i++)
		wl->vwdev[i] = NULL;

	init_waitqueue_head(&wl->dongle_event_wait);
	wl_init_eq(wl);
	err = wl_init_priv_mem(wl);
	if (unlikely(err))
		return err;
	if (unlikely(wl_create_event_handler(wl)))
		return -ENOMEM;
	wl_init_event_handler(wl);
	mutex_init(&wl->usr_sync);
	err = wl_init_scan(wl);
	if (unlikely(err))
		return err;
	wl_init_fw(wl->fw);
	wl_init_conf(wl->conf);
	wl_init_prof(wl);
	wl_link_down(wl);

	return err;
}

static void wl_deinit_priv(struct wl_priv *wl)
{
	wl_destroy_event_handler(wl);
	wl->dongle_up = false;	/* dongle down */
	wl_flush_eq(wl);
	wl_link_down(wl);
	del_timer_sync(&wl->scan_timeout);
	wl_term_iscan(wl);
	wl_deinit_priv_mem(wl);
}

#if defined(DHD_P2P_DEV_ADDR_FROM_SYSFS) && defined(CONFIG_SYSCTL)
s32 wl_cfg80211_sysctl_export_devaddr(void *data)
{
	/* Export the p2p_dev_addr via sysctl interface
	 * so that wpa_supplicant can access it
	 */
	dhd_pub_t *dhd = (dhd_pub_t *)data;
	struct wl_priv *wl = wlcfg_drv_priv;

	wl_cfgp2p_generate_bss_mac(&dhd->mac, &wl->p2p->dev_addr, &wl->p2p->int_addr);

	sprintf((char *)&wl_sysctl_macstring[0], MACSTR, MAC2STR(wl->p2p->dev_addr.octet));
	sprintf((char *)&wl_sysctl_macstring[1], MACSTR, MAC2STR(wl->p2p->int_addr.octet));

	return 0;
}
#endif /* CONFIG_SYSCTL */

s32 wl_cfg80211_attach_post(struct net_device *ndev)
{
	struct wl_priv * wl = NULL;
	s32 err = 0;
	WL_TRACE(("In\n"));
	if (unlikely(!ndev)) {
		WL_ERR(("ndev is invaild\n"));
		return -ENODEV;
	}
	wl = wlcfg_drv_priv;
	if (wl && !wl_get_drv_status(wl, READY)) {
			if (wl->wdev &&
				wl_cfgp2p_supported(wl, ndev)) {
				wl->wdev->wiphy->interface_modes |=
					(BIT(NL80211_IFTYPE_P2P_CLIENT)|
					BIT(NL80211_IFTYPE_P2P_GO));
				if ((err = wl_cfgp2p_init_priv(wl)) != 0)
					goto fail;
#if defined(DHD_P2P_DEV_ADDR_FROM_SYSFS) && defined(CONFIG_SYSCTL)
				wl_cfg80211_sysctl_export_devaddr(wl->pub);
#endif
				wl->p2p_supported = true;
			}
	} else
		return -ENODEV;

	wl_set_drv_status(wl, READY);
fail:
	return err;
}
s32 wl_cfg80211_attach(struct net_device *ndev, void *data)
{
	struct wireless_dev *wdev;
	struct wl_priv *wl;
	s32 err = 0;

	WL_TRACE(("In\n"));
	if (unlikely(!ndev)) {
		WL_ERR(("ndev is invaild\n"));
		return -ENODEV;
	}
	WL_DBG(("func %p\n", wl_cfg80211_get_sdio_func()));
	wdev = wl_alloc_wdev(&wl_cfg80211_get_sdio_func()->dev);
	if (unlikely(IS_ERR(wdev)))
		return -ENOMEM;

	wdev->iftype = wl_mode_to_nl80211_iftype(WL_MODE_BSS);
	wl = (struct wl_priv *)wiphy_priv(wdev->wiphy);
	wl->wdev = wdev;
	wl->pub = data;

	ndev->ieee80211_ptr = wdev;
	SET_NETDEV_DEV(ndev, wiphy_dev(wdev->wiphy));
	wdev->netdev = ndev;

	err = wl_init_priv(wl);
	if (unlikely(err)) {
		WL_ERR(("Failed to init iwm_priv (%d)\n", err));
		goto cfg80211_attach_out;
	}

	err = wl_setup_rfkill(wl, TRUE);
	if (unlikely(err)) {
		WL_ERR(("Failed to setup rfkill %d\n", err));
		goto cfg80211_attach_out;
	}

#if defined(DHD_P2P_DEV_ADDR_FROM_SYSFS) && defined(CONFIG_SYSCTL)
	if (!(wl_sysctl_hdr = register_sysctl_table(wl_sysctl_table))) {
		WL_ERR(("%s: sysctl register failed!! \n", __func__));
		goto cfg80211_attach_out;
	}
#endif
#if defined(COEX_DHCP)
	if (wl_cfg80211_btcoex_init(wl))
		goto cfg80211_attach_out;
#endif /* COEX_DHCP */

	wlcfg_drv_priv = wl;
	return err;

cfg80211_attach_out:
	err = wl_setup_rfkill(wl, FALSE);
	wl_free_wdev(wl);
	return err;
}

void wl_cfg80211_detach(void)
{
	struct wl_priv *wl;

	wl = wlcfg_drv_priv;

	WL_TRACE(("In\n"));

#if defined(COEX_DHCP)
	wl_cfg80211_btcoex_deinit(wl);
#endif /* COEX_DHCP */

#if defined(DHD_P2P_DEV_ADDR_FROM_SYSFS) && defined(CONFIG_SYSCTL)
	if (wl_sysctl_hdr)
		unregister_sysctl_table(wl_sysctl_hdr);
#endif
	wl_setup_rfkill(wl, FALSE);
	if (wl->p2p_supported)
		wl_cfgp2p_deinit_priv(wl);
	wl_deinit_priv(wl);
	wlcfg_drv_priv = NULL;
	wl_clear_sdio_func();
	wl_free_wdev(wl);
}

static void wl_wakeup_event(struct wl_priv *wl)
{
	if (wl->event_tsk.thr_pid >= 0) {
		DHD_OS_WAKE_LOCK(wl->pub);
		up(&wl->event_tsk.sema);
	}
}

static s32 wl_event_handler(void *data)
{
	struct net_device *netdev;
	struct wl_priv *wl = NULL;
	struct wl_event_q *e;
	tsk_ctl_t *tsk = (tsk_ctl_t *)data;

	wl = (struct wl_priv *)tsk->parent;
	complete(&tsk->completed);

	while (down_interruptible (&tsk->sema) == 0) {
		SMP_RD_BARRIER_DEPENDS();
		if (tsk->terminated)
			break;
		while ((e = wl_deq_event(wl))) {
			WL_DBG(("event type (%d), if idx: %d\n", e->etype, e->emsg.ifidx));
			netdev = dhd_idx2net((struct dhd_pub *)(wl->pub), e->emsg.ifidx);
			if (!netdev)
				netdev = wl_to_prmry_ndev(wl);
			if (e->etype < WLC_E_LAST && wl->evt_handler[e->etype]) {
				wl->evt_handler[e->etype] (wl, netdev, &e->emsg, e->edata);
			} else {
				WL_DBG(("Unknown Event (%d): ignoring\n", e->etype));
			}
			wl_put_event(e);
		}
		DHD_OS_WAKE_UNLOCK(wl->pub);
	}
	WL_DBG(("%s was terminated\n", __func__));
	complete_and_exit(&tsk->completed, 0);
	return 0;
}

void
wl_cfg80211_event(struct net_device *ndev, const wl_event_msg_t * e, void *data)
{
	u32 event_type = ntoh32(e->event_type);
	struct wl_priv *wl = wlcfg_drv_priv;

#if (WL_DBG_LEVEL > 0)
	s8 *estr = (event_type <= sizeof(wl_dbg_estr) / WL_DBG_ESTR_MAX - 1) ?
	    wl_dbg_estr[event_type] : (s8 *) "Unknown";
	WL_DBG(("event_type (%d):" "WLC_E_" "%s\n", event_type, estr));
#endif /* (WL_DBG_LEVEL > 0) */

	if (event_type == WLC_E_PFN_NET_FOUND)
		WL_ERR((" PNO Event\n"));

	if (likely(!wl_enq_event(wl, ndev, event_type, e, data)))
		wl_wakeup_event(wl);
}

static void wl_init_eq(struct wl_priv *wl)
{
	wl_init_eq_lock(wl);
	INIT_LIST_HEAD(&wl->eq_list);
}

static void wl_flush_eq(struct wl_priv *wl)
{
	struct wl_event_q *e;
	unsigned long flags;

	flags = wl_lock_eq(wl);
	while (!list_empty(&wl->eq_list)) {
		e = list_first_entry(&wl->eq_list, struct wl_event_q, eq_list);
		list_del(&e->eq_list);
		kfree(e);
	}
	wl_unlock_eq(wl, flags);
}

/*
* retrieve first queued event from head
*/

static struct wl_event_q *wl_deq_event(struct wl_priv *wl)
{
	struct wl_event_q *e = NULL;
	unsigned long flags;

	flags = wl_lock_eq(wl);
	if (likely(!list_empty(&wl->eq_list))) {
		e = list_first_entry(&wl->eq_list, struct wl_event_q, eq_list);
		list_del(&e->eq_list);
	}
	wl_unlock_eq(wl, flags);

	return e;
}

/*
 * push event to tail of the queue
 */

static s32
wl_enq_event(struct wl_priv *wl, struct net_device *ndev, u32 event, const wl_event_msg_t *msg,
	void *data)
{
	struct wl_event_q *e;
	s32 err = 0;
	uint32 evtq_size;
	uint32 data_len;
	unsigned long flags;
	gfp_t aflags;

	data_len = 0;
	if (data)
		data_len = ntoh32(msg->datalen);
	evtq_size = sizeof(struct wl_event_q) + data_len;
	aflags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	e = kzalloc(evtq_size, aflags);
	if (unlikely(!e)) {
		WL_ERR(("event alloc failed\n"));
		return -ENOMEM;
	}
	e->etype = event;
	memcpy(&e->emsg, msg, sizeof(wl_event_msg_t));
	if (data)
		memcpy(e->edata, data, data_len);
	flags = wl_lock_eq(wl);
	list_add_tail(&e->eq_list, &wl->eq_list);
	wl_unlock_eq(wl, flags);

	return err;
}

static void wl_put_event(struct wl_event_q *e)
{
	kfree(e);
}

void wl_cfg80211_set_sdio_func(void *func)
{
	cfg80211_sdio_func = (struct sdio_func *)func;
}

static void wl_clear_sdio_func(void)
{
	cfg80211_sdio_func = NULL;
}

struct sdio_func *wl_cfg80211_get_sdio_func(void)
{
	return cfg80211_sdio_func;
}

static s32 wl_dongle_mode(struct wl_priv *wl, struct net_device *ndev, s32 iftype)
{
	s32 infra = 0;
	s32 err = 0;
	s32 mode = 0;
	switch (iftype) {
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_WDS:
		WL_ERR(("type (%d) : currently we do not support this mode\n",
			iftype));
		err = -EINVAL;
		return err;
	case NL80211_IFTYPE_ADHOC:
		mode = WL_MODE_IBSS;
		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		mode = WL_MODE_BSS;
		infra = 1;
		break;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
		mode = WL_MODE_AP;
		infra = 1;
		break;
	default:
		err = -EINVAL;
		WL_ERR(("invalid type (%d)\n", iftype));
		return err;
	}
	infra = htod32(infra);
	err = wldev_ioctl(ndev, WLC_SET_INFRA, &infra, sizeof(infra), true);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_INFRA error (%d)\n", err));
		return err;
	}

	set_mode_by_netdev(wl, ndev, mode);

	return 0;
}
static s32 wl_dongle_add_remove_eventmsg(struct net_device *ndev, u16 event, bool add)
{
	s8 iovbuf[WL_EVENTING_MASK_LEN + 12];

	s8 eventmask[WL_EVENTING_MASK_LEN];
	s32 err = 0;

	/* Setup event_msgs */
	bcm_mkiovar("event_msgs", NULL, 0, iovbuf,
		sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_GET_VAR, iovbuf, sizeof(iovbuf), false);
	if (unlikely(err)) {
		WL_ERR(("Get event_msgs error (%d)\n", err));
		goto dongle_eventmsg_out;
	}
	memcpy(eventmask, iovbuf, WL_EVENTING_MASK_LEN);
	if (add) {
		setbit(eventmask, event);
	} else {
		clrbit(eventmask, event);
	}
	bcm_mkiovar("event_msgs", eventmask, WL_EVENTING_MASK_LEN, iovbuf,
		sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (unlikely(err)) {
		WL_ERR(("Set event_msgs error (%d)\n", err));
		goto dongle_eventmsg_out;
	}

dongle_eventmsg_out:
	return err;

}


#ifndef EMBEDDED_PLATFORM
static s32 wl_dongle_country(struct net_device *ndev, u8 ccode)
{

	s32 err = 0;

	return err;
}

static s32 wl_dongle_up(struct net_device *ndev, u32 up)
{
	s32 err = 0;

	err = wldev_ioctl(ndev, WLC_UP, &up, sizeof(up), true);
	if (unlikely(err)) {
		WL_ERR(("WLC_UP error (%d)\n", err));
	}
	return err;
}

static s32 wl_dongle_power(struct net_device *ndev, u32 power_mode)
{
	s32 err = 0;

	WL_TRACE(("In\n"));
	err = wldev_ioctl(ndev, WLC_SET_PM, &power_mode, sizeof(power_mode), true);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_PM error (%d)\n", err));
	}
	return err;
}

static s32
wl_dongle_glom(struct net_device *ndev, u32 glom, u32 dongle_align)
{
	s8 iovbuf[WL_EVENTING_MASK_LEN + 12];

	s32 err = 0;

	/* Match Host and Dongle rx alignment */
	bcm_mkiovar("bus:txglomalign", (char *)&dongle_align, 4, iovbuf,
		sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (unlikely(err)) {
		WL_ERR(("txglomalign error (%d)\n", err));
		goto dongle_glom_out;
	}
	/* disable glom option per default */
	bcm_mkiovar("bus:txglom", (char *)&glom, 4, iovbuf, sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (unlikely(err)) {
		WL_ERR(("txglom error (%d)\n", err));
		goto dongle_glom_out;
	}
dongle_glom_out:
	return err;
}

static s32
wl_dongle_roam(struct net_device *ndev, u32 roamvar, u32 bcn_timeout)
{
	s8 iovbuf[WL_EVENTING_MASK_LEN + 12];

	s32 err = 0;

	/* Setup timeout if Beacons are lost and roam is off to report link down */
	if (roamvar) {
		bcm_mkiovar("bcn_timeout", (char *)&bcn_timeout, 4, iovbuf,
			sizeof(iovbuf));
		err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
		if (unlikely(err)) {
			WL_ERR(("bcn_timeout error (%d)\n", err));
			goto dongle_rom_out;
		}
	}
	/* Enable/Disable built-in roaming to allow supplicant to take care of roaming */
	bcm_mkiovar("roam_off", (char *)&roamvar, 4, iovbuf, sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (unlikely(err)) {
		WL_ERR(("roam_off error (%d)\n", err));
		goto dongle_rom_out;
	}
dongle_rom_out:
	return err;
}

static s32
wl_dongle_scantime(struct net_device *ndev, s32 scan_assoc_time,
	s32 scan_unassoc_time)
{
	s32 err = 0;

	err = wldev_ioctl(ndev, WLC_SET_SCAN_CHANNEL_TIME, &scan_assoc_time,
		sizeof(scan_assoc_time), true);
	if (err) {
		if (err == -EOPNOTSUPP) {
			WL_INFO(("Scan assoc time is not supported\n"));
		} else {
			WL_ERR(("Scan assoc time error (%d)\n", err));
		}
		goto dongle_scantime_out;
	}
	err = wldev_ioctl(ndev, WLC_SET_SCAN_UNASSOC_TIME, &scan_unassoc_time,
		sizeof(scan_unassoc_time), true);
	if (err) {
		if (err == -EOPNOTSUPP) {
			WL_INFO(("Scan unassoc time is not supported\n"));
		} else {
			WL_ERR(("Scan unassoc time error (%d)\n", err));
		}
		goto dongle_scantime_out;
	}

dongle_scantime_out:
	return err;
}

static s32
wl_dongle_offload(struct net_device *ndev, s32 arpoe, s32 arp_ol)
{
	/* Room for "event_msgs" + '\0' + bitvec */
	s8 iovbuf[WL_EVENTING_MASK_LEN + 12];

	s32 err = 0;

	/* Set ARP offload */
	bcm_mkiovar("arpoe", (char *)&arpoe, 4, iovbuf, sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (err) {
		if (err == -EOPNOTSUPP)
			WL_INFO(("arpoe is not supported\n"));
		else
			WL_ERR(("arpoe error (%d)\n", err));

		goto dongle_offload_out;
	}
	bcm_mkiovar("arp_ol", (char *)&arp_ol, 4, iovbuf, sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (err) {
		if (err == -EOPNOTSUPP)
			WL_INFO(("arp_ol is not supported\n"));
		else
			WL_ERR(("arp_ol error (%d)\n", err));

		goto dongle_offload_out;
	}

dongle_offload_out:
	return err;
}

static s32 wl_pattern_atoh(s8 *src, s8 *dst)
{
	int i;
	if (strncmp(src, "0x", 2) != 0 && strncmp(src, "0X", 2) != 0) {
		WL_ERR(("Mask invalid format. Needs to start with 0x\n"));
		return -1;
	}
	src = src + 2;		/* Skip past 0x */
	if (strlen(src) % 2 != 0) {
		WL_ERR(("Mask invalid format. Needs to be of even length\n"));
		return -1;
	}
	for (i = 0; *src != '\0'; i++) {
		char num[3];
		strncpy(num, src, 2);
		num[2] = '\0';
		dst[i] = (u8) simple_strtoul(num, NULL, 16);
		src += 2;
	}
	return i;
}

static s32 wl_dongle_filter(struct net_device *ndev, u32 filter_mode)
{
	/* Room for "event_msgs" + '\0' + bitvec */
	s8 iovbuf[WL_EVENTING_MASK_LEN + 12];

	const s8 *str;
	struct wl_pkt_filter pkt_filter;
	struct wl_pkt_filter *pkt_filterp;
	s32 buf_len;
	s32 str_len;
	u32 mask_size;
	u32 pattern_size;
	s8 buf[256];
	s32 err = 0;

	/* add a default packet filter pattern */
	str = "pkt_filter_add";
	str_len = strlen(str);
	strncpy(buf, str, str_len);
	buf[str_len] = '\0';
	buf_len = str_len + 1;

	pkt_filterp = (struct wl_pkt_filter *)(buf + str_len + 1);

	/* Parse packet filter id. */
	pkt_filter.id = htod32(100);

	/* Parse filter polarity. */
	pkt_filter.negate_match = htod32(0);

	/* Parse filter type. */
	pkt_filter.type = htod32(0);

	/* Parse pattern filter offset. */
	pkt_filter.u.pattern.offset = htod32(0);

	/* Parse pattern filter mask. */
	mask_size = htod32(wl_pattern_atoh("0xff",
		(char *)pkt_filterp->u.pattern.
		    mask_and_pattern));

	/* Parse pattern filter pattern. */
	pattern_size = htod32(wl_pattern_atoh("0x00",
		(char *)&pkt_filterp->u.pattern.mask_and_pattern[mask_size]));

	if (mask_size != pattern_size) {
		WL_ERR(("Mask and pattern not the same size\n"));
		err = -EINVAL;
		goto dongle_filter_out;
	}

	pkt_filter.u.pattern.size_bytes = mask_size;
	buf_len += WL_PKT_FILTER_FIXED_LEN;
	buf_len += (WL_PKT_FILTER_PATTERN_FIXED_LEN + 2 * mask_size);

	/* Keep-alive attributes are set in local
	 * variable (keep_alive_pkt), and
	 * then memcpy'ed into buffer (keep_alive_pktp) since there is no
	 * guarantee that the buffer is properly aligned.
	 */
	memcpy((char *)pkt_filterp, &pkt_filter,
		WL_PKT_FILTER_FIXED_LEN + WL_PKT_FILTER_PATTERN_FIXED_LEN);

	err = wldev_ioctl(ndev, WLC_SET_VAR, buf, buf_len, true);
	if (err) {
		if (err == -EOPNOTSUPP) {
			WL_INFO(("filter not supported\n"));
		} else {
			WL_ERR(("filter (%d)\n", err));
		}
		goto dongle_filter_out;
	}

	/* set mode to allow pattern */
	bcm_mkiovar("pkt_filter_mode", (char *)&filter_mode, 4, iovbuf,
		sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (err) {
		if (err == -EOPNOTSUPP) {
			WL_INFO(("filter_mode not supported\n"));
		} else {
			WL_ERR(("filter_mode (%d)\n", err));
		}
		goto dongle_filter_out;
	}

dongle_filter_out:
	return err;
}
#endif				/* !EMBEDDED_PLATFORM */

s32 wl_config_dongle(struct wl_priv *wl, bool need_lock)
{
#ifndef DHD_SDALIGN
#define DHD_SDALIGN	32
#endif
	struct net_device *ndev;
	struct wireless_dev *wdev;
	s32 err = 0;

	WL_TRACE(("In\n"));
	if (wl->dongle_up) {
		WL_ERR(("Dongle is already up\n"));
		return err;
	}

	ndev = wl_to_prmry_ndev(wl);
	wdev = ndev->ieee80211_ptr;
	if (need_lock)
		rtnl_lock();
#ifndef EMBEDDED_PLATFORM
	err = wl_dongle_up(ndev, 0);
	if (unlikely(err)) {
		WL_ERR(("wl_dongle_up failed\n"));
		goto default_conf_out;
	}
	err = wl_dongle_country(ndev, 0);
	if (unlikely(err)) {
		WL_ERR(("wl_dongle_country failed\n"));
		goto default_conf_out;
	}
	err = wl_dongle_power(ndev, PM_FAST);
	if (unlikely(err)) {
		WL_ERR(("wl_dongle_power failed\n"));
		goto default_conf_out;
	}
	err = wl_dongle_glom(ndev, 0, DHD_SDALIGN);
	if (unlikely(err)) {
		WL_ERR(("wl_dongle_glom failed\n"));
		goto default_conf_out;
	}
	err = wl_dongle_roam(ndev, (wl->roam_on ? 0 : 1), 3);
	if (unlikely(err)) {
		WL_ERR(("wl_dongle_roam failed\n"));
		goto default_conf_out;
	}
	wl_dongle_scantime(ndev, 40, 80);
	wl_dongle_offload(ndev, 1, 0xf);
	wl_dongle_filter(ndev, 1);
#endif				/* !EMBEDDED_PLATFORM */

	err = wl_dongle_mode(wl, ndev, wdev->iftype);
	if (unlikely(err && err != -EINPROGRESS)) {
		WL_ERR(("wl_dongle_mode failed\n"));
		goto default_conf_out;
	}
	err = wl_dongle_probecap(wl);
	if (unlikely(err)) {
		WL_ERR(("wl_dongle_probecap failed\n"));
		goto default_conf_out;
	}

	/* -EINPROGRESS: Call commit handler */

default_conf_out:
	if (need_lock)
		rtnl_unlock();

	wl->dongle_up = true;

	return err;

}

static s32 wl_update_wiphybands(struct wl_priv *wl)
{
	struct wiphy *wiphy;
	s8 phylist_buf[128];
	s8 *phy;
	s32 err = 0;

	err = wldev_ioctl(wl_to_prmry_ndev(wl), WLC_GET_PHYLIST, phylist_buf,
		sizeof(phylist_buf), false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	phy = phylist_buf;
	for (; *phy; phy++) {
		if (*phy == 'a' || *phy == 'n') {
			wiphy = wl_to_wiphy(wl);
			wiphy->bands[IEEE80211_BAND_5GHZ] =
				&__wl_band_5ghz_a;
		}
	}
	return err;
}

static s32 __wl_cfg80211_up(struct wl_priv *wl)
{
	s32 err = 0;

	WL_TRACE(("In\n"));
	wl_debugfs_add_netdev_params(wl);

	err = wl_config_dongle(wl, false);
	if (unlikely(err))
		return err;
	dhd_monitor_init(wl->pub);
	wl_invoke_iscan(wl);
	wl_set_drv_status(wl, READY);
	return err;
}

static s32 __wl_cfg80211_down(struct wl_priv *wl)
{
	s32 err = 0;
	unsigned long flags;

	WL_TRACE(("In\n"));
	/* Check if cfg80211 interface is already down */
	if (!wl_get_drv_status(wl, READY))
		return err;	/* it is even not ready */

	wl_set_drv_status(wl, SCAN_ABORTING);

	wl_term_iscan(wl);
	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	if (wl->scan_request) {
		cfg80211_scan_done(wl->scan_request, true);
		wl->scan_request = NULL;
	}
	wl_clr_drv_status(wl, READY);
	wl_clr_drv_status(wl, SCANNING);
	wl_clr_drv_status(wl, SCAN_ABORTING);
	wl_clr_drv_status(wl, CONNECTING);
	wl_clr_drv_status(wl, CONNECTED);
	wl_clr_drv_status(wl, DISCONNECTING);
	if (wl_get_drv_status(wl, AP_CREATED)) {
		wl_clr_drv_status(wl, AP_CREATED);
		wl_clr_drv_status(wl, AP_CREATING);
	}
	wl_to_prmry_ndev(wl)->ieee80211_ptr->iftype =
		NL80211_IFTYPE_STATION;
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);

	wl->dongle_up = false;
	wl_flush_eq(wl);
	wl_link_down(wl);
	if (wl->p2p_supported)
		wl_cfgp2p_down(wl);
	dhd_monitor_uninit();

	wl_debugfs_remove_netdev(wl);

	return err;
}

s32 wl_cfg80211_up(void)
{
	struct wl_priv *wl;
	s32 err = 0;

	WL_TRACE(("In\n"));
	wl = wlcfg_drv_priv;
	mutex_lock(&wl->usr_sync);
	wl_cfg80211_attach_post(wl_to_prmry_ndev(wl));
	err = __wl_cfg80211_up(wl);
	if (err)
		WL_ERR(("__wl_cfg80211_up failed\n"));
	mutex_unlock(&wl->usr_sync);

	return err;
}

/* Private Event  to Supplicant with indication that FW hangs */
int wl_cfg80211_hang(struct net_device *dev, u16 reason)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	WL_ERR(("In : FW crash Eventing\n"));
	cfg80211_disconnected(dev, reason, NULL, 0, GFP_KERNEL);
	if (wl != NULL) {
		wl_link_down(wl);
	}
	return 0;
}

s32 wl_cfg80211_down(void)
{
	struct wl_priv *wl;
	s32 err = 0;

	WL_TRACE(("In\n"));
	wl = wlcfg_drv_priv;
	mutex_lock(&wl->usr_sync);
	err = __wl_cfg80211_down(wl);
	mutex_unlock(&wl->usr_sync);

	return err;
}

static s32 wl_dongle_probecap(struct wl_priv *wl)
{
	s32 err = 0;

	err = wl_update_wiphybands(wl);
	if (unlikely(err))
		return err;

	return err;
}

static void *wl_read_prof(struct wl_priv *wl, s32 item)
{
	unsigned long flags;
	void *rptr = NULL;

	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	switch (item) {
	case WL_PROF_SEC:
		rptr = &wl->profile->sec;
		break;
	case WL_PROF_ACT:
		rptr = &wl->profile->active;
		break;
	case WL_PROF_BSSID:
		rptr = &wl->profile->bssid;
		break;
	case WL_PROF_SSID:
		rptr = &wl->profile->ssid;
		break;
	}
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
	if (!rptr)
		WL_ERR(("invalid item (%d)\n", item));
	return rptr;
}

static s32
wl_update_prof(struct wl_priv *wl, const wl_event_msg_t *e, void *data,
	s32 item)
{
	s32 err = 0;
	struct wlc_ssid *ssid;
	unsigned long flags;

	flags = dhd_os_spin_lock((dhd_pub_t *)(wl->pub));
	switch (item) {
	case WL_PROF_SSID:
		ssid = (wlc_ssid_t *) data;
		memset(wl->profile->ssid.SSID, 0,
			sizeof(wl->profile->ssid.SSID));
		memcpy(wl->profile->ssid.SSID, ssid->SSID, ssid->SSID_len);
		wl->profile->ssid.SSID_len = ssid->SSID_len;
		break;
	case WL_PROF_BSSID:
		if (data)
			memcpy(wl->profile->bssid, data, ETHER_ADDR_LEN);
		else
			memset(wl->profile->bssid, 0, ETHER_ADDR_LEN);
		break;
	case WL_PROF_SEC:
		memcpy(&wl->profile->sec, data, sizeof(wl->profile->sec));
		break;
	case WL_PROF_ACT:
		wl->profile->active = *(bool *)data;
		break;
	case WL_PROF_BEACONINT:
		wl->profile->beacon_interval = *(u16 *)data;
		break;
	case WL_PROF_DTIMPERIOD:
		wl->profile->dtim_period = *(u8 *)data;
		break;
	default:
		WL_ERR(("unsupported item (%d)\n", item));
		err = -EOPNOTSUPP;
		break;
	}
	dhd_os_spin_unlock((dhd_pub_t *)(wl->pub), flags);
	return err;
}

void wl_cfg80211_dbg_level(u32 level)
{
	/*
	* prohibit to change debug level
	* by insmod parameter.
	* eventually debug level will be configured
	* in compile time by using CONFIG_XXX
	*/
	/* wl_dbg_level = level; */
}

static bool wl_is_ibssmode(struct wl_priv *wl, struct net_device *ndev)
{
	return get_mode_by_netdev(wl, ndev) == WL_MODE_IBSS;
}

static __used bool wl_is_ibssstarter(struct wl_priv *wl)
{
	return wl->ibss_starter;
}

static void wl_rst_ie(struct wl_priv *wl)
{
	struct wl_ie *ie = wl_to_ie(wl);

	ie->offset = 0;
}

static __used s32 wl_add_ie(struct wl_priv *wl, u8 t, u8 l, u8 *v)
{
	struct wl_ie *ie = wl_to_ie(wl);
	s32 err = 0;

	if (unlikely(ie->offset + l + 2 > WL_TLV_INFO_MAX)) {
		WL_ERR(("ei crosses buffer boundary\n"));
		return -ENOSPC;
	}
	ie->buf[ie->offset] = t;
	ie->buf[ie->offset + 1] = l;
	memcpy(&ie->buf[ie->offset + 2], v, l);
	ie->offset += l + 2;

	return err;
}

static s32 wl_mrg_ie(struct wl_priv *wl, u8 *ie_stream, u16 ie_size)
{
	struct wl_ie *ie = wl_to_ie(wl);
	s32 err = 0;

	if (unlikely(ie->offset + ie_size > WL_TLV_INFO_MAX)) {
		WL_ERR(("ei_stream crosses buffer boundary\n"));
		return -ENOSPC;
	}
	memcpy(&ie->buf[ie->offset], ie_stream, ie_size);
	ie->offset += ie_size;

	return err;
}

static s32 wl_cp_ie(struct wl_priv *wl, u8 *dst, u16 dst_size)
{
	struct wl_ie *ie = wl_to_ie(wl);
	s32 err = 0;

	if (unlikely(ie->offset > dst_size)) {
		WL_ERR(("dst_size is not enough\n"));
		return -ENOSPC;
	}
	memcpy(dst, &ie->buf[0], ie->offset);

	return err;
}

static u32 wl_get_ielen(struct wl_priv *wl)
{
	struct wl_ie *ie = wl_to_ie(wl);

	return ie->offset;
}

static void wl_link_up(struct wl_priv *wl)
{
	wl->link_up = true;
}

static void wl_link_down(struct wl_priv *wl)
{
	struct wl_connect_info *conn_info = wl_to_conn(wl);

	WL_DBG(("In\n"));
	wl->link_up = false;
	conn_info->req_ie_len = 0;
	conn_info->resp_ie_len = 0;
}

static unsigned long wl_lock_eq(struct wl_priv *wl)
{
	unsigned long flags;

	spin_lock_irqsave(&wl->eq_lock, flags);
	return flags;
}

static void wl_unlock_eq(struct wl_priv *wl, unsigned long flags)
{
	spin_unlock_irqrestore(&wl->eq_lock, flags);
}

static void wl_init_eq_lock(struct wl_priv *wl)
{
	spin_lock_init(&wl->eq_lock);
}

static void wl_delay(u32 ms)
{
	if (ms < 1000 / HZ) {
		cond_resched();
		mdelay(ms);
	} else {
		msleep(ms);
	}
}

s32 wl_cfg80211_read_fw(s8 *buf, u32 size)
{
	const struct firmware *fw_entry;
	struct wl_priv *wl;

	wl = wlcfg_drv_priv;

	fw_entry = wl->fw->fw_entry;

	if (fw_entry->size < wl->fw->ptr + size)
		size = fw_entry->size - wl->fw->ptr;

	memcpy(buf, &fw_entry->data[wl->fw->ptr], size);
	wl->fw->ptr += size;
	return size;
}

void wl_cfg80211_release_fw(void)
{
	struct wl_priv *wl;

	wl = wlcfg_drv_priv;
	release_firmware(wl->fw->fw_entry);
	wl->fw->ptr = 0;
}

void *wl_cfg80211_request_fw(s8 *file_name)
{
	struct wl_priv *wl;
	const struct firmware *fw_entry = NULL;
	s32 err = 0;

	WL_TRACE(("In\n"));
	WL_DBG(("file name : \"%s\"\n", file_name));
	wl = wlcfg_drv_priv;

	if (!test_bit(WL_FW_LOADING_DONE, &wl->fw->status)) {
		err = request_firmware(&wl->fw->fw_entry, file_name,
			&wl_cfg80211_get_sdio_func()->dev);
		if (unlikely(err)) {
			WL_ERR(("Could not download fw (%d)\n", err));
			goto req_fw_out;
		}
		set_bit(WL_FW_LOADING_DONE, &wl->fw->status);
		fw_entry = wl->fw->fw_entry;
		if (fw_entry) {
			WL_DBG(("fw size (%zd), data (%p)\n", fw_entry->size,
				fw_entry->data));
		}
	} else if (!test_bit(WL_NVRAM_LOADING_DONE, &wl->fw->status)) {
		err = request_firmware(&wl->fw->fw_entry, file_name,
			&wl_cfg80211_get_sdio_func()->dev);
		if (unlikely(err)) {
			WL_ERR(("Could not download nvram (%d)\n", err));
			goto req_fw_out;
		}
		set_bit(WL_NVRAM_LOADING_DONE, &wl->fw->status);
		fw_entry = wl->fw->fw_entry;
		if (fw_entry) {
			WL_DBG(("nvram size (%zd), data (%p)\n", fw_entry->size,
				fw_entry->data));
		}
	} else {
		WL_DBG(("Downloading already done. Nothing to do more\n"));
		err = -EPERM;
	}

req_fw_out:
	if (unlikely(err)) {
		return NULL;
	}
	wl->fw->ptr = 0;
	return (void *)fw_entry->data;
}

s8 *wl_cfg80211_get_fwname(void)
{
	struct wl_priv *wl;

	wl = wlcfg_drv_priv;
	strcpy(wl->fw->fw_name, WL_4329_FW_FILE);
	return wl->fw->fw_name;
}

s8 *wl_cfg80211_get_nvramname(void)
{
	struct wl_priv *wl;

	wl = wlcfg_drv_priv;
	strcpy(wl->fw->nvram_name, WL_4329_NVRAM_FILE);
	return wl->fw->nvram_name;
}

s32 wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr)
{
	struct wl_priv *wl;
	dhd_pub_t *dhd_pub;
	struct ether_addr p2pif_addr;

	wl = wlcfg_drv_priv;
	dhd_pub = (dhd_pub_t *)wl->pub;
	wl_cfgp2p_generate_bss_mac(&dhd_pub->mac, p2pdev_addr, &p2pif_addr);

	return 0;
}
s32 wl_cfg80211_set_p2p_noa(struct net_device *net, char* buf, int len)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	return wl_cfgp2p_set_p2p_noa(wl, net, buf, len);
}

s32 wl_cfg80211_get_p2p_noa(struct net_device *net, char* buf, int len)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	return wl_cfgp2p_get_p2p_noa(wl, net, buf, len);
}

s32 wl_cfg80211_set_p2p_ps(struct net_device *net, char* buf, int len)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	return wl_cfgp2p_set_p2p_ps(wl, net, buf, len);
}

s32 wl_cfg80211_set_wps_p2p_ie(struct net_device *net, char *buf, int len,
	enum wl_management_type type)
{
	struct wl_priv *wl;
	struct net_device *ndev = NULL;
	s32 ret = 0;
	s32 bssidx = 0;
	s32 pktflag = 0;
	wl = wlcfg_drv_priv;
	if (wl->p2p && wl->p2p->vif_created) {
		ndev = wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION);
		bssidx = wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_CONNECTION);
	} else if (wl_get_drv_status(wl, AP_CREATING) ||
		wl_get_drv_status(wl, AP_CREATED)) {
		ndev = net;
		bssidx = 0;
	}
	if (ndev != NULL) {
		switch (type) {
			case WL_BEACON:
				pktflag = VNDR_IE_BEACON_FLAG;
				break;
			case WL_PROBE_RESP:
				pktflag = VNDR_IE_PRBRSP_FLAG;
				break;
			case WL_ASSOC_RESP:
				pktflag = VNDR_IE_ASSOCRSP_FLAG;
				break;
		}
		if (pktflag)
			ret = wl_cfgp2p_set_management_ie(wl, ndev, bssidx, pktflag, buf, len);
	}

	return ret;
}

static __used void wl_dongle_poweron(struct wl_priv *wl)
{
	WL_DBG(("Enter \n"));
	dhd_customer_gpio_wlan_ctrl(WLAN_RESET_ON);

#if defined(BCMLXSDMMC)
	sdioh_start(NULL, 0);
#endif
#if defined(BCMLXSDMMC)
	sdioh_start(NULL, 1);
#endif
	wl_cfg80211_resume(wl_to_wiphy(wl));
}

static __used void wl_dongle_poweroff(struct wl_priv *wl)
{
	WL_DBG(("Enter \n"));
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
	wl_cfg80211_suspend(wl_to_wiphy(wl), NULL);
#else
	wl_cfg80211_suspend(wl_to_wiphy(wl));
#endif

#if defined(BCMLXSDMMC)
	sdioh_stop(NULL);
#endif
	/* clean up dtim_skip setting */
	dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);
}

static int wl_debugfs_add_netdev_params(struct wl_priv *wl)
{
	char buf[10+IFNAMSIZ];
	struct dentry *fd;
	s32 err = 0;

	WL_TRACE(("In\n"));
	sprintf(buf, "netdev:%s", wl_to_prmry_ndev(wl)->name);
	wl->debugfsdir = debugfs_create_dir(buf, wl_to_wiphy(wl)->debugfsdir);

	fd = debugfs_create_u16("beacon_int", S_IRUGO, wl->debugfsdir,
		(u16 *)&wl->profile->beacon_interval);
	if (!fd) {
		err = -ENOMEM;
		goto err_out;
	}

	fd = debugfs_create_u8("dtim_period", S_IRUGO, wl->debugfsdir,
		(u8 *)&wl->profile->dtim_period);
	if (!fd) {
		err = -ENOMEM;
		goto err_out;
	}

err_out:
	return err;
}

static void wl_debugfs_remove_netdev(struct wl_priv *wl)
{
	WL_DBG(("Enter \n"));
}

static const struct rfkill_ops wl_rfkill_ops = {
	.set_block = wl_rfkill_set
};

static int wl_rfkill_set(void *data, bool blocked)
{
	struct wl_priv *wl = (struct wl_priv *)data;

	WL_DBG(("Enter \n"));
	WL_DBG(("RF %s\n", blocked ? "blocked" : "unblocked"));

	if (!wl)
		return -EINVAL;

	wl->rf_blocked = blocked;

	return 0;
}

static int wl_setup_rfkill(struct wl_priv *wl, bool setup)
{
	s32 err = 0;

	WL_DBG(("Enter \n"));
	if (!wl)
		return -EINVAL;
	if (setup) {
		wl->rfkill = rfkill_alloc("brcmfmac-wifi",
			&wl_cfg80211_get_sdio_func()->dev,
			RFKILL_TYPE_WLAN, &wl_rfkill_ops, (void *)wl);

		if (!wl->rfkill) {
			err = -ENOMEM;
			goto err_out;
		}

		err = rfkill_register(wl->rfkill);

		if (err)
			rfkill_destroy(wl->rfkill);
	} else {
		if (!wl->rfkill) {
			err = -ENOMEM;
			goto err_out;
		}

		rfkill_unregister(wl->rfkill);
		rfkill_destroy(wl->rfkill);
	}

err_out:
	return err;
}

#if defined(COEX_DHCP)
/*
 * get named driver variable to uint register value and return error indication
 * calling example: dev_wlc_intvar_get_reg(dev, "btc_params",66, &reg_value)
 */
static int
dev_wlc_intvar_get_reg(struct net_device *dev, char *name,
	uint reg, int *retval)
{
	union {
		char buf[WLC_IOCTL_SMLEN];
		int val;
	} var;
	int error;

	bcm_mkiovar(name, (char *)(&reg), sizeof(reg),
		(char *)(&var), sizeof(var.buf));
	error = wldev_ioctl(dev, WLC_GET_VAR, (char *)(&var), sizeof(var.buf), false);

	*retval = dtoh32(var.val);
	return (error);
}

static int
dev_wlc_bufvar_set(struct net_device *dev, char *name, char *buf, int len)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	char ioctlbuf[1024];
#else
	static char ioctlbuf[1024];
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31) */

	bcm_mkiovar(name, buf, len, ioctlbuf, sizeof(ioctlbuf));

	return (wldev_ioctl(dev, WLC_SET_VAR, ioctlbuf, sizeof(ioctlbuf), true));
}
/*
get named driver variable to uint register value and return error indication
calling example: dev_wlc_intvar_set_reg(dev, "btc_params",66, value)
*/
static int
dev_wlc_intvar_set_reg(struct net_device *dev, char *name, char *addr, char * val)
{
	char reg_addr[8];

	memset(reg_addr, 0, sizeof(reg_addr));
	memcpy((char *)&reg_addr[0], (char *)addr, 4);
	memcpy((char *)&reg_addr[4], (char *)val, 4);

	return (dev_wlc_bufvar_set(dev, name, (char *)&reg_addr[0], sizeof(reg_addr)));
}

static bool btcoex_is_sco_active(struct net_device *dev)
{
	int ioc_res = 0;
	bool res = FALSE;
	int sco_id_cnt = 0;
	int param27;
	int i;

	for (i = 0; i < 12; i++) {

		ioc_res = dev_wlc_intvar_get_reg(dev, "btc_params", 27, &param27);

		WL_TRACE(("%s, sample[%d], btc params: 27:%x\n",
			__FUNCTION__, i, param27));

		if (ioc_res < 0) {
			WL_ERR(("%s ioc read btc params error\n", __FUNCTION__));
			break;
		}

		if ((param27 & 0x6) == 2) { /* count both sco & esco  */
			sco_id_cnt++;
		}

		if (sco_id_cnt > 2) {
			WL_TRACE(("%s, sco/esco detected, pkt id_cnt:%d  samples:%d\n",
				__FUNCTION__, sco_id_cnt, i));
			res = TRUE;
			break;
		}

		msleep(5);
	}

	return res;
}

#if defined(BT_DHCP_eSCO_FIX)
/* Enhanced BT COEX settings for eSCO compatibility during DHCP window */
static int set_btc_esco_params(struct net_device *dev, bool trump_sco)
{
	static bool saved_status = FALSE;

	char buf_reg50va_dhcp_on[8] =
		{ 50, 00, 00, 00, 0x22, 0x80, 0x00, 0x00 };
	char buf_reg51va_dhcp_on[8] =
		{ 51, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	char buf_reg64va_dhcp_on[8] =
		{ 64, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	char buf_reg65va_dhcp_on[8] =
		{ 65, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	char buf_reg71va_dhcp_on[8] =
		{ 71, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	uint32 regaddr;
	static uint32 saved_reg50;
	static uint32 saved_reg51;
	static uint32 saved_reg64;
	static uint32 saved_reg65;
	static uint32 saved_reg71;

	if (trump_sco) {
		/* this should reduce eSCO agressive retransmit
		 * w/o breaking it
		 */

		/* 1st save current */
		WL_TRACE(("Do new SCO/eSCO coex algo {save &"
			  "override}\n"));
		if ((!dev_wlc_intvar_get_reg(dev, "btc_params", 50, &saved_reg50)) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 51, &saved_reg51)) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 64, &saved_reg64)) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 65, &saved_reg65)) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 71, &saved_reg71))) {
			saved_status = TRUE;
			WL_TRACE(("%s saved bt_params[50,51,64,65,71]:"
				  "0x%x 0x%x 0x%x 0x%x 0x%x\n",
				  __FUNCTION__, saved_reg50, saved_reg51,
				  saved_reg64, saved_reg65, saved_reg71));
		} else {
			WL_ERR((":%s: save btc_params failed\n",
				__FUNCTION__));
			saved_status = FALSE;
			return -1;
		}

		WL_TRACE(("override with [50,51,64,65,71]:"
			  "0x%x 0x%x 0x%x 0x%x 0x%x\n",
			  *(u32 *)(buf_reg50va_dhcp_on+4),
			  *(u32 *)(buf_reg51va_dhcp_on+4),
			  *(u32 *)(buf_reg64va_dhcp_on+4),
			  *(u32 *)(buf_reg65va_dhcp_on+4),
			  *(u32 *)(buf_reg71va_dhcp_on+4)));

		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg50va_dhcp_on[0], 8);
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg51va_dhcp_on[0], 8);
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg64va_dhcp_on[0], 8);
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg65va_dhcp_on[0], 8);
		dev_wlc_bufvar_set(dev, "btc_params",
			(char *)&buf_reg71va_dhcp_on[0], 8);

		saved_status = TRUE;
	} else if (saved_status) {
		/* restore previously saved bt params */
		WL_TRACE(("Do new SCO/eSCO coex algo {save &"
			  "override}\n"));

		regaddr = 50;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&saved_reg50);
		regaddr = 51;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&saved_reg51);
		regaddr = 64;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&saved_reg64);
		regaddr = 65;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&saved_reg65);
		regaddr = 71;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&saved_reg71);

		WL_TRACE(("restore bt_params[50,51,64,65,71]:"
			"0x%x 0x%x 0x%x 0x%x 0x%x\n",
			saved_reg50, saved_reg51, saved_reg64,
			saved_reg65, saved_reg71));

		saved_status = FALSE;
	} else {
		WL_ERR((":%s att to restore not saved BTCOEX params\n",
			__FUNCTION__));
		return -1;
	}
	return 0;
}
#endif /* BT_DHCP_eSCO_FIX */

static void
wl_cfg80211_bt_setflag(struct net_device *dev, bool set)
{
#if defined(BT_DHCP_USE_FLAGS)
	char buf_flag7_dhcp_on[8] = { 7, 00, 00, 00, 0x1, 0x0, 0x00, 0x00 };
	char buf_flag7_default[8]   = { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};
#endif

#if defined(BT_DHCP_eSCO_FIX)
	/* set = 1, save & turn on  0 - off & restore prev settings */
	set_btc_esco_params(dev, set);
#endif

#if defined(BT_DHCP_USE_FLAGS)
	WL_TRACE(("WI-FI priority boost via bt flags, set:%d\n", set));
	if (set == TRUE)
		/* Forcing bt_flag7  */
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_dhcp_on[0],
			sizeof(buf_flag7_dhcp_on));
	else
		/* Restoring default bt flag7 */
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_default[0],
			sizeof(buf_flag7_default));
#endif
}

static void wl_cfg80211_bt_timerfunc(ulong data)
{
	struct btcoex_info *bt_local = (struct btcoex_info *)data;
	WL_TRACE(("%s\n", __FUNCTION__));
	bt_local->timer_on = 0;
	schedule_work(&bt_local->work);
}

static void wl_cfg80211_bt_handler(struct work_struct *work)
{
	struct btcoex_info *btcx_inf;

	btcx_inf = container_of(work, struct btcoex_info, work);

	if (btcx_inf->timer_on) {
		btcx_inf->timer_on = 0;
		del_timer_sync(&btcx_inf->timer);
	}

	switch (btcx_inf->bt_state) {
		case BT_DHCP_START:
			/* DHCP started
			 * provide OPPORTUNITY window to get DHCP address
			 */
			WL_TRACE(("%s bt_dhcp stm: started \n",
				__FUNCTION__));
			btcx_inf->bt_state = BT_DHCP_OPPR_WIN;
			mod_timer(&btcx_inf->timer,
				jiffies + BT_DHCP_OPPR_WIN_TIME*HZ/1000);
			btcx_inf->timer_on = 1;
			break;

		case BT_DHCP_OPPR_WIN:
			if (btcx_inf->dhcp_done) {
				WL_TRACE(("%s DHCP Done before T1 expiration\n",
					__FUNCTION__));
				goto btc_coex_idle;
			}

			/* DHCP is not over yet, start lowering BT priority
			 * enforce btc_params + flags if necessary
			 */
			WL_TRACE(("%s DHCP T1:%d expired\n", __FUNCTION__,
				BT_DHCP_OPPR_WIN_TIME));
			if (btcx_inf->dev)
				wl_cfg80211_bt_setflag(btcx_inf->dev, TRUE);
			btcx_inf->bt_state = BT_DHCP_FLAG_FORCE_TIMEOUT;
			mod_timer(&btcx_inf->timer,
				jiffies + BT_DHCP_FLAG_FORCE_TIME*HZ/1000);
			btcx_inf->timer_on = 1;
			break;

		case BT_DHCP_FLAG_FORCE_TIMEOUT:
			if (btcx_inf->dhcp_done) {
				WL_TRACE(("%s DHCP Done before T2 expiration\n",
					__FUNCTION__));
			} else {
				/* Noo dhcp during T1+T2, restore BT priority */
				WL_TRACE(("%s DHCP wait interval T2:%d"
					  "msec expired\n", __FUNCTION__,
					  BT_DHCP_FLAG_FORCE_TIME));
			}

			/* Restoring default bt priority */
			if (btcx_inf->dev)
				wl_cfg80211_bt_setflag(btcx_inf->dev, FALSE);
btc_coex_idle:
			btcx_inf->bt_state = BT_DHCP_IDLE;
			btcx_inf->timer_on = 0;
			break;

		default:
			WL_ERR(("%s error g_status=%d !!!\n", __FUNCTION__,
				btcx_inf->bt_state));
			if (btcx_inf->dev)
				wl_cfg80211_bt_setflag(btcx_inf->dev, FALSE);
			btcx_inf->bt_state = BT_DHCP_IDLE;
			btcx_inf->timer_on = 0;
			break;
	}

	net_os_wake_unlock(btcx_inf->dev);
}

static int wl_cfg80211_btcoex_init(struct wl_priv *wl)
{
	struct btcoex_info *btco_inf = NULL;

	btco_inf = kmalloc(sizeof(struct btcoex_info), GFP_KERNEL);
	if (!btco_inf)
		return -ENOMEM;

	btco_inf->bt_state = BT_DHCP_IDLE;
	btco_inf->ts_dhcp_start = 0;
	btco_inf->ts_dhcp_ok = 0;
	/* Set up timer for BT  */
	btco_inf->timer_ms = 10;
	init_timer(&btco_inf->timer);
	btco_inf->timer.data = (ulong)btco_inf;
	btco_inf->timer.function = wl_cfg80211_bt_timerfunc;

	btco_inf->dev = wl->wdev->netdev;

	INIT_WORK(&btco_inf->work, wl_cfg80211_bt_handler);

	wl->btcoex_info = btco_inf;
	return 0;
}

static void
wl_cfg80211_btcoex_deinit(struct wl_priv *wl)
{
	if (!wl->btcoex_info)
		return;

	if (!wl->btcoex_info->timer_on) {
		wl->btcoex_info->timer_on = 0;
		del_timer_sync(&wl->btcoex_info->timer);
	}

	cancel_work_sync(&wl->btcoex_info->work);

	kfree(wl->btcoex_info);
	wl->btcoex_info = NULL;
}
#endif		/* COEX_DHCP */

int wl_cfg80211_set_btcoex_dhcp(struct net_device *dev, char *command)
{
	char powermode_val = 0;
	char buf_reg66va_dhcp_on[8] = { 66, 00, 00, 00, 0x10, 0x27, 0x00, 0x00 };
	char buf_reg41va_dhcp_on[8] = { 41, 00, 00, 00, 0x33, 0x00, 0x00, 0x00 };
	char buf_reg68va_dhcp_on[8] = { 68, 00, 00, 00, 0x90, 0x01, 0x00, 0x00 };

	uint32 regaddr;
	static uint32 saved_reg66;
	static uint32 saved_reg41;
	static uint32 saved_reg68;
	static bool saved_status = FALSE;

#ifdef COEX_DHCP
	char buf_flag7_default[8] =   { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};
	struct btcoex_info *btco_inf = wlcfg_drv_priv->btcoex_info;
#endif /* COEX_DHCP */

	/* Figure out powermode 1 or o command */
	strncpy((char *)&powermode_val, command + strlen("BTCOEXMODE") +1, 1);

	if (strnicmp((char *)&powermode_val, "1", strlen("1")) == 0) {

		WL_TRACE(("%s: DHCP session starts\n", __FUNCTION__));

		/* Retrieve and saved orig regs value */
		if ((saved_status == FALSE) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 66,  &saved_reg66)) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 41,  &saved_reg41)) &&
			(!dev_wlc_intvar_get_reg(dev, "btc_params", 68,  &saved_reg68)))   {
				saved_status = TRUE;
				WL_TRACE(("Saved 0x%x 0x%x 0x%x\n",
					saved_reg66, saved_reg41, saved_reg68));

				/* Disable PM mode during dhpc session */

				/* Disable PM mode during dhpc session */
#ifdef COEX_DHCP
				/* Start  BT timer only for SCO connection */
				if (btcoex_is_sco_active(dev)) {
					/* btc_params 66 */
					dev_wlc_bufvar_set(dev, "btc_params",
						(char *)&buf_reg66va_dhcp_on[0],
						sizeof(buf_reg66va_dhcp_on));
					/* btc_params 41 0x33 */
					dev_wlc_bufvar_set(dev, "btc_params",
						(char *)&buf_reg41va_dhcp_on[0],
						sizeof(buf_reg41va_dhcp_on));
					/* btc_params 68 0x190 */
					dev_wlc_bufvar_set(dev, "btc_params",
						(char *)&buf_reg68va_dhcp_on[0],
						sizeof(buf_reg68va_dhcp_on));
					saved_status = TRUE;

					btco_inf->bt_state = BT_DHCP_START;
					btco_inf->timer_on = 1;
					mod_timer(&btco_inf->timer, btco_inf->timer.expires);
					WL_TRACE(("%s enable BT DHCP Timer\n",
					__FUNCTION__));
				}
#endif /* COEX_DHCP */
		}
		else if (saved_status == TRUE) {
			WL_ERR(("%s was called w/o DHCP OFF. Continue\n", __FUNCTION__));
		}
	}
	else if (strnicmp((char *)&powermode_val, "2", strlen("2")) == 0) {


		/* Restoring PM mode */

#ifdef COEX_DHCP
		/* Stop any bt timer because DHCP session is done */
		WL_TRACE(("%s disable BT DHCP Timer\n", __FUNCTION__));
		if (btco_inf->timer_on) {
			btco_inf->timer_on = 0;
			del_timer_sync(&btco_inf->timer);

			if (btco_inf->bt_state != BT_DHCP_IDLE) {
			/* need to restore original btc flags & extra btc params */
				WL_TRACE(("%s bt->bt_state:%d\n",
					__FUNCTION__, btco_inf->bt_state));
				/* wake up btcoex thread to restore btlags+params  */
				schedule_work(&btco_inf->work);
			}
		}

		/* Restoring btc_flag paramter anyway */
		if (saved_status == TRUE)
			dev_wlc_bufvar_set(dev, "btc_flags",
				(char *)&buf_flag7_default[0], sizeof(buf_flag7_default));
#endif /* COEX_DHCP */

		/* Restore original values */
		if (saved_status == TRUE) {
			regaddr = 66;
			dev_wlc_intvar_set_reg(dev, "btc_params",
				(char *)&regaddr, (char *)&saved_reg66);
			regaddr = 41;
			dev_wlc_intvar_set_reg(dev, "btc_params",
				(char *)&regaddr, (char *)&saved_reg41);
			regaddr = 68;
			dev_wlc_intvar_set_reg(dev, "btc_params",
				(char *)&regaddr, (char *)&saved_reg68);

			WL_TRACE(("restore regs {66,41,68} <- 0x%x 0x%x 0x%x\n",
				saved_reg66, saved_reg41, saved_reg68));
		}
		saved_status = FALSE;

	}
	else {
		WL_ERR(("%s Unkwown yet power setting, ignored\n",
			__FUNCTION__));
	}

	snprintf(command, 3, "OK");

	return (strlen("OK"));
}
