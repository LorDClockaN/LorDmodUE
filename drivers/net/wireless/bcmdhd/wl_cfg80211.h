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
 * $Id: wl_cfg80211.h,v 1.1.4.1.2.8 2011/02/09 01:37:52 Exp $
 */

#ifndef _wl_cfg80211_h_
#define _wl_cfg80211_h_

#include <linux/wireless.h>
#include <typedefs.h>
#include <proto/ethernet.h>
#include <wlioctl.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <linux/rfkill.h>

#include <wl_cfgp2p.h>

struct wl_conf;
struct wl_iface;
struct wl_priv;
struct wl_security;
struct wl_ibss;


#define htod32(i) i
#define htod16(i) i
#define dtoh32(i) i
#define dtoh16(i) i
#define htodchanspec(i) i
#define dtohchanspec(i) i

#define WL_DBG_NONE	0
#define WL_DBG_TRACE	(1 << 4)
#define WL_DBG_SCAN 	(1 << 3)
#define WL_DBG_DBG 	(1 << 2)
#define WL_DBG_INFO	(1 << 1)
#define WL_DBG_ERR	(1 << 0)

/* 0 invalidates all debug messages.  default is 1 */
#define WL_DBG_LEVEL 0xFF

#define	WL_ERR(args)									\
do {										\
	if (wl_dbg_level & WL_DBG_ERR) {				\
			printk(KERN_ERR "CFG80211-ERROR) %s : ", __func__);	\
			printk args;						\
		} 								\
} while (0)
#define	WL_INFO(args)									\
do {										\
	if (wl_dbg_level & WL_DBG_INFO) {				\
			printk(KERN_ERR "CFG80211-INFO) %s : ", __func__);	\
			printk args;						\
		}								\
} while (0)
#define	WL_SCAN(args)								\
do {									\
	if (wl_dbg_level & WL_DBG_SCAN) {			\
		printk(KERN_ERR "CFG80211-SCAN) %s :", __func__);	\
		printk args;							\
	}									\
} while (0)
#define	WL_TRACE(args)								\
do {									\
	if (wl_dbg_level & WL_DBG_TRACE) {			\
		printk(KERN_ERR "CFG80211-TRACE) %s :", __func__);	\
		printk args;							\
	}									\
} while (0)
#if (WL_DBG_LEVEL > 0)
#define	WL_DBG(args)								\
do {									\
	if (wl_dbg_level & WL_DBG_DBG) {			\
		printk(KERN_ERR "CFG80211-DEBUG) %s :", __func__);	\
		printk args;							\
	}									\
} while (0)
#else				/* !(WL_DBG_LEVEL > 0) */
#define	WL_DBG(args)
#endif				/* (WL_DBG_LEVEL > 0) */


#define WL_SCAN_RETRY_MAX	3	/* used for ibss scan */
#define WL_NUM_PMKIDS_MAX	MAXPMKID	/* will be used
						 * for 2.6.33 kernel
						 * or later
						 */
#define WL_SCAN_BUF_MAX 		(1024 * 8)
#define WL_TLV_INFO_MAX 		1024
#define WL_SCAN_IE_LEN_MAX      2048
#define WL_BSS_INFO_MAX			2048
#define WL_ASSOC_INFO_MAX	512	/*
				 * needs to grab assoc info from dongle to
				 * report it to cfg80211 through "connect"
				 * event
				 */
#define WL_IOCTL_LEN_MAX	1024
#define WL_EXTRA_BUF_MAX	2048
#define WL_ISCAN_BUF_MAX	2048	/*
				 * the buf lengh can be WLC_IOCTL_MAXLEN (8K)
				 * to reduce iteration
				 */
#define WL_ISCAN_TIMER_INTERVAL_MS	3000
#define WL_SCAN_ERSULTS_LAST 	(WL_SCAN_RESULTS_NO_MEM+1)
#define WL_AP_MAX	256	/* virtually unlimitted as long
				 * as kernel memory allows
				 */
#define WL_FILE_NAME_MAX	256
#define WL_DWELL_TIME		200
#define WL_LONG_DWELL_TIME	1000
#define VWDEV_CNT 3

#define WL_SCAN_TIMER_INTERVAL_MS	8000 /* Scan timeout */

/* dongle status */
enum wl_status {
	WL_STATUS_READY = 0,
	WL_STATUS_SCANNING,
	WL_STATUS_SCAN_ABORTING,
	WL_STATUS_CONNECTING,
	WL_STATUS_CONNECTED,
	WL_STATUS_DISCONNECTING,
	WL_STATUS_AP_CREATING,
	WL_STATUS_AP_CREATED
};

/* wi-fi mode */
enum wl_mode {
	WL_MODE_BSS,
	WL_MODE_IBSS,
	WL_MODE_AP
};

/* dongle profile list */
enum wl_prof_list {
	WL_PROF_MODE,
	WL_PROF_SSID,
	WL_PROF_SEC,
	WL_PROF_IBSS,
	WL_PROF_BAND,
	WL_PROF_BSSID,
	WL_PROF_ACT,
	WL_PROF_BEACONINT,
	WL_PROF_DTIMPERIOD
};

/* dongle iscan state */
enum wl_iscan_state {
	WL_ISCAN_STATE_IDLE,
	WL_ISCAN_STATE_SCANING
};

/* donlge escan state */
enum wl_escan_state {
    WL_ESCAN_STATE_IDLE,
    WL_ESCAN_STATE_SCANING
};
/* fw downloading status */
enum wl_fw_status {
	WL_FW_LOADING_DONE,
	WL_NVRAM_LOADING_DONE
};

enum wl_management_type {
	WL_BEACON = 0x1,
	WL_PROBE_RESP = 0x2,
	WL_ASSOC_RESP = 0x4
};
/* beacon / probe_response */
struct beacon_proberesp {
	__le64 timestamp;
	__le16 beacon_int;
	__le16 capab_info;
	u8 variable[0];
} __attribute__ ((packed));

/* dongle configuration */
struct wl_conf {
	struct net_mode {
		struct net_device *ndev;
		s32 type;
	} mode [VWDEV_CNT + 1];		/* adhoc , infrastructure or ap */
	u32 frag_threshold;
	u32 rts_threshold;
	u32 retry_short;
	u32 retry_long;
	s32 tx_power;
	struct ieee80211_channel channel;
};

typedef s32(*EVENT_HANDLER) (struct wl_priv *wl,
                            struct net_device *ndev, const wl_event_msg_t *e, void *data);

/* bss inform structure for cfg80211 interface */
struct wl_cfg80211_bss_info {
	u16 band;
	u16 channel;
	s16 rssi;
	u16 frame_len;
	u8 frame_buf[1];
};

/* basic structure of scan request */
struct wl_scan_req {
	struct wlc_ssid ssid;
};

/* basic structure of information element */
struct wl_ie {
	u16 offset;
	u8 buf[WL_TLV_INFO_MAX];
};

/* event queue for cfg80211 main event */
struct wl_event_q {
	struct list_head eq_list;
	u32 etype;
	wl_event_msg_t emsg;
	s8 edata[1];
};

/* security information with currently associated ap */
struct wl_security {
	u32 wpa_versions;
	u32 auth_type;
	u32 cipher_pairwise;
	u32 cipher_group;
	u32 wpa_auth;
};

/* ibss information for currently joined ibss network */
struct wl_ibss {
	u8 beacon_interval;	/* in millisecond */
	u8 atim;		/* in millisecond */
	s8 join_only;
	u8 band;
	u8 channel;
};

/* dongle profile */
struct wl_profile {
	u32 mode;
	struct wlc_ssid ssid;
	u8 bssid[ETHER_ADDR_LEN];
	u16 beacon_interval;
	u8 dtim_period;
	struct wl_security sec;
	struct wl_ibss ibss;
	s32 band;
	bool active;
};

typedef s32(*ISCAN_HANDLER) (struct wl_priv *wl);

/* dongle iscan controller */
struct wl_iscan_ctrl {
	struct net_device *dev;
	struct timer_list timer;
	u32 timer_ms;
	u32 timer_on;
	s32 state;
	struct task_struct *tsk;
	struct semaphore sync;
	ISCAN_HANDLER iscan_handler[WL_SCAN_ERSULTS_LAST];
	void *data;
	s8 ioctl_buf[WLC_IOCTL_SMLEN];
	s8 scan_buf[WL_ISCAN_BUF_MAX];
};

/* association inform */
#define MAX_REQ_LINE 1024
struct wl_connect_info {
	u8 req_ie[MAX_REQ_LINE];
	s32 req_ie_len;
	u8 resp_ie[MAX_REQ_LINE];
	s32 resp_ie_len;
};

/* firmware /nvram downloading controller */
struct wl_fw_ctrl {
	const struct firmware *fw_entry;
	unsigned long status;
	u32 ptr;
	s8 fw_name[WL_FILE_NAME_MAX];
	s8 nvram_name[WL_FILE_NAME_MAX];
};

/* assoc ie length */
struct wl_assoc_ielen {
	u32 req_len;
	u32 resp_len;
};

/* wpa2 pmk list */
struct wl_pmk_list {
	pmkid_list_t pmkids;
	pmkid_t foo[MAXPMKID - 1];
};


#define ESCAN_BUF_SIZE (64 * 1024)

struct escan_info {
    u32 escan_state;
    u8 escan_buf[ESCAN_BUF_SIZE];
    struct wiphy *wiphy;
};

struct ap_info {
/* Structure to hold WPS, WPA IEs for a AP */
	u8   probe_res_ie[IE_MAX_LEN];
	u8   beacon_ie[IE_MAX_LEN];
	u32 probe_res_ie_len;
	u32 beacon_ie_len;
	u8 *wpa_ie;
	u8 *rsn_ie;
	u8 *wps_ie;
	bool security_mode;
};
struct btcoex_info {
	struct timer_list timer;
	uint32 timer_ms;
	uint32 timer_on;
	uint32 ts_dhcp_start;	/* ms ts ecord time stats */
	uint32 ts_dhcp_ok;	/* ms ts ecord time stats */
	bool dhcp_done;		/* flag, indicates that host done with
				 * dhcp before t1/t2 expiration
				 */
	int bt_state;
	struct work_struct work;
	struct net_device *dev;
};

struct sta_info {
	/* Structure to hold WPS IE for a STA */
	u8  probe_req_ie[IE_MAX_LEN];
	u8  assoc_req_ie[IE_MAX_LEN];
	u32 probe_req_ie_len;
	u32 assoc_req_ie_len;
};
/* dongle private data of cfg80211 interface */
struct wl_priv {
	struct wireless_dev *wdev;	/* representing wl cfg80211 device */
	struct wireless_dev *vwdev[VWDEV_CNT];
	struct wl_conf *conf;	/* dongle configuration */
	struct cfg80211_scan_request *scan_request;	/* scan request object */
	EVENT_HANDLER evt_handler[WLC_E_LAST];
	struct list_head eq_list;	/* used for event queue */
	spinlock_t eq_lock;	/* for event queue synchronization */
	struct mutex usr_sync;	/* maily for dongle up/down synchronization */
	struct wl_scan_results *bss_list;
	struct wl_scan_results *scan_results;

	/* scan request object for internal purpose */
	struct wl_scan_req *scan_req_int;

	/* bss information for cfg80211 layer */
	struct wl_cfg80211_bss_info *bss_info;
	/* information element object for internal purpose */
	struct wl_ie ie;

	/* for synchronization of main event thread */
	struct wl_profile *profile;	/* holding dongle profile */
	struct wl_iscan_ctrl *iscan;	/* iscan controller */

	/* association information container */
	struct wl_connect_info conn_info;

	/* control firwmare and nvram paramter downloading */
	struct wl_fw_ctrl *fw;
	struct wl_pmk_list *pmk_list;	/* wpa2 pmk list */
	tsk_ctl_t event_tsk;  		/* task of main event handler thread */
	unsigned long status;		/* current dongle status */
	void *pub;
	u32 channel;		/* current channel */
	bool iscan_on;		/* iscan on/off switch */
	bool iscan_kickstart;	/* indicate iscan already started */
	bool escan_on;      /* escan on/off switch */
	struct escan_info escan_info;   /* escan information */
	bool active_scan;	/* current scan mode */
	bool ibss_starter;	/* indicates this sta is ibss starter */
	bool link_up;		/* link/connection up flag */

	/* indicate whether dongle to support power save mode */
	bool pwr_save;
	bool dongle_up;		/* indicate whether dongle up or not */
	bool roam_on;		/* on/off switch for dongle self-roaming */
	bool scan_tried;	/* indicates if first scan attempted */
	u8 *ioctl_buf;	/* ioctl buffer */
	u8 *escan_ioctl_buf;
	u8 *extra_buf;	/* maily to grab assoc information */
	struct dentry *debugfsdir;
	struct rfkill *rfkill;
	bool rf_blocked;
	struct ieee80211_channel remain_on_chan;
	enum nl80211_channel_type remain_on_chan_type;
	u64 cache_cookie;
	wait_queue_head_t dongle_event_wait;
	struct ap_info *ap_info;
	struct sta_info *sta_info;
	struct p2p_info *p2p;
	bool p2p_supported;
	struct btcoex_info *btcoex_info;
	struct timer_list scan_timeout;   /* Timer for catch scan event timeout */
};

#define wl_to_wiphy(w) (w->wdev->wiphy)
#define wl_to_prmry_ndev(w) (w->wdev->netdev)
#define ndev_to_wl(n) (wdev_to_wl(n->ieee80211_ptr))
#define wl_to_sr(w) (w->scan_req_int)
#define wl_to_ie(w) (&w->ie)
#define iscan_to_wl(i) ((struct wl_priv *)(i->data))
#define wl_to_iscan(w) (w->iscan)
#define wl_to_conn(w) (&w->conn_info)
#define wiphy_from_scan(w) (w->escan_info.wiphy)
#define wl_get_drv_status(wl, stat)   (test_bit(WL_STATUS_ ## stat, &(wl)->status))
#define wl_set_drv_status(wl, stat)   (set_bit(WL_STATUS_ ## stat, &(wl)->status))
#define wl_clr_drv_status(wl, stat)   (clear_bit(WL_STATUS_ ## stat, &(wl)->status))
#define wl_chg_drv_status(wl, stat)   (change_bit(WL_STATUS_ ## stat, &(wl)->status))

static inline struct wl_bss_info *next_bss(struct wl_scan_results *list, struct wl_bss_info *bss)
{
	return bss = bss ?
		(struct wl_bss_info *)((uintptr) bss + dtoh32(bss->length)) : list->bss_info;
}
static inline s32 alloc_idx_vwdev(struct wl_priv *wl)
{
	s32 i = 0;
	for (i = 0; i < VWDEV_CNT; i++) {
		if (wl->vwdev[i] == NULL)
				return i;
	}
	return -1;
}

static inline s32 get_idx_vwdev_by_netdev(struct wl_priv *wl, struct net_device *ndev)
{
	s32 i = 0;
	for (i = 0; i < VWDEV_CNT; i++) {
		if ((wl->vwdev[i] != NULL) && (wl->vwdev[i]->netdev == ndev))
				return i;
	}
	return -1;
}

static inline s32 get_mode_by_netdev(struct wl_priv *wl, struct net_device *ndev)
{
	s32 i = 0;
	for (i = 0; i <= VWDEV_CNT; i++) {
		if (wl->conf->mode[i].ndev != NULL && (wl->conf->mode[i].ndev == ndev))
			return wl->conf->mode[i].type;
	}
	return -1;
}
static inline void set_mode_by_netdev(struct wl_priv *wl, struct net_device *ndev, s32 type)
{
	s32 i = 0;
	for (i = 0; i <= VWDEV_CNT; i++) {
		if (type == -1) {
			/* free the info of netdev */
			if (wl->conf->mode[i].ndev == ndev) {
				wl->conf->mode[i].ndev = NULL;
				wl->conf->mode[i].type = -1;
				break;
			}

		} else {
			if ((wl->conf->mode[i].ndev != NULL)&&
			(wl->conf->mode[i].ndev == ndev)) {
				/* update type of ndev */
				wl->conf->mode[i].type = type;
				break;
			}
			else if ((wl->conf->mode[i].ndev == NULL)&&
			(wl->conf->mode[i].type == -1)) {
				wl->conf->mode[i].ndev = ndev;
				wl->conf->mode[i].type = type;
				break;
			}
		}
	}
}
#define free_vwdev_by_index(wl, __i) do {      \
						if (wl->vwdev[__i] != NULL) \
							kfree(wl->vwdev[__i]); \
						wl->vwdev[__i] = NULL; \
					} while (0)

#define for_each_bss(list, bss, __i)	\
	for (__i = 0; __i < list->count && __i < WL_AP_MAX; __i++, bss = next_bss(list, bss))

/* In case of WPS from wpa_supplicant, pairwise siute and group suite is 0.
 * In addtion to that, wpa_version is WPA_VERSION_1
 */
#define is_wps_conn(_sme) \
	((_sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) && \
	 (!_sme->crypto.n_ciphers_pairwise) && \
	 (!_sme->crypto.cipher_group))
extern s32 wl_cfg80211_attach(struct net_device *ndev, void *data);
extern s32 wl_cfg80211_attach_post(struct net_device *ndev);
extern void wl_cfg80211_detach(void);
/* event handler from dongle */
extern void wl_cfg80211_event(struct net_device *ndev, const wl_event_msg_t *e,
            void *data);
extern void wl_cfg80211_set_sdio_func(void *func);	/* set sdio function info */
extern struct sdio_func *wl_cfg80211_get_sdio_func(void);	/* set sdio function info */
extern s32 wl_cfg80211_up(void);	/* dongle up */
extern s32 wl_cfg80211_down(void);	/* dongle down */
extern s32 wl_cfg80211_notify_ifadd(struct net_device *net, s32 idx, s32 bssidx,
int (*_net_attach)(dhd_pub_t *dhdp, int ifidx));
extern s32 wl_cfg80211_notify_ifdel(struct net_device *ndev);
extern s32 wl_cfg80211_is_progress_ifadd(void);
extern s32 wl_cfg80211_is_progress_ifchange(void);
extern s32 wl_cfg80211_is_progress_ifadd(void);
extern s32 wl_cfg80211_notify_ifchange(void);
extern void wl_cfg80211_dbg_level(u32 level);
extern void *wl_cfg80211_request_fw(s8 *file_name);
extern s32 wl_cfg80211_read_fw(s8 *buf, u32 size);
extern void wl_cfg80211_release_fw(void);
extern s8 *wl_cfg80211_get_fwname(void);
extern s8 *wl_cfg80211_get_nvramname(void);
extern s32 wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr);
extern s32 wl_cfg80211_set_p2p_noa(struct net_device *net, char* buf, int len);
extern s32 wl_cfg80211_get_p2p_noa(struct net_device *net, char* buf, int len);
extern s32 wl_cfg80211_set_wps_p2p_ie(struct net_device *net, char *buf, int len,
	enum wl_management_type type);
extern s32 wl_cfg80211_set_p2p_ps(struct net_device *net, char* buf, int len);
extern int wl_cfg80211_hang(struct net_device *dev, u16 reason);

/* do scan abort */
extern s32
wl_cfg80211_scan_abort(struct wl_priv *wl, struct net_device *ndev);

extern s32
wl_cfg80211_if_is_group_owner(void);
#endif				/* _wl_cfg80211_h_ */
