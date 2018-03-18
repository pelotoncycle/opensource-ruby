/****************************************************************************
 * Ralink Tech Inc.
 * Taiwan, R.O.C.
 *
 * (c) Copyright 2009, Ralink Technology, Inc.
 *
 * All rights reserved. Ralink's source code is an unpublished work and the
 * use of a copyright notice does not imply otherwise. This source code
 * contains confidential trade secret material of Ralink Tech. Any attemp
 * or participation in deciphering, decoding, reverse engineering or in any
 * way altering the source code is stricitly prohibited, unless the prior
 * written consent of Ralink Technology, Inc. is obtained.
 ***************************************************************************/

/****************************************************************************

	Abstract:

	All related CFG80211 function body.

	History:
		1. 2009/09/17	Sample Lin
			(1) Init version.
		2. 2009/10/27	Sample Lin
			(1) Do not use ieee80211_register_hw() to create virtual interface.
				Use wiphy_register() to register nl80211 command handlers.
			(2) Support iw utility.
		3. 2009/11/03	Sample Lin
			(1) Change name MAC80211 to CFG80211.
			(2) Modify CFG80211_OpsChannelSet().
			(3) Move CFG80211_Register()/CFG80211_UnRegister() to open/close.
		4. 2009/12/16	Sample Lin
			(1) Patch for Linux 2.6.32.
			(2) Add more supported functions in CFG80211_Ops.
		5. 2010/12/10	Sample Lin
			(1) Modify for OS_ABL.
		6. 2011/04/19	Sample Lin
			(1) Add more supported functions in CFG80211_Ops v33 ~ 38.

	Note:
		The feature is supported only in "LINUX" 2.6.28 ~ 2.6.38.

***************************************************************************/


#define RTMP_MODULE_OS

#include "rtmp_comm.h"
#include "rt_os_util.h"
#include "rt_os_net.h"
#include "rt_config.h"

extern int RtmpIoctl_rt_ioctl_siwgenie(RTMP_ADAPTER *pAd, const u8 *ie, size_t ie_len);

#ifdef RT_CFG80211_SUPPORT

/* 36 ~ 64, 100 ~ 136, 140 ~ 161 */
#define CFG80211_NUM_OF_CHAN_5GHZ	(sizeof(Cfg80211_Chan)-CFG80211_NUM_OF_CHAN_2GHZ)

/* all available channels */
static const UCHAR Cfg80211_Chan[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,

	/* 802.11 UNI / HyperLan 2 */
	36, 38, 40, 44, 46, 48, 52, 54, 56, 60, 62, 64,

	/* 802.11 HyperLan 2 */
	100, 104, 108, 112, 116, 118, 120, 124, 126, 128, 132, 134, 136,

	/* 802.11 UNII */
	140, 149, 151, 153, 157, 159, 161, 165, 167, 169, 171, 173,

	/* Japan */
	184, 188, 192, 196, 208, 212, 216,
};

static const UINT32 CipherSuites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
#ifdef DOT11W_PMF_SUPPORT
	WLAN_CIPHER_SUITE_AES_CMAC,
#endif /* DOT11W_PMF_SUPPORT */
};

/*
	The driver's regulatory notification callback.
*/
static void CFG80211_RegNotifier(
					 IN struct wiphy *pWiphy,
					 IN struct regulatory_request *pRequest);

/* get RALINK pAd control block in 80211 Ops */
#define MAC80211_PAD_GET(__pAd, __pWiphy)							\
	{																\
		ULONG *__pPriv;												\
		__pPriv = (ULONG *)(wiphy_priv(__pWiphy));					\
		__pAd = (VOID *)(*__pPriv);									\
		if (__pAd == NULL)											\
		{															\
			DBGPRINT(RT_DEBUG_ERROR,								\
					("80211> %s but pAd = NULL!", __func__));	\
		}															\
	}

static int CFG80211_OpsMonitorChannelSet(struct wiphy *pWiphy, struct cfg80211_chan_def *chandef)
{
	VOID *pAd;
	CFG80211_CB *p80211CB;
	CMD_RTPRIV_IOCTL_80211_CHAN ChanInfo;
	UINT32 ChanId;

	struct device *pDev = pWiphy->dev.parent;
	struct net_device *pNetDev = dev_get_drvdata(pDev);
	struct ieee80211_channel *pChan;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	/* return 0; */
	MAC80211_PAD_GET(pAd, pWiphy);
	pChan = chandef->chan;

	printk("control:%d MHz width:%d center: %d/%d MHz",
	       pChan->center_freq, chandef->width, chandef->center_freq1, chandef->center_freq2);

	/* get channel number */
	ChanId = ieee80211_frequency_to_channel(pChan->center_freq);
	CFG80211DBG(RT_DEBUG_TRACE, ("80211> Channel = %d\n", ChanId));
	CFG80211DBG(RT_DEBUG_TRACE, ("80211> ChannelType = %d\n", chandef->width));

	/* init */
	memset(&ChanInfo, 0, sizeof(ChanInfo));
	ChanInfo.ChanId = ChanId;

	p80211CB = NULL;
	RTMP_DRIVER_80211_CB_GET(pAd, &p80211CB);

	if (p80211CB == NULL) {
		CFG80211DBG(RT_DEBUG_ERROR, ("80211> p80211CB == NULL!\n"));
		return 0;
	}

	ChanInfo.IfType = pNetDev->ieee80211_ptr->iftype;

	CFG80211DBG(RT_DEBUG_ERROR, ("80211> ChanInfo.IfType == %d!\n", ChanInfo.IfType));

	if (cfg80211_get_chandef_type(chandef) == NL80211_CHAN_NO_HT)
		ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_NOHT;
	else if (cfg80211_get_chandef_type(chandef) == NL80211_CHAN_HT20)
		ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_HT20;
	else if (cfg80211_get_chandef_type(chandef) == NL80211_CHAN_HT40MINUS)
		ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_HT40MINUS;
	else if (cfg80211_get_chandef_type(chandef) == NL80211_CHAN_HT40PLUS)
		ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_HT40PLUS;

	ChanInfo.MonFilterFlag = p80211CB->MonFilterFlag;

	/* set channel */
	RTMP_DRIVER_80211_CHAN_SET(pAd, &ChanInfo);

	return 0;
}				/* End of CFG80211_OpsChannelSet */

/*
========================================================================
Routine Description:
	Change type/configuration of virtual interface.

Arguments:
	pWiphy			- Wireless hardware description
	IfIndex			- Interface index
	Type			- Interface type, managed/adhoc/ap/station, etc.
	pFlags			- Monitor flags
	pParams			- Mesh parameters

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: set type, set monitor
========================================================================
*/
static int CFG80211_OpsVirtualInfChg(struct wiphy *pWiphy,
				     struct net_device *pNetDevIn,
				     enum nl80211_iftype Type,
				     u32 *pFlags, struct vif_params *pParams)
{
	VOID *pAd;
	CFG80211_CB *pCfg80211_CB;
	struct net_device *pNetDev;
	CMD_RTPRIV_IOCTL_80211_VIF_PARM VifInfo;
	UINT oldType = pNetDevIn->ieee80211_ptr->iftype;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	CFG80211DBG(RT_DEBUG_OFF, ("80211> IfTypeChange %d ==> %d\n", oldType, Type));
	MAC80211_PAD_GET(pAd, pWiphy);

	/* sanity check */
#ifdef CONFIG_STA_SUPPORT
	if ((Type != NL80211_IFTYPE_ADHOC) &&
	    (Type != NL80211_IFTYPE_STATION) &&
	    (Type != NL80211_IFTYPE_MONITOR) && (Type != NL80211_IFTYPE_AP)
	    && (Type != NL80211_IFTYPE_P2P_CLIENT)
	    && (Type != NL80211_IFTYPE_P2P_GO)
	    )
#endif /* CONFIG_STA_SUPPORT */
	{
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Wrong interface type %d!\n", Type));
		return -EINVAL;
	}

	/* End of if */
	/* update interface type */
	pNetDev = pNetDevIn;

	if (pNetDev == NULL)
		return -ENODEV;

	pNetDev->ieee80211_ptr->iftype = Type;

	VifInfo.net_dev = pNetDev;
	VifInfo.newIfType = Type;
	VifInfo.oldIfType = oldType;

	if (pFlags != NULL) {
		VifInfo.MonFilterFlag = 0;

		if (((*pFlags) & NL80211_MNTR_FLAG_FCSFAIL) == NL80211_MNTR_FLAG_FCSFAIL)
			VifInfo.MonFilterFlag |= RT_CMD_80211_FILTER_FCSFAIL;

		if (((*pFlags) & NL80211_MNTR_FLAG_FCSFAIL) == NL80211_MNTR_FLAG_PLCPFAIL)
			VifInfo.MonFilterFlag |= RT_CMD_80211_FILTER_PLCPFAIL;

		if (((*pFlags) & NL80211_MNTR_FLAG_CONTROL) == NL80211_MNTR_FLAG_CONTROL)
			VifInfo.MonFilterFlag |= RT_CMD_80211_FILTER_CONTROL;

		if (((*pFlags) & NL80211_MNTR_FLAG_CONTROL) == NL80211_MNTR_FLAG_OTHER_BSS)
			VifInfo.MonFilterFlag |= RT_CMD_80211_FILTER_OTHER_BSS;
	}

	/* Type transer from linux to driver defined */
	if (Type == NL80211_IFTYPE_STATION) {
		Type = RT_CMD_80211_IFTYPE_STATION;
	} else if (Type == NL80211_IFTYPE_ADHOC) {
		Type = RT_CMD_80211_IFTYPE_ADHOC;
	} else if (Type == NL80211_IFTYPE_MONITOR) {
		Type = RT_CMD_80211_IFTYPE_MONITOR;
	}
#ifdef CONFIG_AP_SUPPORT
	else if (Type == NL80211_IFTYPE_P2P_CLIENT) {
		Type = RT_CMD_80211_IFTYPE_P2P_CLIENT;
	} else if (Type == NL80211_IFTYPE_P2P_GO) {
		Type = RT_CMD_80211_IFTYPE_P2P_GO;
	}
#endif /* CONFIG_AP_SUPPORT */

	RTMP_DRIVER_80211_VIF_CHG(pAd, &VifInfo);

	/*CFG_TODO */
	RTMP_DRIVER_80211_CB_GET(pAd, &pCfg80211_CB);
	pCfg80211_CB->MonFilterFlag = VifInfo.MonFilterFlag;
	return 0;
}

#if defined(SIOCGIWSCAN) || defined(RT_CFG80211_SUPPORT)
extern int rt_ioctl_siwscan(struct net_device *dev,
			    struct iw_request_info *info, union iwreq_data *wreq, char *extra);
#endif /* LINUX_VERSION_CODE: 2.6.30 */

/*
========================================================================
Routine Description:
	Request to do a scan. If returning zero, the scan request is given
	the driver, and will be valid until passed to cfg80211_scan_done().
	For scan results, call cfg80211_inform_bss(); you can call this outside
	the scan/scan_done bracket too.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	pRequest		- Scan request

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: scan

	struct cfg80211_scan_request {
		struct cfg80211_ssid *ssids;
		int n_ssids;
		struct ieee80211_channel **channels;
		u32 n_channels;
		const u8 *ie;
		size_t ie_len;

	 * @ssids: SSIDs to scan for (active scan only)
	 * @n_ssids: number of SSIDs
	 * @channels: channels to scan on.
	 * @n_channels: number of channels for each band
	 * @ie: optional information element(s) to add into Probe Request or %NULL
	 * @ie_len: length of ie in octets
========================================================================
*/
static int CFG80211_OpsScan(struct wiphy *pWiphy,
			    struct cfg80211_scan_request *pRequest)
{
#ifdef CONFIG_STA_SUPPORT
	VOID *pAd;
	CFG80211_CB *pCfg80211_CB;

	struct iw_scan_req IwReq;
	union iwreq_data Wreq;

	struct net_device *pNdev = pRequest->wdev->netdev;
	MAC80211_PAD_GET(pAd, pWiphy);

	CFG80211DBG(RT_DEBUG_TRACE,
		    ("========================================================================\n"));
	CFG80211DBG(RT_DEBUG_TRACE,
		    ("80211> %s ==> %s(%d)\n", __func__, pNdev->name,
		     pNdev->ieee80211_ptr->iftype));

	/* YF_TODO: record the scan_req per netdevice */
	RTMP_DRIVER_80211_CB_GET(pAd, &pCfg80211_CB);
	pCfg80211_CB->pCfg80211_ScanReq = pRequest;	/* used in scan end */

	if (pNdev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) {
		CFG80211OS_ScanEnd(pCfg80211_CB, TRUE);
		return 0;
	}
	/* sanity check */
	if ((pNdev->ieee80211_ptr->iftype != NL80211_IFTYPE_STATION) &&
	    (pNdev->ieee80211_ptr->iftype != NL80211_IFTYPE_AP) &&
	    (pNdev->ieee80211_ptr->iftype != NL80211_IFTYPE_ADHOC)
	    && (pNdev->ieee80211_ptr->iftype != NL80211_IFTYPE_P2P_CLIENT)
	    && (pNdev->ieee80211_ptr->iftype != NL80211_IFTYPE_P2P_GO)
	    ) {
		CFG80211DBG(RT_DEBUG_ERROR,
			    ("80211> DeviceType Not Support Scan ==> %d\n",
			     pNdev->ieee80211_ptr->iftype));
		CFG80211OS_ScanEnd(pCfg80211_CB, TRUE);
		return -EOPNOTSUPP;
	}

	/* Driver Internal SCAN SM Check */
	if (RTMP_DRIVER_IOCTL_SANITY_CHECK(pAd, NULL) != NDIS_STATUS_SUCCESS) {
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Network is down!\n"));
		CFG80211OS_ScanEnd(pCfg80211_CB, TRUE);
		return -ENETDOWN;
	}

	if (RTMP_DRIVER_80211_SCAN(pAd, pNdev->ieee80211_ptr->iftype) != NDIS_STATUS_SUCCESS) {
		CFG80211DBG(RT_DEBUG_ERROR, ("\n\n\n\n\n80211> BUSY - SCANING\n\n\n\n\n"));
		CFG80211OS_ScanEnd(pCfg80211_CB, TRUE);
		return 0;
	}
#ifdef CONFIG_MULTI_CHANNEL
	UCHAR Flag = 0;
	RTMP_DRIVER_ADAPTER_MCC_DHCP_PROTECT_STATUS(pAd, &Flag);
	DBGPRINT(RT_DEBUG_TRACE, ("mcc  Flag %d\n", Flag));
	if (Flag) {
		CFG80211DBG(RT_DEBUG_ERROR, ("MCC Protect DHCP - Aborting Scan\n"));
		return 0;
	}

	RTMP_DRIVER_80211_SET_NOA(pAd, pNdev->name);

#endif /* CONFIG_MULTI_CHANNEL */

	if (pRequest->ie_len != 0) {
		DBGPRINT(RT_DEBUG_TRACE,
			 ("80211> ExtraIEs Not Null in ProbeRequest from upper layer...\n"));
		/* YF@20120321: Using Cfg80211_CB carry on pAd struct to overwirte the pWpsProbeReqIe. */
		RTMP_DRIVER_80211_SCAN_EXTRA_IE_SET(pAd);
	} else {
		DBGPRINT(RT_DEBUG_TRACE,
			 ("80211> ExtraIEs Null in ProbeRequest from upper layer...\n"));
	}

	memset(&Wreq, 0, sizeof(Wreq));
	memset(&IwReq, 0, sizeof(IwReq));

	if (pRequest->n_ssids) {
		DBGPRINT(RT_DEBUG_INFO,
			 ("80211> Num %d of SSID & ssidLen %d from upper layer...\n",
			  pRequest->n_ssids, pRequest->ssids->ssid_len));
	}

	/* %NULL or zero-length SSID is used to indicate wildcard */
	if (pRequest->n_ssids <= 1) {
		if ((pRequest->n_ssids == 0) || (pRequest->ssids == NULL)) {
			DBGPRINT(RT_DEBUG_TRACE, ("80211> Wildcard SSID In ProbeRequest.\n"));
			Wreq.data.flags |= IW_SCAN_ALL_ESSID;
		} else if (pRequest->ssids) {
			if (pRequest->ssids->ssid_len == 0)
				Wreq.data.flags |= IW_SCAN_ALL_ESSID;
			else
				Wreq.data.flags |= IW_SCAN_THIS_ESSID;
		}
	} else {
		DBGPRINT(RT_DEBUG_TRACE,
			 ("80211> Named SSID [%s] In ProbeRequest.\n", pRequest->ssids->ssid));
		Wreq.data.flags |= IW_SCAN_THIS_ESSID;

		/* Fix kernel crash when ssid is null instead of wildcard ssid */
		if (!pRequest->ssids) {
			DBGPRINT(RT_DEBUG_ERROR, ("80211> NULL pRequest->ssids!!!\n"));
			CFG80211OS_ScanEnd(pCfg80211_CB, TRUE);
			return -1;
		}
		/* pRequest->ssids->ssid is 32-b array, never null */
	}

	/* Set Channel List for this Scan Action */
	DBGPRINT(RT_DEBUG_INFO, ("80211> [%d] Channels In ProbeRequest.\n", pRequest->n_channels));
	if (pRequest->n_channels > 0) {
		UINT32 *pChanList;
		UINT idx;
		/* shall use sizeof(UINT32) instead of (UINT32 *)! */
		os_alloc_mem(NULL, (UCHAR **) &pChanList, sizeof(UINT32) * pRequest->n_channels);
		if (pChanList == NULL) {
			DBGPRINT(RT_DEBUG_ERROR, ("%s::Alloc memory fail\n", __func__));
			CFG80211OS_ScanEnd(pCfg80211_CB, TRUE);
			return -1;
		}

		for (idx = 0; idx < pRequest->n_channels; idx++) {
			pChanList[idx] =
			    ieee80211_frequency_to_channel(pRequest->channels[idx]->center_freq);
			CFG80211DBG(RT_DEBUG_INFO, ("%d,", pChanList[idx]));
		}
		CFG80211DBG(RT_DEBUG_INFO, ("\n"));

		RTMP_DRIVER_80211_SCAN_CHANNEL_LIST_SET(pAd, pChanList, pRequest->n_channels);
		os_free_mem(NULL, pChanList);
	}

	/* use 1st SSID in the requested SSID list */
	if (pRequest->n_ssids >= 1 && pRequest->ssids) {
		IwReq.essid_len = pRequest->ssids->ssid_len;
		memcpy(IwReq.essid, pRequest->ssids->ssid, sizeof(IwReq.essid));
	}
	Wreq.data.length = sizeof(struct iw_scan_req);

	/* Scan Type */
	IwReq.scan_type = SCAN_ACTIVE;

#ifdef RT_CFG80211_P2P_SUPPORT
	if ((pNdev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_CLIENT)
	    || (pNdev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_GO)
	    || (pNdev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_DEVICE)
	    ) {
		IwReq.scan_type = SCAN_P2P;
	}

	if (strcmp(pNdev->name, "p2p0") == 0) {
		IwReq.scan_type = SCAN_P2P;
	}
#endif /* RT_CFG80211_P2P_SUPPORT */

	rt_ioctl_siwscan(pNdev, NULL, &Wreq, (char *)&IwReq);
	return 0;

#else
	return -EOPNOTSUPP;
#endif /* CONFIG_STA_SUPPORT */
}

#ifdef CONFIG_STA_SUPPORT
/*
========================================================================
Routine Description:
	Join the specified IBSS (or create if necessary). Once done, call
	cfg80211_ibss_joined(), also call that function when changing BSSID due
	to a merge.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	pParams			- IBSS parameters

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: ibss join

	No fixed-freq and fixed-bssid support.
========================================================================
*/
static int mt76xx_cfg80211_join_ibss(struct wiphy *wiphy,
				     struct net_device *dev,
				     struct cfg80211_ibss_params *ibss_param)
{
	u8 ssid[IEEE80211_MAX_SSID_LEN + 1] = { 0 };
	RTMP_ADAPTER *pAd;
	INT32 success;

	if (!ibss_param || ibss_param->ssid_len > IEEE80211_MAX_SSID_LEN)
		return -EINVAL;

	MAC80211_PAD_GET(pAd, wiphy);
	if (!pAd)
		return -EFAULT;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==> SSID = %s, BI = %d\n",
				     __func__, ibss_param->ssid, ibss_param->beacon_interval));

	pAd->CommonCfg.BeaconPeriod = ibss_param->beacon_interval;
	/* GeK: [todo] safe to use local ssid[] 'cause set_SSID_Proc()
	 * copies it again.
	 */
	memcpy(ssid, ibss_param->ssid, ibss_param->ssid_len);
	ssid[ibss_param->ssid_len] = 0;
	pAd->StaCfg.bAutoReconnect = TRUE;

	/* GeK: [todo] it improperly uses strlen() to get length */
	success = Set_SSID_Proc(pAd, ssid);
	return (success == TRUE) ? 0 : -EIO;
}

/*
========================================================================
Routine Description:
	Leave the IBSS.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: ibss leave
========================================================================
*/
static int CFG80211_OpsIbssLeave(IN struct wiphy *pWiphy, IN struct net_device *pNdev)
{
	VOID *pAd;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);

	/* CFG_TODO */
	RTMP_DRIVER_80211_STA_LEAVE(pAd, pNdev->ieee80211_ptr->iftype);
	return 0;
}
#endif /* CONFIG_STA_SUPPORT */

/*
========================================================================
Routine Description:
	Set the transmit power according to the parameters.

Arguments:
	pWiphy			- Wireless hardware description
	Type			-
	dBm				- dBm

Return Value:
	0				- success
	-x				- fail

Note:
	Type -
	enum nl80211_tx_power_setting - TX power adjustment
	 @NL80211_TX_POWER_AUTOMATIC: automatically determine transmit power
	 @NL80211_TX_POWER_LIMITED: limit TX power by the mBm parameter
	 @NL80211_TX_POWER_FIXED: fix TX power to the mBm parameter
========================================================================
*/
static int CFG80211_OpsTxPwrSet(IN struct wiphy *pWiphy,
				IN struct wireless_dev *wdev,
				IN enum nl80211_tx_power_setting Type,
				IN int dBm)
{

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	return -EOPNOTSUPP;
}

/*
========================================================================
Routine Description:
	Store the current TX power into the dbm variable.

Arguments:
	pWiphy			- Wireless hardware description
	pdBm			- dBm

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int CFG80211_OpsTxPwrGet(IN struct wiphy *pWiphy,
				IN struct wireless_dev *wdev,
				IN int *pdBm)
{
	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	return -EOPNOTSUPP;
}				/* End of CFG80211_OpsTxPwrGet */

/*
========================================================================
Routine Description:
	Power management.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	FlgIsEnabled	-
	Timeout			-

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int CFG80211_OpsPwrMgmt(IN struct wiphy *pWiphy,
			       IN struct net_device *pNdev, IN bool enabled, IN INT32 timeout)
{
	VOID *pAd;
	CFG80211DBG(RT_DEBUG_TRACE,
		    ("80211> %s ==> (%s) power save %s\n", __func__, pNdev->name,
		     (enabled ? "enable" : "disable")));

	MAC80211_PAD_GET(pAd, pWiphy);
	if (strncmp(pNdev->name, "p2p", 3) == 0)
		RTMP_DRIVER_80211_POWER_MGMT_SET(pAd, enabled);
	return 0;
}				/* End of CFG80211_OpsPwrMgmt */

/*
========================================================================
Routine Description:
	Get information for a specific station.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	pMac			- STA MAC
	pSinfo			- STA INFO

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int mt76xx_cfg80211_get_sta(struct wiphy *wiphy, struct net_device *dev,
				   u8 *mac,
				   struct station_info *sinfo)
{
	RTMP_ADAPTER *pAd;
	CMD_IOCTL_80211_STA_INFO sta_info;

	if (!mac)
		return -EINVAL;

	MAC80211_PAD_GET(pAd, wiphy);
	if (!pAd)
		return -EFAULT;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));

	/* init */
	memset(sinfo, 0, sizeof(*sinfo));
	memset(&sta_info, 0, sizeof(sta_info));

	/* get sta information */
	if (CFG80211DRV_StaGet(pAd, mac, &sta_info) != TRUE)
		return -ENOENT;

	if (sta_info.TxRateFlags != RT_CMD_80211_TXRATE_LEGACY) {
		sinfo->txrate.flags = RATE_INFO_FLAGS_MCS;
		if (sta_info.TxRateFlags & RT_CMD_80211_TXRATE_BW_40)
			sinfo->txrate.flags |= RATE_INFO_FLAGS_40_MHZ_WIDTH;

		if (sta_info.TxRateFlags & RT_CMD_80211_TXRATE_SHORT_GI)
			sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;

		sinfo->txrate.mcs = sta_info.TxRateMCS;
	} else
		sinfo->txrate.legacy = sta_info.TxRateMCS;

	sinfo->filled |= STATION_INFO_TX_BITRATE;

	/* fill signal */
	sinfo->signal = sta_info.Signal;
	sinfo->filled |= STATION_INFO_SIGNAL;

#ifdef CONFIG_AP_SUPPORT
	/* fill tx count */
	sinfo->tx_packets = sta_info.TxPacketCnt;
	sinfo->filled |= STATION_INFO_TX_PACKETS;

	/* fill inactive time */
	sinfo->inactive_time = sta_info.InactiveTime;
	sinfo->filled |= STATION_INFO_INACTIVE_TIME;
#endif /* CONFIG_AP_SUPPORT */

#ifdef CONFIG_STA_SUPPORT
	/* fill tx/rx count */
	sinfo->tx_packets = sta_info.tx_packets;
	sinfo->filled |= STATION_INFO_TX_PACKETS;

	sinfo->tx_retries = sta_info.tx_retries;
	sinfo->filled |= STATION_INFO_TX_RETRIES;

	sinfo->tx_failed = sta_info.tx_failed;
	sinfo->filled |= STATION_INFO_TX_FAILED;

	sinfo->rx_packets = sta_info.rx_packets;
	sinfo->filled |= STATION_INFO_RX_PACKETS;

	/* fill inactive time */
	sinfo->inactive_time = sta_info.InactiveTime;
	sinfo->filled |= STATION_INFO_INACTIVE_TIME;
#endif /* CONFIG_STA_SUPPORT */

	return 0;
}

/*
========================================================================
Routine Description:
	List all stations known, e.g. the AP on managed interfaces.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	Idx				-
	pMac			-
	pSinfo			-

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int CFG80211_OpsStaDump(IN struct wiphy *pWiphy,
			       IN struct net_device *pNdev,
			       IN int Idx, IN UINT8 * pMac, IN struct station_info *pSinfo)
{
	VOID *pAd;

	if (Idx != 0)
		return -ENOENT;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);

#ifdef CONFIG_STA_SUPPORT
	if (RTMP_DRIVER_AP_SSID_GET(pAd, pMac) != NDIS_STATUS_SUCCESS)
		return -EBUSY;
	else
		return mt76xx_cfg80211_get_sta(pWiphy, pNdev, pMac, pSinfo);
#endif /* CONFIG_STA_SUPPORT */

	return -EOPNOTSUPP;
}				/* End of CFG80211_OpsStaDump */

/*
========================================================================
Routine Description:
	Notify that wiphy parameters have changed.

Arguments:
	pWiphy			- Wireless hardware description
	Changed			-

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int CFG80211_OpsWiphyParamsSet(IN struct wiphy *pWiphy, IN UINT32 Changed)
{
	VOID *pAd;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);
	if (Changed & WIPHY_PARAM_RTS_THRESHOLD) {
		RTMP_DRIVER_80211_RTS_THRESHOLD_ADD(pAd, &pWiphy->rts_threshold);
		CFG80211DBG(RT_DEBUG_TRACE,
			    ("80211> %s ==> rts_threshold(%d)\n", __func__, pWiphy->rts_threshold));
		return 0;
	} else if (Changed & WIPHY_PARAM_FRAG_THRESHOLD) {
		RTMP_DRIVER_80211_FRAG_THRESHOLD_ADD(pAd, &pWiphy->frag_threshold);
		CFG80211DBG(RT_DEBUG_TRACE,
			    ("80211> %s ==> frag_threshold(%d)\n", __func__,
			     pWiphy->frag_threshold));
		return 0;
	}

	return -EOPNOTSUPP;
}				/* End of CFG80211_OpsWiphyParamsSet */

/*
========================================================================
Routine Description:
	Add a key with the given parameters.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	KeyIdx			-
	Pairwise		-
	pMacAddr		-
	pParams			-

Return Value:
	0				- success
	-x				- fail

Note:
	pMacAddr will be NULL when adding a group key.
========================================================================
*/
static int CFG80211_OpsKeyAdd(struct wiphy *pWiphy, struct net_device *pNdev, UINT8 KeyIdx,
			      bool Pairwise,
			      const UINT8 *pMacAddr, struct key_params *pParams)
{
	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_KEY KeyInfo;
	CFG80211_CB *p80211CB;
	p80211CB = NULL;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);

	/* pAd sould not be null unless pWiphy is incorrect */
	if (pAd == NULL)
		return -EINVAL;

#ifdef RT_CFG80211_DEBUG
	hex_dump("KeyBuf=", (UINT8 *) pParams->key, pParams->key_len);
#endif /* RT_CFG80211_DEBUG */

	CFG80211DBG(RT_DEBUG_OFF,
		    ("80211> KeyIdx = %d, pParams->cipher = %x\n", KeyIdx, pParams->cipher));

	if (pParams->key_len >= sizeof(KeyInfo.KeyBuf))
		return -EINVAL;
	/* End of if */

	/* init */
	memset(&KeyInfo, 0, sizeof(KeyInfo));
	memcpy(KeyInfo.KeyBuf, pParams->key, pParams->key_len);
	KeyInfo.KeyBuf[pParams->key_len] = 0x00;
	KeyInfo.KeyId = KeyIdx;
	KeyInfo.bPairwise = Pairwise;
	KeyInfo.KeyLen = pParams->key_len;

	if ((pParams->cipher == WLAN_CIPHER_SUITE_WEP40)) {
		KeyInfo.KeyType = RT_CMD_80211_KEY_WEP40;
	} else if ((pParams->cipher == WLAN_CIPHER_SUITE_WEP104)) {
		KeyInfo.KeyType = RT_CMD_80211_KEY_WEP104;
	} else if ((pParams->cipher == WLAN_CIPHER_SUITE_TKIP) ||
		   (pParams->cipher == WLAN_CIPHER_SUITE_CCMP)) {
		KeyInfo.KeyType = RT_CMD_80211_KEY_WPA;
		if (pParams->cipher == WLAN_CIPHER_SUITE_TKIP)
			KeyInfo.cipher = Ndis802_11TKIPEnable;
		else if (pParams->cipher == WLAN_CIPHER_SUITE_CCMP)
			KeyInfo.cipher = Ndis802_11AESEnable;
	}
#ifdef DOT11W_PMF_SUPPORT
/* PMF IGTK */
	else if (pParams->cipher == WLAN_CIPHER_SUITE_AES_CMAC) {
		KeyInfo.KeyType = RT_CMD_80211_KEY_AES_CMAC;
		KeyInfo.KeyId = KeyIdx;
		KeyInfo.bPairwise = FALSE;
		KeyInfo.KeyLen = pParams->key_len;
	}
#endif /* DOT11W_PMF_SUPPORT */
	else
		return -ENOTSUPP;

	/* add key */
	RTMP_DRIVER_80211_CB_GET(pAd, &p80211CB);

#ifdef CONFIG_AP_SUPPORT
	if ((pNdev->ieee80211_ptr->iftype == RT_CMD_80211_IFTYPE_AP) ||
	    (pNdev->ieee80211_ptr->iftype == RT_CMD_80211_IFTYPE_P2P_GO)) {
		if (pMacAddr) {
			CFG80211DBG(RT_DEBUG_OFF,
				    ("80211> KeyAdd STA(%02X:%02X:%02X:%02X:%02X:%02X) ==>\n",
				     PRINT_MAC(pMacAddr)));
			NdisCopyMemory(KeyInfo.MAC, pMacAddr, MAC_ADDR_LEN);
		}
		CFG80211DBG(RT_DEBUG_OFF, ("80211> AP Key Add\n"));
		RTMP_DRIVER_80211_AP_KEY_ADD(pAd, &KeyInfo);
	} else
#endif /* CONFIG_AP_SUPPORT */

#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
	if (pNdev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_CLIENT) {
		CFG80211DBG(RT_DEBUG_OFF, ("80211> APCLI Key Add\n"));
		RTMP_DRIVER_80211_P2P_CLIENT_KEY_ADD(pAd, &KeyInfo);
	} else
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */
	{
#ifdef CONFIG_STA_SUPPORT
		CFG80211DBG(RT_DEBUG_OFF, ("80211> STA Key Add\n"));
		RTMP_DRIVER_80211_STA_KEY_ADD(pAd, &KeyInfo);
#endif /* endif */
	}

#ifdef RT_P2P_SPECIFIC_WIRELESS_EVENT
	if (pMacAddr) {
		CFG80211DBG(RT_DEBUG_TRACE,
			    ("80211> P2pSendWirelessEvent(%02X:%02X:%02X:%02X:%02X:%02X) ==>\n",
			     PRINT_MAC(pMacAddr)));
		RTMP_DRIVER_80211_SEND_WIRELESS_EVENT(pAd, pMacAddr);
	}
#endif /* RT_P2P_SPECIFIC_WIRELESS_EVENT */

	return 0;

}

/*
========================================================================
Routine Description:
	Get information about the key with the given parameters.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	KeyIdx			-
	Pairwise		-
	pMacAddr		-
	pCookie			-
	pCallback		-

Return Value:
	0				- success
	-x				- fail

Note:
	pMacAddr will be NULL when requesting information for a group key.

	All pointers given to the pCallback function need not be valid after
	it returns.

	This function should return an error if it is not possible to
	retrieve the key, -ENOENT if it doesn't exist.
========================================================================
*/
static int CFG80211_OpsKeyGet(struct wiphy *pWiphy, struct net_device *pNdev, UINT8 KeyIdx,
			      bool Pairwise,
			      const UINT8 *pMacAddr, void *pCookie,
			      void (*pCallback) (void *, struct key_params *))
{

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	return -ENOTSUPP;
}

/*
========================================================================
Routine Description:
	Remove a key given the pMacAddr (NULL for a group key) and KeyIdx.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	KeyIdx			-
	pMacAddr		-

Return Value:
	0				- success
	-x				- fail

Note:
	return -ENOENT if the key doesn't exist.
========================================================================
*/
static int CFG80211_OpsKeyDel(struct wiphy *pWiphy, struct net_device *pNdev, UINT8 KeyIdx,
			      bool Pairwise,
			      const UINT8 *pMacAddr)
{
	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_KEY KeyInfo;
	CFG80211_CB *p80211CB;
	p80211CB = NULL;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	if (pMacAddr) {
		CFG80211DBG(RT_DEBUG_OFF, ("80211> KeyDel STA(%02X:%02X:%02X:%02X:%02X:%02X) ==>\n",
					   PRINT_MAC(pMacAddr)));
		NdisCopyMemory(KeyInfo.MAC, pMacAddr, MAC_ADDR_LEN);
	}

	MAC80211_PAD_GET(pAd, pWiphy);
	RTMP_DRIVER_80211_CB_GET(pAd, &p80211CB);

	memset(&KeyInfo, 0, sizeof(KeyInfo));
	KeyInfo.KeyId = KeyIdx;
	CFG80211DBG(RT_DEBUG_OFF, ("80211> KeyDel isPairwise %d\n", Pairwise));
	KeyInfo.bPairwise = Pairwise;

#ifdef CONFIG_AP_SUPPORT
	if ((pNdev->ieee80211_ptr->iftype == RT_CMD_80211_IFTYPE_AP) ||
	    (pNdev->ieee80211_ptr->iftype == RT_CMD_80211_IFTYPE_P2P_GO)) {
		CFG80211DBG(RT_DEBUG_TRACE, ("80211> AP Key Del\n"));
		RTMP_DRIVER_80211_AP_KEY_DEL(pAd, &KeyInfo);
	} else
#endif /* CONFIG_AP_SUPPORT */
	{
		CFG80211DBG(RT_DEBUG_TRACE, ("80211> STA Key Del\n"));

		if (pMacAddr) {
			CFG80211DBG(RT_DEBUG_OFF, ("80211> STA Key Del -- DISCONNECT\n"));
			RTMP_DRIVER_80211_STA_LEAVE(pAd, pNdev->ieee80211_ptr->iftype);
		}
	}

	return 0;
}

/*
========================================================================
Routine Description:
	Set the default key on an interface.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	KeyIdx			-

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int mt76xx_cfg80211_set_default_key_impl(struct wiphy *wiphy,
						struct net_device *dev, u8 key_index)
{
	RTMP_ADAPTER *pAd;
	MAC80211_PAD_GET(pAd, wiphy);
	if (!pAd)
		return -EFAULT;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==> Default KeyIdx %d\n", __func__, key_index));

#ifdef CONFIG_AP_SUPPORT
	if ((dev->ieee80211_ptr->iftype == RT_CMD_80211_IFTYPE_AP) ||
	    (dev->ieee80211_ptr->iftype == RT_CMD_80211_IFTYPE_P2P_GO))
		RTMP_DRIVER_80211_AP_KEY_DEFAULT_SET(pAd, key_index);
	else
#endif /* CONFIG_AP_SUPPORT */
		RTMP_DRIVER_80211_STA_KEY_DEFAULT_SET(pAd, key_index);

	return 0;
}

static int mt76xx_cfg80211_set_default_key(struct wiphy *wiphy,
					   struct net_device *dev,
					   u8 key_index, bool unicast, bool multicast)
{
	return mt76xx_cfg80211_set_default_key_impl(wiphy, dev, key_index);
}

/*
========================================================================
Routine Description:
	Set the Mgmt default key on an interface.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			-
	KeyIdx			-

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
#ifdef DOT11W_PMF_SUPPORT
static int CFG80211_OpsMgmtKeyDefaultSet(IN struct wiphy *pWiphy,
					 IN struct net_device *pNdev, IN UINT8 KeyIdx)
{
	VOID *pAd;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> Mgmt Default KeyIdx = %d\n", KeyIdx));

	RTMP_DRIVER_80211_STA_MGMT_KEY_DEFAULT_SET(pAd, KeyIdx);

	return 0;
}				/* End of CFG80211_OpsMgmtKeyDefaultSet */
#endif /* DOT11W_PMF_SUPPORT */

/*
========================================================================
Routine Description:
	Connect to the ESS with the specified parameters. When connected,
	call cfg80211_connect_result() with status code %WLAN_STATUS_SUCCESS.
	If the connection fails for some reason, call cfg80211_connect_result()
	with the status from the AP.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	pSme			-

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: connect

	You must use "iw ra0 connect xxx", then "iw ra0 disconnect";
	You can not use "iw ra0 connect xxx" twice without disconnect;
	Or you will suffer "command failed: Operation already in progress (-114)".

	You must support add_key and set_default_key function;
	Or kernel will crash without any error message in linux 2.6.32.

   struct cfg80211_connect_params - Connection parameters

   This structure provides information needed to complete IEEE 802.11
   authentication and association.

   @channel: The channel to use or %NULL if not specified (auto-select based
	on scan results)
   @bssid: The AP BSSID or %NULL if not specified (auto-select based on scan
	results)
   @ssid: SSID
   @ssid_len: Length of ssid in octets
   @auth_type: Authentication type (algorithm)

   @ie: IEs for association request
   @ie_len: Length of assoc_ie in octets

   @privacy: indicates whether privacy-enabled APs should be used
   @crypto: crypto settings
   @key_len: length of WEP key for shared key authentication
   @key_idx: index of WEP key for shared key authentication
   @key: WEP key for shared key authentication
========================================================================
*/
static int mt76xx_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
				   struct cfg80211_connect_params *sme)
{
#ifdef CONFIG_STA_SUPPORT
	RTMP_ADAPTER *pAd;
	INT32 Pairwise;
	INT32 Groupwise;
	INT32 Keymgmt = 0;
	int i;
	CMD_RTPRIV_IOCTL_80211_CONNECT ConnInfo;

	MAC80211_PAD_GET(pAd, wiphy);
	if (!pAd)
		return -EFAULT;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==> %s(%d)\n", __func__,
				dev->name, dev->ieee80211_ptr->iftype));

	/* GeK: [todo] neither sme->channel nor sme->channel_hint is used? */

	CFG80211DBG(RT_DEBUG_OFF, ("Groupwise: %x\n", sme->crypto.cipher_group));
	Groupwise = sme->crypto.cipher_group;

	if (sme->crypto.n_ciphers_pairwise)
		Pairwise = sme->crypto.ciphers_pairwise[0];
	else
		Pairwise = 0;
	CFG80211DBG(RT_DEBUG_OFF, ("Pairwise %x\n", sme->crypto.ciphers_pairwise[0]));

	for (i = 0; i < sme->crypto.n_akm_suites; ++i)
		Keymgmt |= sme->crypto.akm_suites[i];

	memset(&ConnInfo, 0, sizeof(ConnInfo));

	if (!sme->crypto.wpa_versions)
		ConnInfo.WpaVer = 0;
	else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)
		ConnInfo.WpaVer = 2;
	else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1)
		ConnInfo.WpaVer = 1;
	else {
		CFG80211DBG(RT_DEBUG_ERROR, ("version %x not supported\n",
					     sme->crypto.wpa_versions));
		return -ENOTSUPP;
	}
	CFG80211DBG(RT_DEBUG_OFF, ("wpa_versions %x\n", sme->crypto.wpa_versions));

	/* GeK: [todo] WLAN_AKM_SUITE_PSK and others? */
	CFG80211DBG(RT_DEBUG_OFF, ("Keymgmt %x\n", Keymgmt));
	if (Keymgmt == WLAN_AKM_SUITE_8021X)
		ConnInfo.FlgIs8021x = TRUE;
	else
		ConnInfo.FlgIs8021x = FALSE;

	CFG80211DBG(RT_DEBUG_OFF, ("Auth_type %x\n", sme->auth_type));
	if (sme->auth_type == NL80211_AUTHTYPE_SHARED_KEY)
		ConnInfo.AuthType = Ndis802_11AuthModeShared;
	else if (sme->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM)
		ConnInfo.AuthType = Ndis802_11AuthModeOpen;
	else
		ConnInfo.AuthType = Ndis802_11AuthModeAutoSwitch;

	switch (Pairwise) {
	case 0:
		CFG80211DBG(RT_DEBUG_OFF, ("NONE...\n"));
		ConnInfo.PairwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_NONE;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		CFG80211DBG(RT_DEBUG_OFF, ("WLAN_CIPHER_SUITE_CCMP...\n"));
		ConnInfo.PairwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_CCMP;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		CFG80211DBG(RT_DEBUG_OFF, ("WLAN_CIPHER_SUITE_TKIP...\n"));
		ConnInfo.PairwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_TKIP;
		break;
	case WLAN_CIPHER_SUITE_WEP40:
		/* fall through */
	case WLAN_CIPHER_SUITE_WEP104:
		CFG80211DBG(RT_DEBUG_OFF, ("WLAN_CIPHER_SUITE_WEP...\n"));
		ConnInfo.PairwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_WEP;
		break;
	default:
		CFG80211DBG(RT_DEBUG_ERROR, ("pairwise %x not supported\n", Pairwise));
		return -ENOTSUPP;
	}

	if (Groupwise == WLAN_CIPHER_SUITE_CCMP)
		ConnInfo.GroupwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_CCMP;
	else if (Groupwise == WLAN_CIPHER_SUITE_TKIP)
		ConnInfo.GroupwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_TKIP;
	else
		ConnInfo.GroupwiseEncrypType |= RT_CMD_80211_CONN_ENCRYPT_NONE;

	CFG80211DBG(RT_DEBUG_OFF, ("ConnInfo.KeyLen ===> %d\n", sme->key_len));
	CFG80211DBG(RT_DEBUG_OFF, ("ConnInfo.KeyIdx ===> %d\n", sme->key_idx));

	ConnInfo.pKey = (UINT8 *) (sme->key);
	ConnInfo.KeyLen = sme->key_len;
	ConnInfo.pSsid = sme->ssid;
	ConnInfo.SsidLen = sme->ssid_len;
	ConnInfo.KeyIdx = sme->key_idx;
	/* YF@20120328: Reset to default */
	ConnInfo.bWpsConnection = FALSE;
	/* Check if WPS is triggerred */
	if (sme->ie && sme->ie_len &&
	    sme->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM &&
	    ConnInfo.PairwiseEncrypType == RT_CMD_80211_CONN_ENCRYPT_NONE) {
		if (RTMPFindWPSIE(sme->ie, (UINT32) sme->ie_len) != NULL) {
			ConnInfo.bWpsConnection = TRUE;
			pAd->StaCfg.wpa_supplicant_info.WpaSupplicantUP |= WPA_SUPPLICANT_ENABLE_WPS;
		}
	}

	/* YF@20120328: Use SIOCSIWGENIE to make out the WPA/WPS IEs in AssocReq. */
#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
	if (dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_CLIENT) {
		if (sme->ie_len > 0)
			CFG80211DRV_SetP2pCliAssocIe(pAd, sme->ie, sme->ie_len);
		else
			CFG80211DRV_SetP2pCliAssocIe(pAd, NULL, 0);
	} else
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */
	{
		if (sme->ie_len > 0)
			RtmpIoctl_rt_ioctl_siwgenie(pAd, sme->ie, sme->ie_len);
		else
			RtmpIoctl_rt_ioctl_siwgenie(pAd, NULL, 0);
	}

#ifdef DOT11W_PMF_SUPPORT
	CFG80211DBG(RT_DEBUG_OFF, ("80211> PMF Connect %d\n", sme->mfp));
	if (sme->mfp)
		ConnInfo.mfp = TRUE;
	else
		ConnInfo.mfp = FALSE;
#endif /* DOT11W_PMF_SUPPORT */

	/* %NULL if not specified (auto-select based on scan) */
	if (sme->bssid != NULL && !MAC_ADDR_EQUAL(sme->bssid, ZERO_MAC_ADDR)) {
		CFG80211DBG(RT_DEBUG_OFF, ("80211> Connect bssid %02x:%02x:%02x:%02x:%02x:%02x\n",
					   PRINT_MAC(sme->bssid)));
		ConnInfo.pBssid = sme->bssid;
	} else
		ConnInfo.pBssid = NULL;

	if (sme->channel) {
		PCFG80211_CTRL cfg80211_ctrl = &pAd->cfg80211_ctrl;
		cfg80211_ctrl->Cfg80211ConnectChanIndex = ieee80211_frequency_to_channel(sme->channel->center_freq);
		CFG80211DBG(RT_DEBUG_OFF, ("80211> Force to connect channel %d\n,",
					     cfg80211_ctrl->Cfg80211ConnectChanIndex));
	}

	RTMP_DRIVER_80211_CONNECT(pAd, &ConnInfo, dev->ieee80211_ptr->iftype);
#endif /*CONFIG_STA_SUPPORT */
	return 0;
}				/* mt76xx_cfg80211_connect */

/*
========================================================================
Routine Description:
	Disconnect from the BSS/ESS.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	ReasonCode		-

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: connect
========================================================================
*/
static int CFG80211_OpsDisconnect(IN struct wiphy *pWiphy,
				  IN struct net_device *pNdev, IN u16 ReasonCode)
{
#ifdef CONFIG_STA_SUPPORT

	VOID *pAd;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==> ReasonCode: %d, iftype: %d\n",
			__func__, ReasonCode, pNdev->ieee80211_ptr->iftype));

	MAC80211_PAD_GET(pAd, pWiphy);

	RTMP_DRIVER_80211_STA_LEAVE(pAd, pNdev->ieee80211_ptr->iftype);
#endif /*CONFIG_STA_SUPPORT */
	return 0;
}

#ifdef RFKILL_HW_SUPPORT
static int CFG80211_OpsRFKill(IN struct wiphy *pWiphy)
{
	VOID *pAd;
	BOOLEAN active;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);

	RTMP_DRIVER_80211_RFKILL(pAd, &active);
	wiphy_rfkill_set_hw_state(pWiphy, !active);
	return active;
}

VOID CFG80211_RFKillStatusUpdate(IN PVOID pAd, IN BOOLEAN active)
{
	struct wiphy *pWiphy;
	CFG80211_CB *pCfg80211_CB;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));
	RTMP_DRIVER_80211_CB_GET(pAd, &pCfg80211_CB);
	pWiphy = pCfg80211_CB->pCfg80211_Wdev->wiphy;
	wiphy_rfkill_set_hw_state(pWiphy, !active);
	return;
}
#endif /* RFKILL_HW_SUPPORT */

/*
========================================================================
Routine Description:
	Get site survey information.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	Idx				-
	pSurvey			-

Return Value:
	0				- success
	-x				- fail

Note:
	For iw utility: survey dump
========================================================================
*/
static int CFG80211_OpsSurveyGet(IN struct wiphy *pWiphy,
				 IN struct net_device *pNdev,
				 IN int Idx, IN struct survey_info *pSurvey)
{
	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_SURVEY SurveyInfo;

	if (Idx != 0)
		return -ENOENT;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));

	MAC80211_PAD_GET(pAd, pWiphy);

	/* get information from driver */
	RTMP_DRIVER_80211_SURVEY_GET(pAd, &SurveyInfo);

	/* return the information to upper layer */
	pSurvey->channel = ((CFG80211_CB *) (SurveyInfo.pCfg80211))->pCfg80211_Channels;
	pSurvey->filled = SURVEY_INFO_CHANNEL_TIME_BUSY | SURVEY_INFO_CHANNEL_TIME_EXT_BUSY;
	pSurvey->channel_time_busy = SurveyInfo.ChannelTimeBusy;	/* unit: us */
	pSurvey->channel_time_ext_busy = SurveyInfo.ChannelTimeExtBusy;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> busy time = %ld %ld\n",
				     (ULONG) SurveyInfo.ChannelTimeBusy,
				     (ULONG) SurveyInfo.ChannelTimeExtBusy));
	return 0;
}				/* End of CFG80211_OpsSurveyGet */

int mt76xx_set_pmk(RTMP_ADAPTER *pAd, const u8 *bssid, const u8 *pmkid)
{
	unsigned int idx;
	unsigned int last;
	STA_ADMIN_CONFIG *cfg = &pAd->StaCfg;

	BUILD_BUG_ON(ARRAY_SIZE(cfg->SavedPMK[0].BSSID) != MAC_ADDR_LEN);
	BUILD_BUG_ON(ARRAY_SIZE(cfg->SavedPMK[0].PMKID) != WLAN_PMKID_LEN);
	BUILD_BUG_ON_NOT_POWER_OF_2(PMKID_NO);

	last = min_t(size_t, cfg->SavedPMKNum, ARRAY_SIZE(cfg->SavedPMK));
	/* find the bssid */
	for (idx = 0; idx < last; ++idx)
		if (!memcmp(bssid, cfg->SavedPMK[idx].BSSID, MAC_ADDR_LEN))
			break;

	if (idx >= PMKID_NO) {
		DBGPRINT(RT_DEBUG_WARN, ("%s PMKID full\n", __func__));
		return -EINVAL;
	}

	/* found or (!found && space available) */
	DBGPRINT(RT_DEBUG_OFF, ("Update PMKID idx %d\n", idx));
	memcpy(cfg->SavedPMK[idx].BSSID, bssid, MAC_ADDR_LEN);
	memcpy(cfg->SavedPMK[idx].PMKID, pmkid, WLAN_PMKID_LEN);
	/* Not found, updated the last one */
	if (idx == cfg->SavedPMKNum)
		cfg->SavedPMKNum++;
	/* GeK: [todo][bug] original code might have problems:
	 * 1. not increasing SavedPMKNum?
	 * 2. random-add but sequential-search & remove?
	 */

	DBGPRINT(RT_DEBUG_TRACE, ("%s - IW_PMKSA_ADD\n", __func__));
	return 0;
}

/*
========================================================================
Routine Description:
	Cache a PMKID for a BSSID.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	pPmksa			- PMKID information

Return Value:
	0				- success
	-x				- fail

Note:
	This is mostly useful for fullmac devices running firmwares capable of
	generating the (re) association RSN IE.
	It allows for faster roaming between WPA2 BSSIDs.
========================================================================
*/
static int mt76xx_cfg80211_set_pmksa(struct wiphy *wiphy,
				     struct net_device *dev, struct cfg80211_pmksa *pmksa)
{
#ifdef CONFIG_STA_SUPPORT
	RTMP_ADAPTER *pAd;

	if (!pmksa || !pmksa->bssid || !pmksa->pmkid)
		return -EINVAL;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, wiphy);

	if (!pAd)
		return -EINVAL;

	/* RtmpIoctl_rt_ioctl_siwpmksa */
	return mt76xx_set_pmk(pAd, pmksa->bssid, pmksa->pmkid);
#else
	return -ENOTSUPP;
#endif /* CONFIG_STA_SUPPORT */
}				/* mt76xx_cfg80211_set_pmksa */

int mt76xx_del_pmk(RTMP_ADAPTER *pAd, const u8 *bssid)
{
	unsigned int idx;
	unsigned int last;
	STA_ADMIN_CONFIG *cfg = &pAd->StaCfg;

	if (!bssid) {
		memset(&cfg->SavedPMK, 0, sizeof(cfg->SavedPMK));
		cfg->SavedPMKNum = 0;
		DBGPRINT(RT_DEBUG_TRACE, ("%s IW_PMKSA_FLUSH\n", __func__));
		return 0;
	}

	BUILD_BUG_ON(ARRAY_SIZE(cfg->SavedPMK[0].BSSID) != MAC_ADDR_LEN);
	BUILD_BUG_ON(ARRAY_SIZE(cfg->SavedPMK[0].PMKID) != WLAN_PMKID_LEN);
	BUILD_BUG_ON_NOT_POWER_OF_2(PMKID_NO);

	last = min_t(size_t, cfg->SavedPMKNum, ARRAY_SIZE(cfg->SavedPMK));
	/* find the bssid */
	for (idx = 0; idx < last; ++idx)
		if (!memcmp(bssid, cfg->SavedPMK[idx].BSSID, MAC_ADDR_LEN))
			break;

	if (!cfg->SavedPMKNum || idx >= PMKID_NO) {
		DBGPRINT(RT_DEBUG_WARN, ("%s PMKID bssid not found\n", __func__));
		return -EINVAL;
	}

	/* idx is now the one to be removed */
	memset(&cfg->SavedPMK[idx], 0, sizeof(cfg->SavedPMK[idx]));

	--last;
	for (; idx < last; idx++)
		memcpy(&cfg->SavedPMK[idx], &cfg->SavedPMK[idx + 1], sizeof(cfg->SavedPMK[idx]));
	--cfg->SavedPMKNum;
	DBGPRINT(RT_DEBUG_TRACE, ("%s IW_PMKSA_REMOVE\n", __func__));
	return 0;
}				/* mt76xx_del_pmk */

/*
========================================================================
Routine Description:
	Delete a cached PMKID.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface
	pPmksa			- PMKID information

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int mt76xx_cfg80211_del_pmksa(struct wiphy *wiphy,
				     struct net_device *dev, struct cfg80211_pmksa *pmksa)
{
#ifdef CONFIG_STA_SUPPORT
	RTMP_ADAPTER *pAd;

	if (!pmksa || !pmksa->bssid)
		return -EINVAL;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, wiphy);

	if (!pAd)
		return -EINVAL;

	/*RtmpIoctl_rt_ioctl_siwpmksa() */
	return mt76xx_del_pmk(pAd, pmksa->bssid);
#else
	return -ENOTSUPP;
#endif /* CONFIG_STA_SUPPORT */
}				/* mt76xx_cfg80211_del_pmksa */

/*
========================================================================
Routine Description:
	Flush all cached PMKIDs.

Arguments:
	pWiphy			- Wireless hardware description
	pNdev			- Network device interface

Return Value:
	0				- success
	-x				- fail

Note:
========================================================================
*/
static int mt76xx_cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *dev)
{
#ifdef CONFIG_STA_SUPPORT
	RTMP_ADAPTER *pAd;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, wiphy);

	if (!pAd)
		return -EINVAL;

	/* RtmpIoctl_rt_ioctl_siwpmksa */
	return mt76xx_del_pmk(pAd, NULL);
#else
	return -ENOTSUPP;
#endif /* CONFIG_STA_SUPPORT */
}				/* mt76xx_cfg80211_flush_pmksa */

#ifdef RT_CFG80211_P2P_SUPPORT
static int CFG80211_OpsRemainOnChannel(IN struct wiphy *pWiphy,
				       IN struct wireless_dev *pWdev,
				       IN struct ieee80211_channel *pChan,
				       IN unsigned int duration, OUT u64 *cookie)
{
	VOID *pAd;
	UINT32 ChanId;
	CMD_RTPRIV_IOCTL_80211_CHAN ChanInfo;
	u32 rndCookie;

	struct net_device *dev = NULL;
	INT ChannelType = RT_CMD_80211_CHANTYPE_HT20;
	dev = pWdev->netdev;

	/* rndCookie = ((RandomByte2(pAd) * 256 * 256* 256) + (RandomByte2(pAd) * 256 * 256) + (RandomByte2(pAd) * 256) + RandomByte2(pAd)) |1; */
	/* rndCookie = random32() | 1; */
	rndCookie = 1234;

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));

	MAC80211_PAD_GET(pAd, pWiphy);

	/*CFG_TODO: Shall check channel type */

	/* get channel number */
	ChanId = ieee80211_frequency_to_channel(pChan->center_freq);
	CFG80211DBG(RT_DEBUG_TRACE,
		    ("%s: CH = %d, Type = %d, duration = %d, cookie=%d\n", __func__, ChanId,
		     ChannelType, duration, rndCookie));

	/* init */
	*cookie = 1234;
	memset(&ChanInfo, 0, sizeof(ChanInfo));
	ChanInfo.ChanId = ChanId;
	ChanInfo.IfType = dev->ieee80211_ptr->iftype;
	ChanInfo.ChanType = ChannelType;
	ChanInfo.chan = pChan;
	ChanInfo.cookie = *cookie;
	ChanInfo.pWdev = pWdev;

	/* set channel */
	RTMP_DRIVER_80211_REMAIN_ON_CHAN_SET(pAd, &ChanInfo, duration);
	return 0;
}
#endif /* RT_CFG80211_P2P_SUPPORT */

static void CFG80211_OpsMgmtFrameRegister(struct wiphy *pWiphy,
					  struct wireless_dev *wdev,
					  u16 frame_type, bool reg)
{
	VOID *pAd;
	struct net_device *dev = wdev->netdev;

	MAC80211_PAD_GET(pAd, pWiphy);

	CFG80211DBG(RT_DEBUG_INFO, ("80211> %s ==>\n", __func__));
	CFG80211DBG(RT_DEBUG_INFO,
		    ("frame_type = %x, req = %d , (%d)\n", frame_type, reg,
		     dev->ieee80211_ptr->iftype));

	if (frame_type == IEEE80211_STYPE_PROBE_REQ)
		RTMP_DRIVER_80211_MGMT_FRAME_REG(pAd, dev, reg);
	else if (frame_type == IEEE80211_STYPE_ACTION)
		RTMP_DRIVER_80211_ACTION_FRAME_REG(pAd, dev, reg);
	else
		CFG80211DBG(RT_DEBUG_ERROR,
			    ("Unkown frame_type = %x, req = %d\n", frame_type, reg));

}

/* Supplicant_NEW_TDLS */

static int mt76xx_cfg80211_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
				   struct ieee80211_channel *chan,
				   bool offchan, unsigned int wait,
				   const u8 *buf, size_t len, bool no_cck, bool dont_wait_for_ack,
				   u64 *pCookie)
{
	RTMP_ADAPTER *pAd;
	UINT32 channel;
	struct net_device *pDev = wdev->netdev;

	CFG80211DBG(RT_DEBUG_INFO, ("80211> [%s] %s ==>\n", pDev->name, __func__));
	MAC80211_PAD_GET(pAd, wiphy);
	if (!pAd)
		return -EFAULT;

	/* get channel number */

	channel = ieee80211_frequency_to_channel(chan->center_freq);
	CFG80211DBG(RT_DEBUG_TRACE, ("80211> Mgmt chan %d freq %d, no ack= %d\n", channel,
				chan->center_freq, dont_wait_for_ack));

	/* Send the Frame with basic rate 6 */
	*pCookie = 5678;
	pAd->cfg80211_ctrl.TxStatusSeq = 0;
	pAd->cfg80211_ctrl.IsNeedTxStatus = !dont_wait_for_ack;
	RTMP_DRIVER_80211_TX_NETDEV_SET(pAd, pDev);
	RTMP_DRIVER_80211_CHANNEL_LOCK(pAd, channel);
	RTMP_DRIVER_80211_MGMT_FRAME_SEND(pAd, (void *)buf, len);

	/* Mark it for using Supplicant-Based off-channel wait
	   if (offchan)
	   RTMP_DRIVER_80211_CHANNEL_RESTORE(pAd);
	 */
	if (pAd->cfg80211_ctrl.IsNeedTxStatus)
		*pCookie = pAd->cfg80211_ctrl.TxStatusSeq & 0xff;

	CFG80211DBG(RT_DEBUG_TRACE, ("%s, Cookie = 0x%x\n", __func__, (int)*pCookie));
	/*reset IsNeedTxStatus to FALSE*/
	pAd->cfg80211_ctrl.IsNeedTxStatus = FALSE;
	return 0;
}

static int CFG80211_OpsTxCancelWait(IN struct wiphy *pWiphy,
				    IN struct wireless_dev *pDev,
				    u64 cookie)
{
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	return 0;
}

#ifdef RT_CFG80211_P2P_SUPPORT
static int CFG80211_OpsCancelRemainOnChannel(struct wiphy *pWiphy,
					     struct wireless_dev *wdev,
					     u64 cookie)
{
	VOID *pAd;
	CFG80211DBG(RT_DEBUG_INFO, ("80211> %s ==>\n", __func__));
	MAC80211_PAD_GET(pAd, pWiphy);
	/* It cause the Supplicant-based OffChannel Hang */
	RTMP_DRIVER_80211_CANCEL_REMAIN_ON_CHAN_SET(pAd, cookie);
	return 0;
}
#endif /* RT_CFG80211_P2P_SUPPORT */

#ifdef CONFIG_AP_SUPPORT
static VOID CFG80211_UpdateAssocRespExtraIe(VOID *pAdOrg,
					    UCHAR *assocresp_ies, size_t assocresp_ies_len)
{
	PRTMP_ADAPTER pAd = (PRTMP_ADAPTER) pAdOrg;
	PUCHAR pAssocRespBuf = (PUCHAR) pAd->ApCfg.MBSSID[MAIN_MBSSID].AssocRespExtraIe;

	DBGPRINT(RT_DEBUG_TRACE, ("%s: IE len = %zu\n", __func__, assocresp_ies_len));
	if (assocresp_ies_len > sizeof(pAd->ApCfg.MBSSID[MAIN_MBSSID].AssocRespExtraIe)) {
		DBGPRINT(RT_DEBUG_ERROR, ("%s: AssocResp buf size not enough\n", __func__));
		return;
	}
	NdisCopyMemory(pAssocRespBuf, assocresp_ies, assocresp_ies_len);
	pAd->ApCfg.MBSSID[MAIN_MBSSID].AssocRespExtraIeLen = assocresp_ies_len;
}

static int CFG80211_OpsStartAp(struct wiphy *pWiphy,
			       struct net_device *netdev, struct cfg80211_ap_settings *settings)
{
	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_BEACON bcn;
	UCHAR *beacon_head_buf, *beacon_tail_buf;
#ifdef MT76XX_BTCOEX_SUPPORT
	struct COEX_LINKUP_INFO info;
	NdisZeroMemory(&info, sizeof(info));
#endif
	beacon_head_buf = NULL;
	beacon_tail_buf = NULL;
	MAC80211_PAD_GET(pAd, pWiphy);
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	NdisZeroMemory(&bcn, sizeof(CMD_RTPRIV_IOCTL_80211_BEACON));
	if (settings->beacon.head_len > 0) {
		os_alloc_mem(NULL, &beacon_head_buf, settings->beacon.head_len);
		NdisCopyMemory(beacon_head_buf, settings->beacon.head, settings->beacon.head_len);
	}

	if (settings->beacon.tail_len > 0) {
		os_alloc_mem(NULL, &beacon_tail_buf, settings->beacon.tail_len);
		NdisCopyMemory(beacon_tail_buf, settings->beacon.tail, settings->beacon.tail_len);
	}

	bcn.beacon_head_len = settings->beacon.head_len;
	bcn.beacon_tail_len = settings->beacon.tail_len;
	bcn.beacon_head = beacon_head_buf;
	bcn.beacon_tail = beacon_tail_buf;
	bcn.dtim_period = settings->dtim_period;
	bcn.interval = settings->beacon_interval;
	bcn.hidden_ssid = settings->hidden_ssid;

	/* set channel callback has been replaced by using chandef of cfg80211_ap_settings */
	if (settings->chandef.chan) {
		CFG80211_CB *p80211CB;
		CMD_RTPRIV_IOCTL_80211_CHAN ChanInfo;
		UINT32 ChanId;

		/* init */
		memset(&ChanInfo, 0, sizeof(ChanInfo));

		p80211CB = NULL;
		RTMP_DRIVER_80211_CB_GET(pAd, &p80211CB);

		if (p80211CB == NULL) {
			CFG80211DBG(RT_DEBUG_ERROR, ("80211> p80211CB == NULL!\n"));
			return 0;
		}

		/* get channel number */
		ChanId = ieee80211_frequency_to_channel(settings->chandef.chan->center_freq);
		CFG80211DBG(RT_DEBUG_OFF, ("80211> Channel = %d\n", ChanId));
		ChanInfo.ChanId = ChanId;

		ChanInfo.IfType = RT_CMD_80211_IFTYPE_P2P_GO;

		CFG80211DBG(RT_DEBUG_ERROR, ("80211> ChanInfo.IfType == %d!\n", ChanInfo.IfType));

		if (cfg80211_get_chandef_type(&settings->chandef) == NL80211_CHAN_NO_HT)
			ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_NOHT;
		else if (cfg80211_get_chandef_type(&settings->chandef) == NL80211_CHAN_HT20)
			ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_HT20;
		else if (cfg80211_get_chandef_type(&settings->chandef) == NL80211_CHAN_HT40MINUS)
			ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_HT40MINUS;
		else if (cfg80211_get_chandef_type(&settings->chandef) == NL80211_CHAN_HT40PLUS)
			ChanInfo.ChanType = RT_CMD_80211_CHANTYPE_HT40PLUS;

		CFG80211DBG(RT_DEBUG_ERROR,
			    ("80211> ChanInfo.ChanType == %d!\n", ChanInfo.ChanType));
		ChanInfo.MonFilterFlag = p80211CB->MonFilterFlag;

		/* set channel */
		RTMP_DRIVER_80211_CHAN_SET(pAd, &ChanInfo);
	}

	RTMP_DRIVER_80211_BEACON_ADD(pAd, &bcn);

	if (beacon_head_buf)
		os_free_mem(NULL, beacon_head_buf);
	if (beacon_tail_buf)
		os_free_mem(NULL, beacon_tail_buf);

#ifdef MT76XX_BTCOEX_SUPPORT
	{
		RTMP_ADAPTER *ad = (RTMP_ADAPTER *)pAd;
		info.coexType = COEX_OPMODE_GO;
		info.BBPCurrentBW = ad->CommonCfg.BBPCurrentBW;
		info.Channel = ad->CommonCfg.Channel;
		info.CentralChannel = ad->CommonCfg.CentralChannel;
		info.OwnMac = ad->cfg80211_ctrl.P2PCurrentAddress;
		info.Bssid = ad->cfg80211_ctrl.P2PCurrentAddress;
		info.wdev = &ad->ApCfg.MBSSID[MAIN_MBSSID].wdev;
		info.pEntry = &ad->MacTab.Content[0]; /* From APStartUp */
		CoexLinkUpHandler(ad, info);
	}
#endif /*MT76XX_BTCOEX_SUPPORT */
	return 0;
}

static int CFG80211_OpsChangeBeacon(struct wiphy *pWiphy,
				    struct net_device *netdev, struct cfg80211_beacon_data *info)
{
	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_BEACON bcn;
	UCHAR *beacon_head_buf, *beacon_tail_buf;
	beacon_head_buf = NULL;
	beacon_tail_buf = NULL;
	memset(&bcn, 0, sizeof(CMD_RTPRIV_IOCTL_80211_BEACON));

	MAC80211_PAD_GET(pAd, pWiphy);
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));

	if (info->head_len > 0) {
		os_alloc_mem(NULL, &beacon_head_buf, info->head_len);
		NdisCopyMemory(beacon_head_buf, info->head, info->head_len);
	}

	if (info->tail_len > 0) {
		os_alloc_mem(NULL, &beacon_tail_buf, info->tail_len);
		NdisCopyMemory(beacon_tail_buf, info->tail, info->tail_len);
	}

	bcn.beacon_head_len = info->head_len;
	bcn.beacon_tail_len = info->tail_len;
	bcn.beacon_head = beacon_head_buf;
	bcn.beacon_tail = beacon_tail_buf;
	/* Update assoc resp extra ie */
	if (info->assocresp_ies_len && info->assocresp_ies) {
		CFG80211_UpdateAssocRespExtraIe(pAd, (UCHAR *) info->assocresp_ies,
						info->assocresp_ies_len);
	}

	RTMP_DRIVER_80211_BEACON_SET(pAd, &bcn);

	if (beacon_head_buf)
		os_free_mem(NULL, beacon_head_buf);
	if (beacon_tail_buf)
		os_free_mem(NULL, beacon_tail_buf);
	return 0;

}

static int CFG80211_OpsStopAp(struct wiphy *pWiphy, struct net_device *netdev)
{
	RTMP_ADAPTER *pAd;

	MAC80211_PAD_GET(pAd, pWiphy);

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));

	RTMP_DRIVER_80211_BEACON_DEL(pAd);

	RTMPReadParametersHook(pAd);
	SetCommonHT(pAd);

#ifdef MT76XX_BTCOEX_SUPPORT
	pAd->CoexMode.LinkStatus &= ~COEX_GO_LINK;
	COEXLinkDown(pAd, pAd->cfg80211_ctrl.P2PCurrentAddress);
#endif /*MT76XX_BTCOEX_SUPPORT */

	return 0;
}
#endif /* CONFIG_AP_SUPPORT */
static int CFG80211_OpsChangeBss(struct wiphy *pWiphy,
				 struct net_device *netdev, struct bss_parameters *params)
{

	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_BSS_PARM bssInfo;
	MAC80211_PAD_GET(pAd, pWiphy);

	CFG80211DBG(RT_DEBUG_TRACE, ("80211> %s ==>\n", __func__));

	bssInfo.use_short_preamble = params->use_short_preamble;
	bssInfo.use_short_slot_time = params->use_short_slot_time;
	bssInfo.use_cts_prot = params->use_cts_prot;

	RTMP_DRIVER_80211_CHANGE_BSS_PARM(pAd, &bssInfo);

	return 0;
}

static int mt76xx_cfg80211_del_sta(struct wiphy *wiphy, struct net_device *dev,
				   u8 *mac)
{
	RTMP_ADAPTER *pAd;
	MAC80211_PAD_GET(pAd, wiphy);

	if (!pAd)
		return -EFAULT;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	if (mac) {
		CFG80211DBG(RT_DEBUG_OFF,
			    ("80211> Delete STA(%02X:%02X:%02X:%02X:%02X:%02X) ==>\n",
			     PRINT_MAC(mac)));
	}
	CFG80211_ApStaDel(pAd, mac);

	return 0;
}

static int mt76xx_cfg80211_add_sta(struct wiphy *wiphy, struct net_device *dev,
				   u8 *mac,
				   struct station_parameters *params)
{
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s ==>\n", __func__));
	return 0;
}

static int mt76xx_cfg80211_change_sta(struct wiphy *wiphy, struct net_device *dev,
				      u8 *mac,
				      struct station_parameters *params)
{
	RTMP_ADAPTER *pAd;

	if (!mac)
		return -EINVAL;

	MAC80211_PAD_GET(pAd, wiphy);
	if (!pAd)
		return -EFAULT;

	CFG80211DBG(RT_DEBUG_OFF, ("80211> Change STA(%02X:%02X:%02X:%02X:%02X:%02X) ==> %d\n",
				     PRINT_MAC(mac), dev->ieee80211_ptr->iftype));

	if ((dev->ieee80211_ptr->iftype != RT_CMD_80211_IFTYPE_AP)
	    && (dev->ieee80211_ptr->iftype != RT_CMD_80211_IFTYPE_P2P_GO)
	    )
		return -EOPNOTSUPP;

	if (!(params->sta_flags_mask & BIT(NL80211_STA_FLAG_AUTHORIZED))) {
		CFG80211DBG(RT_DEBUG_ERROR, ("80211> %x ==>\n", params->sta_flags_mask));
		return -EOPNOTSUPP;
	}

	if (params->sta_flags_set & BIT(NL80211_STA_FLAG_AUTHORIZED)) {
		CFG80211DBG(RT_DEBUG_TRACE,
			    ("80211> STA(%02X:%02X:%02X:%02X:%02X:%02X) ==> PortSecured\n",
			     PRINT_MAC(mac)));
		CFG80211_StaPortSecured(pAd, mac, 1);
	} else {
		CFG80211DBG(RT_DEBUG_TRACE,
			    ("80211> STA(%02X:%02X:%02X:%02X:%02X:%02X) ==> PortNotSecured\n",
			     PRINT_MAC(mac)));
		CFG80211_StaPortSecured(pAd, mac, 0);
	}
	return 0;
}

#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
static struct wireless_dev *CFG80211_OpsVirtualInfAdd(IN struct wiphy *pWiphy,
						      IN const char *name,
						      IN enum nl80211_iftype Type,
						      IN u32 *pFlags, struct vif_params *pParams)
{
	VOID *pAd;
	CMD_RTPRIV_IOCTL_80211_VIF_SET vifInfo;
	PWIRELESS_DEV pDev = NULL;
	MAC80211_PAD_GET(pAd, pWiphy);

	CFG80211DBG(RT_DEBUG_OFF,
		    ("80211> %s [%s,%d, %zu] ==>\n", __func__, name, Type, strlen(name)));

	vifInfo.vifType = Type;
	vifInfo.vifNameLen = strlen(name);
	memset(vifInfo.vifName, 0, sizeof(vifInfo.vifName));
	NdisCopyMemory(vifInfo.vifName, name, vifInfo.vifNameLen);

	if (RTMP_DRIVER_80211_VIF_ADD(pAd, &vifInfo) != NDIS_STATUS_SUCCESS) {
		/* Fix kernel crash problem in Linux-3.10 */
		CFG80211DBG(RT_DEBUG_ERROR, ("80211> %s add failed! ERROR!\n", __func__));
		/* return NULL; */
		return ERR_PTR(-ENODEV);
	}
	pDev = RTMP_CFG80211_FindVifEntryWdev_ByType(pAd, Type);

	return pDev;
}

static int CFG80211_OpsVirtualInfDel(IN struct wiphy *pWiphy,
				     IN struct wireless_dev *pwdev
    )
{
	void *pAd;
	struct net_device *dev = NULL;
	dev = pwdev->netdev;

	if (!dev)
		return 0;

	CFG80211DBG(RT_DEBUG_OFF,
		    ("80211> %s, %s [%d]==>\n", __func__, dev->name, dev->ieee80211_ptr->iftype));
	MAC80211_PAD_GET(pAd, pWiphy);

	RTMP_DRIVER_80211_VIF_DEL(pAd, dev, dev->ieee80211_ptr->iftype);
	return 0;
}
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */

static int CFG80211_OpsBitrateSet(IN struct wiphy *pWiphy,
				  IN struct net_device *dev,
				  IN const u8 *peer, IN const struct cfg80211_bitrate_mask *mask)
{
	CFG80211DBG(RT_DEBUG_OFF,
		    ("80211> %s, %s [%d]==>\n", __func__, dev->name, dev->ieee80211_ptr->iftype));
	return 0;
}

static int CFG80211_start_p2p_device(struct wiphy *pWiphy, struct wireless_dev *wdev)
{
	void *pAd;
	struct net_device *dev = wdev->netdev;
	CFG80211DBG(RT_DEBUG_OFF,
		    ("80211> %s, %s [%d]==>\n", __func__, dev->name, dev->ieee80211_ptr->iftype));
	MAC80211_PAD_GET(pAd, pWiphy);
	return 0;
}

static void CFG80211_stop_p2p_device(struct wiphy *pWiphy, struct wireless_dev *wdev)
{
	void *pAd;
	struct net_device *dev = wdev->netdev;
	CFG80211DBG(RT_DEBUG_OFF,
		    ("80211> %s, %s [%d]==>\n", __func__, dev->name, dev->ieee80211_ptr->iftype));
	MAC80211_PAD_GET(pAd, pWiphy);
}

#ifdef NEW_WOW_SUPPORT
int CFG80211_OpsSuspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow)
{
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s\n", __func__));

	if (wow) {
		CFG80211DBG(RT_DEBUG_OFF,
			    ("Trigger\n\r\tany: %d\n\r\tdisconnect: %d\n\r\tmagic_pkt: %d\n\r\tpattern_num: %d\n\r\tpatterns: %p\n",
			     wow->any, wow->disconnect, wow->magic_pkt, wow->n_patterns,
			     wow->patterns));

		CFG80211DBG(RT_DEBUG_OFF,
			    ("\r\teap_identity_req: %d\n\r\tfour_way_handshake: %d\n\r\tgtk_rekey_failure: %d\n\r\trfkill_release: %d\n",
			     wow->eap_identity_req, wow->four_way_handshake, wow->gtk_rekey_failure,
			     wow->rfkill_release));

		CFG80211DBG(RT_DEBUG_OFF, ("\r\ttcp: %p\n", wow->tcp));
	}

	return 0;
}

int CFG80211_OpsResume(struct wiphy *wiphy)
{
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s\n", __func__));

	return 0;
}

void CFG80211_OpsSetWakeup(struct wiphy *wiphy, bool enabled)
{
	CFG80211DBG(RT_DEBUG_OFF, ("80211> %s, Set_wakeup = %d\n", __func__, enabled));
}
#endif /* NEW_WOW_SUPPORT */

static const struct ieee80211_txrx_stypes ralink_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_STATION] = {
				    .tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
				    BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
				    .rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
				    BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
				    },
	[NL80211_IFTYPE_P2P_CLIENT] = {
				       .tx = BIT(IEEE80211_STYPE_ACTION >> 4) |
				       BIT(IEEE80211_STYPE_PROBE_RESP >> 4),
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
			       BIT(IEEE80211_STYPE_DEAUTH >> 4) | BIT(IEEE80211_STYPE_ACTION >> 4),
			       },
	[NL80211_IFTYPE_P2P_GO] = {
				   .tx = 0xffff,
				   .rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
				   BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
				   BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
				   BIT(IEEE80211_STYPE_DISASSOC >> 4) |
				   BIT(IEEE80211_STYPE_AUTH >> 4) |
				   BIT(IEEE80211_STYPE_DEAUTH >> 4) |
				   BIT(IEEE80211_STYPE_ACTION >> 4),
				   },

};

static const struct ieee80211_iface_limit ra_p2p_sta_go_limits[] = {
	{
	 .max = 3,
	 .types = BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP),
	 },
	{
	 .max = 1,
	 .types = BIT(NL80211_IFTYPE_P2P_GO) | BIT(NL80211_IFTYPE_P2P_CLIENT),
	 },
};

static const struct ieee80211_iface_combination ra_iface_combinations_ap_sta[] = {
	{
#ifdef RT_CFG80211_P2P_MULTI_CHAN_SUPPORT
	 .num_different_channels = 2,
#else
	 .num_different_channels = 1,
#endif /* RT_CFG80211_P2P_MULTI_CHAN_SUPPORT */
	 .max_interfaces = 3,
	 /* .beacon_int_infra_match = true, */
	 .limits = ra_p2p_sta_go_limits,
	 .n_limits = 1,		/* does not include p2p part */
	 },
};

static const struct ieee80211_iface_combination ra_iface_combinations_p2p[] = {
	{
#ifdef RT_CFG80211_P2P_MULTI_CHAN_SUPPORT
	 .num_different_channels = 2,
#else
	 .num_different_channels = 1,
#endif /* RT_CFG80211_P2P_MULTI_CHAN_SUPPORT */
	 .max_interfaces = 3,
	 /* .beacon_int_infra_match = true, */
	 .limits = ra_p2p_sta_go_limits,
	 .n_limits = ARRAY_SIZE(ra_p2p_sta_go_limits),	/* include p2p */
	 },
};

const struct ieee80211_iface_combination *p_ra_iface_combinations_ap_sta = ra_iface_combinations_ap_sta;
const INT ra_iface_combinations_ap_sta_num = ARRAY_SIZE(ra_iface_combinations_ap_sta);

const struct ieee80211_iface_combination *p_ra_iface_combinations_p2p = ra_iface_combinations_p2p;
const INT ra_iface_combinations_p2p_num = ARRAY_SIZE(ra_iface_combinations_p2p);

struct cfg80211_ops CFG80211_Ops = {
#ifdef NEW_WOW_SUPPORT
	.suspend = CFG80211_OpsSuspend,
	.resume = CFG80211_OpsResume,
	.set_wakeup = CFG80211_OpsSetWakeup,
#endif /* NEW_WOW_SUPPORT */

#ifdef CONFIG_AP_SUPPORT
	.start_ap = CFG80211_OpsStartAp,
	.change_beacon = CFG80211_OpsChangeBeacon,
	.stop_ap = CFG80211_OpsStopAp,
#endif /* CONFIG_AP_SUPPORT */
	/* set channel for a given wireless interface */
	.set_monitor_channel = CFG80211_OpsMonitorChannelSet,

	/* change type/configuration of virtual interface */
	.change_virtual_intf = CFG80211_OpsVirtualInfChg,
#ifdef RT_CFG80211_P2P_CONCURRENT_DEVICE
	.add_virtual_intf = CFG80211_OpsVirtualInfAdd,
	.del_virtual_intf = CFG80211_OpsVirtualInfDel,
#endif /* RT_CFG80211_P2P_CONCURRENT_DEVICE */

	.start_p2p_device = CFG80211_start_p2p_device,
	.stop_p2p_device = CFG80211_stop_p2p_device,

	/* request to do a scan */
	/*
	   Note: must exist whatever AP or STA mode; Or your kernel will crash
	   in v2.6.38.
	 */
	.scan = CFG80211_OpsScan,

#ifdef CONFIG_STA_SUPPORT
	/* join the specified IBSS (or create if necessary) */
	.join_ibss = mt76xx_cfg80211_join_ibss,
	/* leave the IBSS */
	.leave_ibss = CFG80211_OpsIbssLeave,
#endif /* CONFIG_STA_SUPPORT */

	/* configure WLAN power management */
	.set_power_mgmt = CFG80211_OpsPwrMgmt,
	/* get station information for the station identified by @mac */
	.get_station = mt76xx_cfg80211_get_sta,
	/* dump station callback */
	.dump_station = CFG80211_OpsStaDump,
	/* notify that wiphy parameters have changed */
	.set_wiphy_params = CFG80211_OpsWiphyParamsSet,
	/* add a key with the given parameters */
	.add_key = CFG80211_OpsKeyAdd,
	/* get information about the key with the given parameters */
	.get_key = CFG80211_OpsKeyGet,
	/* remove a key given the @mac_addr */
	.del_key = CFG80211_OpsKeyDel,
	/* set the default key on an interface */
	.set_default_key = mt76xx_cfg80211_set_default_key,
#ifdef DOT11W_PMF_SUPPORT
	.set_default_mgmt_key = CFG80211_OpsMgmtKeyDefaultSet,
#endif /* DOT11W_PMF_SUPPORT */

	/* connect to the ESS with the specified parameters */
	.connect = mt76xx_cfg80211_connect,
	/* disconnect from the BSS/ESS */
	.disconnect = CFG80211_OpsDisconnect,

#ifdef RFKILL_HW_SUPPORT
	/* polls the hw rfkill line */
	.rfkill_poll = CFG80211_OpsRFKill,
#endif /* RFKILL_HW_SUPPORT */

	/* set the bitrate mask configuration */
	.set_bitrate_mask = CFG80211_OpsBitrateSet,
	/* get site survey information */
	.dump_survey = CFG80211_OpsSurveyGet,
	/* cache a PMKID for a BSSID */
	.set_pmksa = mt76xx_cfg80211_set_pmksa,
	/* delete a cached PMKID */
	.del_pmksa = mt76xx_cfg80211_del_pmksa,
	/* flush all cached PMKIDs */
	.flush_pmksa = mt76xx_cfg80211_flush_pmksa,

#ifdef RT_CFG80211_P2P_SUPPORT
	/*
	   Request the driver to remain awake on the specified
	   channel for the specified duration to complete an off-channel
	   operation (e.g., public action frame exchange).
	 */
	.remain_on_channel = CFG80211_OpsRemainOnChannel,
	/* cancel an on-going remain-on-channel operation */
	.cancel_remain_on_channel = CFG80211_OpsCancelRemainOnChannel,
#endif /* RT_CFG80211_P2P_SUPPORT */
	.mgmt_tx = mt76xx_cfg80211_mgmt_tx,

	.mgmt_tx_cancel_wait = CFG80211_OpsTxCancelWait,

	/* configure connection quality monitor RSSI threshold */
	.set_cqm_rssi_config = NULL,

	/* notify driver that a management frame type was registered */
	.mgmt_frame_register = CFG80211_OpsMgmtFrameRegister,

	/* set antenna configuration (tx_ant, rx_ant) on the device */
	.set_antenna = NULL,
	/* get current antenna configuration from device (tx_ant, rx_ant) */
	.get_antenna = NULL,
	.change_bss = CFG80211_OpsChangeBss,
	.del_station = mt76xx_cfg80211_del_sta,
	.add_station = mt76xx_cfg80211_add_sta,
	.change_station = mt76xx_cfg80211_change_sta,
/* .set_bitrate_mask                       = CFG80211_OpsBitrateSet, */
	.set_tx_power = CFG80211_OpsTxPwrSet,
	.get_tx_power = CFG80211_OpsTxPwrGet,
};

/* =========================== Global Function ============================== */

static INT CFG80211NetdevNotifierEvent(struct notifier_block *nb, ULONG state, VOID *ndev)
{
	VOID *pAd;
	struct net_device *pNev = ndev;
	struct wireless_dev *pWdev = pNev->ieee80211_ptr;

	CFG80211DBG(RT_DEBUG_TRACE, ("[%s] state: %ld\n", __func__, state));

	if (!ndev || !pWdev || !pWdev->wiphy)
		return NOTIFY_DONE;
	if (NETDEV_POST_INIT)
		return NOTIFY_DONE;

	MAC80211_PAD_GET(pAd, pWdev->wiphy);

	if (!pAd)
		return NOTIFY_DONE;

	switch (state) {
	case NETDEV_UNREGISTER:
		break;

	case NETDEV_GOING_DOWN:
		RTMP_DRIVER_80211_NETDEV_EVENT(pAd, pNev, state);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

struct notifier_block cfg80211_netdev_notifier = {
	.notifier_call = CFG80211NetdevNotifierEvent,
};

static const struct ieee80211_regdomain rtmp_custom_regd = {
	.n_reg_rules = 2,
	.alpha2 = "00",
	.reg_rules = {
		      REG_RULE(5260 - 20, 5320 + 20, 40, 6, 20,
			       NL80211_RRF_DFS | NL80211_RRF_PASSIVE_SCAN),
		      REG_RULE(5725 - 10, 5850 + 10, 40, 0, 30, NL80211_RRF_DFS),
		      }
};

/*
========================================================================
Routine Description:
	Allocate a wireless device.

Arguments:
	pAd				- WLAN control block pointer
	pDev			- Generic device interface

Return Value:
	wireless device

Note:
========================================================================
*/
static struct wireless_dev *CFG80211_WdevAlloc(IN CFG80211_CB *pCfg80211_CB,
					       IN CFG80211_BAND * pBandInfo,
					       IN VOID *pAd, IN struct device *pDev)
{
	struct wireless_dev *pWdev;
	ULONG *pPriv;

	/*
	 * We're trying to have the following memory layout:
	 *
	 * +------------------------+
	 * | struct wiphy                       |
	 * +------------------------+
	 * | pAd pointer                        |
	 * +------------------------+
	 */

	pWdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
	if (pWdev == NULL) {
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Wireless device allocation fail!\n"));
		return NULL;
	}
	/* End of if */
#if defined(MESH_SUPPORT) && defined(MESH_WIRELESS_BRIDGE_SUPPORT)
	pWdev->use_4addr = true;
#endif /* endif */
	pWdev->wiphy = wiphy_new(&CFG80211_Ops, sizeof(ULONG *));
	if (pWdev->wiphy == NULL) {
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Wiphy device allocation fail!\n"));
		goto LabelErrWiphyNew;
	}

	/* End of if */
	/* keep pAd pointer */
	pPriv = (ULONG *) (wiphy_priv(pWdev->wiphy));
	*pPriv = (ULONG) pAd;

	set_wiphy_dev(pWdev->wiphy, pDev);

	/* max_scan_ssids means in each scan request, how many ssids can driver handle to send probe-req.
	   In current design, we only support 1 ssid at a time. So we should set to 1.
	 */
	/* pWdev->wiphy->max_scan_ssids = pBandInfo->MaxBssTable; */
	pWdev->wiphy->max_scan_ssids = 1;

	/* @NL80211_FEATURE_INACTIVITY_TIMER:
	   This driver takes care of freeingup
	   the connected inactive stations in AP mode. */

	/*what if you get compile error for below flag, please add the patch into your kernel */
	/* http://www.permalink.gmane.org/gmane.linux.kernel.wireless.general/86454 */
	pWdev->wiphy->features |= NL80211_FEATURE_INACTIVITY_TIMER;

	pWdev->wiphy->interface_modes = BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_STATION)
#ifdef CONFIG_SNIFFER_SUPPORT
	    | BIT(NL80211_IFTYPE_MONITOR)
#endif /* CONFIG_SNIFFER_SUPPORT */
	    ;

#ifdef CONFIG_STA_SUPPORT

	pWdev->wiphy->interface_modes |= BIT(NL80211_IFTYPE_ADHOC);

#ifdef RT_CFG80211_P2P_SINGLE_DEVICE
	pWdev->wiphy->interface_modes |= (BIT(NL80211_IFTYPE_P2P_CLIENT)
					  | BIT(NL80211_IFTYPE_P2P_GO));
	pWdev->wiphy->software_iftypes |= BIT(NL80211_IFTYPE_P2P_DEVICE);
#endif /* RT_CFG80211_P2P_SINGLE_DEVICE */
#endif /* CONFIG_STA_SUPPORT */

#ifdef RT_CFG80211_P2P_SUPPORT

	/*
	   pWdev->wiphy->software_iftypes |= (BIT(NL80211_IFTYPE_P2P_CLIENT)
	   | BIT(NL80211_IFTYPE_P2P_GO));
	 */
	/* NL80211_IFTYPE_P2P_DEVICE Kernel Symbol start from 3.7 */
	pWdev->wiphy->software_iftypes |= BIT(NL80211_IFTYPE_P2P_DEVICE);
#endif /* RT_CFG80211_P2P_SUPPORT */

	pWdev->wiphy->reg_notifier = CFG80211_RegNotifier;

	/* init channel information */
	CFG80211_SupBandInit(pCfg80211_CB, pBandInfo, pWdev->wiphy, NULL, NULL);

#ifdef CONFIG_ANDROID
	/* only defined in Android kernel with CONFIG_ANDROID? */
	/* Ensure passive/beaconing flags not be lifted by cfg80211 due to
	 * regulatory beacon hints
	 */
	pWdev->wiphy->flags |= WIPHY_FLAG_DISABLE_BEACON_HINTS;
#endif /* endif */

	/* CFG80211_SIGNAL_TYPE_MBM: signal strength in mBm (100*dBm) */
	pWdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	pWdev->wiphy->max_scan_ie_len = IEEE80211_MAX_DATA_LEN;

	pWdev->wiphy->max_num_pmkids = PMKID_NO;

	pWdev->wiphy->max_remain_on_channel_duration = MAX_ROC_TIME;

	pWdev->wiphy->mgmt_stypes = ralink_mgmt_stypes;

	pWdev->wiphy->cipher_suites = CipherSuites;
	pWdev->wiphy->n_cipher_suites = ARRAY_SIZE(CipherSuites);

	pWdev->wiphy->flags |= WIPHY_FLAG_AP_UAPSD;

	/*what if you get compile error for below flag, please add the patch into your kernel */
	/* 018-cfg80211-internal-ap-mlme.patch */
	pWdev->wiphy->flags |= WIPHY_FLAG_HAVE_AP_SME;

	/*what if you get compile error for below flag, please add the patch into your kernel */
	/* 008-cfg80211-offchan-flags.patch */
	pWdev->wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL;
	/* CFG_TODO */
	/* pWdev->wiphy->flags |= WIPHY_FLAG_STRICT_REGULATORY; */

#if defined(MESH_SUPPORT) && defined(MESH_WIRELESS_BRIDGE_SUPPORT)
	pWdev->wiphy->flags |= WIPHY_FLAG_4ADDR_STATION;
	/* pWdev->wiphy->flags |= WIPHY_FLAG_4ADDR_AP; */
#endif /* endif */

	/* Driver Report Support TDLS to supplicant */

	pWdev->wiphy->iface_combinations = ra_iface_combinations_ap_sta;
	pWdev->wiphy->n_iface_combinations = ARRAY_SIZE(ra_iface_combinations_ap_sta);

#ifdef NEW_WOW_SUPPORT
	/* Add WoWL capability support for wiphy */
	pWdev->wiphy->wowlan.flags |= WIPHY_WOWLAN_MAGIC_PKT;
#endif /* NEW_WOW_SUPPORT */

	if (wiphy_register(pWdev->wiphy) < 0) {
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Register wiphy device fail!\n"));
		goto LabelErrReg;
	}
	/* End of if */
	return pWdev;

LabelErrReg:
	wiphy_free(pWdev->wiphy);

LabelErrWiphyNew:
	os_free_mem(NULL, pWdev);

	return NULL;
}				/* End of CFG80211_WdevAlloc */

/*
========================================================================
Routine Description:
	Register MAC80211 Module.

Arguments:
	pAdCB			- WLAN control block pointer
	pDev			- Generic device interface
	pNetDev			- Network device

Return Value:
	NONE

Note:
	pDev != pNetDev
	#define SET_NETDEV_DEV(net, pdev)	((net)->dev.parent = (pdev))

	Can not use pNetDev to replace pDev; Or kernel panic.
========================================================================
*/
BOOLEAN CFG80211_Register(IN VOID *pAd, IN struct device *pDev, IN struct net_device *pNetDev)
{
	CFG80211_CB *pCfg80211_CB = NULL;
	CFG80211_BAND BandInfo;
	INT err;

	/* allocate Main Device Info structure */
	os_alloc_mem(NULL, (UCHAR **) &pCfg80211_CB, sizeof(CFG80211_CB));
	if (pCfg80211_CB == NULL) {
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Allocate MAC80211 CB fail!\n"));
		return FALSE;
	}

	/* allocate wireless device */
	RTMP_DRIVER_80211_BANDINFO_GET(pAd, &BandInfo);

	pCfg80211_CB->pCfg80211_Wdev = CFG80211_WdevAlloc(pCfg80211_CB, &BandInfo, pAd, pDev);
	if (pCfg80211_CB->pCfg80211_Wdev == NULL) {
		DBGPRINT(RT_DEBUG_ERROR, ("80211> Allocate Wdev fail!\n"));
		os_free_mem(NULL, pCfg80211_CB);
		return FALSE;
	}

	/* bind wireless device with net device */
#ifdef CONFIG_AP_SUPPORT
	/* default we are AP mode */
	pCfg80211_CB->pCfg80211_Wdev->iftype = NL80211_IFTYPE_AP;
#endif /* CONFIG_AP_SUPPORT */

#ifdef CONFIG_STA_SUPPORT
	/* default we are station mode */
	pCfg80211_CB->pCfg80211_Wdev->iftype = NL80211_IFTYPE_STATION;
#endif /* CONFIG_STA_SUPPORT */

	pNetDev->ieee80211_ptr = pCfg80211_CB->pCfg80211_Wdev;
	SET_NETDEV_DEV(pNetDev, wiphy_dev(pCfg80211_CB->pCfg80211_Wdev->wiphy));
	pCfg80211_CB->pCfg80211_Wdev->netdev = pNetDev;

#ifdef RFKILL_HW_SUPPORT
	wiphy_rfkill_start_polling(pCfg80211_CB->pCfg80211_Wdev->wiphy);
#endif /* RFKILL_HW_SUPPORT */

	RTMP_DRIVER_80211_CB_SET(pAd, pCfg80211_CB);
	RTMP_DRIVER_80211_RESET(pAd);
	RTMP_DRIVER_80211_SCAN_STATUS_LOCK_INIT(pAd, TRUE);

#ifdef CONFIG_STA_SUPPORT	/* reg notifier in AP case 7620&7612 will hang */
	err = register_netdevice_notifier(&cfg80211_netdev_notifier);
	if (err) {
		CFG80211DBG(RT_DEBUG_ERROR, ("80211> Failed to register notifierl %d\n", err));
	}
#endif /* CONFIG_STA_SUPPORT */

	CFG80211DBG(RT_DEBUG_ERROR, ("80211> CFG80211_Register\n"));
	return TRUE;
}				/* End of CFG80211_Register */

/* =========================== Local Function =============================== */

/*
========================================================================
Routine Description:
	The driver's regulatory notification callback.

Arguments:
	pWiphy			- Wireless hardware description
	pRequest		- Regulatory request

Return Value:
	0

Note:
========================================================================
*/
static void CFG80211_RegNotifier(
					 IN struct wiphy *pWiphy,
					 IN struct regulatory_request *pRequest)
{
	VOID *pAd;
	ULONG *pPriv;

	/* sanity check */
	pPriv = (ULONG *) (wiphy_priv(pWiphy));
	pAd = (VOID *) (*pPriv);

	if (pAd == NULL) {
		DBGPRINT(RT_DEBUG_ERROR, ("crda> reg notify but pAd = NULL!"));
		return;
	}

	/* End of if */
	/*
	   Change the band settings (PASS scan, IBSS allow, or DFS) in mac80211
	   based on EEPROM.

	   IEEE80211_CHAN_DISABLED: This channel is disabled.
	   IEEE80211_CHAN_PASSIVE_SCAN: Only passive scanning is permitted
	   on this channel.
	   IEEE80211_CHAN_NO_IBSS: IBSS is not allowed on this channel.
	   IEEE80211_CHAN_RADAR: Radar detection is required on this channel.
	   IEEE80211_CHAN_NO_FAT_ABOVE: extension channel above this channel
	   is not permitted.
	   IEEE80211_CHAN_NO_FAT_BELOW: extension channel below this channel
	   is not permitted.
	 */

	/*
	   Change regulatory rule here.

	   struct ieee80211_channel {
	   enum ieee80211_band band;
	   u16 center_freq;
	   u8 max_bandwidth;
	   u16 hw_value;
	   u32 flags;
	   int max_antenna_gain;
	   int max_power;
	   bool beacon_found;
	   u32 orig_flags;
	   int orig_mag, orig_mpwr;
	   };

	   In mac80211 layer, it will change flags, max_antenna_gain,
	   max_bandwidth, max_power.
	 */

	switch (pRequest->initiator) {
	case NL80211_REGDOM_SET_BY_CORE:
		/*
		   Core queried CRDA for a dynamic world regulatory domain.
		 */
		CFG80211DBG(RT_DEBUG_ERROR, ("crda> requlation requestion by core: "));
		break;

	case NL80211_REGDOM_SET_BY_USER:
		/*
		   User asked the wireless core to set the regulatory domain.
		   (when iw, network manager, wpa supplicant, etc.)
		 */
		CFG80211DBG(RT_DEBUG_ERROR, ("crda> requlation requestion by user: "));
		break;

	case NL80211_REGDOM_SET_BY_DRIVER:
		/*
		   A wireless drivers has hinted to the wireless core it thinks
		   its knows the regulatory domain we should be in.
		   (when driver initialization, calling regulatory_hint)
		 */
		CFG80211DBG(RT_DEBUG_ERROR, ("crda> requlation requestion by driver: "));
		break;

	case NL80211_REGDOM_SET_BY_COUNTRY_IE:
		/*
		   The wireless core has received an 802.11 country information
		   element with regulatory information it thinks we should consider.
		   (when beacon receive, calling regulatory_hint_11d)
		 */
		CFG80211DBG(RT_DEBUG_ERROR, ("crda> requlation requestion by country IE: "));
		break;
	}			/* End of switch */

	CFG80211DBG(RT_DEBUG_OFF, ("%c%c\n", pRequest->alpha2[0], pRequest->alpha2[1]));

	/* only follow rules from user */
	if (pRequest->initiator == NL80211_REGDOM_SET_BY_USER) {
		/* keep Alpha2 and we can re-call the function when interface is up */
		CMD_RTPRIV_IOCTL_80211_REG_NOTIFY RegInfo;

		RegInfo.Alpha2[0] = pRequest->alpha2[0];
		RegInfo.Alpha2[1] = pRequest->alpha2[1];
		RegInfo.pWiphy = pWiphy;

		RTMP_DRIVER_80211_REG_NOTIFY(pAd, &RegInfo);
	}
	/* End of if */
	return;
}				/* End of CFG80211_RegNotifier */

#endif /* RT_CFG80211_SUPPORT */

/* End of crda.c */
