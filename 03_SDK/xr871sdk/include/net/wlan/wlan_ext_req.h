/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _NET_WLAN_WLAN_EXT_REQ_H_
#define _NET_WLAN_WLAN_EXT_REQ_H_

#include <stdint.h>
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Wlan extended command definition
 */
typedef enum wlan_ext_cmd {
    WLAN_EXT_CMD_SET_PM_DTIM = 0,
    WLAN_EXT_CMD_GET_PM_DTIM,
    WLAN_EXT_CMD_SET_PS_CFG,
    WLAN_EXT_CMD_SET_AMPDU_TXNUM,
    WLAN_EXT_CMD_SET_TX_RETRY_CNT,
    WLAN_EXT_CMD_SET_PM_TX_NULL_PERIOD,
    WLAN_EXT_CMD_SET_BCN_WIN_US,
    WLAN_EXT_CMD_GET_BCN_STATUS,
    WLAN_EXT_CMD_SET_PHY_PARAM,
    WLAN_EXT_CMD_SET_SCAN_PARAM,
    WLAN_EXT_CMD_SET_LISTEN_INTERVAL,
    WLAN_EXT_CMD_SET_AUTO_SCAN,
    WLAN_EXT_CMD_SET_P2P_SVR,
    WLAN_EXT_CMD_SET_P2P_WKP_CFG,
    WLAN_EXT_CMD_SET_P2P_KPALIVE_CFG,
    WLAN_EXT_CMD_SET_P2P_HOST_SLEEP,
    WLAN_EXT_CMD_SET_BCN_WIN_CFG,
    WLAN_EXT_CMD_SET_FORCE_B_RATE,
    WLAN_EXT_CMD_SET_RCV_SPECIAL_FRM,
    WLAN_EXT_CMD_SET_SEND_RAW_FRM_CFG,
    WLAN_EXT_CMD_SET_SNIFF_AUTO_WKP_CFG,
    WLAN_EXT_CMD_SET_RCV_FRM_FILTER_CFG,
    WLAN_EXT_CMD_SET_POWER_LEVEL_TAB,
    WLAN_EXT_CMD_GET_POWER_LEVEL_TAB,

    WLAN_EXT_CMD_GET_TX_RATE = 50,
    WLAN_EXT_CMD_GET_SIGNAL,

    WLAN_EXT_CMD_SET_MBUF_LIMIT,
    WLAN_EXT_CMD_SET_AMPDU_REORDER_AGE,
    WLAN_EXT_CMD_SET_SCAN_FREQ,
} wlan_ext_cmd_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_PS_CFG
 */
typedef struct wlan_ext_ps_cfg {
	uint8_t ps_mode;
	uint8_t ps_idle_period;
	uint8_t ps_change_period;
} wlan_ext_ps_cfg_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_PM_DTIM
 */
typedef struct wlan_ext_pm_dtim {
	uint8_t pm_join_dtim_period;
	uint8_t pm_dtim_period_extend;
} wlan_ext_pm_dtim_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_BCN_STATUS
 */
typedef struct wlan_ext_bcn_status {
	uint32_t bcn_duration;
	int32_t  bcn_delay_max;
	int32_t  bcn_delay_sum;
	uint16_t bcn_delay_cnt[8];
	uint16_t bcn_rx_cnt;
	uint16_t bcn_miss_cnt;
} wlan_ext_bcn_status_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_SIGNAL
 */
typedef struct wlan_ext_signal {
	int8_t rssi;	/* unit is 0.5db */
	int8_t noise;	/* unit is dbm */
} wlan_ext_signal_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_MBUF_LIMIT
 */
typedef struct wlan_ext_mbuf_limit {
	uint32_t tx;
	uint32_t rx;
	uint32_t txrx;
} wlan_ext_mbuf_limit_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_PHY_PARAM
 */
typedef struct wlan_ext_phy_param {
	uint8_t data[6];
} wlan_ext_phy_param_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SCAN_PARAM
 */
typedef struct wlan_ext_scan_param {
	uint8_t  num_probes;  /* number of probe requests (per SSID) sent to
	                         one channel, default to 2 */
	uint8_t  probe_delay; /* delay time (in us) before sending a probe request,
	                         default to 100 us */
	uint16_t min_dwell;   /* min channel dwell time (in ms), default to 15 ms */
	uint16_t max_dwell;   /* max channel dwell time (in ms), default to 35 ms */
} wlan_ext_scan_param_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SCAN_FREQ
 */
typedef struct wlan_ext_scan_freq {
	uint16_t freq_num;
	int32_t  *freq_list;
} wlan_ext_scan_freq_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_AUTO_SCAN
 */
typedef struct wlan_ext_auto_scan_param {
	uint8_t  auto_scan_enable;  /* enable auto scan, default disable(0) */
 	uint32_t auto_scan_interval; /* auto scan interval(in second), defualt 0s */
} wlan_ext_auto_scan_param_t;

#define IPC_P2P_KPALIVE_PAYLOAD_LEN_MAX 36
#define IPC_P2P_WUP_PAYLOAD_LEN_MAX     36
#define IPC_P2P_SERVER_MAX 3
/**
 * @brief Parameter for WLAN_EXT_CMD_SET_P2P_SVR
 */
typedef struct wlan_ext_p2p_svr
{
    uint16_t  Enable;
    uint16_t  IPIdInit;
    uint32_t  TcpSeqInit;
    uint32_t  TcpAckInit;
    uint8_t   DstIPv4Addr[4];
    uint16_t  SrcTcpPort;
    uint16_t  DstTcpPort;
    uint8_t   DstMacAddr[6];
    uint16_t  Reserve;
} wlan_ext_p2p_svr_t;

typedef struct wlan_ext_p2p_svr_set
{
    uint8_t   EncHdrSize;
    uint8_t   EncTailSize;
    uint16_t  reserved1;
    uint8_t   SrcIPv4Addr[4];
    wlan_ext_p2p_svr_t  P2PServerCfgs[IPC_P2P_SERVER_MAX];
} wlan_ext_p2p_svr_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_P2P_WKP_CFG
 */
typedef struct wlan_ext_p2p_wkp_param_cfg
{
    uint16_t  Enable;
    uint16_t  PayloadLen;
    uint8_t   Payload[IPC_P2P_WUP_PAYLOAD_LEN_MAX];
}wlan_ext_p2p_wkp_param_cfg_t;

typedef struct wlan_ext_p2p_wkp_param_set
{
    wlan_ext_p2p_wkp_param_cfg_t P2PWkpParamCfgs[IPC_P2P_SERVER_MAX];
}wlan_ext_p2p_wkp_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_P2P_KPALIVE_CFG
 */
typedef struct wlan_ext_p2p_kpalive_param_cfg
{
    uint16_t  Enable;
    uint16_t  PayloadLen;
    uint8_t   Payload[IPC_P2P_KPALIVE_PAYLOAD_LEN_MAX];
}wlan_ext_p2p_kpalive_param_cfg_t;

typedef struct wlan_ext_p2p_kpalive_param_set
{
    uint8_t   KeepAlivePeriod_s;	  //unit:Second
    uint8_t   TxTimeOut_s;			  //unit:Second  Keep alive packet tx timeout value
    uint8_t   TxRetryLimit; 		  //keep alive packet tx retry limit
    uint8_t   reserved1;
    wlan_ext_p2p_kpalive_param_cfg_t P2PKeepAliveParamCfgs[IPC_P2P_SERVER_MAX];
}wlan_ext_p2p_kpalive_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_RCV_SPECIAL_FRM
 */
#define RECV_BROADCAST_FRAME_ENABLE              (1 << 0)
#define RECV_UNICAST_FRAME_ENABLE                (1 << 1)
#define RECV_MGMT_FRAME_ENABLE                   (1 << 2)
#define RECV_DATA_FRAME_ENABLE                   (1 << 3)
#define RECV_BROADCAST_A2_FILTER_ENABLE          (1 << 4)
#define RECV_BROADCAST_A3_FILTER_ENABLE          (1 << 5)
#define SEND_DUPLICATE_FRAME_TO_HOST_ENABLE      (1 << 6)
#define SEND_FRAME_NOT_BSSID_TO_HOST_ENABLE      (1 << 7)
#define D11_SUB_MGMT_ASRQ       (1 << (0+8))
#define D11_SUB_MGMT_ASRSP      (1 << (1+8))
#define D11_SUB_MGMT_RSRQ       (1 << (2+8))
#define D11_SUB_MGMT_RSRSP      (1 << (3+8))
#define D11_SUB_MGMT_PBRQ       (1 << (4+8))
#define D11_SUB_MGMT_PBRSP      (1 << (5+8))
#define D11_SUB_MGMT_BCN        (1 << (8+8))
#define D11_SUB_MGMT_ATIM       (1 << (9+8))
#define D11_SUB_MGMT_DAS        (1 << (10+8))
#define D11_SUB_MGMT_AUTH       (1 << (11+8))
#define D11_SUB_MGMT_DAUTH      (1 << (12+8))
#define D11_SUB_MGMT_ACTION     (1 << (13+8))

typedef struct wlan_ext_rcv_spec_frm_param_set
{
    uint8_t   Enable;
    uint8_t   Reserved;
    uint16_t  Reserved0;
    uint32_t  u32RecvSpecFrameCfg;
    uint8_t   MacAddrA2[6];
    uint8_t   MacAddrA3[6];
}wlan_ext_rcv_spec_frm_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SEND_RAW_FRM_CFG
 */
typedef struct wlan_ext_send_raw_frm_param_set
{
	uint8_t     Enable;
	uint16_t    u16SendRawFrameCfg;//reserved for now
	uint8_t     Reserved;
}wlan_ext_send_raw_frm_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_SNIFF_AUTO_WKP_CFG
 */
typedef struct wlan_ext_sniff_auto_wkp_param_set
{
    uint8_t   Enable;
    uint8_t   WakeupPeriod_s;         //uint:Second
    uint8_t   KeepActivePeriod_s;     //uint:Second
    uint8_t   Flag;                   //send received frame to host if set to 1
    uint32_t  StartTime;              //uint:microsecond
    uint16_t  ChannelNum;
}wlan_ext_sniff_auto_wkp_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_RCV_FRM_FILTER_CFG
 */
#define RCV_FRM_FILTER_FRAME_TYPE        (1 << 0)
#define RCV_FRM_FILTER_MAC_ADDRESS       (1 << 1)
#define RCV_FRM_FILTER_PAYLOAD           (1 << 2)

#define RCV_FRM_FILTER_MAC_ADDR_A1       (1 << 0)
#define RCV_FRM_FILTER_MAC_ADDR_A2       (1 << 1)
#define RCV_FRM_FILTER_MAC_ADDR_A3       (1 << 2)

#define RCV_FRM_FILTER_PAYLOAD_LEN_MAX     36
typedef struct rcv_frm_filter
{
    uint16_t  FilterEnable;
    uint16_t  AndOperationMask;
    uint16_t  OrOperationMask;
    uint8_t   FrameType;	//The same as MAC header define
    uint8_t   MacAddrMask;
    uint8_t   MacAddrA1[6];
    uint8_t   MacAddrA2[6];
    uint8_t   MacAddrA3[6];
	uint16_t  Reserved;
    uint16_t  PayloadOffset;
    uint16_t  PayloadLength;
    uint8_t   Payload[RCV_FRM_FILTER_PAYLOAD_LEN_MAX];
}rcv_frm_filter_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_BCN_WIN_CFG
 */
typedef struct wlan_ext_bcn_win_param_set
{
    uint32_t  BeaconWindowAdjAmpUs;
    uint8_t   BeaconWindowAdjStartNum;
    uint8_t   BeaconWindowAdjStopNum;
    uint8_t   BeaconWindowMaxStartNum;
	uint8_t   Reserved;
}wlan_ext_bcn_win_param_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_FORCE_B_RATE
 */
typedef struct wlan_ext_force_b_rate_set
{
    uint8_t   ForceBRateEnable;
    uint8_t   Force2mThreshold;
    uint8_t   Force1mThreshold;
	uint8_t   Reserved;
}wlan_ext_force_b_rate_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_SET_POWER_LEVEL_TAB
 */
#define POWER_LEVEL_TAB_USE_LENGTH     11
typedef struct wlan_ext_power_level_tab_set
{
    uint16_t   PowerTab[POWER_LEVEL_TAB_USE_LENGTH];
}wlan_ext_power_level_tab_set_t;

/**
 * @brief Parameter for WLAN_EXT_CMD_GET_POWER_LEVEL_TAB
 */
#define POWER_LEVEL_TAB_TYPE_MAX       0
#define POWER_LEVEL_TAB_TYPE_CUR       1
#define POWER_LEVEL_TAB_TYPE_USER      2
typedef struct wlan_ext_power_level_tab_get
{
    uint16_t   PowerTabType;
    uint16_t   PowerTab[POWER_LEVEL_TAB_USE_LENGTH];
}wlan_ext_power_level_tab_get_t;


int wlan_ext_request(struct netif *nif, wlan_ext_cmd_t cmd, uint32_t param);

#ifdef __cplusplus
}
#endif

#endif /* _NET_WLAN_WLAN_EXT_REQ_H_ */
