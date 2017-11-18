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

#include "atcmd/at_command.h"
#include "at_private.h"
#include "at_debug.h"

extern s32 at_get_value(char *strbuf, s32 pt, void *pvar, s32 vsize);
extern s32 at_set_value(s32 pt, void *pvar, s32 vsize, at_value_t *value);

static at_status_t at_sts;
static at_peer_t dummy_peer;

static const at_var_descriptor_t at_sts_table[] = {
	//{"version",					APT_TEXT,		APO_RO,		&at_sts.version,					sizeof(at_sts.version),				NULL},
	//{"reset_reason",			APT_DI,			APO_RO,		&at_sts.reset_reason,				sizeof(at_sts.reset_reason),		NULL},
	//{"conf_flag",				APT_DI,			APO_RO,		&at_sts.conf_flag,					sizeof(at_sts.conf_flag),			NULL},
	//{"system_uptime",			APT_DI,			APO_RO,		&at_sts.system_uptime,				sizeof(at_sts.system_uptime),		NULL},
	//{"system_sleeptime",		APT_DI,			APO_RO,		&at_sts.system_sleeptime,			sizeof(at_sts.system_sleeptime),	NULL},
	//{"gpio_enable",				APT_DI,			APO_RO,		&at_sts.gpio_enable,				sizeof(at_sts.gpio_enable),			NULL},
	//{"captiveportal",			APT_DI,			APO_RO,		&at_sts.captiveportal,				sizeof(at_sts.captiveportal),		NULL},
	//{"wifi_state",				APT_DI,			APO_RO,		&at_sts.wifi_state,					sizeof(at_sts.wifi_state),			NULL},
	//{"wifi_bssid",				APT_HEX,		APO_RO,		&at_sts.wifi_bssid,					sizeof(at_sts.wifi_bssid),			NULL},
	//{"wifi_aid",				APT_DI,			APO_RO,		&at_sts.wifi_aid,					sizeof(at_sts.wifi_aid),			NULL},
	//{"wifi_channelnum",			APT_DI,			APO_RO,		&at_sts.wifi_channelnum,			sizeof(at_sts.wifi_channelnum),		NULL},
	//{"wifi_sup_rate_mask",		APT_HI,			APO_RO,		&at_sts.wifi_sup_rate_mask,			sizeof(at_sts.wifi_sup_rate_mask),	NULL},
	//{"wifi_bas_rate_mask",		APT_HI,			APO_RO,		&at_sts.wifi_bas_rate_mask,			sizeof(at_sts.wifi_bas_rate_mask),	NULL},
	//{"wifi_chan_activity2",		APT_HI,			APO_RO,		&at_sts.wifi_chan_activity2,		sizeof(at_sts.wifi_chan_activity2),	NULL},
	//{"wifi_max_tx_power",		APT_DI,			APO_RO,		&at_sts.wifi_max_tx_power,			sizeof(at_sts.wifi_max_tx_power),	NULL},
	//{"wifi_reg_country",		APT_DI,			APO_RO,		&at_sts.wifi_reg_country,			sizeof(at_sts.wifi_reg_country),	NULL},
	//{"wifi_dtim_period",		APT_DI,			APO_RO,		&at_sts.wifi_dtim_period,			sizeof(at_sts.wifi_dtim_period),	NULL},
	//{"wifi_sleeping",			APT_DI,			APO_RO,		&at_sts.wifi_sleeping,				sizeof(at_sts.wifi_sleeping),		NULL},
	{"wifi_num_assoc",			APT_DI,			APO_RO,		&at_sts.wifi_num_assoc,				sizeof(at_sts.wifi_num_assoc),		NULL},
	{"ip_ipaddr",				APT_IP,			APO_RO,		&at_sts.ip_ipaddr,					sizeof(at_sts.ip_ipaddr),			NULL},
	{"ip_netmask",				APT_IP,			APO_RO,		&at_sts.ip_netmask,					sizeof(at_sts.ip_netmask),			NULL},
	{"ip_gw",					APT_IP,			APO_RO,		&at_sts.ip_gw,						sizeof(at_sts.ip_gw),				NULL},
	{"ip_dns",					APT_IP,			APO_RO,		&at_sts.ip_dns,						sizeof(at_sts.ip_dns),				NULL},
	{"ip_sock_open",			APT_DI,			APO_RO,		&at_sts.ip_sock_open,				sizeof(at_sts.ip_sock_open),		NULL},
	{"ip_sockd_port",			APT_DI,			APO_RO,		&at_sts.ip_sockd_port,				sizeof(at_sts.ip_sockd_port),		NULL},
	//{"free_heap",				APT_DI,			APO_RO,		&at_sts.free_heap,					sizeof(at_sts.free_heap),			NULL},
	//{"min_heap",				APT_DI,			APO_RO,		&at_sts.min_heap,					sizeof(at_sts.min_heap),			NULL},
	//{"current_time",			APT_DI,			APO_RO,		&at_sts.current_time,				sizeof(at_sts.current_time),		NULL}
};

static const at_var_descriptor_t at_peer_table[] = {
	{"link_id",					APT_DI,			APO_RO,		&dummy_peer.link_id,				sizeof(dummy_peer.link_id),			NULL},
	{"state",					APT_DI,			APO_RO,		&dummy_peer.state,					sizeof(dummy_peer.state),			NULL},
	{"addr",					APT_HEX,		APO_RO,		&dummy_peer.addr,					sizeof(dummy_peer.addr),			NULL},
	{"last_rx",					APT_DI,			APO_RO,		&dummy_peer.last_rx,				sizeof(dummy_peer.last_rx),			NULL},
	{"last_tx",					APT_DI,			APO_RO,		&dummy_peer.last_tx,				sizeof(dummy_peer.last_tx),			NULL},
	{"rx_drops",				APT_DI,			APO_RO,		&dummy_peer.rx_drops,				sizeof(dummy_peer.rx_drops),		NULL},
	{"tx_drops",				APT_DI,			APO_RO,		&dummy_peer.tx_drops,				sizeof(dummy_peer.tx_drops),		NULL},
	{"rx_pkts",					APT_DI,			APO_RO,		&dummy_peer.rx_pkts,				sizeof(dummy_peer.rx_pkts),			NULL},
	{"tx_pkts",					APT_DI,			APO_RO,		&dummy_peer.tx_pkts,				sizeof(dummy_peer.tx_pkts),			NULL},
	{"tx_errs",					APT_DI,			APO_RO,		&dummy_peer.tx_errs,				sizeof(dummy_peer.tx_errs),			NULL},
	{"rate_mask",				APT_DI,			APO_RO,		&dummy_peer.rate_mask,				sizeof(dummy_peer.rate_mask),		NULL},
	{"cur_rate_idx",			APT_DI,			APO_RO,		&dummy_peer.cur_rate_idx,			sizeof(dummy_peer.cur_rate_idx),	NULL},
	{"cur_rate_ok",				APT_DI,			APO_RO,		&dummy_peer.cur_rate_ok,			sizeof(dummy_peer.cur_rate_ok),		NULL},
	{"cur_rate_fail",			APT_DI,			APO_RO,		&dummy_peer.cur_rate_fail,			sizeof(dummy_peer.cur_rate_fail),	NULL},
	{"tx_consec_fail",			APT_DI,			APO_RO,		&dummy_peer.tx_consec_fail,			sizeof(dummy_peer.tx_consec_fail),	NULL},
	{"rx_seqnum",				APT_HI,			APO_RO,		&dummy_peer.rx_seqnum,				sizeof(dummy_peer.rx_seqnum),		NULL},
	{"rx_seqnum_mc",			APT_HI,			APO_RO,		&dummy_peer.rx_seqnum_mc,			sizeof(dummy_peer.rx_seqnum_mc),	NULL},
	{"rx_rssi",					APT_DI,			APO_RO,		&dummy_peer.rx_rssi,				sizeof(dummy_peer.rx_rssi),			NULL},
	{"rx_rateidx",				APT_DI,			APO_RO,		&dummy_peer.rx_rateidx,				sizeof(dummy_peer.rx_rateidx),		NULL},
	{"setprot",					APT_DI,			APO_RO,		&dummy_peer.setprot,				sizeof(dummy_peer.setprot),			NULL},
	{"listen_interval",			APT_DI,			APO_RO,		&dummy_peer.listen_interval,		sizeof(dummy_peer.listen_interval),	NULL},
	{"capinfo",					APT_HI,			APO_RO,		&dummy_peer.capinfo,				sizeof(dummy_peer.capinfo),			NULL},
};

AT_ERROR_CODE at_status(char *sts_var)
{
	char strbuf[AT_PARA_MAX_SIZE*4];
	s32 i;
	at_callback_para_t para;

	para.sts = &at_sts;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_STATUS,&para,NULL);
	}

	if (sts_var == NULL) { /* display all status variable*/
		for (i = 0; i < TABLE_SIZE(at_sts_table); i++) {
			at_get_value(strbuf, at_sts_table[i].pt, at_sts_table[i].pvar, at_sts_table[i].vsize);

			at_dump("# %s = %s\r\n", at_sts_table[i].key, strbuf);
		}

		return AEC_OK; /* succeed */
	}
	else { /* display specified status variable */
		for (i = 0; i < TABLE_SIZE(at_sts_table); i++) {
			if (!strcmp(sts_var, at_sts_table[i].key)) {
				at_get_value(strbuf, at_sts_table[i].pt, at_sts_table[i].pvar, at_sts_table[i].vsize);

				at_dump("# %s = %s\r\n", at_sts_table[i].key, strbuf);

				return AEC_OK; /* succeed */
			}
		}
	}

	return AEC_NOT_FOUND; /* not found */
}

AT_ERROR_CODE at_setsts(char *key,at_value_t *value)
{
	s32 i;

	if (key == NULL || value == NULL) {
		return AEC_NULL_POINTER; /* null pointer */
	}

	for (i = 0; i < TABLE_SIZE(at_sts_table); i++) {
		if (!strcmp(key, at_sts_table[i].key)) {
			at_set_value(at_sts_table[i].pt, at_sts_table[i].pvar, at_sts_table[i].vsize, value);

			return AEC_OK; /* succeed */
		}
	}

	return AEC_NOT_FOUND; /* not found */
}

AT_ERROR_CODE at_peer(s32 pn, at_peer_t *peer, char *var)
{
	char strbuf[AT_PARA_MAX_SIZE*4];
	s32 i;

	if (peer == NULL) {
		return AEC_NULL_POINTER; /* null pointer */
	}

	if (var == NULL) { /* display all peer variable*/
		for (i = 0; i < TABLE_SIZE(at_peer_table); i++) {
			at_get_value(strbuf, at_peer_table[i].pt, at_peer_table[i].pvar, at_peer_table[i].vsize);

			at_dump("# %s = %s\r\n", at_peer_table[i].key, strbuf);
		}

		return AEC_OK; /* succeed */
	}
	else { /* display specified peer variable */
		for (i = 0; i < TABLE_SIZE(at_peer_table); i++) {
			if (!strcmp(var, at_peer_table[i].key)) {
				at_get_value(strbuf, at_peer_table[i].pt, at_peer_table[i].pvar, at_peer_table[i].vsize);

				at_dump("# %s = %s\r\n", at_peer_table[i].key, strbuf);

				return AEC_OK; /* succeed */
			}
		}
	}

	return AEC_NOT_FOUND; /* not found */
}
