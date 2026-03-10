/*
 * Copyright (c) 2026 Krzysztof Gawrys
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP-Hosted-MCU SDIO WiFi driver — Stage 3: net_if + wifi_mgmt integration.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_nm.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "esp_hosted_sdio.h"

LOG_MODULE_DECLARE(esp_hosted_sdio, LOG_LEVEL_INF);

/* ── pb encode callback — raw bytes field (ssid, password, …) ───────────── */

struct bytes_enc_ctx {
	const uint8_t *data;
	size_t         len;
};

static bool encode_bytes_cb(pb_ostream_t *stream, const pb_field_t *field,
			    void * const *arg)
{
	const struct bytes_enc_ctx *ctx = *arg;

	if (!pb_encode_tag_for_field(stream, field)) {
		return false;
	}
	return pb_encode_string(stream, ctx->data, ctx->len);
}

/* ── pb decode callback — raw bytes into fixed buffer ───────────────────── */

struct bytes_dec_ctx {
	uint8_t *buf;
	size_t   max;
	size_t  *len_out;
};

static bool decode_bytes_cb(pb_istream_t *stream, const pb_field_t *field,
			    void **arg)
{
	struct bytes_dec_ctx *ctx = *arg;
	size_t n = MIN(stream->bytes_left, ctx->max);

	if (!pb_read(stream, ctx->buf, n)) {
		return false;
	}
	if (ctx->len_out) {
		*ctx->len_out = n;
	}
	return pb_read(stream, NULL, stream->bytes_left); /* drain remainder */
}

/* ── MAC address retrieval ───────────────────────────────────────────────── */

int esp_hosted_get_mac(const struct device *dev)
{
	struct esp_hosted_sdio_data *data = dev->data;
	Rpc req = Rpc_init_zero;
	Rpc resp = Rpc_init_zero;

	req.msg_type      = RpcType_Req;
	req.msg_id        = RpcId_Req_GetMACAddress;
	req.which_payload = Rpc_req_get_mac_address_tag;
	req.payload.req_get_mac_address.mode = 0; /* 0 = WIFI_IF_STA */

	uint8_t mac_buf[6] = {0};
	size_t  mac_len    = 0;
	struct bytes_dec_ctx mac_ctx = {
		.buf     = mac_buf,
		.max     = sizeof(mac_buf),
		.len_out = &mac_len,
	};

	resp.which_payload = Rpc_resp_get_mac_address_tag;
	resp.payload.resp_get_mac_address.mac.funcs.decode = decode_bytes_cb;
	resp.payload.resp_get_mac_address.mac.arg          = &mac_ctx;

	int ret = esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(3000));

	if (ret) {
		LOG_ERR("GetMACAddress RPC failed: %d", ret);
		goto fallback;
	}
	if (resp.payload.resp_get_mac_address.resp != 0) {
		LOG_ERR("GetMACAddress slave error: %d",
			resp.payload.resp_get_mac_address.resp);
		goto fallback;
	}
	if (mac_len == 6) {
		memcpy(data->mac_addr, mac_buf, 6);
		LOG_INF("MAC: %02x:%02x:%02x:%02x:%02x:%02x",
			mac_buf[0], mac_buf[1], mac_buf[2],
			mac_buf[3], mac_buf[4], mac_buf[5]);
		if (data->iface) {
			net_if_set_link_addr(data->iface, data->mac_addr, 6,
					     NET_LINK_ETHERNET);
		}
		return 0;
	}
	LOG_WRN("GetMACAddress: unexpected mac len=%zu", mac_len);

fallback:
	/* Locally-administered default MAC */
	data->mac_addr[0] = 0x02;
	data->mac_addr[1] = 0xDE;
	data->mac_addr[2] = 0xAD;
	data->mac_addr[3] = 0xBE;
	data->mac_addr[4] = 0xEF;
	data->mac_addr[5] = 0x01;
	return ret ? ret : -EIO;
}

/* ── RX path (called from rx_thread when STA/AP frame arrives) ──────────── */

void esp_hosted_iface_recv(const struct device *dev, uint8_t *payload,
			   uint16_t payload_len)
{
	struct esp_hosted_sdio_data *data = dev->data;
	struct net_if *iface = data->iface;

	if (!iface || !net_if_flag_is_set(iface, NET_IF_UP)) {
		return;
	}

	struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
		iface, payload_len, AF_UNSPEC, 0, K_MSEC(100));

	if (!pkt) {
		LOG_ERR("No RX pkt buf (len=%u)", payload_len);
		return;
	}

	if (net_pkt_write(pkt, payload, payload_len) < 0) {
		LOG_ERR("net_pkt_write failed");
		net_pkt_unref(pkt);
		return;
	}

	if (net_recv_data(iface, pkt) < 0) {
		LOG_ERR("net_recv_data failed");
		net_pkt_unref(pkt);
	}
}

/* ── Unsolicited SERIAL_IF event handler ─────────────────────────────────── */

void esp_hosted_process_serial_event(const struct device *dev,
				     const uint8_t *pb_buf, uint16_t pb_len)
{
	struct esp_hosted_sdio_data *data = dev->data;
	Rpc evt = Rpc_init_zero;

	pb_istream_t is = pb_istream_from_buffer(pb_buf, pb_len);

	if (!pb_decode(&is, Rpc_fields, &evt)) {
		LOG_WRN("event pb_decode failed: %s", PB_GET_ERROR(&is));
		return;
	}

	LOG_DBG("SERIAL_IF event: msg_id=%d", evt.msg_id);

	if (!data->iface) {
		return;
	}

	switch (evt.msg_id) {
	case RpcId_Event_StaConnected:
		LOG_INF("StaConnected event");
		data->state = WIFI_STATE_COMPLETED;
		net_if_dormant_off(data->iface);
		wifi_mgmt_raise_connect_result_event(data->iface, 0);
		break;

	case RpcId_Event_StaDisconnected:
		LOG_INF("StaDisconnected event");
		data->state = WIFI_STATE_DISCONNECTED;
		net_if_dormant_on(data->iface);
		wifi_mgmt_raise_disconnect_result_event(data->iface, 0);
		break;

	default:
		LOG_DBG("SERIAL_IF unhandled event msg_id=%d", evt.msg_id);
		break;
	}
}

/* ── net_if iface init ───────────────────────────────────────────────────── */

static void esp_hosted_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct esp_hosted_sdio_data *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);
	struct wifi_nm_instance *nm = wifi_nm_get_instance("esp_nm");

	data->iface = iface;
	data->state = WIFI_STATE_DISCONNECTED;

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;

	/* Placeholder MAC — updated via GetMACAddress RPC after ESPInit */
	data->mac_addr[0] = 0x02;
	data->mac_addr[1] = 0xDE;
	data->mac_addr[2] = 0xAD;
	data->mac_addr[3] = 0xBE;
	data->mac_addr[4] = 0xEF;
	data->mac_addr[5] = 0x01;

	net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_dormant_on(iface);
	net_if_carrier_on(iface);

	if (nm) {
		wifi_nm_register_mgd_type_iface(nm, WIFI_TYPE_STA, iface);
	}
}

/* ── TX path ─────────────────────────────────────────────────────────────── */

static int esp_hosted_iface_send(const struct device *dev, struct net_pkt *pkt)
{
	size_t pkt_len = net_pkt_get_len(pkt);

	if (pkt_len == 0 ||
	    pkt_len > (size_t)(ESP_TX_BUFFER_SIZE - ESP_FRAME_HEADER_SIZE)) {
		return -EINVAL;
	}

	uint8_t *frame = k_malloc(pkt_len);

	if (!frame) {
		return -ENOMEM;
	}

	if (net_pkt_read(pkt, frame, pkt_len) < 0) {
		LOG_ERR("net_pkt_read failed");
		k_free(frame);
		return -EIO;
	}

	int ret = esp_hosted_sdio_tx(dev, ESP_STA_IF, 0, frame, (uint16_t)pkt_len);

	k_free(frame);
	return ret;
}

/* ── Scan ────────────────────────────────────────────────────────────────── */

struct scan_ctx {
	struct net_if   *iface;
	scan_result_cb_t cb;
};

static bool decode_ap_ssid_cb(pb_istream_t *stream, const pb_field_t *field,
			      void **arg)
{
	struct wifi_scan_result *r = *arg;
	size_t n = MIN(stream->bytes_left, (size_t)WIFI_SSID_MAX_LEN);

	if (!pb_read(stream, (uint8_t *)r->ssid, n)) {
		return false;
	}
	r->ssid_length = (uint8_t)n;
	return pb_read(stream, NULL, stream->bytes_left);
}

static bool decode_ap_bssid_cb(pb_istream_t *stream, const pb_field_t *field,
			       void **arg)
{
	struct wifi_scan_result *r = *arg;
	size_t n = MIN(stream->bytes_left, (size_t)WIFI_MAC_ADDR_LEN);

	if (!pb_read(stream, r->mac, n)) {
		return false;
	}
	r->mac_length = (uint8_t)n;
	return pb_read(stream, NULL, stream->bytes_left);
}

static int authmode_to_security(int32_t authmode)
{
	switch (authmode) {
	case 0: return WIFI_SECURITY_TYPE_NONE;
	case 1: return WIFI_SECURITY_TYPE_WEP;
	case 2: return WIFI_SECURITY_TYPE_WPA_PSK;
	case 3: return WIFI_SECURITY_TYPE_PSK;        /* WPA2-PSK */
	case 4: return WIFI_SECURITY_TYPE_PSK;        /* WPA/WPA2-PSK */
	case 7: return WIFI_SECURITY_TYPE_SAE;        /* WPA3-SAE */
	case 8: return WIFI_SECURITY_TYPE_SAE;        /* WPA2/WPA3 */
	default: return WIFI_SECURITY_TYPE_UNKNOWN;
	}
}

static bool decode_ap_record_cb(pb_istream_t *stream, const pb_field_t *field,
				void **arg)
{
	struct scan_ctx *ctx = *arg;
	struct wifi_scan_result result = {0};
	wifi_ap_record rec = wifi_ap_record_init_zero;

	rec.ssid.funcs.decode  = decode_ap_ssid_cb;
	rec.ssid.arg           = &result;
	rec.bssid.funcs.decode = decode_ap_bssid_cb;
	rec.bssid.arg          = &result;

	if (!pb_decode(stream, wifi_ap_record_fields, &rec)) {
		LOG_ERR("ap_record decode failed");
		return false;
	}

	result.channel  = (uint8_t)rec.primary;
	result.rssi     = (int8_t)rec.rssi;
	result.security = authmode_to_security(rec.authmode);

	ctx->cb(ctx->iface, 0, &result);
	return true;
}

static int esp_hosted_scan(const struct device *dev,
			   struct wifi_scan_params *params,
			   scan_result_cb_t cb)
{
	struct net_if *iface = net_if_lookup_by_dev(dev);
	Rpc req = Rpc_init_zero;
	Rpc resp = Rpc_init_zero;
	int ret;

	ARG_UNUSED(params);

	/* Step 1: WifiScanStart */
	req.msg_type      = RpcType_Req;
	req.msg_id        = RpcId_Req_WifiScanStart;
	req.which_payload = Rpc_req_wifi_scan_start_tag;

	ret = esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(5000));
	if (ret) {
		LOG_ERR("WifiScanStart RPC failed: %d", ret);
		return ret;
	}

	/* Wait for scan to finish */
	k_msleep(3000);

	/* Step 2: WifiScanGetApRecords */
	memset(&req,  0, sizeof(req));
	memset(&resp, 0, sizeof(resp));

	req.msg_type      = RpcType_Req;
	req.msg_id        = RpcId_Req_WifiScanGetApRecords;
	req.which_payload = Rpc_req_wifi_scan_get_ap_records_tag;
	req.payload.req_wifi_scan_get_ap_records.number = 20;

	struct scan_ctx sctx = { .iface = iface, .cb = cb };

	resp.which_payload = Rpc_resp_wifi_scan_get_ap_records_tag;
	resp.payload.resp_wifi_scan_get_ap_records.ap_records.funcs.decode =
		decode_ap_record_cb;
	resp.payload.resp_wifi_scan_get_ap_records.ap_records.arg = &sctx;

	ret = esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(5000));
	if (ret) {
		LOG_ERR("WifiScanGetApRecords RPC failed: %d", ret);
		return ret;
	}

	cb(iface, 0, NULL); /* end-of-scan sentinel */
	return 0;
}

/* ── Connect ─────────────────────────────────────────────────────────────── */

static int esp_hosted_connect(const struct device *dev,
			      struct wifi_connect_req_params *params)
{
	Rpc req = Rpc_init_zero;
	Rpc resp = Rpc_init_zero;
	int ret;

	struct bytes_enc_ctx ssid_ctx = {
		.data = params->ssid,
		.len  = params->ssid_length,
	};
	struct bytes_enc_ctx pass_ctx = {
		.data = (params->psk && params->psk_length)
				? (const uint8_t *)params->psk
				: (const uint8_t *)"",
		.len  = params->psk_length,
	};

	/* WifiSetConfig — set SSID + password for STA mode */
	req.msg_type      = RpcType_Req;
	req.msg_id        = RpcId_Req_WifiSetConfig;
	req.which_payload = Rpc_req_wifi_set_config_tag;
	req.payload.req_wifi_set_config.iface   = 0; /* 0 = WIFI_IF_STA */
	req.payload.req_wifi_set_config.has_cfg = true;
	req.payload.req_wifi_set_config.cfg.which_u = wifi_config_sta_tag;
	req.payload.req_wifi_set_config.cfg.u.sta.ssid.funcs.encode    = encode_bytes_cb;
	req.payload.req_wifi_set_config.cfg.u.sta.ssid.arg             = &ssid_ctx;
	req.payload.req_wifi_set_config.cfg.u.sta.password.funcs.encode = encode_bytes_cb;
	req.payload.req_wifi_set_config.cfg.u.sta.password.arg          = &pass_ctx;

	ret = esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(3000));
	if (ret) {
		LOG_ERR("WifiSetConfig RPC failed: %d", ret);
		return ret;
	}
	if (resp.payload.resp_wifi_set_config.resp != 0) {
		LOG_ERR("WifiSetConfig slave err: %d",
			resp.payload.resp_wifi_set_config.resp);
		return -EIO;
	}

	/* WifiConnect */
	memset(&req,  0, sizeof(req));
	memset(&resp, 0, sizeof(resp));

	req.msg_type      = RpcType_Req;
	req.msg_id        = RpcId_Req_WifiConnect;
	req.which_payload = Rpc_req_wifi_connect_tag;

	ret = esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(3000));
	if (ret) {
		LOG_ERR("WifiConnect RPC failed: %d", ret);
		return ret;
	}
	if (resp.payload.resp_wifi_connect.resp != 0) {
		LOG_ERR("WifiConnect slave err: %d",
			resp.payload.resp_wifi_connect.resp);
		wifi_mgmt_raise_connect_result_event(
			net_if_lookup_by_dev(dev),
			resp.payload.resp_wifi_connect.resp);
		return -EIO;
	}

	/* Connection confirmed by async StaConnected event in rx_thread */
	return 0;
}

/* ── Disconnect ──────────────────────────────────────────────────────────── */

static int esp_hosted_disconnect(const struct device *dev)
{
	struct esp_hosted_sdio_data *data = dev->data;
	Rpc req = Rpc_init_zero;
	Rpc resp = Rpc_init_zero;
	int ret;

	req.msg_type      = RpcType_Req;
	req.msg_id        = RpcId_Req_WifiDisconnect;
	req.which_payload = Rpc_req_wifi_disconnect_tag;

	ret = esp_hosted_sdio_rpc_call(dev, &req, &resp, K_MSEC(3000));
	if (ret) {
		LOG_ERR("WifiDisconnect RPC failed: %d", ret);
	}

	data->state = WIFI_STATE_DISCONNECTED;
	net_if_dormant_on(data->iface);
	wifi_mgmt_raise_disconnect_result_event(data->iface, ret ? -EIO : 0);
	return ret;
}

/* ── AP mode — not supported ─────────────────────────────────────────────── */

static int esp_hosted_ap_enable(const struct device *dev,
				struct wifi_connect_req_params *params)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(params);
	return -ENOTSUP;
}

static int esp_hosted_ap_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

/* ── Interface status ────────────────────────────────────────────────────── */

static int esp_hosted_iface_status(const struct device *dev,
				   struct wifi_iface_status *status)
{
	const struct esp_hosted_sdio_data *data = dev->data;

	status->state          = data->state;
	status->band           = WIFI_FREQ_BAND_2_4_GHZ;
	status->link_mode      = WIFI_LINK_MODE_UNKNOWN;
	status->iface_mode     = WIFI_MODE_INFRA;
	status->mfp            = WIFI_MFP_DISABLE;
	status->wpa3_ent_type  = WIFI_WPA3_ENTERPRISE_NA;
	return 0;
}

/* ── WiFi management ops ─────────────────────────────────────────────────── */

static const struct wifi_mgmt_ops esp_hosted_sdio_mgmt = {
	.scan         = esp_hosted_scan,
	.connect      = esp_hosted_connect,
	.disconnect   = esp_hosted_disconnect,
	.ap_enable    = esp_hosted_ap_enable,
	.ap_disable   = esp_hosted_ap_disable,
	.iface_status = esp_hosted_iface_status,
};

/* ── WiFi offload API ────────────────────────────────────────────────────── */

const struct net_wifi_mgmt_offload esp_hosted_sdio_api = {
	.wifi_iface.iface_api.init = esp_hosted_iface_init,
	.wifi_iface.send           = esp_hosted_iface_send,
	.wifi_mgmt_api             = &esp_hosted_sdio_mgmt,
};

/* ── WiFi NM instance + connectivity binding ─────────────────────────────── */

DEFINE_WIFI_NM_INSTANCE(esp_nm, &esp_hosted_sdio_mgmt);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));

/* ── Device + net_if registration ───────────────────────────────────────── */

NET_DEVICE_DT_INST_DEFINE(0,
			  esp_hosted_sdio_init, NULL,
			  &esp_hosted_sdio_data_0,
			  &esp_hosted_sdio_cfg_0,
			  CONFIG_WIFI_ESP_HOSTED_SDIO_INIT_PRIORITY,
			  &esp_hosted_sdio_api,
			  ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2),
			  NET_ETH_MTU);
