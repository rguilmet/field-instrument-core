#include "fic_ble_internal.h"
#include "../rtc/fic_pcf85063.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <stdio.h>
#include <string.h>

static void build_feature_array(const FicBleRuntimeState_t *st, JsonArray feats)
{
    uint32_t mask = st->cfg.feature_mask;
    if (mask == 0U) {
        mask = FICP_FEAT_SETTINGS_SYNC |
               FICP_FEAT_CONTROL_HANDOFF |
               FICP_FEAT_FILE_SYNC |
               FICP_FEAT_RTC_SYNC |
               FICP_FEAT_SCREENSHOT;
        if (st->cfg.enable_stream_rx) mask |= FICP_FEAT_STREAM_RX;
        if (st->cfg.enable_stream_tx) mask |= FICP_FEAT_STREAM_TX;
    }
    if ((mask & FICP_FEAT_SETTINGS_SYNC) != 0U)   feats.add("settings_sync");
    if ((mask & FICP_FEAT_CONTROL_HANDOFF) != 0U) feats.add("control_handoff");
    if ((mask & FICP_FEAT_FILE_SYNC) != 0U)       feats.add("file_sync");
    if ((mask & FICP_FEAT_STREAM_RX) != 0U)       feats.add("stream_rx");
    if ((mask & FICP_FEAT_STREAM_TX) != 0U)       feats.add("stream_tx");
    if ((mask & FICP_FEAT_RTC_SYNC) != 0U)        feats.add("rtc_sync");
    if ((mask & FICP_FEAT_SCREENSHOT) != 0U)      feats.add("screenshot");
}

static void handle_sys_hello(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    JsonDocument rx_doc;
    const char *client_name = "unknown";
    const char *client_ver  = "0.0.0";
    if (frame->payload && frame->payload_len > 0U) {
        if (deserializeJson(rx_doc, frame->payload, frame->payload_len) == DeserializationError::Ok) {
            client_name = rx_doc["client_name"] | "unknown";
            client_ver  = rx_doc["client_ver"] | "0.0.0";
        }
    }
    fic_ble_log("[BLE] HELLO from '%s' v%s (MTU=%u)\n",
                client_name, client_ver, (unsigned)st->session.peer_mtu);

    JsonDocument caps;
    caps["ficp_ver"]     = FICP_VERSION_STR;
    caps["device_type"]  = st->cfg.device_type ? st->cfg.device_type : "fic_device";
    caps["device_name"]  = st->cfg.device_name ? st->cfg.device_name : "FIC-Instrument";
    caps["fw_ver"]       = st->cfg.fw_version ? st->cfg.fw_version : "0.0.0";
    caps["core_ver"]     = "1.2.0";
    caps["board"]        = st->cfg.board_name ? st->cfg.board_name : "waveshare_s3_35b";
    caps["auth_mode"]    = st->cfg.auth_mode;
    caps["state_rev"]    = st->session.state_rev;
    caps["control_mode"] = ficp_control_mode_to_str(st->session.control_mode);
    caps["mtu_max"]      = FICP_ATT_MTU_PREFERRED;
    caps["telem_hz_max"] = (st->cfg.telem_hz_default > 0U) ? st->cfg.telem_hz_default : 10U;
    JsonArray feats      = caps["features"].to<JsonArray>();
    build_feature_array(st, feats);

    (void)fic_ble_send_json_frame(FICP_MSG_SYS_CAPS, caps);
    if ((frame->flags & FICP_FLAG_ACK_REQ) != 0U) {
        fic_ble_send_ack(frame->seq, NULL);
    }
}

static void handle_sys_heartbeat(const FicpFrameView_t *frame)
{
    uint32_t peer_up = 0U;
    uint16_t peer_rev = 0U;
    uint8_t  peer_mode = 0U;
    uint8_t  peer_flags = 0U;
    (void)ficp_unpack_heartbeat(frame->payload, frame->payload_len,
                                &peer_up, &peer_rev, &peer_mode, &peer_flags);
    if ((frame->flags & FICP_FLAG_ACK_REQ) != 0U) {
        fic_ble_send_ack(frame->seq, NULL);
    }
}

static void handle_sys_health_req(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    FicBleHealthInfo_t info = {};
    info.dram_free    = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    info.dram_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    info.psram_free   = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    info.sd_ok        = (st->cfg.primary_fs != nullptr);
    info.uptime_s     = (uint32_t)(millis() / 1000UL);
    if (st->cfg.health_cb != nullptr) {
        st->cfg.health_cb(&info);
    }

    JsonDocument rsp;
    rsp["dram_free"]    = info.dram_free;
    rsp["dram_largest"] = info.dram_largest;
    rsp["psram_free"]   = info.psram_free;
    rsp["vbat_mv"]      = info.vbat_mv;
    rsp["batt_pct"]     = info.batt_pct;
    rsp["i2c0_errs"]    = info.i2c0_errs;
    rsp["i2c1_errs"]    = info.i2c1_errs;
    rsp["sd_ok"]        = info.sd_ok;
    rsp["uptime_s"]     = info.uptime_s;
    (void)fic_ble_send_json_frame(FICP_MSG_SYS_HEALTH_RSP, rsp);
    if ((frame->flags & FICP_FLAG_ACK_REQ) != 0U) {
        fic_ble_send_ack(frame->seq, NULL);
    }
}

static void handle_sys_rtc_set(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    int64_t utc_ms = -1LL;

    if ((frame->flags & FICP_FLAG_IS_JSON) != 0U && frame->payload && frame->payload_len > 0U) {
        JsonDocument doc;
        if (deserializeJson(doc, frame->payload, frame->payload_len) == DeserializationError::Ok) {
            utc_ms = doc["utc_ms"] | -1LL;
        }
    } else if (frame->payload && frame->payload_len >= 8U) {
        uint64_t raw = 0ULL;
        for (uint8_t i = 0U; i < 8U; i++) {
            raw |= ((uint64_t)frame->payload[i]) << (8U * i);
        }
        utc_ms = (int64_t)raw;
    }

    FicpUtcCalendar_t cal = {};
    if (!ficp_unix_ms_to_utc_calendar(utc_ms, &cal)) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid utc_ms");
        return;
    }

    bool ok = true;
    if (st->cfg.rtc_set_cb != nullptr) {
        ok = st->cfg.rtc_set_cb(&cal);
    } else {
        fic_rtc_set_from_utc(cal.year, cal.month, cal.day, cal.hour, cal.minute, cal.second, true);
    }
    if (!ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "RTC update failed");
        return;
    }
    fic_ble_send_ack(frame->seq, NULL);
}

static void handle_sys_screenshot_req(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    if (st->cfg.screenshot_cb == nullptr) {
        fic_ble_send_nack(frame->seq, FICP_ERR_UNSUPPORTED_MSG, "Screenshot callback not registered");
        return;
    }
    char path[96] = {0};
    if (!st->cfg.screenshot_cb(path, sizeof(path))) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Screenshot capture failed");
        return;
    }
    JsonDocument d;
    d["path"] = path;
    String raw;
    serializeJson(d, raw);
    fic_ble_send_ack(frame->seq, raw.c_str());
}

static void handle_state_snapshot_req(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    const char *section = "all";
    JsonDocument rx_doc;
    if (frame->payload && frame->payload_len > 0U) {
        if (deserializeJson(rx_doc, frame->payload, frame->payload_len) == DeserializationError::Ok) {
            section = rx_doc["section"] | "all";
        }
    }

    JsonDocument rsp;
    rsp["state_rev"]  = st->session.state_rev;
    rsp["schema_ver"] = (st->cfg.schema_ver > 0U) ? st->cfg.schema_ver : 1U;
    JsonObject settings_obj = rsp["settings"].to<JsonObject>();
    JsonObject runtime_obj  = rsp["runtime"].to<JsonObject>();
    runtime_obj["control_mode"]  = ficp_control_mode_to_str(st->session.control_mode);
    runtime_obj["backlight_pct"] = st->session.backlight_pct;
    runtime_obj["local_audio"]   = ficp_audio_mode_to_str(st->session.local_audio);

    if (st->cfg.snapshot_cb != nullptr) {
        st->cfg.snapshot_cb(settings_obj, runtime_obj, section);
    }

    if (!fic_ble_send_json_frame(FICP_MSG_STATE_SNAPSHOT_RSP, rsp)) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Snapshot exceeds max frame");
        return;
    }
    if ((frame->flags & FICP_FLAG_ACK_REQ) != 0U) {
        fic_ble_send_ack(frame->seq, NULL);
    }
}

static void handle_config_update(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    if (!frame->payload || frame->payload_len == 0U) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Empty config payload");
        return;
    }

    JsonDocument rx_doc;
    if (deserializeJson(rx_doc, frame->payload, frame->payload_len) != DeserializationError::Ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Malformed JSON");
        return;
    }

    const uint32_t base_rev = rx_doc["base_rev"] | 0U;
    if (!ficp_session_check_base_rev(&st->session, base_rev)) {
        fic_ble_send_nack(frame->seq, FICP_ERR_REV_CONFLICT, "state_rev mismatch");
        return;
    }
    if (st->cfg.config_update_cb == nullptr) {
        fic_ble_send_nack(frame->seq, FICP_ERR_UNSUPPORTED_MSG, "No config handler");
        return;
    }

    JsonObjectConst settings_in = rx_doc["settings"].as<JsonObjectConst>();
    if (settings_in.isNull()) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Missing settings object");
        return;
    }

    const bool persist = rx_doc["persist"] | true;
    JsonDocument applied_doc;
    JsonObject applied_obj = applied_doc.to<JsonObject>();
    char reason[96] = {0};

    if (!st->cfg.config_update_cb(settings_in, applied_obj, persist, reason, sizeof(reason))) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM,
                          (reason[0] != '\0') ? reason : "Config rejected");
        return;
    }

    (void)ficp_session_bump_rev(&st->session);
    String applied_raw;
    serializeJson(applied_doc, applied_raw);
    fic_ble_send_ack(frame->seq, applied_raw.c_str());
}

static void handle_control_mode_set(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    if (!frame->payload || frame->payload_len == 0U) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Empty control mode payload");
        return;
    }

    JsonDocument rx_doc;
    if (deserializeJson(rx_doc, frame->payload, frame->payload_len) != DeserializationError::Ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Malformed JSON");
        return;
    }

    const char *mode_str = rx_doc["mode"] | "";
    FicControlMode_t mode = FIC_CTRL_STANDALONE;
    if (!ficp_control_mode_from_str(mode_str, &mode)) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid control mode");
        return;
    }

    const uint8_t backlight_pct = (uint8_t)(rx_doc["backlight_pct"] | 0);
    const uint32_t watchdog_ms  = (uint32_t)(rx_doc["watchdog_ms"] | 0);
    FicLocalAudioMode_t aud     = FIC_AUDIO_MUTED;
    if (rx_doc["local_audio"].is<const char *>()) {
        if (!ficp_audio_mode_from_str(rx_doc["local_audio"].as<const char *>(), &aud)) {
            fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid local_audio");
            return;
        }
    } else if (mode != FIC_CTRL_SLAVE_DIMMED) {
        aud = FIC_AUDIO_ACTIVE;
    }

    if (!ficp_session_apply_control_mode(&st->session, mode, backlight_pct, aud, watchdog_ms, (uint32_t)millis())) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Control mode rejected");
        return;
    }

    if (st->cfg.control_mode_cb != nullptr) {
        st->cfg.control_mode_cb(st->session.control_mode,
                                st->session.backlight_pct,
                                st->session.local_audio,
                                "remote");
    }

    JsonDocument data;
    data["control_mode"]  = ficp_control_mode_to_str(st->session.control_mode);
    data["backlight_pct"] = st->session.backlight_pct;
    data["local_audio"]   = ficp_audio_mode_to_str(st->session.local_audio);
    data["watchdog_ms"]   = st->session.watchdog_ms;
    String raw;
    serializeJson(data, raw);
    fic_ble_send_ack(frame->seq, raw.c_str());
}

static void handle_ext_rpc(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    if (st->cfg.ext_rpc_cb == nullptr) {
        fic_ble_send_nack(frame->seq, FICP_ERR_UNSUPPORTED_MSG, "Extension handler not registered");
        return;
    }

    char ack_json[384]   = {0};
    char err_reason[96]  = {0};
    FicpErrCode_t err_c  = FICP_ERR_INVALID_PARAM;
    const bool ok = st->cfg.ext_rpc_cb(frame->msg_id,
                                       frame->seq,
                                       frame->flags,
                                       frame->payload,
                                       frame->payload_len,
                                       ack_json,
                                       sizeof(ack_json),
                                       &err_c,
                                       err_reason,
                                       sizeof(err_reason));
    if (ok) {
        fic_ble_send_ack(frame->seq, (ack_json[0] != '\0') ? ack_json : NULL);
    } else {
        fic_ble_send_nack(frame->seq, err_c,
                          (err_reason[0] != '\0') ? err_reason : "Extension command failed");
    }
}

void fic_ble_dispatch_bad_crc(uint16_t seq, uint8_t flags, uint8_t msg_id, void *ctx)
{
    (void)flags;
    (void)msg_id;
    (void)ctx;
    fic_ble_send_nack(seq, FICP_ERR_BAD_CRC, "CRC-16 mismatch");
}

void fic_ble_dispatch_frame(const FicpFrameView_t *frame, void *ctx)
{
    (void)ctx;
    if (!frame) {
        return;
    }
    FicBleRuntimeState_t *st = fic_ble_state();
    ficp_session_note_rx(&st->session, (uint32_t)millis());

    if (frame->msg_id >= FICP_MSG_EXT_BASE) {
        handle_ext_rpc(frame);
        return;
    }

    switch ((FicpMsgId_t)frame->msg_id) {
        case FICP_MSG_SYS_HELLO:          handle_sys_hello(frame); break;
        case FICP_MSG_SYS_HEARTBEAT:      handle_sys_heartbeat(frame); break;
        case FICP_MSG_SYS_HEALTH_REQ:     handle_sys_health_req(frame); break;
        case FICP_MSG_SYS_RTC_SET:        handle_sys_rtc_set(frame); break;
        case FICP_MSG_SYS_SCREENSHOT_REQ: handle_sys_screenshot_req(frame); break;
        case FICP_MSG_STATE_SNAPSHOT_REQ: handle_state_snapshot_req(frame); break;
        case FICP_MSG_CONFIG_UPDATE:      handle_config_update(frame); break;
        case FICP_MSG_CONTROL_MODE_SET:   handle_control_mode_set(frame); break;
        case FICP_MSG_FILE_LIST_REQ:      fic_ble_files_handle_list_req(frame); break;
        case FICP_MSG_FILE_OPEN_REQ:      fic_ble_files_handle_open_req(frame); break;
        case FICP_MSG_FILE_CHUNK:         fic_ble_files_handle_chunk(frame); break;
        case FICP_MSG_FILE_CLOSE_REQ:     fic_ble_files_handle_close_req(frame); break;
        default:
            fic_ble_send_nack(frame->seq, FICP_ERR_UNSUPPORTED_MSG, "Unsupported msg_id");
            break;
    }
}

void fic_ble_send_ack(uint16_t ack_seq, const char *data_json_raw)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    JsonDocument doc;
    doc["ack_seq"]   = ack_seq;
    doc["state_rev"] = st->session.state_rev;

    if (data_json_raw != NULL && data_json_raw[0] != '\0') {
        JsonDocument data_doc;
        if (deserializeJson(data_doc, data_json_raw) == DeserializationError::Ok) {
            doc["data"] = data_doc.as<JsonVariantConst>();
        }
    }
    (void)fic_ble_send_json_frame(FICP_MSG_SYS_ACK, doc);
}

void fic_ble_send_nack(uint16_t ack_seq, FicpErrCode_t err_code, const char *reason)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    JsonDocument doc;
    doc["ack_seq"]   = ack_seq;
    doc["err_code"]  = (uint8_t)err_code;
    doc["reason"]    = (reason && reason[0] != '\0') ? reason : ficp_err_name(err_code);
    doc["state_rev"] = st->session.state_rev;
    (void)fic_ble_send_json_frame(FICP_MSG_SYS_NACK, doc);
}

void fic_ble_emit_control_mode_event(const char *origin)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    if (!st->active || !st->session.connected || !st->ctrl_tx_subscribed) {
        return;
    }
    JsonDocument doc;
    doc["state_rev"]     = st->session.state_rev;
    doc["mode"]          = ficp_control_mode_to_str(st->session.control_mode);
    doc["backlight_pct"] = st->session.backlight_pct;
    doc["local_audio"]   = ficp_audio_mode_to_str(st->session.local_audio);
    doc["origin"]        = (origin && origin[0] != '\0') ? origin : "local";
    (void)fic_ble_send_json_frame(FICP_MSG_CONTROL_MODE_EVENT, doc);
}

extern "C" void fic_ble_companion_notify_local_change(const char *origin,
                                                      const char *settings_json,
                                                      const char *runtime_json)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    const uint32_t new_rev = ficp_session_bump_rev(&st->session);
    if (!st->active || !st->session.connected || !st->ctrl_tx_subscribed) {
        return;
    }

    JsonDocument doc;
    doc["state_rev"] = new_rev;
    doc["origin"]    = (origin && origin[0] != '\0') ? origin : "local_ui";

    JsonDocument s_doc;
    if (settings_json && settings_json[0] != '\0' &&
        deserializeJson(s_doc, settings_json) == DeserializationError::Ok) {
        doc["settings"] = s_doc.as<JsonVariantConst>();
    }
    JsonDocument r_doc;
    if (runtime_json && runtime_json[0] != '\0' &&
        deserializeJson(r_doc, runtime_json) == DeserializationError::Ok) {
        doc["runtime"] = r_doc.as<JsonVariantConst>();
    }
    (void)fic_ble_send_json_frame(FICP_MSG_STATE_EVENT, doc);
}

extern "C" bool fic_ble_companion_reclaim_local(const char *origin)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    const bool changed = ficp_session_reclaim_local(&st->session);
    if (changed) {
        if (st->cfg.control_mode_cb != nullptr) {
            st->cfg.control_mode_cb(st->session.control_mode,
                                    st->session.backlight_pct,
                                    st->session.local_audio,
                                    (origin && origin[0] != '\0') ? origin : "local_reclaim");
        }
        fic_ble_emit_control_mode_event((origin && origin[0] != '\0') ? origin : "local_reclaim");
    }
    return changed;
}

extern "C" bool fic_ble_companion_send_ext_packet(uint8_t msg_id,
                                                  uint8_t flags,
                                                  const uint8_t *payload,
                                                  uint16_t len)
{
    if (msg_id < FICP_MSG_EXT_BASE) {
        return false;
    }
    return fic_ble_send_frame(msg_id, flags, payload, len);
}

extern "C" void fic_ble_companion_loop(void)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    if (!st->active) {
        return;
    }

    if (st->disconnect_cleanup_pending) {
        st->disconnect_cleanup_pending = false;
        fic_ble_files_abort_all();
    }

    uint8_t drain_buf[256];
    for (uint8_t pass = 0U; pass < 8U; pass++) {
        size_t popped = 0U;
        if (st->ctrl_rx_ring_buf != nullptr) {
            taskENTER_CRITICAL(&st->rx_spinlock);
            popped = fic_ring_buf_pop(&st->ctrl_rx_ring, drain_buf, sizeof(drain_buf));
            taskEXIT_CRITICAL(&st->rx_spinlock);
        }
        if (popped == 0U) {
            break;
        }
        ficp_parser_feed(&st->parser,
                         drain_buf,
                         popped,
                         fic_ble_dispatch_frame,
                         fic_ble_dispatch_bad_crc,
                         st);
    }

    const uint32_t now_ms = (uint32_t)millis();
    if (ficp_session_check_watchdog(&st->session, now_ms)) {
        fic_ble_log("[BLE] Watchdog timeout -> restored standalone control\n");
        if (st->cfg.control_mode_cb != nullptr) {
            st->cfg.control_mode_cb(st->session.control_mode,
                                    st->session.backlight_pct,
                                    st->session.local_audio,
                                    "watchdog_timeout");
        }
        fic_ble_emit_control_mode_event("watchdog_timeout");
    }

    if (st->session.connected && st->ctrl_tx_subscribed) {
        if ((uint32_t)(now_ms - st->session.last_hb_tx_ms) >= FICP_HEARTBEAT_INTERVAL_MS) {
            st->session.last_hb_tx_ms = now_ms;
            uint8_t hb[FICP_HEARTBEAT_PAYLOAD_LEN] = {};
            (void)ficp_pack_heartbeat(hb, sizeof(hb), now_ms, st->session.state_rev,
                                      st->session.control_mode, 0U);
            (void)fic_ble_send_frame(FICP_MSG_SYS_HEARTBEAT, 0U, hb, sizeof(hb));
        }

        if (st->cfg.telem_pack_cb != nullptr) {
            const uint8_t hz = (st->cfg.telem_hz_default > 0U && st->cfg.telem_hz_default <= 20U)
                                   ? st->cfg.telem_hz_default
                                   : 5U;
            const uint32_t interval_ms = 1000UL / (uint32_t)hz;
            if ((uint32_t)(now_ms - st->last_telem_ms) >= interval_ms) {
                st->last_telem_ms = now_ms;
                uint8_t telem_buf[FICP_SINGLE_NOTIFY_MAX - FICP_OVERHEAD_LEN];
                uint8_t msg_id = (uint8_t)(FICP_MSG_EXT_BASE + 1U);
                const size_t tlen = st->cfg.telem_pack_cb(telem_buf, sizeof(telem_buf), &msg_id);
                if (tlen > 0U && tlen <= sizeof(telem_buf)) {
                    (void)fic_ble_send_frame(msg_id, 0U, telem_buf, (uint16_t)tlen);
                }
            }
        }

        fic_ble_files_tick();
    }
}
