#include "fic_ble_internal.h"
#include "../storage/fic_fs_manager.h"

#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    bool     open;
    bool     is_write;
    bool     read_streaming;
    bool     has_crc32;
    uint16_t handle;
    uint32_t total_bytes;
    uint32_t offset;
    uint32_t expected_crc32;
    uint32_t running_crc32;
    fs::FS  *fs;
    char     fs_key[8];
    char     path[96];
    char     tmp_path[108];
    File     file;
} FicBleFileSession_t;

static FicBleFileSession_t s_xfer = {};
static uint16_t s_next_handle     = 1U;

static SemaphoreHandle_t get_fs_mux(const FicBleRuntimeState_t *st)
{
    return (st->cfg.fs_mux != NULL) ? st->cfg.fs_mux : fic_fs_get_mutex();
}

static fs::FS *resolve_target_fs(const FicBleRuntimeState_t *st, const char *fs_key)
{
    if (fs_key != nullptr && strcmp(fs_key, "sd") == 0) {
        return st->cfg.primary_fs;
    }
    if (fs_key != nullptr && strcmp(fs_key, "ffat") == 0) {
        return st->cfg.fallback_fs;
    }
    return st->cfg.primary_fs ? st->cfg.primary_fs : st->cfg.fallback_fs;
}

static bool sanitize_path(const char *raw, char *out, size_t out_sz)
{
    if (!raw || raw[0] == '\0' || !out || out_sz < 4U) {
        return false;
    }
    if (strstr(raw, "..") != nullptr || strchr(raw, '\\') != nullptr) {
        return false;
    }
    if (raw[0] == '/') {
        if (strlen(raw) >= out_sz) return false;
        snprintf(out, out_sz, "%s", raw);
    } else {
        if (strlen(raw) + 1U >= out_sz) return false;
        snprintf(out, out_sz, "/%s", raw);
    }
    return true;
}

void fic_ble_files_abort_all(void)
{
    if (!s_xfer.open) {
        return;
    }
    FicBleRuntimeState_t *st = fic_ble_state();
    SemaphoreHandle_t mux = get_fs_mux(st);
    if (!mux || xSemaphoreTake(mux, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (s_xfer.file) {
            s_xfer.file.close();
        }
        if (s_xfer.is_write && s_xfer.fs && s_xfer.tmp_path[0] != '\0') {
            (void)s_xfer.fs->remove(s_xfer.tmp_path);
        }
        if (mux) xSemaphoreGive(mux);
    } else if (s_xfer.file) {
        s_xfer.file.close();
    }
    s_xfer.open           = false;
    s_xfer.is_write       = false;
    s_xfer.read_streaming = false;
    s_xfer.fs             = nullptr;
}

void fic_ble_files_handle_list_req(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    JsonDocument rx_doc;
    if (!frame->payload || frame->payload_len == 0U ||
        deserializeJson(rx_doc, frame->payload, frame->payload_len) != DeserializationError::Ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid list JSON");
        return;
    }

    const char *fs_key = rx_doc["fs"] | "sd";
    const char *raw_p  = rx_doc["path"] | "/";
    const size_t offset = (size_t)(rx_doc["offset"] | 0U);
    size_t limit = (size_t)(rx_doc["limit"] | 16U);
    if (limit == 0U) limit = 16U;
    if (limit > 24U) limit = 24U;

    char clean_path[96] = {0};
    if (!sanitize_path(raw_p, clean_path, sizeof(clean_path))) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid path");
        return;
    }

    fs::FS *fs = resolve_target_fs(st, fs_key);
    if (!fs) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Storage not mounted");
        return;
    }

    SemaphoreHandle_t mux = get_fs_mux(st);
    if (mux && xSemaphoreTake(mux, pdMS_TO_TICKS(500)) != pdTRUE) {
        fic_ble_send_nack(frame->seq, FICP_ERR_BUSY_LOCKED, "Storage busy");
        return;
    }

    JsonDocument rsp;
    rsp["fs"]       = fs_key;
    rsp["path"]     = clean_path;
    rsp["has_more"] = false;
    JsonArray entries = rsp["entries"].to<JsonArray>();

    File root = fs->open(clean_path);
    if (!root || !root.isDirectory()) {
        if (root) root.close();
        if (mux) xSemaphoreGive(mux);
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Directory not found");
        return;
    }

    size_t idx = 0U;
    File entry = root.openNextFile();
    while (entry) {
        if (idx >= offset) {
            if (entries.size() >= limit) {
                rsp["has_more"] = true;
                entry.close();
                break;
            }
            const char *name = entry.name();
            const char *base = strrchr(name, '/');
            base = base ? (base + 1) : name;

            JsonObject e = entries.add<JsonObject>();
            e["name"]   = base;
            e["size"]   = (uint32_t)entry.size();
            e["is_dir"] = entry.isDirectory();

            // Guard total JSON length inside FICP max payload
            if (measureJson(rsp) > (FICP_MAX_PAYLOAD_LEN - 64U)) {
                entries.remove(entries.size() - 1U);
                rsp["has_more"] = true;
                entry.close();
                break;
            }
        }
        idx++;
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
    if (mux) xSemaphoreGive(mux);

    (void)fic_ble_send_json_frame(FICP_MSG_FILE_LIST_RSP, rsp);
    if ((frame->flags & FICP_FLAG_ACK_REQ) != 0U) {
        fic_ble_send_ack(frame->seq, NULL);
    }
}

void fic_ble_files_handle_open_req(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    JsonDocument rx_doc;
    if (!frame->payload || frame->payload_len == 0U ||
        deserializeJson(rx_doc, frame->payload, frame->payload_len) != DeserializationError::Ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid open JSON");
        return;
    }

    if (s_xfer.open) {
        fic_ble_send_nack(frame->seq, FICP_ERR_BUSY_LOCKED, "File handle already open");
        return;
    }

    const char *op     = rx_doc["op"] | "";
    const char *fs_key = rx_doc["fs"] | "sd";
    const char *raw_p  = rx_doc["path"] | "";
    const bool is_write = (strcmp(op, "write") == 0);
    const bool is_read  = (strcmp(op, "read") == 0);
    if (!is_write && !is_read) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Unsupported file op");
        return;
    }

    char clean_path[96] = {0};
    if (!sanitize_path(raw_p, clean_path, sizeof(clean_path))) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid path");
        return;
    }

    if (is_write && st->cfg.file_lock_check_cb != nullptr) {
        char lock_reason[96] = {0};
        if (st->cfg.file_lock_check_cb(lock_reason, sizeof(lock_reason))) {
            fic_ble_send_nack(frame->seq, FICP_ERR_BUSY_LOCKED,
                              (lock_reason[0] != '\0') ? lock_reason : "Active field op locks storage");
            return;
        }
    }

    fs::FS *fs = resolve_target_fs(st, fs_key);
    if (!fs) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Storage not mounted");
        return;
    }

    SemaphoreHandle_t mux = get_fs_mux(st);
    if (mux && xSemaphoreTake(mux, pdMS_TO_TICKS(500)) != pdTRUE) {
        fic_ble_send_nack(frame->seq, FICP_ERR_BUSY_LOCKED, "Storage busy");
        return;
    }

    s_xfer.fs            = fs;
    s_xfer.is_write      = is_write;
    s_xfer.offset        = 0U;
    s_xfer.running_crc32 = 0U;
    s_xfer.has_crc32     = rx_doc["crc32"].is<uint32_t>();
    s_xfer.expected_crc32 = rx_doc["crc32"] | 0U;
    s_xfer.total_bytes   = rx_doc["total_bytes"] | 0U;
    snprintf(s_xfer.fs_key, sizeof(s_xfer.fs_key), "%s", fs_key);
    snprintf(s_xfer.path, sizeof(s_xfer.path), "%s", clean_path);

    if (is_write) {
        snprintf(s_xfer.tmp_path, sizeof(s_xfer.tmp_path), "%s.ficp.tmp", clean_path);
        s_xfer.file = fs->open(s_xfer.tmp_path, FILE_WRITE);
        s_xfer.read_streaming = false;
    } else {
        s_xfer.tmp_path[0] = '\0';
        s_xfer.file = fs->open(clean_path, FILE_READ);
        if (s_xfer.file) {
            s_xfer.total_bytes = (uint32_t)s_xfer.file.size();
        }
        s_xfer.read_streaming = true;
    }

    if (!s_xfer.file) {
        if (mux) xSemaphoreGive(mux);
        s_xfer.open = false;
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Failed to open file");
        return;
    }

    if (s_next_handle == 0U) s_next_handle = 1U;
    s_xfer.handle = s_next_handle++;
    s_xfer.open   = true;
    if (mux) xSemaphoreGive(mux);

    JsonDocument data;
    data["handle"]      = s_xfer.handle;
    data["total_bytes"] = s_xfer.total_bytes;
    data["chunk_max"]   = FICP_FILE_CHUNK_MAX_DATA;
    String raw;
    serializeJson(data, raw);
    fic_ble_send_ack(frame->seq, raw.c_str());
}

void fic_ble_files_handle_chunk(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    uint16_t handle = 0U;
    uint32_t offset = 0U;
    if (!ficp_unpack_file_chunk_hdr(frame->payload, frame->payload_len, &handle, &offset)) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Short file chunk");
        return;
    }
    if (!s_xfer.open || !s_xfer.is_write || handle != s_xfer.handle) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid write handle");
        return;
    }
    if (offset != s_xfer.offset) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Unexpected chunk offset");
        return;
    }

    const uint8_t *chunk_data = frame->payload + FICP_FILE_CHUNK_HDR_LEN;
    const size_t chunk_len    = (size_t)frame->payload_len - FICP_FILE_CHUNK_HDR_LEN;
    if (chunk_len > 0U) {
        SemaphoreHandle_t mux = get_fs_mux(st);
        if (mux && xSemaphoreTake(mux, pdMS_TO_TICKS(500)) != pdTRUE) {
            fic_ble_send_nack(frame->seq, FICP_ERR_BUSY_LOCKED, "Storage busy");
            return;
        }
        const size_t written = s_xfer.file.write(chunk_data, chunk_len);
        if (mux) xSemaphoreGive(mux);
        if (written != chunk_len) {
            fic_ble_files_abort_all();
            fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Write short count");
            return;
        }
        s_xfer.running_crc32 = ficp_crc32_update(s_xfer.running_crc32, chunk_data, chunk_len);
        s_xfer.offset += (uint32_t)chunk_len;
    }

    if ((frame->flags & FICP_FLAG_ACK_REQ) != 0U) {
        JsonDocument data;
        data["offset"] = s_xfer.offset;
        String raw;
        serializeJson(data, raw);
        fic_ble_send_ack(frame->seq, raw.c_str());
    }
}

void fic_ble_files_handle_close_req(const FicpFrameView_t *frame)
{
    FicBleRuntimeState_t *st = fic_ble_state();
    JsonDocument rx_doc;
    if (!frame->payload || frame->payload_len == 0U ||
        deserializeJson(rx_doc, frame->payload, frame->payload_len) != DeserializationError::Ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Invalid close JSON");
        return;
    }

    const uint16_t handle = (uint16_t)(rx_doc["handle"] | 0U);
    const bool abort_req  = rx_doc["abort"] | false;
    if (!s_xfer.open || handle != s_xfer.handle) {
        fic_ble_send_nack(frame->seq, FICP_ERR_INVALID_PARAM, "Unknown handle");
        return;
    }

    if (abort_req) {
        fic_ble_files_abort_all();
        fic_ble_send_ack(frame->seq, NULL);
        return;
    }

    const bool is_write       = s_xfer.is_write;
    const uint32_t bytes_xfer = s_xfer.offset;
    const uint32_t final_crc  = ficp_crc32_finish(s_xfer.running_crc32);
    const bool size_mismatch  = is_write && (s_xfer.total_bytes > 0U) && (bytes_xfer != s_xfer.total_bytes);
    const bool crc_mismatch   = is_write && s_xfer.has_crc32 && (final_crc != s_xfer.expected_crc32);

    SemaphoreHandle_t mux = get_fs_mux(st);
    if (mux && xSemaphoreTake(mux, pdMS_TO_TICKS(500)) != pdTRUE) {
        fic_ble_send_nack(frame->seq, FICP_ERR_BUSY_LOCKED, "Storage busy");
        return;
    }

    s_xfer.file.close();

    bool rename_ok = true;
    if (is_write) {
        if (size_mismatch || crc_mismatch) {
            (void)s_xfer.fs->remove(s_xfer.tmp_path);
            rename_ok = false;
        } else {
            if (s_xfer.fs->exists(s_xfer.path)) {
                (void)s_xfer.fs->remove(s_xfer.path);
            }
            rename_ok = s_xfer.fs->rename(s_xfer.tmp_path, s_xfer.path);
            if (!rename_ok) {
                (void)s_xfer.fs->remove(s_xfer.tmp_path);
            }
        }
    }
    if (mux) xSemaphoreGive(mux);

    char committed_fs[8];
    char committed_path[96];
    snprintf(committed_fs, sizeof(committed_fs), "%s", s_xfer.fs_key);
    snprintf(committed_path, sizeof(committed_path), "%s", s_xfer.path);

    s_xfer.open           = false;
    s_xfer.read_streaming = false;

    if (size_mismatch) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Upload byte count mismatch");
        return;
    }
    if (crc_mismatch) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Upload CRC-32 mismatch");
        return;
    }
    if (!rename_ok) {
        fic_ble_send_nack(frame->seq, FICP_ERR_STORAGE_FAIL, "Atomic rename failed");
        return;
    }

    if (is_write && st->cfg.file_committed_cb != nullptr) {
        st->cfg.file_committed_cb(committed_fs, committed_path);
    }

    JsonDocument data;
    data["path"]  = committed_path;
    data["bytes"] = bytes_xfer;
    data["crc32"] = final_crc;
    String raw;
    serializeJson(data, raw);
    fic_ble_send_ack(frame->seq, raw.c_str());
}

void fic_ble_files_tick(void)
{
    if (!s_xfer.open || s_xfer.is_write || !s_xfer.read_streaming) {
        return;
    }
    FicBleRuntimeState_t *st = fic_ble_state();
    if (!st->session.connected || !st->ctrl_tx_subscribed) {
        return;
    }

    const uint16_t eff_mtu = (st->session.peer_mtu > 3U) ? (uint16_t)(st->session.peer_mtu - 3U) : 20U;
    size_t max_chunk = (size_t)FICP_FILE_CHUNK_MAX_DATA;
    if (eff_mtu <= FICP_SINGLE_NOTIFY_MAX &&
        eff_mtu > (FICP_OVERHEAD_LEN + FICP_FILE_CHUNK_HDR_LEN)) {
        const size_t single_pkt_data = eff_mtu - FICP_OVERHEAD_LEN - FICP_FILE_CHUNK_HDR_LEN;
        if (single_pkt_data < max_chunk) {
            max_chunk = single_pkt_data;
        }
    }

    uint8_t pkt[FICP_FILE_CHUNK_HDR_LEN + FICP_FILE_CHUNK_MAX_DATA];
    SemaphoreHandle_t mux = get_fs_mux(st);
    if (mux && xSemaphoreTake(mux, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    const size_t nread = s_xfer.file.read(&pkt[FICP_FILE_CHUNK_HDR_LEN], max_chunk);
    if (mux) xSemaphoreGive(mux);

    if (nread == 0U) {
        s_xfer.read_streaming = false;
        return;
    }

    (void)ficp_pack_file_chunk_hdr(pkt, FICP_FILE_CHUNK_HDR_LEN, s_xfer.handle, s_xfer.offset);
    if (fic_ble_send_frame(FICP_MSG_FILE_CHUNK, 0U, pkt, (uint16_t)(FICP_FILE_CHUNK_HDR_LEN + nread))) {
        s_xfer.running_crc32 = ficp_crc32_update(s_xfer.running_crc32,
                                                 &pkt[FICP_FILE_CHUNK_HDR_LEN], nread);
        s_xfer.offset += (uint32_t)nread;
        if (s_xfer.offset >= s_xfer.total_bytes) {
            s_xfer.read_streaming = false;
        }
    }
}
