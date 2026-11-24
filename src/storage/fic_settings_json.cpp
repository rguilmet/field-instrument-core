#include "fic_settings_json.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

namespace {

void json_log(fic_log_cb_t cb, const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (cb != nullptr) {
        cb("%s", buf);
    } else {
        printf("%s", buf);
    }
}

} // namespace

void fic_json_backup_file(fs::FS &fs, const char *src_path, const char *bak_path)
{
    if (!src_path || !bak_path || !fs.exists(src_path)) {
        return;
    }
    File src = fs.open(src_path, "r");
    if (!src) {
        return;
    }
    if (src.size() == 0) {
        src.close();
        return;
    }
    File dst = fs.open(bak_path, "w");
    if (!dst) {
        src.close();
        return;
    }
    uint8_t buf[512];
    while (src.available()) {
        size_t n = src.read(buf, sizeof(buf));
        if (n > 0) {
            dst.write(buf, n);
        }
    }
    dst.flush();
    dst.close();
    src.close();
}

bool fic_json_write_atomic(fs::FS &fs,
                           const char *path,
                           const char *bak_path,
                           const char *tmp_path,
                           const String &payload,
                           fic_log_cb_t log_cb)
{
    if (!path || !tmp_path) {
        return false;
    }

    const size_t expected_len = payload.length();
    if (bak_path != nullptr) {
        fic_json_backup_file(fs, path, bak_path);
    }
    if (fs.exists(tmp_path)) {
        fs.remove(tmp_path);
    }

    File tmp_file = fs.open(tmp_path, "w");
    if (!tmp_file) {
        json_log(log_cb, "[SETTINGS ERR] Failed to open temp file %s\n", tmp_path);
        return false;
    }
    size_t written = tmp_file.write(reinterpret_cast<const uint8_t *>(payload.c_str()), expected_len);
    tmp_file.flush();
    tmp_file.close();

    File verify = fs.open(tmp_path, "r");
    bool ok_verify = verify && (verify.size() == expected_len);
    if (verify) {
        verify.close();
    }
    if (written != expected_len || !ok_verify) {
        fs.remove(tmp_path);
        json_log(log_cb, "[SETTINGS ERR] Atomic verify failed on %s (wrote %lu/%lu B); kept existing config\n",
                 tmp_path, (unsigned long)written, (unsigned long)expected_len);
        return false;
    }

    if (fs.exists(path)) {
        fs.remove(path);
    }
    if (fs.rename(tmp_path, path)) {
        return true;
    }

    // Fallback if VFS rename fails on an external driver
    File direct = fs.open(path, "w");
    if (!direct) {
        fs.remove(tmp_path);
        return false;
    }
    size_t dw = direct.write(reinterpret_cast<const uint8_t *>(payload.c_str()), expected_len);
    direct.flush();
    direct.close();
    fs.remove(tmp_path);
    return (dw == expected_len);
}

bool fic_json_try_load(fs::FS &fs, const char *path, JsonDocument &doc, fic_log_cb_t log_cb)
{
    if (!path || !fs.exists(path)) {
        return false;
    }
    File f = fs.open(path, "r");
    if (!f || f.size() == 0) {
        if (f) f.close();
        return false;
    }
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        json_log(log_cb, "[SETTINGS] Deserialization error in %s: %s\n", path, err.c_str());
        return false;
    }
    return true;
}

bool fic_json_load_with_recovery(fs::FS &fs,
                                 const char *path,
                                 const char *bak_path,
                                 const char *tmp_path,
                                 JsonDocument &doc,
                                 fic_log_cb_t log_cb)
{
    if (fic_json_try_load(fs, path, doc, log_cb)) {
        return true;
    }
    if (bak_path && fic_json_try_load(fs, bak_path, doc, log_cb)) {
        json_log(log_cb, "[SETTINGS] Recovered %s from backup %s\n", path ? path : "json", bak_path);
        return true;
    }
    if (tmp_path && fic_json_try_load(fs, tmp_path, doc, log_cb)) {
        json_log(log_cb, "[SETTINGS] Recovered %s from temp %s\n", path ? path : "json", tmp_path);
        return true;
    }
    return false;
}

const char *fic_json_pick_first_str(const JsonDocument &doc, const char * const *keys, size_t count)
{
    if (!keys) {
        return nullptr;
    }
    for (size_t i = 0; i < count; i++) {
        if (keys[i] && !doc[keys[i]].isNull()) {
            return doc[keys[i]];
        }
    }
    return nullptr;
}

void fic_json_clamp_float(const JsonDocument &doc, const char *key, float *dst, float lo, float hi)
{
    if (!key || !dst || doc[key].isNull()) {
        return;
    }
    float v = doc[key];
    if (!isfinite(v)) {
        return;
    }
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    *dst = v;
}

bool fic_json_is_placeholder_ssid(const char *ssid)
{
    if (!ssid || ssid[0] == '\0') return true;
    if (strcasecmp(ssid, "Your_SSID") == 0) return true;
    if (strcasecmp(ssid, "Your-SSID-Here") == 0) return true;
    if (strcasecmp(ssid, "Your-SSID") == 0) return true;
    if (strcasecmp(ssid, "SSID") == 0) return true;
    return false;
}
