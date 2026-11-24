#ifndef FIC_SETTINGS_JSON_H
#define FIC_SETTINGS_JSON_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
#include <ArduinoJson.h>
#include <FS.h>
#include "../boards/fic_board_types.h"

void fic_json_backup_file(fs::FS &fs, const char *src_path, const char *bak_path);
bool fic_json_write_atomic(fs::FS &fs,
                           const char *path,
                           const char *bak_path,
                           const char *tmp_path,
                           const String &payload,
                           fic_log_cb_t log_cb);
bool fic_json_try_load(fs::FS &fs, const char *path, JsonDocument &doc, fic_log_cb_t log_cb);
bool fic_json_load_with_recovery(fs::FS &fs,
                                 const char *path,
                                 const char *bak_path,
                                 const char *tmp_path,
                                 JsonDocument &doc,
                                 fic_log_cb_t log_cb);
const char *fic_json_pick_first_str(const JsonDocument &doc, const char * const *keys, size_t count);
void fic_json_clamp_float(const JsonDocument &doc, const char *key, float *dst, float lo, float hi);
bool fic_json_is_placeholder_ssid(const char *ssid);
#endif // __cplusplus

#endif // FIC_SETTINGS_JSON_H
