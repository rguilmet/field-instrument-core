#ifndef FIC_WEB_PORTAL_H
#define FIC_WEB_PORTAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "../boards/fic_board_types.h"

typedef bool (*fic_web_wifi_creds_cb_t)(char *ssid_out, size_t ssid_sz, char *pass_out, size_t pass_sz);
typedef bool (*fic_web_ap_creds_cb_t)(char *ssid_out, size_t ssid_sz, char *pass_out, size_t pass_sz);
typedef bool (*fic_web_write_lock_cb_t)(char *reason_out, size_t reason_sz);
typedef void (*fic_web_status_cb_t)(const char *status_text);
typedef void (*fic_web_set_time_cb_t)(uint16_t year, uint8_t month, uint8_t day,
                                      uint8_t hour, uint8_t minute, uint8_t second);
typedef void (*fic_web_file_uploaded_cb_t)(const char *filename, const char *fs_key);

typedef struct {
    const char                *portal_title;
    const char                *version_str;
    const char                *ap_ssid;
    const char                *ap_password;
    const char                *ntp_server;
    uint16_t                   http_port;
    fs::FS                    *primary_fs;
    const char                *primary_label;
    const char                *primary_vfs_path;
    fs::FS                    *fallback_fs;
    const char                *fallback_label;
    const char                *fallback_vfs_path;
    SemaphoreHandle_t          fs_mux;
    const char                *mirror_filename;
    fic_web_wifi_creds_cb_t    get_wifi_creds_cb;
    fic_web_status_cb_t        on_status_cb;
    fic_web_set_time_cb_t      on_set_time_cb;
    fic_web_file_uploaded_cb_t on_file_uploaded_cb;
    fic_log_cb_t               log_cb;
    fic_web_ap_creds_cb_t      get_ap_creds_cb;
    fic_web_write_lock_cb_t    is_write_locked_cb;
} FicWebPortalCfg_t;

extern "C" {
void fic_web_portal_configure(const FicWebPortalCfg_t *cfg);
}
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*fic_web_ble_is_active_cb_t)(void);
typedef void (*fic_web_ble_suspend_cb_t)(void);
typedef void (*fic_web_ble_resume_cb_t)(void);

void fic_web_portal_register_ble_hooks(fic_web_ble_is_active_cb_t is_active_cb,
                                       fic_web_ble_suspend_cb_t suspend_cb,
                                       fic_web_ble_resume_cb_t resume_cb);

void fic_web_portal_start(void);
void fic_web_portal_stop(void);
void fic_web_portal_handle(void);
bool fic_web_portal_is_active(void);
bool fic_web_portal_is_softap(void);
bool fic_web_portal_is_starting(void);
const char *fic_web_portal_get_ip(void);

#ifdef __cplusplus
}
#endif

#endif // FIC_WEB_PORTAL_H
