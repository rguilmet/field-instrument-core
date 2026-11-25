#ifndef FIC_BLE_COMPANION_H
#define FIC_BLE_COMPANION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fic_ble_protocol.h"
#include "../boards/fic_board_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t dram_free;
    uint32_t dram_largest;
    uint32_t psram_free;
    uint16_t vbat_mv;
    uint8_t  batt_pct;
    uint32_t i2c0_errs;
    uint32_t i2c1_errs;
    bool     sd_ok;
    uint32_t uptime_s;
} FicBleHealthInfo_t;

typedef struct {
    bool             active;
    bool             connected;
    bool             ctrl_tx_subscribed;
    bool             stream_tx_subscribed;
    FicControlMode_t control_mode;
    uint32_t         state_rev;
    uint16_t         peer_mtu;
    uint32_t         frames_rx;
    uint32_t         frames_tx;
    uint32_t         crc_errors;
    uint32_t         framing_errors;
    uint32_t         ring_drops;
    uint32_t         watchdog_trips;
    uint32_t         stream_rx_bytes;
    uint32_t         stream_rx_packets;
} FicBleStats_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <ArduinoJson.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

typedef void (*fic_ble_snapshot_cb_t)(JsonObject settings_out,
                                      JsonObject runtime_out,
                                      const char *section);

typedef bool (*fic_ble_config_update_cb_t)(JsonObjectConst settings_in,
                                           JsonObject applied_out,
                                           bool persist,
                                           char *err_reason,
                                           size_t err_reason_sz);

typedef void (*fic_ble_control_mode_cb_t)(FicControlMode_t mode,
                                          uint8_t backlight_pct,
                                          FicLocalAudioMode_t local_audio,
                                          const char *origin);

typedef bool (*fic_ble_ext_rpc_cb_t)(uint8_t msg_id,
                                     uint16_t seq,
                                     uint8_t flags,
                                     const uint8_t *payload,
                                     uint16_t len,
                                     char *ack_json_out,
                                     size_t ack_json_sz,
                                     FicpErrCode_t *err_code_out,
                                     char *err_reason_out,
                                     size_t err_reason_sz);

typedef void (*fic_ble_stream_rx_cb_t)(const uint8_t *data, size_t len);
typedef size_t (*fic_ble_telem_pack_cb_t)(uint8_t *out, size_t max_len, uint8_t *msg_id_out);
typedef void (*fic_ble_health_cb_t)(FicBleHealthInfo_t *out);
typedef bool (*fic_ble_rtc_set_cb_t)(const FicpUtcCalendar_t *utc);
typedef bool (*fic_ble_screenshot_cb_t)(char *out_path, size_t out_path_sz);
typedef bool (*fic_ble_file_lock_check_cb_t)(char *reason_out, size_t reason_sz);
typedef void (*fic_ble_file_committed_cb_t)(const char *fs_key, const char *path);
typedef void (*fic_ble_conn_event_cb_t)(bool connected, uint16_t peer_mtu);

typedef struct {
    const char                   *device_type;
    const char                   *device_name;
    const char                   *fw_version;
    const char                   *board_name;
    uint32_t                      schema_ver;
    uint8_t                       auth_mode;
    uint32_t                      feature_mask;
    uint8_t                       telem_hz_default;
    bool                          enable_stream_rx;
    bool                          enable_stream_tx;
    size_t                        ctrl_rx_ring_bytes;
    fs::FS                       *primary_fs;
    fs::FS                       *fallback_fs;
    SemaphoreHandle_t             fs_mux;
    fic_ble_snapshot_cb_t         snapshot_cb;
    fic_ble_config_update_cb_t    config_update_cb;
    fic_ble_control_mode_cb_t     control_mode_cb;
    fic_ble_ext_rpc_cb_t          ext_rpc_cb;
    fic_ble_stream_rx_cb_t        stream_rx_cb;
    fic_ble_telem_pack_cb_t       telem_pack_cb;
    fic_ble_health_cb_t           health_cb;
    fic_ble_rtc_set_cb_t          rtc_set_cb;
    fic_ble_screenshot_cb_t       screenshot_cb;
    fic_ble_file_lock_check_cb_t  file_lock_check_cb;
    fic_ble_file_committed_cb_t   file_committed_cb;
    fic_ble_conn_event_cb_t       conn_event_cb;
    fic_log_cb_t                  log_cb;
} FicBleCompanionCfg_t;

extern "C" {
void fic_ble_companion_configure(const FicBleCompanionCfg_t *cfg);
}
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

bool fic_ble_companion_start(void);
void fic_ble_companion_stop(void);
void fic_ble_companion_loop(void);

bool fic_ble_companion_is_active(void);
bool fic_ble_companion_is_connected(void);
FicControlMode_t fic_ble_companion_get_control_mode(void);
uint32_t fic_ble_companion_get_state_rev(void);
void fic_ble_companion_get_stats(FicBleStats_t *out);

void fic_ble_companion_notify_local_change(const char *origin,
                                           const char *settings_json,
                                           const char *runtime_json);
bool fic_ble_companion_reclaim_local(const char *origin);
bool fic_ble_companion_send_ext_packet(uint8_t msg_id,
                                       uint8_t flags,
                                       const uint8_t *payload,
                                       uint16_t len);
bool fic_ble_companion_send_stream_tx(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // FIC_BLE_COMPANION_H
