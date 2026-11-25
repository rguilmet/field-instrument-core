#ifndef FIC_BLE_INTERNAL_H
#define FIC_BLE_INTERNAL_H

#include "fic_ble_companion.h"
#include "../storage/fic_ring_buf.h"

#include <ArduinoJson.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>

typedef struct {
    FicBleCompanionCfg_t cfg;
    bool                 configured;
    bool                 active;
    bool                 ctrl_tx_subscribed;
    bool                 stream_tx_subscribed;
    bool                 disconnect_cleanup_pending;
    uint16_t             conn_handle;
    uint16_t             attr_ctrl_rx;
    uint16_t             attr_ctrl_tx;
    uint16_t             attr_stream_rx;
    uint16_t             attr_stream_tx;
    FicpSessionState_t   session;
    FicpParser_t         parser;
    FicByteRing_t        ctrl_rx_ring;
    uint8_t             *ctrl_rx_ring_buf;
    uint8_t             *parser_buf;
    uint8_t             *tx_frame_buf;
    uint8_t             *stream_rx_scratch;
    size_t               stream_rx_scratch_cap;
    SemaphoreHandle_t    tx_mux;
    portMUX_TYPE         rx_spinlock;
    uint32_t             last_telem_ms;
    uint32_t             frames_tx;
    uint32_t             stream_rx_bytes;
    uint32_t             stream_rx_packets;
} FicBleRuntimeState_t;

FicBleRuntimeState_t *fic_ble_state(void);

void fic_ble_log(const char *fmt, ...);

bool fic_ble_send_frame(uint8_t msg_id, uint8_t flags, const uint8_t *payload, uint16_t len);
bool fic_ble_send_json_frame(uint8_t msg_id, const JsonDocument &doc);
void fic_ble_send_ack(uint16_t ack_seq, const char *data_json_raw);
void fic_ble_send_nack(uint16_t ack_seq, FicpErrCode_t err_code, const char *reason);
void fic_ble_emit_control_mode_event(const char *origin);

void fic_ble_dispatch_frame(const FicpFrameView_t *frame, void *ctx);
void fic_ble_dispatch_bad_crc(uint16_t seq, uint8_t flags, uint8_t msg_id, void *ctx);

void fic_ble_files_handle_list_req(const FicpFrameView_t *frame);
void fic_ble_files_handle_open_req(const FicpFrameView_t *frame);
void fic_ble_files_handle_chunk(const FicpFrameView_t *frame);
void fic_ble_files_handle_close_req(const FicpFrameView_t *frame);
void fic_ble_files_tick(void);
void fic_ble_files_abort_all(void);

#endif // FIC_BLE_INTERNAL_H
