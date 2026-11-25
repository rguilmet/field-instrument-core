#ifndef FIC_BLE_PROTOCOL_H
#define FIC_BLE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// FICP Wire Protocol Constants (FICP v1.0.0)
// ============================================================================
#define FICP_VERSION_STR               "1.0.0"
#define FICP_WIRE_VER                  0x01U
#define FICP_SYNC_0                    0xAAU
#define FICP_SYNC_1                    0x55U

#define FICP_HEADER_LEN                9U
#define FICP_CRC_LEN                   2U
#define FICP_OVERHEAD_LEN              (FICP_HEADER_LEN + FICP_CRC_LEN)
#define FICP_MAX_PAYLOAD_LEN           1024U
#define FICP_MAX_FRAME_LEN             (FICP_OVERHEAD_LEN + FICP_MAX_PAYLOAD_LEN)
#define FICP_SINGLE_NOTIFY_MAX         509U

#define FICP_HEARTBEAT_PAYLOAD_LEN     8U
#define FICP_FILE_CHUNK_HDR_LEN        6U
#define FICP_FILE_CHUNK_MAX_DATA       480U
#define FICP_ATT_MTU_PREFERRED         517U

#define FICP_WATCHDOG_DEFAULT_MS       5000U
#define FICP_WATCHDOG_MIN_MS           3000U
#define FICP_WATCHDOG_MAX_MS           15000U
#define FICP_HEARTBEAT_INTERVAL_MS     1000U

#define FICP_SLAVE_BACKLIGHT_MIN_PCT     10U
#define FICP_SLAVE_BACKLIGHT_MAX_PCT     30U
#define FICP_SLAVE_BACKLIGHT_DEFAULT_PCT 15U

// Frame Flags Bitmask
#define FICP_FLAG_ACK_REQ              (1U << 0)
#define FICP_FLAG_IS_JSON              (1U << 1)
#define FICP_FLAG_FRAGMENT             (1U << 2)

// Access Control Modes (MSG_SYS_CAPS.auth_mode)
#define FICP_AUTH_OPEN_GATED           0U
#define FICP_AUTH_SMP_BONDED           1U
#define FICP_AUTH_SESSION_TOKEN        2U

// Capability Feature Mask Bits
#define FICP_FEAT_SETTINGS_SYNC        (1UL << 0)
#define FICP_FEAT_CONTROL_HANDOFF      (1UL << 1)
#define FICP_FEAT_FILE_SYNC            (1UL << 2)
#define FICP_FEAT_STREAM_RX            (1UL << 3)
#define FICP_FEAT_STREAM_TX            (1UL << 4)
#define FICP_FEAT_RTC_SYNC             (1UL << 5)
#define FICP_FEAT_SCREENSHOT           (1UL << 6)

// Canonical FIC GATT UUID Strings
#define FICP_UUID_SVC_STR              "5f494350-0001-4b10-a8e1-464943500001"
#define FICP_UUID_CTRL_RX_STR          "5f494350-0002-4b10-a8e1-464943500001"
#define FICP_UUID_CTRL_TX_STR          "5f494350-0003-4b10-a8e1-464943500001"
#define FICP_UUID_STREAM_RX_STR        "5f494350-0004-4b10-a8e1-464943500001"
#define FICP_UUID_STREAM_TX_STR        "5f494350-0005-4b10-a8e1-464943500001"

// Core Standard Message IDs (0x00..0x7F; 0x80..0xFF reserved per instrument)
typedef enum {
    FICP_MSG_SYS_HELLO           = 0x01,
    FICP_MSG_SYS_CAPS            = 0x02,
    FICP_MSG_SYS_HEARTBEAT       = 0x03,
    FICP_MSG_SYS_ACK             = 0x04,
    FICP_MSG_SYS_NACK            = 0x05,
    FICP_MSG_SYS_HEALTH_REQ      = 0x06,
    FICP_MSG_SYS_HEALTH_RSP      = 0x07,
    FICP_MSG_SYS_RTC_SET         = 0x08,
    FICP_MSG_SYS_SCREENSHOT_REQ  = 0x09,
    FICP_MSG_STATE_SNAPSHOT_REQ  = 0x10,
    FICP_MSG_STATE_SNAPSHOT_RSP  = 0x11,
    FICP_MSG_CONFIG_UPDATE       = 0x12,
    FICP_MSG_STATE_EVENT         = 0x13,
    FICP_MSG_CONTROL_MODE_SET    = 0x20,
    FICP_MSG_CONTROL_MODE_EVENT  = 0x21,
    FICP_MSG_FILE_LIST_REQ       = 0x30,
    FICP_MSG_FILE_LIST_RSP       = 0x31,
    FICP_MSG_FILE_OPEN_REQ       = 0x32,
    FICP_MSG_FILE_CHUNK          = 0x33,
    FICP_MSG_FILE_CLOSE_REQ      = 0x34,
    FICP_MSG_EXT_BASE            = 0x80
} FicpMsgId_t;

// Canonical NACK Error Codes
typedef enum {
    FICP_ERR_NONE            = 0,
    FICP_ERR_BAD_CRC         = 1,
    FICP_ERR_UNSUPPORTED_MSG = 2,
    FICP_ERR_INVALID_PARAM   = 3,
    FICP_ERR_REV_CONFLICT    = 4,
    FICP_ERR_BUSY_LOCKED     = 5,
    FICP_ERR_STORAGE_FAIL    = 6,
    FICP_ERR_UNAUTHORIZED    = 7
} FicpErrCode_t;

// Control Handoff Modes
typedef enum {
    FIC_CTRL_STANDALONE       = 0,
    FIC_CTRL_COMPANION_MIRROR = 1,
    FIC_CTRL_SLAVE_DIMMED     = 2
} FicControlMode_t;

// Local Audio Disposition During Companion/Slave Operation
typedef enum {
    FIC_AUDIO_ACTIVE = 0,
    FIC_AUDIO_MUTED  = 1,
    FIC_AUDIO_FOLLOW = 2
} FicLocalAudioMode_t;

typedef struct {
    uint8_t        ver;
    uint8_t        msg_id;
    uint8_t        flags;
    uint16_t       seq;
    uint16_t       payload_len;
    const uint8_t *payload;
} FicpFrameView_t;

typedef enum {
    FICP_PARSE_SYNC0 = 0,
    FICP_PARSE_SYNC1 = 1,
    FICP_PARSE_HDR   = 2,
    FICP_PARSE_BODY  = 3
} FicpParseState_t;

typedef struct {
    FicpParseState_t state;
    uint8_t         *buf;
    size_t           cap;
    size_t           pos;
    uint16_t         expected_len;
    uint32_t         frames_ok;
    uint32_t         crc_errors;
    uint32_t         framing_errors;
} FicpParser_t;

typedef struct {
    bool                connected;
    uint32_t            state_rev;
    uint16_t            tx_seq;
    uint16_t            peer_mtu;
    FicControlMode_t    control_mode;
    uint8_t             backlight_pct;
    FicLocalAudioMode_t local_audio;
    uint32_t            watchdog_ms;
    uint32_t            last_rx_ms;
    uint32_t            last_hb_tx_ms;
    uint32_t            watchdog_trips;
} FicpSessionState_t;

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} FicpUtcCalendar_t;

typedef void (*ficp_on_frame_cb_t)(const FicpFrameView_t *frame, void *ctx);
typedef void (*ficp_on_bad_crc_cb_t)(uint16_t seq, uint8_t flags, uint8_t msg_id, void *ctx);

uint16_t ficp_crc16_ccitt(const uint8_t *data, size_t len);
uint32_t ficp_crc32_update(uint32_t crc_state, const uint8_t *data, size_t len);
uint32_t ficp_crc32_finish(uint32_t crc_state);

size_t ficp_encode_frame(uint8_t *out,
                         size_t out_cap,
                         uint8_t msg_id,
                         uint8_t flags,
                         uint16_t seq,
                         const uint8_t *payload,
                         uint16_t payload_len);

size_t ficp_pack_heartbeat(uint8_t *out,
                           size_t out_cap,
                           uint32_t uptime_ms,
                           uint32_t state_rev,
                           FicControlMode_t control_mode,
                           uint8_t flags);

bool ficp_unpack_heartbeat(const uint8_t *payload,
                           size_t payload_len,
                           uint32_t *uptime_ms_out,
                           uint16_t *state_rev_lo16_out,
                           uint8_t *control_mode_out,
                           uint8_t *flags_out);

size_t ficp_pack_file_chunk_hdr(uint8_t *out, size_t out_cap, uint16_t handle, uint32_t offset);
bool ficp_unpack_file_chunk_hdr(const uint8_t *payload,
                                size_t payload_len,
                                uint16_t *handle_out,
                                uint32_t *offset_out);

void ficp_parser_init(FicpParser_t *p, uint8_t *backing_buf, size_t backing_cap);
void ficp_parser_reset(FicpParser_t *p);
void ficp_parser_feed(FicpParser_t *p,
                      const uint8_t *data,
                      size_t len,
                      ficp_on_frame_cb_t on_frame,
                      ficp_on_bad_crc_cb_t on_bad_crc,
                      void *ctx);

void ficp_session_init(FicpSessionState_t *s, uint32_t initial_rev);
void ficp_session_on_connect(FicpSessionState_t *s, uint32_t now_ms);
void ficp_session_on_disconnect(FicpSessionState_t *s);
uint16_t ficp_session_next_seq(FicpSessionState_t *s);
uint32_t ficp_session_bump_rev(FicpSessionState_t *s);
bool ficp_session_check_base_rev(const FicpSessionState_t *s, uint32_t base_rev);

bool ficp_session_apply_control_mode(FicpSessionState_t *s,
                                     FicControlMode_t mode,
                                     uint8_t backlight_pct,
                                     FicLocalAudioMode_t audio_mode,
                                     uint32_t watchdog_ms,
                                     uint32_t now_ms);
bool ficp_session_reclaim_local(FicpSessionState_t *s);
void ficp_session_note_rx(FicpSessionState_t *s, uint32_t now_ms);
bool ficp_session_check_watchdog(FicpSessionState_t *s, uint32_t now_ms);

const char *ficp_control_mode_to_str(FicControlMode_t mode);
bool ficp_control_mode_from_str(const char *str, FicControlMode_t *out_mode);
const char *ficp_audio_mode_to_str(FicLocalAudioMode_t mode);
bool ficp_audio_mode_from_str(const char *str, FicLocalAudioMode_t *out_mode);
const char *ficp_err_name(FicpErrCode_t code);

bool ficp_unix_ms_to_utc_calendar(int64_t unix_utc_ms, FicpUtcCalendar_t *out);

#ifdef __cplusplus
}
#endif

#endif // FIC_BLE_PROTOCOL_H
