#include "fic_ble_protocol.h"

#include <string.h>

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

static void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
    p[2] = (uint8_t)((v >> 16U) & 0xFFU);
    p[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

uint16_t ficp_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    if (!data && len > 0U) {
        return crc;
    }
    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

uint32_t ficp_crc32_update(uint32_t crc_state, const uint8_t *data, size_t len)
{
    uint32_t crc = crc_state ^ 0xFFFFFFFFUL;
    if (!data && len > 0U) {
        return crc_state;
    }
    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t ficp_crc32_finish(uint32_t crc_state)
{
    return crc_state;
}

size_t ficp_encode_frame(uint8_t *out,
                         size_t out_cap,
                         uint8_t msg_id,
                         uint8_t flags,
                         uint16_t seq,
                         const uint8_t *payload,
                         uint16_t payload_len)
{
    if (!out || payload_len > FICP_MAX_PAYLOAD_LEN) {
        return 0U;
    }
    if (payload_len > 0U && !payload) {
        return 0U;
    }
    const size_t total_len = (size_t)FICP_OVERHEAD_LEN + (size_t)payload_len;
    if (out_cap < total_len) {
        return 0U;
    }

    out[0] = FICP_SYNC_0;
    out[1] = FICP_SYNC_1;
    out[2] = FICP_WIRE_VER;
    out[3] = msg_id;
    out[4] = flags;
    write_u16_le(&out[5], seq);
    write_u16_le(&out[7], payload_len);
    if (payload_len > 0U) {
        memcpy(&out[9], payload, payload_len);
    }

    const uint16_t crc = ficp_crc16_ccitt(&out[2], (size_t)(FICP_HEADER_LEN - 2U) + (size_t)payload_len);
    write_u16_le(&out[9U + payload_len], crc);
    return total_len;
}

size_t ficp_pack_heartbeat(uint8_t *out,
                           size_t out_cap,
                           uint32_t uptime_ms,
                           uint32_t state_rev,
                           FicControlMode_t control_mode,
                           uint8_t flags)
{
    if (!out || out_cap < FICP_HEARTBEAT_PAYLOAD_LEN) {
        return 0U;
    }
    write_u32_le(&out[0], uptime_ms);
    write_u16_le(&out[4], (uint16_t)(state_rev & 0xFFFFU));
    out[6] = (uint8_t)control_mode;
    out[7] = flags;
    return FICP_HEARTBEAT_PAYLOAD_LEN;
}

bool ficp_unpack_heartbeat(const uint8_t *payload,
                           size_t payload_len,
                           uint32_t *uptime_ms_out,
                           uint16_t *state_rev_lo16_out,
                           uint8_t *control_mode_out,
                           uint8_t *flags_out)
{
    if (!payload || payload_len < FICP_HEARTBEAT_PAYLOAD_LEN) {
        return false;
    }
    if (uptime_ms_out)        *uptime_ms_out        = read_u32_le(&payload[0]);
    if (state_rev_lo16_out)   *state_rev_lo16_out   = read_u16_le(&payload[4]);
    if (control_mode_out)     *control_mode_out     = payload[6];
    if (flags_out)            *flags_out            = payload[7];
    return true;
}

size_t ficp_pack_file_chunk_hdr(uint8_t *out, size_t out_cap, uint16_t handle, uint32_t offset)
{
    if (!out || out_cap < FICP_FILE_CHUNK_HDR_LEN) {
        return 0U;
    }
    write_u16_le(&out[0], handle);
    write_u32_le(&out[2], offset);
    return FICP_FILE_CHUNK_HDR_LEN;
}

bool ficp_unpack_file_chunk_hdr(const uint8_t *payload,
                                size_t payload_len,
                                uint16_t *handle_out,
                                uint32_t *offset_out)
{
    if (!payload || payload_len < FICP_FILE_CHUNK_HDR_LEN) {
        return false;
    }
    if (handle_out) *handle_out = read_u16_le(&payload[0]);
    if (offset_out) *offset_out = read_u32_le(&payload[2]);
    return true;
}

void ficp_parser_init(FicpParser_t *p, uint8_t *backing_buf, size_t backing_cap)
{
    if (!p) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = FICP_PARSE_SYNC0;
    if (backing_buf != NULL && backing_cap >= FICP_MAX_FRAME_LEN) {
        p->buf = backing_buf;
        p->cap = backing_cap;
    }
}

void ficp_parser_reset(FicpParser_t *p)
{
    if (!p) {
        return;
    }
    p->state        = FICP_PARSE_SYNC0;
    p->pos          = 0U;
    p->expected_len = 0U;
}

static void parser_rescan_tail(FicpParser_t *p,
                               size_t start_idx,
                               ficp_on_frame_cb_t on_frame,
                               ficp_on_bad_crc_cb_t on_bad_crc,
                               void *ctx)
{
    uint8_t replay[FICP_MAX_FRAME_LEN];
    if (!p || !p->buf || start_idx >= p->pos || p->pos > sizeof(replay)) {
        ficp_parser_reset(p);
        return;
    }
    const size_t remain = p->pos - start_idx;
    memcpy(replay, &p->buf[start_idx], remain);
    ficp_parser_reset(p);
    ficp_parser_feed(p, replay, remain, on_frame, on_bad_crc, ctx);
}

void ficp_parser_feed(FicpParser_t *p,
                      const uint8_t *data,
                      size_t len,
                      ficp_on_frame_cb_t on_frame,
                      ficp_on_bad_crc_cb_t on_bad_crc,
                      void *ctx)
{
    if (!p || !p->buf || p->cap < FICP_MAX_FRAME_LEN || !data || len == 0U) {
        return;
    }

    for (size_t i = 0U; i < len; i++) {
        const uint8_t b = data[i];
        switch (p->state) {
            case FICP_PARSE_SYNC0:
                if (b == FICP_SYNC_0) {
                    p->buf[0] = b;
                    p->pos    = 1U;
                    p->state  = FICP_PARSE_SYNC1;
                }
                break;

            case FICP_PARSE_SYNC1:
                if (b == FICP_SYNC_1) {
                    p->buf[1] = b;
                    p->pos    = 2U;
                    p->state  = FICP_PARSE_HDR;
                } else if (b == FICP_SYNC_0) {
                    p->buf[0] = FICP_SYNC_0;
                    p->pos    = 1U;
                } else {
                    ficp_parser_reset(p);
                }
                break;

            case FICP_PARSE_HDR:
                p->buf[p->pos++] = b;
                if (p->pos == FICP_HEADER_LEN) {
                    const uint8_t  ver  = p->buf[2];
                    const uint16_t plen = read_u16_le(&p->buf[7]);
                    if (ver != FICP_WIRE_VER || plen > FICP_MAX_PAYLOAD_LEN) {
                        p->framing_errors++;
                        parser_rescan_tail(p, 1U, on_frame, on_bad_crc, ctx);
                    } else {
                        p->expected_len = (uint16_t)(FICP_OVERHEAD_LEN + plen);
                        p->state        = FICP_PARSE_BODY;
                    }
                }
                break;

            case FICP_PARSE_BODY:
                p->buf[p->pos++] = b;
                if (p->pos == (size_t)p->expected_len) {
                    const uint8_t  ver    = p->buf[2];
                    const uint8_t  msg_id = p->buf[3];
                    const uint8_t  flags  = p->buf[4];
                    const uint16_t seq    = read_u16_le(&p->buf[5]);
                    const uint16_t plen   = read_u16_le(&p->buf[7]);
                    const uint16_t rx_crc = read_u16_le(&p->buf[9U + plen]);
                    const uint16_t calc_crc =
                        ficp_crc16_ccitt(&p->buf[2], (size_t)(FICP_HEADER_LEN - 2U) + (size_t)plen);

                    if (rx_crc == calc_crc) {
                        p->frames_ok++;
                        if (on_frame) {
                            FicpFrameView_t view = {
                                ver,
                                msg_id,
                                flags,
                                seq,
                                plen,
                                (plen > 0U) ? &p->buf[9] : NULL
                            };
                            on_frame(&view, ctx);
                        }
                        ficp_parser_reset(p);
                    } else {
                        p->crc_errors++;
                        if (on_bad_crc && ((flags & FICP_FLAG_ACK_REQ) != 0U)) {
                            on_bad_crc(seq, flags, msg_id, ctx);
                        }
                        parser_rescan_tail(p, 1U, on_frame, on_bad_crc, ctx);
                    }
                }
                break;

            default:
                ficp_parser_reset(p);
                break;
        }
    }
}

void ficp_session_init(FicpSessionState_t *s, uint32_t initial_rev)
{
    if (!s) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->state_rev     = (initial_rev == 0U) ? 1U : initial_rev;
    s->tx_seq        = 1U;
    s->peer_mtu      = 23U;
    s->control_mode  = FIC_CTRL_STANDALONE;
    s->backlight_pct = 100U;
    s->local_audio   = FIC_AUDIO_ACTIVE;
    s->watchdog_ms   = FICP_WATCHDOG_DEFAULT_MS;
}

void ficp_session_on_connect(FicpSessionState_t *s, uint32_t now_ms)
{
    if (!s) {
        return;
    }
    s->connected     = true;
    s->peer_mtu      = 23U;
    s->last_rx_ms    = now_ms;
    s->last_hb_tx_ms = now_ms;
}

void ficp_session_on_disconnect(FicpSessionState_t *s)
{
    if (!s) {
        return;
    }
    s->connected = false;
    s->peer_mtu  = 23U;
}

uint16_t ficp_session_next_seq(FicpSessionState_t *s)
{
    if (!s) {
        return 1U;
    }
    if (s->tx_seq == 0U) {
        s->tx_seq = 1U;
    }
    const uint16_t cur = s->tx_seq;
    s->tx_seq = (s->tx_seq == 0xFFFFU) ? 1U : (uint16_t)(s->tx_seq + 1U);
    return cur;
}

uint32_t ficp_session_bump_rev(FicpSessionState_t *s)
{
    if (!s) {
        return 1U;
    }
    s->state_rev++;
    if (s->state_rev == 0U) {
        s->state_rev = 1U;
    }
    return s->state_rev;
}

bool ficp_session_check_base_rev(const FicpSessionState_t *s, uint32_t base_rev)
{
    if (!s) {
        return false;
    }
    return (base_rev == 0U) || (base_rev == s->state_rev);
}

bool ficp_session_apply_control_mode(FicpSessionState_t *s,
                                     FicControlMode_t mode,
                                     uint8_t backlight_pct,
                                     FicLocalAudioMode_t audio_mode,
                                     uint32_t watchdog_ms,
                                     uint32_t now_ms)
{
    if (!s) {
        return false;
    }
    if ((uint8_t)mode > (uint8_t)FIC_CTRL_SLAVE_DIMMED) {
        return false;
    }

    uint32_t wd = (watchdog_ms == 0U) ? FICP_WATCHDOG_DEFAULT_MS : watchdog_ms;
    if (wd < FICP_WATCHDOG_MIN_MS) wd = FICP_WATCHDOG_MIN_MS;
    if (wd > FICP_WATCHDOG_MAX_MS) wd = FICP_WATCHDOG_MAX_MS;

    uint8_t bl = backlight_pct;
    FicLocalAudioMode_t aud = audio_mode;

    if (mode == FIC_CTRL_SLAVE_DIMMED) {
        if (bl == 0U) {
            bl = FICP_SLAVE_BACKLIGHT_DEFAULT_PCT;
        }
        if (bl < FICP_SLAVE_BACKLIGHT_MIN_PCT) bl = FICP_SLAVE_BACKLIGHT_MIN_PCT;
        if (bl > FICP_SLAVE_BACKLIGHT_MAX_PCT) bl = FICP_SLAVE_BACKLIGHT_MAX_PCT;
        if (aud != FIC_AUDIO_FOLLOW && aud != FIC_AUDIO_ACTIVE) {
            aud = FIC_AUDIO_MUTED;
        }
    } else if (mode == FIC_CTRL_STANDALONE) {
        bl  = 100U;
        aud = FIC_AUDIO_ACTIVE;
    } else {
        if (bl == 0U) bl = 100U;
        if (bl > 100U) bl = 100U;
    }

    s->control_mode  = mode;
    s->backlight_pct = bl;
    s->local_audio   = aud;
    s->watchdog_ms   = wd;
    s->last_rx_ms    = now_ms;
    (void)ficp_session_bump_rev(s);
    return true;
}

bool ficp_session_reclaim_local(FicpSessionState_t *s)
{
    if (!s) {
        return false;
    }
    const bool changed = (s->control_mode != FIC_CTRL_STANDALONE) ||
                         (s->backlight_pct != 100U) ||
                         (s->local_audio != FIC_AUDIO_ACTIVE);
    s->control_mode  = FIC_CTRL_STANDALONE;
    s->backlight_pct = 100U;
    s->local_audio   = FIC_AUDIO_ACTIVE;
    (void)ficp_session_bump_rev(s);
    return changed;
}

void ficp_session_note_rx(FicpSessionState_t *s, uint32_t now_ms)
{
    if (!s) {
        return;
    }
    s->last_rx_ms = now_ms;
}

bool ficp_session_check_watchdog(FicpSessionState_t *s, uint32_t now_ms)
{
    if (!s || s->control_mode != FIC_CTRL_SLAVE_DIMMED) {
        return false;
    }
    const uint32_t elapsed = (uint32_t)(now_ms - s->last_rx_ms);
    if (!s->connected || elapsed >= s->watchdog_ms) {
        s->control_mode  = FIC_CTRL_STANDALONE;
        s->backlight_pct = 100U;
        s->local_audio   = FIC_AUDIO_ACTIVE;
        s->watchdog_trips++;
        (void)ficp_session_bump_rev(s);
        return true;
    }
    return false;
}

const char *ficp_control_mode_to_str(FicControlMode_t mode)
{
    switch (mode) {
        case FIC_CTRL_STANDALONE:       return "standalone";
        case FIC_CTRL_COMPANION_MIRROR: return "companion_mirror";
        case FIC_CTRL_SLAVE_DIMMED:     return "slave_dimmed";
        default:                        return "standalone";
    }
}

bool ficp_control_mode_from_str(const char *str, FicControlMode_t *out_mode)
{
    if (!str || !out_mode) {
        return false;
    }
    if (strcmp(str, "standalone") == 0) {
        *out_mode = FIC_CTRL_STANDALONE;
        return true;
    }
    if (strcmp(str, "companion_mirror") == 0) {
        *out_mode = FIC_CTRL_COMPANION_MIRROR;
        return true;
    }
    if (strcmp(str, "slave_dimmed") == 0) {
        *out_mode = FIC_CTRL_SLAVE_DIMMED;
        return true;
    }
    return false;
}

const char *ficp_audio_mode_to_str(FicLocalAudioMode_t mode)
{
    switch (mode) {
        case FIC_AUDIO_ACTIVE: return "active";
        case FIC_AUDIO_MUTED:  return "muted";
        case FIC_AUDIO_FOLLOW: return "follow_instrument";
        default:               return "active";
    }
}

bool ficp_audio_mode_from_str(const char *str, FicLocalAudioMode_t *out_mode)
{
    if (!str || !out_mode) {
        return false;
    }
    if (strcmp(str, "active") == 0) {
        *out_mode = FIC_AUDIO_ACTIVE;
        return true;
    }
    if (strcmp(str, "muted") == 0) {
        *out_mode = FIC_AUDIO_MUTED;
        return true;
    }
    if (strcmp(str, "follow_instrument") == 0) {
        *out_mode = FIC_AUDIO_FOLLOW;
        return true;
    }
    return false;
}

const char *ficp_err_name(FicpErrCode_t code)
{
    switch (code) {
        case FICP_ERR_NONE:            return "NONE";
        case FICP_ERR_BAD_CRC:         return "ERR_BAD_CRC";
        case FICP_ERR_UNSUPPORTED_MSG: return "ERR_UNSUPPORTED_MSG";
        case FICP_ERR_INVALID_PARAM:   return "ERR_INVALID_PARAM";
        case FICP_ERR_REV_CONFLICT:    return "ERR_REV_CONFLICT";
        case FICP_ERR_BUSY_LOCKED:     return "ERR_BUSY_LOCKED";
        case FICP_ERR_STORAGE_FAIL:    return "ERR_STORAGE_FAIL";
        case FICP_ERR_UNAUTHORIZED:    return "ERR_UNAUTHORIZED";
        default:                       return "ERR_UNKNOWN";
    }
}

bool ficp_unix_ms_to_utc_calendar(int64_t unix_utc_ms, FicpUtcCalendar_t *out)
{
    if (!out || unix_utc_ms < 0LL) {
        return false;
    }
    const int64_t total_sec = unix_utc_ms / 1000LL;
    const int64_t days      = total_sec / 86400LL;
    const int64_t rem_sec   = total_sec % 86400LL;

    // Howard Hinnant civil-from-days algorithm (0 = 1970-01-01)
    const int64_t z   = days + 719468LL;
    const int64_t era = (z >= 0LL ? z : z - 146096LL) / 146097LL;
    const uint32_t doe = (uint32_t)(z - era * 146097LL);
    const uint32_t yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
    int64_t y = (int64_t)yoe + era * 400LL;
    const uint32_t doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);
    const uint32_t mp  = (5U * doy + 2U) / 153U;
    const uint32_t d   = doy - (153U * mp + 2U) / 5U + 1U;
    const uint32_t m   = (mp < 10U) ? (mp + 3U) : (mp - 9U);
    y += (m <= 2U) ? 1LL : 0LL;

    if (y < 2000LL || y > 2099LL) {
        return false;
    }

    out->year   = (uint16_t)y;
    out->month  = (uint8_t)m;
    out->day    = (uint8_t)d;
    out->hour   = (uint8_t)(rem_sec / 3600LL);
    out->minute = (uint8_t)((rem_sec % 3600LL) / 60LL);
    out->second = (uint8_t)(rem_sec % 60LL);
    return true;
}
