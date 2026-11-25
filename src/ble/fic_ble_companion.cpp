#include "fic_ble_internal.h"
#include "../net/fic_web_portal.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)
#include <esp_bt.h>
#include <esp32-hal-bt.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_att.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

extern "C" void ble_store_config_init(void);
#endif // CONFIG_BT_NIMBLE_ENABLED

static FicBleRuntimeState_t s_state = {
    {},
    false,
    false,
    false,
    false,
    false,
    0U,
    0U,
    0U,
    0U,
    0U,
    {},
    {},
    {},
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    0U,
    nullptr,
    portMUX_INITIALIZER_UNLOCKED,
    0U,
    0U,
    0U,
    0U
};

#if defined(CONFIG_BT_NIMBLE_ENABLED)
static volatile bool s_nimble_synced       = false;
static volatile bool s_host_task_running   = false;
static uint8_t       s_own_addr_type       = BLE_OWN_ADDR_PUBLIC;
static struct ble_gatt_chr_def s_gatt_chrs[5];
static struct ble_gatt_svc_def s_gatt_svcs[2];

// 128-bit UUIDs in Little-Endian byte order matching FICP v1.0.0:
// Service:   5f494350-0001-4b10-a8e1-464943500001
// CTRL_RX:   5f494350-0002-4b10-a8e1-464943500001
// CTRL_TX:   5f494350-0003-4b10-a8e1-464943500001
// STREAM_RX: 5f494350-0004-4b10-a8e1-464943500001
// STREAM_TX: 5f494350-0005-4b10-a8e1-464943500001
static const ble_uuid128_t kUuidSvc = {
    {BLE_UUID_TYPE_128},
    {0x01, 0x00, 0x50, 0x43, 0x49, 0x46, 0xe1, 0xa8, 0x10, 0x4b, 0x01, 0x00, 0x50, 0x43, 0x49, 0x5f}
};
static const ble_uuid128_t kUuidCtrlRx = {
    {BLE_UUID_TYPE_128},
    {0x01, 0x00, 0x50, 0x43, 0x49, 0x46, 0xe1, 0xa8, 0x10, 0x4b, 0x02, 0x00, 0x50, 0x43, 0x49, 0x5f}
};
static const ble_uuid128_t kUuidCtrlTx = {
    {BLE_UUID_TYPE_128},
    {0x01, 0x00, 0x50, 0x43, 0x49, 0x46, 0xe1, 0xa8, 0x10, 0x4b, 0x03, 0x00, 0x50, 0x43, 0x49, 0x5f}
};
static const ble_uuid128_t kUuidStreamRx = {
    {BLE_UUID_TYPE_128},
    {0x01, 0x00, 0x50, 0x43, 0x49, 0x46, 0xe1, 0xa8, 0x10, 0x4b, 0x04, 0x00, 0x50, 0x43, 0x49, 0x5f}
};
static const ble_uuid128_t kUuidStreamTx = {
    {BLE_UUID_TYPE_128},
    {0x01, 0x00, 0x50, 0x43, 0x49, 0x46, 0xe1, 0xa8, 0x10, 0x4b, 0x05, 0x00, 0x50, 0x43, 0x49, 0x5f}
};

static int gap_event_cb(struct ble_gap_event *event, void *arg);
#endif // CONFIG_BT_NIMBLE_ENABLED

FicBleRuntimeState_t *fic_ble_state(void)
{
    return &s_state;
}

void fic_ble_log(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (s_state.cfg.log_cb != nullptr) {
        s_state.cfg.log_cb("%s", buf);
    } else {
        printf("%s", buf);
    }
}

static void *alloc_psram_pref(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return p;
}

static void free_buffers(void)
{
    if (s_state.ctrl_rx_ring_buf) {
        heap_caps_free(s_state.ctrl_rx_ring_buf);
        s_state.ctrl_rx_ring_buf = nullptr;
    }
    if (s_state.parser_buf) {
        heap_caps_free(s_state.parser_buf);
        s_state.parser_buf = nullptr;
    }
    if (s_state.tx_frame_buf) {
        heap_caps_free(s_state.tx_frame_buf);
        s_state.tx_frame_buf = nullptr;
    }
    if (s_state.stream_rx_scratch) {
        heap_caps_free(s_state.stream_rx_scratch);
        s_state.stream_rx_scratch = nullptr;
    }
    s_state.stream_rx_scratch_cap = 0U;
    fic_ring_buf_init(&s_state.ctrl_rx_ring, nullptr, 0U);
    ficp_parser_init(&s_state.parser, nullptr, 0U);
}

static bool alloc_buffers(void)
{
    free_buffers();
    size_t rx_cap = s_state.cfg.ctrl_rx_ring_bytes;
    if (rx_cap < 2048U) {
        rx_cap = 4096U;
    }
    s_state.ctrl_rx_ring_buf  = static_cast<uint8_t *>(alloc_psram_pref(rx_cap));
    s_state.parser_buf        = static_cast<uint8_t *>(alloc_psram_pref(FICP_MAX_FRAME_LEN));
    s_state.tx_frame_buf      = static_cast<uint8_t *>(alloc_psram_pref(FICP_MAX_FRAME_LEN));
    s_state.stream_rx_scratch = static_cast<uint8_t *>(alloc_psram_pref(FICP_ATT_MTU_PREFERRED));

    if (!s_state.ctrl_rx_ring_buf || !s_state.parser_buf ||
        !s_state.tx_frame_buf || !s_state.stream_rx_scratch) {
        free_buffers();
        return false;
    }

    s_state.stream_rx_scratch_cap = FICP_ATT_MTU_PREFERRED;
    fic_ring_buf_init(&s_state.ctrl_rx_ring, s_state.ctrl_rx_ring_buf, rx_cap);
    ficp_parser_init(&s_state.parser, s_state.parser_buf, FICP_MAX_FRAME_LEN);
    return true;
}

#if defined(CONFIG_BT_NIMBLE_ENABLED)
static int gatt_ctrl_rx_access_cb(uint16_t conn_handle,
                                  uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || !s_state.ctrl_rx_ring_buf) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    struct os_mbuf *m = ctxt->om;
    while (m != nullptr) {
        if (m->om_len > 0U) {
            taskENTER_CRITICAL(&s_state.rx_spinlock);
            (void)fic_ring_buf_push(&s_state.ctrl_rx_ring, m->om_data, m->om_len, false);
            taskEXIT_CRITICAL(&s_state.rx_spinlock);
        }
        m = SLIST_NEXT(m, om_next);
    }
    return 0;
}

static int gatt_notify_only_access_cb(uint16_t conn_handle,
                                      uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt,
                                      void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
}

static int gatt_stream_rx_access_cb(uint16_t conn_handle,
                                    uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt,
                                    void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    const uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
    if (total == 0U || !s_state.stream_rx_scratch || total > s_state.stream_rx_scratch_cap) {
        return 0;
    }
    uint16_t out_len = 0U;
    if (ble_hs_mbuf_to_flat(ctxt->om, s_state.stream_rx_scratch, total, &out_len) == 0 && out_len > 0U) {
        s_state.stream_rx_bytes += (uint32_t)out_len;
        s_state.stream_rx_packets++;
        ficp_session_note_rx(&s_state.session, (uint32_t)millis());
        if (s_state.cfg.stream_rx_cb != nullptr) {
            s_state.cfg.stream_rx_cb(s_state.stream_rx_scratch, (size_t)out_len);
        }
    }
    return 0;
}

static void build_gatt_defs(void)
{
    memset(s_gatt_chrs, 0, sizeof(s_gatt_chrs));
    memset(s_gatt_svcs, 0, sizeof(s_gatt_svcs));

    size_t idx = 0U;
    s_gatt_chrs[idx].uuid       = &kUuidCtrlRx.u;
    s_gatt_chrs[idx].access_cb  = gatt_ctrl_rx_access_cb;
    s_gatt_chrs[idx].flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;
    s_gatt_chrs[idx].val_handle = &s_state.attr_ctrl_rx;
    idx++;

    s_gatt_chrs[idx].uuid       = &kUuidCtrlTx.u;
    s_gatt_chrs[idx].access_cb  = gatt_notify_only_access_cb;
    s_gatt_chrs[idx].flags      = BLE_GATT_CHR_F_NOTIFY;
    s_gatt_chrs[idx].val_handle = &s_state.attr_ctrl_tx;
    idx++;

    if (s_state.cfg.enable_stream_rx) {
        s_gatt_chrs[idx].uuid       = &kUuidStreamRx.u;
        s_gatt_chrs[idx].access_cb  = gatt_stream_rx_access_cb;
        s_gatt_chrs[idx].flags      = BLE_GATT_CHR_F_WRITE_NO_RSP;
        s_gatt_chrs[idx].val_handle = &s_state.attr_stream_rx;
        idx++;
    }
    if (s_state.cfg.enable_stream_tx) {
        s_gatt_chrs[idx].uuid       = &kUuidStreamTx.u;
        s_gatt_chrs[idx].access_cb  = gatt_notify_only_access_cb;
        s_gatt_chrs[idx].flags      = BLE_GATT_CHR_F_NOTIFY;
        s_gatt_chrs[idx].val_handle = &s_state.attr_stream_tx;
        idx++;
    }

    s_gatt_svcs[0].type            = BLE_GATT_SVC_TYPE_PRIMARY;
    s_gatt_svcs[0].uuid            = &kUuidSvc.u;
    s_gatt_svcs[0].characteristics = s_gatt_chrs;
}

static bool start_advertising(void)
{
    if (!s_state.active || !s_nimble_synced) {
        return false;
    }

    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags               = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128            = &kUuidSvc;
    adv_fields.num_uuids128        = 1U;
    adv_fields.uuids128_is_complete = 1U;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        fic_ble_log("[BLE] adv_set_fields failed: rc=%d\n", rc);
        return false;
    }

    const char *dev_name = (s_state.cfg.device_name && s_state.cfg.device_name[0] != '\0')
                               ? s_state.cfg.device_name
                               : "FIC-Instrument";
    size_t name_len = strlen(dev_name);
    if (name_len > 26U) {
        name_len = 26U;
    }

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name                  = reinterpret_cast<const uint8_t *>(dev_name);
    rsp_fields.name_len              = (uint8_t)name_len;
    rsp_fields.name_is_complete      = 1U;
    rsp_fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    rsp_fields.tx_pwr_lvl_is_present = 1U;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        fic_ble_log("[BLE] adv_rsp_set_fields failed: rc=%d\n", rc);
        return false;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        fic_ble_log("[BLE] adv_start failed: rc=%d\n", rc);
        return false;
    }
    return true;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_state.conn_handle          = event->connect.conn_handle;
                s_state.ctrl_tx_subscribed   = false;
                s_state.stream_tx_subscribed = false;
                ficp_session_on_connect(&s_state.session, (uint32_t)millis());
                ficp_parser_reset(&s_state.parser);
                fic_ble_log("[BLE] Peer connected (handle=%u)\n", (unsigned)s_state.conn_handle);
                if (s_state.cfg.conn_event_cb != nullptr) {
                    s_state.cfg.conn_event_cb(true, s_state.session.peer_mtu);
                }
            } else {
                (void)start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ficp_session_on_disconnect(&s_state.session);
            s_state.ctrl_tx_subscribed       = false;
            s_state.stream_tx_subscribed     = false;
            s_state.disconnect_cleanup_pending = true;
            fic_ble_log("[BLE] Peer disconnected (reason=%d)\n", event->disconnect.reason);
            if (s_state.cfg.conn_event_cb != nullptr) {
                s_state.cfg.conn_event_cb(false, s_state.session.peer_mtu);
            }
            if (s_state.active) {
                (void)start_advertising();
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            s_state.session.peer_mtu = event->mtu.value;
            fic_ble_log("[BLE] Negotiated ATT MTU: %u\n", (unsigned)event->mtu.value);
            if (s_state.cfg.conn_event_cb != nullptr) {
                s_state.cfg.conn_event_cb(s_state.session.connected, s_state.session.peer_mtu);
            }
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_state.attr_ctrl_tx) {
                s_state.ctrl_tx_subscribed = (event->subscribe.cur_notify != 0U);
            } else if (event->subscribe.attr_handle == s_state.attr_stream_tx) {
                s_state.stream_tx_subscribed = (event->subscribe.cur_notify != 0U);
            }
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (s_state.active && !s_state.session.connected) {
                (void)start_advertising();
            }
            return 0;

        default:
            return 0;
    }
}

static void on_ble_reset(int reason)
{
    s_nimble_synced = false;
    fic_ble_log("[BLE] Host reset (reason=%d)\n", reason);
}

static void on_ble_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    }
    if (rc != 0) {
        fic_ble_log("[BLE] Address infer failed: rc=%d\n", rc);
        return;
    }

    const char *dev_name = (s_state.cfg.device_name && s_state.cfg.device_name[0] != '\0')
                               ? s_state.cfg.device_name
                               : "FIC-Instrument";
    (void)ble_svc_gap_device_name_set(dev_name);

    s_nimble_synced = true;
    (void)start_advertising();
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    s_host_task_running = false;
    vTaskDelete(nullptr);
}
#endif // CONFIG_BT_NIMBLE_ENABLED

static bool web_hook_is_active(void)
{
    return fic_ble_companion_is_active();
}

static void web_hook_suspend(void)
{
    fic_ble_companion_stop();
}

static void web_hook_resume(void)
{
    (void)fic_ble_companion_start();
}

extern "C" void fic_ble_companion_configure(const FicBleCompanionCfg_t *cfg)
{
    if (cfg != nullptr) {
        s_state.cfg        = *cfg;
        s_state.configured = true;
    } else {
        memset(&s_state.cfg, 0, sizeof(s_state.cfg));
        s_state.configured = false;
    }
    ficp_session_init(&s_state.session, (s_state.session.state_rev > 0U) ? s_state.session.state_rev : 1U);
    fic_web_portal_register_ble_hooks(web_hook_is_active, web_hook_suspend, web_hook_resume);
}

extern "C" bool fic_ble_companion_start(void)
{
#if !defined(CONFIG_BT_NIMBLE_ENABLED)
    fic_ble_log("[BLE] NimBLE is disabled in SDK configuration.\n");
    return false;
#else
    if (s_state.active) {
        return true;
    }
    if (fic_web_portal_is_active() || fic_web_portal_is_starting()) {
        fic_ble_log("[BLE] Refusing start while Wi-Fi portal is active.\n");
        return false;
    }
    if (btMemReleased(BT_MODE_BLE)) {
        fic_ble_log("[BLE] BT BLE controller memory was released; include <ble/fic_ble_enable.h> in sketch.\n");
        return false;
    }

    if (s_state.tx_mux == nullptr) {
        s_state.tx_mux = xSemaphoreCreateMutex();
        if (s_state.tx_mux == nullptr) {
            return false;
        }
    }
    if (!alloc_buffers()) {
        fic_ble_log("[BLE] PSRAM buffer allocation failed.\n");
        return false;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        fic_ble_log("[BLE] nimble_port_init failed: %d\n", (int)err);
        free_buffers();
        return false;
    }

    s_nimble_synced                  = false;
    s_state.ctrl_tx_subscribed       = false;
    s_state.stream_tx_subscribed     = false;
    s_state.disconnect_cleanup_pending = false;
    ficp_session_on_disconnect(&s_state.session);

    ble_hs_cfg.reset_cb    = on_ble_reset;
    ble_hs_cfg.sync_cb     = on_ble_sync;
    ble_hs_cfg.sm_io_cap   = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding  = 0U;
    ble_hs_cfg.sm_mitm     = 0U;
    ble_hs_cfg.sm_sc       = 1U;

    ble_gatts_reset();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    build_gatt_defs();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_gatt_svcs);
    }
    if (rc != 0) {
        fic_ble_log("[BLE] GATT service registration failed: rc=%d\n", rc);
        (void)nimble_port_deinit();
        free_buffers();
        return false;
    }

    (void)ble_att_set_preferred_mtu(FICP_ATT_MTU_PREFERRED);
    ble_store_config_init();

    s_state.active      = true;
    s_host_task_running = true;
    nimble_port_freertos_init(ble_host_task);

    uint32_t wait_ms = 0U;
    while (!s_nimble_synced && wait_ms < 2000U) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_ms += 10U;
    }

    const char *dev_name = (s_state.cfg.device_name && s_state.cfg.device_name[0] != '\0')
                               ? s_state.cfg.device_name
                               : "FIC-Instrument";
    fic_ble_log("[BLE] Started '%s' (synced=%d) | Internal Free: %u B (Largest: %u B) | PSRAM Free: %u KB\n",
                dev_name,
                s_nimble_synced ? 1 : 0,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024U));
    return s_nimble_synced;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

extern "C" void fic_ble_companion_stop(void)
{
#if defined(CONFIG_BT_NIMBLE_ENABLED)
    if (!s_state.active) {
        return;
    }

    if (s_state.session.control_mode == FIC_CTRL_SLAVE_DIMMED) {
        (void)fic_ble_companion_reclaim_local("radio_switch");
    }
    fic_ble_files_abort_all();

    if (s_state.session.connected) {
        (void)ble_gap_terminate(s_state.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    (void)ble_gap_adv_stop();

    s_state.active               = false;
    s_nimble_synced              = false;
    s_state.ctrl_tx_subscribed   = false;
    s_state.stream_tx_subscribed = false;
    ficp_session_on_disconnect(&s_state.session);

    (void)nimble_port_stop();
    uint32_t wait_ms = 0U;
    while (s_host_task_running && wait_ms < 500U) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_ms += 10U;
    }
    (void)nimble_port_deinit();

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        (void)esp_bt_controller_disable();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        (void)esp_bt_controller_deinit();
    }
    // Never call btMemRelease(BT_MODE_BLE) so subsequent fic_ble_companion_start() succeeds.

    free_buffers();
    fic_ble_log("[BLE] Stopped | Internal Free: %u B (Largest: %u B)\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
#endif // CONFIG_BT_NIMBLE_ENABLED
}

bool fic_ble_send_frame(uint8_t msg_id, uint8_t flags, const uint8_t *payload, uint16_t len)
{
#if !defined(CONFIG_BT_NIMBLE_ENABLED)
    (void)msg_id;
    (void)flags;
    (void)payload;
    (void)len;
    return false;
#else
    if (!s_state.active || !s_state.session.connected || !s_state.ctrl_tx_subscribed || !s_state.tx_frame_buf) {
        return false;
    }
    if (xSemaphoreTake(s_state.tx_mux, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }

    const uint16_t seq = ficp_session_next_seq(&s_state.session);
    const size_t total = ficp_encode_frame(s_state.tx_frame_buf, FICP_MAX_FRAME_LEN,
                                           msg_id, flags, seq, payload, len);
    if (total == 0U) {
        xSemaphoreGive(s_state.tx_mux);
        return false;
    }

    size_t max_chunk = (s_state.session.peer_mtu > 3U) ? (size_t)(s_state.session.peer_mtu - 3U) : 20U;
    if (max_chunk > FICP_SINGLE_NOTIFY_MAX) {
        max_chunk = FICP_SINGLE_NOTIFY_MAX;
    }

    bool ok = true;
    size_t sent = 0U;
    while (sent < total) {
        size_t n = total - sent;
        if (n > max_chunk) {
            n = max_chunk;
        }
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_state.tx_frame_buf[sent], (uint16_t)n);
        if (!om) {
            ok = false;
            break;
        }
        // ble_gatts_notify_custom ALWAYS frees `om`, even when returning an error.
        const int rc = ble_gatts_notify_custom(s_state.conn_handle, s_state.attr_ctrl_tx, om);
        if (rc != 0) {
            ok = false;
            break;
        }
        sent += n;
    }

    if (ok) {
        s_state.frames_tx++;
    }
    xSemaphoreGive(s_state.tx_mux);
    return ok;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

bool fic_ble_send_json_frame(uint8_t msg_id, const JsonDocument &doc)
{
    const size_t needed = measureJson(doc);
    if (needed == 0U || needed > FICP_MAX_PAYLOAD_LEN) {
        return false;
    }
    String json_str;
    json_str.reserve(needed + 2U);
    serializeJson(doc, json_str);
    return fic_ble_send_frame(msg_id,
                              FICP_FLAG_IS_JSON,
                              reinterpret_cast<const uint8_t *>(json_str.c_str()),
                              (uint16_t)json_str.length());
}

extern "C" bool fic_ble_companion_send_stream_tx(const uint8_t *data, size_t len)
{
#if !defined(CONFIG_BT_NIMBLE_ENABLED)
    (void)data;
    (void)len;
    return false;
#else
    if (!s_state.active || !s_state.cfg.enable_stream_tx || !s_state.session.connected ||
        !s_state.stream_tx_subscribed || !data || len == 0U) {
        return false;
    }
    if (xSemaphoreTake(s_state.tx_mux, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }
    size_t max_chunk = (s_state.session.peer_mtu > 3U) ? (size_t)(s_state.session.peer_mtu - 3U) : 20U;
    if (max_chunk > FICP_SINGLE_NOTIFY_MAX) {
        max_chunk = FICP_SINGLE_NOTIFY_MAX;
    }
    bool ok = true;
    size_t off = 0U;
    while (off < len) {
        size_t n = len - off;
        if (n > max_chunk) n = max_chunk;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&data[off], (uint16_t)n);
        if (!om) {
            ok = false;
            break;
        }
        const int rc = ble_gatts_notify_custom(s_state.conn_handle, s_state.attr_stream_tx, om);
        if (rc != 0) {
            ok = false;
            break;
        }
        off += n;
    }
    xSemaphoreGive(s_state.tx_mux);
    return ok;
#endif // CONFIG_BT_NIMBLE_ENABLED
}

extern "C" bool fic_ble_companion_is_active(void)
{
    return s_state.active;
}

extern "C" bool fic_ble_companion_is_connected(void)
{
    return s_state.active && s_state.session.connected;
}

extern "C" FicControlMode_t fic_ble_companion_get_control_mode(void)
{
    return s_state.session.control_mode;
}

extern "C" uint32_t fic_ble_companion_get_state_rev(void)
{
    return s_state.session.state_rev;
}

extern "C" void fic_ble_companion_get_stats(FicBleStats_t *out)
{
    if (!out) {
        return;
    }
    out->active               = s_state.active;
    out->connected            = s_state.session.connected;
    out->ctrl_tx_subscribed   = s_state.ctrl_tx_subscribed;
    out->stream_tx_subscribed = s_state.stream_tx_subscribed;
    out->control_mode         = s_state.session.control_mode;
    out->state_rev            = s_state.session.state_rev;
    out->peer_mtu             = s_state.session.peer_mtu;
    out->frames_rx            = s_state.parser.frames_ok;
    out->frames_tx            = s_state.frames_tx;
    out->crc_errors           = s_state.parser.crc_errors;
    out->framing_errors       = s_state.parser.framing_errors;
    out->ring_drops           = s_state.ctrl_rx_ring.drop_count;
    out->watchdog_trips       = s_state.session.watchdog_trips;
    out->stream_rx_bytes      = s_state.stream_rx_bytes;
    out->stream_rx_packets    = s_state.stream_rx_packets;
}
