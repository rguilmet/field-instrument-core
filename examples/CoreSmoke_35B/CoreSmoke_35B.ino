/*
 * CoreSmoke_35B.ino - Standalone compile & hardware smoke test for Field_Instrument_Core v1.0.0
 */

#include <Arduino.h>
#include <FFat.h>
#include <Field_Instrument_Core.h>
#include <boards/waveshare_s3_35b.h>
#include <boards/waveshare_s3_349_v3.h>

static FicBoardProfile_t s_board;
static FicSen0696_t      s_sen0696;
static Ahrs9DofState_t   s_ahrs;
static FicRingLogger_t   s_ring_log;
static SemaphoreHandle_t s_fs_mux     = nullptr;
static lv_obj_t         *s_status_lbl = nullptr;

static void smoke_create_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    s_status_lbl = lv_label_create(scr);
    lv_label_set_text(s_status_lbl, "Field_Instrument_Core v1.0.0");
    lv_obj_center(s_status_lbl);
}

static bool save_smoke_screenshot(void)
{
    char path[96] = {0};
    FicScreenshotCfg_t cfg = {};
    cfg.primary_fs   = fic_fs_get_active();
    cfg.primary_name = "FFat";
    cfg.fs_mux       = s_fs_mux;
    cfg.dir_path     = "/screenshots";
    return fic_screenshot_save_bmp(&cfg, path, sizeof(path));
}

static bool exercise_atomic_json(void)
{
    JsonDocument doc;
    doc["version"] = FIC_CORE_VERSION_STR;
    String payload;
    serializeJson(doc, payload);
    if (!fic_json_write_atomic(FFat, "/smoke.json", "/smoke.json.bak", "/smoke.json.tmp", payload, fic_core_log_printf)) {
        return false;
    }
    JsonDocument loaded;
    return fic_json_load_with_recovery(FFat, "/smoke.json", "/smoke.json.bak", "/smoke.json.tmp", loaded, fic_core_log_printf);
}

void setup()
{
    Serial.begin(115200);
    s_fs_mux = xSemaphoreCreateMutex();
    fic_fs_set_mutex(s_fs_mux);

    FicStorageHandle_t ffat_h = { &FFat, "FFat", "/ffat", s_fs_mux, false, false };
    fic_fs_register_fallback(&ffat_h);

    FicCoreLogCfg_t log_cfg = {};
    log_cfg.ring_bytes        = 32768U;
    log_cfg.flush_interval_ms = 500U;
    log_cfg.dir_path          = "/serial";
    log_cfg.file_prefix       = "serial-log";
    log_cfg.timestamp_enabled = true;
    log_cfg.capture_esp_log   = true;
    fic_core_log_init(&log_cfg);

    s_board = fic_board_waveshare_s3_35b();
    const FicBoardProfile_t alt_board = fic_board_waveshare_s3_349_v3();
    fic_core_log_printf("[SMOKE] Board: %s (Alt: %s) | Core v%s\n",
                        s_board.name, alt_board.name, FIC_CORE_VERSION_STR);

    (void)fic_i2c_init_from_board(&s_board);

    FicLvglDisplayCfg_t disp_cfg = {};
    fic_lvgl_cfg_from_board(&s_board, &disp_cfg);
    disp_cfg.create_ui_cb  = smoke_create_ui;
    disp_cfg.log_printf_cb = fic_core_log_printf;
    fic_lvgl_port_init(&disp_cfg);

    FicAudioBeeperCfg_t audio_cfg = {};
    fic_audio_beeper_default_cfg(&audio_cfg, "S3_LCD_3_5B", fic_lvgl_get_io_expander(), (1UL << 7));
    audio_cfg.log_cb = fic_core_log_printf;
    if (fic_audio_beeper_init(&audio_cfg)) {
        (void)fic_audio_beeper_start_task();
        fic_audio_beeper_play(1500U, 40U);
    }

    fic_axp2101_init();
    FicAxp2101Status_t pmic = {};
    fic_axp2101_read(&pmic);

    fic_qmi8658_init();
    FicQmi8658Data_t imu = {};
    fic_qmi8658_read_tilt(&imu);

    (void)fic_rtc_init();
    FicRtcTime_t now = fic_rtc_get_time();
    fic_core_log_printf("[SMOKE] RTC %04u-%02u-%02u %02u:%02u:%02u\n",
                        now.year, now.month, now.day, now.hour, now.minute, now.second);

    (void)fic_mmc5983ma_init_default();
    FicMmc5983maData_t mag = {};
    (void)fic_mmc5983ma_read(imu.pitch_deg, imu.roll_deg, &mag);

    (void)fic_sen0696_init_on_bus1(&s_sen0696, SEN0696_I2C_ADDR_DEFAULT);

    FicRingLoggerCfg_t ring_cfg = {};
    ring_cfg.tag               = "SMOKE LOG";
    ring_cfg.fs                = &FFat;
    ring_cfg.fs_mux            = s_fs_mux;
    ring_cfg.dir_path          = "/logs";
    ring_cfg.file_prefix       = "smoke";
    ring_cfg.file_ext          = ".txt";
    ring_cfg.ring_bytes        = 4096U;
    ring_cfg.flush_interval_ms = 500U;
    ring_cfg.log_cb            = fic_core_log_printf;
    (void)fic_ring_logger_init(&s_ring_log, &ring_cfg);

    FicWebPortalCfg_t web_cfg = {};
    web_cfg.portal_title      = "Core Smoke Portal";
    web_cfg.version_str       = FIC_CORE_VERSION_STR;
    web_cfg.ap_ssid           = "FIC-Smoke";
    web_cfg.ap_password       = "12345678";
    web_cfg.fallback_fs       = &FFat;
    web_cfg.fallback_label    = "Internal Flash (FFat)";
    web_cfg.fallback_vfs_path = "/ffat";
    web_cfg.fs_mux            = s_fs_mux;
    web_cfg.log_cb            = fic_core_log_printf;
    fic_web_portal_configure(&web_cfg);

    Ahrs9DofConfig_t ahrs_cfg = {};
    ahrs_9dof_default_config(&ahrs_cfg, AHRS_ORIENT_DOWN_0);
    ahrs_9dof_init(&s_ahrs, &ahrs_cfg);

    double m[9][10] = {{2.0, 1.0}, {1.0, 3.0}};
    double b[9]     = {1.0, 2.0};
    double x[9]     = {0.0};
    (void)matrix_solve_nxn(2, m, b, x);
}

void loop()
{
    if (Serial.available() > 0) {
        int ch = Serial.read();
        if (ch == 's') {
            (void)save_smoke_screenshot();
        } else if (ch == 'j') {
            (void)exercise_atomic_json();
        } else if (ch == 'w') {
            fic_web_portal_start();
        } else if (ch == 'q') {
            fic_web_portal_stop();
        }
    }

    FicQmi8658Data_t imu = {};
    fic_qmi8658_read_tilt(&imu);

    FicSen0696State_t beam = {};
    if (fic_sen0696_is_online(&s_sen0696)) {
        (void)fic_sen0696_poll_into(&s_sen0696, &beam);
    }

    Ahrs9DofOutput_t ahrs_out = {};
    ahrs_9dof_update(&s_ahrs, imu.acc_g, imu.gyro_dps, beam.mag_ut, beam.mag_valid, true, 0.05f, &ahrs_out);

    fic_ring_logger_feed(&s_ring_log, reinterpret_cast<const uint8_t *>("tick\n"), 5U);
    fic_ring_logger_flush(&s_ring_log);
    fic_core_log_flush();
    fic_web_portal_handle();
    delay(50);
}
