#include "fic_web_portal.h"
#include "fic_web_portal_html.h"
#include "../rtc/fic_pcf85063.h"
#include "../storage/fic_fs_manager.h"
#include "../storage/fic_settings_json.h"

#include <WebServer.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static FicWebPortalCfg_t s_cfg                   = {0};
static volatile bool     s_wifi_active           = false;
static volatile bool     s_wifi_starting         = false;
static volatile bool     s_wifi_abort_requested  = false;
static volatile bool     s_wifi_stop_pending     = false;
static volatile bool     s_ble_resume_after_wifi = false;
static volatile bool     s_ble_resume_pending    = false;
static char              s_wifi_ip[32]           = {0};
static WebServer        *s_server                = nullptr;

static fic_web_ble_is_active_cb_t s_ble_is_active_cb = nullptr;
static fic_web_ble_suspend_cb_t   s_ble_suspend_cb   = nullptr;
static fic_web_ble_resume_cb_t    s_ble_resume_cb    = nullptr;

static void web_log(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (s_cfg.log_cb != nullptr) {
        s_cfg.log_cb("%s", buf);
    } else {
        printf("%s", buf);
    }
}

static void notify_status(const char *status_text)
{
    if (s_cfg.on_status_cb != nullptr) {
        s_cfg.on_status_cb(status_text);
    }
}

static SemaphoreHandle_t active_fs_mux(void)
{
    return (s_cfg.fs_mux != NULL) ? s_cfg.fs_mux : fic_fs_get_mutex();
}

static fs::FS *get_target_fs(const String &fs_param)
{
    if (fs_param == "sd" && s_cfg.primary_fs != nullptr) {
        return s_cfg.primary_fs;
    }
    if (s_cfg.fallback_fs != nullptr) {
        return s_cfg.fallback_fs;
    }
    return s_cfg.primary_fs ? s_cfg.primary_fs : fic_fs_get_active();
}

static void handleRoot(void)
{
    if (!s_server) return;
    s_server->send(200, "text/html; charset=utf-8", fic_web_render_portal_page(&s_cfg));
}

static void handleSetTime(void)
{
    if (!s_server) return;
    if (s_server->hasArg("y") && s_server->hasArg("m") && s_server->hasArg("d") &&
        s_server->hasArg("h") && s_server->hasArg("min") && s_server->hasArg("s")) {
        uint16_t y   = (uint16_t)s_server->arg("y").toInt();
        uint8_t  m   = (uint8_t)s_server->arg("m").toInt();
        uint8_t  d   = (uint8_t)s_server->arg("d").toInt();
        uint8_t  h   = (uint8_t)s_server->arg("h").toInt();
        uint8_t  min = (uint8_t)s_server->arg("min").toInt();
        uint8_t  s   = (uint8_t)s_server->arg("s").toInt();
        if (s_cfg.on_set_time_cb != nullptr) {
            s_cfg.on_set_time_cb(y, m, d, h, min, s);
        } else {
            fic_rtc_set_time(y, m, d, h, min, s);
        }
    }
    s_server->send(200, "text/plain", "OK");
}

static void handleDownload(void)
{
    if (!s_server) return;
    if (!s_server->hasArg("file")) {
        s_server->send(400, "text/plain", "Missing file parameter");
        return;
    }
    String path    = s_server->arg("file");
    String fs_type = s_server->hasArg("fs") ? s_server->arg("fs") : "ffat";
    fs::FS *target_fs = get_target_fs(fs_type);
    if (!target_fs) {
        s_server->send(503, "text/plain", "Storage not mounted");
        return;
    }

    if (!path.startsWith("/")) path = "/" + path;
    File file = target_fs->open(path, "r");
    if (!file) {
        s_server->send(404, "text/plain", "File not found: " + path);
        return;
    }

    int slashIndex = path.lastIndexOf('/');
    String filename = (slashIndex >= 0) ? path.substring(slashIndex + 1) : path;

    String mimeType = "application/octet-stream";
    if (filename.endsWith(".csv")) {
        mimeType = "text/csv; charset=utf-8";
    } else if (filename.endsWith(".bmp")) {
        mimeType = "image/bmp";
    } else if (filename.endsWith(".json")) {
        mimeType = "application/json; charset=utf-8";
    } else if (filename.endsWith(".log") || filename.endsWith(".txt")) {
        mimeType = "text/plain; charset=utf-8";
    }

    if (!s_server->hasArg("raw")) {
        s_server->sendHeader("Content-Disposition", "inline; filename=\"" + filename + "\"");
    }
    s_server->sendHeader("X-Content-Type-Options", "nosniff");
    s_server->sendHeader("Content-Length", String(file.size()));
    s_server->sendHeader("Connection", "close");
    s_server->streamFile(file, mimeType);
    file.close();
}

static void handleView(void)
{
    if (!s_server) return;
    if (!s_server->hasArg("file")) {
        s_server->send(400, "text/plain", "Missing file parameter");
        return;
    }
    String path    = s_server->arg("file");
    String fs_type = s_server->hasArg("fs") ? s_server->arg("fs") : "ffat";
    int slashIndex = path.lastIndexOf('/');
    String filename = (slashIndex >= 0) ? path.substring(slashIndex + 1) : path;
    s_server->send(200, "text/html; charset=utf-8", fic_web_render_view_image_page(path, filename, fs_type));
}

static void handleDelete(void)
{
    if (!s_server) return;
    if (!s_server->hasArg("file")) {
        s_server->send(400, "text/plain", "Missing file parameter");
        return;
    }
    String path    = s_server->arg("file");
    String fs_type = s_server->hasArg("fs") ? s_server->arg("fs") : "ffat";
    fs::FS *target_fs = get_target_fs(fs_type);
    if (target_fs) {
        if (!path.startsWith("/")) path = "/" + path;
        SemaphoreHandle_t mux = active_fs_mux();
        if (!mux || xSemaphoreTake(mux, pdMS_TO_TICKS(500)) == pdTRUE) {
            target_fs->remove(path);
            if (mux) xSemaphoreGive(mux);
        }
    }
    s_server->sendHeader("Location", "/");
    s_server->send(303);
}

static File s_uploadFile;
static File s_uploadMirrorFile;

static void handleUpload(void)
{
    if (!s_server) return;
    HTTPUpload &upload = s_server->upload();
    String fs_type = s_server->hasArg("fs") ? s_server->arg("fs") : "ffat";
    fs::FS *target_fs = get_target_fs(fs_type);
    if (!target_fs) {
        return;
    }

    if (upload.status == UPLOAD_FILE_START) {
        String filename = upload.filename;
        if (!filename.startsWith("/")) filename = "/" + filename;
        s_uploadFile = target_fs->open(filename, "w");
        if (s_cfg.mirror_filename && s_cfg.mirror_filename[0] != '\0' &&
            upload.filename.endsWith(s_cfg.mirror_filename) &&
            s_cfg.primary_fs != nullptr && s_cfg.fallback_fs != nullptr) {
            fs::FS *other_fs = (target_fs == s_cfg.fallback_fs) ? s_cfg.primary_fs : s_cfg.fallback_fs;
            if (other_fs) {
                s_uploadMirrorFile = other_fs->open(filename, "w");
            }
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (s_uploadFile)       s_uploadFile.write(upload.buf, upload.currentSize);
        if (s_uploadMirrorFile) s_uploadMirrorFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (s_uploadFile)       s_uploadFile.close();
        if (s_uploadMirrorFile) s_uploadMirrorFile.close();
        if (s_cfg.on_file_uploaded_cb != nullptr) {
            s_cfg.on_file_uploaded_cb(upload.filename.c_str(), fs_type.c_str());
        }
    }
}

static void setup_web_routes(void)
{
    if (!s_server) {
        const uint16_t port = s_cfg.http_port ? s_cfg.http_port : 80U;
        s_server = new WebServer(port);
    }
    s_server->on("/", handleRoot);
    s_server->on("/set_time", handleSetTime);
    s_server->on("/download", handleDownload);
    s_server->on("/view", handleView);
    s_server->on("/delete", handleDelete);
    s_server->on("/upload", HTTP_POST, []() {
        if (s_server) {
            s_server->sendHeader("Location", "/");
            s_server->send(303);
        }
    }, handleUpload);
}

extern "C" void fic_web_portal_register_ble_hooks(fic_web_ble_is_active_cb_t is_active_cb,
                                               fic_web_ble_suspend_cb_t suspend_cb,
                                               fic_web_ble_resume_cb_t resume_cb)
{
    s_ble_is_active_cb = is_active_cb;
    s_ble_suspend_cb   = suspend_cb;
    s_ble_resume_cb    = resume_cb;
}

extern "C" void fic_web_portal_configure(const FicWebPortalCfg_t *cfg)
{
    if (cfg != nullptr) {
        s_cfg = *cfg;
    } else {
        memset(&s_cfg, 0, sizeof(s_cfg));
    }
}

extern "C" bool fic_web_portal_is_active(void)
{
    return s_wifi_active;
}

extern "C" bool fic_web_portal_is_softap(void)
{
    return s_wifi_active && ((WiFi.getMode() & WIFI_AP) != 0);
}

extern "C" bool fic_web_portal_is_starting(void)
{
    return s_wifi_starting;
}

extern "C" const char *fic_web_portal_get_ip(void)
{
    return s_wifi_ip;
}

extern "C" void fic_web_portal_start(void)
{
    if (s_wifi_active || s_wifi_starting) {
        return;
    }
    s_wifi_abort_requested = false;
    s_wifi_starting        = true;
    notify_status("Connecting...");

    // Non-blocking connection task pinned to Core 1 (UI & Event Core) at Priority 3
    xTaskCreatePinnedToCore([](void *arg) {
        (void)arg;
        int stop_wait = 0;
        while (s_wifi_stop_pending && stop_wait < 30) {
            vTaskDelay(pdMS_TO_TICKS(15));
            stop_wait++;
        }
        if (s_wifi_abort_requested) {
            s_wifi_starting = false;
            notify_status("Wi-Fi Off");
            web_log("[WIFI] Connection cancelled before init.\n");
            vTaskDelete(NULL);
            return;
        }

        // Core-enforced BLE <-> Wi-Fi mutual exclusion: tear down NimBLE cleanly to reclaim DRAM0.
        if (s_ble_is_active_cb != nullptr && s_ble_is_active_cb() && s_ble_suspend_cb != nullptr) {
            s_ble_resume_after_wifi = true;
            notify_status("Stopping BLE...");
            web_log("[WIFI] Suspending active BLE companion before Wi-Fi init...\n");
            s_ble_suspend_cb();
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (s_wifi_abort_requested) {
            s_wifi_starting = false;
            if (s_ble_resume_after_wifi) {
                s_ble_resume_after_wifi = false;
                s_ble_resume_pending    = true;
            }
            notify_status("Wi-Fi Off");
            web_log("[WIFI] Connection cancelled after BLE suspend.\n");
            vTaskDelete(NULL);
            return;
        }

        web_log("[WIFI] Pre-Init Heap -> Internal Free: %u B (Largest: %u B, DMA: %u B) | PSRAM Free: %u KB\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024U));

        WiFi.persistent(false);

        static wifi_event_id_t s_wifi_event_id = 0;
        if (s_wifi_event_id == 0) {
            s_wifi_event_id = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
                if (event == ARDUINO_EVENT_WIFI_STA_START) {
                    web_log("[WIFI EVENT] STA Interface Started\n");
                } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
                    web_log("[WIFI EVENT] STA Associated with AP\n");
                } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
                    uint8_t reason = info.wifi_sta_disconnected.reason;
                    web_log("[WIFI EVENT] STA Disconnected (Reason: %u - %s)\n",
                            reason, WiFi.disconnectReasonName((wifi_err_reason_t)reason));
                } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
                    web_log("[WIFI EVENT] STA Got IP: %s\n",
                            IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
                } else if (event == ARDUINO_EVENT_WIFI_AP_START) {
                    web_log("[WIFI EVENT] SoftAP Started\n");
                }
            });
        }

        char ssid[64] = {0};
        char pass[64] = {0};
        bool has_creds = (s_cfg.get_wifi_creds_cb != nullptr) &&
                         s_cfg.get_wifi_creds_cb(ssid, sizeof(ssid), pass, sizeof(pass));
        bool has_valid_ssid = has_creds && !fic_json_is_placeholder_ssid(ssid);

        bool sta_connected = false;
        if (has_valid_ssid) {
            web_log("[WIFI] Connecting to '%s' (password len=%d)...\n", ssid, (int)strlen(pass));
            if ((WiFi.getMode() & WIFI_MODE_AP) != 0) {
                WiFi.softAPdisconnect(true);
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if ((WiFi.getMode() & WIFI_MODE_STA) != 0) {
                WiFi.disconnect(false, false);
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            bool mode_ok = WiFi.mode(WIFI_STA);
            web_log("[WIFI] WiFi.mode(WIFI_STA) -> %s | Internal Free: %u B (Largest: %u B)\n",
                    mode_ok ? "OK" : "FAIL",
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

            WiFi.setSleep(false);
            WiFi.setAutoReconnect(true);
            WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
            WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
            WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);

            notify_status("Connecting...");
            wl_status_t begin_res = WiFi.begin(ssid, pass);
            web_log("[WIFI] WiFi.begin('%s') returned status: %d\n", ssid, (int)begin_res);

            int polls = 0;
            const int max_polls = 200;
            wl_status_t last_status = (wl_status_t)999;

            while (!s_wifi_abort_requested &&
                   (WiFi.status() != WL_CONNECTED && WiFi.localIP() == IPAddress(0, 0, 0, 0)) &&
                   polls < max_polls) {
                vTaskDelay(pdMS_TO_TICKS(100));
                polls++;
                wl_status_t cur_status = WiFi.status();
                if (cur_status != last_status || (polls % 20 == 0)) {
                    last_status = cur_status;
                    const char *st_name = (cur_status == WL_IDLE_STATUS)     ? "IDLE" :
                                          (cur_status == WL_NO_SSID_AVAIL)   ? "NO_SSID_AVAIL" :
                                          (cur_status == WL_SCAN_COMPLETED)  ? "SCAN_COMPLETED" :
                                          (cur_status == WL_CONNECTED)       ? "CONNECTED" :
                                          (cur_status == WL_CONNECT_FAILED)  ? "CONNECT_FAILED" :
                                          (cur_status == WL_CONNECTION_LOST) ? "CONNECTION_LOST" :
                                          (cur_status == WL_DISCONNECTED)    ? "DISCONNECTED" :
                                          ((int)cur_status == 254)           ? "INITIALIZING" : "UNKNOWN";
                    web_log("[WIFI] Status: %s (%d) | %.1fs / 20.0s\n", st_name, (int)cur_status, polls * 0.1f);
                    char prog[32];
                    snprintf(prog, sizeof(prog), "Connecting (%ds)...", (int)(polls / 10));
                    notify_status(prog);
                }
            }
            sta_connected = (!s_wifi_abort_requested &&
                             (WiFi.status() == WL_CONNECTED || WiFi.localIP() != IPAddress(0, 0, 0, 0)));
        } else {
            web_log("[WIFI] No valid STA credentials configured. Launching SoftAP hotspot...\n");
        }

        if (s_wifi_abort_requested) {
            if ((WiFi.getMode() & WIFI_MODE_AP) != 0) {
                WiFi.softAPdisconnect(true);
            }
            WiFi.disconnect(false, false);
            WiFi.mode(WIFI_OFF);
            s_wifi_active   = false;
            s_wifi_starting = false;
            s_wifi_ip[0]    = '\0';
            if (s_ble_resume_after_wifi) {
                s_ble_resume_after_wifi = false;
                s_ble_resume_pending    = true;
            }
            notify_status("Wi-Fi Off");
            web_log("[WIFI] Connection cancelled by user.\n");
            vTaskDelete(NULL);
            return;
        }

        if (sta_connected) {
            String ip = WiFi.localIP().toString();
            strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
            s_wifi_ip[sizeof(s_wifi_ip) - 1] = '\0';

            setup_web_routes();
            s_server->begin();
            s_wifi_active   = true;
            s_wifi_starting = false;
            notify_status(s_wifi_ip);
            web_log("[WIFI] Connected to '%s'! Web Portal: http://%s/\n", ssid, s_wifi_ip);

            if (!s_wifi_abort_requested && !s_wifi_stop_pending) {
                const char *ntp = (s_cfg.ntp_server && s_cfg.ntp_server[0] != '\0') ? s_cfg.ntp_server : "pool.ntp.org";
                configTime(0, 0, ntp);
                struct tm timeinfo;
                if (getLocalTime(&timeinfo, 2500)) {
                    uint16_t yr = (uint16_t)(timeinfo.tm_year + 1900);
                    uint8_t  mo = (uint8_t)(timeinfo.tm_mon + 1);
                    uint8_t  dy = (uint8_t)timeinfo.tm_mday;
                    uint8_t  hr = (uint8_t)timeinfo.tm_hour;
                    uint8_t  mn = (uint8_t)timeinfo.tm_min;
                    uint8_t  sc = (uint8_t)timeinfo.tm_sec;
                    if (s_cfg.on_set_time_cb != nullptr) {
                        s_cfg.on_set_time_cb(yr, mo, dy, hr, mn, sc);
                    } else {
                        fic_rtc_set_time(yr, mo, dy, hr, mn, sc);
                    }
                    web_log("[WIFI] RTC synced via NTP!\n");
                }
            }
        } else {
            const char *ap_ssid = (s_cfg.ap_ssid && s_cfg.ap_ssid[0] != '\0') ? s_cfg.ap_ssid : "Field-Instrument";
            const char *ap_pass = s_cfg.ap_password ? s_cfg.ap_password : "12345678";
            if (has_valid_ssid) {
                web_log("[WIFI] STA connection to '%s' failed (Status: %d). Starting SoftAP '%s'...\n",
                        ssid, (int)WiFi.status(), ap_ssid);
            }
            notify_status("Starting AP...");
            if ((WiFi.getMode() & WIFI_MODE_STA) != 0) {
                WiFi.disconnect(false, false);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            bool ap_mode_ok = WiFi.mode(WIFI_AP);
            bool ap_ok      = WiFi.softAP(ap_ssid, ap_pass);
            web_log("[WIFI] SoftAP init -> mode:%s softAP:%s\n", ap_mode_ok ? "OK" : "FAIL", ap_ok ? "OK" : "FAIL");

            int ap_wait = 0;
            while (!s_wifi_abort_requested && WiFi.softAPIP() == IPAddress(0, 0, 0, 0) && ap_wait < 10) {
                vTaskDelay(pdMS_TO_TICKS(100));
                ap_wait++;
            }

            if (s_wifi_abort_requested) {
                WiFi.softAPdisconnect(true);
                WiFi.mode(WIFI_OFF);
                s_wifi_active   = false;
                s_wifi_starting = false;
                s_wifi_ip[0]    = '\0';
                if (s_ble_resume_after_wifi) {
                    s_ble_resume_after_wifi = false;
                    s_ble_resume_pending    = true;
                }
                notify_status("Wi-Fi Off");
                web_log("[WIFI] SoftAP startup cancelled by user.\n");
                vTaskDelete(NULL);
                return;
            }

            IPAddress apIP = WiFi.softAPIP();
            if (apIP == IPAddress(0, 0, 0, 0)) {
                apIP = IPAddress(192, 168, 4, 1);
            }
            String ip = apIP.toString();
            strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
            s_wifi_ip[sizeof(s_wifi_ip) - 1] = '\0';
            setup_web_routes();
            s_server->begin();
            s_wifi_active   = true;
            s_wifi_starting = false;
            notify_status(s_wifi_ip);
            web_log("[WIFI] SoftAP Active! Connect to SSID '%s' -> http://%s/\n", ap_ssid, s_wifi_ip);
        }
        s_wifi_starting = false;
        vTaskDelete(NULL);
    }, "wifi_connect", 8192, NULL, 3, NULL, 1);
}

extern "C" void fic_web_portal_stop(void)
{
    s_wifi_abort_requested = true;
    bool was_active   = s_wifi_active;
    bool was_starting = s_wifi_starting;
    s_wifi_starting   = false;

    if (was_active) {
        s_wifi_active = false;
        s_wifi_ip[0]  = '\0';
        notify_status("Stopping Wi-Fi...");
        // Defer server->stop() and WiFi.mode(WIFI_OFF) to loop() (fic_web_portal_handle)
        s_wifi_stop_pending = true;
    } else if (was_starting) {
        notify_status("Stopping Wi-Fi...");
    }
}

extern "C" void fic_web_portal_handle(void)
{
    if (s_wifi_stop_pending) {
        s_wifi_stop_pending = false;
        if (s_server) {
            s_server->stop();
        }
        if ((WiFi.getMode() & WIFI_MODE_AP) != 0) {
            WiFi.softAPdisconnect(true);
        }
        WiFi.disconnect(false, false);
        WiFi.mode(WIFI_OFF);
        s_wifi_active = false;
        s_wifi_ip[0]  = '\0';
        notify_status("Wi-Fi Off");
        web_log("[WIFI] Web Portal Stopped.\n");
        if (s_ble_resume_after_wifi) {
            s_ble_resume_after_wifi = false;
            s_ble_resume_pending    = true;
        }
        return;
    }
    if (s_ble_resume_pending && !s_wifi_active && !s_wifi_starting && !s_wifi_stop_pending) {
        s_ble_resume_pending = false;
        if (s_ble_resume_cb != nullptr) {
            web_log("[WIFI] Restoring BLE companion after Wi-Fi stop...\n");
            s_ble_resume_cb();
        }
    }
    if (s_wifi_active && s_server) {
        s_server->handleClient();
    }
}
