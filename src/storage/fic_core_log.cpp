#include "fic_core_log.h"
#include "fic_fs_manager.h"
#include "fic_ring_buf.h"
#include "../rtc/fic_pcf85063.h"

#include <Arduino.h>
#include <FS.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

static size_t               s_ring_cap         = 32768U;
static uint32_t             s_flush_ms         = 500U;
static char                 s_dir_path[32]     = "/serial";
static char                 s_file_prefix[32]  = "serial-log";
static fic_core_log_ts_cb_t s_ts_cb            = NULL;

static Stream              *s_hw_stream        = &::Serial;
static volatile bool        s_log_active       = false;
static uint32_t             s_bytes_logged     = 0U;
static char                 s_log_filename[80] = {0};
static File                 s_log_file;
static SemaphoreHandle_t    s_log_mux          = NULL;
static SemaphoreHandle_t    s_file_mux         = NULL;

static FicByteRing_t        s_ring             = {0};
static uint8_t             *s_ring_buf         = NULL;
static volatile TaskHandle_t s_io_owner_task   = NULL;
static uint32_t             s_last_sd_flush_ms = 0U;
static bool                 s_timestamp_enabled = true;
static bool                 s_at_start_of_line  = true;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static bool ensure_ring_buf(void)
{
    if (s_ring_buf != NULL) {
        return true;
    }
    const size_t cap = (s_ring_cap >= 1024U) ? s_ring_cap : 32768U;
    s_ring_buf = (uint8_t *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_ring_buf == NULL) {
        s_ring_buf = (uint8_t *)malloc(cap);
    }
    if (s_ring_buf == NULL) {
        return false;
    }
    fic_ring_buf_init(&s_ring, s_ring_buf, cap);
    return true;
}

static int core_log_esp_vprintf(const char *fmt, va_list args)
{
    if (xPortInIsrContext()) {
        return 0;
    }
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        size_t write_len = ((size_t)len < sizeof(buf)) ? (size_t)len : (sizeof(buf) - 1U);
        (void)fic_core_log_write((const uint8_t *)buf, write_len);
    }
    return len;
}

static size_t format_record_timestamp(char *buf, size_t max_len)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    // If real-world epoch is valid (year >= 2025, epoch >= 1735689600)
    if (tv.tv_sec >= 1735689600) {
        int sec_in_day = (int)(tv.tv_sec % 86400);
        if (sec_in_day < 0) sec_in_day += 86400;
        int hh   = sec_in_day / 3600;
        int mm   = (sec_in_day % 3600) / 60;
        int ss   = sec_in_day % 60;
        int msec = (int)(tv.tv_usec / 1000);
        return (size_t)snprintf(buf, max_len, "[%02d:%02d:%02d.%03d] ", hh, mm, ss, msec);
    }

    uint32_t now = now_ms();
    uint32_t ms  = now % 1000U;
    uint32_t sec = (now / 1000U) % 60U;
    uint32_t min = (now / 60000U) % 60U;
    uint32_t hr  = (now / 3600000U);
    return (size_t)snprintf(buf, max_len, "[%02lu:%02lu:%02lu.%03lu] ",
                            (unsigned long)hr, (unsigned long)min, (unsigned long)sec, (unsigned long)ms);
}

static void hw_write(const uint8_t *data, size_t len)
{
    if (s_hw_stream != NULL && data != NULL && len > 0U) {
        s_hw_stream->write(data, len);
    }
}

void fic_core_log_attach_stream(Stream *console_stream)
{
    s_hw_stream = console_stream;
}

extern "C" void fic_core_log_init(const FicCoreLogCfg_t *cfg)
{
    if (s_log_mux == NULL) {
        s_log_mux = xSemaphoreCreateRecursiveMutex();
    }
    if (s_file_mux == NULL) {
        s_file_mux = xSemaphoreCreateMutex();
    }
    if (cfg != NULL) {
        if (cfg->ring_bytes >= 1024U) s_ring_cap = cfg->ring_bytes;
        if (cfg->flush_interval_ms > 0U) s_flush_ms = cfg->flush_interval_ms;
        if (cfg->dir_path && cfg->dir_path[0] == '/') {
            strncpy(s_dir_path, cfg->dir_path, sizeof(s_dir_path) - 1U);
            s_dir_path[sizeof(s_dir_path) - 1U] = '\0';
        }
        if (cfg->file_prefix && cfg->file_prefix[0] != '\0') {
            strncpy(s_file_prefix, cfg->file_prefix, sizeof(s_file_prefix) - 1U);
            s_file_prefix[sizeof(s_file_prefix) - 1U] = '\0';
        }
        s_timestamp_enabled = cfg->timestamp_enabled;
        s_ts_cb             = cfg->timestamp_cb;
    }
    (void)ensure_ring_buf();
    if (cfg == NULL || cfg->capture_esp_log) {
        esp_log_set_vprintf(core_log_esp_vprintf);
    }
}

extern "C" size_t fic_core_log_write(const uint8_t *buffer, size_t size)
{
    if (!buffer || size == 0U) {
        return 0U;
    }

    // Only skip ring-buffer capture if the exact task currently inside SD/FFat VFS I/O
    // triggers a re-entrant print; all other tasks always capture to s_ring.
    TaskHandle_t owner = s_io_owner_task;
    if (owner != NULL && owner == xTaskGetCurrentTaskHandle()) {
        hw_write(buffer, size);
        return size;
    }

    if (s_log_mux == NULL) {
        s_log_mux = xSemaphoreCreateRecursiveMutex();
    }

    if (s_log_mux && xSemaphoreTakeRecursive(s_log_mux, pdMS_TO_TICKS(50)) == pdTRUE) {
        (void)ensure_ring_buf();
        if (!s_timestamp_enabled) {
            hw_write(buffer, size);
            (void)fic_ring_buf_push(&s_ring, buffer, size, true);
            xSemaphoreGiveRecursive(s_log_mux);
            return size;
        }

        size_t idx = 0U;
        while (idx < size) {
            if (s_at_start_of_line) {
                uint8_t first_char = buffer[idx];
                if (first_char != '\r' && first_char != '\n') {
                    char ts_buf[32];
                    size_t ts_len = format_record_timestamp(ts_buf, sizeof(ts_buf));
                    hw_write((const uint8_t *)ts_buf, ts_len);
                    (void)fic_ring_buf_push(&s_ring, (const uint8_t *)ts_buf, ts_len, true);
                    s_at_start_of_line = false;
                }
            }

            size_t seg_start = idx;
            while (idx < size && buffer[idx] != '\n') {
                idx++;
            }
            if (idx < size && buffer[idx] == '\n') {
                idx++;
                s_at_start_of_line = true;
            }
            size_t seg_len = idx - seg_start;
            if (seg_len > 0U) {
                hw_write(buffer + seg_start, seg_len);
                (void)fic_ring_buf_push(&s_ring, buffer + seg_start, seg_len, true);
            }
        }
        xSemaphoreGiveRecursive(s_log_mux);
    } else {
        hw_write(buffer, size);
    }
    return size;
}

extern "C" int fic_core_log_printf(const char *format, ...)
{
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
        size_t write_len = ((size_t)len < sizeof(buf)) ? (size_t)len : (sizeof(buf) - 1U);
        (void)fic_core_log_write((const uint8_t *)buf, write_len);
    }
    return len;
}

static void drain_to_file_locked(bool force_sync)
{
    if (!s_log_file || s_ring_buf == NULL) {
        return;
    }

    uint8_t write_buf[1024];
    bool wrote_any = false;

    for (;;) {
        size_t bytes_to_write = 0U;
        if (s_log_mux && xSemaphoreTakeRecursive(s_log_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            bytes_to_write = fic_ring_buf_pop(&s_ring, write_buf, sizeof(write_buf));
            xSemaphoreGiveRecursive(s_log_mux);
        } else {
            break;
        }
        if (bytes_to_write == 0U) {
            break;
        }
        s_io_owner_task = xTaskGetCurrentTaskHandle();
        s_log_file.write(write_buf, bytes_to_write);
        s_io_owner_task = NULL;
        s_bytes_logged += (uint32_t)bytes_to_write;
        wrote_any = true;
    }

    uint32_t t = now_ms();
    if (force_sync || (wrote_any && (t - s_last_sd_flush_ms >= s_flush_ms))) {
        s_last_sd_flush_ms = t;
        s_io_owner_task = xTaskGetCurrentTaskHandle();
        s_log_file.flush();
        s_io_owner_task = NULL;
    }
}

extern "C" bool fic_core_log_start(void)
{
    if (s_log_mux == NULL)  s_log_mux  = xSemaphoreCreateRecursiveMutex();
    if (s_file_mux == NULL) s_file_mux = xSemaphoreCreateMutex();
    if (s_file_mux) xSemaphoreTake(s_file_mux, portMAX_DELAY);

    if (s_log_active) {
        if (s_file_mux) xSemaphoreGive(s_file_mux);
        return true;
    }

    fs::FS *fs = fic_fs_get_active();
    if (fs == NULL) {
        if (s_file_mux) xSemaphoreGive(s_file_mux);
        return false;
    }

    (void)fic_fs_lock(0U);
    s_io_owner_task = xTaskGetCurrentTaskHandle();
    if (s_log_file) {
        s_log_file.close();
    }
    if (!fs->exists(s_dir_path)) {
        fs->mkdir(s_dir_path);
    }

    char ts[48] = {0};
    if (s_ts_cb != NULL) {
        s_ts_cb(ts, sizeof(ts));
    } else if (fic_rtc_is_online()) {
        fic_rtc_get_formatted_timestamp(ts, sizeof(ts), true);
    }
    if (ts[0] == '\0') {
        snprintf(ts, sizeof(ts), "uptime_%lu", (unsigned long)now_ms());
    }

    snprintf(s_log_filename, sizeof(s_log_filename), "%s/%s_%s.txt", s_dir_path, s_file_prefix, ts);
    s_log_file = fs->open(s_log_filename, FILE_WRITE);
    if (!s_log_file) {
        snprintf(s_log_filename, sizeof(s_log_filename), "/%s_%s.txt", s_file_prefix, ts);
        s_log_file = fs->open(s_log_filename, FILE_WRITE);
    }
    s_io_owner_task = NULL;

    if (!s_log_file) {
        s_log_active = false;
        fic_fs_unlock();
        if (s_file_mux) xSemaphoreGive(s_file_mux);
        return false;
    }

    s_bytes_logged     = 0U;
    s_last_sd_flush_ms = now_ms();
    s_log_active       = true;
    drain_to_file_locked(true);

    fic_fs_unlock();
    if (s_file_mux) xSemaphoreGive(s_file_mux);
    return true;
}

extern "C" void fic_core_log_stop(void)
{
    if (s_file_mux == NULL) {
        return;
    }
    xSemaphoreTake(s_file_mux, portMAX_DELAY);
    if (s_log_active) {
        (void)fic_fs_lock(0U);
        drain_to_file_locked(true);
        s_io_owner_task = xTaskGetCurrentTaskHandle();
        if (s_log_file) {
            s_log_file.flush();
            s_log_file.close();
        }
        s_io_owner_task = NULL;
        s_log_active = false;
        fic_fs_unlock();
    }
    xSemaphoreGive(s_file_mux);
}

extern "C" bool fic_core_log_is_active(void)
{
    return s_log_active;
}

extern "C" void fic_core_log_flush(void)
{
    if (!s_log_active || s_ring_buf == NULL) {
        return;
    }
    const bool has_data   = !fic_ring_buf_is_empty(&s_ring);
    const bool needs_sync = (now_ms() - s_last_sd_flush_ms >= s_flush_ms);
    if (!has_data && !needs_sync) {
        return;
    }
    if (s_file_mux == NULL || xSemaphoreTake(s_file_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    if (!fic_fs_lock(15U)) {
        xSemaphoreGive(s_file_mux);
        return;
    }
    if (s_log_active) {
        drain_to_file_locked(false);
    }
    fic_fs_unlock();
    xSemaphoreGive(s_file_mux);
}

extern "C" const char *fic_core_log_get_filename(void)
{
    return s_log_filename;
}

extern "C" uint32_t fic_core_log_get_bytes_logged(void)
{
    return s_bytes_logged;
}

extern "C" void fic_core_log_set_timestamp_enabled(bool enabled)
{
    s_timestamp_enabled = enabled;
}

extern "C" bool fic_core_log_is_timestamp_enabled(void)
{
    return s_timestamp_enabled;
}

size_t FicLogStream::write(uint8_t c)
{
    return fic_core_log_write(&c, 1U);
}

size_t FicLogStream::write(const uint8_t *buffer, size_t size)
{
    return fic_core_log_write(buffer, size);
}

int FicLogStream::available()
{
    return (s_hw_stream != NULL) ? s_hw_stream->available() : 0;
}

int FicLogStream::read()
{
    return (s_hw_stream != NULL) ? s_hw_stream->read() : -1;
}

int FicLogStream::peek()
{
    return (s_hw_stream != NULL) ? s_hw_stream->peek() : -1;
}

void FicLogStream::flush()
{
    if (s_hw_stream != NULL) {
        s_hw_stream->flush();
    }
    if (s_log_active) {
        fic_core_log_flush();
    }
}
