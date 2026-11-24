#include "fic_ring_logger.h"
#include "fic_fs_manager.h"
#include "../rtc/fic_pcf85063.h"

#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

void rlog_printf(fic_log_cb_t cb, const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (cb != nullptr) {
        cb("%s", buf);
    } else {
        printf("%s", buf);
    }
}

uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

bool ensure_backing_ring(FicRingLogger_t *logger)
{
    if (logger->backing_buf != nullptr) {
        return true;
    }
    const size_t cap = (logger->cfg.ring_bytes >= 512U) ? logger->cfg.ring_bytes : 8192U;
    uint8_t *buf = static_cast<uint8_t *>(heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buf == nullptr) {
        buf = static_cast<uint8_t *>(malloc(cap));
    }
    if (buf == nullptr) {
        return false;
    }
    logger->backing_buf = buf;
    fic_ring_buf_init(&logger->ring, buf, cap);
    return true;
}

void drain_to_file_locked(FicRingLogger_t *logger, bool force_sync)
{
    if (!logger->file || logger->backing_buf == nullptr) {
        return;
    }

    uint8_t write_buf[1024];
    bool wrote_any = false;

    for (;;) {
        size_t n = 0U;
        if (logger->buf_mux && xSemaphoreTake(logger->buf_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            n = fic_ring_buf_pop(&logger->ring, write_buf, sizeof(write_buf));
            xSemaphoreGive(logger->buf_mux);
        } else {
            break;
        }
        if (n == 0U) {
            break;
        }
        logger->file.write(write_buf, n);
        logger->bytes_logged += (uint32_t)n;
        wrote_any = true;
    }

    const uint32_t t = now_ms();
    const uint32_t interval = logger->cfg.flush_interval_ms ? logger->cfg.flush_interval_ms : 500U;
    if (force_sync || (wrote_any && (t - logger->last_flush_ms >= interval))) {
        logger->last_flush_ms = t;
        logger->file.flush();
    }
}

} // namespace

extern "C" bool fic_ring_logger_init(FicRingLogger_t *logger, const FicRingLoggerCfg_t *cfg)
{
    if (!logger || !cfg) {
        return false;
    }
    memset(logger, 0, sizeof(*logger));
    logger->cfg = *cfg;
    if (logger->cfg.ring_bytes < 512U) {
        logger->cfg.ring_bytes = 8192U;
    }
    if (logger->cfg.flush_interval_ms == 0U) {
        logger->cfg.flush_interval_ms = 500U;
    }
    logger->buf_mux  = xSemaphoreCreateMutex();
    logger->file_mux = xSemaphoreCreateMutex();
    return (logger->buf_mux != NULL) && (logger->file_mux != NULL) && ensure_backing_ring(logger);
}

extern "C" bool fic_ring_logger_start(FicRingLogger_t *logger)
{
    if (!logger) {
        return false;
    }
    if (!logger->buf_mux)  logger->buf_mux  = xSemaphoreCreateMutex();
    if (!logger->file_mux) logger->file_mux = xSemaphoreCreateMutex();

    if (logger->file_mux) xSemaphoreTake(logger->file_mux, portMAX_DELAY);
    if (!ensure_backing_ring(logger)) {
        if (logger->file_mux) xSemaphoreGive(logger->file_mux);
        return false;
    }
    if (logger->active) {
        if (logger->file_mux) xSemaphoreGive(logger->file_mux);
        return true;
    }

    fs::FS *fs = logger->cfg.fs ? logger->cfg.fs : fic_fs_get_active();
    SemaphoreHandle_t fs_mux = logger->cfg.fs_mux ? logger->cfg.fs_mux : fic_fs_get_mutex();
    const char *tag = logger->cfg.tag ? logger->cfg.tag : "RING LOG";
    if (!fs) {
        if (logger->file_mux) xSemaphoreGive(logger->file_mux);
        rlog_printf(logger->cfg.log_cb, "[%s] No filesystem mounted\n", tag);
        return false;
    }

    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

    if (logger->file) {
        logger->file.close();
    }

    const char *dir = (logger->cfg.dir_path && logger->cfg.dir_path[0] == '/') ? logger->cfg.dir_path : nullptr;
    if (dir && !fs->exists(dir)) {
        fs->mkdir(dir);
    }

    char ts[48] = {0};
    if (logger->cfg.timestamp_cb) {
        logger->cfg.timestamp_cb(ts, sizeof(ts));
    } else if (fic_rtc_is_online()) {
        fic_rtc_get_formatted_timestamp(ts, sizeof(ts), true);
    }
    if (ts[0] == '\0') {
        snprintf(ts, sizeof(ts), "uptime_%lu", (unsigned long)now_ms());
    }

    const char *prefix = logger->cfg.file_prefix ? logger->cfg.file_prefix : "log";
    const char *ext    = logger->cfg.file_ext    ? logger->cfg.file_ext    : ".txt";
    if (dir) {
        snprintf(logger->filename, sizeof(logger->filename), "%s/%s_%s%s", dir, prefix, ts, ext);
        logger->file = fs->open(logger->filename, FILE_WRITE);
    }
    if (!logger->file) {
        snprintf(logger->filename, sizeof(logger->filename), "/%s_%s%s", prefix, ts, ext);
        logger->file = fs->open(logger->filename, FILE_WRITE);
    }

    if (!logger->file) {
        logger->active = false;
        if (fs_mux) xSemaphoreGive(fs_mux);
        if (logger->file_mux) xSemaphoreGive(logger->file_mux);
        rlog_printf(logger->cfg.log_cb, "[%s] Failed to open %s for write\n", tag, logger->filename);
        return false;
    }

    if (logger->cfg.header_line && logger->cfg.header_line[0] != '\0') {
        logger->file.print(logger->cfg.header_line);
        logger->file.flush();
    }

    if (logger->buf_mux) xSemaphoreTake(logger->buf_mux, portMAX_DELAY);
    fic_ring_buf_reset(&logger->ring);
    logger->bytes_logged  = 0U;
    logger->last_flush_ms = now_ms();
    if (logger->buf_mux) xSemaphoreGive(logger->buf_mux);

    logger->active = true;

    if (fs_mux) xSemaphoreGive(fs_mux);
    if (logger->file_mux) xSemaphoreGive(logger->file_mux);
    rlog_printf(logger->cfg.log_cb, "[%s] Started recording to %s\n", tag, logger->filename);
    return true;
}

extern "C" void fic_ring_logger_stop(FicRingLogger_t *logger)
{
    if (!logger || !logger->file_mux) {
        return;
    }
    xSemaphoreTake(logger->file_mux, portMAX_DELAY);
    bool stopped = false;
    uint32_t total = 0U;
    SemaphoreHandle_t fs_mux = logger->cfg.fs_mux ? logger->cfg.fs_mux : fic_fs_get_mutex();
    if (logger->active) {
        if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);
        drain_to_file_locked(logger, true);
        if (logger->file) {
            logger->file.flush();
            logger->file.close();
        }
        logger->active = false;
        total = logger->bytes_logged;
        stopped = true;
        if (fs_mux) xSemaphoreGive(fs_mux);
    }
    xSemaphoreGive(logger->file_mux);
    if (stopped) {
        const char *tag = logger->cfg.tag ? logger->cfg.tag : "RING LOG";
        rlog_printf(logger->cfg.log_cb, "[%s] Stopped. Total: %lu B (%lu drops) -> %s\n",
                    tag, (unsigned long)total, (unsigned long)logger->ring.drop_count, logger->filename);
    }
}

extern "C" void fic_ring_logger_feed(FicRingLogger_t *logger, const uint8_t *data, size_t len)
{
    if (!logger || !logger->active || !logger->backing_buf || !data || len == 0U) {
        return;
    }
    if (!logger->buf_mux) {
        logger->buf_mux = xSemaphoreCreateMutex();
    }
    if (logger->buf_mux && xSemaphoreTake(logger->buf_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        (void)fic_ring_buf_push(&logger->ring, data, len, logger->cfg.drop_oldest);
        xSemaphoreGive(logger->buf_mux);
    }
}

extern "C" void fic_ring_logger_flush(FicRingLogger_t *logger)
{
    if (!logger || !logger->active || !logger->backing_buf) {
        return;
    }
    const bool has_data = !fic_ring_buf_is_empty(&logger->ring);
    const uint32_t interval = logger->cfg.flush_interval_ms ? logger->cfg.flush_interval_ms : 500U;
    const bool needs_sync = (now_ms() - logger->last_flush_ms >= interval);
    if (!has_data && !needs_sync) {
        return;
    }

    if (!logger->file_mux || xSemaphoreTake(logger->file_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    SemaphoreHandle_t fs_mux = logger->cfg.fs_mux ? logger->cfg.fs_mux : fic_fs_get_mutex();
    if (fs_mux && xSemaphoreTake(fs_mux, pdMS_TO_TICKS(15)) != pdTRUE) {
        xSemaphoreGive(logger->file_mux);
        return;
    }
    if (logger->active) {
        drain_to_file_locked(logger, false);
    }
    if (fs_mux) xSemaphoreGive(fs_mux);
    xSemaphoreGive(logger->file_mux);
}

extern "C" bool fic_ring_logger_is_active(const FicRingLogger_t *logger)
{
    return logger && logger->active;
}

extern "C" const char *fic_ring_logger_get_filename(const FicRingLogger_t *logger)
{
    return logger ? logger->filename : "";
}

extern "C" uint32_t fic_ring_logger_get_bytes_logged(const FicRingLogger_t *logger)
{
    return logger ? logger->bytes_logged : 0U;
}

extern "C" uint32_t fic_ring_logger_get_drops(const FicRingLogger_t *logger)
{
    return logger ? logger->ring.drop_count : 0U;
}
