#include "fic_screenshot.h"
#include "fic_lvgl_port.h"
#include "../rtc/fic_pcf85063.h"

#include <errno.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

void scr_log(fic_log_cb_t cb, const char *fmt, ...)
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

File open_screenshot_file(fs::FS &fs, const char *dir_path, const char *ts,
                          char *out_path, size_t out_path_sz,
                          bool *out_mkdir_failed)
{
    *out_mkdir_failed = false;
    const char *dir = (dir_path && dir_path[0] == '/') ? dir_path : "/screenshots";
    if (!fs.exists(dir)) {
        if (!fs.mkdir(dir)) {
            *out_mkdir_failed = true;
        }
    }

    snprintf(out_path, out_path_sz, "%s/screenshot_%s.bmp", dir, ts);
    File file = fs.open(out_path, FILE_WRITE);
    if (!file) {
        // Fallback to flat root filename if target directory creation is unsupported
        snprintf(out_path, out_path_sz, "/screenshot_%s.bmp", ts);
        file = fs.open(out_path, FILE_WRITE);
    }
    return file;
}

} // namespace

extern "C" bool fic_screenshot_save_bmp(const FicScreenshotCfg_t *cfg, char *out_path, size_t out_path_sz)
{
    if (!cfg || !cfg->primary_fs) {
        return false;
    }

    const uint32_t t0_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    const fic_log_cb_t log_cb = cfg->log_cb;

    uint16_t res_w = cfg->width;
    uint16_t res_h = cfg->height;
    if (res_w == 0U || res_h == 0U) {
        fic_lvgl_get_resolution(&res_w, &res_h);
    }
    if (res_w == 0U || res_h == 0U) {
        scr_log(log_cb, "[SCREENSHOT ERR] Zero resolution\n");
        return false;
    }

    const uint32_t w = res_w;
    const uint32_t h = res_h;
    const uint32_t fb_bytes = w * h * 2U;
    const uint32_t row_size = ((w * 3U + 3U) & ~3U);
    const uint32_t image_size = row_size * h;
    const uint32_t file_size = 54U + image_size;

    uint8_t *fb_copy = (uint8_t *)heap_caps_malloc((size_t)fb_bytes,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fb_copy) {
        scr_log(log_cb, "[SCREENSHOT ERR] PSRAM fb alloc failed (%lu B, PSRAM free=%u B)\n",
                (unsigned long)fb_bytes,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return false;
    }

    if (!fic_lvgl_capture_framebuffer(fb_copy, (size_t)fb_bytes)) {
        scr_log(log_cb, "[SCREENSHOT ERR] Framebuffer capture timed out waiting for LVGL render cycle\n");
        heap_caps_free(fb_copy);
        return false;
    }

    // Use small internal DMA-capable bounce buffer (0 B static DRAM0) so SDMMC/FFat VFS
    // writes never DMA directly from Octal PSRAM.
    uint32_t chunk_rows = 2U;
    uint8_t *chunk_buf = (uint8_t *)heap_caps_malloc((size_t)row_size * chunk_rows,
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!chunk_buf) {
        chunk_rows = 1U;
        chunk_buf = (uint8_t *)heap_caps_malloc((size_t)row_size,
                                                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!chunk_buf) {
        scr_log(log_cb, "[SCREENSHOT ERR] Internal DMA chunk alloc failed (%lu B, DMA free=%u B)\n",
                (unsigned long)row_size,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));
        heap_caps_free(fb_copy);
        return false;
    }

    char path_buf[96];
    char *path_dst = (out_path && out_path_sz > 0U) ? out_path : path_buf;
    size_t path_cap = (out_path && out_path_sz > 0U) ? out_path_sz : sizeof(path_buf);

    char ts[32];
    ts[0] = '\0';
    if (cfg->timestamp_cb != nullptr) {
        cfg->timestamp_cb(ts, sizeof(ts));
    } else if (fic_rtc_is_online()) {
        fic_rtc_get_formatted_timestamp(ts, sizeof(ts), true);
    }
    if (ts[0] == '\0') {
        snprintf(ts, sizeof(ts), "uptime_%lu", (unsigned long)t0_ms);
    }

    if (cfg->fs_mux && xSemaphoreTake(cfg->fs_mux, pdMS_TO_TICKS(5000)) != pdTRUE) {
        scr_log(log_cb, "[SCREENSHOT ERR] Timed out waiting for fs_mux\n");
        heap_caps_free(chunk_buf);
        heap_caps_free(fb_copy);
        return false;
    }

    fs::FS *primary_fs = cfg->primary_fs;
    const char *primary_name = cfg->primary_name ? cfg->primary_name : "PRIMARY_FS";
    fs::FS *target_fs = primary_fs;
    const char *target_name = primary_name;

    bool primary_mkdir_failed = false;
    bool fallback_mkdir_failed = false;
    bool used_fallback = false;
    int saved_errno = 0;

    errno = 0;
    File file = open_screenshot_file(*primary_fs, cfg->dir_path, ts,
                                     path_dst, path_cap, &primary_mkdir_failed);
    if (!file && cfg->fallback_fs != nullptr && cfg->fallback_fs != primary_fs) {
        saved_errno = errno;
        used_fallback = true;
        target_fs = cfg->fallback_fs;
        target_name = cfg->fallback_name ? cfg->fallback_name : "FALLBACK_FS";
        errno = 0;
        file = open_screenshot_file(*target_fs, cfg->dir_path, ts,
                                    path_dst, path_cap, &fallback_mkdir_failed);
        if (!file) {
            saved_errno = errno;
        }
    }

    if (!file) {
        uint64_t pri_free_kb = (cfg->free_kb_cb) ? cfg->free_kb_cb(primary_fs) : 0;
        uint64_t fb_free_kb  = (cfg->free_kb_cb && cfg->fallback_fs) ? cfg->free_kb_cb(cfg->fallback_fs) : 0;

        if (cfg->fs_mux) xSemaphoreGive(cfg->fs_mux);
        if (primary_mkdir_failed) {
            scr_log(log_cb, "[SCREENSHOT WARN] mkdir %s failed on %s\n",
                    cfg->dir_path ? cfg->dir_path : "/screenshots", primary_name);
        }
        if (used_fallback) {
            scr_log(log_cb, "[SCREENSHOT WARN] Open failed on %s -- falling back to %s\n", primary_name, target_name);
            if (fallback_mkdir_failed) {
                scr_log(log_cb, "[SCREENSHOT WARN] mkdir %s failed on %s\n",
                        cfg->dir_path ? cfg->dir_path : "/screenshots", target_name);
            }
        }
        scr_log(log_cb, "[SCREENSHOT ERR] Open failed on %s (errno=%d, pri_free=%lluKB, fb_free=%lluKB)\n",
                target_name, saved_errno,
                (unsigned long long)pri_free_kb, (unsigned long long)fb_free_kb);
        heap_caps_free(chunk_buf);
        heap_caps_free(fb_copy);
        return false;
    }

    const int32_t neg_h = -(int32_t)h;
    const uint32_t neg_h_u = (uint32_t)neg_h;

    uint8_t header[54] = {
        'B', 'M',
        (uint8_t)(file_size), (uint8_t)(file_size >> 8), (uint8_t)(file_size >> 16), (uint8_t)(file_size >> 24),
        0, 0, 0, 0,
        54, 0, 0, 0,
        40, 0, 0, 0,
        (uint8_t)(w), (uint8_t)(w >> 8), (uint8_t)(w >> 16), (uint8_t)(w >> 24),
        (uint8_t)(neg_h_u), (uint8_t)(neg_h_u >> 8), (uint8_t)(neg_h_u >> 16), (uint8_t)(neg_h_u >> 24),
        1, 0,
        24, 0,
        0, 0, 0, 0,
        (uint8_t)(image_size), (uint8_t)(image_size >> 8), (uint8_t)(image_size >> 16), (uint8_t)(image_size >> 24),
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
    };

    uint32_t bytes_written = 0;
    errno = 0;
    bool write_ok = (file.write(header, 54) == 54);
    if (write_ok) {
        bytes_written += 54;
    } else {
        saved_errno = errno;
    }

    const uint8_t *p565 = fb_copy;
    for (uint32_t y_base = 0; write_ok && y_base < h; y_base += chunk_rows) {
        uint32_t rows_in_chunk = h - y_base;
        if (rows_in_chunk > chunk_rows) {
            rows_in_chunk = chunk_rows;
        }

        uint32_t pos = 0;
        for (uint32_t r_idx = 0; r_idx < rows_in_chunk; r_idx++) {
            uint32_t row_start = pos;
            for (uint32_t x = 0; x < w; x++) {
                // fic_lvgl_flush_cb() byte-swaps RGB565 in place before DMA (BE bytes)
                uint16_t color = ((uint16_t)p565[0] << 8) | (uint16_t)p565[1];
                p565 += 2;

                uint8_t r = (uint8_t)((color >> 11) & 0x1F);
                uint8_t g = (uint8_t)((color >> 5) & 0x3F);
                uint8_t b = (uint8_t)(color & 0x1F);

                chunk_buf[pos++] = (uint8_t)((b * 255U + 15U) / 31U);
                chunk_buf[pos++] = (uint8_t)((g * 255U + 31U) / 63U);
                chunk_buf[pos++] = (uint8_t)((r * 255U + 15U) / 31U);
            }
            while ((pos - row_start) < row_size) {
                chunk_buf[pos++] = 0;
            }
        }

        errno = 0;
        size_t written = file.write(chunk_buf, pos);
        bytes_written += (uint32_t)written;
        if (written != pos) {
            saved_errno = errno;
            write_ok = false;
        }
    }

    if (write_ok) {
        file.flush();
    }
    file.close();

    size_t verified_size = 0;
    if (write_ok) {
        errno = 0;
        File verify = target_fs->open(path_dst, FILE_READ);
        verified_size = verify ? verify.size() : 0;
        if (verify) {
            verify.close();
        }
        if (verified_size != (size_t)file_size) {
            saved_errno = errno;
            write_ok = false;
            target_fs->remove(path_dst);
        }
    } else {
        target_fs->remove(path_dst);
    }

    uint64_t free_kb = (!write_ok && cfg->free_kb_cb) ? cfg->free_kb_cb(target_fs) : 0;

    if (cfg->fs_mux) xSemaphoreGive(cfg->fs_mux);

    heap_caps_free(chunk_buf);
    heap_caps_free(fb_copy);

    if (primary_mkdir_failed) {
        scr_log(log_cb, "[SCREENSHOT WARN] mkdir %s failed on %s\n",
                cfg->dir_path ? cfg->dir_path : "/screenshots", primary_name);
    }
    if (used_fallback) {
        scr_log(log_cb, "[SCREENSHOT WARN] Open failed on %s -- falling back to %s\n", primary_name, target_name);
        if (fallback_mkdir_failed) {
            scr_log(log_cb, "[SCREENSHOT WARN] mkdir %s failed on %s\n",
                    cfg->dir_path ? cfg->dir_path : "/screenshots", target_name);
        }
    }

    const uint32_t elapsed_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - t0_ms;
    if (!write_ok) {
        scr_log(log_cb, "[SCREENSHOT ERR] Write/verify failed on %s (%s) wrote=%lu/%lu verified=%lu errno=%d free=%lluKB\n",
                target_name, path_dst,
                (unsigned long)bytes_written,
                (unsigned long)file_size,
                (unsigned long)verified_size,
                saved_errno,
                (unsigned long long)free_kb);
        return false;
    }

    scr_log(log_cb, "[SCREENSHOT] Saved %s on %s (%lu bytes verified, %lu ms)\n",
            path_dst, target_name,
            (unsigned long)verified_size,
            (unsigned long)elapsed_ms);
    return true;
}
