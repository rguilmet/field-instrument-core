#ifndef FIC_SCREENSHOT_H
#define FIC_SCREENSHOT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "../boards/fic_board_types.h"

typedef void (*fic_screenshot_timestamp_cb_t)(char *buf, size_t max_len);
typedef uint64_t (*fic_screenshot_free_kb_cb_t)(fs::FS *fs);

typedef struct {
    fs::FS                        *primary_fs;
    const char                    *primary_name;
    fs::FS                        *fallback_fs;
    const char                    *fallback_name;
    SemaphoreHandle_t              fs_mux;
    const char                    *dir_path;
    uint16_t                       width;
    uint16_t                       height;
    fic_screenshot_timestamp_cb_t  timestamp_cb;
    fic_screenshot_free_kb_cb_t    free_kb_cb;
    fic_log_cb_t                   log_cb;
} FicScreenshotCfg_t;

extern "C" {
bool fic_screenshot_save_bmp(const FicScreenshotCfg_t *cfg, char *out_path, size_t out_path_sz);
}
#endif // __cplusplus

#endif // FIC_SCREENSHOT_H
