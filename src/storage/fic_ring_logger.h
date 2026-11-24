#ifndef FIC_RING_LOGGER_H
#define FIC_RING_LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "../boards/fic_board_types.h"
#include "fic_ring_buf.h"

typedef void (*fic_ring_timestamp_cb_t)(char *buf, size_t max_len);

typedef struct {
    const char             *tag;
    fs::FS                 *fs;
    SemaphoreHandle_t       fs_mux;
    const char             *dir_path;
    const char             *file_prefix;
    const char             *file_ext;
    const char             *header_line;
    size_t                  ring_bytes;
    uint32_t                flush_interval_ms;
    bool                    drop_oldest;
    fic_ring_timestamp_cb_t timestamp_cb;
    fic_log_cb_t            log_cb;
} FicRingLoggerCfg_t;

typedef struct {
    FicRingLoggerCfg_t cfg;
    FicByteRing_t      ring;
    uint8_t           *backing_buf;
    File               file;
    SemaphoreHandle_t  buf_mux;
    SemaphoreHandle_t  file_mux;
    volatile bool      active;
    uint32_t           bytes_logged;
    uint32_t           last_flush_ms;
    char               filename[80];
} FicRingLogger_t;

extern "C" {
bool fic_ring_logger_init(FicRingLogger_t *logger, const FicRingLoggerCfg_t *cfg);
bool fic_ring_logger_start(FicRingLogger_t *logger);
void fic_ring_logger_stop(FicRingLogger_t *logger);
void fic_ring_logger_feed(FicRingLogger_t *logger, const uint8_t *data, size_t len);
void fic_ring_logger_flush(FicRingLogger_t *logger);
bool fic_ring_logger_is_active(const FicRingLogger_t *logger);
const char *fic_ring_logger_get_filename(const FicRingLogger_t *logger);
uint32_t fic_ring_logger_get_bytes_logged(const FicRingLogger_t *logger);
uint32_t fic_ring_logger_get_drops(const FicRingLogger_t *logger);
}
#endif // __cplusplus

#endif // FIC_RING_LOGGER_H
