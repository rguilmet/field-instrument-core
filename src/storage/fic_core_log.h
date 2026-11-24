#ifndef FIC_CORE_LOG_H
#define FIC_CORE_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#include <Stream.h>

class FicLogStream : public Stream {
public:
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int available() override;
    int read() override;
    int peek() override;
    void flush() override;
};

void fic_core_log_attach_stream(Stream *console_stream);

extern "C" {
#endif

typedef void (*fic_core_log_ts_cb_t)(char *buf, size_t max_len);

typedef struct {
    size_t               ring_bytes;
    uint32_t             flush_interval_ms;
    const char          *dir_path;
    const char          *file_prefix;
    bool                 timestamp_enabled;
    bool                 capture_esp_log;
    fic_core_log_ts_cb_t timestamp_cb;
} FicCoreLogCfg_t;

void fic_core_log_init(const FicCoreLogCfg_t *cfg);
size_t fic_core_log_write(const uint8_t *buf, size_t len);
int fic_core_log_printf(const char *format, ...);
bool fic_core_log_start(void);
void fic_core_log_stop(void);
bool fic_core_log_is_active(void);
void fic_core_log_flush(void);
const char *fic_core_log_get_filename(void);
uint32_t fic_core_log_get_bytes_logged(void);
void fic_core_log_set_timestamp_enabled(bool enabled);
bool fic_core_log_is_timestamp_enabled(void);

#ifdef __cplusplus
}
#endif

#endif // FIC_CORE_LOG_H
