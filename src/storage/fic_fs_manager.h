#ifndef FIC_FS_MANAGER_H
#define FIC_FS_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifdef __cplusplus
#include <FS.h>

typedef struct {
    fs::FS           *fs;
    const char       *name;
    const char       *vfs_mount_path;
    SemaphoreHandle_t fs_mux;
    bool              is_sd;
    bool              is_sd_spi;
} FicStorageHandle_t;

extern "C" {
void fic_fs_register_primary(const FicStorageHandle_t *handle);
void fic_fs_register_fallback(const FicStorageHandle_t *handle);
bool fic_fs_has_primary(void);
fs::FS *fic_fs_get_active(void);
const FicStorageHandle_t *fic_fs_get_primary(void);
const FicStorageHandle_t *fic_fs_get_fallback(void);
}
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

void fic_fs_set_mutex(SemaphoreHandle_t mux);
SemaphoreHandle_t fic_fs_get_mutex(void);
bool fic_fs_lock(uint32_t timeout_ms);
void fic_fs_unlock(void);

#ifdef __cplusplus
}
#endif

#endif // FIC_FS_MANAGER_H
