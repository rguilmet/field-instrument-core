#include "fic_fs_manager.h"
#include <string.h>

static FicStorageHandle_t s_primary     = {0};
static FicStorageHandle_t s_fallback    = {0};
static bool               s_has_primary = false;
static bool               s_has_fallback = false;
static SemaphoreHandle_t  s_fs_mux      = NULL;

extern "C" void fic_fs_set_mutex(SemaphoreHandle_t mux)
{
    s_fs_mux = mux;
}

extern "C" SemaphoreHandle_t fic_fs_get_mutex(void)
{
    return s_fs_mux;
}

extern "C" bool fic_fs_lock(uint32_t timeout_ms)
{
    if (s_fs_mux == NULL) {
        return true;
    }
    const TickType_t ticks = (timeout_ms == 0U) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_fs_mux, ticks) == pdTRUE;
}

extern "C" void fic_fs_unlock(void)
{
    if (s_fs_mux != NULL) {
        xSemaphoreGive(s_fs_mux);
    }
}

extern "C" void fic_fs_register_primary(const FicStorageHandle_t *handle)
{
    if (handle == nullptr || handle->fs == nullptr) {
        memset(&s_primary, 0, sizeof(s_primary));
        s_has_primary = false;
        return;
    }
    s_primary     = *handle;
    s_has_primary = true;
    if (s_fs_mux == NULL && handle->fs_mux != NULL) {
        s_fs_mux = handle->fs_mux;
    }
}

extern "C" void fic_fs_register_fallback(const FicStorageHandle_t *handle)
{
    if (handle == nullptr || handle->fs == nullptr) {
        memset(&s_fallback, 0, sizeof(s_fallback));
        s_has_fallback = false;
        return;
    }
    s_fallback     = *handle;
    s_has_fallback = true;
    if (s_fs_mux == NULL && handle->fs_mux != NULL) {
        s_fs_mux = handle->fs_mux;
    }
}

extern "C" bool fic_fs_has_primary(void)
{
    return s_has_primary && (s_primary.fs != nullptr);
}

extern "C" fs::FS *fic_fs_get_active(void)
{
    if (s_has_primary && s_primary.fs != nullptr) {
        return s_primary.fs;
    }
    if (s_has_fallback && s_fallback.fs != nullptr) {
        return s_fallback.fs;
    }
    return nullptr;
}

extern "C" const FicStorageHandle_t *fic_fs_get_primary(void)
{
    return s_has_primary ? &s_primary : nullptr;
}

extern "C" const FicStorageHandle_t *fic_fs_get_fallback(void)
{
    return s_has_fallback ? &s_fallback : nullptr;
}
