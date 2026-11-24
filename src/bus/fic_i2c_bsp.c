#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "fic_i2c_internal.h"
#include "freertos/task.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_sig_map.h"

i2c_master_bus_handle_t fic_i2c0_bus     = NULL;
i2c_master_bus_handle_t fic_i2c1_bus     = NULL;
i2c_master_dev_handle_t fic_touch_dev    = NULL;
i2c_master_dev_handle_t fic_rtc_dev      = NULL;
i2c_master_dev_handle_t fic_imu_dev      = NULL;
i2c_master_dev_handle_t fic_pmic_dev     = NULL;
i2c_master_dev_handle_t fic_mag_dev      = NULL;
i2c_master_dev_handle_t fic_beam_imu_dev = NULL;

FicI2cBusCfg_t g_fic_i2c0_cfg      = {0};
FicI2cAuxCfg_t g_fic_i2c1_cfg      = {0};
bool           g_fic_i2c0_configured = false;
bool           g_fic_i2c1_configured = false;
uint8_t        g_fic_beam_imu_addr = 0;
uint8_t        g_fic_beam_imu_bus  = 0;
uint32_t       g_fic_i2c_timeout_ticks = 40;

static fic_log_cb_t s_log_cb = NULL;

fic_i2c_port_state_t g_fic_i2c_ports[2] = {
    {
        .port_num = I2C_NUM_0,
        .scl_pin = GPIO_NUM_NC,
        .sda_pin = GPIO_NUM_NC,
        .scl_out_idx = I2CEXT0_SCL_OUT_IDX,
        .scl_in_idx  = I2CEXT0_SCL_IN_IDX,
        .sda_out_idx = I2CEXT0_SDA_OUT_IDX,
        .sda_in_idx  = I2CEXT0_SDA_IN_IDX,
        .bus_ptr = &fic_i2c0_bus,
        .mutex = NULL,
        .consec_timeouts = 0,
        .last_gpio_clear_ms = 0,
        .last_reinit_ms = 0,
        .last_warn_ms = 0,
        .err_count = 0,
    },
    {
        .port_num = I2C_NUM_1,
        .scl_pin = GPIO_NUM_NC,
        .sda_pin = GPIO_NUM_NC,
        .scl_out_idx = I2CEXT1_SCL_OUT_IDX,
        .scl_in_idx  = I2CEXT1_SCL_IN_IDX,
        .sda_out_idx = I2CEXT1_SDA_OUT_IDX,
        .sda_in_idx  = I2CEXT1_SDA_IN_IDX,
        .bus_ptr = &fic_i2c1_bus,
        .mutex = NULL,
        .consec_timeouts = 0,
        .last_gpio_clear_ms = 0,
        .last_reinit_ms = 0,
        .last_warn_ms = 0,
        .err_count = 0,
    }
};

static fic_i2c_bus_hook_entry_t s_bus_hooks[2][FIC_I2C_MAX_BUS_HOOKS] = {0};

void fic_i2c_set_log_callback(fic_log_cb_t cb)
{
    s_log_cb = cb;
}

void fic_i2c_internal_log(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (s_log_cb != NULL) {
        s_log_cb("%s", buf);
    } else {
        printf("%s", buf);
    }
}

bool fic_i2c_lock_port(uint8_t port, uint32_t timeout_ms)
{
    if (port > 1U) {
        return false;
    }
    SemaphoreHandle_t mtx = g_fic_i2c_ports[port].mutex;
    if (mtx == NULL) {
        return true;
    }
    const TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(mtx, ticks) == pdTRUE;
}

void fic_i2c_unlock_port(uint8_t port)
{
    if (port > 1U) {
        return;
    }
    SemaphoreHandle_t mtx = g_fic_i2c_ports[port].mutex;
    if (mtx != NULL) {
        xSemaphoreGiveRecursive(mtx);
    }
}

bool fic_i2c_lock(uint32_t timeout_ms)
{
    return fic_i2c_lock_port(0U, timeout_ms);
}

void fic_i2c_unlock(void)
{
    fic_i2c_unlock_port(0U);
}

static const char *fic_i2c_dev_name(i2c_master_dev_handle_t h)
{
    if (h == fic_touch_dev)    return "Touch";
    if (h == fic_imu_dev)      return "IMU";
    if (h == fic_rtc_dev)      return "RTC";
    if (h == fic_pmic_dev)     return "PMIC";
    if (h == fic_mag_dev)      return "MAG";
    if (h == fic_beam_imu_dev) return "10DOF";
    return "I2C_DEV";
}

static bool is_corrupt_sentinel_byte(uint8_t b)
{
    return (b == 0x61U || b == 0x71U);
}

bool fic_i2c_has_sentinel_run(const uint8_t *buf, size_t len, size_t min_run)
{
    if (buf == NULL || min_run == 0 || len < min_run) {
        return false;
    }
    if (min_run == 1 && is_corrupt_sentinel_byte(buf[0])) {
        return true;
    }
    size_t run = 1;
    for (size_t i = 1; i < len; i++) {
        if (buf[i] == buf[i - 1] && is_corrupt_sentinel_byte(buf[i])) {
            run++;
            if (run >= min_run) {
                return true;
            }
        } else {
            run = 1;
        }
    }
    return false;
}

bool fic_i2c_has_sentinel_tail(const uint8_t *buf, size_t len, size_t tail_len)
{
    if (buf == NULL || tail_len == 0 || len < tail_len) {
        return false;
    }
    const uint8_t s = buf[len - tail_len];
    if (!is_corrupt_sentinel_byte(s)) {
        return false;
    }
    for (size_t i = len - tail_len + 1; i < len; i++) {
        if (buf[i] != s) {
            return false;
        }
    }
    return true;
}

bool fic_i2c_register_hook_port(uint8_t port, fic_i2c_bus_hook_t detach_fn, fic_i2c_bus_hook_t reattach_fn, void *ctx)
{
    if (port > 1U || (detach_fn == NULL && reattach_fn == NULL)) {
        return false;
    }
    if (!fic_i2c_lock_port(port, 200)) {
        return false;
    }
    int free_idx = -1;
    for (int i = 0; i < FIC_I2C_MAX_BUS_HOOKS; i++) {
        if (s_bus_hooks[port][i].ctx == ctx &&
            s_bus_hooks[port][i].detach_fn == detach_fn &&
            s_bus_hooks[port][i].reattach_fn == reattach_fn) {
            fic_i2c_unlock_port(port);
            return true;
        }
        if (free_idx < 0 && s_bus_hooks[port][i].detach_fn == NULL && s_bus_hooks[port][i].reattach_fn == NULL) {
            free_idx = i;
        }
    }
    if (free_idx < 0) {
        fic_i2c_unlock_port(port);
        return false;
    }
    s_bus_hooks[port][free_idx].detach_fn = detach_fn;
    s_bus_hooks[port][free_idx].reattach_fn = reattach_fn;
    s_bus_hooks[port][free_idx].ctx = ctx;
    fic_i2c_unlock_port(port);
    return true;
}

void fic_i2c_unregister_hook_port(uint8_t port, void *ctx)
{
    if (port > 1U || !fic_i2c_lock_port(port, 200)) {
        return;
    }
    for (int i = 0; i < FIC_I2C_MAX_BUS_HOOKS; i++) {
        if (s_bus_hooks[port][i].ctx == ctx) {
            s_bus_hooks[port][i].detach_fn = NULL;
            s_bus_hooks[port][i].reattach_fn = NULL;
            s_bus_hooks[port][i].ctx = NULL;
        }
    }
    fic_i2c_unlock_port(port);
}

void fic_i2c_invoke_hooks_port(uint8_t port, bool is_reattach)
{
    if (port > 1U) {
        return;
    }
    i2c_master_bus_handle_t bus_handle = *g_fic_i2c_ports[port].bus_ptr;
    if (bus_handle == NULL) {
        return;
    }
    for (int i = 0; i < FIC_I2C_MAX_BUS_HOOKS; i++) {
        fic_i2c_bus_hook_t fn = is_reattach ? s_bus_hooks[port][i].reattach_fn : s_bus_hooks[port][i].detach_fn;
        if (fn != NULL) {
            fn(bus_handle, s_bus_hooks[port][i].ctx);
        }
    }
}

bool fic_i2c_register_bus_hooks(fic_i2c_bus_hook_t detach_fn, fic_i2c_bus_hook_t reattach_fn, void *ctx)
{
    return fic_i2c_register_hook_port(0U, detach_fn, reattach_fn, ctx);
}

void fic_i2c_unregister_bus_hooks(void *ctx)
{
    fic_i2c_unregister_hook_port(0U, ctx);
}

// NXP I2C Spec UM10204 Section 3.1.16 Bus Clear:
// Because an always-on LiPo UPS keeps I2C slave chips continuously powered across
// ESP32 warm resets, an interrupted read can leave a slave holding SDA LOW.
// Pre-drive HIGH before configuring open-drain to avoid synthetic START glitches, and
// skip clocking completely if SDA and SCL are already idle HIGH.
void fic_i2c_gpio_bus_clear_pins(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    if (scl_pin < 0 || sda_pin < 0) {
        return;
    }
    gpio_set_level(scl_pin, 1);
    gpio_set_level(sda_pin, 1);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << scl_pin) | (1ULL << sda_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(scl_pin, 1);
    gpio_set_level(sda_pin, 1);
    esp_rom_delay_us(50);

    // Do not clock an already-idle bus (avoids shifting slaves during benign stalls)
    if (gpio_get_level(sda_pin) == 1 && gpio_get_level(scl_pin) == 1) {
        return;
    }

    // Clock up to 3 x 9 SCL pulses (~50 kHz), stopping cleanly once SDA releases HIGH and STOP succeeds
    for (int chunk = 0; chunk < 3; chunk++) {
        if (gpio_get_level(sda_pin) == 1) {
            break;
        }
        for (int i = 0; i < 9; i++) {
            gpio_set_level(scl_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level(scl_pin, 1);
            for (int w = 0; w < 10 && gpio_get_level(scl_pin) == 0; w++) {
                esp_rom_delay_us(10);
            }
            esp_rom_delay_us(10);
        }
        if (gpio_get_level(sda_pin) == 1) {
            gpio_set_level(scl_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level(sda_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level(scl_pin, 1);
            esp_rom_delay_us(10);
            gpio_set_level(sda_pin, 1);
            esp_rom_delay_us(100);
            if (gpio_get_level(sda_pin) == 1) {
                break;
            }
        }
    }

    gpio_set_level(sda_pin, 1);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(50);
}

static bool fic_i2c_runtime_gpio_bus_clear_locked(uint8_t port, int *sda_gpio_after_out)
{
    fic_i2c_port_state_t *ps = &g_fic_i2c_ports[port];
    // Pre-drive HIGH output latch before detaching I2CEXT so switching to open-drain never glitches LOW
    gpio_set_level(ps->scl_pin, 1);
    gpio_set_level(ps->sda_pin, 1);
    esp_rom_gpio_connect_out_signal(ps->scl_pin, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(ps->sda_pin, SIG_GPIO_OUT_IDX, false, false);

    fic_i2c_gpio_bus_clear_pins(ps->scl_pin, ps->sda_pin);

    int sda_gpio_after = gpio_get_level(ps->sda_pin);
    int scl_gpio_after = gpio_get_level(ps->scl_pin);
    if (sda_gpio_after_out != NULL) {
        *sda_gpio_after_out = sda_gpio_after;
    }

    // Restore open-drain levels and reattach GPIO pins to hardware I2C peripheral
    gpio_set_level(ps->scl_pin, 1);
    gpio_set_level(ps->sda_pin, 1);
    esp_rom_gpio_connect_out_signal(ps->scl_pin, ps->scl_out_idx, false, false);
    esp_rom_gpio_connect_in_signal(ps->scl_pin, ps->scl_in_idx, false);
    esp_rom_gpio_connect_out_signal(ps->sda_pin, ps->sda_out_idx, false, false);
    esp_rom_gpio_connect_in_signal(ps->sda_pin, ps->sda_in_idx, false);
    return (sda_gpio_after == 1 && scl_gpio_after == 1);
}

void fic_i2c_recover_port_locked(uint8_t port, const char *action, i2c_master_dev_handle_t dev_handle, int reg, esp_err_t err)
{
    if (port > 1U) {
        return;
    }
    fic_i2c_port_state_t *ps = &g_fic_i2c_ports[port];
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (ps->err_count < UINT32_MAX) {
        ps->err_count++;
    }
    if (now - ps->last_warn_ms >= 1000) {
        ps->last_warn_ms = now;
        if (reg >= 0) {
            fic_i2c_internal_log("[I2C%u WARN] %s failed on %s (reg=0x%02X, err=0x%X) -> recovering bus\n",
                                 (unsigned)port, action, fic_i2c_dev_name(dev_handle), (uint8_t)reg, (unsigned int)err);
        } else {
            fic_i2c_internal_log("[I2C%u WARN] %s failed on %s (err=0x%X) -> recovering bus\n",
                                 (unsigned)port, action, fic_i2c_dev_name(dev_handle), (unsigned int)err);
        }
    }

    const bool is_bus_stall = (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE);
    if (is_bus_stall && ps->consec_timeouts < 255) {
        ps->consec_timeouts++;
    }

    i2c_master_bus_handle_t bus_handle = *ps->bus_ptr;
    if (bus_handle == NULL || ps->scl_pin < 0 || ps->sda_pin < 0) {
        return;
    }

    bool escalate_reinit = false;
    int sda_before = gpio_get_level(ps->sda_pin);
    int scl_before = gpio_get_level(ps->scl_pin);

    // Tier 1: Only run open-drain GPIO clocking if SDA or SCL is actually held LOW; never disturb idle SDA=1/SCL=1
    if ((ps->consec_timeouts >= 3 || sda_before == 0 || scl_before == 0) &&
        (now - ps->last_gpio_clear_ms >= 1500U)) {
        ps->last_gpio_clear_ms = now;
        int sda_gpio_after = sda_before;
        if (sda_before == 0 || scl_before == 0) {
            fic_i2c_internal_log("[I2C%u RECOVER] %u consecutive bus stalls -> running GPIO bus-clear (SDA=%d SCL=%d)\n",
                                 (unsigned)port, (unsigned int)ps->consec_timeouts, sda_before, scl_before);
            (void)fic_i2c_runtime_gpio_bus_clear_locked(port, &sda_gpio_after);
        }
        i2c_master_bus_reset(bus_handle);
        vTaskDelay(pdMS_TO_TICKS(5));
        int sda_after = gpio_get_level(ps->sda_pin);
        int scl_after = gpio_get_level(ps->scl_pin);
        if (sda_before == 0 || scl_before == 0) {
            fic_i2c_internal_log("[I2C%u RECOVER] Runtime bus-clear complete (GPIO SDA=%d | I2C SDA=%d SCL=%d)\n",
                                 (unsigned)port, sda_gpio_after, sda_after, scl_after);
        }
        if (sda_after == 0 || scl_after == 0 || ps->consec_timeouts >= 6) {
            escalate_reinit = true;
        }
    } else {
        i2c_master_bus_reset(bus_handle);
        vTaskDelay(pdMS_TO_TICKS(5));
        if (ps->consec_timeouts >= 6) {
            escalate_reinit = true;
        }
    }

    // Tier 2: Persistent stall (or peripheral matrix stuck LOW) -> full ESP-IDF I2C peripheral teardown & reinit
    if (escalate_reinit && (now - ps->last_reinit_ms >= 5000U)) {
        ps->last_reinit_ms = now;
        ps->consec_timeouts = 0;
        fic_i2c_internal_log("[I2C%u RECOVER] Persistent stall (SDA=%d SCL=%d) -> running full I2C%u peripheral reinit\n",
                             (unsigned)port, gpio_get_level(ps->sda_pin), gpio_get_level(ps->scl_pin), (unsigned)port);
        if (port == 0U) {
            fic_i2c_reinit_bus();
        } else {
            fic_i2c_reinit_bus1();
        }
        fic_i2c_internal_log("[I2C%u RECOVER] Full I2C%u reinit complete (SDA=%d SCL=%d)\n",
                             (unsigned)port, (unsigned)port, gpio_get_level(ps->sda_pin), gpio_get_level(ps->scl_pin));
    }
}

void fic_i2c_gpio_bus_clear(void)
{
    fic_i2c_gpio_bus_clear_pins(g_fic_i2c_ports[0].scl_pin, g_fic_i2c_ports[0].sda_pin);
}

void fic_i2c_recover_bus(void)
{
    if (!fic_i2c_lock(250)) {
        return;
    }
    if (fic_i2c0_bus != NULL) {
        i2c_master_bus_reset(fic_i2c0_bus);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    fic_i2c_unlock();
}

bool fic_i2c_add_device_on_port(uint8_t port, uint8_t dev_addr, uint32_t scl_speed_hz, uint32_t scl_wait_us, i2c_master_dev_handle_t *out_handle)
{
    if (port > 1U || out_handle == NULL || dev_addr < 0x08U || dev_addr > 0x77U) {
        return false;
    }
    i2c_master_bus_handle_t bus_handle = *g_fic_i2c_ports[port].bus_ptr;
    if (bus_handle == NULL) {
        return false;
    }
    if (!fic_i2c_lock_port(port, 200)) {
        return false;
    }
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = dev_addr,
        .scl_speed_hz    = scl_speed_hz ? scl_speed_hz : 400000U,
        .scl_wait_us     = scl_wait_us ? scl_wait_us : 20000U,
    };
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, out_handle);
    fic_i2c_unlock_port(port);
    if (err != ESP_OK) {
        *out_handle = NULL;
        return false;
    }
    return true;
}

bool fic_i2c_add_i2c0_device(uint8_t dev_addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_handle)
{
    uint32_t spd = scl_speed_hz ? scl_speed_hz : (g_fic_i2c0_cfg.speed_hz ? g_fic_i2c0_cfg.speed_hz : 400000U);
    uint32_t wait_us = g_fic_i2c0_cfg.scl_wait_us ? g_fic_i2c0_cfg.scl_wait_us : 20000U;
    return fic_i2c_add_device_on_port(0U, dev_addr, spd, wait_us, out_handle);
}

static bool fic_i2c0_bringup_locked(const FicI2cBusCfg_t *cfg)
{
    if (cfg == NULL || cfg->scl_pin < 0 || cfg->sda_pin < 0) {
        return false;
    }
    if (fic_i2c0_bus != NULL) {
        return true;
    }

    g_fic_i2c_ports[0].scl_pin = (gpio_num_t)cfg->scl_pin;
    g_fic_i2c_ports[0].sda_pin = (gpio_num_t)cfg->sda_pin;
    // 40 ms software timeout clears 20 ms .scl_wait_us slave stretching margin plus RTOS jitter
    g_fic_i2c_timeout_ticks = 40;

    // Clear any slave holding SDA low across warm/UPS reset before attaching hardware I2C master
    fic_i2c_gpio_bus_clear();

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)cfg->sda_pin,
        .scl_io_num = (gpio_num_t)cfg->scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = cfg->enable_internal_pullup,
        },
    };
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &fic_i2c0_bus);
    if (err != ESP_OK || fic_i2c0_bus == NULL) {
        fic_i2c0_bus = NULL;
        fic_i2c_internal_log("[I2C0 WARN] i2c_new_master_bus failed err=0x%X\n", (unsigned int)err);
        return false;
    }
    i2c_master_bus_reset(fic_i2c0_bus);
    vTaskDelay(pdMS_TO_TICKS(10));

    const uint32_t speed_hz = cfg->speed_hz ? cfg->speed_hz : 400000U;
    const uint32_t wait_us  = cfg->scl_wait_us ? cfg->scl_wait_us : 20000U;

    struct {
        uint8_t                  addr;
        i2c_master_dev_handle_t *handle;
    } devs[] = {
        { cfg->addr_rtc,   &fic_rtc_dev },
        { cfg->addr_imu,   &fic_imu_dev },
        { cfg->addr_touch, &fic_touch_dev },
        { cfg->addr_pmic,  &fic_pmic_dev },
        { cfg->addr_mag,   &fic_mag_dev },
    };

    for (size_t i = 0; i < (sizeof(devs) / sizeof(devs[0])); i++) {
        if (devs[i].addr != 0U) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = devs[i].addr,
                .scl_speed_hz    = speed_hz,
                .scl_wait_us     = wait_us,
            };
            if (i2c_master_bus_add_device(fic_i2c0_bus, &dev_cfg, devs[i].handle) != ESP_OK) {
                *(devs[i].handle) = NULL;
            }
        }
    }
    return true;
}

bool fic_i2c_init(const FicI2cBusCfg_t *bus0_cfg, const FicI2cAuxCfg_t *bus1_cfg)
{
    if (bus0_cfg == NULL) {
        return false;
    }
    if (g_fic_i2c_ports[0].mutex == NULL) {
        g_fic_i2c_ports[0].mutex = xSemaphoreCreateRecursiveMutex();
    }
    g_fic_i2c0_cfg = *bus0_cfg;
    g_fic_i2c0_configured = true;

    bool ok0 = true;
    if (fic_i2c_lock_port(0U, 500)) {
        ok0 = fic_i2c0_bringup_locked(&g_fic_i2c0_cfg);
        fic_i2c_unlock_port(0U);
    } else {
        ok0 = false;
    }

    if (bus1_cfg != NULL && bus1_cfg->enabled) {
        (void)fic_i2c_init_bus1(bus1_cfg);
    }
    return ok0;
}

bool fic_i2c_init_from_board(const FicBoardProfile_t *profile)
{
    if (profile == NULL) {
        return false;
    }
    return fic_i2c_init(&profile->i2c0, profile->i2c1.enabled ? &profile->i2c1 : NULL);
}

bool fic_i2c_attach_beam_imu_locked(uint8_t port, uint8_t addr_7bit)
{
    if (port > 1U || addr_7bit < 0x08U || addr_7bit > 0x77U) {
        return false;
    }
    i2c_master_bus_handle_t bus_handle = *g_fic_i2c_ports[port].bus_ptr;
    if (bus_handle == NULL) {
        return false;
    }
    if (fic_beam_imu_dev != NULL && g_fic_beam_imu_addr == addr_7bit && g_fic_beam_imu_bus == port) {
        return true;
    }
    if (fic_beam_imu_dev != NULL) {
        i2c_master_bus_rm_device(fic_beam_imu_dev);
        fic_beam_imu_dev = NULL;
    }

    uint32_t spd = 400000U;
    uint32_t wait_us = 20000U;
    if (port == 1U && g_fic_i2c1_configured) {
        if (g_fic_i2c1_cfg.speed_hz) spd = g_fic_i2c1_cfg.speed_hz;
        if (g_fic_i2c1_cfg.scl_wait_us) wait_us = g_fic_i2c1_cfg.scl_wait_us;
    } else if (port == 0U && g_fic_i2c0_configured) {
        if (g_fic_i2c0_cfg.speed_hz) spd = g_fic_i2c0_cfg.speed_hz;
        if (g_fic_i2c0_cfg.scl_wait_us) wait_us = g_fic_i2c0_cfg.scl_wait_us;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr_7bit,
        .scl_speed_hz    = spd,
        .scl_wait_us     = wait_us,
    };
    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &fic_beam_imu_dev) != ESP_OK) {
        fic_beam_imu_dev = NULL;
        return false;
    }
    g_fic_beam_imu_addr = addr_7bit;
    g_fic_beam_imu_bus  = port;
    return true;
}

bool fic_i2c_attach_beam_imu_port(uint8_t port, uint8_t addr_7bit)
{
    if (port > 1U) {
        return false;
    }
    if (fic_beam_imu_dev != NULL && g_fic_beam_imu_bus != port) {
        fic_i2c_detach_beam_imu();
    }
    if (!fic_i2c_lock_port(port, 200)) {
        return false;
    }
    bool ok = fic_i2c_attach_beam_imu_locked(port, addr_7bit);
    fic_i2c_unlock_port(port);
    return ok;
}

bool fic_i2c_attach_beam_imu(uint8_t addr_7bit)
{
    return fic_i2c_attach_beam_imu_port(0U, addr_7bit);
}

void fic_i2c_detach_beam_imu(void)
{
    uint8_t bus = g_fic_beam_imu_bus;
    if (!fic_i2c_lock_port(bus, 200)) {
        return;
    }
    if (fic_beam_imu_dev != NULL) {
        i2c_master_bus_rm_device(fic_beam_imu_dev);
        fic_beam_imu_dev = NULL;
    }
    g_fic_beam_imu_addr = 0;
    g_fic_beam_imu_bus  = 0;
    fic_i2c_unlock_port(bus);
}

uint8_t fic_i2c_get_beam_imu_bus(void)
{
    return g_fic_beam_imu_bus;
}

uint32_t fic_i2c_get_bus_err_count(uint8_t port)
{
    return (port <= 1U) ? g_fic_i2c_ports[port].err_count : 0U;
}

void fic_i2c_reinit_bus(void)
{
    if (!g_fic_i2c0_configured || !fic_i2c_lock_port(0U, 500)) {
        return;
    }
    fic_i2c_invoke_hooks_port(0U, false);

    if (fic_rtc_dev)      { i2c_master_bus_rm_device(fic_rtc_dev);      fic_rtc_dev = NULL; }
    if (fic_imu_dev)      { i2c_master_bus_rm_device(fic_imu_dev);      fic_imu_dev = NULL; }
    if (fic_touch_dev && g_fic_i2c0_cfg.addr_touch != 0U) {
        i2c_master_bus_rm_device(fic_touch_dev);
        fic_touch_dev = NULL;
    }
    if (fic_pmic_dev)     { i2c_master_bus_rm_device(fic_pmic_dev);     fic_pmic_dev = NULL; }
    if (fic_mag_dev)      { i2c_master_bus_rm_device(fic_mag_dev);      fic_mag_dev = NULL; }
    if (fic_beam_imu_dev && g_fic_beam_imu_bus == 0U) {
        i2c_master_bus_rm_device(fic_beam_imu_dev);
        fic_beam_imu_dev = NULL;
    }

    if (fic_i2c0_bus != NULL) {
        esp_err_t del_err = i2c_del_master_bus(fic_i2c0_bus);
        if (del_err != ESP_OK) {
            fic_i2c_internal_log("[I2C0 WARN] i2c_del_master_bus returned err=0x%X; falling back to bus_reset\n",
                                 (unsigned int)del_err);
            i2c_master_bus_reset(fic_i2c0_bus);
        } else {
            fic_i2c0_bus = NULL;
        }
    }

    if (fic_i2c0_bus == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
        (void)fic_i2c0_bringup_locked(&g_fic_i2c0_cfg);
    }
    if (g_fic_beam_imu_addr != 0 && g_fic_beam_imu_bus == 0U) {
        (void)fic_i2c_attach_beam_imu_locked(0U, g_fic_beam_imu_addr);
    }
    fic_i2c_invoke_hooks_port(0U, true);
    g_fic_i2c_ports[0].consec_timeouts = 0;
    fic_i2c_unlock_port(0U);
}
