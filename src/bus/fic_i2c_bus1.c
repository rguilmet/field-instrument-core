#include "fic_i2c_internal.h"
#include "freertos/task.h"

void fic_i2c_bus1_gpio_bus_clear(void)
{
    fic_i2c_gpio_bus_clear_pins(g_fic_i2c_ports[1].scl_pin, g_fic_i2c_ports[1].sda_pin);
}

bool fic_i2c_lock_bus1(uint32_t timeout_ms)
{
    return fic_i2c_lock_port(1U, timeout_ms);
}

void fic_i2c_unlock_bus1(void)
{
    fic_i2c_unlock_port(1U);
}

void fic_i2c_recover_bus1(void)
{
    if (!fic_i2c_lock_port(1U, 250)) {
        return;
    }
    if (fic_i2c1_bus != NULL) {
        i2c_master_bus_reset(fic_i2c1_bus);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    fic_i2c_unlock_port(1U);
}

bool fic_i2c1_bringup_locked(const FicI2cAuxCfg_t *cfg)
{
    if (cfg == NULL || !cfg->enabled || cfg->scl_pin < 0 || cfg->sda_pin < 0) {
        return false;
    }
    if (fic_i2c1_bus != NULL) {
        return true;
    }

    g_fic_i2c_ports[1].scl_pin = (gpio_num_t)cfg->scl_pin;
    g_fic_i2c_ports[1].sda_pin = (gpio_num_t)cfg->sda_pin;
    if (g_fic_i2c_timeout_ticks == 0) {
        g_fic_i2c_timeout_ticks = 40;
    }

    // Release any slave holding external I2C1 SDA LOW across resets
    fic_i2c_bus1_gpio_bus_clear();

    i2c_master_bus_config_t i2c1_bus_config = {
        .i2c_port = I2C_NUM_1,
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
    esp_err_t err = i2c_new_master_bus(&i2c1_bus_config, &fic_i2c1_bus);
    if (err != ESP_OK || fic_i2c1_bus == NULL) {
        fic_i2c1_bus = NULL;
        fic_i2c_internal_log("[I2C1 WARN] i2c_new_master_bus(I2C1 SCL=%d SDA=%d) failed err=0x%X\n",
                             cfg->scl_pin, cfg->sda_pin, (unsigned int)err);
        return false;
    }
    i2c_master_bus_reset(fic_i2c1_bus);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (cfg->addr_touch != 0U && fic_touch_dev == NULL) {
        i2c_device_config_t touch_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = cfg->addr_touch,
            .scl_speed_hz    = cfg->speed_hz ? cfg->speed_hz : 200000U,
            .scl_wait_us     = cfg->scl_wait_us ? cfg->scl_wait_us : 20000U,
        };
        esp_err_t touch_err = i2c_master_bus_add_device(fic_i2c1_bus, &touch_cfg, &fic_touch_dev);
        if (touch_err != ESP_OK) {
            fic_touch_dev = NULL;
            fic_i2c_internal_log("[I2C1 WARN] add touch 0x%02X failed err=0x%X\n",
                                 cfg->addr_touch, (unsigned int)touch_err);
        }
    }
    return true;
}

bool fic_i2c_init_bus1(const FicI2cAuxCfg_t *bus1_cfg)
{
    if (bus1_cfg == NULL || !bus1_cfg->enabled) {
        return false;
    }
    if (g_fic_i2c_ports[1].mutex == NULL) {
        g_fic_i2c_ports[1].mutex = xSemaphoreCreateRecursiveMutex();
    }
    g_fic_i2c1_cfg = *bus1_cfg;
    g_fic_i2c1_configured = true;

    if (!fic_i2c_lock_port(1U, 500)) {
        return false;
    }
    bool ok = fic_i2c1_bringup_locked(&g_fic_i2c1_cfg);
    fic_i2c_unlock_port(1U);
    return ok;
}

bool fic_i2c_add_i2c1_device(uint8_t dev_addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_handle)
{
    uint32_t spd = scl_speed_hz ? scl_speed_hz : (g_fic_i2c1_cfg.speed_hz ? g_fic_i2c1_cfg.speed_hz : 400000U);
    uint32_t wait_us = g_fic_i2c1_cfg.scl_wait_us ? g_fic_i2c1_cfg.scl_wait_us : 20000U;
    return fic_i2c_add_device_on_port(1U, dev_addr, spd, wait_us, out_handle);
}

bool fic_i2c_register_bus1_hooks(fic_i2c_bus_hook_t detach_fn, fic_i2c_bus_hook_t reattach_fn, void *ctx)
{
    return fic_i2c_register_hook_port(1U, detach_fn, reattach_fn, ctx);
}

void fic_i2c_unregister_bus1_hooks(void *ctx)
{
    fic_i2c_unregister_hook_port(1U, ctx);
}

void fic_i2c_reinit_bus1(void)
{
    if (!g_fic_i2c1_configured || !fic_i2c_lock_port(1U, 500)) {
        return;
    }
    fic_i2c_invoke_hooks_port(1U, false);

    if (fic_touch_dev != NULL && g_fic_i2c1_cfg.addr_touch != 0U) {
        i2c_master_bus_rm_device(fic_touch_dev);
        fic_touch_dev = NULL;
    }
    if (fic_beam_imu_dev != NULL && g_fic_beam_imu_bus == 1U) {
        i2c_master_bus_rm_device(fic_beam_imu_dev);
        fic_beam_imu_dev = NULL;
    }
    if (fic_i2c1_bus != NULL) {
        esp_err_t del_err = i2c_del_master_bus(fic_i2c1_bus);
        if (del_err != ESP_OK) {
            fic_i2c_internal_log("[I2C1 WARN] i2c_del_master_bus returned err=0x%X; falling back to bus_reset\n",
                                 (unsigned int)del_err);
            i2c_master_bus_reset(fic_i2c1_bus);
        } else {
            fic_i2c1_bus = NULL;
        }
    }
    if (fic_i2c1_bus == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
        (void)fic_i2c1_bringup_locked(&g_fic_i2c1_cfg);
    }
    if (g_fic_beam_imu_addr != 0 && g_fic_beam_imu_bus == 1U && fic_i2c1_bus != NULL) {
        (void)fic_i2c_attach_beam_imu_locked(1U, g_fic_beam_imu_addr);
    }
    fic_i2c_invoke_hooks_port(1U, true);
    g_fic_i2c_ports[1].consec_timeouts = 0;
    fic_i2c_unlock_port(1U);
}

uint8_t fic_i2c_write_buff_bus1(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    return fic_i2c_write_buff_port(1U, dev_handle, reg, buf, len);
}

uint8_t fic_i2c_read_buff_bus1(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    return fic_i2c_read_buff_port(1U, dev_handle, reg, buf, len);
}
