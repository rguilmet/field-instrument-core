#ifndef FIC_I2C_INTERNAL_H
#define FIC_I2C_INTERNAL_H

#include "fic_i2c_bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FIC_I2C_MAX_BUS_HOOKS 4

typedef struct {
    i2c_port_t              port_num;
    gpio_num_t              scl_pin;
    gpio_num_t              sda_pin;
    uint32_t                scl_out_idx;
    uint32_t                scl_in_idx;
    uint32_t                sda_out_idx;
    uint32_t                sda_in_idx;
    i2c_master_bus_handle_t *bus_ptr;
    SemaphoreHandle_t       mutex;
    uint8_t                 consec_timeouts;
    uint32_t                last_gpio_clear_ms;
    uint32_t                last_reinit_ms;
    uint32_t                last_warn_ms;
    uint32_t                err_count;
} fic_i2c_port_state_t;

typedef struct {
    fic_i2c_bus_hook_t detach_fn;
    fic_i2c_bus_hook_t reattach_fn;
    void              *ctx;
} fic_i2c_bus_hook_entry_t;

extern fic_i2c_port_state_t g_fic_i2c_ports[2];
extern FicI2cBusCfg_t       g_fic_i2c0_cfg;
extern FicI2cAuxCfg_t       g_fic_i2c1_cfg;
extern bool                 g_fic_i2c0_configured;
extern bool                 g_fic_i2c1_configured;
extern uint8_t              g_fic_beam_imu_addr;
extern uint8_t              g_fic_beam_imu_bus;
extern uint32_t             g_fic_i2c_timeout_ticks;

void fic_i2c_internal_log(const char *fmt, ...);
void fic_i2c_gpio_bus_clear_pins(gpio_num_t scl_pin, gpio_num_t sda_pin);
void fic_i2c_recover_port_locked(uint8_t port, const char *action, i2c_master_dev_handle_t dev_handle, int reg, esp_err_t err);
bool fic_i2c_attach_beam_imu_locked(uint8_t port, uint8_t addr_7bit);
bool fic_i2c_add_device_on_port(uint8_t port, uint8_t dev_addr, uint32_t scl_speed_hz, uint32_t scl_wait_us, i2c_master_dev_handle_t *out_handle);
bool fic_i2c_register_hook_port(uint8_t port, fic_i2c_bus_hook_t detach_fn, fic_i2c_bus_hook_t reattach_fn, void *ctx);
void fic_i2c_unregister_hook_port(uint8_t port, void *ctx);
void fic_i2c_invoke_hooks_port(uint8_t port, bool is_reattach);
bool fic_i2c1_bringup_locked(const FicI2cAuxCfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif // FIC_I2C_INTERNAL_H
