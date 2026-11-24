#ifndef FIC_I2C_BSP_H
#define FIC_I2C_BSP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "../boards/fic_board_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*fic_i2c_bus_hook_t)(i2c_master_bus_handle_t bus_handle, void *ctx);

extern i2c_master_bus_handle_t fic_i2c0_bus;
extern i2c_master_bus_handle_t fic_i2c1_bus;
extern i2c_master_dev_handle_t fic_touch_dev;
extern i2c_master_dev_handle_t fic_rtc_dev;
extern i2c_master_dev_handle_t fic_imu_dev;
extern i2c_master_dev_handle_t fic_pmic_dev;
extern i2c_master_dev_handle_t fic_mag_dev;
extern i2c_master_dev_handle_t fic_beam_imu_dev;

void fic_i2c_set_log_callback(fic_log_cb_t cb);
bool fic_i2c_init(const FicI2cBusCfg_t *bus0_cfg, const FicI2cAuxCfg_t *bus1_cfg);
bool fic_i2c_init_from_board(const FicBoardProfile_t *profile);
bool fic_i2c_init_bus1(const FicI2cAuxCfg_t *bus1_cfg);

void fic_i2c_gpio_bus_clear(void);
void fic_i2c_bus1_gpio_bus_clear(void);
void fic_i2c_reinit_bus(void);
void fic_i2c_reinit_bus1(void);
void fic_i2c_recover_bus(void);
void fic_i2c_recover_bus1(void);

bool fic_i2c_add_i2c0_device(uint8_t dev_addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_handle);
bool fic_i2c_add_i2c1_device(uint8_t dev_addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t *out_handle);

bool fic_i2c_attach_beam_imu(uint8_t addr_7bit);
bool fic_i2c_attach_beam_imu_port(uint8_t port, uint8_t addr_7bit);
void fic_i2c_detach_beam_imu(void);
uint8_t fic_i2c_get_beam_imu_bus(void);
uint32_t fic_i2c_get_bus_err_count(uint8_t port);

bool fic_i2c_register_bus_hooks(fic_i2c_bus_hook_t detach_fn, fic_i2c_bus_hook_t reattach_fn, void *ctx);
void fic_i2c_unregister_bus_hooks(void *ctx);
bool fic_i2c_register_bus1_hooks(fic_i2c_bus_hook_t detach_fn, fic_i2c_bus_hook_t reattach_fn, void *ctx);
void fic_i2c_unregister_bus1_hooks(void *ctx);

bool fic_i2c_has_sentinel_run(const uint8_t *buf, size_t len, size_t min_run);
bool fic_i2c_has_sentinel_tail(const uint8_t *buf, size_t len, size_t tail_len);

void fic_i2c_report_status(i2c_master_dev_handle_t dev_handle, const char *action, int err_code);
void fic_i2c_report_status_port(uint8_t port, i2c_master_dev_handle_t dev_handle, const char *action, int err_code);

bool fic_i2c_lock(uint32_t timeout_ms);
void fic_i2c_unlock(void);
bool fic_i2c_lock_bus1(uint32_t timeout_ms);
void fic_i2c_unlock_bus1(void);
bool fic_i2c_lock_port(uint8_t port, uint32_t timeout_ms);
void fic_i2c_unlock_port(uint8_t port);

bool fic_i2c_probe_addr(uint8_t addr_7bit);
bool fic_i2c_probe_optional_addr(uint8_t addr_7bit);
bool fic_i2c_probe_optional_addr_port(uint8_t port, uint8_t addr_7bit);

uint8_t fic_i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);
uint8_t fic_i2c_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen);
uint8_t fic_i2c_write_read_dev_port(uint8_t port, i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen);
uint8_t fic_i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);

uint8_t fic_i2c_write_buff_port(uint8_t port, i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);
uint8_t fic_i2c_read_buff_port(uint8_t port, i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);
uint8_t fic_i2c_write_buff_bus1(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);
uint8_t fic_i2c_read_buff_bus1(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // FIC_I2C_BSP_H
