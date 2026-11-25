#ifndef FIC_BOARD_TYPES_H
#define FIC_BOARD_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Canonical FreeRTOS dual-core & priority architecture shared across instruments.
 *        Core 0 handles wireless/network & high-rate sensor acquisition.
 *        Core 1 isolates real-time I2S audio (Prio 4) above LVGL (Prio 3),
 *        UI feed / power tasks (Prio 2), and background Flash/SD I/O (Prio 1).
 */
#define FIC_CORE_ACQ        0
#define FIC_CORE_UI         1
#define FIC_PRIO_AUDIO      4U
#define FIC_PRIO_LVGL       3U
#define FIC_PRIO_UI_FEED    2U
#define FIC_PRIO_BACKGROUND 1U

/**
 * @brief Shared printf-compatible log callback injected by consumer firmware.
 */
typedef int (*fic_log_cb_t)(const char *fmt, ...);

/**
 * @brief Primary I2C0 bus pinout, timing, and known onboard 7-bit slave addresses (0 = absent).
 */
typedef struct {
    int      scl_pin;
    int      sda_pin;
    uint32_t speed_hz;
    uint32_t scl_wait_us;
    bool     enable_internal_pullup;
    uint8_t  addr_rtc;
    uint8_t  addr_imu;
    uint8_t  addr_touch;
    uint8_t  addr_pmic;
    uint8_t  addr_mag;
} FicI2cBusCfg_t;

/**
 * @brief Secondary I2C1 bus pinout and timing configuration.
 */
typedef struct {
    bool     enabled;
    int      scl_pin;
    int      sda_pin;
    uint32_t speed_hz;
    uint32_t scl_wait_us;
    bool     enable_internal_pullup;
    uint8_t  addr_touch;
} FicI2cAuxCfg_t;

/**
 * @brief QSPI LCD and TCA9554 EXIO wiring descriptor for a Waveshare ESP32-S3 board.
 */
typedef struct {
    uint16_t h_res;
    uint16_t v_res;
    int      spi_host;
    int      pin_cs;
    int      pin_pclk;
    int      pin_data0;
    int      pin_data1;
    int      pin_data2;
    int      pin_data3;
    int      pin_te;
    int      pin_rst;
    int      pin_bk_light;
    uint32_t pclk_hz;
    uint8_t  tca9554_addr;
    uint32_t exio_input_mask;
    uint32_t exio_output_mask;
    uint32_t exio_lcd_rst_mask;
    uint32_t exio_cam_pwdn_mask;
    uint32_t exio_bl_en_mask;
    uint32_t exio_sys_en_mask;
    uint16_t lcd_rst_low_ms;
    bool     touch_swap_xy;
    bool     touch_invert_y;
    bool     touch_ignore_event_flag;
} FicBoardDisplayDesc_t;

/**
 * @brief Top-level hardware profile descriptor for supported ESP32-S3 carrier boards.
 */
typedef struct {
    const char           *name;
    FicI2cBusCfg_t        i2c0;
    FicI2cAuxCfg_t        i2c1;
    FicBoardDisplayDesc_t display;
    int                   pin_boot_btn;
    int                   pin_bat_adc;
    bool                  has_pmic_axp2101;
    bool                  has_sd_mmc;
    bool                  has_sd_spi;
} FicBoardProfile_t;

#ifdef __cplusplus
}
#endif

#endif // FIC_BOARD_TYPES_H
