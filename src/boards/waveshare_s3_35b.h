#ifndef FIC_WAVESHARE_S3_35B_H
#define FIC_WAVESHARE_S3_35B_H

#include "fic_board_types.h"
#include "hal/spi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hardware board preset for Waveshare ESP32-S3-Touch-LCD-3.5B.
 *
 * I2C0 (SCL=7, SDA=8): TCA9554 (0x20), MMC5983MA (0x30), AXP2101 (0x34),
 * Touch (0x3B), PCF85063 (0x51), QMI8658 (0x6B), ES8311 (0x18).
 * I2C1 (SCL=48, SDA=47): External/outboard sensors (e.g. SEN0696 10-DOF IMU).
 */
static inline FicBoardProfile_t fic_board_waveshare_s3_35b(void)
{
    FicBoardProfile_t p = {
        .name = "Waveshare ESP32-S3-Touch-LCD-3.5B",
        .i2c0 = {
            .scl_pin                = 7,
            .sda_pin                = 8,
            .speed_hz               = 400000U,
            .scl_wait_us            = 20000U,
            .enable_internal_pullup = true,
            .addr_rtc               = 0x51U,
            .addr_imu               = 0x6BU,
            .addr_touch             = 0x3BU,
            .addr_pmic              = 0x34U,
            .addr_mag               = 0x30U,
        },
        .i2c1 = {
            .enabled                = true,
            .scl_pin                = 48,
            .sda_pin                = 47,
            .speed_hz               = 400000U,
            .scl_wait_us            = 20000U,
            .enable_internal_pullup = true,
        },
        .display = {
            .h_res              = 320U,
            .v_res              = 480U,
            .spi_host           = SPI3_HOST,
            .pin_cs             = 12,
            .pin_pclk           = 5,
            .pin_data0          = 1,
            .pin_data1          = 2,
            .pin_data2          = 3,
            .pin_data3          = 4,
            .pin_te             = -1,
            .pin_rst            = -1,
            .pin_bk_light       = 6,
            .pclk_hz            = 40000000U,
            .tca9554_addr       = 0x20U,
            .exio_input_mask    = (1UL << 2) | (1UL << 4) | (1UL << 5) | (1UL << 6),
            .exio_output_mask   = (1UL << 0) | (1UL << 1) | (1UL << 7),
            .exio_lcd_rst_mask  = (1UL << 1),
            .exio_cam_pwdn_mask = (1UL << 0),
            .exio_bl_en_mask    = 0UL,
        },
        .pin_boot_btn     = 0,
        .pin_bat_adc      = -1,
        .has_pmic_axp2101 = true,
        .has_sd_mmc       = true,
        .has_sd_spi       = false,
    };
    return p;
}

#ifdef __cplusplus
}
#endif

#endif // FIC_WAVESHARE_S3_35B_H
