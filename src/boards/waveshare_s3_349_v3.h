#ifndef FIC_WAVESHARE_S3_349_V3_H
#define FIC_WAVESHARE_S3_349_V3_H

#include "fic_board_types.h"
#include "hal/spi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hardware board preset for Waveshare ESP32-S3-Touch-LCD-3.49 v3.
 *
 * I2C0 (SCL=48, SDA=47): TCA9554 (0x20), PCF85063 (0x51), QMI8658 (0x6B).
 * I2C1 (SCL=18, SDA=17): Capacitive touch controller (0x3B).
 */
static inline FicBoardProfile_t fic_board_waveshare_s3_349_v3(void)
{
    FicBoardProfile_t p = {
        .name = "Waveshare ESP32-S3-Touch-LCD-3.49-v3",
        .i2c0 = {
            .scl_pin                = 48,
            .sda_pin                = 47,
            .speed_hz               = 100000U,
            .scl_wait_us            = 20000U,
            .enable_internal_pullup = true,
            .addr_rtc               = 0x51U,
            .addr_imu               = 0x6BU,
            .addr_touch             = 0x00U,
            .addr_pmic              = 0x00U,
            .addr_mag               = 0x00U,
        },
        .i2c1 = {
            .enabled                = true,
            .scl_pin                = 18,
            .sda_pin                = 17,
            .speed_hz               = 400000U,
            .scl_wait_us            = 20000U,
            .enable_internal_pullup = true,
        },
        .display = {
            .h_res              = 172U,
            .v_res              = 640U,
            .spi_host           = SPI3_HOST,
            .pin_cs             = 9,
            .pin_pclk           = 10,
            .pin_data0          = 11,
            .pin_data1          = 12,
            .pin_data2          = 13,
            .pin_data3          = 14,
            .pin_te             = 21,
            .pin_rst            = -1,
            .pin_bk_light       = 42,
            .pclk_hz            = 40000000U,
            .tca9554_addr       = 0x20U,
            .exio_input_mask    = (1UL << 0) | (1UL << 2) | (1UL << 3) | (1UL << 4),
            .exio_output_mask   = (1UL << 1) | (1UL << 5) | (1UL << 6) | (1UL << 7),
            .exio_lcd_rst_mask  = (1UL << 5),
            .exio_cam_pwdn_mask = 0UL,
            .exio_bl_en_mask    = (1UL << 1),
        },
        .pin_boot_btn     = 0,
        .pin_bat_adc      = 4,
        .has_pmic_axp2101 = false,
        .has_sd_mmc       = false,
        .has_sd_spi       = true,
    };
    return p;
}

#ifdef __cplusplus
}
#endif

#endif // FIC_WAVESHARE_S3_349_V3_H
