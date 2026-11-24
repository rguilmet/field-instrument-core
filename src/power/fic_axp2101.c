#include <stdio.h>
#include <esp_err.h>
#include "fic_axp2101.h"
#include "../bus/fic_i2c_bsp.h"

// AXP2101 Core Telemetry & LDO Registers (per X-Powers AXP2101 Datasheet V1.0 & Waveshare 3.5B BSP)
#define AXP2101_REG_STATUS1       0x00 // Bit 5: VBUS good, Bit 3: BAT present
#define AXP2101_REG_STATUS2       0x01 // Bits [6:5]: Bat dir (00=standby,01=charging,10=discharging); Bits [2:0]: Chg phase (1..3=active,4=done,5=off)
#define AXP2101_REG_IC_TYPE       0x03 // Expected Chip ID = 0x4A
#define AXP2101_REG_COMM_CFG      0x10 // Bit 0: Soft PWROFF
#define AXP2101_REG_ADC_EN        0x30 // Bit 0: VBAT, Bit 1: TS (Disable=0), Bit 2: VBUS, Bit 3: VSYS
#define AXP2101_REG_VBAT_H        0x34 // 14-bit VBAT mV: ((H & 0x3F) << 8) | L
#define AXP2101_REG_VBAT_L        0x35
#define AXP2101_REG_VBUS_H        0x38 // 14-bit VBUS mV: ((H & 0x3F) << 8) | L
#define AXP2101_REG_VBUS_L        0x39
#define AXP2101_REG_ICC_CFG       0x62 // Bits [4:0] Constant-current charge limit (0x0B = 500mA)
#define AXP2101_REG_DC_ONOFF      0x80 // Bits [4:0] = DCDC1..5 Enable
#define AXP2101_REG_LDO_ONOFF0    0x90 // Bits [3:0]=ALDO1..4, [5:4]=BLDO1..2, [6]=CPUSLDO, [7]=DLDO1
#define AXP2101_REG_LDO_ONOFF1    0x91 // Bit 0 = DLDO2 Enable
#define AXP2101_REG_ALDO1_VOL     0x92 // 0x1C = 3300 mV ((3300 - 500) / 100)
#define AXP2101_REG_ALDO2_VOL     0x93
#define AXP2101_REG_ALDO3_VOL     0x94
#define AXP2101_REG_ALDO4_VOL     0x95
#define AXP2101_REG_BLDO1_VOL     0x96 // 0x0A = 1500 mV (or 0x1C = 3300 mV)
#define AXP2101_REG_BLDO2_VOL     0x97 // 0x17 = 2800 mV (or 0x1C = 3300 mV)
#define AXP2101_REG_CPUSLDO_VOL   0x98 // 0x0A = 1000 mV
#define AXP2101_REG_DLDO1_VOL     0x99 // 0x1C = 3300 mV
#define AXP2101_REG_DLDO2_VOL     0x9A // 0x1C = 3300 mV
#define AXP2101_REG_BAT_PERCENT   0xA4 // Fuel gauge percentage (0..100%)

static bool         s_axp_online = false;
static fic_log_cb_t s_axp_log_cb = NULL;

#define axp_log(...)                                \
    do {                                            \
        if (s_axp_log_cb != NULL) {                 \
            s_axp_log_cb(__VA_ARGS__);              \
        } else {                                    \
            printf(__VA_ARGS__);                    \
        }                                           \
    } while (0)

void fic_axp2101_set_log_callback(fic_log_cb_t cb)
{
    s_axp_log_cb = cb;
}

void fic_axp2101_init(void)
{
    if (fic_pmic_dev == NULL) {
        return;
    }

    uint8_t chip_id = 0;
    if (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_IC_TYPE, &chip_id, 1) == ESP_OK) {
        s_axp_online = true;
        // Enable VBAT (bit 0), VBUS (bit 2), VSYS (bit 3) ADCs; disable TS pin (bit 1 = 0) per Waveshare BSP
        uint8_t adc_en = 0x0D;
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_ADC_EN, &adc_en, 1);

        // Configure and enable all LDO rails per Waveshare 01_factory/components/esp_bsp/bsp_axp2101.cpp
        uint8_t vol_3v3  = 0x1C; // 3300 mV
        uint8_t vol_1v5  = 0x0A; // 1500 mV
        uint8_t vol_2v8  = 0x17; // 2800 mV
        uint8_t vol_1v0  = 0x0A; // 1000 mV (CPUSLDO)
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_ALDO1_VOL,   &vol_3v3, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_ALDO2_VOL,   &vol_3v3, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_ALDO3_VOL,   &vol_3v3, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_ALDO4_VOL,   &vol_3v3, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_BLDO1_VOL,   &vol_1v5, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_BLDO2_VOL,   &vol_2v8, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_CPUSLDO_VOL, &vol_1v0, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_DLDO1_VOL,   &vol_3v3, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_DLDO2_VOL,   &vol_3v3, 1);

        // Enable ALDO1..4, BLDO1..2, CPUSLDO, DLDO1 (0x90 = 0xFF) and DLDO2 (0x91 |= 0x01)
        uint8_t ldo0 = 0xFF;
        uint8_t ldo1 = 0x01;
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_LDO_ONOFF0, &ldo0, 1);
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_LDO_ONOFF1, &ldo1, 1);

        uint8_t dc_en = 0;
        if (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_DC_ONOFF, &dc_en, 1) == ESP_OK) {
            dc_en |= 0x1F; // Ensure DCDC1..5 enabled
            fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_DC_ONOFF, &dc_en, 1);
        }

        // Configure safe 500mA charge current (0.5C for 1000mAh battery) per AXP2101 REG 0x62 (01011b = 0x0B)
        uint8_t icc_500ma = 0x0B;
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_ICC_CFG, &icc_500ma, 1);

        FicAxp2101Status_t st = {0};
        fic_axp2101_read(&st);
        axp_log("[PMIC] AXP2101 (0x34) ONLINE -- ChipID: 0x%02X | LDO0:0x%02X DC:0x%02X | VBAT: %.2fV (%u%%) | VBUS: %.2fV (%s) | Charging: %s\n",
                chip_id, ldo0, dc_en, st.battery_voltage_v, st.battery_percent,
                st.vbus_voltage_v, st.vbus_good ? "YES" : "NO", st.is_charging ? "YES" : "NO");
    } else {
        axp_log("[PMIC] AXP2101 (0x34) FAIL -- Not responding on I2C0\n");
    }
}

void fic_axp2101_read(FicAxp2101Status_t *status)
{
    if (!status) return;
    status->online = s_axp_online;
    if (!s_axp_online || fic_pmic_dev == NULL) {
        status->battery_voltage_v = 0.0f;
        status->battery_percent   = 0;
        status->is_charging       = false;
        status->vbus_good         = false;
        status->vbus_voltage_v    = 0.0f;
        return;
    }

    uint8_t st[2] = {0};
    bool raw_vbus_good = false;
    bool bat_present   = false;
    const bool st_ok = (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_STATUS1, st, 2) == ESP_OK);
    // Running system cannot physically have STATUS1==0x00 && STATUS2==0x00 or repeated 0x61/0x71 sentinel bytes
    const bool st_valid = st_ok && !(st[0] == 0x00U && st[1] == 0x00U) && !fic_i2c_has_sentinel_run(st, sizeof(st), 2U);
    if (st_valid) {
        raw_vbus_good = (st[0] & (1 << 5)) != 0;
        bat_present   = (st[0] & (1 << 3)) != 0;

        // Decode STATUS2 (0x01): Bits [6:5] = Battery Current Direction (01b = Charging, 10b = Discharging),
        // Bits [2:0] = Charger Phase (1=Tri-charge, 2=CC, 3=CV, 4=Done, 5=Not charging).
        uint8_t chg_dir  = (st[1] >> 5) & 0x03;
        uint8_t chg_stat = st[1] & 0x07;
        status->vbus_good   = raw_vbus_good;
        status->is_charging = raw_vbus_good && (chg_dir == 0x01) && (chg_stat >= 1 && chg_stat <= 3);
    } else {
        // Retain prior valid vbus_good on transient I2C bus contention
        raw_vbus_good = status->vbus_good;
        bat_present   = (status->battery_voltage_v > 2.5f);
    }

    // Read VBUS ADC strictly gated on STATUS1 bit 5 comparator (VBUS good).
    // Note: On AXP2101, when VBUS is unplugged, the 14-bit VBUS ADC channel (0x38/0x39)
    // floats near full-scale (0x3FF2 = 16,370 mV = 16.37V), which previously latched vbus_good=true.
    if (raw_vbus_good) {
        uint8_t vbus_raw[2] = {0};
        if (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_VBUS_H, vbus_raw, 2) == ESP_OK) {
            uint16_t vbus_mv = ((uint16_t)(vbus_raw[0] & 0x3F) << 8) | vbus_raw[1];
            if (vbus_mv >= 3800 && vbus_mv <= 6500) {
                status->vbus_voltage_v = (float)vbus_mv / 1000.0f;
            } else if (status->vbus_voltage_v < 3.8f || status->vbus_voltage_v > 6.5f) {
                status->vbus_voltage_v = 5.00f;
            }
        } else if (status->vbus_voltage_v < 3.8f || status->vbus_voltage_v > 6.5f) {
            status->vbus_voltage_v = 5.00f;
        }
    } else {
        status->vbus_good      = false;
        status->vbus_voltage_v = 0.0f;
        status->is_charging    = false;
    }

    if (bat_present) {
        uint8_t vbat_raw[2] = {0};
        if (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_VBAT_H, vbat_raw, 2) == ESP_OK) {
            uint16_t mv = ((uint16_t)(vbat_raw[0] & 0x3F) << 8) | vbat_raw[1];
            if (mv >= 2500 && mv <= 4550) {
                status->battery_voltage_v = (float)mv / 1000.0f;
            }
        }
    } else {
        status->battery_voltage_v = 0.0f;
    }

    uint8_t pct = 0;
    if (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_BAT_PERCENT, &pct, 1) == ESP_OK) {
        if (pct <= 100) {
            status->battery_percent = pct;
        } else if (status->battery_voltage_v > 3.0f) {
            // Linear fallback if fuel gauge is uncalibrated
            float p = (status->battery_voltage_v - 3.30f) / (4.18f - 3.30f) * 100.0f;
            if (p < 0.0f) p = 0.0f;
            if (p > 100.0f) p = 100.0f;
            status->battery_percent = (uint8_t)p;
        }
    }
}

void fic_axp2101_power_off(void)
{
    if (fic_pmic_dev == NULL || !s_axp_online) return;
    axp_log("[PMIC] Initiating AXP2101 Soft PWROFF...\n");
    uint8_t reg_val = 0;
    if (fic_i2c_read_buff(fic_pmic_dev, AXP2101_REG_COMM_CFG, &reg_val, 1) == ESP_OK) {
        reg_val |= 0x01; // Bit 0 = Soft PWROFF
        fic_i2c_write_buff(fic_pmic_dev, AXP2101_REG_COMM_CFG, &reg_val, 1);
    }
}
