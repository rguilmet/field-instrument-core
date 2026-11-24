#ifndef FIC_MMC5983MA_H
#define FIC_MMC5983MA_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "../boards/fic_board_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FIC_MMC5983MA_DEFAULT_ADDR 0x30U

// =========================================================================
// MEMSIC MMC5983MA 3-Axis 18-Bit AMR Magnetometer Driver
// Features:
//  - Auto-detection (probes target address and verifies Reg 0x2F == 0x30)
//  - Hardware SET/RESET degaussing & exact bridge-offset cancellation
//  - 18-bit resolution (16,384 LSB/Gauss = 163.84 LSB/uT, 0.4 mG noise floor)
//  - Tilt-compensated horizontal azimuth using host pitch & roll
//  - Optional hard-iron & 3x3 soft-iron ellipsoid calibration
// =========================================================================

typedef struct {
    bool     present;              // True if MMC5983MA detected on boot
    bool     valid;                // True if latest 18-bit reading is valid
    float    mag_uT[3];            // Bridge-offset-cancelled Bx, By, Bz in microteslas (uT)
    float    total_uT;             // Total magnetic field magnitude ||B|| (uT)
    float    raw_mag_heading_deg;  // Tilt-compensated magnetic heading (0..360 deg, before True-North tare)
    float    die_temp_c;           // On-chip die temperature (deg C)
    float    bridge_offset_lsb[3]; // Measured SET/RESET null-field bridge offset (nominal ~131072 LSB)
    uint32_t set_reset_count;      // Total hardware SET/RESET calibrations completed
} FicMmc5983maData_t;

void fic_mmc5983ma_set_log_callback(fic_log_cb_t cb);
bool fic_mmc5983ma_init_with_device(i2c_master_dev_handle_t dev_handle, uint8_t addr_7bit);
bool fic_mmc5983ma_init(i2c_master_dev_handle_t dev_handle);
bool fic_mmc5983ma_init_default(void);
bool fic_mmc5983ma_is_present(void);
bool fic_mmc5983ma_is_online(void);
bool fic_mmc5983ma_calibrate_bridge_offset(void);
bool fic_mmc5983ma_degauss(void);
void fic_mmc5983ma_set_calibration(const float hard_iron_uT[3], const float soft_iron_3x3[9], bool valid);
bool fic_mmc5983ma_poll(float pitch_deg, float roll_deg, FicMmc5983maData_t *state_out);
bool fic_mmc5983ma_read(float pitch_deg, float roll_deg, FicMmc5983maData_t *state_out);
bool fic_mmc5983ma_read_calibrated(float pitch_deg, float roll_deg, FicMmc5983maData_t *state_out);
void fic_mmc5983ma_get_state(FicMmc5983maData_t *state_out);

#ifdef __cplusplus
}
#endif

#endif // FIC_MMC5983MA_H
