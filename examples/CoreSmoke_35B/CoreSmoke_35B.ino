/*
 * CoreSmoke_35B.ino - Standalone compile & hardware smoke test for Field_Instrument_Core v0.1.0
 */

#include <Arduino.h>
#include <Field_Instrument_Core.h>
#include <boards/waveshare_s3_35b.h>
#include <boards/waveshare_s3_349_v3.h>

static FicBoardProfile_t      s_board;
static esp_io_expander_handle_t s_exio = nullptr;
static FicSen0696_t           s_sen0696;
static Ahrs9DofState_t        s_ahrs;

void setup()
{
    Serial.begin(115200);
    s_board = fic_board_waveshare_s3_35b();
    const FicBoardProfile_t alt_board = fic_board_waveshare_s3_349_v3();
    Serial.printf("[SMOKE] Board: %s (Alt: %s) | Core v%s\n",
                  s_board.name, alt_board.name, FIC_CORE_VERSION_STR);

    (void)fic_i2c_init_from_board(&s_board);
    if (fic_i2c0_bus != nullptr) {
        (void)esp_io_expander_new_i2c_tca9554(fic_i2c0_bus, s_board.display.tca9554_addr, &s_exio);
    }

    fic_axp2101_init();
    FicAxp2101Status_t pmic = {};
    fic_axp2101_read(&pmic);

    fic_qmi8658_init();
    FicQmi8658Data_t imu = {};
    fic_qmi8658_read_tilt(&imu);

    (void)fic_rtc_init();
    FicRtcTime_t now = fic_rtc_get_time();
    Serial.printf("[SMOKE] RTC %04u-%02u-%02u %02u:%02u:%02u\n",
                  now.year, now.month, now.day, now.hour, now.minute, now.second);

    (void)fic_mmc5983ma_init_default();
    FicMmc5983maData_t mag = {};
    (void)fic_mmc5983ma_read(imu.pitch_deg, imu.roll_deg, &mag);

    (void)fic_sen0696_init_on_bus1(&s_sen0696, SEN0696_I2C_ADDR_DEFAULT);

    Ahrs9DofConfig_t ahrs_cfg = {};
    ahrs_9dof_default_config(&ahrs_cfg, AHRS_ORIENT_DOWN_0);
    ahrs_9dof_init(&s_ahrs, &ahrs_cfg);

    double m[9][10] = {{2.0, 1.0}, {1.0, 3.0}};
    double b[9]     = {1.0, 2.0};
    double x[9]     = {0.0};
    (void)matrix_solve_nxn(2, m, b, x);
}

void loop()
{
    FicQmi8658Data_t imu = {};
    fic_qmi8658_read_tilt(&imu);

    FicSen0696State_t beam = {};
    if (fic_sen0696_is_online(&s_sen0696)) {
        (void)fic_sen0696_poll_into(&s_sen0696, &beam);
    }

    Ahrs9DofOutput_t ahrs_out = {};
    ahrs_9dof_update(&s_ahrs, imu.acc_g, imu.gyro_dps, beam.mag_ut, beam.mag_valid, true, 0.05f, &ahrs_out);
    delay(50);
}
