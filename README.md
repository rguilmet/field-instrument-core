# Field_Instrument_Core

Shared ESP32-S3 BSP, resilient dual-bus I2C recovery, display, audio, sensor drivers, and 9-DOF AHRS math core for Waveshare ESP32-S3 field instruments.

## Included Modules (v0.2.0)
- `boards/`: Declarative board pin/bus descriptors (`FicBoardProfile_t`) with presets for Waveshare ESP32-S3-Touch-LCD-3.5B (`waveshare_s3_35b.h`) and Waveshare ESP32-S3-Touch-LCD-3.49 v3 (`waveshare_s3_349_v3.h`).
- `bus/`: Parameterized ESP-IDF `i2c_master` BSP (`fic_i2c_bsp.h`) with FreeRTOS recursive locks, 18-clock open-drain GPIO bus clear, `0x61`/`0x71` sentinel detection, two-tier runtime bus recovery, and bus lifecycle hooks across `I2C_NUM_0` and `I2C_NUM_1`.
- `io/`: `esp_io_expander` + `esp_io_expander_tca9554` wired to `fic_i2c_bsp` bus locks and reattach hooks.
- `display/`:
  - `fic_axs15231b.h`: AXS15231B QSPI LCD vendor driver.
  - `fic_lvgl_port.h`: Board-parameterized LVGL port with `heap_caps_malloc_extmem_enable(64)` -> `4096` UI allocation isolation, lock-free framebuffer capture handshake, sentinel touch filtering, and `250 ms` touch fault backoff.
  - `fic_screenshot.h`: Lock-free 24-bit BMP writer using a `2`-row internal DMA chunk buffer (`0 B` static `DRAM0`), primary -> fallback FS failover, and post-close byte-size verification.
- `audio/`:
  - `esp_codec_dev/` & `codec_board/`: Shared ES8311/ES7210 codec framework wired to `fic_i2c_bsp` locks and hooks.
  - `fic_audio_beeper.h`: Priority-4 Core 1 tone synthesizer & queue with dynamic internal DMA PCM buffer.
- `math/`: `fic_matrix_math.h` (NxN partial-pivot linear solver, 3x3 inverse/Jacobi eigensolver, 3D ellipsoid & Kabsch alignment).
- `fusion/`: `fic_ahrs_9dof.h` (instance-safe 9-DOF Madgwick filter, 8 right-handed orientation permutations, stationary gyro FOC, gated gravity fusion).
- `sensors/`:
  - `fic_qmi8658.h`: QMI8658 6-axis IMU + pose/bubble tilt solver.
  - `fic_sen0696.h`: DFRobot SEN0696 10-DOF IMU + magnetometer + barometer (`BMI323 + BMM350 + BMP581`) with instance-safe `FicSen0696_t` contexts and atomic two-phase transactions.
  - `fic_mmc5983ma.h`: MEMSIC MMC5983MA 18-bit AMR magnetometer with SET/RESET bridge-offset cancellation.
- `rtc/`: `fic_pcf85063.h` (PCF85063 RTC with 1 Hz rate-limited polling and callback-injected UTC offset & system-clock sync guard).
- `power/`: `fic_axp2101.h` (AXP2101 PMIC rail bring-up, VBUS-gated telemetry, soft power-off).

## Dependencies
- ESP32 Arduino Core (`esp32:esp32`, ESP-IDF v5 `driver/i2c_master.h`)
- `SensorLib` (`RtcDrv.hpp` / `SensorPCF85063`)
- `lvgl` (`v9.x`, configured via sketch-local `lv_conf.h` or shared fallback)

## License
MIT License -- see [LICENSE](LICENSE). Upstream Espressif `src/io/`, `src/display/fic_axs15231b.*`, and `src/audio/esp_codec_dev/` files retain Apache-2.0; `src/audio/codec_board/` retains ESPRESSIF MIT.
