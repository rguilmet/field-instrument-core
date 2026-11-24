# Changelog

## [v0.1.0] - 2026-11-24
### Added
- Initialized `Field_Instrument_Core` Arduino library (`MIT` license, Apache-2.0 preserved on vendored `src/io/`).
- Added declarative board profile types (`fic_board_types.h`) and presets (`waveshare_s3_35b.h`, `waveshare_s3_349_v3.h`).
- Added parameterized dual-bus I2C BSP (`fic_i2c_bsp.{h,c}`, `fic_i2c_bus1.c`, `fic_i2c_internal.h`) with 18-clock GPIO bus-clear, sentinel run filters, two-tier recovery, and detach/reattach hooks for both I2C0 and I2C1.
- Added TCA9554 IO expander (`esp_io_expander.{h,c}`, `esp_io_expander_tca9554.{h,c}`) integrated with `fic_i2c` bus hooks.
- Added `fic_matrix_math.h` and instance-safe `fic_ahrs_9dof.{h,cpp}`.
- Added `fic_qmi8658.{h,c}`, `fic_sen0696.{h,cpp}`, `fic_mmc5983ma.{h,cpp}`, `fic_pcf85063.{h,cpp}`, and `fic_axp2101.{h,c}` with injectable log callbacks and zero consumer globals.
- Added `Field_Instrument_Core.h`, `examples/CoreSmoke_35B/CoreSmoke_35B.ino`, and `scripts/check_secrets.py`.
