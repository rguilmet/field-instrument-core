# Changelog

## [v0.2.0] - 2026-11-24
### Added
- Added port-parameterized `fic_i2c_write_read_dev_port()` in `src/bus/` for I2C0/I2C1 touch portability.
- Added `src/display/`:
  - `fic_axs15231b.{h,c}` (AXS15231B QSPI LCD vendor driver)
  - `fic_lvgl_port.{h,c}` (board-parameterized QSPI/DMA/PSRAM LVGL port with `heap_caps_malloc_extmem_enable(64)` -> `4096` UI scoping, lock-free framebuffer capture handshake, sentinel touch filtering, and `250 ms` touch fault backoff)
  - `fic_screenshot.{h,cpp}` (lock-free top-down 24-bit BMP writer with `2`-row internal DMA chunk buffer, primary -> fallback FS failover, and post-close byte-size verification)
- Added `src/audio/`:
  - Vendored `esp_codec_dev/` wired to `fic_i2c_lock()`, `fic_i2c_report_status()`, and `fic_i2c_register_bus_hooks()`
  - Vendored `codec_board/` supporting both `S3_LCD_3_5B` and `S3_LCD_3_49`
  - `fic_audio_beeper.{h,cpp}` (Priority-4 Core 1 ES8311 tone synthesizer & queue with dynamic internal DMA PCM buffer)
- Expanded `examples/CoreSmoke_35B/CoreSmoke_35B.ino` to compile and link display, LVGL, screenshot, codec, and beeper modules.

## [v0.1.0] - 2026-11-24
### Added
- Initialized `Field_Instrument_Core` Arduino library (`MIT` license, Apache-2.0 preserved on vendored `src/io/`).
- Added declarative board profile types (`fic_board_types.h`) and presets (`waveshare_s3_35b.h`, `waveshare_s3_349_v3.h`).
- Added parameterized dual-bus I2C BSP (`fic_i2c_bsp.{h,c}`, `fic_i2c_bus1.c`, `fic_i2c_rw.c`, `fic_i2c_internal.h`) with 18-clock GPIO bus-clear, sentinel run filters, two-tier recovery, and detach/reattach hooks for both I2C0 and I2C1.
- Added TCA9554 IO expander (`esp_io_expander.{h,c}`, `esp_io_expander_tca9554.{h,c}`) integrated with `fic_i2c` bus hooks.
- Added `fic_matrix_math.h` and instance-safe `fic_ahrs_9dof.{h,cpp}`.
- Added `fic_qmi8658.{h,c}`, `fic_sen0696.{h,cpp}`, `fic_mmc5983ma.{h,cpp}`, `fic_pcf85063.{h,cpp}`, and `fic_axp2101.{h,c}` with injectable log callbacks and zero consumer globals.
- Added `Field_Instrument_Core.h`, `examples/CoreSmoke_35B/CoreSmoke_35B.ino`, and `scripts/check_secrets.py`.
