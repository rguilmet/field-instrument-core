# Changelog

## [v1.0.0] - 2026-11-24
### Added
- Completed **Phase 1.2C** (`Field_Instrument_Core v1.0.0` baseline):
  - Added `src/storage/`:
    - `fic_ring_buf.h`: Byte ring buffer primitive (pure C/C++, host-testable).
    - `fic_fs_manager.{h,cpp}`: Primary/fallback `fs::FS` registry and `fs_mux` lock helpers.
    - `fic_ring_logger.{h,cpp}`: Multi-instance PSRAM file ring logger with decoupled `buf_mux` / `file_mux` / `fs_mux` and directory/root fallback.
    - `fic_core_log.{h,cpp}`: `32 KB` PSRAM console + file logger with task-scoped `s_io_owner_task` re-entrancy protection and zero `#define Serial` hijack.
    - `fic_settings_json.{h,cpp}` (`F-12`): Atomic JSON persistence (`.tmp` write + read-back length verify + `.bak` + `rename()`) and multi-tier recovery loader.
  - Added `src/net/`:
    - `fic_web_portal.h`, `fic_web_portal_html.{h,cpp}`, `fic_web_portal.cpp`: Parameterized Core 1 Priority-3 Wi-Fi STA/SoftAP file server, NTP RTC sync, sortable file browser, Blob downloads, and BMP->PNG viewer.
  - Added `extras/unit_tests/` (`test_fic_core.cpp`, `run_tests.py`) and `extras/ci_compile_all.ps1`.
  - Updated `Field_Instrument_Core.h`, `library.properties`, and `examples/CoreSmoke_35B/CoreSmoke_35B.ino` to `1.0.0`.

## [v0.2.0] - 2026-11-24
### Added
- Added port-parameterized `fic_i2c_write_read_dev_port()` in `src/bus/` for I2C0/I2C1 touch portability.
- Added `src/display/` (`fic_axs15231b.{h,c}`, `fic_lvgl_port.{h,c}`, `fic_screenshot.{h,cpp}`).
- Added `src/audio/` (`esp_codec_dev/`, `codec_board/`, `fic_audio_beeper.{h,cpp}`).

## [v0.1.0] - 2026-11-24
### Added
- Initialized `Field_Instrument_Core` Arduino library (`MIT` license, Apache-2.0 preserved on vendored `src/io/`).
- Added board profiles (`boards/`), parameterized dual-bus I2C (`bus/`), TCA9554 IO expander (`io/`), math/fusion (`fic_matrix_math.h`, `fic_ahrs_9dof.{h,cpp}`), sensors (`fic_qmi8658`, `fic_sen0696`, `fic_mmc5983ma`), RTC (`fic_pcf85063`), PMIC (`fic_axp2101`), and `scripts/check_secrets.py`.
