# Changelog

## [v1.1.1] - 2026-11-24
### Added
- `src/boards/fic_board_types.h` & `README.md`: Exported canonical dual-core FreeRTOS task constants (`FIC_CORE_ACQ = 0`, `FIC_CORE_UI = 1`, `FIC_PRIO_AUDIO = 4`, `FIC_PRIO_LVGL = 3`, `FIC_PRIO_UI_FEED = 2`, `FIC_PRIO_BACKGROUND = 1`) and documented the shared memory/RTOS contract across instruments.

### Changed
- `src/display/fic_lvgl_port.c`: Updated `fic_lvgl_cfg_from_board()` to pick the largest divisor `dma_rows <= 64` of `v_res` keeping `h_res * dma_rows * 2 <= 16384 B` (`24` rows / `20` strips on `320x480`, `40` rows / `16` strips on `172x640`), reducing QSPI flush overhead on narrow panels while keeping internal DMA usage `<= 16 KB`; anchored `fic_lvgl_port_task` to `FIC_PRIO_LVGL` (`3`) on `FIC_CORE_UI` (`1`).

## [v1.1.0] - 2026-11-24
### Added
- `src/boards/fic_board_types.h` & `src/boards/waveshare_s3_349_v3.h`: Added I2C1 `addr_touch`, `exio_sys_en_mask`, `lcd_rst_low_ms`, `touch_swap_xy`, `touch_invert_y`, and `touch_ignore_event_flag` board descriptors for full Waveshare ESP32-S3-Touch-LCD-3.49 v3 support (`200 kHz` I2C0/I2C1, `EXIO6` battery latch, `250 ms` LCD reset, swapped/Y-inverted `172x640` touch).
- `src/bus/fic_i2c_bus1.c`: Attach and reattach `fic_touch_dev` automatically when `cfg->addr_touch != 0`.

### Fixed
- `src/bus/fic_i2c_bsp.c`: Guarded `fic_touch_dev` removal in `fic_i2c_reinit_bus()` on `g_fic_i2c0_cfg.addr_touch != 0` so I2C0 recovery never detaches an I2C1-hosted touch controller.
- `src/display/fic_lvgl_port.{h,c}`:
  - Computed `dma_buf_bytes` rows in `fic_lvgl_cfg_from_board()` as the largest divisor of `v_res` `<= 24` (`20` rows for `640`, `24` rows for `480`) so flush strips cover `100%` of the panel height.
  - Sequenced `exio_sys_en_mask` HIGH immediately and kept `exio_bl_en_mask` LOW during LCD reset/init until `esp_lcd_panel_init()` finishes.
  - Added `fic_touch_dev` fallback across both I2C ports and board-driven X/Y swap, Y-invert, and event-flag relaxation in `TouchInputReadCallback()`.

## [v1.0.1] - 2026-11-24
### Fixed
- `src/boards/waveshare_s3_35b.h`: Set `i2c0.speed_hz` to `400000U` matching Waveshare `01_factory` and added `EXIO2` (`TP_INT`) to `exio_input_mask`; fixed `exio_cam_pwdn_mask` to `(1UL << 0)` (`EXIO0`) so `EXIO7` (`PA_CTRL`) is not treated as camera power-down.
- `src/display/fic_lvgl_port.{h,c}`: Added `fic_lvgl_set_io_expander()` so applications initializing TCA9554 prior to display setup (e.g. MicroSD `EXIO3` probing) can hand off their existing expander handle instead of attaching a second I2C device instance at `0x20`.

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
