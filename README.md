# Field_Instrument_Core

Shared ESP32-S3 BSP, resilient dual-bus I2C recovery, display, audio, storage, web portal, sensor drivers, and 9-DOF AHRS math core for Waveshare ESP32-S3 field instruments.

## Included Modules (v1.1.1)
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
- `storage/`:
  - `fic_ring_buf.h`: Lock-free-friendly byte ring primitive (`drop_oldest` / `drop_newest` modes).
  - `fic_fs_manager.h`: Primary/fallback `fs::FS` registry and `fs_mux` helpers.
  - `fic_ring_logger.h`: Multi-instance PSRAM file ring logger with decoupled `buf_mux` / `file_mux` / `fs_mux`.
  - `fic_core_log.h`: Hardened `32 KB` PSRAM console + SD logger with task-scoped re-entrancy (`s_io_owner_task`) and zero `#define Serial` hijack.
  - `fic_settings_json.h`: Atomic JSON persistence (`.tmp` + read-back verify + `.bak` + `rename()`) and recovery loader.
- `net/`:
  - `fic_web_portal.h`: Core 1 Priority-3 Wi-Fi STA/SoftAP file portal with deferred teardown, NTP RTC sync, sortable file browser, Blob downloads, and BMP->PNG viewer.
- `math/`: `fic_matrix_math.h` (NxN partial-pivot linear solver, 3x3 inverse/Jacobi eigensolver, 3D ellipsoid & Kabsch alignment).
- `fusion/`: `fic_ahrs_9dof.h` (instance-safe 9-DOF Madgwick filter, 8 right-handed orientation permutations, stationary gyro FOC, gated gravity fusion).
- `sensors/`: `fic_qmi8658.h`, `fic_sen0696.h`, `fic_mmc5983ma.h`.
- `rtc/` & `power/`: `fic_pcf85063.h`, `fic_axp2101.h`.

## Standard FreeRTOS & Memory Contract
All consumer instruments built on `Field_Instrument_Core` share a single ESP32-S3 dual-core and memory architecture (`fic_board_types.h`):
- **Core `0` (`FIC_CORE_ACQ`):** Wireless/TCP-IP stacks and real-time sensor/radio acquisition (`FIC_PRIO_UI_FEED = 2`).
- **Core `1` (`FIC_CORE_UI`):** Real-time I2S audio synthesis (`FIC_PRIO_AUDIO = 4`) above `LVGL` rendering (`FIC_PRIO_LVGL = 3`), UI state/power tasks (`FIC_PRIO_UI_FEED = 2`), and background `loop()` Flash/SD writes, debounced settings saves, and HTTP polling (`FIC_PRIO_BACKGROUND = 1`).
- **Heap & Flash Discipline:** Bulk LVGL UI trees route allocations `>= 64 B` to Octal PSRAM (`heap_caps_malloc_extmem_enable`), keeping `>= 150 KB` internal `DRAM0` free for Wi-Fi/BLE/DMA (`<= 25%` static `.bss`); Flash/SD settings writes must never run inside LVGL or UI tasks.

## Verification & Extras
- `extras/unit_tests/`: Host C++17 unit tests & ESP32-S3 syntax gate (`python extras/unit_tests/run_tests.py`).
- `extras/ci_compile_all.ps1`: Dual-compile CI script (`-Target All|Core|Rover|Mfs`) enforcing `0` warnings and static `DRAM0 <= 25%`.
- `scripts/check_secrets.py`: Pre-push secrets & privacy scanner.

## Dependencies
- ESP32 Arduino Core (`esp32:esp32`, ESP-IDF v5 `driver/i2c_master.h`)
- `SensorLib` (`RtcDrv.hpp` / `SensorPCF85063`)
- `lvgl` (`v9.x`)
- `ArduinoJson` (`v7.x`)

## License
MIT License -- see [LICENSE](LICENSE). Upstream Espressif `src/io/`, `src/display/fic_axs15231b.*`, and `src/audio/esp_codec_dev/` files retain Apache-2.0; `src/audio/codec_board/` retains ESPRESSIF MIT.
