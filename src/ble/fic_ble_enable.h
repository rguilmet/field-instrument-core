#ifndef FIC_BLE_ENABLE_H
#define FIC_BLE_ENABLE_H

/**
 * @brief Opt-in header for sketches that use fic_ble_companion.
 *
 * Include this header once in the main sketch translation unit so Arduino-ESP32
 * retains BLE controller memory instead of calling btMemRelease(BT_MODE_BLE)
 * inside initArduino(). Sketches that do not include this header reclaim
 * ~36 KB of BT controller BSS automatically.
 */
#if defined(ARDUINO_ARCH_ESP32) && __has_include("esp32-hal-alloc-ble-mem.h")
#include "esp32-hal-alloc-ble-mem.h"
#endif

#endif // FIC_BLE_ENABLE_H
