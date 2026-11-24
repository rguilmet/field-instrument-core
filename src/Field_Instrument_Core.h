#ifndef FIELD_INSTRUMENT_CORE_H
#define FIELD_INSTRUMENT_CORE_H

#define FIC_CORE_VERSION_MAJOR 0
#define FIC_CORE_VERSION_MINOR 2
#define FIC_CORE_VERSION_PATCH 0
#define FIC_CORE_VERSION_STR   "0.2.0"

#include "boards/fic_board_types.h"
#include "bus/fic_i2c_bsp.h"
#include "io/esp_io_expander.h"
#include "io/esp_io_expander_tca9554.h"
#include "math/fic_matrix_math.h"
#include "fusion/fic_ahrs_9dof.h"
#include "sensors/fic_qmi8658.h"
#include "sensors/fic_sen0696.h"
#include "sensors/fic_mmc5983ma.h"
#include "rtc/fic_pcf85063.h"
#include "power/fic_axp2101.h"
#include "display/fic_axs15231b.h"
#include "display/fic_lvgl_port.h"
#include "display/fic_screenshot.h"
#include "audio/codec_board/codec_board.h"
#include "audio/codec_board/codec_init.h"
#include "audio/fic_audio_beeper.h"

#endif // FIELD_INSTRUMENT_CORE_H
