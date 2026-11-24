#ifndef FIELD_INSTRUMENT_CORE_H
#define FIELD_INSTRUMENT_CORE_H

#define FIC_CORE_VERSION_MAJOR 0
#define FIC_CORE_VERSION_MINOR 1
#define FIC_CORE_VERSION_PATCH 0
#define FIC_CORE_VERSION_STR   "0.1.0"

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

#endif // FIELD_INSTRUMENT_CORE_H
