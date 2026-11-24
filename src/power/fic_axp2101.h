#ifndef FIC_AXP2101_H
#define FIC_AXP2101_H

#include <stdbool.h>
#include <stdint.h>
#include "../boards/fic_board_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool    online;
    float   battery_voltage_v;
    uint8_t battery_percent;
    bool    is_charging;
    bool    vbus_good;
    float   vbus_voltage_v;
} FicAxp2101Status_t;

void fic_axp2101_set_log_callback(fic_log_cb_t cb);
void fic_axp2101_init(void);
void fic_axp2101_read(FicAxp2101Status_t *status);
void fic_axp2101_power_off(void);

#ifdef __cplusplus
}
#endif

#endif // FIC_AXP2101_H
