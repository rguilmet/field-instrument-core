#ifndef FIC_PCF85063_H
#define FIC_PCF85063_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../boards/fic_board_types.h"

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  week;
} FicRtcTime_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef int16_t (*fic_rtc_utc_offset_cb_t)(void);
typedef bool    (*fic_rtc_allow_sys_clock_sync_cb_t)(void);

void fic_rtc_set_log_callback(fic_log_cb_t cb);
void fic_rtc_set_utc_offset_minutes(int16_t offset_min);
int16_t fic_rtc_get_utc_offset_minutes(void);
void fic_rtc_set_utc_offset_callback(fic_rtc_utc_offset_cb_t provider_cb);
void fic_rtc_set_sys_clock_guard_callback(fic_rtc_allow_sys_clock_sync_cb_t allow_cb);

bool fic_rtc_init(void);
bool fic_rtc_is_online(void);
void fic_rtc_set_from_utc(uint16_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute, uint8_t second,
                          bool sync_sys_clock);
void fic_rtc_set_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
void fic_rtc_sync_from_gnss_utc(uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second);
FicRtcTime_t fic_rtc_get_time(void);
void fic_rtc_get_local_datetime(FicRtcTime_t *local_dt);
void fic_rtc_get_formatted_timestamp(char *buffer, size_t max_len, bool filename_safe);

#ifdef __cplusplus
}
#endif

#endif // FIC_PCF85063_H
