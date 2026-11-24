#ifndef FIC_QMI8658_H
#define FIC_QMI8658_H

#include <stdbool.h>
#include <stdint.h>
#include "../boards/fic_board_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FIC_DISPLAY_POSE_FLAT     = 0,
    FIC_DISPLAY_POSE_ANGLED   = 1,
    FIC_DISPLAY_POSE_VERTICAL = 2
} FicDisplayPose_t;

typedef struct {
    float            acc_g[3];
    float            gyro_dps[3];
    int16_t          temp_raw;
    float            pitch_deg;
    float            roll_deg;
    float            bubble_x_deg;
    float            bubble_y_deg;
    FicDisplayPose_t pose;
} FicQmi8658Data_t;

void fic_qmi8658_set_log_callback(fic_log_cb_t cb);
void fic_qmi8658_set_debug_logging(bool enable);
void fic_qmi8658_init(void);
void fic_qmi8658_read(float *acc, float *gyr, int16_t *temp);
void fic_qmi8658_compute_pose_and_bubble(const float *acc,
                                         float tare_pitch_deg,
                                         float tare_roll_deg,
                                         float *pitch_deg_out,
                                         float *roll_deg_out,
                                         float *bubble_x_deg_out,
                                         float *bubble_y_deg_out,
                                         FicDisplayPose_t *pose_out);
void fic_qmi8658_tare_pose(float tare_pitch_deg, float tare_roll_deg);
void fic_qmi8658_clear_tare(void);
void fic_qmi8658_read_tilt(FicQmi8658Data_t *out);

#ifdef __cplusplus
}
#endif

#endif // FIC_QMI8658_H
