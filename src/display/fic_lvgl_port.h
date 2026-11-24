#ifndef FIC_LVGL_PORT_H
#define FIC_LVGL_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <lvgl.h>
#include "driver/i2c_master.h"
#include "../boards/fic_board_types.h"
#include "../io/esp_io_expander_tca9554.h"
#include "fic_axs15231b.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*fic_lvgl_void_cb_t)(void);
typedef bool (*fic_lvgl_bool_cb_t)(void);

typedef struct {
    uint16_t                        h_res;
    uint16_t                        v_res;
    size_t                          dma_buf_bytes;
    size_t                          spiram_buf_bytes;
    int                             spi_host;
    int                             pin_cs;
    int                             pin_pclk;
    int                             pin_data0;
    int                             pin_data1;
    int                             pin_data2;
    int                             pin_data3;
    int                             pin_bk_light;
    uint32_t                        pclk_hz;
    uint8_t                         tca9554_addr;
    uint32_t                        exio_input_mask;
    uint32_t                        exio_output_mask;
    uint32_t                        exio_lcd_rst_mask;
    uint32_t                        exio_cam_pwdn_mask;
    uint32_t                        exio_bl_en_mask;
    uint8_t                         touch_i2c_port;
    i2c_master_dev_handle_t         touch_dev_handle;
    const axs15231b_lcd_init_cmd_t *init_cmds;
    uint16_t                        init_cmds_size;
    uint32_t                        tick_period_ms;
    uint32_t                        task_max_delay_ms;
    uint32_t                        task_min_delay_ms;
    uint32_t                        task_stack_size;
    uint8_t                         task_priority;
    fic_lvgl_void_cb_t              init_backlight_cb;
    fic_lvgl_void_cb_t              create_ui_cb;
    fic_lvgl_bool_cb_t              touch_log_enabled_cb;
    fic_log_cb_t                    log_printf_cb;
} FicLvglDisplayCfg_t;

void fic_lvgl_cfg_from_board(const FicBoardProfile_t *board, FicLvglDisplayCfg_t *out);
void fic_lvgl_port_init(const FicLvglDisplayCfg_t *cfg);
bool fic_lvgl_lock(int timeout_ms);
void fic_lvgl_unlock(void);
bool fic_lvgl_capture_framebuffer(uint8_t *dst_rgb565_be, size_t dst_bytes);
void fic_lvgl_get_resolution(uint16_t *h_res_out, uint16_t *v_res_out);
esp_io_expander_handle_t fic_lvgl_get_io_expander(void);
uint32_t fic_lvgl_get_last_user_activity_ms(void);
void fic_lvgl_note_user_activity(void);

#ifdef __cplusplus
}
#endif

#endif // FIC_LVGL_PORT_H
