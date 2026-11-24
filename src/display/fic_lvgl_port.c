#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "fic_lvgl_port.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "../bus/fic_i2c_bsp.h"

static const char *TAG = "fic_lvgl_port";
static SemaphoreHandle_t lvgl_mux = NULL;
static uint16_t *trans_buf_1 = NULL;
static uint16_t *s_last_flushed_fb = NULL;
static uint8_t * volatile s_fb_capture_dst = NULL;
static SemaphoreHandle_t s_fb_capture_done_sem = NULL;
static SemaphoreHandle_t flush_done_semaphore = NULL;
static esp_io_expander_handle_t s_io_expander = NULL;

static FicLvglDisplayCfg_t s_disp_cfg = {0};
static size_t s_fb_bytes = 0;

#define LCD_BIT_PER_PIXEL 16
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))

static const axs15231b_lcd_init_cmd_t s_default_lcd_init_cmds[] = {
    {0x11, (const uint8_t[]){0x00}, 0, 100},
    {0x29, (const uint8_t[]){0x00}, 0, 100},
};

void fic_lvgl_cfg_from_board(const FicBoardProfile_t *board, FicLvglDisplayCfg_t *out)
{
    if (!board || !out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->h_res                   = board->display.h_res;
    out->v_res                   = board->display.v_res;
    uint16_t dma_rows            = 24U;
    if (board->display.v_res > 0U) {
        while (dma_rows > 1U && (board->display.v_res % dma_rows) != 0U) {
            dma_rows--;
        }
    }
    out->dma_buf_bytes           = (size_t)board->display.h_res * (size_t)dma_rows * 2U;
    out->spiram_buf_bytes        = (size_t)board->display.h_res * (size_t)board->display.v_res * 2U;
    out->spi_host                = board->display.spi_host;
    out->pin_cs                  = board->display.pin_cs;
    out->pin_pclk                = board->display.pin_pclk;
    out->pin_data0               = board->display.pin_data0;
    out->pin_data1               = board->display.pin_data1;
    out->pin_data2               = board->display.pin_data2;
    out->pin_data3               = board->display.pin_data3;
    out->pin_bk_light            = board->display.pin_bk_light;
    out->pclk_hz                 = board->display.pclk_hz ? board->display.pclk_hz : 40000000U;
    out->tca9554_addr            = board->display.tca9554_addr;
    out->exio_input_mask         = board->display.exio_input_mask;
    out->exio_output_mask        = board->display.exio_output_mask;
    out->exio_lcd_rst_mask       = board->display.exio_lcd_rst_mask;
    out->exio_cam_pwdn_mask      = board->display.exio_cam_pwdn_mask;
    out->exio_bl_en_mask         = board->display.exio_bl_en_mask;
    out->exio_sys_en_mask        = board->display.exio_sys_en_mask;
    out->lcd_rst_low_ms          = board->display.lcd_rst_low_ms ? board->display.lcd_rst_low_ms : 150U;
    out->touch_swap_xy           = board->display.touch_swap_xy;
    out->touch_invert_y          = board->display.touch_invert_y;
    out->touch_ignore_event_flag = board->display.touch_ignore_event_flag;
    out->touch_i2c_port          = (board->i2c0.addr_touch != 0U) ? 0U : 1U;
    out->tick_period_ms          = 5U;
    out->task_max_delay_ms       = 20U;
    out->task_min_delay_ms       = 5U;
    out->task_stack_size         = 10U * 1024U;
    out->task_priority           = 3U;
}

static bool fic_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                        esp_lcd_panel_io_event_data_t *edata,
                                        void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    BaseType_t TaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(flush_done_semaphore, &TaskWoken);
    return (TaskWoken == pdTRUE);
}

static void fic_increase_lvgl_tick(void *arg)
{
    (void)arg;
    lv_tick_inc(s_disp_cfg.tick_period_ms);
}

static void fic_lvgl_service_fb_capture(const uint16_t *src_fb)
{
    uint8_t *dst = s_fb_capture_dst;
    if (dst != NULL && src_fb != NULL && s_fb_bytes > 0) {
        memcpy(dst, src_fb, s_fb_bytes);
        s_fb_capture_dst = NULL;
        if (s_fb_capture_done_sem != NULL) {
            xSemaphoreGive(s_fb_capture_done_sem);
        }
    }
}

static void fic_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    lv_draw_sw_rgb565_swap(color_p, lv_area_get_width(area) * lv_area_get_height(area));

    const int flush_count = (s_disp_cfg.dma_buf_bytes > 0)
                                ? (int)(s_disp_cfg.spiram_buf_bytes / s_disp_cfg.dma_buf_bytes)
                                : 1;
    const int offgap      = (flush_count > 0) ? ((int)s_disp_cfg.v_res / flush_count) : (int)s_disp_cfg.v_res;
    const int dmalen      = (int)(s_disp_cfg.dma_buf_bytes / 2U);
    int offsetx1 = 0;
    int offsety1 = 0;
    int offsetx2 = (int)s_disp_cfg.h_res;
    int offsety2 = offgap;

    uint16_t *map = (uint16_t *)color_p;
    xSemaphoreGive(flush_done_semaphore);
    for (int i = 0; i < flush_count; i++) {
        xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
        memcpy(trans_buf_1, map, s_disp_cfg.dma_buf_bytes);
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2, offsety2, trans_buf_1);
        offsety1 += offgap;
        offsety2 += offgap;
        map += dmalen;
    }
    xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
    if (area->x1 == 0 && area->y1 == 0 &&
        lv_area_get_width(area) == (int32_t)s_disp_cfg.h_res &&
        lv_area_get_height(area) == (int32_t)s_disp_cfg.v_res) {
        s_last_flushed_fb = (uint16_t *)color_p;
        fic_lvgl_service_fb_capture(s_last_flushed_fb);
    }
    lv_disp_flush_ready(disp);
}

static volatile uint32_t s_last_user_activity_ms = 0;

uint32_t fic_lvgl_get_last_user_activity_ms(void)
{
    return s_last_user_activity_ms;
}

void fic_lvgl_note_user_activity(void)
{
    s_last_user_activity_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void TouchInputReadCallback(lv_indev_t *indev, lv_indev_data_t *indevData)
{
    (void)indev;
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    static lv_indev_state_t prev_state = LV_INDEV_STATE_RELEASED;
    static uint32_t last_log_ms = 0;
    static uint8_t  err_streak = 0;
    static uint32_t backoff_until_ms = 0;

    i2c_master_dev_handle_t touch_dev = s_disp_cfg.touch_dev_handle;
    if (touch_dev == NULL) {
        touch_dev = fic_touch_dev;
    }
    if (touch_dev == NULL) {
        indevData->point.x = last_x;
        indevData->point.y = last_y;
        indevData->state   = LV_INDEV_STATE_RELEASED;
        return;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (err_streak >= 5 && (int32_t)(now_ms - backoff_until_ms) < 0) {
        indevData->point.x = last_x;
        indevData->point.y = last_y;
        indevData->state   = LV_INDEV_STATE_RELEASED;
        return;
    }

    // Command byte 7 = 0x0E (14 bytes): matches Waveshare AXS15231B capacitive touch protocol
    uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00};
    uint8_t buff[14] = {0};

    bool touch_log_enabled = (s_disp_cfg.touch_log_enabled_cb != NULL) && s_disp_cfg.touch_log_enabled_cb();
    bool corrupt_sentinel = false;
    esp_err_t err = fic_i2c_write_read_dev_port(s_disp_cfg.touch_i2c_port, touch_dev, read_touchpad_cmd, 11, buff, 14);
    if (err == ESP_OK) {
        err_streak = 0;
        // Reject corrupt 0x61 / 0x71 repeated-sentinel I2C glitches in addition to 0xFF invalid frames
        corrupt_sentinel = fic_i2c_has_sentinel_run(buff, 6U, 2U);
        uint8_t event_flag = (buff[2] >> 6) & 0x03;
        uint8_t max_pts = s_disp_cfg.touch_ignore_event_flag ? 4U : 2U;
        bool event_ok = s_disp_cfg.touch_ignore_event_flag || (event_flag != 0x01U);
        if (!corrupt_sentinel && buff[0] != 0xFF && buff[1] >= 1 && buff[1] <= max_pts && event_ok) {
            uint16_t raw_x = (((uint16_t)buff[2] & 0x0F) << 8) | (uint16_t)buff[3];
            uint16_t raw_y = (((uint16_t)buff[4] & 0x0F) << 8) | (uint16_t)buff[5];
            uint16_t pointX;
            uint16_t pointY;
            if (s_disp_cfg.touch_swap_xy) {
                if (raw_x > s_disp_cfg.v_res) raw_x = s_disp_cfg.v_res;
                if (raw_y > s_disp_cfg.h_res) raw_y = s_disp_cfg.h_res;
                pointX = raw_y;
                pointY = s_disp_cfg.touch_invert_y ? (uint16_t)(s_disp_cfg.v_res - raw_x) : raw_x;
            } else {
                pointX = raw_x;
                pointY = s_disp_cfg.touch_invert_y
                             ? (raw_y > s_disp_cfg.v_res ? 0U : (uint16_t)(s_disp_cfg.v_res - raw_y))
                             : raw_y;
            }
            if (pointX >= s_disp_cfg.h_res) pointX = s_disp_cfg.h_res - 1U;
            if (pointY >= s_disp_cfg.v_res) pointY = s_disp_cfg.v_res - 1U;
            last_x = (int16_t)pointX;
            last_y = (int16_t)pointY;
            indevData->point.x = last_x;
            indevData->point.y = last_y;
            indevData->state   = LV_INDEV_STATE_PRESSED;
            s_last_user_activity_ms = now_ms;

            if (touch_log_enabled && s_disp_cfg.log_printf_cb != NULL) {
                if (prev_state != LV_INDEV_STATE_PRESSED) {
                    s_disp_cfg.log_printf_cb("[TOUCH DOWN] (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
                                             last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
                                             event_flag, buff[1]);
                    last_log_ms = now_ms;
                } else if (now_ms - last_log_ms >= 250) {
                    s_disp_cfg.log_printf_cb("[TOUCH HELD] (%d, %d) | buf: %02X %02X %02X %02X | pts:%u\n",
                                             last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[1]);
                    last_log_ms = now_ms;
                }
            }
            prev_state = LV_INDEV_STATE_PRESSED;
            return;
        } else if (!corrupt_sentinel && buff[1] >= 1 && buff[0] == 0xFF && touch_log_enabled && s_disp_cfg.log_printf_cb != NULL) {
            s_disp_cfg.log_printf_cb("[TOUCH REJECTED: buff[0]==0xFF] pts=%u evt=%u\n", buff[1], event_flag);
        }
    } else {
        if (err_streak < 255) err_streak++;
        if (err_streak >= 5) {
            // Back off 250 ms during I2C fault recovery so LVGL task does not thrash locks
            backoff_until_ms = (uint32_t)(esp_timer_get_time() / 1000ULL) + 250U;
        }
    }

    // Always preserve last_x / last_y on release so LVGL 9 recognizes swipes and clicks
    indevData->point.x = last_x;
    indevData->point.y = last_y;
    indevData->state   = LV_INDEV_STATE_RELEASED;

    if (touch_log_enabled && s_disp_cfg.log_printf_cb != NULL &&
        prev_state == LV_INDEV_STATE_PRESSED && err == ESP_OK && !corrupt_sentinel) {
        uint8_t event_flag = (buff[2] >> 6) & 0x03;
        s_disp_cfg.log_printf_cb("[TOUCH UP]   (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
                                 last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
                                 event_flag, buff[1]);
    }
    prev_state = LV_INDEV_STATE_RELEASED;
}

bool fic_lvgl_lock(int timeout_ms)
{
    if (!lvgl_mux) return false;
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void fic_lvgl_unlock(void)
{
    if (lvgl_mux) {
        xSemaphoreGive(lvgl_mux);
    }
}

bool fic_lvgl_capture_framebuffer(uint8_t *dst_rgb565_be, size_t dst_bytes)
{
    if (!dst_rgb565_be || s_fb_bytes == 0 || dst_bytes < s_fb_bytes || s_fb_capture_done_sem == NULL) {
        return false;
    }
    // Clear any stale completion token before arming the one-shot copy
    (void)xSemaphoreTake(s_fb_capture_done_sem, 0);
    s_fb_capture_dst = dst_rgb565_be;
    if (xSemaphoreTake(s_fb_capture_done_sem, pdMS_TO_TICKS(2000)) == pdTRUE) {
        return true;
    }
    // Timeout fallback: disarm request cleanly before caller frees dst_rgb565_be
    s_fb_capture_dst = NULL;
    if (xSemaphoreTake(s_fb_capture_done_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        return true;
    }
    return false;
}

void fic_lvgl_get_resolution(uint16_t *h_res_out, uint16_t *v_res_out)
{
    if (h_res_out) *h_res_out = s_disp_cfg.h_res;
    if (v_res_out) *v_res_out = s_disp_cfg.v_res;
}

void fic_lvgl_set_io_expander(esp_io_expander_handle_t handle)
{
    s_io_expander = handle;
}

esp_io_expander_handle_t fic_lvgl_get_io_expander(void)
{
    return s_io_expander;
}

static void fic_lvgl_port_task(void *arg)
{
    (void)arg;
    uint32_t task_delay_ms = s_disp_cfg.task_max_delay_ms;
    for (;;) {
        if (fic_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            fic_lvgl_service_fb_capture(s_last_flushed_fb);
            fic_lvgl_unlock();
        }
        if (task_delay_ms > s_disp_cfg.task_max_delay_ms) {
            task_delay_ms = s_disp_cfg.task_max_delay_ms;
        } else if (task_delay_ms < s_disp_cfg.task_min_delay_ms) {
            task_delay_ms = s_disp_cfg.task_min_delay_ms;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static void fic_lcd_pwm_off_early(void)
{
    if (s_disp_cfg.pin_bk_light < 0) {
        return;
    }
    gpio_config_t gpio_conf = {0};
    gpio_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_conf.mode         = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = ((uint64_t)1 << s_disp_cfg.pin_bk_light);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)s_disp_cfg.pin_bk_light, 0));
}

static void fic_lcd_exio_init(void)
{
    if (s_io_expander == NULL && s_disp_cfg.tca9554_addr != 0 && fic_i2c0_bus != NULL) {
        (void)esp_io_expander_new_i2c_tca9554(fic_i2c0_bus, s_disp_cfg.tca9554_addr, &s_io_expander);
    }
    if (s_io_expander == NULL) {
        return;
    }

    // Leave EXIO3 (SD_CS / DAT3) untouched here so SD_MMC / SPI setup remains undisturbed
    esp_io_expander_set_dir(s_io_expander, s_disp_cfg.exio_input_mask, IO_EXPANDER_INPUT);
    esp_io_expander_set_dir(s_io_expander, s_disp_cfg.exio_output_mask, IO_EXPANDER_OUTPUT);
    if (s_disp_cfg.exio_sys_en_mask != 0) {
        esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_sys_en_mask, 1);
    }
    if (s_disp_cfg.exio_cam_pwdn_mask != 0) {
        esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_cam_pwdn_mask, 1);
    }
    if (s_disp_cfg.exio_lcd_rst_mask != 0) {
        esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_lcd_rst_mask, 1);
    }
    if (s_disp_cfg.exio_bl_en_mask != 0) {
        esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_bl_en_mask, 0);
    }
}

static void fic_lcd_reset(void)
{
    if (s_io_expander == NULL || s_disp_cfg.exio_lcd_rst_mask == 0) {
        return;
    }
    uint16_t low_ms = s_disp_cfg.lcd_rst_low_ms ? s_disp_cfg.lcd_rst_low_ms : 150U;
    esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_lcd_rst_mask, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_lcd_rst_mask, 0);
    vTaskDelay(pdMS_TO_TICKS(low_ms));
    esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_lcd_rst_mask, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
}

void fic_lvgl_port_init(const FicLvglDisplayCfg_t *cfg)
{
    if (cfg == NULL || cfg->h_res == 0 || cfg->v_res == 0 || cfg->dma_buf_bytes == 0) {
        ESP_LOGE(TAG, "Invalid FicLvglDisplayCfg_t passed to fic_lvgl_port_init");
        return;
    }

    s_disp_cfg = *cfg;
    s_fb_bytes = (size_t)s_disp_cfg.h_res * (size_t)s_disp_cfg.v_res * BYTES_PER_PIXEL;
    if (s_disp_cfg.spiram_buf_bytes == 0)  s_disp_cfg.spiram_buf_bytes  = s_fb_bytes;
    if (s_disp_cfg.pclk_hz == 0)           s_disp_cfg.pclk_hz           = 40000000U;
    if (s_disp_cfg.tick_period_ms == 0)    s_disp_cfg.tick_period_ms    = 5U;
    if (s_disp_cfg.task_max_delay_ms == 0) s_disp_cfg.task_max_delay_ms = 20U;
    if (s_disp_cfg.task_min_delay_ms == 0) s_disp_cfg.task_min_delay_ms = 5U;
    if (s_disp_cfg.task_stack_size == 0)   s_disp_cfg.task_stack_size   = 10U * 1024U;
    if (s_disp_cfg.task_priority == 0)     s_disp_cfg.task_priority     = 3U;

    flush_done_semaphore = xSemaphoreCreateBinary();
    assert(flush_done_semaphore);
    s_fb_capture_done_sem = xSemaphoreCreateBinary();
    assert(s_fb_capture_done_sem);

    ESP_LOGI(TAG, "Initialize LCD reset and backlight");
    fic_lcd_pwm_off_early();
    fic_lcd_exio_init();

    ESP_LOGI(TAG, "Initialize QSPI bus");
    spi_bus_config_t buscfg = {0};
    buscfg.data0_io_num    = s_disp_cfg.pin_data0;
    buscfg.data1_io_num    = s_disp_cfg.pin_data1;
    buscfg.sclk_io_num     = s_disp_cfg.pin_pclk;
    buscfg.data2_io_num    = s_disp_cfg.pin_data2;
    buscfg.data3_io_num    = s_disp_cfg.pin_data3;
    buscfg.max_transfer_sz = (int)s_disp_cfg.dma_buf_bytes;
    ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)s_disp_cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_panel_handle_t panel = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {0};
    io_config.cs_gpio_num         = s_disp_cfg.pin_cs;
    io_config.dc_gpio_num         = -1;
    io_config.spi_mode            = 3;
    io_config.pclk_hz             = s_disp_cfg.pclk_hz;
    io_config.trans_queue_depth   = 10;
    io_config.on_color_trans_done = fic_notify_lvgl_flush_ready;
    io_config.lcd_cmd_bits        = 32;
    io_config.lcd_param_bits      = 8;
    io_config.flags.quad_mode     = true;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)s_disp_cfg.spi_host, &io_config, &panel_io));

    axs15231b_vendor_config_t vendor_config = {0};
    vendor_config.flags.use_qspi_interface = 1;
    vendor_config.init_cmds      = s_disp_cfg.init_cmds ? s_disp_cfg.init_cmds : s_default_lcd_init_cmds;
    vendor_config.init_cmds_size = s_disp_cfg.init_cmds
                                       ? s_disp_cfg.init_cmds_size
                                       : (uint16_t)(sizeof(s_default_lcd_init_cmds) / sizeof(s_default_lcd_init_cmds[0]));

    esp_lcd_panel_dev_config_t panel_config = {0};
    panel_config.reset_gpio_num = -1;
    panel_config.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = LCD_BIT_PER_PIXEL;
    panel_config.vendor_config  = &vendor_config;

    ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(panel_io, &panel_config, &panel));

    fic_lcd_reset();
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    if (s_io_expander != NULL && s_disp_cfg.exio_bl_en_mask != 0) {
        esp_io_expander_set_level(s_io_expander, s_disp_cfg.exio_bl_en_mask, 1);
    }
    if (s_disp_cfg.init_backlight_cb != NULL) {
        s_disp_cfg.init_backlight_cb();
    }

    lv_init();

    lv_display_t *disp = lv_display_create(s_disp_cfg.h_res, s_disp_cfg.v_res);
    lv_display_set_flush_cb(disp, fic_lvgl_flush_cb);

    uint8_t *buffer_1 = (uint8_t *)heap_caps_malloc(s_fb_bytes, MALLOC_CAP_SPIRAM);
    uint8_t *buffer_2 = (uint8_t *)heap_caps_malloc(s_fb_bytes, MALLOC_CAP_SPIRAM);
    assert(buffer_1);
    assert(buffer_2);
    trans_buf_1 = (uint16_t *)heap_caps_malloc(s_disp_cfg.dma_buf_bytes, MALLOC_CAP_DMA);
    assert(trans_buf_1);

    lv_display_set_buffers(disp, buffer_1, buffer_2, s_fb_bytes, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_user_data(disp, panel);

    lv_indev_t *touch_indev = lv_indev_create();
    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch_indev, TouchInputReadCallback);

    esp_timer_create_args_t lvgl_tick_timer_args = {0};
    lvgl_tick_timer_args.callback = &fic_increase_lvgl_tick;
    lvgl_tick_timer_args.name     = "lvgl_tick";
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, s_disp_cfg.tick_period_ms * 1000U));

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    if (fic_lvgl_lock(-1)) {
        // Route LVGL widget/style/table allocations (>=64B) into Octal PSRAM so internal
        // 8-bit/DMA SRAM remains completely free & unfragmented for Wi-Fi/BLE stacks
        heap_caps_malloc_extmem_enable(64);
        if (s_disp_cfg.create_ui_cb != NULL) {
            s_disp_cfg.create_ui_cb();
        }
        heap_caps_malloc_extmem_enable(4096);
        fic_lvgl_unlock();
    }

    xTaskCreatePinnedToCore(fic_lvgl_port_task, "LVGL", s_disp_cfg.task_stack_size, NULL, s_disp_cfg.task_priority, NULL, 1);
}
