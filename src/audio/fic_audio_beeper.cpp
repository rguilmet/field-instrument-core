#include "fic_audio_beeper.h"
#include "../bus/fic_i2c_bsp.h"

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    uint16_t freq1_hz;
    uint16_t dur1_ms;
    uint16_t gap_ms;
    uint16_t freq2_hz;
    uint16_t dur2_ms;
    uint8_t  gain_pct;
} FicBeepItem_t;

static QueueHandle_t          s_beep_queue    = NULL;
static esp_codec_dev_handle_t s_play_handle   = NULL;
static TaskHandle_t           s_audio_task    = NULL;
static uint32_t               s_sample_rate   = 24000U;
static volatile uint8_t       s_volume_pct    = 60U;
static volatile bool          s_muted         = false;
static uint8_t                s_task_prio     = 4U;
static uint32_t               s_task_stack    = 3072U;
static fic_log_cb_t           s_audio_log_cb  = NULL;

extern "C" void fic_audio_beeper_default_cfg(FicAudioBeeperCfg_t *out,
                                             const char *board_type,
                                             esp_io_expander_handle_t io_expander,
                                             uint32_t exio_pa_ctrl_mask)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->board_type        = board_type ? board_type : "S3_LCD_3_5B";
    out->io_expander       = io_expander;
    out->exio_pa_ctrl_mask = exio_pa_ctrl_mask;
    out->sample_rate_hz    = 24000U;
    out->initial_volume    = 60U;
    out->initial_muted     = false;
    out->in_mode           = CODEC_I2S_MODE_NONE;
    out->out_mode          = CODEC_I2S_MODE_STD;
    out->in_use_tdm        = false;
    out->reuse_dev         = false;
    out->task_priority     = 4U;
    out->task_stack_size   = 3072U;
    out->log_cb            = NULL;
}

extern "C" void fic_audio_beeper_set_volume(uint8_t volume_pct)
{
    if (volume_pct > 100U) {
        volume_pct = 100U;
    }
    s_volume_pct = volume_pct;
    s_muted      = (volume_pct == 0U);
    if (s_play_handle != NULL) {
        if (fic_i2c_lock(250)) {
            esp_codec_dev_set_out_vol(s_play_handle, (int)volume_pct);
            fic_i2c_unlock();
        }
    }
}

extern "C" uint8_t fic_audio_beeper_get_volume(void)
{
    return s_volume_pct;
}

extern "C" void fic_audio_beeper_set_muted(bool muted)
{
    s_muted = muted;
}

extern "C" bool fic_audio_beeper_is_muted(void)
{
    return s_muted;
}

extern "C" esp_codec_dev_handle_t fic_audio_beeper_get_handle(void)
{
    return s_play_handle;
}

extern "C" bool fic_audio_beeper_init(const FicAudioBeeperCfg_t *cfg)
{
    FicAudioBeeperCfg_t local_cfg;
    if (cfg != NULL) {
        local_cfg = *cfg;
    } else {
        fic_audio_beeper_default_cfg(&local_cfg, "S3_LCD_3_5B", NULL, 0UL);
    }

    s_audio_log_cb = local_cfg.log_cb;
    s_sample_rate  = local_cfg.sample_rate_hz ? local_cfg.sample_rate_hz : 24000U;
    s_volume_pct   = (local_cfg.initial_volume > 100U) ? 100U : local_cfg.initial_volume;
    s_muted        = local_cfg.initial_muted || (s_volume_pct == 0U);
    s_task_prio    = local_cfg.task_priority ? local_cfg.task_priority : 4U;
    s_task_stack   = local_cfg.task_stack_size ? local_cfg.task_stack_size : 3072U;

    if (local_cfg.io_expander != NULL && local_cfg.exio_pa_ctrl_mask != 0UL) {
        esp_io_expander_set_dir(local_cfg.io_expander, local_cfg.exio_pa_ctrl_mask, IO_EXPANDER_OUTPUT);
        esp_io_expander_set_level(local_cfg.io_expander, local_cfg.exio_pa_ctrl_mask, 1);
    }

    set_codec_board_type(local_cfg.board_type ? local_cfg.board_type : "S3_LCD_3_5B");
    codec_init_cfg_t codec_cfg = {
        .in_mode    = local_cfg.in_mode,
        .out_mode   = local_cfg.out_mode,
        .in_use_tdm = local_cfg.in_use_tdm,
        .reuse_dev  = local_cfg.reuse_dev,
    };

    bool locked = fic_i2c_lock(250);
    int rc = init_codec(&codec_cfg);
    if (rc == 0) {
        s_play_handle = get_playback_handle();
        if (s_play_handle != NULL) {
            esp_codec_dev_set_out_vol(s_play_handle, (int)s_volume_pct);
            esp_codec_dev_sample_info_t fs = {};
            fs.sample_rate     = s_sample_rate;
            fs.channel         = 2;
            fs.bits_per_sample = 16;
            esp_codec_dev_open(s_play_handle, &fs);
        }
    }
    if (locked) {
        fic_i2c_unlock();
    }

    if (s_audio_log_cb != NULL) {
        s_audio_log_cb("[AUDIO] Codec %s (%s @ %lu Hz, vol=%u%%)\n",
                       (s_play_handle != NULL) ? "ONLINE" : "FAIL",
                       local_cfg.board_type ? local_cfg.board_type : "S3_LCD_3_5B",
                       (unsigned long)s_sample_rate,
                       (unsigned)s_volume_pct);
    }
    return (s_play_handle != NULL);
}

extern "C" void fic_audio_beeper_play(uint16_t freq_hz, uint16_t duration_ms)
{
    fic_audio_beeper_play_gain(freq_hz, duration_ms, 100U);
}

extern "C" void fic_audio_beeper_play_gain(uint16_t freq_hz, uint16_t duration_ms, uint8_t gain_pct)
{
    if (s_beep_queue != NULL) {
        FicBeepItem_t b = { freq_hz, duration_ms, 0, 0, 0, gain_pct };
        xQueueSend(s_beep_queue, &b, 0);
    }
}

extern "C" void fic_audio_beeper_play_dual(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms,
                                           uint16_t freq2_hz, uint16_t dur2_ms)
{
    fic_audio_beeper_play_dual_gain(freq1_hz, dur1_ms, gap_ms, freq2_hz, dur2_ms, 100U);
}

extern "C" void fic_audio_beeper_play_dual_gain(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms,
                                                uint16_t freq2_hz, uint16_t dur2_ms, uint8_t gain_pct)
{
    if (s_beep_queue != NULL) {
        FicBeepItem_t b = { freq1_hz, dur1_ms, gap_ms, freq2_hz, dur2_ms, gain_pct };
        xQueueSend(s_beep_queue, &b, 0);
    }
}

static void audio_write_tone_or_silence(int16_t *pcm_buf, uint16_t freq_hz, uint16_t dur_ms, uint8_t gain_pct)
{
    if (!s_play_handle || !pcm_buf || dur_ms == 0U) {
        return;
    }
    uint32_t total_samples = (s_sample_rate * (uint32_t)dur_ms) / 1000U;
    if (total_samples == 0U) {
        return;
    }

    if (gain_pct < 10U)  gain_pct = 10U;
    if (gain_pct > 100U) gain_pct = 100U;
    int16_t amp = (int16_t)((6000L * (long)gain_pct) / 100L);

    float phase = 0.0f;
    float phase_inc = (freq_hz > 0U) ? (2.0f * (float)M_PI * (float)freq_hz / (float)s_sample_rate) : 0.0f;
    uint32_t written = 0U;

    while (written < total_samples) {
        uint32_t rem   = total_samples - written;
        uint32_t chunk = (rem < 256U) ? rem : 256U;
        if (freq_hz == 0U) {
            memset(pcm_buf, 0, chunk * 2U * sizeof(int16_t));
        } else {
            for (uint32_t i = 0; i < chunk; i++) {
                int16_t s = (phase < (float)M_PI) ? amp : -amp;
                pcm_buf[i * 2U]      = s;
                pcm_buf[i * 2U + 1U] = s;
                phase += phase_inc;
                if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
            }
        }
        esp_codec_dev_write(s_play_handle, pcm_buf, chunk * 2U * sizeof(int16_t));
        written += chunk;
    }
}

static void task_audio_beep(void *pvParameters)
{
    (void)pvParameters;
    int16_t *pcm_buf = (int16_t *)heap_caps_malloc(256U * 2U * sizeof(int16_t),
                                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    FicBeepItem_t beep_req;

    for (;;) {
        if (s_beep_queue && xQueueReceive(s_beep_queue, &beep_req, portMAX_DELAY) == pdPASS) {
            if (pcm_buf && s_play_handle && !s_muted && s_volume_pct > 0U) {
                if (beep_req.freq1_hz > 0U && beep_req.dur1_ms > 0U) {
                    audio_write_tone_or_silence(pcm_buf, beep_req.freq1_hz, beep_req.dur1_ms, beep_req.gain_pct);
                }
                if (beep_req.gap_ms > 0U) {
                    audio_write_tone_or_silence(pcm_buf, 0U, beep_req.gap_ms, 100U);
                }
                if (beep_req.freq2_hz > 0U && beep_req.dur2_ms > 0U) {
                    audio_write_tone_or_silence(pcm_buf, beep_req.freq2_hz, beep_req.dur2_ms, beep_req.gain_pct);
                }
                if ((beep_req.freq1_hz > 0U && beep_req.dur1_ms > 0U) ||
                    (beep_req.freq2_hz > 0U && beep_req.dur2_ms > 0U)) {
                    audio_write_tone_or_silence(pcm_buf, 0U, 4U, 100U);
                }
            }
        }
    }
}

extern "C" bool fic_audio_beeper_start_task(void)
{
    if (s_audio_task != NULL) {
        return true;
    }
    if (s_beep_queue == NULL) {
        s_beep_queue = xQueueCreate(12, sizeof(FicBeepItem_t));
    }
    if (s_beep_queue == NULL) {
        return false;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(task_audio_beep, "FicAudio", s_task_stack, NULL, s_task_prio, &s_audio_task, 1);
    return (ok == pdPASS);
}
