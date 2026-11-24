#ifndef FIC_AUDIO_BEEPER_H
#define FIC_AUDIO_BEEPER_H

#include <stdbool.h>
#include <stdint.h>
#include "../boards/fic_board_types.h"
#include "../io/esp_io_expander.h"
#include "codec_board/codec_board.h"
#include "codec_board/codec_init.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char              *board_type;
    esp_io_expander_handle_t io_expander;
    uint32_t                 exio_pa_ctrl_mask;
    uint32_t                 sample_rate_hz;
    uint8_t                  initial_volume;
    bool                     initial_muted;
    codec_i2s_mode_t         in_mode;
    codec_i2s_mode_t         out_mode;
    bool                     in_use_tdm;
    bool                     reuse_dev;
    uint8_t                  task_priority;
    uint32_t                 task_stack_size;
    fic_log_cb_t             log_cb;
} FicAudioBeeperCfg_t;

void fic_audio_beeper_default_cfg(FicAudioBeeperCfg_t *out,
                                  const char *board_type,
                                  esp_io_expander_handle_t io_expander,
                                  uint32_t exio_pa_ctrl_mask);
bool fic_audio_beeper_init(const FicAudioBeeperCfg_t *cfg);
bool fic_audio_beeper_start_task(void);
void fic_audio_beeper_set_volume(uint8_t volume_pct);
uint8_t fic_audio_beeper_get_volume(void);
void fic_audio_beeper_set_muted(bool muted);
bool fic_audio_beeper_is_muted(void);
void fic_audio_beeper_play(uint16_t freq_hz, uint16_t duration_ms);
void fic_audio_beeper_play_gain(uint16_t freq_hz, uint16_t duration_ms, uint8_t gain_pct);
void fic_audio_beeper_play_dual(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms,
                                uint16_t freq2_hz, uint16_t dur2_ms);
void fic_audio_beeper_play_dual_gain(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms,
                                     uint16_t freq2_hz, uint16_t dur2_ms, uint8_t gain_pct);
esp_codec_dev_handle_t fic_audio_beeper_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif // FIC_AUDIO_BEEPER_H
