/**
 * @file ws_audio.h
 * @brief ES8311 codec + I2S microphone driver for Waveshare ESP32-S3 1.8" AMOLED.
 *
 * Captures 16-bit mono PCM from the on-board SMD microphone via the ES8311
 * audio codec and converts it into NUM_BARS amplitude bins suitable for
 * driving the music visualizer.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise the ES8311 codec and I2S peripheral for microphone capture.
 * Powers up BLDO2 on the AXP2101 PMIC and enables the PA GPIO (GPIO 46).
 * Safe to call multiple times (idempotent).
 *
 * @note The shared I2C bus (I2C_NUM_0, SDA=15, SCL=14) must already be
 *       initialised by display_manager before calling this function.
 */
esp_err_t ws_audio_init(void);

/**
 * De-initialise the codec, I2S, and release resources.
 */
void ws_audio_deinit(void);

/**
 * Start the background mic-capture task.  Audio data is analysed and
 * pushed to the music visualizer amplitude queue automatically.
 */
void ws_audio_start(void);

/**
 * Stop the background mic-capture task.
 */
void ws_audio_stop(void);

/**
 * @return true if the mic-capture task is currently running.
 */
bool ws_audio_is_running(void);

#ifdef __cplusplus
}
#endif
