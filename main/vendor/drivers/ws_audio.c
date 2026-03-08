/**
 * @file ws_audio.c
 * @brief ES8311 codec + I2S microphone driver for Waveshare ESP32-S3 1.8" AMOLED.
 *
 * Captures 16-bit mono PCM from the on-board SMD microphone via the ES8311
 * audio codec over I2S, runs a lightweight Goertzel spectrum analysis, and
 * pushes NUM_BARS amplitude bins into the music visualizer's amplitudeQueue.
 *
 * Uses the official espressif/es8311 managed component for codec configuration,
 * matching the Waveshare 06_I2SCodec reference example.
 *
 * Hardware wiring (Waveshare ESP32-S3 1.8" AMOLED):
 *   ES8311 I2C  : SDA=15, SCL=14, addr=0x18  (shared I2C_NUM_0 bus)
 *   I2S         : MCLK=16, BCLK=9, WS/LRCK=45, DOUT=8, DIN=10
 *   PA_ENABLE   : GPIO 46 (active-high)
 *   AXP2101 BLDO2 powers the audio subsystem – enabled during init.
 */

#include "vendor/drivers/ws_audio.h"

#ifdef CONFIG_USE_WAVESHARE_AMOLED

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "es8311.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "managers/views/music_visualizer.h"

static const char *TAG = "ws_audio";

/* ─────────── Pin definitions ─────────── */
#define AUDIO_I2C_NUM       I2C_NUM_0
#define ES8311_ADDR         ES8311_ADDRESS_0  /* 0x18 – CE pin low */

#define I2S_PORT            I2S_NUM_0
#define I2S_MCLK_PIN        GPIO_NUM_16
#define I2S_BCLK_PIN        GPIO_NUM_9
#define I2S_WS_PIN          GPIO_NUM_45
#define I2S_DOUT_PIN        GPIO_NUM_8   /* ESP → codec (speaker) */
#define I2S_DIN_PIN         GPIO_NUM_10  /* codec → ESP (microphone) */
#define PA_ENABLE_PIN       GPIO_NUM_46

/* AXP2101 BLDO2 control registers */
#define AXP2101_I2C_ADDR    0x34
#define AXP2101_REG_BLDO2_CTRL  0x97  /* BLDO2 voltage */
#define AXP2101_REG_LDO_EN1     0x90  /* LDO enable control 1 */
#define AXP2101_BLDO2_EN_BIT    (1 << 1)  /* Bit 1 of LDO_EN1 */

/* ─────────── I2S / capture config ─────────── */
#define SAMPLE_RATE         16000
#define MCLK_MULTIPLE       384   /* Waveshare reference uses 384× */
#define MCLK_FREQ_HZ        (SAMPLE_RATE * MCLK_MULTIPLE) /* 6.144 MHz */
#define DMA_BUF_COUNT       4
#define DMA_BUF_LEN         256     /* samples per DMA buffer */
#define CAPTURE_BUF_SAMPLES 512     /* ~32 ms at 16 kHz */
#define MIC_GAIN            ES8311_MIC_GAIN_24DB
#define VOICE_VOLUME        70

/* ─────────── State ─────────── */
static bool               s_initialized   = false;
static bool               s_running       = false;
static TaskHandle_t        s_capture_task  = NULL;
static i2s_chan_handle_t   s_tx_chan       = NULL;
static i2s_chan_handle_t   s_rx_chan       = NULL;
static es8311_handle_t     s_es_handle    = NULL;

/* ─────────── AXP2101 I2C helpers ─────────── */

static esp_err_t axp_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(AUDIO_I2C_NUM, AXP2101_I2C_ADDR,
                                      buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static esp_err_t axp_read_reg(uint8_t reg, uint8_t *val) {
    esp_err_t ret = i2c_master_write_to_device(AUDIO_I2C_NUM, AXP2101_I2C_ADDR,
                                                &reg, 1, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return ret;
    return i2c_master_read_from_device(AUDIO_I2C_NUM, AXP2101_I2C_ADDR,
                                       val, 1, pdMS_TO_TICKS(50));
}

/* ─────────── AXP2101 BLDO2 power control ─────────── */

static esp_err_t audio_power_on(void) {
    /* Set BLDO2 voltage to 3.3 V  (formula: voltage = 0.5 + reg_val * 0.1 V)
     * 3.3 V → (3.3 - 0.5) / 0.1 = 28 = 0x1C */
    esp_err_t ret = axp_write_reg(AXP2101_REG_BLDO2_CTRL, 0x1C);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set BLDO2 voltage: %s", esp_err_to_name(ret));
    }

    /* Enable BLDO2 */
    uint8_t en = 0;
    ret = axp_read_reg(AXP2101_REG_LDO_EN1, &en);
    if (ret == ESP_OK) {
        en |= AXP2101_BLDO2_EN_BIT;
        axp_write_reg(AXP2101_REG_LDO_EN1, en);
    }

    /* Enable PA amplifier */
    gpio_config_t pa_cfg = {
        .pin_bit_mask = (1ULL << PA_ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pa_cfg);
    gpio_set_level(PA_ENABLE_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(30)); /* let rails stabilize */
    ESP_LOGI(TAG, "Audio power on (BLDO2 3.3V, PA enabled)");
    return ESP_OK;
}

static void audio_power_off(void) {
    gpio_set_level(PA_ENABLE_PIN, 0);

    uint8_t en = 0;
    if (axp_read_reg(AXP2101_REG_LDO_EN1, &en) == ESP_OK) {
        en &= ~AXP2101_BLDO2_EN_BIT;
        axp_write_reg(AXP2101_REG_LDO_EN1, en);
    }
    ESP_LOGI(TAG, "Audio power off");
}

/* ─────────── I2S driver init (matches Waveshare 06_I2SCodec) ─────────── */

static esp_err_t i2s_driver_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num  = DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = DMA_BUF_LEN;

    /* Create both TX and RX channels (matching Waveshare reference) */
    esp_err_t ret = i2s_new_channel(&chan_cfg, &s_tx_chan, &s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = MCLK_MULTIPLE;

    ret = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s TX init failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    ret = i2s_channel_init_std_mode(s_rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s RX init failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* Enable both channels so MCLK starts running before ES8311 init */
    ret = i2s_channel_enable(s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s TX enable failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    ret = i2s_channel_enable(s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s RX enable failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    ESP_LOGI(TAG, "I2S initialized (%d Hz, MCLK %d Hz, stereo)", SAMPLE_RATE, MCLK_FREQ_HZ);
    return ESP_OK;

fail:
    if (s_rx_chan) { i2s_del_channel(s_rx_chan); s_rx_chan = NULL; }
    if (s_tx_chan) { i2s_del_channel(s_tx_chan); s_tx_chan = NULL; }
    return ret;
}

/* ─────────── ES8311 codec init using official component ─────────── */

static esp_err_t es8311_codec_init(void) {
    /* Create ES8311 handle (I2C bus must already be installed) */
    s_es_handle = es8311_create(AUDIO_I2C_NUM, ES8311_ADDR);
    if (!s_es_handle) {
        ESP_LOGE(TAG, "es8311_create failed (I2C 0x%02X)", ES8311_ADDR);
        return ESP_FAIL;
    }

    /* Configure clocks — MCLK from I2S MCLK pin at 384× sample rate */
    const es8311_clock_config_t clk_cfg = {
        .mclk_inverted     = false,
        .sclk_inverted     = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency    = MCLK_FREQ_HZ,
        .sample_frequency  = SAMPLE_RATE,
    };

    esp_err_t ret = es8311_init(s_es_handle, &clk_cfg,
                                ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "es8311_init failed: %s", esp_err_to_name(ret));
        es8311_delete(s_es_handle);
        s_es_handle = NULL;
        return ret;
    }

    ret = es8311_sample_frequency_config(s_es_handle, MCLK_FREQ_HZ, SAMPLE_RATE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "es8311_sample_frequency_config: %s (continuing)", esp_err_to_name(ret));
    }

    es8311_voice_volume_set(s_es_handle, VOICE_VOLUME, NULL);

    /* Configure microphone: analog mic (not digital), gain +24 dB */
    es8311_microphone_config(s_es_handle, false);
    es8311_microphone_gain_set(s_es_handle, MIC_GAIN);

    ESP_LOGI(TAG, "ES8311 codec initialized (16 kHz, 16-bit, PGA +24 dB)");
    return ESP_OK;
}

/* ─────────── Lightweight frequency-bin analysis ─────────── */

/**
 * Compute approximate power in a band [bin_lo, bin_hi) of a real signal
 * using the Goertzel algorithm.  Much cheaper than a full FFT for
 * NUM_BARS (10) bands.
 */
static float goertzel_band_power(const int16_t *samples, int n,
                                 int bin_lo, int bin_hi) {
    float total = 0.0f;
    for (int k = bin_lo; k < bin_hi; k++) {
        float w  = 2.0f * M_PI * k / n;
        float coeff = 2.0f * cosf(w);
        float s0 = 0, s1 = 0, s2 = 0;
        for (int i = 0; i < n; i++) {
            s0 = (float)samples[i] + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
        total += power;
    }
    return sqrtf(total / (bin_hi - bin_lo + 1)) / n;
}

/**
 * Map PCM capture buffer into NUM_BARS amplitude values [0..255].
 */
static void compute_bars(const int16_t *samples, int n, uint8_t *out_bars) {
    const int max_bin = n / 2;

    /* Log-spaced band edges.  Lowest band starts at bin 1 (~31 Hz). */
    float band_edges[NUM_BARS + 1];
    float log_min = logf(1.0f);
    float log_max = logf((float)max_bin);
    for (int i = 0; i <= NUM_BARS; i++) {
        float t = (float)i / NUM_BARS;
        band_edges[i] = expf(log_min + t * (log_max - log_min));
    }

    float max_power = 0;
    float powers[NUM_BARS];

    for (int i = 0; i < NUM_BARS; i++) {
        int lo = (int)band_edges[i];
        int hi = (int)band_edges[i + 1];
        if (hi <= lo) hi = lo + 1;
        if (hi > max_bin) hi = max_bin;
        powers[i] = goertzel_band_power(samples, n, lo, hi);
        if (powers[i] > max_power) max_power = powers[i];
    }

    /* Normalize to 0–255 with AGC */
    if (max_power < 1.0f) max_power = 1.0f;
    for (int i = 0; i < NUM_BARS; i++) {
        float norm = powers[i] / max_power * 255.0f;
        out_bars[i] = (uint8_t)(norm > 255.0f ? 255 : norm);
    }
}

/* ─────────── Capture task ─────────── */

static void audio_capture_task(void *arg) {
    /* Stereo 16-bit: 2 bytes/sample × 2 channels = 4 bytes per frame.
     * We read CAPTURE_BUF_SAMPLES stereo frames, then extract left channel. */
    const size_t stereo_buf_bytes = CAPTURE_BUF_SAMPLES * 2 * sizeof(int16_t);
    int16_t *stereo_buf = heap_caps_malloc(stereo_buf_bytes, MALLOC_CAP_INTERNAL);
    int16_t *mono_buf   = heap_caps_malloc(CAPTURE_BUF_SAMPLES * sizeof(int16_t),
                                           MALLOC_CAP_INTERNAL);
    if (!stereo_buf || !mono_buf) {
        ESP_LOGE(TAG, "Failed to allocate capture buffers");
        free(stereo_buf);
        free(mono_buf);
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Audio capture task started");

    while (s_running) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_rx_chan, stereo_buf,
                                          stereo_buf_bytes,
                                          &bytes_read,
                                          pdMS_TO_TICKS(200));
        if (ret != ESP_OK || bytes_read == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Extract left channel from interleaved stereo [L,R,L,R,...] */
        int stereo_samples = bytes_read / sizeof(int16_t);
        int mono_count = stereo_samples / 2;
        if (mono_count > CAPTURE_BUF_SAMPLES) mono_count = CAPTURE_BUF_SAMPLES;
        for (int i = 0; i < mono_count; i++) {
            mono_buf[i] = stereo_buf[i * 2];  /* left channel */
        }

        /* Compute frequency bars */
        uint8_t bars[NUM_BARS];
        compute_bars(mono_buf, mono_count, bars);

        /* Push to visualizer queue (non-blocking — drop if full) */
        extern QueueHandle_t amplitudeQueue;
        if (amplitudeQueue) {
            typedef struct { int bars[NUM_BARS]; } AmplitudeData;
            AmplitudeData ad;
            for (int i = 0; i < NUM_BARS; i++) {
                ad.bars[i] = bars[i];
            }
            xQueueSend(amplitudeQueue, &ad, 0);
        }
    }

    free(stereo_buf);
    free(mono_buf);
    ESP_LOGI(TAG, "Audio capture task stopped");
    s_capture_task = NULL;
    vTaskDelete(NULL);
}

/* ─────────── Public API ─────────── */

esp_err_t ws_audio_init(void) {
    if (s_initialized) return ESP_OK;

    ESP_LOGI(TAG, "Initialising ES8311 audio subsystem");

    /* 1. Power on audio rail and PA */
    esp_err_t ret = audio_power_on();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "audio_power_on returned %s (continuing anyway)",
                 esp_err_to_name(ret));
    }

    /* 2. Init I2S and enable channels — starts MCLK for ES8311 */
    ret = i2s_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S driver init failed");
        audio_power_off();
        return ret;
    }

    /* 3. Configure ES8311 codec (needs MCLK to be running) */
    ret = es8311_codec_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 codec init failed");
        if (s_rx_chan) { i2s_channel_disable(s_rx_chan); i2s_del_channel(s_rx_chan); s_rx_chan = NULL; }
        if (s_tx_chan) { i2s_channel_disable(s_tx_chan); i2s_del_channel(s_tx_chan); s_tx_chan = NULL; }
        audio_power_off();
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Audio subsystem ready");
    return ESP_OK;
}

void ws_audio_deinit(void) {
    if (!s_initialized) return;

    ws_audio_stop();

    if (s_es_handle) {
        es8311_delete(s_es_handle);
        s_es_handle = NULL;
    }

    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }

    audio_power_off();
    s_initialized = false;
    ESP_LOGI(TAG, "Audio subsystem de-initialized");
}

void ws_audio_start(void) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "Audio not initialized — call ws_audio_init() first");
        return;
    }
    if (s_running) return;

    s_running = true;
    xTaskCreate(audio_capture_task, "audio_cap", 8192, NULL, 5, &s_capture_task);
}

void ws_audio_stop(void) {
    if (!s_running) return;
    s_running = false;

    /* Wait for the task to exit */
    int timeout = 50;  /* 500 ms max */
    while (s_capture_task && --timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool ws_audio_is_running(void) {
    return s_running;
}

#else /* !CONFIG_USE_WAVESHARE_AMOLED */

/* Stubs for other boards */
esp_err_t ws_audio_init(void)     { return ESP_ERR_NOT_SUPPORTED; }
void      ws_audio_deinit(void)   {}
void      ws_audio_start(void)    {}
void      ws_audio_stop(void)     {}
bool      ws_audio_is_running(void) { return false; }

#endif /* CONFIG_USE_WAVESHARE_AMOLED */
