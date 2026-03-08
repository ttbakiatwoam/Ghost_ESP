/**
 * @file ws_audio.c
 * @brief ES8311 codec + I2S microphone driver for Waveshare ESP32-S3 1.8" AMOLED.
 *
 * Captures 16-bit mono PCM from the on-board SMD microphone via the ES8311
 * audio codec over I2S, runs a lightweight FFT, and pushes NUM_BARS amplitude
 * bins into the music visualizer's amplitudeQueue.
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
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "managers/views/music_visualizer.h"

static const char *TAG = "ws_audio";

/* ─────────── Pin definitions ─────────── */
#define AUDIO_I2C_NUM       I2C_NUM_0
#define ES8311_I2C_ADDR     0x18

#define I2S_MCLK_PIN        16
#define I2S_BCLK_PIN        9
#define I2S_WS_PIN          45
#define I2S_DOUT_PIN        8   /* ESP → codec (speaker) – unused for mic */
#define I2S_DIN_PIN         10  /* codec → ESP (microphone) */
#define PA_ENABLE_PIN       46

/* AXP2101 BLDO2 control registers */
#define AXP2101_I2C_ADDR    0x34
#define AXP2101_REG_BLDO2_CTRL  0x97  /* BLDO2 voltage */
#define AXP2101_REG_LDO_EN1     0x90  /* LDO enable control 1 */
#define AXP2101_BLDO2_EN_BIT    (1 << 1)  /* Bit 1 of LDO_EN1 */

/* ─────────── ES8311 register map (subset) ─────────── */
#define ES8311_REG_RESET            0x00
#define ES8311_REG_CLK_MANAGER1     0x01
#define ES8311_REG_CLK_MANAGER2     0x02
#define ES8311_REG_CLK_MANAGER3     0x03
#define ES8311_REG_CLK_MANAGER4     0x04
#define ES8311_REG_CLK_MANAGER5     0x05
#define ES8311_REG_CLK_MANAGER6     0x06
#define ES8311_REG_CLK_MANAGER7     0x07
#define ES8311_REG_CLK_MANAGER8     0x08
#define ES8311_REG_SDP_IN           0x09
#define ES8311_REG_SDP_OUT          0x0A
#define ES8311_REG_SYSTEM           0x0D
#define ES8311_REG_ADC1             0x17
#define ES8311_REG_ADC2             0x18
#define ES8311_REG_ADC_VOLUME       0x19  /* 0–255; 0xBF = 0 dB */
#define ES8311_REG_GPIO             0x44

/* ─────────── I2S / capture config ─────────── */
#define SAMPLE_RATE         16000
#define DMA_BUF_COUNT       4
#define DMA_BUF_LEN         256     /* samples per DMA buffer */

/* FFT is done as a simplified power-spectrum via DFT at selected bins,
 * using a small rolling buffer.  No full FFT library needed for 10 bars. */
#define CAPTURE_BUF_SAMPLES 512     /* ~32 ms at 16 kHz */

/* ─────────── State ─────────── */
static bool           s_initialized   = false;
static bool           s_running       = false;
static TaskHandle_t   s_capture_task  = NULL;
static i2s_chan_handle_t s_rx_chan     = NULL;

/* ─────────── I2C helpers ─────────── */

static esp_err_t es8311_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(AUDIO_I2C_NUM, ES8311_I2C_ADDR,
                                      buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static esp_err_t es8311_read_reg(uint8_t reg, uint8_t *val) {
    esp_err_t ret = i2c_master_write_to_device(AUDIO_I2C_NUM, ES8311_I2C_ADDR,
                                                &reg, 1, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) return ret;
    return i2c_master_read_from_device(AUDIO_I2C_NUM, ES8311_I2C_ADDR,
                                       val, 1, pdMS_TO_TICKS(50));
}

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
        /* non-fatal — some boards may not need this */
    }

    /* Enable BLDO2 */
    uint8_t en = 0;
    ret = axp_read_reg(AXP2101_REG_LDO_EN1, &en);
    if (ret == ESP_OK) {
        en |= AXP2101_BLDO2_EN_BIT;
        axp_write_reg(AXP2101_REG_LDO_EN1, en);
    }

    /* Enable PA */
    gpio_config_t pa_cfg = {
        .pin_bit_mask = (1ULL << PA_ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pa_cfg);
    gpio_set_level(PA_ENABLE_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(20)); /* let rails stabilize */
    return ESP_OK;
}

static void audio_power_off(void) {
    gpio_set_level(PA_ENABLE_PIN, 0);

    uint8_t en = 0;
    if (axp_read_reg(AXP2101_REG_LDO_EN1, &en) == ESP_OK) {
        en &= ~AXP2101_BLDO2_EN_BIT;
        axp_write_reg(AXP2101_REG_LDO_EN1, en);
    }
}

/* ─────────── ES8311 codec init ─────────── */

static esp_err_t es8311_codec_init(void) {
    esp_err_t ret;

    /* Soft-reset the codec */
    ret = es8311_write_reg(ES8311_REG_RESET, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 not responding at I2C 0x%02X: %s",
                 ES8311_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    es8311_write_reg(ES8311_REG_RESET, 0x00);    /* de-assert reset */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Clock configuration
     * Assume MCLK = 256 × fs = 256 × 16000 = 4.096 MHz from I2S driver.
     * CLK_MANAGER1: MCLK source = from pad (bit7=0), MCLK divider = /1 */
    es8311_write_reg(ES8311_REG_CLK_MANAGER1, 0x30);

    /* CLK_MANAGER2: MCLK/BCLK ratio — with MCLK=4.096 MHz, BCLK=16k*16*2=512 kHz
     * ratio = 8 → register = 0x00 (divider index) */
    es8311_write_reg(ES8311_REG_CLK_MANAGER2, 0x00);

    /* CLK_MANAGER3: ADC over-sample rate = 128×, enable ADC clock */
    es8311_write_reg(ES8311_REG_CLK_MANAGER3, 0x10);

    /* CLK_MANAGER5: ADC_MCLK divider */
    es8311_write_reg(ES8311_REG_CLK_MANAGER5, 0x00);

    /* CLK_MANAGER6: BCLK divider for ADC side */
    es8311_write_reg(ES8311_REG_CLK_MANAGER6, 0x30);

    /* CLK_MANAGER7/8: not strictly needed at 16 kHz, keep defaults */

    /* SDP_OUT (ADC I2S output): 16-bit, I2S format */
    es8311_write_reg(ES8311_REG_SDP_OUT, 0x0C);

    /* SYSTEM: power-up ADC + analog, keep DAC off */
    es8311_write_reg(ES8311_REG_SYSTEM, 0x01);

    /* ADC1: ADC input from LIN2 (on-board mic), PGA gain = +24 dB */
    es8311_write_reg(ES8311_REG_ADC1, 0x25);

    /* ADC2: ALC off, HPF on */
    es8311_write_reg(ES8311_REG_ADC2, 0x08);

    /* ADC digital volume: 0xBF = 0 dB */
    es8311_write_reg(ES8311_REG_ADC_VOLUME, 0xBF);

    /* GPIO: defaults are fine */
    es8311_write_reg(ES8311_REG_GPIO, 0x00);

    ESP_LOGI(TAG, "ES8311 codec initialized (16 kHz, 16-bit mono ADC, PGA +24 dB)");
    return ESP_OK;
}

/* ─────────── I2S RX channel setup ─────────── */

static esp_err_t i2s_rx_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = DMA_BUF_LEN;

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,   /* not transmitting */
            .din  = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(s_rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(ret));
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "I2S RX channel initialized (%d Hz, 16-bit mono)", SAMPLE_RATE);
    return ESP_OK;
}

/* ─────────── Lightweight frequency-bin analysis ─────────── */

/**
 * Compute approximate power in a band [bin_lo, bin_hi) of a real signal
 * using the Goertzel algorithm.  Much cheaper than a full FFT for
 * NUM_BARS (10) bands.
 *
 * @param samples   16-bit signed PCM buffer
 * @param n         number of samples
 * @param bin_lo    lower frequency bin (freq = bin * sample_rate / n)
 * @param bin_hi    upper frequency bin
 * @return          approximate RMS power (linear scale)
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
 * Map PCM capture buffer into NUM_BARS amplitude values [0..max_h].
 */
static void compute_bars(const int16_t *samples, int n, uint8_t *out_bars) {
    /* Divide the Nyquist range (0 – Fs/2 = 8 kHz) into NUM_BARS bands.
     * Use logarithmic spacing to emphasize bass.  Each bin = Fs/n ≈ 31.25 Hz.*/
    const int max_bin = n / 2;  /* Nyquist bin */

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
    int16_t *buf = heap_caps_malloc(CAPTURE_BUF_SAMPLES * sizeof(int16_t),
                                    MALLOC_CAP_INTERNAL);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate capture buffer");
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    i2s_channel_enable(s_rx_chan);
    ESP_LOGI(TAG, "Audio capture task started");

    while (s_running) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_rx_chan, buf,
                                          CAPTURE_BUF_SAMPLES * sizeof(int16_t),
                                          &bytes_read,
                                          pdMS_TO_TICKS(100));
        if (ret != ESP_OK || bytes_read == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int samples_read = bytes_read / sizeof(int16_t);

        /* Compute frequency bars */
        uint8_t bars[NUM_BARS];
        compute_bars(buf, samples_read, bars);

        /* Push to visualizer queue (non-blocking — drop if full) */
        extern QueueHandle_t amplitudeQueue;
        if (amplitudeQueue) {
            /* AmplitudeData is defined in music_visualizer.c as:
             *   typedef struct { int bars[NUM_BARS]; } AmplitudeData;
             * We build it here. */
            typedef struct { int bars[NUM_BARS]; } AmplitudeData;
            AmplitudeData ad;
            for (int i = 0; i < NUM_BARS; i++) {
                ad.bars[i] = bars[i];
            }
            xQueueSend(amplitudeQueue, &ad, 0);
        }
    }

    i2s_channel_disable(s_rx_chan);
    free(buf);
    ESP_LOGI(TAG, "Audio capture task stopped");
    s_capture_task = NULL;
    vTaskDelete(NULL);
}

/* ─────────── Public API ─────────── */

esp_err_t ws_audio_init(void) {
    if (s_initialized) return ESP_OK;

    ESP_LOGI(TAG, "Initialising ES8311 audio subsystem");

    esp_err_t ret = audio_power_on();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "audio_power_on returned %s (continuing anyway)",
                 esp_err_to_name(ret));
    }

    ret = es8311_codec_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 codec init failed");
        audio_power_off();
        return ret;
    }

    ret = i2s_rx_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S RX init failed");
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

    if (s_rx_chan) {
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
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
    xTaskCreate(audio_capture_task, "audio_cap", 4096, NULL, 5, &s_capture_task);
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
