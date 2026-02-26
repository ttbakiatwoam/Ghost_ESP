/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP LCD RM67162 Panel Driver Implementation
 * Based on MicroPython AMOLED driver for Waveshare ESP32-S3 1.8" AMOLED (SH8601 mode)
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_rm67162.h"
#include "rm67162_qspi.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_lcd_rm67162.h"
#include "rm67162_qspi.h"  // For QSPI functions

static const char *TAG = "lcd_rm67162";

// SH8601/RM67162 specific commands
#define LCD_CMD_SWITCHMODE      0xFE
#define LCD_CMD_SETSPIMODE      0xC4
#define LCD_CMD_SETDISPMODE     0xC2
#define LCD_CMD_WRCTRLD1        0x53
#define LCD_CMD_SETTSCANL       0x44
#define LCD_CMD_WRDISBV         0x51
#define LCD_CMD_TEON            0x35
#define LCD_CMD_COLMOD          0x3A

// MADCTL bits
#define LCD_CMD_MADCTL_MY       0x80
#define LCD_CMD_MADCTL_MX       0x40
#define LCD_CMD_MADCTL_MV       0x20
#define LCD_CMD_MADCTL_ML       0x10
#define LCD_CMD_MADCTL_BGR      0x08
#define LCD_CMD_MADCTL_MH       0x04

// Color modes (COLMOD register values)
#define COLMOD_16BPP            0x55
#define COLMOD_18BPP            0x66
#define COLMOD_24BPP            0x77

typedef struct {
    esp_lcd_panel_t base;
    void *qspi_ctx;  // QSPI context for direct calls
    int reset_gpio;
    uint16_t width;
    uint16_t height;
    uint16_t x_gap;
    uint16_t y_gap;
    uint8_t madctl_val;
    uint8_t colmod_val;
    bool reset_level;
} rm67162_panel_t;

static esp_err_t panel_rm67162_del(esp_lcd_panel_t *panel);
static esp_err_t panel_rm67162_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_rm67162_init(esp_lcd_panel_t *panel);
static esp_err_t panel_rm67162_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_rm67162_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_rm67162_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_rm67162_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_rm67162_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_rm67162_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

esp_err_t esp_lcd_new_panel_rm67162(void *qspi_ctx,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(qspi_ctx && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid arguments");

    esp_err_t ret = ESP_OK;
    rm67162_panel_t *rm67162 = calloc(1, sizeof(rm67162_panel_t));
    ESP_RETURN_ON_FALSE(rm67162, ESP_ERR_NO_MEM, TAG, "no mem for rm67162 panel");

    if (panel_dev_config && panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST failed");
    }

    rm67162->qspi_ctx = qspi_ctx;
    rm67162->reset_gpio = panel_dev_config ? panel_dev_config->reset_gpio_num : -1;
    rm67162->reset_level = (panel_dev_config && panel_dev_config->flags.reset_active_high);
    rm67162->width = 368;   // Waveshare 1.8" AMOLED width
    rm67162->height = 448;  // Waveshare 1.8" AMOLED height
    rm67162->x_gap = 0;
    rm67162->y_gap = 0;
    rm67162->madctl_val = 0x00;  // BGR mode for correct color order
    rm67162->colmod_val = COLMOD_16BPP;  // Default to 16bpp

    rm67162->base.del = panel_rm67162_del;
    rm67162->base.reset = panel_rm67162_reset;
    rm67162->base.init = panel_rm67162_init;
    rm67162->base.draw_bitmap = panel_rm67162_draw_bitmap;
    rm67162->base.invert_color = panel_rm67162_invert_color;
    rm67162->base.set_gap = panel_rm67162_set_gap;
    rm67162->base.mirror = panel_rm67162_mirror;
    rm67162->base.swap_xy = panel_rm67162_swap_xy;
    rm67162->base.disp_on_off = panel_rm67162_disp_on_off;

    *ret_panel = &(rm67162->base);
    ESP_LOGI(TAG, "new RM67162 panel @%p", rm67162);

    return ESP_OK;

err:
    if (rm67162) {
        free(rm67162);
    }
    return ret;
}

static esp_err_t panel_rm67162_del(esp_lcd_panel_t *panel)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);

    if (rm67162->reset_gpio >= 0) {
        gpio_reset_pin(rm67162->reset_gpio);
    }

    free(rm67162);
    ESP_LOGD(TAG, "del rm67162 panel @%p", rm67162);
    return ESP_OK;
}

static esp_err_t panel_rm67162_reset(esp_lcd_panel_t *panel)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;

    if (rm67162->reset_gpio >= 0) {
        gpio_set_level(rm67162->reset_gpio, rm67162->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(rm67162->reset_gpio, !rm67162->reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else {
        // Software reset
        rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    return ESP_OK;
}

static esp_err_t panel_rm67162_init(esp_lcd_panel_t *panel)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;

    // Sleep out
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    // SH8601 specific initialization sequence
    // Switch to HBM mode
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SWITCHMODE, (uint8_t[]){0x20}, 1);
    rm67162_qspi_tx_param(qspi_ctx, 0x63, (uint8_t[]){0xFF}, 1);  // Brightness HBM
    rm67162_qspi_tx_param(qspi_ctx, 0x26, (uint8_t[]){0x0A}, 1);
    rm67162_qspi_tx_param(qspi_ctx, 0x24, (uint8_t[]){0x80}, 1);

    // Back to command mode
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SWITCHMODE, (uint8_t[]){0x20}, 1);
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SETSPIMODE, (uint8_t[]){0x80}, 1);   // QSPI MODE
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SETDISPMODE, (uint8_t[]){0x00}, 1);  // DSPI MODE OFF
    vTaskDelay(pdMS_TO_TICKS(10));

    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_WRCTRLD1, (uint8_t[]){0x20}, 1);     // Brightness control ON
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_SETTSCANL, (uint8_t[]){0x01, 0xC0}, 2);  // Tear scanline N=448

    // Set pixel format
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_COLMOD, &rm67162->colmod_val, 1);

    // Set brightness minimum
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_WRDISBV, (uint8_t[]){0x00}, 1);

    // Tearing effect off
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_TEON, (uint8_t[]){0x00}, 1);

    // Set MADCTL
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_MADCTL, &rm67162->madctl_val, 1);

    // Display on
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_DISPON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set brightness to max
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_WRDISBV, (uint8_t[]){0xFF}, 1);

    ESP_LOGI(TAG, "RM67162 panel initialized");

    return ESP_OK;
}

static esp_err_t panel_rm67162_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, 
                                           int x_end, int y_end, const void *color_data)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;

    ESP_RETURN_ON_FALSE(x_start < x_end && y_start < y_end, ESP_ERR_INVALID_ARG, TAG, "invalid coordinates");

    x_start += rm67162->x_gap;
    x_end += rm67162->x_gap;
    y_start += rm67162->y_gap;
    y_end += rm67162->y_gap;

    // Set column address
    uint8_t col_data[] = {
        (x_start >> 8) & 0xFF, x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF, (x_end - 1) & 0xFF
    };
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_CASET, col_data, 4);

    // Set row address
    uint8_t row_data[] = {
        (y_start >> 8) & 0xFF, y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF, (y_end - 1) & 0xFF
    };
    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_RASET, row_data, 4);

    // Transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * 2;  // 16bpp = 2 bytes per pixel
    rm67162_qspi_tx_color(qspi_ctx, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_rm67162_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;
    int command = invert_color_data ? LCD_CMD_INVON : LCD_CMD_INVOFF;
    rm67162_qspi_tx_param(qspi_ctx, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_rm67162_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;

    if (mirror_x) {
        rm67162->madctl_val |= LCD_CMD_MADCTL_MX;
    } else {
        rm67162->madctl_val &= ~LCD_CMD_MADCTL_MX;
    }

    if (mirror_y) {
        rm67162->madctl_val |= LCD_CMD_MADCTL_MY;
    } else {
        rm67162->madctl_val &= ~LCD_CMD_MADCTL_MY;
    }

    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_MADCTL, &rm67162->madctl_val, 1);
    return ESP_OK;
}

static esp_err_t panel_rm67162_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;

    if (swap_axes) {
        rm67162->madctl_val |= LCD_CMD_MADCTL_MV;
    } else {
        rm67162->madctl_val &= ~LCD_CMD_MADCTL_MV;
    }

    rm67162_qspi_tx_param(qspi_ctx, LCD_CMD_MADCTL, &rm67162->madctl_val, 1);
    return ESP_OK;
}

static esp_err_t panel_rm67162_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    rm67162->x_gap = x_gap;
    rm67162->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_rm67162_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    rm67162_panel_t *rm67162 = __containerof(panel, rm67162_panel_t, base);
    void *qspi_ctx = rm67162->qspi_ctx;
    int command = on_off ? LCD_CMD_DISPON : LCD_CMD_DISPOFF;
    rm67162_qspi_tx_param(qspi_ctx, command, NULL, 0);
    return ESP_OK;
}
