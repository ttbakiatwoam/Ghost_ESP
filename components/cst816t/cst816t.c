/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CST816T Touch Controller Driver Implementation
 * Based on MicroPython driver from Lilygo Waveshare AMOLED
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "cst816t.h"

static const char *TAG = "CST816T";

// CST816T Register addresses
#define CST816T_REG_DATA        0x00
#define CST816T_REG_GESTURE     0x01
#define CST816T_REG_POINTS      0x02
#define CST816T_REG_EVENT       0x03
#define CST816T_REG_X_H         0x03
#define CST816T_REG_X_L         0x04
#define CST816T_REG_Y_H         0x05
#define CST816T_REG_Y_L         0x06
#define CST816T_REG_CHIP_ID     0xA7
#define CST816T_REG_FW_VERSION  0xA9

#define CST816T_I2C_TIMEOUT_MS  100

/**
 * @brief CST816T device structure
 */
struct cst816t_dev_s {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t int_gpio;
    gpio_num_t rst_gpio;
    uint16_t width;
    uint16_t height;
    bool swap_xy;
    bool invert_x;
    bool invert_y;
};

/**
 * @brief Read register from CST816T
 */
static esp_err_t cst816t_read_reg(cst816t_handle_t handle, uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(CST816T_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Write register to CST816T
 */
static esp_err_t cst816t_write_reg(cst816t_handle_t handle, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (handle->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(handle->i2c_port, cmd, pdMS_TO_TICKS(CST816T_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t cst816t_init(const cst816t_config_t *config, cst816t_handle_t *handle_out)
{
    ESP_RETURN_ON_FALSE(config && handle_out, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    esp_err_t ret = ESP_OK;
    cst816t_handle_t handle = calloc(1, sizeof(struct cst816t_dev_s));
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, TAG, "No memory for CST816T handle");
    
    handle->i2c_port = config->i2c_port;
    handle->i2c_addr = config->i2c_addr;
    handle->int_gpio = config->int_gpio;
    handle->rst_gpio = config->rst_gpio;
    handle->width = config->width;
    handle->height = config->height;
    handle->swap_xy = config->swap_xy;
    handle->invert_x = config->invert_x;
    handle->invert_y = config->invert_y;
    
    // Configure interrupt pin if provided
    if (handle->int_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << handle->int_gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), cleanup, TAG, "Failed to configure INT GPIO");
    }
    
    // Hardware reset if reset pin is provided
    if (handle->rst_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << handle->rst_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), cleanup, TAG, "Failed to configure RST GPIO");
        
        gpio_set_level(handle->rst_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(handle->rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Verify chip ID
    uint8_t chip_id;
    ESP_GOTO_ON_ERROR(cst816t_read_reg(handle, CST816T_REG_CHIP_ID, &chip_id, 1), 
                      cleanup, TAG, "Failed to read chip ID");
    
    ESP_LOGI(TAG, "CST816T initialized - Chip ID: 0x%02X", chip_id);
    
    *handle_out = handle;
    return ESP_OK;

cleanup:
    free(handle);
    return ESP_FAIL;
}

esp_err_t cst816t_deinit(cst816t_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    free(handle);
    return ESP_OK;
}

esp_err_t cst816t_read_touch(cst816t_handle_t handle, cst816t_touch_data_t *data)
{
    ESP_RETURN_ON_FALSE(handle && data, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    uint8_t buf[7];  // Need 7 bytes: buf[0] through buf[6]
    ESP_RETURN_ON_ERROR(cst816t_read_reg(handle, CST816T_REG_DATA, buf, 7), TAG, "Failed to read touch data");
    
    // Parse touch data
    data->points = buf[2] & 0x0F;
    
    if (data->points == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Extract coordinates (bits 0-3 of buf[3] are upper 4 bits of X)
    uint16_t x = ((buf[3] & 0x0F) << 8) | buf[4];
    uint16_t y = ((buf[5] & 0x0F) << 8) | buf[6];
    
    // Extract event type from upper 2 bits of buf[3]
    data->event = (buf[3] >> 6) & 0x03;
    
    // Apply transformations
    if (handle->swap_xy) {
        uint16_t temp = x;
        x = y;
        y = temp;
    }
    
    if (handle->invert_x) {
        x = handle->width - x - 1;
    }
    
    if (handle->invert_y) {
        y = handle->height - y - 1;
    }
    
    data->x = x;
    data->y = y;
    
    ESP_LOGD(TAG, "Touch: x=%u, y=%u, event=%d, points=%d", x, y, data->event, data->points);
    
    return ESP_OK;
}

esp_err_t cst816t_read_gesture(cst816t_handle_t handle, cst816t_gesture_t *gesture)
{
    ESP_RETURN_ON_FALSE(handle && gesture, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    uint8_t gest;
    ESP_RETURN_ON_ERROR(cst816t_read_reg(handle, CST816T_REG_GESTURE, &gest, 1), 
                        TAG, "Failed to read gesture");
    
    *gesture = (cst816t_gesture_t)gest;
    
    return ESP_OK;
}

esp_err_t cst816t_get_chip_id(cst816t_handle_t handle, uint8_t *chip_id)
{
    ESP_RETURN_ON_FALSE(handle && chip_id, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    return cst816t_read_reg(handle, CST816T_REG_CHIP_ID, chip_id, 1);
}
