/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CST816T Touch Controller Driver for ESP-IDF
 * Based on MicroPython implementation from Lilygo Waveshare AMOLED
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CST816T touch event types
 */
typedef enum {
    CST816T_EVENT_NONE = 0x00,
    CST816T_EVENT_PRESS_DOWN = 0x01,
    CST816T_EVENT_LIFT_UP = 0x02,
    CST816T_EVENT_CONTACT = 0x03,
} cst816t_event_t;

/**
 * @brief CST816T gesture types
 */
typedef enum {
    CST816T_GESTURE_NONE = 0x00,
    CST816T_GESTURE_SWIPE_UP = 0x01,
    CST816T_GESTURE_SWIPE_DOWN = 0x02,
    CST816T_GESTURE_SWIPE_LEFT = 0x03,
    CST816T_GESTURE_SWIPE_RIGHT = 0x04,
    CST816T_GESTURE_SINGLE_CLICK = 0x05,
    CST816T_GESTURE_DOUBLE_CLICK = 0x0B,
    CST816T_GESTURE_LONG_PRESS = 0x0C,
} cst816t_gesture_t;

/**
 * @brief CST816T touch point data
 */
typedef struct {
    uint16_t x;                     ///< X coordinate
    uint16_t y;                     ///< Y coordinate
    cst816t_event_t event;          ///< Touch event type
    uint8_t points;                 ///< Number of touch points
} cst816t_touch_data_t;

/**
 * @brief CST816T configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;            ///< I2C port number
    uint8_t i2c_addr;               ///< I2C device address (default: 0x15)
    gpio_num_t int_gpio;            ///< Interrupt GPIO pin (-1 if not used)
    gpio_num_t rst_gpio;            ///< Reset GPIO pin (-1 if not used)
    uint16_t width;                 ///< Display width for coordinate validation
    uint16_t height;                ///< Display height for coordinate validation
    bool swap_xy;                   ///< Swap X and Y coordinates
    bool invert_x;                  ///< Invert X coordinate
    bool invert_y;                  ///< Invert Y coordinate
} cst816t_config_t;

/**
 * @brief CST816T device handle
 */
typedef struct cst816t_dev_s *cst816t_handle_t;

/**
 * @brief Initialize CST816T touch controller
 *
 * @param config Configuration structure
 * @param handle_out Output handle
 * @return ESP_OK on success
 */
esp_err_t cst816t_init(const cst816t_config_t *config, cst816t_handle_t *handle_out);

/**
 * @brief Deinitialize CST816T touch controller
 *
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t cst816t_deinit(cst816t_handle_t handle);

/**
 * @brief Read touch data from CST816T
 *
 * @param handle Device handle
 * @param data Output touch data
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no touch detected
 */
esp_err_t cst816t_read_touch(cst816t_handle_t handle, cst816t_touch_data_t *data);

/**
 * @brief Read gesture from CST816T
 *
 * @param handle Device handle
 * @param gesture Output gesture type
 * @return ESP_OK on success
 */
esp_err_t cst816t_read_gesture(cst816t_handle_t handle, cst816t_gesture_t *gesture);

/**
 * @brief Get chip ID from CST816T
 *
 * @param handle Device handle
 * @param chip_id Output chip ID
 * @return ESP_OK on success
 */
esp_err_t cst816t_get_chip_id(cst816t_handle_t handle, uint8_t *chip_id);

#ifdef __cplusplus
}
#endif
