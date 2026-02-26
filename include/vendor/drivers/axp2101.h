#ifndef AXP2101_H
#define AXP2101_H

#include "esp_err.h"
#include <stdbool.h>

// I2C configuration
// On Waveshare ESP32-S3 1.8" AMOLED the AXP2101 shares the main I2C bus
// (SDA=15, SCL=14) at I2C_NUM_0 with CST816T touch and QMI8658 IMU.
#ifdef CONFIG_USE_WAVESHARE_AMOLED
#define I2C_MASTER_NUM   (I2C_NUM_0)
#define I2C_MASTER_SDA_IO 15
#define I2C_MASTER_SCL_IO 14
#define I2C_MASTER_FREQ_HZ 400000
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define I2C_MASTER_NUM (I2C_NUM_1) // I2C port number
#define I2C_MASTER_SDA_IO 10      // SDA pin
#define I2C_MASTER_SCL_IO 11      // SCL pin
#define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency
#else
#define I2C_MASTER_NUM (I2C_NUM_0) // I2C port number
#define I2C_MASTER_SDA_IO 10      // SDA pin
#define I2C_MASTER_SCL_IO 11      // SCL pin
#define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency
#endif
#define AXP2101_I2C_ADDR 0x34     // AXP2101 I2C address

/**
 * @brief Check if the battery is connected
 *
 * @return true if the battery is connected, false otherwise
 */
bool axp202_is_battery_connected(void);

/**
 * @brief Check if the device is charging
 *
 * @return true if charging, false otherwise
 */
bool axp202_is_charging(void);

/**
 * @brief Initialize I2C and AXP2101 communication
 *
 * @return esp_err_t ESP_OK if successful, an error code otherwise
 */
esp_err_t axp2101_init(void);

/**
 * @brief Deinitialize I2C and AXP2101 communication
 *
 * @return esp_err_t ESP_OK if successful, an error code otherwise
 */
esp_err_t axp2101_deinit(void);

/**
 * @brief Get the power level from the AXP2101 module
 *
 * @param[out] power_level Pointer to store the power level (0-100%)
 * @return esp_err_t ESP_OK if successful, an error code otherwise
 */
esp_err_t axp2101_get_power_level(uint8_t *power_level);

#endif /* AXP2101_H */