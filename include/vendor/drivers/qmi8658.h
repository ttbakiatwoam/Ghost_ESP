#ifndef QMI8658_H
#define QMI8658_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// QMI8658 I2C address (AD0 high = 0x6B, AD0 low = 0x6A)
#define QMI8658_I2C_ADDR     0x6B

// WHO_AM_I expected value
#define QMI8658_CHIP_ID      0x05

/**
 * @brief Initialize the QMI8658 accelerometer on an existing I2C bus.
 *        Does NOT install the I2C driver — caller must have done that.
 *
 * @param i2c_port  I2C port number (e.g. I2C_NUM_0)
 * @return ESP_OK on success
 */
esp_err_t qmi8658_init(int i2c_port);

/**
 * @brief Read accelerometer values in g units.
 *
 * @param[out] ax  X-axis acceleration (g)
 * @param[out] ay  Y-axis acceleration (g)
 * @param[out] az  Z-axis acceleration (g)
 * @return ESP_OK on success
 */
esp_err_t qmi8658_get_accel(float *ax, float *ay, float *az);

/**
 * @brief Check if the QMI8658 was successfully initialised.
 */
bool qmi8658_is_ready(void);

#endif // QMI8658_H
