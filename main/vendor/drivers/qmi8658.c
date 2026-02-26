/**
 * @file qmi8658.c
 * @brief Minimal QMI8658 IMU driver (accelerometer only) for ESP-IDF.
 *
 * Uses the legacy i2c_master_cmd_begin() API to stay consistent with the
 * rest of Ghost ESP's I2C peripherals (CST816T, TCA9554, AXP2101).
 * Requires the I2C bus to be already installed by the caller.
 */

#include "vendor/drivers/qmi8658.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "io_manager/i2c_bus_lock.h"
#include <math.h>

static const char *TAG = "QMI8658";

// ---- QMI8658 registers (subset needed for accel-only orientation) ----
#define REG_WHO_AM_I     0x00
#define REG_CTRL1        0x02   // SPI/I2C config, sensor enable bits
#define REG_CTRL2        0x03   // Accelerometer config (range + ODR)
#define REG_CTRL5        0x06   // Low-pass filter config
#define REG_CTRL7        0x08   // Sensor enable (bit0=accel, bit1=gyro)
#define REG_AX_L         0x35   // Accel X low byte
#define REG_RESET        0x60   // Soft-reset register

#define RESET_CMD        0xB0
#define RST_RESULT_REG   0x4D
#define RST_RESULT_VAL   0x80

// ACC_RANGE_4G  = 0x01, ODR 250 Hz = 0x06 → CTRL2 = (1<<4) | 6 = 0x16
// LPF mode 0, accel LPF enable bit in CTRL5 bit 0
#define CTRL2_ACCEL_CFG  0x16   // 4G range, 250Hz ODR
#define ACCEL_SCALE      (4.0f / 32768.0f)

static int s_i2c_port = -1;
static bool s_ready = false;

// ---- Low-level I2C helpers ----

static esp_err_t qmi8658_write_reg(uint8_t reg, uint8_t val) {
    i2c_bus_lock(s_i2c_port, 100);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    i2c_bus_unlock(s_i2c_port);
    return ret;
}

static esp_err_t qmi8658_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_bus_lock(s_i2c_port, 100);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // repeated start
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &buf[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    i2c_bus_unlock(s_i2c_port);
    return ret;
}

static esp_err_t qmi8658_set_register_bit(uint8_t reg, uint8_t bit) {
    uint8_t val;
    esp_err_t ret = qmi8658_read_reg(reg, &val, 1);
    if (ret != ESP_OK) return ret;
    val |= (1 << bit);
    return qmi8658_write_reg(reg, val);
}

// ---- Public API ----

esp_err_t qmi8658_init(int i2c_port) {
    s_i2c_port = i2c_port;
    s_ready = false;

    // Soft-reset
    esp_err_t ret = qmi8658_write_reg(REG_RESET, RESET_CMD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Could not access QMI8658 (write reset): %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // Verify reset result
    uint8_t rst_result = 0;
    for (int i = 0; i < 10; i++) {
        ret = qmi8658_read_reg(RST_RESULT_REG, &rst_result, 1);
        if (ret == ESP_OK && rst_result == RST_RESULT_VAL) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (rst_result != RST_RESULT_VAL) {
        ESP_LOGW(TAG, "Reset result unexpected: 0x%02X (expected 0x%02X)", rst_result, RST_RESULT_VAL);
    }

    // Verify WHO_AM_I
    uint8_t chip_id = 0;
    ret = qmi8658_read_reg(REG_WHO_AM_I, &chip_id, 1);
    if (ret != ESP_OK || chip_id != QMI8658_CHIP_ID) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", chip_id, QMI8658_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "QMI8658 detected (chip ID: 0x%02X)", chip_id);

    // Enable address auto-increment (CTRL1 bit 6)
    ret = qmi8658_set_register_bit(REG_CTRL1, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set CTRL1 auto-increment");
        return ret;
    }

    // Configure accelerometer: 4G range, 250Hz ODR
    ret = qmi8658_write_reg(REG_CTRL2, CTRL2_ACCEL_CFG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accel (CTRL2)");
        return ret;
    }

    // Enable accel LPF (CTRL5 bit 0)
    ret = qmi8658_set_register_bit(REG_CTRL5, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable accel LPF (non-fatal)");
    }

    // Enable accelerometer only (CTRL7 bit 0)
    ret = qmi8658_set_register_bit(REG_CTRL7, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer (CTRL7)");
        return ret;
    }

    s_ready = true;
    ESP_LOGI(TAG, "Accelerometer enabled (4G, 250Hz)");
    return ESP_OK;
}

esp_err_t qmi8658_get_accel(float *ax, float *ay, float *az) {
    if (!s_ready) return ESP_ERR_INVALID_STATE;

    uint8_t buf[6];
    esp_err_t ret = qmi8658_read_reg(REG_AX_L, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t raw_x = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t raw_y = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t raw_z = (int16_t)(buf[4] | (buf[5] << 8));

    *ax = raw_x * ACCEL_SCALE;
    *ay = raw_y * ACCEL_SCALE;
    *az = raw_z * ACCEL_SCALE;

    return ESP_OK;
}

bool qmi8658_is_ready(void) {
    return s_ready;
}
