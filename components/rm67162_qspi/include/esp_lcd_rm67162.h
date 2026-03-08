/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP LCD RM67162 Panel Driver
 */

#pragma once

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create LCD panel for RM67162 controller via QSPI
 *
 * @note This panel driver uses direct QSPI communication, not standard panel IO
 * 
 * @param qspi_ctx QSPI context from rm67162_qspi_init
 * @param panel_dev_config General panel device configuration
 * @param ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameter is invalid
 *      - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t esp_lcd_new_panel_rm67162(void *qspi_ctx,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Set hardware rotation on the RM67162/SH8601 panel via MADCTL.
 *
 * @param panel   Panel handle from esp_lcd_new_panel_rm67162()
 * @param rotation  0 = 0°, 1 = 90° CW, 2 = 180°, 3 = 270° CW
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_panel_rm67162_set_rotation(esp_lcd_panel_handle_t panel, uint8_t rotation);

#ifdef __cplusplus
}
#endif
