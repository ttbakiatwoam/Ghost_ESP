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

#ifdef __cplusplus
}
#endif
