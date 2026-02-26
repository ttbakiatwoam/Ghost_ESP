idf_component_register(SRCS "rm67162_qspi.c"
                             "esp_lcd_rm67162.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_lcd lvgl esp_timer)
