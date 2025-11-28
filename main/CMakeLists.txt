idf_component_register(SRCS "bt_app_avrc_ct.c" "bt_app_avrc_tg.c" "main.c" "bt_app_core.c" "uart_app.c" "utils.c" "adc_app.c" 
                    PRIV_REQUIRES bt nvs_flash esp_driver_i2s esp_driver_gpio esp_ringbuf esp_driver_uart esp_adc
                    INCLUDE_DIRS ".")
