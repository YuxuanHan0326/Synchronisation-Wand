idf_component_register(SRCS "ble_gatt.c" "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash driver esp_hw_support log lwip esp_wifi esp_netif
                                  bt fatfs esp_timer icm20948 sd_card time_client u8g2_esp32_hal
                                  wifi_connect)
