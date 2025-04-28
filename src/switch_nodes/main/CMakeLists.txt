idf_component_register(SRCS "internal_switch.c"
                       INCLUDE_DIRS "." 
                       REQUIRES esp_eth esp_netif esp_hw_support esp_wifi nvs_flash lwip log sd_card fatfs esp_driver_uart 
                       WHOLE_ARCHIVE)
