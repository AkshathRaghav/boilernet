set(srcs "read_eth.c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "." 
                       REQUIRES esp_eth esp_netif esp_hw_support esp_wifi nvs_flash lwip log
                       WHOLE_ARCHIVE)
