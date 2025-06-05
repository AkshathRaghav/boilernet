idf_component_register(
    SRCS
        "main.cc"
        "lodepng.cc"
        "stationary_model_data.cc" # Replace here 
    REQUIRES esp_hw_support nvs_flash log esp_timer esp_driver_gpio esp_driver_spi esp_driver_uart
    INCLUDE_DIRS ".")

    