idf_component_register(
    SRCS
        "main.cc"
        "lodepng.cc"
        "fruits_model_data.cc"
    REQUIRES esp_hw_support nvs_flash log esp_timer esp_driver_gpio esp_driver_spi
    INCLUDE_DIRS ".")

    