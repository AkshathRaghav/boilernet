idf_component_register(
    SRCS
        "main.cc"
        "dogs_model_data.cc"
        "lodepng.cc"
    REQUIRES esp_driver_uart esp_hw_support nvs_flash log esp_timer
    INCLUDE_DIRS ".")
