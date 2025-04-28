idf_component_register(
    SRCS
        "main.cc"
        "lodepng.cc"
        "cats_model_data.cc"
    REQUIRES esp_hw_support nvs_flash log esp_timer
    INCLUDE_DIRS ".")

    