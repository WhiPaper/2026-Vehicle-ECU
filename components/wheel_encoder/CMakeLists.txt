idf_component_register(
    SRCS
        "wheel_encoder.cpp"
        "wheel_encoder_math.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_common
    PRIV_REQUIRES espp__encoder esp_driver_gpio esp_timer
)
