idf_component_register(
    SRCS "board.c"
    INCLUDE_DIRS "include"
    REQUIRES drive esp_common imu motor wheel_encoder
    PRIV_REQUIRES esp_driver_gpio
)
