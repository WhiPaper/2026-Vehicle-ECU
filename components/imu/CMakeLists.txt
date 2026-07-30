idf_component_register(
    SRCS
        "imu.c"
        "imu_math.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_common
    PRIV_REQUIRES esp_driver_i2c esp_timer
)
