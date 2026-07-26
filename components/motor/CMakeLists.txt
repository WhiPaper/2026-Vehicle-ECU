idf_component_register(
    SRCS
        "motor.c"
        "motor_mix.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_common
    PRIV_REQUIRES espressif__bdc_motor esp_driver_gpio
)
