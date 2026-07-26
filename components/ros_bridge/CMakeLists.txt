idf_component_register(
    SRCS
        "ros_serial.c"
        "ros_bridge.c"
        "ros_diagnostics.c"
        "ros_messages.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES
        drive
        esp_common
        imu
        micro_ros
    PRIV_REQUIRES
        esp_driver_uart
        esp_timer
)
