idf_component_register(
    SRCS
        "ros_serial.c"
        "ros_bridge.c"
        "ros_diagnostics.c"
        "ros_messages.c"
        "ros_session_policy.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES
        drive
        esp_common
        imu
        micro_ros
    PRIV_REQUIRES
        esp_app_format
        esp_driver_uart
        esp_system
        esp_timer
)
