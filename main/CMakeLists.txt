idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        board
        drive
        wheel_encoder
        imu
        motor
        ros_bridge
)
