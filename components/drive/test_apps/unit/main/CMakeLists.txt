idf_component_register(
    SRCS
        "test_app_main.c"
        "test_drive_mailbox.c"
        "test_drive_math.c"
    REQUIRES
        drive
        imu
        unity
        wheel_encoder
    WHOLE_ARCHIVE
)

target_include_directories(${COMPONENT_LIB} PRIVATE
    "../../../private_include"
    "../../../../imu/private_include"
    "../../../../wheel_encoder/private_include"
)
