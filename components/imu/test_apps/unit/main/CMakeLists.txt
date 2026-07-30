idf_component_register(
    SRCS
        "test_app_main.c"
        "test_imu.c"
    REQUIRES
        imu
        unity
    WHOLE_ARCHIVE
)

target_include_directories(${COMPONENT_LIB} PRIVATE "../../../private_include")
