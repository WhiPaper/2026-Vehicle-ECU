idf_component_register(
    SRCS
        "test_app_main.c"
        "test_motor_mix.c"
        "../../../motor_mix.c"
    INCLUDE_DIRS
        "../../../include"
        "../../../private_include"
    REQUIRES unity
    WHOLE_ARCHIVE
)
