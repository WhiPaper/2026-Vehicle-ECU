idf_component_register(
    SRCS
        "test_app_main.c"
        "test_ros_messages.c"
    REQUIRES
        ros_bridge
        unity
    WHOLE_ARCHIVE
)

target_include_directories(${COMPONENT_LIB} PRIVATE "../../..")
