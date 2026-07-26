cmake_minimum_required(VERSION 3.22)

set(COMPONENTS main)
set(EXTRA_COMPONENT_DIRS
    "../.."
    "../../../drive"
    "../../../imu"
    "../../../motor"
    "../../../wheel_encoder"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ros_bridge_test)
