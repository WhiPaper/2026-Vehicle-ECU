cmake_minimum_required(VERSION 3.22)

set(COMPONENTS main)
set(EXTRA_COMPONENT_DIRS
    "../../../imu"
    "../../../motor"
    "../../../wheel_encoder"
    "../.."
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drive_test)
