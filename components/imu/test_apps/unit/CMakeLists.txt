cmake_minimum_required(VERSION 3.22)

set(COMPONENTS main)
set(EXTRA_COMPONENT_DIRS "../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(imu_test)
