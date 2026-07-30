#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ROS_PUBLISH_OK,
    ROS_PUBLISH_SKIPPED,
    ROS_PUBLISH_FAILED,
} ros_publish_result_t;

uint8_t ros_publish_failure_count(ros_publish_result_t result, uint8_t current_count);
bool ros_session_recovery_required(uint8_t imu_failures, uint8_t drive_failures,
                                   uint8_t diagnostic_failures, uint8_t failure_limit);
