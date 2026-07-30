#include "ros_session_policy.h"

#include <stdint.h>

uint8_t ros_publish_failure_count(ros_publish_result_t result, uint8_t current_count)
{
    if (result != ROS_PUBLISH_FAILED)
    {
        return 0;
    }
    return current_count < UINT8_MAX ? current_count + 1 : UINT8_MAX;
}

bool ros_session_recovery_required(uint8_t imu_failures, uint8_t drive_failures,
                                   uint8_t diagnostic_failures, uint8_t failure_limit)
{
    return failure_limit > 0 && (imu_failures >= failure_limit || drive_failures >= failure_limit ||
                                 diagnostic_failures >= failure_limit);
}
