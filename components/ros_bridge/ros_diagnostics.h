#pragma once

#include "drive.h"

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <stdbool.h>
#include <stdint.h>

enum
{
    ROS_FAULT_IMU_READ = 1U << 16,
    ROS_FAULT_TIME = 1U << 17,
    ROS_FAULT_IMU_PUBLISH = 1U << 18,
    ROS_FAULT_DRIVE_PUBLISH = 1U << 19,
    ROS_FAULT_DIAGNOSTIC_PUBLISH = 1U << 20,
};

typedef struct
{
    const char* session_state;
    bool time_synchronized;
    bool drive_state_valid;
    const drive_state_t* drive_state;
    uint32_t local_faults;
    int64_t command_age_ms;
    bool imu_calibrated;
} ros_diagnostics_input_t;

bool ros_diagnostics_initialize(diagnostic_msgs__msg__DiagnosticArray* diagnostic);
void ros_diagnostics_update(diagnostic_msgs__msg__DiagnosticArray* diagnostic,
                            const ros_diagnostics_input_t* input);
