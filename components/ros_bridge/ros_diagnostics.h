#pragma once

#include "drive.h"
#include "imu.h"

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
    ROS_FAULT_DRIVE_DATA = 1U << 21,
};

typedef struct
{
    const char* session_state;
    bool time_synchronized;
    bool drive_state_valid;
    const drive_state_t* drive_state;
    uint32_t local_faults;
    int64_t command_age_ms;
    const imu_snapshot_t* imu_snapshot;
    int64_t imu_age_ms;
    int64_t drive_age_ms;
    const char* last_entity_stage;
    int32_t last_rcl_error;
    const char* firmware_version;
    const char* build_id;
    const char* idf_version;
} ros_diagnostics_input_t;

bool ros_diagnostics_initialize(diagnostic_msgs__msg__DiagnosticArray* diagnostic);
void ros_diagnostics_update(diagnostic_msgs__msg__DiagnosticArray* diagnostic,
                            const ros_diagnostics_input_t* input);
