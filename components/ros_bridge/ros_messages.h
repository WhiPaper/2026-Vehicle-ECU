#pragma once

#include "drive.h"
#include "imu.h"

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <stdbool.h>

typedef struct
{
    geometry_msgs__msg__Twist command;
    sensor_msgs__msg__Imu imu;
    sensor_msgs__msg__JointState joints;
    nav_msgs__msg__Odometry odom;
    diagnostic_msgs__msg__DiagnosticArray diagnostic;
    bool initialized;
} ros_messages_t;

bool ros_messages_initialize(ros_messages_t* messages);
void ros_messages_finalize(ros_messages_t* messages);
void ros_messages_map_imu(ros_messages_t* messages, const imu_sample_t* sample,
                          builtin_interfaces__msg__Time stamp);
void ros_messages_map_drive(ros_messages_t* messages, const drive_state_t* state,
                            builtin_interfaces__msg__Time stamp);
