#include "ros_messages.h"

#include "ros_diagnostics.h"

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>

#include <math.h>
#include <stddef.h>

#define RPM_TO_RAD_PER_SECOND 0.10471975511965977

bool ros_messages_initialize(ros_messages_t* messages)
{
    messages->initialized = true;
    if (!geometry_msgs__msg__Twist__init(&messages->command) ||
        !sensor_msgs__msg__Imu__init(&messages->imu) ||
        !sensor_msgs__msg__JointState__init(&messages->joints) ||
        !nav_msgs__msg__Odometry__init(&messages->odom) ||
        !diagnostic_msgs__msg__DiagnosticArray__init(&messages->diagnostic))
    {
        return false;
    }
    if (!rosidl_runtime_c__String__assign(&messages->imu.header.frame_id, "imu_link") ||
        !rosidl_runtime_c__String__assign(&messages->odom.header.frame_id, "odom") ||
        !rosidl_runtime_c__String__assign(&messages->odom.child_frame_id, "base_link") ||
        !rosidl_runtime_c__String__Sequence__init(&messages->joints.name, 2) ||
        !rosidl_runtime_c__double__Sequence__init(&messages->joints.position, 2) ||
        !rosidl_runtime_c__double__Sequence__init(&messages->joints.velocity, 2) ||
        !rosidl_runtime_c__String__assign(&messages->joints.name.data[0], "left_wheel_joint") ||
        !rosidl_runtime_c__String__assign(&messages->joints.name.data[1], "right_wheel_joint") ||
        !rosidl_runtime_c__String__assign(&messages->joints.header.frame_id, "base_link") ||
        !ros_diagnostics_initialize(&messages->diagnostic))
    {
        return false;
    }

    messages->imu.orientation_covariance[0] = -1.0;
    for (size_t axis = 0; axis < 3; ++axis)
    {
        const size_t diagonal = axis * 3 + axis;
        messages->imu.angular_velocity_covariance[diagonal] =
            CONFIG_ROS_IMU_ANGULAR_VARIANCE_MICRO / 1000000.0;
        messages->imu.linear_acceleration_covariance[diagonal] =
            CONFIG_ROS_IMU_ACCEL_VARIANCE_MICRO / 1000000.0;
    }
    for (size_t i = 0; i < 36; ++i)
    {
        messages->odom.pose.covariance[i] = 1000000.0;
        messages->odom.twist.covariance[i] = 1000000.0;
    }
    messages->odom.pose.covariance[0] = CONFIG_ROS_ODOM_LINEAR_VARIANCE_MICRO / 1000000.0;
    messages->odom.pose.covariance[7] = CONFIG_ROS_ODOM_LINEAR_VARIANCE_MICRO / 1000000.0;
    messages->odom.pose.covariance[35] = CONFIG_ROS_ODOM_ANGULAR_VARIANCE_MICRO / 1000000.0;
    messages->odom.twist.covariance[0] = CONFIG_ROS_ODOM_LINEAR_VARIANCE_MICRO / 1000000.0;
    messages->odom.twist.covariance[35] = CONFIG_ROS_ODOM_ANGULAR_VARIANCE_MICRO / 1000000.0;
    return true;
}

void ros_messages_finalize(ros_messages_t* messages)
{
    if (!messages->initialized)
    {
        return;
    }
    diagnostic_msgs__msg__DiagnosticArray__fini(&messages->diagnostic);
    nav_msgs__msg__Odometry__fini(&messages->odom);
    sensor_msgs__msg__JointState__fini(&messages->joints);
    sensor_msgs__msg__Imu__fini(&messages->imu);
    geometry_msgs__msg__Twist__fini(&messages->command);
    messages->initialized = false;
}

void ros_messages_map_imu(ros_messages_t* messages, const imu_sample_t* sample,
                          builtin_interfaces__msg__Time stamp)
{
    messages->imu.header.stamp = stamp;
    messages->imu.linear_acceleration.x = sample->acceleration_mps2[0];
    messages->imu.linear_acceleration.y = sample->acceleration_mps2[1];
    messages->imu.linear_acceleration.z = sample->acceleration_mps2[2];
    messages->imu.angular_velocity.x = sample->angular_velocity_rps[0];
    messages->imu.angular_velocity.y = sample->angular_velocity_rps[1];
    messages->imu.angular_velocity.z = sample->angular_velocity_rps[2];
}

void ros_messages_map_drive(ros_messages_t* messages, const drive_state_t* state,
                            builtin_interfaces__msg__Time stamp)
{
    messages->joints.header.stamp = stamp;
    messages->joints.position.data[0] = state->wheel_position_rad[0];
    messages->joints.position.data[1] = state->wheel_position_rad[1];
    messages->joints.velocity.data[0] = state->measured_rpm[0] * RPM_TO_RAD_PER_SECOND;
    messages->joints.velocity.data[1] = state->measured_rpm[1] * RPM_TO_RAD_PER_SECOND;

    messages->odom.header.stamp = stamp;
    messages->odom.pose.pose.position.x = state->x_m;
    messages->odom.pose.pose.position.y = state->y_m;
    messages->odom.pose.pose.orientation.z = sinf(state->yaw_rad * 0.5f);
    messages->odom.pose.pose.orientation.w = cosf(state->yaw_rad * 0.5f);
    messages->odom.twist.twist.linear.x = state->linear_velocity_mps;
    messages->odom.twist.twist.angular.z = state->angular_velocity_rps;
}
