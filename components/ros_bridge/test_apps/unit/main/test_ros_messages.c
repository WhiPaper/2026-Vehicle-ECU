#include "ros_messages.h"

#include "unity.h"

#include <math.h>
#include <string.h>

static ros_messages_t initialize_messages(void)
{
    ros_messages_t messages = {0};
    TEST_ASSERT_TRUE(ros_messages_initialize(&messages));
    return messages;
}

TEST_CASE("ROS messages initialize fixed metadata and covariance", "[ros_bridge]")
{
    ros_messages_t messages = initialize_messages();

    TEST_ASSERT_EQUAL_STRING("imu_link", messages.imu.header.frame_id.data);
    TEST_ASSERT_EQUAL_STRING("odom", messages.odom.header.frame_id.data);
    TEST_ASSERT_EQUAL_STRING("base_link", messages.odom.child_frame_id.data);
    TEST_ASSERT_EQUAL_STRING("left_wheel_joint", messages.joints.name.data[0].data);
    TEST_ASSERT_EQUAL_STRING("right_wheel_joint", messages.joints.name.data[1].data);
    TEST_ASSERT_EQUAL_DOUBLE(-1.0, messages.imu.orientation_covariance[0]);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0.0, messages.imu.angular_velocity_covariance[0]);
    TEST_ASSERT_GREATER_THAN_DOUBLE(0.0, messages.odom.twist.covariance[0]);

    ros_messages_finalize(&messages);
    TEST_ASSERT_FALSE(messages.initialized);
}

TEST_CASE("IMU mapper copies SI samples and timestamp", "[ros_bridge]")
{
    ros_messages_t messages = initialize_messages();
    const imu_sample_t sample = {
        .acceleration_mps2 = {1.0f, -2.0f, 9.5f},
        .angular_velocity_rps = {0.1f, -0.2f, 0.3f},
        .valid = true,
    };
    const builtin_interfaces__msg__Time stamp = {
        .sec = 123,
        .nanosec = 456,
    };

    ros_messages_map_imu(&messages, &sample, stamp);

    TEST_ASSERT_EQUAL_INT32(123, messages.imu.header.stamp.sec);
    TEST_ASSERT_EQUAL_UINT32(456, messages.imu.header.stamp.nanosec);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, messages.imu.linear_acceleration.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.2f, messages.imu.angular_velocity.y);

    ros_messages_finalize(&messages);
}

TEST_CASE("Drive mapper converts wheel state and planar pose", "[ros_bridge]")
{
    ros_messages_t messages = initialize_messages();
    const drive_state_t state = {
        .measured_rpm = {60.0f, -30.0f},
        .wheel_position_rad = {1.25f, -2.5f},
        .x_m = 2.0f,
        .y_m = -1.0f,
        .yaw_rad = 1.0f,
        .linear_velocity_mps = 0.4f,
        .angular_velocity_rps = -0.5f,
    };
    const builtin_interfaces__msg__Time stamp = {
        .sec = 10,
        .nanosec = 20,
    };

    ros_messages_map_drive(&messages, &state, stamp);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.25f, messages.joints.position.data[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.0 * M_PI, messages.joints.velocity.data[0]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -M_PI, messages.joints.velocity.data[1]);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, sin(0.5), messages.odom.pose.pose.orientation.z);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, cos(0.5), messages.odom.pose.pose.orientation.w);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.4, messages.odom.twist.twist.linear.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, -0.5, messages.odom.twist.twist.angular.z);

    ros_messages_finalize(&messages);
}
