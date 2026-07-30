#include "ros_session_policy.h"
#include "unity.h"

TEST_CASE("sensor unavailable does not count as transport publish failure", "[ros_bridge][session]")
{
    uint8_t failures = 2;
    failures = ros_publish_failure_count(ROS_PUBLISH_SKIPPED, failures);

    TEST_ASSERT_EQUAL_UINT8(0, failures);
    TEST_ASSERT_FALSE(ros_session_recovery_required(failures, 0, 0, 3));
}

TEST_CASE("RCL publish failures trigger recovery at configured limit", "[ros_bridge][session]")
{
    uint8_t failures = 0;
    failures = ros_publish_failure_count(ROS_PUBLISH_FAILED, failures);
    failures = ros_publish_failure_count(ROS_PUBLISH_FAILED, failures);
    TEST_ASSERT_FALSE(ros_session_recovery_required(failures, 0, 0, 3));

    failures = ros_publish_failure_count(ROS_PUBLISH_FAILED, failures);
    TEST_ASSERT_TRUE(ros_session_recovery_required(failures, 0, 0, 3));
}

TEST_CASE("publish failure count saturates instead of wrapping", "[ros_bridge][session]")
{
    TEST_ASSERT_EQUAL_UINT8(UINT8_MAX, ros_publish_failure_count(ROS_PUBLISH_FAILED, UINT8_MAX));
}
