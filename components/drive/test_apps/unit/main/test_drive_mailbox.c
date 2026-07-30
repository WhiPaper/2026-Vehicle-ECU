#include "drive_mailbox.h"
#include "unity.h"

TEST_CASE("drive command mailbox overwrites the pending command", "[drive][mailbox]")
{
    drive_mailbox_t mailbox;
    TEST_ASSERT_TRUE(drive_mailbox_init(&mailbox));

    const drive_command_t first = {.left_rpm = 10, .right_rpm = 20};
    const drive_command_t latest = {.left_rpm = 30, .right_rpm = 40};
    TEST_ASSERT_TRUE(drive_mailbox_overwrite(&mailbox, &first));
    TEST_ASSERT_TRUE(drive_mailbox_overwrite(&mailbox, &latest));

    drive_command_t received;
    TEST_ASSERT_TRUE(drive_mailbox_take(&mailbox, &received));
    TEST_ASSERT_EQUAL_FLOAT(latest.left_rpm, received.left_rpm);
    TEST_ASSERT_EQUAL_FLOAT(latest.right_rpm, received.right_rpm);
    TEST_ASSERT_FALSE(drive_mailbox_take(&mailbox, &received));
}

TEST_CASE("drive emergency stop flag is non-blocking and edge consumed", "[drive][mailbox]")
{
    drive_mailbox_t mailbox;
    TEST_ASSERT_TRUE(drive_mailbox_init(&mailbox));
    TEST_ASSERT_FALSE(drive_mailbox_take_stop(&mailbox));

    drive_mailbox_request_stop(&mailbox);
    TEST_ASSERT_TRUE(drive_mailbox_take_stop(&mailbox));
    TEST_ASSERT_FALSE(drive_mailbox_take_stop(&mailbox));
}
