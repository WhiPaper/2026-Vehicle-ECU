#include "motor.h"
#include "motor_mix.h"
#include "unity.h"
#include <limits.h>

TEST_CASE("motor speed clamp preserves values inside range", "[motor][mixer]")
{
    TEST_ASSERT_EQUAL_INT16(0, motor_clamp_normalized(0));
    TEST_ASSERT_EQUAL_INT16(425, motor_clamp_normalized(425));
    TEST_ASSERT_EQUAL_INT16(-425, motor_clamp_normalized(-425));
    TEST_ASSERT_EQUAL_INT16(MOTOR_NORMALIZED_SPEED_MAX,
                            motor_clamp_normalized(MOTOR_NORMALIZED_SPEED_MAX));
    TEST_ASSERT_EQUAL_INT16(-MOTOR_NORMALIZED_SPEED_MAX,
                            motor_clamp_normalized(-MOTOR_NORMALIZED_SPEED_MAX));
}

TEST_CASE("motor speed clamp limits values outside range", "[motor][mixer]")
{
    TEST_ASSERT_EQUAL_INT16(MOTOR_NORMALIZED_SPEED_MAX,
                            motor_clamp_normalized(MOTOR_NORMALIZED_SPEED_MAX + 1));
    TEST_ASSERT_EQUAL_INT16(-MOTOR_NORMALIZED_SPEED_MAX,
                            motor_clamp_normalized(-MOTOR_NORMALIZED_SPEED_MAX - 1));
    TEST_ASSERT_EQUAL_INT16(MOTOR_NORMALIZED_SPEED_MAX, motor_clamp_normalized(INT32_MAX));
    TEST_ASSERT_EQUAL_INT16(-MOTOR_NORMALIZED_SPEED_MAX, motor_clamp_normalized(INT32_MIN));
}

TEST_CASE("zero command stops both sides", "[motor][mixer]")
{
    const motor_mix_output_t output = motor_mix(0, 0);

    TEST_ASSERT_EQUAL_INT16(0, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(0, output.right_speed);
}

TEST_CASE("throttle drives both sides equally", "[motor][mixer]")
{
    motor_mix_output_t output = motor_mix(600, 0);
    TEST_ASSERT_EQUAL_INT16(600, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(600, output.right_speed);

    output = motor_mix(-600, 0);
    TEST_ASSERT_EQUAL_INT16(-600, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(-600, output.right_speed);
}

TEST_CASE("steering pivots the vehicle in place", "[motor][mixer]")
{
    motor_mix_output_t output = motor_mix(0, 500);
    TEST_ASSERT_EQUAL_INT16(500, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(-500, output.right_speed);

    output = motor_mix(0, -500);
    TEST_ASSERT_EQUAL_INT16(-500, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(500, output.right_speed);
}

TEST_CASE("mixed drive commands saturate each side safely", "[motor][mixer]")
{
    motor_mix_output_t output = motor_mix(800, 500);
    TEST_ASSERT_EQUAL_INT16(MOTOR_NORMALIZED_SPEED_MAX, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(300, output.right_speed);

    output = motor_mix(-800, -500);
    TEST_ASSERT_EQUAL_INT16(-MOTOR_NORMALIZED_SPEED_MAX, output.left_speed);
    TEST_ASSERT_EQUAL_INT16(-300, output.right_speed);
}

TEST_CASE("startup boost is limited to starts and low-speed reversals", "[motor][boost]")
{
    TEST_ASSERT_TRUE(motor_needs_start_boost(0, 500, 700));
    TEST_ASSERT_TRUE(motor_needs_start_boost(500, -500, 700));
    TEST_ASSERT_FALSE(motor_needs_start_boost(500, 300, 700));
    TEST_ASSERT_FALSE(motor_needs_start_boost(0, 800, 700));
    TEST_ASSERT_FALSE(motor_needs_start_boost(0, 0, 700));
    TEST_ASSERT_FALSE(motor_needs_start_boost(0, 500, 0));
}

TEST_CASE("startup boost preserves requested direction", "[motor][boost]")
{
    TEST_ASSERT_EQUAL_INT16(700, motor_start_boost_command(200, 700));
    TEST_ASSERT_EQUAL_INT16(-700, motor_start_boost_command(-200, 700));
    TEST_ASSERT_EQUAL_INT16(0, motor_start_boost_command(0, 700));
}

TEST_CASE("startup plan contains initial and steady-state commands", "[motor][boost]")
{
    motor_start_plan_t plan = motor_plan_start(0, 500, 700);
    TEST_ASSERT_TRUE(plan.boosted);
    TEST_ASSERT_EQUAL_INT16(700, plan.initial);
    TEST_ASSERT_EQUAL_INT16(500, plan.target);

    plan = motor_plan_start(500, 300, 700);
    TEST_ASSERT_FALSE(plan.boosted);
    TEST_ASSERT_EQUAL_INT16(300, plan.initial);
    TEST_ASSERT_EQUAL_INT16(300, plan.target);

    plan = motor_plan_start(500, -300, 700);
    TEST_ASSERT_TRUE(plan.boosted);
    TEST_ASSERT_EQUAL_INT16(-700, plan.initial);
    TEST_ASSERT_EQUAL_INT16(-300, plan.target);
}

TEST_CASE("motor percent conversion clamps and preserves sign", "[motor][percent]")
{
    TEST_ASSERT_EQUAL_INT16(0, motor_percent_to_normalized(0));
    TEST_ASSERT_EQUAL_INT16(500, motor_percent_to_normalized(50));
    TEST_ASSERT_EQUAL_INT16(1000, motor_percent_to_normalized(100));
    TEST_ASSERT_EQUAL_INT16(-500, motor_percent_to_normalized(-50));
    TEST_ASSERT_EQUAL_INT16(1000, motor_percent_to_normalized(101));
    TEST_ASSERT_EQUAL_INT16(-1000, motor_percent_to_normalized(-101));
}
