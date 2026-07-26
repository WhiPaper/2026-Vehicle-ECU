#include "drive.h"
#include "drive_math.h"
#include "imu.h"
#include "imu_math.h"
#include "unity.h"
#include "wheel_encoder.h"
#include "wheel_encoder_math.h"
#include <math.h>
#include <string.h>

TEST_CASE("encoder RPM conversion preserves direction", "[encoder][math]")
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 60.0f, wheel_encoder_calculate_rpm(100, 100000, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -30.0f, wheel_encoder_calculate_rpm(-50, 100000, 1000));
    TEST_ASSERT_EQUAL_FLOAT(0, wheel_encoder_calculate_rpm(100, 0, 1000));
    TEST_ASSERT_EQUAL_FLOAT(0, wheel_encoder_calculate_rpm(100, 100000, 0));
}

TEST_CASE("PID applies proportional gain and output saturation", "[drive][pid]")
{
    drive_pid_state_t state = {0};
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, drive_pid_step(&state, 10, 0, 0.02f, 2, 0, 0, 1000));

    memset(&state, 0, sizeof(state));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f,
                             drive_pid_step(&state, 100, 0, 0.02f, 20, 10, 0, 1000));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, state.integral);
}

TEST_CASE("PID rejects non-finite control inputs", "[drive][pid]")
{
    drive_pid_state_t state = {0};
    TEST_ASSERT_EQUAL_FLOAT(0, drive_pid_step(&state, NAN, 0, 0.02f, 2, 0, 0, 1000));
    TEST_ASSERT_FALSE(state.initialized);
}

TEST_CASE("stall timeout accounts for encoder resolution", "[drive][stall]")
{
    TEST_ASSERT_EQUAL_INT32(500000, (int32_t)drive_calculate_stall_timeout_us(100.0f, 1000, 500));
    TEST_ASSERT_EQUAL_INT32(24000000, (int32_t)drive_calculate_stall_timeout_us(5.0f, 1, 500));
    TEST_ASSERT_EQUAL_INT32(500000, (int32_t)drive_calculate_stall_timeout_us(0.0f, 1, 500));
}

TEST_CASE("MPU6050 raw conversion produces SI units", "[imu][math]")
{
    uint8_t raw[14] = {0};
    raw[4] = 0x20; // Z acceleration: 8192 LSB = 1 g at +/-4 g.
    raw[8] = 0x02; // X gyro: 655 LSB = 10 dps at +/-500 dps.
    raw[9] = 0x8F;

    imu_sample_t sample = {0};
    imu_convert_raw(raw, &sample);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.80665f, sample.acceleration_mps2[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1745329f, sample.angular_velocity_rps[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 36.53f, sample.temperature_c);
}
