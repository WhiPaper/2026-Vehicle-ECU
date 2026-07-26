#include "imu_math.h"

#include <stddef.h>

#define STANDARD_GRAVITY_MPS2 9.80665f
#define DEG_TO_RAD 0.01745329251994329577f

static int16_t read_be_i16(const uint8_t* data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

void imu_convert_raw(const uint8_t raw[14], imu_sample_t* sample)
{
    if (raw == NULL || sample == NULL)
    {
        return;
    }

    for (size_t axis = 0; axis < 3; ++axis)
    {
        const int16_t accel = read_be_i16(&raw[axis * 2]);
        const int16_t gyro = read_be_i16(&raw[8 + axis * 2]);
        sample->acceleration_mps2[axis] = (float)accel * STANDARD_GRAVITY_MPS2 / 8192.0f;
        sample->angular_velocity_rps[axis] = (float)gyro * DEG_TO_RAD / 65.5f;
    }
    sample->temperature_c = (float)read_be_i16(&raw[6]) / 340.0f + 36.53f;
}
