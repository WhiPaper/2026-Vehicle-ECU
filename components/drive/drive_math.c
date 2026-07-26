#include "drive_math.h"

#include <math.h>
#include <stddef.h>

#define STALL_MIN_TARGET_RPM 5.0f
#define STALL_REQUIRED_PULSES 2.0f

static float clampf(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float drive_pid_step(drive_pid_state_t* state, float target, float measured, float dt_s, float kp,
                     float ki, float kd, float output_limit)
{
    if (state == NULL || !isfinite(target) || !isfinite(measured) || !isfinite(dt_s) ||
        !isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(output_limit) || dt_s <= 0 ||
        output_limit <= 0)
    {
        return 0;
    }
    const float error = target - measured;
    const float derivative = state->initialized ? (error - state->previous_error) / dt_s : 0;
    const float candidate_integral = state->integral + error * dt_s;
    const float unconstrained = kp * error + ki * candidate_integral + kd * derivative;
    const float output = clampf(unconstrained, -output_limit, output_limit);
    if (unconstrained == output || (unconstrained > output && error < 0) ||
        (unconstrained < output && error > 0))
    {
        state->integral = candidate_integral;
    }
    state->previous_error = error;
    state->initialized = true;
    return output;
}

int64_t drive_calculate_stall_timeout_us(float target_rpm, uint32_t cpr,
                                         uint32_t configured_timeout_ms)
{
    const int64_t timeout_us = (int64_t)configured_timeout_ms * 1000;
    if (!isfinite(target_rpm) || cpr == 0 || fabsf(target_rpm) < STALL_MIN_TARGET_RPM)
    {
        return timeout_us;
    }

    const float pulse_rate_hz = fabsf(target_rpm) * (float)cpr / 60.0f;
    const int64_t resolution_timeout_us =
        (int64_t)ceilf(STALL_REQUIRED_PULSES * 1000000.0f / pulse_rate_hz);
    return resolution_timeout_us > timeout_us ? resolution_timeout_us : timeout_us;
}
