#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float integral;
    float previous_error;
    bool initialized;
} drive_pid_state_t;

float drive_pid_step(drive_pid_state_t* state, float target, float measured, float dt_s, float kp,
                     float ki, float kd, float output_limit);
int64_t drive_calculate_stall_timeout_us(float target_rpm, uint32_t cpr,
                                         uint32_t configured_timeout_ms);
