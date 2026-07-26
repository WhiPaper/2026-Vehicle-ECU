#pragma once

#include "motor.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    motor_normalized_speed_t left_speed;
    motor_normalized_speed_t right_speed;
} motor_mix_output_t;

typedef struct
{
    motor_normalized_speed_t target;
    motor_normalized_speed_t initial;
    bool boosted;
} motor_start_plan_t;

motor_normalized_speed_t motor_clamp_normalized(int32_t normalized_speed);
motor_normalized_speed_t motor_percent_to_normalized(int32_t speed_percent);
bool motor_needs_start_boost(motor_normalized_speed_t previous_speed,
                             motor_normalized_speed_t requested_speed,
                             motor_normalized_speed_t boost_speed);
motor_normalized_speed_t motor_start_boost_command(motor_normalized_speed_t requested_speed,
                                                   motor_normalized_speed_t boost_speed);
motor_start_plan_t motor_plan_start(motor_normalized_speed_t previous_speed,
                                    motor_normalized_speed_t requested_speed,
                                    motor_normalized_speed_t boost_speed);
motor_mix_output_t motor_mix(motor_normalized_speed_t normalized_throttle,
                             motor_normalized_speed_t normalized_steering);
