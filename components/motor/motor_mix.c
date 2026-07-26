#include "motor_mix.h"

#include "motor.h"

motor_normalized_speed_t motor_clamp_normalized(int32_t normalized_speed)
{
    if (normalized_speed > MOTOR_NORMALIZED_SPEED_MAX)
    {
        return MOTOR_NORMALIZED_SPEED_MAX;
    }
    if (normalized_speed < -MOTOR_NORMALIZED_SPEED_MAX)
    {
        return -MOTOR_NORMALIZED_SPEED_MAX;
    }
    return (motor_normalized_speed_t)normalized_speed;
}

motor_normalized_speed_t motor_percent_to_normalized(int32_t speed_percent)
{
    if (speed_percent > MOTOR_SPEED_PERCENT_MAX)
    {
        speed_percent = MOTOR_SPEED_PERCENT_MAX;
    }
    else if (speed_percent < -MOTOR_SPEED_PERCENT_MAX)
    {
        speed_percent = -MOTOR_SPEED_PERCENT_MAX;
    }

    return (motor_normalized_speed_t)(speed_percent * MOTOR_NORMALIZED_SPEED_MAX /
                                      MOTOR_SPEED_PERCENT_MAX);
}

bool motor_needs_start_boost(motor_normalized_speed_t previous_speed,
                             motor_normalized_speed_t requested_speed,
                             motor_normalized_speed_t boost_speed)
{
    if (requested_speed == 0 || boost_speed <= 0)
    {
        return false;
    }

    const bool starting = previous_speed == 0;
    const bool reversing =
        (previous_speed < 0 && requested_speed > 0) || (previous_speed > 0 && requested_speed < 0);
    const motor_normalized_speed_t requested_magnitude =
        requested_speed < 0 ? (motor_normalized_speed_t)-requested_speed : requested_speed;
    return (starting || reversing) && requested_magnitude < boost_speed;
}

motor_normalized_speed_t motor_start_boost_command(motor_normalized_speed_t requested_speed,
                                                   motor_normalized_speed_t boost_speed)
{
    const motor_normalized_speed_t clamped_boost = motor_clamp_normalized(boost_speed);
    if (requested_speed > 0)
    {
        return clamped_boost;
    }
    if (requested_speed < 0)
    {
        return (motor_normalized_speed_t)-clamped_boost;
    }
    return 0;
}

motor_start_plan_t motor_plan_start(motor_normalized_speed_t previous_speed,
                                    motor_normalized_speed_t requested_speed,
                                    motor_normalized_speed_t boost_speed)
{
    const motor_normalized_speed_t target = motor_clamp_normalized(requested_speed);
    const bool boosted = motor_needs_start_boost(previous_speed, target, boost_speed);

    return (motor_start_plan_t){
        .target = target,
        .initial = boosted ? motor_start_boost_command(target, boost_speed) : target,
        .boosted = boosted,
    };
}

motor_mix_output_t motor_mix(motor_normalized_speed_t normalized_throttle,
                             motor_normalized_speed_t normalized_steering)
{
    const motor_mix_output_t output = {
        .left_speed = motor_clamp_normalized((int32_t)normalized_throttle + normalized_steering),
        .right_speed = motor_clamp_normalized((int32_t)normalized_throttle - normalized_steering),
    };
    return output;
}
