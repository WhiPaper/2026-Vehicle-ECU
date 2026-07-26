#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MOTOR_COUNT 4
#define MOTOR_NORMALIZED_SPEED_MAX 1000
#define MOTOR_SPEED_PERCENT_MAX 100
#define MOTOR_PWM_FREQUENCY_MIN_HZ 1000U
#define MOTOR_PWM_FREQUENCY_MAX_HZ 40000U
/* Backward-compatible name; values are normalized commands, not percentages. */
#define MOTOR_SPEED_MAX MOTOR_NORMALIZED_SPEED_MAX

    typedef int16_t motor_normalized_speed_t;
    typedef int16_t motor_speed_percent_t;

    typedef struct
    {
        int ia_gpio;
        int ib_gpio;
        bool inverted;
    } motor_channel_config_t;

    typedef struct
    {
        motor_channel_config_t channels[MOTOR_COUNT];
        uint32_t pwm_frequency_hz;
    } motor_config_t;

    typedef enum
    {
        MOTOR_FRONT_LEFT,
        MOTOR_FRONT_RIGHT,
        MOTOR_REAR_LEFT,
        MOTOR_REAR_RIGHT,
    } motor_id_t;

    esp_err_t motor_init(const motor_config_t* config);

    /**
     * Normalized speed APIs use [-MOTOR_NORMALIZED_SPEED_MAX,
     * MOTOR_NORMALIZED_SPEED_MAX]. Inputs outside that range are saturated.
     * Percentage APIs use [-100, 100] and also saturate out-of-range inputs.
     *
     * Any hardware write failure triggers a best-effort stop of every motor. A
     * call that starts or reverses a motor may block for the configured startup
     * boost duration.
     */
    esp_err_t motor_set_normalized(motor_id_t motor, motor_normalized_speed_t normalized_speed);
    esp_err_t motor_set_tank_normalized(motor_normalized_speed_t left_normalized_speed,
                                        motor_normalized_speed_t right_normalized_speed);
    /**
     * Apply a tank command without the blocking startup boost. Intended for
     * periodic closed-loop controllers that provide their own startup policy.
     */
    esp_err_t motor_set_tank_direct_normalized(motor_normalized_speed_t left_normalized_speed,
                                               motor_normalized_speed_t right_normalized_speed);
    esp_err_t motor_drive_normalized(motor_normalized_speed_t normalized_throttle,
                                     motor_normalized_speed_t normalized_steering);

    esp_err_t motor_set_percent(motor_id_t motor, motor_speed_percent_t speed_percent);
    esp_err_t motor_set_tank_percent(motor_speed_percent_t left_speed_percent,
                                     motor_speed_percent_t right_speed_percent);
    esp_err_t motor_drive_percent(motor_speed_percent_t throttle_percent,
                                  motor_speed_percent_t steering_percent);

    /*
     * Backward-compatible normalized APIs. New call sites should prefer the
     * unit-explicit *_normalized() or *_percent() names above.
     */
    esp_err_t motor_set(motor_id_t motor, motor_normalized_speed_t normalized_speed);
    esp_err_t motor_set_tank(motor_normalized_speed_t left_normalized_speed,
                             motor_normalized_speed_t right_normalized_speed);
    esp_err_t motor_drive(motor_normalized_speed_t normalized_throttle,
                          motor_normalized_speed_t normalized_steering);
    esp_err_t motor_stop(void);

#ifdef __cplusplus
}
#endif
