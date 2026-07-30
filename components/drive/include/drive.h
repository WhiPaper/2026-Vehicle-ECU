#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    enum
    {
        DRIVE_FAULT_NONE = 0,
        DRIVE_FAULT_NOT_CALIBRATED = 1U << 0,
        DRIVE_FAULT_COMMAND_TIMEOUT = 1U << 1,
        DRIVE_FAULT_ENCODER = 1U << 2,
        DRIVE_FAULT_STALL = 1U << 3,
        DRIVE_FAULT_MOTOR = 1U << 4,
    };

    typedef struct
    {
        float wheel_radius_m;
        float track_width_m;
        float max_wheel_rpm;
        float pid_kp;
        float pid_ki;
        float pid_kd;
        uint32_t control_period_ms;
        uint32_t command_timeout_ms;
        uint32_t stall_timeout_ms;
    } drive_config_t;

    typedef struct
    {
        float target_rpm[2];
        float measured_rpm[2];
        int64_t encoder_count[2];
        float wheel_position_rad[2];
        float x_m;
        float y_m;
        float yaw_rad;
        float linear_velocity_mps;
        float angular_velocity_rps;
        uint32_t faults;
        int64_t timestamp_us;
        bool command_active;
        bool ready;
        bool encoder_valid;
    } drive_state_t;

    esp_err_t drive_init(const drive_config_t* config);
    esp_err_t drive_start(void);
    esp_err_t drive_set_twist(float linear_mps, float angular_rps);
    esp_err_t drive_set_rpm(float left_rpm, float right_rpm);
    esp_err_t drive_stop(void);
    esp_err_t drive_get_state(drive_state_t* state);
    esp_err_t drive_report_fault(uint32_t fault);

#ifdef __cplusplus
}
#endif
