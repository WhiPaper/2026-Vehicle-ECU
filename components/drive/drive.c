#include "drive.h"

#include "drive_mailbox.h"
#include "drive_math.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "wheel_encoder.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRIVE_TAG "drive"
#define PI 3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f
#define RPM_TO_RAD_PER_SEC (TWO_PI / 60.0f)
#define DRIVE_TASK_STACK 4096
#define DRIVE_TASK_PRIORITY 8
#define STALL_MIN_TARGET_RPM 5.0f
#define MAX_CONTROL_PERIOD_MULTIPLIER 5.0f

static drive_config_t s_config;
static drive_state_t s_state;
static drive_state_t s_snapshot;
static drive_pid_state_t s_pid[2];
static drive_mailbox_t s_mailbox;
static portMUX_TYPE s_shared_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task;
static int64_t s_command_deadline_us;
static int64_t s_last_motion_us[WHEEL_ENCODER_COUNT];
static uint32_t s_external_faults;
static bool s_initialized;

static void reset_pid(void) { memset(s_pid, 0, sizeof(s_pid)); }

static bool config_is_calibrated(void)
{
    return wheel_encoder_cpr() > 0 && s_config.wheel_radius_m > 0 && s_config.track_width_m > 0 &&
           s_config.max_wheel_rpm > 0;
}

static bool config_is_valid(const drive_config_t* config)
{
    return config != NULL && config->control_period_ms > 0 && config->control_period_ms <= 20 &&
           config->command_timeout_ms > 0 && config->stall_timeout_ms > 0 &&
           isfinite(config->wheel_radius_m) && isfinite(config->track_width_m) &&
           isfinite(config->max_wheel_rpm) && isfinite(config->pid_kp) &&
           isfinite(config->pid_ki) && isfinite(config->pid_kd) && config->wheel_radius_m >= 0 &&
           config->track_width_m >= 0 && config->max_wheel_rpm >= 0 && config->pid_kp >= 0 &&
           config->pid_ki >= 0 && config->pid_kd >= 0;
}

static void publish_snapshot(void)
{
    taskENTER_CRITICAL(&s_shared_lock);
    s_snapshot = s_state;
    taskEXIT_CRITICAL(&s_shared_lock);
}

static void stop_state(uint32_t fault)
{
    s_state.target_rpm[0] = 0;
    s_state.target_rpm[1] = 0;
    s_state.command_active = false;
    s_state.faults |= fault;
    s_command_deadline_us = 0;
    reset_pid();
    (void)motor_stop();
}

static int64_t stall_timeout_us(float target_rpm)
{
    return drive_calculate_stall_timeout_us(target_rpm, wheel_encoder_cpr(),
                                            s_config.stall_timeout_ms);
}

static size_t encoder_side(size_t encoder)
{
    return (encoder == WHEEL_ENCODER_FRONT_LEFT || encoder == WHEEL_ENCODER_REAR_LEFT) ? 0 : 1;
}

static void update_odometry(float left_rpm, float right_rpm, float dt_s)
{
    if (s_config.wheel_radius_m <= 0 || s_config.track_width_m <= 0)
    {
        s_state.linear_velocity_mps = 0;
        s_state.angular_velocity_rps = 0;
        return;
    }

    const float left_omega = left_rpm * RPM_TO_RAD_PER_SEC;
    const float right_omega = right_rpm * RPM_TO_RAD_PER_SEC;
    const float left_velocity = left_omega * s_config.wheel_radius_m;
    const float right_velocity = right_omega * s_config.wheel_radius_m;
    const float linear = (left_velocity + right_velocity) * 0.5f;
    const float angular = (right_velocity - left_velocity) / s_config.track_width_m;

    s_state.linear_velocity_mps = linear;
    s_state.angular_velocity_rps = angular;
    s_state.yaw_rad += angular * dt_s;
    while (s_state.yaw_rad > PI)
    {
        s_state.yaw_rad -= TWO_PI;
    }
    while (s_state.yaw_rad < -PI)
    {
        s_state.yaw_rad += TWO_PI;
    }
    s_state.x_m += linear * cosf(s_state.yaw_rad) * dt_s;
    s_state.y_m += linear * sinf(s_state.yaw_rad) * dt_s;
}

static uint32_t external_faults(void)
{
    taskENTER_CRITICAL(&s_shared_lock);
    const uint32_t faults = s_external_faults;
    taskEXIT_CRITICAL(&s_shared_lock);
    return faults;
}

static void apply_command(const drive_command_t* command)
{
    float left_rpm = command->left_rpm;
    float right_rpm = command->right_rpm;
    const float peak = fmaxf(fabsf(left_rpm), fabsf(right_rpm));
    if (peak > s_config.max_wheel_rpm)
    {
        const float scale = s_config.max_wheel_rpm / peak;
        left_rpm *= scale;
        right_rpm *= scale;
    }

    s_state.target_rpm[0] = left_rpm;
    s_state.target_rpm[1] = right_rpm;
    s_state.command_active = left_rpm != 0 || right_rpm != 0;
    s_state.faults &= ~(DRIVE_FAULT_COMMAND_TIMEOUT | DRIVE_FAULT_STALL);
    const int64_t now = esp_timer_get_time();
    s_command_deadline_us = now + (int64_t)s_config.command_timeout_ms * 1000;
    if (!s_state.command_active)
    {
        reset_pid();
        (void)motor_stop();
    }
    else
    {
        for (size_t i = 0; i < WHEEL_ENCODER_COUNT; ++i)
        {
            s_last_motion_us[i] = now;
        }
    }
}

static void drive_task(void* argument)
{
    (void)argument;
    int64_t previous_us = esp_timer_get_time();
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(s_config.control_period_ms);
    const float nominal_dt_s = (float)s_config.control_period_ms / 1000.0f;

    for (;;)
    {
        const int64_t now = esp_timer_get_time();
        float dt_s = (float)(now - previous_us) / 1000000.0f;
        previous_us = now;
        const bool timing_discontinuity =
            dt_s < nominal_dt_s * 0.25f || dt_s > nominal_dt_s * MAX_CONTROL_PERIOD_MULTIPLIER;
        if (timing_discontinuity)
        {
            dt_s = nominal_dt_s;
            reset_pid();
        }

        if (drive_mailbox_take_stop(&s_mailbox))
        {
            drive_mailbox_clear(&s_mailbox);
            stop_state(DRIVE_FAULT_NONE);
        }
        else
        {
            drive_command_t command;
            if (drive_mailbox_take(&s_mailbox, &command))
            {
                apply_command(&command);
            }
        }

        const uint32_t reported_faults = external_faults();
        if (reported_faults != 0)
        {
            stop_state(reported_faults);
        }

        wheel_encoder_sample_t encoders[WHEEL_ENCODER_COUNT];
        const esp_err_t encoder_result = wheel_encoder_sample_all(encoders);
        if (encoder_result != ESP_OK)
        {
            s_state.encoder_valid = false;
            s_state.ready = false;
            s_state.timestamp_us = now;
            stop_state(DRIVE_FAULT_ENCODER);
            publish_snapshot();
            if (xTaskDelayUntil(&last_wake, period_ticks) == pdFALSE)
            {
                last_wake = xTaskGetTickCount();
            }
            continue;
        }

        s_state.ready = true;
        s_state.encoder_valid = true;
        s_state.faults &= ~DRIVE_FAULT_ENCODER;
        for (size_t i = 0; i < WHEEL_ENCODER_COUNT; ++i)
        {
            const size_t side = encoder_side(i);
            if (encoders[i].delta_count != 0 ||
                fabsf(s_state.target_rpm[side]) < STALL_MIN_TARGET_RPM)
            {
                s_last_motion_us[i] = now;
            }
        }
        s_state.measured_rpm[0] =
            (encoders[WHEEL_ENCODER_FRONT_LEFT].rpm +
             encoders[WHEEL_ENCODER_REAR_LEFT].rpm) *
            0.5f;
        s_state.measured_rpm[1] =
            (encoders[WHEEL_ENCODER_FRONT_RIGHT].rpm +
             encoders[WHEEL_ENCODER_REAR_RIGHT].rpm) *
            0.5f;
        s_state.encoder_count[0] =
            (encoders[WHEEL_ENCODER_FRONT_LEFT].count +
             encoders[WHEEL_ENCODER_REAR_LEFT].count) /
            2;
        s_state.encoder_count[1] =
            (encoders[WHEEL_ENCODER_FRONT_RIGHT].count +
             encoders[WHEEL_ENCODER_REAR_RIGHT].count) /
            2;
        if (wheel_encoder_cpr() > 0)
        {
            for (size_t side = 0; side < 2; ++side)
            {
                s_state.wheel_position_rad[side] =
                    (float)s_state.encoder_count[side] * TWO_PI / (float)wheel_encoder_cpr();
            }
        }
        update_odometry(s_state.measured_rpm[0], s_state.measured_rpm[1], dt_s);
        s_state.timestamp_us = now;

        if (s_state.command_active && now > s_command_deadline_us)
        {
            stop_state(DRIVE_FAULT_COMMAND_TIMEOUT);
        }
        else if (s_state.command_active &&
                 ((((now - s_last_motion_us[WHEEL_ENCODER_FRONT_LEFT] >
                         stall_timeout_us(s_state.target_rpm[0])) ||
                    (now - s_last_motion_us[WHEEL_ENCODER_REAR_LEFT] >
                         stall_timeout_us(s_state.target_rpm[0]))) &&
                   fabsf(s_state.target_rpm[0]) >= STALL_MIN_TARGET_RPM) ||
                  (((now - s_last_motion_us[WHEEL_ENCODER_FRONT_RIGHT] >
                         stall_timeout_us(s_state.target_rpm[1])) ||
                    (now - s_last_motion_us[WHEEL_ENCODER_REAR_RIGHT] >
                         stall_timeout_us(s_state.target_rpm[1]))) &&
                   fabsf(s_state.target_rpm[1]) >= STALL_MIN_TARGET_RPM)))
        {
            stop_state(DRIVE_FAULT_STALL);
        }
        else if (s_state.command_active)
        {
            const float left_output = drive_pid_step(
                &s_pid[0], s_state.target_rpm[0], s_state.measured_rpm[0], dt_s, s_config.pid_kp,
                s_config.pid_ki, s_config.pid_kd, MOTOR_NORMALIZED_SPEED_MAX);
            const float right_output = drive_pid_step(
                &s_pid[1], s_state.target_rpm[1], s_state.measured_rpm[1], dt_s, s_config.pid_kp,
                s_config.pid_ki, s_config.pid_kd, MOTOR_NORMALIZED_SPEED_MAX);
            if (motor_set_tank_direct_normalized((motor_normalized_speed_t)left_output,
                                                 (motor_normalized_speed_t)right_output) != ESP_OK)
            {
                stop_state(DRIVE_FAULT_MOTOR);
            }
            else
            {
                s_state.faults &= ~DRIVE_FAULT_MOTOR;
            }
        }

        publish_snapshot();
        if (xTaskDelayUntil(&last_wake, period_ticks) == pdFALSE)
        {
            last_wake = xTaskGetTickCount();
        }
    }
}

esp_err_t drive_init(const drive_config_t* config)
{
    ESP_RETURN_ON_FALSE(config_is_valid(config), ESP_ERR_INVALID_ARG, DRIVE_TAG,
                        "Invalid controller configuration");
    if (s_initialized)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(drive_mailbox_init(&s_mailbox), ESP_ERR_NO_MEM, DRIVE_TAG,
                        "Could not create command mailbox");
    s_config = *config;
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    if (!config_is_calibrated())
    {
        s_state.faults = DRIVE_FAULT_NOT_CALIBRATED;
    }
    const int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < WHEEL_ENCODER_COUNT; ++i)
    {
        s_last_motion_us[i] = now;
    }
    s_initialized = true;
    publish_snapshot();
    return ESP_OK;
}

esp_err_t drive_start(void)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Controller is not initialized");
    if (s_task != NULL)
    {
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(xTaskCreate(drive_task, "drive", DRIVE_TASK_STACK, NULL,
                                    DRIVE_TASK_PRIORITY, &s_task) == pdPASS,
                        ESP_ERR_NO_MEM, DRIVE_TAG, "Could not create control task");
    return ESP_OK;
}

esp_err_t drive_set_rpm(float left_rpm, float right_rpm)
{
    ESP_RETURN_ON_FALSE(s_initialized && s_task != NULL, ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Controller is not running");
    ESP_RETURN_ON_FALSE(config_is_calibrated(), ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Vehicle calibration is incomplete");
    ESP_RETURN_ON_FALSE(isfinite(left_rpm) && isfinite(right_rpm), ESP_ERR_INVALID_ARG, DRIVE_TAG,
                        "Wheel targets must be finite");
    const drive_command_t command = {
        .left_rpm = left_rpm,
        .right_rpm = right_rpm,
    };
    return drive_mailbox_overwrite(&s_mailbox, &command) ? ESP_OK : ESP_FAIL;
}

esp_err_t drive_set_twist(float linear_mps, float angular_rps)
{
    ESP_RETURN_ON_FALSE(s_initialized && s_task != NULL, ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Controller is not running");
    ESP_RETURN_ON_FALSE(s_config.wheel_radius_m > 0 && s_config.track_width_m > 0,
                        ESP_ERR_INVALID_STATE, DRIVE_TAG, "Vehicle geometry is not configured");
    ESP_RETURN_ON_FALSE(isfinite(linear_mps) && isfinite(angular_rps), ESP_ERR_INVALID_ARG,
                        DRIVE_TAG, "Twist command must be finite");
    const float left_mps = linear_mps - angular_rps * s_config.track_width_m * 0.5f;
    const float right_mps = linear_mps + angular_rps * s_config.track_width_m * 0.5f;
    const float meters_per_minute_per_rpm = TWO_PI * s_config.wheel_radius_m;
    return drive_set_rpm(left_mps * 60.0f / meters_per_minute_per_rpm,
                         right_mps * 60.0f / meters_per_minute_per_rpm);
}

esp_err_t drive_stop(void)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Controller is not initialized");
    drive_mailbox_request_stop(&s_mailbox);
    return ESP_OK;
}

esp_err_t drive_get_state(drive_state_t* state)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Controller is not initialized");
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, DRIVE_TAG, "State output is required");
    taskENTER_CRITICAL(&s_shared_lock);
    *state = s_snapshot;
    taskEXIT_CRITICAL(&s_shared_lock);
    return ESP_OK;
}

esp_err_t drive_report_fault(uint32_t fault)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, DRIVE_TAG,
                        "Controller is not initialized");
    ESP_RETURN_ON_FALSE((fault & ~(DRIVE_FAULT_ENCODER | DRIVE_FAULT_MOTOR)) == 0,
                        ESP_ERR_INVALID_ARG, DRIVE_TAG, "Unsupported external fault");
    taskENTER_CRITICAL(&s_shared_lock);
    s_external_faults |= fault;
    s_snapshot.faults |= fault;
    taskEXIT_CRITICAL(&s_shared_lock);
    drive_mailbox_request_stop(&s_mailbox);
    return ESP_OK;
}
