#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int i2c_port;
        int sda_gpio;
        int scl_gpio;
        uint8_t address;
        uint16_t calibration_rate_hz;
        uint16_t calibration_samples;
    } imu_config_t;

    typedef struct
    {
        float acceleration_mps2[3];
        float angular_velocity_rps[3];
        float temperature_c;
        int64_t timestamp_us;
        bool valid;
    } imu_sample_t;

    typedef struct
    {
        float acceleration_mps2[3];
        float angular_velocity_rps[3];
    } imu_bias_t;

    typedef enum
    {
        IMU_STATE_STOPPED,
        IMU_STATE_INITIALIZING,
        IMU_STATE_CALIBRATING,
        IMU_STATE_READY,
        IMU_STATE_RECOVERING,
    } imu_state_t;

    typedef struct
    {
        imu_sample_t sample;
        int64_t last_success_us;
        esp_err_t last_error;
        imu_state_t state;
        bool initialized;
        bool calibrated;
        bool valid;
    } imu_snapshot_t;

    esp_err_t imu_start(const imu_config_t* config);
    esp_err_t imu_get_snapshot(imu_snapshot_t* snapshot);
    bool imu_snapshot_is_fresh(const imu_snapshot_t* snapshot, int64_t now_us,
                               uint32_t maximum_age_ms);
    esp_err_t imu_init(const imu_config_t* config);
    esp_err_t imu_read(imu_sample_t* sample);
    esp_err_t imu_calibrate_stationary(void);
    esp_err_t imu_recover(void);
    const imu_bias_t* imu_get_bias(void);
    bool imu_is_calibrated(void);
    const char* imu_state_name(imu_state_t state);

#ifdef __cplusplus
}
#endif
