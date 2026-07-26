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

    esp_err_t imu_init(const imu_config_t* config);
    esp_err_t imu_read(imu_sample_t* sample);
    esp_err_t imu_calibrate_stationary(void);
    esp_err_t imu_recover(void);
    const imu_bias_t* imu_get_bias(void);
    bool imu_is_calibrated(void);

#ifdef __cplusplus
}
#endif
