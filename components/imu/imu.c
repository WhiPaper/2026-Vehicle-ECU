#include "imu.h"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_TAG "imu"
#define STANDARD_GRAVITY_MPS2 9.80665f
#define DEG_TO_RAD 0.01745329251994329577f
#define MPU6050_I2C_CLOCK_HZ 400000U
#define MPU6050_WHO_AM_I_REGISTER 0x75U
#define MPU6050_WHO_AM_I_MASK 0x7EU
#define MPU6050_WHO_AM_I_VALUE 0x68U

static imu_config_t s_config;
static i2c_master_bus_handle_t s_bus;
static mpu6050_handle_t s_sensor;
static imu_bias_t s_bias;
static bool s_initialized;
static bool s_calibrated;

static esp_err_t configure_sensor(void)
{
    const mpu6050_config_t sensor_config = {
        .accel_fs = ACCEL_FS_4G,
        .gyro_fs = GYRO_FS_500DPS,
        .wake_auto = true,
    };
    return mpu6050_config(&s_sensor, sensor_config);
}

static esp_err_t verify_sensor_identity(void)
{
    uint8_t register_address = MPU6050_WHO_AM_I_REGISTER;
    uint8_t identity = 0;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(s_sensor.dev_handle, &register_address, 1, &identity, 1, -1),
        IMU_TAG, "Could not read WHO_AM_I");
    ESP_RETURN_ON_FALSE((identity & MPU6050_WHO_AM_I_MASK) == MPU6050_WHO_AM_I_VALUE,
                        ESP_ERR_NOT_FOUND, IMU_TAG, "Unexpected WHO_AM_I value: 0x%02x", identity);
    return ESP_OK;
}

esp_err_t imu_init(const imu_config_t* config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, IMU_TAG, "Configuration is required");
    ESP_RETURN_ON_FALSE(config->calibration_rate_hz >= 10 && config->calibration_rate_hz <= 1000,
                        ESP_ERR_INVALID_ARG, IMU_TAG, "Invalid calibration rate");
    if (s_initialized)
    {
        return ESP_OK;
    }

    s_config = *config;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->sda_gpio,
        .scl_io_num = config->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_bus), IMU_TAG,
                        "Could not create I2C bus");

    const mpu6050_info_t sensor_info = {
        .address = config->address,
        .clock_speed = MPU6050_I2C_CLOCK_HZ,
    };
    esp_err_t result = mpu6050_create(s_bus, sensor_info, &s_sensor);
    if (result == ESP_OK)
    {
        result = verify_sensor_identity();
    }
    if (result == ESP_OK)
    {
        result = configure_sensor();
    }
    if (result != ESP_OK)
    {
        if (s_sensor.dev_handle != NULL)
        {
            (void)mpu6050_delete(s_sensor);
            memset(&s_sensor, 0, sizeof(s_sensor));
        }
        (void)i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return result;
    }

    memset(&s_bias, 0, sizeof(s_bias));
    s_calibrated = false;
    s_initialized = true;
    return ESP_OK;
}

esp_err_t imu_read(imu_sample_t* sample)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, IMU_TAG, "IMU is not initialized");
    ESP_RETURN_ON_FALSE(sample != NULL, ESP_ERR_INVALID_ARG, IMU_TAG, "Sample output is required");

    mpu6050_accel_value_t acceleration;
    mpu6050_gyro_value_t angular_velocity;
    mpu6050_temp_value_t temperature;
    memset(sample, 0, sizeof(*sample));
    ESP_RETURN_ON_ERROR(mpu6050_get_all(s_sensor, &acceleration, &angular_velocity, &temperature),
                        IMU_TAG, "Sample read failed");

    const float acceleration_g[3] = {
        acceleration.accel_x,
        acceleration.accel_y,
        acceleration.accel_z,
    };
    const float angular_velocity_dps[3] = {
        angular_velocity.gyro_x,
        angular_velocity.gyro_y,
        angular_velocity.gyro_z,
    };
    for (size_t axis = 0; axis < 3; ++axis)
    {
        sample->acceleration_mps2[axis] =
            acceleration_g[axis] * STANDARD_GRAVITY_MPS2 - s_bias.acceleration_mps2[axis];
        sample->angular_velocity_rps[axis] =
            angular_velocity_dps[axis] * DEG_TO_RAD - s_bias.angular_velocity_rps[axis];
    }
    sample->temperature_c = temperature.temp;
    sample->timestamp_us = esp_timer_get_time();
    sample->valid = true;
    return ESP_OK;
}

esp_err_t imu_calibrate_stationary(void)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, IMU_TAG, "IMU is not initialized");
    s_calibrated = false;
    if (s_config.calibration_samples == 0)
    {
        memset(&s_bias, 0, sizeof(s_bias));
        s_calibrated = true;
        return ESP_OK;
    }

    imu_bias_t sums = {0};
    float max_gyro = 0;
    float accel_norm_sum = 0;
    const TickType_t delay = pdMS_TO_TICKS(1000U / s_config.calibration_rate_hz);
    memset(&s_bias, 0, sizeof(s_bias));

    for (uint16_t i = 0; i < s_config.calibration_samples; ++i)
    {
        imu_sample_t sample;
        ESP_RETURN_ON_ERROR(imu_read(&sample), IMU_TAG, "Calibration sample failed");
        float accel_norm_sq = 0;
        for (size_t axis = 0; axis < 3; ++axis)
        {
            sums.acceleration_mps2[axis] += sample.acceleration_mps2[axis];
            sums.angular_velocity_rps[axis] += sample.angular_velocity_rps[axis];
            accel_norm_sq += sample.acceleration_mps2[axis] * sample.acceleration_mps2[axis];
            const float gyro_abs = fabsf(sample.angular_velocity_rps[axis]);
            if (gyro_abs > max_gyro)
            {
                max_gyro = gyro_abs;
            }
        }
        accel_norm_sum += sqrtf(accel_norm_sq);
        vTaskDelay(delay > 0 ? delay : 1);
    }

    const float count = (float)s_config.calibration_samples;
    const float mean_norm = accel_norm_sum / count;
    ESP_RETURN_ON_FALSE(max_gyro < 0.15f && fabsf(mean_norm - STANDARD_GRAVITY_MPS2) < 1.5f,
                        ESP_ERR_INVALID_STATE, IMU_TAG,
                        "Sensor moved during stationary calibration");

    for (size_t axis = 0; axis < 3; ++axis)
    {
        s_bias.acceleration_mps2[axis] = sums.acceleration_mps2[axis] / count;
        s_bias.angular_velocity_rps[axis] = sums.angular_velocity_rps[axis] / count;
    }
    const size_t gravity_axis =
        fabsf(s_bias.acceleration_mps2[2]) >= fabsf(s_bias.acceleration_mps2[0]) &&
                fabsf(s_bias.acceleration_mps2[2]) >= fabsf(s_bias.acceleration_mps2[1])
            ? 2
            : (fabsf(s_bias.acceleration_mps2[1]) >= fabsf(s_bias.acceleration_mps2[0]) ? 1 : 0);
    s_bias.acceleration_mps2[gravity_axis] -= s_bias.acceleration_mps2[gravity_axis] >= 0
                                                  ? STANDARD_GRAVITY_MPS2
                                                  : -STANDARD_GRAVITY_MPS2;
    s_calibrated = true;
    return ESP_OK;
}

esp_err_t imu_recover(void)
{
    if (!s_initialized)
    {
        return imu_init(&s_config);
    }
    ESP_RETURN_ON_ERROR(i2c_master_bus_reset(s_bus), IMU_TAG, "Could not reset I2C bus");
    ESP_RETURN_ON_ERROR(verify_sensor_identity(), IMU_TAG, "MPU6050 identity check failed");
    return configure_sensor();
}

const imu_bias_t* imu_get_bias(void) { return &s_bias; }

bool imu_is_calibrated(void) { return s_initialized && s_calibrated; }
