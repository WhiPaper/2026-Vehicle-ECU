#include "imu.h"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_backend.h"
#include "imu_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_TAG "imu"
#define IMU_TASK_STACK 4096
#define IMU_TASK_PRIORITY 6
#define STANDARD_GRAVITY_MPS2 9.80665f
#define MPU6050_I2C_CLOCK_HZ 400000U
#define MPU6050_GYRO_CONFIG_REGISTER 0x1BU
#define MPU6050_ACCEL_XOUT_H_REGISTER 0x3BU
#define MPU6050_PWR_MGMT_1_REGISTER 0x6BU
#define MPU6050_WHO_AM_I_REGISTER 0x75U
#define MPU6050_WHO_AM_I_MASK 0x7EU
#define MPU6050_WHO_AM_I_VALUE 0x68U
#define MPU6050_FS_4G 1U
#define MPU6050_FS_500DPS 1U

static imu_config_t s_config;
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_sensor;
static imu_bias_t s_bias;
static imu_snapshot_t s_snapshot;
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task;
static bool s_initialized;
static bool s_calibrated;
static bool s_started;

static esp_err_t hardware_open(const imu_config_t* config)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->sda_gpio,
        .scl_io_num = config->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t result = i2c_new_master_bus(&bus_config, &s_bus);
    if (result != ESP_OK)
    {
        return result;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->address,
        .scl_speed_hz = MPU6050_I2C_CLOCK_HZ,
    };
    result = i2c_master_bus_add_device(s_bus, &device_config, &s_sensor);
    if (result != ESP_OK)
    {
        (void)i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    return result;
}

static void hardware_close(void)
{
    if (s_sensor != NULL)
    {
        (void)i2c_master_bus_rm_device(s_sensor);
        s_sensor = NULL;
    }
    if (s_bus != NULL)
    {
        (void)i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

static esp_err_t hardware_read(uint8_t address, uint8_t* data, size_t length, uint32_t timeout_ms)
{
    return i2c_master_transmit_receive(s_sensor, &address, 1, data, length, timeout_ms);
}

static esp_err_t hardware_write(uint8_t address, const uint8_t* data, size_t length,
                                uint32_t timeout_ms)
{
    uint8_t buffer[3];
    if (data == NULL || length == 0 || length > sizeof(buffer) - 1)
    {
        return ESP_ERR_INVALID_ARG;
    }
    buffer[0] = address;
    memcpy(&buffer[1], data, length);
    return i2c_master_transmit(s_sensor, buffer, length + 1, timeout_ms);
}

static esp_err_t hardware_reset(void) { return i2c_master_bus_reset(s_bus); }

static const imu_backend_t s_hardware_backend = {
    .open = hardware_open,
    .close = hardware_close,
    .read = hardware_read,
    .write = hardware_write,
    .reset = hardware_reset,
};
static const imu_backend_t* s_backend = &s_hardware_backend;

static bool config_is_valid(const imu_config_t* config)
{
    return config != NULL && config->address >= 0x68 && config->address <= 0x69 &&
           config->calibration_rate_hz >= 10 && config->calibration_rate_hz <= 1000;
}

static bool backend_is_valid(const imu_backend_t* backend)
{
    return backend != NULL && backend->open != NULL && backend->close != NULL &&
           backend->read != NULL && backend->write != NULL && backend->reset != NULL;
}

static esp_err_t register_write(uint8_t address, const uint8_t* data, size_t length)
{
    ESP_RETURN_ON_FALSE(data != NULL && length > 0, ESP_ERR_INVALID_ARG, IMU_TAG,
                        "Invalid register write");
    return s_backend->write(address, data, length, CONFIG_IMU_I2C_TIMEOUT_MS);
}

static esp_err_t register_read(uint8_t address, uint8_t* data, size_t length)
{
    ESP_RETURN_ON_FALSE(data != NULL && length > 0, ESP_ERR_INVALID_ARG, IMU_TAG,
                        "Invalid register read");
    return s_backend->read(address, data, length, CONFIG_IMU_I2C_TIMEOUT_MS);
}

static esp_err_t verify_sensor_identity(void)
{
    uint8_t identity = 0;
    ESP_RETURN_ON_ERROR(register_read(MPU6050_WHO_AM_I_REGISTER, &identity, 1), IMU_TAG,
                        "Could not read WHO_AM_I");
    ESP_RETURN_ON_FALSE((identity & MPU6050_WHO_AM_I_MASK) == MPU6050_WHO_AM_I_VALUE,
                        ESP_ERR_NOT_FOUND, IMU_TAG, "Unexpected WHO_AM_I value: 0x%02x", identity);
    return ESP_OK;
}

static esp_err_t configure_sensor(void)
{
    const uint8_t ranges[2] = {
        MPU6050_FS_500DPS << 3,
        MPU6050_FS_4G << 3,
    };
    ESP_RETURN_ON_ERROR(register_write(MPU6050_GYRO_CONFIG_REGISTER, ranges, sizeof(ranges)),
                        IMU_TAG, "Could not configure sensor ranges");

    uint8_t power = 0;
    ESP_RETURN_ON_ERROR(register_read(MPU6050_PWR_MGMT_1_REGISTER, &power, 1), IMU_TAG,
                        "Could not read power state");
    power &= ~(1U << 6);
    return register_write(MPU6050_PWR_MGMT_1_REGISTER, &power, 1);
}

static void release_sensor(void)
{
    s_backend->close();
    s_initialized = false;
    s_calibrated = false;
}

static void update_snapshot(imu_state_t state, esp_err_t error, const imu_sample_t* sample)
{
    taskENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot.state = state;
    s_snapshot.last_error = error;
    s_snapshot.initialized = s_initialized;
    s_snapshot.calibrated = s_calibrated;
    s_snapshot.valid = sample != NULL && sample->valid && state == IMU_STATE_READY;
    if (sample != NULL)
    {
        s_snapshot.sample = *sample;
        if (s_snapshot.valid)
        {
            s_snapshot.last_success_us = sample->timestamp_us;
        }
    }
    taskEXIT_CRITICAL(&s_snapshot_lock);
}

esp_err_t imu_init(const imu_config_t* config)
{
    ESP_RETURN_ON_FALSE(config_is_valid(config), ESP_ERR_INVALID_ARG, IMU_TAG,
                        "Invalid configuration");
    if (s_initialized)
    {
        return ESP_OK;
    }

    s_config = *config;
    esp_err_t result = s_backend->open(config);
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
        release_sensor();
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

    uint8_t raw[14];
    memset(sample, 0, sizeof(*sample));
    ESP_RETURN_ON_ERROR(register_read(MPU6050_ACCEL_XOUT_H_REGISTER, raw, sizeof(raw)), IMU_TAG,
                        "Sample read failed");
    imu_convert_raw(raw, sample);
    for (size_t axis = 0; axis < 3; ++axis)
    {
        sample->acceleration_mps2[axis] -= s_bias.acceleration_mps2[axis];
        sample->angular_velocity_rps[axis] -= s_bias.angular_velocity_rps[axis];
    }
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
    ESP_RETURN_ON_ERROR(s_backend->reset(), IMU_TAG, "Could not reset I2C bus");
    ESP_RETURN_ON_ERROR(verify_sensor_identity(), IMU_TAG, "MPU6050 identity check failed");
    return configure_sensor();
}

static void imu_task(void* argument)
{
    (void)argument;
    const TickType_t sample_period = pdMS_TO_TICKS(1000U / CONFIG_IMU_SAMPLE_RATE_HZ);
    const TickType_t recovery_period = pdMS_TO_TICKS(CONFIG_IMU_RECOVERY_PERIOD_MS);

    for (;;)
    {
        if (!s_initialized)
        {
            update_snapshot(IMU_STATE_INITIALIZING, ESP_OK, NULL);
            const esp_err_t result = imu_init(&s_config);
            if (result != ESP_OK)
            {
                update_snapshot(IMU_STATE_RECOVERING, result, NULL);
                vTaskDelay(recovery_period);
                continue;
            }
        }

        if (!s_calibrated)
        {
            update_snapshot(IMU_STATE_CALIBRATING, ESP_OK, NULL);
            const esp_err_t result = imu_calibrate_stationary();
            if (result != ESP_OK)
            {
                update_snapshot(IMU_STATE_RECOVERING, result, NULL);
                if (result != ESP_ERR_INVALID_STATE)
                {
                    release_sensor();
                }
                vTaskDelay(recovery_period);
                continue;
            }
        }

        TickType_t last_wake = xTaskGetTickCount();
        while (s_initialized && s_calibrated)
        {
            imu_sample_t sample;
            const esp_err_t result = imu_read(&sample);
            if (result != ESP_OK)
            {
                update_snapshot(IMU_STATE_RECOVERING, result, NULL);
                break;
            }
            update_snapshot(IMU_STATE_READY, ESP_OK, &sample);
            if (xTaskDelayUntil(&last_wake, sample_period > 0 ? sample_period : 1) == pdFALSE)
            {
                last_wake = xTaskGetTickCount();
            }
        }

        vTaskDelay(recovery_period);
        const esp_err_t recovery_result = imu_recover();
        if (recovery_result != ESP_OK)
        {
            update_snapshot(IMU_STATE_RECOVERING, recovery_result, NULL);
            release_sensor();
        }
        else
        {
            s_calibrated = false;
        }
    }
}

esp_err_t imu_start(const imu_config_t* config)
{
    ESP_RETURN_ON_FALSE(config_is_valid(config), ESP_ERR_INVALID_ARG, IMU_TAG,
                        "Invalid configuration");
    taskENTER_CRITICAL(&s_snapshot_lock);
    if (s_started)
    {
        taskEXIT_CRITICAL(&s_snapshot_lock);
        return ESP_OK;
    }
    s_config = *config;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.state = IMU_STATE_INITIALIZING;
    s_started = true;
    taskEXIT_CRITICAL(&s_snapshot_lock);

    if (xTaskCreate(imu_task, "imu", IMU_TASK_STACK, NULL, IMU_TASK_PRIORITY, &s_task) != pdPASS)
    {
        taskENTER_CRITICAL(&s_snapshot_lock);
        s_started = false;
        s_snapshot.state = IMU_STATE_STOPPED;
        s_snapshot.last_error = ESP_ERR_NO_MEM;
        taskEXIT_CRITICAL(&s_snapshot_lock);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t imu_get_snapshot(imu_snapshot_t* snapshot)
{
    ESP_RETURN_ON_FALSE(snapshot != NULL, ESP_ERR_INVALID_ARG, IMU_TAG,
                        "Snapshot output is required");
    taskENTER_CRITICAL(&s_snapshot_lock);
    const bool started = s_started;
    *snapshot = s_snapshot;
    taskEXIT_CRITICAL(&s_snapshot_lock);
    return started ? ESP_OK : ESP_ERR_INVALID_STATE;
}

bool imu_snapshot_is_fresh(const imu_snapshot_t* snapshot, int64_t now_us, uint32_t maximum_age_ms)
{
    return snapshot != NULL && snapshot->valid && snapshot->last_success_us > 0 &&
           now_us >= snapshot->last_success_us &&
           now_us - snapshot->last_success_us <= (int64_t)maximum_age_ms * 1000;
}

const imu_bias_t* imu_get_bias(void) { return &s_bias; }

bool imu_is_calibrated(void) { return s_calibrated; }

const char* imu_state_name(imu_state_t state)
{
    switch (state)
    {
    case IMU_STATE_STOPPED:
        return "stopped";
    case IMU_STATE_INITIALIZING:
        return "initializing";
    case IMU_STATE_CALIBRATING:
        return "calibrating";
    case IMU_STATE_READY:
        return "ready";
    case IMU_STATE_RECOVERING:
        return "recovering";
    default:
        return "unknown";
    }
}

void imu_test_set_backend(const imu_backend_t* backend)
{
    if (!s_initialized && !s_started && backend_is_valid(backend))
    {
        s_backend = backend;
    }
}

void imu_test_reset(void)
{
    if (s_initialized)
    {
        release_sensor();
    }
    memset(&s_config, 0, sizeof(s_config));
    memset(&s_bias, 0, sizeof(s_bias));
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_initialized = false;
    s_calibrated = false;
    s_started = false;
    s_task = NULL;
    s_backend = &s_hardware_backend;
}
