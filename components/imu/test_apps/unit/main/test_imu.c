#include "imu.h"
#include "imu_backend.h"
#include "unity.h"

#include <string.h>

#define WHO_AM_I_REGISTER 0x75U
#define POWER_REGISTER 0x6BU
#define DATA_REGISTER 0x3BU

static uint8_t s_raw[14];
static esp_err_t s_open_result;
static esp_err_t s_read_result;
static esp_err_t s_reset_result;
static uint32_t s_last_timeout_ms;
static unsigned s_close_count;

static esp_err_t fake_open(const imu_config_t* config)
{
    (void)config;
    return s_open_result;
}

static void fake_close(void) { s_close_count++; }

static esp_err_t fake_read(uint8_t address, uint8_t* data, size_t length, uint32_t timeout_ms)
{
    s_last_timeout_ms = timeout_ms;
    if (s_read_result != ESP_OK)
    {
        return s_read_result;
    }
    if (address == WHO_AM_I_REGISTER && length == 1)
    {
        data[0] = 0x68;
    }
    else if (address == POWER_REGISTER && length == 1)
    {
        data[0] = 1U << 6;
    }
    else if (address == DATA_REGISTER && length == sizeof(s_raw))
    {
        memcpy(data, s_raw, sizeof(s_raw));
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t fake_write(uint8_t address, const uint8_t* data, size_t length,
                            uint32_t timeout_ms)
{
    (void)address;
    (void)data;
    (void)length;
    s_last_timeout_ms = timeout_ms;
    return ESP_OK;
}

static esp_err_t fake_reset(void) { return s_reset_result; }

static const imu_backend_t s_fake_backend = {
    .open = fake_open,
    .close = fake_close,
    .read = fake_read,
    .write = fake_write,
    .reset = fake_reset,
};

static const imu_config_t s_config = {
    .i2c_port = 0,
    .sda_gpio = 21,
    .scl_gpio = 22,
    .address = 0x68,
    .calibration_rate_hz = 100,
    .calibration_samples = 1,
};

void setUp(void)
{
    imu_test_reset();
    memset(s_raw, 0, sizeof(s_raw));
    s_raw[4] = 0x20;
    s_open_result = ESP_OK;
    s_read_result = ESP_OK;
    s_reset_result = ESP_OK;
    s_last_timeout_ms = 0;
    s_close_count = 0;
    imu_test_set_backend(&s_fake_backend);
}

void tearDown(void) { imu_test_reset(); }

TEST_CASE("IMU backend uses finite timeout and calibrates stationary sample", "[imu]")
{
    TEST_ASSERT_EQUAL(ESP_OK, imu_init(&s_config));
    TEST_ASSERT_EQUAL_UINT32(CONFIG_IMU_I2C_TIMEOUT_MS, s_last_timeout_ms);
    TEST_ASSERT_EQUAL(ESP_OK, imu_calibrate_stationary());
    TEST_ASSERT_TRUE(imu_is_calibrated());
}

TEST_CASE("IMU initialization propagates timeout and releases backend", "[imu]")
{
    s_read_result = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, imu_init(&s_config));
    TEST_ASSERT_EQUAL_UINT32(CONFIG_IMU_I2C_TIMEOUT_MS, s_last_timeout_ms);
    TEST_ASSERT_EQUAL_UINT32(1, s_close_count);
}

TEST_CASE("IMU rejects invalid configuration before opening backend", "[imu]")
{
    imu_config_t invalid = s_config;
    invalid.address = 0x67;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, imu_init(&invalid));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, imu_start(&invalid));
    TEST_ASSERT_EQUAL_UINT32(0, s_close_count);

    invalid = s_config;
    invalid.calibration_rate_hz = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, imu_init(&invalid));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, imu_start(&invalid));
    TEST_ASSERT_EQUAL_UINT32(0, s_close_count);
}

TEST_CASE("IMU calibration rejects motion", "[imu]")
{
    s_raw[8] = 0x02;
    s_raw[9] = 0x8F;
    TEST_ASSERT_EQUAL(ESP_OK, imu_init(&s_config));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, imu_calibrate_stationary());
    TEST_ASSERT_FALSE(imu_is_calibrated());
}

TEST_CASE("IMU recovery propagates bounded backend reset failure", "[imu]")
{
    TEST_ASSERT_EQUAL(ESP_OK, imu_init(&s_config));
    s_reset_result = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, imu_recover());
}

TEST_CASE("IMU recovers after consecutive bounded read failures", "[imu]")
{
    TEST_ASSERT_EQUAL(ESP_OK, imu_init(&s_config));
    s_read_result = ESP_ERR_TIMEOUT;
    imu_sample_t sample;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, imu_read(&sample));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, imu_read(&sample));

    s_read_result = ESP_OK;
    TEST_ASSERT_EQUAL(ESP_OK, imu_recover());
    TEST_ASSERT_EQUAL(ESP_OK, imu_calibrate_stationary());
    TEST_ASSERT_EQUAL(ESP_OK, imu_read(&sample));
    TEST_ASSERT_TRUE(sample.valid);
}

TEST_CASE("IMU snapshot freshness rejects stale and future samples", "[imu]")
{
    imu_snapshot_t snapshot = {
        .last_success_us = 1000000,
        .valid = true,
    };
    TEST_ASSERT_TRUE(imu_snapshot_is_fresh(&snapshot, 1099999, 100));
    TEST_ASSERT_FALSE(imu_snapshot_is_fresh(&snapshot, 1100001, 100));
    TEST_ASSERT_FALSE(imu_snapshot_is_fresh(&snapshot, 999999, 100));
    snapshot.valid = false;
    TEST_ASSERT_FALSE(imu_snapshot_is_fresh(&snapshot, 1000000, 100));
}
