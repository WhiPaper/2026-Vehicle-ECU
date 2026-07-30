#include "board.h"
#include "drive.h"
#include "esp_err.h"
#include "esp_log.h"
#include "imu.h"
#include "motor.h"
#include "ros_bridge.h"
#include "wheel_encoder.h"
#include <stdbool.h>

static const char* TAG = "vehicle_ecu";

void app_main(void)
{
    ESP_ERROR_CHECK(board_validate());
    ESP_ERROR_CHECK(ros_bridge_start());

    esp_err_t result = imu_start(board_imu_config());
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "IMU task initialization failed: %s", esp_err_to_name(result));
    }

    result = motor_init(board_motor_config());
    const bool motor_ready = result == ESP_OK;
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor initialization failed: %s", esp_err_to_name(result));
    }

    result = wheel_encoder_init(board_wheel_encoder_config());
    const bool encoder_ready = result == ESP_OK;
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Encoder initialization failed: %s", esp_err_to_name(result));
    }

    ESP_ERROR_CHECK(drive_init(board_drive_config()));
    if (!motor_ready)
    {
        ESP_ERROR_CHECK(drive_report_fault(DRIVE_FAULT_MOTOR));
    }
    if (!encoder_ready)
    {
        ESP_ERROR_CHECK(drive_report_fault(DRIVE_FAULT_ENCODER));
    }
    if (motor_ready && encoder_ready)
    {
        ESP_ERROR_CHECK(drive_start());
    }
    else
    {
        ESP_LOGE(TAG, "Drive task disabled because required hardware is unavailable");
    }

    ESP_LOGI(TAG, "Vehicle ECU started; motors remain stopped until cmd_vel");
}
