#pragma once

#include "drive.h"
#include "esp_err.h"
#include "imu.h"
#include "motor.h"
#include "wheel_encoder.h"

#ifdef __cplusplus
extern "C"
{
#endif

    const motor_config_t* board_motor_config(void);
    const wheel_encoder_config_t* board_wheel_encoder_config(void);
    const imu_config_t* board_imu_config(void);
    const drive_config_t* board_drive_config(void);
    esp_err_t board_validate(void);

#ifdef __cplusplus
}
#endif
