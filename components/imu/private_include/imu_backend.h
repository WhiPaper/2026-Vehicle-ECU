#pragma once

#include "imu.h"

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    esp_err_t (*open)(const imu_config_t* config);
    void (*close)(void);
    esp_err_t (*read)(uint8_t address, uint8_t* data, size_t length, uint32_t timeout_ms);
    esp_err_t (*write)(uint8_t address, const uint8_t* data, size_t length, uint32_t timeout_ms);
    esp_err_t (*reset)(void);
} imu_backend_t;

void imu_test_set_backend(const imu_backend_t* backend);
void imu_test_reset(void);
