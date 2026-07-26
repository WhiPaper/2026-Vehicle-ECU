#pragma once

#include "imu.h"
#include <stdint.h>

void imu_convert_raw(const uint8_t raw[14], imu_sample_t* sample);
