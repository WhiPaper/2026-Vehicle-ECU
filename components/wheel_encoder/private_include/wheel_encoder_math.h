#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    float wheel_encoder_calculate_rpm(int32_t delta_count, int64_t elapsed_us, uint32_t cpr);

#ifdef __cplusplus
}
#endif
