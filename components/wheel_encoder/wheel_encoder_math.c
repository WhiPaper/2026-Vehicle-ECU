#include "wheel_encoder_math.h"

float wheel_encoder_calculate_rpm(int32_t delta_count, int64_t elapsed_us, uint32_t cpr)
{
    if (elapsed_us <= 0 || cpr == 0)
    {
        return 0.0f;
    }
    return (float)delta_count * 60000000.0f / ((float)elapsed_us * (float)cpr);
}
