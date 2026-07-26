#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        WHEEL_ENCODER_LEFT = 0,
        WHEEL_ENCODER_RIGHT,
        WHEEL_ENCODER_SIDE_COUNT,
    } wheel_encoder_id_t;

    typedef struct
    {
        int a_gpio;
        int b_gpio;
        bool inverted;
    } wheel_encoder_channel_config_t;

    typedef struct
    {
        wheel_encoder_channel_config_t channels[WHEEL_ENCODER_SIDE_COUNT];
        uint32_t cpr;
        uint32_t glitch_filter_ns;
    } wheel_encoder_config_t;

    typedef struct
    {
        int64_t count;
        int32_t delta_count;
        float rpm;
        int64_t timestamp_us;
        bool calibrated;
    } wheel_encoder_sample_t;

    esp_err_t wheel_encoder_init(const wheel_encoder_config_t* config);
    esp_err_t wheel_encoder_sample(wheel_encoder_id_t id, wheel_encoder_sample_t* sample);
    esp_err_t wheel_encoder_sample_all(wheel_encoder_sample_t samples[WHEEL_ENCODER_SIDE_COUNT]);
    esp_err_t wheel_encoder_clear(void);
    uint32_t wheel_encoder_cpr(void);

#ifdef __cplusplus
}
#endif
