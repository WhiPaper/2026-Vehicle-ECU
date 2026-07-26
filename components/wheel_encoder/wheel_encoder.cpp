#include "wheel_encoder.h"

#include "abi_encoder.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "wheel_encoder_math.h"
#include <cstddef>
#include <cstring>
#include <new>

namespace
{
constexpr int16_t PCNT_HIGH_LIMIT = 30000;
constexpr int16_t PCNT_LOW_LIMIT = -30000;

struct EncoderRuntime
{
    espp::AbiEncoder<>* driver;
    bool inverted;
    int32_t last_driver_count;
    int64_t total_count;
    int64_t last_sample_us;
};

EncoderRuntime encoders[WHEEL_ENCODER_SIDE_COUNT]{};
uint32_t encoder_cpr;
bool initialized;

bool valid_config(const wheel_encoder_config_t* config)
{
    if (config == nullptr)
    {
        return false;
    }
    for (size_t i = 0; i < WHEEL_ENCODER_SIDE_COUNT; ++i)
    {
        const auto& channel = config->channels[i];
        if (!GPIO_IS_VALID_GPIO(channel.a_gpio) || !GPIO_IS_VALID_GPIO(channel.b_gpio) ||
            channel.a_gpio == channel.b_gpio)
        {
            return false;
        }
        for (size_t j = 0; j < i; ++j)
        {
            const auto& other = config->channels[j];
            if (channel.a_gpio == other.a_gpio || channel.a_gpio == other.b_gpio ||
                channel.b_gpio == other.a_gpio || channel.b_gpio == other.b_gpio)
            {
                return false;
            }
        }
    }
    return true;
}

void release_encoders()
{
    for (auto& encoder : encoders)
    {
        delete encoder.driver;
        encoder = {};
    }
}

esp_err_t sample_one(EncoderRuntime& encoder, wheel_encoder_sample_t* sample, int64_t now)
{
    const int32_t driver_count = encoder.driver->get_count();
    int64_t delta = static_cast<int64_t>(driver_count) - encoder.last_driver_count;
    if (encoder.inverted)
    {
        delta = -delta;
    }
    encoder.total_count += delta;

    sample->count = encoder.total_count;
    sample->delta_count = static_cast<int32_t>(delta);
    sample->rpm =
        wheel_encoder_calculate_rpm(sample->delta_count, now - encoder.last_sample_us, encoder_cpr);
    sample->timestamp_us = now;
    sample->calibrated = encoder_cpr != 0;
    encoder.last_driver_count = driver_count;
    encoder.last_sample_us = now;
    return ESP_OK;
}
} // namespace

esp_err_t wheel_encoder_init(const wheel_encoder_config_t* config)
{
    if (!valid_config(config))
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (initialized)
    {
        return ESP_OK;
    }

    std::memset(encoders, 0, sizeof(encoders));
    for (size_t i = 0; i < WHEEL_ENCODER_SIDE_COUNT; ++i)
    {
        espp::AbiEncoder<>::Config driver_config{};
        driver_config.a_gpio = config->channels[i].a_gpio;
        driver_config.b_gpio = config->channels[i].b_gpio;
        driver_config.high_limit = PCNT_HIGH_LIMIT;
        driver_config.low_limit = PCNT_LOW_LIMIT;
        driver_config.counts_per_revolution = config->cpr > 0 ? config->cpr : PCNT_HIGH_LIMIT;
        driver_config.max_glitch_ns = config->glitch_filter_ns;
        driver_config.log_level = espp::Logger::Verbosity::WARN;

        encoders[i].driver = new (std::nothrow) espp::AbiEncoder<>(driver_config);
        const bool allocation_failed = encoders[i].driver == nullptr;
        if (allocation_failed || !encoders[i].driver->start())
        {
            release_encoders();
            return allocation_failed ? ESP_ERR_NO_MEM : ESP_FAIL;
        }
        encoders[i].inverted = config->channels[i].inverted;
        encoders[i].last_sample_us = esp_timer_get_time();
    }
    encoder_cpr = config->cpr;
    initialized = true;
    return ESP_OK;
}

esp_err_t wheel_encoder_sample(wheel_encoder_id_t id, wheel_encoder_sample_t* sample)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (static_cast<unsigned>(id) >= WHEEL_ENCODER_SIDE_COUNT || sample == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return sample_one(encoders[id], sample, esp_timer_get_time());
}

esp_err_t wheel_encoder_sample_all(wheel_encoder_sample_t samples[WHEEL_ENCODER_SIDE_COUNT])
{
    if (samples == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t now = esp_timer_get_time();
    for (size_t i = 0; i < WHEEL_ENCODER_SIDE_COUNT; ++i)
    {
        const esp_err_t result = sample_one(encoders[i], &samples[i], now);
        if (result != ESP_OK)
        {
            return result;
        }
    }
    return ESP_OK;
}

esp_err_t wheel_encoder_clear(void)
{
    if (!initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t now = esp_timer_get_time();
    for (auto& encoder : encoders)
    {
        if (!encoder.driver->clear())
        {
            return ESP_FAIL;
        }
        encoder.last_driver_count = 0;
        encoder.total_count = 0;
        encoder.last_sample_us = now;
    }
    return ESP_OK;
}

uint32_t wheel_encoder_cpr(void) { return encoder_cpr; }
