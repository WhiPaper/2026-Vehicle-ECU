#include "motor.h"

#include "bdc_motor.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_mix.h"
#include <stdbool.h>
#include <stddef.h>

#define MOTOR_TAG "motor"
#define MOTOR_MASK(motor) (1U << (unsigned)(motor))
#define MOTOR_ALL_MASK ((1U << MOTOR_COUNT) - 1U)
#define MOTOR_MCPWM_RESOLUTION_HZ 10000000U
#define MOTOR_MCPWM_PER_GROUP 3U
#define MOTOR_DIRECTION_UNKNOWN 2

typedef uint32_t motor_mask_t;

static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_storage;
static portMUX_TYPE s_init_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_initialized;
static motor_config_t s_config;
static bdc_motor_handle_t s_motors[MOTOR_COUNT];
static int8_t s_directions[MOTOR_COUNT];
static motor_normalized_speed_t s_last_speeds[MOTOR_COUNT];

static esp_err_t first_error(esp_err_t first, esp_err_t second)
{
    return first != ESP_OK ? first : second;
}

static esp_err_t validate_config(const motor_config_t* config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, MOTOR_TAG,
                        "Configuration is required");
    ESP_RETURN_ON_FALSE(config->pwm_frequency_hz >= MOTOR_PWM_FREQUENCY_MIN_HZ &&
                            config->pwm_frequency_hz <= MOTOR_PWM_FREQUENCY_MAX_HZ,
                        ESP_ERR_INVALID_ARG, MOTOR_TAG,
                        "PWM frequency is outside the supported range");

    int pins[MOTOR_COUNT * 2];
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        pins[i * 2] = config->channels[i].ia_gpio;
        pins[i * 2 + 1] = config->channels[i].ib_gpio;
    }
    for (size_t i = 0; i < MOTOR_COUNT * 2; ++i)
    {
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(pins[i]), ESP_ERR_INVALID_ARG, MOTOR_TAG,
                            "Motor GPIO must support output");
        for (size_t j = i + 1; j < MOTOR_COUNT * 2; ++j)
        {
            ESP_RETURN_ON_FALSE(pins[i] != pins[j], ESP_ERR_INVALID_ARG, MOTOR_TAG,
                                "Motor GPIOs must be unique");
        }
    }
    return ESP_OK;
}

static bool config_matches(const motor_config_t* config)
{
    if (config->pwm_frequency_hz != s_config.pwm_frequency_hz)
    {
        return false;
    }
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const motor_channel_config_t* a = &config->channels[i];
        const motor_channel_config_t* b = &s_config.channels[i];
        if (a->ia_gpio != b->ia_gpio || a->ib_gpio != b->ib_gpio || a->inverted != b->inverted)
        {
            return false;
        }
    }
    return true;
}

static uint32_t normalized_to_ticks(motor_normalized_speed_t speed)
{
    int32_t magnitude = speed;
    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }
    const uint32_t period_ticks = MOTOR_MCPWM_RESOLUTION_HZ / s_config.pwm_frequency_hz;
    return ((uint32_t)magnitude * period_ticks + MOTOR_NORMALIZED_SPEED_MAX / 2U) /
           MOTOR_NORMALIZED_SPEED_MAX;
}

static esp_err_t backend_stop_all(void)
{
    esp_err_t result = ESP_OK;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (s_motors[i] == NULL)
        {
            continue;
        }
        const esp_err_t stop_result = bdc_motor_coast(s_motors[i]);
        result = first_error(result, stop_result);
        s_directions[i] = stop_result == ESP_OK ? 0 : MOTOR_DIRECTION_UNKNOWN;
    }
    return result;
}

static esp_err_t backend_set(size_t motor, motor_normalized_speed_t normalized_speed)
{
    int32_t physical_speed = normalized_speed;
    if (s_config.channels[motor].inverted)
    {
        physical_speed = -physical_speed;
    }
    const int8_t direction = physical_speed > 0 ? 1 : (physical_speed < 0 ? -1 : 0);
    if (direction == 0)
    {
        const esp_err_t result = bdc_motor_coast(s_motors[motor]);
        s_directions[motor] = result == ESP_OK ? 0 : MOTOR_DIRECTION_UNKNOWN;
        return result;
    }

    if (s_directions[motor] != 0 && s_directions[motor] != direction)
    {
        ESP_RETURN_ON_ERROR(bdc_motor_coast(s_motors[motor]), MOTOR_TAG,
                            "Could not clear bridge before direction change");
        s_directions[motor] = 0;
    }

    ESP_RETURN_ON_ERROR(
        bdc_motor_set_speed(s_motors[motor],
                            normalized_to_ticks((motor_normalized_speed_t)physical_speed)),
        MOTOR_TAG, "Could not set motor duty");
    const esp_err_t result =
        direction > 0 ? bdc_motor_forward(s_motors[motor]) : bdc_motor_reverse(s_motors[motor]);
    if (result == ESP_OK)
    {
        s_directions[motor] = direction;
        return ESP_OK;
    }

    (void)bdc_motor_coast(s_motors[motor]);
    s_directions[motor] = MOTOR_DIRECTION_UNKNOWN;
    return result;
}

static void clear_last_speeds(void)
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        s_last_speeds[i] = 0;
    }
}

static void fail_safe_stop(void)
{
    (void)backend_stop_all();
    clear_last_speeds();
}

static esp_err_t lock_motor(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, MOTOR_TAG,
                        "Motor lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE, ESP_ERR_TIMEOUT, MOTOR_TAG,
                        "Could not acquire motor lock");
    if (!s_initialized)
    {
        xSemaphoreGive(s_lock);
        ESP_LOGE(MOTOR_TAG, "Motor is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t ensure_lock_initialized(void)
{
    portENTER_CRITICAL(&s_init_lock);
    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_storage);
    }
    portEXIT_CRITICAL(&s_init_lock);

    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, MOTOR_TAG, "Could not create motor mutex");
    return ESP_OK;
}

static void wait_for_start_boost(void)
{
#if CONFIG_MOTOR_START_BOOST_SPEED > 0
    vTaskDelay(pdMS_TO_TICKS(CONFIG_MOTOR_START_BOOST_DURATION_MS));
#endif
}

static bool motor_is_selected(motor_mask_t selected_motors, size_t motor)
{
    return (selected_motors & MOTOR_MASK(motor)) != 0;
}

static esp_err_t apply_targets_locked(const motor_normalized_speed_t targets[MOTOR_COUNT],
                                      motor_mask_t selected_motors, bool allow_start_boost)
{
    motor_start_plan_t plans[MOTOR_COUNT] = {0};
    bool any_boosted = false;
    esp_err_t result = ESP_OK;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (!motor_is_selected(selected_motors, i))
        {
            continue;
        }

        plans[i] = motor_plan_start(s_last_speeds[i], targets[i],
                                    allow_start_boost ? CONFIG_MOTOR_START_BOOST_SPEED : 0);
        any_boosted = any_boosted || plans[i].boosted;
        result = backend_set(i, plans[i].initial);
        if (result != ESP_OK)
        {
            break;
        }
    }

    if (result == ESP_OK && any_boosted)
    {
        wait_for_start_boost();
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            if (!motor_is_selected(selected_motors, i) || !plans[i].boosted)
            {
                continue;
            }

            result = backend_set(i, plans[i].target);
            if (result != ESP_OK)
            {
                break;
            }
        }
    }

    if (result != ESP_OK)
    {
        fail_safe_stop();
        return result;
    }

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (motor_is_selected(selected_motors, i))
        {
            s_last_speeds[i] = plans[i].target;
        }
    }
    return ESP_OK;
}

esp_err_t motor_init(const motor_config_t* config)
{
    ESP_RETURN_ON_ERROR(validate_config(config), MOTOR_TAG, "Invalid motor configuration");
    ESP_RETURN_ON_ERROR(ensure_lock_initialized(), MOTOR_TAG, "Could not initialize motor lock");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE, ESP_ERR_TIMEOUT, MOTOR_TAG,
                        "Could not acquire motor lock");
    if (s_initialized)
    {
        const esp_err_t result = config_matches(config) ? ESP_OK : ESP_ERR_INVALID_STATE;
        xSemaphoreGive(s_lock);
        return result;
    }

    s_config = *config;
    esp_err_t result = ESP_OK;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const bdc_motor_config_t motor_config = {
            .pwma_gpio_num = (uint32_t)config->channels[i].ia_gpio,
            .pwmb_gpio_num = (uint32_t)config->channels[i].ib_gpio,
            .pwm_freq_hz = config->pwm_frequency_hz,
        };
        const bdc_motor_mcpwm_config_t mcpwm_config = {
            .group_id = (int)(i / MOTOR_MCPWM_PER_GROUP),
            .resolution_hz = MOTOR_MCPWM_RESOLUTION_HZ,
        };
        result = bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &s_motors[i]);
        if (result == ESP_OK)
        {
            result = bdc_motor_enable(s_motors[i]);
        }
        if (result != ESP_OK)
        {
            break;
        }
    }
    if (result == ESP_OK)
    {
        result = backend_stop_all();
    }
    if (result == ESP_OK)
    {
        s_initialized = true;
        clear_last_speeds();
    }
    else
    {
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
        {
            if (s_motors[i] != NULL)
            {
                (void)bdc_motor_disable(s_motors[i]);
                (void)bdc_motor_del(s_motors[i]);
                s_motors[i] = NULL;
            }
        }
    }
    xSemaphoreGive(s_lock);
    return result;
}

esp_err_t motor_set_tank_normalized(motor_normalized_speed_t left_normalized_speed,
                                    motor_normalized_speed_t right_normalized_speed)
{
    ESP_RETURN_ON_ERROR(lock_motor(), MOTOR_TAG, "Motor unavailable");

    const motor_normalized_speed_t left_target = motor_clamp_normalized(left_normalized_speed);
    const motor_normalized_speed_t right_target = motor_clamp_normalized(right_normalized_speed);
    const motor_normalized_speed_t targets[MOTOR_COUNT] = {
        [MOTOR_FRONT_LEFT] = left_target,
        [MOTOR_FRONT_RIGHT] = right_target,
        [MOTOR_REAR_LEFT] = left_target,
        [MOTOR_REAR_RIGHT] = right_target,
    };
    const esp_err_t result = apply_targets_locked(targets, MOTOR_ALL_MASK, true);
    xSemaphoreGive(s_lock);
    return result;
}

esp_err_t motor_set_tank_direct_normalized(motor_normalized_speed_t left_normalized_speed,
                                           motor_normalized_speed_t right_normalized_speed)
{
    ESP_RETURN_ON_ERROR(lock_motor(), MOTOR_TAG, "Motor unavailable");
    const motor_normalized_speed_t left_target = motor_clamp_normalized(left_normalized_speed);
    const motor_normalized_speed_t right_target = motor_clamp_normalized(right_normalized_speed);
    const motor_normalized_speed_t targets[MOTOR_COUNT] = {
        [MOTOR_FRONT_LEFT] = left_target,
        [MOTOR_FRONT_RIGHT] = right_target,
        [MOTOR_REAR_LEFT] = left_target,
        [MOTOR_REAR_RIGHT] = right_target,
    };
    const esp_err_t result = apply_targets_locked(targets, MOTOR_ALL_MASK, false);
    xSemaphoreGive(s_lock);
    return result;
}

esp_err_t motor_set_normalized(motor_id_t motor, motor_normalized_speed_t normalized_speed)
{
    ESP_RETURN_ON_FALSE((unsigned)motor < MOTOR_COUNT, ESP_ERR_INVALID_ARG, MOTOR_TAG,
                        "Invalid motor ID");
    ESP_RETURN_ON_ERROR(lock_motor(), MOTOR_TAG, "Motor unavailable");

    motor_normalized_speed_t targets[MOTOR_COUNT] = {0};
    targets[motor] = motor_clamp_normalized(normalized_speed);
    const esp_err_t result = apply_targets_locked(targets, MOTOR_MASK(motor), true);
    xSemaphoreGive(s_lock);
    return result;
}

esp_err_t motor_drive_normalized(motor_normalized_speed_t normalized_throttle,
                                 motor_normalized_speed_t normalized_steering)
{
    const motor_mix_output_t output = motor_mix(normalized_throttle, normalized_steering);
    return motor_set_tank_normalized(output.left_speed, output.right_speed);
}

esp_err_t motor_set_percent(motor_id_t motor, motor_speed_percent_t speed_percent)
{
    return motor_set_normalized(motor, motor_percent_to_normalized(speed_percent));
}

esp_err_t motor_set_tank_percent(motor_speed_percent_t left_speed_percent,
                                 motor_speed_percent_t right_speed_percent)
{
    return motor_set_tank_normalized(motor_percent_to_normalized(left_speed_percent),
                                     motor_percent_to_normalized(right_speed_percent));
}

esp_err_t motor_drive_percent(motor_speed_percent_t throttle_percent,
                              motor_speed_percent_t steering_percent)
{
    return motor_drive_normalized(motor_percent_to_normalized(throttle_percent),
                                  motor_percent_to_normalized(steering_percent));
}

esp_err_t motor_set(motor_id_t motor, motor_normalized_speed_t normalized_speed)
{
    return motor_set_normalized(motor, normalized_speed);
}

esp_err_t motor_set_tank(motor_normalized_speed_t left_normalized_speed,
                         motor_normalized_speed_t right_normalized_speed)
{
    return motor_set_tank_normalized(left_normalized_speed, right_normalized_speed);
}

esp_err_t motor_drive(motor_normalized_speed_t normalized_throttle,
                      motor_normalized_speed_t normalized_steering)
{
    return motor_drive_normalized(normalized_throttle, normalized_steering);
}

esp_err_t motor_stop(void)
{
    ESP_RETURN_ON_ERROR(lock_motor(), MOTOR_TAG, "Motor unavailable");
    const esp_err_t result = backend_stop_all();
    clear_last_speeds();
    xSemaphoreGive(s_lock);
    return result;
}
