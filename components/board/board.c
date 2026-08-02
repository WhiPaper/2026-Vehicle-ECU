#include "board.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include <stddef.h>

#define BOARD_TAG "board"

#ifdef CONFIG_MOTOR_FRONT_LEFT_INVERTED
#define MOTOR_FRONT_LEFT_INVERTED true
#else
#define MOTOR_FRONT_LEFT_INVERTED false
#endif

#ifdef CONFIG_MOTOR_FRONT_RIGHT_INVERTED
#define MOTOR_FRONT_RIGHT_INVERTED true
#else
#define MOTOR_FRONT_RIGHT_INVERTED false
#endif

#ifdef CONFIG_MOTOR_REAR_LEFT_INVERTED
#define MOTOR_REAR_LEFT_INVERTED true
#else
#define MOTOR_REAR_LEFT_INVERTED false
#endif

#ifdef CONFIG_MOTOR_REAR_RIGHT_INVERTED
#define MOTOR_REAR_RIGHT_INVERTED true
#else
#define MOTOR_REAR_RIGHT_INVERTED false
#endif

#ifdef CONFIG_ENCODER_FRONT_LEFT_INVERTED
#define ENCODER_FRONT_LEFT_INVERTED true
#else
#define ENCODER_FRONT_LEFT_INVERTED false
#endif

#ifdef CONFIG_ENCODER_FRONT_RIGHT_INVERTED
#define ENCODER_FRONT_RIGHT_INVERTED true
#else
#define ENCODER_FRONT_RIGHT_INVERTED false
#endif

#ifdef CONFIG_ENCODER_REAR_LEFT_INVERTED
#define ENCODER_REAR_LEFT_INVERTED true
#else
#define ENCODER_REAR_LEFT_INVERTED false
#endif

#ifdef CONFIG_ENCODER_REAR_RIGHT_INVERTED
#define ENCODER_REAR_RIGHT_INVERTED true
#else
#define ENCODER_REAR_RIGHT_INVERTED false
#endif

static const motor_config_t s_motor_config = {
    .channels =
        {
            {
                .ia_gpio = CONFIG_MOTOR_FRONT_LEFT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_FRONT_LEFT_IB_GPIO,
                .inverted = MOTOR_FRONT_LEFT_INVERTED,
            },
            {
                .ia_gpio = CONFIG_MOTOR_FRONT_RIGHT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_FRONT_RIGHT_IB_GPIO,
                .inverted = MOTOR_FRONT_RIGHT_INVERTED,
            },
            {
                .ia_gpio = CONFIG_MOTOR_REAR_LEFT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_REAR_LEFT_IB_GPIO,
                .inverted = MOTOR_REAR_LEFT_INVERTED,
            },
            {
                .ia_gpio = CONFIG_MOTOR_REAR_RIGHT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_REAR_RIGHT_IB_GPIO,
                .inverted = MOTOR_REAR_RIGHT_INVERTED,
            },
        },
    .pwm_frequency_hz = CONFIG_MOTOR_PWM_FREQUENCY_HZ,
};

static const wheel_encoder_config_t s_wheel_encoder_config = {
    .channels =
        {
            {
                .a_gpio = CONFIG_ENCODER_FRONT_LEFT_A_GPIO,
                .b_gpio = CONFIG_ENCODER_FRONT_LEFT_B_GPIO,
                .inverted = ENCODER_FRONT_LEFT_INVERTED,
            },
            {
                .a_gpio = CONFIG_ENCODER_FRONT_RIGHT_A_GPIO,
                .b_gpio = CONFIG_ENCODER_FRONT_RIGHT_B_GPIO,
                .inverted = ENCODER_FRONT_RIGHT_INVERTED,
            },
            {
                .a_gpio = CONFIG_ENCODER_REAR_LEFT_A_GPIO,
                .b_gpio = CONFIG_ENCODER_REAR_LEFT_B_GPIO,
                .inverted = ENCODER_REAR_LEFT_INVERTED,
            },
            {
                .a_gpio = CONFIG_ENCODER_REAR_RIGHT_A_GPIO,
                .b_gpio = CONFIG_ENCODER_REAR_RIGHT_B_GPIO,
                .inverted = ENCODER_REAR_RIGHT_INVERTED,
            },
        },
    .cpr = CONFIG_ENCODER_CPR,
    .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER_NS,
};

static const imu_config_t s_imu_config = {
    .i2c_port = 0,
    .sda_gpio = CONFIG_MPU6050_SDA_GPIO,
    .scl_gpio = CONFIG_MPU6050_SCL_GPIO,
    .address = CONFIG_MPU6050_I2C_ADDRESS,
    .calibration_rate_hz = CONFIG_MPU6050_CALIBRATION_RATE_HZ,
    .calibration_samples = CONFIG_MPU6050_CALIBRATION_SAMPLES,
};

static const drive_config_t s_drive_config = {
    .wheel_radius_m = CONFIG_DRIVE_WHEEL_RADIUS_MM / 1000.0f,
    .track_width_m = CONFIG_DRIVE_TRACK_WIDTH_MM / 1000.0f,
    .max_wheel_rpm = CONFIG_DRIVE_MAX_WHEEL_RPM,
    .pid_kp = CONFIG_DRIVE_PID_KP_MILLI / 1000.0f,
    .pid_ki = CONFIG_DRIVE_PID_KI_MILLI / 1000.0f,
    .pid_kd = CONFIG_DRIVE_PID_KD_MILLI / 1000.0f,
    .control_period_ms = CONFIG_DRIVE_CONTROL_PERIOD_MS,
    .command_timeout_ms = CONFIG_DRIVE_COMMAND_TIMEOUT_MS,
    .stall_timeout_ms = CONFIG_DRIVE_STALL_TIMEOUT_MS,
};

const motor_config_t* board_motor_config(void) { return &s_motor_config; }

const wheel_encoder_config_t* board_wheel_encoder_config(void) { return &s_wheel_encoder_config; }

const imu_config_t* board_imu_config(void) { return &s_imu_config; }

const drive_config_t* board_drive_config(void) { return &s_drive_config; }

static bool pin_is_reserved(int pin)
{
    return pin == 0 || pin == 1 || pin == 2 || pin == 3 || pin == 5 || (pin >= 6 && pin <= 12) ||
           pin == 15;
}

esp_err_t board_validate(void)
{
    int pins[MOTOR_COUNT * 2 + WHEEL_ENCODER_COUNT * 2 + 2];
    size_t pin_count = 0;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const int motor_pins[] = {
            s_motor_config.channels[i].ia_gpio,
            s_motor_config.channels[i].ib_gpio,
        };
        for (size_t j = 0; j < 2; ++j)
        {
            ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(motor_pins[j]), ESP_ERR_INVALID_ARG,
                                BOARD_TAG, "Motor GPIO %d cannot output", motor_pins[j]);
            pins[pin_count++] = motor_pins[j];
        }
    }

    for (size_t i = 0; i < WHEEL_ENCODER_COUNT; ++i)
    {
        const int encoder_pins[] = {
            s_wheel_encoder_config.channels[i].a_gpio,
            s_wheel_encoder_config.channels[i].b_gpio,
        };
        for (size_t j = 0; j < 2; ++j)
        {
            ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(encoder_pins[j]), ESP_ERR_INVALID_ARG, BOARD_TAG,
                                "Encoder GPIO %d is invalid", encoder_pins[j]);
            pins[pin_count++] = encoder_pins[j];
        }
    }

    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(s_imu_config.sda_gpio) &&
                            GPIO_IS_VALID_GPIO(s_imu_config.scl_gpio),
                        ESP_ERR_INVALID_ARG, BOARD_TAG, "IMU I2C GPIO is invalid");
    pins[pin_count++] = s_imu_config.sda_gpio;
    pins[pin_count++] = s_imu_config.scl_gpio;

    for (size_t i = 0; i < pin_count; ++i)
    {
        ESP_RETURN_ON_FALSE(!pin_is_reserved(pins[i]), ESP_ERR_INVALID_ARG, BOARD_TAG,
                            "GPIO %d is reserved", pins[i]);
        for (size_t j = i + 1; j < pin_count; ++j)
        {
            ESP_RETURN_ON_FALSE(pins[i] != pins[j], ESP_ERR_INVALID_ARG, BOARD_TAG,
                                "GPIO %d is assigned twice", pins[i]);
        }
    }
    return ESP_OK;
}
