#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "unity.h"
#include "unity_test_runner.h"

#define TEST_SPEED_PERCENT 60
#define DIRECTION_TIME_MS 800
#define DIRECTION_PAUSE_MS 500
#define SUSTAINED_TEST_SECONDS 12
#define FULL_OUTPUT_TEST_SECONDS 3
#define MIN_OUTPUT_STEP_PERCENT 5
#define MIN_OUTPUT_HOLD_MS 2000
#define MIN_OUTPUT_STOP_MS 1000

#ifdef CONFIG_MOTOR_FRONT_LEFT_INVERTED
#define TEST_FRONT_LEFT_INVERTED true
#else
#define TEST_FRONT_LEFT_INVERTED false
#endif
#ifdef CONFIG_MOTOR_FRONT_RIGHT_INVERTED
#define TEST_FRONT_RIGHT_INVERTED true
#else
#define TEST_FRONT_RIGHT_INVERTED false
#endif
#ifdef CONFIG_MOTOR_REAR_LEFT_INVERTED
#define TEST_REAR_LEFT_INVERTED true
#else
#define TEST_REAR_LEFT_INVERTED false
#endif
#ifdef CONFIG_MOTOR_REAR_RIGHT_INVERTED
#define TEST_REAR_RIGHT_INVERTED true
#else
#define TEST_REAR_RIGHT_INVERTED false
#endif

static const motor_config_t s_test_motor_config = {
    .channels =
        {
            {
                .ia_gpio = CONFIG_MOTOR_FRONT_LEFT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_FRONT_LEFT_IB_GPIO,
                .inverted = TEST_FRONT_LEFT_INVERTED,
            },
            {
                .ia_gpio = CONFIG_MOTOR_FRONT_RIGHT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_FRONT_RIGHT_IB_GPIO,
                .inverted = TEST_FRONT_RIGHT_INVERTED,
            },
            {
                .ia_gpio = CONFIG_MOTOR_REAR_LEFT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_REAR_LEFT_IB_GPIO,
                .inverted = TEST_REAR_LEFT_INVERTED,
            },
            {
                .ia_gpio = CONFIG_MOTOR_REAR_RIGHT_IA_GPIO,
                .ib_gpio = CONFIG_MOTOR_REAR_RIGHT_IB_GPIO,
                .inverted = TEST_REAR_RIGHT_INVERTED,
            },
        },
    .pwm_frequency_hz = CONFIG_MOTOR_PWM_FREQUENCY_HZ,
};

static const char* TAG = "motor_hardware_test";

static void stop_motors_and_wait(void)
{
    TEST_ESP_OK(motor_stop());
    vTaskDelay(pdMS_TO_TICKS(DIRECTION_PAUSE_MS));
}

static void test_single_motor(motor_id_t motor, const char* name)
{
    ESP_LOGW(TAG, "%s: forward", name);
    TEST_ESP_OK(motor_set_percent(motor, TEST_SPEED_PERCENT));
    vTaskDelay(pdMS_TO_TICKS(DIRECTION_TIME_MS));
    stop_motors_and_wait();

    ESP_LOGW(TAG, "%s: reverse", name);
    TEST_ESP_OK(motor_set_percent(motor, -TEST_SPEED_PERCENT));
    vTaskDelay(pdMS_TO_TICKS(DIRECTION_TIME_MS));
    stop_motors_and_wait();
}

static void test_tank(motor_speed_percent_t left_speed_percent,
                      motor_speed_percent_t right_speed_percent, const char* name)
{
    ESP_LOGW(TAG, "%s: left=%d%%, right=%d%%", name, left_speed_percent, right_speed_percent);
    TEST_ESP_OK(motor_set_tank_percent(left_speed_percent, right_speed_percent));
    vTaskDelay(pdMS_TO_TICKS(DIRECTION_TIME_MS));
    stop_motors_and_wait();
}

static void test_sustained_forward_percent(motor_speed_percent_t speed_percent,
                                           int duration_seconds, const char* name)
{
    ESP_LOGW(TAG, "%s: command=%d%% for %d seconds", name, speed_percent, duration_seconds);
    TEST_ESP_OK(motor_set_tank_percent(speed_percent, speed_percent));

    for (int elapsed = 1; elapsed <= duration_seconds; ++elapsed)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "%s: %d/%d seconds", name, elapsed, duration_seconds);
    }

    stop_motors_and_wait();
}

static void test_sustained_axle_forward_percent(motor_id_t left_motor, motor_id_t right_motor,
                                                motor_speed_percent_t speed_percent,
                                                int duration_seconds, const char* name)
{
    ESP_LOGW(TAG, "%s: command=%d%% for %d seconds", name, speed_percent, duration_seconds);
    TEST_ESP_OK(motor_set_percent(left_motor, speed_percent));
    TEST_ESP_OK(motor_set_percent(right_motor, speed_percent));

    for (int elapsed = 1; elapsed <= duration_seconds; ++elapsed)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "%s at %d%%: %d/%d seconds", name, speed_percent, elapsed, duration_seconds);
    }

    stop_motors_and_wait();
}

static void set_axle_percent(motor_id_t left_motor, motor_id_t right_motor,
                             motor_speed_percent_t speed_percent)
{
    TEST_ESP_OK(motor_set_percent(left_motor, speed_percent));
    TEST_ESP_OK(motor_set_percent(right_motor, speed_percent));
}

static void test_minimum_start_output(motor_id_t left_motor, motor_id_t right_motor,
                                      const char* name)
{
#if CONFIG_MOTOR_START_BOOST_SPEED > 0
    TEST_FAIL_MESSAGE("Disable MOTOR_START_BOOST_SPEED before measuring minimum start output");
#endif

    ESP_LOGW(TAG, "%s: watch for the first level that starts both wheels", name);

    for (motor_speed_percent_t speed_percent = MIN_OUTPUT_STEP_PERCENT;
         speed_percent <= MOTOR_SPEED_PERCENT_MAX; speed_percent += MIN_OUTPUT_STEP_PERCENT)
    {
        ESP_LOGW(TAG, "%s: start attempt at %d%%", name, speed_percent);
        set_axle_percent(left_motor, right_motor, speed_percent);
        vTaskDelay(pdMS_TO_TICKS(MIN_OUTPUT_HOLD_MS));
        TEST_ESP_OK(motor_stop());
        vTaskDelay(pdMS_TO_TICKS(MIN_OUTPUT_STOP_MS));
    }
}

static void test_minimum_hold_output(motor_id_t left_motor, motor_id_t right_motor,
                                     const char* name)
{
    ESP_LOGW(TAG, "%s: watch for the level where either wheel stops", name);

    for (motor_speed_percent_t speed_percent = MOTOR_SPEED_PERCENT_MAX;
         speed_percent >= MIN_OUTPUT_STEP_PERCENT; speed_percent -= MIN_OUTPUT_STEP_PERCENT)
    {
        ESP_LOGW(TAG, "%s: hold at %d%%", name, speed_percent);
        set_axle_percent(left_motor, right_motor, speed_percent);
        vTaskDelay(pdMS_TO_TICKS(MIN_OUTPUT_HOLD_MS));
    }

    stop_motors_and_wait();
}

void setUp(void) { TEST_ESP_OK(motor_stop()); }

void tearDown(void) { TEST_ESP_OK(motor_stop()); }

TEST_CASE("front-left motor forward and reverse", "[motor][hardware]")
{
    test_single_motor(MOTOR_FRONT_LEFT, "front-left");
}

TEST_CASE("front-right motor forward and reverse", "[motor][hardware]")
{
    test_single_motor(MOTOR_FRONT_RIGHT, "front-right");
}

TEST_CASE("rear-left motor forward and reverse", "[motor][hardware]")
{
    test_single_motor(MOTOR_REAR_LEFT, "rear-left");
}

TEST_CASE("rear-right motor forward and reverse", "[motor][hardware]")
{
    test_single_motor(MOTOR_REAR_RIGHT, "rear-right");
}

TEST_CASE("all motors forward", "[motor][hardware]")
{
    test_tank(TEST_SPEED_PERCENT, TEST_SPEED_PERCENT, "all forward");
}

TEST_CASE("all motors reverse", "[motor][hardware]")
{
    test_tank(-TEST_SPEED_PERCENT, -TEST_SPEED_PERCENT, "all reverse");
}

TEST_CASE("vehicle pivots right", "[motor][hardware]")
{
    test_tank(TEST_SPEED_PERCENT, -TEST_SPEED_PERCENT, "pivot right");
}

TEST_CASE("vehicle pivots left", "[motor][hardware]")
{
    test_tank(-TEST_SPEED_PERCENT, TEST_SPEED_PERCENT, "pivot left");
}

TEST_CASE("all motors sustain forward operation for 12 seconds", "[motor][hardware][sustained]")
{
    test_sustained_forward_percent(TEST_SPEED_PERCENT, SUSTAINED_TEST_SECONDS,
                                   "sustained forward at 60% output");
}

TEST_CASE("front axle sustains 25 percent load for 12 seconds",
          "[motor][hardware][sustained][front-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, 25,
                                        SUSTAINED_TEST_SECONDS, "front-axle load");
}

TEST_CASE("front axle sustains 50 percent load for 12 seconds",
          "[motor][hardware][sustained][front-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, 50,
                                        SUSTAINED_TEST_SECONDS, "front-axle load");
}

TEST_CASE("front axle sustains 75 percent load for 12 seconds",
          "[motor][hardware][sustained][front-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, 75,
                                        SUSTAINED_TEST_SECONDS, "front-axle load");
}

TEST_CASE("front axle sustains 100 percent load for 12 seconds",
          "[motor][hardware][sustained][front-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT,
                                        MOTOR_SPEED_PERCENT_MAX, SUSTAINED_TEST_SECONDS,
                                        "front-axle load");
}

TEST_CASE("rear axle sustains 25 percent load for 12 seconds",
          "[motor][hardware][sustained][rear-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, 25,
                                        SUSTAINED_TEST_SECONDS, "rear-axle load");
}

TEST_CASE("rear axle sustains 50 percent load for 12 seconds",
          "[motor][hardware][sustained][rear-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, 50,
                                        SUSTAINED_TEST_SECONDS, "rear-axle load");
}

TEST_CASE("rear axle sustains 75 percent load for 12 seconds",
          "[motor][hardware][sustained][rear-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, 75,
                                        SUSTAINED_TEST_SECONDS, "rear-axle load");
}

TEST_CASE("rear axle sustains 100 percent load for 12 seconds",
          "[motor][hardware][sustained][rear-axle]")
{
    test_sustained_axle_forward_percent(MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_SPEED_PERCENT_MAX,
                                        SUSTAINED_TEST_SECONDS, "rear-axle load");
}

TEST_CASE("find front axle minimum start output",
          "[motor][hardware][minimum-output][front-axle][start]")
{
    test_minimum_start_output(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, "front-axle minimum start");
}

TEST_CASE("find rear axle minimum start output",
          "[motor][hardware][minimum-output][rear-axle][start]")
{
    test_minimum_start_output(MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, "rear-axle minimum start");
}

TEST_CASE("find front axle minimum hold output",
          "[motor][hardware][minimum-output][front-axle][hold]")
{
    test_minimum_hold_output(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, "front-axle minimum hold");
}

TEST_CASE("find rear axle minimum hold output",
          "[motor][hardware][minimum-output][rear-axle][hold]")
{
    test_minimum_hold_output(MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, "rear-axle minimum hold");
}

TEST_CASE("all motors run at full output without PWM switching for 3 seconds",
          "[motor][hardware][full-output]")
{
    test_sustained_forward_percent(MOTOR_SPEED_PERCENT_MAX, FULL_OUTPUT_TEST_SECONDS,
                                   "full output without PWM switching");
}

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init(&s_test_motor_config));
    ESP_LOGW(TAG, "Interactive hardware test ready at 60%% output");
    ESP_LOGW(TAG, "Lift all wheels before selecting a test");
    unity_run_menu();
}
