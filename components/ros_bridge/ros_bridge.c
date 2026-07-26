#include "ros_bridge.h"

#include "drive.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "ros_diagnostics.h"
#include "ros_messages.h"
#include "ros_serial.h"

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <geometry_msgs/msg/twist.h>
#include <math.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microros/rmw_microros.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ROS_TASK_STACK 14336
#define ROS_TASK_PRIORITY 5
#define ROS_PING_TIMEOUT_MS 100
#define ROS_RECONNECT_DELAY_MS 500
#define ROS_CONNECTED_PING_PERIOD_US ((int64_t)CONFIG_ROS_AGENT_PING_PERIOD_MS * 1000)
#define ROS_TIME_SYNC_RETRY_PERIOD_US ((int64_t)CONFIG_ROS_TIME_SYNC_RETRY_MS * 1000)
#define ROS_IMU_PERIOD_US (1000000LL / CONFIG_ROS_IMU_PUBLISH_RATE_HZ)
#define ROS_DRIVE_PERIOD_US (1000000LL / CONFIG_ROS_DRIVE_PUBLISH_RATE_HZ)
#define ROS_DIAGNOSTIC_PERIOD_US (1000000LL / CONFIG_ROS_DIAGNOSTIC_PUBLISH_RATE_HZ)

enum
{
    ROS_ENTITY_SUPPORT = 1U << 0,
    ROS_ENTITY_NODE = 1U << 1,
    ROS_ENTITY_SUBSCRIPTION = 1U << 2,
    ROS_ENTITY_IMU_PUBLISHER = 1U << 3,
    ROS_ENTITY_JOINT_PUBLISHER = 1U << 4,
    ROS_ENTITY_ODOM_PUBLISHER = 1U << 5,
    ROS_ENTITY_DIAGNOSTIC_PUBLISHER = 1U << 6,
    ROS_ENTITY_EXECUTOR = 1U << 7,
};

typedef enum
{
    ROS_SESSION_WAITING_AGENT,
    ROS_SESSION_CREATING_ENTITIES,
    ROS_SESSION_CONNECTED,
    ROS_SESSION_RECOVERING,
} ros_session_state_t;

typedef struct
{
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_subscription_t command_subscription;
    rcl_publisher_t imu_publisher;
    rcl_publisher_t joint_publisher;
    rcl_publisher_t odom_publisher;
    rcl_publisher_t diagnostic_publisher;
    ros_messages_t messages;
    uint32_t entity_mask;
    uint32_t local_faults;
    uint8_t imu_publish_failures;
    uint8_t drive_publish_failures;
    uint8_t diagnostic_publish_failures;
} ros_context_t;

static uart_port_t s_uart_port = UART_NUM_0;
static TaskHandle_t s_ros_task;
static ros_session_state_t s_session_state = ROS_SESSION_WAITING_AGENT;
static int64_t s_last_command_us;

static void ignore_rcl_result(rcl_ret_t result) { (void)result; }

static builtin_interfaces__msg__Time ros_now(void)
{
    if (!rmw_uros_epoch_synchronized())
    {
        return (builtin_interfaces__msg__Time){0};
    }
    int64_t nanoseconds = rmw_uros_epoch_nanos();
    return (builtin_interfaces__msg__Time){
        .sec = (int32_t)(nanoseconds / 1000000000LL),
        .nanosec = (uint32_t)(nanoseconds % 1000000000LL),
    };
}

static const char* session_state_name(void)
{
    switch (s_session_state)
    {
    case ROS_SESSION_WAITING_AGENT:
        return "WAITING_AGENT";
    case ROS_SESSION_CREATING_ENTITIES:
        return "CREATING_ENTITIES";
    case ROS_SESSION_CONNECTED:
        return "CONNECTED";
    case ROS_SESSION_RECOVERING:
        return "RECOVERING";
    default:
        return "UNKNOWN";
    }
}

static void command_callback(const void* message)
{
    const geometry_msgs__msg__Twist* command = message;
    if (!isfinite(command->linear.x) || !isfinite(command->angular.z) ||
        drive_set_twist((float)command->linear.x, (float)command->angular.z) != ESP_OK)
    {
        (void)drive_stop();
        return;
    }
    s_last_command_us = esp_timer_get_time();
}

static bool create_entities(ros_context_t* context)
{
    context->allocator = rcl_get_default_allocator();
    context->support = (rclc_support_t){0};
    context->node = rcl_get_zero_initialized_node();
    context->executor = rclc_executor_get_zero_initialized_executor();
    context->command_subscription = rcl_get_zero_initialized_subscription();
    context->imu_publisher = rcl_get_zero_initialized_publisher();
    context->joint_publisher = rcl_get_zero_initialized_publisher();
    context->odom_publisher = rcl_get_zero_initialized_publisher();
    context->diagnostic_publisher = rcl_get_zero_initialized_publisher();

    if (rclc_support_init(&context->support, 0, NULL, &context->allocator) != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_SUPPORT;
    if (rclc_node_init_default(&context->node, "vehicle_ecu", "", &context->support) != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_NODE;
    if (rclc_subscription_init_default(&context->command_subscription, &context->node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       "cmd_vel") != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_SUBSCRIPTION;
    if (rclc_publisher_init_best_effort(&context->imu_publisher, &context->node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                        "imu/data_raw") != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_IMU_PUBLISHER;
    if (rclc_publisher_init_best_effort(&context->joint_publisher, &context->node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                        "joint_states") != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_JOINT_PUBLISHER;
    if (rclc_publisher_init_best_effort(&context->odom_publisher, &context->node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                        "odom") != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_ODOM_PUBLISHER;
    if (rclc_publisher_init_default(
            &context->diagnostic_publisher, &context->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
            "diagnostics") != RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_DIAGNOSTIC_PUBLISHER;
    if (rclc_executor_init(&context->executor, &context->support.context, 1, &context->allocator) !=
        RCL_RET_OK)
        return false;
    context->entity_mask |= ROS_ENTITY_EXECUTOR;
    if (rclc_executor_add_subscription(&context->executor, &context->command_subscription,
                                       &context->messages.command, command_callback,
                                       ON_NEW_DATA) != RCL_RET_OK)
        return false;

    (void)rmw_uros_sync_session(1000);
    return true;
}

static void destroy_entities(ros_context_t* context)
{
    (void)drive_stop();
    if (context->entity_mask & ROS_ENTITY_SUPPORT)
    {
        rmw_context_t* rmw_context = rcl_context_get_rmw_context(&context->support.context);
        if (rmw_context != NULL)
        {
            rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }
    if (context->entity_mask & ROS_ENTITY_EXECUTOR)
        (void)rclc_executor_fini(&context->executor);
    if (context->entity_mask & ROS_ENTITY_DIAGNOSTIC_PUBLISHER)
        ignore_rcl_result(rcl_publisher_fini(&context->diagnostic_publisher, &context->node));
    if (context->entity_mask & ROS_ENTITY_ODOM_PUBLISHER)
        ignore_rcl_result(rcl_publisher_fini(&context->odom_publisher, &context->node));
    if (context->entity_mask & ROS_ENTITY_JOINT_PUBLISHER)
        ignore_rcl_result(rcl_publisher_fini(&context->joint_publisher, &context->node));
    if (context->entity_mask & ROS_ENTITY_IMU_PUBLISHER)
        ignore_rcl_result(rcl_publisher_fini(&context->imu_publisher, &context->node));
    if (context->entity_mask & ROS_ENTITY_SUBSCRIPTION)
        ignore_rcl_result(rcl_subscription_fini(&context->command_subscription, &context->node));
    if (context->entity_mask & ROS_ENTITY_NODE)
        ignore_rcl_result(rcl_node_fini(&context->node));
    if (context->entity_mask & ROS_ENTITY_SUPPORT)
        (void)rclc_support_fini(&context->support);
    context->entity_mask = 0;
}

static void fini_messages(ros_context_t* context) { ros_messages_finalize(&context->messages); }

static uint32_t publish_imu(ros_context_t* context)
{
    static uint16_t consecutive_read_failures;
    imu_sample_t sample;
    if (imu_read(&sample) != ESP_OK)
    {
        consecutive_read_failures++;
        if (consecutive_read_failures == 1 || consecutive_read_failures >= 100)
        {
            (void)imu_recover();
            if (consecutive_read_failures >= 100)
            {
                consecutive_read_failures = 1;
            }
        }
        return ROS_FAULT_IMU_READ;
    }
    consecutive_read_failures = 0;
    ros_messages_map_imu(&context->messages, &sample, ros_now());
    return rcl_publish(&context->imu_publisher, &context->messages.imu, NULL) == RCL_RET_OK
               ? 0
               : ROS_FAULT_IMU_PUBLISH;
}

static uint32_t publish_drive(ros_context_t* context)
{
    drive_state_t state;
    if (drive_get_state(&state) != ESP_OK)
    {
        return ROS_FAULT_DRIVE_PUBLISH;
    }
    const builtin_interfaces__msg__Time stamp = ros_now();
    ros_messages_map_drive(&context->messages, &state, stamp);
    const rcl_ret_t joint_result =
        rcl_publish(&context->joint_publisher, &context->messages.joints, NULL);
    const rcl_ret_t odom_result =
        rcl_publish(&context->odom_publisher, &context->messages.odom, NULL);
    return joint_result == RCL_RET_OK && odom_result == RCL_RET_OK ? 0 : ROS_FAULT_DRIVE_PUBLISH;
}

static bool publish_diagnostics(ros_context_t* context)
{
    drive_state_t state = {0};
    const bool drive_state_valid = drive_get_state(&state) == ESP_OK;
    const bool time_synchronized = rmw_uros_epoch_synchronized();
    if (time_synchronized)
    {
        context->local_faults &= ~ROS_FAULT_TIME;
    }
    else
    {
        context->local_faults |= ROS_FAULT_TIME;
    }
    context->messages.diagnostic.header.stamp = ros_now();
    const int64_t age_ms =
        s_last_command_us > 0 ? (esp_timer_get_time() - s_last_command_us) / 1000 : 0;
    const ros_diagnostics_input_t input = {
        .session_state = session_state_name(),
        .time_synchronized = time_synchronized,
        .drive_state_valid = drive_state_valid,
        .drive_state = &state,
        .local_faults = context->local_faults,
        .command_age_ms = age_ms,
        .imu_calibrated = imu_is_calibrated(),
    };
    ros_diagnostics_update(&context->messages.diagnostic, &input);

    return rcl_publish(&context->diagnostic_publisher, &context->messages.diagnostic, NULL) ==
           RCL_RET_OK;
}

static void run_connected_session(ros_context_t* context)
{
    int64_t last_imu = 0;
    int64_t last_drive = 0;
    int64_t last_diagnostic = 0;
    int64_t last_ping = esp_timer_get_time();
    int64_t last_time_sync_attempt = 0;
    for (;;)
    {
        const rcl_ret_t spin_result = rclc_executor_spin_some(&context->executor, RCL_MS_TO_NS(5));
        if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT)
        {
            break;
        }
        const int64_t now = esp_timer_get_time();
        if (now - last_ping >= ROS_CONNECTED_PING_PERIOD_US)
        {
            if (rmw_uros_ping_agent(ROS_PING_TIMEOUT_MS, 1) != RMW_RET_OK)
            {
                break;
            }
            last_ping = now;
        }
        if (!rmw_uros_epoch_synchronized() &&
            now - last_time_sync_attempt >= ROS_TIME_SYNC_RETRY_PERIOD_US)
        {
            (void)rmw_uros_sync_session(100);
            last_time_sync_attempt = now;
        }

        const bool time_synchronized = rmw_uros_epoch_synchronized();
        if (time_synchronized && now - last_imu >= ROS_IMU_PERIOD_US)
        {
            const uint32_t imu_fault = publish_imu(context);
            context->local_faults =
                (context->local_faults & ~(ROS_FAULT_IMU_READ | ROS_FAULT_IMU_PUBLISH)) | imu_fault;
            if ((imu_fault & ROS_FAULT_IMU_PUBLISH) != 0)
            {
                context->imu_publish_failures++;
            }
            else if ((imu_fault & ROS_FAULT_IMU_READ) == 0)
            {
                context->imu_publish_failures = 0;
            }
            last_imu = now;
        }
        if (time_synchronized && now - last_drive >= ROS_DRIVE_PERIOD_US)
        {
            const uint32_t drive_fault = publish_drive(context);
            context->local_faults =
                (context->local_faults & ~ROS_FAULT_DRIVE_PUBLISH) | drive_fault;
            context->drive_publish_failures =
                drive_fault != 0 ? context->drive_publish_failures + 1 : 0;
            last_drive = now;
        }
        if (now - last_diagnostic >= ROS_DIAGNOSTIC_PERIOD_US)
        {
            if (publish_diagnostics(context))
            {
                context->local_faults &= ~ROS_FAULT_DIAGNOSTIC_PUBLISH;
                context->diagnostic_publish_failures = 0;
            }
            else
            {
                context->local_faults |= ROS_FAULT_DIAGNOSTIC_PUBLISH;
                context->diagnostic_publish_failures++;
            }
            last_diagnostic = now;
        }
        if (context->imu_publish_failures >= CONFIG_ROS_PUBLISH_FAILURE_LIMIT ||
            context->drive_publish_failures >= CONFIG_ROS_PUBLISH_FAILURE_LIMIT ||
            context->diagnostic_publish_failures >= CONFIG_ROS_PUBLISH_FAILURE_LIMIT)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void ros_task(void* argument)
{
    (void)argument;
    rmw_uros_set_custom_transport(true, &s_uart_port, ros_serial_open, ros_serial_close,
                                  ros_serial_write, ros_serial_read);
    ros_context_t context = {0};
    for (;;)
    {
        switch (s_session_state)
        {
        case ROS_SESSION_WAITING_AGENT:
            (void)drive_stop();
            if (rmw_uros_ping_agent(ROS_PING_TIMEOUT_MS, 1) == RMW_RET_OK)
            {
                s_session_state = ROS_SESSION_CREATING_ENTITIES;
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(ROS_RECONNECT_DELAY_MS));
            }
            break;

        case ROS_SESSION_CREATING_ENTITIES:
            memset(&context, 0, sizeof(context));
            if (ros_messages_initialize(&context.messages) && create_entities(&context))
            {
                s_session_state = ROS_SESSION_CONNECTED;
            }
            else
            {
                s_session_state = ROS_SESSION_RECOVERING;
            }
            break;

        case ROS_SESSION_CONNECTED:
            run_connected_session(&context);
            s_session_state = ROS_SESSION_RECOVERING;
            break;

        case ROS_SESSION_RECOVERING:
            (void)drive_stop();
            s_last_command_us = 0;
            destroy_entities(&context);
            fini_messages(&context);
            memset(&context, 0, sizeof(context));
            s_session_state = ROS_SESSION_WAITING_AGENT;
            vTaskDelay(pdMS_TO_TICKS(ROS_RECONNECT_DELAY_MS));
            break;
        }
    }
}

esp_err_t ros_bridge_start(void)
{
    if (s_ros_task != NULL)
    {
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(xTaskCreate(ros_task, "ros_bridge", ROS_TASK_STACK, NULL, ROS_TASK_PRIORITY,
                                    &s_ros_task) == pdPASS,
                        ESP_ERR_NO_MEM, "ros_bridge", "Could not create micro-ROS task");
    return ESP_OK;
}
