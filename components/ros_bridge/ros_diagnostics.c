#include "ros_diagnostics.h"

#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <rosidl_runtime_c/string_functions.h>

#include <stdio.h>
#include <string.h>

#define DIAGNOSTIC_VALUE_CAPACITY 32
#define DIAGNOSTIC_MESSAGE_CAPACITY 64

enum
{
    DIAGNOSTIC_TRANSPORT = 0,
    DIAGNOSTIC_DRIVE = 1,
    DIAGNOSTIC_IMU = 2,
    DIAGNOSTIC_STATUS_COUNT = 3,
};

static bool string_reserve(rosidl_runtime_c__String* string, size_t capacity)
{
    char placeholder[DIAGNOSTIC_MESSAGE_CAPACITY + 1];
    if (capacity > DIAGNOSTIC_MESSAGE_CAPACITY)
    {
        return false;
    }
    memset(placeholder, ' ', capacity);
    placeholder[capacity] = '\0';
    return rosidl_runtime_c__String__assign(string, placeholder);
}

static void string_write(rosidl_runtime_c__String* string, const char* value)
{
    if (string->data == NULL || string->capacity == 0)
    {
        return;
    }
    const size_t length = strnlen(value, string->capacity - 1);
    memcpy(string->data, value, length);
    string->data[length] = '\0';
    string->size = length;
}

static bool initialize_status(diagnostic_msgs__msg__DiagnosticStatus* status, const char* name,
                              size_t value_count, const char* const keys[])
{
    if (!rosidl_runtime_c__String__assign(&status->name, name) ||
        !rosidl_runtime_c__String__assign(&status->hardware_id, "esp32-vehicle-ecu") ||
        !string_reserve(&status->message, DIAGNOSTIC_MESSAGE_CAPACITY) ||
        !diagnostic_msgs__msg__KeyValue__Sequence__init(&status->values, value_count))
    {
        return false;
    }
    for (size_t i = 0; i < value_count; ++i)
    {
        if (!rosidl_runtime_c__String__assign(&status->values.data[i].key, keys[i]) ||
            !string_reserve(&status->values.data[i].value, DIAGNOSTIC_VALUE_CAPACITY))
        {
            return false;
        }
    }
    return true;
}

bool ros_diagnostics_initialize(diagnostic_msgs__msg__DiagnosticArray* diagnostic)
{
    static const char* const transport_keys[] = {
        "session_state",
        "agent_connected",
        "time_synchronized",
        "last_error",
    };
    static const char* const drive_keys[] = {
        "calibrated", "command_active", "command_age_ms", "encoder_ok",
        "stalled",    "motor_ok",       "fault_mask",
    };
    static const char* const imu_keys[] = {
        "imu_ok",
        "calibrated",
        "last_error",
    };

    if (!diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&diagnostic->status,
                                                                DIAGNOSTIC_STATUS_COUNT))
    {
        return false;
    }
    return initialize_status(&diagnostic->status.data[DIAGNOSTIC_TRANSPORT],
                             "vehicle_ecu/transport", 4, transport_keys) &&
           initialize_status(&diagnostic->status.data[DIAGNOSTIC_DRIVE], "vehicle_ecu/drive", 7,
                             drive_keys) &&
           initialize_status(&diagnostic->status.data[DIAGNOSTIC_IMU], "vehicle_ecu/imu", 3,
                             imu_keys);
}

static void set_key_value(diagnostic_msgs__msg__DiagnosticStatus* status, size_t index,
                          const char* value)
{
    string_write(&status->values.data[index].value, value);
}

static void set_bool_value(diagnostic_msgs__msg__DiagnosticStatus* status, size_t index, bool value)
{
    set_key_value(status, index, value ? "true" : "false");
}

void ros_diagnostics_update(diagnostic_msgs__msg__DiagnosticArray* diagnostic,
                            const ros_diagnostics_input_t* input)
{
    const bool ros_publish_ok =
        (input->local_faults &
         (ROS_FAULT_IMU_PUBLISH | ROS_FAULT_DRIVE_PUBLISH | ROS_FAULT_DIAGNOSTIC_PUBLISH)) == 0;
    diagnostic_msgs__msg__DiagnosticStatus* transport =
        &diagnostic->status.data[DIAGNOSTIC_TRANSPORT];
    transport->level = input->time_synchronized && ros_publish_ok
                           ? diagnostic_msgs__msg__DiagnosticStatus__OK
                           : diagnostic_msgs__msg__DiagnosticStatus__WARN;
    string_write(&transport->message, input->time_synchronized
                                          ? (ros_publish_ok ? "connected" : "publisher failure")
                                          : "epoch not synchronized");
    set_key_value(transport, 0, input->session_state);
    set_bool_value(transport, 1, true);
    set_bool_value(transport, 2, input->time_synchronized);
    set_key_value(transport, 3,
                  !input->time_synchronized ? "time synchronization failed"
                                            : (!ros_publish_ok ? "ROS publisher failure" : "none"));

    const drive_state_t zero_state = {0};
    const drive_state_t* state = input->drive_state != NULL ? input->drive_state : &zero_state;
    const bool calibrated =
        input->drive_state_valid && (state->faults & DRIVE_FAULT_NOT_CALIBRATED) == 0;
    const bool encoder_ok = input->drive_state_valid && (state->faults & DRIVE_FAULT_ENCODER) == 0;
    const bool stalled = input->drive_state_valid && (state->faults & DRIVE_FAULT_STALL) != 0;
    const bool motor_ok = input->drive_state_valid && (state->faults & DRIVE_FAULT_MOTOR) == 0;
    const bool drive_error =
        input->drive_state_valid && (!calibrated || !encoder_ok || stalled || !motor_ok);
    const bool drive_publish_ok = (input->local_faults & ROS_FAULT_DRIVE_PUBLISH) == 0;

    diagnostic_msgs__msg__DiagnosticStatus* drive = &diagnostic->status.data[DIAGNOSTIC_DRIVE];
    drive->level = !input->drive_state_valid ? diagnostic_msgs__msg__DiagnosticStatus__STALE
                   : drive_error
                       ? diagnostic_msgs__msg__DiagnosticStatus__ERROR
                       : (!drive_publish_ok || (state->faults & DRIVE_FAULT_COMMAND_TIMEOUT) != 0
                              ? diagnostic_msgs__msg__DiagnosticStatus__WARN
                              : diagnostic_msgs__msg__DiagnosticStatus__OK);
    string_write(&drive->message, !input->drive_state_valid ? "drive state unavailable"
                                  : drive_error             ? "drive unavailable"
                                  : !drive_publish_ok       ? "drive state publish failed"
                                  : drive->level == diagnostic_msgs__msg__DiagnosticStatus__WARN
                                      ? "command timeout"
                                      : "ready");
    set_bool_value(drive, 0, calibrated);
    set_bool_value(drive, 1, state->command_active);
    char value[DIAGNOSTIC_VALUE_CAPACITY + 1];
    snprintf(value, sizeof(value), "%lld", (long long)input->command_age_ms);
    set_key_value(drive, 2, value);
    set_bool_value(drive, 3, encoder_ok);
    set_bool_value(drive, 4, stalled);
    set_bool_value(drive, 5, motor_ok);
    snprintf(value, sizeof(value), "0x%08lx", (unsigned long)(state->faults | input->local_faults));
    set_key_value(drive, 6, value);

    const bool imu_ok = (input->local_faults & ROS_FAULT_IMU_READ) == 0;
    const bool imu_publish_ok = (input->local_faults & ROS_FAULT_IMU_PUBLISH) == 0;
    diagnostic_msgs__msg__DiagnosticStatus* imu = &diagnostic->status.data[DIAGNOSTIC_IMU];
    imu->level = imu_ok && input->imu_calibrated && imu_publish_ok
                     ? diagnostic_msgs__msg__DiagnosticStatus__OK
                     : diagnostic_msgs__msg__DiagnosticStatus__WARN;
    string_write(&imu->message, !imu_ok                  ? "IMU sample unavailable"
                                : !input->imu_calibrated ? "IMU not calibrated"
                                : !imu_publish_ok        ? "IMU publish failed"
                                                         : "ready");
    set_bool_value(imu, 0, imu_ok);
    set_bool_value(imu, 1, input->imu_calibrated);
    set_key_value(imu, 2,
                  !imu_ok                  ? "sample unavailable"
                  : !input->imu_calibrated ? "stationary calibration incomplete"
                  : !imu_publish_ok        ? "ROS publisher failure"
                                           : "none");
}
