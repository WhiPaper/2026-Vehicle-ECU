#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float left_rpm;
    float right_rpm;
} drive_command_t;

typedef struct
{
    QueueHandle_t queue;
    StaticQueue_t queue_control;
    uint8_t queue_storage[sizeof(drive_command_t)];
    bool emergency_stop;
} drive_mailbox_t;

bool drive_mailbox_init(drive_mailbox_t* mailbox);
bool drive_mailbox_overwrite(drive_mailbox_t* mailbox, const drive_command_t* command);
bool drive_mailbox_take(drive_mailbox_t* mailbox, drive_command_t* command);
void drive_mailbox_request_stop(drive_mailbox_t* mailbox);
bool drive_mailbox_take_stop(drive_mailbox_t* mailbox);
void drive_mailbox_clear(drive_mailbox_t* mailbox);
