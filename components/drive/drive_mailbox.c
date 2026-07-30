#include "drive_mailbox.h"

#include <stddef.h>
#include <string.h>

bool drive_mailbox_init(drive_mailbox_t* mailbox)
{
    if (mailbox == NULL)
    {
        return false;
    }
    memset(mailbox, 0, sizeof(*mailbox));
    mailbox->queue = xQueueCreateStatic(1, sizeof(drive_command_t), mailbox->queue_storage,
                                        &mailbox->queue_control);
    return mailbox->queue != NULL;
}

bool drive_mailbox_overwrite(drive_mailbox_t* mailbox, const drive_command_t* command)
{
    return mailbox != NULL && mailbox->queue != NULL && command != NULL &&
           xQueueOverwrite(mailbox->queue, command) == pdTRUE;
}

bool drive_mailbox_take(drive_mailbox_t* mailbox, drive_command_t* command)
{
    return mailbox != NULL && mailbox->queue != NULL && command != NULL &&
           xQueueReceive(mailbox->queue, command, 0) == pdTRUE;
}

void drive_mailbox_request_stop(drive_mailbox_t* mailbox)
{
    if (mailbox != NULL)
    {
        __atomic_store_n(&mailbox->emergency_stop, true, __ATOMIC_RELEASE);
    }
}

bool drive_mailbox_take_stop(drive_mailbox_t* mailbox)
{
    return mailbox != NULL &&
           __atomic_exchange_n(&mailbox->emergency_stop, false, __ATOMIC_ACQ_REL);
}

void drive_mailbox_clear(drive_mailbox_t* mailbox)
{
    if (mailbox != NULL && mailbox->queue != NULL)
    {
        (void)xQueueReset(mailbox->queue);
    }
}
