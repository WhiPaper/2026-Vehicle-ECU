#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct uxrCustomTransport;

bool ros_serial_open(struct uxrCustomTransport* transport);
bool ros_serial_close(struct uxrCustomTransport* transport);
size_t ros_serial_write(struct uxrCustomTransport* transport, const uint8_t* buffer, size_t length,
                        uint8_t* error);
size_t ros_serial_read(struct uxrCustomTransport* transport, uint8_t* buffer, size_t length,
                       int timeout_ms, uint8_t* error);
