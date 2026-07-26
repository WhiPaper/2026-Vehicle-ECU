#include "ros_serial.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <uxr/client/transport.h>

#define ROS_UART_RX_BUFFER_SIZE 4096

bool ros_serial_open(struct uxrCustomTransport* transport)
{
    const uart_port_t port = *(uart_port_t*)transport->args;
    const uart_config_t config = {
        .baud_rate = CONFIG_ROS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(port, &config) != ESP_OK ||
        uart_set_pin(port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_driver_install(port, ROS_UART_RX_BUFFER_SIZE, 0, 0, NULL, 0) != ESP_OK)
    {
        return false;
    }
    (void)uart_flush(port);
    return true;
}

bool ros_serial_close(struct uxrCustomTransport* transport)
{
    const uart_port_t port = *(uart_port_t*)transport->args;
    return uart_driver_delete(port) == ESP_OK;
}

size_t ros_serial_write(struct uxrCustomTransport* transport, const uint8_t* buffer, size_t length,
                        uint8_t* error)
{
    const uart_port_t port = *(uart_port_t*)transport->args;
    const int written = uart_write_bytes(port, buffer, length);
    if (error != NULL)
    {
        *error = written < 0;
    }
    return written > 0 ? (size_t)written : 0;
}

size_t ros_serial_read(struct uxrCustomTransport* transport, uint8_t* buffer, size_t length,
                       int timeout_ms, uint8_t* error)
{
    const uart_port_t port = *(uart_port_t*)transport->args;
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms > 0 && ticks == 0)
    {
        ticks = 1;
    }
    const int received = uart_read_bytes(port, buffer, length, ticks);
    if (error != NULL)
    {
        *error = received < 0;
    }
    return received > 0 ? (size_t)received : 0;
}
