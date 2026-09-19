#ifndef ENCODER_UART_H
#define ENCODER_UART_H

#include <stdbool.h>
#include <stdint.h>

// Initializes the RS-485 UART encoder and its DMA receiver.
void encoder_uart_init(void);

// Position corresponding to the configured mechanical zero before the first
// asynchronous response arrives.
int32_t encoder_uart_initial_position(void);

// Starts one asynchronous position request. Returns false while a prior request
// or its unread response is still pending.
bool encoder_uart_request_position(void);

// Decodes a completed position response, if one is available.
bool encoder_uart_read_position(int32_t *angle_17bit);

typedef struct {
    uint32_t request_count;
    uint32_t response_count;
    uint32_t timeout_count;
    uint32_t last_response_us;
    uint32_t max_response_us;
} encoder_uart_stats_t;

// Copies communication statistics. Counters start at encoder_uart_init().
// Response time is measured after the request byte leaves the UART.
encoder_uart_stats_t encoder_uart_get_stats(void);

#endif
