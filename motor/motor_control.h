#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float vd, vq, iq_ref;
    int32_t i_d, i_q;
    uint32_t spi_error_count;
    uint32_t encoder_request_count;
    uint32_t encoder_response_count;
    uint32_t encoder_timeout_count;
    uint32_t encoder_last_response_us;
    uint32_t encoder_max_response_us;
    bool spi_error;
} motor_status_t;

void motor_control_init(void);
void motor_read_position(int32_t *angle, int32_t *rotation_count);
void motor_set_ffb_command(int16_t magnitude, int8_t rotation_limit);
void motor_set_gate_enabled(bool enabled);
motor_status_t motor_get_status(void);

#endif
