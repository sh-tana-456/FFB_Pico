#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

// HID axis range and motor-side software end-stop configuration.
#define MOTOR_WHEEL_RANGE_DEGREES 540
#define MOTOR_WHEEL_REDUCTION_RATIO 4
#define MOTOR_ENCODER_COUNTS_PER_REV 131072
#define WHEEL_DIRECTION_REVERSED 0
#define MOTOR_SOFT_LIMIT_HALF_COUNTS \
    ((MOTOR_WHEEL_RANGE_DEGREES * MOTOR_WHEEL_REDUCTION_RATIO * MOTOR_ENCODER_COUNTS_PER_REV) / 720)
#define MOTOR_SOFT_LIMIT_RAMP_COUNTS \
    ((15 * MOTOR_WHEEL_REDUCTION_RATIO * MOTOR_ENCODER_COUNTS_PER_REV) / 360)
#define MOTOR_SOFT_LIMIT_MAX_IQ_MA 5000

typedef struct {
    float vd, vq, iq_ref;
    int32_t i_d, i_q;
    int32_t soft_limit_iq;
    int32_t motor_position_counts;
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
void motor_set_ffb_command(int16_t magnitude);
void motor_set_gate_enabled(bool enabled);
motor_status_t motor_get_status(void);

#endif
