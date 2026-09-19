/**
 * Application entry point: USB HID/CDC and the FFB-to-motor bridge.
 */
#include <stdarg.h>
#include <stdio.h>

#include "bsp/board.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#include "hid/ffb_hid.h"
#include "hid/gamepad_hid.h"
#include "motor/motor_control.h"

#define LED_FFB_ACTIVE 4
#define LED_FFB_MAGNITUDE 25
#define CDC_LOG_INTERVAL_MS 137
#define WHEEL_ANGLE_DEGREES 540.0f
#define WHEEL_REDUCTION_RATIO 4.0f

typedef struct {
    int16_t x;
    int8_t rotation_limit;
} wheel_input_t;

static wheel_input_t wheel_input_from_encoder(int32_t angle_17bit, int32_t rotation_count)
{
    int32_t full_turns = rotation_count < 0 ? -rotation_count : rotation_count;
    float combined_angle = (float)angle_17bit;
    if (rotation_count < 0) {
        combined_angle += 131070.0f * (float)full_turns;
    } else if (rotation_count > 0) {
        combined_angle -= 131070.0f * (float)full_turns;
    }

    int32_t x = (int32_t)(-32767.0f *
        (360.0f * (combined_angle / 131072.0f) /
         (WHEEL_ANGLE_DEGREES * WHEEL_REDUCTION_RATIO / 2.0f)));
    if (x > 32767) return (wheel_input_t){.x = 32767, .rotation_limit = 1};
    if (x < -32767) return (wheel_input_t){.x = -32767, .rotation_limit = -1};
    return (wheel_input_t){.x = (int16_t)x, .rotation_limit = 0};
}

static void cdc_logf(const char *fmt, ...)
{
    if (!tud_cdc_connected()) return;
    char buffer[192];
    va_list args;
    va_start(args, fmt);
    int length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (length <= 0) return;
    uint32_t bytes = (uint32_t)length;
    if (bytes >= sizeof(buffer)) bytes = sizeof(buffer) - 1;
    if (bytes > tud_cdc_write_available()) bytes = tud_cdc_write_available();
    if (bytes) tud_cdc_write(buffer, bytes);
}

void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buffer[64];
    while (tud_cdc_n_available(itf)) {
        (void)tud_cdc_n_read(itf, buffer, sizeof(buffer));
    }
}

int main(void)
{
    stdio_init_all();
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    board_init();
    tusb_init();
    motor_control_init();

    gpio_init(LED_FFB_ACTIVE);
    gpio_init(LED_FFB_MAGNITUDE);
    gpio_set_dir(LED_FFB_ACTIVE, GPIO_OUT);
    gpio_set_dir(LED_FFB_MAGNITUDE, GPIO_OUT);

    absolute_time_t next_log = make_timeout_time_ms(CDC_LOG_INTERVAL_MS);
    while (true)
    {
        tud_task();
        bool ffb_state_sent = ffb_hid_send_state();

        int32_t raw_angle, rotation_count;
        // core0側に角度と回転数をコピー
        motor_read_position(&raw_angle, &rotation_count);
        wheel_input_t wheel = wheel_input_from_encoder(raw_angle, rotation_count);
        // ハンドル位置、ゲーム司令からmagnitudeを生成
        int16_t magnitude = ffb_hid_update(wheel.x);
        bool active = ffb_hid_active();
        motor_set_ffb_command(magnitude, wheel.rotation_limit);

        adc_select_input(2);
        uint16_t adc_value = adc_read();
        uint16_t pedal = adc_value > 3000 ? 32767 : adc_value < 1200 ? 0 :
            (uint16_t)((float)(adc_value - 1200) * 32767.0f / 1800.0f);
        if (!ffb_state_sent) gamepad_hid_send(wheel.x, pedal);
        motor_set_gate_enabled(active);

        gpio_put(LED_FFB_ACTIVE, active);
        gpio_put(LED_FFB_MAGNITUDE, magnitude != 0);

        if (absolute_time_diff_us(get_absolute_time(), next_log) <= 0) {
            uint8_t effect = ffb_hid_active_effect();
            motor_status_t status = motor_get_status();
            cdc_logf("spi_error=%lu encoder_req=%lu rsp=%lu timeout=%lu rsp_us=%lu max_rsp_us=%lu magnitude=%d active=%d effect=%u type=%02X base=%d period=%lu angle=%ld rotNum=%ld I_d=%ld I_q=%ld hid_rx=%lu last=%02X/%u/%u\r\n",
                     status.spi_error_count, status.encoder_request_count,
                     status.encoder_response_count, status.encoder_timeout_count,
                     status.encoder_last_response_us, status.encoder_max_response_us, magnitude, active,
                     effect, ffb_hid_effect_type(effect), ffb_hid_effect_magnitude(effect),
                     ffb_hid_effect_period(effect), raw_angle, rotation_count,
                     status.i_d, status.i_q, ffb_hid_received_report_count(),
                     ffb_hid_last_report_id(), ffb_hid_last_report_type(),
                     ffb_hid_last_report_length());
            next_log = make_timeout_time_ms(CDC_LOG_INTERVAL_MS);
        }
    }
}
