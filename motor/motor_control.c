/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
・TinyUSB使用
・三相ハイサイドシグナル生成（三角波カウンタ、irq割り込みでwrap更新）
・UARTでエンコーダーにコマンド送信、直後にRXに来たデータを処理。
・MCP3204の電流センサをSPIで読み込み、DMAで非同期取得。
*/
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "motor_control.h"
#include "encoder_uart.h"
#include "mcp3204.h"
// 設定：使用GPIOピンと周波数
#define PIN_UH 20
#define PIN_VH 18
#define PIN_WH 16
#define PIN_U_SD PIN_UH + 1
#define PIN_V_SD PIN_VH + 1
#define PIN_W_SD PIN_WH + 1
#define DEBUG_PIN 22     // デバッグ用GPIO
#define DEBUG_PIN_2 23   // デバッグ用GPIO
#define DEBUG_PIN_3 24   // デバッグ用GPIO

// 三相PWM設定
#define CARRIER_FREQ_HZ 20000.0f // 16kHz キャリア
#define PHASE_RSL (1U << 11)
#define Q15_SHIFT 15
#define Q15_ONE (1 << Q15_SHIFT)
#define Q15_SQRT3 56756 // sqrt(3) * 2^15
#define CURRENT_MA_PER_COUNT (3.3f / 4095.f) / (0.044f / 1.0f) * 1000.0f
static volatile uint16_t current_offset_u = 2048;
static volatile uint16_t current_offset_v = 2048;
static volatile uint32_t offset_sum_u = 0;
static volatile uint32_t offset_sum_v = 0;
static volatile uint16_t offset_count = 0;
static volatile bool offset_calibrating = false;

static float current_lpf_ch0 = 2048.0f;
static float current_lpf_ch1 = 2048.0f;
#define CURRENT_LPF_HZ 2000.0f
#define CURRENT_LPF_ALPHA (1.0f - expf(-2.0f * (float)M_PI * CURRENT_LPF_HZ / CARRIER_FREQ_HZ))

static volatile bool spi_error = false;
static volatile uint32_t spi_error_count = 0;
static volatile int32_t soft_limit_iq_ref;
static volatile int32_t motor_position_counts;
/*--------------共有変数------------*/
volatile int32_t angle_share = 0;     // エンコーダーで読み取った角度
volatile int32_t magnitude_share = 0; // FFB指令値
volatile int32_t rotateNum_share = 0;  // モーターの原点からの回転回数
spin_lock_t *lock;
/*----------------------------------*/

int32_t I_u_global = 0;
int32_t I_v_global = 0;
int32_t I_alpha_global = 0;
int32_t I_beta_global = 0;
int32_t I_d_global = 0;
int32_t I_q_global = 0;
float Vd_global = 0.0f, Vq_global = 0.0f;

/*-------------only use in CORE1-------------*/
// PWM wrap 値（TOP）
uint32_t wrap_val;
uint32_t deadtime;
// 各スライス番号・チャネル
uint slice_u, slice_v, slice_w;
uint chan_u, chan_v, chan_w;
float torque_max = 0.2f; // 最大変調率設定
int16_t sinValues[2 * PHASE_RSL];
int16_t cosValues[2 * PHASE_RSL];
volatile int32_t angle = 0; // 17bit treated as 360deg
volatile int32_t rotateNum_core1 = 0;
volatile int32_t elec_angle = 0;
volatile int32_t electrical_offset = 32768 - 8829; // theta_e=0固定のときの電気角を差し引く
volatile int32_t theta_e = 0;
// Set by the PWM wrap ISR and consumed by the core1 foreground loop.
// UART/RS-485 handling must never execute in the FOC ISR.
static volatile bool encoder_request_due;
static bool encoder_request_phase;

float Id_ref = 0.0f;
float Iq_ref = 0.0f;
float error_d = 0.0f;
float error_q = 0.0f;
// Written by core1 foreground code. The PWM ISR uses this Q15 value only.
static volatile int32_t modulation_q15;

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} pi_controller_t;

pi_controller_t pi_d = {
    .kp = 0.0001f,
    .ki = 0.01f,
    .integral = 0.0f,
    .out_min = -1.0f,
    .out_max =  1.0f
};

pi_controller_t pi_q = {
    .kp = 0.0001f,
    .ki = 0.05f,
    .integral = 0.0f,
    .out_min = -1.0f,
    .out_max =  1.0f
};

/*-------------------settings------------------*/
typedef enum {
    CONTROL_MODE_FFB = 0,
    CONTROL_MODE_TORQUE = 1,
    CONTROL_MODE_SPEED = 2,
} control_mode_t;

volatile control_mode_t control_mode = CONTROL_MODE_FFB; // 制御モード選択
/*---------------------------------------------*/

int apply_limit(int value, int max)
{
    if (value < 0)
    {
        value = 0;
    }
    if (value > max)
    {
        value = max;
    }
    return value;
}

static inline int32_t q15_multiply(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> Q15_SHIFT);
}

static inline int32_t q15_from_unit_float(float value)
{
    if (value >= 1.0f) return Q15_ONE - 1;
    if (value <= -1.0f) return -Q15_ONE;
    return (int32_t)(value * (float)Q15_ONE);
}

static inline uint32_t pwm_duty_from_q15(int32_t phase_voltage_q15,
                                          int32_t modulation_q15_value)
{
    int32_t centered_voltage = phase_voltage_q15 + Q15_ONE;
    if (centered_voltage < 0) centered_voltage = 0;
    if (centered_voltage > 2 * Q15_ONE - 1) centered_voltage = 2 * Q15_ONE - 1;
    if (modulation_q15_value < 0) modulation_q15_value = 0;
    if (modulation_q15_value > Q15_ONE) modulation_q15_value = Q15_ONE;

    // Modulation * ((phase_voltage + 1) / 2) * wrap, using only integers.
    uint32_t scaled_voltage = ((uint32_t)modulation_q15_value *
                               (uint32_t)centered_voltage) >> Q15_SHIFT;
    return (scaled_voltage * wrap_val) >> 16;
}

static inline int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static inline int32_t motor_continuous_position(int32_t encoder_angle,
                                                 int32_t rotation_count)
{
    return encoder_angle - MOTOR_ENCODER_COUNTS_PER_REV * rotation_count;
}

static int32_t motor_soft_endstop_iq(int32_t position_counts)
{
    const int32_t ramp_start = MOTOR_SOFT_LIMIT_HALF_COUNTS - MOTOR_SOFT_LIMIT_RAMP_COUNTS;
    if (position_counts >= ramp_start) {
        int32_t penetration = position_counts - ramp_start;
        int32_t force = (int32_t)(((int64_t)penetration * MOTOR_SOFT_LIMIT_MAX_IQ_MA) /
                                  MOTOR_SOFT_LIMIT_RAMP_COUNTS);
        return -clamp_i32(force, 0, MOTOR_SOFT_LIMIT_MAX_IQ_MA);
    }
    if (position_counts <= -ramp_start) {
        int32_t penetration = -ramp_start - position_counts;
        int32_t force = (int32_t)(((int64_t)penetration * MOTOR_SOFT_LIMIT_MAX_IQ_MA) /
                                  MOTOR_SOFT_LIMIT_RAMP_COUNTS);
        return clamp_i32(force, 0, MOTOR_SOFT_LIMIT_MAX_IQ_MA);
    }
    return 0;
}

static int32_t motor_apply_soft_endstop(int32_t ffb_iq, int32_t position_counts)
{
    int32_t endstop_iq = motor_soft_endstop_iq(position_counts);

    // At or beyond the configured range, never allow FFB to push farther out.
    if (position_counts >= MOTOR_SOFT_LIMIT_HALF_COUNTS && ffb_iq > 0) {
        ffb_iq = 0;
    } else if (position_counts <= -MOTOR_SOFT_LIMIT_HALF_COUNTS && ffb_iq < 0) {
        ffb_iq = 0;
    }

    soft_limit_iq_ref = endstop_iq;
    if (endstop_iq == 0) return ffb_iq;
    return clamp_i32(ffb_iq + endstop_iq,
                     -MOTOR_SOFT_LIMIT_MAX_IQ_MA,
                     MOTOR_SOFT_LIMIT_MAX_IQ_MA);
}
static void motor_process_current_sample(uint16_t raw_u, uint16_t raw_v)
{
    if (raw_u < 200 || raw_u > 4000 || raw_v < 200 || raw_v > 4000) {
        spi_error_count++;
        spi_error = true;
    } else {
        spi_error = false;
        current_lpf_ch0 = CURRENT_LPF_ALPHA * raw_u + (1.0f - CURRENT_LPF_ALPHA) * current_lpf_ch0;
        current_lpf_ch1 = CURRENT_LPF_ALPHA * raw_v + (1.0f - CURRENT_LPF_ALPHA) * current_lpf_ch1;
    }
    gpio_put(DEBUG_PIN_3, spi_error);
    if (offset_calibrating) {
        offset_sum_u += (uint16_t)current_lpf_ch0;
        offset_sum_v += (uint16_t)current_lpf_ch1;
        if (++offset_count >= 10000) {
            current_offset_u = offset_sum_u / offset_count;
            current_offset_v = offset_sum_v / offset_count;
            offset_calibrating = false;
        }
    }
    gpio_put(DEBUG_PIN_2, !gpio_get(DEBUG_PIN_2));
}

static inline float pi_update(pi_controller_t *pi, float error, float dt)
{
    float p = pi->kp * error;
    float integral_new = pi->integral + pi->ki * error * dt;

    float output = p + integral_new;

    // 飽和
    if (output > pi->out_max) {
        output = pi->out_max;

        // anti-windup
        if (error < 0.0f)
            pi->integral = integral_new;
    }
    else if (output < pi->out_min) {
        output = pi->out_min;

        // anti-windup
        if (error > 0.0f)
            pi->integral = integral_new;
    }
    else {
        pi->integral = integral_new;
    }
    return output;
}

// wrap IRQ ハンドラ
// PWM割り込みでADC読み込み開始、電流計算、PI制御、デューティ計算を行う
void pwm_wrap_irq_handler()
{
    // IRQ フラグクリア（U相スライス）
    pwm_clear_irq(slice_u);
    gpio_put(DEBUG_PIN, 1);
    // 次割り込み用のADC読み込み開始
    (void)mcp3204_start_read();

    // Schedule the position request every second PWM period. The
    // foreground loop performs the UART/DMA operation after this ISR returns.
    encoder_request_phase = !encoder_request_phase;
    if (encoder_request_phase) {
        encoder_request_due = true;
    }

    int32_t iw_ma = (int32_t)((current_lpf_ch0 - (float)current_offset_u) * CURRENT_MA_PER_COUNT);
    int32_t iv_ma = (int32_t)((current_lpf_ch1 - (float)current_offset_v) * CURRENT_MA_PER_COUNT);
    int32_t iu_ma = -iw_ma - iv_ma;
    int32_t I_alpha = iu_ma;
    int32_t I_beta = ((iu_ma + 2 * iv_ma) * 37837) >> 16; // 1/sqrt(3) -> 0.577350269×65536≈37837
    uint32_t trig_index = (uint32_t)theta_e >> 4;
    int32_t sin_theta_q15 = sinValues[trig_index];
    int32_t cos_theta_q15 = cosValues[trig_index];

    // Previous float dq transform (kept for numerical comparison):
    // int32_t I_d = I_alpha * cosValues[trig_index] + I_beta * sinValues[trig_index];
    // int32_t I_q = -I_alpha * sinValues[trig_index] + I_beta * cosValues[trig_index];
    int32_t I_d = q15_multiply(I_alpha, cos_theta_q15) +
                  q15_multiply(I_beta, sin_theta_q15);
    int32_t I_q = -q15_multiply(I_alpha, sin_theta_q15) +
                  q15_multiply(I_beta, cos_theta_q15);

    error_d = Id_ref - (float)I_d;
    error_q = Iq_ref - (float)I_q;

    float Vd = pi_update(&pi_d, error_d, 1.0f / CARRIER_FREQ_HZ);
    float Vq = pi_update(&pi_q, error_q, 1.0f / CARRIER_FREQ_HZ);

    int32_t Vd_q15 = q15_from_unit_float(Vd);
    int32_t Vq_q15 = q15_from_unit_float(Vq);

    // Previous float inverse-dq and three-phase transform:
    // float V_alpha = Vd * cosValues[trig_index] - Vq * sinValues[trig_index];
    // float V_beta  = Vd * sinValues[trig_index] + Vq * cosValues[trig_index];
    // float Vu = V_alpha;
    // float Vv = -0.5f * V_alpha + 0.8660254f * V_beta;
    // float Vw = -0.5f * V_alpha - 0.8660254f * V_beta;
    int32_t V_alpha_q15 = q15_multiply(Vd_q15, cos_theta_q15) -
                          q15_multiply(Vq_q15, sin_theta_q15);
    int32_t V_beta_q15 = q15_multiply(Vd_q15, sin_theta_q15) +
                         q15_multiply(Vq_q15, cos_theta_q15);
    int32_t Vu_q15 = V_alpha_q15;
    int32_t Vv_q15 = (-V_alpha_q15 + q15_multiply(Q15_SQRT3, V_beta_q15)) >> 1;
    int32_t Vw_q15 = (-V_alpha_q15 - q15_multiply(Q15_SQRT3, V_beta_q15)) >> 1;

    I_u_global = iu_ma;
    I_v_global = iv_ma;
    I_alpha_global = I_alpha;
    I_beta_global = I_beta;
    I_d_global = I_d;
    I_q_global = I_q;
    Vd_global = Vd;
    Vq_global = Vq;
    // PWM ISRでの浮動小数点三相計算は行わない。
    uint32_t du = pwm_duty_from_q15(Vu_q15, modulation_q15);
    uint32_t dv = pwm_duty_from_q15(Vv_q15, modulation_q15);
    uint32_t dw = pwm_duty_from_q15(Vw_q15, modulation_q15);

    // 比較レジスタ更新：次周期から反映
    pwm_set_chan_level(slice_u, chan_u, apply_limit(du - deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_v, chan_v, apply_limit(dv - deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_w, chan_w, apply_limit(dw - deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_u, !chan_u, apply_limit(du + deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_v, !chan_v, apply_limit(dv + deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_w, !chan_w, apply_limit(dw + deadtime / 2, wrap_val));

    gpio_put(DEBUG_PIN, 0);
}

void pwm_init_set()
{
    // PWM GPIO 設定
    gpio_set_function(PIN_UH, GPIO_FUNC_PWM);
    gpio_set_function(PIN_VH, GPIO_FUNC_PWM);
    gpio_set_function(PIN_WH, GPIO_FUNC_PWM);
    // gpio_set_function(PIN_UL, GPIO_FUNC_PWM);
    // gpio_set_function(PIN_VL, GPIO_FUNC_PWM);
    // gpio_set_function(PIN_WL, GPIO_FUNC_PWM);
    slice_u = pwm_gpio_to_slice_num(PIN_UH);
    chan_u = pwm_gpio_to_channel(PIN_UH);
    slice_v = pwm_gpio_to_slice_num(PIN_VH);
    chan_v = pwm_gpio_to_channel(PIN_VH);
    slice_w = pwm_gpio_to_slice_num(PIN_WH);
    chan_w = pwm_gpio_to_channel(PIN_WH);

    // sysclk 周波数取得
    uint sysclk_hz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000;

    // clkdiv 設定（ここでは 1.0f、必要に応じ変更可）
    pwm_set_clkdiv(slice_u, 1.0f);
    pwm_set_clkdiv(slice_v, 1.0f);
    pwm_set_clkdiv(slice_w, 1.0f);

    // wrap (TOP) 計算: sysclk / キャリア周波数x2 - 1 (三角波キャリアにしたため)
    wrap_val = (uint32_t)(sysclk_hz / (CARRIER_FREQ_HZ * 2.0f)) - 1;
    pwm_set_wrap(slice_u, wrap_val);
    pwm_set_wrap(slice_v, wrap_val);
    pwm_set_wrap(slice_w, wrap_val);

    deadtime = (wrap_val / 80) * 2;

    pwm_set_phase_correct(slice_u, true);
    pwm_set_phase_correct(slice_v, true);
    pwm_set_phase_correct(slice_w, true);

    // 初期デューティ: 0（出力OFF相当）
    pwm_set_chan_level(slice_u, chan_u, 0);
    pwm_set_chan_level(slice_v, chan_v, 0);
    pwm_set_chan_level(slice_w, chan_w, 0);
    pwm_set_chan_level(slice_u, !chan_u, 0);
    pwm_set_chan_level(slice_v, !chan_v, 0);
    pwm_set_chan_level(slice_w, !chan_w, 0);

    // pwm_set_output_polarity(slice_u, false, true);
    // pwm_set_output_polarity(slice_v, false, true);
    // pwm_set_output_polarity(slice_w, false, true);

    // wrap IRQ を U相スライスで有効化
    encoder_request_due = false;
    encoder_request_phase = false;
    pwm_clear_irq(slice_u);
    pwm_set_irq_enabled(slice_u, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // PWM 同時有効化
    pwm_set_counter(slice_u, 0);
    pwm_set_counter(slice_v, 0);
    pwm_set_counter(slice_w, 0);
    uint32_t slice_mask = (1 << slice_u) | (1 << slice_v) | (1 << slice_w);
    pwm_set_mask_enabled(slice_mask);
}

static inline void motor_request_encoder_on_pwm_slot(void)
{
    uint32_t irq = save_and_disable_interrupts();
    bool due = encoder_request_due;
    encoder_request_due = false;
    restore_interrupts(irq);

    if (due) {
        (void)encoder_uart_request_position();
    }
}

void ffb_process()
{
    mcp3204_init(motor_process_current_sample);
    mcp3204_start_read();

    sleep_ms(100);
    offset_calibrating = true;

    gpio_init(PIN_U_SD);
    gpio_init(PIN_V_SD);
    gpio_init(PIN_W_SD);
    gpio_init(DEBUG_PIN);
    gpio_init(DEBUG_PIN_2);
    gpio_init(DEBUG_PIN_3);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN_2, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN_3, GPIO_OUT);
    gpio_set_dir(PIN_U_SD, GPIO_OUT);
    gpio_set_dir(PIN_V_SD, GPIO_OUT);
    gpio_set_dir(PIN_W_SD, GPIO_OUT);
    // ゲートドライバ有効化
    gpio_put(PIN_U_SD, 0);
    gpio_put(PIN_V_SD, 0);
    gpio_put(PIN_W_SD, 0);

    // モーター制御用数値設定
    angle = encoder_uart_initial_position();
    int32_t previous_angle = angle;
    int32_t local_magnitude = 0; // max 10000
    int delta_angle = 0;

    pwm_init_set();

    while (true)
    {
        previous_angle = angle;

        int32_t encoder_angle;
        if (encoder_uart_read_position(&encoder_angle)) {
            angle = encoder_angle;
        }
        
        delta_angle = angle - previous_angle;
        if (delta_angle > 65535)
        {
            rotateNum_core1++;
            delta_angle = 0;
        }
        else if (delta_angle < -65535)
        {
            rotateNum_core1--;
            delta_angle = 0;
        }
        // 排他制御して読み込み
        uint32_t irq = save_and_disable_interrupts();
        spin_lock_unsafe_blocking(lock);
        local_magnitude = magnitude_share; // from core0
        angle_share = angle;               // to core0
        rotateNum_share = rotateNum_core1; // to core0
        spin_unlock_unsafe(lock);
        restore_interrupts(irq);

        // 電気角算出
        if (angle >= 0 && angle < 32768)
        {
            elec_angle = angle;
        }
        else if (angle >= 32768 && angle < 65536)
        {
            elec_angle = angle - 32768;
        }
        else if (angle >= -32768 && angle < 0)
        {
            elec_angle = 32768 + angle;
        }
        else if (angle >= -65536 && angle < -32768)
        {
            elec_angle = 65536 + angle;
        }
        motor_position_counts = motor_continuous_position(angle, rotateNum_core1);
        int32_t ffb_iq = local_magnitude / 2;
        Iq_ref = (float)motor_apply_soft_endstop(ffb_iq, motor_position_counts);
        modulation_q15 = q15_from_unit_float(torque_max);
        Id_ref = 0.0f;

        // 電気角オフセット
        theta_e = (elec_angle + electrical_offset) & 0x7FFF;

        motor_request_encoder_on_pwm_slot();
    }
}

void torque_mode_process()
{
    mcp3204_init(motor_process_current_sample);
    mcp3204_start_read();

    sleep_ms(100);
    offset_calibrating = true;

    gpio_init(PIN_U_SD);
    gpio_init(PIN_V_SD);
    gpio_init(PIN_W_SD);
    gpio_init(DEBUG_PIN);
    gpio_init(DEBUG_PIN_2);
    gpio_init(DEBUG_PIN_3);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN_2, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN_3, GPIO_OUT);
    gpio_set_dir(PIN_U_SD, GPIO_OUT);
    gpio_set_dir(PIN_V_SD, GPIO_OUT);
    gpio_set_dir(PIN_W_SD, GPIO_OUT);
    // ゲートドライバ有効化
    gpio_put(PIN_U_SD, 0);
    gpio_put(PIN_V_SD, 0);
    gpio_put(PIN_W_SD, 0);

    // モーター制御用数値設定
    angle = encoder_uart_initial_position();
    float torque = 0.0f;
    int32_t previous_angle = angle;
    int32_t local_magnitude = 10000;
    int delta_angle = 0;

    pwm_init_set();

    while (true)
    {
        previous_angle = angle;

        int32_t encoder_angle;
        if (encoder_uart_read_position(&encoder_angle)) {
            angle = encoder_angle;
        }
        
        delta_angle = angle - previous_angle;
        if (delta_angle > 65535)
        {
            rotateNum_core1++;
            delta_angle = 0;
        }
        else if (delta_angle < -65535)
        {
            rotateNum_core1--;
            delta_angle = 0;
        }
        // 排他制御して読み込み
        uint32_t irq = save_and_disable_interrupts();
        spin_lock_unsafe_blocking(lock);
        angle_share = angle;               // to core0
        rotateNum_share = rotateNum_core1; // to core0
        spin_unlock_unsafe(lock);
        restore_interrupts(irq);

        // 電気角算出
        if (angle >= 0 && angle < 32768)
        {
            elec_angle = angle;
        }
        else if (angle >= 32768 && angle < 65536)
        {
            elec_angle = angle - 32768;
        }
        else if (angle >= -32768 && angle < 0)
        {
            elec_angle = 32768 + angle;
        }
        else if (angle >= -65536 && angle < -32768)
        {
            elec_angle = 65536 + angle;
        }
        torque = torque_max * (float)local_magnitude / 10000.f;
        modulation_q15 = q15_from_unit_float(fabs(torque));

        // 電気角オフセット
        theta_e = (elec_angle + electrical_offset) & 0x7FFF;

        motor_request_encoder_on_pwm_slot();
    }
}

void speed_mode_process()
{
}

// core_1 モーター制御関係
void core1_main()
{
    switch (control_mode){
    case CONTROL_MODE_FFB:
        ffb_process();
        break;

    case CONTROL_MODE_TORQUE:
        // トルク制御用
        torque_mode_process();
        break;

    case CONTROL_MODE_SPEED:
        // 回転数制御用
        speed_mode_process();
        break;
    }
}



void motor_control_init(void)
{
    for (int x = 0; x < 2 * PHASE_RSL; ++x) {
        float s = 2.0f * (float)M_PI * (float)x / (float)PHASE_RSL;
        sinValues[x] = (int16_t)(sinf(s) * (float)(Q15_ONE - 1));
        cosValues[x] = (int16_t)(cosf(s) * (float)(Q15_ONE - 1));
    }
    encoder_uart_init();
    lock = spin_lock_instance(0);
    multicore_reset_core1();
    sleep_ms(100);
    multicore_launch_core1(core1_main);
}

void motor_read_position(int32_t *out_angle, int32_t *out_rotation_count)
{
    uint32_t irq = save_and_disable_interrupts();
    spin_lock_unsafe_blocking(lock);
    *out_angle = angle_share;
    *out_rotation_count = rotateNum_share;
    spin_unlock_unsafe(lock);
    restore_interrupts(irq);
}

void motor_set_ffb_command(int16_t magnitude)
{
#if WHEEL_DIRECTION_REVERSED
    // The HID axis is reversed in main.c. Reverse the resulting force at the
    // motor boundary as well so position-dependent effects remain restorative.
    magnitude = -magnitude;
#endif
    uint32_t irq = save_and_disable_interrupts();
    spin_lock_unsafe_blocking(lock);
    magnitude_share = magnitude;
    spin_unlock_unsafe(lock);
    restore_interrupts(irq);
}

void motor_set_gate_enabled(bool enabled)
{
    gpio_put(PIN_U_SD, enabled ? 0 : 1);
    gpio_put(PIN_V_SD, enabled ? 0 : 1);
    gpio_put(PIN_W_SD, enabled ? 0 : 1);
}

motor_status_t motor_get_status(void)
{
    encoder_uart_stats_t encoder_stats = encoder_uart_get_stats();
    return (motor_status_t){
        .vd = Vd_global, .vq = Vq_global, .iq_ref = Iq_ref,
        .i_d = I_d_global, .i_q = I_q_global,
        .soft_limit_iq = soft_limit_iq_ref,
        .motor_position_counts = motor_position_counts,
        .spi_error_count = spi_error_count,
        .encoder_request_count = encoder_stats.request_count,
        .encoder_response_count = encoder_stats.response_count,
        .encoder_timeout_count = encoder_stats.timeout_count,
        .encoder_last_response_us = encoder_stats.last_response_us,
        .encoder_max_response_us = encoder_stats.max_response_us,
        .spi_error = spi_error,
    };
}
