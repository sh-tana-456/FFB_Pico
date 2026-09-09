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
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "bsp/board.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
// 設定：使用GPIOピンと周波数
#define PIN_UH 20
#define PIN_VH 16
#define PIN_WH 18
#define PIN_U_SD PIN_UH + 1
#define PIN_V_SD PIN_VH + 1
#define PIN_W_SD PIN_WH + 1
#define DE_PIN 10            // MAX485 DE
#define RE_PIN 6             // MAX485 RE
#define UART_TX_PIN_uart1 8  // TXピン
#define UART_RX_PIN_uart1 9  // RXピン
#define MCP3204_PIN_DOUT  12
#define MCP3204_PIN_CS    13
#define MCP3204_PIN_CLK   14
#define MCP3204_PIN_DIN   15

#define LED_FFB_ACTIVE 4     // FFBアクティブLED
#define LED_FFB_MAGNITUDE 24 // FFBマグニチュードLED
// 三相PWM設定
#define CARRIER_FREQ_HZ 20000.0f // 20kHz キャリア
#define SINE_FREQ_HZ 0.0f        // 変調周波数
#define PHASE_MAX (1U << 31)
#define PHASE_RSL (1U << 11)
// UART設定
#define UART_ID_1 uart1
#define UART_BAUD_1 2500000
// SPI設定
#define MCP3204_SPI       spi1
#define CURRENT_MA_PER_COUNT (3.3f / 4095.f) / (0.044f / 1.0f) * 1000.0f

#define MCP3204_SPI_HZ    1200000
#define MCP3204_NUM_CHANNELS     2
#define MCP3204_BYTES_PER_SAMPLE 3
static int mcp3204_dma_tx_channel = -1;
static int mcp3204_dma_rx_channel = -1;
static uint8_t mcp3204_dma_tx_buffer[MCP3204_BYTES_PER_SAMPLE];
static uint8_t mcp3204_dma_rx_buffer[MCP3204_BYTES_PER_SAMPLE];
static volatile bool mcp3204_dma_busy = false;
static volatile bool mcp3204_sample_ready = false;
static volatile uint16_t mcp3204_latest_raw[2] = {0, 0};
static volatile uint8_t mcp3204_active_channel = 0;
static volatile uint32_t mcp3204_dma_overrun_count = 0;

#define WHEEL_ANGLE 540 // degree
#define REDUCTION_RATIO 4

/*--------------共有変数------------*/
volatile int32_t angle_share = 0;     // エンコーダーで読み取った角度
volatile int32_t magnitude_share = 0; // FFB指令値
volatile int8_t rotateNum_share = 0;  // モーターの原点からの回転回数
volatile int8_t limitRot_share = 0;   // 回転制限 0 or 1
spin_lock_t *lock;
/*----------------------------------*/

volatile int16_t ffb_magnitude = 0;
volatile bool ffb_active = false;
volatile uint8_t current_block_idx = 4;
volatile bool effect_playing = false;
volatile int32_t angle_core0 = 0;
volatile int8_t rotateNum_core0 = 0;
volatile int8_t limitRot_core0 = 0;

uint16_t adc0 = 0;
uint16_t apps_y = 0;
int16_t I_d_global = 0;
int16_t I_q_global = 0;

/*-------------only use in CORE1-------------*/
// 位相変数（rad単位）、3相分はオフセットで扱う
volatile uint32_t phase = 0;
const uint32_t phase_offset = 57445188; // 9.63deg angleオフセット2090041344
uint32_t delta_phase;                   // 1キャリア周期ごとの位相ステップ
// PWM wrap 値（TOP）
uint32_t wrap_val;
uint32_t deadtime;
// 各スライス番号・チャネル
uint slice_u, slice_v, slice_w;
uint chan_u, chan_v, chan_w;
float MR = 0.0f; // 変調率
float torque_max = 0.10f;
float sinValues[2 * PHASE_RSL];
float cosValues[2 * PHASE_RSL];
float rdmValues[2 * PHASE_RSL];
int rotate_mode = 1;
uint8_t enc_rx_buffer[12];
uint8_t error = 0;
volatile int32_t angle = 0, pre_angle = 0; // 17bit treated as 360deg
volatile uint8_t direction_cmd = 0;        // 0:stop, 1:+, 2:-

volatile int8_t rotateNum_core1 = 0;
volatile int8_t limitRot_core1 = 0;
const uint32_t angle_offset = 7420; // 生のエンコーダーの0点と物理ハンドルの0点を合わせるためのオフセット値
volatile int32_t elec_angle = 0;

static volatile int32_t current_offset_raw = 2048;
/*---------------------------------------------*/

/*---------------------------------------------*/

static void mcp3204_start_channel(uint8_t channel);
static void mcp3204_prepare_command(uint8_t channel);

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
// DMA RX完了割り込みハンドラ
static void mcp3204_dma_irq_handler(void)
{
    dma_hw->ints0 = 1u << mcp3204_dma_rx_channel;

    gpio_put(MCP3204_PIN_CS, 1);

    uint16_t raw =
        ((mcp3204_dma_rx_buffer[1] & 0x0F) << 8) |
        mcp3204_dma_rx_buffer[2];

    mcp3204_latest_raw[mcp3204_active_channel] = raw;

    if (mcp3204_active_channel < MCP3204_NUM_CHANNELS - 1)
    {
        mcp3204_start_channel(mcp3204_active_channel + 1);
    }
    else
    {
        mcp3204_dma_busy = false;
        mcp3204_sample_ready = true;
    }
}

static void mcp3204_dma_init(void)
{
    spi_init(MCP3204_SPI, MCP3204_SPI_HZ);
    spi_set_format(
        MCP3204_SPI,
        8,
        SPI_CPOL_0,
        SPI_CPHA_0,
        SPI_MSB_FIRST
    );
    gpio_set_function(MCP3204_PIN_DOUT, GPIO_FUNC_SPI);
    gpio_set_function(MCP3204_PIN_CLK, GPIO_FUNC_SPI);
    gpio_set_function(MCP3204_PIN_DIN, GPIO_FUNC_SPI);
    gpio_init(MCP3204_PIN_CS);
    gpio_set_dir(MCP3204_PIN_CS, GPIO_OUT);
    gpio_put(MCP3204_PIN_CS, 1);

    mcp3204_dma_tx_channel = dma_claim_unused_channel(true);
    mcp3204_dma_rx_channel = dma_claim_unused_channel(true);
    // TX DMA:
    // メモリからSPI TX FIFOへ送る。
    dma_channel_config tx_config = dma_channel_get_default_config(mcp3204_dma_tx_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, spi_get_dreq(MCP3204_SPI, true));
    dma_channel_configure(
        mcp3204_dma_tx_channel,
        &tx_config,
        &spi_get_hw(MCP3204_SPI)->dr,
        mcp3204_dma_tx_buffer,
        3,
        false);
    // RX DMA:
    // SPI RX FIFOからメモリへ格納する。
    dma_channel_config rx_config = dma_channel_get_default_config(mcp3204_dma_rx_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, spi_get_dreq(MCP3204_SPI, false));
    dma_channel_configure(
        mcp3204_dma_rx_channel,
        &rx_config,
        mcp3204_dma_rx_buffer,
        &spi_get_hw(MCP3204_SPI)->dr,
        3,
        false);
    // RX完了でDMA IRQ 0を発生させる。
    dma_channel_set_irq0_enabled(mcp3204_dma_rx_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, mcp3204_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    mcp3204_dma_busy = false;
    mcp3204_sample_ready = false;
}

void mcp3204_start_read_dma(void)
{
    if (mcp3204_dma_busy)
        return;

    mcp3204_dma_busy = true;
    mcp3204_sample_ready = false;

    mcp3204_start_channel(0);
}

static void mcp3204_start_channel(uint8_t channel)
{
    mcp3204_active_channel = channel;

    mcp3204_prepare_command(channel);

    gpio_put(MCP3204_PIN_CS, 0);

    dma_channel_set_read_addr(mcp3204_dma_rx_channel, &spi_get_hw(MCP3204_SPI)->dr, false);
    dma_channel_set_write_addr(mcp3204_dma_rx_channel, mcp3204_dma_rx_buffer, false);
    dma_channel_set_trans_count(mcp3204_dma_rx_channel, MCP3204_BYTES_PER_SAMPLE, false);
    dma_channel_set_read_addr(mcp3204_dma_tx_channel, mcp3204_dma_tx_buffer, false);
    dma_channel_set_trans_count(mcp3204_dma_tx_channel, MCP3204_BYTES_PER_SAMPLE, false);
    dma_start_channel_mask((1u << mcp3204_dma_rx_channel) | (1u << mcp3204_dma_tx_channel));
}

static void mcp3204_prepare_command(uint8_t channel)
{
    mcp3204_dma_tx_buffer[0] = 0x06;
    mcp3204_dma_tx_buffer[1] = channel << 6;
    mcp3204_dma_tx_buffer[2] = 0x00;
}

// wrap IRQ ハンドラ（例としてU相のラップIRQを使い、3相同時更新）
void pwm_wrap_irq_handler()
{
    // IRQ フラグクリア（U相スライス）
    pwm_clear_irq(slice_u);
    
    int16_t iu_ma = (int16_t)((float)(mcp3204_latest_raw[0] - current_offset_raw) * CURRENT_MA_PER_COUNT);
    int16_t iv_ma = (int16_t)((float)(mcp3204_latest_raw[1] - current_offset_raw) * CURRENT_MA_PER_COUNT);
    int16_t I_alpha = iu_ma;
    int16_t I_beta = ((iu_ma - iv_ma) * 37837) >> 16; // 1/sqrt(3) -> 0.577350269×65536≈37837
    int16_t I_d = I_alpha * cosValues[elec_angle >> 4] + I_beta * sinValues[elec_angle >> 4];
    int16_t I_q = -I_alpha * sinValues[elec_angle >> 4] + I_beta * cosValues[elec_angle >> 4];
    I_d_global = I_d;
    I_q_global = I_q;
    // 位相更新
    phase = phase & (PHASE_MAX - 1);

    // 3相の振幅計算（浮動小数点 sinf)
    float su = 0, sv = 0, sw = 0;
    if (rotate_mode == 1)
    {
        // U相: θ
        su = sinValues[phase >> 20];
        // V相: θ + 120° = θ + 4π/3
        sv = sinValues[(phase >> 20) + 683 /*→(2^11 * 1/3)*/];
        // W相: θ + 240° = θ + 2π/3
        sw = sinValues[(phase >> 20) + 1365 /*→(2^11 * 2/3)*/];
    }
    else if (rotate_mode == -1)
    {
        // U相: θ
        su = sinValues[phase >> 20];
        // V相: θ + 240° = θ + 2π/3
        sv = sinValues[(phase >> 20) + 1365 /*→(2^11 * 2/3)*/];
        // W相: θ + 120° = θ + 4π/3
        sw = sinValues[(phase >> 20) + 683 /*→(2^11 * 1/3)*/];
    }
    // デューティ（0 ～ wrap_val）の計算: (sin*0.5 + 0.5) を乗算
    uint32_t du = (uint32_t)(MR * (su * 0.5f + 0.5f) * (float)wrap_val); // + rdmValues[phase >> 20]
    uint32_t dv = (uint32_t)(MR * (sv * 0.5f + 0.5f) * (float)wrap_val); // + rdmValues[(phase >> 20) + 683]
    uint32_t dw = (uint32_t)(MR * (sw * 0.5f + 0.5f) * (float)wrap_val); // + rdmValues[(phase >> 20) + 1365]

    // 比較レジスタ更新：次周期から反映
    pwm_set_chan_level(slice_u, chan_u, apply_limit(du - deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_v, chan_v, apply_limit(dv - deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_w, chan_w, apply_limit(dw - deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_u, !chan_u, apply_limit(du + deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_v, !chan_v, apply_limit(dv + deadtime / 2, wrap_val));
    pwm_set_chan_level(slice_w, !chan_w, apply_limit(dw + deadtime / 2, wrap_val));
    // 次割り込み用のADC読み込み開始
    (void)mcp3204_start_read_dma();
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

int convert_angle(int8_t rotateNum, int angle_17bit)
{ // {(17bit int) and (multi rotate data)} into 16bit int
    int angle_conv;
    if (rotateNum == 0)
    {
        angle_conv = -32767 * (float)(360.f * ((float)angle_17bit / 131072.f) / (float)(WHEEL_ANGLE * REDUCTION_RATIO / 2.0f));
    }
    else if (rotateNum <= -1)
    {
        angle_conv = -32767 * (float)(360.f * ((float)(angle_17bit + (65535 + 65535) * abs(rotateNum)) / 131072.f) / (float)(WHEEL_ANGLE * REDUCTION_RATIO / 2.0f));
    }
    else if (rotateNum >= 1)
    {
        angle_conv = -32767 * (float)(360.f * ((float)(angle_17bit - (65535 + 65535) * abs(rotateNum)) / 131072.f) / (float)(WHEEL_ANGLE * REDUCTION_RATIO / 2.0f));
    }

    if (angle_conv > 32767)
    {
        angle_conv = 32767, limitRot_core0 = 1;
    }
    else if (angle_conv < -32767)
    {
        angle_conv = -32767, limitRot_core0 = -1;
    }
    else
    {
        limitRot_core0 = 0;
    }
    return angle_conv;
}

typedef struct
{
    int16_t x;
    int16_t y;
} __attribute__((packed)) hid_report_t;

void send_hid_report()
{
    static hid_report_t report = {0};
    report.x = convert_angle(rotateNum_core0, angle_core0);
    report.y = apps_y;

    if (tud_hid_ready())
    {
        tud_hid_report(/*report_id*/ 0x01, &report, sizeof(report));
    }
}

// UART function
void uart1_send(uint8_t cmd)
{
    // UART送信、DE・REピン切り替えは割り込み禁止状態で行う
    uint32_t irq = save_and_disable_interrupts();
    // 送信モードに切り替え
    gpio_put(RE_PIN, 1); // 受信無効
    gpio_put(DE_PIN, 1); // 送信有効
    //  送信前にRX FIFOをクリアしておく（古いデータ・ノイズ排除）
    while (uart_is_readable(UART_ID_1))
    {
        (void)uart_getc(UART_ID_1);
    }
    // UART送信
    // uart_write_blocking は内部でFIFOに書き込み、そのまま戻る
    uart_write_blocking(UART_ID_1, &cmd, 1);

    // 送信完了待ち: 1バイト/2.5Mbps ≈ 3.2μs。確実に送出完了させるため、数十μs待機
    // UARTハードウェアレジスタで直にBUSYを確認
    while (uart_get_hw(UART_ID_1)->fr & UART_UARTFR_BUSY_BITS) {
        tight_loop_contents();
    }
    // 受信モードに戻す
    gpio_put(RE_PIN, 0);
    gpio_put(DE_PIN, 0);
    restore_interrupts(irq);
}
// エンコーダー受信用
void uart1_receive()
{
    uint8_t const expected_bytes = 12;
    int idx = 0;
    uint64_t start = get_absolute_time();
    uint8_t pre_buffer[12] = {0};
    for (int i = 0; i < expected_bytes; i++)
    {
        pre_buffer[i] = enc_rx_buffer[i];
    }

    while (absolute_time_diff_us(start, get_absolute_time()) < 200)
    { // 受信タイムアウト
        while (uart_is_readable(UART_ID_1) && idx < expected_bytes)
        {
            enc_rx_buffer[idx++] = uart_getc(UART_ID_1);
        }
        if (idx >= expected_bytes)
            break;
    }
    if (idx < expected_bytes)
    {
        // printf("%d\n",idx);
        for (int i = 0; i < expected_bytes; i++)
        {
            enc_rx_buffer[i] = pre_buffer[i];
        }
    }
}
// エンコーダーデコード : 17bitデータの復号 (-65535~65535) 0点を物理ハンドルと合わせるためオフセット有り
int decode_encoder_angle(uint8_t abs0, uint8_t abs1, uint8_t abs2)
{ // 17bitデータの復号 -65535~65535
    int32_t position = ((abs2 & 0x7F) << 16) + (abs1 << 8) + abs0 - angle_offset;
    if (position > 131072 / 2)
    {
        position -= 131072;
    }
    if (position < -131072 / 2)
    {
        position += 131072;
    }
    return position;
}

void uart_init_set()
{
    // UART1初期化
    uart_init(UART_ID_1, UART_BAUD_1);
    gpio_set_function(UART_TX_PIN_uart1, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_uart1, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_ID_1, true);
    // DE/REピン設定
    gpio_init(DE_PIN);
    gpio_set_dir(DE_PIN, GPIO_OUT);
    gpio_put(DE_PIN, 0);
    gpio_init(RE_PIN);
    gpio_set_dir(RE_PIN, GPIO_OUT);
    gpio_put(RE_PIN, 0);
    // リセットコマンド
    uart1_send(0xBA);
    sleep_ms(1);
    uart1_send(0xC2);
    sleep_ms(1);
    uart1_send(0x62);
    sleep_ms(1);
    uart1_send(0xEA);
    sleep_ms(1);
}
// 電流センサ出力を読む
static uint16_t mcp3204_read_raw(uint8_t channel)
{
    if (channel > 3) {
        return 0;
    }

    uint8_t tx[3] = {
        0x06,
        (uint8_t)(channel << 6),
        0x00
    };

    uint8_t rx[3] = {0};

    gpio_put(MCP3204_PIN_CS, 0);

    spi_write_read_blocking(
        MCP3204_SPI,
        tx,
        rx,
        sizeof(tx)
    );

    gpio_put(MCP3204_PIN_CS, 1);

    return (uint16_t)(((uint16_t)(rx[1] & 0x0f) << 8) | rx[2]);
}

// TinyUSB CDC logging. Do not enable pico_stdio_usb concurrently.
static void cdc_logf(const char *fmt, ...)
{
    if (!tud_cdc_connected()) return;
    char buffer[192];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (len <= 0) return;
    uint32_t bytes = (uint32_t)len;
    if (bytes >= sizeof(buffer)) bytes = sizeof(buffer) - 1;
    uint32_t available = tud_cdc_write_available();
    if (bytes > available) bytes = available;
    if (bytes) {
        tud_cdc_write(buffer, bytes);
        // tud_cdc_write_flush();
    }
}

void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buffer[64];
    while (tud_cdc_n_available(itf)) {
        (void)tud_cdc_n_read(itf, buffer, sizeof(buffer));
    }
}

// core_1 モーター制御関係
void core1_main()
{
    mcp3204_dma_init();
    mcp3204_start_read_dma();

    gpio_init(PIN_U_SD);
    gpio_init(PIN_V_SD);
    gpio_init(PIN_W_SD);
    gpio_set_dir(PIN_U_SD, GPIO_OUT);
    gpio_set_dir(PIN_V_SD, GPIO_OUT);
    gpio_set_dir(PIN_W_SD, GPIO_OUT);
    gpio_put(PIN_U_SD, 1);
    gpio_put(PIN_V_SD, 1);
    gpio_put(PIN_W_SD, 1);

    // モーター制御用数値設定
    phase = phase_offset;
    angle = angle_offset;
    int pre_elec = 0;
    float torque = 0.0f;
    int32_t local_magnitude = 0, pre_local_magnitude = 0;
    float Kd = 0.0f, alpha = 0.001f, beta = 0.25f;
    int delta_angle = 0, pre_delta_angle = 0, a = 0;

    pwm_init_set();

    while (true)
    {
        uart1_send(0x1A); // エンコーダーコマンド送信
        sleep_us(100);
        uart1_receive(); // エンコーダーデータ受信

        pre_angle = angle;
        pre_delta_angle = delta_angle;
        pre_local_magnitude = local_magnitude;

        angle = decode_encoder_angle(enc_rx_buffer[3], enc_rx_buffer[4], enc_rx_buffer[5]); // -65535~65535
        delta_angle = angle - pre_angle;
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
        limitRot_core1 = limitRot_share;   // from core0
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
        else
        {
            error = 1;
        }

        pre_elec = elec_angle;

        // 回転限界処理、粘性抵抗
        delta_angle = pre_delta_angle * (1.0f - alpha) + delta_angle * alpha; // ωフィルタ
        local_magnitude = pre_local_magnitude * (1.0f - beta) + local_magnitude * beta;
        torque = -((float)delta_angle / 65536.f * Kd) + (torque_max * (float)local_magnitude / 10000.f);
        // if(rotateNum_core1!=0){MR = 0.07f;}else{MR = fabs(torque);} // (0.10f * fabs(angle) / 65536.f) +
        MR = fabs(torque);
        if (limitRot_core1 > 0)
        {
            a = 8192;
        }
        else if (limitRot_core1 < 0)
        {
            a = -8192;
        }
        else
        {
            a = 0;
            if (torque > 0)
            {
                a = -8192;
            }
            else if (torque < 0)
            {
                a = 8192;
            }
        }

        elec_angle = (elec_angle + a); // 8192 = 90deg

        // 位相計算
        if (elec_angle < 0)
        {
            elec_angle = 32768 + elec_angle;
        }
        elec_angle = elec_angle & ((1 << 15) - 1);
        phase = phase_offset + (float)(elec_angle) / (float)(1 << 15) * (float)PHASE_MAX;
    }
}
// core_0 USBデータ送信・メインループ
int main()
{
    stdio_init_all();
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    // 三相基本波配列計算
    for (int x = 0; x < 2 * PHASE_RSL; x++)
    {
        float s = (float)(2.0f * M_PI * (float)x) / (float)PHASE_RSL;
        sinValues[x] = sin(s);
        cosValues[x] = cos(s);
        // printf(x + "\n");
    }
    for (int x = 0; x < 2 * PHASE_RSL; x++)
    {
        rdmValues[x] = (float)(rand() % 100 - 50) / 1000.0f;
    }

    board_init();
    tusb_init();
    uart_init_set();

    lock = spin_lock_instance(0);
    multicore_reset_core1();
    sleep_ms(100);
    multicore_launch_core1(core1_main);

    gpio_init(LED_FFB_ACTIVE);
    gpio_init(LED_FFB_MAGNITUDE);
    gpio_set_dir(LED_FFB_ACTIVE, GPIO_OUT);
    gpio_set_dir(LED_FFB_MAGNITUDE, GPIO_OUT);

    absolute_time_t next_cdc_log = make_timeout_time_ms(500);
    while (true)
    {
        tud_task(); // HID + CDC USB task processing
        if (absolute_time_diff_us(get_absolute_time(), next_cdc_log) <= 0) {
            cdc_logf("MCP3204 CH0=%4u CH1=%4u magnitude=%d angle=%d limRot=%d I_d=%d I_q=%d\r\n",
                     mcp3204_latest_raw[0], mcp3204_latest_raw[1],
                     ffb_magnitude, angle_core0, limitRot_core0, I_d_global, I_q_global);
            next_cdc_log = make_timeout_time_ms(500);
        }

        // 排他制御して読み込み
        uint32_t irq = save_and_disable_interrupts();
        spin_lock_unsafe_blocking(lock);
        magnitude_share = -ffb_magnitude; // to core1
        limitRot_share = limitRot_core0;
        angle_core0 = angle_share; // from core1
        rotateNum_core0 = rotateNum_share;
        spin_unlock_unsafe(lock);
        restore_interrupts(irq);

        adc_select_input(1);
        adc0 = adc_read();
        if (adc0 > 3000)
        {
            apps_y = 32767;
        }
        else if (adc0 < 1200)
        {
            apps_y = 0;
        }
        else
        {
            apps_y = (int16_t)((float)(adc0 - 1200) / (3000 - 1200) * 32767);
        }

        send_hid_report(); // ゲームパッド状態送信
        // sleep_ms(5);       // 約200Hzで送信
        gpio_put(LED_FFB_ACTIVE, ffb_active);
        gpio_put(LED_FFB_MAGNITUDE, ffb_magnitude != 0);
        if (!ffb_active){
            // 出力停止（magnitudeを保持してしまうため）
            ffb_magnitude = 0;
            gpio_put(PIN_U_SD, 1);
            gpio_put(PIN_V_SD, 1);
            gpio_put(PIN_W_SD, 1);
        }else{
            // ゲートドライバ有効化
            gpio_put(PIN_U_SD, 0);
            gpio_put(PIN_V_SD, 0);
            gpio_put(PIN_W_SD, 0);
        }
    }
}

typedef struct __attribute__((packed))
{
    uint8_t id; /* 0x01 */
    uint8_t block_idx;
    uint8_t effect_type;
    uint16_t duration_ms;
    uint16_t trig_rep_int_ms;
    uint16_t sample_prd_ms;
    uint8_t gain;
    uint8_t trig_button;
    uint8_t axes_enable; /* bit0=X, bit1=Y */
    uint8_t dir_enable;  /* bit0=1 */
    uint8_t direction_x; /* 0‑255 (角度) */
    uint8_t direction_y;
    uint16_t ts_offset1;
    uint16_t ts_offset2;
} __attribute__((packed)) set_effect_rpt_t;
/* -------- PID State / Block‑Load (ID 0x02) -------- */
typedef struct __attribute__((packed))
{
    // uint8_t  id;              // = 0x02
    uint8_t effectBlockIndex; /* = 1 だけ使う */
    uint8_t loadStatus;       /* 0 = OK, 1 = FULL */
    uint8_t poolAvailableLo;  /* 0xFF → 32 kB 空き */
    uint8_t poolAvailableHi;
    uint8_t actuatorsEnabled; /* bit0 */
    uint8_t safetySwitch;     /* 0 固定 */
    uint8_t effectPlaying;    /* bit0 */
} __attribute__((packed)) pid_state_rpt_t;

/* -------- Block‑Load Status (ID 0x06) --------
 *  byte0: 0x06 (Report ID)
 *  byte1: effectBlockIndex (1 を固定で OK)
 *  byte2: blockLoadStatus  (1 = Success, 2 = Full, 3 = Error)
 *  byte3: ramPoolAvailableLo   (0xFF = 32 kB 空き)
 *  byte4: ramPoolAvailableHi   (0x7F)
 */
typedef struct __attribute__((packed))
{
    // uint8_t  id;          // = 0x05
    uint16_t effect_idx; // block index the FW just allocated
    uint8_t rom_flag;    // 0 = RAM
    uint8_t status;      // 0 = success
} pid_var_blockload_rpt_t;
typedef struct __attribute__((packed))
{
    // uint8_t id;              /* = 0x06 */
    uint8_t effectBlockIndex; /* 1 */
    uint8_t loadStatus;       /* 1 */
    uint8_t poolAvailLo;      /* 0xFF */
    uint8_t poolAvailHi;      /* 0x7F */
} pid_blockload_status_t;
typedef struct __attribute__((packed))
{
    // uint8_t  id;             // = 0x07
    uint16_t ram_pool_size; // 0x8000 → little‑endian: 00 80
    uint8_t effect_max;     // 8
    uint8_t caps;           // bit0=1, bit1=1 → 0x03
} pid_pool_rpt_t;           // sizeof = 5

// デバイスに関する情報
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t rid, hid_report_type_t type, uint8_t *buffer, uint16_t reqlen)
{
    if (type != HID_REPORT_TYPE_FEATURE)
        return 0;

    // if (rid == 0x02) { // && reqlen >= sizeof(pid_state_rpt_t)
    //     pid_state_rpt_t* r = (pid_state_rpt_t*)buffer;
    //     //r->id               = 0x02;
    //     r->effectBlockIndex = current_block_idx;           // Block ID
    //     r->loadStatus       = 0;           // 0 = Success
    //     r->poolAvailableLo  = 0xFF;        // 32 kB 空き
    //     r->poolAvailableHi  = 0x7F;
    //     r->actuatorsEnabled = effect_playing;
    //     r->safetySwitch     = 0;
    //     r->effectPlaying    = ffb_active;
    //     return 8;
    // }

    if (rid == 0x06 && reqlen >= 4)
    {
        pid_blockload_status_t rpt = {
            //.id               = 0x06,
            .effectBlockIndex = 1, // 前回の idx:current_block_idx
            .loadStatus = 1,       // (1=SUCCESS,2=FULL,3=ERROR…)
            .poolAvailLo = 0xFF,
            .poolAvailHi = 0x7F};
        memcpy(buffer, &rpt, sizeof(rpt));
        return sizeof(rpt);
    }

    if (rid == 0x07)
    {
        pid_pool_rpt_t rpt = {
            //.id            = 0x07,
            .ram_pool_size = 0x8000, /* 32 kB  */
            .effect_max = 8,         /* 同時 8 効果 */
            .caps = 0x03             /* bit0:DeviceManaged, bit1:SharedParams */
        };
        memcpy(buffer, &rpt, sizeof(rpt));
        return sizeof(rpt) + 1;
    }

    /* 未対応 → STALL (=0) */
    else
    {
        return 0;
    }
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t type, uint8_t const *buffer, uint16_t bufsize)
{
    // ここでPCからの出力（FFBなど）も受け取れる
    // if (type != HID_REPORT_TYPE_OUTPUT) return;

    switch (buffer[0])
    {
    case 0x01:
        if (bufsize > 18)
        {
            current_block_idx = buffer[1];
        }
        break;
    case 0x04:
        if (bufsize >= 3)
        {
            ffb_magnitude = (int16_t)(buffer[4] | (buffer[5] << 8)); // read offset value as magnitude
        }
        break;
    case 0x05:
        if (bufsize >= 2)
        {
            current_block_idx = buffer[1];
            uint8_t LSB = buffer[2];
            uint8_t MSB = buffer[3];
            ffb_magnitude = (int16_t)(buffer[2] | (buffer[3] << 8));
        }
        break;
    case 0x0A:
        if (bufsize < 3)
            break;
        uint8_t idx = buffer[1];
        uint8_t op = buffer[2]; // 3=All Stop 1=Start
        if (idx != current_block_idx)
            break; /* ×: 未知ハンドルなら無視 */
        ffb_active = (op == 1);
        break;
    default:
        break;
    }
    return;
}
