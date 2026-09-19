#include "encoder_uart.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define ENCODER_UART uart1
#define ENCODER_UART_BAUD 2500000
#define ENCODER_TX_PIN 8
#define ENCODER_RX_PIN 9
#define ENCODER_RS485_DE_PIN 10
#define ENCODER_RS485_RE_PIN 6
#define ENCODER_PACKET_SIZE 12
#define ENCODER_ANGLE_OFFSET 7420
#define ENCODER_RESPONSE_TIMEOUT_US 1000
// The wire transfer itself takes about 48 us at 2.5 Mbps.  Keep a quiet
// interval between queries so PWM switching noise cannot turn continuous
// polling into dropped frames. PWM slots schedule 4 kHz updates; this is a
// secondary guard and does not add work to the 8 kHz PWM ISR.
#define ENCODER_REQUEST_INTERVAL_US 200

static int rx_dma_channel = -1;
static uint8_t rx_dma_buffer[ENCODER_PACKET_SIZE];
static volatile bool rx_busy;
static volatile bool packet_ready;
static uint32_t request_started_us;
static volatile uint32_t request_count;
static volatile uint32_t response_count;
static volatile uint32_t timeout_count;
static volatile uint32_t last_response_us;
static volatile uint32_t max_response_us;

static void encoder_uart_dma_irq_handler(void)
{
    uint32_t mask = 1u << rx_dma_channel;
    if (dma_hw->ints1 & mask) {
        dma_hw->ints1 = mask;
        uint32_t response_us = time_us_32() - request_started_us;
        last_response_us = response_us;
        if (response_us > max_response_us) {
            max_response_us = response_us;
        }
        ++response_count;
        rx_busy = false;
        packet_ready = true;
    }
}

static bool encoder_uart_start_rx_dma(void)
{
    if (rx_busy) return false;

    dma_hw->ints1 = 1u << rx_dma_channel;
    rx_busy = true;
    packet_ready = false;
    dma_channel_set_write_addr(rx_dma_channel, rx_dma_buffer, false);
    dma_channel_set_read_addr(rx_dma_channel, &uart_get_hw(ENCODER_UART)->dr, false);
    dma_channel_set_trans_count(rx_dma_channel, ENCODER_PACKET_SIZE, false);
    dma_start_channel_mask(1u << rx_dma_channel);
    return true;
}

static bool encoder_uart_recover_timeout(void)
{
    if (!rx_busy || time_us_32() - request_started_us < ENCODER_RESPONSE_TIMEOUT_US) {
        return false;
    }

    uint32_t irq = save_and_disable_interrupts();
    if (rx_busy && time_us_32() - request_started_us >= ENCODER_RESPONSE_TIMEOUT_US) {
        dma_channel_set_irq1_enabled(rx_dma_channel, false);
        dma_channel_abort(rx_dma_channel);
        dma_hw->ints1 = 1u << rx_dma_channel;
        rx_busy = false;
        packet_ready = false;
        dma_channel_set_irq1_enabled(rx_dma_channel, true);
        ++timeout_count;
    }
    restore_interrupts(irq);
    return true;
}

static void encoder_uart_send_init_commands(void)
{
    const uint8_t commands[] = {0xBA, 0xC2, 0x62, 0xEA};
    for (uint32_t i = 0; i < sizeof(commands); ++i) {
        gpio_put(ENCODER_RS485_RE_PIN, 1);
        gpio_put(ENCODER_RS485_DE_PIN, 1);
        uart_write_blocking(ENCODER_UART, &commands[i], 1);
        while (uart_get_hw(ENCODER_UART)->fr & UART_UARTFR_BUSY_BITS) {
            tight_loop_contents();
        }
        gpio_put(ENCODER_RS485_RE_PIN, 0);
        gpio_put(ENCODER_RS485_DE_PIN, 0);
        sleep_ms(1);
    }
}

void encoder_uart_init(void)
{
    uart_init(ENCODER_UART, ENCODER_UART_BAUD);
    gpio_set_function(ENCODER_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ENCODER_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(ENCODER_UART, true);

    gpio_init(ENCODER_RS485_DE_PIN);
    gpio_set_dir(ENCODER_RS485_DE_PIN, GPIO_OUT);
    gpio_put(ENCODER_RS485_DE_PIN, 0);
    gpio_init(ENCODER_RS485_RE_PIN);
    gpio_set_dir(ENCODER_RS485_RE_PIN, GPIO_OUT);
    gpio_put(ENCODER_RS485_RE_PIN, 0);
    encoder_uart_send_init_commands();

    rx_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(rx_dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, uart_get_dreq(ENCODER_UART, false));
    dma_channel_configure(rx_dma_channel, &config, rx_dma_buffer,
                          &uart_get_hw(ENCODER_UART)->dr, ENCODER_PACKET_SIZE, false);
    dma_channel_set_irq1_enabled(rx_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_1, encoder_uart_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

int32_t encoder_uart_initial_position(void)
{
    return ENCODER_ANGLE_OFFSET;
}

bool encoder_uart_request_position(void)
{
    if (packet_ready) return false;
    (void)encoder_uart_recover_timeout();
    if (rx_busy || packet_ready) return false;
    if (time_us_32() - request_started_us < ENCODER_REQUEST_INTERVAL_US) {
        return false;
    }

    while (uart_is_readable(ENCODER_UART)) {
        (void)uart_getc(ENCODER_UART);
    }
    if (!encoder_uart_start_rx_dma()) return false;

    uint32_t irq = save_and_disable_interrupts();
    gpio_put(ENCODER_RS485_RE_PIN, 1);
    gpio_put(ENCODER_RS485_DE_PIN, 1);
    while (uart_is_readable(ENCODER_UART)) {
        (void)uart_getc(ENCODER_UART);
    }
    uint8_t command = 0x1A;
    uart_write_blocking(ENCODER_UART, &command, 1);
    while (uart_get_hw(ENCODER_UART)->fr & UART_UARTFR_BUSY_BITS) {
        tight_loop_contents();
    }
    gpio_put(ENCODER_RS485_RE_PIN, 0);
    gpio_put(ENCODER_RS485_DE_PIN, 0);
    request_started_us = time_us_32();
    ++request_count;
    restore_interrupts(irq);
    return true;
}

bool encoder_uart_read_position(int32_t *angle_17bit)
{
    if (!packet_ready) return false;

    packet_ready = false;
    int32_t position = ((rx_dma_buffer[5] & 0x7F) << 16) |
                       (rx_dma_buffer[4] << 8) | rx_dma_buffer[3];
    position -= ENCODER_ANGLE_OFFSET;
    if (position >= 65536) position -= 131072;
    if (position < -65536) position += 131072;
    *angle_17bit = position;
    return true;
}

encoder_uart_stats_t encoder_uart_get_stats(void)
{
    uint32_t irq = save_and_disable_interrupts();
    encoder_uart_stats_t stats = {
        .request_count = request_count,
        .response_count = response_count,
        .timeout_count = timeout_count,
        .last_response_us = last_response_us,
        .max_response_us = max_response_us,
    };
    restore_interrupts(irq);
    return stats;
}
