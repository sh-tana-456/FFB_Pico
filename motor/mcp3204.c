#include "mcp3204.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"

#define MCP3204_SPI spi1
#define MCP3204_PIN_DOUT 12
#define MCP3204_PIN_CS 13
#define MCP3204_PIN_CLK 14
#define MCP3204_PIN_DIN 15
#define MCP3204_SPI_HZ 1000000
#define MCP3204_NUM_CHANNELS 2
#define MCP3204_BYTES_PER_SAMPLE 3

static int dma_tx_channel = -1;
static int dma_rx_channel = -1;
static uint8_t dma_tx_buffer[MCP3204_BYTES_PER_SAMPLE];
static uint8_t dma_rx_buffer[MCP3204_BYTES_PER_SAMPLE];
static uint16_t work_raw[MCP3204_NUM_CHANNELS];
static volatile uint8_t active_channel;
static volatile bool dma_busy;
static mcp3204_sample_callback_t sample_callback;

static void mcp3204_prepare_command(uint8_t channel)
{
    dma_tx_buffer[0] = 0x06;
    dma_tx_buffer[1] = channel << 6;
    dma_tx_buffer[2] = 0x00;
}

static void mcp3204_start_channel(uint8_t channel)
{
    active_channel = channel;
    mcp3204_prepare_command(channel);
    gpio_put(MCP3204_PIN_CS, 0);

    dma_channel_set_read_addr(dma_rx_channel, &spi_get_hw(MCP3204_SPI)->dr, false);
    dma_channel_set_write_addr(dma_rx_channel, dma_rx_buffer, false);
    dma_channel_set_trans_count(dma_rx_channel, MCP3204_BYTES_PER_SAMPLE, false);
    dma_channel_set_read_addr(dma_tx_channel, dma_tx_buffer, false);
    dma_channel_set_trans_count(dma_tx_channel, MCP3204_BYTES_PER_SAMPLE, false);
    dma_start_channel_mask((1u << dma_rx_channel) | (1u << dma_tx_channel));
}

static void mcp3204_dma_irq_handler(void)
{
    dma_hw->ints0 = 1u << dma_rx_channel;
    gpio_put(MCP3204_PIN_CS, 1);

    work_raw[active_channel] =
        ((dma_rx_buffer[1] & 0x0F) << 8) | dma_rx_buffer[2];

    if (active_channel < MCP3204_NUM_CHANNELS - 1) {
        mcp3204_start_channel(active_channel + 1);
        return;
    }

    dma_busy = false;
    if (sample_callback) {
        sample_callback(work_raw[0], work_raw[1]);
    }
}

void mcp3204_init(mcp3204_sample_callback_t callback)
{
    sample_callback = callback;
    spi_init(MCP3204_SPI, MCP3204_SPI_HZ);
    spi_set_format(MCP3204_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(MCP3204_PIN_DOUT, GPIO_FUNC_SPI);
    gpio_set_function(MCP3204_PIN_CLK, GPIO_FUNC_SPI);
    gpio_set_function(MCP3204_PIN_DIN, GPIO_FUNC_SPI);
    gpio_init(MCP3204_PIN_CS);
    gpio_set_dir(MCP3204_PIN_CS, GPIO_OUT);
    gpio_put(MCP3204_PIN_CS, 1);

    dma_tx_channel = dma_claim_unused_channel(true);
    dma_rx_channel = dma_claim_unused_channel(true);

    dma_channel_config tx_config = dma_channel_get_default_config(dma_tx_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, spi_get_dreq(MCP3204_SPI, true));
    dma_channel_configure(dma_tx_channel, &tx_config, &spi_get_hw(MCP3204_SPI)->dr,
                          dma_tx_buffer, MCP3204_BYTES_PER_SAMPLE, false);

    dma_channel_config rx_config = dma_channel_get_default_config(dma_rx_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, spi_get_dreq(MCP3204_SPI, false));
    dma_channel_configure(dma_rx_channel, &rx_config, dma_rx_buffer,
                          &spi_get_hw(MCP3204_SPI)->dr, MCP3204_BYTES_PER_SAMPLE, false);

    dma_channel_set_irq0_enabled(dma_rx_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, mcp3204_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_busy = false;
}

bool mcp3204_start_read(void)
{
    if (dma_busy) return false;
    dma_busy = true;
    mcp3204_start_channel(0);
    return true;
}
