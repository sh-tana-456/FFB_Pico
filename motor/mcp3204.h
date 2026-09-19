#ifndef MCP3204_H
#define MCP3204_H

#include <stdbool.h>
#include <stdint.h>

// Called from the DMA IRQ after both configured channels have been sampled.
typedef void (*mcp3204_sample_callback_t)(uint16_t channel_0, uint16_t channel_1);

void mcp3204_init(mcp3204_sample_callback_t sample_callback);
bool mcp3204_start_read(void);

#endif
