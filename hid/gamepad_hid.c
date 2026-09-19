#include "gamepad_hid.h"

#include "tusb.h"

typedef struct {
    int16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t buttons;
} __attribute__((packed)) gamepad_report_t;

void gamepad_hid_send(int16_t wheel_x, uint16_t accelerator_y,
                      uint16_t brake_z, uint8_t buttons)
{
    if (!tud_hid_ready()) return;
    gamepad_report_t report = {
        .x = wheel_x,
        .y = accelerator_y,
        .z = brake_z,
        .buttons = buttons & 0x0f,
    };
    tud_hid_report(/* report_id */ 0x01, &report, sizeof(report));
}
