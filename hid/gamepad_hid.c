#include "gamepad_hid.h"

#include "tusb.h"

typedef struct {
    int16_t x;
    int16_t y;
} __attribute__((packed)) gamepad_report_t;

void gamepad_hid_send(int16_t wheel_x, uint16_t pedal_y)
{
    if (!tud_hid_ready()) return;
    gamepad_report_t report = {.x = wheel_x, .y = (int16_t)pedal_y};
    tud_hid_report(/* report_id */ 0x01, &report, sizeof(report));
}
