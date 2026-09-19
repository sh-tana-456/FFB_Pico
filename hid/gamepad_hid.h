#ifndef GAMEPAD_HID_H
#define GAMEPAD_HID_H

#include <stdint.h>

// buttons uses bits 0..3 for HID Buttons 1..4.
void gamepad_hid_send(int16_t wheel_x, uint16_t accelerator_y,
                      uint16_t brake_z, uint8_t buttons);

#endif
