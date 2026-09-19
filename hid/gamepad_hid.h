#ifndef GAMEPAD_HID_H
#define GAMEPAD_HID_H

#include <stdint.h>

void gamepad_hid_send(int16_t wheel_x, uint16_t pedal_y);

#endif
