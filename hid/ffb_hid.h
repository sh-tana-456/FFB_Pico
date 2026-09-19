#ifndef FFB_HID_H
#define FFB_HID_H

#include <stdbool.h>
#include <stdint.h>

// Update PID force synthesis using the current HID X-axis position.
int16_t ffb_hid_update(int16_t position);
// Sends the PID State Input Report when due. True means it used the HID IN endpoint.
bool ffb_hid_send_state(void);
bool ffb_hid_active(void);

// Compact state intended for CDC diagnostics.
uint8_t ffb_hid_active_effect(void);
uint8_t ffb_hid_effect_type(uint8_t effect_index);
int16_t ffb_hid_effect_magnitude(uint8_t effect_index);
uint32_t ffb_hid_effect_period(uint8_t effect_index);
uint32_t ffb_hid_received_report_count(void);
uint8_t ffb_hid_last_report_id(void);
uint8_t ffb_hid_last_report_type(void);
uint16_t ffb_hid_last_report_length(void);

#endif
