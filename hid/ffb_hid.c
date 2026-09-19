#include "ffb_hid.h"

#include <math.h>
#include <string.h>

#include "pico/time.h"
#include "tusb.h"

#define FFB_BLOCKS 40
enum { ET_CONSTANT = 1, ET_RAMP, ET_SQUARE, ET_SINE, ET_TRIANGLE,
       ET_SAW_UP, ET_SAW_DOWN, ET_SPRING, ET_DAMPER, ET_INERTIA,
       ET_FRICTION, ET_CUSTOM };

typedef struct {
    uint8_t type, gain, custom_count;
    uint16_t duration, phase, dead_band, pos_sat, neg_sat;
    int16_t magnitude, offset, ramp_start, ramp_end, center, pos_coef, neg_coef;
    uint32_t period, attack_time, fade_time;
    uint16_t attack_level, fade_level, custom_period;
} effect_t;

static effect_t effects[FFB_BLOCKS + 1];
static int8_t custom_samples[FFB_BLOCKS + 1][256];
static uint8_t active_block, next_block = 1, last_allocated = 1, device_gain = 255;
static uint32_t effect_start_us;
static bool active, actuators_enabled = true, paused;
static uint32_t received_report_count;
static uint8_t last_report_id, last_report_type;
static uint16_t last_report_length;

static inline int16_t i16(uint8_t const *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static inline uint16_t u16(uint8_t const *p) { return (uint16_t)i16(p); }
static inline uint32_t u32(uint8_t const *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline bool valid(uint8_t n) { return n >= 1 && n <= FFB_BLOCKS; }
static int16_t clamp(float x) { return x > 10000 ? 10000 : x < -10000 ? -10000 : (int16_t)x; }
static float limit(float x, const effect_t *e) {
    if (x > e->pos_sat) return e->pos_sat;
    if (x < -(float)e->neg_sat) return -(float)e->neg_sat;
    return x;
}

bool ffb_hid_active(void) { return active; }
uint8_t ffb_hid_active_effect(void) { return active_block; }
uint8_t ffb_hid_effect_type(uint8_t n) { return valid(n) ? effects[n].type : 0; }
int16_t ffb_hid_effect_magnitude(uint8_t n) { return valid(n) ? effects[n].magnitude : 0; }
uint32_t ffb_hid_effect_period(uint8_t n) { return valid(n) ? effects[n].period : 0; }
uint32_t ffb_hid_received_report_count(void) { return received_report_count; }
uint8_t ffb_hid_last_report_id(void) { return last_report_id; }
uint8_t ffb_hid_last_report_type(void) { return last_report_type; }
uint16_t ffb_hid_last_report_length(void) { return last_report_length; }

bool ffb_hid_send_state(void)
{
    static uint32_t last_state_us;
    uint32_t now = time_us_32();
    if (now - last_state_us < 10000 || !tud_hid_ready()) return false;

    // PID State: paused, actuators enabled, safety, override, actuator power;
    // followed by effect-playing and its block index.
    uint8_t report[2] = {
        (paused ? 0x01 : 0) | (actuators_enabled ? 0x02 : 0) | 0x10,
        active ? (uint8_t)(0x01 | (active_block << 1)) : 0,
    };
    if (!tud_hid_report(/* report_id */ 0x02, report, sizeof(report))) return false;
    last_state_us = now;
    return true;
}

int16_t ffb_hid_update(int16_t position)
{
    static int16_t prev_position;
    static float velocity, prev_velocity, acceleration;
    static uint32_t prev_sample_us;
    if (!active || !actuators_enabled || paused || !valid(active_block)) return 0;

    effect_t const *e = &effects[active_block];
    uint32_t now = time_us_32(), dt = now - prev_sample_us;
    if (!prev_sample_us || dt >= 1000) {
        if (prev_sample_us) {
            float v = ((float)(position - prev_position) * 1000000.0f) / dt;
            acceleration = ((v - prev_velocity) * 1000000.0f) / dt;
            velocity = v;
            prev_velocity = v;
        }
        prev_position = position;
        prev_sample_us = now;
    }
    uint32_t elapsed = (now - effect_start_us) / 1000u;
    if (e->duration && e->duration != UINT16_MAX && elapsed >= e->duration) {
        active = false;
        return 0;
    }

    float value = 0;
    switch (e->type) {
    case ET_CONSTANT: value = e->magnitude; break;
    case ET_RAMP:
        value = (!e->duration || e->duration == UINT16_MAX) ? e->ramp_end :
                e->ramp_start + ((float)(e->ramp_end - e->ramp_start) * elapsed / e->duration);
        break;
    case ET_SQUARE: case ET_SINE: case ET_TRIANGLE: case ET_SAW_UP: case ET_SAW_DOWN: {
        uint32_t period = e->period ? e->period : 1;
        float c = (float)(elapsed % period) / period + (float)e->phase / 36000.0f;
        c -= floorf(c);
        float wave = e->type == ET_SQUARE ? (c < .5f ? 1 : -1) :
                     e->type == ET_SINE ? sinf(2 * (float)M_PI * c) :
                     e->type == ET_TRIANGLE ? 1 - 4 * fabsf(c - .5f) :
                     e->type == ET_SAW_UP ? 2 * c - 1 : 1 - 2 * c;
        value = e->offset + e->magnitude * wave;
        break;
    }
    case ET_SPRING: {
        int32_t d = position - e->center;
        if (d > e->dead_band) d -= e->dead_band;
        else if (d < -(int32_t)e->dead_band) d += e->dead_band;
        else d = 0;
        value = d >= 0 ? (float)d * e->pos_coef / 10000.0f : (float)d * e->neg_coef / 10000.0f;
        value = limit(value, e);
        break;
    }
    case ET_DAMPER:
        value = limit(velocity * (velocity >= 0 ? e->pos_coef : e->neg_coef) / 32767.0f, e);
        break;
    case ET_INERTIA:
        value = limit(acceleration * (acceleration >= 0 ? e->pos_coef : e->neg_coef) / 327670.0f, e);
        break;
    case ET_FRICTION:
        if (fabsf(velocity) > 1) value = copysignf(velocity >= 0 ? e->pos_coef : e->neg_coef, velocity);
        value = limit(value, e);
        break;
    case ET_CUSTOM:
        if (e->custom_count) {
            uint32_t p = e->custom_period ? e->custom_period : 1;
            value = (float)custom_samples[active_block][(elapsed / p) % e->custom_count] * 10000.0f / 127.0f;
        }
        break;
    default: break;
    }
    float envelope = 1;
    if (e->attack_time && elapsed < e->attack_time)
        envelope = (e->attack_level + (10000.0f - e->attack_level) * elapsed / e->attack_time) / 10000.0f;
    else if (e->duration && e->duration != UINT16_MAX && e->fade_time && elapsed + e->fade_time > e->duration)
        envelope = (e->fade_level + (10000.0f - e->fade_level) * (e->duration - elapsed) / e->fade_time) / 10000.0f;
    return clamp(value * envelope * e->gain * device_gain / (255.0f * 255.0f));
}

typedef struct __attribute__((packed)) { uint8_t block, status, pool_lo, pool_hi; } block_load_t;
typedef struct __attribute__((packed)) { uint16_t size; uint8_t max, caps; } pool_t;

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    if (type != HID_REPORT_TYPE_FEATURE) return 0;
    if (report_id == 0x06 && reqlen >= sizeof(block_load_t)) {
        block_load_t r = {last_allocated, 1, 0xff, 0x7f};
        memcpy(buffer, &r, sizeof(r)); return sizeof(r);
    }
    if (report_id == 0x07 && reqlen >= sizeof(pool_t)) {
        pool_t r = {0x8000, 8, 0x03};
        memcpy(buffer, &r, sizeof(r)); return sizeof(r);
    }
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t const *b, uint16_t n)
{
    (void)instance;
    if (!n) return;
    if (!report_id) { report_id = *b++; --n; }
    else if (*b == report_id) { ++b; --n; }

    ++received_report_count;
    last_report_id = report_id;
    last_report_type = (uint8_t)type;
    last_report_length = n;

    if (type == HID_REPORT_TYPE_FEATURE && report_id == 0x05 && n) {
        uint8_t block = next_block;
        next_block = block == FFB_BLOCKS ? 1 : block + 1;
        last_allocated = block;
        memset(&effects[block], 0, sizeof(effects[block]));
        effects[block].type = b[0]; effects[block].gain = 255;
        effects[block].attack_level = effects[block].fade_level = 10000;
        memset(custom_samples[block], 0, sizeof(custom_samples[block]));
        return;
    }
    if (type != HID_REPORT_TYPE_OUTPUT) return;
    uint8_t block;
    switch (report_id) {
    case 0x01:
        if (n >= 9 && valid(b[0])) { block=b[0]; effects[block].type=b[1]; effects[block].duration=u16(b+2); effects[block].gain=b[8]; }
        break;
    case 0x02:
        if (n >= 13 && valid(b[0])) { block=b[0]; effects[block].attack_level=u16(b+1); effects[block].fade_level=u16(b+3); effects[block].attack_time=u32(b+5); effects[block].fade_time=u32(b+9); }
        break;
    case 0x03:
        if (n >= 14 && valid(b[0]) && !(b[1] & 0x0f)) { block=b[0]; effects[block].center=i16(b+2); effects[block].pos_coef=i16(b+4); effects[block].neg_coef=i16(b+6); effects[block].pos_sat=u16(b+8); effects[block].neg_sat=u16(b+10); effects[block].dead_band=u16(b+12); }
        break;
    case 0x04:
        if (n >= 11 && valid(b[0])) { block=b[0]; effects[block].magnitude=i16(b+1); effects[block].offset=i16(b+3); effects[block].phase=u16(b+5); effects[block].period=u32(b+7); }
        break;
    case 0x05:
        if (n >= 3 && valid(b[0])) effects[b[0]].magnitude=i16(b+1);
        break;
    case 0x09:
        if (n >= 5 && valid(b[0])) { effects[b[0]].ramp_start=i16(b+1); effects[b[0]].ramp_end=i16(b+3); }
        break;
    case 0x07:
        if (n >= 4 && valid(b[0])) { block=b[0]; uint16_t off=u16(b+1), count=n-3; if (off < 256) { if (count > 256-off) count=256-off; memcpy(&custom_samples[block][off], b+3, count); } }
        break;
    case 0x0a:
        if (n >= 3 && valid(b[0])) { if ((b[1] == 1 || b[1] == 2) && actuators_enabled && !paused) { active_block=b[0]; effect_start_us=time_us_32(); active=true; } else if (b[1] == 3 && b[0] == active_block) active=false; }
        break;
    case 0x0c:
        if (!n) break;
        switch (b[0]) {
        case 1: actuators_enabled = true; break;                  // Enable Actuators
        case 2: actuators_enabled = false; active = false; break; // Disable Actuators
        case 3: active = false; break;                            // Stop All Effects
        case 4: active = false; paused = false; break;            // Reset
        case 5: paused = true; active = false; break;             // Pause
        case 6: paused = false; break;                            // Continue
        default: break;
        }
        break;
    case 0x0d: if (n) device_gain=b[0]; break;
    case 0x0e:
        if (n >= 4 && valid(b[0])) { effects[b[0]].custom_count=b[1]; effects[b[0]].custom_period=u16(b+2); }
        break;
    default: break;
    }
}
