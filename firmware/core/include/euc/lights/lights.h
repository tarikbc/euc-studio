/*
 * Light on PB12: mode, brightness and the software PWM that drives the pin.
 *
 * Mode = request count modulo 3: 0 off, 1 on, 2 strobe. A short press of the power button (pressed for 21 to
 * 229 ticks) counts up; BLE 'E' / 'Q' / 'T' set the count to 0 / 1 / 2. Every press that lands on "off" also
 * toggles the LED-strip pattern ('W' 'M' value 0/1). The count starts from BKP DR11 (stock never writes it).
 *
 * On: brightness 0..40 of the 41-step PWM. Faster than 1.8 m/s it rises by 1 every 6 ticks up to 40; at a
 * standstill (|v| <= 0.7 m/s) for 10 s it falls by 1 every 11 ticks down to 12; in between it holds.
 * Strobe: 24/41 duty for 49 ticks, then off for 51, on the alarm time base.
 * Transport lock: telemetry shows the light off and the strip pattern 0. Stock leaves PB12 at its last duty
 * (quirk LOCK_LIGHT_STAYS_ON); clean turns it off.
 *
 * The PWM step runs on every TIM8 update event (20 or 24 kHz): a 41-step counter, PB12 high while the counter
 * is below the duty.
 */
#ifndef EUC_LIGHTS_LIGHTS_H
#define EUC_LIGHTS_LIGHTS_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_LIGHT_OFF    0U
#define EUC_LIGHT_ON     1U
#define EUC_LIGHT_STROBE 2U

struct euc_lights_state {
    uint8_t request_count; /* mode = count % 3 */
    uint8_t press_ticks;
    uint8_t hold_ticks;
    uint8_t strip_pattern; /* 'W' 'M' 0/1 */
    uint8_t brightness;
    uint8_t brighten_ticks;
    uint16_t dim_ticks;
    uint16_t still_seconds;
    uint8_t duty;        /* 0..40 */
    uint8_t light_state; /* frame 04 byte 15 */
    uint8_t pwm_counter;
};

struct euc_lights_in {
    bool power_button_pressed;
    int16_t speed_dm_per_s;
    uint16_t time_base; /* alarm time base */
    bool transport_locked;
};

struct euc_lights_out {
    uint8_t light_state;   /* frame 04 byte 15 */
    uint8_t strip_pattern; /* LED-strip pattern for the strip driver */
    bool wm_changed;       /* a press toggled the pattern: store it as the 'W' 'M' setting */
};

/* Power-on state: request count from BKP DR11, strip pattern from the 'W' 'M' setting. */
void euc_lights_init(struct euc_lights_state *state, uint16_t backup_light_mode, uint8_t wm_value);

/* BLE 'E' (0), 'Q' (1), 'T' (2). */
void euc_lights_set_mode(struct euc_lights_state *state, uint8_t mode);

/* BLE 'W' 'M' d. */
void euc_lights_set_strip_pattern(struct euc_lights_state *state, uint8_t pattern);

/* One RTC tick, after the alarms. Clears `out` first. */
void euc_lights_tick(struct euc_lights_state *state, const struct euc_lights_in *in,
                     struct euc_lights_out *out);

/* One TIM8 update event: the PB12 level. */
bool euc_lights_pwm_step(struct euc_lights_state *state);

#endif
