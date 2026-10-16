/*
 * Light on PB12 (see euc/lights/lights.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/lights/lights.h"
#include "euc/quirks.h"

#define LIGHT_MODES 3U

#define PRESS_DEBOUNCE_TICKS 10U
#define HOLD_LIMIT_TICKS     220U
#define SHORT_PRESS_MIN      11U
#define SHORT_PRESS_SPAN     209U /* hold ticks 11..219 */
#define HOLD_AT_BOOT         220U /* a press held since power-on is not a short press */

#define BRIGHTNESS_AT_BOOT   8U
#define BRIGHTNESS_MAX       40U
#define BRIGHTNESS_MIN       12U
#define BRIGHTEN_EVERY_TICKS 5U /* +1 when the count passes this */
#define DIM_EVERY_TICKS      11U
#define DIM_AFTER_SECONDS    10U
#define SECOND_PERIOD        500U
#define MOVING_DM_PER_S      18 /* faster: brighten */
#define STILL_DM_PER_S       7  /* slower or equal: dim after a while */

#define STROBE_PERIOD   100U
#define STROBE_OFF_PART 51U
#define STROBE_DUTY     24U

#define PWM_STEPS_LAST 40U

void euc_lights_init(struct euc_lights_state *state, uint16_t backup_light_mode, uint8_t wm_value)
{
    (void)memset(state, 0, sizeof(*state));
    state->request_count = (uint8_t)backup_light_mode;
    state->light_state = (uint8_t)backup_light_mode;
    state->hold_ticks = HOLD_AT_BOOT;
    state->brightness = BRIGHTNESS_AT_BOOT;
    state->strip_pattern = wm_value;
}

void euc_lights_set_mode(struct euc_lights_state *state, uint8_t mode)
{
    state->request_count = mode;
}

void euc_lights_set_strip_pattern(struct euc_lights_state *state, uint8_t pattern)
{
    state->strip_pattern = pattern;
}

static void read_button(struct euc_lights_state *state, bool pressed, struct euc_lights_out *out)
{
    if (pressed) {
        state->press_ticks++;
        if (state->press_ticks > PRESS_DEBOUNCE_TICKS) {
            state->press_ticks = PRESS_DEBOUNCE_TICKS + 1U;
            state->hold_ticks++;
            if (state->hold_ticks > HOLD_LIMIT_TICKS) {
                state->hold_ticks = HOLD_LIMIT_TICKS;
            }
        }
        return;
    }
    state->press_ticks = 0U;
    const bool short_press = ((uint32_t)state->hold_ticks - SHORT_PRESS_MIN) < SHORT_PRESS_SPAN;
    state->hold_ticks = 0U;
    if (short_press) {
        state->request_count++;
        if ((state->request_count % LIGHT_MODES) == 0U) {
            state->strip_pattern = (uint8_t)((state->strip_pattern >= 1U) ? 0U : 1U);
            out->wm_changed = true;
        }
    }
}

static void follow_speed(struct euc_lights_state *state, const struct euc_lights_in *in)
{
    const int16_t speed = in->speed_dm_per_s;
    if ((speed > MOVING_DM_PER_S) || (speed < -MOVING_DM_PER_S)) {
        state->still_seconds = 0U;
        state->dim_ticks = 0U;
        state->brighten_ticks++;
        if (state->brighten_ticks > BRIGHTEN_EVERY_TICKS) {
            state->brightness++;
            state->brighten_ticks = 0U;
        }
        if (state->brightness > BRIGHTNESS_MAX) {
            state->brightness = BRIGHTNESS_MAX;
            state->dim_ticks = 0U;
        }
        return;
    }
    if ((speed > STILL_DM_PER_S) || (speed < -STILL_DM_PER_S)) {
        state->still_seconds = 0U;
        state->dim_ticks = 0U;
        return;
    }
    if ((in->time_base % SECOND_PERIOD) == 0U) {
        state->still_seconds++;
    }
    state->dim_ticks++;
    if ((state->dim_ticks >= DIM_EVERY_TICKS) && (state->still_seconds >= DIM_AFTER_SECONDS)) {
        state->still_seconds = DIM_AFTER_SECONDS;
        state->dim_ticks = 0U;
        state->brightness--;
        if (state->brightness < BRIGHTNESS_MIN) {
            state->brightness = BRIGHTNESS_MIN;
        }
    }
}

void euc_lights_tick(struct euc_lights_state *state, const struct euc_lights_in *in,
                     struct euc_lights_out *out)
{
    (void)memset(out, 0, sizeof(*out));

    if (in->transport_locked) {
        state->light_state = EUC_LIGHT_OFF;
        state->strip_pattern = 0U;
        if (!EUC_QUIRK(LOCK_LIGHT_STAYS_ON)) {
            state->duty = 0U;
        }
        out->light_state = state->light_state;
        return;
    }

    read_button(state, in->power_button_pressed, out);
    switch (state->request_count % LIGHT_MODES) {
    case EUC_LIGHT_OFF:
        state->duty = 0U;
        state->light_state = EUC_LIGHT_OFF;
        break;
    case EUC_LIGHT_ON:
        follow_speed(state, in);
        state->light_state = EUC_LIGHT_ON;
        state->duty = state->brightness;
        break;
    default:
        state->light_state = EUC_LIGHT_STROBE;
        state->duty = ((in->time_base % STROBE_PERIOD) < STROBE_OFF_PART) ? 0U : (uint8_t)STROBE_DUTY;
        break;
    }
    out->light_state = state->light_state;
    out->strip_pattern = state->strip_pattern;
}

bool euc_lights_pwm_step(struct euc_lights_state *state)
{
    const uint8_t count = (uint8_t)(state->pwm_counter + 1U);
    state->pwm_counter = (count > PWM_STEPS_LAST) ? 0U : count;
    return count < state->duty;
}
