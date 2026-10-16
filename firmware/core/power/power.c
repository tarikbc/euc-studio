/*
 * Power latch and buttons (see euc/power/power.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/power/power.h"
#include "euc/settings/settings.h"

#define WARM_START_BOOT_COUNT 9U /* above: no button check */
#define COLD_POLL_MS          2U
#define WARM_POLL_MS          10U
#define WARM_STILL_POLLS      3U   /* stay on after more still polls than this */
#define HOLD_PRESSED_POLLS    220U /* more: the rider held the button long enough */
#define HOLD_RELEASED_POLLS   250U /* more: released */
#define HOLD_MAX_POLLS        1000U
#define HOLD_MIN_PRESSED      200U /* at the end, fewer pressed polls mean "turn off" */

#define GESTURE_SPEED_CM_PER_S    49
#define GESTURE_PRESS_TICKS       30U
#define GESTURE_PRESSES           5U
#define GESTURE_GAP_TICKS         2500U
#define GESTURE_HANDLE_OFF_TICKS  20U
#define GESTURE_PRESS_BEEP_TICKS  10U
#define GESTURE_TOGGLE_BEEP_TICKS 250U
#define UNLOCKED_STATE            1U

#define POWER_OFF_SPEED_CM_PER_S  50
#define POWER_OFF_HOLD_TICKS      401U
#define POWER_OFF_BEEP_HOLD_TICKS 400U
#define POWER_OFF_HOLD_LIMIT      500U
#define POWER_OFF_HOLD_AT_BOOT    1100U /* above the limit: a button held since power-on does not count */
#define POWER_OFF_BEEP_TICKS      50U

#define TICKS_PER_SECOND    501U /* the second counter restarts at 1 after passing 500 */
#define AUTO_OFF_SECONDS    7200U
#define AUTO_OFF_BEEPS      8U
#define AUTO_OFF_WARNINGS   12U
#define AUTO_OFF_BEEP_TICKS 300U

void euc_power_boot_init(struct euc_power_boot_state *state, uint16_t boot_count)
{
    (void)memset(state, 0, sizeof(*state));
    state->warm_start = boot_count > WARM_START_BOOT_COUNT;
}

uint16_t euc_power_boot_poll_ms(const struct euc_power_boot_state *state)
{
    return state->warm_start ? (uint16_t)WARM_POLL_MS : (uint16_t)COLD_POLL_MS;
}

enum euc_power_boot_decision euc_power_boot_poll(struct euc_power_boot_state *state, bool button_pressed,
                                                 int16_t hall_position)
{
    const bool moved = hall_position != state->last_position;
    state->last_position = hall_position;

    if (state->warm_start) {
        state->still_polls = moved ? 0U : (uint16_t)(state->still_polls + 1U);
        return (state->still_polls > WARM_STILL_POLLS) ? EUC_POWER_BOOT_STAY_ON : EUC_POWER_BOOT_WAIT;
    }

    if (button_pressed) {
        state->pressed_polls++;
        state->released_polls = 0U;
    } else {
        state->released_polls++;
        state->pressed_polls = 0U;
        state->polls_since_release = 0U;
    }
    bool finished = false;
    if (moved) {
        /* The wheel turned: start again. */
        state->pressed_polls = 0U;
        state->released_polls = 0U;
    } else {
        finished =
            (state->pressed_polls > HOLD_PRESSED_POLLS) || (state->released_polls > HOLD_RELEASED_POLLS);
    }
    state->polls_since_release++;
    if (state->polls_since_release >= HOLD_MAX_POLLS) {
        finished = true;
    }
    if (!finished) {
        return EUC_POWER_BOOT_WAIT;
    }
    return (state->pressed_polls < HOLD_MIN_PRESSED) ? EUC_POWER_BOOT_TURN_OFF : EUC_POWER_BOOT_STAY_ON;
}

void euc_power_init(struct euc_power_state *state)
{
    (void)memset(state, 0, sizeof(*state));
    state->press_ticks = 50U; /* above the press length: no press counted until the button is released */
    state->hold_ticks = POWER_OFF_HOLD_AT_BOOT;
}

static void add_beep(struct euc_power_out *out, uint16_t ticks)
{
    if (out->beep_count < EUC_POWER_MAX_BEEPS) {
        out->beep_ticks[out->beep_count] = ticks;
        out->beep_count++;
    }
}

static bool speed_within(int16_t speed, int16_t limit)
{
    return (speed >= -limit) && (speed <= limit);
}

static void lock_gesture(struct euc_power_state *state, const struct euc_power_in *in,
                         struct euc_power_out *out)
{
    if (!speed_within(in->speed_cm_per_s, GESTURE_SPEED_CM_PER_S)) {
        return;
    }
    if (!in->handle_button_pressed) {
        state->handle_release_ticks++;
        if (state->handle_release_ticks > GESTURE_HANDLE_OFF_TICKS) {
            state->handle_release_ticks = 10U;
            state->gesture_blocked = false;
            state->button_release_ticks = 0U;
            state->press_ticks = 0U;
            state->presses = 0U;
        }
        return;
    }
    state->handle_release_ticks = 0U;

    if (!in->power_button_pressed) {
        state->press_ticks = 0U;
        if ((state->button_release_ticks + 1U) <= GESTURE_GAP_TICKS) {
            state->button_release_ticks++;
        } else {
            /* More than 5 s since the last press: start over, and count no more presses until the handle
             * button is released. */
            state->button_release_ticks = GESTURE_GAP_TICKS;
            state->presses = 0U;
            state->gesture_blocked = true;
        }
        return;
    }

    state->button_release_ticks = 0U;
    if (state->press_ticks < GESTURE_PRESS_TICKS) {
        state->press_ticks++;
    }
    if (state->press_ticks == GESTURE_PRESS_TICKS) {
        state->press_ticks = GESTURE_PRESS_TICKS + 1U;
        if (!state->gesture_blocked) {
            state->presses++;
            add_beep(out, GESTURE_PRESS_BEEP_TICKS);
        }
        if (state->presses == GESTURE_PRESSES) {
            if (out->lock_state == EUC_SETTINGS_LOCKED) {
                out->lock_state = UNLOCKED_STATE;
            } else {
                out->lock_state = EUC_SETTINGS_LOCKED;
                out->disable_drive = true;
                out->hard_cut = true;
            }
            add_beep(out, GESTURE_TOGGLE_BEEP_TICKS);
        }
    }
}

static void power_off_button(struct euc_power_state *state, const struct euc_power_in *in,
                             struct euc_power_out *out)
{
    if (in->power_button_pressed) {
        state->hold_ticks++;
        if (state->hold_ticks > POWER_OFF_HOLD_LIMIT) {
            state->hold_ticks = POWER_OFF_HOLD_LIMIT;
        }
        if (speed_within(in->speed_cm_per_s, POWER_OFF_SPEED_CM_PER_S)) {
            if (state->hold_ticks == POWER_OFF_BEEP_HOLD_TICKS) {
                add_beep(out, POWER_OFF_BEEP_TICKS);
            }
            if (state->hold_ticks == POWER_OFF_HOLD_TICKS) {
                out->hard_cut = true;
                out->disable_drive = true;
                state->powering_off = true;
                out->save_settings = true;
                out->power_off = true;
            }
            return;
        }
    }
    state->hold_ticks = 0U;
}

static void auto_power_off(struct euc_power_state *state, const struct euc_power_in *in,
                           struct euc_power_out *out)
{
    if (!in->drive_disabled) {
        if (in->hall_position != state->last_position) {
            state->idle_seconds = 0U;
        }
        state->last_position = in->hall_position;
    }

    const bool second = state->second_ticks >= TICKS_PER_SECOND;
    state->second_ticks++;
    if (second) {
        state->second_ticks = 1U;
        state->idle_seconds++;
        if (state->idle_seconds > AUTO_OFF_SECONDS) {
            state->hold_ticks = 0U;
            state->warnings++;
            if (state->warnings <= AUTO_OFF_BEEPS) {
                add_beep(out, AUTO_OFF_BEEP_TICKS);
            }
            if (state->warnings > AUTO_OFF_WARNINGS) {
                out->save_settings = true;
                out->power_off = true;
                state->warnings = AUTO_OFF_WARNINGS;
            }
        }
    }
    out->auto_off_remaining_s = (uint16_t)(AUTO_OFF_SECONDS - (uint32_t)state->idle_seconds);
}

void euc_power_tick(struct euc_power_state *state, const struct euc_power_in *in, struct euc_power_out *out)
{
    (void)memset(out, 0, sizeof(*out));
    out->lock_state = in->lock_state;
    lock_gesture(state, in, out);
    power_off_button(state, in, out);
    auto_power_off(state, in, out);
}
