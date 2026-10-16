/*
 * Balance task: per-tick order, pedal-mode gains, handle button and the hall speed estimate.
 * Tiltbacks: tiltback.c. Cut-offs, start-up and recovery: cutoff.c. Law and output limits: law.c.
 */
#include "euc/balance/balance.h"

#include <stddef.h>

#include "balance_internal.h"
#include "euc/math/float_ops.h"
#include "euc/quirks.h"

/* Power-on wait before the first start-up check (ticks). */
#define START_COUNTDOWN_TICKS (-200)

/* Handle button: a press counts after 50 ticks (0.1 s) at |v| <= 0.2 m/s; one press alone times out after 1
 * s. */
#define HANDLE_SPEED_LIMIT_DM_PER_S 2
#define HANDLE_PRESS_TICKS          50U
#define HANDLE_PRESS_REARM_TICKS    110U
#define HANDLE_SINGLE_TIMEOUT_TICKS 500U
#define HANDLE_RESTORE_BEEP_TICKS   25U

struct pedal_gains {
    float kp_base;
    float kd_base;
    int16_t kc_base;
    float accel_gain;
};

void euc_balance_init(struct euc_balance_state *state)
{
    *state = (struct euc_balance_state){0};
    euc_attitude_init(&state->attitude);
    state->start_state = START_COUNTDOWN_TICKS;
}

void euc_balance_restart_countdown(struct euc_balance_state *state, int16_t ticks)
{
    state->start_state = (int16_t)-ticks;
}

/* Gains ------------------------------------------------------------------------------------------------ */

void euc_balance_step_gains(struct euc_balance_state *state, const struct euc_balance_settings *settings)
{
    /* Index: pedal mode 0 soft, 1 medium, 2 hard; any other value acts as soft. */
    static const struct pedal_gains voltage_mode[3] = {
        {50.0F, 600.0F, 22, 0.6F}, {55.0F, 600.0F, 25, 0.45F}, {60.0F, 600.0F, 30, 0.2F}};
    static const struct pedal_gains current_mode[3] = {
        {120.0F, 1500.0F, 0, 0.6F}, {150.0F, 1800.0F, 0, 0.45F}, {200.0F, 2300.0F, 0, 0.3F}};
    const size_t mode = (settings->pedal_mode <= 2U) ? settings->pedal_mode : 0U;
    struct euc_balance_gains *gains = &state->gains;

    if (settings->drive_mode == 0U) {
        gains->kp_base = voltage_mode[mode].kp_base;
        gains->kd_base = voltage_mode[mode].kd_base;
        gains->kc_base = voltage_mode[mode].kc_base;
        gains->accel_gain = voltage_mode[mode].accel_gain;
        gains->kc_slope = 8;
        /* Stock tuning works on a hall-period speed that reads 1.2 x high in this mode. Clean reads true mm/s
         * (quirk HALL_SPEED_1_2X), so its gain is 1.2 x higher and the feed-forward stays the same. */
        gains->kv = EUC_QUIRK(HALL_SPEED_1_2X) ? 30.0F : 36.0F;
    } else {
        /* Current mode: no current feedback, no speed feed-forward; the current-feedback slope is left as is.
         */
        gains->kp_base = current_mode[mode].kp_base;
        gains->kd_base = current_mode[mode].kd_base;
        gains->kc_base = current_mode[mode].kc_base;
        gains->accel_gain = current_mode[mode].accel_gain;
        gains->kv = 0.0F;
    }
}

/* Handle button ---------------------------------------------------------------------------------------- */

static void handle_released(struct euc_balance_state *state, struct euc_balance_out *out)
{
    state->handle_press_ticks = 0U;
    state->handle_press_latched = false;
    if (state->handle_presses == 1U) {
        state->handle_release_ticks++;
        if (state->handle_release_ticks > HANDLE_SINGLE_TIMEOUT_TICKS) {
            state->handle_release_ticks = HANDLE_SINGLE_TIMEOUT_TICKS;
            state->handle_presses = 0U;
        }
        state->handle_offset_deg = balance_fade_offset(state->handle_offset_deg);
    } else if (state->handle_presses == 2U) {
        /* Motor cut: hold the output limit at 1 duty unit. */
        state->command_limit = 1.0F;
        state->handle_offset_deg = 0.0F;
    } else if (state->handle_presses > 2U) {
        /* Third press: restore, starting from the current error so the pedals do not jump. */
        state->handle_offset_deg = state->error_deg;
        balance_request_beep(out, HANDLE_RESTORE_BEEP_TICKS);
        state->handle_presses = 0U;
        state->handle_offset_deg = balance_fade_offset(state->handle_offset_deg);
    } else {
        state->handle_offset_deg = balance_fade_offset(state->handle_offset_deg);
    }
}

static void handle_pressed(struct euc_balance_state *state, struct euc_balance_out *out)
{
    state->handle_release_ticks = 0U;
    const int32_t speed_dm_per_s = state->speed_dm_per_s;
    if ((speed_dm_per_s >= -HANDLE_SPEED_LIMIT_DM_PER_S) && (speed_dm_per_s <= HANDLE_SPEED_LIMIT_DM_PER_S)) {
        state->handle_press_ticks++;
        if ((state->handle_press_ticks > HANDLE_PRESS_TICKS) && !state->handle_press_latched) {
            state->handle_presses++;
            if (state->handle_presses == 2U) {
                out->handle_double_beep = true;
            }
            state->handle_press_latched = true;
            state->handle_press_ticks = HANDLE_PRESS_REARM_TICKS;
        } else if (state->handle_press_ticks > HANDLE_PRESS_REARM_TICKS) {
            state->handle_press_ticks = 0U;
        } else {
            /* counting */
        }
    } else {
        state->handle_press_ticks = 0U;
    }
    state->handle_offset_deg = balance_fade_offset(state->handle_offset_deg);
}

void euc_balance_step_handle(struct euc_balance_state *state, const struct euc_balance_in *in,
                             struct euc_balance_out *out)
{
    if (in->handle_button_released) {
        handle_released(state, out);
    } else {
        handle_pressed(state, out);
    }
    state->error_deg = euc_f32_sub(state->error_deg, state->handle_offset_deg);
}

/* Speed estimate --------------------------------------------------------------------------------------- */

void euc_balance_step_speed(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    const int32_t hall_steps = in->hall_steps_per_window;

    /* Gain-scheduling speed: (98 x previous - 842 x steps) / 100, time constant about 0.1 s. */
    state->speed_filter = ((state->speed_filter * 98) + (hall_steps * -842)) / 100;
    state->speed_cm_per_s = (int16_t)(state->speed_filter / 10);

    /* Display speed: 0.9 x previous + 0.1 x (-421 mm/s per step). */
    state->speed_mm_per_s = (int16_t)(((hall_steps * -421) + ((int32_t)state->speed_mm_per_s * 9)) / 10);
    state->speed_dm_per_s = (int16_t)(state->speed_mm_per_s / 100);
    state->speed_m_per_s = euc_f32_div((float)state->speed_dm_per_s, 10.0F);

    /* Frame 01 speed copy: refreshed while |current / 20| exceeds |speed| (meaning unknown). */
    int32_t current_term = in->motor_current_lpf_ma / 20;
    current_term = (current_term < 0) ? -current_term : current_term;
    const int32_t speed_magnitude =
        (state->speed_dm_per_s < 0) ? -(int32_t)state->speed_dm_per_s : (int32_t)state->speed_dm_per_s;
    if ((int16_t)(current_term - speed_magnitude) > 0) {
        state->frame01_speed_copy_dm_per_s = state->speed_dm_per_s;
    }
}

/* Tick ------------------------------------------------------------------------------------------------- */

static void running_tick(struct euc_balance_state *state, const struct euc_balance_in *in,
                         const struct euc_attitude_in *attitude_in, struct euc_balance_out *out)
{
    struct euc_attitude_out attitude_out;
    euc_attitude_update(&state->attitude, attitude_in, &attitude_out);
    if (!attitude_out.valid) {
        balance_numeric_fault(state); /* clean only: the attitude filter has been reset */
    }
    state->error_deg = attitude_out.pitch_deg;

    euc_balance_step_tiltbacks(state, in);
    euc_balance_step_handle(state, in, out);
    euc_balance_step_speed(state, in);
    euc_balance_step_cutoffs(state, in, out);

    if (!state->motor_disabled) {
        euc_balance_step_law(state, in);
    } else {
        euc_balance_step_ramp_down(state);
        /* The recovery check waits until the command has reached zero (a zero exponent). */
        const bool command_zero = (euc_f32_bits(state->command) & EUC_F32_EXPONENT_MASK) == 0U;
        if (command_zero && state->fallen) {
            euc_balance_step_recovery(state, in, out);
        }
    }
}

void euc_balance_tick(struct euc_balance_state *state, const struct euc_balance_in *in,
                      struct euc_balance_out *out)
{
    *out = (struct euc_balance_out){0};

    struct euc_attitude_in attitude_in;
    attitude_in.gyro_at_rail = false;
    for (size_t axis = 0U; axis < EUC_IMU_AXES; axis++) {
        attitude_in.accel[axis] = euc_imu_correct(in->accel_raw[axis], in->accel_offset[axis]);
        attitude_in.gyro[axis] = euc_imu_correct(in->gyro_raw[axis], in->gyro_offset[axis]);
        attitude_in.gyro_at_rail = attitude_in.gyro_at_rail || (in->gyro_raw[axis] == INT16_MAX) ||
                                   (in->gyro_raw[axis] == INT16_MIN);
    }
    euc_attitude_update_tilt(&state->attitude, attitude_in.accel);
    euc_balance_step_gains(state, &in->settings);

    if (state->start_state < 1) {
        euc_balance_step_startup(state);
    } else {
        attitude_in.accel_gain = state->gains.accel_gain;
        attitude_in.turn_compensation = in->settings.turn_compensation;
        attitude_in.speed_dm_per_s = state->speed_dm_per_s;
        attitude_in.speed_mm_per_s = state->speed_mm_per_s;
        attitude_in.motor_disabled = state->motor_disabled;
        attitude_in.hard_inhibit = in->hard_inhibit;
        running_tick(state, in, &attitude_in, out);
    }

    euc_balance_step_output(state);

    /* The stock command executor runs after the balance task, so a disable request acts from the next tick.
     */
    if (in->disable_request) {
        state->motor_disabled = true;
    }

    out->motor_command = state->motor_command;
    out->pitch_deg = state->attitude.pitch_deg;
    out->roll_deg = state->attitude.roll_deg;
    out->speed_cm_per_s = state->speed_cm_per_s;
    out->motor_disabled = state->motor_disabled;
    out->numeric_fault = state->numeric_fault;
}
