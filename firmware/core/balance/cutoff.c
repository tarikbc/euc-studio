/*
 * When the balance stops and when it may start again.
 *
 * Cut-offs (every running tick, after the speed estimate):
 *   slow roll   |roll| > 40 + 5 x setting deg for 21 ticks                       -> tipped
 *   fast roll   |roll| > 50 / 58 / 66 deg and (hall activity or Vq > 30000)        -> tipped
 *   pitch       |pitch| > 40 deg, or tipped, for 52 ticks  -> soft disable, automatic recovery allowed
 *   error       |error| > 40 deg, or tipped, for 11 ticks  -> soft disable, one 121 ms beep
 *   overload    |Vq| > 31000 (command above 1033 of 1050) for 752 ticks (1.5 s)    -> soft disable
 * Start-up: after the power-on countdown, the wheel must stand nearly level (accelerometer-only tilt).
 * Recovery: after a pitch cut-off, 101 ticks level and still re-enable the motor, unless a hard fault is set.
 */
#include <stdint.h>

#include "balance_internal.h"
#include "euc/balance/balance.h"

#define PITCH_CUTOFF_DEG        40.0F
#define PITCH_CUTOFF_TICKS      51U
#define ERROR_CUTOFF_TICKS      10U
#define ERROR_CUTOFF_BEEP_TICKS 60U
#define OVERLOAD_VOLTAGE_Q      31000
#define OVERLOAD_TICKS          750U
#define SLOW_ROLL_BASE_DEG      40U
#define SLOW_ROLL_STEP_DEG      5U
#define SLOW_ROLL_TICKS         20U
#define FAST_ROLL_HALL_STEPS    30
#define FAST_ROLL_VOLTAGE_Q     30000
#define HALL_11_WINDOWS_LIMIT   25

/* Accelerometer-only tilt windows per sensor axis, 0.01 deg: x about +-8 deg, y +-20 deg, z above 45 deg. */
#define START_OUTER_CENTIDEG    4500
#define START_X_CENTIDEG        800
#define START_Y_CENTIDEG        2000
#define START_Z_CENTIDEG        4500
#define RECOVERY_X_MIN_CENTIDEG (-799)
#define RECOVERY_X_MAX_CENTIDEG 800
#define RECOVERY_SPEED_DM_PER_S 2
#define RECOVERY_TICKS          100
#define RECOVERY_BEEP_TICKS     20U

static void roll_check(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    const float roll_deg = state->attitude.roll_deg;

    const float slow_limit_deg =
        (float)(uint8_t)((in->settings.roll_limit_step * SLOW_ROLL_STEP_DEG) + SLOW_ROLL_BASE_DEG);
    if ((slow_limit_deg < roll_deg) || (roll_deg < -slow_limit_deg)) {
        state->roll_over_ticks++;
        if (state->roll_over_ticks > SLOW_ROLL_TICKS) {
            state->tipped = true;
        }
    } else {
        state->roll_over_ticks = 0U;
    }

    static const float fast_limit_deg[3] = {50.0F, 58.0F, 66.0F};
    const float fast_limit =
        fast_limit_deg[(in->settings.roll_cutoff_mode <= 2U) ? in->settings.roll_cutoff_mode : 0U];
    if ((fast_limit < roll_deg) || (roll_deg < -fast_limit)) {
        int32_t hall_activity = in->hall_steps_11_windows;
        if (hall_activity > HALL_11_WINDOWS_LIMIT) {
            hall_activity = HALL_11_WINDOWS_LIMIT;
        } else if (hall_activity < -HALL_11_WINDOWS_LIMIT) {
            hall_activity = -HALL_11_WINDOWS_LIMIT;
        } else {
            /* in range */
        }
        hall_activity = (hall_activity < 0) ? -hall_activity : hall_activity;
        if (hall_activity > FAST_ROLL_HALL_STEPS) {
            state->tipped = true; /* never true after the +-25 clamp; kept as in the stock code */
        }
        if (in->motor_voltage_q > FAST_ROLL_VOLTAGE_Q) {
            state->tipped = true; /* positive Vq only, as in the stock code */
        }
    }
}

static bool within_40_deg(float angle_deg)
{
    return (angle_deg <= PITCH_CUTOFF_DEG) && (angle_deg >= -PITCH_CUTOFF_DEG);
}

void euc_balance_step_cutoffs(struct euc_balance_state *state, const struct euc_balance_in *in,
                              struct euc_balance_out *out)
{
    roll_check(state, in);

    if (within_40_deg(state->attitude.pitch_deg) && !state->tipped) {
        state->pitch_over_ticks = 0U;
    } else if (state->pitch_over_ticks < PITCH_CUTOFF_TICKS) {
        state->pitch_over_ticks++;
    } else {
        state->motor_disabled = true;
        state->fallen = true;
    }

    if (within_40_deg(state->error_deg) && !state->tipped) {
        state->error_over_ticks = 0U;
    } else if (state->error_over_ticks < ERROR_CUTOFF_TICKS) {
        state->error_over_ticks++;
    } else {
        if (state->error_over_ticks == ERROR_CUTOFF_TICKS) {
            state->error_over_ticks = ERROR_CUTOFF_TICKS + 1U;
            balance_request_beep(out, ERROR_CUTOFF_BEEP_TICKS);
        }
        state->motor_disabled = true;
    }

    if ((in->motor_voltage_q > OVERLOAD_VOLTAGE_Q) || (in->motor_voltage_q < -OVERLOAD_VOLTAGE_Q)) {
        if (state->overload_ticks > OVERLOAD_TICKS) {
            state->motor_disabled = true;
        } else {
            state->overload_ticks++;
        }
    } else {
        state->overload_ticks = 0U;
    }
}

void euc_balance_step_startup(struct euc_balance_state *state)
{
    if (state->start_state >= 1) {
        return;
    }
    const int32_t tilt_x = state->attitude.tilt_centideg[0];
    const int32_t tilt_y = state->attitude.tilt_centideg[1];
    const int32_t tilt_z = state->attitude.tilt_centideg[2];
    const bool roughly_level = (tilt_x >= -START_OUTER_CENTIDEG) && (tilt_x <= START_OUTER_CENTIDEG) &&
                               (tilt_y >= -START_OUTER_CENTIDEG) && (tilt_y <= START_OUTER_CENTIDEG);
    if (roughly_level && (state->start_state == 0)) {
        if ((tilt_x > -START_X_CENTIDEG) && (tilt_x <= START_X_CENTIDEG) && (tilt_y > -START_Y_CENTIDEG) &&
            (tilt_y < START_Y_CENTIDEG) && (tilt_z > START_Z_CENTIDEG)) {
            state->start_state = 1;
            state->command_limit = 0.0F;
        }
    } else if (state->start_state < 0) {
        state->start_state++;
    } else {
        /* waiting for a level wheel */
    }
}

void euc_balance_step_recovery(struct euc_balance_state *state, const struct euc_balance_in *in,
                               struct euc_balance_out *out)
{
    const int32_t tilt_x = state->attitude.tilt_centideg[0];
    const int32_t tilt_y = state->attitude.tilt_centideg[1];
    const int32_t tilt_z = state->attitude.tilt_centideg[2];
    const bool level = (tilt_x >= RECOVERY_X_MIN_CENTIDEG) && (tilt_x <= RECOVERY_X_MAX_CENTIDEG) &&
                       (tilt_y > -START_Y_CENTIDEG) && (tilt_y < START_Y_CENTIDEG) &&
                       (tilt_z > START_Z_CENTIDEG);
    if (!level) {
        state->recovery_ticks = 0;
        return;
    }
    const int32_t speed_dm_per_s = state->speed_dm_per_s;
    if ((speed_dm_per_s >= -RECOVERY_SPEED_DM_PER_S) && (speed_dm_per_s <= RECOVERY_SPEED_DM_PER_S)) {
        state->recovery_ticks++;
    }
    if (state->recovery_ticks > RECOVERY_TICKS) {
        state->tipped = false;
        state->handle_presses = 0U;
        state->fallen = false;
        state->recovery_ticks = RECOVERY_TICKS;
        if (!in->hall_fault && !in->hard_inhibit && !state->numeric_fault) {
            state->motor_disabled = false;
            balance_request_beep(out, RECOVERY_BEEP_TICKS);
        } else {
            state->motor_disabled = true;
        }
        state->command_limit = 1.0F;
        state->restart_offset_deg = state->error_deg; /* the pedals start from where they are */
    }
}
