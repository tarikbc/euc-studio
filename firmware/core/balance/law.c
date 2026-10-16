/*
 * Balance law and output limits.
 *
 * Voltage mode (drive mode 0, default):
 *   u = Kp(v) x error + Kc(v) x current / 10000 + 0.1 x Kd(v) x pitch rate + Kv x hall speed / 1000
 * Current mode (drive mode 1):
 *   u = clamp(Kp(v) x error, +-1050) + 0.1 x Kd(v) x pitch rate + 160 x (error - previous error)
 * with Kp = Kp_base + |v| / 100, Kd = Kd_base + 32 |v| / 100, Kc = Kc_base - Kc_slope |v| / 1000 (v in cm/s,
 * clamped to +-25 m/s). The command is then scaled to a 128.0 V battery: command = u x 1280 / V (0.1 V,
 * clamped 106.0..134.0 V).
 *
 * Output: slew limit 120 per tick, then the soft-start limit (+1 per tick up to 1050), then published through
 * the actuator boundary (finite, within the limit). While soft-disabled the law does not run and the command
 * ramps to zero by 2 per tick. Clean (NAN_FAIL_OPEN off): a non-finite value at the law, ramp-down or output
 * stage stops the drive (balance_numeric_fault).
 */
#include <float.h>
#include <stdint.h>

#include "balance_internal.h"
#include "euc/balance/balance.h"
#include "euc/math/float_ops.h"

#define LAW_SPEED_LIMIT_CM_PER_S   2500
#define LAW_VOLTAGE_MIN_DV         1060
#define LAW_VOLTAGE_MAX_DV         1340
#define LAW_REFERENCE_X10_DV       1280.0F /* 128.0 V */
#define LAW_ERROR_RATE_GAIN        160.0F
#define COMMAND_SLEW_PER_TICK      120.0F
#define COMMAND_RAMP_DOWN_PER_TICK 2.0F
#define COMMAND_ZERO_BAND          3.0F

/*
 * The stock law adds Ki x I, with Ki = 120 and an integral I that nothing ever writes: always +0.0. Kept for
 * the compat profile because adding +0.0 turns a -0.0 sum into +0.0.
 */
#define LAW_INTEGRAL_GAIN 120.0F
#define LAW_INTEGRAL_TERM 0.0F

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void euc_balance_step_law(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    const struct euc_balance_gains *gains = &state->gains;

    int32_t speed_cm_per_s = state->speed_cm_per_s;
    if (speed_cm_per_s > LAW_SPEED_LIMIT_CM_PER_S) {
        speed_cm_per_s = LAW_SPEED_LIMIT_CM_PER_S;
    } else if (speed_cm_per_s < -LAW_SPEED_LIMIT_CM_PER_S) {
        speed_cm_per_s = -LAW_SPEED_LIMIT_CM_PER_S;
    } else {
        /* in range */
    }
    state->speed_cm_per_s = (int16_t)speed_cm_per_s;
    const int32_t speed_magnitude = abs_i32(speed_cm_per_s);

    const int16_t kc = (int16_t)(gains->kc_base - ((gains->kc_slope * speed_magnitude) / 1000));
    const float speed_feed_forward =
        euc_f32_div(euc_f32_mul((float)in->hall_period_speed, gains->kv), 1000.0F);
    const int32_t kd_speed_term = (speed_magnitude * 32) / 100;
    const int16_t kd = (int16_t)(int32_t)euc_f32_add((float)kd_speed_term, gains->kd_base);
    const int32_t kp_speed_term = speed_magnitude / 100;
    const float kp = euc_f32_add((float)kp_speed_term, gains->kp_base);

    int32_t voltage_dv = (int16_t)(in->battery_voltage_cv / 10);
    if (voltage_dv < LAW_VOLTAGE_MIN_DV) {
        voltage_dv = LAW_VOLTAGE_MIN_DV;
    } else if (voltage_dv > LAW_VOLTAGE_MAX_DV) {
        voltage_dv = LAW_VOLTAGE_MAX_DV;
    } else {
        /* in range */
    }

    const float error = state->error_deg;
    const float error_change = euc_f32_sub(error, state->error_prev_deg);
    state->error_prev_deg = error;
    const float error_change_term = euc_f32_mul(error_change, LAW_ERROR_RATE_GAIN);
    const double rate_term = ((double)kd * 0.1) * (double)state->attitude.pitch_rate_lpf_rad_per_s;
    const double integral_term = (double)euc_f32_mul(LAW_INTEGRAL_GAIN, LAW_INTEGRAL_TERM);

    float proportional = euc_f32_mul(kp, error);
    double u;
    if (in->settings.drive_mode == 0U) {
        const int32_t current_feedback = ((int32_t)kc * in->motor_current_lpf_ma) / 10000;
        const float current_term = (float)current_feedback;
        u = (double)euc_f32_add(proportional, current_term) + rate_term;
        u = u + integral_term;
        u = u + (double)speed_feed_forward;
    } else {
        if (proportional > EUC_BALANCE_COMMAND_LIMIT) {
            proportional = EUC_BALANCE_COMMAND_LIMIT;
        } else if (proportional < -EUC_BALANCE_COMMAND_LIMIT) {
            proportional = -EUC_BALANCE_COMMAND_LIMIT;
        } else {
            /* in range */
        }
        u = (double)proportional + rate_term;
        u = u + integral_term;
        u = u + (double)error_change_term;
    }
    const float command = euc_f32_from_f64(u);
    state->command = euc_f32_div(euc_f32_mul(command, LAW_REFERENCE_X10_DV), (float)voltage_dv);
    if (!EUC_QUIRK(NAN_FAIL_OPEN) && !euc_f32_is_finite(state->command)) {
        balance_numeric_fault(state);
    }
}

void euc_balance_step_ramp_down(struct euc_balance_state *state)
{
    if (!EUC_QUIRK(NAN_FAIL_OPEN) && !euc_f32_is_finite(state->command)) {
        balance_numeric_fault(state);
    }
    float command = state->command;
    if (command > EUC_BALANCE_COMMAND_LIMIT) {
        command = euc_f32_sub(EUC_BALANCE_COMMAND_LIMIT, COMMAND_RAMP_DOWN_PER_TICK);
    } else if (command < -EUC_BALANCE_COMMAND_LIMIT) {
        command = euc_f32_add(-EUC_BALANCE_COMMAND_LIMIT, COMMAND_RAMP_DOWN_PER_TICK);
    } else if (command >= FLT_MIN) {
        command = euc_f32_sub(command, COMMAND_RAMP_DOWN_PER_TICK);
    } else if (command <= -FLT_MIN) {
        command = euc_f32_add(command, COMMAND_RAMP_DOWN_PER_TICK);
    } else {
        /* zero or denormal: unchanged */
    }
    if ((command >= -COMMAND_ZERO_BAND) && (command <= COMMAND_ZERO_BAND)) {
        command = 0.0F;
    }
    state->command = command;
}

/*
 * Actuator boundary, both profiles. A deliberate difference from the stock firmware, also in compat: the
 * command the current loop reads is always finite and within +-limit, and a non-finite command becomes 0.
 * (Comparisons are false for NaN, so a plain `if (limit < command)` clamp lets a NaN through, and
 * `if (!(command <= limit)) command = limit` would turn it into full power.) Finite values within the limit,
 * the only ones the stock code produces in normal operation, pass unchanged, so lock-step is not affected.
 */
static float actuator_command(float command, float limit)
{
    if (!euc_f32_is_finite(command) || !euc_f32_is_finite(limit) || !(limit >= 0.0F)) {
        return 0.0F;
    }
    if (!(command <= limit)) {
        return limit;
    }
    if (!(command >= -limit)) {
        return -limit;
    }
    return command;
}

void euc_balance_step_output(struct euc_balance_state *state)
{
    if (!EUC_QUIRK(NAN_FAIL_OPEN) && !euc_f32_is_finite(state->command)) {
        balance_numeric_fault(state);
    }
    float command = state->command;
    const float previous = state->command_prev;
    if (euc_f32_sub(previous, command) > COMMAND_SLEW_PER_TICK) {
        command = euc_f32_sub(previous, COMMAND_SLEW_PER_TICK);
    }
    if (euc_f32_sub(command, previous) > COMMAND_SLEW_PER_TICK) {
        command = euc_f32_add(previous, COMMAND_SLEW_PER_TICK);
    }

    /* Soft start: the limit grows while the handle has not cut the motor. Stock detail: at exactly 1050 it
     * still adds 1 (1051), and the next tick sets it back to 1050. */
    if (state->handle_presses < 2U) {
        state->command_limit = (state->command_limit > EUC_BALANCE_COMMAND_LIMIT)
                                   ? EUC_BALANCE_COMMAND_LIMIT
                                   : euc_f32_add(state->command_limit, 1.0F);
    }
    const float limit = state->command_limit;
    if (limit < command) {
        command = limit;
    }
    if (command < -limit) {
        command = -limit;
    }
    state->command = command;
    state->command_prev = command;
    state->motor_command = actuator_command(command, limit);
}
