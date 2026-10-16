/*
 * Motor drive (see euc/motor/motor.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/fixed.h"
#include "euc/math/float_ops.h"
#include "euc/motor/motor.h"
#include "euc/quirks.h"

/* Iq taps: 32768 counts = 437.5 A. */
#define IQ_MA_PER_FULL_SCALE  437500
#define IQ_01A_PER_FULL_SCALE 4375
#define Q15_DIVISOR           32768
#define IQ_LPF_OLD_WEIGHT     9999
#define IQ_LPF_DIVISOR        10000
#define IQ_LPF_LIMIT_MA       90000

/* d-axis measurement gain k(speed) = clamp(2 |v| / 10, 0.3, 1.0), v in 0.1 m/s. */
#define ID_GAIN_MIN     3
#define ID_GAIN_MAX     10
#define ID_GAIN_DIVISOR 10

/* Command to reference: mode 1 Iq* = 20 x command, mode 0 Vq = 30 x command. */
#define CURRENT_MODE_IQ_PER_COMMAND 20.0
#define VOLTAGE_MODE_VQ_PER_COMMAND 30.0

/* Voltage mode: above +-280 A (Iq in 0.1 A) the command backs off by the excess in Q1.15 counts, taking 75
 * counts per ampere (32768 / 437.5 = 74.9). */
#define VOLTAGE_MODE_LIMIT_01A     2800
#define VOLTAGE_MODE_BACKOFF_Q15   21000
#define VOLTAGE_MODE_SLEW_PER_STEP 60.0F

/* Stop brake: disabled and |speed| <= 0.4 m/s. */
#define BRAKE_SPEED_DM_PER_S 4

/* q integral limit at balance rate: 160 x |v| volts after the /8192 integral divisor. */
#define Q_INTEGRAL_SPEED_LIMIT_DM_PER_S 200
#define Q_INTEGRAL_PER_DM_PER_S         0x140000

#define BALANCE_PWM_PERCENT_DIVISOR 315 /* 31500 = 100 % */

struct euc_motor_timing euc_motor_timing_for_mode(uint8_t drive_mode)
{
    /* Voltage mode: 10 kHz. The stock dead time there is one count (13.9 ns), unsafe unless the gate driver
     * adds its own; clean uses current mode's 29 counts (403 ns) (quirk MODE0_DEAD_TIME). */
    static const struct euc_motor_timing voltage_mode = {10000U, 3600U, 14400U, 24941U, 1U};
    static const struct euc_motor_timing current_mode = {12000U, 3000U, 12000U, 20784U, 29U};

    if (drive_mode != EUC_MOTOR_VOLTAGE_MODE) {
        return current_mode;
    }
    struct euc_motor_timing timing = voltage_mode;
    if (!EUC_QUIRK(MODE0_DEAD_TIME)) {
        timing.dead_time_counts = current_mode.dead_time_counts;
    }
    return timing;
}

void euc_motor_init(struct euc_motor_state *state, uint8_t drive_mode, const struct euc_motor_config *config)
{
    /* d axis: Kp 2.15, Ki 0.0146, Kd 0.244, integral +-32767 after its divisor. */
    static const struct euc_pid pid_d_power_on = {
        .kp = 2200,
        .kp_div = 1024U,
        .ki = 120,
        .ki_div = 8192U,
        .kd = 2000,
        .kd_div = 8192U,
        .out_min = INT16_MIN,
        .out_max = INT16_MAX,
        .integral_min = -0x10000000,
        .integral_max = 0x0FFFE000,
        .integral = 0,
        .previous_error = 0,
    };
    /* q axis: Kp 2.93, Ki 0.00366, Kd 0.610, output +-32000; the integral limit follows the speed later. */
    static const struct euc_pid pid_q_power_on = {
        .kp = 3000,
        .kp_div = 1024U,
        .ki = 30,
        .ki_div = 8192U,
        .kd = 5000,
        .kd_div = 8192U,
        .out_min = -32000,
        .out_max = 32000,
        .integral_min = -0x0FA00000,
        .integral_max = 0x0FA00000,
        .integral = 0,
        .previous_error = 0,
    };

    (void)memset(state, 0, sizeof(*state));
    state->drive_mode =
        (uint8_t)((drive_mode == EUC_MOTOR_VOLTAGE_MODE) ? EUC_MOTOR_VOLTAGE_MODE : EUC_MOTOR_CURRENT_MODE);
    state->timing = euc_motor_timing_for_mode(state->drive_mode);
    state->config = *config;
    euc_current_init(&state->current);
    euc_hall_init(&state->hall);
    state->pid_d = pid_d_power_on;
    state->pid_q = pid_q_power_on;
    state->enable_pending = true;
}

void euc_motor_set_command(struct euc_motor_state *state, float command)
{
    float checked = euc_f32_is_finite(command) ? command : 0.0F;
    if (checked > EUC_MOTOR_COMMAND_LIMIT) {
        checked = EUC_MOTOR_COMMAND_LIMIT;
    } else if (checked < -EUC_MOTOR_COMMAND_LIMIT) {
        checked = -EUC_MOTOR_COMMAND_LIMIT;
    } else {
        /* within the limit */
    }
    state->command = checked;
}

static int32_t magnitude_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void euc_motor_rtc_step(struct euc_motor_state *state, int16_t speed_dm_per_s)
{
    const int32_t speed =
        clamp_i32(speed_dm_per_s, -Q_INTEGRAL_SPEED_LIMIT_DM_PER_S, Q_INTEGRAL_SPEED_LIMIT_DM_PER_S);
    const int32_t limit = magnitude_i32(speed) * Q_INTEGRAL_PER_DM_PER_S;
    state->pid_q.integral_min = -limit;
    state->pid_q.integral_max = limit;
}

void euc_motor_hall_step(struct euc_motor_state *state, uint8_t lines, struct euc_hall_out *out)
{
    euc_hall_step(&state->hall, lines, state->timing.pwm_frequency_hz, out);
}

/* Iq in mA. The stock product is 32 bits wide and wraps above 4908 counts (65.5 A) (quirk IQ_MA_WRAP). */
static int32_t iq_milliamps(int16_t iq_q15)
{
    int64_t product = (int64_t)iq_q15 * IQ_MA_PER_FULL_SCALE;
    if (EUC_QUIRK(IQ_MA_WRAP)) {
        const uint32_t low_word = (uint32_t)(uint64_t)product; /* modulo 2^32 */
        product = (int64_t)low_word;
        if (product > INT32_MAX) {
            product -= INT64_C(4294967296);
        }
    }
    return (int32_t)(product / Q15_DIVISOR);
}

static void update_current_taps(struct euc_motor_state *state)
{
    const int16_t iq = state->i_qd.q_q15;

    const int32_t filtered = ((state->iq_lpf_ma * IQ_LPF_OLD_WEIGHT) + iq_milliamps(iq)) / IQ_LPF_DIVISOR;
    state->iq_lpf_ma = clamp_i32(filtered, -IQ_LPF_LIMIT_MA, IQ_LPF_LIMIT_MA);

    state->iq_01a = ((int32_t)iq * IQ_01A_PER_FULL_SCALE) / Q15_DIVISOR;
    if (magnitude_i32(state->iq_peak_01a) < magnitude_i32(state->iq_01a)) {
        state->iq_peak_01a = (int16_t)state->iq_01a;
    }
}

/* Voltage mode: back off above +-280 A, slew +-60 per step, clamp to +-1050. */
static float voltage_mode_limit(struct euc_motor_state *state)
{
    float command = state->command;
    const int32_t excess_q15 = magnitude_i32(state->i_qd.q_q15) - VOLTAGE_MODE_BACKOFF_Q15;
    const float excess = (float)excess_q15;
    if (state->iq_01a > VOLTAGE_MODE_LIMIT_01A) {
        command = euc_f32_sub(command, excess);
    }
    if (state->iq_01a < -VOLTAGE_MODE_LIMIT_01A) {
        command = euc_f32_add(command, excess);
    }

    const float previous = state->command_previous;
    if (euc_f32_sub(command, previous) > VOLTAGE_MODE_SLEW_PER_STEP) {
        command = euc_f32_add(previous, VOLTAGE_MODE_SLEW_PER_STEP);
    }
    if (euc_f32_sub(command, previous) < -VOLTAGE_MODE_SLEW_PER_STEP) {
        command = euc_f32_sub(previous, VOLTAGE_MODE_SLEW_PER_STEP);
    }
    if (command > EUC_MOTOR_COMMAND_LIMIT) {
        command = EUC_MOTOR_COMMAND_LIMIT;
    } else if (command < -EUC_MOTOR_COMMAND_LIMIT) {
        command = -EUC_MOTOR_COMMAND_LIMIT;
    } else {
        /* within the limit */
    }
    state->command_previous = command;
    return command;
}

/* command x factor, truncated toward zero. Exact in double for every command within +-1050. */
static int16_t scaled_command(float command, double factor)
{
    const double product = (double)command * factor;
    return (int16_t)product;
}

static int16_t regulate(struct euc_motor_state *state, bool disabled)
{
    /* The integrals stay at zero while the stop brake held on the previous step. */
    const bool clear_integral = disabled && (state->vq_active == 0);
    int16_t vq;
    if (state->drive_mode == EUC_MOTOR_VOLTAGE_MODE) {
        state->command = voltage_mode_limit(state);
        state->v_qd.d_q15 = euc_pid_step(&state->pid_d, 0, state->i_qd.d_q15, clear_integral);
        vq = scaled_command(state->command, VOLTAGE_MODE_VQ_PER_COMMAND);
    } else {
        state->v_qd.d_q15 = euc_pid_step(&state->pid_d, 0, state->i_qd.d_q15, clear_integral);
        const int16_t iq_reference = scaled_command(state->command, CURRENT_MODE_IQ_PER_COMMAND);
        vq = euc_pid_step(&state->pid_q, iq_reference, state->i_qd.q_q15, clear_integral);
    }
    state->v_qd.q_q15 = vq;
    return vq;
}

static void stop_brake(struct euc_motor_state *state, int16_t vq, const struct euc_motor_foc_in *in)
{
    state->vq_active = 0;
    if (in->drive_disabled) {
        if ((in->speed_dm_per_s >= -BRAKE_SPEED_DM_PER_S) && (in->speed_dm_per_s <= BRAKE_SPEED_DM_PER_S)) {
            state->brake_latched = true;
        } else if (!state->brake_latched) {
            state->vq_active = vq;
        } else {
            /* latched: keep braking until the drive is enabled again */
        }
    } else {
        state->brake_latched = false;
        state->vq_active = vq;
    }
}

static void gate_outputs(struct euc_motor_state *state, const struct euc_motor_foc_in *in,
                         struct euc_motor_foc_out *out)
{
    if (state->current.overcurrent_trip) {
        state->hard_cut = true;
    }
    if (state->hall.fault || state->hard_cut) {
        state->outputs_enabled = false;
        state->duty = (struct euc_foc_duty){0U, 0U, 0U};
    } else {
        if (state->enable_pending && state->hall.standstill_seen) {
            state->enable_pending = false;
            state->outputs_enabled = true;
            out->start_balance = true;
        }
        if (in->drive_disabled && (state->vq_active == 0)) {
            state->duty = (struct euc_foc_duty){0U, 0U, 0U}; /* all low sides on: short-circuit brake */
        }
    }
    out->duty = state->duty;
    out->outputs_enabled = state->outputs_enabled;
}

void euc_motor_foc_step(struct euc_motor_state *state, const struct euc_motor_foc_in *in,
                        struct euc_motor_foc_out *out)
{
    (void)memset(out, 0, sizeof(*out));
    if (in->hard_cut_request) {
        state->hard_cut = true;
    }

    struct euc_current_out phases;
    euc_current_read(&state->current, &state->config.current, in->sample, &phases);
    if (phases.start_check_failed) {
        state->hard_cut = true;
        out->stop_drive = true;
        out->error_bits |= EUC_MOTOR_ERROR_CURRENT_SENSOR;
    }

    state->i_alpha_beta = euc_foc_clarke(phases.ia_q15, phases.ib_q15);
    state->trig = euc_foc_trig_from_angle(state->hall.angle);
    const struct euc_foc_qd measured = euc_foc_park(state->i_alpha_beta, state->trig);
    state->i_qd.q_q15 = measured.q_q15;
    update_current_taps(state);

    const int32_t gain = clamp_i32(2 * magnitude_i32(in->speed_dm_per_s), ID_GAIN_MIN, ID_GAIN_MAX);
    state->id_gain = (int16_t)gain;
    state->i_qd.d_q15 = (int16_t)((gain * measured.d_q15) / ID_GAIN_DIVISOR);

    const int16_t vq = regulate(state, in->drive_disabled);
    stop_brake(state, vq, in);

    state->v_qd = euc_foc_circle_limit(state->v_qd);
    state->v_alpha_beta = euc_foc_inverse_park(state->v_qd, state->trig);
    state->duty = euc_foc_svpwm(state->v_alpha_beta, state->timing.svpwm_t, state->timing.svpwm_t_sqrt3);
    gate_outputs(state, in, out);
}

int8_t euc_motor_pwm_percent(const struct euc_motor_state *state)
{
    return (int8_t)(-(int32_t)state->vq_active / BALANCE_PWM_PERCENT_DIVISOR);
}
