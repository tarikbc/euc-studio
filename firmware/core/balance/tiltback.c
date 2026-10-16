/*
 * Offsets added to the error angle each tick, in the stock order. A positive offset at forward speed makes
 * the balance hold the frame nose-up by that angle: the pedals tilt back and the rider slows down.
 *
 *   battery / temperature   voltage outside 99.20..137.60 V or board above about 80 degC
 *   speed                   above the tiltback speed (setting 'W' 'Y', or 200 = off)
 *   PWM                     PWM alarm active and PWM tiltback on ('e'), clamped at +-12 deg
 *   pedal trim              setting 'W' 'U': slews toward (d - 4) x 1.2 deg
 *   restart offset          set at recovery, fades to zero
 *
 * The filters run in double precision and store floats, as in the stock firmware.
 */
#include <stdint.h>

#include "balance_internal.h"
#include "euc/balance/balance.h"
#include "euc/math/float_ops.h"

/*
 * Battery and temperature tiltback window. Inside the outer window the offset holds; inside the inner window
 * it releases. Voltages in 0.01 V, temperature in the MPU6050 counts of telemetry frame 00.
 */
#define BATTERY_OUTER_MIN_CV          9920     /* 99.20 V */
#define BATTERY_OUTER_MAX_CV          13760    /* 137.60 V */
#define BATTERY_INNER_MIN_CV          10021    /* 100.21 V */
#define BATTERY_INNER_MAX_CV          13759    /* 137.59 V */
#define TEMPERATURE_OUTER_MAX_MPU6050 14500.0F /* about 80.2 degC */
#define TEMPERATURE_INNER_MAX_MPU6050 14400.0F /* about 79.9 degC */

#define PWM_TILTBACK_LIMIT_DEG 12.0F

/* Speed tiltback threshold, m/s, from the km/h setting. */
static float speed_threshold_m_per_s(uint16_t tiltback_speed_kmh)
{
    return euc_f32_from_f64((double)tiltback_speed_kmh / 3.6);
}

static float battery_tiltback(float offset_deg, int32_t voltage_cv, float temperature, int16_t speed_cm_per_s)
{
    const bool in_outer_window = (voltage_cv >= BATTERY_OUTER_MIN_CV) &&
                                 (voltage_cv <= BATTERY_OUTER_MAX_CV) &&
                                 (temperature <= TEMPERATURE_OUTER_MAX_MPU6050);
    if (in_outer_window) {
        const bool in_inner_window = (voltage_cv >= BATTERY_INNER_MIN_CV) &&
                                     (voltage_cv <= BATTERY_INNER_MAX_CV) &&
                                     (temperature < TEMPERATURE_INNER_MAX_MPU6050);
        /* Release with a 20 s time constant inside the inner window, hold in the gap between the windows. */
        return in_inner_window ? euc_f32_from_f64((double)offset_deg * 0.9999) : offset_deg;
    }
    /* Ramp 2.38 deg/s nose-up (forward), 1.98 deg/s the other way when rolling backward. */
    if (speed_cm_per_s < 0) {
        return euc_f32_from_f64(((double)offset_deg * 0.9999) - 0.004);
    }
    return euc_f32_from_f64(((double)offset_deg * 0.9999) + 0.0048);
}

static float speed_tiltback(float offset_deg, float speed_m_per_s, float threshold_m_per_s)
{
    /* 10 deg per m/s of overspeed in steady state, time constant 4 s; releases with 2 s. */
    if (speed_m_per_s < -threshold_m_per_s) {
        const double overspeed = (double)euc_f32_abs(euc_f32_add(speed_m_per_s, threshold_m_per_s));
        return euc_f32_from_f64(((double)offset_deg * 0.9995) - ((overspeed * 10.0) * 0.0005));
    }
    if (threshold_m_per_s < speed_m_per_s) {
        const double overspeed = (double)euc_f32_abs(euc_f32_sub(speed_m_per_s, threshold_m_per_s));
        return euc_f32_from_f64(((double)offset_deg * 0.9995) + ((overspeed * 10.0) * 0.0005));
    }
    return euc_f32_from_f64((double)offset_deg * 0.999);
}

static float pwm_tiltback(float offset_deg, bool active, int16_t speed_cm_per_s)
{
    float offset = offset_deg;
    if (active) {
        offset = (speed_cm_per_s < 0) ? euc_f32_from_f64(((double)offset * 0.9999) - 0.005)
                                      : euc_f32_from_f64(((double)offset * 0.9999) + 0.0055);
    } else {
        offset = euc_f32_from_f64((double)offset * 0.999);
    }
    if (offset > PWM_TILTBACK_LIMIT_DEG) {
        offset = PWM_TILTBACK_LIMIT_DEG;
    } else if (offset < -PWM_TILTBACK_LIMIT_DEG) {
        offset = -PWM_TILTBACK_LIMIT_DEG;
    } else {
        /* within the limit */
    }
    return offset;
}

/* Slews 0.005 deg per tick (2.48 deg/s) toward (setting - 4) x 1.2 deg, without overshoot. */
static float pedal_trim(float offset_deg, uint8_t setting)
{
    const float target = euc_f32_from_f64(((double)euc_f32_from_f64((double)setting - 4.0) * 12.0) / 10.0);
    float offset = offset_deg;
    if (offset < target) {
        offset = euc_f32_from_f64((double)offset + 0.005);
        if (target < offset) {
            offset = target;
        }
    }
    if (target < offset) {
        offset = euc_f32_from_f64((double)offset - 0.005);
        if (offset < target) {
            offset = target;
        }
    }
    return offset;
}

void euc_balance_step_tiltbacks(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    const float threshold_m_per_s = speed_threshold_m_per_s(in->settings.tiltback_speed_kmh);

    state->tiltback_battery_deg = battery_tiltback(state->tiltback_battery_deg, in->battery_voltage_cv,
                                                   in->board_temperature_mpu6050, state->speed_cm_per_s);
    state->error_deg = euc_f32_add(state->error_deg, state->tiltback_battery_deg);

    state->tiltback_speed_deg =
        speed_tiltback(state->tiltback_speed_deg, state->speed_m_per_s, threshold_m_per_s);
    state->error_deg = euc_f32_add(state->error_deg, state->tiltback_speed_deg);

    const bool pwm_active = in->settings.pwm_tiltback_enabled && in->pwm_alarm_active;
    state->tiltback_pwm_deg = pwm_tiltback(state->tiltback_pwm_deg, pwm_active, state->speed_cm_per_s);
    state->error_deg = euc_f32_add(state->error_deg, state->tiltback_pwm_deg);

    state->pedal_trim_deg = pedal_trim(state->pedal_trim_deg, in->settings.pedal_trim);
    state->error_deg = euc_f32_add(state->error_deg, state->pedal_trim_deg);

    state->restart_offset_deg = balance_fade_offset(state->restart_offset_deg);
    state->error_deg = euc_f32_sub(state->error_deg, state->restart_offset_deg);
}
