/*
 * Offsets added to the error angle each tick, in the stock order. A positive offset at forward speed makes
 * the balance hold the frame nose-up by that angle: the pedals tilt back and the rider slows down.
 *
 *   battery / temperature   voltage outside 99.20..137.60 V or board above about 80 degC; stock ramps it
 *                           without a limit (about 48 deg in the end); clean caps it at 8 deg and limits
 *                           the speed instead (quirk BATTERY_TILTBACK_UNBOUNDED)
 *   speed                   above the tiltback speed (setting 'W' 'Y', or 200 = off). In limp mode and,
 *                           in clean, with the battery or temperature out of its window: at most
 *                           EUC_BALANCE_LIMP_SPEED_KMH, growing at most 2.5 deg/s, at most 12 deg
 *   PWM                     PWM alarm active and PWM tiltback on ('e'), clamped at +-12 deg; clean also
 *                           whenever the voltage vector |Vd, Vq| passes 75 % (quirk PWM_TILTBACK_VQ_ONLY)
 *   pedal trim              setting 'W' 'U': slews toward (d - 4) x 1.2 deg
 *   restart offset          set at recovery, fades to zero
 *
 * The filters run in double precision and store floats, as in the stock firmware.
 */
#include <stdint.h>

#include "balance_internal.h"
#include "euc/balance/balance.h"
#include "euc/math/float_ops.h"
#include "euc/quirks.h"

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

/* Clean headroom tiltback: the voltage vector above 75 % of full modulation (23625 of 31500), squared. */
#define HEADROOM_START_SQ 558140625U

/* Speed limit (limp mode; clean low battery or hot board): the speed tiltback grows at most 2.5 deg/s
 * (0.00504 deg per tick), however fast the wheel goes when the limit starts, and stops at 12 deg. */
#define LIMP_TILTBACK_STEP_DEG 0.00504F
#define LIMP_TILTBACK_MAX_DEG  12.0F

/* Clean: the battery / temperature tiltback stops at 8 deg; the speed limit does the rest. */
#define BATTERY_TILTBACK_MAX_DEG 8.0F

enum battery_window { BATTERY_INNER, BATTERY_GAP, BATTERY_OUTSIDE };

/* Speed tiltback threshold, m/s, from the km/h setting. */
static float speed_threshold_m_per_s(uint16_t tiltback_speed_kmh)
{
    return euc_f32_from_f64((double)tiltback_speed_kmh / 3.6);
}

static enum battery_window battery_window(int32_t voltage_cv, float temperature)
{
    const bool in_outer_window = (voltage_cv >= BATTERY_OUTER_MIN_CV) &&
                                 (voltage_cv <= BATTERY_OUTER_MAX_CV) &&
                                 (temperature <= TEMPERATURE_OUTER_MAX_MPU6050);
    if (!in_outer_window) {
        return BATTERY_OUTSIDE;
    }
    const bool in_inner_window = (voltage_cv >= BATTERY_INNER_MIN_CV) &&
                                 (voltage_cv <= BATTERY_INNER_MAX_CV) &&
                                 (temperature < TEMPERATURE_INNER_MAX_MPU6050);
    return in_inner_window ? BATTERY_INNER : BATTERY_GAP;
}

static float battery_tiltback(float offset_deg, enum battery_window window, int16_t speed_cm_per_s)
{
    if (window != BATTERY_OUTSIDE) {
        /* Release with a 20 s time constant inside the inner window, hold in the gap between the windows. */
        return (window == BATTERY_INNER) ? euc_f32_from_f64((double)offset_deg * 0.9999) : offset_deg;
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

/* `next`, with its magnitude growing at most `step` beyond `previous` (a release is not limited). */
static float limit_growth(float previous, float next, float step)
{
    if ((next > previous) && (next > 0.0F)) {
        const float most = euc_f32_add((previous > 0.0F) ? previous : 0.0F, step);
        return (next > most) ? most : next;
    }
    if ((next < previous) && (next < 0.0F)) {
        const float least = euc_f32_sub((previous < 0.0F) ? previous : 0.0F, step);
        return (next < least) ? least : next;
    }
    return next;
}

static float clamp_magnitude(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void euc_balance_step_tiltbacks(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    const enum battery_window window = battery_window(in->battery_voltage_cv, in->board_temperature_mpu6050);
    const bool battery_capped = !EUC_QUIRK(BATTERY_TILTBACK_UNBOUNDED);
    if (battery_capped) {
        if (window == BATTERY_OUTSIDE) {
            state->battery_limit = true;
        } else if (window == BATTERY_INNER) {
            state->battery_limit = false;
        } else {
            /* the gap between the windows: hold */
        }
    }
    const bool speed_limited = in->limp || state->battery_limit;

    uint16_t tiltback_speed_kmh = in->settings.tiltback_speed_kmh;
    if (speed_limited && (tiltback_speed_kmh > EUC_BALANCE_LIMP_SPEED_KMH)) {
        tiltback_speed_kmh = EUC_BALANCE_LIMP_SPEED_KMH; /* slow the rider down */
    }
    const float threshold_m_per_s = speed_threshold_m_per_s(tiltback_speed_kmh);

    float battery_offset = battery_tiltback(state->tiltback_battery_deg, window, state->speed_cm_per_s);
    if (battery_capped) {
        battery_offset = clamp_magnitude(battery_offset, BATTERY_TILTBACK_MAX_DEG);
    }
    state->tiltback_battery_deg = battery_offset;
    state->error_deg = euc_f32_add(state->error_deg, state->tiltback_battery_deg);

    float speed_offset = speed_tiltback(state->tiltback_speed_deg, state->speed_m_per_s, threshold_m_per_s);
    if (speed_limited) {
        speed_offset = limit_growth(state->tiltback_speed_deg, speed_offset, LIMP_TILTBACK_STEP_DEG);
        speed_offset = clamp_magnitude(speed_offset, LIMP_TILTBACK_MAX_DEG);
    }
    state->tiltback_speed_deg = speed_offset;
    state->error_deg = euc_f32_add(state->error_deg, state->tiltback_speed_deg);

    bool pwm_active = in->settings.pwm_tiltback_enabled && in->pwm_alarm_active;
    if (!EUC_QUIRK(PWM_TILTBACK_VQ_ONLY) && (in->motor_voltage_sq > HEADROOM_START_SQ)) {
        pwm_active = true; /* clean: the voltage headroom is low, whatever the settings say */
    }
    state->tiltback_pwm_deg = pwm_tiltback(state->tiltback_pwm_deg, pwm_active, state->speed_cm_per_s);
    state->error_deg = euc_f32_add(state->error_deg, state->tiltback_pwm_deg);

    state->pedal_trim_deg = pedal_trim(state->pedal_trim_deg, in->settings.pedal_trim);
    state->error_deg = euc_f32_add(state->error_deg, state->pedal_trim_deg);

    state->restart_offset_deg = balance_fade_offset(state->restart_offset_deg);
    state->error_deg = euc_f32_sub(state->error_deg, state->restart_offset_deg);
}
