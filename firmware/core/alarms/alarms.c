/*
 * Beeper and alarms (see euc/alarms/alarms.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/alarms/alarms.h"
#include "euc/math/fixed.h"
#include "euc/math/float_ops.h"
#include "euc/quirks.h"

#define TIME_BASE_LIMIT  30000 /* the time base runs -29999..30001 */
#define TIME_BASE_OFFSET 30000
#define TIME_BASE_START  (-180)

#define VOLUME_STEP         30
#define VOLUME_COMPARE_MAX  270
#define VOLUME_COMPARE_SPAN 65536 /* the stock volume arithmetic is 16-bit unsigned */
#define PWM_ALARM_VOLUME    9U

#define BEEP_FAULT_TICKS   20U
#define BEEP_DOUBLE_TICKS  25U
#define BEEP_SHORT_TICKS   35U
#define BEEP_WARNING_TICKS 50U
#define BEEP_BMS_TICKS     500U

#define HARD_CUT_COUNT_LIMIT    30000U
#define HARD_CUT_REMINDER_TICKS 2500U

#define PWM_ALARM_MIN_PERCENT 50U
#define PWM_ALARM_MAX_PERCENT 90U
#define PWM_VQ_DIVISOR        30

/* 14500.0f: the board NTC alarm level in MPU6050 counts (about 80 degC). Compared as float bits like the
 * stock code, so +infinity and +NaN raise the alarm. */
#define TEMPERATURE_LIMIT_BITS 0x46629000U
#define FLOAT_SIGN_BIT         0x80000000U

#define UNDER_VOLTAGE_CV 10560 /* 105.60 V */
#define OVER_VOLTAGE_CV  13761 /* 137.61 V */

#define SPEED_LEVEL_1_DM_PER_S 84  /* 8.4 m/s */
#define SPEED_LEVEL_2_DM_PER_S 126 /* 12.6 m/s */

#define HORN_SECOND_BEEP_TICK 100U

void euc_alarm_init(struct euc_alarm_state *state)
{
    (void)memset(state, 0, sizeof(*state));
    state->time_base = TIME_BASE_START;
}

void euc_beeper_beep(struct euc_beeper *beeper, uint16_t ticks)
{
    beeper->ticks_left = ticks;
    beeper->compare = beeper->volume_compare;
}

uint16_t euc_beeper_volume_compare(uint8_t volume)
{
    if ((volume == 0U) && (!EUC_QUIRK(VOLUME_ZERO_MAX))) {
        return 1U;
    }
    int32_t compare = (((int32_t)volume - 1) * VOLUME_STEP) + 1;
    if (compare < 0) {
        compare += VOLUME_COMPARE_SPAN; /* volume 0 wraps to a large value */
    }
    if (compare > VOLUME_COMPARE_MAX) {
        compare = VOLUME_COMPARE_MAX;
    }
    return (compare == 0) ? 1U : (uint16_t)compare;
}

static void beeper_count(struct euc_beeper *beeper)
{
    if (beeper->ticks_left != 0U) {
        beeper->ticks_left--;
    } else {
        beeper->compare = 0U;
    }
}

static bool phase_is(uint32_t time_base, uint32_t period, uint32_t phase)
{
    return (time_base % period) == phase;
}

static void hard_cut_reminder(struct euc_alarm_state *state, const struct euc_alarm_in *in,
                              uint32_t time_base)
{
    if ((!in->hard_cut) || in->powering_off) {
        state->hard_cut_ticks = 0U;
        state->hard_cut_alarm = false;
        return;
    }
    if (state->hard_cut_ticks < HARD_CUT_COUNT_LIMIT) {
        state->hard_cut_ticks++;
    }
    state->hard_cut_alarm = true;
    /* Stock has a second reminder every 1000 ticks after this, behind a constant that never lets it run. */
    if (phase_is(time_base, 500U, 0U) && (state->hard_cut_ticks < HARD_CUT_REMINDER_TICKS)) {
        euc_beeper_beep(&state->beeper, BEEP_WARNING_TICKS);
    }
}

static void pwm_alarm(struct euc_alarm_state *state, const struct euc_alarm_in *in, uint32_t time_base,
                      struct euc_alarm_out *out)
{
    uint8_t percent = in->pwm_alarm_percent;
    if (percent > PWM_ALARM_MAX_PERCENT) {
        percent = PWM_ALARM_MAX_PERCENT;
    } else if (percent < PWM_ALARM_MIN_PERCENT) {
        percent = PWM_ALARM_MIN_PERCENT;
    } else {
        /* in range */
    }
    out->pwm_alarm_percent = percent;

    const int32_t threshold = (int32_t)percent * 10;
    const int32_t load = (int32_t)in->motor_voltage_q / PWM_VQ_DIVISOR;
    if ((load > threshold) || (load < -threshold)) {
        state->pwm_alarm = true;
        state->alerts |= EUC_ALERT_PWM_HIGH;
        state->beeper.volume_compare = euc_beeper_volume_compare(PWM_ALARM_VOLUME);
        if ((!state->hard_cut_alarm) && phase_is(time_base, 100U, 0U)) {
            euc_beeper_beep(&state->beeper, BEEP_SHORT_TICKS);
        }
    } else {
        state->alerts &= (uint8_t)~EUC_ALERT_PWM_HIGH;
        state->pwm_alarm = false;
    }
}

static void temperature_alarm(struct euc_alarm_state *state, const struct euc_alarm_in *in,
                              uint32_t time_base)
{
    const uint32_t bits = euc_f32_bits(in->board_temperature_mpu6050);
    if ((bits < FLOAT_SIGN_BIT) && (bits > TEMPERATURE_LIMIT_BITS)) {
        state->temperature_alarm = true;
        state->alerts |= EUC_ALERT_OVER_TEMPERATURE;
        if ((!state->hard_cut_alarm) && (!state->pwm_alarm) &&
            (phase_is(time_base, 1000U, 1U) || phase_is(time_base, 1000U, 100U))) {
            euc_beeper_beep(&state->beeper, BEEP_WARNING_TICKS);
        }
    } else {
        state->alerts &= (uint8_t)~EUC_ALERT_OVER_TEMPERATURE;
        state->temperature_alarm = false;
    }
}

static int32_t speed_magnitude(int16_t value)
{
    return (value < 0) ? -(int32_t)value : (int32_t)value;
}

static void voltage_alarm(struct euc_alarm_state *state, const struct euc_alarm_in *in, uint32_t time_base)
{
    const int16_t voltage_cv = sat_i32_to_i16(in->battery_voltage_cv);
    const bool quiet = state->hard_cut_alarm || state->pwm_alarm || state->temperature_alarm;
    const uint32_t phase = time_base % 1000U;
    bool beep = false;

    if (voltage_cv < UNDER_VOLTAGE_CV) {
        state->voltage_alarm = true;
        state->alerts |= EUC_ALERT_UNDER_VOLTAGE;
        /* More beeps the faster the wheel goes: up to 2 m/s, up to 4 m/s, above. */
        const int32_t speed = speed_magnitude(in->speed_dm_per_s);
        if (speed <= 20) {
            beep = (phase == 2U);
        } else if (speed <= 40) {
            beep = (phase == 1U) || (phase == 200U);
        } else {
            beep = (phase == 1U) || (phase == 200U) || (phase == 400U);
        }
    } else if (voltage_cv < OVER_VOLTAGE_CV) {
        state->alerts &= (uint8_t)~(EUC_ALERT_UNDER_VOLTAGE | EUC_ALERT_OVER_VOLTAGE);
        state->voltage_alarm = false;
    } else {
        state->voltage_alarm = true;
        state->alerts |= EUC_ALERT_OVER_VOLTAGE;
        beep = (phase == 1U) || (phase == 80U) || (phase == 160U);
    }
    if (beep && (!quiet)) {
        euc_beeper_beep(&state->beeper, BEEP_WARNING_TICKS);
    }
}

static void double_beep(struct euc_alarm_state *state, uint32_t time_base)
{
    if (!state->double_beep_pending) {
        return;
    }
    if (phase_is(time_base, 100U, 0U) && (state->double_beeps_done == 0U)) {
        euc_beeper_beep(&state->beeper, BEEP_DOUBLE_TICKS);
        state->double_beeps_done++;
    }
    if (phase_is(time_base, 100U, 60U) && (state->double_beeps_done == 1U)) {
        euc_beeper_beep(&state->beeper, BEEP_DOUBLE_TICKS);
        state->double_beeps_done++;
    }
    if (state->double_beeps_done > 1U) {
        state->double_beep_pending = false;
        state->double_beeps_done = 0U;
    }
}

static void speed_alarm(struct euc_alarm_state *state, const struct euc_alarm_in *in, uint32_t time_base)
{
    const int16_t speed = in->speed_dm_per_s;
    const bool over_level_2 = (speed >= SPEED_LEVEL_2_DM_PER_S) || (speed <= -SPEED_LEVEL_2_DM_PER_S);
    const bool over_level_1 = (speed >= SPEED_LEVEL_1_DM_PER_S) || (speed <= -SPEED_LEVEL_1_DM_PER_S);
    const bool level_2 = over_level_2 && (in->speed_alarm_mode <= 1U);
    const bool level_1 = (!level_2) && over_level_1 && (in->speed_alarm_mode == 0U);
    const bool quiet =
        state->hard_cut_alarm || state->pwm_alarm || state->temperature_alarm || state->voltage_alarm;
    const uint32_t phase = time_base % 500U;

    if (level_2) {
        state->speed_alarm = true;
        if ((!quiet) && ((phase == 0U) || (phase == 100U) || (phase == 200U))) {
            euc_beeper_beep(&state->beeper, BEEP_SHORT_TICKS);
        }
    } else if (level_1) {
        state->speed_alarm = true;
        if ((!quiet) && ((phase == 0U) || (phase == 100U))) {
            euc_beeper_beep(&state->beeper, BEEP_SHORT_TICKS);
        }
    } else {
        state->speed_alarm = false;
        double_beep(state, time_base);
    }
}

static void horn(struct euc_alarm_state *state, bool request)
{
    if (request) {
        state->horn_active = true;
    } else if (!state->horn_active) {
        state->horn_ticks = 0U;
        return;
    } else {
        /* sounding */
    }
    state->horn_ticks++;
    if (state->horn_ticks == 1U) {
        euc_beeper_beep(&state->beeper, BEEP_SHORT_TICKS);
    }
    if (state->horn_ticks == HORN_SECOND_BEEP_TICK) {
        euc_beeper_beep(&state->beeper, BEEP_SHORT_TICKS);
        state->horn_active = false;
    }
}

void euc_alarm_tick(struct euc_alarm_state *state, const struct euc_alarm_in *in, struct euc_alarm_out *out)
{
    (void)memset(out, 0, sizeof(*out));

    if (state->time_base > TIME_BASE_LIMIT) {
        state->time_base = -TIME_BASE_LIMIT;
    }
    state->time_base++;
    const int32_t shifted_time_base = (int32_t)state->time_base + TIME_BASE_OFFSET; /* 1..60001 */
    const uint32_t time_base = (uint32_t)shifted_time_base;

    state->alerts |= in->raise_alerts;
    if (in->handle_double_beep) {
        state->double_beep_pending = true;
    }

    beeper_count(&state->beeper);
    if (in->hall_fault || ((state->alerts & EUC_ALERT_HALL_SENSOR) != 0U)) {
        if (phase_is(time_base, 250U, 2U) || phase_is(time_base, 250U, 80U)) {
            euc_beeper_beep(&state->beeper, BEEP_FAULT_TICKS);
        }
    }
    if (in->main_loop_stalled && (phase_is(time_base, 500U, 3U) || phase_is(time_base, 500U, 63U) ||
                                  phase_is(time_base, 500U, 123U))) {
        euc_beeper_beep(&state->beeper, BEEP_FAULT_TICKS);
    }
    if (in->riderless && phase_is(time_base, 250U, 5U)) {
        euc_beeper_beep(&state->beeper, BEEP_WARNING_TICKS);
    }
    state->beeper.volume_compare = euc_beeper_volume_compare(in->beeper_volume);

    hard_cut_reminder(state, in, time_base);
    pwm_alarm(state, in, time_base, out);
    temperature_alarm(state, in, time_base);
    if (in->bms_fault && phase_is(time_base, 5000U, 1U)) {
        euc_beeper_beep(&state->beeper, BEEP_BMS_TICKS);
    }
    voltage_alarm(state, in, time_base);
    speed_alarm(state, in, time_base);
    horn(state, in->horn_request);

    if (in->transport_locked) {
        state->alerts |= EUC_ALERT_TRANSPORT_LOCK;
    } else {
        state->alerts &= (uint8_t)~EUC_ALERT_TRANSPORT_LOCK;
    }

    out->alerts = state->alerts;
    out->pwm_alarm = state->pwm_alarm;
    out->time_base = (uint16_t)time_base;
}
