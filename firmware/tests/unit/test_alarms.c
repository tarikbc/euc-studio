/*
 * Unit tests for the beeper and the alarms (core/alarms/). A beep start is seen as ticks_left equal to the
 * beep length at the end of a tick.
 */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/alarms/alarms.h"
#include "euc/sil.h"
#include "unity.h"

static void use_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

void setUp(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
}

void tearDown(void)
{
}

static struct euc_alarm_in quiet_input(void)
{
    struct euc_alarm_in in;
    (void)memset(&in, 0, sizeof(in));
    in.battery_voltage_cv = 12800;
    in.board_temperature_mpu6050 = -3846.0F;
    in.beeper_volume = 6U;
    in.pwm_alarm_percent = 80U;
    in.speed_alarm_mode = 0U;
    return in;
}

/* Runs `ticks` ticks and counts beep starts of length `beep_ticks`. */
static uint32_t count_beeps(struct euc_alarm_state *state, const struct euc_alarm_in *in, uint32_t ticks,
                            uint16_t beep_ticks)
{
    struct euc_alarm_out out;
    uint32_t starts = 0U;
    for (uint32_t i = 0U; i < ticks; i++) {
        euc_alarm_tick(state, in, &out);
        if (state->beeper.ticks_left == beep_ticks) {
            starts++;
        }
    }
    return starts;
}

static void test_volume_compare(void)
{
    TEST_ASSERT_EQUAL_UINT16(1U, euc_beeper_volume_compare(1U));
    TEST_ASSERT_EQUAL_UINT16(151U, euc_beeper_volume_compare(6U));
    TEST_ASSERT_EQUAL_UINT16(241U, euc_beeper_volume_compare(9U));
    TEST_ASSERT_EQUAL_UINT16(270U, euc_beeper_volume_compare(200U));
    TEST_ASSERT_EQUAL_UINT16(270U, euc_beeper_volume_compare(0U)); /* VOLUME_ZERO_MAX */
    use_profile(EUC_SIL_PROFILE_CLEAN);
    TEST_ASSERT_EQUAL_UINT16(1U, euc_beeper_volume_compare(0U));
}

static void test_beep_length_and_silence(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    const struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    euc_alarm_tick(&state, &in, &out); /* sets the volume */
    euc_beeper_beep(&state.beeper, 3U);
    TEST_ASSERT_EQUAL_UINT16(151U, state.beeper.compare);
    for (unsigned i = 0U; i < 3U; i++) {
        euc_alarm_tick(&state, &in, &out);
        TEST_ASSERT_EQUAL_UINT16(151U, state.beeper.compare);
    }
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT16(0U, state.beeper.compare);
}

static void test_time_base_wraps(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    const struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT16(29821U, out.time_base); /* power-on value -180, then +1 */
    state.time_base = 30001;
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT16(1U, out.time_base);
}

static void test_pwm_alarm(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    in.motor_voltage_q = 24029; /* / 30 = 800 = 10 x 80: not above */
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_FALSE(out.pwm_alarm);
    in.motor_voltage_q = -24030; /* -801 */
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_TRUE(out.pwm_alarm);
    TEST_ASSERT_EQUAL_HEX8(EUC_ALERT_PWM_HIGH, out.alerts);
    TEST_ASSERT_EQUAL_UINT16(241U, state.beeper.volume_compare); /* full volume while it lasts */
    TEST_ASSERT_EQUAL_UINT32(10U, count_beeps(&state, &in, 1000U, 35U));
    in.pwm_alarm_percent = 95U;
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT8(90U, out.pwm_alarm_percent);
    TEST_ASSERT_FALSE(out.pwm_alarm); /* 801 is below 900 */
    TEST_ASSERT_EQUAL_HEX8(0U, out.alerts);
}

static void test_voltage_alarms(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    in.battery_voltage_cv = 10559;
    TEST_ASSERT_EQUAL_UINT32(1U, count_beeps(&state, &in, 1000U, 50U));
    in.speed_dm_per_s = -30;
    TEST_ASSERT_EQUAL_UINT32(2U, count_beeps(&state, &in, 1000U, 50U));
    in.speed_dm_per_s = 50;
    TEST_ASSERT_EQUAL_UINT32(3U, count_beeps(&state, &in, 1000U, 50U));
    TEST_ASSERT_EQUAL_HEX8(EUC_ALERT_UNDER_VOLTAGE, state.alerts);

    in.battery_voltage_cv = 13761;
    TEST_ASSERT_EQUAL_UINT32(3U, count_beeps(&state, &in, 1000U, 50U));
    TEST_ASSERT_EQUAL_HEX8(EUC_ALERT_UNDER_VOLTAGE | EUC_ALERT_OVER_VOLTAGE, state.alerts);
    in.battery_voltage_cv = 13760;
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_HEX8(0U, out.alerts);
}

static void test_temperature_alarm(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    struct euc_alarm_in in = quiet_input();
    const float values[5] = {14500.0F, 14500.002F, INFINITY, NAN, -INFINITY};
    const bool alarm[5] = {false, true, true, true, false};
    euc_alarm_init(&state);
    for (size_t i = 0U; i < 5U; i++) {
        in.board_temperature_mpu6050 = values[i];
        euc_alarm_tick(&state, &in, &out);
        TEST_ASSERT_EQUAL(alarm[i], (out.alerts & EUC_ALERT_OVER_TEMPERATURE) != 0U);
    }
    in.board_temperature_mpu6050 = 15000.0F;
    TEST_ASSERT_EQUAL_UINT32(2U, count_beeps(&state, &in, 1000U, 50U));
}

static void test_speed_alarms_by_mode(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    in.speed_dm_per_s = 84;
    TEST_ASSERT_EQUAL_UINT32(2U, count_beeps(&state, &in, 500U, 35U));
    in.speed_dm_per_s = -126;
    TEST_ASSERT_EQUAL_UINT32(3U, count_beeps(&state, &in, 500U, 35U));
    in.speed_alarm_mode = 1U;
    TEST_ASSERT_EQUAL_UINT32(3U, count_beeps(&state, &in, 500U, 35U));
    in.speed_dm_per_s = 100;
    TEST_ASSERT_EQUAL_UINT32(0U, count_beeps(&state, &in, 500U, 35U));
    in.speed_alarm_mode = 2U;
    in.speed_dm_per_s = 200;
    TEST_ASSERT_EQUAL_UINT32(0U, count_beeps(&state, &in, 500U, 35U));
}

static void test_horn_and_handle_double_beep(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    in.horn_request = true;
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT16(35U, state.beeper.ticks_left);
    in.horn_request = false;
    TEST_ASSERT_EQUAL_UINT32(1U, count_beeps(&state, &in, 99U, 35U)); /* the second beep, 99 ticks later */
    TEST_ASSERT_EQUAL_UINT32(0U, count_beeps(&state, &in, 300U, 35U));

    in.handle_double_beep = true;
    euc_alarm_tick(&state, &in, &out);
    in.handle_double_beep = false;
    TEST_ASSERT_EQUAL_UINT32(2U, count_beeps(&state, &in, 300U, 25U));
}

static void test_hard_cut_reminder_and_hall_beeps(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    in.hard_cut = true;
    TEST_ASSERT_EQUAL_UINT32(5U, count_beeps(&state, &in, 2500U, 50U));
    TEST_ASSERT_EQUAL_UINT32(0U, count_beeps(&state, &in, 5000U, 50U)); /* only the first 5 s */
    in.hard_cut = false;
    in.raise_alerts = EUC_ALERT_HALL_SENSOR;
    TEST_ASSERT_EQUAL_UINT32(8U, count_beeps(&state, &in, 1000U, 20U));
    in.raise_alerts = 0U;
    TEST_ASSERT_EQUAL_HEX8(EUC_ALERT_HALL_SENSOR, state.alerts & EUC_ALERT_HALL_SENSOR); /* latched */
}

static void test_main_loop_stall_beeps(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    TEST_ASSERT_EQUAL_UINT32(0U, count_beeps(&state, &in, 1000U, 20U));
    in.main_loop_stalled = true;
    TEST_ASSERT_EQUAL_UINT32(6U, count_beeps(&state, &in, 1000U, 20U)); /* 3 beeps every 500 ticks */
}

static void test_transport_lock_bit(void)
{
    struct euc_alarm_state state;
    struct euc_alarm_out out;
    struct euc_alarm_in in = quiet_input();
    euc_alarm_init(&state);
    in.transport_locked = true;
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_HEX8(EUC_ALERT_TRANSPORT_LOCK, out.alerts);
    in.transport_locked = false;
    euc_alarm_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_HEX8(0U, out.alerts);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_volume_compare);
    RUN_TEST(test_beep_length_and_silence);
    RUN_TEST(test_time_base_wraps);
    RUN_TEST(test_pwm_alarm);
    RUN_TEST(test_voltage_alarms);
    RUN_TEST(test_temperature_alarm);
    RUN_TEST(test_speed_alarms_by_mode);
    RUN_TEST(test_horn_and_handle_double_beep);
    RUN_TEST(test_hard_cut_reminder_and_hall_beeps);
    RUN_TEST(test_main_loop_stall_beeps);
    RUN_TEST(test_transport_lock_bit);
    return UNITY_END();
}
