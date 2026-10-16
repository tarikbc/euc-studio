/*
 * Unit tests for core/balance. Expected values are worked out by hand from the law and the step rules
 * described in the core/balance sources. Bit-exact comparison with the stock firmware is the job of the
 * lock-step harness, not of these tests.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/balance/balance.h"
#include "euc/sil.h"
#include "unity.h"

static struct euc_balance_in level_input(void)
{
    struct euc_balance_in in;
    memset(&in, 0, sizeof(in));
    in.accel_raw[2] = EUC_IMU_ACCEL_COUNTS_PER_G;
    in.battery_voltage_cv = 12800;           /* 128.0 V: the law's reference voltage */
    in.board_temperature_mpu6050 = -3846.0F; /* about 25 degC */
    in.handle_button_released = true;
    in.settings.pedal_mode = 2U; /* hard */
    in.settings.tiltback_speed_kmh = 25U;
    in.settings.pedal_trim = 4U; /* neutral */
    in.settings.turn_compensation = 3U;
    return in;
}

/* Power-on, then 201 level ticks: the countdown ends and the start-up check passes. */
static void start_running(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    struct euc_balance_out out;
    euc_balance_init(state);
    for (unsigned tick = 0U; tick < 201U; tick++) {
        euc_balance_tick(state, in, &out);
    }
}

/* A running state with the law inputs zeroed, ready for direct step calls. */
static void law_ready(struct euc_balance_state *state, struct euc_balance_in *in)
{
    *in = level_input();
    start_running(state, in);
    euc_balance_step_gains(state, &in->settings);
    state->speed_cm_per_s = 0;
    state->attitude.pitch_rate_lpf_rad_per_s = 0.0F;
    state->error_prev_deg = 0.0F;
}

void setUp(void)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT));
}

void tearDown(void)
{
}

/* Start-up --------------------------------------------------------------------------------------------- */

static void test_power_on_countdown_then_start(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    const struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    for (unsigned tick = 0U; tick < 200U; tick++) {
        euc_balance_tick(&state, &in, &out);
    }
    TEST_ASSERT_EQUAL_INT16(0, state.start_state);
    euc_balance_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_INT16(1, state.start_state);
    /* Soft start: the start-up check sets the limit to 0, the output step of the same tick adds 1. */
    TEST_ASSERT_EQUAL_FLOAT(1.0F, state.command_limit);
    euc_balance_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, state.command_limit);
}

static void test_tilted_wheel_does_not_start(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in = level_input();
    in.accel_raw[0] = 2802; /* 20 deg */
    in.accel_raw[2] = 7698;
    start_running(&state, &in);
    TEST_ASSERT_EQUAL_INT16(0, state.start_state);
}

/* Balance law ------------------------------------------------------------------------------------------ */

static void test_law_proportional_term_at_reference_voltage(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in;
    law_ready(&state, &in);
    state.error_deg = 2.0F;
    euc_balance_step_law(&state, &in);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, state.command); /* Kp 60 (hard) x 2 deg, x 1280 / 1280 */
}

static void test_law_scales_to_battery_voltage_within_limits(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in;
    law_ready(&state, &in);
    state.error_deg = 2.0F;
    in.battery_voltage_cv = 6400; /* clamped to 106.0 V */
    euc_balance_step_law(&state, &in);
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 120.0F * 1280.0F / 1060.0F, state.command);
    in.battery_voltage_cv = 20000; /* clamped to 134.0 V */
    euc_balance_step_law(&state, &in);
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 120.0F * 1280.0F / 1340.0F, state.command);
}

static void test_law_gains_follow_speed(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in;
    law_ready(&state, &in);
    state.speed_cm_per_s = 1000;
    state.error_deg = 1.0F;
    state.attitude.pitch_rate_lpf_rad_per_s = 1.0F;
    in.motor_current_lpf_ma = 10000;
    in.hall_period_speed = 1200;
    euc_balance_step_law(&state, &in);
    /* Kp 60 + 10 = 70; 0.1 x Kd (600 + 320) = 92; Kc (30 - 8) x 10000 / 10000 = 22; Kv 30 x 1200 / 1000 = 36
     */
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 70.0F + 92.0F + 22.0F + 36.0F, state.command);
}

static void test_voltage_mode_feed_forward_is_the_same_in_both_profiles(void)
{
    /* At 20 kHz hall calls the stock period speed reads 1.2 x high (1200 for 1000 mm/s) with Kv 30; clean
     * reads true mm/s (quirk HALL_SPEED_1_2X) with Kv 36. Both give 36 duty units. */
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    const int32_t period_speed[2] = {1200, 1000};
    for (size_t p = 0U; p < 2U; p++) {
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[p]));
        struct euc_balance_state state;
        struct euc_balance_in in;
        law_ready(&state, &in);
        state.error_deg = 0.0F;
        in.hall_period_speed = period_speed[p];
        euc_balance_step_law(&state, &in);
        TEST_ASSERT_FLOAT_WITHIN(0.001F, 36.0F, state.command);
    }
}

static void test_law_clamps_speed_for_gain_scheduling(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in;
    law_ready(&state, &in);
    state.speed_cm_per_s = 3000;
    euc_balance_step_law(&state, &in);
    TEST_ASSERT_EQUAL_INT16(2500, state.speed_cm_per_s);
}

static void test_law_current_mode(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in;
    law_ready(&state, &in);
    in.settings.drive_mode = 1U;
    euc_balance_step_gains(&state, &in.settings);
    state.error_deg = 10.0F;
    euc_balance_step_law(&state, &in);
    /* Kp 200 x 10 clamped to 1050, plus 160 x (10 - 0) for the change of the error */
    TEST_ASSERT_EQUAL_FLOAT(1050.0F + 1600.0F, state.command);
    euc_balance_step_law(&state, &in);
    TEST_ASSERT_EQUAL_FLOAT(1050.0F, state.command);
}

/* Output ----------------------------------------------------------------------------------------------- */

static void test_output_slew_and_soft_start(void)
{
    struct euc_balance_state state;
    euc_balance_init(&state);
    state.command_limit = 1000.0F;
    state.command = 500.0F;
    euc_balance_step_output(&state);
    TEST_ASSERT_EQUAL_FLOAT(120.0F, state.command); /* 120 per tick */
    TEST_ASSERT_EQUAL_FLOAT(1001.0F, state.command_limit);
    state.command = 500.0F;
    euc_balance_step_output(&state);
    TEST_ASSERT_EQUAL_FLOAT(240.0F, state.command);

    state.command_limit = 5.0F;
    state.command = 200.0F;
    euc_balance_step_output(&state);
    TEST_ASSERT_EQUAL_FLOAT(6.0F, state.command); /* limited by the soft start */
}

static void test_soft_start_limit_settles_between_1050_and_1051(void)
{
    struct euc_balance_state state;
    euc_balance_init(&state);
    state.command_limit = 1050.0F;
    euc_balance_step_output(&state);
    TEST_ASSERT_EQUAL_FLOAT(1051.0F, state.command_limit);
    euc_balance_step_output(&state);
    TEST_ASSERT_EQUAL_FLOAT(1050.0F, state.command_limit);
}

static void test_ramp_down_to_zero(void)
{
    struct euc_balance_state state;
    euc_balance_init(&state);
    state.command = 1060.0F;
    euc_balance_step_ramp_down(&state);
    TEST_ASSERT_EQUAL_FLOAT(1048.0F, state.command);
    state.command = -7.0F;
    euc_balance_step_ramp_down(&state);
    TEST_ASSERT_EQUAL_FLOAT(-5.0F, state.command);
    euc_balance_step_ramp_down(&state);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, state.command); /* -3 is inside the zero band */
}

/* Tiltbacks -------------------------------------------------------------------------------------------- */

static void test_speed_tiltback(void)
{
    struct euc_balance_state state;
    const struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    state.speed_m_per_s = 8.0F; /* 28.8 km/h, threshold 25 km/h = 6.944 m/s */
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_balance_step_tiltbacks(&state, &in);
    }
    /* Toward 10 deg per m/s of overspeed (10.56 deg) with a 4 s time constant: 39 % after 2 s. */
    TEST_ASSERT_FLOAT_WITHIN(0.1F, 10.56F * (1.0F - powf(0.9995F, 1000.0F)), state.tiltback_speed_deg);
    state.speed_m_per_s = 0.0F;
    const float before = state.tiltback_speed_deg;
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, before * 0.999F, state.tiltback_speed_deg);
}

static void test_lost_hall_line_lowers_the_speed_tiltback(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in = level_input(); /* tiltback 25 km/h */
    euc_balance_init(&state);
    state.speed_m_per_s = 5.0F; /* 18 km/h */
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, state.tiltback_speed_deg);
    in.hall_degraded = true; /* limp: 10 km/h = 2.778 m/s */
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_balance_step_tiltbacks(&state, &in);
    }
    const float overspeed_m_per_s = 5.0F - (10.0F / 3.6F);
    TEST_ASSERT_FLOAT_WITHIN(0.1F, 10.0F * overspeed_m_per_s * (1.0F - powf(0.9995F, 1000.0F)),
                             state.tiltback_speed_deg);
    /* A lower setting stays */
    euc_balance_init(&state);
    in.settings.tiltback_speed_kmh = 8U;
    state.speed_m_per_s = 2.5F; /* 9 km/h: above 8, below the limp speed */
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_TRUE(state.tiltback_speed_deg > 0.0F);
}

static void test_battery_tiltback_window(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    in.battery_voltage_cv = 9850; /* below 99.20 V */
    for (unsigned tick = 0U; tick < 100U; tick++) {
        euc_balance_step_tiltbacks(&state, &in);
    }
    const float ramped = state.tiltback_battery_deg;
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.48F, ramped);

    in.battery_voltage_cv = 10000; /* 99.20..100.21 V: hold */
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_EQUAL_FLOAT(ramped, state.tiltback_battery_deg);

    in.battery_voltage_cv = 12000; /* inside the window: release */
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_TRUE(state.tiltback_battery_deg < ramped);

    in.board_temperature_mpu6050 = 14600.0F; /* above about 80 degC */
    const float before = state.tiltback_battery_deg;
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_TRUE(state.tiltback_battery_deg > before);
}

static void test_pwm_tiltback_needs_the_setting_and_clamps(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    in.pwm_alarm_active = true;
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, state.tiltback_pwm_deg); /* 'x': off */

    in.settings.pwm_tiltback_enabled = true;
    for (unsigned tick = 0U; tick < 5000U; tick++) {
        euc_balance_step_tiltbacks(&state, &in);
    }
    TEST_ASSERT_EQUAL_FLOAT(12.0F, state.tiltback_pwm_deg);
}

static void test_pedal_trim_slews_without_overshoot(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    in.settings.pedal_trim = 7U; /* (7 - 4) x 1.2 = 3.6 deg */
    euc_balance_step_tiltbacks(&state, &in);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.005F, state.pedal_trim_deg);
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_balance_step_tiltbacks(&state, &in);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 3.6F, state.pedal_trim_deg);
}

/* Cut-offs and recovery -------------------------------------------------------------------------------- */

static void test_pitch_cutoff_then_recovery(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    state.attitude.pitch_deg = 45.0F;
    for (unsigned tick = 0U; tick < 51U; tick++) {
        euc_balance_step_cutoffs(&state, &in, &out);
    }
    TEST_ASSERT_FALSE(state.fallen);
    euc_balance_step_cutoffs(&state, &in, &out);
    TEST_ASSERT_TRUE(state.motor_disabled);
    TEST_ASSERT_TRUE(state.fallen);

    /* Level (accelerometer tilt) and still: 101 ticks re-enable the motor. */
    state.attitude.tilt_centideg[2] = 8800;
    state.error_deg = 3.5F;
    memset(&out, 0, sizeof(out));
    for (unsigned tick = 0U; tick < 100U; tick++) {
        euc_balance_step_recovery(&state, &in, &out);
    }
    TEST_ASSERT_TRUE(state.motor_disabled);
    euc_balance_step_recovery(&state, &in, &out);
    TEST_ASSERT_FALSE(state.motor_disabled);
    TEST_ASSERT_FALSE(state.fallen);
    TEST_ASSERT_EQUAL_UINT8(1U, out.beep_count);
    TEST_ASSERT_EQUAL_UINT8(20U, out.beep_ticks[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, state.command_limit);
    TEST_ASSERT_EQUAL_FLOAT(3.5F, state.restart_offset_deg);
}

static void test_recovery_blocked_by_a_hard_fault(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    state.motor_disabled = true;
    state.fallen = true;
    state.attitude.tilt_centideg[2] = 8800;
    in.hard_inhibit = true;
    memset(&out, 0, sizeof(out));
    for (unsigned tick = 0U; tick < 101U; tick++) {
        euc_balance_step_recovery(&state, &in, &out);
    }
    TEST_ASSERT_TRUE(state.motor_disabled);
    TEST_ASSERT_FALSE(state.fallen);
    TEST_ASSERT_EQUAL_UINT8(0U, out.beep_count);
}

static void test_recovery_blocked_by_a_lost_hall_line(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    state.motor_disabled = true;
    state.fallen = true;
    state.attitude.tilt_centideg[2] = 8800;
    in.hall_degraded = true;
    memset(&out, 0, sizeof(out));
    for (unsigned tick = 0U; tick < 101U; tick++) {
        euc_balance_step_recovery(&state, &in, &out);
    }
    TEST_ASSERT_TRUE(state.motor_disabled);
    TEST_ASSERT_EQUAL_UINT8(0U, out.beep_count);
}

static void test_error_cutoff_beeps_once(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    const struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    state.error_deg = -41.0F;
    unsigned beeps = 0U;
    for (unsigned tick = 0U; tick < 30U; tick++) {
        memset(&out, 0, sizeof(out));
        euc_balance_step_cutoffs(&state, &in, &out);
        beeps += out.beep_count;
        TEST_ASSERT_EQUAL(tick >= 10U, state.motor_disabled);
    }
    TEST_ASSERT_EQUAL_UINT(1U, beeps);
    TEST_ASSERT_FALSE(state.fallen); /* no automatic recovery from this one */
}

static void test_overload_cutoff_after_1_5_s(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    in.motor_voltage_q = -31001;
    for (unsigned tick = 0U; tick < 751U; tick++) {
        euc_balance_step_cutoffs(&state, &in, &out);
    }
    TEST_ASSERT_FALSE(state.motor_disabled);
    euc_balance_step_cutoffs(&state, &in, &out);
    TEST_ASSERT_TRUE(state.motor_disabled);
}

static void test_roll_cutoffs(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();

    /* Slow: |roll| > 40 + 5 x 1 = 45 deg for more than 20 ticks. */
    euc_balance_init(&state);
    in.settings.roll_limit_step = 1U;
    state.attitude.roll_deg = 46.0F;
    for (unsigned tick = 0U; tick < 20U; tick++) {
        euc_balance_step_cutoffs(&state, &in, &out);
    }
    TEST_ASSERT_FALSE(state.tipped);
    euc_balance_step_cutoffs(&state, &in, &out);
    TEST_ASSERT_TRUE(state.tipped);

    /* Fast: |roll| > 50 deg (mode 0) with Vq above 30000 trips at once. */
    euc_balance_init(&state);
    in.settings.roll_limit_step = 9U; /* slow limit 85 deg: out of the way */
    state.attitude.roll_deg = -51.0F;
    in.motor_voltage_q = 30001;
    euc_balance_step_cutoffs(&state, &in, &out);
    TEST_ASSERT_TRUE(state.tipped);
}

/* Handle button, disable request, speed ---------------------------------------------------------------- */

static void press(struct euc_balance_state *state, struct euc_balance_in *in, bool pressed, unsigned ticks,
                  struct euc_balance_out *out)
{
    in->handle_button_released = !pressed;
    for (unsigned tick = 0U; tick < ticks; tick++) {
        memset(out, 0, sizeof(*out));
        euc_balance_step_handle(state, in, out);
        euc_balance_step_output(state);
    }
}

static void test_handle_cut_and_restore(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    state.command_limit = 1050.0F;

    press(&state, &in, true, 51U, &out);
    TEST_ASSERT_EQUAL_UINT8(1U, state.handle_presses);
    press(&state, &in, false, 10U, &out);
    press(&state, &in, true, 50U, &out);
    in.handle_button_released = false;
    memset(&out, 0, sizeof(out));
    euc_balance_step_handle(&state, &in, &out); /* 51st tick of the second press */
    TEST_ASSERT_EQUAL_UINT8(2U, state.handle_presses);
    TEST_ASSERT_TRUE(out.handle_double_beep);

    press(&state, &in, false, 20U, &out);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, state.command_limit); /* motor cut */

    state.error_deg = 2.0F;
    press(&state, &in, true, 51U, &out);
    in.handle_button_released = true;
    memset(&out, 0, sizeof(out));
    euc_balance_step_handle(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT8(0U, state.handle_presses);
    TEST_ASSERT_EQUAL_UINT8(25U, out.beep_ticks[0]);
}

static void test_disable_request_acts_after_the_tick(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    struct euc_balance_in in = level_input();
    start_running(&state, &in);
    in.disable_request = true;
    euc_balance_tick(&state, &in, &out);
    TEST_ASSERT_TRUE(out.motor_disabled);
    TEST_ASSERT_EQUAL_FLOAT(state.command_prev, out.motor_command); /* the law still ran in this tick */
}

static void test_speed_estimate_from_hall_steps(void)
{
    struct euc_balance_state state;
    struct euc_balance_in in = level_input();
    euc_balance_init(&state);
    in.hall_steps_per_window = -10;
    for (unsigned tick = 0U; tick < 2000U; tick++) {
        euc_balance_step_speed(&state, &in);
    }
    /* Nominal: -421 mm/s per step, and the same for 10 x cm/s in the gain filter. The integer divisions round
     * toward zero, so from standstill the filters stop a little short: 4201 mm/s and 416 cm/s. */
    TEST_ASSERT_EQUAL_INT16(4201, state.speed_mm_per_s);
    TEST_ASSERT_EQUAL_INT16(416, state.speed_cm_per_s);
    TEST_ASSERT_INT_WITHIN(1, 42, state.speed_dm_per_s);
    TEST_ASSERT_FLOAT_WITHIN(0.11F, 4.2F, state.speed_m_per_s);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_power_on_countdown_then_start);
    RUN_TEST(test_tilted_wheel_does_not_start);
    RUN_TEST(test_law_proportional_term_at_reference_voltage);
    RUN_TEST(test_law_scales_to_battery_voltage_within_limits);
    RUN_TEST(test_law_gains_follow_speed);
    RUN_TEST(test_voltage_mode_feed_forward_is_the_same_in_both_profiles);
    RUN_TEST(test_law_clamps_speed_for_gain_scheduling);
    RUN_TEST(test_law_current_mode);
    RUN_TEST(test_output_slew_and_soft_start);
    RUN_TEST(test_soft_start_limit_settles_between_1050_and_1051);
    RUN_TEST(test_ramp_down_to_zero);
    RUN_TEST(test_speed_tiltback);
    RUN_TEST(test_lost_hall_line_lowers_the_speed_tiltback);
    RUN_TEST(test_battery_tiltback_window);
    RUN_TEST(test_pwm_tiltback_needs_the_setting_and_clamps);
    RUN_TEST(test_pedal_trim_slews_without_overshoot);
    RUN_TEST(test_pitch_cutoff_then_recovery);
    RUN_TEST(test_recovery_blocked_by_a_hard_fault);
    RUN_TEST(test_recovery_blocked_by_a_lost_hall_line);
    RUN_TEST(test_error_cutoff_beeps_once);
    RUN_TEST(test_overload_cutoff_after_1_5_s);
    RUN_TEST(test_roll_cutoffs);
    RUN_TEST(test_handle_cut_and_restore);
    RUN_TEST(test_disable_request_acts_after_the_tick);
    RUN_TEST(test_speed_estimate_from_hall_steps);
    return UNITY_END();
}
