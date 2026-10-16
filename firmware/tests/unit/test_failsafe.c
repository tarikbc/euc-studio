/*
 * Fail-closed rules of the balance task (firmware/README.md): NaN and infinity injected at each stage.
 *
 *   clean (NAN_FAIL_OPEN off)  the command goes to 0, the motor stays off, numeric_fault is latched
 *   both profiles              the published motor command is always finite and within the output limit
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "euc/balance/balance.h"
#include "euc/sil.h"
#include "unity.h"

static const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};

static struct euc_balance_in level_input(void)
{
    struct euc_balance_in in;
    memset(&in, 0, sizeof(in));
    in.accel_raw[2] = EUC_IMU_ACCEL_COUNTS_PER_G;
    in.battery_voltage_cv = 12800;
    in.board_temperature_mpu6050 = -3846.0F;
    in.handle_button_released = true;
    in.settings.pedal_mode = 2U;
    in.settings.tiltback_speed_kmh = 25U;
    in.settings.pedal_trim = 4U;
    return in;
}

/* Power-on, start-up, then 300 more ticks so the soft-start limit is well above zero. */
static void start_running(struct euc_balance_state *state, const struct euc_balance_in *in)
{
    struct euc_balance_out out;
    euc_balance_init(state);
    for (unsigned tick = 0U; tick < 500U; tick++) {
        euc_balance_tick(state, in, &out);
    }
    TEST_ASSERT_EQUAL_INT16(1, state->start_state);
}

static void assert_published_command_safe(const struct euc_balance_state *state)
{
    TEST_ASSERT_TRUE(isfinite(state->motor_command));
    TEST_ASSERT_TRUE(fabsf(state->motor_command) <= fabsf(state->command_limit));
}

static void assert_clean_stopped(const struct euc_balance_state *state)
{
    TEST_ASSERT_TRUE(state->numeric_fault);
    TEST_ASSERT_TRUE(state->motor_disabled);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, state->motor_command);
}

void setUp(void)
{
}

void tearDown(void)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT));
}

static void test_nan_in_the_attitude_filter(void)
{
    for (size_t index = 0U; index < 2U; index++) {
        struct euc_balance_state state;
        struct euc_balance_out out;
        const struct euc_balance_in in = level_input();
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
        start_running(&state, &in);
        state.attitude.quaternion[1] = NAN;
        for (unsigned tick = 0U; tick < 50U; tick++) {
            euc_balance_tick(&state, &in, &out);
            assert_published_command_safe(&state);
            TEST_ASSERT_TRUE(isfinite(out.motor_command));
        }
        if (profiles[index] == EUC_SIL_PROFILE_CLEAN) {
            assert_clean_stopped(&state);
            TEST_ASSERT_EQUAL_FLOAT(1.0F, state.attitude.quaternion[0]); /* filter reset, then still level */
            TEST_ASSERT_TRUE(isfinite(state.attitude.pitch_deg));
        } else {
            TEST_ASSERT_FALSE(state.numeric_fault); /* compat keeps the stock behaviour inside the task */
            TEST_ASSERT_FALSE(isfinite(state.command));
        }
    }
}

static void test_nan_and_inf_at_the_law(void)
{
    const float poison[2] = {NAN, INFINITY};
    for (size_t index = 0U; index < 2U; index++) {
        for (size_t value = 0U; value < 2U; value++) {
            struct euc_balance_state state;
            struct euc_balance_in in = level_input();
            euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
            start_running(&state, &in);
            state.error_deg = poison[value];
            euc_balance_step_law(&state, &in);
            euc_balance_step_output(&state);
            assert_published_command_safe(&state);
            if (profiles[index] == EUC_SIL_PROFILE_CLEAN) {
                assert_clean_stopped(&state);
            }
        }
    }
}

static void test_nan_in_the_pitch_rate(void)
{
    for (size_t index = 0U; index < 2U; index++) {
        struct euc_balance_state state;
        struct euc_balance_in in = level_input();
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
        start_running(&state, &in);
        state.attitude.pitch_rate_lpf_rad_per_s = NAN;
        euc_balance_step_law(&state, &in);
        euc_balance_step_output(&state);
        assert_published_command_safe(&state);
        if (profiles[index] == EUC_SIL_PROFILE_CLEAN) {
            assert_clean_stopped(&state);
        }
    }
}

static void test_nan_during_the_ramp_down(void)
{
    for (size_t index = 0U; index < 2U; index++) {
        struct euc_balance_state state;
        const struct euc_balance_in in = level_input();
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
        start_running(&state, &in);
        state.motor_disabled = true;
        state.command = NAN;
        euc_balance_step_ramp_down(&state);
        euc_balance_step_output(&state);
        assert_published_command_safe(&state);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, state.motor_command);
        if (profiles[index] == EUC_SIL_PROFILE_CLEAN) {
            assert_clean_stopped(&state);
            TEST_ASSERT_EQUAL_FLOAT(0.0F, state.command);
        }
    }
}

static void test_infinity_at_the_output(void)
{
    for (size_t index = 0U; index < 2U; index++) {
        struct euc_balance_state state;
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
        euc_balance_init(&state);
        state.command_limit = 500.0F;
        state.command_prev = 500.0F;
        state.command = -INFINITY;
        euc_balance_step_output(&state);
        assert_published_command_safe(&state);
        if (profiles[index] == EUC_SIL_PROFILE_CLEAN) {
            assert_clean_stopped(&state);
        } else {
            /* Compat: the stock slew and limit clamp a finite-signed infinity like any large value. */
            TEST_ASSERT_EQUAL_FLOAT(380.0F, state.motor_command);
        }
    }
}

static void test_nan_output_limit_publishes_zero(void)
{
    for (size_t index = 0U; index < 2U; index++) {
        struct euc_balance_state state;
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
        euc_balance_init(&state);
        state.command_limit = NAN;
        state.command = 100.0F;
        euc_balance_step_output(&state);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, state.motor_command);
    }
}

static void test_recovery_stays_blocked_after_a_numeric_fault(void)
{
    struct euc_balance_state state;
    struct euc_balance_out out;
    const struct euc_balance_in in = level_input();
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_CLEAN));
    start_running(&state, &in);
    state.attitude.quaternion[2] = INFINITY;
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_balance_tick(&state, &in, &out);
    }
    TEST_ASSERT_TRUE(out.numeric_fault);
    TEST_ASSERT_TRUE(out.motor_disabled);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, out.motor_command);
}

static void test_pitch_stays_finite_through_90_degrees(void)
{
    for (size_t index = 0U; index < 2U; index++) {
        euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profiles[index]));
        for (int step = 0; step < 20000; step++) {
            const double theta_rad = (89.99 + (step * 0.000001)) * (3.14159265358979323846 / 180.0);
            struct euc_attitude_state state;
            struct euc_attitude_out out;
            struct euc_attitude_in in = {0};
            euc_attitude_init(&state);
            state.quaternion[0] = (float)cos(theta_rad / 2.0);
            state.quaternion[2] = (float)sin(theta_rad / 2.0);
            in.accel[0] = -EUC_IMU_ACCEL_COUNTS_PER_G;
            euc_attitude_update(&state, &in, &out);
            TEST_ASSERT_TRUE(isfinite(out.pitch_deg));
        }
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nan_in_the_attitude_filter);
    RUN_TEST(test_nan_and_inf_at_the_law);
    RUN_TEST(test_nan_in_the_pitch_rate);
    RUN_TEST(test_nan_during_the_ramp_down);
    RUN_TEST(test_infinity_at_the_output);
    RUN_TEST(test_nan_output_limit_publishes_zero);
    RUN_TEST(test_recovery_stays_blocked_after_a_numeric_fault);
    RUN_TEST(test_pitch_stays_finite_through_90_degrees);
    return UNITY_END();
}
