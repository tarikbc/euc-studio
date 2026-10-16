/*
 * Unit tests for the service sequences (core/service/): the zero-command wait and, in the clean profile, the
 * no-rider and power-button confirmation (quirk SERVICE_WITHOUT_CONFIRM).
 */
#include <stdbool.h>
#include <stdint.h>

#include "euc/service/service.h"
#include "euc/sil.h"
#include "unity.h"

#define WINDOW_TICKS 4960U

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

static struct euc_service_out tick(struct euc_service_state *state, float command, bool no_rider,
                                   bool pressed)
{
    const struct euc_service_in in = {command, no_rider, pressed};
    struct euc_service_out out;
    euc_service_tick(state, &in, &out);
    return out;
}

/* `count` ticks; returns true if any of them cut the motor. */
static bool ticks(struct euc_service_state *state, uint32_t count, bool no_rider, bool pressed)
{
    bool motor_off = false;
    for (uint32_t i = 0U; i < count; i++) {
        motor_off = tick(state, 0.0F, no_rider, pressed).motor_off || motor_off;
    }
    return motor_off;
}

/* Compat ---------------------------------------------------------------------------------------------------
 */

static void test_service_waits_for_a_zero_command(void)
{
    struct euc_service_state state;
    euc_service_init(&state);
    struct euc_service_out out = tick(&state, 100.0F, false, false);
    TEST_ASSERT_FALSE(out.motor_off);

    euc_service_request(&state, EUC_SERVICE_FIRMWARE_UPDATE);
    out = tick(&state, 100.0F, false, false); /* compat: a rider does not matter */
    TEST_ASSERT_TRUE(out.motor_off);
    TEST_ASSERT_FALSE(out.enter_bootloader);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_NONE, out.reply);
    out = tick(&state, -0.0F, false, false);
    TEST_ASSERT_TRUE(out.enter_bootloader);
    out = tick(&state, 1e-40F, false, false); /* a denormal counts as zero, as in the stock exponent test */
    TEST_ASSERT_TRUE(out.enter_bootloader);
}

static void test_service_actions_run_once(void)
{
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_CALIBRATION);
    euc_service_request(&state, EUC_SERVICE_DRIVE_MODE_TOGGLE);
    struct euc_service_out out = tick(&state, 0.0F, true, false);
    TEST_ASSERT_TRUE(out.erase_calibration);
    TEST_ASSERT_TRUE(out.toggle_drive_mode);
    out = tick(&state, 0.0F, true, false);
    TEST_ASSERT_FALSE(out.erase_calibration || out.toggle_drive_mode || out.motor_off);
}

/* Clean ----------------------------------------------------------------------------------------------------
 */

static void test_clean_denies_with_a_rider(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_FIRMWARE_UPDATE);
    const struct euc_service_out out = tick(&state, 0.0F, false, false);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_DENIED, out.reply);
    TEST_ASSERT_FALSE(out.motor_off || out.enter_bootloader);
    TEST_ASSERT_FALSE(euc_service_confirming(&state));
    TEST_ASSERT_FALSE(ticks(&state, 100U, true, false));
}

static void test_clean_runs_after_a_short_press(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_DRIVE_MODE_TOGGLE);
    struct euc_service_out out = tick(&state, 0.0F, true, false);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_CONFIRM, out.reply);
    TEST_ASSERT_EQUAL_UINT8(15U, out.beep_ticks);
    TEST_ASSERT_TRUE(euc_service_confirming(&state));
    TEST_ASSERT_FALSE(ticks(&state, 200U, true, false));
    TEST_ASSERT_FALSE(ticks(&state, 50U, true, true)); /* the press: nothing yet */
    out = tick(&state, 0.0F, true, false);             /* the release */
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_OK, out.reply);
    TEST_ASSERT_EQUAL_UINT8(100U, out.beep_ticks);
    TEST_ASSERT_TRUE(out.motor_off);
    TEST_ASSERT_TRUE(out.toggle_drive_mode);
    TEST_ASSERT_FALSE(euc_service_confirming(&state));
}

static void test_clean_prompt_beeps(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_CALIBRATION);
    uint32_t beeps = 0U;
    for (uint32_t t = 0U; t < 2000U; t++) {
        const struct euc_service_out out = tick(&state, 0.0F, true, false);
        if (out.beep_ticks != 0U) {
            TEST_ASSERT_TRUE(((t % 1000U) == 0U) || ((t % 1000U) == 50U) || ((t % 1000U) == 100U));
            beeps++;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(6U, beeps);
}

static void test_clean_denies_after_10_s(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_FIRMWARE_UPDATE);
    TEST_ASSERT_FALSE(ticks(&state, WINDOW_TICKS, true, false));
    TEST_ASSERT_TRUE(euc_service_confirming(&state));
    const struct euc_service_out out = tick(&state, 0.0F, true, false);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_DENIED, out.reply);
    TEST_ASSERT_FALSE(out.motor_off || out.enter_bootloader);
    TEST_ASSERT_FALSE(ticks(&state, 50U, true, true)); /* a late press does nothing */
    TEST_ASSERT_FALSE(tick(&state, 0.0F, true, false).motor_off);
}

static void test_clean_denies_when_a_rider_steps_on(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_CALIBRATION);
    TEST_ASSERT_FALSE(ticks(&state, 100U, true, false));
    const struct euc_service_out out = tick(&state, 0.0F, false, false);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_DENIED, out.reply);
    TEST_ASSERT_FALSE(ticks(&state, 50U, true, true));
    TEST_ASSERT_FALSE(tick(&state, 0.0F, true, false).motor_off);
}

static void test_clean_press_must_be_short_and_after_the_sequence(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    TEST_ASSERT_FALSE(ticks(&state, 20U, true, true)); /* held before the sequence */
    euc_service_request(&state, EUC_SERVICE_CALIBRATION);
    TEST_ASSERT_FALSE(ticks(&state, 30U, true, true));
    TEST_ASSERT_FALSE(ticks(&state, 10U, true, false)); /* that release does not count */
    TEST_ASSERT_FALSE(ticks(&state, 300U, true, true)); /* too long */
    TEST_ASSERT_FALSE(ticks(&state, 10U, true, false));
    TEST_ASSERT_FALSE(ticks(&state, 5U, true, true)); /* too short */
    TEST_ASSERT_FALSE(ticks(&state, 10U, true, false));
    TEST_ASSERT_TRUE(euc_service_confirming(&state));
    TEST_ASSERT_FALSE(ticks(&state, 11U, true, true));
    const struct euc_service_out out = tick(&state, 0.0F, true, false);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_OK, out.reply);
    TEST_ASSERT_TRUE(out.erase_calibration);
}

static void test_clean_new_sequence_replaces_a_waiting_one(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_service_state state;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_FIRMWARE_UPDATE);
    TEST_ASSERT_FALSE(ticks(&state, 100U, true, false));
    euc_service_request(&state, EUC_SERVICE_CALIBRATION);
    TEST_ASSERT_EQUAL(EUC_SERVICE_REPLY_CONFIRM, tick(&state, 0.0F, true, false).reply);
    TEST_ASSERT_FALSE(ticks(&state, 20U, true, true));
    const struct euc_service_out out = tick(&state, 0.0F, true, false);
    TEST_ASSERT_TRUE(out.erase_calibration);
    TEST_ASSERT_FALSE(out.enter_bootloader);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_service_waits_for_a_zero_command);
    RUN_TEST(test_service_actions_run_once);
    RUN_TEST(test_clean_denies_with_a_rider);
    RUN_TEST(test_clean_runs_after_a_short_press);
    RUN_TEST(test_clean_prompt_beeps);
    RUN_TEST(test_clean_denies_after_10_s);
    RUN_TEST(test_clean_denies_when_a_rider_steps_on);
    RUN_TEST(test_clean_press_must_be_short_and_after_the_sequence);
    RUN_TEST(test_clean_new_sequence_replaces_a_waiting_one);
    return UNITY_END();
}
