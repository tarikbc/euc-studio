/*
 * Unit tests for the watchdog supervisor (core/safety/supervisor.c): what feeds the watchdog and when a
 * stalled main loop is latched (clean), and that the stock profile does neither (quirk NO_WATCHDOG).
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "euc/safety/supervisor.h"
#include "euc/sil.h"
#include "unity.h"

static void use_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

void setUp(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
}

void tearDown(void)
{
}

static struct euc_supervisor_out tick(struct euc_supervisor_state *state, uint32_t passes, uint32_t foc_steps,
                                      float command)
{
    const struct euc_supervisor_in in = {passes, foc_steps, command};
    struct euc_supervisor_out out;
    euc_supervisor_tick(state, &in, &out);
    return out;
}

static void test_compat_never_feeds_or_latches(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
    struct euc_supervisor_state state;
    euc_supervisor_init(&state);
    for (uint32_t t = 1U; t < 1000U; t++) {
        const struct euc_supervisor_out out = tick(&state, 0U, t, 0.0F);
        TEST_ASSERT_FALSE(out.feed_watchdog);
        TEST_ASSERT_FALSE(out.main_loop_stalled);
    }
}

static void test_fed_by_a_tick_with_foc_and_a_valid_command(void)
{
    struct euc_supervisor_state state;
    euc_supervisor_init(&state);
    TEST_ASSERT_TRUE(tick(&state, 1U, 24U, 500.0F).feed_watchdog);
    TEST_ASSERT_TRUE(tick(&state, 2U, 48U, -1050.0F).feed_watchdog);
    TEST_ASSERT_FALSE(tick(&state, 3U, 48U, 0.0F).feed_watchdog); /* the FOC did not run */
    TEST_ASSERT_TRUE(tick(&state, 4U, 72U, 0.0F).feed_watchdog);
    TEST_ASSERT_FALSE(tick(&state, 5U, 96U, NAN).feed_watchdog);
    TEST_ASSERT_FALSE(tick(&state, 6U, 120U, INFINITY).feed_watchdog);
    TEST_ASSERT_FALSE(tick(&state, 7U, 144U, 1050.5F).feed_watchdog);
    TEST_ASSERT_TRUE(tick(&state, 8U, 168U, 1050.0F).feed_watchdog);
}

static void test_a_stalled_main_loop_latches_limp_not_a_reset(void)
{
    struct euc_supervisor_state state;
    euc_supervisor_init(&state);
    uint32_t foc = 0U;
    /* passes keep the counter at zero */
    for (uint32_t t = 0U; t < 1000U; t++) {
        foc += 24U;
        TEST_ASSERT_FALSE(tick(&state, t + 1U, foc, 0.0F).main_loop_stalled);
    }
    /* the main loop stops: 248 quiet ticks are allowed, the next one latches */
    for (uint32_t t = 0U; t < EUC_SUPERVISOR_STALL_TICKS; t++) {
        foc += 24U;
        const struct euc_supervisor_out quiet = tick(&state, 1000U, foc, 0.0F);
        TEST_ASSERT_FALSE(quiet.main_loop_stalled);
        TEST_ASSERT_TRUE(quiet.feed_watchdog); /* the tick and the FOC still run: no reset */
    }
    foc += 24U;
    const struct euc_supervisor_out out = tick(&state, 1000U, foc, 0.0F);
    TEST_ASSERT_TRUE(out.main_loop_stalled);
    TEST_ASSERT_TRUE(out.feed_watchdog);
    /* latched, even when the main loop comes back */
    foc += 24U;
    TEST_ASSERT_TRUE(tick(&state, 1001U, foc, 0.0F).main_loop_stalled);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_compat_never_feeds_or_latches);
    RUN_TEST(test_fed_by_a_tick_with_foc_and_a_valid_command);
    RUN_TEST(test_a_stalled_main_loop_latches_limp_not_a_reset);
    return UNITY_END();
}
