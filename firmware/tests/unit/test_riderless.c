/*
 * Unit tests for the riderless detector (core/safety/riderless.c): when an empty wheel is suspected, what a
 * rider looks like, what keeps the detector off, and when it releases (clean); and that the stock profile
 * never suspects (quirk RIDERLESS_UNDETECTED).
 */
#include <stdbool.h>
#include <stdint.h>

#include "euc/safety/riderless.h"
#include "euc/sil.h"
#include "unity.h"

#define TICKS_PER_S 496U
#define QUIET_TICKS 298U /* 0.6 s */
#define TICK_S      0.002016F

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

static struct euc_riderless_in riding(float speed_m_per_s, int32_t current_01a)
{
    const struct euc_riderless_in in = {true, false, speed_m_per_s, current_01a};
    return in;
}

/* Runs `ticks` ticks with the same inputs; returns the tick (from 1) of the first suspicion, or 0. */
static uint32_t run(struct euc_riderless_state *state, const struct euc_riderless_in *in, uint32_t ticks)
{
    uint32_t first = 0U;
    for (uint32_t t = 1U; t <= ticks; t++) {
        if (euc_riderless_tick(state, in) && (first == 0U)) {
            first = t;
        }
    }
    return first;
}

/* A rider at 15 km/h (11 A) for 5 s: settled filters, past the start-up wait. */
static void ride_with_rider(struct euc_riderless_state *state)
{
    euc_riderless_init(state);
    const struct euc_riderless_in rider = riding(4.2F, 110);
    TEST_ASSERT_EQUAL_UINT32(0U, run(state, &rider, 5U * TICKS_PER_S));
}

static void test_compat_never_suspects(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
    struct euc_riderless_state state;
    euc_riderless_init(&state);
    const struct euc_riderless_in empty = riding(4.2F, 0);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &empty, 10U * TICKS_PER_S));
    TEST_ASSERT_EQUAL_UINT8(0U, state.triggers);
}

static void test_an_empty_wheel_is_suspected_within_a_second(void)
{
    struct euc_riderless_state state;
    ride_with_rider(&state);
    const struct euc_riderless_in empty = riding(4.2F, 5);
    const uint32_t first = run(&state, &empty, 2U * TICKS_PER_S);
    /* the 0.3 s current filter falls from 11 A below 3.5 A in about 0.34 s, then 0.6 s of hold */
    TEST_ASSERT_GREATER_THAN_UINT32(QUIET_TICKS + (TICKS_PER_S / 4U), first);
    TEST_ASSERT_LESS_THAN_UINT32(TICKS_PER_S, first);
    TEST_ASSERT_TRUE(state.suspected);
    TEST_ASSERT_EQUAL_UINT8(1U, state.triggers);
}

static void test_never_in_the_first_3_s_of_balancing(void)
{
    struct euc_riderless_state state;
    euc_riderless_init(&state);
    const struct euc_riderless_in rider = riding(4.2F, 110);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &rider, TICKS_PER_S));
    /* the load drops after 1 s: the 1.5 s window ends before the 3 s start-up wait */
    const struct euc_riderless_in empty = riding(4.2F, 0);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &empty, 5U * TICKS_PER_S));
}

static void test_no_load_drop_no_suspicion(void)
{
    struct euc_riderless_state state;
    euc_riderless_init(&state);
    const struct euc_riderless_in empty = riding(4.2F, 0);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &empty, 10U * TICKS_PER_S)); /* never loaded */

    /* a rider who coasts: the current passes zero slowly (11 A to -6 A in 4 s) at an even speed */
    ride_with_rider(&state);
    for (uint32_t t = 0U; t < (4U * TICKS_PER_S); t++) {
        const int32_t current = 110 - (int32_t)((170U * t) / (4U * TICKS_PER_S));
        const struct euc_riderless_in coasting = riding(4.2F, current);
        TEST_ASSERT_FALSE(euc_riderless_tick(&state, &coasting));
    }
    TEST_ASSERT_EQUAL_UINT8(0U, state.triggers);
}

static void test_a_rider_cruising_or_braking_is_not_suspected(void)
{
    struct euc_riderless_state state;
    ride_with_rider(&state);
    const struct euc_riderless_in cruising = riding(5.5F, 90);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &cruising, 20U * TICKS_PER_S));
    const struct euc_riderless_in braking = riding(5.5F, -150);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &braking, 2U * TICKS_PER_S));
    const struct euc_riderless_in backward = riding(-2.0F, -60);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &backward, 5U * TICKS_PER_S));
}

static void test_slowing_down_with_no_current_is_not_suspected(void)
{
    struct euc_riderless_state state;
    ride_with_rider(&state);
    /* a rider who stops pushing: no current, the wheel slows at 0.8 m/s^2 from 15 km/h to 1.5 m/s */
    float speed = 4.2F;
    while (speed > 1.5F) {
        const struct euc_riderless_in coasting = riding(speed, 0);
        TEST_ASSERT_FALSE(euc_riderless_tick(&state, &coasting));
        speed -= 0.8F * TICK_S;
    }
    TEST_ASSERT_EQUAL_UINT8(0U, state.triggers);
}

static void test_never_at_walking_speed_or_with_another_limit(void)
{
    struct euc_riderless_state state;
    ride_with_rider(&state);
    const struct euc_riderless_in walking = riding(0.9F, 0);
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &walking, 5U * TICKS_PER_S));

    ride_with_rider(&state);
    struct euc_riderless_in limited = riding(4.2F, 0);
    limited.other_limit = true;
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &limited, 5U * TICKS_PER_S));

    ride_with_rider(&state);
    struct euc_riderless_in stopped = riding(4.2F, 0);
    stopped.balancing = false;
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &stopped, 5U * TICKS_PER_S));
    TEST_ASSERT_EQUAL_UINT8(0U, state.triggers);
}

static void test_released_by_a_rider_or_by_stopping(void)
{
    struct euc_riderless_state state;
    ride_with_rider(&state);
    const struct euc_riderless_in empty = riding(4.2F, 0);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, run(&state, &empty, 2U * TICKS_PER_S));

    /* the current swings both ways (a weaving empty wheel): no release */
    for (uint32_t t = 0U; t < (2U * TICKS_PER_S); t++) {
        const struct euc_riderless_in swinging = riding(4.2F, ((t / 50U) % 2U) == 0U ? 120 : -120);
        TEST_ASSERT_TRUE(euc_riderless_tick(&state, &swinging));
    }

    /* a rider leans against the tiltback: 12 A releases within 0.4 s */
    const struct euc_riderless_in leaning = riding(4.2F, 120);
    uint32_t held = 0U;
    while (euc_riderless_tick(&state, &leaning)) {
        held++;
    }
    TEST_ASSERT_LESS_THAN_UINT32(TICKS_PER_S * 4U / 10U, held);

    /* the rider rides on, steps off again: suspected again, then the wheel slows below 1 m/s */
    TEST_ASSERT_EQUAL_UINT32(0U, run(&state, &leaning, 2U * TICKS_PER_S));
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, run(&state, &empty, 3U * TICKS_PER_S));
    TEST_ASSERT_EQUAL_UINT8(2U, state.triggers);
    const struct euc_riderless_in slow = riding(0.5F, 0);
    (void)run(&state, &slow, TICKS_PER_S);
    TEST_ASSERT_FALSE(state.suspected);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_compat_never_suspects);
    RUN_TEST(test_an_empty_wheel_is_suspected_within_a_second);
    RUN_TEST(test_never_in_the_first_3_s_of_balancing);
    RUN_TEST(test_no_load_drop_no_suspicion);
    RUN_TEST(test_a_rider_cruising_or_braking_is_not_suspected);
    RUN_TEST(test_slowing_down_with_no_current_is_not_suspected);
    RUN_TEST(test_never_at_walking_speed_or_with_another_limit);
    RUN_TEST(test_released_by_a_rider_or_by_stopping);
    return UNITY_END();
}
