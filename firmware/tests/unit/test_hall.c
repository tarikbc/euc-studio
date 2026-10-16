/*
 * Unit tests for the hall sensors (core/motor/hall.c): glitch filter, sector angles, direction and step
 * count, period speed, interpolation, faults, 20 ms windows and odometer. Lines: bit 0 = A, 1 = B, 2 = C.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "euc/motor/hall.h"
#include "euc/sil.h"
#include "unity.h"

#define F_CURRENT_MODE 12000U /* calls at 24 kHz */
#define F_VOLTAGE_MODE 10000U /* calls at 20 kHz */

/* Forward (+1) order of the hall states. */
static const uint8_t forward_order[6] = {5U, 1U, 3U, 2U, 6U, 4U};

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

static struct euc_hall_out run(struct euc_hall_state *state, uint8_t lines, uint32_t calls, uint16_t f)
{
    struct euc_hall_out out = {false, false};
    struct euc_hall_out any = {false, false};
    for (uint32_t i = 0U; i < calls; i++) {
        euc_hall_step(state, lines, f, &out);
        any.odometer_step = any.odometer_step || out.odometer_step;
        any.fault_raised = any.fault_raised || out.fault_raised;
    }
    return any;
}

/* Power-on, then the wheel resting in state 5 long enough to set standstill_seen. */
static void start_at_rest(struct euc_hall_state *state, uint16_t f)
{
    euc_hall_init(state);
    (void)run(state, 5U, 400U, f);
    TEST_ASSERT_TRUE(state->standstill_seen);
}

/* Turn through `edges` sector changes, `period` calls each, forward (+1) or backward (-1). */
static void turn(struct euc_hall_state *state, int direction, uint32_t edges, uint32_t period, uint16_t f)
{
    size_t index = 0U; /* state 5 */
    for (size_t i = 0U; i < 6U; i++) {
        if (forward_order[i] == state->sector) {
            index = i;
        }
    }
    for (uint32_t e = 0U; e < edges; e++) {
        index = (direction > 0) ? ((index + 1U) % 6U) : ((index + 5U) % 6U);
        (void)run(state, forward_order[index], period, f);
    }
}

static void test_sector_centres(void)
{
    const int16_t centre[8] = {0, -0x6AAC, -0x1555, -0x4001, 0x4000, 0x6AAA, 0x1556, 0};
    struct euc_hall_state state;
    start_at_rest(&state, F_CURRENT_MODE);
    for (size_t i = 0U; i < 6U; i++) {
        (void)run(&state, forward_order[i], 300U, F_CURRENT_MODE); /* past the interpolation window */
        TEST_ASSERT_EQUAL_UINT8(forward_order[i], state.sector);
        TEST_ASSERT_EQUAL_INT16(centre[forward_order[i]], state.angle);
        TEST_ASSERT_FALSE(state.interpolating);
    }
}

static void test_glitch_filter_ignores_chatter(void)
{
    struct euc_hall_state state;
    euc_hall_init(&state);
    (void)run(&state, 1U, 10U, F_CURRENT_MODE);  /* A high: state 1 */
    (void)run(&state, 3U, 1U, F_CURRENT_MODE);   /* B rises: state 3, B locked */
    (void)run(&state, 1U, 150U, F_CURRENT_MODE); /* B falls back within 200 calls: ignored */
    TEST_ASSERT_EQUAL_UINT8(3U, state.sector);
    (void)run(&state, 1U, 60U, F_CURRENT_MODE); /* lock over: B's fall is taken */
    TEST_ASSERT_EQUAL_UINT8(1U, state.sector);

    /* A toggle on another line releases the lock at once. */
    euc_hall_init(&state);
    (void)run(&state, 1U, 300U, F_CURRENT_MODE); /* state 1, A's lock time over */
    (void)run(&state, 3U, 1U, F_CURRENT_MODE);   /* B rises: B locked */
    (void)run(&state, 2U, 1U, F_CURRENT_MODE);   /* A falls: state 2, A locked, B released */
    (void)run(&state, 0U, 1U, F_CURRENT_MODE);   /* B falls at once: taken */
    TEST_ASSERT_EQUAL_UINT8(0U, state.sector);
}

static void test_direction_position_and_windows(void)
{
    struct euc_hall_state state;
    start_at_rest(&state, F_CURRENT_MODE);
    const int16_t position = state.position;
    turn(&state, 1, 12U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT8(1, state.direction);
    TEST_ASSERT_EQUAL_INT16(position + 12, state.position);
    /* Turning back toggles the line that toggled last: it waits out its 200-call lock first. */
    (void)run(&state, state.sector, 250U, F_CURRENT_MODE);
    turn(&state, -1, 5U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT8(-1, state.direction);
    TEST_ASSERT_EQUAL_INT16(position + 7, state.position);

    /* Window: 12000 x 4 / 100 + 2 = 482 calls. At one edge every 241 calls, two steps per window. */
    euc_hall_init(&state);
    (void)run(&state, 5U, 482U, F_CURRENT_MODE);
    state.windows = 0U;
    state.window_calls = 0;
    state.window_steps = 0;
    turn(&state, 1, 2U, 241U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT16(2, state.steps_per_window);
}

static void test_11_window_delta(void)
{
    struct euc_hall_state state;
    euc_hall_init(&state);
    (void)run(&state, 5U, 1U, F_CURRENT_MODE);
    state.window_calls = 0;
    state.windows = 0U;
    state.window_steps = 0;
    state.block_steps = 0;
    /* Block 1: 11 windows with one step each (482 calls per edge). Block 2: two steps per window. */
    turn(&state, 1, 11U, 482U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT16(11, state.steps_11_windows);
    turn(&state, 1, 22U, 241U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT16(11, state.steps_11_windows); /* 22 - 11 */
    TEST_ASSERT_EQUAL_INT16(11, state.steps_11_windows_max);
}

static void test_period_speed(void)
{
    struct euc_hall_state state;
    start_at_rest(&state, F_CURRENT_MODE);
    /* 100 calls per edge forward: -202117 / 100 = -2021 mm/s. The filter has a time constant of 1000 calls.
     * Stock truncation stops it once it is within 999 counts of the input: at -1022. */
    turn(&state, 1, 120U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT32(-2021, state.period_speed_raw);
    TEST_ASSERT_EQUAL_INT32(-1022, state.period_speed);
    turn(&state, -1, 120U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT32(2021, state.period_speed_raw);

    /* No edge for 1000 calls: speed 0, the edge counter stops at 1000. */
    (void)run(&state, state.sector, 1100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT32(0, state.period_speed_raw);
    TEST_ASSERT_EQUAL_UINT16(1000U, state.calls_since_edge);
}

static void test_clean_period_speed_filter_reaches_its_input(void)
{
    use_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_hall_state state;
    start_at_rest(&state, F_CURRENT_MODE);
    turn(&state, 1, 120U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_INT_WITHIN(1, -2021, state.period_speed);
}

static void test_period_speed_is_slew_limited(void)
{
    struct euc_hall_state state;
    start_at_rest(&state, F_CURRENT_MODE);
    turn(&state, 1, 6U, 400U, F_CURRENT_MODE);
    state.period_speed_slewed = 0;
    turn(&state, 1, 1U, 10U, F_CURRENT_MODE); /* the edge after 400 calls: raw -505, slewed to -500 */
    TEST_ASSERT_EQUAL_INT32(-500, state.period_speed_slewed);
}

static void test_period_speed_scale_per_profile(void)
{
    struct euc_hall_state state;
    /* Voltage mode, 20 kHz calls, 100 calls per edge. */
    use_profile(EUC_SIL_PROFILE_COMPAT);
    start_at_rest(&state, F_VOLTAGE_MODE);
    turn(&state, 1, 12U, 100U, F_VOLTAGE_MODE);
    TEST_ASSERT_EQUAL_INT32(-2021, state.period_speed_raw); /* 1.2 x the true speed */

    use_profile(EUC_SIL_PROFILE_CLEAN);
    start_at_rest(&state, F_VOLTAGE_MODE);
    turn(&state, 1, 12U, 100U, F_VOLTAGE_MODE);
    TEST_ASSERT_EQUAL_INT32(-1684, state.period_speed_raw); /* 168431 / 100: true mm/s */

    start_at_rest(&state, F_CURRENT_MODE);
    turn(&state, 1, 12U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_INT32(-2021, state.period_speed_raw); /* 24 kHz: same as stock */
}

static void test_no_speed_before_standstill_is_seen(void)
{
    struct euc_hall_state state;
    euc_hall_init(&state);
    (void)run(&state, 5U, 10U, F_CURRENT_MODE);
    turn(&state, 1, 12U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_FALSE(state.standstill_seen);
    TEST_ASSERT_EQUAL_INT32(0, state.period_speed_raw);
}

static void test_interpolation(void)
{
    struct euc_hall_state state;
    start_at_rest(&state, F_CURRENT_MODE);
    /* 100 calls per edge, so one turn = 600 calls and the step is 65536 / 600 = 109 per call. Two turns end
     * in state 5 again. */
    turn(&state, 1, 12U, 100U, F_CURRENT_MODE);
    TEST_ASSERT_EQUAL_UINT8(5U, state.sector);
    TEST_ASSERT_EQUAL_INT16(109, state.angle_step);
    TEST_ASSERT_EQUAL_INT16(-0x1555, state.entry_offset);
    TEST_ASSERT_TRUE(state.interpolating);
    /* Entered at the centre - 30 deg, then 100 steps of 109. */
    TEST_ASSERT_EQUAL_INT16(0x6AAA - 0x1555 + (100 * 109), state.angle);

    /* A slow turn: the progress stops near 60 deg, at most one step beyond 0x2AA9. */
    turn(&state, 1, 1U, 190U, F_CURRENT_MODE);
    TEST_ASSERT_TRUE(state.angle_progress <= (0x2AA9 + 109));
    TEST_ASSERT_TRUE(state.angle_progress > 0x2AA9);
}

static void test_invalid_state_latches_the_fault(void)
{
    struct euc_hall_state state;
    euc_hall_init(&state);
    /* State 0 from power-on: the fault latches after more than 12000 / 8 = 1500 calls. */
    struct euc_hall_out out = run(&state, 0U, 1500U, F_CURRENT_MODE);
    TEST_ASSERT_FALSE(state.fault);
    TEST_ASSERT_FALSE(out.fault_raised);
    out = run(&state, 0U, 1U, F_CURRENT_MODE);
    TEST_ASSERT_TRUE(state.fault);
    TEST_ASSERT_TRUE(out.fault_raised);
    out = run(&state, 0U, 10U, F_CURRENT_MODE);
    TEST_ASSERT_FALSE(out.fault_raised); /* raised once */
    (void)run(&state, 5U, 400U, F_CURRENT_MODE);
    TEST_ASSERT_TRUE(state.fault); /* latched */
}

static void test_invalid_state_angle_runs_on_in_compat_only(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_hall_state state;
        start_at_rest(&state, F_CURRENT_MODE);
        turn(&state, 1, 12U, 100U, F_CURRENT_MODE); /* ends in state 5 (lines A and C) */
        TEST_ASSERT_EQUAL_UINT8(5U, state.sector);
        (void)run(&state, 7U, 1U, F_CURRENT_MODE); /* B rises: state 7 */
        const int16_t first = state.angle;
        (void)run(&state, 7U, 3U, F_CURRENT_MODE);
        if (profiles[p] == EUC_SIL_PROFILE_COMPAT) {
            TEST_ASSERT_NOT_EQUAL(first, state.angle);
        } else {
            TEST_ASSERT_EQUAL_INT16(first, state.angle);
        }
    }
}

static void test_odometer_every_118_changes(void)
{
    struct euc_hall_state state;
    euc_hall_init(&state);
    (void)run(&state, 5U, 300U, F_CURRENT_MODE); /* change 1: 0 -> 5, then past the line locks */
    uint32_t units = 0U;
    size_t index = 0U;
    for (uint32_t change = 2U; change <= 236U; change++) {
        index = (index + 1U) % 6U;
        struct euc_hall_out out;
        euc_hall_step(&state, forward_order[index], F_CURRENT_MODE, &out);
        (void)run(&state, forward_order[index], 50U, F_CURRENT_MODE);
        if (out.odometer_step) {
            units++;
            TEST_ASSERT_TRUE((change == 118U) || (change == 236U));
        }
    }
    TEST_ASSERT_EQUAL_UINT32(2U, units);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sector_centres);
    RUN_TEST(test_glitch_filter_ignores_chatter);
    RUN_TEST(test_direction_position_and_windows);
    RUN_TEST(test_11_window_delta);
    RUN_TEST(test_period_speed);
    RUN_TEST(test_clean_period_speed_filter_reaches_its_input);
    RUN_TEST(test_period_speed_is_slew_limited);
    RUN_TEST(test_period_speed_scale_per_profile);
    RUN_TEST(test_no_speed_before_standstill_is_seen);
    RUN_TEST(test_interpolation);
    RUN_TEST(test_invalid_state_latches_the_fault);
    RUN_TEST(test_invalid_state_angle_runs_on_in_compat_only);
    RUN_TEST(test_odometer_every_118_changes);
    return UNITY_END();
}
