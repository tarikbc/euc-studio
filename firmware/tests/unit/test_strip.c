/*
 * Unit tests for the LED strip patterns (core/lights/strip.c): ring motion, brake flash, standstill
 * breathing, pattern 0 comets and turn flashes, pattern 1 colour wheel. LEDs are 1-based as in the header;
 * pixels[led - 1].
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "euc/lights/strip.h"
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

/* One RTC tick at time base t (the tick part sees t - 1, the render t). */
static void tick(struct euc_strip_state *state, uint16_t t, uint8_t display_speed, int16_t speed_mm_per_s,
                 float roll_deg, uint8_t pattern)
{
    const struct euc_strip_tick_in tick_in = {(uint16_t)(t - 1U), display_speed, speed_mm_per_s};
    euc_strip_tick(state, &tick_in);
    const struct euc_strip_render_in render_in = {t, display_speed, roll_deg, pattern};
    euc_strip_render(state, &render_in);
}

static void ticks(struct euc_strip_state *state, uint16_t *t, uint32_t count, uint8_t display_speed,
                  int16_t speed_mm_per_s, float roll_deg, uint8_t pattern)
{
    for (uint32_t i = 0U; i < count; i++) {
        (*t)++;
        tick(state, *t, display_speed, speed_mm_per_s, roll_deg, pattern);
    }
}

static void assert_led(const struct euc_strip_state *state, uint32_t led, uint8_t red, uint8_t green,
                       uint8_t blue)
{
    const struct euc_strip_pixel p = state->pixels[led - 1U];
    TEST_ASSERT_EQUAL_UINT8(red, p.red);
    TEST_ASSERT_EQUAL_UINT8(green, p.green);
    TEST_ASSERT_EQUAL_UINT8(blue, p.blue);
}

static void test_nothing_drawn_before_the_first_update(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    tick(&state, 3U, 10U, 0, 0.0F, 0U); /* tick part sees 2: no update yet */
    TEST_ASSERT_FALSE(state.update);
    TEST_ASSERT_EQUAL_UINT32(0U, euc_strip_take_changed(&state));
    tick(&state, 6U, 10U, 0, 0.0F, 0U); /* sees 5 */
    TEST_ASSERT_TRUE(state.update);
    TEST_ASSERT_NOT_EQUAL(0U, euc_strip_take_changed(&state));
    TEST_ASSERT_EQUAL_UINT32(0U, euc_strip_take_changed(&state)); /* cleared */
}

static void test_ring_moves_with_speed_and_direction(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    /* 10 km/h: a step every 100 - 30 = 70 ticks, forward above +4 mm/s */
    for (uint16_t t = 1U; t <= 700U; t++) {
        const struct euc_strip_tick_in in = {t, 10U, 5};
        euc_strip_tick(&state, &in);
    }
    TEST_ASSERT_EQUAL_INT16(10, state.ring);
    /* 4 mm/s: rests */
    for (uint16_t t = 701U; t <= 1400U; t++) {
        const struct euc_strip_tick_in in = {t, 10U, 4};
        euc_strip_tick(&state, &in);
    }
    TEST_ASSERT_EQUAL_INT16(10, state.ring);
    /* 40 km/h: every 10 ticks; past 72 it wraps to 0 */
    for (uint16_t t = 1401U; t <= 2030U; t++) {
        const struct euc_strip_tick_in in = {t, 40U, 1000};
        euc_strip_tick(&state, &in);
    }
    TEST_ASSERT_EQUAL_INT16(10 + 63 - 73, state.ring);
    /* backward from 0: straight to 73, then down */
    state.ring = 0;
    const struct euc_strip_tick_in back = {2031U, 40U, -5};
    euc_strip_tick(&state, &back);
    TEST_ASSERT_EQUAL_INT16(73, state.ring);
    const struct euc_strip_tick_in back_step = {2040U, 40U, -5};
    euc_strip_tick(&state, &back_step);
    TEST_ASSERT_EQUAL_INT16(72, state.ring);
}

static void test_brake_flash_after_a_3_kmh_drop(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    uint16_t t = 0U;
    ticks(&state, &t, 301U, 20U, 5000, 0.0F, 0U); /* speed noted at 150 and 300: 20 */
    TEST_ASSERT_EQUAL_UINT8(3U, state.brake_count);
    ticks(&state, &t, 1U, 17U, 5000, 0.0F, 0U);
    TEST_ASSERT_EQUAL_UINT8(40U, state.brake_count);
    /* Flash: LEDs 1..24 red 100 at time base % 50 == 0, dark at 25 */
    while ((t % 50U) != 0U) {
        ticks(&state, &t, 1U, 17U, 5000, 0.0F, 0U);
    }
    for (uint32_t led = 1U; led <= 24U; led++) {
        assert_led(&state, led, 100U, 0U, 0U);
    }
    ticks(&state, &t, 25U, 17U, 5000, 0.0F, 0U);
    for (uint32_t led = 1U; led <= 24U; led++) {
        assert_led(&state, led, 0U, 0U, 0U);
    }
    assert_led(&state, 25U, 0U, 0U, 0U); /* never written */
    /* The next note (17) ends it: down by 1 every 10 ticks to 3 */
    while (state.brake_count >= 4U) {
        ticks(&state, &t, 1U, 17U, 5000, 0.0F, 0U);
    }
    TEST_ASSERT_TRUE(t < 1000U);
}

static void test_breathing_at_a_standstill(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_strip_state state;
        euc_strip_init(&state);
        uint16_t t = 0U;
        ticks(&state, &t, 400U, 0U, 0, 0.0F, 0U); /* 40 counts of 10 ticks under 2 km/h */
        TEST_ASSERT_EQUAL_UINT8(0U, state.standstill);
        ticks(&state, &t, 1U, 0U, 0, 0.0F, 0U); /* the 41st */
        TEST_ASSERT_EQUAL_UINT8(1U, state.standstill);
        const uint8_t before = state.breath;
        ticks(&state, &t, 10U, 0U, 0, 0.0F, 0U);
        if (profiles[p] == EUC_SIL_PROFILE_COMPAT) {
            TEST_ASSERT_EQUAL_UINT8(before + 12U, state.breath); /* 12 passes in every 10th tick */
        } else {
            TEST_ASSERT_EQUAL_UINT8(before + 10U, state.breath); /* 1 per tick */
        }
        for (uint32_t led = 1U; led <= 24U; led++) {
            assert_led(&state, led, state.breath, 0U, 0U);
        }
        /* Up to 131, then down to 0, then up again */
        uint8_t top = 0U;
        for (uint32_t i = 0U; i < 400U; i++) {
            ticks(&state, &t, 1U, 0U, 0, 0.0F, 0U);
            top = (state.breath > top) ? state.breath : top;
        }
        TEST_ASSERT_TRUE(top <= 131U);
        TEST_ASSERT_TRUE(top >= 120U);
    }
}

static void test_pattern_0_comets(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    tick(&state, 6U, 15U, 5000, 0.0F, 0U); /* ring 0 */
    /* Position i + 1 has brightness 1 2 4 16 40 130 1 2 ...; position 1 = LED 8 (A) and 24 (B) */
    assert_led(&state, 8U, 1U, 0U, 0U);
    assert_led(&state, 24U, 1U, 0U, 0U);
    assert_led(&state, 1U, 130U, 0U, 0U);  /* position 6 on side A */
    assert_led(&state, 19U, 130U, 0U, 0U); /* position 6 on side B */
    assert_led(&state, 7U, 130U, 0U, 0U);  /* position 12 */
    assert_led(&state, 13U, 130U, 0U, 0U);
    assert_led(&state, 25U, 0U, 0U, 0U);
}

static void test_pattern_0_turn_flash(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    tick(&state, 146U, 15U, 5000, 0.0F, 0U); /* first update */
    tick(&state, 150U, 15U, 5000, 5.0F, 0U); /* roll above +4: LEDs 1..12 amber, side B red */
    for (uint32_t led = 1U; led <= 12U; led++) {
        assert_led(&state, led, 100U, 78U, 0U);
    }
    assert_led(&state, 24U, 1U, 0U, 0U);
    tick(&state, 249U, 15U, 5000, 5.0F, 0U);
    assert_led(&state, 1U, 100U, 78U, 0U);
    tick(&state, 250U, 15U, 5000, 5.0F, 0U); /* % 150 == 100: dark */
    assert_led(&state, 1U, 0U, 0U, 0U);

    euc_strip_init(&state);
    tick(&state, 146U, 15U, 5000, 0.0F, 0U);
    tick(&state, 150U, 15U, 5000, -5.0F, 0U); /* roll below -4: LEDs 13..24 amber, side A red */
    assert_led(&state, 13U, 100U, 78U, 0U);
    assert_led(&state, 24U, 100U, 78U, 0U);
    assert_led(&state, 8U, 1U, 0U, 0U);
    tick(&state, 225U, 15U, 5000, -5.0F, 0U); /* % 150 == 75: dark */
    assert_led(&state, 13U, 0U, 0U, 0U);
}

static void test_pattern_1_colour_wheel(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    tick(&state, 6U, 15U, 5000, 0.0F, 1U); /* ring 0, brightness 100 */
    /* Step 1 from red: blue rises to 21; step 12: 255 (magenta) */
    assert_led(&state, 8U, 100U, 0U, 8U);    /* position 1 */
    assert_led(&state, 24U, 100U, 0U, 8U);   /* position 1, side B */
    assert_led(&state, 7U, 100U, 0U, 100U);  /* position 12 */
    assert_led(&state, 13U, 100U, 0U, 100U); /* position 12, side B */
    /* 72 steps make one turn of the wheel: back to red, so the next render draws the same */
    TEST_ASSERT_EQUAL_UINT8(255U, state.wheel_red);
    TEST_ASSERT_EQUAL_UINT8(0U, state.wheel_green);
    TEST_ASSERT_EQUAL_UINT8(0U, state.wheel_blue);
    (void)euc_strip_take_changed(&state);
    tick(&state, 7U, 15U, 5000, 0.0F, 1U);
    TEST_ASSERT_EQUAL_UINT32(0U, euc_strip_take_changed(&state));
}

static void test_pattern_1_turn_flash_is_on_the_other_side(void)
{
    struct euc_strip_state state;
    euc_strip_init(&state);
    tick(&state, 146U, 15U, 5000, 0.0F, 1U);
    tick(&state, 150U, 15U, 5000, 5.0F, 1U); /* roll above +4: LEDs 13..24 */
    assert_led(&state, 13U, 100U, 78U, 0U);
    assert_led(&state, 1U, 100U, 0U, 49U); /* position 6 on side A: blue 126 at brightness 100 */
    tick(&state, 250U, 15U, 5000, 5.0F, 1U);
    assert_led(&state, 13U, 0U, 0U, 0U);
    tick(&state, 300U, 1U, 5000, 5.0F, 1U); /* under 2 km/h: no flash, the wheel colour */
    assert_led(&state, 13U, 100U, 0U, 100U);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nothing_drawn_before_the_first_update);
    RUN_TEST(test_ring_moves_with_speed_and_direction);
    RUN_TEST(test_brake_flash_after_a_3_kmh_drop);
    RUN_TEST(test_breathing_at_a_standstill);
    RUN_TEST(test_pattern_0_comets);
    RUN_TEST(test_pattern_0_turn_flash);
    RUN_TEST(test_pattern_1_colour_wheel);
    RUN_TEST(test_pattern_1_turn_flash_is_on_the_other_side);
    return UNITY_END();
}
