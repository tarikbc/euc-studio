/*
 * Unit tests for the power latch, the buttons (core/power/) and the light (core/lights/).
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/lights/lights.h"
#include "euc/power/power.h"
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

/* Power-on hold ---------------------------------------------------------------------------------------- */

static uint32_t polls_until_decision(struct euc_power_boot_state *state, bool pressed,
                                     enum euc_power_boot_decision *d)
{
    for (uint32_t poll = 1U; poll <= 5000U; poll++) {
        *d = euc_power_boot_poll(state, pressed, 0);
        if (*d != EUC_POWER_BOOT_WAIT) {
            return poll;
        }
    }
    return 0U;
}

static void test_cold_start_needs_a_held_button(void)
{
    struct euc_power_boot_state state;
    enum euc_power_boot_decision decision;
    euc_power_boot_init(&state, 1U);
    TEST_ASSERT_EQUAL_UINT16(2U, euc_power_boot_poll_ms(&state));
    TEST_ASSERT_EQUAL_UINT32(221U, polls_until_decision(&state, true, &decision));
    TEST_ASSERT_EQUAL(EUC_POWER_BOOT_STAY_ON, decision);

    euc_power_boot_init(&state, 1U);
    TEST_ASSERT_EQUAL_UINT32(251U, polls_until_decision(&state, false, &decision));
    TEST_ASSERT_EQUAL(EUC_POWER_BOOT_TURN_OFF, decision);
}

static void test_cold_start_restarts_when_the_wheel_turns(void)
{
    struct euc_power_boot_state state;
    euc_power_boot_init(&state, 1U);
    for (int16_t poll = 1; poll <= 200; poll++) {
        TEST_ASSERT_EQUAL(EUC_POWER_BOOT_WAIT, euc_power_boot_poll(&state, true, 0));
    }
    TEST_ASSERT_EQUAL(EUC_POWER_BOOT_WAIT, euc_power_boot_poll(&state, true, 1)); /* moved: count again */
    enum euc_power_boot_decision decision = EUC_POWER_BOOT_WAIT;
    uint32_t polls = 0U;
    while (decision == EUC_POWER_BOOT_WAIT) {
        decision = euc_power_boot_poll(&state, true, 1);
        polls++;
    }
    TEST_ASSERT_EQUAL_UINT32(221U, polls);
    TEST_ASSERT_EQUAL(EUC_POWER_BOOT_STAY_ON, decision);
}

static void test_warm_start_waits_for_a_still_wheel(void)
{
    struct euc_power_boot_state state;
    enum euc_power_boot_decision decision;
    euc_power_boot_init(&state, 10U);
    TEST_ASSERT_EQUAL_UINT16(10U, euc_power_boot_poll_ms(&state));
    TEST_ASSERT_EQUAL_UINT32(4U, polls_until_decision(&state, false, &decision));
    TEST_ASSERT_EQUAL(EUC_POWER_BOOT_STAY_ON, decision);
}

/* RTC-tick logic --------------------------------------------------------------------------------------- */

static struct euc_power_in idle_power_input(void)
{
    struct euc_power_in in;
    (void)memset(&in, 0, sizeof(in));
    in.lock_state = 1U;
    return in;
}

static void test_long_press_powers_off(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    euc_power_init(&state);

    in.power_button_pressed = true; /* held since power-on: ignored */
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_power_tick(&state, &in, &out);
        TEST_ASSERT_FALSE(out.power_off);
    }
    in.power_button_pressed = false;
    euc_power_tick(&state, &in, &out);

    in.power_button_pressed = true;
    for (unsigned tick = 1U; tick <= 401U; tick++) {
        euc_power_tick(&state, &in, &out);
        if (tick == 400U) {
            TEST_ASSERT_EQUAL_UINT8(1U, out.beep_count);
            TEST_ASSERT_EQUAL_UINT16(50U, out.beep_ticks[0]);
        }
        TEST_ASSERT_EQUAL(tick == 401U, out.power_off);
    }
    TEST_ASSERT_TRUE(out.save_settings);
    TEST_ASSERT_TRUE(out.hard_cut);
    TEST_ASSERT_TRUE(state.powering_off);
}

static void test_long_press_needs_standstill(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    euc_power_init(&state);
    in.power_button_pressed = false;
    euc_power_tick(&state, &in, &out);
    in.power_button_pressed = true;
    in.speed_cm_per_s = 51;
    for (unsigned tick = 0U; tick < 1000U; tick++) {
        euc_power_tick(&state, &in, &out);
        TEST_ASSERT_FALSE(out.power_off);
    }
}

static void press(struct euc_power_state *state, struct euc_power_in *in, unsigned ticks, uint32_t *beeps,
                  struct euc_power_out *out)
{
    in->power_button_pressed = true;
    for (unsigned tick = 0U; tick < ticks; tick++) {
        euc_power_tick(state, in, out);
        *beeps += out->beep_count;
        in->lock_state = out->lock_state;
    }
    in->power_button_pressed = false;
    for (unsigned tick = 0U; tick < 50U; tick++) {
        euc_power_tick(state, in, out);
        in->lock_state = out->lock_state;
    }
}

static void test_presses_held_from_power_on_do_not_count(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    uint32_t beeps = 0U;
    euc_power_init(&state);
    in.handle_button_pressed = true;
    press(&state, &in, 40U, &beeps, &out);
    TEST_ASSERT_EQUAL_UINT32(0U, beeps);
    press(&state, &in, 40U, &beeps, &out);
    TEST_ASSERT_EQUAL_UINT32(1U, beeps);
}

static void test_lock_gesture_toggles_the_transport_lock(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    uint32_t beeps = 0U;
    euc_power_init(&state);
    in.handle_button_pressed = true;
    euc_power_tick(&state, &in, &out); /* the power button seen released once: presses count from now */
    for (unsigned n = 0U; n < 4U; n++) {
        press(&state, &in, 40U, &beeps, &out);
    }
    TEST_ASSERT_EQUAL_UINT8(1U, in.lock_state);
    TEST_ASSERT_EQUAL_UINT32(4U, beeps);
    in.power_button_pressed = true;
    bool cut = false;
    for (unsigned tick = 0U; tick < 40U; tick++) {
        euc_power_tick(&state, &in, &out);
        cut = cut || out.hard_cut;
        in.lock_state = out.lock_state;
    }
    TEST_ASSERT_EQUAL_UINT8(3U, in.lock_state);
    TEST_ASSERT_TRUE(cut);

    /* Release the handle button, start again: five more presses unlock (to 1). */
    in.power_button_pressed = false;
    in.handle_button_pressed = false;
    for (unsigned tick = 0U; tick < 30U; tick++) {
        euc_power_tick(&state, &in, &out);
    }
    in.handle_button_pressed = true;
    for (unsigned n = 0U; n < 5U; n++) {
        press(&state, &in, 40U, &beeps, &out);
    }
    TEST_ASSERT_EQUAL_UINT8(1U, in.lock_state);
}

static void test_lock_gesture_resets_after_a_long_gap(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    uint32_t beeps = 0U;
    euc_power_init(&state);
    in.handle_button_pressed = true;
    euc_power_tick(&state, &in, &out);
    for (unsigned n = 0U; n < 4U; n++) {
        press(&state, &in, 40U, &beeps, &out);
    }
    for (unsigned tick = 0U; tick < 2600U; tick++) { /* more than 5 s without a press */
        euc_power_tick(&state, &in, &out);
    }
    press(&state, &in, 40U, &beeps, &out);
    TEST_ASSERT_EQUAL_UINT8(1U, in.lock_state);
    TEST_ASSERT_EQUAL_UINT32(4U, beeps); /* blocked until the handle button is released */
}

static void test_auto_power_off(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    euc_power_init(&state);
    uint32_t beeps = 0U;
    uint32_t tick = 0U;
    bool off = false;
    while ((!off) && (tick < 8000000U)) {
        in.hall_position = (tick < 1000U) ? (int16_t)(tick / 100U) : 10; /* moves for 2 s, then rests */
        euc_power_tick(&state, &in, &out);
        beeps += out.beep_count;
        off = out.power_off;
        tick++;
    }
    TEST_ASSERT_TRUE(off);
    TEST_ASSERT_TRUE(out.save_settings);
    TEST_ASSERT_EQUAL_UINT32(8U, beeps);
    /* 2 s of motion, then 7213 idle seconds of 501 ticks: power-off at warning 13. */
    TEST_ASSERT_UINT32_WITHIN(1100U, 7213U * 501U + 1000U, tick);
}

static void test_auto_off_countdown(void)
{
    struct euc_power_state state;
    struct euc_power_out out;
    struct euc_power_in in = idle_power_input();
    euc_power_init(&state);
    for (unsigned tick = 0U; tick < 502U * 10U; tick++) {
        euc_power_tick(&state, &in, &out);
    }
    TEST_ASSERT_EQUAL_UINT16(7190U, out.auto_off_remaining_s);
    in.hall_position = 1;
    euc_power_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT16(7200U, out.auto_off_remaining_s);
    in.drive_disabled = true; /* disabled: motion does not count */
    for (unsigned tick = 0U; tick < 502U; tick++) {
        in.hall_position = (int16_t)tick;
        euc_power_tick(&state, &in, &out);
    }
    TEST_ASSERT_EQUAL_UINT16(7199U, out.auto_off_remaining_s);
}

/* Light -------------------------------------------------------------------------------------------------- */

static struct euc_lights_in idle_light_input(void)
{
    struct euc_lights_in in;
    (void)memset(&in, 0, sizeof(in));
    in.time_base = 1U;
    return in;
}

static void light_press(struct euc_lights_state *state, struct euc_lights_in *in, unsigned ticks,
                        struct euc_lights_out *out, bool *wm_changed)
{
    in->power_button_pressed = true;
    for (unsigned tick = 0U; tick < ticks; tick++) {
        euc_lights_tick(state, in, out);
    }
    in->power_button_pressed = false;
    euc_lights_tick(state, in, out);
    *wm_changed = out->wm_changed;
}

static void test_light_button_cycles_modes(void)
{
    struct euc_lights_state state;
    struct euc_lights_out out;
    struct euc_lights_in in = idle_light_input();
    bool wm_changed = false;
    euc_lights_init(&state, 0U, 0U);
    light_press(&state, &in, 300U, &out, &wm_changed); /* held since power-on: not a short press */
    TEST_ASSERT_EQUAL_UINT8(EUC_LIGHT_OFF, out.light_state);

    light_press(&state, &in, 21U, &out, &wm_changed); /* hold 11 ticks after the debounce */
    TEST_ASSERT_EQUAL_UINT8(EUC_LIGHT_ON, out.light_state);
    light_press(&state, &in, 229U, &out, &wm_changed);
    TEST_ASSERT_EQUAL_UINT8(EUC_LIGHT_STROBE, out.light_state);
    light_press(&state, &in, 100U, &out, &wm_changed);
    TEST_ASSERT_EQUAL_UINT8(EUC_LIGHT_OFF, out.light_state);
    TEST_ASSERT_TRUE(wm_changed); /* back to off: the strip pattern toggles */
    TEST_ASSERT_EQUAL_UINT8(1U, out.strip_pattern);

    light_press(&state, &in, 20U, &out, &wm_changed);  /* too short */
    light_press(&state, &in, 230U, &out, &wm_changed); /* too long */
    TEST_ASSERT_EQUAL_UINT8(EUC_LIGHT_OFF, out.light_state);
}

static void test_light_brightness_follows_speed(void)
{
    struct euc_lights_state state;
    struct euc_lights_out out;
    struct euc_lights_in in = idle_light_input();
    euc_lights_init(&state, 0U, 0U);
    euc_lights_set_mode(&state, EUC_LIGHT_ON);
    in.speed_dm_per_s = 30;
    for (unsigned tick = 0U; tick < 300U; tick++) {
        euc_lights_tick(&state, &in, &out);
    }
    TEST_ASSERT_EQUAL_UINT8(40U, state.duty);
    in.speed_dm_per_s = 0;
    for (unsigned tick = 0U; tick < 20000U; tick++) {
        in.time_base = (uint16_t)(tick + 1U);
        euc_lights_tick(&state, &in, &out);
    }
    TEST_ASSERT_EQUAL_UINT8(12U, state.duty);
}

static void test_strobe_and_pwm(void)
{
    struct euc_lights_state state;
    struct euc_lights_out out;
    struct euc_lights_in in = idle_light_input();
    euc_lights_init(&state, 2U, 0U); /* BKP DR11 = 2 */
    in.time_base = 50U;
    euc_lights_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT8(0U, state.duty);
    in.time_base = 51U;
    euc_lights_tick(&state, &in, &out);
    TEST_ASSERT_EQUAL_UINT8(24U, state.duty);
    uint32_t high = 0U;
    for (unsigned step = 0U; step < 41U; step++) {
        high += euc_lights_pwm_step(&state) ? 1U : 0U;
    }
    TEST_ASSERT_EQUAL_UINT32(23U, high);
}

static void test_lock_turns_the_light_off(void)
{
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_lights_state state;
        struct euc_lights_out out;
        struct euc_lights_in in = idle_light_input();
        euc_lights_init(&state, 1U, 1U);
        euc_lights_tick(&state, &in, &out);
        TEST_ASSERT_EQUAL_UINT8(8U, state.duty);
        in.transport_locked = true;
        euc_lights_tick(&state, &in, &out);
        TEST_ASSERT_EQUAL_UINT8(EUC_LIGHT_OFF, out.light_state);
        TEST_ASSERT_EQUAL_UINT8(0U, out.strip_pattern);
        TEST_ASSERT_EQUAL_UINT8((profiles[p] == EUC_SIL_PROFILE_COMPAT) ? 8U : 0U, state.duty);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_cold_start_needs_a_held_button);
    RUN_TEST(test_cold_start_restarts_when_the_wheel_turns);
    RUN_TEST(test_warm_start_waits_for_a_still_wheel);
    RUN_TEST(test_long_press_powers_off);
    RUN_TEST(test_long_press_needs_standstill);
    RUN_TEST(test_presses_held_from_power_on_do_not_count);
    RUN_TEST(test_lock_gesture_toggles_the_transport_lock);
    RUN_TEST(test_lock_gesture_resets_after_a_long_gap);
    RUN_TEST(test_auto_power_off);
    RUN_TEST(test_auto_off_countdown);
    RUN_TEST(test_light_button_cycles_modes);
    RUN_TEST(test_light_brightness_follows_speed);
    RUN_TEST(test_strobe_and_pwm);
    RUN_TEST(test_lock_turns_the_light_off);
    return UNITY_END();
}
