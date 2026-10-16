/*
 * Unit tests for the display content (core/display/) and the service sequences (core/service/).
 */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/display/display.h"
#include "euc/service/service.h"
#include "euc/sil.h"
#include "unity.h"

#define DP 0x01U

void setUp(void)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT));
}

void tearDown(void)
{
}

static struct euc_display_in idle(void)
{
    struct euc_display_in in;
    (void)memset(&in, 0, sizeof(in));
    in.board_temperature_c = 25.3F;
    in.battery_voltage_cv = 12970;
    in.pedal_mode = 1U;
    in.time_base = 1U;
    return in;
}

static uint32_t lit_bars(const uint8_t grids[EUC_DISPLAY_GRIDS])
{
    static const uint8_t bar_grid[5] = {10U, 8U, 6U, 4U, 0U};
    uint32_t bars = 0U;
    for (uint32_t i = 0U; i < 5U; i++) {
        if ((grids[bar_grid[i]] & DP) != 0U) {
            bars++;
        }
    }
    return bars;
}

static void test_digit_patterns_match_the_observed_ones(void)
{
    /* display.toml: the firmware's digit patterns 0-9 */
    const uint8_t observed[10] = {0xFCU, 0x60U, 0xDAU, 0xF2U, 0x66U, 0xB6U, 0xBEU, 0xE0U, 0xFEU, 0xF6U};
    for (uint32_t digit = 0U; digit < 10U; digit++) {
        TEST_ASSERT_EQUAL_HEX8(observed[digit], euc_display_digit(digit));
    }
    TEST_ASSERT_EQUAL_HEX8(0U, euc_display_digit(10U));
}

static void test_idle_wheel(void)
{
    struct euc_display_in in = idle();
    in.odometer_units = 1468670U; /* 1468.6 km */
    uint8_t grids[EUC_DISPLAY_GRIDS];
    euc_display_render(&in, grids);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(0U), grids[11] & 0xFEU); /* speed 000 */
    TEST_ASSERT_EQUAL_HEX8(grids[11], grids[10]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(0U), grids[8] & 0xFEU);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(0U), grids[7] & 0xFEU);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(2U), grids[5] & 0xFEU); /* pedal mode 1 shows 2 */
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(2U), grids[4] & 0xFEU); /* 25.3 degC */
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(5U) | DP, grids[3]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(3U) | DP, grids[2]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(0U), grids[1] & 0xFEU); /* 01468.6 */
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(1U), grids[0] & 0xFEU);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(4U), grids[15]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(6U), grids[14]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(8U) | DP, grids[13]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(6U), grids[12]);
    TEST_ASSERT_EQUAL_UINT32(5U, lit_bars(grids)); /* 129.7 V */
}

static void test_speed_digits(void)
{
    struct euc_display_in in = idle();
    uint8_t grids[EUC_DISPLAY_GRIDS];
    const int16_t speeds[3] = {10000, -10000, 30000};
    const uint32_t kmh[3] = {36U, 36U, 108U};
    for (size_t i = 0U; i < 3U; i++) {
        in.speed_mm_per_s = speeds[i];
        euc_display_render(&in, grids);
        TEST_ASSERT_EQUAL_HEX8(euc_display_digit(kmh[i] / 100U), grids[11] & 0xFEU);
        TEST_ASSERT_EQUAL_HEX8(euc_display_digit((kmh[i] / 10U) % 10U), grids[9] & 0xFEU);
        TEST_ASSERT_EQUAL_HEX8(euc_display_digit(kmh[i] % 10U), grids[6] & 0xFEU);
    }
}

static void test_battery_bars(void)
{
    struct euc_display_in in = idle();
    uint8_t grids[EUC_DISPLAY_GRIDS];
    const int32_t voltage_cv[6] = {10009, 10010, 10910, 11400, 12100, 12700};
    const uint32_t bars[6] = {1U, 1U, 2U, 3U, 4U, 5U};
    for (size_t i = 0U; i < 6U; i++) {
        in.battery_voltage_cv = voltage_cv[i];
        euc_display_render(&in, grids);
        TEST_ASSERT_EQUAL_UINT32(bars[i], lit_bars(grids));
    }
    in.battery_voltage_cv = 9000; /* below 100.1 V: bar 1 blinks */
    in.time_base = 251U;
    euc_display_render(&in, grids);
    TEST_ASSERT_EQUAL_UINT32(0U, lit_bars(grids));
    in.time_base = 250U;
    euc_display_render(&in, grids);
    TEST_ASSERT_EQUAL_UINT32(1U, lit_bars(grids));
}

static void test_temperature_limits(void)
{
    struct euc_display_in in = idle();
    uint8_t grids[EUC_DISPLAY_GRIDS];
    in.board_temperature_c = -5.0F;
    euc_display_render(&in, grids);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(0U), grids[4] & 0xFEU);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(0U) | DP, grids[2]);
    in.board_temperature_c = 120.0F;
    euc_display_render(&in, grids);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(9U), grids[4] & 0xFEU);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(9U) | DP, grids[3]);
    TEST_ASSERT_EQUAL_HEX8(euc_display_digit(9U) | DP, grids[2]);
}

/* Service sequences ----------------------------------------------------------------------------------------
 */

static void test_service_waits_for_a_zero_command(void)
{
    struct euc_service_state state;
    struct euc_service_out out;
    euc_service_init(&state);
    euc_service_tick(&state, 100.0F, &out);
    TEST_ASSERT_FALSE(out.motor_off);

    euc_service_request(&state, EUC_SERVICE_FIRMWARE_UPDATE);
    euc_service_tick(&state, 100.0F, &out);
    TEST_ASSERT_TRUE(out.motor_off);
    TEST_ASSERT_FALSE(out.enter_bootloader);
    euc_service_tick(&state, -0.0F, &out);
    TEST_ASSERT_TRUE(out.enter_bootloader);
    euc_service_tick(&state, 1e-40F, &out); /* a denormal counts as zero, as in the stock exponent test */
    TEST_ASSERT_TRUE(out.enter_bootloader);
}

static void test_service_actions_run_once(void)
{
    struct euc_service_state state;
    struct euc_service_out out;
    euc_service_init(&state);
    euc_service_request(&state, EUC_SERVICE_CALIBRATION);
    euc_service_request(&state, EUC_SERVICE_DRIVE_MODE_TOGGLE);
    euc_service_tick(&state, 0.0F, &out);
    TEST_ASSERT_TRUE(out.erase_calibration);
    TEST_ASSERT_TRUE(out.toggle_drive_mode);
    euc_service_tick(&state, 0.0F, &out);
    TEST_ASSERT_FALSE(out.erase_calibration || out.toggle_drive_mode || out.motor_off);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_digit_patterns_match_the_observed_ones);
    RUN_TEST(test_idle_wheel);
    RUN_TEST(test_speed_digits);
    RUN_TEST(test_battery_bars);
    RUN_TEST(test_temperature_limits);
    RUN_TEST(test_service_waits_for_a_zero_command);
    RUN_TEST(test_service_actions_run_once);
    return UNITY_END();
}
