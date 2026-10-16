/*
 * Unit tests for the slow sensors (core/sensors/) and euc_log (core/math/libm.c).
 */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/libm.h"
#include "euc/sensors/sensors.h"
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

static uint64_t bits_of(double value)
{
    uint64_t bits;
    (void)memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/* log --------------------------------------------------------------------------------------------------- */

static void test_log_matches_the_host_library(void)
{
    uint64_t state = 0x9E3779B97F4A7C15U;
    for (unsigned i = 0U; i < 100000U; i++) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        /* 1e-300 .. 1e300, log-uniform */
        const double x = pow(10.0, ((double)(state >> 11U) / 9007199254740992.0) * 600.0 - 300.0);
        const double ours = euc_log(x);
        const double host = log(x);
        const uint64_t a = bits_of(ours);
        const uint64_t b = bits_of(host);
        TEST_ASSERT_TRUE(((a > b) ? (a - b) : (b - a)) <= 1U);
    }
}

static void test_log_special_values(void)
{
    TEST_ASSERT_EQUAL_UINT64(0U, bits_of(euc_log(1.0)));
    TEST_ASSERT_TRUE(isinf(euc_log(0.0)) && (euc_log(0.0) < 0.0));
    TEST_ASSERT_TRUE(isnan(euc_log(-1.0)));
    TEST_ASSERT_TRUE(isinf(euc_log((double)INFINITY)));
    TEST_ASSERT_TRUE(fabs(euc_log(2.0) - 0.69314718055994530942) < 1e-15);
    TEST_ASSERT_TRUE(fabs(euc_log(4.9406564584124654e-324) + 744.44007192138126) < 1e-12); /* subnormal */
}

/* Battery
 * ---------------------------------------------------------------------------------------------------- */

static uint16_t adc_data(uint32_t raw)
{
    return (uint16_t)(raw << 3U);
}

static void test_battery_voltage_and_filter(void)
{
    struct euc_sensors_state state;
    euc_sensors_init(&state);
    TEST_ASSERT_EQUAL_INT32(12000, state.battery_voltage_cv);
    /* First tick: the 300000 start value is still in the filter: (300000 x 999 + 290000) / 1000 = 299990. */
    euc_sensors_battery(&state, adc_data(2900U));
    TEST_ASSERT_EQUAL_INT32((int32_t)((299990U * 1377U) >> 15U), state.battery_voltage_cv);
    /* Fast start: the previous value is the last sample. */
    euc_sensors_battery(&state, adc_data(2900U));
    TEST_ASSERT_EQUAL_INT32((int32_t)((290000U * 1377U) >> 15U), state.battery_voltage_cv);
    for (unsigned i = 0U; i < 300U; i++) {
        euc_sensors_battery(&state, adc_data(2900U));
    }
    /* Filtering now: a step moves the output by only 1/1000 of it per tick. */
    euc_sensors_battery(&state, adc_data(3000U));
    TEST_ASSERT_EQUAL_INT32((int32_t)((((290000U * 999U) + 300000U) / 1000U * 1377U) >> 15U),
                            state.battery_voltage_cv);
    TEST_ASSERT_EQUAL_INT32(12186, state.battery_voltage_min_cv);
}

static void test_range_estimate(void)
{
    const struct {
        int32_t cv;
        uint32_t km;
    } cases[] = {{9600, 0U},    {9620, 1U},    {10600, 50U},  {10601, 5U},  {12186, 84U},
                 {12799, 114U}, {12800, 160U}, {12801, 115U}, {13440, 122U}};
    for (size_t i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
        TEST_ASSERT_EQUAL_UINT32(cases[i].km, euc_sensors_range_km(cases[i].cv));
    }
}

/* Bus current ----------------------------------------------------------------------------------------------
 */

static void test_bus_current(void)
{
    struct euc_sensors_state state;
    euc_sensors_init(&state);
    for (unsigned i = 0U; i < 10U; i++) {
        euc_sensors_bus_current(&state, 0x4000U); /* raw x 16 = 32768 */
    }
    TEST_ASSERT_EQUAL_UINT32(32768U, state.current_offset);
    euc_sensors_bus_current(&state, 0x4400U); /* 34816: 2048 above the offset */
    TEST_ASSERT_EQUAL_INT32((2048 * 43750) / 32767, state.bus_current_ca);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 27.34F, state.bus_current_filtered_ca);
}

/* NTC -----------------------------------------------------------------------------------------------------
 */

static double expected_ntc_c(uint32_t raw, double t25_kelvin)
{
    /* The firmware stores the volts and the ohms as floats between the steps. */
    const float volts = (float)(((double)raw * 3.3) / 4096.0);
    const float ohms = (float)(((double)volts * 10000.0) / (3.3 - (double)volts));
    return (1.0 / ((log((double)ohms / 47000.0) / 4150.0) + (1.0 / t25_kelvin))) - 273.15;
}

static void test_board_temperature(void)
{
    const uint32_t raw = 3377U; /* about 47 kohm: 25 degC */
    const uint32_t profiles[2] = {EUC_SIL_PROFILE_COMPAT, EUC_SIL_PROFILE_CLEAN};
    const double t25[2] = {298.5, 298.15};
    for (size_t p = 0U; p < 2U; p++) {
        use_profile(profiles[p]);
        struct euc_sensors_state state;
        euc_sensors_init(&state);
        euc_sensors_board_temperature(&state, adc_data(raw));
        TEST_ASSERT_FLOAT_WITHIN(0.001F, (float)expected_ntc_c(raw, t25[p]), state.board_temperature_c);
        TEST_ASSERT_EQUAL_FLOAT(3.0F, state.board_temperature_slewed_c); /* +3 degC per tick from 0 */
        for (unsigned i = 0U; i < 300U; i++) {
            euc_sensors_board_temperature(&state, adc_data(raw));
        }
        TEST_ASSERT_FLOAT_WITHIN(0.01F, state.board_temperature_c, state.board_temperature_filtered_c);
        const double mpu = (((double)state.board_temperature_filtered_c - 21.0) * 334.0) - 5280.0;
        TEST_ASSERT_FLOAT_WITHIN(0.01F, (float)mpu, state.board_temperature_mpu6050);
    }
    /* Compat reads 0.35 degC high. */
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.35F, (float)(expected_ntc_c(raw, 298.5) - expected_ntc_c(raw, 298.15)));
}

/* Calibration, miles
 * ------------------------------------------------------------------------------------------ */

static void test_imu_calibration_validity(void)
{
    const int16_t level[6] = {100, -200, 8192, 10, -20, 30};
    TEST_ASSERT_TRUE(euc_sensors_imu_calibration_valid(level));
    const int16_t tilted[6] = {1500, -200, 8192, 10, -20, 30};
    TEST_ASSERT_FALSE(euc_sensors_imu_calibration_valid(tilted));
    const int16_t upside_down[6] = {100, -200, -8192, 10, -20, 30};
    TEST_ASSERT_FALSE(euc_sensors_imu_calibration_valid(upside_down));
    const int16_t turning[6] = {100, -200, 8192, 10, -700, 30};
    TEST_ASSERT_FALSE(euc_sensors_imu_calibration_valid(turning));
}

static void test_miles(void)
{
    TEST_ASSERT_EQUAL_UINT32(620U, euc_sensors_miles_u32(1000U));
    TEST_ASSERT_EQUAL_UINT32(620U, euc_sensors_miles_u32(1001U));
    TEST_ASSERT_EQUAL_INT32(-620, euc_sensors_miles_i32(-1000));
    TEST_ASSERT_EQUAL_UINT32(910575U, euc_sensors_miles_u32(1468670U));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_log_matches_the_host_library);
    RUN_TEST(test_log_special_values);
    RUN_TEST(test_battery_voltage_and_filter);
    RUN_TEST(test_range_estimate);
    RUN_TEST(test_bus_current);
    RUN_TEST(test_board_temperature);
    RUN_TEST(test_imu_calibration_validity);
    RUN_TEST(test_miles);
    return UNITY_END();
}
