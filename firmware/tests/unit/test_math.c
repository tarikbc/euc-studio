/*
 * Unit tests for core/math: our libm (sqrt, asin, atan, atan2) against the host C library, and the float
 * steps with the stock rounding (euc/math/float_ops.h) in the compat and clean profiles.
 */
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/float_ops.h"
#include "euc/math/libm.h"
#include "euc/sil.h"
#include "unity.h"

#define RANDOM_SAMPLES 200000U

/* Small deterministic generator (xorshift64), so the test does not depend on the host rand(). */
static uint64_t random_state = 0x9E3779B97F4A7C15U;

static double random_unit(void)
{
    random_state ^= random_state << 13U;
    random_state ^= random_state >> 7U;
    random_state ^= random_state << 17U;
    return (double)(random_state >> 11U) / 9007199254740992.0; /* [0, 1) */
}

/* Distance in units in the last place between two finite doubles. */
static uint64_t ulp_distance(double a, double b)
{
    int64_t ia;
    int64_t ib;
    memcpy(&ia, &a, sizeof(ia));
    memcpy(&ib, &b, sizeof(ib));
    ia = (ia < 0) ? (INT64_MIN - ia) : ia;
    ib = (ib < 0) ? (INT64_MIN - ib) : ib;
    return (ia > ib) ? (uint64_t)(ia - ib) : (uint64_t)(ib - ia);
}

static void select_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

void setUp(void)
{
}

void tearDown(void)
{
    select_profile(EUC_SIL_PROFILE_COMPAT);
}

static void test_sqrt_is_correctly_rounded(void)
{
    for (uint32_t sample = 0U; sample < RANDOM_SAMPLES; sample++) {
        const double x = ldexp(random_unit() + 0.5, (int)(random_unit() * 200.0) - 100);
        TEST_ASSERT_TRUE(euc_sqrt(x) == sqrt(x));
    }
    TEST_ASSERT_TRUE(euc_sqrt(4.0) == 2.0);
    TEST_ASSERT_TRUE(euc_sqrt(DBL_MIN / 8.0) == sqrt(DBL_MIN / 8.0)); /* denormal input */
    TEST_ASSERT_TRUE(euc_sqrt(DBL_MAX) == sqrt(DBL_MAX));
    TEST_ASSERT_TRUE(signbit(euc_sqrt(-0.0)) != 0);
    TEST_ASSERT_TRUE(isnan(euc_sqrt(-1.0)));
    TEST_ASSERT_TRUE(isinf(euc_sqrt(HUGE_VAL)));
}

static void test_asin_atan_atan2_are_within_one_ulp(void)
{
    uint64_t worst = 0U;
    for (uint32_t sample = 0U; sample < RANDOM_SAMPLES; sample++) {
        const double x = (random_unit() * 2.0) - 1.0;
        const double y = (random_unit() * 8.0) - 4.0;
        const double z = (random_unit() * 8.0) - 4.0;
        const uint64_t errors[3] = {ulp_distance(euc_asin(x), asin(x)), ulp_distance(euc_atan(y), atan(y)),
                                    ulp_distance(euc_atan2(y, z), atan2(y, z))};
        for (size_t index = 0U; index < 3U; index++) {
            worst = (errors[index] > worst) ? errors[index] : worst;
        }
    }
    TEST_ASSERT_LESS_OR_EQUAL_UINT64(1U, worst);
}

static void test_inverse_trig_special_values(void)
{
    TEST_ASSERT_TRUE(euc_asin(1.0) == asin(1.0));
    TEST_ASSERT_TRUE(euc_asin(-1.0) == asin(-1.0));
    TEST_ASSERT_TRUE(euc_asin(0.0) == 0.0);
    TEST_ASSERT_TRUE(euc_asin(1e-20) == 1e-20);
    TEST_ASSERT_TRUE(isnan(euc_asin(1.5)));
    TEST_ASSERT_TRUE(euc_atan(HUGE_VAL) == atan(HUGE_VAL));
    TEST_ASSERT_TRUE(euc_atan2(0.0, -1.0) == atan2(0.0, -1.0));
    TEST_ASSERT_TRUE(euc_atan2(-0.0, -1.0) == atan2(-0.0, -1.0));
    TEST_ASSERT_TRUE(euc_atan2(1.0, 0.0) == atan2(1.0, 0.0));
    TEST_ASSERT_TRUE(euc_atan2(-1.0, -1.0) == atan2(-1.0, -1.0));
    TEST_ASSERT_TRUE(euc_atan2(HUGE_VAL, -HUGE_VAL) == atan2(HUGE_VAL, -HUGE_VAL));
    TEST_ASSERT_TRUE(euc_atan2(1.0, HUGE_VAL) == 0.0);
    TEST_ASSERT_TRUE(signbit(euc_atan2(-1.0, HUGE_VAL)) != 0);
    TEST_ASSERT_TRUE(isnan(euc_atan2((double)NAN, 1.0)));
}

static void test_compat_flushes_denormal_results(void)
{
    const float tiny = FLT_MIN; /* smallest normal */
    select_profile(EUC_SIL_PROFILE_COMPAT);
    TEST_ASSERT_EQUAL_HEX32(0x00000000U, euc_f32_bits(euc_f32_mul(tiny, 0.5F)));
    TEST_ASSERT_EQUAL_HEX32(0x80000000U, euc_f32_bits(euc_f32_mul(-tiny, 0.5F)));
    TEST_ASSERT_EQUAL_HEX32(0x00000000U, euc_f32_bits(euc_f32_from_f64((double)FLT_MIN / 4.0)));
    TEST_ASSERT_EQUAL_HEX32(euc_f32_bits(tiny), euc_f32_bits(euc_f32_add(tiny, 0.0F)));

    select_profile(EUC_SIL_PROFILE_CLEAN);
    TEST_ASSERT_EQUAL_HEX32(0x00400000U, euc_f32_bits(euc_f32_mul(tiny, 0.5F)));
}

static void test_inverse_square_root(void)
{
    select_profile(EUC_SIL_PROFILE_COMPAT);
    /* Estimate 0x5F3759DF - (bits >> 1), then one Newton step, each step rounded to float. */
    TEST_ASSERT_EQUAL_HEX32(0x3F7F910FU, euc_f32_bits(euc_f32_inv_sqrt(1.0F))); /* 0.99830717 */
    TEST_ASSERT_EQUAL_HEX32(0x3EFF910FU, euc_f32_bits(euc_f32_inv_sqrt(4.0F))); /* 0.49915358 */

    select_profile(EUC_SIL_PROFILE_CLEAN);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, euc_f32_inv_sqrt(1.0F));
    TEST_ASSERT_EQUAL_FLOAT(0.5F, euc_f32_inv_sqrt(4.0F));
}

static void test_abs_clears_only_the_sign(void)
{
    TEST_ASSERT_EQUAL_HEX32(0x00000000U, euc_f32_bits(euc_f32_abs(-0.0F)));
    TEST_ASSERT_EQUAL_FLOAT(2.5F, euc_f32_abs(-2.5F));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sqrt_is_correctly_rounded);
    RUN_TEST(test_asin_atan_atan2_are_within_one_ulp);
    RUN_TEST(test_inverse_trig_special_values);
    RUN_TEST(test_compat_flushes_denormal_results);
    RUN_TEST(test_inverse_square_root);
    RUN_TEST(test_abs_clears_only_the_sign);
    return UNITY_END();
}
