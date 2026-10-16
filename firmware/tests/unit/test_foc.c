/*
 * Unit tests for the FOC arithmetic (core/motor/foc_math.c, pid.c) and the generated lookup tables.
 *
 * The tables are generated from formulas at build time (tools/gen_motor_tables.py). Here they are checked
 * against their formulas, against the documented values, and against the stock tables by CRC-32: the two
 * CRCs below were computed from the stock image (official build, tables at 0x0800CDE6 and 0x0800CE6A) in a
 * private check, so the tests prove equality without the tables being in this repository.
 */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "euc/motor/foc_math.h"
#include "euc/motor/pid.h"
#include "euc/sil.h"
#include "unity.h"

#define STOCK_SINE_TABLE_CRC32   0x4BFBC6E2U
#define STOCK_CIRCLE_TABLE_CRC32 0xB034C1C2U

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

/* CRC-32 (IEEE 802.3, reflected, as zlib) over 16-bit values stored little-endian, as in the image. */
static uint32_t crc32_of_u16(const uint16_t *values, size_t count)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0U; i < count; i++) {
        const uint8_t bytes[2] = {(uint8_t)(values[i] & 0xFFU), (uint8_t)(values[i] >> 8U)};
        for (size_t j = 0U; j < 2U; j++) {
            crc ^= bytes[j];
            for (unsigned bit = 0U; bit < 8U; bit++) {
                crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
            }
        }
    }
    return ~crc;
}

/* Tables ----------------------------------------------------------------------------------------------- */

static void test_sine_table_follows_its_formula(void)
{
    const double pi = 3.14159265358979323846;
    TEST_ASSERT_EQUAL_INT16(0, euc_foc_sine_table[0]);
    TEST_ASSERT_EQUAL_INT16(201, euc_foc_sine_table[1]);
    TEST_ASSERT_EQUAL_INT16(402, euc_foc_sine_table[2]);
    TEST_ASSERT_EQUAL_INT16(32766, euc_foc_sine_table[255]);
    for (size_t i = 0U; i < EUC_FOC_SINE_ENTRIES; i++) {
        const double exact = 32767.0 * sin((double)i * pi / 512.0);
        TEST_ASSERT_EQUAL_INT16((int16_t)floor(exact + 0.5), euc_foc_sine_table[i]);
        if (i > 0U) {
            TEST_ASSERT_TRUE(euc_foc_sine_table[i] > euc_foc_sine_table[i - 1U]);
        }
    }
}

static void test_sine_table_equals_the_stock_table(void)
{
    uint16_t words[EUC_FOC_SINE_ENTRIES];
    for (size_t i = 0U; i < EUC_FOC_SINE_ENTRIES; i++) {
        words[i] = (uint16_t)euc_foc_sine_table[i];
    }
    TEST_ASSERT_EQUAL_HEX32(STOCK_SINE_TABLE_CRC32, crc32_of_u16(words, EUC_FOC_SINE_ENTRIES));
}

static void test_circle_table_documented_values(void)
{
    TEST_ASSERT_EQUAL_UINT16(32635, euc_foc_circle_table[0]);
    TEST_ASSERT_EQUAL_UINT16(22893, euc_foc_circle_table[EUC_FOC_CIRCLE_ENTRIES - 1U]);
    uint32_t sum = 0U;
    for (size_t k = 0U; k < EUC_FOC_CIRCLE_ENTRIES; k++) {
        sum += euc_foc_circle_table[k];
        if (k > 0U) {
            TEST_ASSERT_TRUE(euc_foc_circle_table[k] < euc_foc_circle_table[k - 1U]);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(1777582U, sum);
    /* The two half steps of the stock table: about half of the neighbouring differences. */
    TEST_ASSERT_EQUAL_INT(98, euc_foc_circle_table[13] - euc_foc_circle_table[14]);
    TEST_ASSERT_EQUAL_INT(64, euc_foc_circle_table[39] - euc_foc_circle_table[40]);
    TEST_ASSERT_EQUAL_HEX32(STOCK_CIRCLE_TABLE_CRC32,
                            crc32_of_u16(euc_foc_circle_table, EUC_FOC_CIRCLE_ENTRIES));
}

static void test_circle_table_keeps_every_bin_inside_the_circle(void)
{
    /* The longest vector of bin k, (63 + k) x 2^24, scaled by entry k (Q1.15), stays within 32439. */
    const uint64_t limit = (uint64_t)32439U * 32439U * ((uint64_t)1U << 30U);
    for (uint64_t k = 0U; k < EUC_FOC_CIRCLE_ENTRIES; k++) {
        const uint64_t factor = euc_foc_circle_table[k];
        TEST_ASSERT_TRUE(factor * factor * (63U + k) * ((uint64_t)1U << 24U) <= limit);
    }
}

/* Trigonometry ----------------------------------------------------------------------------------------- */

static void test_trig_at_the_quadrant_edges(void)
{
    struct euc_foc_trig trig = euc_foc_trig_from_angle(0);
    TEST_ASSERT_EQUAL_INT16(32766, trig.cos_q15);
    TEST_ASSERT_EQUAL_INT16(0, trig.sin_q15);
    trig = euc_foc_trig_from_angle(16384); /* +90 deg */
    TEST_ASSERT_EQUAL_INT16(0, trig.cos_q15);
    TEST_ASSERT_EQUAL_INT16(32766, trig.sin_q15);
    trig = euc_foc_trig_from_angle(-16384); /* -90 deg */
    TEST_ASSERT_EQUAL_INT16(0, trig.cos_q15);
    TEST_ASSERT_EQUAL_INT16(-32766, trig.sin_q15);
    trig = euc_foc_trig_from_angle(INT16_MIN); /* -180 deg */
    TEST_ASSERT_EQUAL_INT16(-32766, trig.cos_q15);
    TEST_ASSERT_EQUAL_INT16(0, trig.sin_q15);
}

static void test_trig_follows_cos_and_sin_all_round(void)
{
    const double pi = 3.14159265358979323846;
    for (int32_t angle = INT16_MIN; angle <= INT16_MAX; angle += 7) {
        const struct euc_foc_trig trig = euc_foc_trig_from_angle((int16_t)angle);
        const double radians = (double)angle * pi / 32768.0;
        /* One table step is 64 angle counts (0.35 deg): at most 32767 x 2 pi x 64 / 65536 = 201 counts. */
        TEST_ASSERT_TRUE(fabs((32767.0 * cos(radians)) - (double)trig.cos_q15) <= 202.0);
        TEST_ASSERT_TRUE(fabs((32767.0 * sin(radians)) - (double)trig.sin_q15) <= 202.0);
    }
}

/* Clarke, Park, inverse Park --------------------------------------------------------------------------- */

static void test_clarke_of_balanced_currents(void)
{
    /* Ia = 10000, Ib = Ic = -5000: alpha = Ia, beta = -(Ia + 2 Ib) / sqrt(3) = 0 (one count of truncation).
     */
    struct euc_foc_alpha_beta current = euc_foc_clarke(10000, -5000);
    TEST_ASSERT_EQUAL_INT16(10000, current.alpha_q15);
    TEST_ASSERT_EQUAL_INT16(-1, current.beta_q15); /* -(5773 + 2 x (-2886)) */
    /* Ia = 0, Ib = -Ic = 10000: beta = -2 x 10000 / sqrt(3) = -11546. */
    current = euc_foc_clarke(0, 10000);
    TEST_ASSERT_EQUAL_INT16(0, current.alpha_q15);
    TEST_ASSERT_EQUAL_INT16(-11546, current.beta_q15);
}

static void test_park_and_inverse_park_round_trip(void)
{
    const struct euc_foc_qd vectors[3] = {{12000, -3000}, {-20000, 5000}, {0, 30000}};
    for (int32_t angle = INT16_MIN; angle <= INT16_MAX; angle += 997) {
        const struct euc_foc_trig trig = euc_foc_trig_from_angle((int16_t)angle);
        for (size_t i = 0U; i < 3U; i++) {
            const struct euc_foc_alpha_beta ab = euc_foc_inverse_park(vectors[i], trig);
            const struct euc_foc_qd back = euc_foc_park(ab, trig);
            /* The table cosine is the sine one step (0.35 deg) further on, so cos^2 + sin^2 is within 0.62 %
             * of 1; with the truncations the round trip stays within 0.7 % plus 4 counts. */
            const int32_t q = vectors[i].q_q15;
            const int32_t d = vectors[i].d_q15;
            TEST_ASSERT_INT_WITHIN(4 + ((7 * abs(q)) / 1000), q, back.q_q15);
            TEST_ASSERT_INT_WITHIN(4 + ((7 * abs(d)) / 1000), d, back.d_q15);
        }
    }
}

static void test_park_at_zero_angle(void)
{
    const struct euc_foc_trig trig = euc_foc_trig_from_angle(0);
    const struct euc_foc_alpha_beta current = {10000, -6000};
    const struct euc_foc_qd qd = euc_foc_park(current, trig);
    /* q = 10000 x 32766 / 32768 = 9999 (truncated), d = -6000 x 32766 / 32768 = -5999 (toward zero). */
    TEST_ASSERT_EQUAL_INT16(9999, qd.q_q15);
    TEST_ASSERT_EQUAL_INT16(-5999, qd.d_q15);
}

static void test_out_of_range_wraps_in_compat_and_saturates_in_clean(void)
{
    use_profile(EUC_SIL_PROFILE_COMPAT);
    TEST_ASSERT_EQUAL_INT16(-25536, euc_foc_narrow(40000));
    TEST_ASSERT_EQUAL_INT16(25536, euc_foc_narrow(-40000));
    TEST_ASSERT_EQUAL_INT16(1234, euc_foc_narrow(1234));
    /* Ia = Ib = 30000 (not a balanced set): beta = -(17319 + 2 x 17319) = -51957. */
    TEST_ASSERT_EQUAL_INT16(13579, euc_foc_clarke(30000, 30000).beta_q15);

    use_profile(EUC_SIL_PROFILE_CLEAN);
    TEST_ASSERT_EQUAL_INT16(INT16_MAX, euc_foc_narrow(40000));
    TEST_ASSERT_EQUAL_INT16(INT16_MIN, euc_foc_narrow(-40000));
    TEST_ASSERT_EQUAL_INT16(1234, euc_foc_narrow(1234));
    TEST_ASSERT_EQUAL_INT16(INT16_MIN, euc_foc_clarke(30000, 30000).beta_q15);
}

/* Circle limitation ------------------------------------------------------------------------------------ */

static int64_t length_squared(struct euc_foc_qd v)
{
    return ((int64_t)v.q_q15 * v.q_q15) + ((int64_t)v.d_q15 * v.d_q15);
}

static void test_circle_limit_passes_short_vectors(void)
{
    const struct euc_foc_qd inside[3] = {{32439, 0}, {0, -32439}, {20000, 25000}};
    for (size_t i = 0U; i < 3U; i++) {
        const struct euc_foc_qd out = euc_foc_circle_limit(inside[i]);
        TEST_ASSERT_EQUAL_INT16(inside[i].q_q15, out.q_q15);
        TEST_ASSERT_EQUAL_INT16(inside[i].d_q15, out.d_q15);
    }
}

static void test_circle_limit_scales_long_vectors_inside(void)
{
    /* 32767 along q: squared length 1073676289 is in bin 63 = entry 1. */
    struct euc_foc_qd out = euc_foc_circle_limit((struct euc_foc_qd){32767, 0});
    TEST_ASSERT_EQUAL_INT16((int16_t)((32767 * (int32_t)euc_foc_circle_table[1]) / 32768), out.q_q15);
    TEST_ASSERT_EQUAL_INT16(0, out.d_q15);

    const int64_t limit = (int64_t)EUC_FOC_CIRCLE_MAX_MODULE * EUC_FOC_CIRCLE_MAX_MODULE;
    for (int32_t q = -32768; q <= 32767; q += 509) {
        for (int32_t d = -32768; d <= 32767; d += 1021) {
            const struct euc_foc_qd in = {(int16_t)q, (int16_t)d};
            out = euc_foc_circle_limit(in);
            TEST_ASSERT_TRUE(length_squared(out) <= limit);
            TEST_ASSERT_TRUE(length_squared(out) <= length_squared(in));
        }
    }
    /* The one vector past the last bin reads the last entry (no read outside the table). */
    out = euc_foc_circle_limit((struct euc_foc_qd){INT16_MIN, INT16_MIN});
    TEST_ASSERT_TRUE(length_squared(out) <= limit);
}

/* SVPWM ------------------------------------------------------------------------------------------------ */

static void test_svpwm_zero_vector_is_half_duty(void)
{
    struct euc_foc_duty duty = euc_foc_svpwm((struct euc_foc_alpha_beta){0, 0}, 12000U, 20784U);
    TEST_ASSERT_EQUAL_UINT16(1500U, duty.a);
    TEST_ASSERT_EQUAL_UINT16(1500U, duty.b);
    TEST_ASSERT_EQUAL_UINT16(1500U, duty.c);
    duty = euc_foc_svpwm((struct euc_foc_alpha_beta){0, 0}, 14400U, 24941U);
    TEST_ASSERT_EQUAL_UINT16(1800U, duty.a);
    TEST_ASSERT_EQUAL_UINT16(1800U, duty.b);
    TEST_ASSERT_EQUAL_UINT16(1800U, duty.c);
}

static void test_svpwm_full_scale_axes(void)
{
    /* Current mode, ARR 3000. Full alpha: A high, B and C low. Full beta (negated axis): C high, B low. */
    struct euc_foc_duty duty = euc_foc_svpwm((struct euc_foc_alpha_beta){32767, 0}, 12000U, 20784U);
    TEST_ASSERT_EQUAL_UINT16(2799U, duty.a);
    TEST_ASSERT_EQUAL_UINT16(202U, duty.b);
    TEST_ASSERT_EQUAL_UINT16(202U, duty.c);
    duty = euc_foc_svpwm((struct euc_foc_alpha_beta){0, 32767}, 12000U, 20784U);
    TEST_ASSERT_EQUAL_UINT16(1500U, duty.a);
    TEST_ASSERT_EQUAL_UINT16(1U, duty.b);
    TEST_ASSERT_EQUAL_UINT16(2999U, duty.c);
}

static void test_svpwm_phase_voltages_all_round(void)
{
    /* Phase voltage against the star point, in compare counts: |V| = 32767 gives ARR / sqrt(3). With this
     * drive's beta sign: va = alpha, vb = -alpha / 2 - sqrt(3) beta / 2, vc = -alpha / 2 + sqrt(3) beta / 2.
     */
    const double pi = 3.14159265358979323846;
    const double arr = 3000.0;
    const double counts_per_unit = arr / sqrt(3.0) / 32767.0;
    for (int deg = 0; deg < 360; deg++) {
        const double radians = (double)deg * pi / 180.0;
        const struct euc_foc_alpha_beta v = {(int16_t)lround(32439.0 * cos(radians)),
                                             (int16_t)lround(32439.0 * sin(radians))};
        const struct euc_foc_duty duty = euc_foc_svpwm(v, 12000U, 20784U);
        TEST_ASSERT_TRUE(duty.a <= 3000U);
        TEST_ASSERT_TRUE(duty.b <= 3000U);
        TEST_ASSERT_TRUE(duty.c <= 3000U);
        const double mean = ((double)duty.a + (double)duty.b + (double)duty.c) / 3.0;
        const double alpha = (double)v.alpha_q15;
        const double beta = (double)v.beta_q15;
        const double expected_a = alpha * counts_per_unit;
        const double expected_b = ((-alpha / 2.0) - ((sqrt(3.0) * beta) / 2.0)) * counts_per_unit;
        const double expected_c = ((-alpha / 2.0) + ((sqrt(3.0) * beta) / 2.0)) * counts_per_unit;
        TEST_ASSERT_TRUE(fabs(expected_a - ((double)duty.a - mean)) <= 3.0);
        TEST_ASSERT_TRUE(fabs(expected_b - ((double)duty.b - mean)) <= 3.0);
        TEST_ASSERT_TRUE(fabs(expected_c - ((double)duty.c - mean)) <= 3.0);
    }
}

/* PID -------------------------------------------------------------------------------------------------- */

static struct euc_pid test_pid(void)
{
    const struct euc_pid pid = {
        .kp = 3000,
        .kp_div = 1024U,
        .ki = 30,
        .ki_div = 8192U,
        .kd = 5000,
        .kd_div = 8192U,
        .out_min = -32000,
        .out_max = 32000,
        .integral_min = -100000,
        .integral_max = 100000,
        .integral = 0,
        .previous_error = 0,
    };
    return pid;
}

static void test_pid_terms(void)
{
    struct euc_pid pid = test_pid();
    /* e = 2000: P = 2000 x 3000 / 1024 = 5859; I = 60000 / 8192 = 7; D = 5000 x 2000 / 8192 = 1220. */
    TEST_ASSERT_EQUAL_INT16(5859 + 7 + 1220, euc_pid_step(&pid, 2000, 0, false));
    TEST_ASSERT_EQUAL_INT32(60000, pid.integral);
    TEST_ASSERT_EQUAL_INT32(2000, pid.previous_error);
    /* Same error: no D; the integral reaches its limit 100000: I = 12. */
    TEST_ASSERT_EQUAL_INT16(5859 + 12, euc_pid_step(&pid, 2000, 0, false));
    TEST_ASSERT_EQUAL_INT32(100000, pid.integral);
}

static void test_pid_truncates_toward_zero(void)
{
    struct euc_pid pid = test_pid();
    pid.ki = 0;
    pid.kd = 0;
    /* e = -1: -3000 / 1024 = -2 (toward zero, not -3). */
    TEST_ASSERT_EQUAL_INT16(-2, euc_pid_step(&pid, 0, 1, false));
    TEST_ASSERT_EQUAL_INT32(0, pid.integral); /* Ki = 0 keeps the integral at 0 */
}

static void test_pid_clear_and_output_limits(void)
{
    struct euc_pid pid = test_pid();
    (void)euc_pid_step(&pid, 2000, 0, false);
    (void)euc_pid_step(&pid, 2000, 0, true);
    TEST_ASSERT_EQUAL_INT32(0, pid.integral);
    TEST_ASSERT_EQUAL_INT16(32000, euc_pid_step(&pid, 30000, -30000, false));
    TEST_ASSERT_EQUAL_INT16(-32000, euc_pid_step(&pid, -30000, 30000, false));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_sine_table_follows_its_formula);
    RUN_TEST(test_sine_table_equals_the_stock_table);
    RUN_TEST(test_circle_table_documented_values);
    RUN_TEST(test_circle_table_keeps_every_bin_inside_the_circle);
    RUN_TEST(test_trig_at_the_quadrant_edges);
    RUN_TEST(test_trig_follows_cos_and_sin_all_round);
    RUN_TEST(test_clarke_of_balanced_currents);
    RUN_TEST(test_park_and_inverse_park_round_trip);
    RUN_TEST(test_park_at_zero_angle);
    RUN_TEST(test_out_of_range_wraps_in_compat_and_saturates_in_clean);
    RUN_TEST(test_circle_limit_passes_short_vectors);
    RUN_TEST(test_circle_limit_scales_long_vectors_inside);
    RUN_TEST(test_svpwm_zero_vector_is_half_duty);
    RUN_TEST(test_svpwm_full_scale_axes);
    RUN_TEST(test_svpwm_phase_voltages_all_round);
    RUN_TEST(test_pid_terms);
    RUN_TEST(test_pid_truncates_toward_zero);
    RUN_TEST(test_pid_clear_and_output_limits);
    return UNITY_END();
}
