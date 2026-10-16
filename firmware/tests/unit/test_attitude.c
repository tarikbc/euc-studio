/*
 * Unit tests for core/attitude: IMU offsets, accelerometer-only tilt, the free-fall detector and the Mahony
 * filter, with the compat quirks (ASIN_ACCEL, SEQ_QUAT_UPDATE, DT_500HZ) and in the clean profile.
 *
 * Synthetic IMU: a tilt of theta (pitch, about y) and phi (roll, about x) reads
 *   ax = -g sin(theta), ay = g cos(theta) sin(phi), az = g cos(theta) cos(phi), with g = 8192 counts.
 */
#include <math.h>
#include <stdint.h>

#include "euc/attitude/attitude.h"
#include "euc/quirks.h"
#include "euc/sil.h"
#include "unity.h"

#define DEG_TO_RAD          (3.14159265358979323846 / 180.0)
#define GYRO_COUNTS_PER_DPS 65.5

static void select_profile(uint32_t profile)
{
    euc_sil_quirk_set_mask(euc_sil_quirk_profile_mask(profile));
}

static struct euc_attitude_in tilted_input(double pitch_deg, double roll_deg)
{
    struct euc_attitude_in in = {0};
    const double theta = pitch_deg * DEG_TO_RAD;
    const double phi = roll_deg * DEG_TO_RAD;
    in.accel[0] = (int16_t)lround(-EUC_IMU_ACCEL_COUNTS_PER_G * sin(theta));
    in.accel[1] = (int16_t)lround(EUC_IMU_ACCEL_COUNTS_PER_G * cos(theta) * sin(phi));
    in.accel[2] = (int16_t)lround(EUC_IMU_ACCEL_COUNTS_PER_G * cos(theta) * cos(phi));
    in.accel_gain = 0.6F;
    return in;
}

static void run_ticks(struct euc_attitude_state *state, const struct euc_attitude_in *in, unsigned ticks,
                      struct euc_attitude_out *out)
{
    for (unsigned tick = 0U; tick < ticks; tick++) {
        euc_attitude_update_tilt(state, in->accel);
        euc_attitude_update(state, in, out);
    }
}

void setUp(void)
{
    select_profile(EUC_SIL_PROFILE_COMPAT);
}

void tearDown(void)
{
    select_profile(EUC_SIL_PROFILE_COMPAT);
}

/* Gyro range: compat keeps the stock +-500 deg/s, clean uses +-2000 (GYRO_500DPS_CLIP). Flash offsets stay in
 * +-500 deg/s counts. */
static void test_gyro_range_per_profile(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x08U, euc_imu_gyro_config());
    TEST_ASSERT_EQUAL_INT16(-401, euc_imu_gyro_offset_from_page(-401));
    TEST_ASSERT_EQUAL_INT16(-401, euc_imu_gyro_offset_to_page(-401));
    select_profile(EUC_SIL_PROFILE_CLEAN);
    TEST_ASSERT_EQUAL_HEX8(0x18U, euc_imu_gyro_config());
    TEST_ASSERT_EQUAL_INT16(100, euc_imu_gyro_offset_from_page(401));
    TEST_ASSERT_EQUAL_INT16(-100, euc_imu_gyro_offset_from_page(-401));
    TEST_ASSERT_EQUAL_INT16(101, euc_imu_gyro_offset_from_page(402));
    TEST_ASSERT_EQUAL_INT16(400, euc_imu_gyro_offset_to_page(100));
    TEST_ASSERT_EQUAL_INT16(INT16_MAX, euc_imu_gyro_offset_to_page(9000));
}

static void test_clean_integrates_2000dps_counts(void)
{
    select_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    struct euc_attitude_in in = tilted_input(0.0, 0.0);
    in.accel_gain = 0.0F;
    in.gyro[1] = 164; /* 10 deg/s at 16.4 counts per deg/s */
    euc_attitude_init(&state);
    run_ticks(&state, &in, 496U, &out);
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 10.0F, out.pitch_deg);
}

/* The estimate 24 deg away from a level accelerometer, as after a clipped impact: settle level, then turn the
 * estimate with the gyro alone. */
static void wrong_by_24_deg(struct euc_attitude_state *state)
{
    struct euc_attitude_out out;
    struct euc_attitude_in in = tilted_input(0.0, 0.0);
    in.accel_gain = 0.3F;
    euc_attitude_init(state);
    run_ticks(state, &in, 1500U, &out);
    in.accel_gain = 0.0F;
    in.gyro[1] = (int16_t)lround(24.0 * 16.4); /* 24 deg/s for 1 s */
    run_ticks(state, &in, 496U, &out);
    TEST_ASSERT_FLOAT_WITHIN(0.2F, 24.0F, out.pitch_deg);
}

static void test_clean_reanchors_after_the_gyro_rail(void)
{
    select_profile(EUC_SIL_PROFILE_CLEAN);
    struct euc_attitude_out out;
    struct euc_attitude_in in = tilted_input(0.0, 0.0);
    in.accel_gain = 0.3F;

    struct euc_attitude_state plain;
    wrong_by_24_deg(&plain);
    run_ticks(&plain, &in, 248U, &out); /* 0.5 s at the normal gain: about 24 x exp(-0.15) */
    TEST_ASSERT_TRUE(out.pitch_deg > 18.0F);

    struct euc_attitude_state railed;
    wrong_by_24_deg(&railed);
    in.gyro_at_rail = true;
    run_ticks(&railed, &in, 1U, &out);
    TEST_ASSERT_EQUAL_UINT16(0U, railed.reanchor_ticks); /* no raised gain while the gyro is at its rail */
    in.gyro_at_rail = false;
    run_ticks(&railed, &in, 248U, &out); /* 0.5 s at 10 x the gain: about 24 x exp(-1.5) */
    TEST_ASSERT_TRUE(out.pitch_deg < 7.0F);
    run_ticks(&railed, &in, 300U, &out);
    TEST_ASSERT_EQUAL_UINT16(0U, railed.reanchor_ticks); /* back to the normal gain after 1 s */
}

static void test_compat_never_reanchors(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    struct euc_attitude_in in = tilted_input(0.0, 0.0);
    euc_attitude_init(&state);
    in.gyro_at_rail = true;
    run_ticks(&state, &in, 1U, &out);
    in.gyro_at_rail = false;
    run_ticks(&state, &in, 1U, &out);
    TEST_ASSERT_FALSE(state.reanchor_pending);
    TEST_ASSERT_EQUAL_UINT16(0U, state.reanchor_ticks);
}

static void test_imu_offset_saturates(void)
{
    TEST_ASSERT_EQUAL_INT16(70, euc_imu_correct(100, 30));
    TEST_ASSERT_EQUAL_INT16(INT16_MAX, euc_imu_correct(32000, -1000));
    TEST_ASSERT_EQUAL_INT16(-INT16_MAX, euc_imu_correct(-32000, 1000));
}

static void test_accelerometer_tilt_angles(void)
{
    struct euc_attitude_state state;
    euc_attitude_init(&state);
    const struct euc_attitude_in in = tilted_input(-10.0, 0.0); /* ax = +g sin(10 deg) */
    for (unsigned tick = 0U; tick < 300U; tick++) {
        euc_attitude_update_tilt(&state, in.accel);
    }
    /* asin(1422 counts x 0.000122) = 10.0 deg; the 0.95 low-pass has settled. */
    TEST_ASSERT_INT_WITHIN(25, 990, state.tilt_centideg[0]); /* the integer low-pass settles a little low */
    TEST_ASSERT_INT_WITHIN(5, 0, state.tilt_centideg[1]);
    TEST_ASSERT_TRUE(state.tilt_centideg[2] > 4500);
}

static void test_level_and_still_stays_level(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    euc_attitude_init(&state);
    const struct euc_attitude_in in = tilted_input(0.0, 0.0);
    run_ticks(&state, &in, 2000U, &out);
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.0F, out.pitch_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.0F, out.roll_deg);
}

static void test_asin_accel_settles_below_the_true_tilt(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    const struct euc_attitude_in in = tilted_input(10.0, 0.0);

    /* Compat: the asin-shaped accelerometer vector points at 7.14 deg for a true 10 deg tilt. */
    euc_attitude_init(&state);
    run_ticks(&state, &in, 8000U, &out);
    TEST_ASSERT_FLOAT_WITHIN(0.2F, 7.14F, out.pitch_deg);

    /* Clean: the plain vector, so the filter settles at the true tilt. */
    select_profile(EUC_SIL_PROFILE_CLEAN);
    euc_attitude_init(&state);
    run_ticks(&state, &in, 8000U, &out);
    TEST_ASSERT_FLOAT_WITHIN(0.2F, 10.0F, out.pitch_deg);
}

static void test_roll_sign(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    euc_attitude_init(&state);
    select_profile(EUC_SIL_PROFILE_CLEAN);
    const struct euc_attitude_in in = tilted_input(0.0, 20.0);
    run_ticks(&state, &in, 8000U, &out);
    TEST_ASSERT_FLOAT_WITHIN(0.3F, 20.0F, out.roll_deg);
}

static float pitch_after_one_second_of_gyro(uint64_t quirk_mask)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    struct euc_attitude_in in = tilted_input(0.0, 0.0);
    in.accel_gain = 0.0F;                                     /* gyro only */
    in.gyro[1] = (int16_t)lround(10.0 * GYRO_COUNTS_PER_DPS); /* 10 deg/s about y */
    euc_sil_quirk_set_mask(quirk_mask);
    euc_attitude_init(&state);
    run_ticks(&state, &in, 496U, &out); /* 496 ticks = 1.000 s of RTC time */
    return out.pitch_deg;
}

static void test_integration_step(void)
{
    /* 655 counts are 9.995 deg/s with the stock gyro scale; 1 s of rotation should read 9.995 deg. */
    const uint64_t compat = euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT);
    const uint64_t exact_norm = compat & ~(UINT64_C(1) << EUC_QUIRK_ID_FAST_INV_SQRT);
    const uint64_t gyro_500 = UINT64_C(1) << EUC_QUIRK_ID_GYRO_500DPS_CLIP; /* the counts below are +-500 */
    const uint64_t dt_only = gyro_500 | (UINT64_C(1) << EUC_QUIRK_ID_DT_500HZ);

    /* DT_500HZ alone: 2.000 ms integrated per 2.016 ms tick. */
    TEST_ASSERT_FLOAT_WITHIN(0.005F, 9.995F * (2.000F / 2.016F), pitch_after_one_second_of_gyro(dt_only));
    TEST_ASSERT_FLOAT_WITHIN(0.005F, 9.995F * (2.000F / 2.016F), pitch_after_one_second_of_gyro(exact_norm));
    TEST_ASSERT_FLOAT_WITHIN(0.005F, 9.995F, pitch_after_one_second_of_gyro(gyro_500));

    /* Full compat also has FAST_INV_SQRT, which leaves |q| a little below 1: pitch reads about 0.35 % lower.
     */
    TEST_ASSERT_FLOAT_WITHIN(0.005F, 9.881F, pitch_after_one_second_of_gyro(compat));
}

static void test_sequential_update_is_its_own_switch(void)
{
    struct euc_attitude_state stock;
    struct euc_attitude_state simultaneous;
    struct euc_attitude_out out;
    struct euc_attitude_in in = tilted_input(5.0, 3.0);
    in.gyro[0] = 3000;
    in.gyro[1] = -2000;
    in.gyro[2] = 2500;

    euc_attitude_init(&stock);
    run_ticks(&stock, &in, 50U, &out);

    /* Compat with only SEQ_QUAT_UPDATE switched off. */
    const uint64_t compat = euc_sil_quirk_profile_mask(EUC_SIL_PROFILE_COMPAT);
    euc_sil_quirk_set_mask(compat & ~(UINT64_C(1) << EUC_QUIRK_ID_SEQ_QUAT_UPDATE));
    euc_attitude_init(&simultaneous);
    run_ticks(&simultaneous, &in, 50U, &out);

    TEST_ASSERT_TRUE(stock.quaternion[3] != simultaneous.quaternion[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.01F, stock.quaternion[0], simultaneous.quaternion[0]);
}

static void test_free_fall_keeps_gravity_and_records_the_jump(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    euc_attitude_init(&state);
    const struct euc_attitude_in level = tilted_input(0.0, 0.0);
    run_ticks(&state, &level, 1000U, &out);
    const float settled_z = state.accel_z_lpf;

    struct euc_attitude_in falling = level;
    falling.accel[0] = 100;
    falling.accel[1] = 50;
    falling.accel[2] = 300; /* about 0.04 g */
    run_ticks(&state, &falling, 20U, &out);
    TEST_ASSERT_TRUE(state.free_fall);
    /* 11 ticks fed the real 300 counts, then the override fed 8192 again. */
    TEST_ASSERT_TRUE(state.accel_z_lpf > (settled_z * 0.85F));

    run_ticks(&state, &level, 26U, &out);
    TEST_ASSERT_FALSE(state.free_fall);
    TEST_ASSERT_EQUAL_UINT16(20U, state.last_jump_ticks);
}

static void test_free_fall_override_stops_with_the_motor(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    euc_attitude_init(&state);
    struct euc_attitude_in falling = tilted_input(0.0, 0.0);
    run_ticks(&state, &falling, 1000U, &out);
    falling.accel[0] = 0;
    falling.accel[1] = 0;
    falling.accel[2] = 0;
    falling.motor_disabled = true;
    run_ticks(&state, &falling, 200U, &out);
    TEST_ASSERT_TRUE(state.free_fall);
    TEST_ASSERT_TRUE(state.accel_z_lpf < 2000.0F); /* no 1 g override: the filter follows the real reading */
}

static void test_pitch_rate_filter_opens_above_15_kmh(void)
{
    struct euc_attitude_state state;
    struct euc_attitude_out out;
    euc_attitude_init(&state);
    struct euc_attitude_in in = tilted_input(0.0, 0.0);
    in.speed_mm_per_s = 5000; /* 18 km/h */
    run_ticks(&state, &in, 60U, &out);
    /* Stock detail: at the fast end the coefficient alternates between 0.1 and 0.09. */
    TEST_ASSERT_TRUE((state.pitch_rate_alpha > 0.089F) && (state.pitch_rate_alpha < 0.101F));
    in.speed_mm_per_s = 0;
    run_ticks(&state, &in, 60U, &out);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.6F, state.pitch_rate_alpha);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_gyro_range_per_profile);
    RUN_TEST(test_clean_integrates_2000dps_counts);
    RUN_TEST(test_clean_reanchors_after_the_gyro_rail);
    RUN_TEST(test_compat_never_reanchors);
    RUN_TEST(test_imu_offset_saturates);
    RUN_TEST(test_accelerometer_tilt_angles);
    RUN_TEST(test_level_and_still_stays_level);
    RUN_TEST(test_asin_accel_settles_below_the_true_tilt);
    RUN_TEST(test_roll_sign);
    RUN_TEST(test_integration_step);
    RUN_TEST(test_sequential_update_is_its_own_switch);
    RUN_TEST(test_free_fall_keeps_gravity_and_records_the_jump);
    RUN_TEST(test_free_fall_override_stops_with_the_motor);
    RUN_TEST(test_pitch_rate_filter_opens_above_15_kmh);
    return UNITY_END();
}
