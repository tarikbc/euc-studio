/*
 * Attitude estimation. Every float step names its precision on purpose: the stock firmware mixes float and
 * double, and the compat profile must round exactly like it (see euc/math/float_ops.h).
 */
#include "euc/attitude/attitude.h"

#include <float.h>
#include <stddef.h>

#include "euc/math/float_ops.h"
#include "euc/math/libm.h"
#include "euc/quirks.h"

/* Radians to degrees as the stock firmware writes it (180 / pi = 57.2957795...; the difference is 4e-9). */
#define RADIANS_TO_DEGREES 57.29578

/* Accelerometer counts to g used by the filter (the stock constant, about 1/8192). */
#define ACCEL_G_PER_COUNT 0.000122
/* Gyro counts to rad/s (the stock constant; 1/65.5 deg/s would be 0.000266462). */
#define GYRO_RAD_PER_S_PER_COUNT 0.000266325
/* Clean, +-2000 deg/s: 1/16.4 deg/s per count. */
#define GYRO_2000_RAD_PER_S_PER_COUNT 0.0010642251
/* +-2000 deg/s counts to +-500 deg/s counts: 65.5 / 16.4 (the turn compensation is tuned in those) */
#define GYRO_2000_TO_500_NUM 655
#define GYRO_2000_TO_500_DEN 164
#define GYRO_PAGE_SCALE      4 /* flash offsets are +-500 deg/s counts */

/* Clean re-anchoring after the gyro left its rail. */
#define REANCHOR_TICKS       496U /* 1 s of raised gain */
#define REANCHOR_GAIN_FACTOR 10.0F
#define REANCHOR_GAIN_MAX    3.0F
#define REANCHOR_ONE_G_LOW   6963U /* 0.85 g */
#define REANCHOR_ONE_G_HIGH  9421U /* 1.15 g */

/* Asin input limit for the accelerometer vector (ASIN_ACCEL). */
#define ACCEL_ASIN_LIMIT 0.99F

/* Free fall: every axis below this many counts (about 0.3 g) for more than LOW_G_TICKS ticks. */
#define FREE_FALL_THRESHOLD_COUNTS 2500
#define FREE_FALL_LOW_G_TICKS      10U
#define FREE_FALL_EXIT_TICKS       25U

/* Turn compensation: y acceleration minus clamp(gz) x speed x 0.00533 x k. */
#define TURN_GYRO_LIMIT_COUNTS 3000
#define TURN_COMPENSATION_GAIN 0.00533
#define TURN_COMPENSATION_MAX  9U

/* Pitch-rate filter: alpha slews by 0.01 per tick toward 0.6 (slow) or 0.1 (above 15 km/h). */
#define RATE_FILTER_FAST_KMH 15.0
#define RATE_ALPHA_SLOW      0.6F
#define RATE_ALPHA_FAST      0.1F
#define RATE_ALPHA_STEP      0.01

/*
 * Half the integration step. DT_500HZ: the stock filter assumes 500 Hz; the RTC tick is 8 MHz / 128 / 126 =
 * 496.03 Hz (2.016 ms).
 */
#define HALF_STEP_500HZ_S 0.001F
#define HALF_STEP_TICK_S  0.001008F

/*
 * The stock filter adds an integral correction that nothing ever writes, so it is always +0.0. Adding it is
 * not a no-op in IEEE arithmetic: it turns a -0.0 sum into +0.0, so the compat profile keeps the addition.
 */
#define MAHONY_INTEGRAL_RAD_PER_S 0.0F

/* Accelerometer-only tilt: low-pass 0.95 / 0.05 of asin(a) in 0.01 deg. */
#define TILT_CENTIDEG_PER_RAD_X10 57295.0
#define TILT_LPF_KEEP_PERCENT     95
#define TILT_LPF_NEW_PERCENT      5

void euc_attitude_init(struct euc_attitude_state *state)
{
    *state = (struct euc_attitude_state){0};
    state->quaternion[0] = 1.0F;
    state->pitch_rate_alpha = RATE_ALPHA_SLOW;
}

/* True when every float the filter carries to the next tick is finite. */
static bool filter_finite(const struct euc_attitude_state *state)
{
    const float values[] = {state->quaternion[0], state->quaternion[1],    state->quaternion[2],
                            state->quaternion[3], state->accel_x_lpf,      state->accel_y_lpf,
                            state->accel_z_lpf,   state->pitch_rate_alpha, state->pitch_rate_lpf_rad_per_s,
                            state->pitch_deg,     state->roll_deg};
    for (size_t index = 0U; index < (sizeof(values) / sizeof(values[0])); index++) {
        if (!euc_f32_is_finite(values[index])) {
            return false;
        }
    }
    return true;
}

/* Clean: back to the power-on filter, keeping the accelerometer-only tilt and the free-fall detector. */
static void filter_reset(struct euc_attitude_state *state)
{
    state->quaternion[0] = 1.0F;
    state->quaternion[1] = 0.0F;
    state->quaternion[2] = 0.0F;
    state->quaternion[3] = 0.0F;
    state->accel_x_lpf = 0.0F;
    state->accel_y_lpf = 0.0F;
    state->accel_z_lpf = 0.0F;
    state->pitch_rate_alpha = RATE_ALPHA_SLOW;
    state->pitch_rate_lpf_rad_per_s = 0.0F;
    state->pitch_deg = 0.0F;
    state->roll_deg = 0.0F;
}

uint8_t euc_imu_gyro_config(void)
{
    return EUC_QUIRK(GYRO_500DPS_CLIP) ? (uint8_t)EUC_IMU_GYRO_DPS_500 : (uint8_t)EUC_IMU_GYRO_DPS_2000;
}

int16_t euc_imu_gyro_offset_from_page(int16_t stored)
{
    if (EUC_QUIRK(GYRO_500DPS_CLIP)) {
        return stored;
    }
    /* nearest +-2000 deg/s count */
    const int32_t value = stored;
    const int32_t half = GYRO_PAGE_SCALE / 2;
    const int32_t rounded =
        (value >= 0) ? ((value + half) / GYRO_PAGE_SCALE) : ((value - half) / GYRO_PAGE_SCALE);
    return (int16_t)rounded;
}

int16_t euc_imu_gyro_offset_to_page(int16_t offset)
{
    if (EUC_QUIRK(GYRO_500DPS_CLIP)) {
        return offset;
    }
    const int32_t scaled = (int32_t)offset * GYRO_PAGE_SCALE;
    if (scaled > INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}

int16_t euc_imu_correct(int16_t raw, int16_t offset)
{
    const int32_t corrected = (int32_t)raw - (int32_t)offset;
    if (corrected > INT16_MAX) {
        return INT16_MAX;
    }
    if (corrected < -INT16_MAX) {
        return -INT16_MAX;
    }
    return (int16_t)corrected;
}

void euc_attitude_update_tilt(struct euc_attitude_state *state, const int16_t accel[EUC_IMU_AXES])
{
    for (size_t axis = 0U; axis < EUC_IMU_AXES; axis++) {
        float sine = euc_f32_from_f64((double)accel[axis] * ACCEL_G_PER_COUNT);
        if (sine > 1.0F) {
            sine = 1.0F;
        } else if (sine < -1.0F) {
            sine = -1.0F;
        } else {
            /* in range */
        }
        const int32_t angle_centideg =
            (int32_t)euc_f32_from_f64((euc_asin((double)sine) * TILT_CENTIDEG_PER_RAD_X10) / 10.0);
        const int32_t filtered = (((int32_t)state->tilt_centideg[axis] * TILT_LPF_KEEP_PERCENT) +
                                  (angle_centideg * TILT_LPF_NEW_PERCENT)) /
                                 100;
        state->tilt_centideg[axis] = (int16_t)filtered;
    }
}

/* Free-fall detector. Returns the z acceleration the filter should use: 1 g while falling with the motor on.
 */
static int16_t free_fall_update(struct euc_attitude_state *state, const struct euc_attitude_in *in)
{
    const bool low_g = (in->accel[0] < FREE_FALL_THRESHOLD_COUNTS) &&
                       (in->accel[1] < FREE_FALL_THRESHOLD_COUNTS) &&
                       (in->accel[2] < FREE_FALL_THRESHOLD_COUNTS);
    bool override_allowed = true;
    if (low_g) {
        state->low_g_ticks++;
        state->low_g_seen = true;
        if (state->low_g_ticks > FREE_FALL_LOW_G_TICKS) {
            state->free_fall = true;
        }
        state->normal_g_ticks = 0U;
    } else {
        state->normal_g_ticks++;
        if (state->normal_g_ticks > FREE_FALL_EXIT_TICKS) {
            state->free_fall = false;
            state->normal_g_ticks = FREE_FALL_EXIT_TICKS;
            if (state->low_g_seen && (state->low_g_ticks > FREE_FALL_LOW_G_TICKS)) {
                state->last_jump_ticks = state->low_g_ticks;
            }
            state->low_g_ticks = 0U;
            state->low_g_seen = false;
            override_allowed = false;
        }
    }
    if (override_allowed && state->free_fall && !in->motor_disabled && !in->hard_inhibit) {
        return EUC_IMU_ACCEL_COUNTS_PER_G;
    }
    return in->accel[2];
}

/* Low-pass filters and turn compensation; returns the accelerometer vector the filter corrects toward. */
static void accel_vector(struct euc_attitude_state *state, const struct euc_attitude_in *in, int16_t accel_z,
                         float vector[3])
{
    int32_t gyro_z = in->gyro[2];
    if (!EUC_QUIRK(GYRO_500DPS_CLIP)) {
        gyro_z = (gyro_z * GYRO_2000_TO_500_NUM) / GYRO_2000_TO_500_DEN;
    }
    if (gyro_z > TURN_GYRO_LIMIT_COUNTS) {
        gyro_z = TURN_GYRO_LIMIT_COUNTS;
    } else if (gyro_z < -TURN_GYRO_LIMIT_COUNTS) {
        gyro_z = -TURN_GYRO_LIMIT_COUNTS;
    } else {
        /* in range */
    }
    state->accel_y_lpf = euc_f32_from_f64(((double)state->accel_y_lpf * 0.9) + ((double)in->accel[1] * 0.1));
    const uint8_t turn_k =
        (in->turn_compensation > TURN_COMPENSATION_MAX) ? TURN_COMPENSATION_MAX : in->turn_compensation;
    const int32_t speed_times_rate = (int32_t)in->speed_dm_per_s * gyro_z;
    const float accel_y_compensated = euc_f32_from_f64(
        (double)state->accel_y_lpf - (((double)speed_times_rate * TURN_COMPENSATION_GAIN) * (double)turn_k));
    state->accel_z_lpf = euc_f32_from_f64(((double)state->accel_z_lpf * 0.99) + ((double)accel_z * 0.01));
    state->accel_x_lpf =
        euc_f32_from_f64(((double)state->accel_x_lpf * 0.995) + ((double)in->accel[0] * 0.005));

    const float filtered[3] = {state->accel_x_lpf, accel_y_compensated, state->accel_z_lpf};
    for (size_t axis = 0U; axis < 3U; axis++) {
        if (EUC_QUIRK(ASIN_ACCEL)) {
            /* Stock: each axis as an angle, asin(clamp(a / g, +-0.99)), before normalising. */
            float sine = euc_f32_from_f64((double)filtered[axis] * ACCEL_G_PER_COUNT);
            if (sine >= ACCEL_ASIN_LIMIT) {
                sine = ACCEL_ASIN_LIMIT;
            } else if (sine < -ACCEL_ASIN_LIMIT) {
                sine = -ACCEL_ASIN_LIMIT;
            } else {
                /* in range */
            }
            vector[axis] = euc_f32_from_f64(euc_asin((double)sine));
        } else {
            vector[axis] = filtered[axis]; /* the normalisation removes the scale */
        }
    }
}

/* Pitch-rate low-pass for the D term (raw y gyro, before the filter's correction). */
static void pitch_rate_update(struct euc_attitude_state *state, const struct euc_attitude_in *in,
                              float gyro_y)
{
    const int32_t speed_cm_per_s = in->speed_mm_per_s / 10;
    const int32_t speed_magnitude = (speed_cm_per_s < 0) ? -speed_cm_per_s : speed_cm_per_s;
    const double speed_kmh = ((double)speed_magnitude * 3.6) / 100.0;
    float alpha = state->pitch_rate_alpha;
    if (speed_kmh > RATE_FILTER_FAST_KMH) {
        alpha =
            (alpha < RATE_ALPHA_FAST) ? RATE_ALPHA_FAST : euc_f32_from_f64((double)alpha - RATE_ALPHA_STEP);
    } else {
        alpha = ((alpha >= RATE_ALPHA_SLOW) || (alpha <= -FLT_MIN))
                    ? RATE_ALPHA_SLOW
                    : euc_f32_from_f64((double)alpha + RATE_ALPHA_STEP);
    }
    state->pitch_rate_alpha = alpha;
    const float one_minus_alpha = euc_f32_sub(1.0F, alpha);
    state->pitch_rate_lpf_rad_per_s = euc_f32_add(euc_f32_mul(state->pitch_rate_lpf_rad_per_s, alpha),
                                                  euc_f32_mul(gyro_y, one_minus_alpha));
}

static bool accel_near_one_g(const int16_t accel[EUC_IMU_AXES])
{
    uint32_t sum = 0U; /* at most 3 x 32768^2, below 2^32 */
    for (size_t axis = 0U; axis < EUC_IMU_AXES; axis++) {
        const int32_t value = accel[axis];
        const uint32_t magnitude = (uint32_t)((value < 0) ? -value : value);
        sum += magnitude * magnitude;
    }
    return (sum >= (REANCHOR_ONE_G_LOW * REANCHOR_ONE_G_LOW)) &&
           (sum <= (REANCHOR_ONE_G_HIGH * REANCHOR_ONE_G_HIGH));
}

/* Clean: the accelerometer correction gain for this tick. Once the gyro leaves its rail, the first 1 g
 * reading re-seeds the low-passed accelerometer (the impact that clipped the gyro also shook it) and the gain
 * rises 10 x for REANCHOR_TICKS. */
static float correction_gain(struct euc_attitude_state *state, const struct euc_attitude_in *in)
{
    float gain = in->accel_gain;
    if (in->gyro_at_rail) {
        state->reanchor_pending = true;
        state->reanchor_ticks = 0U;
        return gain;
    }
    if (state->reanchor_pending && accel_near_one_g(in->accel)) {
        state->reanchor_pending = false;
        state->reanchor_ticks = REANCHOR_TICKS;
        state->accel_x_lpf = (float)in->accel[0];
        state->accel_y_lpf = (float)in->accel[1];
        state->accel_z_lpf = (float)in->accel[2];
    }
    if (state->reanchor_ticks > 0U) {
        state->reanchor_ticks--;
        float raised = euc_f32_mul(gain, REANCHOR_GAIN_FACTOR);
        if (raised > REANCHOR_GAIN_MAX) {
            raised = REANCHOR_GAIN_MAX;
        }
        if (raised > gain) {
            gain = raised;
        }
    }
    return gain;
}

/* Integrates the corrected rates into the quaternion and normalises it. */
static void quaternion_integrate(float q[4], const float rate[3])
{
    const float half_step = EUC_QUIRK(DT_500HZ) ? HALF_STEP_500HZ_S : HALF_STEP_TICK_S;
    const float q0 = q[0];
    const float q1 = q[1];
    const float q2 = q[2];
    const float q3 = q[3];
    float n0 =
        euc_f32_add(euc_f32_mul(euc_f32_sub(euc_f32_sub(euc_f32_mul(-q1, rate[0]), euc_f32_mul(q2, rate[1])),
                                            euc_f32_mul(q3, rate[2])),
                                half_step),
                    q0);
    /* SEQ_QUAT_UPDATE: the stock code reads the components it has already updated in this step. */
    const bool sequential = EUC_QUIRK(SEQ_QUAT_UPDATE);
    const float p0 = sequential ? n0 : q0;
    float n1 =
        euc_f32_add(euc_f32_mul(euc_f32_sub(euc_f32_add(euc_f32_mul(p0, rate[0]), euc_f32_mul(q2, rate[2])),
                                            euc_f32_mul(q3, rate[1])),
                                half_step),
                    q1);
    const float p1 = sequential ? n1 : q1;
    float n2 =
        euc_f32_add(euc_f32_mul(euc_f32_add(euc_f32_sub(euc_f32_mul(p0, rate[1]), euc_f32_mul(p1, rate[2])),
                                            euc_f32_mul(q3, rate[0])),
                                half_step),
                    q2);
    const float p2 = sequential ? n2 : q2;
    float n3 =
        euc_f32_add(euc_f32_mul(euc_f32_sub(euc_f32_add(euc_f32_mul(p0, rate[2]), euc_f32_mul(p1, rate[1])),
                                            euc_f32_mul(p2, rate[0])),
                                half_step),
                    q3);

    const float norm = euc_f32_inv_sqrt(
        euc_f32_add(euc_f32_add(euc_f32_add(euc_f32_mul(n0, n0), euc_f32_mul(n1, n1)), euc_f32_mul(n2, n2)),
                    euc_f32_mul(n3, n3)));
    q[0] = euc_f32_mul(n0, norm);
    q[1] = euc_f32_mul(n1, norm);
    q[2] = euc_f32_mul(n2, norm);
    q[3] = euc_f32_mul(n3, norm);
}

void euc_attitude_update(struct euc_attitude_state *state, const struct euc_attitude_in *in,
                         struct euc_attitude_out *out)
{
    float *q = state->quaternion;

    /* Gravity direction from the quaternion before this step. */
    const float q0q0 = euc_f32_mul(q[0], q[0]);
    const float q0q1 = euc_f32_mul(q[0], q[1]);
    const float q0q2 = euc_f32_mul(q[0], q[2]);
    const float q1q1 = euc_f32_mul(q[1], q[1]);
    const float q1q3 = euc_f32_mul(q[1], q[3]);
    const float q2q2 = euc_f32_mul(q[2], q[2]);
    const float q2q3 = euc_f32_mul(q[2], q[3]);
    const float q3q3 = euc_f32_mul(q[3], q[3]);

    const float gain = EUC_QUIRK(GYRO_500DPS_CLIP) ? in->accel_gain : correction_gain(state, in);
    const int16_t accel_z = free_fall_update(state, in);
    float accel[3];
    accel_vector(state, in, accel_z, accel);

    const double rad_per_count =
        EUC_QUIRK(GYRO_500DPS_CLIP) ? GYRO_RAD_PER_S_PER_COUNT : GYRO_2000_RAD_PER_S_PER_COUNT;
    float gyro[3];
    for (size_t axis = 0U; axis < 3U; axis++) {
        gyro[axis] = euc_f32_from_f64((double)in->gyro[axis] * rad_per_count);
    }
    pitch_rate_update(state, in, gyro[1]);

    const float accel_norm_squared =
        euc_f32_add(euc_f32_add(euc_f32_mul(accel[0], accel[0]), euc_f32_mul(accel[1], accel[1])),
                    euc_f32_mul(accel[2], accel[2]));
    float error[3] = {0.0F, 0.0F, 0.0F};
    if (EUC_QUIRK(FAST_INV_SQRT) || (accel_norm_squared > 0.0F)) {
        /* (clean skips the correction for a zero vector, where 1/sqrt would be infinite) */
        const float norm = euc_f32_inv_sqrt(accel_norm_squared);
        const float ax = euc_f32_mul(accel[0], norm);
        const float ay = euc_f32_mul(accel[1], norm);
        const float az = euc_f32_mul(accel[2], norm);
        const float gravity_x = euc_f32_mul(euc_f32_sub(q1q3, q0q2), 2.0F);
        const float gravity_y = euc_f32_mul(euc_f32_add(q0q1, q2q3), 2.0F);
        const float gravity_z = euc_f32_add(euc_f32_sub(euc_f32_sub(q0q0, q1q1), q2q2), q3q3);
        /* error = measured x estimated gravity */
        error[0] = euc_f32_sub(euc_f32_mul(ay, gravity_z), euc_f32_mul(az, gravity_y));
        error[1] = euc_f32_sub(euc_f32_mul(az, gravity_x), euc_f32_mul(ax, gravity_z));
        error[2] = euc_f32_sub(euc_f32_mul(ax, gravity_y), euc_f32_mul(ay, gravity_x));
    }

    float rate[3];
    for (size_t axis = 0U; axis < 3U; axis++) {
        rate[axis] =
            euc_f32_add(euc_f32_add(euc_f32_mul(error[axis], gain), gyro[axis]), MAHONY_INTEGRAL_RAD_PER_S);
    }
    quaternion_integrate(q, rate);

    /* pitch = asin(2 (q0 q2 - q1 q3)), roll = atan2(2 (q2 q3 + q0 q1), 1 - 2 (q1^2 + q2^2)), in degrees */
    const float twice_q0 = euc_f32_mul(q[0], 2.0F);
    const float twice_q2 = euc_f32_mul(q[2], 2.0F);
    float pitch_sine = euc_f32_add(euc_f32_mul(euc_f32_mul(q[1], -2.0F), q[3]), euc_f32_mul(twice_q0, q[2]));
    if (!EUC_QUIRK(NAN_FAIL_OPEN)) {
        /* Rounding can push the sine just past +-1, where asin is NaN. The stock code does not clamp. */
        if (pitch_sine > 1.0F) {
            pitch_sine = 1.0F;
        } else if (pitch_sine < -1.0F) {
            pitch_sine = -1.0F;
        } else {
            /* in range (or NaN, caught below) */
        }
    }
    state->pitch_deg = euc_f32_from_f64(euc_asin((double)pitch_sine) * RADIANS_TO_DEGREES);
    const float roll_cos = euc_f32_add(
        euc_f32_sub(euc_f32_mul(euc_f32_mul(q[1], -2.0F), q[1]), euc_f32_mul(twice_q2, q[2])), 1.0F);
    const float roll_sin = euc_f32_add(euc_f32_mul(twice_q2, q[3]), euc_f32_mul(twice_q0, q[1]));
    state->roll_deg = euc_f32_from_f64(euc_atan2((double)roll_sin, (double)roll_cos) * RADIANS_TO_DEGREES);

    out->valid = true;
    if (!EUC_QUIRK(NAN_FAIL_OPEN) && !filter_finite(state)) {
        filter_reset(state);
        out->valid = false;
    }
    out->pitch_deg = state->pitch_deg;
    out->roll_deg = state->roll_deg;
    out->pitch_rate_rad_per_s = state->pitch_rate_lpf_rad_per_s;
}
