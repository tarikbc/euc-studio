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

    const int16_t accel_z = free_fall_update(state, in);
    float accel[3];
    accel_vector(state, in, accel_z, accel);

    float gyro[3];
    for (size_t axis = 0U; axis < 3U; axis++) {
        gyro[axis] = euc_f32_from_f64((double)in->gyro[axis] * GYRO_RAD_PER_S_PER_COUNT);
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
        rate[axis] = euc_f32_add(euc_f32_add(euc_f32_mul(error[axis], in->accel_gain), gyro[axis]),
                                 MAHONY_INTEGRAL_RAD_PER_S);
    }
    quaternion_integrate(q, rate);

    /* pitch = asin(2 (q0 q2 - q1 q3)), roll = atan2(2 (q2 q3 + q0 q1), 1 - 2 (q1^2 + q2^2)), in degrees */
    const float twice_q0 = euc_f32_mul(q[0], 2.0F);
    const float twice_q2 = euc_f32_mul(q[2], 2.0F);
    const float pitch_sine =
        euc_f32_add(euc_f32_mul(euc_f32_mul(q[1], -2.0F), q[3]), euc_f32_mul(twice_q0, q[2]));
    state->pitch_deg = euc_f32_from_f64(euc_asin((double)pitch_sine) * RADIANS_TO_DEGREES);
    const float roll_cos = euc_f32_add(
        euc_f32_sub(euc_f32_mul(euc_f32_mul(q[1], -2.0F), q[1]), euc_f32_mul(twice_q2, q[2])), 1.0F);
    const float roll_sin = euc_f32_add(euc_f32_mul(twice_q2, q[3]), euc_f32_mul(twice_q0, q[1]));
    state->roll_deg = euc_f32_from_f64(euc_atan2((double)roll_sin, (double)roll_cos) * RADIANS_TO_DEGREES);

    out->pitch_deg = state->pitch_deg;
    out->roll_deg = state->roll_deg;
    out->pitch_rate_rad_per_s = state->pitch_rate_lpf_rad_per_s;
}
