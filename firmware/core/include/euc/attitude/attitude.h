/*
 * Attitude estimation from the MPU6500 (accelerometer +-4 g, gyro +-500 deg/s), once per RTC tick.
 *
 * Three parts, in the order the balance task calls them:
 *   euc_imu_correct()           calibration offset, saturating, per raw sample
 *   euc_attitude_update_tilt()  accelerometer-only tilt per axis (0.01 deg), for start-up and recovery
 *   euc_attitude_update()       free-fall detector and Mahony filter (proportional term only): pitch,
 *                               roll and a low-passed pitch rate for the balance law's D term
 *
 * The filter follows the stock firmware step for step (float or double per step, rounding through
 * euc/math/float_ops.h). Compat behaviours (config/quirks.def): ASIN_ACCEL, SEQ_QUAT_UPDATE, FAST_INV_SQRT,
 * KEIL_FTZ, DT_500HZ, NAN_FAIL_OPEN. In the clean profile a NaN or infinity anywhere in the filter resets it
 * and reports out.valid = false.
 *
 * Axes are the sensor's; pitch is rotation about +y and roll about +x. How the board is mounted in the wheel
 * is not known yet; with x forward, positive pitch is nose down (inferred).
 */
#ifndef EUC_ATTITUDE_ATTITUDE_H
#define EUC_ATTITUDE_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_IMU_ACCEL_COUNTS_PER_G 8192 /* MPU6500 at +-4 g */
#define EUC_IMU_AXES               3U

struct euc_attitude_in {
    int16_t accel[EUC_IMU_AXES]; /* offset-corrected, 8192 counts per g */
    int16_t gyro[EUC_IMU_AXES];  /* offset-corrected, 65.5 counts per deg/s */
    float accel_gain;            /* Mahony gain on the accelerometer error (from the pedal mode) */
    uint8_t turn_compensation;   /* 'W' 'R' d, 0..9: share of centripetal acceleration removed */
    int16_t speed_dm_per_s;      /* wheel speed, 0.1 m/s */
    int16_t speed_mm_per_s;      /* wheel speed, mm/s; above 15 km/h the pitch-rate filter opens up */
    bool motor_disabled;         /* soft disable: the free-fall gravity override stops */
    bool hard_inhibit;           /* hard fault or lock: same */
};

struct euc_attitude_state {
    float quaternion[4]; /* w, x, y, z */
    float accel_x_lpf;   /* low-passed accelerometer, counts */
    float accel_y_lpf;
    float accel_z_lpf;
    float pitch_rate_alpha;         /* pitch-rate filter coefficient, slews between 0.1 and 0.6 */
    float pitch_rate_lpf_rad_per_s; /* low-passed raw y gyro, rad/s */
    float pitch_deg;                /* last estimate */
    float roll_deg;
    int16_t tilt_centideg[EUC_IMU_AXES]; /* accelerometer-only tilt, 0.01 deg */
    /* Free-fall detector: all three axes below 0.3 g for more than 10 ticks. */
    uint16_t low_g_ticks;
    uint16_t normal_g_ticks;
    bool low_g_seen;
    bool free_fall;           /* while set, z reads 1 g so the filter keeps its gravity reference */
    uint16_t last_jump_ticks; /* length of the last free fall (stock debug value "L9") */
};

struct euc_attitude_out {
    float pitch_deg;
    float roll_deg;
    float pitch_rate_rad_per_s;
    bool valid; /* false: a non-finite value appeared and the filter was reset (clean, NAN_FAIL_OPEN off) */
};

/* Power-on state: level quaternion, pitch-rate filter at its low-speed setting. */
void euc_attitude_init(struct euc_attitude_state *state);

/* raw - offset, saturated to +-32767. */
int16_t euc_imu_correct(int16_t raw, int16_t offset);

/* Accelerometer-only tilt angles (every tick, also before start-up). */
void euc_attitude_update_tilt(struct euc_attitude_state *state, const int16_t accel[EUC_IMU_AXES]);

/* Free-fall detector, Mahony update, pitch and roll (every tick while the balance runs). */
void euc_attitude_update(struct euc_attitude_state *state, const struct euc_attitude_in *in,
                         struct euc_attitude_out *out);

#endif
