/*
 * Balance task: one call per RTC tick (2.016 ms) turns IMU, speed and current readings into the motor
 * command.
 *
 * Order inside euc_balance_tick(), the same as the stock firmware:
 *   1. IMU offsets, accelerometer-only tilt, pedal-mode gains
 *   2. waiting for start-up: the start-up check only
 *   3. running: attitude; error angle = pitch + tiltbacks (battery/temperature, speed, PWM)
 *      + pedal trim - restart offset - handle offset; speed estimate; cut-off detectors;
 *      then the balance law, or (soft-disabled) the ramp-down and the recovery check
 *   4. output slew limit, soft-start limit, publish the motor command
 * Each step is also callable on its own (euc_balance_step_*), so a lock-step harness can compare it with the
 * matching stock function.
 *
 * Signals that come from other contexts (hall interrupt, current loop, alarm task) are inputs, read once per
 * tick. The tiltbacks use the speed estimate of the previous tick, as in the stock firmware.
 *
 * Units: angles in degrees, speeds as named (cm/s, mm/s, 0.1 m/s, m/s), the motor command in duty units
 * (+-1050 = full modulation; app PWM % = command / 10.5).
 */
#ifndef EUC_BALANCE_BALANCE_H
#define EUC_BALANCE_BALANCE_H

#include <stdbool.h>
#include <stdint.h>

#include "euc/attitude/attitude.h"

#define EUC_BALANCE_COMMAND_LIMIT 1050.0F /* duty units: full modulation */
#define EUC_BALANCE_MAX_BEEPS     3U

/* Rider settings the balance reads every tick (from the settings store and the BLE commands). */
struct euc_balance_settings {
    uint8_t pedal_mode;          /* 0 soft, 1 medium, 2 hard */
    uint8_t drive_mode;          /* 0 voltage-mode FOC (duty law), 1 current-mode FOC */
    uint16_t tiltback_speed_kmh; /* 200 = off */
    bool pwm_tiltback_enabled;   /* 'e' / 'x' */
    uint8_t pedal_trim;          /* 'W' 'U' d, 0..9: pedal angle (d - 4) x 1.2 deg */
    uint8_t turn_compensation;   /* 'W' 'R' d, 0..9 */
    uint8_t roll_limit_step;     /* 'W' 'X' d, 0..9: slow roll cut-off at 40 + 5 d deg */
    uint8_t roll_cutoff_mode;    /* 0..2: fast roll cut-off at 50 / 58 / 66 deg */
};

struct euc_balance_in {
    struct euc_balance_settings settings;
    int16_t accel_raw[EUC_IMU_AXES]; /* MPU6500 counts */
    int16_t gyro_raw[EUC_IMU_AXES];
    int16_t accel_offset[EUC_IMU_AXES]; /* calibration, counts */
    int16_t gyro_offset[EUC_IMU_AXES];
    int32_t battery_voltage_cv;      /* 0.01 V, filtered controller ADC */
    float board_temperature_mpu6050; /* board NTC in MPU6050 counts (14500 = about 80 degC) */
    bool handle_button_released;     /* handle / trolley button input level: true = not pressed */
    int16_t hall_steps_per_window;   /* signed hall transitions in the last 20 ms window */
    int16_t hall_steps_11_windows;   /* change of the hall count over 11 windows */
    int32_t hall_period_speed;       /* speed from hall periods, about 1200 per m/s */
    int32_t motor_current_lpf_ma;    /* low-passed q-axis current, mA */
    int16_t motor_voltage_q;         /* FOC Vq, 31500 = 100 % modulation */
    bool pwm_alarm_active;           /* alarm task: PWM above the 'W' 'P' threshold */
    bool hard_inhibit;               /* hard fault or lock: no automatic recovery */
    bool hall_fault;                 /* hall sensor fault: no automatic recovery */
    bool disable_request;            /* soft disable from commands or faults; takes effect after this tick */
};

/* Gains written every tick from the pedal mode and the drive mode. */
struct euc_balance_gains {
    float kp_base;    /* duty per degree at standstill (+1 per m/s) */
    float kd_base;    /* x0.1 duty per rad/s at standstill (+0.32 per cm/s) */
    int16_t kc_base;  /* current feedback gain at standstill */
    int16_t kc_slope; /* current feedback gain loss per 1000 cm/s */
    float kv;         /* hall-speed feed-forward, duty per 1000 counts */
    float accel_gain; /* Mahony accelerometer gain */
};

struct euc_balance_state {
    struct euc_attitude_state attitude;
    struct euc_balance_gains gains;

    int16_t start_state; /* < 0: counting up to 0; 0: waiting for a level wheel; 1: running */
    bool motor_disabled; /* soft disable: the command ramps to zero */
    bool fallen;         /* the disable came from the pitch cut-off: automatic recovery allowed */
    bool tipped;         /* a roll cut-off fired */

    /* Error angle, degrees, and the offsets added to it */
    float error_deg;
    float error_prev_deg; /* for the current-mode D term */
    float tiltback_battery_deg;
    float tiltback_speed_deg;
    float tiltback_pwm_deg;
    float pedal_trim_deg;
    float restart_offset_deg;
    float handle_offset_deg;

    /* Speed estimate from the hall counts */
    int32_t speed_filter;   /* internal: 10 x cm/s */
    int16_t speed_cm_per_s; /* gain scheduling and command guards; the law clamps it to +-2500 */
    int16_t speed_mm_per_s; /* also telemetry frame 00 x 10 */
    int16_t speed_dm_per_s;
    float speed_m_per_s;
    int16_t frame01_speed_copy_dm_per_s; /* telemetry frame 01 bytes 4-5 */

    /* Cut-off detectors and recovery */
    uint16_t pitch_over_ticks;
    uint16_t error_over_ticks;
    uint16_t overload_ticks;
    uint16_t roll_over_ticks;
    int16_t recovery_ticks;

    /* Handle button: two presses at standstill cut the motor, the third restores it */
    uint8_t handle_presses;
    uint16_t handle_press_ticks;
    uint16_t handle_release_ticks;
    bool handle_press_latched;

    /* Output */
    float command; /* this tick's command, duty units */
    float command_prev;
    float command_limit; /* soft-start limit: +1 per tick up to 1050 */
};

struct euc_balance_out {
    float motor_command; /* duty units, +-1050; to the current loop */
    float pitch_deg;
    float roll_deg;
    int16_t speed_cm_per_s;
    bool motor_disabled;
    bool handle_double_beep; /* second handle press: the alarm task plays a double beep */
    uint8_t beep_count;
    uint8_t beep_ticks[EUC_BALANCE_MAX_BEEPS]; /* requested beeps, length in ticks */
};

/* Power-on state: level attitude, motor command 0, 200 ticks before the first start-up check. */
void euc_balance_init(struct euc_balance_state *state);

/* The motor side calls this when it first enables the PWM with the wheel still (stock: 10 ticks). */
void euc_balance_restart_countdown(struct euc_balance_state *state, int16_t ticks);

/* One RTC tick. Clears `out` first. */
void euc_balance_tick(struct euc_balance_state *state, const struct euc_balance_in *in,
                      struct euc_balance_out *out);

/* Steps of euc_balance_tick(), for lock-step comparison with the matching stock functions. */
void euc_balance_step_gains(struct euc_balance_state *state, const struct euc_balance_settings *settings);
void euc_balance_step_startup(struct euc_balance_state *state);
void euc_balance_step_tiltbacks(struct euc_balance_state *state, const struct euc_balance_in *in);
void euc_balance_step_handle(struct euc_balance_state *state, const struct euc_balance_in *in,
                             struct euc_balance_out *out);
void euc_balance_step_speed(struct euc_balance_state *state, const struct euc_balance_in *in);
void euc_balance_step_cutoffs(struct euc_balance_state *state, const struct euc_balance_in *in,
                              struct euc_balance_out *out);
void euc_balance_step_law(struct euc_balance_state *state, const struct euc_balance_in *in);
void euc_balance_step_ramp_down(struct euc_balance_state *state);
void euc_balance_step_recovery(struct euc_balance_state *state, const struct euc_balance_in *in,
                               struct euc_balance_out *out);
void euc_balance_step_output(struct euc_balance_state *state);

#endif
