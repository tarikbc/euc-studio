/*
 * Beeper and alarms: one call per RTC tick (2.016 ms), after the balance task and before the buttons.
 *
 * Beeper: TIM4 CH4 at a fixed 2.70 kHz; the loudness is the compare value CCR4. euc_beeper_beep(n) loads the
 * compare with the current volume for n ticks; a later request replaces an earlier one. The volume follows
 * 'W' 'B' d every tick: CCR4 = (d - 1) x 30 + 1, clamped to 1..270. Stock turns d = 0 into the loudest
 * setting (quirk VOLUME_ZERO_MAX); clean makes it the quietest.
 *
 * Alarms, in stock order (each beep pattern repeats on the alarm time base, 1 tick = 2.016 ms):
 *   hall fault        2 x 20-tick beeps every 250 ticks
 *   hard cut          50-tick beep every 500 ticks for the first 2500 ticks (not while powering off)
 *   PWM               |Vq / 30| > 10 x 'W' 'P': alert bit 0, full volume, 35-tick beep every 100 ticks
 *   over-temperature  board NTC > 14500 (MPU6050 counts, about 80 degC): bit 5, 2 beeps every 1000 ticks
 *   BMS fault         500-tick beep every 5000 ticks
 *   battery voltage   < 105.60 V: bit 3, 1 / 2 / 3 beeps every 1000 ticks below 2 / 4 / above 4 m/s;
 *                     >= 137.61 V: bit 4, 3 beeps every 1000 ticks
 *   speed             mode 0 above 8.3 m/s: 2 x 35-tick beeps every 500 ticks; modes 0 and 1 above 12.5 m/s:
 *                     3 beeps; otherwise the handle double beep (2 x 25 ticks) when the balance asks for it
 *   horn ('b')        two 35-tick beeps 99 ticks apart
 *   main loop stall   clean: 3 x 20-tick beeps every 500 ticks while the main loop is stalled
 * A pattern stays quiet while a higher alarm (hard cut, PWM, temperature, voltage) is active.
 *
 * Alert byte (frame 04 byte 14): bits 0, 3, 4, 5 and 7 (transport lock) follow their condition; bits 1, 2
 * and 6 (current sensor, IMU, hall) are set by their owners through raise_alerts and stay set.
 */
#ifndef EUC_ALARMS_ALARMS_H
#define EUC_ALARMS_ALARMS_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_ALERT_PWM_HIGH         0x01U
#define EUC_ALERT_CURRENT_SENSOR   0x02U
#define EUC_ALERT_IMU_FAULT        0x04U
#define EUC_ALERT_UNDER_VOLTAGE    0x08U
#define EUC_ALERT_OVER_VOLTAGE     0x10U
#define EUC_ALERT_OVER_TEMPERATURE 0x20U
#define EUC_ALERT_HALL_SENSOR      0x40U
#define EUC_ALERT_TRANSPORT_LOCK   0x80U

struct euc_beeper {
    uint16_t ticks_left;
    uint16_t compare;        /* TIM4 CCR4 now: 0 = silent */
    uint16_t volume_compare; /* CCR4 a beep starts with */
};

struct euc_alarm_in {
    int16_t speed_dm_per_s;          /* balance speed estimate, 0.1 m/s */
    int32_t battery_voltage_cv;      /* 0.01 V */
    float board_temperature_mpu6050; /* board NTC in MPU6050 counts */
    int16_t motor_voltage_q;         /* active Vq, 31500 = 100 % */
    uint8_t beeper_volume;           /* 'W' 'B' d */
    uint8_t pwm_alarm_percent;       /* 'W' 'P' d d */
    uint8_t speed_alarm_mode;        /* 0..3 */
    bool hard_cut;                   /* bridge off until reset */
    bool powering_off;               /* the power-off path is running: no hard-cut reminder */
    bool hall_fault;
    bool main_loop_stalled; /* clean: the main loop stopped while the tick runs */
    bool bms_fault;
    bool horn_request;       /* 'b' */
    bool handle_double_beep; /* balance: second handle press */
    bool transport_locked;
    uint8_t raise_alerts; /* latched bits to add: EUC_ALERT_CURRENT_SENSOR, _IMU_FAULT, _HALL_SENSOR */
};

struct euc_alarm_state {
    struct euc_beeper beeper;
    int16_t time_base; /* -29999..30001, +1 per tick */
    uint8_t alerts;    /* frame 04 byte 14 */
    bool hard_cut_alarm;
    bool pwm_alarm;
    bool voltage_alarm;
    bool temperature_alarm;
    bool speed_alarm;
    uint16_t hard_cut_ticks;
    bool horn_active;
    uint8_t horn_ticks;
    bool double_beep_pending;
    uint8_t double_beeps_done;
};

struct euc_alarm_out {
    uint8_t alerts;            /* frame 04 byte 14 */
    bool pwm_alarm;            /* balance input pwm_alarm_active (PWM tiltback) */
    uint8_t pwm_alarm_percent; /* 'W' 'P' clamped to 50..90; stock writes the clamp back to the setting */
    uint16_t time_base;        /* alarm time base, 1..60001: the light strobe phase */
};

/* Power-on state. */
void euc_alarm_init(struct euc_alarm_state *state);

/* Starts a beep of `ticks` RTC ticks at the current volume (replaces a running beep). */
void euc_beeper_beep(struct euc_beeper *beeper, uint16_t ticks);

/* CCR4 value for a 'W' 'B' volume digit. */
uint16_t euc_beeper_volume_compare(uint8_t volume);

/* One RTC tick. Clears `out` first. */
void euc_alarm_tick(struct euc_alarm_state *state, const struct euc_alarm_in *in, struct euc_alarm_out *out);

#endif
