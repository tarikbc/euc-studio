/*
 * Motor drive: sensored field-oriented control, one FOC step per PWM period (TIM8 underflow sample).
 *
 * Contexts and rates (the app schedules them):
 *   euc_motor_hall_step()   every TIM8 update event, 2 x fPWM
 *   euc_motor_foc_step()    every PWM period, on the injected ADC sample: 10 kHz (mode 0) or 12 kHz (mode 1)
 *   euc_motor_set_command() and euc_motor_rtc_step()   every balance tick (2.016 ms)
 *
 * One FOC step:
 *   phase currents -> Clarke -> Park(hall angle) -> Iq, Id
 *   d axis: Id x k(speed) -> PI(D) to Id* = 0 -> Vd                     (k = 0.3 at standstill .. 1.0)
 *   q axis, drive mode 1 (current): Iq* = 20 x command -> PI(D) -> Vq   (command +-1050 = +-280 A)
 *          drive mode 0 (voltage):  command back-off above +-280 A, slew +-60 per step -> Vq = 30 x command
 *   stop brake: disabled and |speed| <= 0.4 m/s -> Vq 0, integrals cleared, all low sides on
 *   (Vq, Vd) -> circle limitation (99 %) -> inverse Park -> SVPWM -> duty per phase
 *   output gate: a hall fault or a hard cut turns the bridge off for good; otherwise the bridge turns on
 * once, when the wheel was first seen at standstill, and asks the balance to restart its start-up wait.
 *
 * Telemetry and balance taps: Iq low-passed in mA (balance current feedback), Iq in 0.1 A, the active Vq
 * (balance overload checks, app PWM %).
 *
 * Fail-closed: the command is checked where it enters (finite, within +-1050). In the clean profile a command
 * that is not renewed for 3 ticks (about 6 ms: the balance tick died) latches command_stale, and the FOC
 * ramps the command to 0 at 4200 per second (full command in 0.25 s) and keeps it there until power-off; the
 * watchdog stays the backstop (quirk STALE_COMMAND_HELD). The phase over-current trip exists but is off in
 * both profiles until a bench level is set (quirk NO_OVERCURRENT_TRIP).
 */
#ifndef EUC_MOTOR_MOTOR_H
#define EUC_MOTOR_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "euc/motor/current.h"
#include "euc/motor/foc_math.h"
#include "euc/motor/hall.h"
#include "euc/motor/pid.h"

#define EUC_MOTOR_COMMAND_LIMIT 1050.0F /* duty units, from the balance */

#define EUC_MOTOR_VOLTAGE_MODE 0U /* drive mode 0: no q-current loop, 10 kHz */
#define EUC_MOTOR_CURRENT_MODE 1U /* drive mode 1: q-current PI loop, 12 kHz */

/* Error byte bits the motor layer raises (telemetry and alarms). */
#define EUC_MOTOR_ERROR_CURRENT_SENSOR 0x02U
#define EUC_MOTOR_ERROR_HALL           0x40U

/* PWM timing of one drive mode (TIM8 centre-aligned, 72 MHz). */
struct euc_motor_timing {
    uint16_t pwm_frequency_hz;
    uint16_t pwm_period_counts; /* ARR: fPWM = 72 MHz / (2 ARR) */
    uint16_t svpwm_t;           /* 4 ARR */
    uint16_t svpwm_t_sqrt3;     /* floor(4 ARR sqrt(3)) */
    uint8_t dead_time_counts;   /* BDTR DTG, 13.9 ns per count */
};

struct euc_motor_config {
    struct euc_current_config current; /* clean over-current trip, from the board */
};

struct euc_motor_state {
    uint8_t drive_mode; /* fixed at power-on */
    struct euc_motor_timing timing;
    struct euc_motor_config config;
    struct euc_current_state current;
    struct euc_hall_state hall;
    struct euc_pid pid_d;
    struct euc_pid pid_q;

    float command;          /* from the balance; voltage mode writes its limited value back */
    float command_previous; /* voltage-mode slew */

    struct euc_foc_trig trig;
    struct euc_foc_alpha_beta i_alpha_beta;
    struct euc_foc_qd i_qd; /* d after the speed gain */
    struct euc_foc_qd v_qd; /* after circle limitation */
    struct euc_foc_alpha_beta v_alpha_beta;
    struct euc_foc_duty duty;

    int32_t iq_lpf_ma;   /* balance input motor_current_lpf_ma; IIR 1e-4 per step, clamp +-90 A */
    int32_t iq_01a;      /* instant Iq, 0.1 A (telemetry) */
    int16_t iq_peak_01a; /* largest |Iq| since power-on, signed */
    int16_t id_gain;     /* k x 10: 3..10 */
    int16_t vq_active;   /* Vq, or 0 while the stop brake holds: balance input motor_voltage_q */
    bool brake_latched;
    bool hard_cut;        /* latched until reset: bridge off */
    bool outputs_enabled; /* bridge outputs on (TIM8 MOE) */
    bool enable_pending;  /* the one-time enable has not happened yet */

    /* Clean: a stale balance command (quirk STALE_COMMAND_HELD) */
    bool command_seen;            /* the balance has sent a command since power-on */
    uint16_t steps_since_command; /* FOC steps since the last command */
    uint16_t stale_limit_steps;   /* 3 ticks of FOC steps */
    float stale_ramp_per_step;    /* 4200 per second, per FOC step */
    bool command_stale;           /* latched until power-off: the command ramps to 0 */
};

struct euc_motor_foc_in {
    struct euc_current_sample sample;
    int16_t speed_dm_per_s; /* balance speed estimate, 0.1 m/s */
    bool drive_disabled;    /* balance soft disable */
    bool hard_cut_request;  /* system hard cut (lock, power-off, maintenance commands); latched here */
};

struct euc_motor_foc_out {
    struct euc_foc_duty duty; /* compare counts per phase; 0 = low side on */
    bool outputs_enabled;
    bool start_balance; /* first enable: the balance restarts its start-up wait (10 ticks) */
    bool stop_drive;    /* sensor check failed: the balance disables and zeroes its command */
    uint8_t error_bits; /* EUC_MOTOR_ERROR_* to set in the error byte */
};

/* Timing of a drive mode; any mode other than 0 is current mode. */
struct euc_motor_timing euc_motor_timing_for_mode(uint8_t drive_mode);

/* Power-on state for the drive mode chosen at boot. Current offsets:
 * euc_current_set_offsets(&state->current). */
void euc_motor_init(struct euc_motor_state *state, uint8_t drive_mode, const struct euc_motor_config *config);

/* The balance command (duty units). A non-finite value becomes 0; the value is clamped to +-1050. */
void euc_motor_set_command(struct euc_motor_state *state, float command);

/* Balance rate: the q integral may supply at most 160 x |speed| (0.1 m/s, clamped to 20 m/s) of voltage. */
void euc_motor_rtc_step(struct euc_motor_state *state, int16_t speed_dm_per_s);

/* Twice per PWM period. Bits 0..2 of lines: hall A, B, C. */
void euc_motor_hall_step(struct euc_motor_state *state, uint8_t lines, struct euc_hall_out *out);

/* Once per PWM period. Clears `out` first. */
void euc_motor_foc_step(struct euc_motor_state *state, const struct euc_motor_foc_in *in,
                        struct euc_motor_foc_out *out);

/* App "true PWM" percent: -Vq x 100 / 31500, truncated (frame 07). */
int8_t euc_motor_pwm_percent(const struct euc_motor_state *state);

#endif
