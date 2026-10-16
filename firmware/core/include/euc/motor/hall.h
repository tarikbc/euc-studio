/*
 * Hall sensors: rotor angle, speed, step count and odometer, one call per TIM8 update event (twice per PWM
 * period: 20 kHz in voltage mode, 24 kHz in current mode).
 *
 * Input: the three hall line levels, bit 0 = hall A, bit 1 = hall B, bit 2 = hall C (the board maps lines to
 * pins). A glitch filter ignores a line that toggles again within 200 calls unless another line toggled in
 * between. The filtered state (1..6; 0 and 7 are invalid) gives the sector:
 *
 *   state        1      3     2     6     4     5
 *   centre deg   -150   -90   -30   +30   +90   +150      forward (+1) order: 5 1 3 2 6 4 5
 *
 * Angle: for 200 calls after an edge the angle moves from the sector entry (centre -+30 deg) at the speed of
 * the last electrical turn, at most +-60 deg; later it rests at the sector centre.
 * Speed from the edge period: -202117 / (calls per edge), slew limited to +-500 per call, then filtered over
 * about 1000 calls. Forward gives a negative value. The constant is mm/s at 24 kHz calls; at 20 kHz the stock
 * value reads 1.2 x high, and the clean profile scales it to the call rate (quirk HALL_SPEED_1_2X). The stock
 * filter truncates and stops up to 999 counts short of its input; clean keeps fraction bits
 * (quirk HALL_SPEED_FILTER_STALL).
 * Step count: signed steps per 20 ms window (fPWM / 25 + 2 calls) and the change of an 11-window sum.
 * Faults: an invalid state for more than fPWM / 8 calls (62.5 ms) latches a hall fault.
 */
#ifndef EUC_MOTOR_HALL_H
#define EUC_MOTOR_HALL_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_HALL_LINES          3U
#define EUC_HALL_RING_EDGES     6U   /* edges in one electrical turn */
#define EUC_HALL_ODOMETER_STEPS 118U /* hall steps per odometer unit (about 1 m) */

struct euc_hall_state {
    /* Glitch filter, per line */
    uint8_t line_level[EUC_HALL_LINES];
    bool line_locked[EUC_HALL_LINES]; /* this line toggled last */
    int16_t line_lock_calls[EUC_HALL_LINES];

    /* Angle and period speed */
    uint8_t sector;                           /* filtered state of the previous call */
    int16_t angle;                            /* 0x10000 = 360 deg electrical */
    uint16_t calls_since_edge;                /* stops at 1000 */
    uint16_t edge_calls[EUC_HALL_RING_EDGES]; /* calls between the last six edges */
    uint8_t edge_index;
    int16_t angle_step;     /* interpolation step per call */
    int16_t angle_progress; /* interpolated angle since the edge */
    int16_t entry_offset;   /* -+30 deg: from the centre back to the sector edge */
    bool interpolating;
    bool standstill_seen; /* one-shot: no edge for more than 300 calls since power-on */
    uint16_t invalid_calls;
    bool fault; /* latched until reset */
    int32_t period_speed_raw;
    int32_t period_speed_slewed;
    int32_t period_speed;      /* filtered: balance input hall_period_speed */
    int32_t period_speed_fine; /* clean filter state, 1/1024 count */

    /* Step count */
    uint8_t step_sector; /* filtered state of the previous call, for the step count */
    int8_t direction;    /* +1 forward, -1 backward, 0 before the first step */
    int16_t position;    /* signed steps since power-on */
    int16_t window_steps;
    int16_t window_calls;
    uint8_t windows;
    int16_t steps_per_window; /* last complete 20 ms window: balance input hall_steps_per_window */
    int16_t block_steps;      /* steps in the current block of 11 windows */
    int16_t last_block_steps; /* steps in the previous block */
    int16_t steps_11_windows; /* block - previous block: balance input hall_steps_11_windows */
    int16_t steps_11_windows_max;
    uint8_t odometer_steps;
};

struct euc_hall_out {
    bool odometer_step; /* 118 sector changes since the last unit: add one to odometer and trip */
    bool fault_raised;  /* the hall fault latched in this call (alarm task: error bit 0x40) */
};

/* Power-on state. */
void euc_hall_init(struct euc_hall_state *state);

/* One call per TIM8 update event. pwm_frequency_hz is the PWM frequency (calls come at twice that rate). */
void euc_hall_step(struct euc_hall_state *state, uint8_t lines, uint16_t pwm_frequency_hz,
                   struct euc_hall_out *out);

#endif
