/*
 * Phase-current sensing: offsets at power-on, then one reading per FOC step.
 *
 * Two isolated sensors on phases A and B, sampled together (ADC1 and ADC2, injected) at the centre of the PWM
 * period. Inputs are 12-bit ADC values; raw x 16 is the Q1.15 scale, 32768 = 437.5 A (the stock scale; the
 * true sensor gain is a board fact). The offset is the sum of 16 samples taken with the bridge off.
 *
 *   a  = 16 raw_a - offset_a     Ia = -a     (the A sensor reads the opposite sign)
 *   b  = 16 raw_b - offset_b     Ib = b
 *   c  = a - b
 *
 * Checks on a, b and c, every reading:
 *   - over-current: consecutive readings with any |value| > limit. Stock only counts them (no trip, and the
 *     timer break input is unused). Clean trips: after config.overcurrent_trip_samples readings above
 *     config.overcurrent_trip_q15 it latches overcurrent_trip, which cuts the bridge (quirk
 *     NO_OVERCURRENT_TRIP).
 *   - sensor check at start: during readings 1..999 after power-on, any |value| > 6000 (80 A) latches
 *     sensor_fault; the motor layer cuts the bridge and stops the drive (error bit 0x02).
 */
#ifndef EUC_MOTOR_CURRENT_H
#define EUC_MOTOR_CURRENT_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_CURRENT_OFFSET_SAMPLES 16U

/* Board settings for the clean over-current trip. */
struct euc_current_config {
    int16_t overcurrent_trip_q15;      /* trip level, Q1.15 (32768 = 437.5 A) */
    uint16_t overcurrent_trip_samples; /* consecutive readings above the level, at least 1 */
};

struct euc_current_offsets_in {
    uint32_t phase_a_sum;      /* 16 samples of phase A (ADC1) */
    uint32_t phase_b_sum_adc1; /* 16 samples of phase B taken by ADC1 (stock source) */
    uint32_t
        phase_b_sum_adc2; /* 16 samples of phase B taken by ADC2, the converter that reads it at run time */
    uint16_t boot_count;  /* power-on count (BKP DR2) */
    uint16_t stored_offset_a; /* BKP DR12 */
    uint16_t stored_offset_b; /* BKP DR13 */
};

struct euc_current_sample {
    uint16_t phase_a_raw; /* ADC1, 12 bits */
    uint16_t phase_b_raw; /* ADC2, 12 bits */
};

struct euc_current_state {
    int32_t offset_a; /* sum of 16 samples = raw x 16 */
    int32_t offset_b;
    int16_t a; /* last reading, before the sign change of phase A */
    int16_t b;
    int16_t c;
    int16_t peak;                 /* reading with the largest magnitude since power-on (signed) */
    uint16_t over_limit_readings; /* consecutive readings above the over-current level */
    uint16_t readings_since_boot; /* stops at 1000 */
    bool sensor_fault;            /* latched */
    bool overcurrent_trip;        /* clean: latched */
};

struct euc_current_out {
    int16_t ia_q15; /* phase currents for the Clarke transform */
    int16_t ib_q15;
    bool start_check_failed; /* this reading failed the sensor check at start */
};

/* Power-on state; offsets 0 until euc_current_set_offsets(). */
void euc_current_init(struct euc_current_state *state);

/*
 * Offsets from the samples taken with the bridge off. Stock takes phase B's offset from ADC1 although ADC2
 * reads phase B at run time (quirk IB_OFFSET_FROM_ADC1), and from the tenth power-on uses offsets stored in
 * BKP DR12/DR13 when they are set (quirk BKP_CURRENT_OFFSETS; nothing in the stock image writes them).
 */
void euc_current_set_offsets(struct euc_current_state *state, const struct euc_current_offsets_in *in);

/* One reading per FOC step. */
void euc_current_read(struct euc_current_state *state, const struct euc_current_config *config,
                      struct euc_current_sample sample, struct euc_current_out *out);

#endif
