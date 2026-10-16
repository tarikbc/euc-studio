/*
 * FOC arithmetic for the current loop: table trigonometry, Clarke, Park, inverse Park, circle limitation and
 * space-vector PWM. Integer only.
 *
 * Every product is narrowed by C division by a power of two (truncation toward zero), the rounding of the
 * stock drive, so the two can be compared bit for bit.
 *
 * Formats
 *   angle     int16_t, 0x10000 = 360 deg electrical; wraps naturally
 *   current   Q1.15 counts, 32768 = 437.5 A (the stock scale)
 *   voltage   Q1.15 counts, 32767 = Vbus / sqrt(3), the peak phase voltage of linear SVPWM
 *   duty      TIM8 compare counts, 0..ARR, one per motor phase (the board maps phases to channels)
 *
 * 16-bit results: balanced phase currents and circle-limited voltages always fit. Out of range (a saturated
 * or failed sensor) the stock code wraps around; the clean profile saturates (quirk INT16_WRAP).
 *
 * The two tables are computed from their formulas at build time (tools/gen_motor_tables.py).
 */
#ifndef EUC_MOTOR_FOC_MATH_H
#define EUC_MOTOR_FOC_MATH_H

#include <stdint.h>

#define EUC_FOC_SINE_ENTRIES      256U
#define EUC_FOC_CIRCLE_ENTRIES    66U
#define EUC_FOC_CIRCLE_MAX_MODULE 32439 /* 99 % of full scale */
#define EUC_FOC_CIRCLE_FIRST_BIN  62U   /* entry k serves Vq^2 + Vd^2 in [62 + k, 63 + k) x 2^24 */

/* round(32767 sin(i x 90 deg / 256)), i = 0..255 */
extern const int16_t euc_foc_sine_table[EUC_FOC_SINE_ENTRIES];
/* Circle-limitation scale factors, Q1.15 */
extern const uint16_t euc_foc_circle_table[EUC_FOC_CIRCLE_ENTRIES];

struct euc_foc_trig {
    int16_t cos_q15;
    int16_t sin_q15;
};

struct euc_foc_alpha_beta {
    int16_t alpha_q15;
    int16_t beta_q15;
};

struct euc_foc_qd {
    int16_t q_q15;
    int16_t d_q15;
};

struct euc_foc_duty {
    uint16_t a;
    uint16_t b;
    uint16_t c;
};

/* Narrows to 16 bits: wraps around (INT16_WRAP) or saturates. |value| must be below 2^30. */
int16_t euc_foc_narrow(int32_t value);

/* cos and sin of the angle from the quarter-wave table; resolution 64 angle counts (0.35 deg). */
struct euc_foc_trig euc_foc_trig_from_angle(int16_t angle);

/* alpha = Ia; beta = -(Ia + 2 Ib) / sqrt(3), the sign convention of this drive. */
struct euc_foc_alpha_beta euc_foc_clarke(int16_t ia_q15, int16_t ib_q15);

/* q = alpha cos - beta sin; d = alpha sin + beta cos. */
struct euc_foc_qd euc_foc_park(struct euc_foc_alpha_beta current, struct euc_foc_trig trig);

/* alpha = q cos + d sin; beta = d cos - q sin. */
struct euc_foc_alpha_beta euc_foc_inverse_park(struct euc_foc_qd voltage, struct euc_foc_trig trig);

/* Scales (Vq, Vd) back inside the 99 % circle when Vq^2 + Vd^2 > 32439^2; shorter vectors pass unchanged. */
struct euc_foc_qd euc_foc_circle_limit(struct euc_foc_qd voltage);

/*
 * Space-vector PWM with centre-aligned timing: duty per phase for the voltage vector, 50 % (ARR / 2) for a
 * zero vector. svpwm_t = 4 ARR, svpwm_t_sqrt3 = floor(4 ARR sqrt(3)). The result is clamped to 0..ARR, which
 * only matters for a vector longer than full scale; the circle limitation keeps them shorter.
 */
struct euc_foc_duty euc_foc_svpwm(struct euc_foc_alpha_beta voltage, uint16_t svpwm_t,
                                  uint16_t svpwm_t_sqrt3);

#endif
