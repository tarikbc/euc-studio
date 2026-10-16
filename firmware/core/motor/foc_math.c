/*
 * FOC arithmetic (see euc/motor/foc_math.h). Written from the textbook equations and the constants of the
 * stock drive.
 */
#include <stdbool.h>
#include <stdint.h>

#include "euc/math/fixed.h"
#include "euc/motor/foc_math.h"
#include "euc/quirks.h"
#include "motor_internal.h"

#define Q15_DIVISOR 32768

#define ANGLE_COUNTS_PER_HALF_TURN 32768
#define ANGLE_COUNTS_PER_QUADRANT  16384
#define ANGLE_COUNTS_PER_ENTRY     64 /* 16384 / 256 table entries */
#define SINE_LAST_INDEX            255

#define CLARKE_INV_SQRT3_Q15 18918 /* 2^15 / sqrt(3) */

#define CIRCLE_MAX_MODULE_SQUARED ((uint32_t)EUC_FOC_CIRCLE_MAX_MODULE * (uint32_t)EUC_FOC_CIRCLE_MAX_MODULE)
#define CIRCLE_BIN_SHIFT          24U

/* SVPWM: the switching times come out scaled by 2^17 (Q1.15 voltage times 4 ARR). */
#define SVPWM_TIME_DIVISOR      131072
#define SVPWM_HALF_TIME_DIVISOR 262144

int16_t euc_foc_narrow(int32_t value)
{
    return EUC_QUIRK(INT16_WRAP) ? motor_wrap_i16(value) : sat_i32_to_i16(value);
}

struct euc_foc_trig euc_foc_trig_from_angle(int16_t angle)
{
    /* Shifted by half a turn, the angle splits into quadrants 0..3 = [-180, -90), [-90, 0), [0, 90),
     * [90, 180) deg. */
    const int32_t shifted = (int32_t)angle + ANGLE_COUNTS_PER_HALF_TURN;
    const int32_t quadrant = shifted / ANGLE_COUNTS_PER_QUADRANT;
    const int32_t index = (shifted % ANGLE_COUNTS_PER_QUADRANT) / ANGLE_COUNTS_PER_ENTRY;
    const int16_t rising = euc_foc_sine_table[index];                    /* sine inside the quadrant */
    const int16_t falling = euc_foc_sine_table[SINE_LAST_INDEX - index]; /* cosine inside the quadrant */

    struct euc_foc_trig trig;
    switch (quadrant) {
    case 0:
        trig.cos_q15 = (int16_t)-falling;
        trig.sin_q15 = (int16_t)-rising;
        break;
    case 1:
        trig.cos_q15 = rising;
        trig.sin_q15 = (int16_t)-falling;
        break;
    case 2:
        trig.cos_q15 = falling;
        trig.sin_q15 = rising;
        break;
    default:
        trig.cos_q15 = (int16_t)-rising;
        trig.sin_q15 = falling;
        break;
    }
    return trig;
}

static int32_t q15_product(int16_t a, int16_t b)
{
    return ((int32_t)a * (int32_t)b) / Q15_DIVISOR;
}

struct euc_foc_alpha_beta euc_foc_clarke(int16_t ia_q15, int16_t ib_q15)
{
    const int32_t a_term = q15_product(ia_q15, CLARKE_INV_SQRT3_Q15);
    const int32_t b_term = q15_product(ib_q15, CLARKE_INV_SQRT3_Q15);
    struct euc_foc_alpha_beta current;
    current.alpha_q15 = ia_q15;
    current.beta_q15 = euc_foc_narrow(-(a_term + (2 * b_term)));
    return current;
}

struct euc_foc_qd euc_foc_park(struct euc_foc_alpha_beta current, struct euc_foc_trig trig)
{
    struct euc_foc_qd result;
    result.q_q15 = euc_foc_narrow(q15_product(current.alpha_q15, trig.cos_q15) -
                                  q15_product(current.beta_q15, trig.sin_q15));
    result.d_q15 = euc_foc_narrow(q15_product(current.alpha_q15, trig.sin_q15) +
                                  q15_product(current.beta_q15, trig.cos_q15));
    return result;
}

struct euc_foc_alpha_beta euc_foc_inverse_park(struct euc_foc_qd voltage, struct euc_foc_trig trig)
{
    struct euc_foc_alpha_beta result;
    result.alpha_q15 =
        euc_foc_narrow(q15_product(voltage.q_q15, trig.cos_q15) + q15_product(voltage.d_q15, trig.sin_q15));
    result.beta_q15 =
        euc_foc_narrow(q15_product(voltage.d_q15, trig.cos_q15) - q15_product(voltage.q_q15, trig.sin_q15));
    return result;
}

struct euc_foc_qd euc_foc_circle_limit(struct euc_foc_qd voltage)
{
    const int32_t q_squared = (int32_t)voltage.q_q15 * (int32_t)voltage.q_q15; /* at most 2^30 each */
    const int32_t d_squared = (int32_t)voltage.d_q15 * (int32_t)voltage.d_q15;
    const uint32_t length_squared = (uint32_t)q_squared + (uint32_t)d_squared;
    if (length_squared <= CIRCLE_MAX_MODULE_SQUARED) {
        return voltage;
    }

    /* Bin 62 is the first above 32439^2. Only Vq = Vd = -32768 reaches past the last entry, and the drive
     * never asks for that; the clamp keeps the read inside the table. */
    uint32_t index = (length_squared >> CIRCLE_BIN_SHIFT) - EUC_FOC_CIRCLE_FIRST_BIN;
    if (index >= EUC_FOC_CIRCLE_ENTRIES) {
        index = EUC_FOC_CIRCLE_ENTRIES - 1U;
    }
    const int32_t scale = (int32_t)euc_foc_circle_table[index];

    /* scale < 2^15, so both results are shorter than the inputs and fit in 16 bits. */
    struct euc_foc_qd limited;
    limited.q_q15 = (int16_t)((scale * voltage.q_q15) / Q15_DIVISOR);
    limited.d_q15 = (int16_t)((scale * voltage.d_q15) / Q15_DIVISOR);
    return limited;
}

static uint16_t clamp_duty(int32_t duty, int32_t period)
{
    return (uint16_t)clamp_i32(duty, 0, period);
}

struct euc_foc_duty euc_foc_svpwm(struct euc_foc_alpha_beta voltage, uint16_t svpwm_t, uint16_t svpwm_t_sqrt3)
{
    const int32_t t = (int32_t)svpwm_t;
    const int32_t u_alpha = (int32_t)svpwm_t_sqrt3 * voltage.alpha_q15;
    const int32_t x = -(t * voltage.beta_q15);
    const int32_t y = (x + u_alpha) / 2;
    const int32_t z = (x - u_alpha) / 2;
    const int32_t half_period = t / 8; /* ARR / 2: every phase at 50 % */

    /* Sectors in pairs that share one set of switching times: (1, 4) uses X and Z, (2, 5) Y and Z, (3, 6) X
     * and Y. */
    int32_t a;
    int32_t b;
    int32_t c;
    bool use_y_z;
    bool use_x_z;
    if (y < 0) {
        use_y_z = (z < 0);
        use_x_z = (!use_y_z) && (x <= 0);
    } else {
        use_y_z = (z >= 0);
        use_x_z = (!use_y_z) && (x > 0);
    }
    if (use_y_z) {
        a = half_period + ((t + y - z) / SVPWM_HALF_TIME_DIVISOR);
        b = a + (z / SVPWM_TIME_DIVISOR);
        c = a - (y / SVPWM_TIME_DIVISOR);
    } else if (use_x_z) {
        a = half_period + ((t + x - z) / SVPWM_HALF_TIME_DIVISOR);
        b = a + (z / SVPWM_TIME_DIVISOR);
        c = b - (x / SVPWM_TIME_DIVISOR);
    } else {
        a = half_period + ((t - x + y) / SVPWM_HALF_TIME_DIVISOR);
        c = a - (y / SVPWM_TIME_DIVISOR);
        b = c + (x / SVPWM_TIME_DIVISOR);
    }

    const int32_t period = t / 4;
    struct euc_foc_duty duty;
    duty.a = clamp_duty(a, period);
    duty.b = clamp_duty(b, period);
    duty.c = clamp_duty(c, period);
    return duty;
}
