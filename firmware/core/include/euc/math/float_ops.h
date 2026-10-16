/*
 * Floating-point steps with the rounding of the stock firmware.
 *
 * The stock image does its float and double math in the Keil soft-float library: IEEE 754, round to nearest
 * even, but a result that would be denormal is flushed to zero (quirk KEIL_FTZ). The balance and attitude
 * ports must match the stock firmware bit for bit, so they do every float operation through these helpers, in
 * the same order and the same precision (float or double) as the stock code. With KEIL_FTZ off (clean
 * profile) the helpers are plain IEEE operations.
 *
 * Double-precision steps use plain C double arithmetic (-ffp-contract=off keeps every operation separate) and
 * come back to float through euc_f32_from_f64(), which applies the same flush.
 */
#ifndef EUC_MATH_FLOAT_OPS_H
#define EUC_MATH_FLOAT_OPS_H

#include <stdbool.h>
#include <stdint.h>

#include "euc/math/libm.h"
#include "euc/quirks.h"

_Static_assert(sizeof(float) == 4U, "IEEE 754 single precision expected");
_Static_assert(sizeof(double) == 8U, "IEEE 754 double precision expected");

#define EUC_F32_SIGN_MASK     0x80000000U
#define EUC_F32_EXPONENT_MASK 0x7F800000U
#define EUC_F32_MANTISSA_MASK 0x007FFFFFU

/* Bit pattern of a float, and back. A union is the type-pun C11 defines (MISRA deviation, see the record). */
static inline uint32_t euc_f32_bits(float value)
{
    union {
        float as_float;
        uint32_t as_bits;
    } pun = {.as_float = value};
    return pun.as_bits;
}

static inline float euc_f32_from_bits(uint32_t bits)
{
    union {
        uint32_t as_bits;
        float as_float;
    } pun = {.as_bits = bits};
    return pun.as_float;
}

/* KEIL_FTZ: a denormal result becomes a zero of the same sign. */
static inline float euc_f32_flush(float value)
{
    const uint32_t bits = euc_f32_bits(value);
    if (EUC_QUIRK(KEIL_FTZ) && ((bits & EUC_F32_EXPONENT_MASK) == 0U) &&
        ((bits & EUC_F32_MANTISSA_MASK) != 0U)) {
        return euc_f32_from_bits(bits & EUC_F32_SIGN_MASK);
    }
    return value;
}

/* True for every value except NaN and +-infinity (exponent bits not all ones). */
static inline bool euc_f32_is_finite(float value)
{
    return (euc_f32_bits(value) & EUC_F32_EXPONENT_MASK) != EUC_F32_EXPONENT_MASK;
}

/* |x| by clearing the sign bit (exact, no rounding). */
static inline float euc_f32_abs(float value)
{
    return euc_f32_from_bits(euc_f32_bits(value) & ~EUC_F32_SIGN_MASK);
}

static inline float euc_f32_add(float a, float b)
{
    return euc_f32_flush(a + b);
}

static inline float euc_f32_sub(float a, float b)
{
    return euc_f32_flush(a - b);
}

static inline float euc_f32_mul(float a, float b)
{
    return euc_f32_flush(a * b);
}

static inline float euc_f32_div(float a, float b)
{
    return euc_f32_flush(a / b);
}

/* Double to float, rounded to nearest, then flushed like any other float result. */
static inline float euc_f32_from_f64(double value)
{
    return euc_f32_flush((float)value);
}

/*
 * 1 / sqrt(x) for the vector normalisations. FAST_INV_SQRT: the stock 0x5F3759DF estimate with one Newton
 * step (about 0.2 % error), each step rounded like the stock float operations. Clean: correctly rounded.
 */
static inline float euc_f32_inv_sqrt(float x)
{
    if (EUC_QUIRK(FAST_INV_SQRT)) {
        const float estimate = euc_f32_from_bits(0x5F3759DFU - (euc_f32_bits(x) >> 1U));
        const float half_x = euc_f32_mul(x, 0.5F);
        return euc_f32_mul(euc_f32_sub(1.5F, euc_f32_mul(euc_f32_mul(half_x, estimate), estimate)), estimate);
    }
    return euc_f32_div(1.0F, euc_f32_from_f64(euc_sqrt((double)x)));
}

#endif
