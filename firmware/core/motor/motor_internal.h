/*
 * Helpers shared by the motor modules. Not part of the public interface.
 */
#ifndef EUC_MOTOR_INTERNAL_H
#define EUC_MOTOR_INTERNAL_H

#include <stdint.h>

/*
 * Two's-complement wrap to 16 bits, for values that are meant to wrap (electrical angles, step counters) and
 * for the stock narrowing behind quirk INT16_WRAP. Written with defined arithmetic, so no conversion of an
 * out-of-range value is implementation-defined. |value| must be below 2^30.
 */
static inline int16_t motor_wrap_i16(int32_t value)
{
    int32_t wrapped = (value + 32768) % 65536;
    if (wrapped < 0) {
        wrapped += 65536;
    }
    return (int16_t)(wrapped - 32768);
}

#endif
