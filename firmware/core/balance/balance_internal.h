/*
 * Helpers shared by the balance task's source files. Not part of the public API.
 */
#ifndef EUC_BALANCE_INTERNAL_H
#define EUC_BALANCE_INTERNAL_H

#include "euc/balance/balance.h"
#include "euc/math/float_ops.h"

/* Restart and handle offsets fade by 0.025 deg per tick (12.4 deg/s) and snap to zero below that. */
#define BALANCE_OFFSET_FADE_DEG 0.025F

static inline float balance_fade_offset(float offset_deg)
{
    if (offset_deg >= BALANCE_OFFSET_FADE_DEG) {
        return euc_f32_from_f64((double)offset_deg - 0.025);
    }
    if (offset_deg < -BALANCE_OFFSET_FADE_DEG) {
        return euc_f32_from_f64((double)offset_deg + 0.025);
    }
    return 0.0F;
}

static inline void balance_request_beep(struct euc_balance_out *out, uint8_t ticks)
{
    if (out->beep_count < EUC_BALANCE_MAX_BEEPS) {
        out->beep_ticks[out->beep_count] = ticks;
        out->beep_count++;
    }
}

#endif
