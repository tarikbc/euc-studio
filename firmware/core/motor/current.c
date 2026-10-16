/*
 * Phase-current sensing (see euc/motor/current.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/fixed.h"
#include "euc/motor/current.h"
#include "euc/motor/foc_math.h"
#include "euc/quirks.h"
#include "motor_internal.h"

#define RAW_TO_Q15 16 /* 12-bit sample to the Q1.15 scale (left-aligned register x 2 in the stock code) */

#define STORED_OFFSETS_AFTER_BOOTS 9U /* stored offsets apply from the tenth power-on */
#define STORED_OFFSET_ERASED       0xFFFFU

#define STOCK_OVER_LIMIT_Q15 31500 /* about 420 A */
#define START_CHECK_READINGS 1000U
#define START_CHECK_LIMIT    6000 /* about 80 A */

void euc_current_init(struct euc_current_state *state)
{
    (void)memset(state, 0, sizeof(*state));
}

static bool stored_offset_is_set(uint16_t value)
{
    return (value != 0U) && (value != STORED_OFFSET_ERASED);
}

void euc_current_set_offsets(struct euc_current_state *state, const struct euc_current_offsets_in *in)
{
    state->offset_a = (int32_t)in->phase_a_sum;
    state->offset_b = (int32_t)(EUC_QUIRK(IB_OFFSET_FROM_ADC1) ? in->phase_b_sum_adc1 : in->phase_b_sum_adc2);
    if (EUC_QUIRK(BKP_CURRENT_OFFSETS) && (in->boot_count > STORED_OFFSETS_AFTER_BOOTS)) {
        if (stored_offset_is_set(in->stored_offset_a)) {
            state->offset_a = (int32_t)in->stored_offset_a;
        }
        if (stored_offset_is_set(in->stored_offset_b)) {
            state->offset_b = (int32_t)in->stored_offset_b;
        }
    }
}

static int32_t magnitude(int16_t value)
{
    return (value < 0) ? -(int32_t)value : (int32_t)value;
}

static bool any_above(const struct euc_current_state *state, int32_t limit)
{
    return (magnitude(state->a) > limit) || (magnitude(state->b) > limit) || (magnitude(state->c) > limit);
}

static void track_peak(struct euc_current_state *state, int16_t value)
{
    if (magnitude(value) > magnitude(state->peak)) {
        state->peak = value;
    }
}

/* Ia = -a. The stock code swaps the two saturation branches (unreachable while the offset sits near
 * mid-scale). */
static int16_t phase_a_current(int32_t a)
{
    if (EUC_QUIRK(PHASE_SAT_SWAPPED)) {
        if (a < INT16_MIN) {
            return INT16_MIN;
        }
        if (a <= INT16_MAX) {
            return motor_wrap_i16(-a);
        }
        return INT16_MAX;
    }
    return sat_i32_to_i16(-a);
}

static void check_over_current(struct euc_current_state *state, const struct euc_current_config *config)
{
    const bool stock = EUC_QUIRK(NO_OVERCURRENT_TRIP);
    const int32_t limit = stock ? STOCK_OVER_LIMIT_Q15 : (int32_t)config->overcurrent_trip_q15;
    if (any_above(state, limit)) {
        state->over_limit_readings++;
    } else {
        state->over_limit_readings = 0U;
    }
    if ((!stock) && (state->over_limit_readings > 0U) &&
        (state->over_limit_readings >= config->overcurrent_trip_samples)) {
        state->overcurrent_trip = true;
    }
}

void euc_current_read(struct euc_current_state *state, const struct euc_current_config *config,
                      struct euc_current_sample sample, struct euc_current_out *out)
{
    const int32_t a = (RAW_TO_Q15 * (int32_t)sample.phase_a_raw) - state->offset_a;
    const int32_t b = (RAW_TO_Q15 * (int32_t)sample.phase_b_raw) - state->offset_b;

    out->ia_q15 = phase_a_current(a);
    out->ib_q15 = sat_i32_to_i16(b);
    out->start_check_failed = false;

    state->a = euc_foc_narrow(a);
    state->b = euc_foc_narrow(b);
    state->c =
        EUC_QUIRK(INT16_WRAP) ? motor_wrap_i16((int32_t)state->a - (int32_t)state->b) : sat_i32_to_i16(a - b);
    track_peak(state, state->a);
    track_peak(state, state->b);
    track_peak(state, state->c);

    check_over_current(state, config);

    if (state->readings_since_boot < START_CHECK_READINGS) {
        if ((state->readings_since_boot != 0U) && any_above(state, START_CHECK_LIMIT)) {
            state->sensor_fault = true;
            out->start_check_failed = true;
        }
        state->readings_since_boot++;
    }
}
