/*
 * Riderless detector (see euc/safety/riderless.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/float_ops.h"
#include "euc/quirks.h"
#include "euc/safety/riderless.h"

#define BALANCING_TICKS        1488U    /* 3 s */
#define LOAD_DROP_TICKS        744U     /* 1.5 s: a load drop allows a suspicion this long */
#define QUIET_TICKS            298U     /* 0.6 s */
#define SLOW_FILTER_GAIN       0.00672F /* 2.016 ms / 0.3 s */
#define FAST_FILTER_GAIN       0.04032F /* 2.016 ms / 0.05 s */
#define SPEED_FILTER_GAIN      0.01008F /* 2.016 ms / 0.2 s */
#define MIN_SPEED_M_PER_S      1.0F
#define MAX_SPEED_DROP_M_PER_S 0.15F
#define LOAD_DROP_A            4.0F /* the slow current this far above the fast one */
#define LOAD_DROP_FAST_A       2.5F /* while the fast one is this close to zero */
#define QUIET_CURRENT_A        3.5F
#define RIDER_CURRENT_A        7.0F
#define DECIAMPS_PER_AMP       10.0F

void euc_riderless_init(struct euc_riderless_state *state)
{
    (void)memset(state, 0, sizeof(*state));
}

static float low_pass(float filtered, float value, float gain)
{
    return euc_f32_add(filtered, euc_f32_mul(euc_f32_sub(value, filtered), gain));
}

/* A sudden drop of the load to about zero: the fast current is near zero while the slow one still remembers
 * the rider. A rider who slows down passes zero gradually (coasting) or goes on to a braking current. */
static bool load_dropped(const struct euc_riderless_state *state)
{
    const float slow = euc_f32_abs(state->current_slow_a);
    const float fast = euc_f32_abs(state->current_fast_a);
    return (euc_f32_sub(slow, fast) > LOAD_DROP_A) && (fast < LOAD_DROP_FAST_A);
}

/* Counts the ticks the suspect conditions hold; a speed drop during the count starts it again. */
static uint16_t count_quiet(struct euc_riderless_state *state, bool quiet, float speed)
{
    if (!quiet) {
        return 0U;
    }
    if (state->quiet_ticks == 0U) {
        state->speed_quiet_start_m_per_s = speed;
    }
    if (euc_f32_sub(state->speed_quiet_start_m_per_s, speed) > MAX_SPEED_DROP_M_PER_S) {
        return 0U; /* slowing down: a rider who stopped pushing */
    }
    return (uint16_t)(state->quiet_ticks + 1U);
}

bool euc_riderless_tick(struct euc_riderless_state *state, const struct euc_riderless_in *in)
{
    if (EUC_QUIRK(RIDERLESS_UNDETECTED)) {
        return false;
    }

    const float current = euc_f32_div((float)in->current_01a, DECIAMPS_PER_AMP);
    state->current_slow_a = low_pass(state->current_slow_a, current, SLOW_FILTER_GAIN);
    state->current_fast_a = low_pass(state->current_fast_a, current, FAST_FILTER_GAIN);
    state->speed_filtered_m_per_s =
        low_pass(state->speed_filtered_m_per_s, in->speed_m_per_s, SPEED_FILTER_GAIN);
    const float speed = euc_f32_abs(state->speed_filtered_m_per_s);
    const bool moving = speed > MIN_SPEED_M_PER_S;

    if (!in->balancing) {
        state->balancing_ticks = 0U;
        state->load_drop_ticks = 0U;
        state->quiet_ticks = 0U;
        state->suspected = false;
        return false;
    }
    if (state->balancing_ticks < BALANCING_TICKS) {
        state->balancing_ticks++;
    }

    if (state->suspected) {
        if (!moving || (euc_f32_abs(state->current_slow_a) > RIDER_CURRENT_A)) {
            state->suspected = false; /* stopped, or a rider leans against the tiltback */
            state->quiet_ticks = 0U;
        }
        return state->suspected;
    }

    if (moving && load_dropped(state)) {
        state->load_drop_ticks = LOAD_DROP_TICKS;
    } else if (state->load_drop_ticks > 0U) {
        state->load_drop_ticks--;
    } else {
        /* no recent load drop */
    }
    const bool quiet = moving && (state->load_drop_ticks > 0U) &&
                       (state->balancing_ticks >= BALANCING_TICKS) && !in->other_limit &&
                       (euc_f32_abs(state->current_slow_a) < QUIET_CURRENT_A);
    state->quiet_ticks = count_quiet(state, quiet, speed);
    if (state->quiet_ticks >= QUIET_TICKS) {
        state->suspected = true;
        state->quiet_ticks = 0U;
        state->load_drop_ticks = 0U;
        if (state->triggers < UINT8_MAX) {
            state->triggers++;
        }
    }
    return state->suspected;
}
