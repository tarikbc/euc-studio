/*
 * Hall sensors (see euc/motor/hall.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/motor/hall.h"
#include "euc/quirks.h"
#include "motor_internal.h"

#define LINE_LOCK_CALLS 200 /* about 8 ms at 24 kHz */

#define STATE_INVALID_LOW  0U
#define STATE_INVALID_HIGH 7U

#define EDGE_CALLS_AT_POWER_ON 1000U
#define EDGE_TIMEOUT_CALLS     999U /* no edge for longer: period speed 0, the edge counter stops */
#define STANDSTILL_CALLS       300U
#define INTERPOLATION_CALLS    200U

#define ANGLE_30_DEG         0x1555
#define ANGLE_PROGRESS_LIMIT 0x2AA9 /* about 60 deg */
#define ANGLE_STEP_THRESHOLD 3      /* smaller interpolation steps keep the previous entry offset */
#define ANGLE_ONE_TURN       65536

/* -202117 / (calls per edge) = mm/s for calls at 24 kHz: 421 mm/s per step and 20 ms window, 118.7 steps per
 * metre. */
#define PERIOD_SPEED_CONSTANT   202117
#define PERIOD_SPEED_CALL_RATE  24000
#define PERIOD_SPEED_SLEW       500
#define PERIOD_SPEED_FILTER_OLD 999
#define PERIOD_SPEED_FILTER_DIV 1000
#define PERIOD_SPEED_FRACTION   1024 /* clean filter: 10 fraction bits */

#define WINDOWS_PER_BLOCK 10U /* the block closes after one more: 11 windows */

static bool state_is_valid(uint8_t hall_state)
{
    return (hall_state != STATE_INVALID_LOW) && (hall_state != STATE_INVALID_HIGH);
}

void euc_hall_init(struct euc_hall_state *state)
{
    (void)memset(state, 0, sizeof(*state));
    for (uint32_t i = 0U; i < EUC_HALL_RING_EDGES; i++) {
        state->edge_calls[i] = (uint16_t)EDGE_CALLS_AT_POWER_ON;
    }
}

/* Glitch filter: a line that toggled stays locked for LINE_LOCK_CALLS calls, unless another line toggles. */
static uint8_t filter_lines(struct euc_hall_state *state, uint8_t lines)
{
    for (uint32_t line = 0U; line < EUC_HALL_LINES; line++) {
        const uint8_t level = (uint8_t)((lines >> line) & 1U);
        if ((!state->line_locked[line]) || (state->line_lock_calls[line] == 0)) {
            const bool toggled = (level != state->line_level[line]);
            state->line_level[line] = level;
            if (toggled) {
                state->line_lock_calls[line] = LINE_LOCK_CALLS;
                for (uint32_t other = 0U; other < EUC_HALL_LINES; other++) {
                    state->line_locked[other] = (other == line);
                }
            }
        }
    }
    for (uint32_t line = 0U; line < EUC_HALL_LINES; line++) {
        if (state->line_lock_calls[line] > 0) {
            state->line_lock_calls[line]--;
        }
    }
    return (uint8_t)(state->line_level[0] | (uint8_t)(state->line_level[1] << 1U) |
                     (uint8_t)(state->line_level[2] << 2U));
}

/* Direction, step count, 20 ms windows and odometer. */
static void count_steps(struct euc_hall_state *state, uint8_t hall_state, uint16_t pwm_frequency_hz,
                        struct euc_hall_out *out)
{
    /* Next state in the forward (+1) order 5 1 3 2 6 4. */
    static const uint8_t forward_next[8] = {0U, 3U, 6U, 2U, 5U, 1U, 4U, 0U};

    if (hall_state != state->step_sector) {
        const bool both_valid = state_is_valid(hall_state) && state_is_valid(state->step_sector);
        if (both_valid && (forward_next[state->step_sector] == hall_state)) {
            state->direction = 1;
            state->position = motor_wrap_i16((int32_t)state->position + 1);
            state->window_steps++;
        } else if (both_valid && (forward_next[hall_state] == state->step_sector)) {
            state->direction = -1;
            state->position = motor_wrap_i16((int32_t)state->position - 1);
            state->window_steps--;
        } else {
            /* invalid state or a skipped sector: the direction stays, the odometer still counts */
        }
        state->odometer_steps++;
        if (state->odometer_steps == EUC_HALL_ODOMETER_STEPS) {
            state->odometer_steps = 0U;
            out->odometer_step = true;
        }
    }
    state->step_sector = hall_state;

    const uint32_t window_limit_calls = ((uint32_t)pwm_frequency_hz * 4U) / 100U;
    const int32_t window_limit = (int32_t)window_limit_calls;
    const int16_t calls = state->window_calls;
    state->window_calls++;
    if (calls > window_limit) {
        state->windows++;
        state->window_calls = 0;
        state->steps_per_window = state->window_steps;
        state->block_steps = (int16_t)(state->block_steps + state->window_steps);
        state->window_steps = 0;
        if (state->windows > WINDOWS_PER_BLOCK) {
            state->windows = 0U;
            state->steps_11_windows = (int16_t)(state->block_steps - state->last_block_steps);
            if (state->steps_11_windows_max < state->steps_11_windows) {
                state->steps_11_windows_max = state->steps_11_windows;
            }
            state->last_block_steps = state->block_steps;
            state->block_steps = 0;
        }
    }
}

static int32_t period_speed_constant(uint16_t pwm_frequency_hz)
{
    if (EUC_QUIRK(HALL_SPEED_1_2X)) {
        return PERIOD_SPEED_CONSTANT;
    }
    /* Calls come at 2 fPWM: 202117 x 2 fPWM / 24000 = 202117 fPWM / 12000, rounded. Whole and remainder parts
     * keep the products in 32 bits (no 64-bit division in the image). */
    const uint32_t f = pwm_frequency_hz;
    const uint32_t divisor = (uint32_t)PERIOD_SPEED_CALL_RATE / 2U;
    const uint32_t whole = (uint32_t)PERIOD_SPEED_CONSTANT / divisor;
    const uint32_t rest = (uint32_t)PERIOD_SPEED_CONSTANT % divisor;
    const uint32_t constant = (whole * f) + (((rest * f) + (divisor / 2U)) / divisor);
    return (int32_t)constant;
}

static void take_edge(struct euc_hall_state *state, uint16_t pwm_frequency_hz)
{
    const uint16_t period_calls = state->calls_since_edge;
    state->edge_calls[state->edge_index] = period_calls;
    state->edge_index++;
    if (state->edge_index == EUC_HALL_RING_EDGES) {
        state->edge_index = 0U;
    }

    if (state->standstill_seen && (period_calls != 0U)) {
        state->period_speed_raw =
            (int32_t)state->direction * (-period_speed_constant(pwm_frequency_hz) / (int32_t)period_calls);
    } else {
        state->period_speed_raw = 0;
    }

    uint32_t turn_calls = 0U;
    for (uint32_t i = 0U; i < EUC_HALL_RING_EDGES; i++) {
        turn_calls += state->edge_calls[i];
    }
    /* At least six calls per turn in practice (one per edge), so the step fits easily. */
    const uint32_t turn_step = (turn_calls == 0U) ? 0U : ((uint32_t)ANGLE_ONE_TURN / turn_calls);
    const int32_t step = (int32_t)turn_step;
    state->angle_step = motor_wrap_i16((int32_t)state->direction * step);

    state->calls_since_edge = 0U;
    state->angle_progress = 0;
}

static void filter_period_speed(struct euc_hall_state *state)
{
    if ((state->period_speed_slewed - state->period_speed_raw) > PERIOD_SPEED_SLEW) {
        state->period_speed_raw = state->period_speed_slewed - PERIOD_SPEED_SLEW;
    }
    if ((state->period_speed_raw - state->period_speed_slewed) > PERIOD_SPEED_SLEW) {
        state->period_speed_raw = state->period_speed_slewed + PERIOD_SPEED_SLEW;
    }
    state->period_speed_slewed = state->period_speed_raw;
    if (EUC_QUIRK(HALL_SPEED_FILTER_STALL)) {
        /* Truncation stops the filter once it is within 999 counts of its input. */
        state->period_speed = ((state->period_speed * PERIOD_SPEED_FILTER_OLD) + state->period_speed_raw) /
                              PERIOD_SPEED_FILTER_DIV;
    } else {
        /* Same time constant, with 10 fraction bits kept, so the output reaches its input. */
        const int32_t target = state->period_speed_raw * PERIOD_SPEED_FRACTION;
        state->period_speed_fine += (target - state->period_speed_fine) / PERIOD_SPEED_FILTER_DIV;
        state->period_speed = state->period_speed_fine / PERIOD_SPEED_FRACTION;
    }
}

static void interpolate_angle(struct euc_hall_state *state, bool state_valid)
{
    if (state->calls_since_edge < INTERPOLATION_CALLS) {
        state->interpolating = true;
        if (state->angle_step >= ANGLE_STEP_THRESHOLD) {
            state->entry_offset = -ANGLE_30_DEG;
        } else if (state->angle_step <= -ANGLE_STEP_THRESHOLD) {
            state->entry_offset = ANGLE_30_DEG;
        } else {
            /* slow interpolation: keep the previous offset */
        }
        if ((state->angle_progress >= -ANGLE_PROGRESS_LIMIT) &&
            (state->angle_progress <= ANGLE_PROGRESS_LIMIT)) {
            state->angle_progress = (int16_t)(state->angle_progress + state->angle_step);
        }
        /* An invalid state does not reset the angle to a sector centre. The stock code still adds the
         * progress, so the angle runs on every call until the hall fault; clean holds it. */
        if (state_valid || EUC_QUIRK(HALL_INVALID_ANGLE_RUN)) {
            state->angle = motor_wrap_i16((int32_t)state->angle + (int32_t)state->angle_progress +
                                          (int32_t)state->entry_offset);
        }
    } else {
        state->entry_offset = 0;
        state->interpolating = false;
    }
}

void euc_hall_step(struct euc_hall_state *state, uint8_t lines, uint16_t pwm_frequency_hz,
                   struct euc_hall_out *out)
{
    /* Sector centre angle for each state; invalid states (0, 7) keep the previous angle. */
    static const int16_t sector_centre[8] = {0, -0x6AAC, -0x1555, -0x4001, 0x4000, 0x6AAA, 0x1556, 0};

    out->odometer_step = false;
    out->fault_raised = false;

    const uint8_t hall_state = filter_lines(state, lines);
    const bool state_valid = state_is_valid(hall_state);
    if (state_valid) {
        state->angle = sector_centre[hall_state];
    }
    count_steps(state, hall_state, pwm_frequency_hz, out);

    if (state_valid) {
        state->invalid_calls = 0U;
    } else {
        state->invalid_calls++;
    }
    if (((uint32_t)state->invalid_calls * 8U) > (uint32_t)pwm_frequency_hz) {
        out->fault_raised = !state->fault;
        state->fault = true;
    }

    bool count_call = true;
    if (hall_state == state->sector) {
        if (state->calls_since_edge > EDGE_TIMEOUT_CALLS) {
            state->period_speed_raw = 0;
            count_call = false;
        }
    } else {
        take_edge(state, pwm_frequency_hz);
    }
    if (count_call) {
        state->calls_since_edge++;
    }

    filter_period_speed(state);
    if (state->calls_since_edge > STANDSTILL_CALLS) {
        state->standstill_seen = true;
    }
    state->sector = hall_state;
    interpolate_angle(state, state_valid);
}
