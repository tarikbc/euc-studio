/*
 * Watchdog supervisor (see euc/safety/supervisor.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/float_ops.h"
#include "euc/quirks.h"
#include "euc/safety/supervisor.h"

void euc_supervisor_init(struct euc_supervisor_state *state)
{
    (void)memset(state, 0, sizeof(*state));
}

void euc_supervisor_tick(struct euc_supervisor_state *state, const struct euc_supervisor_in *in,
                         struct euc_supervisor_out *out)
{
    (void)memset(out, 0, sizeof(*out));
    if (EUC_QUIRK(NO_WATCHDOG)) {
        return;
    }

    if (in->main_loop_passes != state->main_loop_passes_seen) {
        state->main_loop_passes_seen = in->main_loop_passes;
        state->quiet_ticks = 0U;
    } else if (state->quiet_ticks < EUC_SUPERVISOR_STALL_TICKS) {
        state->quiet_ticks++;
    } else {
        state->main_loop_stalled = true;
    }
    out->main_loop_stalled = state->main_loop_stalled;

    const bool foc_ran = in->foc_steps != state->foc_steps_seen;
    state->foc_steps_seen = in->foc_steps;
    const float command = in->motor_command;
    const bool command_valid = euc_f32_is_finite(command) && (command <= EUC_SUPERVISOR_COMMAND_LIMIT) &&
                               (command >= -EUC_SUPERVISOR_COMMAND_LIMIT);
    out->feed_watchdog = foc_ran && command_valid;
}
