/*
 * Supervisor of the clean profile's independent watchdog (quirk NO_WATCHDOG; see
 * docs/watchdog-and-overcurrent.md). One call per RTC tick, at the end of the tick.
 *
 * The balance runs in the 496 Hz tick and the FOC in the ADC interrupt; the main loop only does telemetry,
 * the BMS link and deferred flash writes. So:
 *   feed       only a tick in which the FOC ran since the last tick and the balance published a finite
 *              command within +-1050 feeds the watchdog. A dead tick or a dead FOC leaves the drive state
 *              unknown: the watchdog resets after 250 ms, and the boot after a watchdog reset keeps the
 *              bridge off until the next power-on.
 *   main loop  no main-loop pass for 248 ticks (0.5 s) while the tick runs latches "stalled". The balance
 *              still works and a reset would be a fall, so the app goes to limp mode (10 km/h speed
 *              tiltback, no restart after the next stop) and the stall beeps instead.
 * The stock firmware has no watchdog: nothing is fed and nothing is latched.
 */
#ifndef EUC_SAFETY_SUPERVISOR_H
#define EUC_SAFETY_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_SUPERVISOR_WATCHDOG_MS   250U
#define EUC_SUPERVISOR_STALL_TICKS   248U /* 0.5 s without a main-loop pass */
#define EUC_SUPERVISOR_COMMAND_LIMIT 1050.0F

struct euc_supervisor_state {
    uint32_t main_loop_passes_seen;
    uint32_t foc_steps_seen;
    uint16_t quiet_ticks;
    bool main_loop_stalled; /* latched until power-off */
};

struct euc_supervisor_in {
    uint32_t main_loop_passes; /* counted by the main loop */
    uint32_t foc_steps;        /* counted by the ADC interrupt */
    float motor_command;       /* the balance's published command */
};

struct euc_supervisor_out {
    bool feed_watchdog;
    bool main_loop_stalled;
};

void euc_supervisor_init(struct euc_supervisor_state *state);

/* Every RTC tick, after the balance. Clears `out` first. */
void euc_supervisor_tick(struct euc_supervisor_state *state, const struct euc_supervisor_in *in,
                         struct euc_supervisor_out *out);

#endif
