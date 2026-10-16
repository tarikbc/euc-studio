/*
 * Riderless detector of the clean profile (quirk RIDERLESS_UNDETECTED; see docs/riderless-detection.md).
 * One call per RTC tick, before the balance, with the readings the balance left in the previous tick.
 *
 * With a rider at a steady speed the motor carries the rider's weight against drag and rolling resistance
 * (9-13 N m at 15-20 km/h in the simulator); an empty wheel needs about 1 N m, and when the rider steps
 * off the torque falls to that within 0.1 s while the speed stays.
 *
 * Suspect: a sudden load drop (the q current low-passed over 0.05 s under 2.5 A while the one low-passed
 * over 0.3 s is still 4 A above it), then within 1.5 s: faster than 1 m/s and the 0.3 s current under
 * 3.5 A (3 N m at the default 0.86 N m/A, not measured) for 0.6 s, while the speed drops by less than
 * 0.15 m/s. A rider who coasts passes zero current slowly and loses speed, a rider who brakes goes on to a
 * braking current, and a rider cruising down a gentle slope has no load drop. Never in the first 3 s of
 * balancing, nor while a cut-off, limp mode, the battery limit or a tiltback is active.
 *
 * Respond: beeps and the limp speed limit (speed tiltback at 10 km/h, at most 2.5 deg/s and 12 deg). The
 * response stays gentle because a wrong suspicion must never throw a rider.
 *
 * Release: the 0.3 s current above 7 A (a rider leans against the tiltback; an empty wheel that weaves
 * swings the current both ways), slower than 1 m/s, or the balance stops.
 *
 * The stock firmware has no detector.
 */
#ifndef EUC_SAFETY_RIDERLESS_H
#define EUC_SAFETY_RIDERLESS_H

#include <stdbool.h>
#include <stdint.h>

struct euc_riderless_state {
    uint16_t balancing_ticks;        /* since the balance started, up to 3 s */
    uint16_t load_drop_ticks;        /* time left to suspect after a load drop */
    uint16_t quiet_ticks;            /* the suspect conditions held this long */
    float current_slow_a;            /* q current low-passed over 0.3 s, A */
    float current_fast_a;            /* q current low-passed over 0.05 s, A */
    float speed_filtered_m_per_s;    /* speed low-passed over 0.2 s */
    float speed_quiet_start_m_per_s; /* |filtered speed| when the quiet period began */
    bool suspected;
    uint8_t triggers; /* suspicions since power-on, saturating */
};

struct euc_riderless_in {
    bool balancing;      /* running with the drive enabled */
    bool other_limit;    /* a cut-off, limp mode, the battery limit or a tiltback is active */
    float speed_m_per_s; /* the balance's speed estimate */
    int32_t current_01a; /* q current of the last FOC step, 0.1 A */
};

void euc_riderless_init(struct euc_riderless_state *state);

/* Every RTC tick. Returns true while the wheel is suspected to be riderless (never in the stock profile). */
bool euc_riderless_tick(struct euc_riderless_state *state, const struct euc_riderless_in *in);

#endif
