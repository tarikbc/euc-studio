/*
 * Power latch and buttons: the power-on hold check, the transport-lock gesture, the long-press power-off and
 * the idle auto power-off.
 *
 * Hardware: PB15 is the power / light button (pressed = low), PB14 holds the power on once set (the latch),
 * PA12 is the handle button (pressed = low). The port reads the pins; this module only decides.
 *
 * Power-on hold (before the RTC tick starts; euc_power_boot_*): after 100 ms, on a cold start the button must
 * be held for more than 220 polls of 2 ms (0.44 s) with the wheel still; a release of more than 250 polls, or
 * 1000 polls without a release while the wheel turns, means "do not stay on". From the tenth boot counted in
 * BKP DR2 (resets without power loss) the check only waits until the wheel is still for 4 polls of 10 ms.
 * Wheel motion = a change of the hall step position.
 *
 * Every RTC tick (euc_power_tick, after the alarms and lights):
 *   lock gesture  at standstill (|v| <= 0.49 m/s), handle button held: 5 power-button presses of 30 ticks,
 *                 each started within 5 s of the last, toggle the transport lock (lock 3: motor off until
 *                 reset; unlock writes 1). 10-tick beep per press, 250-tick beep on the toggle.
 *   power-off     power button held 401 ticks (0.81 s) at |v| <= 0.50 m/s: motor off, save, latch off. The
 *                 count starts above the limit at power-on, so the button held from boot must be released.
 *   auto-off      no hall step for 7200 s (or 7200 s with the drive disabled): eight 300-tick beeps one per
 *                 second, then save and latch off.
 */
#ifndef EUC_POWER_POWER_H
#define EUC_POWER_POWER_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_POWER_MAX_BEEPS 3U

enum euc_power_boot_decision {
    EUC_POWER_BOOT_WAIT, /* poll again */
    EUC_POWER_BOOT_STAY_ON,
    EUC_POWER_BOOT_TURN_OFF /* keep the latch low: the controller goes off when the button is released */
};

struct euc_power_boot_state {
    bool warm_start; /* boot count > 9 */
    uint16_t pressed_polls;
    uint16_t released_polls;
    uint16_t polls_since_release;
    uint16_t still_polls;
    int16_t last_position;
};

struct euc_power_in {
    bool power_button_pressed;  /* PB15 low */
    bool handle_button_pressed; /* PA12 low */
    int16_t speed_cm_per_s;     /* balance speed estimate */
    int16_t hall_position;      /* signed hall steps since power-on */
    bool drive_disabled;        /* balance soft disable */
    uint8_t lock_state;         /* setting; 3 = locked */
};

struct euc_power_state {
    /* lock gesture */
    bool gesture_blocked;
    uint8_t handle_release_ticks;
    uint8_t press_ticks;
    uint8_t presses;
    uint16_t button_release_ticks;
    /* long-press power-off */
    uint16_t hold_ticks;
    bool powering_off;
    /* auto power-off */
    int16_t last_position;
    uint16_t second_ticks;
    uint16_t idle_seconds;
    uint8_t warnings;
};

struct euc_power_out {
    bool power_off;     /* set the latch low */
    bool save_settings; /* save before the latch goes low */
    bool hard_cut;      /* bridge off until reset */
    bool disable_drive;
    uint8_t lock_state;            /* the lock setting after this tick */
    uint16_t auto_off_remaining_s; /* 7200 - idle seconds (frame 04, may wrap below 0) */
    uint8_t beep_count;
    uint16_t beep_ticks[EUC_POWER_MAX_BEEPS]; /* to start in this order (each replaces the previous) */
};

/* Power-on hold check. `boot_count` is BKP DR2 after this boot's increment. */
void euc_power_boot_init(struct euc_power_boot_state *state, uint16_t boot_count);

/* Poll period: 2 ms on a cold start (wait, then poll), 10 ms on a warm start (poll, then wait). */
uint16_t euc_power_boot_poll_ms(const struct euc_power_boot_state *state);

/* One poll, after the hall step of that poll. */
enum euc_power_boot_decision euc_power_boot_poll(struct euc_power_boot_state *state, bool button_pressed,
                                                 int16_t hall_position);

/* Power-on state of the RTC-tick logic. A lock setting of 3 at power-on also means: motor off until reset. */
void euc_power_init(struct euc_power_state *state);

/* One RTC tick. Clears `out` first. */
void euc_power_tick(struct euc_power_state *state, const struct euc_power_in *in, struct euc_power_out *out);

#endif
