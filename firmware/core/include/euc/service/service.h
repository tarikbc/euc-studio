/*
 * Service sequences from the BLE link (euc/protocol/commands.h detects the two-byte sequences):
 *
 *   '!' '@'  firmware update: erase the request page at 0x08010800, wait 200 ms, reset into the bootloader
 *   'c' 'y'  IMU calibration: erase the "calibrated" page at 0x0800F800; the next power-on calibrates
 *   '+' '-'  drive mode: toggle it, save the settings, wait 200 ms, release the power latch
 *
 * Once a sequence is accepted, every RTC tick cuts the motor (hard cut, drive disabled). The action itself
 * waits until the balance command has decayed to zero (exponent bits 0), so the wheel is never stopped under
 * load.
 *
 * The BLE link has no pairing, so any phone in range can send a sequence. The stock firmware accepts it at
 * once, even with a rider balancing at a standstill (quirk SERVICE_WITHOUT_CONFIRM). Clean accepts it only
 * when both hold:
 *   no rider      the balance is not running (start-up, fallen, tipped, handle cut, any soft disable)
 *   button        a short press of the power button (11..219 ticks) within 10 s (4960 ticks) of the sequence,
 *                 started after the sequence. Meanwhile three 15-tick beeps sound every 2 s (1000 ticks).
 * The wheel replies "SERVICE:CONFIRM" when it starts waiting, "SERVICE:OK" when the press accepts the
 * sequence, and "SERVICE:DENIED" when a rider is present (at the sequence or during the wait) or the 10 s
 * pass. A denied sequence does nothing. While a sequence waits, the press is not a light-mode press. A new
 * sequence replaces a waiting one.
 */
#ifndef EUC_SERVICE_SERVICE_H
#define EUC_SERVICE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_SERVICE_REPLY_CONFIRM_TEXT "SERVICE:CONFIRM\r\n"
#define EUC_SERVICE_REPLY_OK_TEXT      "SERVICE:OK\r\n"
#define EUC_SERVICE_REPLY_DENIED_TEXT  "SERVICE:DENIED\r\n"

enum euc_service_request {
    EUC_SERVICE_FIRMWARE_UPDATE,
    EUC_SERVICE_CALIBRATION,
    EUC_SERVICE_DRIVE_MODE_TOGGLE,
    EUC_SERVICE_COUNT
};

enum euc_service_reply {
    EUC_SERVICE_REPLY_NONE,
    EUC_SERVICE_REPLY_CONFIRM, /* clean: waiting for the power button */
    EUC_SERVICE_REPLY_OK,      /* clean: the press accepted the sequence */
    EUC_SERVICE_REPLY_DENIED   /* clean: rider present or no press within 10 s */
};

struct euc_service_state {
    uint8_t stage[EUC_SERVICE_COUNT]; /* 0 none, 1 wait for a zero command, 2 done, 3 wait for the button */
    uint16_t confirm_ticks;           /* ticks since the waiting sequence arrived */
    uint8_t press_ticks;              /* power button held, ticks (saturates) */
    bool press_after_request;         /* the current press started after the sequence */
};

struct euc_service_in {
    float balance_command; /* the balance's command before its output limits */
    bool no_rider;         /* the balance is not running */
    bool power_button_pressed;
};

struct euc_service_out {
    bool motor_off;         /* hard cut and drive disabled */
    bool enter_bootloader;  /* erase 0x08010800, wait 200 ms, reset */
    bool erase_calibration; /* erase 0x0800F800; the hard-cut reminder starts again */
    bool toggle_drive_mode; /* toggle the setting, save, wait 200 ms, power off */
    enum euc_service_reply reply;
    uint8_t beep_ticks; /* start a beep this long (0: none) */
};

void euc_service_init(struct euc_service_state *state);
void euc_service_request(struct euc_service_state *state, enum euc_service_request request);

/* A sequence waits for the power button: that press is not a light-mode press. */
bool euc_service_confirming(const struct euc_service_state *state);

/* Every RTC tick. Clears `out` first. */
void euc_service_tick(struct euc_service_state *state, const struct euc_service_in *in,
                      struct euc_service_out *out);

#endif
