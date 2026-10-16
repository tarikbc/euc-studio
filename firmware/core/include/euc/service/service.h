/*
 * Service sequences from the BLE link (euc/protocol/commands.h detects the two-byte sequences):
 *
 *   '!' '@'  firmware update: erase the request page at 0x08010800, wait 200 ms, reset into the bootloader
 *   'c' 'y'  IMU calibration: erase the "calibrated" page at 0x0800F800; the next power-on calibrates
 *   '+' '-'  drive mode: toggle it, save the settings, wait 200 ms, release the power latch
 *
 * From the request on, every RTC tick cuts the motor (hard cut, drive disabled). The action itself waits
 * until the balance command has decayed to zero (exponent bits 0), so the wheel is never stopped under load.
 */
#ifndef EUC_SERVICE_SERVICE_H
#define EUC_SERVICE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

enum euc_service_request {
    EUC_SERVICE_FIRMWARE_UPDATE,
    EUC_SERVICE_CALIBRATION,
    EUC_SERVICE_DRIVE_MODE_TOGGLE,
    EUC_SERVICE_COUNT
};

struct euc_service_state {
    uint8_t stage[EUC_SERVICE_COUNT]; /* 0 none, 1 waiting for a zero command, 2 done */
};

struct euc_service_out {
    bool motor_off;         /* hard cut and drive disabled */
    bool enter_bootloader;  /* erase 0x08010800, wait 200 ms, reset */
    bool erase_calibration; /* erase 0x0800F800; the hard-cut reminder starts again */
    bool toggle_drive_mode; /* toggle the setting, save, wait 200 ms, power off */
};

void euc_service_init(struct euc_service_state *state);
void euc_service_request(struct euc_service_state *state, enum euc_service_request request);

/* Every RTC tick. `balance_command` is the balance's command before its output limits. Clears `out` first. */
void euc_service_tick(struct euc_service_state *state, float balance_command, struct euc_service_out *out);

#endif
