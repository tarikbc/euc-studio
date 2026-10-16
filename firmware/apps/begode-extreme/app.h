/*
 * Begode Extreme application: the core modules, their wiring and the execution contexts.
 *
 *   TIM8 update (2 x fPWM, pre-emption 0)   light PWM, hall step, ADC start; FOC flagged at the underflow
 *   ADC1/2 end of injection (fPWM, 0)        FOC step, bridge outputs
 *   USART1 (2)                               BLE byte in, frame bytes out
 *   RTC second (496 Hz, 2)                   sensors, commands, balance, service, alarms, light, buttons
 *   main loop                                BMS link, telemetry and replies, flash writes, power-off, update
 *
 * Context hand-over uses volatile flags written in one context and read in another; every flag is a single
 * aligned word or byte, so no locking is needed.
 */
#ifndef APP_BEGODE_EXTREME_H
#define APP_BEGODE_EXTREME_H

#include <stdbool.h>
#include <stdint.h>

#include "euc/alarms/alarms.h"
#include "euc/balance/balance.h"
#include "euc/bms/bms.h"
#include "euc/lights/lights.h"
#include "euc/motor/motor.h"
#include "euc/power/power.h"
#include "euc/protocol/commands.h"
#include "euc/protocol/telemetry.h"
#include "euc/sensors/sensors.h"
#include "euc/service/service.h"
#include "euc/settings/storage.h"

struct app {
    struct euc_settings settings;
    struct euc_storage_state storage;
    struct euc_sensors_state sensors;
    struct euc_balance_state balance;
    struct euc_balance_in balance_in;
    struct euc_balance_out balance_out;
    struct euc_motor_state motor;
    struct euc_cmd_state commands;
    struct euc_tlm_state telemetry;
    struct euc_alarm_state alarms;
    struct euc_alarm_out alarm_out;
    struct euc_power_state power;
    struct euc_power_out power_out;
    struct euc_lights_state lights;
    struct euc_lights_out lights_out;
    struct euc_bms_state bms;
    struct euc_service_state service;
    uint16_t trip_units;
    uint16_t display_ticks;
    uint8_t bms_reply[EUC_BMS_REPLY_BYTES];

    /* Requests carried to the next RTC tick */
    bool horn_request;
    bool disable_request;
    uint8_t raise_alerts;

    /* RTC tick -> main loop */
    volatile uint32_t rtc_ticks;
    volatile bool save_request;
    volatile bool power_off_request;
    volatile bool power_off_wait;
    volatile bool bootloader_request;
    volatile bool erase_calibration_request;
    volatile bool storage_write_pending;
    struct euc_storage_write storage_write;

    /* RTC tick -> FOC */
    volatile int16_t foc_speed_dm_per_s;
    volatile bool foc_drive_disabled;
    volatile bool foc_hard_cut_request;

    /* TIM8 / ADC -> RTC tick */
    volatile bool foc_pending;
    volatile bool bridge_on;
    volatile bool start_balance;
    volatile bool stop_drive;
    volatile uint8_t motor_alerts;
    volatile uint16_t odometer_steps;
};

extern struct app app;

/* Storage pages from the board linker script (boards/begode-extreme/link_app.ld). */
extern const uint16_t ld_settings_page[];
extern const uint16_t ld_calibrated_page[];
extern const uint16_t ld_imu_offsets_page[];
extern const uint16_t ld_update_request_page[];

/* PB15 low. */
bool app_power_button_pressed(void);

/* RTC tick body (control.c). */
void app_rtc_tick(void);
/* Telemetry, replies, BMS link and the deferred flash and power actions (control.c). */
void app_main_loop_pass(void);
/* Applies the actions of one command call (control.c). */
void app_apply_commands(const struct euc_cmd_out *out);
/* The hall inputs as the hall module expects them. */
uint8_t app_hall_lines(void);

#endif
