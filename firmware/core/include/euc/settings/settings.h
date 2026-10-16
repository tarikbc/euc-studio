/*
 * Rider settings and the odometer, and their 8-halfword record.
 *
 * The record has the same layout in a flash journal slot and in the backup registers DR3..DR10:
 *
 *   HW  bits   field                          HW  bits   field
 *   0   15:0   odometer, low word             4   15:12  'W' 'X' d (roll limit step)
 *   1   15:0   odometer, high word                11:8   'W' 'C' d
 *   2   15:12  pedal mode                         7:4    'W' 'M' d
 *       11:8   speed-alarm mode                   3:0    roll cut-off mode
 *       7:0    tiltback speed, km/h           5   15:8   'W' 'R' d (turn compensation)
 *   3   3:0    'W' 'U' d (pedal trim)             7:4    PWM tiltback ('e' / 'x')
 *       15:4   clean: CRC-16, high 12 bits        3:0    drive mode
 *   6   15:8   lock state (3 = locked)        7   15:8   PWM alarm threshold, %
 *       7:4    IMU start-up fault                 7:4    clean: CRC-16, low 4 bits
 *       3:0    units (1 = miles)                  3:0    beeper volume
 *
 * The CRC lives in bits the stock firmware never sets and never reads, so each firmware can read the other's
 * records (quirk NO_RECORD_CRC; see euc/settings/storage.h).
 *
 * Odometer unit: 118 hall steps (about 1 m). The trip counter (BKP DR1) counts the same units.
 */
#ifndef EUC_SETTINGS_SETTINGS_H
#define EUC_SETTINGS_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_SETTINGS_RECORD_HALFWORDS 8U
#define EUC_SETTINGS_LOCKED           3U /* lock_state value of the transport lock */

struct euc_settings {
    uint16_t odometer_low;
    uint16_t odometer_high;
    uint8_t pedal_mode;          /* 0 soft, 1 medium, 2 hard */
    uint8_t speed_alarm_mode;    /* 0..3: 'o' 'u' 'i' 'I' */
    uint16_t tiltback_speed_kmh; /* 200 = off */
    uint8_t pedal_trim;          /* 'W' 'U' d, 0..9 */
    uint8_t roll_limit_step;     /* 'W' 'X' d */
    uint8_t wc_value;            /* 'W' 'C' d */
    uint8_t wm_value;            /* 'W' 'M' d, 0..1 */
    uint8_t roll_cutoff_mode;    /* 0..2 */
    uint8_t turn_compensation;   /* 'W' 'R' d, 0..9 */
    uint8_t pwm_tiltback;        /* 1 on ('e'), 0 off ('x') */
    uint8_t drive_mode;          /* 0 voltage FOC 10 kHz, 1 current FOC 12 kHz; applies at the next boot */
    uint8_t lock_state;          /* EUC_SETTINGS_LOCKED = transport lock */
    uint8_t imu_fault;           /* 1 = the IMU failed its start-up check */
    uint8_t units_miles;         /* 1 = miles */
    uint8_t pwm_alarm_percent;   /* 'W' 'P' d d, 50..90 */
    uint8_t beeper_volume;       /* 'W' 'B' d, 0..9 */
};

/* Record of an empty journal: medium pedals, turn compensation 3, volume 6, everything else 0. */
void euc_settings_default_record(uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS]);

/* Packs the fields (each masked to its width). Clean profile: also writes the CRC bits. */
void euc_settings_pack(const struct euc_settings *settings, uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS]);

/*
 * Unpacks a record with the stock load rules: tiltback 0 -> 20 km/h, pedal trim > 9 -> 9, 'W' 'M' > 1 -> 0,
 * turn compensation > 9 -> 0, PWM alarm 0 -> 80 %, volume > 9 -> 9.
 */
void euc_settings_unpack(const uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS], struct euc_settings *settings);

/* CRC-16 (CCITT-FALSE) of the record with its CRC bits cleared. */
uint16_t euc_settings_record_crc(const uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS]);

/* The CRC bits of a record (HW3 bits 15:4 and HW7 bits 7:4). */
uint16_t euc_settings_record_stored_crc(const uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS]);

/* Odometer in units, high word x 65536 + low word. */
uint32_t euc_settings_odometer_units(const struct euc_settings *settings);

/*
 * One more odometer unit: the low word, the trip counter and, on carry, the high word. Stock carries when the
 * low word reaches 0xFFFF instead of when it wraps to 0, so for one unit in 65536 the odometer reads 65536
 * too high (quirk ODO_CARRY_EARLY). The caller writes the trip counter to BKP DR1.
 */
void euc_settings_add_odometer_unit(struct euc_settings *settings, uint16_t *trip_units);

#endif
