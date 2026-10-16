/*
 * Settings record (see euc/settings/settings.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/quirks.h"
#include "euc/settings/settings.h"

#define NIBBLE 0xFU
#define BYTE   0xFFU

/* CRC bits: HW3 bits 15:4 hold CRC bits 15:4, HW7 bits 7:4 hold CRC bits 3:0. */
#define CRC_HW3_INDEX 3U
#define CRC_HW7_INDEX 7U
#define CRC_HW3_MASK  0xFFF0U
#define CRC_HW7_MASK  0x00F0U
#define CRC_HW3_KEEP  0x000FU /* the data bits of HW3 */
#define CRC_HW7_KEEP  0xFF0FU /* the data bits of HW7 */
#define CRC_LOW_SHIFT 4U

#define CRC16_POLYNOMIAL 0x1021U
#define CRC16_INITIAL    0xFFFFU

#define DEFAULT_TILTBACK_KMH      20U
#define DEFAULT_PWM_ALARM_PERCENT 80U
#define MAX_DIGIT_SETTING         9U
#define MAX_WM_VALUE              1U
#define DEFAULT_PEDAL_MODE_RECORD 0x1000U /* pedal mode 1 (medium) */
#define DEFAULT_TURN_COMP_RECORD  0x0300U /* 'W' 'R' 3 */
#define DEFAULT_VOLUME_RECORD     0x0006U /* 'W' 'B' 6 */

void euc_settings_default_record(uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS])
{
    (void)memset(record, 0, EUC_SETTINGS_RECORD_HALFWORDS * sizeof(record[0]));
    record[2] = DEFAULT_PEDAL_MODE_RECORD;
    record[5] = DEFAULT_TURN_COMP_RECORD;
    record[7] = DEFAULT_VOLUME_RECORD;
}

static uint16_t field(uint32_t value, uint32_t mask, uint32_t shift)
{
    return (uint16_t)((value & mask) << shift);
}

uint16_t euc_settings_record_crc(const uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS])
{
    uint16_t crc = (uint16_t)CRC16_INITIAL;
    for (uint32_t i = 0U; i < EUC_SETTINGS_RECORD_HALFWORDS; i++) {
        uint16_t word = record[i];
        if (i == CRC_HW3_INDEX) {
            word = (uint16_t)(word & CRC_HW3_KEEP);
        } else if (i == CRC_HW7_INDEX) {
            word = (uint16_t)(word & CRC_HW7_KEEP);
        } else {
            /* no CRC bits */
        }
        const uint8_t bytes[2] = {(uint8_t)(word & BYTE), (uint8_t)(word >> 8U)};
        for (uint32_t b = 0U; b < 2U; b++) {
            crc ^= (uint16_t)((uint16_t)bytes[b] << 8U);
            for (uint32_t bit = 0U; bit < 8U; bit++) {
                const bool top = (crc & 0x8000U) != 0U;
                crc = (uint16_t)(crc << 1U);
                if (top) {
                    crc ^= (uint16_t)CRC16_POLYNOMIAL;
                }
            }
        }
    }
    return crc;
}

uint16_t euc_settings_record_stored_crc(const uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS])
{
    return (uint16_t)((record[CRC_HW3_INDEX] & CRC_HW3_MASK) |
                      ((record[CRC_HW7_INDEX] & CRC_HW7_MASK) >> CRC_LOW_SHIFT));
}

void euc_settings_pack(const struct euc_settings *settings, uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS])
{
    record[0] = settings->odometer_low;
    record[1] = settings->odometer_high;
    record[2] =
        (uint16_t)(field(settings->pedal_mode, NIBBLE, 12U) | field(settings->speed_alarm_mode, NIBBLE, 8U) |
                   field(settings->tiltback_speed_kmh, BYTE, 0U));
    record[3] = field(settings->pedal_trim, NIBBLE, 0U);
    record[4] =
        (uint16_t)(field(settings->roll_limit_step, NIBBLE, 12U) | field(settings->wc_value, NIBBLE, 8U) |
                   field(settings->wm_value, NIBBLE, 4U) | field(settings->roll_cutoff_mode, NIBBLE, 0U));
    record[5] =
        (uint16_t)(field(settings->turn_compensation, BYTE, 8U) | field(settings->pwm_tiltback, NIBBLE, 4U) |
                   field(settings->drive_mode, NIBBLE, 0U));
    record[6] = (uint16_t)(field(settings->lock_state, BYTE, 8U) | field(settings->imu_fault, NIBBLE, 4U) |
                           field(settings->units_miles, NIBBLE, 0U));
    record[7] =
        (uint16_t)(field(settings->pwm_alarm_percent, BYTE, 8U) | field(settings->beeper_volume, NIBBLE, 0U));

    if (!EUC_QUIRK(NO_RECORD_CRC)) {
        const uint16_t crc = euc_settings_record_crc(record);
        record[CRC_HW3_INDEX] = (uint16_t)(record[CRC_HW3_INDEX] | (crc & CRC_HW3_MASK));
        record[CRC_HW7_INDEX] =
            (uint16_t)(record[CRC_HW7_INDEX] | ((uint16_t)(crc << CRC_LOW_SHIFT) & CRC_HW7_MASK));
    }
}

static uint8_t nibble(uint16_t word, uint32_t shift)
{
    return (uint8_t)(((uint32_t)word >> shift) & NIBBLE);
}

static uint8_t high_byte(uint16_t word)
{
    return (uint8_t)(word >> 8U);
}

void euc_settings_unpack(const uint16_t record[EUC_SETTINGS_RECORD_HALFWORDS], struct euc_settings *settings)
{
    settings->odometer_low = record[0];
    settings->odometer_high = record[1];

    settings->pedal_mode = nibble(record[2], 12U);
    settings->speed_alarm_mode = nibble(record[2], 8U);
    settings->tiltback_speed_kmh = (uint16_t)(record[2] & BYTE);
    if (settings->tiltback_speed_kmh == 0U) {
        settings->tiltback_speed_kmh = DEFAULT_TILTBACK_KMH;
    }

    settings->pedal_trim = nibble(record[3], 0U);
    if (settings->pedal_trim > MAX_DIGIT_SETTING) {
        settings->pedal_trim = MAX_DIGIT_SETTING;
    }

    settings->roll_limit_step = nibble(record[4], 12U);
    settings->wc_value = nibble(record[4], 8U);
    settings->wm_value = nibble(record[4], 4U);
    settings->roll_cutoff_mode = nibble(record[4], 0U);
    if (settings->wm_value > MAX_WM_VALUE) {
        settings->wm_value = 0U;
    }

    settings->turn_compensation = high_byte(record[5]);
    settings->pwm_tiltback = nibble(record[5], 4U);
    settings->drive_mode = nibble(record[5], 0U);
    if (settings->turn_compensation > MAX_DIGIT_SETTING) {
        settings->turn_compensation = 0U;
    }

    settings->lock_state = high_byte(record[6]);
    settings->imu_fault = nibble(record[6], 4U);
    settings->units_miles = nibble(record[6], 0U);

    settings->pwm_alarm_percent = high_byte(record[7]);
    settings->beeper_volume = nibble(record[7], 0U);
    if (settings->pwm_alarm_percent == 0U) {
        settings->pwm_alarm_percent = DEFAULT_PWM_ALARM_PERCENT;
    }
    if (settings->beeper_volume > MAX_DIGIT_SETTING) {
        settings->beeper_volume = MAX_DIGIT_SETTING;
    }
}

uint32_t euc_settings_odometer_units(const struct euc_settings *settings)
{
    return ((uint32_t)settings->odometer_high << 16U) | (uint32_t)settings->odometer_low;
}

void euc_settings_add_odometer_unit(struct euc_settings *settings, uint16_t *trip_units)
{
    settings->odometer_low++;
    (*trip_units)++;
    const uint16_t carry_at = EUC_QUIRK(ODO_CARRY_EARLY) ? (uint16_t)0xFFFFU : (uint16_t)0U;
    if (settings->odometer_low == carry_at) {
        settings->odometer_high++;
    }
}
