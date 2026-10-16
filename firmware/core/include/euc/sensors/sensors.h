/*
 * Slow analog sensors, one call each per RTC tick (2.016 ms), from the injected ADC results of the last motor
 * sample: battery voltage (ADC1 JDR2, PA0), bus current (ADC2 JDR2, PC0) and board NTC (ADC1 JDR3, PC1).
 * Inputs are the data registers as read: 12-bit samples left-aligned with the injected sign bit (raw x 8).
 *
 * Battery voltage, 0.01 V: sample = raw x 100; filtered = (previous x 999 + sample) / 1000; volts =
 * filtered x 1377 / 32768. For the first 201 ticks the previous value is the last sample (fast start), then
 * the filter state (time constant 1000 ticks, 2 s). Also the range estimate the app shows.
 * Bus current, 0.01 A: offset = mean of the first 9 samples, then (raw x 16 - offset) x 43750 / 32767,
 * low-passed 0.99 / 0.01 in floating point.
 * Board NTC: 10 kohm pull-up to 3.3 V, 47 kohm NTC, B = 4150, Steinhart-Hart in double precision,
 * slew-limited to 3 degC per tick, low-passed 0.9 / 0.1, and encoded as MPU6050 counts ((T - 21) x 334 -
 * 5280) for the balance and telemetry. Stock takes the reference temperature as 298.5 K, so it reads 0.35
 * degC high (quirk NTC_REFERENCE_298_5); clean uses 298.15 K.
 */
#ifndef EUC_SENSORS_SENSORS_H
#define EUC_SENSORS_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

struct euc_sensors_state {
    /* battery */
    uint32_t voltage_sample;   /* raw x 100 */
    uint32_t voltage_previous; /* the filter input of the next tick */
    uint8_t voltage_settle;    /* tick count of the fast start */
    int32_t battery_voltage_cv;
    int32_t battery_voltage_min_cv;
    uint32_t range_estimate_km;
    /* bus current */
    uint8_t current_samples;
    uint32_t current_offset;
    uint16_t current_raw;
    uint16_t current_peak_raw;
    int32_t bus_current_ca;
    float bus_current_filtered_ca;
    /* board NTC */
    float ntc_volts;
    float board_temperature_c;
    float board_temperature_slewed_c;
    float board_temperature_filtered_c;
    float board_temperature_mpu6050;
};

/* Power-on state (the stock start values: 125.9 V in the filter, 120.00 V shown, 0 degC). */
void euc_sensors_init(struct euc_sensors_state *state);

void euc_sensors_battery(struct euc_sensors_state *state, uint16_t adc1_jdr2);
void euc_sensors_bus_current(struct euc_sensors_state *state, uint16_t adc2_jdr2);
void euc_sensors_board_temperature(struct euc_sensors_state *state, uint16_t adc1_jdr3);

/* Range estimate in km from the battery voltage, the stock piecewise line: 0 below 96.01 V, then 0.05 km per
 * 0.01 V; from 106.01 V 5 km + 0.05 km per 0.01 V; above 128.00 V 115 km + 0.012 km per 0.01 V. The steps
 * at 106.01 V and at exactly 128.00 V (160 km) are the stock comparisons, kept in both profiles. */
uint32_t euc_sensors_range_km(int32_t voltage_cv);

/* IMU calibration means (accel x, y, z, gyro x, y, z) of a level, still wheel: |a_xy| < 1500, a_z > 7000
 * (1 g = 8192), |g| < 700. Otherwise the calibration is not stored. */
bool euc_sensors_imu_calibration_valid(const int16_t means[6]);

/* Distance or speed in miles: x 0.62 in double precision, truncated (frame and display values). */
uint32_t euc_sensors_miles_u32(uint32_t value);
int32_t euc_sensors_miles_i32(int32_t value);

#endif
