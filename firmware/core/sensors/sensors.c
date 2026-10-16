/*
 * Slow analog sensors (see euc/sensors/sensors.h). The order of the floating-point operations follows the
 * stock conversions, so both round alike.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/math/float_ops.h"
#include "euc/math/libm.h"
#include "euc/quirks.h"
#include "euc/sensors/sensors.h"

#define ADC_LEFT_ALIGN_SHIFT 3U /* injected data register = raw x 8 */

/* Battery voltage */
#define VOLTAGE_SAMPLE_SCALE   100U
#define VOLTAGE_FILTER_OLD     999U
#define VOLTAGE_FILTER_DIVISOR 1000U
#define VOLTAGE_GAIN           1377U /* 0.01 V per 32768 counts of the filter */
#define VOLTAGE_GAIN_SHIFT     15U
#define VOLTAGE_FAST_TICKS     200U
#define VOLTAGE_SETTLED_COUNT  220U
#define VOLTAGE_PREVIOUS_AT_ON 300000U
#define VOLTAGE_SHOWN_AT_ON_CV 12000
#define VOLTAGE_MIN_AT_ON_CV   100000

/* Range estimate, piecewise in 0.01 V (the stock steps at 106.00 V and 128.00 V are kept). */
#define RANGE_HIGH_FROM_CV 12801
#define RANGE_HIGH_BASE_CV 12800
#define RANGE_MID_FROM_CV  10601
#define RANGE_MID_TO_CV    12799
#define RANGE_MID_BASE_CV  10600
#define RANGE_LOW_FROM_CV  9601
#define RANGE_LOW_BASE_CV  9600

/* Bus current */
#define CURRENT_OFFSET_SAMPLES 9U
#define CURRENT_LIVE_COUNT     10U
#define CURRENT_GAIN           43750
#define CURRENT_DIVISOR        32767
#define CURRENT_RAW_MASK       0x7FFFU

/* Board NTC */
#define NTC_SUPPLY_VOLTS      3.3
#define NTC_PULLUP_OHMS       10000.0
#define NTC_R25_OHMS          47000.0
#define NTC_BETA              4150.0
#define NTC_INV_T25_STOCK     (1.0 / 298.5)
#define NTC_INV_T25           (1.0 / 298.15)
#define KELVIN_OFFSET         273.15
#define NTC_SLEW_PER_TICK     3.0F
#define NTC_FILTER_OLD        0.9
#define NTC_FILTER_NEW        0.1
#define MPU6050_TEMP_OFFSET_C 21.0
#define MPU6050_COUNTS_PER_C  334.0
#define MPU6050_COUNTS_OFFSET 5280.0

#define MILES_PER_KM 0.62

void euc_sensors_init(struct euc_sensors_state *state)
{
    (void)memset(state, 0, sizeof(*state));
    state->voltage_previous = VOLTAGE_PREVIOUS_AT_ON;
    state->battery_voltage_cv = VOLTAGE_SHOWN_AT_ON_CV;
    state->battery_voltage_min_cv = VOLTAGE_MIN_AT_ON_CV;
}

uint32_t euc_sensors_range_km(int32_t voltage_cv)
{
    double range;
    if (voltage_cv >= RANGE_HIGH_FROM_CV) {
        const int32_t above = voltage_cv - RANGE_HIGH_BASE_CV;
        range = ((double)above * 0.012) + 115.0;
    } else if ((voltage_cv >= RANGE_MID_FROM_CV) && (voltage_cv <= RANGE_MID_TO_CV)) {
        const int32_t above = voltage_cv - RANGE_MID_BASE_CV;
        range = ((double)above * 0.05) + 5.0;
    } else if (voltage_cv >= RANGE_LOW_FROM_CV) {
        /* also 128.00 V exactly, which the stock comparisons send here */
        const int32_t above = voltage_cv - RANGE_LOW_BASE_CV;
        range = (double)above * 0.05;
    } else {
        range = 0.0;
    }
    return (uint32_t)range;
}

void euc_sensors_battery(struct euc_sensors_state *state, uint16_t adc1_jdr2)
{
    state->voltage_sample = ((uint32_t)adc1_jdr2 >> ADC_LEFT_ALIGN_SHIFT) * VOLTAGE_SAMPLE_SCALE;
    const uint32_t filtered =
        ((state->voltage_previous * VOLTAGE_FILTER_OLD) + state->voltage_sample) / VOLTAGE_FILTER_DIVISOR;
    const uint32_t voltage = (filtered * VOLTAGE_GAIN) >> VOLTAGE_GAIN_SHIFT;
    state->battery_voltage_cv = (int32_t)voltage;

    const bool settled = state->voltage_settle > VOLTAGE_FAST_TICKS;
    state->voltage_settle++;
    if (settled) {
        state->voltage_settle = VOLTAGE_SETTLED_COUNT;
        state->voltage_previous = filtered;
        const int32_t min_magnitude = (state->battery_voltage_min_cv < 0) ? -state->battery_voltage_min_cv
                                                                          : state->battery_voltage_min_cv;
        if (min_magnitude > state->battery_voltage_cv) {
            state->battery_voltage_min_cv = state->battery_voltage_cv;
        }
    } else {
        state->voltage_previous = state->voltage_sample;
    }
    state->range_estimate_km = euc_sensors_range_km(state->battery_voltage_cv);
}

void euc_sensors_bus_current(struct euc_sensors_state *state, uint16_t adc2_jdr2)
{
    const uint32_t raw = ((uint32_t)adc2_jdr2 & CURRENT_RAW_MASK) * 2U;
    state->current_raw = (uint16_t)raw;
    state->current_samples++;
    if (state->current_samples < CURRENT_LIVE_COUNT) {
        state->current_offset += raw;
        return;
    }
    if (state->current_samples == CURRENT_LIVE_COUNT) {
        state->current_offset /= CURRENT_OFFSET_SAMPLES;
        return;
    }
    state->current_samples = (uint8_t)CURRENT_LIVE_COUNT;
    const int64_t product = ((int64_t)raw - (int64_t)state->current_offset) * CURRENT_GAIN;
    state->bus_current_ca = (int32_t)(product / CURRENT_DIVISOR);
    const double sample = (double)(float)state->bus_current_ca * 0.01;
    state->bus_current_filtered_ca =
        euc_f32_from_f64(((double)state->bus_current_filtered_ca * 0.99) + sample);
    if (state->current_peak_raw < raw) {
        state->current_peak_raw = (uint16_t)raw;
    }
}

void euc_sensors_board_temperature(struct euc_sensors_state *state, uint16_t adc1_jdr3)
{
    const uint32_t raw = (uint32_t)adc1_jdr3 >> ADC_LEFT_ALIGN_SHIFT;
    state->ntc_volts = euc_f32_from_f64(((double)raw * NTC_SUPPLY_VOLTS) / 4096.0);
    const double volts = (double)state->ntc_volts;
    const float resistance = euc_f32_from_f64((volts * NTC_PULLUP_OHMS) / (NTC_SUPPLY_VOLTS - volts));
    const double inv_t25 = EUC_QUIRK(NTC_REFERENCE_298_5) ? NTC_INV_T25_STOCK : NTC_INV_T25;
    const double inv_t = (euc_log((double)resistance / NTC_R25_OHMS) / NTC_BETA) + inv_t25;
    state->board_temperature_c = euc_f32_from_f64((1.0 / inv_t) - KELVIN_OFFSET);

    const float previous = state->board_temperature_slewed_c;
    float slewed = state->board_temperature_c;
    if (euc_f32_sub(slewed, previous) > NTC_SLEW_PER_TICK) {
        slewed = euc_f32_add(previous, NTC_SLEW_PER_TICK);
    } else if (euc_f32_sub(previous, slewed) > NTC_SLEW_PER_TICK) {
        slewed = euc_f32_sub(previous, NTC_SLEW_PER_TICK);
    } else {
        /* within the slew limit */
    }
    state->board_temperature_slewed_c = slewed;
    state->board_temperature_filtered_c = euc_f32_from_f64(
        ((double)slewed * NTC_FILTER_NEW) + ((double)state->board_temperature_filtered_c * NTC_FILTER_OLD));
    state->board_temperature_mpu6050 = euc_f32_from_f64(
        (((double)state->board_temperature_filtered_c - MPU6050_TEMP_OFFSET_C) * MPU6050_COUNTS_PER_C) -
        MPU6050_COUNTS_OFFSET);
}

uint32_t euc_sensors_miles_u32(uint32_t value)
{
    const double miles = (double)value * MILES_PER_KM;
    return (uint32_t)miles;
}

int32_t euc_sensors_miles_i32(int32_t value)
{
    const double miles = (double)value * MILES_PER_KM;
    return (int32_t)miles;
}

bool euc_sensors_imu_calibration_valid(const int16_t means[6])
{
    /* A level, still wheel: little acceleration on x and y, about 1 g on z, almost no rotation. */
    static const int32_t level_limit = 1500;
    static const int32_t gravity_min = 7000;
    static const int32_t rate_limit = 700;
    for (uint32_t axis = 0U; axis < 2U; axis++) {
        const int32_t value = means[axis];
        if ((value <= -level_limit) || (value >= level_limit)) {
            return false;
        }
    }
    if ((int32_t)means[2] <= gravity_min) {
        return false;
    }
    for (uint32_t axis = 3U; axis < 6U; axis++) {
        const int32_t value = means[axis];
        if ((value <= -rate_limit) || (value >= rate_limit)) {
            return false;
        }
    }
    return true;
}
