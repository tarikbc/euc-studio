/*
 * Front display content: the 16 grid bytes of the TM1640 (grid i = address 0xC0 + i), refreshed every 100 RTC
 * ticks. The mapping of grids and bits to digits follows profiles/begode-extreme/display.toml.
 *
 *   speed      grids 11+10, 8+9, 7+6 (each digit twice): |speed| in km/h (mph), truncated, at most 199
 *   battery    dp bits: bar 1 = grids 10+11, 2 = 8+9, 3 = 6+7, 4 = 4+5, 5 = 0+1; from 100.1 / 109.1 / 114.0 /
 *              121.0 / 127.0 V; below 100.1 V bar 1 blinks (dark when the alarm time base % 500 > 250)
 *   pedal      grid 5: pedal mode + 1
 *   board temp grids 4, 3 (+ point), 2 (+ degree sign): filtered NTC, 0.1 degC, at most 99.9
 *   odometer   grids 1, 0, 15, 14, 13 (+ point), 12: odometer / 100 in 0.1 km (0.1 mi), at most 99999.9
 *
 * Segment bits: a = 7, b = 6, c = 5, d = 4, e = 3, f = 2, g = 1, dp = 0.
 */
#ifndef EUC_DISPLAY_DISPLAY_H
#define EUC_DISPLAY_DISPLAY_H

#include <stdint.h>

#define EUC_DISPLAY_GRIDS 16U

struct euc_display_in {
    int16_t speed_mm_per_s;  /* signed mm/s, x 0.62 when the units are miles */
    uint32_t odometer_units; /* odometer in units of 118 hall steps (miles-scaled when units are miles) */
    uint8_t pedal_mode;
    float board_temperature_c; /* filtered NTC */
    int32_t battery_voltage_cv;
    uint16_t time_base; /* alarm time base */
};

/* The speed the display shows, before its 199 limit: |speed| in whole km/h (mph with miles-scaled input). */
uint32_t euc_display_speed(int16_t speed_mm_per_s);

/* Seven-segment pattern of a decimal digit; 0 (dark) above 9. */
uint8_t euc_display_digit(uint32_t digit);

void euc_display_render(const struct euc_display_in *in, uint8_t grids[EUC_DISPLAY_GRIDS]);

#endif
