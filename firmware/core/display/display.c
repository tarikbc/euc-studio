/*
 * Front display content (see euc/display/display.h). Float steps follow the stock refresh: the temperature
 * and odometer digits come from single-precision values, the speed from a double-precision km/h.
 */
#include <stdbool.h>
#include <stdint.h>

#include "euc/display/display.h"
#include "euc/math/float_ops.h"

#define SEG_A  0x80U
#define SEG_B  0x40U
#define SEG_C  0x20U
#define SEG_D  0x10U
#define SEG_E  0x08U
#define SEG_F  0x04U
#define SEG_G  0x02U
#define SEG_DP 0x01U

#define FLOAT_SIGN_BIT      0x80000000U
#define TEMP_LIMIT_BITS     0x42C7CCCDU /* 99.9f */
#define ODOMETER_LIMIT_BITS 0x497423FEU /* 999999.9f */
#define SPEED_LIMIT_KMH     199U
#define BLINK_PERIOD        500U
#define BLINK_DARK_AFTER    250U

/* Battery levels in units of 0.02 V (stock: 0.1 V x 67200 / 13440). */
#define BAR1_FROM 5001
#define BAR2_FROM 5451
#define BAR3_FROM 5700
#define BAR4_FROM 6050
#define BAR5_FROM 6350

uint8_t euc_display_digit(uint32_t digit)
{
    static const uint8_t segments[10] = {
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,         /* 0 */
        SEG_B | SEG_C,                                         /* 1 */
        SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,                 /* 2 */
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,                 /* 3 */
        SEG_B | SEG_C | SEG_F | SEG_G,                         /* 4 */
        SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,                 /* 5 */
        SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,         /* 6 */
        SEG_A | SEG_B | SEG_C,                                 /* 7 */
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, /* 8 */
        SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,         /* 9 */
    };
    return (digit < 10U) ? segments[digit] : 0U;
}

/* Float to unsigned with truncation; 0 for negative values and NaN, like the soft-float library. */
static uint32_t float_to_unsigned(float value)
{
    if (!(value > 0.0F)) {
        return 0U;
    }
    if (value >= 4294967296.0F) {
        return UINT32_MAX;
    }
    return (uint32_t)value;
}

/* Positive values above the limit (infinity and NaN too) become the limit; the comparison is on the bits. */
static float clamp_positive(float value, uint32_t limit_bits)
{
    const uint32_t bits = euc_f32_bits(value);
    return ((bits < FLOAT_SIGN_BIT) && (bits > limit_bits)) ? euc_f32_from_bits(limit_bits) : value;
}

static void render_temperature(float temperature_c, uint8_t grids[EUC_DISPLAY_GRIDS])
{
    const float t = clamp_positive(temperature_c, TEMP_LIMIT_BITS);
    const uint32_t whole = float_to_unsigned(t) & 0xFFFFU;
    grids[4] = euc_display_digit(whole / 10U);
    grids[3] = (uint8_t)(euc_display_digit(whole % 10U) | SEG_DP);
    const uint32_t tenths = float_to_unsigned(euc_f32_mul(t, 10.0F)) & 0xFFFFU;
    grids[2] = (uint8_t)(euc_display_digit(tenths % 10U) | SEG_DP); /* dp = degree sign */
}

uint32_t euc_display_speed(int16_t speed_mm_per_s)
{
    int32_t speed = (int32_t)speed_mm_per_s / 10;
    speed = (speed < 0) ? -speed : speed;
    const double kmh_exact = ((double)speed * 3.6) / 100.0;
    return (uint32_t)kmh_exact;
}

static void render_speed(int16_t speed_mm_per_s, uint8_t grids[EUC_DISPLAY_GRIDS])
{
    uint32_t kmh = euc_display_speed(speed_mm_per_s);
    if (kmh > SPEED_LIMIT_KMH) {
        kmh = SPEED_LIMIT_KMH;
    }
    grids[11] = euc_display_digit(kmh / 100U);
    grids[10] = grids[11];
    grids[8] = euc_display_digit((kmh % 100U) / 10U);
    grids[9] = grids[8];
    grids[7] = euc_display_digit(kmh % 10U);
    grids[6] = grids[7];
}

static void render_odometer(uint32_t odometer_units, uint8_t grids[EUC_DISPLAY_GRIDS])
{
    const uint32_t tenths = odometer_units / 100U;
    const float tenths_km = clamp_positive((float)tenths, ODOMETER_LIMIT_BITS);
    grids[1] = euc_display_digit(float_to_unsigned(euc_f32_div(tenths_km, 100000.0F)));
    const uint32_t value = float_to_unsigned(tenths_km);
    grids[0] = euc_display_digit((value % 100000U) / 10000U);
    grids[15] = euc_display_digit((value % 10000U) / 1000U);
    grids[14] = euc_display_digit((value % 1000U) / 100U);
    grids[13] = (uint8_t)(euc_display_digit((value % 100U) / 10U) | SEG_DP);
    grids[12] = euc_display_digit(value % 10U);
}

static uint32_t battery_bars(int32_t voltage_cv, uint16_t time_base)
{
    const int32_t level = ((voltage_cv / 10) * 67200) / 13440; /* 0.02 V */
    if (level >= BAR5_FROM) {
        return 5U;
    }
    if (level >= BAR4_FROM) {
        return 4U;
    }
    if (level >= BAR3_FROM) {
        return 3U;
    }
    if (level >= BAR2_FROM) {
        return 2U;
    }
    if (level >= BAR1_FROM) {
        return 1U;
    }
    return ((time_base % BLINK_PERIOD) > BLINK_DARK_AFTER) ? 0U : 1U;
}

void euc_display_render(const struct euc_display_in *in, uint8_t grids[EUC_DISPLAY_GRIDS])
{
    static const uint8_t bar_grids[5][2] = {{10U, 11U}, {8U, 9U}, {6U, 7U}, {4U, 5U}, {0U, 1U}};

    render_temperature(in->board_temperature_c, grids);
    render_speed(in->speed_mm_per_s, grids);
    render_odometer(in->odometer_units, grids);
    grids[5] = euc_display_digit((uint32_t)in->pedal_mode + 1U);

    const uint32_t bars = battery_bars(in->battery_voltage_cv, in->time_base);
    for (uint32_t bar = 0U; bar < bars; bar++) {
        grids[bar_grids[bar][0]] |= SEG_DP;
        grids[bar_grids[bar][1]] |= SEG_DP;
    }
}
