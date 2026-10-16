/*
 * LED strip patterns (see euc/lights/strip.h).
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "euc/lights/strip.h"
#include "euc/quirks.h"

#define RING_LEDS         12U /* LEDs per ring; ring positions 1..12 */
#define RING_FORWARD_LAST 72  /* the ring position wraps to 0 past this going forward */
#define RING_BACKWARD_TOP 73  /* ... and to this below 1 going backward */
#define RING_MOVE_MM_S    4   /* faster than this (either way) moves the ring */
#define RING_SLOW_KMH     30U /* up to this display speed the ring period is 100 - 3 x speed ticks */
#define RING_SLOW_BASE    100U
#define RING_SLOW_SLOPE   3U
#define RING_FAST_TICKS   10U

#define SPEED_NOTE_TICKS     150U
#define BRAKE_DROP_KMH       3
#define BRAKE_COUNT_SET      40U
#define BRAKE_COUNT_REST     3U
#define BRAKE_FLASH_MIN      4U
#define TEN_TICKS            10U
#define STILL_KMH            2U
#define STILL_COUNT_MAX      40U
#define STANDSTILL_BREATHING 1U
#define STANDSTILL_MOVING    3U
#define UPDATE_TICKS         5U

#define BREATH_TOP        130U /* rising past this turns it down: the top is 131 */
#define BREATH_STEP_TICKS 4U   /* one step per 4 ticks: 2.11 s per breath, as on the wheel */

#define BRAKE_PERIOD     50U
#define BRAKE_DARK_PHASE 25U
#define BRAKE_LEDS       24U

#define TURN_PERIOD       150U
#define TURN_OFF_POSITIVE 100U /* roll above +4 deg */
#define TURN_OFF_NEGATIVE 75U  /* roll below -4 deg */
#define TURN_ROLL_DEG     4.0F

#define FULL_BRIGHTNESS 100U
#define COLOUR_MAX      255U

#define WHEEL_STEPS   72U
#define WHEEL_STEP    21U
#define WHEEL_UP_TOP  231U /* past this a rising channel saturates */
#define WHEEL_DOWN_LO 24U  /* under this a falling channel drops to 0 */
#define WHEEL_FULL    0xFFU

/* Ring position (1..12) to LED (1-based), per side. */
static uint32_t side_a_led(uint32_t position)
{
    static const uint8_t leds[RING_LEDS] = {8U, 9U, 10U, 11U, 12U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};
    return ((position >= 1U) && (position <= RING_LEDS)) ? leds[position - 1U] : 0U;
}

static uint32_t side_b_led(uint32_t position)
{
    return ((position >= 1U) && (position <= RING_LEDS)) ? ((2U * RING_LEDS) + 1U - position) : 0U;
}

void euc_strip_init(struct euc_strip_state *state)
{
    (void)memset(state, 0, sizeof(*state));
    state->brake_count = 1U;
    state->breath_rising = true;
    state->wheel_red = WHEEL_FULL;
}

static void move_ring(struct euc_strip_state *state, const struct euc_strip_tick_in *in, uint32_t t)
{
    const uint32_t speed = in->display_speed;
    const uint32_t period =
        (speed <= RING_SLOW_KMH) ? (RING_SLOW_BASE - (RING_SLOW_SLOPE * speed)) : RING_FAST_TICKS;
    const bool step = (t % period) == 0U;
    if (in->speed_mm_per_s > RING_MOVE_MM_S) {
        if (step) {
            state->ring++;
        }
        if (state->ring > RING_FORWARD_LAST) {
            state->ring = 0;
        }
    } else if (in->speed_mm_per_s < -RING_MOVE_MM_S) {
        if (step) {
            state->ring--;
        }
        if (state->ring < 1) {
            state->ring = RING_BACKWARD_TOP;
        }
    } else {
        /* the ring rests */
    }
}

static void watch_braking(struct euc_strip_state *state, uint32_t speed, uint32_t t)
{
    if ((t % SPEED_NOTE_TICKS) == 0U) {
        state->speed_note = (uint8_t)speed;
    }
    if (((int32_t)state->speed_note - (int32_t)speed) < BRAKE_DROP_KMH) {
        if ((t % TEN_TICKS) == 0U) {
            state->brake_count = (state->brake_count > BRAKE_COUNT_REST) ? (uint8_t)(state->brake_count - 1U)
                                                                         : (uint8_t)BRAKE_COUNT_REST;
        }
    } else {
        state->brake_count = BRAKE_COUNT_SET;
    }
}

static void watch_standstill(struct euc_strip_state *state, uint32_t speed, uint32_t t)
{
    if (speed < STILL_KMH) {
        if ((t % TEN_TICKS) == 0U) {
            state->still_count++;
        }
        if (state->still_count > STILL_COUNT_MAX) {
            state->still_count = 0U;
            state->standstill = STANDSTILL_BREATHING;
        }
    } else {
        state->still_count = 0U;
        state->standstill = STANDSTILL_MOVING;
    }
}

void euc_strip_tick(struct euc_strip_state *state, const struct euc_strip_tick_in *in)
{
    const uint32_t t = in->time_base;
    move_ring(state, in, t);
    watch_braking(state, in->display_speed, t);
    watch_standstill(state, in->display_speed, t);
    if ((t % UPDATE_TICKS) == 0U) {
        state->update = true;
    }
}

/* The ring position as an unsigned start index (it is never negative). */
static uint32_t ring_start(const struct euc_strip_state *state)
{
    const int32_t ring = state->ring;
    return (ring > 0) ? (uint32_t)ring : 0U;
}

static uint8_t scale(uint32_t brightness, uint32_t colour)
{
    return (uint8_t)(((brightness * colour) / COLOUR_MAX) & 0xFFU);
}

/* Sets LED `led` (1..32; anything else is ignored) to the colour at `brightness` (0..255). */
static void set_led(struct euc_strip_state *state, uint32_t led, uint32_t brightness,
                    struct euc_strip_pixel colour)
{
    if ((led == 0U) || (led > EUC_STRIP_LEDS)) {
        return;
    }
    const struct euc_strip_pixel pixel = {scale(brightness, colour.red), scale(brightness, colour.green),
                                          scale(brightness, colour.blue)};
    struct euc_strip_pixel *target = &state->pixels[led - 1U];
    if ((target->red != pixel.red) || (target->green != pixel.green) || (target->blue != pixel.blue)) {
        *target = pixel;
        state->changed |= (uint32_t)1U << (led - 1U);
    }
}

static const struct euc_strip_pixel red = {255U, 0U, 0U};
static const struct euc_strip_pixel amber = {255U, 200U, 0U};
static const struct euc_strip_pixel dark = {0U, 0U, 0U};

static void step_breath(struct euc_strip_state *state, uint32_t t)
{
    if ((state->standstill != STANDSTILL_BREATHING) || ((t % BREATH_STEP_TICKS) != 0U)) {
        return;
    }
    state->breath = state->breath_rising ? (uint8_t)(state->breath + 1U) : (uint8_t)(state->breath - 1U);
    if (state->breath > BREATH_TOP) {
        state->breath_rising = false;
    }
    if (state->breath == 0U) {
        state->breath_rising = true;
    }
}

/* Amber turn flash on LEDs first..first + 11: on at phase 0, dark at `off_phase`. */
static void turn_flash(struct euc_strip_state *state, uint32_t first, uint32_t t, uint32_t off_phase)
{
    const uint32_t phase = t % TURN_PERIOD;
    for (uint32_t i = 0U; i < RING_LEDS; i++) {
        if (phase == 0U) {
            set_led(state, first + i, FULL_BRIGHTNESS, amber);
        }
        if (phase == off_phase) {
            set_led(state, first + i, 0U, dark);
        }
    }
}

static void draw_comets(struct euc_strip_state *state, const struct euc_strip_render_in *in)
{
    static const uint8_t levels[6] = {1U, 2U, 4U, 16U, 40U, 130U};
    const uint32_t t = in->time_base;
    for (uint32_t i = 0U; i < RING_LEDS; i++) {
        const uint32_t position = ((ring_start(state) + i) % RING_LEDS) + 1U;
        const uint32_t level = levels[i % 6U];
        if (in->roll_deg > TURN_ROLL_DEG) {
            set_led(state, side_b_led(position), level, red);
            if ((t % TURN_PERIOD) == 0U) {
                set_led(state, position, FULL_BRIGHTNESS, amber);
            }
            if ((t % TURN_PERIOD) == TURN_OFF_POSITIVE) {
                set_led(state, position, 0U, dark);
            }
        } else if (in->roll_deg < -TURN_ROLL_DEG) {
            set_led(state, side_a_led(position), level, red);
            if ((t % TURN_PERIOD) == 0U) {
                set_led(state, position + RING_LEDS, FULL_BRIGHTNESS, amber);
            }
            if ((t % TURN_PERIOD) == TURN_OFF_NEGATIVE) {
                set_led(state, position + RING_LEDS, 0U, dark);
            }
        } else {
            set_led(state, side_b_led(position), level, red);
            set_led(state, side_a_led(position), level, red);
        }
    }
}

static void draw_pattern_0(struct euc_strip_state *state, const struct euc_strip_render_in *in)
{
    if (in->display_speed < STILL_KMH) {
        for (uint32_t led = 1U; led <= (2U * RING_LEDS); led++) {
            set_led(state, led, state->breath, red);
        }
    } else {
        draw_comets(state, in);
    }
}

static uint8_t wheel_up(uint8_t value)
{
    const uint8_t next = (uint8_t)(value + WHEEL_STEP);
    return (next > WHEEL_UP_TOP) ? (uint8_t)WHEEL_FULL : next;
}

static uint8_t wheel_down(uint8_t value)
{
    const uint8_t next = (uint8_t)(value - WHEEL_STEP);
    return (next < WHEEL_DOWN_LO) ? 0U : next;
}

/* One step of the colour wheel: red, magenta, blue, cyan, green, yellow, red, 12 steps each. */
static void wheel_step(struct euc_strip_state *state)
{
    const uint8_t g = state->wheel_green;
    const uint8_t r = state->wheel_red;
    const uint8_t b = state->wheel_blue;
    uint8_t phase = state->wheel_phase;
    if ((g == 0U) && (b == 0U)) {
        phase = 1U;
    } else if ((b == WHEEL_FULL) && (r == WHEEL_FULL)) {
        phase = 2U;
    } else if ((r == 0U) && (g == 0U)) {
        phase = 3U;
    } else if ((g == WHEEL_FULL) && (b == WHEEL_FULL)) {
        phase = 4U;
    } else if ((b == 0U) && (r == 0U)) {
        phase = 5U;
    } else if ((r == WHEEL_FULL) && (g == WHEEL_FULL)) {
        phase = 6U;
    } else {
        /* between corners: keep going */
    }
    switch (phase) {
    case 1U:
        state->wheel_blue = wheel_up(b);
        break;
    case 2U:
        state->wheel_red = wheel_down(r);
        break;
    case 3U:
        state->wheel_green = wheel_up(g);
        break;
    case 4U:
        state->wheel_blue = wheel_down(b);
        break;
    case 5U:
        state->wheel_red = wheel_up(r);
        break;
    default:
        state->wheel_green = wheel_down(g);
        break;
    }
    state->wheel_phase = phase;
}

static void draw_pattern_1(struct euc_strip_state *state, const struct euc_strip_render_in *in)
{
    const uint32_t brightness = (state->standstill == STANDSTILL_BREATHING) ? state->breath : FULL_BRIGHTNESS;
    for (uint32_t i = 0U; i < WHEEL_STEPS; i++) {
        const uint32_t position = ((i + ring_start(state)) % WHEEL_STEPS) + 1U;
        wheel_step(state);
        const struct euc_strip_pixel colour = {state->wheel_red, state->wheel_green, state->wheel_blue};
        set_led(state, side_b_led(position), brightness, colour);
        set_led(state, side_a_led(position), brightness, colour);
    }
    const bool moving = in->display_speed >= STILL_KMH;
    if (moving && (in->roll_deg < -TURN_ROLL_DEG)) {
        turn_flash(state, 1U, in->time_base, TURN_OFF_NEGATIVE);
    } else if (moving && (in->roll_deg > TURN_ROLL_DEG)) {
        turn_flash(state, RING_LEDS + 1U, in->time_base, TURN_OFF_POSITIVE);
    } else {
        /* no flash */
    }
}

static void draw_brake_flash(struct euc_strip_state *state, uint32_t t)
{
    const uint32_t phase = t % BRAKE_PERIOD;
    for (uint32_t led = 1U; led <= BRAKE_LEDS; led++) {
        if (phase == BRAKE_DARK_PHASE) {
            set_led(state, led, 0U, dark);
        }
        if (phase == 0U) {
            set_led(state, led, FULL_BRIGHTNESS, red);
        }
    }
}

void euc_strip_render(struct euc_strip_state *state, const struct euc_strip_render_in *in)
{
    if (!state->update) {
        return; /* until the first time base that is a multiple of 5; never cleared */
    }
    const uint32_t t = in->time_base;
    step_breath(state, t);
    if (state->brake_count < BRAKE_FLASH_MIN) {
        if (in->pattern == 0U) {
            draw_pattern_0(state, in);
        }
        if (in->pattern == 1U) {
            draw_pattern_1(state, in);
        }
    } else {
        draw_brake_flash(state, t);
    }
}

uint32_t euc_strip_take_changed(struct euc_strip_state *state)
{
    const uint32_t changed = state->changed;
    state->changed = 0U;
    return changed;
}
