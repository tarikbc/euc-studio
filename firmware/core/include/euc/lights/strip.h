/*
 * LED strip patterns: 32 WS2812 LEDs, of which the patterns use LEDs 1..24 (1-based, as the stock firmware
 * numbers them) as two rings of 12. Ring position p (1..12) is LED 8 9 10 11 12 1 2 3 4 5 6 7 on side A and
 * LED 24 down to 13 on side B.
 *
 * Tick part, every RTC tick before the alarm task (it reads the time base of the previous tick):
 *   ring        moves one step every 100 - 3 x (display speed) ticks up to 30 km/h, every 10 ticks above;
 *               forward above +4 mm/s (0..72, then 0), backward below -4 (down to 1, then 73)
 *   brake       every 150 ticks the display speed is noted; 3 km/h or more below the note sets the brake
 *               count to 40, else it falls by 1 every 10 ticks to 3; four or more: brake flash
 *   standstill  under 2 km/h for more than 40 x 10 ticks: breathing; 2 km/h or more: moving
 *   update      set when the time base is a multiple of 5
 *
 * Render part, every RTC tick after the alarm task, once `update` has been set:
 *   breathing   while standing: brightness steps +-1 between 0 and 131, one step every 4 ticks
 *   brake flash both patterns: LEDs 1..24 red (100) when time base % 50 == 0, dark when it is 25
 *   pattern 0   standing (display speed < 2): LEDs 1..24 red at the breathing brightness. Moving: two red
 *               comets of 6 (brightness 1 2 4 16 40 130) on both rings, from ring position + 1. Roll above
 *               +4 deg: LEDs 1..12 flash amber instead of ring A (on at time base % 150 == 0, off at 100);
 *               below -4 deg: LEDs 13..24 instead of ring B (off at 75)
 *   pattern 1   a 72-step colour wheel (G R B, 21 per step) laid on positions 1..72 from ring position + 1;
 *               positions 1..12 show on both rings, at the breathing brightness while breathing, else 100.
 *               Moving: roll below -4 deg flashes LEDs 1..12 amber (off at 75), above +4 deg LEDs 13..24
 *               (off at 100): the other side from pattern 0
 * Colours scale as colour x brightness / 255. LEDs the render does not write keep their colour.
 *
 * The stock firmware never clears `update`, so its render runs on every main-loop pass. Everything above
 * depends on the time base only and repeats harmlessly, except the breathing: it steps once per pass during
 * every 10th tick, so its speed follows the CPU load (quirk STRIP_BREATH_PER_PASS, kept by neither profile).
 * On the wheel one breath takes about 2.1 s; one step every 4 ticks gives 2.11 s in both profiles, whatever
 * the main loop does.
 */
#ifndef EUC_LIGHTS_STRIP_H
#define EUC_LIGHTS_STRIP_H

#include <stdbool.h>
#include <stdint.h>

#define EUC_STRIP_LEDS 32U

struct euc_strip_pixel {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct euc_strip_state {
    /* Tick part */
    bool update;
    int16_t ring;        /* 0..73 */
    uint8_t speed_note;  /* display speed noted every 150 ticks */
    uint8_t brake_count; /* brake flash while 4 or more */
    uint8_t still_count; /* tens of ticks under 2 km/h */
    uint8_t standstill;  /* stock values: 1 breathing, 3 moving, 0 at power-on */
    /* Render part */
    uint8_t breath; /* 0..131 */
    bool breath_rising;
    uint8_t wheel_green; /* colour wheel of pattern 1 */
    uint8_t wheel_red;
    uint8_t wheel_blue;
    uint8_t wheel_phase;                           /* 1..6 */
    struct euc_strip_pixel pixels[EUC_STRIP_LEDS]; /* index 0 = LED 1 */
    uint32_t changed; /* bit i: pixels[i] changed since euc_strip_take_changed() */
};

struct euc_strip_tick_in {
    uint16_t time_base;    /* alarm time base of the previous tick */
    uint8_t display_speed; /* the front display's speed (km/h or mph), as it was last shown */
    int16_t speed_mm_per_s;
};

struct euc_strip_render_in {
    uint16_t time_base;
    uint8_t display_speed;
    float roll_deg;
    uint8_t pattern; /* 'W' 'M': 0 or 1; anything else draws nothing */
};

void euc_strip_init(struct euc_strip_state *state);

/* Every RTC tick, before the alarm task. */
void euc_strip_tick(struct euc_strip_state *state, const struct euc_strip_tick_in *in);

/* Every RTC tick after the alarm task: draws into state->pixels once `update` has been set. */
void euc_strip_render(struct euc_strip_state *state, const struct euc_strip_render_in *in);

/* The changed-pixel mask, cleared. */
uint32_t euc_strip_take_changed(struct euc_strip_state *state);

#endif
