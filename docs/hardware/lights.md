# Tail light (Begode Extreme)

The Extreme's tail light is the translucent roller across the rear end of the seat. Inside it, a
strip of WS2812-type RGB LEDs forms two rings of 12, one in each half of the roller beside its black
centre band, shining out through an orange-red diffuser (OBSERVED in the owner's video). The stock
firmware animates it from the wheel speed, the travel direction, the side lean, sudden slowing and the LED mode the rider
picks. This page describes what it shows in each state and how the firmware drives it.
`profiles/begode-extreme/lights.toml` is the machine-readable form (LED positions, ring layout,
states); `crates/euc-display` (`lights` module) parses it.

Evidence tags as elsewhere: **OBSERVED** = the stock firmware (GW1825007) produced it in the
simulator and the code agrees, or, where it says so, the owner's video of his lit tail light;
**INFERRED** = from the code or the photos only; **ESTIMATE** = read off the video by eye;
**UNKNOWN** = not settled. The headlight (PB12, light modes `E`/`Q`/`T`) is a separate output, not described here.

## How the firmware drives the strip

| Fact | Value | Tag |
|---|---|---|
| Bus | PA6 = TIM3 CH1 PWM at 800 kHz, fed by DMA1 channel 2 in circular mode from a 1768-slot buffer at 0x2000115E: 32 LEDs x 24 bits, then 1000 low slots as the latch gap. The strip is resent every 2.21 ms. | OBSERVED |
| Bits | slot duty 20/30 = 1, 8/30 = 0; per LED green, red, blue, most significant bit first (GRB, the WS2812 order) | OBSERVED |
| LEDs used | 1-24. LEDs 25-32 are never written and stay off. | OBSERVED |
| Start | the strip starts (all off) when the 496 Hz tick starts, late in `main`; in calibration mode it never starts and stays dark | OBSERVED |
| Writer | only `led_effects_update` (0x08005A5C) sets pixels. It runs on every main-loop pass once the tick has started (its update flag is set every 5 ticks and never cleared). `led_animation_step` (0x08005EB8) runs in every 496 Hz tick and updates the animation position, the standstill timer and the braking detector. | OBSERVED |
| Inputs | the displayed speed (whole km/h, absolute, mph in miles mode, refreshed at 5 Hz), the signed speed (direction), the roll angle from the attitude filter, the LED mode, the tick counter. Light modes, alarms and faults do not reach the strip. | OBSERVED |

Colours below are the 0-255 channel values the strip receives (PWM duty, linear light).

## The two rings

LEDs 1-12 form ring A and LEDs 13-24 ring B; the data goes through ring A first. The firmware
animates both rings with one shared animation position `p` (1 to 12) and maps it to an LED per ring:

| Ring | Position `p` lights LED | Tag |
|---|---|---|
| A (LEDs 1-12) | `1 + (p + 6) mod 12`: positions 1-12 light LEDs 8, 9, 10, 11, 12, 1, 2 ... 7 | OBSERVED |
| B (LEDs 13-24) | `25 - p`: positions 1-12 light LEDs 24 down to 13 | OBSERVED |

So ring B runs the other way round in LED order (INFERRED: if the two rings are the same board
fitted facing opposite ways, this makes the animation in the two halves mirror each other). Where
round the axis each LED sits, and which way the index grows, are UNKNOWN; `lights.toml` puts each
ring's first LED at the top with the index growing clockwise, seen along the roller axis from that
ring's end.

## Where it is

The 3D scan (MEASURED, +-1 mm; frame of `geometry.toml`: x forward, y left, z up, origin on the
axle) gives the roller: a cylinder along y at x -0.187 m, z 0.449 m (0.68 m above the ground),
radius 26 mm, above and behind the shock's top mount. Its 26.1 mm sleeve reaches |y| 37.5 mm; a dark collar
(radius 27.5-28 mm) follows to 46 mm, then the speaker grille faces at |y| 47-50.5 mm, and the
mounting arms reach 59 mm.

The owner's video (7 s, wheel standing; first looking down from the seat, then from behind) shows
how it lights up (OBSERVED unless marked):

- The roller is the lamp: a clear outer sleeve over an orange-red translucent diffuser, an opaque
  black band about 15.6 mm wide round the middle (ESTIMATE, scaled to the sleeve), dark end caps.
- Both halves glow, brightest next to the band and fading towards the end caps. Through the sleeve,
  a line of 4 or 5 bright LED hot spots shows in each half, 4 to 8 mm from the band (ESTIMATE), round
  the curve of the roller about 30 degrees apart. So each ring of 12 LEDs sits inside the roller,
  around its axis, and shines radially outwards. That is 12 LEDs per half, one ring per half; the
  hot spots on the camera's side are the part of each ring in view.
- The halves stay orange-red when the LEDs are at their dimmest: the diffuser is tinted. Red light
  passes; green and blue light would show only faintly. `lights.toml` gives an estimated tint.
- Both rings breathe together, red, with a period of about 2.1 s (peaks 2.1 s apart). The red
  looks the same in every hot spot. This is the standing pattern; whether the wheel was in LED mode
  0 or 1 cannot be told, because mode 1 at a standstill also shows red-containing colours that the
  tinted diffuser makes look alike.
- A small blue light at the top of the centre band, on its right edge, stays on at a steady
  brightness while the red breathes. The owner identifies it as the status LED of the controller's
  Bluetooth (BLE) data module, which sits inside the tube: it is not on the strip, and the
  controller firmware does not drive it. Modules of this kind (HM-10 class) typically blink while
  advertising with no phone connected and stay steady while a phone is connected (INFERRED); steady
  in the video fits a connected phone. `lights.toml` lists it as the `ble` indicator.
- The two ends of the tube are speakers of a separate Bluetooth audio device (owner), not driven
  by the controller. The scan's "caps" are their round faces, 55 mm across, with a grille of radial
  windows between r 12 and r 22 mm. `lights.toml` lists them as `[[speaker]]` entries, for drawing
  only.

Which ring is on which side: in LED mode 0 the firmware blinks ring A when the roll is above +4
degrees and ring B below -4 degrees. In the simulator's frame (IMU axes = shell axes, y to the
left), +roll is a lean to the right, so ring A is taken to be in the **right** half and ring B in the
**left** half, assuming the indicator blinks on the side the wheel leans. INFERRED.

## What it shows

Priority from the top: a row overrides the ones below it.

| State | Trigger | What the rings show | Tag |
|---|---|---|---|
| Braking (any LED mode) | the displayed speed is 3 km/h or more below the sample taken at the start of the current 150-tick window (0.30 s), so about 2.8 m/s² or harder; lasts until 0.75 s after the last detection | all 24 LEDs flash red 100: 25 ticks on, 25 ticks off (50 ms / 50 ms, 9.9 Hz) | OBSERVED (hard-braking-30kmh) |
| Mode 0, lean while moving | speed 2 km/h or more and roll above +4 degrees (ring A) or below -4 degrees (ring B) | that ring blinks amber (red 100, green 78): +roll on 0.20 s / off 0.10 s, -roll on 0.15 s / off 0.15 s, every 0.30 s; the other ring keeps the comets | OBSERVED (accelerometer bias at 20 km/h) |
| Mode 0, moving | speed 2 km/h or more | two red comets per ring, 6 LEDs each, head to tail red 130, 40, 16, 4, 2, 1; they step one position every 100 - 3 x speed ticks below 31 km/h (0.19 s at 2 km/h, 81 ms at 20 km/h) and every 10 ticks (20 ms) from 31 km/h; they turn the other way when riding backwards | OBSERVED (ride-20kmh-stop, reverse ride) |
| Mode 0, standing | speed below 2 km/h | all 24 LEDs red, breathing: the brightness ramps 0 to about 130 and back | OBSERVED |
| Mode 1, moving | speed 2 km/h or more | rainbow: each ring shows 12 consecutive steps of a 72-step colour wheel (red, yellow, green, cyan, blue, magenta; one channel ramps in steps of 21) at brightness 100, the same colours on both rings in position order; the window slides one step per animation step, so a full rainbow passes every 72 steps (about 5.8 s at 20 km/h) | OBSERVED (LED mode 1 rides) |
| Mode 1, standing | speed below 2 km/h | the same 12-colour window, frozen, at the breathing brightness | OBSERVED |
| Mode 1, lean | as mode 0 | the amber is rewritten by the rainbow on the next pass, so it shows only as a flicker of about one tick every 0.30 s, and on the opposite ring to mode 0 (+roll: ring B) | OBSERVED |
| Power-on | the tick starts (about 1.7 s after power-on in the simulator) | dark, then the standing pattern once the standstill timer has run (0.83 s): first light about 2.5 s after power-on; no animation | OBSERVED |
| Calibration mode, power-off | — | dark | OBSERVED (calibration), INFERRED (power-off) |

Details:

- **Standstill timer.** After the speed falls below 2 km/h, the breathing starts after 41 steps of
  10 ticks (0.83 s); until then the brightness holds its last value. Moving again resets it.
- **Breathing speed depends on the CPU.** The breathing changes the brightness by one step per
  main-loop pass during every 10th tick, so its period depends on how many main-loop passes fit in a
  tick. On the owner's wheel it is about 2.1 s (video), so about 2.5 passes per tick; in the
  simulator it is 0.49 s, about 11 passes per tick: the simulated main loop runs about 4 times too
  often for the time the interrupts leave it.
- **Animation position.** `p` advances while the signed speed is above +4 and goes back while it is
  below -4 (firmware units), so tiny creep at a standstill can move it slowly. It wraps over 73
  values, so the comet pattern skips one step at the wrap.
- **LED mode.** `W` `M` `1` selects mode 1 and `W` `M` `0` (or any other digit) mode 0; every third
  short press of the light button toggles it. It is stored in the settings record and survives a
  power cycle. The wheel lock (transport mode) forces mode 0. The maintainer's settings preset
  (`tarik-2026-10-15`, `wm_value = 1`) boots in mode 1 (OBSERVED).
- **Miles mode.** The thresholds use the displayed speed, so in miles mode they are in mph: moving
  from 2 mph, braking from a 3 mph drop.
- **Light modes.** `E`, `Q` and `T` (headlight off, on, strobe) do not change the strip (OBSERVED).

## Simulator runs

Idle 12 s; idle with `Q`, `T`, `E`; idle with `W` `M` `1`; `ride-20kmh-stop` (mode 0; its 1.4 m/s²
stop does not trigger the brake flash); `hard-braking-30kmh` (maintainer's preset, mode 1); a ride at
20 km/h with a +0.15 g then -0.15 g lateral accelerometer bias (roll about +5 and -5 degrees) in mode
0 and in mode 1; a backwards ride at 8 km/h; the maintainer's preset at a standstill. The strip was
read from the firmware's DMA buffer every 2 to 5 ms.

## Open points

- **Which ring is left (INFERRED).** A video from behind while turning left in LED mode 0 shows which
  half blinks amber.
- **Ring layout (UNKNOWN).** A close, slow-motion video of one half while riding slowly in mode 0
  shows which way the comets go round.
- **LED mode in the video.** The same standing video after `W` `M` `0` and after `W` `M` `1` shows
  whether mode 1 looks different through the diffuser.
- **The BLE status LED.** A video of the blue light before and after a phone connects shows its
  blink pattern.
- **Brake flash on the real light.** A video from behind during a hard stop.
