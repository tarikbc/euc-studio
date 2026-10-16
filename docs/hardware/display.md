# Front display (Begode Extreme)

The Extreme's front display is a separate board, **T16-007** (dated 2023-06-06, "119.5 mm x 79 mm" on
the silkscreen), with a black LED panel of 82.6 x 55.2 mm. It shows three large 7-segment digits for
speed and, below them, five battery bars, a single small digit (the **pedal mode**), the board
temperature `88.8°C` and the odometer `88888.8`. The six round holes under the panel are cut-outs in the board,
not display elements.

`profiles/begode-extreme/display.toml` is the machine-readable spec: every element's position in
millimetres and the driver bits that light it. `crates/euc-display` parses it, decodes a set of grid
bytes into lit elements, readable text (`"000"`, `"25.3°C"`, `"01468.6"`) and outlines, so the studio
can draw an exact replica from what the emulated firmware writes.

Evidence tags as elsewhere: **OBSERVED** = the stock firmware (GW1825007, `pwm_1762161713192.bin`)
produced it in the simulator and the code agrees; **INFERRED** = from the code or the board layout only.
**Owner photo** = seen on the maintainer's own wheel, in two photos of the lit display (one parked,
one riding, taken at different times; private, not in the repo).

## How the firmware drives it

| Fact | Value | Tag |
|---|---|---|
| Driver | TM1640-style, 16 grids x 8 segment lines; the chip name is inferred from the protocol | INFERRED |
| Bus | 2 wires bit-banged on GPIOC: PC5 = CLK, PC4 = DIN, LSB first, start = DIN falls while CLK high, stop = DIN rises while CLK high | OBSERVED |
| Refresh | every 100 ticks of the 496 Hz RTC tick (4.96 Hz, 201.6 ms), inside the RTC interrupt | OBSERVED |
| Each refresh | data command `0x40` (auto-increment), control `0x8F` (display on, brightness 7 = pulse 14/16), then address `0xC0` and the 16 grid bytes from the RAM buffer at 0x20001140 | OBSERVED |
| Writers | only `display_refresh` (0x08004364) fills the buffer; nothing else writes the display (no boot animation, no alarm or error screens) | OBSERVED |
| Segment bits | a = bit 7, b = 6, c = 5, d = 4, e = 3, f = 2, g = 1, dp = 0; digits 0-9 are `FC 60 DA F2 66 B6 BE E0 FE F6` | OBSERVED |
| Brightness | constant: the firmware never dims the display | OBSERVED |

The firmware keeps a 16-byte table at 0x2000111C that assigns each value's digit to a grid address
(`TM1640_Init` 0x080090F8 and every refresh rewrite it with the same constants). Grid index `i` below
is the TM1640 address `0xC0 + i`, which is also the `i`-th byte of the simulator's `Tm1640::grid()`.

| Grid | Bits 7..1 (a..g) | Bit 0 (dp line) |
|---|---|---|
| 0 | odometer, 1000s digit | battery bar 5 |
| 1 | odometer, 10000s digit | battery bar 5 |
| 2 | temperature, tenths digit | `°C` sign, always on |
| 3 | temperature, ones digit | temperature decimal point, always on |
| 4 | temperature, tens digit | battery bar 4 |
| 5 | pedal-mode digit | battery bar 4 |
| 6, 7 | speed, ones digit (same byte on both) | battery bar 3 |
| 8, 9 | speed, tens digit (same byte on both) | battery bar 2 |
| 10, 11 | speed, hundreds digit (same byte on both) | battery bar 1 |
| 12 | odometer, tenths digit | unused |
| 13 | odometer, ones digit | odometer decimal point, always on |
| 14 | odometer, 10s digit | unused |
| 15 | odometer, 100s digit | unused |

Each big digit is wired to two grids, probably to split its long LED strings between two
drive lines; the firmware always writes the same byte to both, so a replica lights a segment when
either grid has its bit. Which LEDs each of the two grids drives inside a segment is not knowable from
the firmware. Each battery bar likewise takes the dp line of two grids, written together.

## What each value shows

All values are recomputed at every refresh from the firmware's own state.

| Element | Value | Format | Tag |
|---|---|---|---|
| Speed | the firmware's own wheel-speed estimate (the one it also sends over BLE), absolute value, converted to km/h (mph in miles mode) and truncated | 3 digits, leading zeros shown (`000` at rest, `021`), clamped to 199 | digit roles OBSERVED (simulator ride, owner photo); truncation and clamp from the code |
| Battery | 5-level gauge from the controller's own pack voltage (ADC, filtered with a time constant near 2 s), not from the BMS; fills from the left | see the next table | OBSERVED (simulator, owner photo) |
| Pedal mode | pedal mode + 1: `1` soft (`s`), `2` medium (`f`, the default), `3` hard (`h`) | 1 digit | OBSERVED (simulator); `2` in both owner photos |
| Temperature | controller board NTC in °C, filtered | `NN.N°C`, truncated, leading zero shown (`05.2`), clamped to `99.9`, `00.0` below 0 °C | OBSERVED (simulator, owner photo) |
| Odometer | **total** (life) distance, not the trip, in 0.1 km (0.1 mi in miles mode), truncated | `NNNNN.N`, leading zeros shown, clamped to `99999.9` | OBSERVED (simulator, owner photo) |

### The small digit is the pedal mode

The single small digit left of the temperature is the pedal mode plus one. `display_refresh` writes
nothing else there, and the code is identical in all three official builds (2024-11, 2025-07,
2025-11). Pedal mode is the pedal firmness that the app sets with `s` (soft), `f` (medium) and `h`
(hard); the controller keeps it in its settings record across power cycles, and drops the command
while the wheel is moving. So the digit is a setting, not a live value: `1` soft, `2` medium,
`3` hard. The candidates it is not: the light mode, the speed-alarm mode, the roll cut-off mode and
the PWM settings never reach the display. In the simulator `s` turns it to `1` and `h` to `3` within
one refresh; the owner's photos show `2` (medium) both parked and riding.

Battery levels (bounds from the code, each level checked in the simulator):

| Pack voltage | Per cell (32S) | Bars | Grids with bit 0 set |
|---|---|---|---|
| below 100.1 V | below 3.13 V | none, bar 1 blinks | 10, 11 on and off |
| 100.1 V | 3.13 V | 1 | 10, 11 |
| 109.1 V | 3.41 V | 2 | 8-11 |
| 114.0 V | 3.56 V | 3 | 6-11 |
| 121.0 V | 3.78 V | 4 | 4-11 |
| 127.0 V | 3.97 V | 5 | 0, 1, 4-11 |

The firmware compares the voltage in 0.1 V steps (truncated), so "100.1 V" means 100.10 V and above.
The maintainer's wheel at 129.7 V shows all five bars. Under hard acceleration the filtered voltage
sags, so bars can drop while riding and come back at rest (INFERRED). The bars fill from the left:
the owner's photos show the three left bars lit parked and the four left bars lit riding.

Low-battery blink: below 100.1 V the firmware lights bar 1 when the tick counter modulo 500 is at most
250, and clears it otherwise. A 500-tick period is 1.008 s, sampled every 100 ticks, so bar 1 is on
for 3 refreshes and off for 2 (or 2 and 3, depending on the boot phase): about 1 Hz, 60/40. In the
simulator at 98 V: on 0.6 s, off 0.4 s. OBSERVED.

## The display in each state of the wheel

| State | What the display shows | Tag |
|---|---|---|
| Power-on | dark until the first refresh; the 496 Hz tick starts late in `main` (after IMU, BMS and current-sensor init), so the first frame appears about 1.9 s after power-on in the simulator. No animation. | OBSERVED (simulator timing) |
| Idle, standing | `000`, battery bars, pedal digit, board temperature, total odometer | OBSERVED |
| Riding | the same layout; the speed digits follow the firmware's speed estimate at 5 Hz (it can differ from the true speed by a few tenths, so the last digit can flicker between two values at a steady speed); battery bars follow the sagging voltage | OBSERVED (simulator ride, owner photo); bar sag INFERRED |
| Miles mode (`m`; `g` back to km) | speed in mph and odometer in miles (both x 0.62); there is no unit indicator on the display. The change shows at the next refresh. | odometer OBSERVED, mph INFERRED (same code path as km/h) |
| Pedal mode change (`s`/`f`/`h`, only below the firmware's speed limit for it) | the small digit changes at the next refresh | OBSERVED |
| Low battery | bar 1 blinks, all other bars dark | OBSERVED |
| Alarms, tilt-back, cut-offs, BMS faults, lock | nothing changes on the display; these go to the buzzer and the LED strip | OBSERVED (no other writer) |
| Calibration mode (no IMU calibration in flash) | dark: the tick never starts, so the buffer is never filled and the driver gets only its init commands; a real TM1640 then shows whatever its RAM held at power-on, normally nothing | OBSERVED in the buffer, INFERRED on the panel |
| Power-off | goes dark when the power latch drops | INFERRED |

Simulator runs that produced the OBSERVED rows: power-on defaults (130 V, 25 °C, empty settings);
battery voltages 98, 100.2, 105, 109.5, 113, 116, 121.5, 126 and 128 V; board temperatures 5, 45.6,
99.95, 120 and -5 °C; the settings record seeded in flash with odometers of 1,468,670 m,
98,765,432 m and 4,000,000,000 m, each pedal mode, km and miles; BLE `s`, `h`, `m`, `g` during a run;
calibration mode; and the riding scenario `ride-20kmh-stop` with the owner's odometer (558,300 m) and
medium pedals seeded, where the display counted `001` up to `021` and back to `000`. In that run the
simulator's TM1640 model, decoding the bus, held exactly the firmware's buffer after every change,
with the display on at brightness 7. `crates/euc-display` keeps the resulting grid bytes as test data.

## Owner photos

| Photo | Speed | Bars | Pedal digit | Temperature | Odometer |
|---|---|---|---|---|---|
| Parked | `000` | 3 left bars lit (114.0 to 121.0 V) | `2` | `24.9°C` | `00558.3` |
| Riding, photographed upside down | `021` (21 km/h; it looks like `120` as photographed) | 4 left bars lit (121.0 to 127.0 V) | `2` | about `3x.9`, too blurred to read every digit | about `0038x.5`, too blurred to read every digit |

Read segment by segment after turning the riding photo 180 degrees: the hundreds digit is `0`
(a b c d e f), the tens `2` (a b g e d) and the ones `1` (b c, on the right side of its cell), the
firmware's patterns for `0`, `2` and `1`. Both photos match the layout, the segment shapes and the
`°C` glyph of the spec. They were taken at different times: the riding photo's odometer is lower.

The idle BLE capture of 2026-10-15 reports an odometer of 1,468,670 m and hard pedals, which the
display would show as `01468.6` and `3`. There is no conflict: the owner confirms that the photos
are older than the capture. Since then the odometer has grown and he has changed the pedals to
hard.

## Look

From the parked photo (a phone camera at night, so the colours are approximate): lit segments are
cool white (`#E2E8FC`); unlit segments stay faintly visible as grey ghosts (`#3B3F57`) behind the
smoked window; the panel around them is near black (`#0A0B1A`). A replica draws every segment, lit or
ghost, and never hides the unlit ones.

## Mount

The panel sits on top of the shell, facing up, beside the handle, behind a smoked window. A rider
facing forward reads it upright: the top of the speed digits points to the front and the battery
bars are on the rider's left. The owner confirmed this. His riding photo shows the panel rotated
180 degrees only because he rode the wheel backwards for it (a red light, taken to be the tail
light, is ahead of the camera). `display.toml` `[mount]` records this as OBSERVED
(`board_up = "front"`, `board_right = "right"`, `rider_view_deg = 0`).

## Geometry

Positions come from a shop photo of a bare board, measured at 6.17 px/mm. Two independent references
agree on that scale: the 79 mm board height (488.5 px) and the 2.0 mm pitch of the four 2 x 4 pin
headers (about 12.3 px). Ellipses fitted to the four round cut-outs have an aspect of 1.01, so the
photo is not stretched.
Inside the panel the positions are good to about 0.2 mm; the board outline to about 1 mm. The full
outline with both side tabs measures 127.2 x 79.2 mm in the photo, so the printed 119.5 mm leaves one
tab out.

| Element | Size | Notes |
|---|---|---|
| Big digits | 20.1 x 39.9 mm box, 4.6 mm stroke, pitch 29.8 mm | upright (no slant), 45-degree miters, pointed middle ends, outer corners cut flat 1.46 mm from the box edge |
| Small digits | 3.7 x 7.9 mm box, 0.85 mm stroke | same shape as the big digits; odometer pitch 4.66 mm |
| Battery bars | 2.2 x 7.6 mm, pitch 4.5 mm | left edge aligned with the hundreds digit |
| Decimal points | 0.8 mm round | at the baseline, right of the ones digit |
| `°C` | about 2 x 3.5 mm | an oval ring and an open C, about 0.3 mm stroke |

The 7-segment outline model (`Seg7Style::polygon`) reproduces the measured corners of the big digits
to within about 1 px of the photo (0.2 mm), and the small digits to within the photo's resolution.

## Open points

- **Pedal digit on the real wheel.** Switching the pedal mode in the app and watching the digit
  (`s` 1, `f` 2, `h` 3) would confirm the firmware mapping on the wheel itself. The photos (`2`,
  medium, from before the owner changed to hard) already agree with it.
- **`°C` and the decimal points.** The firmware always sets these bits, and the photos show them lit;
  which LED each bit drives is read from the layout (the dp line of the digit just left of each).
  Both are always on, so this does not change what the panel shows.
