# Begode / Gotway BLE protocol (Begode Extreme)

This page describes the serial protocol between a Begode (Gotway) wheel and a phone app, as used by the main
board of the **Begode Extreme**. It was written from our own study of the official firmware and from a real
capture of the wheel (`tests/data/begode-extreme/idle-2026-10-15.hex`). The firmware in `firmware/` implements
it (`core/protocol/telemetry.c`, `core/protocol/commands.c`) and `crates/euc-proto-begode` decodes it.

Other Begode models share the frame format; field meanings can differ between models and builds. Everything
here is checked against the Extreme build that answers `V` with `GW1825020`.

How sure we are, per fact:

| Tag | Meaning |
|---|---|
| **confirmed** | seen in the firmware and, where it shows, in the capture |
| **inferred** | follows from what the firmware does, but not measured |
| **unknown** | not known yet; listed under [Open questions](#open-questions) |

Facts without a tag are confirmed.

> **Safety.** Three two-byte sequences stop the motor and change the wheel: `+` `-`, `c` `y` and `!` `@`
> (see [Safety classes](#safety-classes)). An app must never send them, and a rider must never be on the wheel
> when they are sent. EUC Studio's BLE layer refuses to send these bytes at all.

## Link

| Item | Value |
|---|---|
| BLE service / characteristic | `FFE0` / `FFE1` (one characteristic: write without response for commands, notify for telemetry) |
| BLE module | transparent UART bridge; the firmware never sends it AT commands |
| Wheel-side UART | USART1, 115200 baud, 8N1, no flow control, PA9 (TX) / PA10 (RX) |
| Notifications | the module splits each 24-byte frame into 20 + 4 bytes; apps must reassemble a byte stream |
| Direction | phone → wheel: single ASCII bytes; wheel → phone: frames and some plain text |

Inbound bytes arrive one at a time in an interrupt with no FIFO or DMA. While the 496 Hz control interrupt runs,
bytes that arrive back to back can be lost (**inferred**). Send one byte per BLE write.

## Time base

The wheel counts time in **ticks** of its real-time-clock interrupt: 8 MHz crystal / 128 / 126 = 496.03 Hz, so
one tick is **2.016 ms** (**inferred** from the clock set-up; it assumes the 8 MHz crystal). All gaps and
periods on this page are in ticks first, then milliseconds.

## Frames

Every frame is 24 bytes:

| Byte | Content |
|---|---|
| 0-1 | header `55 AA` |
| 2-17 | eight 16-bit fields, big-endian |
| 18 | frame type |
| 19 | sub-index: page 0..3 for types `01`, `02`, `03`; constant `18` hex for the others |
| 20-23 | footer `5A 5A 5A 5A` |

There is no checksum. Signed fields are two's complement.

### Frame group timing

Every **151 ticks (304 ms)** the wheel queues one group of five frames, always in this order:

`00`, `01`(n), `02`(n) or `03`(n), `04`, `07`

- n counts 0, 1, 2, 3 and wraps. Frame `01` carries BMS block n. The cell frame carries page n.
- Each time n wraps, the cell frame switches pack: four groups of `02` (pack A), then four of `03` (pack B).
- The frames are sent one per main-loop pass, so the measured period is a little longer: 309.6 ms mean over the
  76 groups of the capture.
- Text (see [Text replies](#text-replies)) is written to the same UART and can land between frame bytes.
  Decoders must resynchronise on `55 AA` and check the footer.

### Units mode

After the rider picks miles (`m`), the wheel itself multiplies speed, trip, odometer and range by 0.62 before it
fills the frames. Frame `04` byte 7 bit 0 tells the app which unit is in use.

### Frame 00: live values (sub-index `18`)

| Bytes | Field | Type | Scale and unit | Notes |
|---|---|---|---|---|
| 2-3 | battery voltage | u16 | 0.02 V per count (value × 2 / 100 = V) | controller ADC, not the BMS; **clamped at 134.40 V** |
| 4-5 | speed | s16 | 0.01 m/s (value × 3.6 / 100 = km/h) | sign = direction (**inferred**); miles-scaled |
| 6-7 | estimated range | u16 | km (or mi) | piecewise linear in voltage (below); meaning **inferred** |
| 8-9 | trip distance | u16 | m (or mi-scaled) | wraps at 65.535 km; restored after a software reset |
| 10-11 | phase current | s16 | 0.01 A (**inferred**) | motor phase current from the current loop |
| 12-13 | board temperature | s16 | MPU6050 format | **the board NTC, not the IMU**: apps that read raw / 340 + 36.53 °C get the NTC within about 1 °C; exact inverse: °C = (raw + 5280) / 334 + 21 |
| 14-15 | settings echo | u16 | bit field | bits 10-13 turn compensation (`W R`), bits 5-8 pedal trim (`W U`), bit 3 PWM tiltback (`e`/`x`), bit 0 drive mode; other bits 0 |
| 16-17 | beeper volume | u16 | 0..9 | `W B` setting |

Range estimate, from the voltage V in volts (0.01 V steps): above 128.00 V: 115 + (V − 128) × 1.2;
106.01..127.99 V: 5 + (V − 106) × 5; 96.01..106.00 V: (V − 96) × 5; otherwise 0, exactly 128.00 V included.
Then the miles factor. At 129.70 V this gives 117, as in the capture. The jump from 50 to 5 at 106 V looks like
a vendor mistake.

Drive mode (bit 0 of the echo): 0 = voltage-mode FOC at 10 kHz PWM, 1 = current-mode FOC at 12 kHz.

Older app specs read bytes 14-15 as PWM and bytes 12-13 as the IMU temperature. Both readings are wrong for
this firmware.

### Frame 01: BMS block n (sub-index = n, 0..3)

| Bytes | Field | Type | Scale and unit | Notes |
|---|---|---|---|---|
| 2-3 | PWM alarm threshold | u16 | % | `W P` setting, 50..90 |
| 4-5 | speed copy | s16 | 0.1 m/s | a speed value latched while the motor command exceeds a limit; meaning **unknown** |
| 6-7 | battery voltage | u16 | 0.1 V | same controller ADC as frame `00`, unclamped |
| 8-9 | block n current | s16 | 0.1 A (**inferred**) | from the BMS reply |
| 10-11 | highest BMS temperature | s16 | °C | over all 12 BMS sensors |
| 12-13 | lowest BMS temperature | s16 | °C | over all 12 BMS sensors |
| 14-15 | block n word 42 | u16 | raw | BMS reply bytes 42-43; meaning **unknown** (651 in the capture) |
| 16-17 | block n status word | u16 | bit field | `7D01` hex = healthy; see [BMS link](#bms-link) |

### Frames 02 and 03: cell voltages (sub-index = page n, 0..3)

Type `02` = pack A (BMS blocks 0 and 1), type `03` = pack B (blocks 2 and 3). Each pack has 32 cells.

| Bytes | Field | Type | Unit |
|---|---|---|---|
| 2-17 | cells 8n+1 .. 8n+8 of the pack | 8 × u16 | mV |

### Frame 04: status (sub-index `18`)

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 2-5 | odometer | u32 | m (or mi-scaled) |
| 6-7 | modes | u16 | bits 13-14 pedal mode (0 soft, 1 medium, 2 hard); bits 10-11 speed-alarm mode (0..3); bits 7-8 roll cut-off mode (0..2); bit 0 miles; other bits 0 |
| 8-9 | auto power-off countdown | u16 | s; 7200 minus the idle time; motion resets the idle time (**inferred**) |
| 10-11 | tiltback speed | u16 | km/h; 200 = off |
| 12 | 0 | u8 | |
| 13 | `W M` value | u8 | 0 or 1; meaning **unknown** |
| 14 | alert flags | u8 | below |
| 15 | light state | u8 | 0 off, 1 on, 2 strobe |
| 16-17 | constant `0012` hex | u16 | **unknown** (model or protocol id?) |

Alert flags (byte 14):

| Bit | Meaning |
|---|---|
| 0 | PWM above about 0.95 × the `W P` threshold (also forces the beeper to full volume) |
| 1 | current-sensor offset fault at power-on; motor disabled (**inferred**) |
| 2 | IMU did not start; motor disabled |
| 3 | battery below 105.60 V |
| 4 | battery at or above 137.93 V |
| 5 | board temperature above about 80 °C |
| 6 | hall sensor fault (hall state 0 or 7 persists) |
| 7 | transport mode (five button presses at standstill) |

Older app specs call bits 1 and 2 speed alarms. In this firmware they are hardware faults.

### Frame 07: power (sub-index `18`)

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 2-3 | battery current | s16 | filtered DC bus current × 1.2; likely 0.01 A (**inferred**) |
| 4-5 | settings echo | u16 | bits 4-7 `W C` value, bits 0-3 `W X` value |
| 6-7 | 0 | u16 | older specs say motor temperature: this controller never sends one |
| 8-9 | PWM | s16 | whole percent, sign-extended from 8 bits (not hundredths); 100 % = full modulation |
| 10-17 | 0 | | |

### Capture check

All 380 frames of the capture (76 groups) decode with this map. The wheel stood still:
129.70 V, speed 0, range 117 km, trip 6 m, phase current 0.5..1.9 A, board 25.3 °C, turn compensation 5,
pedal trim 5, PWM tiltback on, drive mode 1, volume 9, PWM alarm 80 %, BMS current −0.1..0.0 A,
BMS temperatures 23 / 22 °C, all blocks `7D01`, cells 4.070..4.076 V, odometer 1,468,670 m, pedal hard,
speed-alarm mode 2, roll mode 0, km, auto power-off 7079 → 7056 s, tiltback 69 km/h, `W M` 1, no alerts,
lights off, PWM −1..0 %. The firmware test `tests/unit/test_telemetry.c` decodes every frame and re-encodes it
byte for byte.

## Commands

The phone writes single ASCII bytes. The wheel handles each byte in three places:

1. **At once**, in the receive interrupt: the `W` parser, the two-byte service sequences, the telemetry pause and
   a few direct actions (lights, roll mode, units, PWM tiltback).
2. **On the next tick** (within 2 ms): commands that the receive interrupt leaves in the **command slot**
   (`b`, `"`, `h` `f` `s`, `o` `u` `i` `I`).
3. **At the next telemetry group time** that is not paused (0.6..0.9 s later): the text requests in the slot
   (`V`, `N`, `k`).

### The command slot

The slot holds **one byte, and every received byte overwrites it**, whatever that byte is. A command that is
still waiting is lost when anything else arrives: `V`, `N` and `k` wait for a telemetry group time, so any byte
sent in the next 0.9 s replaces them. Send them alone and wait at least 1 s.

### Telemetry pause

Every received byte pauses telemetry: the next two group times send nothing, and the third sends the group as
usual, or the `V` / `N` / `k` reply instead of it. So each byte costs two frame groups (about 0.6 s), and an app
that writes more often than every 0.6 s stops telemetry completely.

### Speed guards

The guards use the wheel speed in 0.01 m/s:

| Guard | Condition |
|---|---|
| settings | \|v\| ≤ 0.49 m/s (below 1.8 km/h) |
| speed-alarm mode | settings guard, and the wheel has stood still for 101 ticks (0.2 s) |
| service sequences | \|v\| ≤ 0.09 m/s on both bytes |

A pending command that waits for its guard stays in the slot (`"` while moving; the speed-alarm commands until
the wheel has stood still long enough) (**inferred** for the waiting).

### Single-byte commands

| Byte | Effect | Guard | When | Reply |
|---|---|---|---|---|
| `b` | horn: two beeps, the second about 0.2 s after the first | none | tick | |
| `Q` | lights on | none | at once | |
| `E` | lights off | none | at once | |
| `T` | lights strobe | none | at once | |
| `h` / `f` / `s` | pedal mode hard / medium / soft (2 / 1 / 0) | settings; dropped while moving | tick | |
| `o` | speed alarms: all on (mode 0) | speed-alarm | tick | text: "speed alarm on" |
| `u` | speed alarms: level 1 off (mode 1) | speed-alarm | tick | text: "level 1 speed alarm off" |
| `i` | speed alarms: level 2 off (mode 2) | speed-alarm | tick | text: "level 2 speed alarm off" |
| `I` | speed alarms: level 3 off (mode 3) — hidden | speed-alarm | tick | text: "level 3 speed alarm off" |
| `o` `u` `i` while moving | no change | | tick | text: "stop the wheel first" (`I`: no reply) |
| `>` / `=` / `<` | roll cut-off angle 50° / 58° / 66° (mode 0 / 1 / 2) | none | at once | |
| `g` / `m` | units km / miles | none | at once | |
| `e` / `x` | PWM tiltback on / off — hidden | none | at once | |
| `"` | tiltback off (speed 200) | settings; waits while moving | tick | |
| `V` | firmware banner | | group time | `GW1825020` (9 bytes, no line end) |
| `N` | model name | | group time | `NAME:EXTREME` CR LF |
| `k` | every second `k` asks for a debug dump — hidden | | group time | IMU and angle text (below) |
| `p` | nothing except cancelling service sequences | | at once | |
| any other byte | nothing except the telemetry pause | | | |

Speed-alarm levels: level 1 beeps above about 8.3 m/s (30 km/h, two short beeps per second), level 2 above
about 12.5 m/s (45 km/h, three per second); the speed units of these thresholds are **inferred**. Mode 0 sounds both, mode 1 only level 2, modes 2 and 3 none.
The alarms never change the motor power.

PWM tiltback (`e`): when the motor command exceeds about 0.95 × the `W P` threshold, the pedals tilt back slowly
(clamped at 12°).

### `W` sequences

`W`, then a letter, then one or two digits. Hidden ones are not in the published app specs.

| Bytes | Setting | Range | Guard | Notes |
|---|---|---|---|---|
| `W` `Y` d d | tiltback speed, km/h | 00..99, no clamp | settings, on every byte | applies on the second digit; `b` after it is only a beep |
| `W` `P` d d | PWM alarm threshold, % — hidden | clamped to 50..90 | settings | |
| `W` `B` d | beeper volume | 0..9 | none | 0 gives full volume (a vendor quirk) |
| `W` `M` d | unknown setting | 0 or 1; other digits store 0 | none | also toggled by every third light-button press |
| `W` `R` d | turn compensation gain — hidden | 0..9 | none | removes centripetal acceleration from the tilt estimate; default 3 |
| `W` `U` d | pedal angle trim — hidden | 0..9 | none | (d − 4) × 1.2 slewed into the balance angle (**inferred**) |
| `W` `C` d | stored, saved and echoed in frame `07` — hidden | 0..9 | none | no effect in this build; beeps |
| `W` `X` d | angle limit 40 + 5·d degrees — hidden | 0..9 | none | beeps (**inferred** meaning) |

How the stock parser reads them (**inferred** in detail):

- Every `W` arms all eight sub-commands at once. A sub-command stays armed until **it** completes, so a letter
  sent much later, without a new `W`, can still complete it.
- While a `W` `Y` entry has its first digit, the single-digit sub-commands ignore digits.
- `W` `Y` and `W` `P` ignore every byte while the wheel moves.

Settings commands change RAM only. The wheel writes its settings to flash only when it powers itself off
(button, auto power-off, or `+` `-`). If power is cut another way (BMS cut-off, battery unplugged), changes
since power-on are lost.

### Service sequences

| Bytes | Minimum gap | Effect |
|---|---|---|
| `!` then `@` | 100 ticks (202 ms) | motor off at once (soft ramp-down, up to about 1 s); then the firmware erases one flash page and resets. The bootloader then waits for a new image over the same UART (**inferred**; YMODEM details **unknown**). No reply. |
| `c` then `y` | 50 ticks (101 ms) | motor off at once; after the ramp-down, erases the IMU calibration marker. The IMU is recalibrated at the next power-on. No reset, no reply. |
| `+` then `-` | 50 ticks (101 ms) | hidden: motor off, toggles the drive mode (voltage FOC 10 kHz ↔ current FOC 12 kHz), saves settings, **powers the wheel off**. |

Rules:

- The second byte counts only if it comes **after** the gap and **no other byte** came in between. The gap is a
  minimum: there is no upper limit.
- A second byte that comes too early cancels the sequence. So does any other byte, and the first byte sent
  while the wheel moves faster than 0.09 m/s.
- Sending the first byte again restarts the gap.

### Safety classes

| Class | Bytes | Rule for apps |
|---|---|---|
| read | `V`, `N` | safe; send alone, wait 1 s for the reply |
| comfort | `b`, `Q`, `E`, `T`, `g`, `m`, `W` `B`, `W` `M` | safe while riding |
| ride setting | `h` `f` `s`, `o` `u` `i` `I`, `<` `=` `>`, `e` `x`, `"`, `W` `Y`, `W` `P`, `W` `R`, `W` `U`, `W` `C`, `W` `X` | change how the wheel behaves; send at standstill only, and confirm with the frames |
| debug | `k` | hidden; prints text that interrupts telemetry; not for apps |
| **never send** | `+` `-`, `c` `y`, `!` `@` | stop the motor; `+` `-` also powers off; `!` `@` leaves the wheel in its bootloader. Only a dedicated, gated tool (EUC Studio's flasher) may send `!` `@`, never with a rider on board |

App guidance: one byte per BLE write, at least 20 ms apart; at least 200 ms around service bytes; expect a
0.6 s telemetry gap after every byte.

### Text replies

All text goes to the same UART as the frames, without framing.

| Trigger | Text |
|---|---|
| `V` | the firmware banner, 9 ASCII bytes, no line end (`GW1825020` on this build; Begode's update server lists the same build under another type name) |
| `N` | `NAME:EXTREME` CR LF |
| `o`, `u`, `i`, `I` accepted; `o`, `u`, `i` refused | CR LF, a short Chinese message in GBK encoding ("speed alarm on", "level n speed alarm off", "this needs the wheel stopped"), five dots, CR LF |
| second `k` | IMU and angle dump: labelled decimal fields `AZ`, `AY`, `AX`, `GZ`, `GY`, `GX`, `Warn_Speed`, `L9`, `Pitch`, `Rool` (sic) |
| power-on | a GBK message saying whether the IMU started, then one with the boot count |

## BMS link

The main board polls the battery management boards over a second UART. Apps never see this link directly; frame
`01` and the cell frames carry what it reads.

| Item | Value |
|---|---|
| UART | USART3, 9600 baud, 8N1, PB10 (TX) / PB11 (RX); replies arrive by DMA |
| Poll | every 1500 ticks (3.03 s) the board sends the 5 bytes `1C 03 40 31 FE` (meaning **unknown**) |
| Reply | 176 bytes = 4 blocks × 44 bytes, big-endian; parsed when all 176 bytes are in, or 500 ticks (1 s) after the request, once per request |
| Checks | none: no header, length or checksum check; a short reply parses stale bytes (**inferred**) |

Block layout (block k = bytes 44k .. 44k+43 of the reply). Blocks 0 and 1 form pack A, blocks 2 and 3 pack B:

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0-5 | three temperatures | 3 × s16 | °C (**inferred**) |
| 6-37 | 16 cell voltages | 16 × u16 | mV |
| 38-39 | status word | u16 | bit field, copied to frame `01` bytes 16-17 |
| 40-41 | current | s16 | raw; × 90 / 32.769 / 100 gives the frame `01` value in 0.1 A (**inferred**) |
| 42-43 | unknown | u16 | copied to frame `01` bytes 14-15 |

Health check per block, with status bytes s_hi (offset 38) and s_lo (offset 39). A block fails when
(s_hi >> 4) & 3 = 0, or (s_hi >> 2) & 3 ≠ 3, or (s_lo >> 4) & 7 ≠ 0. Three failed polls in a row on any block
latch a BMS fault until power-off: a 1 s beep every 10 s. The fault is in no frame. No control code uses BMS
data; the battery voltage in the frames comes from the controller's own ADC (**inferred**).

## Our firmware

`firmware/core/protocol/` implements this page. The **compat** profile reproduces the behaviours above, so our
image can run in lock-step with the stock one. The **clean** profile keeps every byte on the wire the same and
changes only these behaviours (`firmware/docs/quirks.md`):

| Quirk | Stock (compat) | Clean |
|---|---|---|
| `CMD_LAST_BYTE_WINS` | one command slot; later bytes erase waiting commands | tick commands queue up (8 bytes), `V` / `N` / `k` are flags; nothing is erased |
| `TLM_HOLDOFF` | each byte pauses telemetry for two groups; a text reply replaces a group | no pause; the reply goes out before the group |
| `W_ARM_STICKY` | `W` arms every sub-command until each completes | strict: `W`, one letter, its digits; any other byte ends the sequence |
| `DEFERRED_TILTBACK_OFF` | `"` while moving waits and applies when the wheel slows | `"` while moving is dropped |

In clean, a speed-alarm command that arrives before the wheel has stood still for 0.2 s gets the "stop first"
reply instead of waiting.

**Debug text.** Our firmware sends no debug text in either profile: no power-on messages, no Chinese
speed-alarm messages, no `k` dump (quirk `DEBUG_PRINTF`, compat 0). The parser still reports these events to the
rest of the firmware (for example to beep), but they put no bytes on the UART. The only text our firmware sends is
the `V` and `N` identity replies. When our image runs in lock-step with the stock one, the comparator checks the
frames and the `V` / `N` replies byte for byte, and marks any other stock text as "debug text, ignored".

## Open questions

1. Meaning of `W M`, frame `01` bytes 4-5 and 14-15, frame `04` bytes 16-17 (`0012`), and BMS reply bytes 42-43.
2. Units and signs of speed, phase current and battery current (scales above are inferred from the code).
3. Meaning of the BMS request `1C 03 40 31 FE`, and whether one board answers for all four blocks.
4. Bootloader: how it reads the erased page after `!` `@`, its YMODEM speed and framing.
5. Exact stock behaviour of the parser in corner cases (bytes between `W` letters and digits, speed-alarm
   commands before 0.2 s of standstill). Lock-step runs against the stock firmware will settle these.
6. How often bytes are lost when they arrive during the control interrupt.
