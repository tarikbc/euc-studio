# Comparing firmware (A/B)

`euc sim compare` rides one scenario twice, with two firmware images, on the same inputs: the same
wheel, plant, rider, settings preset, BLE commands, pushes and faults. Both runs are deterministic, so
every difference comes from the firmware. It reports where the runs first part ways and how far apart
they get.

```sh
cargo run --release -p euc-cli --features unicorn -- sim compare \
    --scenario scenarios/ride-20kmh-stop.toml --a stock --b compat \
    [--settings neutral] [--tolerance scenarios/compare/compat-vs-stock.toml] [--out report.json]
```

`--a` and `--b` take `stock` (the pinned official build, fetched if needed), `compat` or `clean` (our
builds in `firmware/build/target-*/`, found in the current folder or above it), or a `.bin` file.
`--settings` overrides the scenario's preset for both runs. `--json` prints the report as JSON; `--out`
writes the same JSON to a file and still prints the table.

## What is compared

- **Telemetry channels**: the 17 channels of `euc-telemetry` (speed, wheel speed, phase currents, q
  current, torque, battery, charge, pitch, roll, temperatures, distance), every sample (100 per
  simulated second by default). Samples of the two runs sit at the same simulated instants within a
  few cycles (a sample ends at the first quantum boundary after its period; at most 1 ms is accepted).
- **Firmware variables** by name. The stock build's names come from `profiles/<wheel>/stockmap.toml`;
  our builds' names come from `profiles/<wheel>/probes.toml` (ELF symbol and offset, resolved against the
  `.elf` next to the `.bin`), using the stock map's name for the same quantity. A name only one run has
  is listed as unpaired.
- **The BLE stream**: telemetry frames one by one (the first frame that differs, when it was sent, and
  the field of its first differing byte, as named in [protocol/begode.md](protocol/begode.md)), and the
  text between frames (boot messages, command replies).

For each channel and variable: the largest |A − B| and when, the RMS of A − B, and the first time the
difference exceeds its tolerance (or is not zero, without one). One side `NaN` and the other a number
counts as an infinite difference. The **first divergence** is the earliest of all of these: a channel,
a variable or a frame.

## Tolerance file

```toml
# Allowed differences; anything listed and exceeded fails the comparison (exit code 1).
same_checks = true           # every scenario check both runs judge must give the same verdict
ble_identical = false        # true: the two runs must send the same telemetry frames
until_s = 30.0               # optional: compare only before this simulated second

[channels]                   # largest |A - B| of telemetry channels, in the channel's unit
speed_kmh = 0.3
pitch_deg = 1.0
default = 10.0               # optional: every channel not listed

[channels_rms]               # RMS of A - B, for signals with sharp transients (currents, torque)
torque_nm = 3.0

[vars]                       # largest |A - B| of firmware variables, by name
balance_disabled = 0

[scenario.hard-braking-30kmh]          # adjustments for one scenario
note = "why this scenario needs them"
until_s = 20.0
[scenario.hard-braking-30kmh.channels_rms]
torque_nm = 9.0
```

Names not listed are reported but not judged, unless the section has a `default`. A scenario's
adjustments replace the names they list and keep the rest. `until_s` exists for rides that end in a
fall: after it the two runs part ways for good, and only the checks (when the wheel fell, which
warnings came first) still compare.

`scenarios/compare/compat-vs-stock.toml` is the tolerance CI uses (`.github/workflows/compare.yml`):
our compat profile against the stock firmware on every scenario in `scenarios/`.

Checks that read a firmware variable (`firmware_bits`, `record` of a `var`, ...) need its name in the
run's map. A run whose map lacks it skips those checks; the report lists them per run
(`checks_skipped`), and they do not count for `same_checks`.

## JSON report

One object (also what `--out` writes):

| Field | Meaning |
|---|---|
| `scenario`, `settings` | the scenario name and the settings preset (null: blank) |
| `a`, `b` | each run: `spec` (`stock`, `compat`, a path), `label`, `sha256`, `passed` (the scenario's own checks), `checks_skipped`, and the scenario outcome (`checks`, `top_speed_kmh`, `max_abs_pitch_deg`, `distance_m`) |
| `tolerance_file`, `tolerance_note` | the file used, or null; the note of its adjustments for this scenario |
| `check_disagreements` | checks both runs judge with different verdicts, as text |
| `ok` | the verdict: `within_tolerance`, and no `check_disagreements` when the file asks for `same_checks` |
| `wall_seconds` | time for both runs |
| `samples`, `duration_s`, `sample_skew_us`, `compared_until_s` | samples compared, simulated seconds, largest sample-time difference, the window's end (null: the whole run) |
| `first_divergence` | null, or `{t_s, kind: "channel" \| "var" \| "ble", name, a, b}` (values as text) |
| `channels`, `vars` | one entry per series: `name`, `unit`, `max_abs`, `rms`, `at_max_s`, `a_at_max`, `b_at_max`, `tolerance` and `rms_tolerance` (null: not judged), `first_s` (null: never), `within` |
| `vars_unpaired` | variable names only one run has |
| `ble` | `bytes_a`, `bytes_b`, `frames_a`, `frames_b`, `frames_identical`, `first_frame` (null, or `{index, t_a, t_b, field, a, b}` with the frames as hex), `max_frame_skew_us`, `text_bytes_a`, `text_bytes_b`, `text_identical` |
| `within_tolerance` | every judged series within its tolerance (and the frames identical when `ble_identical`) |

Infinite differences serialize as `null` (JSON has no infinity). The command exits 0 when `ok` is
true, 1 otherwise.
