# ride-log

Compares a real ride log with a simulated run, per speed band (issue #35).
It makes the motor and rolling-resistance calibration repeatable: select steady stretches, take out acceleration and
road slope, and compare the firmware's PWM and the battery current band by band.

```sh
python3 tools/ride-log/compare.py ~/rides/log.csv                       # the log alone

# against a simulated run: steady cruises on a level road, exported to CSV
cargo run --release -p euc-cli -- sim run --scenario tools/ride-log/cruise-flat.toml --firmware stock --record out/cruise.eucrec
cargo run --release -p euc-cli -- rec export out/cruise.eucrec --csv out/run
python3 tools/ride-log/compare.py ~/rides/log.csv --mass 100 --sim out/run --csv out/bands.csv --svg out/bands.svg
```

**Never commit a ride log.** It holds GPS positions. Keep it outside the repository; only derived numbers (a speed, a
current, a ratio) belong in a pull request. The same holds for `--mass` when it tells someone's weight.

## Input

- **Log:** the CSV of a ride-logging app (EUC Remote and DarknessBot write the same columns). Needed: `Date`
  (`dd.MM.yyyy HH:mm:ss.SSS`; a `T` separator, no milliseconds and ISO dates also work), `Speed`, `Voltage`, `PWM`,
  `Current`. Used when present: `GPS speed`, `Altitude`. Other columns are ignored.
- **Run (`--sim DIR`):** a recording exported with `euc rec export run.eucrec --csv DIR` ([Recordings](../../docs/recordings.md)).
  It reads `motion.csv` (`speed_kmh`, taken as the true ground speed), `drive.csv` (`pwm_pct`) and `battery.csv`
  (`battery_v`, `battery_a`). Ride it on a level, ideal road: the run's slope is taken as 0.

## What it does

1. Drops the rows that repeat the previous row (the apps resend the last frame; 15 % of the rows in a 30-minute test log).
2. Measures the GPS delay against the wheel's speed (0-5 s; about 1.5-2 s in that log) and the true / shown speed ratio from
   GPS (`--speed-scale` sets it by hand; the Begode Extreme shows about 1.105 × the true speed). GPS speed 0 while
   the wheel moves, or a ratio outside 0.5-1.5, stops the tool with an error.
3. Keeps non-overlapping windows (`--window`, 2 s) whose speed range is at most `--span` (2 km/h) above
   `--min-speed` (15 km/h true). A window with a hole of more than 1 s in the log, or whose samples cover less than
   60 % of its length, is dropped.
4. With `--mass` (rider + wheel + gear): the battery power minus `m·(a + g·slope)·v`, the power of a flat, steady
   cruise. The slope comes from the GPS altitude change over ± 10 s; slopes above 15 % are dropped as GPS noise.
5. Prints one row per `--band` (5 km/h of true speed): medians of PWM, voltage, current and flat power, and with
   `--sim` the run's values and the difference (run − log). `--csv` and `--svg` write the table and a plot.

`cruise-flat.toml` is the simulated run: steady cruises at 22, 32, 42, 52 and 57 km/h (true), 20 s each, on an
ideal level road, with the `tiltback-69` settings preset (the factory 20 km/h tiltback stops the rider near
30 km/h). It lives here, not in `scenarios/`, so CI does not run it. Ride it with `--firmware stock`
when the log comes from the stock firmware. It gives about 10 steady windows per cruise.

## Limits

- Steady windows from a 4 Hz log are few (about 50 in a 30-minute city ride), so each band holds only a few windows.
- The GPS slope over ± 10 s is noisy. Treat the flat power as a band median, not as one window's value.
- A row equal to the one before is taken as a resend and dropped, even if it was a real sample.
- Times are the phone's local time: do not use a ride across a daylight-saving change.
- The battery current in the log is the wheel's own reading; see [ride-log analysis](../../docs/models/ride-log-analysis.md)
  for evidence that it reads low at high power.

```sh
python3 -m unittest discover -s tools/ride-log -p 'test_*.py'
```
