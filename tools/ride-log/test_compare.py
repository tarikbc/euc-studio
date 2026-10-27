"""Tests for compare.py on synthetic logs and runs (no real ride data)."""

from __future__ import annotations

import contextlib
import csv
import io
import tempfile
import unittest
from datetime import datetime, timedelta
from itertools import accumulate
from pathlib import Path

import compare

SCALE = 0.9  # true / shown speed of the synthetic wheel
LAG = 1.0  # s, GPS behind the wheel
HZ = 4


def shown_speed(t: float) -> float:
    """Cruises at 20, 40 and 60 km/h (shown), 30 s each, with 10 s ramps between."""
    for start, kmh in ((0, 20), (40, 40), (80, 60)):
        if start <= t < start + 30:
            return kmh
        if start + 30 <= t < start + 40:
            return kmh + (t - start - 30) * 2
    return 60.0


def put(path: Path, header: list[str], rows) -> None:
    with path.open("w", newline="") as f:
        out = csv.writer(f)
        out.writerow(header)
        out.writerows(rows)


def write_log(path: Path, gap: tuple[float, float] | None = None, climb: float = 0.0) -> None:
    """A log as EUC Remote writes it: negative speed (riding direction), a resent row every 5th row,
    GPS speed and altitude LAG s late, altitude rising by `climb` m per m of true distance. The voltage
    alternates by 0.1 V (mean 128 V in every window), so that no two real samples are equal."""
    start = datetime(2026, 1, 1, 10, 0, 0)
    ridden = [0.0, *accumulate(shown_speed(i / HZ) * SCALE / 3.6 / HZ for i in range(int(110 * HZ)))]
    true_alt = lambda t: 500.0 + climb * ridden[int(t * HZ)]
    rows = []
    for i in range(int(110 * HZ)):
        t = i / HZ
        if gap and gap[0] <= t < gap[1]:
            continue
        kmh, late = shown_speed(t), max(t - LAG, 0)
        row = [(start + timedelta(seconds=t)).strftime("%d.%m.%Y %H:%M:%S.%f")[:-3], f"{-kmh:.2f}",
               f"{128.1 if i % 2 else 127.9:.2f}", f"{kmh:.1f}", f"{kmh / 5:.2f}",
               f"{shown_speed(late) * SCALE:.2f}", f"{true_alt(late):.2f}"]
        rows += [row, row] if i % 5 == 0 else [row]
    put(path, ["Date", "Speed", "Voltage", "PWM", "Current", "GPS speed", "Altitude"], rows)


def write_run(folder: Path) -> None:
    """An export of a simulated run: true speed 18, 36, 54 km/h; PWM 1 point and current 0.5 A above the log."""
    folder.mkdir()
    t = [i / 100 for i in range(110 * 100)]
    true = [shown_speed(x) * SCALE for x in t]
    put(folder / "motion.csv", ["t_s", "speed_kmh", "wheel_rad_s", "distance_m", "target_speed_kmh"],
        ((x, v, 0, 0, v) for x, v in zip(t, true)))
    put(folder / "drive.csv", ["t_s", "pwm_pct", "phase_a_a", "phase_b_a", "phase_c_a", "iq_a", "torque_nm",
                               "modulation_pct"], ((x, v / SCALE + 1, 0, 0, 0, 0, 0, 0) for x, v in zip(t, true)))
    put(folder / "battery.csv", ["t_s", "battery_v", "battery_a", "soc_pct"],
        ((x, 128.0, v / SCALE / 5 + 0.5, 80.0) for x, v in zip(t, true)))


class CompareTests(unittest.TestCase):
    def setUp(self) -> None:
        self.dir = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.log = self.dir / "log.csv"
        write_log(self.log)

    def windows(self, window_s: float = 2.0, mass: float | None = None) -> list[compare.Window]:
        rows, _, _ = compare.calibrate(compare.read_log(self.log))
        return compare.windows(rows, window_s, 2.0, 15.0, mass)

    def test_resent_rows_are_dropped_and_speed_is_unsigned(self) -> None:
        rows = compare.read_log(self.log)
        self.assertEqual(len(rows), 110 * HZ)
        self.assertTrue(all(r.kmh >= 0 for r in rows))

    def test_gps_lag_and_speed_scale_are_found(self) -> None:
        _, lag, scale = compare.calibrate(compare.read_log(self.log))
        self.assertAlmostEqual(lag, LAG, delta=0.25)
        self.assertAlmostEqual(scale, SCALE, places=6)

    def test_bands_hold_the_cruise_values(self) -> None:
        got = compare.bands(self.windows(mass=100.0), 5.0)
        self.assertEqual(sorted(got), [15.0, 35.0, 50.0])
        for band, shown in zip(sorted(got), (20, 40, 60)):
            b = got[band]
            self.assertAlmostEqual(b["kmh"], shown * SCALE, places=6)
            self.assertAlmostEqual(b["pwm"], shown, places=6)
            self.assertAlmostEqual(b["amps"], shown / 5, places=6)
            self.assertAlmostEqual(b["flat_w"], 128.0 * shown / 5, places=6)  # flat and steady: all battery power

    def test_climb_power_is_taken_out(self) -> None:
        write_log(self.log, climb=0.05)
        b = compare.bands(self.windows(mass=100.0), 5.0)[35.0]
        climb_w = 100.0 * compare.G * 0.05 * 40 * SCALE / 3.6
        self.assertAlmostEqual(b["flat_w"], 128.0 * 40 / 5 - climb_w, delta=0.02 * climb_w)

    def test_no_window_crosses_a_gap(self) -> None:
        write_log(self.log, gap=(10.0, 15.0))
        ws = self.windows(window_s=8.0)  # [8, 16) holds the gap
        self.assertEqual(len([w for w in ws if w.kmh < 20]), 2)  # [0, 8) and [16, 24) of the first cruise

    def test_ramps_are_not_steady(self) -> None:
        self.assertEqual(len(self.windows()), 3 * 15)  # 30 s cruises, 2 s windows

    def test_short_and_out_of_order_lines_are_handled(self) -> None:
        text = self.log.read_text().splitlines()
        self.log.write_text("\n".join([text[0], text[3], text[2], *text[4:], "01.01.2026 10:02:00.000,-1.0"]) + "\n")
        rows = compare.read_log(self.log)
        self.assertEqual(rows, sorted(rows, key=lambda r: r.t))
        self.assertEqual(len(rows), 110 * HZ)  # rows 0 and 1 swapped, the short line skipped

    def test_a_log_without_movement_has_no_lag_and_no_windows(self) -> None:
        put(self.log, ["Date", "Speed", "Voltage", "PWM", "Current", "GPS speed"],
            [[f"01.01.2026 10:00:{i // 4:02d}.{i % 4 * 250:03d}", "0", f"{128 + i % 2 * 0.1:.1f}", "0", "-1", "0"]
             for i in range(200)])
        rows, lag, scale = compare.calibrate(compare.read_log(self.log))
        self.assertEqual((lag, scale), (0.0, 1.0))
        self.assertEqual(compare.windows(rows, 2.0, 2.0, 15.0, None), [])

    def test_darknessbot_columns_and_date_forms_are_read(self) -> None:
        put(self.log, ["Date", "Speed", "Voltage", "PWM", "Current", "Power", "Battery level", "Total mileage",
                       "Temperature", "Latitude", "Longitude", "Altitude", "GPS speed"],
            [["01.01.2026T10:00:00.5", "-20.00", "128.0", "20", "4.00", "512", "80", "100.0", "30", "", "", "", ""],
             ["01.01.2026 10:00:01", "-20.10", "128.0", "20", "4.00", "512", "80", "100.0", "30", "", "", "", ""],
             ["2026-01-01T10:00:01.500+02:00", "-20.20", "128.0", "20", "4.00", "512", "80", "100.0", "30", "", "", "", ""]])
        rows = compare.read_log(self.log)
        self.assertEqual([r.t for r in rows], [0.0, 0.5, 1.0])
        self.assertEqual([r.gps for r in rows], [None] * 3)

    def test_zero_gps_speed_while_moving_is_refused(self) -> None:
        text = self.log.read_text().splitlines()
        rows = [line.rsplit(",", 2) for line in text[1:]]
        self.log.write_text("\n".join([text[0], *(f"{a},0.00,{alt}" for a, _, alt in rows)]) + "\n")
        with self.assertRaisesRegex(ValueError, "GPS speed is 0"):
            compare.calibrate(compare.read_log(self.log))
        with contextlib.redirect_stderr(io.StringIO()) as err, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(compare.main([str(self.log)]), 2)
        self.assertIn("--speed-scale", err.getvalue())

    def test_an_implausible_scale_is_refused(self) -> None:
        text = self.log.read_text().splitlines()
        rows = [line.rsplit(",", 2) for line in text[1:]]
        self.log.write_text("\n".join([text[0], *(f"{a},{float(g) * 3:.2f},{alt}" for a, g, alt in rows)]) + "\n")
        with self.assertRaisesRegex(ValueError, "outside"):
            compare.calibrate(compare.read_log(self.log))

    def test_a_window_needs_a_time_span_not_only_samples(self) -> None:
        rows = [compare.Row(t, 30.0, 128.0 + i * 0.1, 30.0, 6.0) for i, t in enumerate((0.0, 0.25, 0.5))]
        self.assertEqual(compare.windows(rows, 2.0, 2.0, 15.0, None), [])
        rows += [compare.Row(t, 30.0, 129.0 + i * 0.1, 30.0, 6.0) for i, t in enumerate((0.75, 1.0, 1.25))]
        self.assertEqual(len(compare.windows(rows, 2.0, 2.0, 15.0, None)), 1)  # 1.25 s of 2 s: enough

    def test_no_steady_windows_is_an_error(self) -> None:
        put(self.log, ["Date", "Speed", "Voltage", "PWM", "Current"],
            [[f"01.01.2026 10:00:{i // 4:02d}.{i % 4 * 250:03d}", "0", f"{128 + i % 2 * 0.1:.1f}", "0", "-1"]
             for i in range(200)])
        with contextlib.redirect_stderr(io.StringIO()) as err, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(compare.main([str(self.log)]), 1)
        self.assertIn("no steady windows", err.getvalue())

    def test_missing_columns_are_named(self) -> None:
        self.log.write_text("Date,Speed\n01.01.2026 10:00:00.000,1\n")
        with self.assertRaisesRegex(ValueError, "Current"):
            compare.read_log(self.log)

    def test_run_comparison_writes_table_csv_and_plot(self) -> None:
        run = self.dir / "run"
        write_run(run)
        out_csv, out_svg = self.dir / "t.csv", self.dir / "p.svg"
        with contextlib.redirect_stdout(io.StringIO()) as printed:
            compare.main([str(self.log), "--sim", str(run), "--mass", "100", "--csv", str(out_csv),
                          "--svg", str(out_svg)])
        self.assertIn("true / shown speed 0.900", printed.getvalue())
        with out_csv.open() as f:
            rows = list(csv.DictReader(f))
        self.assertEqual([r["band_kmh"] for r in rows], ["15-20", "35-40", "50-55"])
        for r in rows:
            self.assertEqual((r["d_pwm"], r["d_amps"], r["d_flat_w"]), ("+1.0", "+0.50", "+64"))
        self.assertTrue(out_svg.read_text().startswith("<svg"))

    def test_run_with_mismatched_times_is_refused(self) -> None:
        run = self.dir / "run"
        write_run(run)
        (run / "drive.csv").write_text("t_s,pwm_pct\n0.0,1\n")
        with self.assertRaisesRegex(ValueError, "sample times"):
            compare.read_sim(run)


if __name__ == "__main__":
    unittest.main()
