#!/usr/bin/env python3
"""Compare a real ride log with a simulated run, per speed band (issue #35).

The input is a ride-logging app's CSV (EUC Remote and DarknessBot write the same columns):

    Date,Speed,Voltage,PWM,Current,...,Altitude,GPS speed
    01.01.2026 10:00:00.250,-30.00,126.40,32.0,6.50,...,100.0,27.10

The script keeps only steady stretches: non-overlapping windows (2 s by default) whose speed stays within a
range (2 km/h) above a minimum speed (15 km/h). Rows that repeat the previous row are app resends, not new
samples, and a window across a gap in the log is dropped. It then prints a table per band of true speed: the
firmware's PWM, the pack voltage, the battery current and, with `--mass`, the battery power with the
acceleration and the road slope (from GPS altitude) taken out, so that the band shows a flat, steady cruise.

The log's speed is the speed the wheel shows; the true speed is that times the GPS / shown ratio measured in the
same log (`--speed-scale` sets it by hand). GPS lags the wheel; the lag is measured and taken out first.

`--sim DIR` adds the same table for a simulated run exported with `euc rec export run.eucrec --csv DIR`, and
the difference to the log. Its `speed_kmh` channel is taken as the true ground speed, and its road as level
(run it on an ideal flat road).

Never commit a ride log: it holds GPS positions. Only derived numbers belong in the repository.

Python 3.11+ standard library only.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from bisect import bisect_left
from collections import defaultdict
from dataclasses import dataclass, replace
from datetime import datetime
from itertools import accumulate, groupby, pairwise
from pathlib import Path

G = 9.81
GAP_S = 1.0  # a longer step between two rows is a hole in the log
GRADE_SPAN_S = 10.0  # the slope comes from GPS altitude over this much time on each side of a window
MAX_GRADE = 0.15  # steeper GPS slopes are noise
MIN_SCALE_KMH = 15.0  # rows slower than this do not count for the GPS / shown ratio
SCALE_RANGE = (0.5, 1.5)  # a true / shown ratio outside this is a bad GPS column, not a wheel
MIN_SPAN_SHARE = 0.6  # a window's samples must cover this share of its length (a 2 s window: 1.2 s)


@dataclass(frozen=True)
class Row:
    t: float  # s
    kmh: float  # speed, km/h (shown by the wheel in a raw log, true after calibrate() and in a simulated run)
    volts: float
    pwm: float  # %
    amps: float  # battery current, + = discharge
    gps: float | None = None  # km/h
    alt: float | None = None  # m


@dataclass(frozen=True)
class Window:
    kmh: float  # true speed
    volts: float
    pwm: float
    amps: float
    flat_w: float | None  # battery power of a flat, steady cruise


# Band columns: key, format, and the format of the difference run - log (None: no difference column).
METRICS = [("n", "{:.0f}", None), ("kmh", "{:.1f}", None), ("pwm", "{:.1f}", "{:+.1f}"),
           ("volts", "{:.1f}", None), ("amps", "{:.2f}", "{:+.2f}"), ("flat_w", "{:.0f}", "{:+.0f}")]
PLOTTED = (("pwm", "#1f77b4"), ("amps", "#d62728"))


def number(text: str | None) -> float | None:
    """A finite float, or None for an empty, short (None) or bad field."""
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def read_log(path: Path) -> list[Row]:
    rows: list[Row] = []
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        missing = {"Date", "Speed", "Voltage", "PWM", "Current"} - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns {sorted(missing)}")
        start = None
        for line in reader:
            when = timestamp(line["Date"])
            values = [number(line[k]) for k in ("Speed", "Voltage", "PWM", "Current")]
            if when is None or None in values:
                continue
            start = start or when
            speed, volts, pwm, amps = values
            rows.append(Row((when - start).total_seconds(), abs(speed), volts, pwm, amps,
                            number(line.get("GPS speed")), number(line.get("Altitude"))))
    return drop_repeats(sorted(rows, key=lambda r: r.t))


def timestamp(text: str | None) -> datetime | None:
    """DarknessBot and EUC Remote write `dd.MM.yyyy HH:mm:ss.SSS`; a `T` separator, no milliseconds and ISO dates
    are accepted too."""
    text = (text or "").replace("T", " ", 1)
    for form in ("%d.%m.%Y %H:%M:%S.%f", "%d.%m.%Y %H:%M:%S"):
        try:
            return datetime.strptime(text, form)
        except ValueError:
            pass
    try:
        return datetime.fromisoformat(text).replace(tzinfo=None)  # the wall clock, as in the other forms
    except ValueError:
        return None


def drop_repeats(rows: list[Row]) -> list[Row]:
    """The apps resend the last frame when no new one came; such a row carries no new sample."""
    return [next(run) for _, run in groupby(rows, key=lambda r: (r.kmh, r.volts, r.pwm, r.amps))]


def read_sim(folder: Path) -> list[Row]:
    """A run exported by `euc rec export`: motion.csv, drive.csv and battery.csv share their t_s column.
    The run is on a level road, so its altitude is a constant 0."""
    def read(name: str) -> list[dict[str, str]]:
        with (folder / name).open(newline="") as f:
            return list(csv.DictReader(f))

    motion, drive, battery = read("motion.csv"), read("drive.csv"), read("battery.csv")
    if not [r["t_s"] for r in motion] == [r["t_s"] for r in drive] == [r["t_s"] for r in battery]:
        raise ValueError(f"{folder}: motion.csv, drive.csv and battery.csv have different sample times")
    rows = []
    for m, d, b in zip(motion, drive, battery):
        t, kmh, volts, pwm, amps = (number(m["t_s"]), number(m["speed_kmh"]), number(b["battery_v"]),
                                    number(d["pwm_pct"]), number(b["battery_a"]))
        if None not in (t, kmh, volts, pwm, amps):
            rows.append(Row(t, abs(kmh), volts, pwm, amps, alt=0.0))
    return sorted(rows, key=lambda r: r.t)


def later(rows: list[Row], lag: float) -> list[Row | None]:
    """For each row, the row nearest to lag s later, if one lies within GAP_S (rows sorted by time)."""
    ts = [r.t for r in rows]
    out: list[Row | None] = []
    for t in ts:
        i = bisect_left(ts, t + lag)
        if i == len(ts) or (i > 0 and t + lag - ts[i - 1] < ts[i] - t - lag):
            i -= 1
        out.append(rows[i] if abs(ts[i] - t - lag) <= GAP_S else None)
    return out


def gps_lag(rows: list[Row]) -> float:
    """The delay (s) that best lines GPS speed up with the wheel's speed (0 without GPS)."""
    def fit(lag: float) -> float:
        pairs = [(r.kmh, q.gps) for r, q in zip(rows, later(rows, lag)) if q is not None and q.gps is not None]
        try:
            return statistics.correlation(*zip(*pairs)) if len(pairs) > 10 else -1.0
        except statistics.StatisticsError:  # a constant speed or GPS: no information
            return -1.0
    return max((step * 0.25 for step in range(21)), key=fit) if any(r.gps is not None for r in rows) else 0.0


def calibrate(rows: list[Row], scale: float | None = None) -> tuple[list[Row], float, float]:
    """Take the GPS lag out of GPS speed and altitude, and turn the shown speed into true speed.
    Returns the new rows, the lag and the true / shown scale (measured from GPS unless given, else 1)."""
    lag = gps_lag(rows)
    shift = later(rows, lag)
    gps = [q.gps if q else None for q in shift]
    alt = [q.alt if q else None for q in shift]
    if scale is None:
        moving = [(r, g) for r, g in zip(rows, gps) if g is not None and r.kmh >= MIN_SCALE_KMH]
        ratios = [g / r.kmh for r, g in moving if g > 0]
        if moving and not ratios:
            raise ValueError("GPS speed is 0 while the wheel moves: no true / shown ratio; set --speed-scale")
        scale = statistics.median(ratios) if ratios else 1.0
        if not SCALE_RANGE[0] <= scale <= SCALE_RANGE[1]:
            raise ValueError(f"measured true / shown speed {scale:.3f} is outside {SCALE_RANGE}: check the GPS "
                             "speed column or set --speed-scale")
    return [replace(r, kmh=r.kmh * scale, gps=g, alt=a) for r, g, a in zip(rows, gps, alt)], lag, scale


def windows(rows: list[Row], window_s: float, span_kmh: float, min_kmh: float, mass: float | None) -> list[Window]:
    """Steady windows of calibrated rows. With a mass, each gets the battery power of a flat, steady cruise."""
    ts = [r.t for r in rows]
    # Prefix sums: metres ridden and holes in the log up to each row, for the slope over any span.
    metres = [0.0, *accumulate((b.t - a.t) * (a.kmh + b.kmh) / 7.2 for a, b in pairwise(rows))]
    holes = [0, *accumulate(int(b.t - a.t > GAP_S) for a, b in pairwise(rows))]

    def grade(t: float) -> float | None:
        i, j = bisect_left(ts, t - GRADE_SPAN_S), bisect_left(ts, t + GRADE_SPAN_S) - 1
        if j <= i or holes[j] != holes[i] or metres[j] == metres[i] or rows[i].alt is None or rows[j].alt is None:
            return None
        slope = (rows[j].alt - rows[i].alt) / (metres[j] - metres[i])
        return slope if abs(slope) <= MAX_GRADE else None

    out: list[Window] = []
    for _, run in groupby(rows, key=lambda r: int(r.t // window_s)):
        group = list(run)
        if len(group) < 3 or any(b.t - a.t > GAP_S for a, b in pairwise(group)):
            continue
        speeds = [r.kmh for r in group]
        mean = statistics.fmean(speeds)
        if max(speeds) - min(speeds) > span_kmh or mean < min_kmh:
            continue
        t0, t1 = group[0].t, group[-1].t
        if t1 - t0 < MIN_SPAN_SHARE * window_s:
            continue
        power = statistics.fmean(r.volts * r.amps for r in group)
        slope = grade((t0 + t1) / 2) if mass is not None else None
        accel = (speeds[-1] - speeds[0]) / 3.6 / (t1 - t0)
        flat_w = None if slope is None else power - mass * (accel + G * slope) * mean / 3.6
        out.append(Window(mean, statistics.fmean(r.volts for r in group), statistics.fmean(r.pwm for r in group),
                          statistics.fmean(r.amps for r in group), flat_w))
    return out


def median(values: list[float | None]) -> float | None:
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def bands(ws: list[Window], width: float) -> dict[float, dict[str, float | None]]:
    by_band: dict[float, list[Window]] = defaultdict(list)
    for w in ws:
        by_band[math.floor(w.kmh / width) * width].append(w)
    return {band: {"n": len(group), **{k: median([getattr(w, k) for w in group]) for k, _, _ in METRICS if k != "n"}}
            for band, group in sorted(by_band.items())}


def table(log: dict, sim: dict | None, width: float) -> list[list[str]]:
    """Header and rows: the log's columns, then the run's and the differences (run - log) when there is a run."""
    head = ["band_kmh"] + [f"log_{k}" for k, _, _ in METRICS]
    if sim is not None:
        head += [f"sim_{k}" for k, _, _ in METRICS] + [f"d_{k}" for k, _, d in METRICS if d]

    def fmt(value: float | None, spec: str) -> str:
        return "" if value is None else spec.format(value)

    out = [head]
    for band in sorted(set(log) | set(sim or {})):
        a, b = log.get(band, {}), (sim or {}).get(band, {})
        row = [f"{band:g}-{band + width:g}"] + [fmt(a.get(k), s) for k, s, _ in METRICS]
        if sim is not None:
            row += [fmt(b.get(k), s) for k, s, _ in METRICS]
            row += [fmt(None if a.get(k) is None or b.get(k) is None else b[k] - a[k], d) for k, _, d in METRICS if d]
        out.append(row)
    return out


def svg(log: dict, sim: dict | None, path: Path) -> None:
    """PWM (%) and battery current (A) against true speed: dots for the log, rings for the run."""
    w, h, pad = 640, 360, 50
    series = [(log, True), (sim or {}, False)]
    points = [(b["kmh"], b[k]) for s, _ in series for b in s.values() for k, _ in PLOTTED if b.get(k) is not None]
    if not points:
        raise ValueError("nothing to plot: no steady windows")
    xmax = max(x for x, _ in points) * 1.1
    ymin, ymax = min(min(y for _, y in points) * 1.1, 0.0), max(max(y for _, y in points) * 1.1, 1.0)
    sx = lambda x: pad + x / xmax * (w - 2 * pad)
    sy = lambda y: h - pad - (y - ymin) / (ymax - ymin) * (h - 2 * pad)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" font-family="sans-serif" font-size="12">',
             f'<rect width="{w}" height="{h}" fill="white"/>',
             f'<line x1="{pad}" y1="{sy(0):.1f}" x2="{w - pad}" y2="{sy(0):.1f}" stroke="black"/>',
             f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{h - pad}" stroke="black"/>',
             f'<text x="{w / 2}" y="{h - 12}" text-anchor="middle">true speed (km/h), 0 to {xmax:.0f}</text>',
             f'<text x="{pad}" y="{pad - 12}">{ymin:.0f} to {ymax:.0f}: PWM % (blue), battery current A (red); '
             f'dots = log, rings = run</text>']
    for data, filled in series:
        for b in data.values():
            for key, colour in PLOTTED:
                if b.get(key) is not None:
                    parts.append(f'<circle cx="{sx(b["kmh"]):.1f}" cy="{sy(b[key]):.1f}" r="4" '
                                 f'fill="{colour if filled else "none"}" stroke="{colour}"/>')
    path.write_text("\n".join(parts + ["</svg>"]) + "\n", encoding="utf-8")


def positive(text: str) -> float:
    value = float(text)
    if not value > 0:
        raise argparse.ArgumentTypeError(f"must be above 0: {text}")
    return value


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log", type=Path, help="ride log CSV (EUC Remote or DarknessBot)")
    ap.add_argument("--sim", type=Path, help="folder written by `euc rec export run.eucrec --csv DIR`")
    ap.add_argument("--mass", type=positive, help="rider + wheel + gear, kg: adds the flat, steady battery power")
    ap.add_argument("--window", type=positive, default=2.0, help="window length, s (default 2)")
    ap.add_argument("--span", type=positive, default=2.0, help="largest speed range in a window, km/h (default 2)")
    ap.add_argument("--min-speed", type=float, default=15.0, help="smallest true speed, km/h (default 15)")
    ap.add_argument("--band", type=positive, default=5.0, help="speed band width, km/h (default 5)")
    ap.add_argument("--speed-scale", type=positive, help="true / shown speed (default: measured from GPS)")
    ap.add_argument("--csv", type=Path, help="also write the table as CSV")
    ap.add_argument("--svg", type=Path, help="also write a plot")
    args = ap.parse_args(argv)

    try:
        rows, lag, scale = calibrate(read_log(args.log), args.speed_scale)
        sim_rows = read_sim(args.sim) if args.sim else None
    except ValueError as why:
        print(f"error: {why}", file=sys.stderr)
        return 2
    log_windows = windows(rows, args.window, args.span, args.min_speed, args.mass)
    if args.mass and not any(r.alt is not None for r in rows):
        print("note: the log has no Altitude column, so there is no slope and no flat power")
    print(f"log: {len(rows)} rows, {len(log_windows)} steady windows; GPS lag {lag:.2f} s; "
          f"true / shown speed {scale:.3f}")
    if not log_windows:
        print("error: no steady windows in the log; nothing to compare", file=sys.stderr)
        return 1
    log = bands(log_windows, args.band)
    sim = None
    if sim_rows is not None:
        sim_windows = windows(sim_rows, args.window, args.span, args.min_speed, args.mass)
        print(f"run: {len(sim_windows)} steady windows")
        if not sim_windows:
            print("error: no steady windows in the run; nothing to compare", file=sys.stderr)
            return 1
        sim = bands(sim_windows, args.band)
    rows_out = table(log, sim, args.band)
    print("| " + " | ".join(rows_out[0]) + " |")
    print("|" + "---|" * len(rows_out[0]))
    for row in rows_out[1:]:
        print("| " + " | ".join(row) + " |")
    if args.csv:
        with args.csv.open("w", newline="") as f:
            csv.writer(f).writerows(rows_out)
    if args.svg:
        svg(log, sim, args.svg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
