#!/usr/bin/env python3
"""Writes imu-frame-synthetic.hex: the stock telemetry of idle-2026-10-15.hex with IMU frames (type 20) added.

The IMU frame is the optional frame of our firmware (docs/protocol/begode.md, "IMU frame"). This file is synthetic
test data for decoders: 3 s of stock frame groups every 304 ms, taken in order from the real idle capture, and an
IMU frame every 5 control ticks (99.2 Hz) with a known motion:

- pitch 3.00° sin(2π 0.8 t), nose down positive; roll 1.50° sin(2π 0.4 t), right side down positive;
- gyro x and y the angles' rates, gyro z 0; accel the gravity those angles give (x = -sin pitch, y = sin roll,
  z = cos pitch cos roll, in mg);
- frame 100 is dropped (its sequence number is skipped), frame 150 carries an impact sample whose payload holds
  `55 AA` across two fields, so the encoder adds 1 to the field that holds the `55`.

The bytes go out as our firmware would send them: the UART at 115200 baud (86.8 µs a byte), a frame group first
when both are due, and the BLE module's 20-byte notifications, a shorter one after 1 ms of idle line
(profiles/begode-extreme/ble.toml). Run it from anywhere: python3 make_imu_synthetic.py
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, "idle-2026-10-15.hex")
OUT = os.path.join(HERE, "imu-frame-synthetic.hex")

SECONDS = 3.0
TICK_S = 128 * 126 / 8e6  # the control interrupt: 2.016 ms
IMU_PERIOD_S = 5 * TICK_S  # 99.2 Hz
GROUP_PERIOD_S = 151 * TICK_S  # 304 ms
BYTE_S = 10 / 115200  # 8N1
CHUNK = 20
IDLE_S = 0.001
TYPE_IMU = 0x20
DROPPED = 100
IMPACT = 150


def stock_frames():
    """The 24-byte frames of the real capture, in order."""
    stream = bytearray()
    with open(SOURCE, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            stream += bytes.fromhex("".join(line.split()[1:]))
    frames, i = [], 0
    while i + 24 <= len(stream):
        if stream[i : i + 2] == b"\x55\xaa" and stream[i + 20 : i + 24] == b"\x5a" * 4:
            frames.append(bytes(stream[i : i + 24]))
            i += 24
        else:
            i += 1
    return frames


def stock_groups():
    """The capture's frame groups, each starting with a type 00 frame."""
    groups = []
    for frame in stock_frames():
        if frame[18] == 0x00:
            groups.append([])
        if groups:
            groups[-1].append(frame)
    return [g for g in groups if len(g) == 5]


def s16(value):
    return max(-32768, min(32767, int(round(value))))


def no_55aa(fields):
    """The encoder's rule: while bytes 2-17 hold 55 AA, add 1 to the field that holds the 55."""
    fields = list(fields)
    while True:
        payload = b"".join((v & 0xFFFF).to_bytes(2, "big") for v in fields)
        at = payload.find(b"\x55\xaa")
        if at < 0:
            return fields, payload
        fields[at // 2] = fields[at // 2] + 1


def imu_frame(n, t):
    pitch = 3.0 * math.sin(2 * math.pi * 0.8 * t)
    roll = 1.5 * math.sin(2 * math.pi * 0.4 * t)
    pitch_rate = 3.0 * 2 * math.pi * 0.8 * math.cos(2 * math.pi * 0.8 * t)
    roll_rate = 1.5 * 2 * math.pi * 0.4 * math.cos(2 * math.pi * 0.4 * t)
    p, r = math.radians(pitch), math.radians(roll)
    fields = [
        s16(pitch * 100), s16(roll * 100),
        s16(roll_rate * 10), s16(pitch_rate * 10), 0,
        s16(-1000 * math.sin(p)), s16(1000 * math.sin(r)), s16(1000 * math.cos(p) * math.cos(r)),
    ]
    if n == IMPACT:
        # An impact sample: accel y low byte 55, then accel z high byte AA (about -22 g).
        fields[6], fields[7] = 0x0155, s16(-21984)
    fields, payload = no_55aa(fields)
    return b"\x55\xaa" + payload + bytes([TYPE_IMU, n & 0xFF]) + b"\x5a" * 4


def notifications(sends):
    """`sends` = [(t, bytes)], in send order: the UART bytes, packed into the module's notifications."""
    out, pending, line_free, last_byte = [], bytearray(), 0.0, 0.0
    for t, data in sends:
        start = max(t, line_free)
        if pending and start - last_byte >= IDLE_S:
            out.append((last_byte + IDLE_S, bytes(pending)))
            pending.clear()
        for k, byte in enumerate(data):
            last_byte = start + (k + 1) * BYTE_S
            pending.append(byte)
            if len(pending) == CHUNK:
                out.append((last_byte, bytes(pending)))
                pending.clear()
        line_free = last_byte
    if pending:
        out.append((last_byte + IDLE_S, bytes(pending)))
    return out


def main():
    groups = stock_groups()
    sends, g = [], 0
    imu_times = [k * IMU_PERIOD_S for k in range(int(SECONDS / IMU_PERIOD_S))]
    group_times = [0.010 + k * GROUP_PERIOD_S for k in range(int(SECONDS / GROUP_PERIOD_S) + 1)]
    for t in sorted(group_times + imu_times):
        if t in group_times:  # the group goes first when both are due
            sends.append((t, b"".join(groups[g % len(groups)])))
            g += 1
        if t in imu_times:
            n = imu_times.index(t)
            if n != DROPPED:
                sends.append((t, imu_frame(n, t)))
    sends.sort(key=lambda s: s[0])
    lines = notifications(sends)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("# euc-capture v1\n# brand: Begode\n# wheel: begode-extreme\n# name: GotWay_EMU\n")
        f.write("# note: synthetic, made by make_imu_synthetic.py (stock groups of idle-2026-10-15.hex + IMU frames)\n")
        for t, data in lines:
            f.write(f"{t:.4f} {data.hex(' ')}\n")

    # Self-check: reassemble, count, and confirm the rules.
    stream = b"".join(d for _, d in lines)
    imu = [stream[i : i + 24] for i in range(len(stream) - 23)
           if stream[i : i + 2] == b"\x55\xaa" and stream[i + 18] == TYPE_IMU and stream[i + 20 : i + 24] == b"\x5a" * 4]
    seqs = [f[19] for f in imu]
    gaps = [(a, b) for a, b in zip(seqs, seqs[1:]) if (b - a) % 256 != 1]
    assert len(imu) == len(imu_times) - 1, len(imu)
    assert gaps == [((DROPPED - 1) & 0xFF, (DROPPED + 1) & 0xFF)], gaps
    assert all(b"\x55\xaa" not in f[2:18] for f in imu)
    print(f"{OUT}: {len(lines)} notifications, {len(imu)} IMU frames, {g} groups, gap {gaps[0]}")


if __name__ == "__main__":
    main()
