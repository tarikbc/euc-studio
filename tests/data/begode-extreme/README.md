# Begode Extreme test data

## `idle-2026-10-15.hex`

Telemetry from a real Begode Extreme (the maintainer's own wheel), recorded on 2026-10-15 with a macOS CoreBluetooth
central subscribed to characteristic `FFE1` of service `FFE0`. The wheel stood still, powered on, battery at about 87 %.

- Format: one BLE notification per line, `<seconds since start> <hex bytes>`. Lines keep the real packet splits
  (frames arrive as 20 + 4 byte notifications), so decoders must reassemble a byte stream.
- 456 notifications in 25 s, which reassemble into 380 frames of 24 bytes: `55 AA`, 16 payload bytes, type at byte 18,
  sub-index at byte 19, footer `5A 5A 5A 5A`. One frame group (types 00, 01, 02 or 03, 04, 07) every ~310 ms.
- Values at the time: pack 129.7 V, two packs of 32 cells at about 4.07 V, board temperature about 25 °C, odometer
  1,468,670 m, firmware banner `GW1825020`, drive mode 1.
- It holds telemetry values only: no BLE name, address or other identifiers.

## `imu-frame-synthetic.hex`

Synthetic, not recorded: 3 s of the stock frame groups of `idle-2026-10-15.hex` (in order, one every 304 ms) with the
optional IMU frame of our firmware (type `20`, `docs/protocol/begode.md`, "IMU frame") every 5 control ticks
(99.2 Hz). Made by `make_imu_synthetic.py` next to it, which also checks its output; run it again after a format
change.

- The IMU frames carry a known motion: pitch 3.00° sin(2π 0.8 t), roll 1.50° sin(2π 0.4 t), their rates on gyro y and
  x, and the gravity those angles give on the accel axes.
- Frame 100 is dropped (sequence 99 → 101), and frame 150 holds a synthetic sample beyond the sensor's range (accel z
  about -22 g; the ±4 g sensor cannot read it) whose payload needed the `55 AA` adjustment. Real values never need it.
- The notifications follow the module model of `profiles/begode-extreme/ble.toml` (20 bytes, a shorter one after 1 ms
  of idle line) on a 115200-baud UART: 646 notifications, 50 stock frames and 296 IMU frames.
