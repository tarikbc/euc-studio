# BLE bridge

`euc sim serve` runs a simulated wheel at real time and exposes its BLE link as a TCP stream, so a
BLE peripheral emulator (EUC Remote's `eucctl emulate` on macOS, for example) can present the
simulated wheel to a phone or a watch app as if it were the real one. Everything the firmware sends
to its BLE module comes out as notifications; everything a client writes goes into the firmware's
BLE UART, as the module would deliver it.

```sh
cargo run --release -p euc-cli --features unicorn -- sim serve begode-extreme --ble-bridge tcp://127.0.0.1:7720
cargo run --release -p euc-cli --features unicorn -- sim bridge-test          # in another terminal
```

The simulated wheel is the full engine: the stock firmware on the emulated MCU, the motor and battery,
and the body with a rider who stands still (or rides a `--scenario`). The stand lets the wheel go at
2.5 s (`--release-s`).

## Connection

- TCP, one client at a time. The server keeps the wheel running between clients; a new client gets a
  fresh `hello` and the stream from that moment on.
- Default address `127.0.0.1:7720`. A non-loopback address works, with a warning: anyone who can reach
  it can drive the simulated wheel.
- Both directions carry **newline-delimited JSON**: one UTF-8 JSON object per line, ended by `\n`.
  Every object has a `type`. Readers must ignore unknown `type`s and unknown fields, so later versions
  can add them.
- `t` is simulated time in seconds since the wheel was switched on. The server paces simulated time to
  wall time (`--rate`, 1.0 by default); `state` reports the rate it achieves.

## Server to client

| `type` | Fields | Meaning |
|---|---|---|
| `hello` | `version` (1), `wheel`, `firmware`, `notify` (`split` or `burst`), `rate`, `t` | First line after connecting. |
| `notify` | `t`, `hex` | One BLE notification on FFE1 (the characteristic the wheel's module uses), payload as lowercase hex. `t` is when its last byte left the controller's UART. |
| `log` | `t`, `level` (`info` or `warn`), `text` | Something happened: stand released, firmware cut the motor, the wheel fell, a fault, a never-send command, a request the server could not use. For people; do not parse `text`. |
| `state` | `t`, `speed_kmh`, `pitch_deg`, `battery_v`, `battery_a`, `rate` | The simulated wheel as it really is (not what the firmware reports), five times per simulated second. `pitch_deg` is nose up positive. |

```json
{"type":"hello","version":1,"wheel":"begode-extreme","firmware":"GW1825007","notify":"split","rate":1.0,"t":1.71}
{"type":"notify","t":1.913,"hex":"55aa195500000075000600a0f0fb14a9000900185a"}
{"type":"state","t":2.0,"speed_kmh":0.0,"pitch_deg":0.0,"battery_v":129.7,"battery_a":0.0,"rate":1.0}
```

### How bytes become notifications

`--notify split` (the default) does what the wheel's BLE module does: it packs the UART byte stream
into notifications of 20 bytes and sends a shorter one when the line has been quiet for 1 ms. A lone
24-byte frame arrives as 20 + 4 bytes; the stock firmware's bursts of five frames (120 bytes, frames
2.2 ms apart) arrive as six notifications of 20 bytes, as in the capture
`tests/data/begode-extreme/idle-2026-10-15.hex`. Frames therefore straddle notifications: a client
reassembles the stream (the frame format is in [protocol/begode.md](protocol/begode.md);
`euc_proto_begode::Assembler` does it in Rust).

`--notify burst` sends each burst (bytes less than 1 ms apart) as one notification of any length, for
clients that do not care about the module's packet size.

## Client to server

| `type` | Fields | Effect |
|---|---|---|
| `write` | `hex` | Bytes a phone wrote to FFE1. They reach the firmware's BLE UART at the current simulated time, at the UART's pace. |
| `rider` | `kmh` | The rider rides at this ground speed (0 stands still). Takes the rider over from a scenario. |
| `push` | `newtons`, `seconds` | Pushes the rider forward (negative: back) at the centre of mass for a while. |

```json
{"type":"write","hex":"56"}
{"type":"rider","kmh":15}
```

Write one command byte per `write`, at least 0.1 s apart, as the vendor app does: the stock
firmware handles one received byte per control tick and keeps only the last one, so bytes that
arrive together are lost, exactly as on a real wheel (see [protocol/begode.md](protocol/begode.md)).

A line the server cannot use gets a `log` with `level` `warn` back and changes nothing.

### Never-send commands

Some commands are dangerous on a real wheel: `+` then `-` (switches the drive mode, saves it and powers
the wheel off), `c` then `y` (erases the IMU calibration), `!` then `@` (starts a firmware update). The
bridge is a simulator, so it passes them through unchanged, and logs each one as a `warn` on the
client and on the server's console. A client that also talks to real wheels should refuse to send
them there.

## Test client

`euc sim bridge-test [--url tcp://127.0.0.1:7720] [--seconds 6] [--send VN]` connects, writes the
`--send` bytes one per second from 1 s on, reassembles the notifications into frames and text, and
prints what it saw: notification sizes, frames, the decoded frame 00 (speed, voltage), text replies
(`GW1825020`, `NAME:EXTREME`), the last `state`. It fails when no telemetry frame came through.

## Versioning

`hello.version` is 1. A change that old clients would misread (a field changing meaning, a message
they must act on) raises it; new message types and fields do not.
