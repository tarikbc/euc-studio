# BLE radio

`--ble` puts the wheel's BLE link on this Mac's own Bluetooth radio. The Mac then advertises and serves
what the wheel's BLE module does, so a phone or a watch app connects to the simulated wheel, or to a replayed
capture, exactly as it connects to a real wheel. No app change, no emulator in between.

```sh
# The simulated wheel (stock firmware on the emulated MCU), live:
cargo run --release -p euc-cli -- sim serve begode-extreme --ble [--name GotWay_71105]

# A capture played back as a wheel, with the firmware's answers to writes:
cargo run --release -p euc-cli -- replay tests/data/begode-extreme/idle-2026-10-15.hex --ble [--loop] [--from 5]

# The same replay without a radio, on the TCP bridge of docs/bridge.md (for tests and other emulators):
cargo run --release -p euc-cli -- replay tests/data/begode-extreme/idle-2026-10-15.hex --bridge tcp://127.0.0.1:7720
```

`sim serve --ble` keeps its TCP bridge ([bridge.md](bridge.md)) open beside the radio, so a test script can
watch the stream and send controls while an app is connected over the air. `replay` takes `--ble`, `--bridge`
or both. Add `--verbose` to log every notification.

The radio is macOS only for now (CoreBluetooth). On other systems `--ble` stops with "no BLE radio here"; use
`--bridge` with a BLE peripheral emulator there.

## How it fits together

```text
 simulated wheel ─ UART bytes ─▶ module model ─┐                        ┌─▶ radio (CoreBluetooth peripheral)
 (or a BLE node of the wheel) ──────────────────┼─ notifications ─▶ link ┤
 capture file ─▶ replayer (+ brand responder) ──┘   per characteristic   └─▶ TCP bridge
                                                  ◀─ writes ─────────────
```

Everything right of the module is brand neutral: it knows advertisements, GATT tables, notifications and
writes per characteristic, never the meaning of the bytes.

| Piece | Where | Role |
|---|---|---|
| BLE profile | `profiles/<wheel>/ble.toml`, `euc_ble_radio::BleProfile` | what the module advertises, its GATT table, how it sends |
| Module model | `euc_ble_radio::UartBridge` | an HM-10 style transparent UART bridge (Begode): UART bytes in, notifications out by the profile's rules; writes on `rx` characteristics into the UART |
| Link | `euc_ble_radio::Link` | test controls, the log, text replies (through the brand's `Protocol`) |
| Radio | `euc_ble_radio::Radio` (feature `radio`) | serves any BLE profile on CoreBluetooth |
| Replay | `euc_ble_radio::Replayer` + the brand's `Responder` | a capture played back, with the firmware's answers to writes |
| Brand protocol | `euc_proto_begode::ble` (feature `ble`) | `TextFinder` (text replies in the stream), `StockResponder` (the stock firmware's command slot) |

A wheel whose BLE chip runs firmware of its own (`kind = "node"` in its profile) has no module model: that chip
is a node of the simulated wheel and hands notifications per characteristic to the link itself. The simulator
does not run such a node yet, so `sim serve` refuses those wheels; `replay` serves them.

## The BLE profile

`profiles/begode-extreme/ble.toml` describes the Begode module:

| Item | Begode Extreme |
|---|---|
| `protocol` | `begode`: euc-proto-begode reads the stream |
| `[advertising]` | `name = "GotWay_EMU"` (the simulator's own; `--name` sets any name of 1 to 29 bytes, e.g. a real wheel's `GotWay_71105`), `name_pattern = "GotWay_*"` (what real wheels advertise), `services = ["FFE0"]`, `manufacturer = ""`, optional `tx_power` |
| `[[service]]` | `FFE0`, primary |
| `[[service.characteristic]]` | `FFE1`, properties `notify`, `write`, `write-without-response`, roles `tx` (the wheel's output goes out on it) and `rx` (writes on it go into the wheel) |
| `[module]` | `kind = "uart-bridge"`, `chunk = 20` (notification size), `idle_ms = 1` (a shorter notification after 1 ms of idle line: a lone 24-byte frame goes out as 20 + 4), `interval_ms = 0` (no pacing) |

UUIDs are 16-bit (`FFE0`), 32-bit or 128-bit (`6E400001-B5A3-F393-E0A9-E50E24DCCA9E`). Properties are `read`,
`write`, `write-without-response`, `notify`, `indicate`. A characteristic may carry a fixed `value` (hex; a model
string, say), which makes it read only. Other characteristics answer reads with the last value sent on them.

The radio serves the table as written: permissions follow the properties (writeable with a write property,
readable with `read`), write requests are answered with success (one answer per batch, CoreBluetooth's rule),
and every written byte reaches the wheel in order. CoreBluetooth advertises only the local name and the service
UUIDs: manufacturer data and TX power are left out, and an `error` line says so at start. The GAP device name a
central reads after connecting is the Mac's name; apps that match the advertised local name (as EUC Remote
does) see the wheel's name.

Backpressure: when CoreBluetooth's transmit queue is full (`updateValue` returns false), the refused
notification and every later one wait in the radio's own queue, in order, until
`peripheralManagerIsReadyToUpdateSubscribers` (or, should that callback never come, a new try after 1 s). At
most 256 wait (about 13 s of Begode telemetry); only when that queue overflows do the oldest go, and a `dropped`
line (at most one a second) says how many and how deep the queue was. With `--verbose`, a `busy` line marks each
refusal. A notification longer than a central's limit is cut to it (never the case for the module's 20 bytes).

While the radio runs, `euc` holds a latency-critical activity with macOS: App Nap would otherwise slow its
timers by seconds once the terminal is in the background, and idle system sleep would stop the wheel.

## Test controls

A device test changes the link while an app is connected. Type a control on stdin and press Enter, or send it on
the TCP bridge as `{"type":"control","command":"silent"}`.

| Control | Effect |
|---|---|
| `silent` | Keep every connection and send nothing (a hung controller). Writes still reach the wheel. |
| `resume` | Send again. |
| `off` | Remove the services and stop advertising, as when the wheel is switched off. Writes are refused. The simulated wheel itself keeps running. |
| `on` | Serve and advertise again; a new `ready` line follows. |
| `quit` | End the program. |

`silent` and `off` act on every path: the radio and the TCP bridge's `notify` messages alike. After `off`,
connected centrals keep their link-layer connection but lose the services; CoreBluetooth does not report them,
so no `unsubscribe` lines follow and the next `subscribe` counts from zero.

## Log

Everything the link does is one line on stdout, written as it happens (line by line, also through a pipe).
Device test scripts wait for these lines, so their format is stable:

```text
ble <event> t=<seconds> <field>=<value> ...
```

- `t` is wall seconds since the program started, three decimals.
- Fields are `key=value` without spaces, except the last field of `ready` (`name`), `write` (`data`), `reply`
  (`text`) and `error` (`text`), which runs to the end of the line.
- Written bytes, text and names are escaped, because anyone in radio range can write: printable ASCII (space
  included) as is, every other byte and the backslash as `\xNN` (two lowercase hex digits).
- `char` is a characteristic's UUID as the profile writes it (`FFE1`). Lists (`services`, `tx`, `rx`) are comma
  separated.
- `central` is CoreBluetooth's identifier for the central (a UUID this Mac assigns, not a Bluetooth address), or
  `tcp` for a TCP bridge client.

| Line | When |
|---|---|
| `ble ready t=0.033 services=FFE0 tx=FFE1 rx=FFE1 name=GotWay_EMU` | advertising started: centrals can find and connect |
| `ble subscribe t=5.120 central=6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D char=FFE1 centrals=1 max=182` | a central subscribed; `centrals` subscribed to anything now, `max` its largest notification |
| `ble unsubscribe t=9.870 central=6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D char=FFE1 centrals=0` | a central unsubscribed or disconnected |
| `ble write t=5.161 central=6B1C2D3E-0F10-4A5B-8C9D-0E1F2A3B4C5D char=FFE1 len=1 data=N` | bytes written to the wheel (one line per write request) |
| `ble reply t=6.003 char=FFE1 centrals=1 len=14 text=NAME:EXTREME\x0d\x0a` | the wheel sent text: a reply, or other firmware text (the stock firmware's GBK boot messages too) |
| `ble silent t=12.000`, `ble resume ...`, `ble off ...`, `ble on ...`, `ble quit ...` | a control took effect |
| `ble bluetooth t=0.024 state=poweredOn` | the Mac's Bluetooth state: `poweredOn`, `poweredOff`, `unauthorized`, `unsupported`, `resetting`, `unknown` |
| `ble error t=0.030 text=cannot advertise: ...` | something failed, or a stdin line was not a control |
| `ble lap t=23.810 lap=2` | `replay --loop` started the capture again |
| `ble end t=23.255` | `replay` reached the end of its capture (without `--loop`) |
| `ble dropped t=40.000 count=7 queued=256` | notifications thrown away because the radio's queue overflowed: `count` since the last such line, `queued` the queue's depth; at most one line a second |
| `ble notify t=6.304 char=FFE1 len=20 hex=55aa...` | `--verbose`: one notification went out |
| `ble busy t=6.305 queued=3` | `--verbose`: the transmit queue was full; notifications wait |

A `reply` is logged when the brand's protocol finds text in the wheel's stream (Begode: between frames) and the
text is complete: at its line end, at the next frame, or 0.1 s after the stream went quiet (the answer to `V`
has no line end). `centrals` counts BLE centrals only (0 when only a TCP client listens). Text that goes out
while `silent` or `off` is not logged, since it does not go out. A wheel without a protocol crate logs no
replies.

Other stdout lines (`radio`, `bridge`, `client`, `replay`, `done`, ...) are for people; do not parse them.

## Captures and replay

The capture format is the one EUC Remote records and `tests/data/begode-extreme/` uses, with an optional
header for captures that need more:

```text
# euc-capture v1
# brand: Begode
# wheel: begode-extreme
# name: GotWay_71105
0.000 55 aa 19 50 00 00 00 75 00 05 00 b4 f2 5b 14 a9 00 09 00 18
0.000 5a 5a 5a 5a 55 aa 00 50 ff f8 05 10 ff ff 00 17 00 17 02 8b
1.204 write 56
2.150 notify:6E400003-B5A3-F393-E0A9-E50E24DCCA9E 01 02 03
```

- A data line is `<seconds since start> [<direction>[:<characteristic>]] <bytes as two-digit hex, space
  separated>`. The direction is `notify` (the wheel sent it; the default) or `write` (a central wrote it). The
  characteristic is a UUID of the wheel's BLE profile; without one, a notification is on the first `tx`
  characteristic and a write on the first `rx` one. So EUC Remote's files (no header, no direction) read as
  single-characteristic captures, unchanged.
- Lines starting with `#` are comments. The comment `euc-capture v1` marks the format; after it, comments of
  the form `key: value` are header fields: `brand`, `wheel` (a profile id: `replay` then needs no `--wheel`),
  `name` (what the wheel advertised); other keys are kept and ignored. A newer `euc-capture v2` is refused.
- Empty lines are skipped. Times must not go back.

The replay sends each notification line at its time, relative to the first line it plays (`--from <seconds>`:
the first line at or after that capture time, which plays at once), on its characteristic, as one notification
(no re-splitting). Write lines are what the recording central wrote; the replay leaves them out (the live
central writes for itself) and says how many. With `--loop`, a lap starts one group period (the median spacing
of the capture's groups) after the last group of the lap before.

If the program falls behind (it was paused), notifications more than 0.5 s late are skipped instead of sent in
one burst, and an `error` line says how many (`text=the replay fell behind ...`); replies to writes still go
out. `sim serve` does the same when the simulator catches up after a pause. A real wheel never sends old
telemetry, and a burst would fill the radio's queue.

Writes get the firmware's answers from the brand's responder. Begode's (`StockResponder`) does what the stock
firmware does ([protocol page](protocol/begode.md), Commands):

- The wheel sends a telemetry group at each **group time**. Every Begode group starts with frame `00`, so a
  capture line that starts frame `00` is a group time. (For a protocol whose responder marks no group starts:
  a line after 0.1 s of quiet.)
- **One command slot**, overwritten by every byte. `V` and `N` wait there; any other byte leaves it empty.
- **Every byte pauses telemetry**: the next two group times send nothing; the third sends the group, or, when
  `V` or `N` waits, the reply instead of the group: `GW1825020` for `V`, `NAME:EXTREME` CR LF for `N`. A later
  byte restarts the pause and replaces what waits. (`k` uses up the group time like the stock firmware, but its
  debug dump is not replayed.)

So expect a gap of about three group times (0.9 to 1.1 s with a real capture's spacing) after each write: the
real wheel does the same. An app that treats 1 s without bytes as stale can see the link stale right after a
write.

## Brand radios

A brand without a simulated wheel can still be replayed: its folder in `profiles/` holds only a `ble.toml`, a
**radio-only profile**. `euc replay` presents it when no wheel of that name exists (`--wheel <id>`, or the
capture's `# wheel: <id>` header). It sends the capture's notifications at their times; with no protocol crate
it answers no writes, and `euc sim serve` does not apply (there is no controller behind it). Apps under test
see the brand's advertisement and GATT table, so they pick the brand's decoder.

| Id | Brand | GATT | Writes | Emulator name | Apps accept |
|---|---|---|---|---|---|
| `inmotion-v2` | InMotion, newer protocol (V11 to V14 and others) | Nordic UART: notify `6E400003`, write `6E400002` | with response | `V11-EMU` | a case-sensitive model prefix (`V11-`, `V12-`, `V12Pro-`, `V13-`, `V14-`, ...); `--name` sets another model |
| `kingsong` | KingSong | `FFE0` / `FFE1` both ways | without response | `KS-EMU` | manufacturer data, an `FFF0` service, or a lower-case name that starts with `ks` (never a name that starts with `NO`) |
| `leaperkim` | LeaperKim / Veteran | `FFE0` / `FFE1` both ways | without response | `LK_EMU` | a lower-case name that starts with `lk` or `nf` |
| `vesc` | VESC-based boards and wheels | Nordic UART: notify `6E400003`, write `6E400002` | without response | `VESC_EMU` | an upper-case name that starts with `VESC`, `UNITY`, `FOCBOX`, `LITTLE FOCER`, `FOCSTROT` or `BKB` |

The facts come from the EUC Remote project's clean-room fact sheets (one per family; transport rows marked
confirmed there). `--name` advertises another name, for example the model prefix a capture came from.

## Adding a brand

1. **Profile.** Write `profiles/<wheel>/ble.toml` (and register it in `crates/euc-profile/src/builtin.rs`):
   advertisement, GATT table with `tx` and `rx` roles, module. A brand without a simulated wheel stops here
   with a radio-only profile (see [Brand radios](#brand-radios)) and adds a row to that table. A Nordic UART
   module, for example:

   ```toml
   schema = 1
   protocol = "examplebrand"

   [advertising]
   name = "EXAMPLE_EMU"
   name_pattern = "EXAMPLE-*"
   services = ["6E400001-B5A3-F393-E0A9-E50E24DCCA9E"]

   [[service]]
   uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

   [[service.characteristic]]
   uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
   properties = ["notify"]
   roles = ["tx"]

   [[service.characteristic]]
   uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
   properties = ["write", "write-without-response"]
   roles = ["rx"]

   [module]
   kind = "uart-bridge"   # or "node" for a BLE chip with firmware of its own (chunk only)
   chunk = 20
   idle_ms = 1
   interval_ms = 7.5      # one notification per connection event, if the module paces them
   ```

   With that alone, `replay --ble` serves the brand's captures (no text replies, writes change nothing) and the
   radio, controls and log work.
2. **Protocol.** In the brand's protocol crate, behind a `ble` feature that depends on `euc-ble-radio` without
   default features, implement `euc_ble_radio::Protocol` (find text replies in the stream) and
   `euc_ble_radio::Responder` (group starts in a capture, and what the firmware sends at a group time after a
   write, or at once). `crates/euc-proto-begode/src/ble.rs` is the model, with its tests on a real capture.
3. **Register** the protocol name in `crates/euc-cli/src/ble.rs` (`protocol`, `responder`, `known_protocol`).
4. **Simulator.** A `uart-bridge` wheel works with `sim serve --ble` as soon as its board wires the module's UART
   (the engine's BLE UART taps). A `node` wheel needs its BLE chip as a node of the simulated wheel, handing
   notifications per characteristic to `Link::notify` and taking `Action::Write`s.
5. **TCP bridge.** Its `notify` messages carry the first `tx` characteristic only (bridge protocol 1 has no
   characteristic field); a multi-characteristic brand needs a bridge field for it.

## Bluetooth permission (macOS)

macOS lets a program use Bluetooth only with the user's permission, given to the app that runs it: the terminal
(Terminal, iTerm, VS Code, ...) or the app that started the command. On the first `--ble` run macOS may show
"“<app>” would like to use Bluetooth": click **Allow**. `euc` prints a `bluetooth` hint before it starts the
radio when the permission is not decided yet. If it was denied, `euc` stops with: open **System Settings >
Privacy & Security > Bluetooth**, turn on that app, run again.

The `euc` binary carries an Info.plist (`__TEXT,__info_plist`, written by `crates/euc-cli/build.rs`) with a
Bluetooth usage text, so macOS also accepts it when `euc` itself is the app that asks.

## Known limits

- A Mac cannot connect to its own peripheral: test with another device.
- A Mac signed into the same iCloud account as an iPhone already holds a system LE link to that iPhone, and the
  iPhone's connection to the Mac's peripheral fails (status 157, reason 722). The Apple Watch connects. Use a
  phone on another account, or the watch.
- `off` cannot drop a central's connection (CoreBluetooth has no call for it); the central sees the services go
  away and no more data, then times out.
- CoreBluetooth does not say whether a write was a request or a command; both reach the wheel the same way.

## Device test checklist

For the over-the-air test with EUC Remote on an Apple Watch. Use a capture with known values, e.g.
`tests/data/begode-extreme/idle-2026-10-15.hex` (129.7 V, 0 km/h, odometer 1,468,670 m).

1. Start the replay and keep the log:
   `cargo run --release -p euc-cli -- replay tests/data/begode-extreme/idle-2026-10-15.hex --ble --loop | tee radio.log`.
   Wait for `ble ready`.
2. On the watch, scan: `GotWay_EMU` appears. Connect. Expect `ble subscribe ... char=FFE1 centrals=1`.
3. Right after it, expect `ble write ... char=FFE1 len=1 data=N`, then
   `ble reply ... text=NAME:EXTREME\x0d\x0a` within about 1 s. 1.5 s after the `N`, expect
   `ble write ... data=V` and `ble reply ... text=GW1825020`.
4. The watch shows the capture's values (129.7 V, 0 km/h).
5. Type `silent`. The app marks the data stale after 1 s and disconnects after 3 s: expect
   `ble unsubscribe ... centrals=0`, then a new `ble subscribe` and `N` / `V` writes when it reconnects (the wheel
   still advertises). Data stays stale. Type `resume`: data again.
6. Type `off`. The app loses data and disconnects after 3 s, and scanning does not find the wheel. Type `on`:
   expect `ble ready`; the app finds and reconnects to the wheel.
7. Type `quit`.

Repeat with the live simulator, `sim serve begode-extreme --ble`, and make the wheel move with a TCP client
(`{"type":"rider","kmh":15}` on the bridge, see [bridge.md](bridge.md)): the watch shows the speed.
