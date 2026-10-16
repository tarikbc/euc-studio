# Safety

An electric unicycle has no fallback: if the controller firmware stops balancing, the rider falls, often at speed.
The controllers this project targets run on 100-170 V packs and drive hundreds of amps. A firmware bug can injure the
rider, destroy the power stage or start a battery fire. Read this before you build, flash or ride anything from this
repository.

## Rules

1. **The simulator first.** Every firmware change must pass the scenario suite in the simulator, including the
   lock-step comparison against the stock firmware where it applies, before it touches hardware.
2. **The bench second.** Run new firmware on a controller on the bench, with the wheel off the ground and a
   current-limited supply, before any ride.
3. **Keep a way back.** Before the first flash of a wheel, make a full SWD backup of its flash, bootloader included.
   The vendor bootloader is not public; if an update goes wrong, SWD may be the only recovery.
4. **First flash = stock.** The flasher requires the first flash on any wheel to be the official stock image. That
   proves the update path without any firmware risk.
5. **Never ride an untested build.** Development builds are marked as such in their info block and the flasher warns
   about them. Protective gear, an empty area, low speed, and someone nearby.
6. **Do not change the drive mode or dead time** without knowing the gate driver of your board. The stock firmware has
   a hidden command that switches the motor drive mode; the flasher and the app never send it.
7. **Service commands need a hand on the wheel.** The wheel's BLE link has no pairing: any phone in range can send
   the service sequences (firmware update, IMU calibration, drive-mode toggle), and the stock firmware stops the
   motor at once, even under a rider. Our clean firmware runs them only when nobody rides (the wheel is off, on
   its side, or cut with the handle button) and the power button is pressed within 10 s. Flashing a wheel that
   runs our clean firmware therefore needs that press. The simulator shows both sides
   (`scenarios/ble-service-cut-standing.toml`): with a rider standing still, a phone sends "c" then "y"; the stock
   firmware cuts the motor 10 ms later and wheel and rider fall, while the clean firmware answers `SERVICE:DENIED`
   and keeps balancing ([docs/findings/stock-firmware.md](docs/findings/stock-firmware.md)).

## Reporting a safety problem

If you find a behavior in this project's firmware, simulator or flasher that could hurt a rider, report it privately
through GitHub's "Report a vulnerability" (Security tab) instead of a public issue. Include the scenario or steps,
the firmware build and what you expected.

## No warranty

This project is provided without any warranty. You alone are responsible for what you flash and ride. See LICENSE.
