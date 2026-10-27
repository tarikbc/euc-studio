# Ride-log analysis: what one real ride says about the plant

Status: PRELIMINARY. One ride log, about 4 Hz. Refs #35, #36, #37, #59.

## Main findings

1. **Speed scale confirmed.** True / shown speed is 0.89-0.91 by three independent methods. It does not tell 31 pole
   pairs from a smaller rolling radius.
2. **Extra battery draw (#36): about 200-340 W, nearly constant from 7 to 60 km/h.** A loss that grows with speed
   would need crr = 0.058, which is not plausible.
3. **Current offset of about −0.8 to −1.1 A.** With PWM exactly 0 at a standstill, the battery current reads negative.
4. **The battery current reads low at high power.** The inertia in the battery power is 0.89-0.94 of the expected
   value below 30 km/h and 0.77-0.79 above it. The copper loss is almost not visible.
5. **Pack resistance about 310 mΩ** for loads up to 30 s. The plant has 160 mΩ. The voltage reading is probably
   filtered (about 3 s).
6. **The motor inductance decides the voltage headroom near top speed.** It explains 0.81-0.94 of the variance
   (Sobol first-order). In 80 % of the plausible parameter sets, the headroom for a further 1 m/s² request ends below
   the speed of the 80 % PWM alarm. Measuring the inductance (#37) has the largest single effect.
7. **Simulator against the log (stock firmware, plant on `main`).** At the same true speed the simulated PWM is within
   2 points of the log, and the simulated battery current is 0.9-3.1 A lower (flat power 100-300 W lower).
8. **Steady flat riding cannot calibrate the current reading.** Only a climb of known slope has a known coefficient.
   A 15-minute calibration ride with a hill is designed below.

## Data and inputs

The ride log is a Begode Extreme on stock firmware: 30 minutes, 13.3 km, about 4 Hz, in the EUC Remote CSV format.
The log and its GPS track are not in the repository; only derived numbers are here. Values marked REDACTED are
private: the models and every result below use them, but this page does not publish them. The ride's date, time and
place are not published either. The comparison tool is [tools/ride-log](../../tools/ride-log/README.md).

| Kind | Input | Value |
| --- | --- | --- |
| Measured | Ride log columns | speed, voltage, PWM, battery current, temperature, GPS speed and altitude |
| Measured | Wheel mass | 39 kg |
| Measured | Total mass (rider, wheel, gear; the gear estimated) | REDACTED |
| ESTIMATE | Rotating part of the wheel, as a mass | about 10 kg |
| ESTIMATE | Motor inductance span (headroom only) | 100-400 µH, from [finding 10](../findings/stock-firmware.md) |
| Researched | Weather during the ride (ERA5 reanalysis, hourly): temperature, humidity, pressure, wind, sun | REDACTED |
| Calculated | Air density from that weather | 1.10 kg/m³ |
| Repository | Plant values (`profiles/begode-extreme/plant.toml`, branch `p1-physics`) | ψ 18.0 mWb, R 80 mΩ, L 200 µH, pack 160 mΩ, crr 0.014, CdA 0.6, air 1.2 |

**Data quality:** 15 % of the rows repeat the previous row (the app resends the last frame). A 116 s hole hides about
1.07 km of riding (the odometer jumps over it). GPS speed is 1.5-2 s behind the wheel. PWM comes in whole percent.
"Battery level" follows the voltage under load. The battery current and voltage fit best 0.3 s after the speed.

## Methods

| Question | Method |
| --- | --- |
| True speed, acceleration | Kalman filter and RTS smoother on the wheel speed |
| Road slope | RTS smoother on GPS altitude over a 10 m distance grid; smoothing chosen by time-block cross-validation |
| Losses | P_bat = F·v + 1.5·R·Iq² + P0 + c·V, F = m_eff·a + m·g·(sin θ + crr) + ½·ρ·CdA·v²; robust least squares on 3,306 moving rows |
| Battery | V = OCV(charge) − R0·I − RC parts; a search over the time constants |
| Temperature | C·dT/dt = R·I² + k·sun − h·(T − T_air) |
| Error ranges | Block bootstrap (30 s blocks), time-block cross-validation |
| Method check | Fake logs with known values, through the same pipeline |
| Missing physics | Gradient boosting on the residuals (out-of-block R²), partial dependence |
| Headroom | Steady FOC voltage model (id = 0) with pack sag; Monte Carlo (20,000 draws) and Sobol first-order indices |
| Calibration ride | Fisher information, D-optimal design (greedy start, simulated annealing) |

**Method checks on fake logs:** speed scale 0.9052 ± 0.0015 for a true 0.905. Through the full power pipeline:
m_eff, crr, CdA, R and P0 within 0.1 %, 12 %, 2 %, 0 % and 1 % of their true values.

## Results in detail

1. **Speed scale.** Steady windows give 0.898 (0.892 with the GPS lag corrected, half the spread). GPS positions
   against the speed integral give 0.898-0.901. This supports the 0.905 on the `p1-physics` branch.
2. **Flux.** A fit of ψ through zero gives the R·Iq drop to the flux: on a fake log with R = 80 mΩ it reads 7 % high.
   Steady windows cannot separate ψ from R (± 40 mΩ in R moves ψ by ± 4 %). On steady data the flux it implies falls
   with speed, the sign of an offset or of an R·Iq drop. A joint fit on all rows removes the bias when Iq is known.
3. **Extra battery draw.** With the plant's crr, CdA and R fixed, the unexplained power is about 200 W, or about
   340 W with the offset of item 4. Per band it is 150-270 W from 7 to 60 km/h (without the offset), and about 0 above
   60 km/h, where a tucked rider has a lower CdA. With every value free, the loss moves into crr = 0.058; both forms fit
   equally (cross-validation 403 W and 408 W). With the wind direction of the weather data and a wind of up to 2.5 m/s at rider height, the result changes by less
   than 20 W.
4. **Current offset.** With PWM exactly 0 at a standstill the battery current reads −0.73 A on average (−0.95 A in the
   first 5 minutes). A pack that is not charging cannot give a negative current. The moving rows cannot find the
   offset: at 121-132 V it looks like a constant loss.
5. **Current reading at high power.** The fitted inertia is 0.77-0.84 of the expected inertia (95 % range), the same
   when speeding up and when slowing down. The copper loss gives R_eff 14 mΩ (95 %: −4 to 31 mΩ) against 84 mΩ.
   Gradient boosting explains 14 % of the residual variance out of sample; the residual falls from +85 W at low power
   to −250 W at 3.7 kW. Read as the phase current, the fit gives half the inertia and a negative resistance, so the
   column is the battery current.
6. **Battery.** R0 + two RC parts (3 s, 30 s) give about 310 mΩ (error 0.34 V); one RC at 5 s gives 275 mΩ
   (95 %: 262-288). R0 alone is only 20-50 mΩ. If the current reads low (item 5), the true resistance is lower by the
   same factor.
7. **Temperature.** With the air temperature and the sun of the weather data, half of the 8.5 K rise comes from the
   sun and half from the current; time constant about 7.5 min.
   The column has 0.01 °C steps like the board NTC in frame `00`, not the whole-degree BMS values.
8. **Headroom at 80 % charge** (20,000 draws over ψ 17.0-18.5 mWb, R 40-100 mΩ, L 100-400 µH, pack 250-310 mΩ,
   crr 0.012-0.018, CdA 0.42-0.60, air 1.10-1.20, P0 150-350 W):
   - the 80 % PWM alarm comes at 79-85 km/h shown (5-95 %), with a true modulation of 82-104 %;
   - a further 1 m/s² request saturates the voltage below the alarm speed in 80 % of the
     draws (median 7 km/h below); with the plant's own values, 3 km/h below;
   - with the plant's values the model's 1 m/s² saturation speed (78 km/h shown) is close to the simulator's upright
     overlean near 77 km/h.
9. **Simulator comparison.** `tools/ride-log/cruise-flat.toml` on stock firmware with the `tiltback-69` settings
   (plant on `main`: 28 pole pairs, ψ 20.5 mWb, crr 0.012), exported and compared with `tools/ride-log/compare.py` (bands of true speed, 2 s windows):

   | Band (km/h) | Log PWM | Sim PWM | Log current | Sim current | Δ current | Δ flat power |
   | --- | --- | --- | --- | --- | --- | --- |
   | 20-25 | 26.0 % | 24.0 % | 3.10 A | 2.20 A | −0.90 A | −98 W |
   | 40-45 | 44.8 % | 44.0 % | 10.79 A | 7.67 A | −3.11 A | −302 W |
   | 50-55 | 55.7 % | 53.8 % | 14.07 A | 12.72 A | −1.35 A | −109 W |
   | 55-60 | 59.8 % | 59.1 % | 18.05 A | 16.22 A | −1.83 A | −164 W |

   The simulated rider is the default rider, not the rider of the log; on a level, steady cruise the mass changes the
   power only through rolling resistance.
10. **Wheel mass.** The plant's moving masses add up to 42 kg (wheel 16, shell 20, unsprung 6); the weighed wheel is
   39 kg.

## Limitations

- One ride at about 4 Hz. Many values are not identifiable from it; the ranges above are bootstrap ranges, not a full
  Bayesian posterior.
- The headroom model is steady-state: no dead time, no inverter drops, no balance dynamics. Its result depends on the
  inductance span, which comes from finding 10, not from data.
- The simulator comparison uses the plant on `main`. The values on the `p1-physics` branch and the values found here
  (pack resistance, extra loss) were not run.

## What would settle the open points

| Open point | Best evidence |
| --- | --- |
| Inductance and phase resistance (#37) | LCR meter on the phases |
| Current offset and reading scale | A raw BLE capture (#34) with phase and BMS currents; the calibration ride |
| crr, CdA, P0 apart | The calibration ride |

**Calibration ride** (D-optimal over 1-minute blocks, 15 minutes, at most 50 km/h shown): 2 min powered with the motor
not driven, 2 min standing still on the wheel, 2 min steady at 30 km/h and 1 min at 50 km/h on a flat road (both
directions), 1 min of speed-up / slow-down cycles around 20 km/h and 5 min around 35 km/h (0.7 m/s², ± 14 km/h), and
2 min steady at 25 km/h up an 8 % hill of measured slope. A 10 % low current reading only makes crr, CdA and P0 10 %
lower on flat ground; speed changes see the inertia, but its rotating part is unknown; a climb has a known coefficient.
At 150 W of noise per sample (an assumption) the plan gives the current scale to ± 8 % (95 %), crr ± 0.006,
CdA ± 0.10 m² and P0 ± 43 W.
