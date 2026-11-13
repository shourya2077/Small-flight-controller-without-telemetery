# RocketSimple – single-board parachute ejection controller

ESP32-C3 Super Mini · BMP280 · MPU6500 · 5 V relay (switching the 7.4 V pack) · WiFi dashboard on the pad only.
No radio telemetry, no ground station.

## Install
1. Arduino IDE → Board: **ESP32C3 Dev Module** (esp32 core 2.0.x or 3.x). *USB CDC On Boot: Enabled* if you want serial.
2. Open the `RocketSimple` folder (all files in one folder, sketch = `RocketSimple.ino`). **No extra libraries needed** (BMP280/MPU6500 drivers are built in).
3. Check pins in `Config.h`, upload.
4. Phone → WiFi `RocketFC` / `rocket1234` → browser `http://192.168.4.1`.

## Default wiring (change in Config.h)
| Signal | Pin |
|---|---|
| I2C SDA / SCL (both sensors, 3.3 V) | GPIO6 / GPIO7 |
| Relay IN | GPIO5 |
| BMP280 SDO→GND (0x76), MPU6500 AD0→GND (0x68) | – |
| Optional launch-sense pin | `PIN_LAUNCH_SENSE` (disabled) |

### ⚠ Relay wiring & safety (read this before connecting an e-match)
**Use the NO (normally open) terminal – never NC.** COM–NC is *closed while the relay is OFF*, so an e-match on NC is live the moment
the battery is connected (no click, because the relay isn't doing anything) – this is exactly how an e-match gets lit by simply turning the battery on.

```
 Battery +  ──[ ARM SWITCH / removable jumper ]──  relay COM
                                                   relay NO  ──  e-match  ──  Battery –
 (relay coil: 5 V + GND from the buck converter, IN from the ESP GPIO, grounds common)
```
1. **Terminal check with a multimeter (battery OFF, ESP unpowered):** COM–NO must be OPEN (no beep). COM–NC beeps. Wire the e-match across COM/NO only.
2. **Arming switch in series** with the e-match supply (or a removable "remove-before-flight" jumper). Dashboard ARM is not a hardware safety.
3. **Never test with an e-match.** Use a dummy load (7.4 V bulb / LED + resistor, or a 2–5 Ω resistor) while checking power-up, reset, relay click and TEST EJECTION. Fit the e-match last, on the pad, with the arming switch OFF; keep its leads shorted until then.
4. **Find your module's polarity with the ESP unplugged:** touch IN to GND, then to 5 V, and listen for the click. "Clicks on GND" = active-LOW → set `RELAY_ACTIVE_HIGH 0` in Config.h. Wrong polarity = relay energised all the time. Many cheap opto boards are active-LOW.
5. Watch the relay (LED/click) through power-up and after pressing reset: it must stay OFF the whole time. If it ever blips, add a 10 kΩ pull resistor on IN (to the module's OFF level) or switch to a MOSFET driver with a pull-down.
6. Firing an e-match pulls several amps. If the ESP shares the battery via a buck converter, the dip can reset it – add 470–1000 µF at the buck output (or a separate ESP supply). The firmware survives a reset (flight record in RTC memory) but prevention is better.
7. Common ground between ESP, relay module and battery. Keep the charge/igniter disconnected on the bench; use DISARM.

## How it decides (any ONE fires; they are independent, not a vote)
| | Detector | Needs |
|---|---|---|
| A | Baro: altitude > 3 m below peak & descending, ~150 ms | baro |
| B | Tilt: gyro-integrated angle from pad-vertical > limit (110°) for 400 ms | IMU |
| C | IMU vertical speed below −(2 m/s + its uncertainty) for 150 ms (accel integrated along "up", blended with baro) | IMU |
| C2 | Free-fall: sensed accel < 0.3 g for 200 ms after burnout | IMU |
| D | **Backup timer** from launch | nothing |
| E | **Max timer** from launch (never held off) | nothing |

### "Eject AFTER apogee" – how early firing is prevented
* B, C, C2 are **held off while any working sensor says the rocket is still climbing** (baro climb > 3 m/s, or – with no baro – the IMU speed estimate minus its uncertainty).
* The IMU-only speed has a built-in error model (3 % of peak speed + 0.8 m/s per second since the baro was lost). Without a baro it therefore waits for stronger evidence of descent: it errs **late rather than early**.
* **Timer D is held off while a sensor says we are still climbing fast** (> 3 m/s baro, > 5 m/s IMU-only) – a too-short timer can't deploy at speed. Timer E and the "all sensors dead" case are never held off.
* Result in 8000 randomized flights: **0 early, 0 late, 0 missed** (median +0.15 s after apogee, 95 % within +1.1 s).

A–C2 are ignored for the first 1 s after launch (thrust/rail transients). D/E are checked first, every cycle.

**Launch detection**: accel > threshold for 100 ms (then confirmed by 250 ms of it *or* +5 m altitude, else discarded as a bump after 2 s) · baro +6 m & >4 m/s (cross-checked with IMU if alive) · optional launch pin · dashboard "blind timer".

## Failure matrix (tested: `sim/sim_test.cpp` 23 scenarios + `sim/mc_test.cpp` 1000-flight Monte-Carlo, many seeds)
| Failure | Result |
|---|---|
| Baro dead before launch | launch by accel; apogee by IMU velocity / free-fall / tilt / timers |
| IMU dead before launch | launch by baro; apogee by baro (≈1 s after apogee) / timers |
| One sensor dies in flight | other sensor + timers continue; dead one is re-initialised every 0.5 s (bus freed by 9-clock recovery) |
| **Both die in flight** | **backup timer** ejects |
| MCU resets / brown-out / watchdog in flight | flight clock, peaks and ejection state restored from RTC memory; timers continue; if reset hit during the fire pulse it is repeated |
| Baro single-sample spikes | median-3 + 50 m slew guard; no false launch/eject |
| Bump / handling on pad | rejected (needs sustained high-g + confirmation) |
| Relay pulse | 2 s ON, 1 s gap, 2 s ON again (`RELAY_REPEAT_PULSE`) |

### The one case no software can solve
**Both sensors dead BEFORE launch and no launch pin** → the firmware cannot know launch happened, so no timer can start.
It tells you (LED fast-blinks, dashboard NO-GO). Mitigations: fix the sensor, use the **launch-sense pin** (`PIN_LAUNCH_SENSE`; a
wire/jumper to GND that the rocket's departure pulls out → pin goes HIGH → flight clock starts), or use the dashboard **Blind timer**
(starts the clock N s after you press it – then close up and launch within that window).

## Pre-flight checklist
1. Power on **at the pad**, rocket vertical and still for ~5 s (gyro cal + pad pressure).
2. Dashboard banner **GO** (green). Fix any red ✖ item. Check baro noise/IMU noise chips are GOOD/OK, `|accel| ≈ 1 g`.
3. Set **Backup timer** a little after your *simulated* apogee time, **Max timer** a few s later. Save.
4. Bench (charge disconnected): **Relay click**, then **TEST EJECTION** – verify pulse, then reconnect charge.
5. Vent the avionics bay (static ports) – a sealed bay makes the baro read wrong. Keep the baro out of the airstream/motor ejection gases.
6. Dashboard WiFi turns **off at liftoff** and comes back ~8 s after ejection; read the peaks, press *Reset for next flight* before re-arming.
7. LED: short blip/2 s = armed & healthy · fast blink = a sensor is down · long on/off = disarmed · 1 Hz = ejected.

## Tuning notes
* Defaults: backup 12 s, max 20 s, tilt 110°, launch 3.0 g, pulse 2000 ms – **set to your rocket**.
* IMU accel ±16 g / gyro ±2000 °/s. If your motor exceeds ~14 g sustained, raise nothing – it simply saturates harmlessly (launch still detected).
* Any orientation of the IMU works for tilt (it measures angle from the pad's "up", whatever axis that is). Roll spin below ~2000 °/s is fine.
* Without a baro, free-fall/IMU-speed apogee can be up to ~0.5 s early; with baro only, ~1 s late. That's the price of independence.
* Tests (PC, no hardware): `cd sim && g++ -std=c++11 -O2 -I.. sim_test.cpp ../FlightCore.cpp -o sim && ./sim` and
  `g++ -std=c++11 -O2 -I.. mc_test.cpp ../FlightCore.cpp -o mc && ./mc 1000 <seed>` (random motors, IMU mounting, offsets/scale errors, noise, tumbling, sensor deaths) and
  `g++ -std=c++11 -O2 -I.. att_test.cpp ../FlightCore.cpp -o att && ./att` (signed angles: anticlockwise/clockwise, return to zero, uncalibrated gyro, no creep).

## IMU responsiveness
* Sensor path: MPU6500 filters ~92 Hz gyro / ~99 Hz accel (≈ 3–4 ms delay), read at 200 Hz.
* Dashboard path: a dedicated `/fast` endpoint (IMU + altitude, refreshed every 5 ms in firmware) is polled back-to-back by the page (typically 15–30 Hz) instead of the old 4 Hz poll; the **IMU live** panel shows accel/gyro bars, tilt, IMU speed, the **data age** (ms) and the display rate.
* **Signed Roll / Pitch / Yaw** (right-hand rule about the chip's X/Y/Z: anticlockwise = +, clockwise = −; for a board lying flat, Z up, anticlockwise seen from above = +yaw). The arrow follows the axis you pick; "flip direction" handles a board mounted upside-down.
* Angles start at **0** at power-on (reference latches once the board has been still for 1 s and the gyro has settled), return to 0 when you put the board back (roll/pitch are pulled back to gravity at rest; yaw is frozen at rest so it can't creep), and **Zero angles** re-zeroes at the current pose. An uncalibrated gyro offset (up to ~12 dps) is learned automatically.
* **Display smoothing** (`DISPLAY_TAU_MS`, default 60 ms) affects the dashboard numbers only; the flight logic always uses the raw 200 Hz data. Sensor filters: gyro ≈ 92 Hz, accel ≈ 99 Hz.
* WiFi power-save is off for lower, steadier latency.

## Notes from the code audit
* "Still" detection no longer needs |a| = 1.00 g (stock MPU6500s have up to ±80 mg offset); the pad reference now always latches, and IMU speed uses the measured rest value as 1 g.
* A flight record in RTC memory is valid only for the firmware build that wrote it, so re-flashing can never resurrect an old flight; a power cycle also clears it.
* Web handlers are registered once (the radio restarts after every flight).
