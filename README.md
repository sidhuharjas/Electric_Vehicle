# Electric Vehicle C — Autonomous Drive Controller

Firmware and simulator for a Science Olympiad **Electric Vehicle (EV-C)** robot.

This repo contains the onboard control firmware that actually drives the robot, and a Python simulator that mirrors the same control logic so the drive parameters can be tuned and visualized before ever touching hardware.

## The event

Science Olympiad EV-C is a build-and-drive event. The rules the controller is designed around:

- The vehicle launches from a start line and drives a **straight-line distance to a target**.
- It has **no onboard distance sensing** beyond wheel encoders and an IMU — no beacons, no external references, no line to follow.
- It can optionally route out to a **bonus-can offset** partway through the run for extra points.
- It has to **stop as close as possible to both a target distance and a target time** — not just distance alone.
- All of the above has to happen **repeatably, run after run** — a single lucky run doesn't count for much; the score is the vehicle's worst run as much as its best one.

Everything in `src/main.cpp` and the matching logic in `sim.py` exists to satisfy those five constraints at once, which is what most of the tuning surface below is actually fighting for.

![EV-C vehicle](demo/evc.png)

## How it works

The controller runs a single closed-loop routine (`driveDistanceMeters`) once per run:

- **Forward distance** is tracked from dual wheel encoders (`enc1`/`enc2`), converted from degrees to meters using wheel diameter.
- **Heading** is tracked primarily from IMU yaw-rate integration (with a low-pass filter and deadband to reject drift/noise), with a wheel-odometry fallback.
- **Lateral position** is reconstructed by integrating forward travel against heading, so the controller always knows where it is relative to a straight line to the target.
- **S-curve bonus-can routing** (optional): a sine-based heading/lateral profile (`computeArcHeading` / `computeArcTargetLateral`) steers the vehicle out to a lateral offset and back to center by the finish, with the arc fading out near the end so a separate recenter controller takes over for the final approach.
- **Speed profile**: proportional accel/decel zones scaled to the target distance, a low-power "creep" zone near the finish for fine stopping accuracy, and a time-scaling term that nudges overall power up or down to hit a target finish time without blowing past the distance target.
- **Motor balancing**: encoder-based speed matching between the two drive motors, plus heading-error correction, combine into the final differential power sent to each motor.

At the end of every run, the firmware prints a telemetry summary over serial (final heading, drift, forward/lateral position, run time, etc.) for tuning between runs.

## Engineering approach

### Why the simulator exists

The firmware runs open-loop on a robot with no ground-truth position feedback — everything it "knows" about where it is comes from integrating noisy encoder and gyro readings. That makes bad tuning expensive to discover: a wrong gain doesn't show up as an error message, it shows up as the robot missing the target by half a meter on a field, with battery, run count, and setup time all limited on competition day.

`sim.py` was built to break that loop. It re-implements the exact same integration math as `main.cpp` — same accel/decel fractions, same S-curve equations, same lateral-error controller, same time-scaling — in Python, and animates the resulting path with matplotlib. That turns "flash firmware → walk to the field → run it → guess why it drifted → repeat" into "change a constant → rerun a script → watch the trajectory redraw in real time." Parameter sweeps that would take a full afternoon of physical runs (arc angle, offset, gains, power curves) take minutes in the simulator, and because the two implementations share the same control structure line-for-line, a setting validated in `sim.py` transfers directly to `main.cpp` instead of needing to be rediscovered on hardware.

The simulator also models sources of error the firmware has to fight blind: `wheel_speed_noise_mps`, per-side `left_scale`/`right_scale` mismatch, and a fixed `seed` for repeatable noise — so a control strategy isn't just tuned to a clean, idealized path, it's tuned to survive the kind of asymmetry a real motor pair actually has.

### Sensor fusion: why IMU-primary, encoder-fallback

Pure wheel odometry (`heading = (d_right − d_left) / wheelbase`) is available and used as the fallback path, but it silently degrades the moment one wheel slips — which is common under acceleration on a light robot. The IMU's yaw rate doesn't care about wheel slip, so it's used as the primary heading source (`USE_IMU_HEADING`), with two corrections layered on:

- **Bias calibration at the start of every run** — 200 samples are taken before launch, outliers beyond `GYRO_CAL_OUTLIER_DPS` are rejected, and the mean of what's left becomes `gyroBias`. Gyros drift run to run with temperature and battery voltage, so this is recomputed every launch rather than hardcoded once.
- **Low-pass filtering + deadband** (`GYRO_LPF_ALPHA`, `GYRO_DEADBAND_DPS`) — raw yaw rate is noisy enough that integrating it directly accumulates drift from sensor noise alone. The filter smooths it; the deadband stops near-zero noise from integrating into fake heading error while the robot is actually driving straight.

Forward and lateral position are then reconstructed by resolving encoder-measured forward travel through the tracked heading (`forwardX += d_center·cos(heading)`, `lateralY += d_center·sin(heading)`) — a simple dead-reckoning model, but one that's only as good as the heading estimate feeding it, which is why the heading pipeline gets the most tuning attention.

This is what that heading pipeline actually buys you — on a treadmill (wheels driven, zero net forward travel), the vehicle holds a straight heading with no lateral drift instead of walking off to one side, which is the whole point of the bias calibration + filtering above:

![EV heading test on treadmill — tracks straight with no drift](demo/EVonTreadmill.gif)

### S-curve arc, by the numbers

The bonus-can detour is generated from two sine-based envelopes rather than a fixed waypoint path, so it scales automatically to any target distance:

```
heading(progress) = sin(2π·progress) · sin(π·progress) · ARC_MAX_ANGLE · returnBoost · DRIVE_SIDE
lateral(progress)  = DRIVE_SIDE · ARC_TARGET_OFFSET_M · sin(progress^shape_exp · π)
```

`progress` is forward distance traveled as a fraction of target distance (0→1), not time — so the arc shape stays consistent whether the robot is running fast or slow that particular heat. The `sin(π·progress)` envelope forces both curves to 0 at the start and finish by construction, so the vehicle always launches and lands pointed straight without needing a separate case for "beginning of arc" vs "end of arc." `ARC_LATERAL_SHAPE_EXP` shifts where the lateral peak falls (1.0 = exact midpoint); `returnBoost` adds extra correction authority in the second half so the vehicle doesn't just mirror its outbound drift but actively fights back to center.

Near the finish, the arc is deliberately faded out (`endBlend`, `END_CENTER_START`) and handed off to a dedicated recenter term (`END_CENTER_BOOST`, `END_HEADING_DAMP`) — the S-curve is good at shaping a smooth detour, but it's not designed to reject the specific error the robot actually accumulated on that particular run, so a separate, higher-authority controller takes over for the last stretch where finishing position matters most.

### Speed profile & time-scaling

Accel/decel zones are defined as **fractions of target distance** (`ACCEL_FRAC`, `DECEL_FRAC`) rather than fixed meters, so the same profile shape holds whether the target is 7 m or 10 m instead of needing re-tuning per distance. Layered on top is a proportional time-scaling controller:

```
desiredProgress = elapsedTime / targetTime
timeScale = clamp(1 + Kp · (desiredProgress − actualProgress), min, max)
```

This nudges overall power up if the run is behind schedule and down if it's ahead — letting the vehicle target a specific finish *time* (a scored quantity in EV-C) without abandoning the distance-based accel/decel/creep shaping that keeps the stop accurate. `timeScale` is clamped (`TIME_SCALE_MIN/MAX`) and power is hard-capped (`MAX_SAFE_POWER`) so the correction can't turn into an aggressive speed-up that would hurt stopping precision or tip-over stability — a deliberate trade of a little time accuracy for repeatability.

### Why this much tuning surface

Nearly every constant in `main.cpp` exists because an earlier, simpler version of that logic failed in a specific, observed way — e.g. `ENC_DEADBAND_DEG` and `GYRO_DEADBAND_DPS` exist because sensor noise alone was enough to accumulate false drift at rest; the launch `GO_RELEASE_SETTLE_MS` delay exists because the button press itself was injecting IMU bias before the run even started. The simulator is what makes carrying that many interacting parameters tractable — instead of changing one constant and hoping nothing else broke, every change gets visually re-validated against the full trajectory before it goes anywhere near the hardware.

## Repo layout

```
src/main.cpp     Onboard firmware (PlatformIO / Arduino framework)
sim.py           Python simulator — mirrors main.cpp's control logic offline
platformio.ini   Board/build configuration
demo/            Demo images/gif (full video hosted on YouTube — see Demo section)
include/, lib/   PlatformIO project scaffolding
test/            PlatformIO test scaffolding
```

## Hardware

- **MCU:** Raspberry Pi Pico 2 (`rpipico2`, Arduino framework via `earlephilhower` core)
- **Drive library:** [TektiteRotEv](https://github.com/TektiteBiz/TektiteRotEv) — motor, encoder, IMU, button, and LED interface for the Tektite EV board
- Dual drive motors with quadrature encoders, onboard IMU for yaw-rate sensing, go/stop buttons, and an RGB status LED (yellow = standby, blue = running, red = finished)

## Firmware — build & flash

Built with [PlatformIO](https://platformio.org/).

```bash
# Build
pio run -e pico

# Flash to a connected Pico 2
pio run -e pico -t upload

# Serial monitor (telemetry output)
pio device monitor -b 115200
```

Static analysis (cppcheck + clang-tidy) is configured and can be run with:

```bash
pio check -e pico
```

### Competition-day tuning

All the parameters you'd actually change between events live at the top of `src/main.cpp`:

| Parameter | Purpose |
|---|---|
| `TARGET_DISTANCE_M` | Final target distance for the run |
| `TARGET_RUN_TIME_S` | Target completion time (10–20s, snapped to 0.5s) |
| `useArc`, `ARC_MAX_ANGLE`, `ARC_TARGET_OFFSET_M` | Enable/shape the bonus-can S-curve |
| `basePower`, `minPower` | Drive power floor/ceiling |
| `DRIVE_SIDE` | Which side of center the bonus-can arc drives toward (`1` = left, `-1` = right) |

The vehicle arms on a GO button press and launches after release + a short settle delay (to avoid button-press IMU bias), and can be stopped early with the stop button.

## Simulator

`sim.py` re-implements the same control loop in Python and animates the resulting trajectory in real time with matplotlib, so tuning changes (arc shape, gains, power curves) can be visualized without deploying to hardware first.

```bash
pip install matplotlib

# Run with defaults
python3 sim.py

# Example: tune a run with the bonus-can arc enabled
python3 sim.py --distance 8.4 --arc --arc-max-angle 7 --arc-offset 0.86 --target-time 14 --speedup 3
```

Key flags: `--distance`, `--base-power`, `--min-power`, `--arc` / `--no-arc`, `--arc-max-angle`, `--arc-offset`, `--arc-shape-exp`, `--drive-side`, `--target-time`, `--no-time-scaling`, `--speedup`, `--seed`. Run `python3 sim.py --help` for the full list.

## Demo

Full run video: **[watch on YouTube](https://youtu.be/6F0KFUp0sm0)** *(112MB — too large for a plain GitHub file; hosted unlisted on YouTube instead)*

[![Watch the demo](https://img.youtube.com/vi/6F0KFUp0sm0/0.jpg)](https://youtu.be/6F0KFUp0sm0)

## Status

Actively tuned between competitions — parameter values in `main.cpp` reflect the most recent event setup, not necessarily the final/optimal configuration.
