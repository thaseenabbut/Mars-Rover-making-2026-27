# Mars Rover Firmware

Firmware for a 6-wheel skid-steer rover, built for a college Mars rover
competition. Runs on an ESP32-S3 and supports both manual BLE-controlled
driving and fully autonomous, sensor-based obstacle avoidance, switchable
live over the same BLE link.

## Competition context

The competition has two stages:
1. **Open maze** — visible and driven manually.
2. **Cave** — the same maze, but enclosed with no visibility, requiring
   autonomous navigation using onboard sensors (limited manual commands are
   still allowed as a fallback if the rover gets stuck).

## Hardware

- ESP32-S3 (DevKitC-1)
- 6x 12V DC gearmotors with encoders, 300RPM, regular (non-mecanum) wheels
- 2x L298N motor drivers — one drives 4 motors (front-left + mid-left
  paired on one channel, front-right + mid-right paired on the other), the
  other drives the 2 independent rear motors
- 3x HC-SR04 ultrasonic sensors (left / center / right) for obstacle
  detection
- MPU6050 IMU
- 11.1V 3S LiPo battery + LM2596 buck converter (5V logic rail)
- Control app: a generic BLE controller app (Nordic UART Service),
  sending single-character commands

## Driving modes

The rover boots into **Manual** mode by default. Mode is switched live over
BLE and persists until changed again.

| Command | Mode    | Effect                                          |
|---------|---------|--------------------------------------------------|
| `F`/`f` | Manual  | Drive forward (latched until a new command)       |
| `B`/`b` | Manual  | Drive backward (latched until a new command)      |
| `L`/`l` | Manual  | Pivot left (latched until a new command)          |
| `R`/`r` | Manual  | Pivot right (latched until a new command)         |
| `S`/`s` | Both    | Stop immediately                                  |
| `M`/`m` | —       | Force Manual mode                                 |
| `A`/`a` | —       | Force Autonomous mode                             |

**Manual mode:** commands are latching — send `F` once and the rover keeps
driving forward until a different command is received. If the BLE
connection drops entirely, the rover stops immediately regardless of the
last command.

**Autonomous mode:** drive commands are ignored (`S` still works as an
emergency stop). The rover continuously reads all three ultrasonic sensors
and steers proportionally toward whichever side has more open space,
scaling speed down smoothly as obstacles get closer, with gradual
(non-jumpy) PWM changes rather than a stop-detect-then-turn approach.

Live telemetry (sensor distances, encoder counts, IMU data) streams over
BLE continuously in both modes.

## Building / flashing

Written in C++ using the Arduino framework, consolidated into a single
`main.cpp` (required by the competition submission format). Flash with the
Arduino IDE or PlatformIO targeting an ESP32-S3 board.

## Project status

Actively being built and tested for the competition. Expect the pin
layout, thresholds, and tuning values to change as testing progresses.

## License

MIT — see [LICENSE](LICENSE). Use it however you like.