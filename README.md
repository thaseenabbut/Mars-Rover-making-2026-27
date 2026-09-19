# Mars Rover Firmware

Firmware for a 6-wheel skid-steer rover, built for a college Mars rover
competition. Runs on an ESP32-S3 and supports both manual BLE-controlled
driving and fully autonomous, sensor-based obstacle avoidance, switchable
live over the same BLE link.

## Competition context

The competition is structured as a race with three challenge rounds:

1. **Terrain traverse** — drive across large rocks, then pebbles, followed by
   small hills. The final hill is particularly rough and tests traction and
   chassis stability.
2. **Ramp traverse** — climb to the top of a ramp and descend safely.
3. **Autonomous cave navigation** — climb a ramp, enter an obstacle-filled
   cave, navigate through it using the onboard sensors, and exit the cave.

## Hardware

- ESP32-S3 (DevKitC-1)
- 6x 12V DC gearmotors with encoders, 300RPM, regular (non-mecanum) wheels
- 2x L298N motor drivers — Driver 1 drives the paired front-left + middle-left
  wheels on one channel and front-right + middle-right wheels on the other;
  Driver 2 drives the independent rear-left and rear-right wheels
- 4x HC-SR04 ultrasonic sensors (left / front / right / rear) for obstacle
  detection and safe reversing in autonomous mode
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
emergency stop). The rover continuously reads all four ultrasonic sensors.
When the front is blocked, it stops briefly, scans for the clearest side, and
turns left or right. If the front and both sides are blocked, it makes a short
rear-sensor-checked reverse movement, then scans again rather than reversing
continuously.

Live telemetry (sensor distances and encoder counts) streams over
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
