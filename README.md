# Mars Rover Firmware

Firmware for a six-wheel skid-steer Mars rover built for a college rover
competition. An ESP32-S3 provides manual Bluetooth Low Energy (BLE) driving
and a four-ultrasonic-sensor autonomous mode over the same BLE connection.

> This project controls real motors. Test with the rover raised off the ground
> first, keep an emergency power cutoff available, and do not rely on autonomous
> mode until every sensor has been verified.

## Competition context

The competition is a race with three challenge rounds:

1. **Terrain traverse** — large rocks, pebbles, small hills, and a particularly
   rough final hill test traction and chassis stability.
2. **Ramp traverse** — climb to the top of a ramp and descend safely.
3. **Autonomous cave navigation** — climb a ramp, enter an obstacle-filled
   cave, navigate through it with onboard sensors, and exit the cave.

## Features

- Six-wheel differential/skid-steer drive
- Manual BLE control through Nordic UART Service (NUS)
- Four HC-SR04 ultrasonic sensors: left, front, right, and rear
- Autonomous stop, scan, turn, and short-reverse behavior
- BLE telemetry for live sensor distances
- Immediate motor stop on BLE disconnect or `S` command

## Hardware architecture

The rover uses two L298N dual H-bridge motor drivers and four logical motor
groups. The two motors in each front/middle group are wired in parallel.

```text
                         ESP32-S3 DevKitC-1
                                  |
           +----------------------+----------------------+
           |                                             |
    L298N Driver 1                                L298N Driver 2
    Front + middle wheels                          Rear wheels
    OUT1/OUT2 -> front-left + middle-left          OUT1/OUT2 -> rear-left
    OUT3/OUT4 -> front-right + middle-right        OUT3/OUT4 -> rear-right
```

### Motor-control wiring

Connect the ESP32 GPIOs to the L298N input pins as follows. `ENA` and `ENB`
jumper caps are assumed to be fitted, so the current firmware uses direction
control only; it does not provide PWM speed control.

```text
Driver 1 — front + middle wheels
  IN1 -> GPIO 2       IN2 -> GPIO 1
  IN3 -> GPIO 6       IN4 -> GPIO 7

Driver 2 — rear wheels
  IN1 -> GPIO 11      IN2 -> GPIO 10
  IN3 -> GPIO 12      IN4 -> GPIO 13
```

If `F` moves an individual wheel in the wrong physical direction, turn off
motor power and swap that motor's two output wires on its L298N channel. Do
not swap GPIO input wires while diagnosing a single wheel.

### Ultrasonic-sensor wiring

```text
Sensor    TRIG GPIO    ECHO GPIO    Mounting direction
Left      38           39           Left side
Front     40           41           Forward
Right     42           36           Right side
Rear      15           16           Backward
```

### Power and electrical safety

- Use the 11.1 V 3S LiPo only for the motor-driver motor supply.
- Use a regulated 5 V supply, such as an LM2596 buck converter, for the
  sensor/logic rail as appropriate for the hardware.
- Connect the ESP32 ground, both L298N grounds, sensor grounds, and battery
  negative to a **common ground**.
- An HC-SR04 `ECHO` pin can output 5 V. Protect every ESP32 input with a
  logic-level shifter or resistor voltage divider; ESP32 GPIOs are 3.3 V only.
- L298N boards can run hot and lose substantial voltage under load. Stop testing
  if a driver overheats, a motor stalls, or the power wiring becomes warm.

## Software architecture

The firmware is intentionally contained in [`src/main.cpp`](src/main.cpp).

- `Motors` translates left/right movement commands into L298N direction pins.
- `UltrasonicSensors` samples all four HC-SR04 sensors and exposes distances in
  centimetres.
- BLE NUS receives one-character commands and publishes sensor telemetry.
- Manual mode directly applies the selected drive command.
- Autonomous mode runs a small state machine: forward -> stop/scan -> turn or
  short reverse -> scan again.

The rear sensor is checked during every reverse movement. A missing sensor
returns no echo and is currently treated as open space by the navigation code,
so all four sensors must be connected and tested before autonomous operation.

## Prerequisites

- ESP32-S3 DevKitC-1
- PlatformIO Core or the PlatformIO extension for VS Code
- USB data cable for flashing the ESP32-S3
- A BLE client capable of writing text to Nordic UART Service

The project target is configured in [`platformio.ini`](platformio.ini):

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
```

## Setup and flashing

1. Clone the repository and open the folder in VS Code with PlatformIO, or use
   a terminal in the project directory.
2. Complete the motor, sensor, power, and common-ground wiring above.
3. Build the firmware:

   ```sh
   platformio run
   ```

4. Connect the ESP32-S3 over USB and upload:

   ```sh
   platformio run --target upload
   ```

5. Open the serial monitor at 115200 baud:

   ```sh
   platformio device monitor --baud 115200
   ```

6. Connect a BLE NUS client to the rover, advertised as `RedRovers`.

## BLE controls

The rover starts in **Manual** mode. Commands are single characters and are
case-insensitive.

| Command | Available in | Action |
| --- | --- | --- |
| `F` | Manual | Drive forward until another command arrives |
| `B` | Manual | Drive backward until another command arrives |
| `L` | Manual | Pivot left |
| `R` | Manual | Pivot right |
| `S` | Manual / Auto | Stop immediately |
| `M` | Any time | Enter Manual mode and stop |
| `A` | Any time | Enter Autonomous mode |

If the BLE connection drops, the firmware immediately stops the motors and
returns to advertising.

## Autonomous behavior

After receiving `A`, the rover:

1. Drives forward while the front path is clear.
2. Stops for 500 ms when the front sensor detects an obstacle within 35 cm.
3. Scans left, front, right, and rear; then pivots toward the clearest open
   side.
4. If front, left, and right are blocked but the rear is clear, reverses for a
   short 450 ms burst and scans again.
5. Limits itself to two consecutive reverse bursts before choosing the less
   blocked side. If all four directions are blocked, it remains stopped and
   rescans.

Tune these values in `src/main.cpp` for the real course: `AUTO_BLOCKED_DIST_CM`,
`AUTO_SCAN_PAUSE_MS`, `AUTO_TURN_TIME_MS`, and `AUTO_REVERSE_TIME_MS`.

## First-test checklist

1. Raise all wheels off the ground.
2. In Manual mode, verify `F`, `B`, `L`, `R`, and `S` before driving on the
   floor.
3. Confirm every sensor reports a sensible value in the serial monitor by
   placing a hand in front of it. Telemetry uses `L`, `C`, `R`, and `S4` for
   left, front/centre, right, and rear.
4. Test `A` in a clear, low-speed area with a spotter and a power cutoff.
5. Tune the autonomous thresholds only after the basic stop/turn/reverse
   sequence behaves reliably.

## Project status

The drivetrain and manual BLE control are working and are the primary tested
control path. Four-sensor autonomous navigation is implemented for the cave
round and is actively being field-tuned. Thresholds, turn durations, sensor
placement, and motor-driver capacity must be validated on the competition
terrain before relying on autonomous mode.

## License

MIT — see [LICENSE](LICENSE).
