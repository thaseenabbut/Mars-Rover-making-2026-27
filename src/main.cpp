/*******************************************************************************
 * Mars Rover — Single-file firmware for ESP32-S3-DevKitC-1
 *
 * Features:
 *   • 4 motor groups across 2x L298N drivers (differential drive)
 *     - Driver 1 ch A: front-left + mid-left motors in parallel  ("LF group")
 *     - Driver 1 ch B: front-right + mid-right motors in parallel ("RF group")
 *     - Driver 2 ch A: rear-left motor                           ("RL group")
 *     - Driver 2 ch B: rear-right motor                          ("RR group")
 *   • MPU6050 IMU over I2C (telemetry)
 *   • 3x HC-SR04 ultrasonic sensors (telemetry display only)
 *   • BLE Nordic UART Service (NUS) — single-char commands F/B/L/R/S
 *   • 500 ms command-timeout auto-stop
 *
 * Pin budget (ESP32-S3-DevKitC-1, N8 — no PSRAM):
 *   Avoid: GPIO 0,3,45,46 (strapping), 19,20 (USB), 43,44 (UART0),
 *          26-32 (not exposed / SPI flash), 33-37 (octal flash)
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS  (TEMP: 2-driver / 4-group wiring for ESP32 heat diagnosis)
// ═══════════════════════════════════════════════════════════════════════════════

// ---- Driver 1, Channel A — LF group (front-left + mid-left in parallel) ----
//   Physical wires: L298N-1 OUT1 -> FL motor + ML motor  |  OUT2 -> FL motor + ML motor
#define LF_IN1   1
#define LF_IN2   2
#define LF_EN    4      // LEDC channel 0

// ---- Driver 1, Channel B — RF group (front-right + mid-right in parallel) ----
//   Physical wires: L298N-1 OUT3 -> FR motor + MR motor  |  OUT4 -> FR motor + MR motor
#define RF_IN1   6
#define RF_IN2   7
#define RF_EN    5      // LEDC channel 1

// ---- Driver 2, Channel A — RL group (rear-left, independent) ----
//   Physical wires: L298N-2 OUT1 -> RL motor  |  OUT2 -> RL motor
#define RL_IN1   10
#define RL_IN2   11
#define RL_EN    14     // LEDC channel 2

// ---- Driver 2, Channel B — RR group (rear-right, independent) ----
//   Physical wires: L298N-2 OUT3 -> RR motor  |  OUT4 -> RR motor
#define RR_IN1   12
#define RR_IN2   13
#define RR_EN    21     // LEDC channel 3

// ---- I2C (MPU6050) ----
#define I2C_SDA  47
#define I2C_SCL  48

// ---- HC-SR04 Ultrasonic Sensors ----
#define SENSOR_LEFT_TRIG    38
#define SENSOR_LEFT_ECHO    39
#define SENSOR_CENTER_TRIG  40
#define SENSOR_CENTER_ECHO  41
#define SENSOR_RIGHT_TRIG   42
#define SENSOR_RIGHT_ECHO   36

// NOTE on GPIO36: safe on N8 (no PSRAM). If board is N8R8, change to another free pin.

// ---- LEDC PWM settings ----
#define LEDC_FREQ       1000
#define LEDC_RESOLUTION 8       // 0-255

// ═══════════════════════════════════════════════════════════════════════════════
//  STRAPPING PIN CHECK
// ═══════════════════════════════════════════════════════════════════════════════

static void checkStrappingPins() {
  // Check only the EN (PWM) pins — the ones most likely to cause boot issues.
  const int enPins[]   = { LF_EN, RF_EN, RL_EN, RR_EN };
  const char* enNames[] = { "LF_EN", "RF_EN", "RL_EN", "RR_EN" };
  const int strapping[] = { 0, 3, 45, 46 };
  bool anyFlag = false;

  for (int i = 0; i < 4; i++) {
    for (int s = 0; s < 4; s++) {
      if (enPins[i] == strapping[s]) {
        Serial.printf("  WARNING: %s (GPIO %d) is a strapping pin!\n",
                       enNames[i], enPins[i]);
        anyFlag = true;
      }
    }
  }
  if (!anyFlag) {
    Serial.println("  No EN pins on strapping GPIOs (0, 3, 45, 46).");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTORS CLASS
// ═══════════════════════════════════════════════════════════════════════════════

class Motors {
public:
  bool begin() {
    Serial.println("[Motors] Initialising direction pins...");

    // LF group (D1-A): front-left + mid-left in parallel
    pinMode(LF_IN1, OUTPUT); pinMode(LF_IN2, OUTPUT);
    // RF group (D1-B): front-right + mid-right in parallel
    pinMode(RF_IN1, OUTPUT); pinMode(RF_IN2, OUTPUT);
    // RL group (D2-A): rear-left
    pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
    // RR group (D2-B): rear-right
    pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

    Serial.println("[Motors] Configuring LEDC PWM channels...");

    // ESP32-S3 Arduino core >= 3.x: new ledcAttach() API.
    // Arduino core 2.x (this build): legacy ledcSetup()/ledcAttachPin().
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttach(LF_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: LF_EN"); return false; }
    if (!ledcAttach(RF_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: RF_EN"); return false; }
    if (!ledcAttach(RL_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: RL_EN"); return false; }
    if (!ledcAttach(RR_EN, LEDC_FREQ, LEDC_RESOLUTION)) { Serial.println("  LEDC attach failed: RR_EN"); return false; }
#else
    // 4 channels instead of the previous 6
    ledcSetup(0, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(LF_EN, 0);
    ledcSetup(1, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(RF_EN, 1);
    ledcSetup(2, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(RL_EN, 2);
    ledcSetup(3, LEDC_FREQ, LEDC_RESOLUTION); ledcAttachPin(RR_EN, 3);
#endif

    Serial.println("  All 4 LEDC channels attached.");
    stop();
    return true;
  }

  // Set speed for left side and right side independently.
  //   speed: -255..+255  (positive = forward, negative = backward)
  //   Left side  = LF group (D1-A) + RL group (D2-A)
  //   Right side = RF group (D1-B) + RR group (D2-B)
  void setSpeed(int left, int right) {
    setMotor(LF_IN1, LF_IN2, LF_EN, left);
    setMotor(RL_IN1, RL_IN2, RL_EN, left);

    setMotor(RF_IN1, RF_IN2, RF_EN, right);
    setMotor(RR_IN1, RR_IN2, RR_EN, right);
  }

  void forward(int spd = 200)  { setSpeed( spd,  spd); }
  void backward(int spd = 200) { setSpeed(-spd, -spd); }
  void left(int spd = 180)     { setSpeed(-spd,  spd); }   // pivot left
  void right(int spd = 180)    { setSpeed( spd, -spd); }   // pivot right
  void stop()                   { setSpeed(0, 0); }

private:
  void setMotor(int in1, int in2, int enPin, int speed) {
    if (speed > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (speed < 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    int pwm = constrain(abs(speed), 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(enPin, pwm);
#else
    int ch = pinToChannel(enPin);
    if (ch >= 0) ledcWrite(ch, pwm);
#endif
  }

#if ESP_ARDUINO_VERSION_MAJOR < 3
  int pinToChannel(int pin) {
    if (pin == LF_EN) return 0;
    if (pin == RF_EN) return 1;
    if (pin == RL_EN) return 2;
    if (pin == RR_EN) return 3;
    return -1;
  }
#endif
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ULTRASONIC SENSORS CLASS
// ═══════════════════════════════════════════════════════════════════════════════

class UltrasonicSensors {
public:
  float distLeft   = -1;
  float distCenter = -1;
  float distRight  = -1;

  void begin() {
    Serial.println("[Sensors] Configuring HC-SR04 pins...");
    Serial.printf("  LEFT   TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_LEFT_TRIG, SENSOR_LEFT_ECHO);
    Serial.printf("  CENTER TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_CENTER_TRIG, SENSOR_CENTER_ECHO);
    Serial.printf("  RIGHT  TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_RIGHT_TRIG, SENSOR_RIGHT_ECHO);

    pinMode(SENSOR_LEFT_TRIG,   OUTPUT);
    pinMode(SENSOR_LEFT_ECHO,   INPUT);
    pinMode(SENSOR_CENTER_TRIG, OUTPUT);
    pinMode(SENSOR_CENTER_ECHO, INPUT);
    pinMode(SENSOR_RIGHT_TRIG,  OUTPUT);
    pinMode(SENSOR_RIGHT_ECHO,  INPUT);

    Serial.println("  Sensor pins configured.");
  }

  void readAll() {
    distLeft   = readSensor(SENSOR_LEFT_TRIG,   SENSOR_LEFT_ECHO);
    distCenter = readSensor(SENSOR_CENTER_TRIG,  SENSOR_CENTER_ECHO);
    distRight  = readSensor(SENSOR_RIGHT_TRIG,   SENSOR_RIGHT_ECHO);
  }

  void printTelemetry() {
    Serial.printf("  DIST L=%.1fcm  C=%.1fcm  R=%.1fcm\n",
                   distLeft, distCenter, distRight);
  }

private:
  float readSensor(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 30000);  // 30 ms timeout
    if (duration == 0) return -1.0f;  // no echo / out of range
    return (float)duration * 0.0343f / 2.0f;   // cm
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  HARDWARE INSTANCES & DRIVE MODE DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════

Motors            motors;
UltrasonicSensors sensors;

// Driving mode state: MANUAL (default on boot) or AUTONOMOUS
enum DriveMode {
  MODE_MANUAL,
  MODE_AUTONOMOUS
};

static volatile DriveMode currentMode = MODE_MANUAL;

// ---- Autonomous Mode Tuning Parameters ----
// Distance thresholds (in centimeters):
static const float AUTO_HARD_STOP_DIST_CM  = 20.0f; // Hard emergency stop if center obstacle is closer than this
static const float AUTO_SLOWDOWN_DIST_CM   = 80.0f; // Distance where forward speed begins scaling down
static const float AUTO_MAX_SENSE_DIST_CM  = 150.0f;// Distances beyond this (or no echo) are treated as wide open space

// Speed limits (PWM: 0 to 255):
static const float AUTO_CRUISE_SPEED       = 180.0f;// Maximum forward speed in clear open space
static const float AUTO_MIN_SPEED          = 85.0f; // Minimum forward speed near obstacles (overcomes friction)

// Steering sensitivity:
static const float AUTO_STEER_GAIN         = 1.2f;  // Multiplier scaling left vs right distance difference to steering bias

// Frame-to-frame PWM slew rate (ramp limit):
static const float AUTO_MAX_PWM_SLEW       = 8.0f;  // Max change in PWM per loop cycle (smooth ramps, prevents current spikes)

// Slew rate state tracking for smooth acceleration/deceleration
static float currentLeftPwm  = 0.0f;
static float currentRightPwm = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════════
//  BLE NUS (Nordic UART Service)
// ═══════════════════════════════════════════════════════════════════════════════

// Standard NUS UUIDs
#define NUS_SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_CHAR_UUID        "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // write (app → rover)
#define NUS_TX_CHAR_UUID        "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // notify (rover → app)

static BLECharacteristic* pTxCharacteristic = nullptr;
static bool               bleConnected      = false;
static volatile char      lastCommand       = 'S';
static volatile unsigned long lastCommandTimeMs = 0;

// ---- Server callbacks ----
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConnected = true;
    Serial.println("[BLE] Client connected!");
  }
  void onDisconnect(BLEServer* pServer) override {
    bleConnected = false;
    // Failsafe: immediate full stop if BLE link drops, clearing any latched drive command
    lastCommand = 'S';
    currentLeftPwm  = 0.0f;
    currentRightPwm = 0.0f;
    motors.stop();
    Serial.println("[BLE] Client disconnected - immediate motor stop applied, restarting advertising...");
    delay(100);
    BLEDevice::startAdvertising();
  }
};

// ---- RX characteristic callbacks (commands from app) ----
class NusRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    if (val.length() > 0) {
      char cmd = toupper(val[0]);
      unsigned long nowMs = millis();

      // Mode switching commands ('M' / 'A')
      if (cmd == 'M') {
        // Force MANUAL mode explicitly
        currentMode = MODE_MANUAL;
        lastCommand = 'S';
        lastCommandTimeMs = nowMs;
        currentLeftPwm  = 0.0f;
        currentRightPwm = 0.0f;
        motors.stop();
        Serial.println("[MODE] MANUAL mode activated ('M')");
        return;
      }

      if (cmd == 'A') {
        // Force AUTONOMOUS mode explicitly
        currentMode = MODE_AUTONOMOUS;
        lastCommand = 'A';
        lastCommandTimeMs = nowMs;
        currentLeftPwm  = 0.0f;
        currentRightPwm = 0.0f;
        Serial.println("[MODE] AUTONOMOUS mode activated ('A')");
        return;
      }

      // Stop command ('S') works in both MANUAL and AUTONOMOUS modes
      if (cmd == 'S') {
        lastCommand = 'S';
        lastCommandTimeMs = nowMs;
        currentLeftPwm  = 0.0f;
        currentRightPwm = 0.0f;
        motors.stop();
        Serial.println("[CMD] Instant full stop ('S')");
        return;
      }

      // Drive commands ('F', 'B', 'L', 'R')
      if (currentMode == MODE_AUTONOMOUS) {
        // In AUTONOMOUS mode: ignore incoming F/B/L/R drive commands entirely
        Serial.printf("[BLE] In AUTONOMOUS mode — ignoring drive cmd '%c'\n", cmd);
        return;
      }

      // In MANUAL mode: accept drive command as usual
      lastCommand = cmd;
      lastCommandTimeMs = nowMs;
      Serial.printf("[BLE] RX cmd: '%c'  (t=%lu ms)\n", cmd, lastCommandTimeMs);
    }
  }
};

static void setupBLE() {
  Serial.println("[BLE] Initialising BLE stack...");

  // 1. Initialise with device name BEFORE anything else
  BLEDevice::init("RedRovers");

  // 2. Create server + set callbacks
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 3. Create NUS service
  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  // 4. TX characteristic (notify) — rover → app
  pTxCharacteristic = pService->createCharacteristic(
      NUS_TX_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // 5. RX characteristic (write / write-no-response) — app → rover
  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
      NUS_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new NusRxCallbacks());

  // 6. Start service, THEN start advertising
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);   // helps with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started - device name: \"RedRovers\"");
  Serial.printf("[BLE]   NUS Service UUID : %s\n", NUS_SERVICE_UUID);
  Serial.printf("[BLE]   RX Char UUID     : %s\n", NUS_RX_CHAR_UUID);
  Serial.printf("[BLE]   TX Char UUID     : %s\n", NUS_TX_CHAR_UUID);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TELEMETRY TIMERS
// ═══════════════════════════════════════════════════════════════════════════════

static unsigned long lastTelemetryMs = 0;
static const unsigned long TELEMETRY_INTERVAL_MS = 200;  // 5 Hz — fast enough to see hand-wave

// ═══════════════════════════════════════════════════════════════════════════════
//  AUTONOMOUS OBSTACLE AVOIDANCE LOGIC
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Runs one iteration of continuous, non-blocking obstacle avoidance:
 * 1. Reads all three ultrasonic sensors (left, center, right) every iteration.
 * 2. Enforces hard safety stop if center distance is below AUTO_HARD_STOP_DIST_CM.
 * 3. Dynamically scales forward speed down as the closest distance decreases.
 * 4. Steers proportionally toward whichever side has more clearance (differential speed bias).
 * 5. Ramps PWM changes frame-to-frame (capped by AUTO_MAX_PWM_SLEW) to avoid sudden jumps.
 */
static void runAutonomousAvoidance() {
  // Step 1: Read all three sensors every loop iteration
  sensors.readAll();

  // Step 2: Condition sensor readings.
  // HC-SR04 returns -1.0 if no echo was received within the 30 ms timeout.
  // No echo or distance > AUTO_MAX_SENSE_DIST_CM means unobstructed free space.
  float left   = (sensors.distLeft   <= 0.0f || sensors.distLeft   > AUTO_MAX_SENSE_DIST_CM)
                 ? AUTO_MAX_SENSE_DIST_CM : sensors.distLeft;
  float center = (sensors.distCenter <= 0.0f || sensors.distCenter > AUTO_MAX_SENSE_DIST_CM)
                 ? AUTO_MAX_SENSE_DIST_CM : sensors.distCenter;
  float right  = (sensors.distRight  <= 0.0f || sensors.distRight  > AUTO_MAX_SENSE_DIST_CM)
                 ? AUTO_MAX_SENSE_DIST_CM : sensors.distRight;

  // Step 3: HARD MINIMUM SAFETY STOP
  // If center distance drops below the hard safety threshold, stop immediately regardless of steering.
  if (sensors.distCenter > 0.0f && sensors.distCenter <= AUTO_HARD_STOP_DIST_CM) {
    currentLeftPwm  = 0.0f;
    currentRightPwm = 0.0f;
    motors.stop();
    return;
  }

  // Step 4: DYNAMIC FORWARD SPEED SCALING
  // Continuously scale base speed based on the closest obstacle distance.
  float minDist = left;
  if (center < minDist) minDist = center;
  if (right < minDist)  minDist = right;

  float baseSpeed = 0.0f;
  if (minDist <= AUTO_HARD_STOP_DIST_CM) {
    baseSpeed = 0.0f;
  } else if (minDist >= AUTO_SLOWDOWN_DIST_CM) {
    // Open path: cruise at full forward speed
    baseSpeed = AUTO_CRUISE_SPEED;
  } else {
    // Smooth continuous linear interpolation between AUTO_MIN_SPEED and AUTO_CRUISE_SPEED
    float ratio = (minDist - AUTO_HARD_STOP_DIST_CM) / (AUTO_SLOWDOWN_DIST_CM - AUTO_HARD_STOP_DIST_CM);
    baseSpeed = AUTO_MIN_SPEED + ratio * (AUTO_CRUISE_SPEED - AUTO_MIN_SPEED);
  }

  // Step 5: PROPORTIONAL STEERING BIAS
  // Difference = (right clearance - left clearance)
  // Positive diff -> right has more clearance -> turn right (slow down right group)
  // Negative diff -> left has more clearance  -> turn left  (slow down left group)
  float diff = right - left;
  float steer = (diff / AUTO_MAX_SENSE_DIST_CM) * AUTO_STEER_GAIN;
  steer = constrain(steer, -1.0f, 1.0f);

  float targetLeft  = baseSpeed;
  float targetRight = baseSpeed;

  if (steer > 0.0f) {
    // Steer right: keep left at baseSpeed, slow down right group proportionally
    targetRight = baseSpeed * (1.0f - steer);
  } else if (steer < 0.0f) {
    // Steer left: keep right at baseSpeed, slow down left group proportionally
    targetLeft = baseSpeed * (1.0f + steer);
  }

  // Step 6: FRAME-TO-FRAME PWM SLEW RATE LIMITING (RAMPING)
  // Gradually ramp PWM values to target to prevent sudden torque and current spikes.
  float deltaLeft = targetLeft - currentLeftPwm;
  deltaLeft = constrain(deltaLeft, -AUTO_MAX_PWM_SLEW, AUTO_MAX_PWM_SLEW);
  currentLeftPwm += deltaLeft;

  float deltaRight = targetRight - currentRightPwm;
  deltaRight = constrain(deltaRight, -AUTO_MAX_PWM_SLEW, AUTO_MAX_PWM_SLEW);
  currentRightPwm += deltaRight;

  int outLeft  = (int)round(currentLeftPwm);
  int outRight = (int)round(currentRightPwm);
  motors.setSpeed(outLeft, outRight);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1500);   // let serial monitor attach
  Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
  Serial.println("\n\n+--------------------------------------+");
  Serial.println("|       MARS ROVER - ESP32-S3          |");
  Serial.println("+--------------------------------------+\n");

  // ── PHASE 1 — Motor drivers ────────────────────────────────────────────────
  Serial.println("------ PHASE 1: Motor Drivers ------");
  checkStrappingPins();

  if (!motors.begin()) {
    Serial.println("[PHASE 1] FAIL: LEDC channel setup error");
    while (1) delay(1000);   // halt
  }
  Serial.println("[PHASE 1] PASS");

  // ── PHASE 2 — Ultrasonic sensors ───────────────────────────────────────────
  Serial.println("------ PHASE 2: Ultrasonic Sensors ------");
  sensors.begin();
  sensors.readAll();
  sensors.printTelemetry();
  Serial.println("[PHASE 2] PASS");

  // ── PHASE 3 — BLE NUS ─────────────────────────────────────────────────────
  Serial.println("------ PHASE 3: BLE NUS Server ------");
  setupBLE();
  Serial.println("[PHASE 3] PASS");

  // ── PHASE 4 — Connection-loss failsafe ─────────────────────────────────────
  Serial.println("------ PHASE 4: Connection-Loss Failsafe ------");
  Serial.println("  MANUAL drive commands: latching (indefinite until next cmd)");
  Serial.println("  BLE disconnect: immediate full motor cutoff failsafe active");
  Serial.println("[PHASE 4] PASS");

  Serial.println("\n[MODE] Default drive mode: MANUAL ('M'=Manual, 'A'=Autonomous, 'S'=Stop)");
  Serial.println("\n=== SETUP COMPLETE - entering main loop ===\n");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Process BLE commands & Driving Mode ────────────────────────────────────
  if (bleConnected) {
    if (currentMode == MODE_MANUAL) {
      // ---- MANUAL MODE (LATCHING) ----
      // Drive commands ('F', 'B', 'L', 'R') latch continuously until a new command or stop arrives.
      char cmd = lastCommand;

      switch (cmd) {
        case 'F': motors.forward();  break;
        case 'B': motors.backward(); break;
        case 'L': motors.left();     break;
        case 'R': motors.right();    break;
        case 'S': // fallthrough
        default:  motors.stop();     break;
      }
    } else {
      // ---- AUTONOMOUS MODE ----
      if (lastCommand == 'S') {
        // Instant full stop requested while in autonomous mode
        currentLeftPwm  = 0.0f;
        currentRightPwm = 0.0f;
        motors.stop();
      } else {
        // Run continuous, proportional obstacle avoidance
        runAutonomousAvoidance();
      }
    }
  } else {
    // Not connected — ensure motors are stopped and latched state is cleared
    lastCommand = 'S';
    currentLeftPwm  = 0.0f;
    currentRightPwm = 0.0f;
    motors.stop();
  }

  // ── Periodic sensor telemetry ─────────────────────────────────────────────
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;

    // In MANUAL mode, update sensor readings now.
    // In AUTONOMOUS mode, sensors are already updated every loop iteration.
    if (currentMode == MODE_MANUAL) {
      sensors.readAll();
    }

    // Format each channel: show cm value if sensor replied, else "NO ECHO"
    char fmtL[16], fmtC[16], fmtR[16];
    if (sensors.distLeft   >= 0) snprintf(fmtL, sizeof(fmtL), "%6.1f cm", sensors.distLeft);   else snprintf(fmtL, sizeof(fmtL), " NO ECHO");
    if (sensors.distCenter >= 0) snprintf(fmtC, sizeof(fmtC), "%6.1f cm", sensors.distCenter); else snprintf(fmtC, sizeof(fmtC), " NO ECHO");
    if (sensors.distRight  >= 0) snprintf(fmtR, sizeof(fmtR), "%6.1f cm", sensors.distRight);  else snprintf(fmtR, sizeof(fmtR), " NO ECHO");

    Serial.printf("[DIST]  LEFT: %s  |  CENTER: %s  |  RIGHT: %s  (BLE:%s CMD:'%c' MODE:%s)\n",
                   fmtL, fmtC, fmtR,
                   bleConnected ? "CONN" : "----",
                   (char)lastCommand,
                   currentMode == MODE_AUTONOMOUS ? "AUTO" : "MAN");

    // Send over BLE if connected
    if (bleConnected && pTxCharacteristic) {
      char buf[80];
      if (sensors.distLeft >= 0 && sensors.distCenter >= 0 && sensors.distRight >= 0) {
        snprintf(buf, sizeof(buf), "L=%.0fcm C=%.0fcm R=%.0fcm",
                 sensors.distLeft, sensors.distCenter, sensors.distRight);
      } else {
        snprintf(buf, sizeof(buf), "L=%s C=%s R=%s", fmtL, fmtC, fmtR);
      }
      pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pTxCharacteristic->notify();
    }
  }

  delay(20);   // ~50 Hz loop
}