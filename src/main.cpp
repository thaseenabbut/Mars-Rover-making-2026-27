/*******************************************************************************
 * Mars Rover — 6-wheel differential-drive setup for ESP32-S3-DevKitC-1
 *
 * Motor Configuration (ENA/ENB jumpers fitted; no PWM speed control):
 *   • Driver 1 controls the paired front + middle wheels:
 *     - OUT1/OUT2: front-left + middle-left   (IN1 -> GPIO 2, IN2 -> GPIO 1)
 *     - OUT3/OUT4: front-right + middle-right (IN3 -> GPIO 6, IN4 -> GPIO 7)
 *   • Driver 2 controls the independent rear wheels:
 *     - OUT1/OUT2: rear-left                  (IN1 -> GPIO 11, IN2 -> GPIO 10)
 *     - OUT3/OUT4: rear-right                 (IN3 -> GPIO 12, IN4 -> GPIO 13)
 *
 * Movement Functions:
 *   • forward()   - all 6 wheels drive forward
 *   • backward()  - all 6 wheels drive backward
 *   • left()      - differential pivot left (left reverse, right forward)
 *   • right()     - differential pivot right (left forward, right reverse)
 *   • stopRobot() - all 6 wheels stopped
 ******************************************************************************/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTOR PIN DEFINITIONS — FRONT/MIDDLE PAIRED, REAR INDEPENDENT
// ═══════════════════════════════════════════════════════════════════════════════

// Driver 1: paired front + middle wheels
#define D1_LEFT_IN1   2    // IN1 / OUT1-OUT2: front-left + middle-left
#define D1_LEFT_IN2   1    // IN2 / OUT1-OUT2: front-left + middle-left
#define D1_RIGHT_IN3  6    // IN3 / OUT3-OUT4: front-right + middle-right
#define D1_RIGHT_IN4  7    // IN4 / OUT3-OUT4: front-right + middle-right

// Driver 2: independent rear wheels
#define D2_LEFT_IN1   11   // IN1 / OUT1-OUT2: rear-left
#define D2_LEFT_IN2   10   // IN2 / OUT1-OUT2: rear-left
#define D2_RIGHT_IN3  12   // IN3 / OUT3-OUT4: rear-right
#define D2_RIGHT_IN4  13   // IN4 / OUT3-OUT4: rear-right

// ---- HC-SR04 Ultrasonic Sensors (4 sensors) ----
#define SENSOR_LEFT_TRIG    38
#define SENSOR_LEFT_ECHO    39
#define SENSOR_CENTER_TRIG  40
#define SENSOR_CENTER_ECHO  41
#define SENSOR_RIGHT_TRIG   42
#define SENSOR_RIGHT_ECHO   36

// 4th Ultrasonic Sensor
#define SENSOR_4_TRIG       15
#define SENSOR_4_ECHO       16

// NOTE on GPIO36: safe on N8 (no PSRAM). If board is N8R8, change to another free pin.

// ═══════════════════════════════════════════════════════════════════════════════
//  MOTORS CLASS (FRONT/MIDDLE PAIRED, REAR INDEPENDENT)
// ═══════════════════════════════════════════════════════════════════════════════

class Motors {
public:
  bool begin() {
    Serial.println("[Motors] Initialising 6-wheel direction pins...");
    Serial.printf("  Driver 1 (front+middle): L(IN1=%d, IN2=%d) | R(IN3=%d, IN4=%d)\n", D1_LEFT_IN1, D1_LEFT_IN2, D1_RIGHT_IN3, D1_RIGHT_IN4);
    Serial.printf("  Driver 2 (rear):         L(IN1=%d, IN2=%d) | R(IN3=%d, IN4=%d)\n", D2_LEFT_IN1, D2_LEFT_IN2, D2_RIGHT_IN3, D2_RIGHT_IN4);

    pinMode(D1_LEFT_IN1, OUTPUT);
    pinMode(D1_LEFT_IN2, OUTPUT);
    pinMode(D1_RIGHT_IN3, OUTPUT);
    pinMode(D1_RIGHT_IN4, OUTPUT);
    pinMode(D2_LEFT_IN1, OUTPUT);
    pinMode(D2_LEFT_IN2, OUTPUT);
    pinMode(D2_RIGHT_IN3, OUTPUT);
    pinMode(D2_RIGHT_IN4, OUTPUT);

    stopRobot();
    return true;
  }

  // A positive value is forward; a negative value is reverse.
  // Both driver channels on the same side always receive the same direction.
  void setSpeed(int left, int right) {
    setMotor(D1_LEFT_IN1, D1_LEFT_IN2, left);    // Front-left + middle-left
    setMotor(D2_LEFT_IN1, D2_LEFT_IN2, left);    // Rear-left
    setMotor(D1_RIGHT_IN3, D1_RIGHT_IN4, right); // Front-right + middle-right
    setMotor(D2_RIGHT_IN3, D2_RIGHT_IN4, right); // Rear-right
  }

  void forward(int spd = 200)  { setSpeed(spd, spd); }
  void backward(int spd = 200) { setSpeed(-spd, -spd); }
  void left(int spd = 180)     { setSpeed(-spd, spd); }
  void right(int spd = 180)    { setSpeed(spd, -spd); }
  void stopRobot()             { setSpeed(0, 0); }
  void stop()                  { stopRobot(); }

private:
  void setMotor(int in1, int in2, int direction) {
    if (direction > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (direction < 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ULTRASONIC SENSORS CLASS
// ═══════════════════════════════════════════════════════════════════════════════

class UltrasonicSensors {
public:
  float distLeft    = -1;
  float distCenter  = -1;
  float distRight   = -1;
  float distSensor4 = -1;

  void begin() {
    Serial.println("[Sensors] Configuring HC-SR04 pins (4 sensors)...");
    Serial.printf("  LEFT    TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_LEFT_TRIG, SENSOR_LEFT_ECHO);
    Serial.printf("  CENTER  TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_CENTER_TRIG, SENSOR_CENTER_ECHO);
    Serial.printf("  RIGHT   TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_RIGHT_TRIG, SENSOR_RIGHT_ECHO);

    pinMode(SENSOR_LEFT_TRIG,   OUTPUT);
    pinMode(SENSOR_LEFT_ECHO,   INPUT);
    pinMode(SENSOR_CENTER_TRIG, OUTPUT);
    pinMode(SENSOR_CENTER_ECHO, INPUT);
    pinMode(SENSOR_RIGHT_TRIG,  OUTPUT);
    pinMode(SENSOR_RIGHT_ECHO,  INPUT);

    if (SENSOR_4_TRIG >= 0 && SENSOR_4_ECHO >= 0) {
      Serial.printf("  SENSOR4 TRIG=GPIO%d  ECHO=GPIO%d\n", SENSOR_4_TRIG, SENSOR_4_ECHO);
      pinMode(SENSOR_4_TRIG, OUTPUT);
      pinMode(SENSOR_4_ECHO, INPUT);
    } else {
      Serial.println("  SENSOR4: Unassigned pins (-1) — awaiting user GPIO configuration");
    }

    Serial.println("  Sensor pins configured.");
  }

  void readAll() {
    distLeft   = readSensor(SENSOR_LEFT_TRIG,   SENSOR_LEFT_ECHO);
    distCenter = readSensor(SENSOR_CENTER_TRIG,  SENSOR_CENTER_ECHO);
    distRight  = readSensor(SENSOR_RIGHT_TRIG,   SENSOR_RIGHT_ECHO);
    if (SENSOR_4_TRIG >= 0 && SENSOR_4_ECHO >= 0) {
      distSensor4 = readSensor(SENSOR_4_TRIG,   SENSOR_4_ECHO);
    } else {
      distSensor4 = -1.0f;
    }
  }

  void printTelemetry() {
    Serial.printf("  DIST L=%.1fcm  C=%.1fcm  R=%.1fcm  S4=%.1fcm\n",
                   distLeft, distCenter, distRight, distSensor4);
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

// ═══════════════════════════════════════════════════════════════════════════════
//  MOVEMENT FUNCTIONS (6-WHEEL DIFFERENTIAL DRIVE)
// ═══════════════════════════════════════════════════════════════════════════════

void forward()   { motors.forward(); }
void backward()  { motors.backward(); }
void left()      { motors.left(); }
void right()     { motors.right(); }
void stopRobot() { motors.stopRobot(); }

// Driving mode state: MANUAL (default on boot) or AUTONOMOUS
enum DriveMode {
  MODE_MANUAL,
  MODE_AUTONOMOUS
};

static volatile DriveMode currentMode = MODE_MANUAL;

// ---- Autonomous Mode Tuning Parameters ----
// All distances are centimetres. A reading with no echo is treated as clear.
static const float AUTO_BLOCKED_DIST_CM      = 35.0f;
static const float AUTO_MAX_SENSE_DIST_CM    = 150.0f;
static const unsigned long AUTO_SCAN_PAUSE_MS = 500;
static const unsigned long AUTO_TURN_TIME_MS  = 600;
static const unsigned long AUTO_REVERSE_TIME_MS = 450;
static const uint8_t AUTO_MAX_REVERSE_BURSTS = 2;

// Kept for BLE mode transitions and the existing stop failsafe.
static float currentLeftPwm  = 0.0f;
static float currentRightPwm = 0.0f;

enum AutoState : uint8_t {
  AUTO_DRIVE_FORWARD,
  AUTO_STOP_AND_SCAN,
  AUTO_TURN_LEFT,
  AUTO_TURN_RIGHT,
  AUTO_REVERSE
};

static AutoState     autoState = AUTO_DRIVE_FORWARD;
static unsigned long autoStateStartedMs = 0;
static uint8_t       reverseBurstCount = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  BLE NUS (Nordic UART Service)
// ═══════════════════════════════════════════════════════════════════════════════

// Standard NUS UUIDs
#define BLE_DEVICE_NAME        "RedRovers"
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
        autoState = AUTO_DRIVE_FORWARD;
        autoStateStartedMs = nowMs;
        reverseBurstCount = 0;
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
  BLEDevice::init(BLE_DEVICE_NAME);

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

  Serial.printf("[BLE] Advertising started - device name: \"%s\"\n", BLE_DEVICE_NAME);
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

#if 0  // Legacy three-sensor avoidance retained for reference; not compiled.
/**
 * Runs one iteration of continuous, non-blocking obstacle avoidance:
 * 1. Reads all three ultrasonic sensors (left, center, right) every iteration.
 * 2. Enforces hard safety escape if center distance is below AUTO_HARD_STOP_DIST_CM:
 *    - Reverses briefly if both left and right are also blocked (cul-de-sac / dead end).
 *    - Otherwise pivots in place toward whichever side has more clearance.
 *    - Continues pivoting until center distance clears above AUTO_HARD_STOP_DIST_CM.
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
  float leftDistance = (sensors.distLeft   <= 0.0f || sensors.distLeft   > AUTO_MAX_SENSE_DIST_CM)
                       ? AUTO_MAX_SENSE_DIST_CM : sensors.distLeft;
  float center = (sensors.distCenter <= 0.0f || sensors.distCenter > AUTO_MAX_SENSE_DIST_CM)
                 ? AUTO_MAX_SENSE_DIST_CM : sensors.distCenter;
  float rightDistance = (sensors.distRight  <= 0.0f || sensors.distRight  > AUTO_MAX_SENSE_DIST_CM)
                        ? AUTO_MAX_SENSE_DIST_CM : sensors.distRight;

  // Step 3: HARD SAFETY CHECK & OBSTACLE ESCAPE
  // If center obstacle is within AUTO_HARD_STOP_DIST_CM or an escape maneuver is currently in progress:
  bool isBlocked = (sensors.distCenter > 0.0f && sensors.distCenter <= AUTO_HARD_STOP_DIST_CM);
  bool inReverse = (millis() < reverseUntilMs);

  float targetLeft  = 0.0f;
  float targetRight = 0.0f;

  if (inReverse || isBlocked) {
    // Step 3a: Newly detected obstacle -> completely stop all motors and pause for 3–4 seconds
    if (!isEscaping) {
      isEscaping = true;
      escapePauseUntilMs = millis() + AUTO_ESCAPE_PAUSE_MS;
      currentLeftPwm  = 0.0f;
      currentRightPwm = 0.0f;
      motors.stop();
      return;
    }

    // Step 3b: While inside the 3–4 second pre-escape pause window, keep all motors stopped
    if (millis() < escapePauseUntilMs) {
      currentLeftPwm  = 0.0f;
      currentRightPwm = 0.0f;
      motors.stop();
      return;
    }

    // Step 3c: 3–4 second stop elapsed -> now execute required escape movement / direction change
    // Check if dead end / cul-de-sac: both sides are also blocked near hard-stop threshold
    if (!inReverse && (leftDistance <= AUTO_DEAD_END_DIST_CM && rightDistance <= AUTO_DEAD_END_DIST_CM)) {
      reverseUntilMs = millis() + AUTO_REVERSE_TIME_MS;
      inReverse = true;
      lastEscapeDir = 0;
    }

    if (inReverse) {
      // Reversing straight back to gain clearance from cul-de-sac
      targetLeft  = -AUTO_REVERSE_SPEED;
      targetRight = -AUTO_REVERSE_SPEED;
    } else {
      // Pivot in place toward whichever side currently has more clearance.
      // Hysteresis deadband prevents oscillating back and forth when left and right are close.
      if (lastEscapeDir == -1) {
        // Currently pivoting left: only switch to right if right clearance is decisively greater
        if (rightDistance > leftDistance + AUTO_ESCAPE_HYSTERESIS_CM) {
          lastEscapeDir = 1;
        }
      } else if (lastEscapeDir == 1) {
        // Currently pivoting right: only switch to left if left clearance is decisively greater
        if (leftDistance > rightDistance + AUTO_ESCAPE_HYSTERESIS_CM) {
          lastEscapeDir = -1;
        }
      } else {
        // Initial escape decision: pick side with larger clearance
        lastEscapeDir = (rightDistance >= leftDistance) ? 1 : -1;
      }

      if (lastEscapeDir == -1) {
        // Pivot left in place: left backward, right forward
        targetLeft  = -AUTO_PIVOT_SPEED;
        targetRight =  AUTO_PIVOT_SPEED;
      } else {
        // Pivot right in place: left forward, right backward
        targetLeft  =  AUTO_PIVOT_SPEED;
        targetRight = -AUTO_PIVOT_SPEED;
      }
    }
  } else {
    // Center is clear (> AUTO_HARD_STOP_DIST_CM) and not reversing:
    // Reset escape state so control smoothly hands back to normal forward navigation
    isEscaping         = false;
    escapePauseUntilMs = 0;
    lastEscapeDir      = 0;

    // Step 4: DYNAMIC FORWARD SPEED SCALING
    // Continuously scale base speed based on the closest obstacle distance.
    float minDist = leftDistance;
    if (center < minDist) minDist = center;
    if (rightDistance < minDist) minDist = rightDistance;

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
    float diff = rightDistance - leftDistance;
    float steer = (diff / AUTO_MAX_SENSE_DIST_CM) * AUTO_STEER_GAIN;
    steer = constrain(steer, -1.0f, 1.0f);

    targetLeft  = baseSpeed;
    targetRight = baseSpeed;

    if (steer > 0.0f) {
      // Steer right: keep left at baseSpeed, slow down right group proportionally
      targetRight = baseSpeed * (1.0f - steer);
    } else if (steer < 0.0f) {
      // Steer left: keep right at baseSpeed, slow down left group proportionally
      targetLeft = baseSpeed * (1.0f + steer);
    }
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
#endif

static float safeDistance(float distanceCm) {
  return (distanceCm <= 0.0f || distanceCm > AUTO_MAX_SENSE_DIST_CM)
       ? AUTO_MAX_SENSE_DIST_CM
       : distanceCm;
}

static void beginAutoState(AutoState nextState, unsigned long nowMs) {
  autoState = nextState;
  autoStateStartedMs = nowMs;
}

/**
 * Four-sensor autonomous state machine. Manual BLE commands are not changed.
 * It stops and scans before turning, reverses only in short bursts, and checks
 * the rear sensor continuously while reversing.
 */
static void runAutonomousAvoidance() {
  sensors.readAll();
  const unsigned long nowMs = millis();

  const float leftDistance  = safeDistance(sensors.distLeft);
  const float frontDistance = safeDistance(sensors.distCenter);
  const float rightDistance = safeDistance(sensors.distRight);
  const float rearDistance  = safeDistance(sensors.distSensor4);

  const bool frontBlocked = frontDistance <= AUTO_BLOCKED_DIST_CM;
  const bool leftClear    = leftDistance  > AUTO_BLOCKED_DIST_CM;
  const bool rightClear   = rightDistance > AUTO_BLOCKED_DIST_CM;
  const bool rearClear    = rearDistance  > AUTO_BLOCKED_DIST_CM;

  switch (autoState) {
    case AUTO_DRIVE_FORWARD:
      if (frontBlocked) {
        motors.stop();
        beginAutoState(AUTO_STOP_AND_SCAN, nowMs);
        Serial.println("[AUTO] Front blocked: stopping to scan");
      } else {
        motors.forward();
        reverseBurstCount = 0;
      }
      break;

    case AUTO_STOP_AND_SCAN:
      motors.stop();
      if (nowMs - autoStateStartedMs < AUTO_SCAN_PAUSE_MS) break;

      // Example: front and left blocked, right clear -> turn right.
      if (leftClear || rightClear) {
        const bool chooseRight = rightClear && (!leftClear || rightDistance >= leftDistance);
        beginAutoState(chooseRight ? AUTO_TURN_RIGHT : AUTO_TURN_LEFT, nowMs);
        Serial.printf("[AUTO] F=%.0f L=%.0f R=%.0f B=%.0f -> turn %s\n",
                      frontDistance, leftDistance, rightDistance, rearDistance,
                      chooseRight ? "right" : "left");
      } else if (rearClear && reverseBurstCount < AUTO_MAX_REVERSE_BURSTS) {
        ++reverseBurstCount;
        beginAutoState(AUTO_REVERSE, nowMs);
        Serial.printf("[AUTO] Front/left/right blocked -> short reverse %u\n", reverseBurstCount);
      } else if (!rearClear) {
        // Fully boxed in: remain still and scan again rather than colliding.
        Serial.println("[AUTO] All directions blocked: waiting to rescan");
        autoStateStartedMs = nowMs;
      } else {
        // Reverse attempts are exhausted: turn toward the less-blocked side.
        beginAutoState(rightDistance >= leftDistance ? AUTO_TURN_RIGHT : AUTO_TURN_LEFT, nowMs);
      }
      break;

    case AUTO_TURN_LEFT:
      motors.left();
      if (nowMs - autoStateStartedMs >= AUTO_TURN_TIME_MS) {
        beginAutoState(AUTO_DRIVE_FORWARD, nowMs);
      }
      break;

    case AUTO_TURN_RIGHT:
      motors.right();
      if (nowMs - autoStateStartedMs >= AUTO_TURN_TIME_MS) {
        beginAutoState(AUTO_DRIVE_FORWARD, nowMs);
      }
      break;

    case AUTO_REVERSE:
      if (!rearClear || nowMs - autoStateStartedMs >= AUTO_REVERSE_TIME_MS) {
        motors.stop();
        beginAutoState(AUTO_STOP_AND_SCAN, nowMs);
      } else {
        motors.backward();
      }
      break;
  }
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
  if (!motors.begin()) {
    Serial.println("[PHASE 1] FAIL: Motor initialization error");
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
        case 'F': forward();   break;
        case 'B': backward();  break;
        case 'L': left();      break;
        case 'R': right();     break;
        case 'S': // fallthrough
        default:  stopRobot(); break;
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
    char fmtL[16], fmtC[16], fmtR[16], fmtS4[16];
    if (sensors.distLeft    >= 0) snprintf(fmtL, sizeof(fmtL), "%6.1f cm", sensors.distLeft);   else snprintf(fmtL, sizeof(fmtL), " NO ECHO");
    if (sensors.distCenter  >= 0) snprintf(fmtC, sizeof(fmtC), "%6.1f cm", sensors.distCenter); else snprintf(fmtC, sizeof(fmtC), " NO ECHO");
    if (sensors.distRight   >= 0) snprintf(fmtR, sizeof(fmtR), "%6.1f cm", sensors.distRight);  else snprintf(fmtR, sizeof(fmtR), " NO ECHO");
    if (sensors.distSensor4 >= 0) snprintf(fmtS4, sizeof(fmtS4), "%6.1f cm", sensors.distSensor4); else snprintf(fmtS4, sizeof(fmtS4), " NO ECHO");

    Serial.printf("[DIST]  L: %s  |  C: %s  |  R: %s  |  S4: %s  (BLE:%s CMD:'%c' MODE:%s)\n",
                   fmtL, fmtC, fmtR, fmtS4,
                   bleConnected ? "CONN" : "----",
                   (char)lastCommand,
                   currentMode == MODE_AUTONOMOUS ? "AUTO" : "MAN");

    // Send over BLE if connected
    if (bleConnected && pTxCharacteristic) {
      char buf[96];
      if (sensors.distLeft >= 0 && sensors.distCenter >= 0 && sensors.distRight >= 0 && sensors.distSensor4 >= 0) {
        snprintf(buf, sizeof(buf), "L=%.0fcm C=%.0fcm R=%.0fcm S4=%.0fcm",
                 sensors.distLeft, sensors.distCenter, sensors.distRight, sensors.distSensor4);
      } else {
        snprintf(buf, sizeof(buf), "L=%s C=%s R=%s S4=%s", fmtL, fmtC, fmtR, fmtS4);
      }
      pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
      pTxCharacteristic->notify();
    }
  }

  delay(20);   // ~50 Hz loop
}
