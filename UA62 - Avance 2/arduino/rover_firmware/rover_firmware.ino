/*
  arduino/rover_firmware/rover_firmware.ino
  Rover Actuation & Perception Node — Object-Oriented Firmware for Arduino Uno (UA62 - Avance 2)

  Hardware Architecture:
    - 4x Modified MG996R (360° Continuous Rotation) for 87.5 mm Rover Wheels:
        * Left Wheel Pair  : Pin D6 (PWM Servo signal 1000–2000 us, 1500 us = STOP)
        * Right Wheel Pair : Pin D9 (PWM Servo signal 1000–2000 us, 1500 us = STOP)
    - 2x Standard MG996R (0°–180°) for 2-DOF Robotic Arm:
        * Arm Base Servo   : Pin D10
        * Arm Elbow Servo  : Pin D11
    - 1x MPU-6050 6-Axis IMU (I2C @ 400 kHz):
        * SDA = A4, SCL = A5 (I2C Address 0x68)
        * Hardware DLPF @ 44 Hz + Gyro Z integration for exact closed-loop angle turns
    - 2x HC-SR04 Ultrasonic Sensors (FIFO + Median Filter N=5 from UA34):
        * Ultrasonic #1    : TRIG = D2, ECHO = D3
        * Ultrasonic #2    : TRIG = D4, ECHO = D5
        * Automatic obstacle evasion: stops and reverses when obstacle < 20.0 cm
*/

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// ── Pin & Hardware Constants ─────────────────────────────────────────────────
constexpr uint8_t  PIN_TRIG1            = 2;
constexpr uint8_t  PIN_ECHO1            = 3;
constexpr uint8_t  PIN_TRIG2            = 4;
constexpr uint8_t  PIN_ECHO2            = 5;

constexpr uint8_t  PIN_SERVO_LEFT       = 6;
constexpr uint8_t  PIN_SERVO_RIGHT      = 9;
constexpr uint8_t  PIN_ARM_BASE         = 10;
constexpr uint8_t  PIN_ARM_ELBOW        = 11;

// Wheel servo polarity (opposite sides of chassis are physically mirrored)
constexpr bool     INVERT_LEFT_WHEELS   = false;
constexpr bool     INVERT_RIGHT_WHEELS  = true;

// Pulse widths (microseconds) for modified continuous MG996R servos
constexpr int      SERVO_STOP_US        = 1500;
constexpr int      SERVO_MAX_DELTA_US   = 500;  // 1500 +/- 500 => [1000, 2000] us

// Preset angles (degrees) for 2-DOF Robotic Arm (unmodified MG996R)
constexpr uint8_t  ARM_BASE_HOME        = 90;
constexpr uint8_t  ARM_ELBOW_HOME       = 25;
constexpr uint8_t  ARM_BASE_EXTEND      = 90;
constexpr uint8_t  ARM_ELBOW_EXTEND     = 145;

// Ultrasonic & Multitasking Constants (UA34 Slide 10, 12, 14, 15 Methodology)
constexpr uint8_t  FIFO_SIZE            = 5;
constexpr uint32_t US_SAMPLE_PERIOD_MS  = 30;
constexpr uint32_t PULSE_TIMEOUT_US     = 15000UL; // ~2.5 m max range, keeps gyro loop fast
constexpr float    OBSTACLE_DIST_CM     = 20.0f;   // Auto-reverse trigger distance
constexpr uint32_t OBSTACLE_REVERSE_MS  = 850;     // Duration of automatic reverse maneuver
constexpr int      OBSTACLE_REV_SPEED   = 50;      // Speed (%) during automatic reverse
constexpr int      DEFAULT_GYRO_TURN_SPD = 55;     // Speed (%) during gyro-assisted turns

// ── C Functions for FIFO & Median Filter (UA34 Methodology) ──────────────────
void applyFIFO(float readings[FIFO_SIZE], float newReading) {
  for (int i = 0; i < FIFO_SIZE - 1; i++) {
    readings[i] = readings[i + 1];
  }
  readings[FIFO_SIZE - 1] = newReading;
}

void sortArray(float arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j]     = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

float calculateMedian(const float readings[FIFO_SIZE]) {
  float readingsCopy[FIFO_SIZE];
  memcpy(readingsCopy, readings, sizeof(readingsCopy));
  sortArray(readingsCopy, FIFO_SIZE);
  return readingsCopy[FIFO_SIZE / 2];
}

// ── Class 1: FIFO Median Filter Object ───────────────────────────────────────
class MedianFIFO {
  private:
    float _readings[FIFO_SIZE];

  public:
    MedianFIFO(float initialVal = 400.0f) {
      for (int i = 0; i < FIFO_SIZE; i++) {
        _readings[i] = initialVal;
      }
    }

    void push(float newReading) {
      applyFIFO(_readings, newReading);
    }

    float median() const {
      return calculateMedian(_readings);
    }
};

// ── Class 2: Ultrasonic Sensor (HC-SR04) ─────────────────────────────────────
class UltrasonicSensor {
  private:
    const uint8_t _trigPin;
    const uint8_t _echoPin;
    MedianFIFO    _filter;
    float         _lastValidCm = 400.0f;

  public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
      : _trigPin(trigPin), _echoPin(echoPin), _filter(400.0f) {}

    void init() {
      pinMode(_trigPin, OUTPUT);
      pinMode(_echoPin, INPUT);
      digitalWrite(_trigPin, LOW);
    }

    void sample() {
      digitalWrite(_trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(_trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(_trigPin, LOW);

      unsigned long time_us = pulseIn(_echoPin, HIGH, PULSE_TIMEOUT_US);
      float distance = (time_us == 0) ? 400.0f : (0.0343f * (float)time_us) / 2.0f;
      if (time_us > 0) {
        _lastValidCm = distance;
      }
      _filter.push(distance);
    }

    float getFilteredCm() const {
      return _filter.median();
    }
};

// ── Class 3: MPU-6050 IMU Sensor (Direct Fast-I2C Register Access) ───────────
class MPU6050Sensor {
  private:
    static constexpr uint8_t ADDR        = 0x68;
    static constexpr uint8_t REG_PWR_1   = 0x6B;
    static constexpr uint8_t REG_CONFIG  = 0x1A;
    static constexpr uint8_t REG_GYRO_CFG = 0x1B;
    static constexpr uint8_t REG_GYRO_Z  = 0x47;
    static constexpr float   LSB_PER_DPS = 65.5f; // +/- 500 deg/s full scale

    bool          _connected     = false;
    float         _gzBias        = 0.0f;
    float         _yawDeg        = 0.0f;
    float         _turnAccumDeg  = 0.0f;
    float         _lastRateDps   = 0.0f;
    unsigned long _lastMicros    = 0;

    bool _writeReg(uint8_t reg, uint8_t val) {
      Wire.beginTransmission(ADDR);
      Wire.write(reg);
      Wire.write(val);
      return (Wire.endTransmission() == 0);
    }

    bool _readGyroZRaw(int16_t &rawGz) {
      Wire.beginTransmission(ADDR);
      Wire.write(REG_GYRO_Z);
      if (Wire.endTransmission(false) != 0) {
        return false;
      }
      if (Wire.requestFrom(ADDR, (uint8_t)2) != 2) {
        return false;
      }
      uint8_t hi = Wire.read();
      uint8_t lo = Wire.read();
      rawGz = (int16_t)((hi << 8) | lo);
      return true;
    }

  public:
    void init() {
      Wire.begin();
      Wire.setClock(400000L); // 400 kHz Fast I2C
      // Set Wire timeout to avoid lockups on noisy I2C lines
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.setWireTimeout(3000, true);
      #endif

      if (!_writeReg(REG_PWR_1, 0x00)) {
        _connected = false;
        return;
      }
      delay(10);
      _writeReg(REG_CONFIG,   0x03); // DLPF ~44 Hz to reject motor/chassis vibration
      _writeReg(REG_GYRO_CFG, 0x08); // +/- 500 deg/s full scale

      // Calibrate Z-axis gyro offset over 150 fast samples
      long sum = 0;
      uint16_t valid = 0;
      for (uint16_t i = 0; i < 150; i++) {
        int16_t raw = 0;
        if (_readGyroZRaw(raw)) {
          sum += raw;
          valid++;
        }
        delayMicroseconds(1500);
      }

      if (valid > 50) {
        _connected = true;
        _gzBias = (float)sum / (float)valid;
      } else {
        _connected = false;
      }
      _lastMicros = micros();
    }

    void update() {
      unsigned long now = micros();
      float dt = (float)(now - _lastMicros) * 1e-6f;
      _lastMicros = now;

      if (dt <= 0.0f || dt > 0.5f) {
        return;
      }

      if (!_connected) {
        return;
      }

      int16_t rawGz = 0;
      if (!_readGyroZRaw(rawGz)) {
        return;
      }

      float rateDps = ((float)rawGz - _gzBias) / LSB_PER_DPS;
      // Deadzone filter to suppress stationary integration drift
      if (fabs(rateDps) < 0.40f) {
        rateDps = 0.0f;
      }
      _lastRateDps   = rateDps;
      float deltaDeg = rateDps * dt;
      _yawDeg       += deltaDeg;
      _turnAccumDeg += fabs(deltaDeg);
    }

    void resetTurnAngle() {
      _turnAccumDeg = 0.0f;
      _lastMicros   = micros();
    }

    bool isConnected() const     { return _connected; }
    float getYawDeg() const      { return _yawDeg; }
    float getTurnedDeg() const   { return _turnAccumDeg; }
    float getRateDps() const     { return _lastRateDps; }
};

// ── Class 4: 4-Wheel Continuous Servo Drive (87.5 mm Wheels) ─────────────────
class WheelDrive {
  private:
    Servo   _leftServo;
    Servo   _rightServo;
    uint8_t _pinLeft;
    uint8_t _pinRight;
    int     _leftPct  = 0;
    int     _rightPct = 0;

    static int _pctToUs(int pct, bool invert) {
      int clamped = constrain(pct, -100, 100);
      if (invert) {
        clamped = -clamped;
      }
      return SERVO_STOP_US + (clamped * SERVO_MAX_DELTA_US) / 100;
    }

  public:
    WheelDrive(uint8_t pinLeft, uint8_t pinRight)
      : _pinLeft(pinLeft), _pinRight(pinRight) {}

    void init() {
      _leftServo.attach(_pinLeft);
      _rightServo.attach(_pinRight);
      stop();
    }

    void setDifferential(int leftPct, int rightPct) {
      _leftPct  = constrain(leftPct,  -100, 100);
      _rightPct = constrain(rightPct, -100, 100);
      _leftServo.writeMicroseconds(_pctToUs(_leftPct,  INVERT_LEFT_WHEELS));
      _rightServo.writeMicroseconds(_pctToUs(_rightPct, INVERT_RIGHT_WHEELS));
    }

    void moveForward(int speedPct) {
      int s = constrain(abs(speedPct), 0, 100);
      setDifferential(s, s);
    }

    void moveBackward(int speedPct) {
      int s = constrain(abs(speedPct), 0, 100);
      setDifferential(-s, -s);
    }

    void spinRight(int speedPct) {
      int s = constrain(abs(speedPct), 0, 100);
      setDifferential(s, -s);
    }

    void spinLeft(int speedPct) {
      int s = constrain(abs(speedPct), 0, 100);
      setDifferential(-s, s);
    }

    void stop() {
      setDifferential(0, 0);
    }

    int getLeftPct() const  { return _leftPct; }
    int getRightPct() const { return _rightPct; }
    bool isMovingForward() const { return (_leftPct > 0 && _rightPct > 0); }
};

// ── Class 5: 2-DOF Robotic Arm (2x Standard MG996R Servos) ───────────────────
class RoboticArm2DOF {
  private:
    Servo   _baseServo;
    Servo   _elbowServo;
    uint8_t _pinBase;
    uint8_t _pinElbow;
    uint8_t _baseAngle  = ARM_BASE_HOME;
    uint8_t _elbowAngle = ARM_ELBOW_HOME;

  public:
    RoboticArm2DOF(uint8_t pinBase, uint8_t pinElbow)
      : _pinBase(pinBase), _pinElbow(pinElbow) {}

    void init() {
      _baseServo.attach(_pinBase);
      _elbowServo.attach(_pinElbow);
      home();
    }

    void setAngles(int baseDeg, int elbowDeg) {
      _baseAngle  = (uint8_t)constrain(baseDeg,  0, 180);
      _elbowAngle = (uint8_t)constrain(elbowDeg, 0, 180);
      _baseServo.write(_baseAngle);
      _elbowServo.write(_elbowAngle);
    }

    void extend() {
      setAngles(ARM_BASE_EXTEND, ARM_ELBOW_EXTEND);
    }

    void home() {
      setAngles(ARM_BASE_HOME, ARM_ELBOW_HOME);
    }

    uint8_t getBaseAngle() const  { return _baseAngle; }
    uint8_t getElbowAngle() const { return _elbowAngle; }
};

// ── Class 6: Non-Blocking Rover Controller Node ──────────────────────────────
enum RoverMode : uint8_t {
  MODE_IDLE = 0,
  MODE_MOVING_FWD,
  MODE_MOVING_BWD,
  MODE_TURN_DIFF,
  MODE_TURN_GYRO,
  MODE_AUTO_REVERSE
};

class RoverActuatorNode {
  private:
    WheelDrive       _wheels;
    RoboticArm2DOF   _arm;
    MPU6050Sensor    _imu;
    UltrasonicSensor _us1;
    UltrasonicSensor _us2;

    RoverMode        _mode               = MODE_IDLE;
    unsigned long    _lastUsSampleMs     = 0;
    unsigned long    _lastImuUpdateMs    = 0;
    unsigned long    _stateStartMs       = 0;
    unsigned long    _fallbackTurnMs     = 0;
    float            _targetTurnDeg      = 0.0f;
    bool             _turnRightDir       = true;
    bool             _toggleUs           = false;
    bool             _obstacleLatched    = false;

    char             _rxBuf[64];
    uint8_t          _rxIdx              = 0;

    void _sendStatus() const {
      char s_yaw[12], s_us1[12], s_us2[12], out[128];
      dtostrf(_imu.getYawDeg(),      4, 1, s_yaw);
      dtostrf(_us1.getFilteredCm(),  4, 1, s_us1);
      dtostrf(_us2.getFilteredCm(),  4, 1, s_us2);

      snprintf(
        out, sizeof(out),
        "[STATUS] l=%d,r=%d,arm=%d:%d,yaw=%s,imu=%d,us1=%s,us2=%s,mode=%d",
        _wheels.getLeftPct(),
        _wheels.getRightPct(),
        _arm.getBaseAngle(),
        _arm.getElbowAngle(),
        s_yaw,
        _imu.isConnected() ? 1 : 0,
        s_us1,
        s_us2,
        (int)_mode
      );
      Serial.println(out);
    }

    void _startGyroTurn(bool rightDir, float targetDeg, int speedPct) {
      _turnRightDir  = rightDir;
      _targetTurnDeg = fabs(targetDeg);
      _stateStartMs  = millis();
      _imu.resetTurnAngle();

      // Kinematic fallback duration in case IMU I2C is disconnected:
      // With 87.5 mm wheels @ ~55% speed, ~90 deg/s chassis rotation rate
      float estRateDps = 90.0f * ((float)constrain(speedPct, 20, 100) / 55.0f);
      _fallbackTurnMs  = (unsigned long)((_targetTurnDeg / estRateDps) * 1000.0f);
      if (_fallbackTurnMs < 100) _fallbackTurnMs = 100;
      if (_fallbackTurnMs > 8000) _fallbackTurnMs = 8000;

      if (rightDir) {
        _wheels.spinRight(speedPct);
      } else {
        _wheels.spinLeft(speedPct);
      }
      _mode = MODE_TURN_GYRO;

      char s_deg[12], out[80];
      dtostrf(_targetTurnDeg, 4, 1, s_deg);
      snprintf(
        out, sizeof(out),
        "[ACK] TURN_GYRO_START|%s|%s|%d",
        rightDir ? "RIGHT" : "LEFT",
        s_deg,
        speedPct
      );
      Serial.println(out);
    }

    void _handleCommand(const char* rawCmd) {
      while (*rawCmd == ' ' || *rawCmd == '\t') rawCmd++;
      if (*rawCmd == '\0') return;

      char up[64];
      uint8_t i = 0;
      while (rawCmd[i] != '\0' && i < sizeof(up) - 1) {
        up[i] = (char)toupper((unsigned char)rawCmd[i]);
        i++;
      }
      while (i > 0 && (up[i - 1] == ' ' || up[i - 1] == '\t')) i--;
      up[i] = '\0';

      // 1. STOP
      if (strcmp(up, "STOP") == 0) {
        _wheels.stop();
        _mode = MODE_IDLE;
        Serial.println("[ACK] STOP");
        return;
      }

      // 2. STATUS / READ
      if (strcmp(up, "STATUS") == 0 || strcmp(up, "READ") == 0) {
        _sendStatus();
        return;
      }

      // 3. ARM-EXTEND / ARM-DEPLOYMENT
      if (strcmp(up, "ARM-EXTEND") == 0 || strcmp(up, "ARM-DEPLOYMENT") == 0) {
        _arm.extend();
        char out[64];
        snprintf(out, sizeof(out), "[ACK] ARM_EXTEND|%d|%d", _arm.getBaseAngle(), _arm.getElbowAngle());
        Serial.println(out);
        return;
      }

      // 4. ARM-RETRIEVE / ARM-RETREIVE / ARM-HOME
      if (strcmp(up, "ARM-RETRIEVE") == 0 || strcmp(up, "ARM-RETREIVE") == 0 || strcmp(up, "ARM-HOME") == 0) {
        _arm.home();
        char out[64];
        snprintf(out, sizeof(out), "[ACK] ARM_HOME|%d|%d", _arm.getBaseAngle(), _arm.getElbowAngle());
        Serial.println(out);
        return;
      }

      // 5. ARM <base_deg> <elbow_deg>
      if (strncmp(up, "ARM ", 4) == 0) {
        int b = ARM_BASE_HOME, e = ARM_ELBOW_HOME;
        if (sscanf(up + 4, "%d %d", &b, &e) == 2) {
          _arm.setAngles(b, e);
          char out[64];
          snprintf(out, sizeof(out), "[ACK] ARM_ANGLES|%d|%d", _arm.getBaseAngle(), _arm.getElbowAngle());
          Serial.println(out);
        } else {
          Serial.println("[ERROR] Syntax: ARM <base 0-180> <elbow 0-180>");
        }
        return;
      }

      // 6. MOVE-FORWARD <speed>
      if (strncmp(up, "MOVE-FORWARD", 12) == 0) {
        int spd = 50;
        if (strlen(up) > 12) {
          spd = atoi(up + 12);
        }
        spd = constrain(spd, 0, 100);
        _wheels.moveForward(spd);
        _mode = (spd > 0) ? MODE_MOVING_FWD : MODE_IDLE;
        char out[64];
        snprintf(out, sizeof(out), "[ACK] MOVE_FORWARD|%d", spd);
        Serial.println(out);
        return;
      }

      // 7. MOVE-BACKWARD <speed>
      if (strncmp(up, "MOVE-BACKWARD", 13) == 0) {
        int spd = 50;
        if (strlen(up) > 13) {
          spd = atoi(up + 13);
        }
        spd = constrain(spd, 0, 100);
        _wheels.moveBackward(spd);
        _mode = (spd > 0) ? MODE_MOVING_BWD : MODE_IDLE;
        char out[64];
        snprintf(out, sizeof(out), "[ACK] MOVE_BACKWARD|%d", spd);
        Serial.println(out);
        return;
      }

      // 8. TURN-RIGHT <deg> OR TURN-RIGHT <left_spd> <right_spd>
      if (strncmp(up, "TURN-RIGHT", 10) == 0) {
        int v1 = 0, v2 = 0;
        int count = sscanf(up + 10, "%d %d", &v1, &v2);
        if (count == 2) {
          // Differential motor speed turn from slide: e.g. TURN-RIGHT 80 20
          _wheels.setDifferential(v1, v2);
          _mode = MODE_TURN_DIFF;
          char out[64];
          snprintf(out, sizeof(out), "[ACK] TURN_DIFF|RIGHT|%d|%d", v1, v2);
          Serial.println(out);
        } else if (count == 1 && v1 > 0) {
          // Gyroscope angle turn: e.g. TURN-RIGHT 20
          _startGyroTurn(true, (float)v1, DEFAULT_GYRO_TURN_SPD);
        } else {
          _startGyroTurn(true, 20.0f, DEFAULT_GYRO_TURN_SPD);
        }
        return;
      }

      // 9. TURN-LEFT <deg> OR TURN-LEFT <left_spd> <right_spd>
      if (strncmp(up, "TURN-LEFT", 9) == 0) {
        int v1 = 0, v2 = 0;
        int count = sscanf(up + 9, "%d %d", &v1, &v2);
        if (count == 2) {
          // Differential motor speed turn: e.g. TURN-LEFT 20 80
          _wheels.setDifferential(v1, v2);
          _mode = MODE_TURN_DIFF;
          char out[64];
          snprintf(out, sizeof(out), "[ACK] TURN_DIFF|LEFT|%d|%d", v1, v2);
          Serial.println(out);
        } else if (count == 1 && v1 > 0) {
          // Gyroscope angle turn: e.g. TURN-LEFT 20
          _startGyroTurn(false, (float)v1, DEFAULT_GYRO_TURN_SPD);
        } else {
          _startGyroTurn(false, 20.0f, DEFAULT_GYRO_TURN_SPD);
        }
        return;
      }

      Serial.print("[ERROR] Unknown command: ");
      Serial.println(up);
    }

  public:
    RoverActuatorNode()
      : _wheels(PIN_SERVO_LEFT, PIN_SERVO_RIGHT),
        _arm(PIN_ARM_BASE, PIN_ARM_ELBOW),
        _us1(PIN_TRIG1, PIN_ECHO1),
        _us2(PIN_TRIG2, PIN_ECHO2) {}

    void init() {
      _wheels.init();
      _arm.init();
      _us1.init();
      _us2.init();
      _imu.init();

      // Pre-fill 5-element ultrasonic FIFO queues
      for (uint8_t i = 0; i < FIFO_SIZE; i++) {
        _us1.sample();
        delay(10);
        _us2.sample();
        delay(10);
      }

      Serial.print("[READY] RoverActuatorNode | Wheels=87.5mm(4xMG996R) | Arm=2DOF | IMU=");
      Serial.println(_imu.isConnected() ? "MPU6050_OK" : "FALLBACK");
    }

    void update() {
      unsigned long nowMs = millis();

      // 1. High-frequency non-blocking IMU integration (~200 Hz)
      if (nowMs - _lastImuUpdateMs >= 5) {
        _lastImuUpdateMs = nowMs;
        _imu.update();
      }

      // 2. Non-blocking Gyro Turn check
      if (_mode == MODE_TURN_GYRO) {
        bool reachedByImu = _imu.isConnected() && (_imu.getTurnedDeg() >= _targetTurnDeg);
        bool reachedByFallback = (!_imu.isConnected()) && (nowMs - _stateStartMs >= _fallbackTurnMs);
        bool safetyTimeout = (nowMs - _stateStartMs >= 10000UL);

        if (reachedByImu || reachedByFallback || safetyTimeout) {
          _wheels.stop();
          _mode = MODE_IDLE;
          float finalDeg = reachedByImu ? _imu.getTurnedDeg() : _targetTurnDeg;
          char s_deg[12], out[80];
          dtostrf(finalDeg, 4, 1, s_deg);
          snprintf(
            out, sizeof(out),
            "[EVENT] TURN_DONE|%s|%s",
            _turnRightDir ? "RIGHT" : "LEFT",
            s_deg
          );
          Serial.println(out);
        }
      }

      // 3. Staggered Ultrasonic Sampling (every 30 ms, alternating US1 / US2)
      if (nowMs - _lastUsSampleMs >= US_SAMPLE_PERIOD_MS) {
        _lastUsSampleMs = nowMs;
        if (!_toggleUs) {
          _us1.sample();
        } else {
          _us2.sample();
        }
        _toggleUs = !_toggleUs;

        float d1 = _us1.getFilteredCm();
        float d2 = _us2.getFilteredCm();
        float minDist = (d1 < d2) ? d1 : d2;

        // Automatic obstacle avoidance: if obstacle < 20 cm, reverse automatically
        if (_mode != MODE_AUTO_REVERSE && _mode != MODE_MOVING_BWD) {
          if (minDist > 1.0f && minDist < OBSTACLE_DIST_CM && !_obstacleLatched) {
            _obstacleLatched = true;
            _wheels.moveBackward(OBSTACLE_REV_SPEED);
            _mode = MODE_AUTO_REVERSE;
            _stateStartMs = nowMs;

            char s_dist[12], out[80];
            dtostrf(minDist, 4, 1, s_dist);
            snprintf(out, sizeof(out), "[EVENT] OBSTACLE|%s|REVERSING", s_dist);
            Serial.println(out);
          } else if (minDist >= OBSTACLE_DIST_CM + 5.0f) {
            _obstacleLatched = false;
          }
        }
      }

      // 4. Complete automatic obstacle reverse maneuver after OBSTACLE_REVERSE_MS
      if (_mode == MODE_AUTO_REVERSE) {
        if (nowMs - _stateStartMs >= OBSTACLE_REVERSE_MS) {
          _wheels.stop();
          _mode = MODE_IDLE;
          char s_d1[12], s_d2[12], out[80];
          dtostrf(_us1.getFilteredCm(), 4, 1, s_d1);
          dtostrf(_us2.getFilteredCm(), 4, 1, s_d2);
          snprintf(out, sizeof(out), "[EVENT] OBSTACLE_CLEARED|%s|%s", s_d1, s_d2);
          Serial.println(out);
        }
      }

      // 5. Non-blocking UART Command Reader
      while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
          if (_rxIdx > 0) {
            _rxBuf[_rxIdx] = '\0';
            _handleCommand(_rxBuf);
            _rxIdx = 0;
          }
        } else if (_rxIdx < sizeof(_rxBuf) - 1) {
          _rxBuf[_rxIdx++] = c;
        }
      }
    }
};

// ── Global Node Object ───────────────────────────────────────────────────────
RoverActuatorNode roverNode;

void setup() {
  Serial.begin(115200);
  roverNode.init();
}

void loop() {
  roverNode.update();
}
