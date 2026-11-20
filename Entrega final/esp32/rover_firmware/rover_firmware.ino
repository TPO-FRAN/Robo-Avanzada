/*
  esp32/rover_firmware/rover_firmware.ino
  Real-Time Rover Firmware for ESP32 DevKit V1 — Entrega Final

  Hardware & Bus Layout:
    - 2x L298N Mini (MX1508) DC Motor Drivers (8 independent LEDC PWM GPIOs @ 2 kHz, 10-bit):
        * L298N Mini Left  - FL (0): IN1=GPIO 25 (ch 0), IN2=GPIO 26 (ch 1)
        * L298N Mini Left  - RL (1): IN3=GPIO 27 (ch 2), IN4=GPIO 14 (ch 3)
        * L298N Mini Right - FR (2): IN1=GPIO 32 (ch 4), IN2=GPIO 33 (ch 5) [inverted=1]
        * L298N Mini Right - RR (3): IN3=GPIO 16 (ch 6), IN4=GPIO 17 (ch 7) [inverted=1]
    - 2-DOF Robotic Arm (2x Standard MG996R Servos directly on ESP32 LEDC @ 50 Hz, 14-bit):
        * Arm Base  : GPIO 23 (LEDC ch 8, LS timer group)
        * Arm Elbow : GPIO 13 (LEDC ch 9, LS timer group)
        * Note: Power servo V+ (5-6V) from external battery supply with common GND (NOT ESP32 3.3V/5V).
    - GY-511 (LSM303DLHC 2D Magnetometer @ 0x1E) on dedicated Hardware I2C (400 kHz):
        * SDA = GPIO 21, SCL = GPIO 22, VCC = 3.3V, GND = GND
    - 2x HC-SR04 Ultrasonic Sensors (IRAM_ATTR GPIO interrupts, N=5 median FIFO):
        * US1 : TRIG1 = GPIO 18, ECHO1 = GPIO 34 (input-only; requires 5V->3.3V divider)
        * US2 : TRIG2 = GPIO 19, ECHO2 = GPIO 35 (input-only; requires 5V->3.3V divider)

  UART & Serial Commands (115200 baud, Spanish or Compact Opcodes):
    - ADELANTE [vel]        / W <l> <r>   : Forward / differential speed (-100..100 %)
    - ATRAS [vel]                         : Reverse
    - GIRARD [vel]                        : Differential right spin (L=+vel, R=-vel)
    - GIRARI [vel]                        : Differential left spin  (L=-vel, R=+vel)
    - BRAZOEXT              / A <b> <e>   : Arm extend / target angles (0..180 deg)
    - BRAZOH                              : Arm home (90, 25 deg)
    - ALTO                  / X           : Immediate active brake stop + freeze arm
    - ESTADO                / ?           : Telemetry query ([STATUS] 9-field CSV)
    - GIROD / GIROI <deg>   / G <ddeg>    : Closed-loop turn using GY-511 (0.1 deg units)

  Online Calibration Commands (Serial Monitor @ 115200):
    - P <0-3> <-100..100>   : Raw motor duty test on wheel 0..3 (0=FL,1=RL,2=FR,3=RR)
    - V <0-3> <0|1>         : Set polarity inversion for wheel 0..3
    - M <fl> <rl> <fr> <rr> : Set maxDutyPct (20..100) for all 4 wheels (straight-line trim)
    - M <0-3> <min> <max>   : Set minDutyPct (0..80) and maxDutyPct (20..100) for wheel 0..3
    - B <0|1>               : Active short-brake on stop (1=enabled 80ms pulse, 0=coast)
    - I [ox oy | C | P <+1|-1> | 0] : Query IMU, set offsets, 360 auto-cal (I C), CW polarity, reset
    - E [0]                 : Save motor calibration to NVS (E 0 = restore defaults)
    - K                     : Print active motor & NVS calibration
    - STOP / S              : Stop all motors and freeze arm
*/

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#include <Preferences.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <math.h>
#include <stdlib.h>
#include <strings.h>

// ── ESP32 Hardware Pinout ────────────────────────────────────────────────────
constexpr uint8_t PIN_FL_A   = 25, PIN_FL_B = 26; // L298N Mini Left  - FL (ch 0, 1)
constexpr uint8_t PIN_RL_A   = 27, PIN_RL_B = 14; // L298N Mini Left  - RL (ch 2, 3)
constexpr uint8_t PIN_FR_A   = 32, PIN_FR_B = 33; // L298N Mini Right - FR (ch 4, 5)
constexpr uint8_t PIN_RR_A   = 16, PIN_RR_B = 17; // L298N Mini Right - RR (ch 6, 7)

constexpr uint8_t PIN_ARM_B  = 23; // Direct ESP32 LEDC ch 8 (Arm Base, 50 Hz)
constexpr uint8_t PIN_ARM_E  = 13; // Direct ESP32 LEDC ch 9 (Arm Elbow, 50 Hz)

constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;

constexpr uint8_t PIN_TRIG1  = 18, PIN_ECHO1 = 34; // Input-only; requires 5V->3.3V divider
constexpr uint8_t PIN_TRIG2  = 19, PIN_ECHO2 = 35; // Input-only; requires 5V->3.3V divider

// ── LEDC PWM & Motion Constants ──────────────────────────────────────────────
constexpr uint32_t MOTOR_PWM_HZ      = 2000UL;
constexpr uint8_t  MOTOR_PWM_BITS    = 10U;
constexpr uint32_t MOTOR_PWM_MAX     = (1UL << MOTOR_PWM_BITS) - 1UL; // 1023

constexpr uint32_t SERVO_PWM_HZ      = 50UL;
constexpr uint8_t  SERVO_PWM_BITS    = 14U;
constexpr uint32_t SERVO_PWM_PERIOD_US = 20000UL;
constexpr uint32_t SERVO_DUTY_RANGE  = (1UL << SERVO_PWM_BITS); // 16384

constexpr uint8_t  CH_ARM_B          = 8;
constexpr uint8_t  CH_ARM_E          = 9;

constexpr uint8_t  NUM_WHEELS        = 4;
constexpr uint8_t  W_FL = 0, W_RL = 1, W_FR = 2, W_RR = 3;

constexpr uint8_t  ARM_B_HOME        = 90,   ARM_E_HOME = 25;
constexpr int16_t  ARM_MIN_US        = 544,  ARM_MAX_US = 2400;
constexpr uint16_t US_TIMEOUT_US     = 15000U;
constexpr uint16_t OBS_TRIG_MM       = 200U; // 20.0 cm
constexpr uint16_t OBS_CLEAR_MM      = 250U; // 25.0 cm
constexpr uint8_t  TURN_MAX_SPD      = 85,   TURN_MIN_SPD = 30;
constexpr uint8_t  TURN_CAL_SPD      = 60;
constexpr int16_t  TURN_SLOW_DDEG    = 250;  // Linear ramp in last 25.0 deg
constexpr uint32_t BRAKE_PULSE_MS    = 80UL; // Active L298N short-brake duration before coast
constexpr int8_t   ZONA_MUERTA       = 5;    // |pct| below this stops wheel

// ── Core 2.x / 3.x LEDC Compatibility Helpers ────────────────────────────────
static inline void pwmAttach(uint8_t pin, uint32_t freq, uint8_t bits, uint8_t ch) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcAttachChannel(pin, freq, bits, ch);
#else
  ledcSetup(ch, freq, bits);
  ledcAttachPin(pin, ch);
#endif
}

static inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

// ── Per-Wheel DC Calibration Structure & NVS Persistence ─────────────────────
struct MotorCal {
  uint8_t pinA;
  uint8_t pinB;
  uint8_t chA;
  uint8_t chB;
  uint8_t inverted;    // 0 = normal, 1 = inverted
  uint8_t minDutyPct;  // 0..80% duty at ZONA_MUERTA (5%) to overcome static friction
  uint8_t maxDutyPct;  // 20..100% duty at 100% command for straight-line trim
};

constexpr MotorCal DEFAULT_MOTOR_CAL[NUM_WHEELS] = {
  // pinA,     pinB,     chA, chB, inv, minPct, maxPct
  { PIN_FL_A, PIN_FL_B, 0,   1,   0,   28,     100 }, // 0: FL (L298N Mini Left A)
  { PIN_RL_A, PIN_RL_B, 2,   3,   0,   28,     100 }, // 1: RL (L298N Mini Left B)
  { PIN_FR_A, PIN_FR_B, 4,   5,   1,   28,     100 }, // 2: FR (L298N Mini Right A)
  { PIN_RR_A, PIN_RR_B, 6,   7,   1,   28,     100 }  // 3: RR (L298N Mini Right B)
};

struct NvsMotorBlob {
  uint16_t magic;
  uint8_t  version;
  uint8_t  activeBrake;
  uint8_t  inverted[NUM_WHEELS];
  uint8_t  minDutyPct[NUM_WHEELS];
  uint8_t  maxDutyPct[NUM_WHEELS];
};

constexpr uint16_t NVS_MOTOR_MAGIC = 0xCA63U;
constexpr uint8_t  NVS_MOTOR_VER   = 1U;

// ── Class 1: Ultrasonic Sensor (ESP32 IRAM_ATTR ISR + 5-Sample Median FIFO) ──
enum EchoState : uint8_t { US_IDLE = 0, US_RISE, US_FALL, US_DONE };

struct EchoISR {
  uint8_t           echoPin;
  volatile uint8_t  state;
  volatile uint32_t riseUs;
  volatile uint16_t pulseUs;
  portMUX_TYPE      mux;
};

static EchoISR g_ec1 = { PIN_ECHO1, US_IDLE, 0, 0, portMUX_INITIALIZER_UNLOCKED };
static EchoISR g_ec2 = { PIN_ECHO2, US_IDLE, 0, 0, portMUX_INITIALIZER_UNLOCKED };

IRAM_ATTR static void echoIsrHandler(void* arg) {
  EchoISR* ec = static_cast<EchoISR*>(arg);
  const uint32_t now = (uint32_t)micros();
  const int level = gpio_get_level((gpio_num_t)ec->echoPin);
  portENTER_CRITICAL_ISR(&ec->mux);
  if (ec->state == US_RISE && level == 1) {
    ec->riseUs = now;
    ec->state  = US_FALL;
  } else if (ec->state == US_FALL && level == 0) {
    ec->pulseUs = (uint16_t)(now - ec->riseUs);
    ec->state   = US_DONE;
  }
  portEXIT_CRITICAL_ISR(&ec->mux);
}

class UltrasonicSensor {
  uint8_t  _trig, _echo;
  EchoISR* _isr;
  uint16_t _fifo[5] = { 4000, 4000, 4000, 4000, 4000 };
  uint32_t _trigUs  = 0;
  bool     _busy    = false;

  void _pushMm(uint16_t mm) {
    for (uint8_t i = 0; i < 4; i++) _fifo[i] = _fifo[i + 1];
    _fifo[4] = mm;
  }

public:
  UltrasonicSensor(uint8_t t, uint8_t e, EchoISR* ch) : _trig(t), _echo(e), _isr(ch) {}

  void init() {
    pinMode(_trig, OUTPUT);
    digitalWrite(_trig, LOW);
    pinMode(_echo, INPUT);
    portENTER_CRITICAL(&_isr->mux);
    _isr->echoPin = _echo;
    _isr->state   = US_IDLE;
    portEXIT_CRITICAL(&_isr->mux);
    attachInterruptArg(_echo, echoIsrHandler, _isr, CHANGE);
  }

  bool busy() const { return _busy; }

  void trigger() {
    if (gpio_get_level((gpio_num_t)_echo) == 1) return;
    portENTER_CRITICAL(&_isr->mux);
    _isr->state = US_RISE;
    portEXIT_CRITICAL(&_isr->mux);
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);
    _trigUs = (uint32_t)micros();
    _busy   = true;
  }

  bool poll() {
    if (!_busy) return false;
    uint8_t st;
    uint16_t p;
    portENTER_CRITICAL(&_isr->mux);
    st = _isr->state;
    p  = _isr->pulseUs;
    if (st == US_DONE) _isr->state = US_IDLE;
    portEXIT_CRITICAL(&_isr->mux);

    if (st == US_DONE) {
      _busy = false;
      // Fixed-point speed of sound: pulse_us * 0.171875 mm/us = (pulse_us * 11) >> 6
      _pushMm((p == 0 || p > US_TIMEOUT_US) ? 4000U : (uint16_t)(((uint32_t)p * 11UL) >> 6));
      return true;
    }
    if ((uint32_t)((uint32_t)micros() - _trigUs) >= US_TIMEOUT_US) {
      portENTER_CRITICAL(&_isr->mux);
      _isr->state = US_IDLE;
      portEXIT_CRITICAL(&_isr->mux);
      _busy = false;
      _pushMm(4000U);
      return true;
    }
    return false;
  }

  uint16_t medianMm() const {
    uint16_t c[5] = { _fifo[0], _fifo[1], _fifo[2], _fifo[3], _fifo[4] };
    for (uint8_t i = 1; i < 5; i++) {
      uint16_t k = c[i];
      int8_t j = (int8_t)i - 1;
      while (j >= 0 && c[j] > k) {
        c[j + 1] = c[j];
        j--;
      }
      c[j + 1] = k;
    }
    return c[2];
  }
};

// ── Class 2: GY-511 LSM303DLHC 2D Compass (DRDY + Median3 + Hard/Soft Iron) ─
constexpr int32_t IMU_MIN_SPAN_LSB = 120; // Min X/Y span for valid 360 calibration

class ImuSensor {
  static constexpr uint8_t MAG_ADDR      = 0x1E;
  static constexpr uint8_t REG_CRA_M     = 0x00;
  static constexpr uint8_t REG_CRB_M     = 0x01;
  static constexpr uint8_t REG_MR_M      = 0x02;
  static constexpr uint8_t REG_OUT_X_H_M = 0x03;
  static constexpr uint8_t REG_SR_M      = 0x09;

  bool     _ok           = false;
  bool     _hasNvsCal    = false;
  bool     _turnIsRight  = true;
  bool     _calActive    = false;
  bool     _calHasAnchor = false;
  uint8_t  _fails        = 0;
  uint8_t  _histIdx      = 0;
  uint8_t  _calQuadMask  = 0;
  int8_t   _cwSign       = 1;   // +1 if atan2f heading increases on clockwise (right) spin, -1 otherwise
  int16_t  _rawX         = 0, _rawY = 0;
  int16_t  _filtX        = 0, _filtY = 0;
  int16_t  _hx[3]        = { 0, 0, 0 };
  int16_t  _hy[3]        = { 0, 0, 0 };
  int16_t  _calMinX      = 32767, _calMaxX = -32767;
  int16_t  _calMinY      = 32767, _calMaxY = -32767;
  float    _offX         = 0.0f, _offY = 0.0f;
  float    _scaleX       = 1.0f, _scaleY = 1.0f;
  float    _yawDeg       = 0.0f;
  float    _turnDeg      = 0.0f;
  float    _calAnchorX   = 0.0f, _calAnchorY = 0.0f;
  float    _calPrevX     = 0.0f, _calPrevY = 0.0f;
  float    _calSignedArea = 0.0f;
  uint32_t _lastSampleUs = 0;
  uint32_t _calStartMs   = 0;

  static int16_t _median3(int16_t a, int16_t b, int16_t c) {
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (b > c) { int16_t t = b; b = c; c = t; }
    if (a > b) { int16_t t = a; a = b; b = t; }
    return b;
  }

  void _seedFilter(int16_t x, int16_t y) {
    for (uint8_t i = 0; i < 3; i++) {
      _hx[i] = x;
      _hy[i] = y;
    }
    _histIdx = 0;
    _filtX   = x;
    _filtY   = y;
  }

  void _pushMedian3(int16_t x, int16_t y, int16_t &medX, int16_t &medY) {
    _hx[_histIdx] = x;
    _hy[_histIdx] = y;
    _histIdx = (uint8_t)((_histIdx + 1U) % 3U);
    medX = _median3(_hx[0], _hx[1], _hx[2]);
    medY = _median3(_hy[0], _hy[1], _hy[2]);
  }

  static bool _writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission(true) == 0;
  }

  static bool _readDrdy(bool &ready) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(REG_SR_M);
    if (Wire.endTransmission(true) != 0) return false;
    if (Wire.requestFrom(MAG_ADDR, (uint8_t)1) != 1) return false;
    uint8_t sr = (uint8_t)Wire.read();
    ready = (sr & 0x01U) != 0;
    return true;
  }

  static bool _readMag(int16_t &x, int16_t &y) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(REG_OUT_X_H_M);
    if (Wire.endTransmission(true) != 0) return false;
    if (Wire.requestFrom(MAG_ADDR, (uint8_t)6) != 6) return false;

    // LSM303DLHC register order: X_H, X_L, Z_H, Z_L, Y_H, Y_L
    uint8_t xh = (uint8_t)Wire.read();
    uint8_t xl = (uint8_t)Wire.read();
    (void)Wire.read(); // Skip Z_H
    (void)Wire.read(); // Skip Z_L
    uint8_t yh = (uint8_t)Wire.read();
    uint8_t yl = (uint8_t)Wire.read();

    int16_t rx = (int16_t)(((uint16_t)xh << 8) | xl);
    int16_t ry = (int16_t)(((uint16_t)yh << 8) | yl);
    if (rx == -4096 || ry == -4096 || (rx == 0 && ry == 0)) return false;

    x = rx;
    y = ry;
    return true;
  }

  float _calcHeadingDeg(int16_t x, int16_t y) const {
    float cx = ((float)x - _offX) * _scaleX;
    float cy = ((float)y - _offY) * _scaleY;
    if (cx == 0.0f && cy == 0.0f) return _yawDeg;
    float deg = atan2f(cy, cx) * (180.0f / 3.14159265f);
    if (deg < 0.0f) deg += 360.0f;
    if (deg >= 360.0f) deg -= 360.0f;
    return deg;
  }

  void _saveCalNvs() {
    Preferences prefs;
    if (!prefs.begin("imu", false)) return;
    prefs.putBool("calOk", _hasNvsCal);
    prefs.putFloat("offX", _offX);
    prefs.putFloat("offY", _offY);
    prefs.putFloat("scaleX", _scaleX);
    prefs.putFloat("scaleY", _scaleY);
    prefs.putChar("cwSign", _cwSign);
    prefs.end();
  }

public:
  void loadCalFromNvs() {
    _offX = 0.0f;
    _offY = 0.0f;
    _scaleX = 1.0f;
    _scaleY = 1.0f;
    _cwSign = 1;
    _hasNvsCal = false;

    Preferences prefs;
    if (!prefs.begin("imu", true)) return;
    bool calOk  = prefs.getBool("calOk", false);
    float ox    = prefs.getFloat("offX", 0.0f);
    float oy    = prefs.getFloat("offY", 0.0f);
    float sx    = prefs.getFloat("scaleX", 1.0f);
    float sy    = prefs.getFloat("scaleY", 1.0f);
    int8_t cw   = (int8_t)prefs.getChar("cwSign", 1);
    prefs.end();

    if (isfinite(ox) && isfinite(oy) && ox >= -2048.0f && ox <= 2048.0f && oy >= -2048.0f && oy <= 2048.0f &&
        isfinite(sx) && isfinite(sy) && sx >= 0.3f && sx <= 3.0f && sy >= 0.3f && sy <= 3.0f &&
        (cw == 1 || cw == -1)) {
      _offX      = ox;
      _offY      = oy;
      _scaleX    = sx;
      _scaleY    = sy;
      _cwSign    = cw;
      _hasNvsCal = calOk;
    }
  }

  void setManualOffsets(int16_t ox, int16_t oy) {
    _offX      = (float)ox;
    _offY      = (float)oy;
    _scaleX    = 1.0f;
    _scaleY    = 1.0f;
    _hasNvsCal = (ox != 0 || oy != 0);
    _saveCalNvs();
    _yawDeg = _calcHeadingDeg(_filtX, _filtY);
  }

  void setCwSign(int8_t cwSign) {
    _cwSign = (cwSign >= 0) ? 1 : -1;
    _saveCalNvs();
  }

  void resetCal() {
    _offX      = 0.0f;
    _offY      = 0.0f;
    _scaleX    = 1.0f;
    _scaleY    = 1.0f;
    _cwSign    = 1;
    _hasNvsCal = false;
    _saveCalNvs();
    _yawDeg = _calcHeadingDeg(_filtX, _filtY);
  }

  void init(bool reloadNvs = false) {
    if (reloadNvs) loadCalFromNvs();
    _ok        = false;
    _fails     = 0;
    _turnDeg   = 0.0f;
    _calActive = false;

    if (!_writeReg(REG_CRA_M, 0x18)) return; // 75 Hz data output rate
    if (!_writeReg(REG_CRB_M, 0x40)) return; // +/- 1.9 Gauss gain
    if (!_writeReg(REG_MR_M, 0x00))  return; // Continuous-conversion mode
    delay(15);

    int16_t x, y;
    if (_readMag(x, y)) {
      _ok   = true;
      _rawX = x;
      _rawY = y;
      _seedFilter(x, y);
      _yawDeg = _calcHeadingDeg(x, y);
    }
    _lastSampleUs = (uint32_t)micros();
  }

  void update() {
    if (!_ok) return;
    uint32_t nowUs = (uint32_t)micros();
    uint32_t dtUs  = nowUs - _lastSampleUs;
    if (dtUs < 10000UL) return; // Min 10 ms gate before polling DRDY (75 Hz = 13.33 ms)

    if (dtUs < 20000UL) {
      bool drdy = false;
      if (!_readDrdy(drdy)) {
        if (++_fails >= 10) _ok = false;
        return;
      }
      if (!drdy) return;
    }
    _lastSampleUs = nowUs;

    int16_t x, y;
    if (!_readMag(x, y)) {
      if (++_fails >= 10) _ok = false;
      return;
    }
    _fails = 0;
    _rawX  = x;
    _rawY  = y;

    int16_t mx = 0, my = 0;
    _pushMedian3(x, y, mx, my);
    _filtX = mx;
    _filtY = my;

    if (_calActive) {
      if (mx < _calMinX) _calMinX = mx;
      if (mx > _calMaxX) _calMaxX = mx;
      if (my < _calMinY) _calMinY = my;
      if (my > _calMaxY) _calMaxY = my;

      if (!_calHasAnchor) {
        _calAnchorX   = (float)mx;
        _calAnchorY   = (float)my;
        _calPrevX     = 0.0f;
        _calPrevY     = 0.0f;
        _calHasAnchor = true;
      } else {
        float dx = (float)mx - _calAnchorX;
        float dy = (float)my - _calAnchorY;
        _calSignedArea += (_calPrevX * dy - _calPrevY * dx);
        _calPrevX = dx;
        _calPrevY = dy;
      }

      if (((int32_t)_calMaxX - (int32_t)_calMinX) >= IMU_MIN_SPAN_LSB &&
          ((int32_t)_calMaxY - (int32_t)_calMinY) >= IMU_MIN_SPAN_LSB) {
        float cx = 0.5f * ((float)_calMinX + (float)_calMaxX);
        float cy = 0.5f * ((float)_calMinY + (float)_calMaxY);
        uint8_t q = (((float)mx >= cx) ? 1U : 0U) | (((float)my >= cy) ? 2U : 0U);
        _calQuadMask |= (uint8_t)(1U << q);
      }
    }

    float curYaw = _calcHeadingDeg(mx, my);
    float diff   = curYaw - _yawDeg;
    if (diff > 180.0f)       diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    _yawDeg   = curYaw;
    _turnDeg += diff;
  }

  void resetTurn(bool isRight = true) {
    _turnIsRight = isRight;
    _turnDeg     = 0.0f;
  }

  void beginCalibration() {
    _calActive     = true;
    _calHasAnchor  = false;
    _calMinX       = 32767;
    _calMaxX       = -32767;
    _calMinY       = 32767;
    _calMaxY       = -32767;
    _calQuadMask   = 0;
    _calSignedArea = 0.0f;
    _calStartMs    = millis();
  }

  bool calReady(uint32_t nowMs) const {
    return _calActive && (_calQuadMask == 0x0FU) && ((nowMs - _calStartMs) >= 2200UL);
  }

  void abortCalibration() { _calActive = false; }

  bool finishCalibration(int16_t &oxOut, int16_t &oyOut, float &sxOut, float &syOut, int8_t &cwOut) {
    _calActive = false;
    int32_t spanX = (int32_t)_calMaxX - (int32_t)_calMinX;
    int32_t spanY = (int32_t)_calMaxY - (int32_t)_calMinY;
    if (spanX < IMU_MIN_SPAN_LSB || spanY < IMU_MIN_SPAN_LSB || _calQuadMask != 0x0FU) {
      return false;
    }
    _offX   = 0.5f * ((float)_calMinX + (float)_calMaxX);
    _offY   = 0.5f * ((float)_calMinY + (float)_calMaxY);
    float rx = 0.5f * (float)spanX;
    float ry = 0.5f * (float)spanY;
    float rAvg = 0.5f * (rx + ry);
    _scaleX = constrain(rAvg / rx, 0.4f, 2.5f);
    _scaleY = constrain(rAvg / ry, 0.4f, 2.5f);
    if (fabsf(_calSignedArea) > 10.0f) {
      _cwSign = (_calSignedArea > 0.0f) ? 1 : -1;
    }
    _hasNvsCal = true;
    _saveCalNvs();
    _yawDeg  = _calcHeadingDeg(_filtX, _filtY);
    _turnDeg = 0.0f;

    oxOut = (int16_t)lroundf(_offX);
    oyOut = (int16_t)lroundf(_offY);
    sxOut = _scaleX;
    syOut = _scaleY;
    cwOut = _cwSign;
    return true;
  }

  bool pollDiagnostic() {
    if (!_ok) init(false);
    int16_t x, y;
    if (_ok && _readMag(x, y)) {
      _rawX = x;
      _rawY = y;
      return true;
    }
    return false;
  }

  bool ok() const { return _ok; }
  bool hasCal() const { return _hasNvsCal; }
  int8_t cwSign() const { return _cwSign; }

  int32_t yawDdeg() const {
    long d = lroundf(_yawDeg * 10.0f) % 3600L;
    if (d < 0) d += 3600L;
    return (int32_t)d;
  }

  int8_t expectedSign() const { return _turnIsRight ? _cwSign : (int8_t)(-_cwSign); }
  float signedTurnDeg() const { return _turnDeg * (float)expectedSign(); }
  int32_t turnDdeg() const { return (int32_t)lroundf(_turnDeg * 10.0f); }

  int16_t progressDdeg() const {
    float p = signedTurnDeg();
    if (p <= 0.0f) return 0;
    long d = lroundf(p * 10.0f);
    if (d > 32767L) d = 32767L;
    return (int16_t)d;
  }

  int16_t rawX() const { return _rawX; }
  int16_t rawY() const { return _rawY; }
  int16_t offX() const { return (int16_t)lroundf(_offX); }
  int16_t offY() const { return (int16_t)lroundf(_offY); }
  float scaleX() const { return _scaleX; }
  float scaleY() const { return _scaleY; }
};

// ── Class 3: 2x L298N Mini (8-GPIO LEDC) + Direct ESP32 LEDC 2-DOF Arm ───────
class Actuators {
  MotorCal _m[NUM_WHEELS];
  int8_t   _lPct = 0, _rPct = 0;
  bool     _activeBrake = true;
  bool     _braking     = false;
  bool     _nvsLoaded   = false;
  uint32_t _brakeStartMs = 0;

  uint8_t  _curB = ARM_B_HOME, _curE = ARM_E_HOME;
  uint8_t  _tgtB = ARM_B_HOME, _tgtE = ARM_E_HOME;
  uint32_t _armMs = 0;

  static int16_t _degToUs(uint8_t deg) {
    return (int16_t)(ARM_MIN_US + (((int32_t)(ARM_MAX_US - ARM_MIN_US) * deg + 90L) / 180L));
  }

  static uint32_t _usToServoDuty(int16_t us) {
    int16_t c = constrain(us, 500, 2500);
    return (uint32_t)(((uint32_t)c * SERVO_DUTY_RANGE + (SERVO_PWM_PERIOD_US / 2UL)) / SERVO_PWM_PERIOD_US);
  }

  static void _writeMotorBridge(const MotorCal &m, uint32_t dutyA, uint32_t dutyB) {
    pwmWrite(m.pinA, m.chA, dutyA);
    pwmWrite(m.pinB, m.chB, dutyB);
  }

  void _coastAll() {
    for (uint8_t i = 0; i < NUM_WHEELS; i++) _writeMotorBridge(_m[i], 0, 0);
    _braking = false;
  }

  void _driveWheelMapped(uint8_t idx, int8_t pct) {
    const MotorCal &m = _m[idx];
    if (pct > -ZONA_MUERTA && pct < ZONA_MUERTA) {
      _writeMotorBridge(m, 0, 0);
      return;
    }
    uint16_t a   = (uint16_t)(pct < 0 ? -pct : pct);
    bool     fwd = (pct >= 0);
    if (m.inverted != 0) fwd = !fwd;

    uint32_t dutyPct = m.minDutyPct +
      (((uint32_t)(m.maxDutyPct - m.minDutyPct) * (a - (uint16_t)ZONA_MUERTA)) / (100UL - (uint32_t)ZONA_MUERTA));
    uint32_t duty = (dutyPct * MOTOR_PWM_MAX + 50UL) / 100UL;
    if (duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;

    if (fwd) _writeMotorBridge(m, duty, 0);
    else     _writeMotorBridge(m, 0, duty);
  }

public:
  void loadDefaults() {
    for (uint8_t i = 0; i < NUM_WHEELS; i++) _m[i] = DEFAULT_MOTOR_CAL[i];
    _activeBrake = true;
    _nvsLoaded   = false;
  }

  bool loadFromNvs() {
    loadDefaults();
    Preferences prefs;
    if (!prefs.begin("rover", true)) return false;
    NvsMotorBlob blob{};
    size_t len = prefs.getBytes("cal", &blob, sizeof(blob));
    prefs.end();
    if (len != sizeof(blob) || blob.magic != NVS_MOTOR_MAGIC || blob.version != NVS_MOTOR_VER) {
      return false;
    }
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      if (blob.inverted[i] > 1 || blob.minDutyPct[i] > 80 ||
          blob.maxDutyPct[i] < 20 || blob.maxDutyPct[i] > 100 ||
          blob.minDutyPct[i] > blob.maxDutyPct[i]) {
        return false;
      }
    }
    _activeBrake = (blob.activeBrake != 0);
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      _m[i].inverted   = blob.inverted[i];
      _m[i].minDutyPct = blob.minDutyPct[i];
      _m[i].maxDutyPct = blob.maxDutyPct[i];
    }
    _nvsLoaded = true;
    return true;
  }

  void saveToNvs() const {
    NvsMotorBlob blob{};
    blob.magic       = NVS_MOTOR_MAGIC;
    blob.version     = NVS_MOTOR_VER;
    blob.activeBrake = _activeBrake ? 1U : 0U;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      blob.inverted[i]   = _m[i].inverted;
      blob.minDutyPct[i] = _m[i].minDutyPct;
      blob.maxDutyPct[i] = _m[i].maxDutyPct;
    }
    Preferences prefs;
    if (prefs.begin("rover", false)) {
      prefs.putBytes("cal", &blob, sizeof(blob));
      prefs.end();
    }
  }

  void invalidateNvs() {
    Preferences prefs;
    if (prefs.begin("rover", false)) {
      prefs.remove("cal");
      prefs.end();
    }
    loadDefaults();
  }

  void init() {
    loadFromNvs();
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      pwmAttach(_m[i].pinA, MOTOR_PWM_HZ, MOTOR_PWM_BITS, _m[i].chA);
      pwmAttach(_m[i].pinB, MOTOR_PWM_HZ, MOTOR_PWM_BITS, _m[i].chB);
      _writeMotorBridge(_m[i], 0, 0);
    }
    pwmAttach(PIN_ARM_B, SERVO_PWM_HZ, SERVO_PWM_BITS, CH_ARM_B);
    pwmAttach(PIN_ARM_E, SERVO_PWM_HZ, SERVO_PWM_BITS, CH_ARM_E);
    pwmWrite(PIN_ARM_B, CH_ARM_B, _usToServoDuty(_degToUs(_curB)));
    pwmWrite(PIN_ARM_E, CH_ARM_E, _usToServoDuty(_degToUs(_curE)));
  }

  void brakeStop() {
    _lPct = 0;
    _rPct = 0;
    if (_activeBrake) {
      for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        _writeMotorBridge(_m[i], MOTOR_PWM_MAX, MOTOR_PWM_MAX);
      }
      _braking      = true;
      _brakeStartMs = millis();
    } else {
      _coastAll();
    }
  }

  void setWheels(int16_t l, int16_t r) {
    bool wasMoving = (_lPct != 0 || _rPct != 0);
    int8_t cl = (int8_t)constrain(l, -100, 100);
    int8_t cr = (int8_t)constrain(r, -100, 100);
    _lPct = (cl > -ZONA_MUERTA && cl < ZONA_MUERTA) ? 0 : cl;
    _rPct = (cr > -ZONA_MUERTA && cr < ZONA_MUERTA) ? 0 : cr;

    if (_lPct == 0 && _rPct == 0) {
      if (wasMoving) brakeStop();
      else if (!_braking) _coastAll();
      return;
    }

    _braking = false;
    _driveWheelMapped(W_FL, _lPct);
    _driveWheelMapped(W_RL, _lPct);
    _driveWheelMapped(W_FR, _rPct);
    _driveWheelMapped(W_RR, _rPct);
  }

  // Raw wheel duty test (bypasses min/max trim so static-friction threshold can be measured)
  void rawWheelPct(uint8_t idx, int16_t pct) {
    if (idx >= NUM_WHEELS) return;
    _braking = false;
    _lPct = 0;
    _rPct = 0;
    int16_t c = constrain(pct, -100, 100);
    if (c == 0) {
      _writeMotorBridge(_m[idx], 0, 0);
      return;
    }
    bool fwd = (c > 0);
    if (_m[idx].inverted != 0) fwd = !fwd;
    uint32_t duty = ((uint32_t)abs(c) * MOTOR_PWM_MAX + 50UL) / 100UL;
    if (duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
    if (fwd) _writeMotorBridge(_m[idx], duty, 0);
    else     _writeMotorBridge(_m[idx], 0, duty);
  }

  void setWheelInverted(uint8_t idx, bool inv) {
    if (idx >= NUM_WHEELS) return;
    _m[idx].inverted = inv ? 1U : 0U;
    if (_lPct != 0 || _rPct != 0) setWheels(_lPct, _rPct);
  }

  void setMaxDuties4(uint8_t mFL, uint8_t mRL, uint8_t mFR, uint8_t mRR) {
    uint8_t mx[NUM_WHEELS] = { mFL, mRL, mFR, mRR };
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      _m[i].maxDutyPct = mx[i];
      if (_m[i].minDutyPct > _m[i].maxDutyPct) _m[i].minDutyPct = _m[i].maxDutyPct;
    }
    if (_lPct != 0 || _rPct != 0) setWheels(_lPct, _rPct);
  }

  void setWheelRange(uint8_t idx, uint8_t minPct, uint8_t maxPct) {
    if (idx >= NUM_WHEELS) return;
    _m[idx].minDutyPct = minPct;
    _m[idx].maxDutyPct = maxPct;
    if (_lPct != 0 || _rPct != 0) setWheels(_lPct, _rPct);
  }

  void setActiveBrake(bool en) {
    _activeBrake = en;
    if (!en && _braking) _coastAll();
  }

  void setArm(int16_t b, int16_t e) {
    _tgtB = (uint8_t)constrain(b, 0, 180);
    _tgtE = (uint8_t)constrain(e, 0, 180);
  }

  void stopAll() {
    brakeStop();
    _tgtB = _curB;
    _tgtE = _curE;
  }

  void update(uint32_t nowMs) {
    if (_braking && (nowMs - _brakeStartMs >= BRAKE_PULSE_MS)) {
      _coastAll();
    }
    if ((_curB == _tgtB && _curE == _tgtE) || (nowMs - _armMs < 15UL)) return;
    _armMs = nowMs;
    if (_curB != _tgtB) {
      int16_t diff = (int16_t)_tgtB - (int16_t)_curB;
      _curB = (uint8_t)((int16_t)_curB + constrain(diff, -2, 2));
      pwmWrite(PIN_ARM_B, CH_ARM_B, _usToServoDuty(_degToUs(_curB)));
    }
    if (_curE != _tgtE) {
      int16_t diff = (int16_t)_tgtE - (int16_t)_curE;
      _curE = (uint8_t)((int16_t)_curE + constrain(diff, -2, 2));
      pwmWrite(PIN_ARM_E, CH_ARM_E, _usToServoDuty(_degToUs(_curE)));
    }
  }

  const MotorCal& motor(uint8_t idx) const { return _m[idx < NUM_WHEELS ? idx : 0]; }
  bool activeBrake() const { return _activeBrake; }
  bool nvsLoaded() const { return _nvsLoaded; }
  int8_t left() const { return _lPct; }
  int8_t right() const { return _rPct; }
  uint8_t armB() const { return _curB; }
  uint8_t armE() const { return _curE; }
  bool movingFwd() const {
    return (_lPct > 0 && _rPct >= 0) || (_rPct > 0 && _lPct >= 0);
  }
};

// ── Global Hardware Instances & Compact State Machine ────────────────────────
enum Mode : uint8_t { M_IDLE = 0, M_DRIVE, M_GYRO, M_REV, M_CAL, M_CAL_IMU };

static UltrasonicSensor us1(PIN_TRIG1, PIN_ECHO1, &g_ec1);
static UltrasonicSensor us2(PIN_TRIG2, PIN_ECHO2, &g_ec2);
static ImuSensor        imu;
static Actuators        act;

static Mode     mode        = M_IDLE;
static uint32_t tUs = 0, tState = 0, tFallback = 0, tMaxTurn = 6000UL;
static int16_t  tgtTurnDdeg = 0;
static bool     turnRight   = true, toggleUs = false, obsLatched = false, rxOvf = false;
static char     rxBuf[64];
static uint8_t  rxIdx       = 0;

static uint16_t minUsMm() {
  uint16_t a = us1.medianMm(), b = us2.medianMm();
  return a < b ? a : b;
}

static void startAutoRev(uint16_t mm, bool blockedCmd) {
  obsLatched = true;
  act.setWheels(-50, -50);
  mode   = M_REV;
  tState = millis();
  Serial.print(blockedCmd ? F("[ACK] BLK|") : F("[EVENT] OBS|"));
  Serial.println(mm);
}

static void startGyroTurn(bool isRight, int16_t absDdeg) {
  if (!imu.ok()) imu.init(false);
  turnRight   = isRight;
  tgtTurnDdeg = absDdeg;
  tFallback   = constrain(((uint32_t)tgtTurnDdeg * 220UL) / 90UL, 150UL, 15000UL);
  tMaxTurn    = constrain(tFallback + 3000UL, 6000UL, 18000UL);
  imu.resetTurn(turnRight);
  act.setWheels(turnRight ? TURN_MAX_SPD : -TURN_MAX_SPD, turnRight ? -TURN_MAX_SPD : TURN_MAX_SPD);
  tState = millis();
  mode   = M_GYRO;
  Serial.print(F("[ACK] G|"));
  Serial.print(turnRight ? 'R' : 'L');
  Serial.print('|');
  Serial.println(tgtTurnDdeg);
}

static uint8_t parseInts(const char* s, int16_t* out, uint8_t maxCount) {
  uint8_t count = 0;
  const char* p = s;
  while (*p != '\0') {
    while (*p == ' ') p++;
    if (*p == '\0') break;
    if (count >= maxCount) return 0;
    char* endP = nullptr;
    long v = strtol(p, &endP, 10);
    if (endP == p) return 0;
    out[count++] = (int16_t)v;
    p = endP;
  }
  return count;
}

static bool parseTwoInts(const char* s, int16_t &a, int16_t &b) {
  int16_t tmp[2];
  if (parseInts(s, tmp, 2) != 2) return false;
  a = tmp[0];
  b = tmp[1];
  return true;
}

static void printStatusTelemetry() {
  Serial.print(F("[STATUS] "));
  Serial.print(act.left());       Serial.print(',');
  Serial.print(act.right());      Serial.print(',');
  Serial.print(act.armB());       Serial.print(',');
  Serial.print(act.armE());       Serial.print(',');
  Serial.print(imu.yawDdeg());    Serial.print(',');
  Serial.print(imu.ok() ? 1 : 0); Serial.print(',');
  Serial.print(us1.medianMm());   Serial.print(',');
  Serial.print(us2.medianMm());   Serial.print(',');
  Serial.println((uint8_t)mode);
}

static void handleOpcode(const char* cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') return;

  // ── Spanish Direct Commands (UART / Serial Monitor) ───────────────────────
  if (strncasecmp(cmd, "ADELANTE", 8) == 0 && (cmd[8] == ' ' || cmd[8] == '\0')) {
    const char* p = cmd + 8;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP = nullptr;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] ADELANTE <0-100>")); return; }
      spd = (int16_t)v;
    }
    imu.abortCalibration();
    if (spd >= ZONA_MUERTA) {
      uint16_t mm = minUsMm();
      if (mm > 10 && mm < OBS_TRIG_MM) { startAutoRev(mm, true); return; }
      obsLatched = false;
    }
    act.setWheels(spd, spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|")); Serial.print(spd); Serial.print('|'); Serial.println(spd);
    return;
  }

  if (strncasecmp(cmd, "ATRAS", 5) == 0 && (cmd[5] == ' ' || cmd[5] == '\0')) {
    const char* p = cmd + 5;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP = nullptr;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] ATRAS <0-100>")); return; }
      spd = (int16_t)v;
    }
    imu.abortCalibration();
    act.setWheels(-spd, -spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|-")); Serial.print(spd); Serial.print(F("|-")); Serial.println(spd);
    return;
  }

  if (strncasecmp(cmd, "GIRARD", 6) == 0 && (cmd[6] == ' ' || cmd[6] == '\0')) {
    const char* p = cmd + 6;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP = nullptr;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] GIRARD <1-100>")); return; }
      spd = (int16_t)v;
    }
    imu.abortCalibration();
    act.setWheels(spd, -spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|")); Serial.print(spd); Serial.print(F("|-")); Serial.println(spd);
    return;
  }

  if (strncasecmp(cmd, "GIRARI", 6) == 0 && (cmd[6] == ' ' || cmd[6] == '\0')) {
    const char* p = cmd + 6;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP = nullptr;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] GIRARI <1-100>")); return; }
      spd = (int16_t)v;
    }
    imu.abortCalibration();
    act.setWheels(-spd, spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|-")); Serial.print(spd); Serial.print('|'); Serial.println(spd);
    return;
  }

  if (strcasecmp(cmd, "BRAZOEXT") == 0) {
    act.setArm(90, 145);
    Serial.println(F("[ACK] A|90|145"));
    return;
  }

  if (strcasecmp(cmd, "BRAZOH") == 0) {
    act.setArm(ARM_B_HOME, ARM_E_HOME);
    Serial.print(F("[ACK] A|")); Serial.print(ARM_B_HOME); Serial.print('|'); Serial.println(ARM_E_HOME);
    return;
  }

  if (strncasecmp(cmd, "BRAZO ", 6) == 0) {
    int16_t b, e;
    if (parseTwoInts(cmd + 6, b, e) && b >= 0 && b <= 180 && e >= 0 && e <= 180) {
      act.setArm(b, e);
      Serial.print(F("[ACK] A|")); Serial.print(b); Serial.print('|'); Serial.println(e);
    } else Serial.println(F("[ERROR] BRAZO <base> <codo>"));
    return;
  }

  if (strcasecmp(cmd, "ALTO") == 0) {
    imu.abortCalibration();
    act.stopAll();
    mode = M_IDLE;
    Serial.println(F("[ACK] X"));
    return;
  }

  if (strcasecmp(cmd, "ESTADO") == 0) {
    printStatusTelemetry();
    return;
  }

  if (strncasecmp(cmd, "GIROD", 5) == 0 || strncasecmp(cmd, "GIROI", 5) == 0) {
    bool isR = (cmd[4] == 'D' || cmd[4] == 'd');
    const char* p = cmd + 5;
    while (*p == ' ') p++;
    int16_t deg = 0;
    if (parseInts(p, &deg, 1) == 1 && deg > 0 && deg <= 360) {
      imu.abortCalibration();
      startGyroTurn(isR, (int16_t)(deg * 10));
    } else {
      Serial.println(F("[ERROR] GIROD/GIROI <1-360>"));
    }
    return;
  }

  char op = cmd[0];
  if (op >= 'a' && op <= 'z') op = (char)(op - 32);
  const char* arg = cmd + 1;

  switch (op) {
    case 'X': {
      while (*arg == ' ') arg++;
      if (*arg != '\0') { Serial.println(F("[ERROR] OP")); break; }
      imu.abortCalibration();
      act.stopAll();
      mode = M_IDLE;
      Serial.println(F("[ACK] X"));
      break;
    }
    case '?': {
      while (*arg == ' ') arg++;
      if (*arg != '\0') { Serial.println(F("[ERROR] OP")); break; }
      printStatusTelemetry();
      break;
    }
    case 'A': {
      int16_t b, e;
      if (parseTwoInts(arg, b, e) && b >= 0 && b <= 180 && e >= 0 && e <= 180) {
        act.setArm(b, e);
        Serial.print(F("[ACK] A|")); Serial.print(b); Serial.print('|'); Serial.println(e);
      } else Serial.println(F("[ERROR] A <b> <e>"));
      break;
    }
    case 'W': {
      int16_t l, r;
      if (parseTwoInts(arg, l, r) && l >= -100 && l <= 100 && r >= -100 && r <= 100) {
        imu.abortCalibration();
        // Block only net forward translation so in-place pivot turns remain possible near obstacles
        if ((l >= ZONA_MUERTA && r >= 0) || (r >= ZONA_MUERTA && l >= 0)) {
          uint16_t mm = minUsMm();
          if (mm > 10 && mm < OBS_TRIG_MM) { startAutoRev(mm, true); return; }
          obsLatched = false;
        }
        act.setWheels(l, r);
        mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
        Serial.print(F("[ACK] W|")); Serial.print(l); Serial.print('|'); Serial.println(r);
      } else Serial.println(F("[ERROR] W <l> <r>"));
      break;
    }
    case 'G': {
      int16_t ddeg;
      if (parseInts(arg, &ddeg, 1) == 1 && ddeg != 0 && ddeg >= -3600 && ddeg <= 3600) {
        imu.abortCalibration();
        startGyroTurn(ddeg > 0, (int16_t)abs(ddeg));
      } else Serial.println(F("[ERROR] G <ddeg>"));
      break;
    }

    // ── Online Calibration Commands (Serial Monitor) ────────────────────────
    case 'P': { // P <0-3> <-100..100> : Raw duty test on single wheel
      int16_t v[2];
      if (parseInts(arg, v, 2) == 2 && v[0] >= 0 && v[0] < NUM_WHEELS && v[1] >= -100 && v[1] <= 100) {
        imu.abortCalibration();
        act.rawWheelPct((uint8_t)v[0], v[1]);
        mode = M_CAL;
        Serial.print(F("[ACK] P|")); Serial.print(v[0]); Serial.print('|'); Serial.println(v[1]);
      } else Serial.println(F("[ERROR] P <0-3> <-100..100>"));
      break;
    }
    case 'V': { // V <0-3> <0|1> : Polarity inversion per wheel
      int16_t v[2];
      if (parseInts(arg, v, 2) == 2 && v[0] >= 0 && v[0] < NUM_WHEELS && (v[1] == 0 || v[1] == 1)) {
        act.setWheelInverted((uint8_t)v[0], v[1] == 1);
        Serial.print(F("[ACK] V|")); Serial.print(v[0]); Serial.print('|'); Serial.println(v[1]);
      } else Serial.println(F("[ERROR] V <0-3> <0|1>"));
      break;
    }
    case 'M': { // M <fl> <rl> <fr> <rr> OR M <0-3> <minPct> <maxPct>
      int16_t v[4];
      uint8_t n = parseInts(arg, v, 4);
      if (n == 4 && v[0] >= 20 && v[0] <= 100 && v[1] >= 20 && v[1] <= 100 &&
          v[2] >= 20 && v[2] <= 100 && v[3] >= 20 && v[3] <= 100) {
        act.setMaxDuties4((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3]);
        Serial.print(F("[ACK] M|"));
        Serial.print(v[0]); Serial.print('|'); Serial.print(v[1]); Serial.print('|');
        Serial.print(v[2]); Serial.print('|'); Serial.println(v[3]);
      } else if (n == 3 && v[0] >= 0 && v[0] < NUM_WHEELS &&
                 v[1] >= 0 && v[1] <= 80 && v[2] >= 20 && v[2] <= 100 && v[1] <= v[2]) {
        act.setWheelRange((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2]);
        Serial.print(F("[ACK] M|")); Serial.print(v[0]); Serial.print('|');
        Serial.print(v[1]); Serial.print('|'); Serial.println(v[2]);
      } else Serial.println(F("[ERROR] M <fl> <rl> <fr> <rr> | M <0-3> <min> <max>"));
      break;
    }
    case 'B': { // B <0|1> : Toggle active short-brake pulse
      while (*arg == ' ') arg++;
      if ((arg[0] == '0' || arg[0] == '1') && arg[1] == '\0') {
        act.setActiveBrake(arg[0] == '1');
        Serial.print(F("[ACK] B|")); Serial.println(arg[0]);
      } else Serial.println(F("[ERROR] B <0|1>"));
      break;
    }
    case 'E': { // E = save motor cal to NVS; E 0 = restore defaults
      while (*arg == ' ') arg++;
      if (*arg == '\0') {
        act.saveToNvs();
        Serial.println(F("[ACK] E|SAVED"));
      } else if (arg[0] == '0' && arg[1] == '\0') {
        act.invalidateNvs();
        Serial.println(F("[ACK] E|DEFAULT"));
      } else Serial.println(F("[ERROR] E [0]"));
      break;
    }
    case 'I': { // I | I C | I P <+1|-1> | I <ox> <oy> | I 0
      while (*arg == ' ') arg++;
      if (*arg == '\0') {
        imu.pollDiagnostic();
        Serial.print(F("[IMU] ok="));  Serial.print(imu.ok() ? 1 : 0);
        Serial.print(F(",raw=("));     Serial.print(imu.rawX()); Serial.print(','); Serial.print(imu.rawY());
        Serial.print(F("),cal=("));    Serial.print(imu.rawX() - imu.offX()); Serial.print(','); Serial.print(imu.rawY() - imu.offY());
        Serial.print(F("),off=("));    Serial.print(imu.offX()); Serial.print(','); Serial.print(imu.offY());
        Serial.print(F("),sc=("));     Serial.print(imu.scaleX(), 2); Serial.print(','); Serial.print(imu.scaleY(), 2);
        Serial.print(F("),cw="));      Serial.print(imu.cwSign());
        Serial.print(F(",yaw="));      Serial.print(imu.yawDdeg());
        Serial.print(F(",turn="));     Serial.println(imu.turnDdeg());
      } else if ((arg[0] == 'C' || arg[0] == 'c') && arg[1] == '\0') {
        if (!imu.ok()) imu.init(false);
        if (!imu.ok()) { Serial.println(F("[ERROR] IMU")); break; }
        imu.beginCalibration();
        act.setWheels(TURN_CAL_SPD, -TURN_CAL_SPD);
        tState = millis();
        mode   = M_CAL_IMU;
        Serial.println(F("[ACK] I|CAL|START"));
      } else if ((arg[0] == 'P' || arg[0] == 'p') && (arg[1] == ' ' || arg[1] == '\0')) {
        int16_t s = 0;
        if (parseInts(arg + 1, &s, 1) == 1 && (s == 1 || s == -1)) {
          imu.setCwSign((int8_t)s);
          Serial.print(F("[ACK] I|P|")); Serial.println(imu.cwSign());
        } else Serial.println(F("[ERROR] I P <+1|-1>"));
      } else {
        int16_t v[2];
        uint8_t n = parseInts(arg, v, 2);
        if (n == 1 && v[0] == 0) {
          imu.resetCal();
          Serial.println(F("[ACK] I|0|0"));
        } else if (n == 2 && v[0] >= -2048 && v[0] <= 2048 && v[1] >= -2048 && v[1] <= 2048) {
          imu.setManualOffsets(v[0], v[1]);
          Serial.print(F("[ACK] I|")); Serial.print(v[0]); Serial.print('|'); Serial.println(v[1]);
        } else Serial.println(F("[ERROR] I [ox oy | C | P <+1|-1> | 0]"));
      }
      break;
    }
    case 'K': {
      Serial.print(F("[CAL] nvs=")); Serial.print(act.nvsLoaded() ? 1 : 0);
      Serial.print(F(",brk="));      Serial.print(act.activeBrake() ? 1 : 0);
      Serial.print(F(",imuCal="));   Serial.print(imu.hasCal() ? 1 : 0);
      for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        const MotorCal &m = act.motor(i);
        Serial.print(F(" | M"));   Serial.print(i);
        Serial.print(F("(inv="));  Serial.print(m.inverted);
        Serial.print(F(",min="));  Serial.print(m.minDutyPct);
        Serial.print(F(",max="));  Serial.print(m.maxDutyPct);
        Serial.print(')');
      }
      Serial.println();
      break;
    }
    case 'S': { // "S" or "STOP" in Serial Monitor
      while (*arg == ' ') arg++;
      if (*arg == '\0' || strcasecmp(arg, "TOP") == 0) {
        imu.abortCalibration();
        act.stopAll();
        mode = M_IDLE;
        Serial.println(F("[ACK] X"));
      } else Serial.println(F("[ERROR] OP"));
      break;
    }
    default:
      Serial.println(F("[ERROR] OP"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000UL);
  Wire.setTimeOut(50);

  act.init();
  us1.init();
  us2.init();
  imu.init(true);

  // Pre-fill the 5-sample median FIFO of both HC-SR04 sensors
  for (uint8_t i = 0; i < 5; i++) {
    us1.trigger();
    while (us1.busy()) us1.poll();
    delay(4);
    us2.trigger();
    while (us2.busy()) us2.poll();
    delay(4);
  }

  Serial.print(F("[READY] ESP32_ROVER|IMU=")); Serial.print(imu.ok() ? 1 : 0);
  Serial.print(F("|ARM=LEDC|NVS="));            Serial.println(act.nvsLoaded() ? 1 : 0);
}

void loop() {
  uint32_t now = millis();
  act.update(now);
  imu.update();

  if (mode == M_GYRO) {
    int16_t turned = imu.progressDdeg();
    bool done = imu.ok() ? (turned >= tgtTurnDdeg) : ((now - tState) >= tFallback);
    if (done || ((now - tState) >= tMaxTurn)) {
      act.brakeStop();
      mode = M_IDLE;
      Serial.print(F("[EVENT] TD|"));
      Serial.print(turnRight ? 'R' : 'L');
      Serial.print('|');
      Serial.println(imu.ok() ? turned : tgtTurnDdeg);
    } else if (imu.ok() && (tgtTurnDdeg - turned < TURN_SLOW_DDEG)) {
      int16_t rem = tgtTurnDdeg - turned;
      if (rem < 0) rem = 0;
      int16_t spd = TURN_MIN_SPD + (int16_t)(((int32_t)(TURN_MAX_SPD - TURN_MIN_SPD) * rem) / TURN_SLOW_DDEG);
      act.setWheels(turnRight ? spd : -spd, turnRight ? -spd : spd);
    }
  } else if (mode == M_CAL_IMU) {
    if (imu.calReady(now) || ((now - tState) >= 4000UL)) {
      act.brakeStop();
      mode = M_IDLE;
      int16_t ox = 0, oy = 0;
      float sx = 1.0f, sy = 1.0f;
      int8_t cw = 1;
      if (imu.finishCalibration(ox, oy, sx, sy, cw)) {
        Serial.print(F("[ACK] I|CAL|"));
        Serial.print(ox); Serial.print('|'); Serial.print(oy); Serial.print('|');
        Serial.print(sx, 2); Serial.print('|'); Serial.print(sy, 2);
        Serial.print(F("|cw=")); Serial.println(cw);
      } else {
        Serial.println(F("[ERROR] I|CAL"));
      }
    }
  }

  bool newUs = us1.poll() | us2.poll();
  if (mode != M_GYRO && mode != M_CAL_IMU && (now - tUs >= 30UL) && !us1.busy() && !us2.busy()) {
    tUs = now;
    if (!toggleUs) us1.trigger();
    else           us2.trigger();
    toggleUs = !toggleUs;
  }

  if (newUs) {
    uint16_t mm = minUsMm();
    if (mode == M_IDLE || (mode == M_DRIVE && act.movingFwd())) {
      if (mm > 10 && mm < OBS_TRIG_MM && !obsLatched) startAutoRev(mm, false);
      else if (mm >= OBS_CLEAR_MM) obsLatched = false;
    } else if (mm >= OBS_CLEAR_MM && mode != M_REV) {
      obsLatched = false;
    }
  }

  if (mode == M_REV) {
    uint32_t el = now - tState;
    uint16_t mm = minUsMm();
    if ((el >= 350UL && mm >= OBS_CLEAR_MM) || el >= 900UL) {
      act.brakeStop();
      mode = M_IDLE;
      if (mm >= OBS_CLEAR_MM) obsLatched = false;
      Serial.print(F("[EVENT] CLR|"));
      Serial.print(us1.medianMm());
      Serial.print('|');
      Serial.println(us2.medianMm());
    }
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxOvf) {
        rxIdx = 0;
        rxOvf = false;
        Serial.println(F("[ERROR] LEN"));
      } else if (rxIdx > 0) {
        rxBuf[rxIdx] = '\0';
        handleOpcode(rxBuf);
        rxIdx = 0;
      }
    } else if (rxIdx < sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    } else {
      rxOvf = true;
    }
  }
}
