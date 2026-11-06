/*
  arduino/rover_firmware/rover_firmware.ino
  Lean Real-Time Firmware for Arduino Uno (ATmega328P) — UA62 - Avance 2
  100% Integer Fixed-Point Math (Zero float emulation, zero printf/strtod bloat).

  Hardware & Bus Layout:
    - I2C Bus (400 kHz, SDA=A4, SCL=A5):
        * PCA9685 16-Ch 12-Bit PWM Driver @ 0x40 (50 Hz servo refresh):
            - CH 0 : Wheel FL (Front-Left  / Delantera Izquierda, Continuous MG996R, inverted)
            - CH 1 : Wheel RL (Rear-Left   / Trasera Izquierda,   Continuous MG996R, inverted)
            - CH 2 : Wheel FR (Front-Right / Delantera Derecha,   Continuous MG996R)
            - CH 3 : Wheel RR (Rear-Right  / Trasera Derecha,     Continuous MG996R)
            - CH 4 : Arm Base  (Standard MG996R, 0..180 deg, smooth ramp)
            - CH 5 : Arm Elbow (Standard MG996R, 0..180 deg, smooth ramp)
        * GY-511 (LSM303DLHC 3-Axis Magnetometer / Digital Compass @ 0x1E):
            - Pinout: VCC -> 5V, GND -> GND, SCL -> A5 (PC5), SDA -> A4 (PC4)
    - Ultrasonic Sensors (2x HC-SR04 via PCINT2, N=5 median FIFO):
        * US1 : TRIG1 = D2, ECHO1 = D3 (PCINT19)
        * US2 : TRIG2 = D4, ECHO2 = D5 (PCINT21)

  UART & Serial Commands (115200 baud, Spanish or Compact Opcodes):
    - ADELANTE [vel]        / W <l> <r>   : Avance recto / velocidad diferencial (-100..100 %)
    - ATRAS [vel]                         : Retroceso recto
    - GIRARD <vel>                        : Giro diferencial derecha (Izq=+vel, Der=-vel)
    - GIRARI <vel>                        : Giro diferencial izquierda (Izq=-vel, Der=+vel)
    - BRAZOEXT              / A <b> <e>   : Extensión de brazo / ángulos específicos (0..180 deg)
    - BRAZOH                              : Retorno de brazo a home
    - ALTO                  / X           : Parada inmediata (FULL_OFF o centerUs + brazo congelado)
    - ESTADO                / ?           : Consulta de telemetría STATUS
    - GIROD / GIROI <grados>/ G <ddeg>    : Giro exacto en lazo cerrado con brújula GY-511

  Online Calibration Commands (Arduino Serial Monitor @ 115200, "Newline"):
    - P <0-3> <us>          : Raw PWM (1000..2000 us) on single wheel (0=FL,1=RL,2=FR,3=RR)
    - L <us> / R <us>       : Raw PWM (1000..2000 us) on left pair (FL,RL) or right pair (FR,RR)
    - C <fl> <rl> <fr> <rr> : Set neutral centers (1300..1700 us) for all 4 wheels (or C <l> <r>)
    - M <fl> <rl> <fr> <rr> : Set 50% speed offsets (20..400 us) for all 4 wheels
    - M <0-3> <fwd> <rev>   : Set asymmetric 50% speed offsets (FWD/REV) for wheel 0..3
    - O [hz]                : Query or set PCA9685 real oscillator frequency (20000000..30000000 Hz)
    - D <0|1>               : Stop mode: 1 = PCA9685 FULL_OFF (0 pulses), 0 = send calibrated centerUs
    - E [0]                 : Save active calibration to EEPROM (E 0 = restore defaults)
    - K                     : Print active 4-wheel calibration & EEPROM/PCA9685 state
    - STOP (or S)           : Stop all wheels (same as X)
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// ── Hardware & PCA9685 Constants ─────────────────────────────────────────────
constexpr uint8_t  PIN_TRIG1 = 2, PIN_ECHO1 = 3; // D3 = PD3 (PCINT19)
constexpr uint8_t  PIN_TRIG2 = 4, PIN_ECHO2 = 5; // D5 = PD5 (PCINT21)

constexpr uint8_t  PCA9685_ADDR   = 0x40;
constexpr uint16_t SERVO_HZ       = 50;
constexpr uint32_t PCA_OSC_HZ     = 27000000UL; // Calibrated PCA9685 internal RC clock (27 MHz real -> prescale 131)
constexpr uint32_t PCA_OSC_HZ_MIN = 20000000UL;
constexpr uint32_t PCA_OSC_HZ_MAX = 30000000UL;
constexpr uint8_t  CH_WHEEL_FL    = 0; // Front-Left  (Delantera Izquierda)
constexpr uint8_t  CH_WHEEL_RL    = 1; // Rear-Left   (Trasera Izquierda)
constexpr uint8_t  CH_WHEEL_FR    = 2; // Front-Right (Delantera Derecha)
constexpr uint8_t  CH_WHEEL_RR    = 3; // Rear-Right  (Trasera Derecha)
constexpr uint8_t  CH_ARM_B       = 4; // Arm Base
constexpr uint8_t  CH_ARM_E       = 5; // Arm Elbow

constexpr uint8_t  NUM_WHEELS    = 4;
constexpr uint8_t  W_FL = 0, W_RL = 1, W_FR = 2, W_RR = 3;

constexpr uint8_t  ARM_B_HOME    = 90,   ARM_E_HOME    = 25;
constexpr int16_t  ARM_MIN_US    = 544,  ARM_MAX_US    = 2400;
constexpr int16_t  PWM_MIN_US    = 1000, PWM_MAX_US    = 2000;
constexpr uint16_t US_TIMEOUT_US = 15000U;
constexpr uint16_t OBS_TRIG_MM   = 200U;  // 20.0 cm
constexpr uint16_t OBS_CLEAR_MM  = 250U;  // 25.0 cm
constexpr uint8_t  TURN_MAX_SPD  = 50,   TURN_MIN_SPD  = 28;
constexpr int8_t   ZONA_MUERTA   = 5;     // |pct| below this is treated as 0 (STOP)
constexpr bool     STOP_CON_DETACH = true; // true: PCA9685 FULL_OFF on stop; false: send centerUs

// ── Per-Wheel Independent Calibration Structure & EEPROM Persistence ─────────
// Matches arduino/servo_calibration/servo_calibration.ino binary layout in EEPROM.
struct WheelCal {
  uint8_t channel;      // PCA9685 channel (0=FL, 1=RL, 2=FR, 3=RR)
  uint8_t inverted;     // 0 = normal, 1 = inverted
  int16_t centerUs;     // Exact neutral stop pulse in us (wheel does NOT creep)
  int16_t fwdStartOff;  // +us offset from centerUs where wheel starts spinning FWD (5%)
  int16_t revStartOff;  // +us offset from centerUs where wheel starts spinning REV (-5%)
  int16_t fwdMidOff;    // +us offset from centerUs for exact equalized 50% speed FWD
  int16_t revMidOff;    // +us offset from centerUs for exact equalized 50% speed REV
  int16_t fwdMaxOff;    // +us offset from centerUs for exact equalized 100% speed FWD
  int16_t revMaxOff;    // +us offset from centerUs for exact equalized 100% speed REV
};

// Default per-wheel calibration (used when EEPROM is unprogrammed or reset with `E 0`).
// Copy-paste the table printed by `servo_calibration.ino` (`SHOW`) here if desired.
constexpr WheelCal DEFAULT_WHEEL_CAL[NUM_WHEELS] = {
  // ch,          inv, center, fStart, rStart, fMid, rMid, fMax, rMax
  { CH_WHEEL_FL,   1,   1500,     30,     30,   95,   95,  180,  180 }, // 0: FL
  { CH_WHEEL_RL,   1,   1500,     30,     30,   95,   95,  180,  180 }, // 1: RL
  { CH_WHEEL_FR,   0,   1500,     30,     30,   95,   95,  180,  180 }, // 2: FR
  { CH_WHEEL_RR,   0,   1500,     30,     30,   95,   95,  180,  180 }  // 3: RR
};

struct EepromCalBlob {
  uint16_t magic;                // 0xCA62
  uint8_t  version;              // 2
  uint8_t  stopDetach;           // 1 = PCA9685 FULL_OFF on stop, 0 = send centerUs
  uint32_t oscHz;                // Calibrated PCA9685 internal oscillator frequency (Hz)
  WheelCal wheels[NUM_WHEELS];   // FL, RL, FR, RR
  uint16_t crc;                  // CRC-16 over preceding bytes
};

constexpr uint16_t CAL_EEPROM_MAGIC = 0xCA62U;
constexpr uint8_t  CAL_EEPROM_VER   = 2U;
constexpr int      CAL_EEPROM_ADDR  = 0;

static uint16_t calcCrc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xA001U) : (crc >> 1);
    }
  }
  return crc;
}

static bool isWheelCalValid(const WheelCal &w) {
  if (w.channel > 15 || w.inverted > 1) return false;
  if (w.centerUs < 1300 || w.centerUs > 1700) return false;
  if (w.fwdStartOff < 5 || w.fwdStartOff > w.fwdMidOff || w.fwdMidOff > w.fwdMaxOff || w.fwdMaxOff > 450) return false;
  if (w.revStartOff < 5 || w.revStartOff > w.revMidOff || w.revMidOff > w.revMaxOff || w.revMaxOff > 450) return false;
  return true;
}

// ── PCINT2 Zero-Blocking Ultrasonic Echo Capture ─────────────────────────────
enum EchoState : uint8_t { US_IDLE = 0, US_RISE, US_FALL, US_DONE };
struct EchoISR { volatile uint8_t state; volatile uint32_t riseUs; volatile uint16_t pulseUs; };
static EchoISR g_ec1 = { US_IDLE, 0, 0 }, g_ec2 = { US_IDLE, 0, 0 };

ISR(PCINT2_vect) {
  const uint32_t now = micros();
  const uint8_t  pd  = PIND;
  if (g_ec1.state == US_RISE && (pd & _BV(PD3)))        { g_ec1.riseUs = now; g_ec1.state = US_FALL; }
  else if (g_ec1.state == US_FALL && !(pd & _BV(PD3)))  { g_ec1.pulseUs = (uint16_t)(now - g_ec1.riseUs); g_ec1.state = US_DONE; }
  if (g_ec2.state == US_RISE && (pd & _BV(PD5)))        { g_ec2.riseUs = now; g_ec2.state = US_FALL; }
  else if (g_ec2.state == US_FALL && !(pd & _BV(PD5)))  { g_ec2.pulseUs = (uint16_t)(now - g_ec2.riseUs); g_ec2.state = US_DONE; }
}

// ── Class 1: Ultrasonic Sensor (Integer mm + 5-Sample Median FIFO) ───────────
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
    pinMode(_trig, OUTPUT); pinMode(_echo, INPUT); digitalWrite(_trig, LOW);
    noInterrupts();
    _isr->state = US_IDLE;
    PCICR |= _BV(PCIE2); PCMSK2 |= _BV(PCINT19) | _BV(PCINT21);
    interrupts();
  }

  bool busy() const { return _busy; }

  void trigger() {
    if (digitalRead(_echo) == HIGH) return;
    noInterrupts(); _isr->state = US_RISE; interrupts();
    digitalWrite(_trig, HIGH); delayMicroseconds(10); digitalWrite(_trig, LOW);
    _trigUs = micros(); _busy = true;
  }

  bool poll() {
    if (!_busy) return false;
    uint8_t st = _isr->state;
    if (st == US_DONE) {
      noInterrupts(); uint16_t p = _isr->pulseUs; _isr->state = US_IDLE; interrupts();
      _busy = false;
      // Fixed-point speed of sound: pulse_us * 0.171875 mm/us = (pulse_us * 11) >> 6
      _pushMm((p == 0 || p > US_TIMEOUT_US) ? 4000U : (uint16_t)(((uint32_t)p * 11UL) >> 6));
      return true;
    }
    if ((uint32_t)(micros() - _trigUs) >= US_TIMEOUT_US) {
      noInterrupts(); _isr->state = US_IDLE; interrupts();
      _busy = false; _pushMm(4000U);
      return true;
    }
    return false;
  }

  uint16_t medianMm() const {
    uint16_t c[5] = { _fifo[0], _fifo[1], _fifo[2], _fifo[3], _fifo[4] };
    for (uint8_t i = 1; i < 5; i++) {
      uint16_t k = c[i]; int8_t j = i - 1;
      while (j >= 0 && c[j] > k) { c[j + 1] = c[j]; j--; }
      c[j + 1] = k;
    }
    return c[2];
  }
};

// ── Class 2: GY-511 LSM303DLHC Digital Compass (Absolute Heading in 0.1 deg) ──
class ImuSensor {
  static constexpr uint8_t MAG_ADDR = 0x1E;
  bool     _ok = false;
  uint8_t  _fails = 0;
  int16_t  _yawDdeg = 0;         // 0..3599 (0.1 deg absolute compass heading)
  int16_t  _lastCompassYaw = 0;
  int32_t  _turnDdeg = 0;        // Continuous accumulated relative turn in 0.1 deg
  uint32_t _lastMs = 0;

  static bool _write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission(true) != 0) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
      return false;
    }
    return true;
  }

  static bool _readMag(int16_t &x, int16_t &y) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x03); // OUT_X_H_M
    // Always send STOP (true) so a NACK never leaves the shared I2C bus locked before PCA9685 writes
    if (Wire.endTransmission(true) != 0) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
      return false;
    }
    if (Wire.requestFrom(MAG_ADDR, (uint8_t)6) != 6) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
      return false;
    }
    // LSM303DLHC register sequence: X_H, X_L, Z_H, Z_L, Y_H, Y_L
    uint8_t xh = (uint8_t)Wire.read();
    uint8_t xl = (uint8_t)Wire.read();
    (void)Wire.read(); // Skip Z_H
    (void)Wire.read(); // Skip Z_L
    uint8_t yh = (uint8_t)Wire.read();
    uint8_t yl = (uint8_t)Wire.read();

    int16_t rx = (int16_t)(((uint16_t)xh << 8) | xl);
    int16_t ry = (int16_t)(((uint16_t)yh << 8) | yl);

    // Reject ADC overflow (-4096 / 0xF000 per LSM303DLHC datasheet Sec 7.2.4) or null vector
    if (rx == -4096 || ry == -4096 || (rx == 0 && ry == 0)) return false;

    x = rx;
    y = ry;
    return true;
  }

  static int16_t _calcHeading(int16_t x, int16_t y) {
    float rad = atan2((float)y, (float)x);
    int16_t ddeg = (int16_t)(rad * (1800.0f / 3.14159265f));
    if (ddeg < 0) ddeg += 3600;
    return ddeg;
  }

public:
  void init() {
    _ok = false;
    _fails = 0;
    _turnDdeg = 0;

    // CRA_REG_M (0x00): 75 Hz data rate (0x18)
    if (!_write(0x00, 0x18)) return;
    // CRB_REG_M (0x01): +/- 1.9 Gauss gain (0x40) to avoid saturation near servo magnets
    if (!_write(0x01, 0x40)) return;
    // MR_REG_M (0x02): Continuous-conversion mode (0x00)
    if (!_write(0x02, 0x00)) return;
    delay(15);

    int16_t x, y;
    if (_readMag(x, y)) {
      _ok = true;
      int16_t h = _calcHeading(x, y);
      _yawDdeg = h;
      _lastCompassYaw = h;
      _turnDdeg = 0;
    }
    _lastMs = millis();
  }

  void update() {
    uint32_t now = millis();
    if (!_ok || (now - _lastMs < 13)) return; // 75 Hz ODR (~13.3 ms); do not touch I2C if sensor unavailable
    _lastMs = now;

    int16_t x, y;
    if (!_readMag(x, y)) {
      if (++_fails >= 10) _ok = false;
      return;
    }
    _fails = 0;

    int16_t curYaw = _calcHeading(x, y);
    int16_t diff = curYaw - _lastCompassYaw;
    if (diff > 1800)       diff -= 3600;
    else if (diff < -1800) diff += 3600;

    _lastCompassYaw = curYaw;
    _yawDdeg = curYaw;
    _turnDdeg += diff;
  }

  void resetTurn() {
    _turnDdeg = 0;
    int16_t x, y;
    if (_ok && _readMag(x, y)) {
      _yawDdeg = _calcHeading(x, y);
      _lastMs = millis();
    }
    _lastCompassYaw = _yawDdeg;
  }

  bool ok() const { return _ok; }
  int32_t yawDdeg() const { return _yawDdeg; }
  int16_t turnedAbsDdeg() const { return (int16_t)(_turnDdeg < 0 ? -_turnDdeg : _turnDdeg); }
};

// ── Class 3: Direct I2C PCA9685 Driver + 4-Wheel Equalized Actuators ─────────
class Actuators {
  static constexpr uint8_t REG_MODE1      = 0x00;
  static constexpr uint8_t REG_MODE2      = 0x01;
  static constexpr uint8_t REG_LED0_ON_L  = 0x06;
  static constexpr uint8_t REG_ALL_ON_L   = 0xFA;
  static constexpr uint8_t REG_PRESCALE   = 0xFE;

  WheelCal _w[NUM_WHEELS];
  uint32_t _oscHz   = PCA_OSC_HZ;
  uint8_t  _prescale = 121;
  int16_t  _lastUs[NUM_WHEELS] = { 0, 0, 0, 0 };
  bool     _active[NUM_WHEELS] = { false, false, false, false };
  bool     _stopDetach = STOP_CON_DETACH;
  bool     _pcaOk      = false;
  bool     _eepromLoaded = false;
  int8_t   _lPct = 0, _rPct = 0;
  uint8_t  _curB = ARM_B_HOME, _curE = ARM_E_HOME, _tgtB = ARM_B_HOME, _tgtE = ARM_E_HOME;
  uint32_t _armMs = 0;

  static bool _writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
      return false;
    }
    return true;
  }

  static void _clearAllOutputs() {
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(REG_ALL_ON_L);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x10); // ALL_LED_OFF_H = FULL_OFF
    if (Wire.endTransmission() != 0) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
    }
  }

  static uint8_t _calcPrescale(uint32_t oscHz) {
    uint32_t denom = 4096UL * (uint32_t)SERVO_HZ;
    uint32_t p = (oscHz + (denom >> 1)) / denom;
    if (p < 4UL)   p = 4UL;
    if (p > 255UL) p = 255UL;
    return (uint8_t)(p - 1UL);
  }

  // Convert microseconds to 12-bit PCA9685 ticks using exact calibrated _oscHz and (_prescale + 1)
  uint16_t _usToTicks(int16_t us) const {
    int16_t  c      = constrain(us, 500, 2500);
    uint32_t osc100 = (_oscHz + 50UL) / 100UL;
    uint32_t den    = 10000UL * (uint32_t)(_prescale + 1);
    uint32_t num    = (uint32_t)c * osc100 + (den >> 1);
    uint16_t ticks  = (uint16_t)(num / den);
    if (ticks > 4095U) ticks = 4095U;
    return ticks;
  }

  void _pcaSetUs(uint8_t ch, int16_t us) const {
    uint16_t ticks = _usToTicks(us);
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write((uint8_t)(REG_LED0_ON_L + 4U * ch));
    Wire.write((uint8_t)0x00);                  // ON_L = 0
    Wire.write((uint8_t)0x00);                  // ON_H = 0
    Wire.write((uint8_t)(ticks & 0xFFU));       // OFF_L
    Wire.write((uint8_t)((ticks >> 8) & 0x0FU));// OFF_H (FULL_OFF = 0)
    if (Wire.endTransmission() != 0) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
    }
  }

  // Physically cuts PWM pulses on channel `ch` (constant 0V output -> zero servo creep)
  static void _pcaSetFullOff(uint8_t ch) {
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write((uint8_t)(REG_LED0_ON_L + 4U * ch));
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x10); // Bit 4 = FULL_OFF
    if (Wire.endTransmission() != 0) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
    }
  }

  static int16_t _degToUs(uint8_t deg) {
    return (int16_t)(ARM_MIN_US + (((int32_t)(ARM_MAX_US - ARM_MIN_US) * deg + 90L) / 180L));
  }

  // 3-point piecewise-linear interpolation per direction:
  //   |pct| == ZONA_MUERTA (5%) -> startOff
  //   |pct| == 50%              -> midOff   (exact 50% speed equalization across all 4 servos)
  //   |pct| == 100%             -> maxOff   (exact 100% speed equalization across all 4 servos)
  static int16_t _pctToUs(const WheelCal &w, int8_t pct) {
    int16_t a = pct < 0 ? -pct : pct;
    bool    isFwd = (pct >= 0);
    int16_t startOff = isFwd ? w.fwdStartOff : w.revStartOff;
    int16_t midOff   = isFwd ? w.fwdMidOff   : w.revMidOff;
    int16_t maxOff   = isFwd ? w.fwdMaxOff   : w.revMaxOff;
    int16_t off;
    if (a <= 50) {
      off = startOff + (int16_t)(((int32_t)(midOff - startOff) * (a - ZONA_MUERTA)) / (50 - ZONA_MUERTA));
    } else {
      off = midOff + (int16_t)(((int32_t)(maxOff - midOff) * (a - 50)) / 50);
    }
    if ((pct < 0) != (w.inverted != 0)) off = -off;
    return constrain((int16_t)(w.centerUs + off), PWM_MIN_US, PWM_MAX_US);
  }

  void _outWheel(uint8_t idx, int16_t us) {
    if (!_active[idx] || us != _lastUs[idx]) {
      _pcaSetUs(_w[idx].channel, us);
      _lastUs[idx] = us;
      _active[idx] = true;
    }
  }

  void _stopWheel(uint8_t idx) {
    if (_stopDetach) {
      if (_active[idx] || _lastUs[idx] != 0) {
        _pcaSetFullOff(_w[idx].channel);
        _active[idx] = false;
        _lastUs[idx] = 0;
      }
    } else {
      _outWheel(idx, _w[idx].centerUs);
    }
  }

  void _setWheelPct(uint8_t idx, int8_t pct) {
    if (pct > -ZONA_MUERTA && pct < ZONA_MUERTA) _stopWheel(idx);
    else _outWheel(idx, _pctToUs(_w[idx], pct));
  }

public:
  void loadDefaults() {
    for (uint8_t i = 0; i < NUM_WHEELS; i++) _w[i] = DEFAULT_WHEEL_CAL[i];
    _stopDetach = STOP_CON_DETACH;
    _oscHz      = PCA_OSC_HZ;
    _prescale   = _calcPrescale(_oscHz);
    _eepromLoaded = false;
  }

  bool loadFromEeprom() {
    EepromCalBlob blob;
    EEPROM.get(CAL_EEPROM_ADDR, blob);
    if (blob.magic != CAL_EEPROM_MAGIC || blob.version != CAL_EEPROM_VER) return false;
    uint16_t expectedCrc = calcCrc16((const uint8_t*)&blob, sizeof(EepromCalBlob) - sizeof(uint16_t));
    if (blob.crc != expectedCrc) return false;
    if (blob.oscHz < PCA_OSC_HZ_MIN || blob.oscHz > PCA_OSC_HZ_MAX) return false;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      if (!isWheelCalValid(blob.wheels[i])) return false;
    }
    for (uint8_t i = 0; i < NUM_WHEELS; i++) _w[i] = blob.wheels[i];
    _stopDetach   = (blob.stopDetach != 0);
    _oscHz        = blob.oscHz;
    _prescale     = _calcPrescale(_oscHz);
    _eepromLoaded = true;
    return true;
  }

  void saveToEeprom() const {
    EepromCalBlob blob;
    blob.magic      = CAL_EEPROM_MAGIC;
    blob.version    = CAL_EEPROM_VER;
    blob.stopDetach = _stopDetach ? 1U : 0U;
    blob.oscHz      = _oscHz;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) blob.wheels[i] = _w[i];
    blob.crc = calcCrc16((const uint8_t*)&blob, sizeof(EepromCalBlob) - sizeof(uint16_t));
    EEPROM.put(CAL_EEPROM_ADDR, blob);
  }

  void invalidateEeprom() {
    uint16_t zero = 0;
    EEPROM.put(CAL_EEPROM_ADDR, zero);
    loadDefaults();
  }

  void _applyHardwareConfig() {
    _prescale = _calcPrescale(_oscHz);

    // 1. Put PCA9685 in SLEEP with Auto-Increment enabled (AI=1, SLEEP=1) FIRST
    _pcaOk = _writeReg(REG_MODE1, 0x30);

    // 2. Force all 16 PCA9685 channels to FULL_OFF before waking oscillator
    _clearAllOutputs();
    delay(10); // Wait for internal oscillator to stop before writing PRESCALE (datasheet sec 7.3.5)

    // 3. Configure 50 Hz prescaler while in SLEEP mode, then wake with Auto-Increment
    _writeReg(REG_PRESCALE, _prescale);
    _writeReg(REG_MODE2, 0x04); // OUTDRV=1 (totem-pole)
    _writeReg(REG_MODE1, 0x20); // AI=1, SLEEP=0
    delayMicroseconds(1000);
    _writeReg(REG_MODE1, 0xA0); // RESTART=1, AI=1

    // 4. Re-latch ALL_LED_OFF_H = 0x10 while oscillator is running, then apply state
    _clearAllOutputs();
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      _active[i] = true;
      _lastUs[i] = -1;
      _stopWheel(i);
    }
    _pcaSetUs(CH_ARM_B, _degToUs(_curB));
    _pcaSetUs(CH_ARM_E, _degToUs(_curE));
  }

  void init() {
    Wire.begin();
    Wire.setClock(400000L);
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(2500, true);
    #endif

    if (!loadFromEeprom()) loadDefaults();
    _applyHardwareConfig();
  }

  void setOscHz(uint32_t hz) {
    if (hz < PCA_OSC_HZ_MIN || hz > PCA_OSC_HZ_MAX) return;
    _oscHz = hz;
    _applyHardwareConfig();
    setWheels(_lPct, _rPct);
  }

  uint32_t oscHz() const { return _oscHz; }
  uint8_t prescale() const { return _prescale; }

  void setWheels(int16_t l, int16_t r) {
    int8_t cl = (int8_t)constrain(l, -100, 100);
    int8_t cr = (int8_t)constrain(r, -100, 100);
    _lPct = (cl > -ZONA_MUERTA && cl < ZONA_MUERTA) ? 0 : cl;
    _rPct = (cr > -ZONA_MUERTA && cr < ZONA_MUERTA) ? 0 : cr;
    _setWheelPct(W_FL, _lPct);
    _setWheelPct(W_RL, _lPct);
    _setWheelPct(W_FR, _rPct);
    _setWheelPct(W_RR, _rPct);
  }

  // ── Calibration helpers (Serial Monitor) ──
  void rawWheelUs(uint8_t idx, int16_t us) {
    if (idx >= NUM_WHEELS) return;
    _outWheel(idx, constrain(us, PWM_MIN_US, PWM_MAX_US));
    _lPct = 0; _rPct = 0;
  }

  void rawSideUs(bool left, int16_t us) {
    int16_t c = constrain(us, PWM_MIN_US, PWM_MAX_US);
    if (left) { _outWheel(W_FL, c); _outWheel(W_RL, c); }
    else      { _outWheel(W_FR, c); _outWheel(W_RR, c); }
    _lPct = 0; _rPct = 0;
  }

  void setWheelCenter(uint8_t idx, int16_t cUs) {
    if (idx >= NUM_WHEELS) return;
    _w[idx].centerUs = cUs;
    _lastUs[idx] = -1;
    setWheels(_lPct, _rPct);
  }

  void setCenters4(int16_t cFL, int16_t cRL, int16_t cFR, int16_t cRR) {
    _w[W_FL].centerUs = cFL; _w[W_RL].centerUs = cRL;
    _w[W_FR].centerUs = cFR; _w[W_RR].centerUs = cRR;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) _lastUs[i] = -1;
    setWheels(_lPct, _rPct);
  }

  void setMidOffsets4(int16_t mFL, int16_t mRL, int16_t mFR, int16_t mRR) {
    int16_t m[NUM_WHEELS] = { mFL, mRL, mFR, mRR };
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      _w[i].fwdMidOff = m[i];
      _w[i].revMidOff = m[i];
      if (_w[i].fwdStartOff > m[i]) _w[i].fwdStartOff = m[i];
      if (_w[i].revStartOff > m[i]) _w[i].revStartOff = m[i];
      if (_w[i].fwdMaxOff < m[i])   _w[i].fwdMaxOff   = m[i];
      if (_w[i].revMaxOff < m[i])   _w[i].revMaxOff   = m[i];
      _lastUs[i] = -1;
    }
    setWheels(_lPct, _rPct);
  }

  void setWheelMidAsym(uint8_t idx, int16_t fMid, int16_t rMid) {
    if (idx >= NUM_WHEELS) return;
    _w[idx].fwdMidOff = fMid;
    _w[idx].revMidOff = rMid;
    if (_w[idx].fwdStartOff > fMid) _w[idx].fwdStartOff = fMid;
    if (_w[idx].revStartOff > rMid) _w[idx].revStartOff = rMid;
    if (_w[idx].fwdMaxOff < fMid)   _w[idx].fwdMaxOff   = fMid;
    if (_w[idx].revMaxOff < rMid)   _w[idx].revMaxOff   = rMid;
    _lastUs[idx] = -1;
    setWheels(_lPct, _rPct);
  }

  void setStopDetach(bool d) {
    _stopDetach = d;
    for (uint8_t i = 0; i < NUM_WHEELS; i++) _lastUs[i] = -1;
    setWheels(_lPct, _rPct);
  }

  const WheelCal& wheel(uint8_t idx) const { return _w[idx < NUM_WHEELS ? idx : 0]; }
  bool stopDetach() const { return _stopDetach; }
  bool pcaOk() const { return _pcaOk; }
  bool eepromLoaded() const { return _eepromLoaded; }

  void setArm(int16_t b, int16_t e) {
    _tgtB = (uint8_t)constrain(b, 0, 180);
    _tgtE = (uint8_t)constrain(e, 0, 180);
  }

  void stopAll() { setWheels(0, 0); _tgtB = _curB; _tgtE = _curE; }

  void updateArm(uint32_t nowMs) {
    if ((_curB == _tgtB && _curE == _tgtE) || (nowMs - _armMs < 15)) return;
    _armMs = nowMs;
    if (_curB != _tgtB) {
      _curB += (_tgtB > _curB) ? min(2, _tgtB - _curB) : -min(2, _curB - _tgtB);
      _pcaSetUs(CH_ARM_B, _degToUs(_curB));
    }
    if (_curE != _tgtE) {
      _curE += (_tgtE > _curE) ? min(2, _tgtE - _curE) : -min(2, _curE - _tgtE);
      _pcaSetUs(CH_ARM_E, _degToUs(_curE));
    }
  }

  int8_t left() const { return _lPct; }
  int8_t right() const { return _rPct; }
  uint8_t armB() const { return _curB; }
  uint8_t armE() const { return _curE; }
  bool movingFwd() const { return _lPct > 0 || _rPct > 0; }
};

// ── Global Hardware Instances & Compact State Machine ────────────────────────
enum Mode : uint8_t { M_IDLE = 0, M_DRIVE, M_GYRO, M_REV, M_CAL };
static UltrasonicSensor us1(PIN_TRIG1, PIN_ECHO1, &g_ec1), us2(PIN_TRIG2, PIN_ECHO2, &g_ec2);
static ImuSensor        imu;
static Actuators        act;

static Mode     mode = M_IDLE;
static uint32_t tImu = 0, tUs = 0, tState = 0, tFallback = 0;
static int16_t  tgtTurnDdeg = 0;
static bool     turnRight = true, toggleUs = false, obsLatched = false, rxOvf = false;
static char     rxBuf[48];
static uint8_t  rxIdx = 0;

static uint16_t minUsMm() { uint16_t a = us1.medianMm(), b = us2.medianMm(); return a < b ? a : b; }

static void startAutoRev(uint16_t mm, bool blockedCmd) {
  obsLatched = true; act.setWheels(-50, -50); mode = M_REV; tState = millis();
  Serial.print(blockedCmd ? F("[ACK] BLK|") : F("[EVENT] OBS|")); Serial.println(mm);
}

static uint8_t parseInts(const char* s, int16_t* out, uint8_t maxCount) {
  uint8_t count = 0;
  const char* p = s;
  while (*p != '\0') {
    while (*p == ' ') p++;
    if (*p == '\0') break;
    if (count >= maxCount) return 0;
    char* endP;
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
  a = tmp[0]; b = tmp[1]; return true;
}

static void printStatusTelemetry() {
  Serial.print(F("[STATUS] "));
  Serial.print(act.left());    Serial.print(',');
  Serial.print(act.right());   Serial.print(',');
  Serial.print(act.armB());    Serial.print(',');
  Serial.print(act.armE());    Serial.print(',');
  Serial.print(imu.yawDdeg()); Serial.print(',');
  Serial.print(imu.ok() ? 1 : 0); Serial.print(',');
  Serial.print(us1.medianMm()); Serial.print(',');
  Serial.print(us2.medianMm()); Serial.print(',');
  Serial.println((uint8_t)mode);
}

static void handleOpcode(const char* cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') return;

  // ── Spanish Direct Commands (UART / Monitor Serie) ────────────────────────
  if (strncasecmp_P(cmd, PSTR("ADELANTE"), 8) == 0 && (cmd[8] == ' ' || cmd[8] == '\0')) {
    const char* p = cmd + 8;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] ADELANTE <0-100>")); return; }
      spd = (int16_t)v;
    }
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

  if (strncasecmp_P(cmd, PSTR("ATRAS"), 5) == 0 && (cmd[5] == ' ' || cmd[5] == '\0')) {
    const char* p = cmd + 5;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] ATRAS <0-100>")); return; }
      spd = (int16_t)v;
    }
    act.setWheels(-spd, -spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|-")); Serial.print(spd); Serial.print(F("|-")); Serial.println(spd);
    return;
  }

  if (strncasecmp_P(cmd, PSTR("GIRARD"), 6) == 0 && (cmd[6] == ' ' || cmd[6] == '\0')) {
    const char* p = cmd + 6;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] GIRARD <1-100>")); return; }
      spd = (int16_t)v;
    }
    act.setWheels(spd, -spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|")); Serial.print(spd); Serial.print(F("|-")); Serial.println(spd);
    return;
  }

  if (strncasecmp_P(cmd, PSTR("GIRARI"), 6) == 0 && (cmd[6] == ' ' || cmd[6] == '\0')) {
    const char* p = cmd + 6;
    while (*p == ' ') p++;
    int16_t spd = 50;
    if (*p != '\0') {
      char* endP;
      long v = strtol(p, &endP, 10);
      if (endP == p || v < 0 || v > 100) { Serial.println(F("[ERROR] GIRARI <1-100>")); return; }
      spd = (int16_t)v;
    }
    act.setWheels(-spd, spd);
    mode = (act.left() == 0 && act.right() == 0) ? M_IDLE : M_DRIVE;
    Serial.print(F("[ACK] W|-")); Serial.print(spd); Serial.print('|'); Serial.println(spd);
    return;
  }

  if (strcasecmp_P(cmd, PSTR("BRAZOEXT")) == 0) {
    act.setArm(90, 145);
    Serial.println(F("[ACK] A|90|145"));
    return;
  }

  if (strcasecmp_P(cmd, PSTR("BRAZOH")) == 0) {
    act.setArm(ARM_B_HOME, ARM_E_HOME);
    Serial.print(F("[ACK] A|")); Serial.print(ARM_B_HOME); Serial.print('|'); Serial.println(ARM_E_HOME);
    return;
  }

  if (strncasecmp_P(cmd, PSTR("BRAZO "), 6) == 0) {
    int16_t b, e;
    if (parseTwoInts(cmd + 6, b, e) && b >= 0 && b <= 180 && e >= 0 && e <= 180) {
      act.setArm(b, e);
      Serial.print(F("[ACK] A|")); Serial.print(b); Serial.print('|'); Serial.println(e);
    } else Serial.println(F("[ERROR] BRAZO <base> <codo>"));
    return;
  }

  if (strcasecmp_P(cmd, PSTR("ALTO")) == 0) {
    act.stopAll(); mode = M_IDLE;
    Serial.println(F("[ACK] X"));
    return;
  }

  if (strcasecmp_P(cmd, PSTR("ESTADO")) == 0) {
    printStatusTelemetry();
    return;
  }

  if (strncasecmp_P(cmd, PSTR("GIROD"), 5) == 0 || strncasecmp_P(cmd, PSTR("GIROI"), 5) == 0) {
    bool isR = (cmd[4] == 'D' || cmd[4] == 'd');
    const char* p = cmd + 5;
    while (*p == ' ') p++;
    int16_t deg = 0;
    if (parseInts(p, &deg, 1) == 1 && deg > 0 && deg <= 360) {
      turnRight = isR;
      tgtTurnDdeg = deg * 10;
      tFallback = constrain((uint32_t)tgtTurnDdeg * 100UL / 90UL, 100UL, 6000UL);
      tState = millis(); imu.resetTurn();
      act.setWheels(turnRight ? TURN_MAX_SPD : -TURN_MAX_SPD, turnRight ? -TURN_MAX_SPD : TURN_MAX_SPD);
      mode = M_GYRO;
      Serial.print(F("[ACK] G|")); Serial.print(turnRight ? 'R' : 'L'); Serial.print('|'); Serial.println(tgtTurnDdeg);
      return;
    } else {
      Serial.println(F("[ERROR] GIROD/GIROI <1-360>"));
      return;
    }
  }

  char op = cmd[0];
  if (op >= 'a' && op <= 'z') op -= 32;
  const char* arg = cmd + 1;

  switch (op) {
    case 'X': {
      while (*arg == ' ') arg++;
      if (*arg != '\0') { Serial.println(F("[ERROR] OP")); break; }
      act.stopAll(); mode = M_IDLE;
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
        if (l >= ZONA_MUERTA || r >= ZONA_MUERTA) {
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
        turnRight = (ddeg > 0);
        tgtTurnDdeg = (int16_t)abs(ddeg);
        tFallback = constrain((uint32_t)tgtTurnDdeg * 100UL / 90UL, 100UL, 6000UL);
        tState = millis(); imu.resetTurn();
        act.setWheels(turnRight ? TURN_MAX_SPD : -TURN_MAX_SPD, turnRight ? -TURN_MAX_SPD : TURN_MAX_SPD);
        mode = M_GYRO;
        Serial.print(F("[ACK] G|")); Serial.print(turnRight ? 'R' : 'L'); Serial.print('|'); Serial.println(tgtTurnDdeg);
      } else Serial.println(F("[ERROR] G <ddeg>"));
      break;
    }
    // ── Calibration commands (Serial Monitor only; never sent by control.py) ──
    case 'P': { // P <0-3> <1000-2000> : Independent raw pulse on wheel 0..3
      int16_t v[2];
      if (parseInts(arg, v, 2) == 2 && v[0] >= 0 && v[0] < NUM_WHEELS && v[1] >= 1000 && v[1] <= 2000) {
        act.rawWheelUs((uint8_t)v[0], v[1]); mode = M_CAL;
        Serial.print(F("[ACK] P|")); Serial.print(v[0]); Serial.print('|'); Serial.println(v[1]);
      } else Serial.println(F("[ERROR] P <0-3> <1000-2000>"));
      break;
    }
    case 'L': case 'R': { // L <us> / R <us> : Raw pulse on left pair or right pair
      int16_t us;
      if (parseInts(arg, &us, 1) == 1 && us >= 1000 && us <= 2000) {
        act.rawSideUs(op == 'L', us); mode = M_CAL;
        Serial.print(F("[ACK] ")); Serial.print(op); Serial.print('|'); Serial.println(us);
      } else Serial.println(F("[ERROR] L|R <1000-2000>"));
      break;
    }
    case 'C': { // C <fl> <rl> <fr> <rr> OR C <0-3> <us> OR C <l> <r>
      int16_t v[4];
      uint8_t n = parseInts(arg, v, 4);
      if (n == 4 && v[0] >= 1300 && v[0] <= 1700 && v[1] >= 1300 && v[1] <= 1700 &&
          v[2] >= 1300 && v[2] <= 1700 && v[3] >= 1300 && v[3] <= 1700) {
        act.setCenters4(v[0], v[1], v[2], v[3]);
        Serial.print(F("[ACK] C|"));
        Serial.print(v[0]); Serial.print('|'); Serial.print(v[1]); Serial.print('|');
        Serial.print(v[2]); Serial.print('|'); Serial.println(v[3]);
      } else if (n == 2 && v[0] >= 0 && v[0] < NUM_WHEELS && v[1] >= 1300 && v[1] <= 1700) {
        act.setWheelCenter((uint8_t)v[0], v[1]);
        Serial.print(F("[ACK] C|")); Serial.print(v[0]); Serial.print('|'); Serial.println(v[1]);
      } else if (n == 2 && v[0] >= 1300 && v[0] <= 1700 && v[1] >= 1300 && v[1] <= 1700) {
        act.setCenters4(v[0], v[0], v[1], v[1]);
        Serial.print(F("[ACK] C|")); Serial.print(v[0]); Serial.print('|'); Serial.println(v[1]);
      } else Serial.println(F("[ERROR] C <fl> <rl> <fr> <rr> | C <0-3> <us>"));
      break;
    }
    case 'M': { // M <fl> <rl> <fr> <rr> OR M <idx 0-3> <fwdMid> <revMid>
      int16_t v[4];
      uint8_t n = parseInts(arg, v, 4);
      if (n == 4 && v[0] >= 20 && v[0] <= 400 && v[1] >= 20 && v[1] <= 400 &&
          v[2] >= 20 && v[2] <= 400 && v[3] >= 20 && v[3] <= 400) {
        act.setMidOffsets4(v[0], v[1], v[2], v[3]);
        Serial.print(F("[ACK] M|"));
        Serial.print(v[0]); Serial.print('|'); Serial.print(v[1]); Serial.print('|');
        Serial.print(v[2]); Serial.print('|'); Serial.println(v[3]);
      } else if (n == 3 && v[0] >= 0 && v[0] < NUM_WHEELS && v[1] >= 20 && v[1] <= 400 && v[2] >= 20 && v[2] <= 400) {
        act.setWheelMidAsym((uint8_t)v[0], v[1], v[2]);
        Serial.print(F("[ACK] M|")); Serial.print(v[0]); Serial.print('|');
        Serial.print(v[1]); Serial.print('|'); Serial.println(v[2]);
      } else Serial.println(F("[ERROR] M <fl> <rl> <fr> <rr> | M <0-3> <fwd> <rev>"));
      break;
    }
    case 'O': { // O [hz] : Query or set PCA9685 real oscillator frequency in Hz
      while (*arg == ' ') arg++;
      if (*arg == '\0') {
        Serial.print(F("[ACK] O|")); Serial.print(act.oscHz());
        Serial.print(F("|pre="));    Serial.println(act.prescale());
      } else {
        char* endP;
        unsigned long hz = strtoul(arg, &endP, 10);
        if (endP != arg && hz >= PCA_OSC_HZ_MIN && hz <= PCA_OSC_HZ_MAX) {
          act.setOscHz((uint32_t)hz);
          Serial.print(F("[ACK] O|")); Serial.print(act.oscHz());
          Serial.print(F("|pre="));    Serial.println(act.prescale());
        } else Serial.println(F("[ERROR] O <20000000-30000000>"));
      }
      break;
    }
    case 'D': {
      while (*arg == ' ') arg++;
      if ((arg[0] == '0' || arg[0] == '1') && arg[1] == '\0') {
        act.setStopDetach(arg[0] == '1');
        Serial.print(F("[ACK] D|")); Serial.println(arg[0]);
      } else Serial.println(F("[ERROR] D <0|1>"));
      break;
    }
    case 'E': { // E = save calibration to EEPROM; E 0 = restore defaults
      while (*arg == ' ') arg++;
      if (*arg == '\0') {
        act.saveToEeprom();
        Serial.println(F("[ACK] E|SAVED"));
      } else if (arg[0] == '0' && arg[1] == '\0') {
        act.invalidateEeprom();
        Serial.println(F("[ACK] E|DEFAULT"));
      } else Serial.println(F("[ERROR] E [0]"));
      break;
    }
    case 'K': {
      Serial.print(F("[CAL] pca=")); Serial.print(act.pcaOk() ? 1 : 0);
      Serial.print(F(",eep="));      Serial.print(act.eepromLoaded() ? 1 : 0);
      Serial.print(F(",det="));      Serial.print(act.stopDetach() ? 1 : 0);
      Serial.print(F(",osc="));      Serial.print(act.oscHz());
      Serial.print(F(",pre="));      Serial.print(act.prescale());
      for (uint8_t i = 0; i < NUM_WHEELS; i++) {
        const WheelCal &w = act.wheel(i);
        Serial.print(F(" | W")); Serial.print(i);
        Serial.print(F("(ch=")); Serial.print(w.channel);
        Serial.print(F(",c="));  Serial.print(w.centerUs);
        Serial.print(F(",mF=")); Serial.print(w.fwdMidOff);
        Serial.print(F(",mR=")); Serial.print(w.revMidOff);
        Serial.print(')');
      }
      Serial.println();
      break;
    }
    case 'S': { // "S" or "STOP" typed in the Serial Monitor
      while (*arg == ' ') arg++;
      if (*arg == '\0' || strcasecmp_P(arg, PSTR("TOP")) == 0) {
        act.stopAll(); mode = M_IDLE;
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
  act.init(); us1.init(); us2.init(); imu.init();
  for (uint8_t i = 0; i < 5; i++) {
    us1.trigger(); while (us1.busy()) us1.poll(); delay(4);
    us2.trigger(); while (us2.busy()) us2.poll(); delay(4);
  }
  Serial.print(F("[READY] UNO_ROVER|IMU=")); Serial.print(imu.ok() ? 1 : 0);
  Serial.print(F("|PCA=")); Serial.print(act.pcaOk() ? 1 : 0);
  Serial.print(F("|EEP=")); Serial.println(act.eepromLoaded() ? 1 : 0);
}

void loop() {
  uint32_t now = millis();
  act.updateArm(now);

  if (now - tImu >= 2) { tImu = now; imu.update(); }

  if (mode == M_GYRO) {
    int16_t turned = imu.turnedAbsDdeg();
    bool done = imu.ok() ? (turned >= tgtTurnDdeg) : (now - tState >= tFallback);
    if (done || (now - tState >= 6000UL)) {
      act.setWheels(0, 0); mode = M_IDLE;
      Serial.print(F("[EVENT] TD|")); Serial.print(turnRight ? 'R' : 'L'); Serial.print('|');
      Serial.println(imu.ok() ? turned : tgtTurnDdeg);
    } else if (imu.ok() && (tgtTurnDdeg - turned < 60)) {
      int16_t spd = TURN_MIN_SPD + ((int16_t)(TURN_MAX_SPD - TURN_MIN_SPD) * (tgtTurnDdeg - turned)) / 60;
      act.setWheels(turnRight ? spd : -spd, turnRight ? -spd : spd);
    }
  }

  bool newUs = us1.poll() | us2.poll();
  if (mode != M_GYRO && (now - tUs >= 30) && !us1.busy() && !us2.busy()) {
    tUs = now;
    if (!toggleUs) us1.trigger(); else us2.trigger();
    toggleUs = !toggleUs;
  }

  if (newUs) {
    uint16_t mm = minUsMm();
    if (mode == M_IDLE || (mode == M_DRIVE && act.movingFwd())) {
      if (mm > 10 && mm < OBS_TRIG_MM && !obsLatched) startAutoRev(mm, false);
      else if (mm >= OBS_CLEAR_MM) obsLatched = false;
    } else if (mm >= OBS_CLEAR_MM && mode != M_REV) obsLatched = false;
  }

  if (mode == M_REV) {
    uint32_t el = now - tState; uint16_t mm = minUsMm();
    if ((el >= 350 && mm >= OBS_CLEAR_MM) || el >= 900) {
      act.setWheels(0, 0); mode = M_IDLE;
      if (mm >= OBS_CLEAR_MM) obsLatched = false;
      Serial.print(F("[EVENT] CLR|")); Serial.print(us1.medianMm()); Serial.print('|'); Serial.println(us2.medianMm());
    }
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxOvf) { rxIdx = 0; rxOvf = false; Serial.println(F("[ERROR] LEN")); }
      else if (rxIdx > 0) { rxBuf[rxIdx] = '\0'; handleOpcode(rxBuf); rxIdx = 0; }
    } else if (rxIdx < sizeof(rxBuf) - 1) rxBuf[rxIdx++] = c;
    else rxOvf = true;
  }
}
