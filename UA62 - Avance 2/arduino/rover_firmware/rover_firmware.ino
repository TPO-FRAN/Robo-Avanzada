/*
  arduino/rover_firmware/rover_firmware.ino
  Lean Real-Time Firmware for Arduino Uno (ATmega328P) — UA62 - Avance 2
  100% Integer Fixed-Point Math (Zero float emulation, zero printf/strtod bloat).

  Pins:
    - Wheels (4x Continuous MG996R, Ø 87.5 mm) : Left = D6, Right = D9 (auto-detach on 0%)
    - Arm    (2x Standard MG996R, 2-DOF)       : Base = D10, Elbow = D11 (smooth ramp)
    - IMU    (MPU-6050 I2C @ 400 kHz)          : SDA = A4, SCL = A5 (500 Hz trapezoidal)
    - US     (2x HC-SR04 via PCINT2, N=5 FIFO) : US1(T=D2, E=D3), US2(T=D4, E=D5)

  Compact UART Protocol (115200 baud, parsed by Raspberry Pi control.py):
    - W <l> <r>   : Differential wheel speed (-100..100 %)
    - G <ddeg>    : Closed-loop gyro turn in 0.1 deg (+ = RIGHT, - = LEFT)
    - A <b> <e>   : 2-DOF arm target angles (0..180 deg)
    - X           : Immediate STOP (wheels detached + arm frozen)
    - ?           : Query STATUS telemetry

  Calibration commands (Arduino Serial Monitor @ 115200, "Newline"; wheels in the air):
    - L <us> / R <us> : Raw PWM on left/right wheel pair (1000..2000), e.g. L 1475
    - C <l> <r>       : Set neutral center in RAM to test W with calibration, e.g. C 1492 1508
    - D <0|1>         : Stop mode: 1 = detach() PWM, 0 = send neutral PWM
    - K               : Print active calibration
    - STOP (or S)     : Stop both wheels (same as X)
*/

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// ── Hardware Constants ───────────────────────────────────────────────────────
constexpr uint8_t  PIN_TRIG1 = 2, PIN_ECHO1 = 3; // D3 = PD3 (PCINT19)
constexpr uint8_t  PIN_TRIG2 = 4, PIN_ECHO2 = 5; // D5 = PD5 (PCINT21)
constexpr uint8_t  PIN_WHEEL_L = 6, PIN_WHEEL_R = 9;
constexpr uint8_t  PIN_ARM_B = 10,  PIN_ARM_E = 11;

constexpr bool     INV_LEFT = false, INV_RIGHT = true;
constexpr uint8_t  ARM_B_HOME = 90,  ARM_E_HOME = 25;
constexpr uint16_t US_TIMEOUT_US = 15000U;
constexpr uint16_t OBS_TRIG_MM   = 200U;  // 20.0 cm
constexpr uint16_t OBS_CLEAR_MM  = 250U;  // 25.0 cm
constexpr uint8_t  TURN_MAX_SPD  = 55, TURN_MIN_SPD = 28;

// ── Wheel Calibration (tune with L/R/C commands in the Serial Monitor) ───────
constexpr int16_t PWM_CENTRO_L   = 1500, PWM_CENTRO_R   = 1500; // Neutral: wheel does NOT spin
constexpr int16_t PWM_MIN_L      = 1000, PWM_MAX_L      = 2000; // Absolute pulse limits
constexpr int16_t PWM_MIN_R      = 1000, PWM_MAX_R      = 2000;
constexpr int16_t PWM_ARRANQUE_L = 30,   PWM_ARRANQUE_R = 30;   // us from neutral where wheel starts spinning
constexpr int16_t PWM_SATURA_L   = 200,  PWM_SATURA_R   = 200;  // us from neutral where wheel reaches max speed
constexpr int8_t  ZONA_MUERTA    = 5;     // |pct| below this is treated as 0
constexpr bool    STOP_CON_DETACH = true; // true: cut PWM on stop; false: send neutral PWM

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

// ── Class 2: MPU-6050 Gyro Z (Fixed-Point Trapezoidal Integration in 0.1 deg)
class ImuSensor {
  // +/-500 dps => 65.5 LSB/dps => 0.1 deg = 13,100,000 units of (2*LSB * us)
  static constexpr int32_t TENTH_DEG_UNIT = 13100000L;
  bool     _ok = false;
  uint8_t  _fails = 0;
  int16_t  _bias = 0, _prevCorr = 0;
  int32_t  _rem = 0, _yawDdeg = 0, _turnDdeg = 0;
  uint32_t _lastUs = 0;

  static bool _write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  static bool _readGz(int16_t &gz) {
    Wire.beginTransmission(0x68); Wire.write(0x47);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)0x68, (uint8_t)2) != 2) {
      #if defined(WIRE_HAS_TIMEOUT)
      Wire.clearWireTimeoutFlag();
      #endif
      return false;
    }
    gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }

public:
  void init() {
    Wire.begin(); Wire.setClock(400000L);
    #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(2500, true);
    #endif
    if (!_write(0x6B, 0x00)) return;
    delay(10);
    _write(0x1A, 0x03); // 44 Hz DLPF
    _write(0x1B, 0x08); // +/- 500 dps
    int32_t sum = 0; uint8_t n = 0;
    for (uint8_t i = 0; i < 128; i++) {
      int16_t raw;
      if (_readGz(raw)) { sum += raw; n++; }
      delayMicroseconds(1200);
    }
    if (n >= 64) { _ok = true; _bias = (int16_t)(sum / n); }
    _lastUs = micros();
  }

  void update() {
    uint32_t now = micros(), dt = now - _lastUs;
    _lastUs = now;
    if (!_ok || dt == 0 || dt > 30000UL) return;
    int16_t raw;
    if (!_readGz(raw)) { if (++_fails >= 10) _ok = false; return; }
    _fails = 0;
    int16_t corr = raw - _bias;
    if (corr > -23 && corr < 23) corr = 0; // 0.35 dps deadzone
    _rem += ((int32_t)_prevCorr + corr) * (int32_t)dt;
    _prevCorr = corr;
    while (_rem >= TENTH_DEG_UNIT)  { _rem -= TENTH_DEG_UNIT; _yawDdeg++; _turnDdeg++; }
    while (_rem <= -TENTH_DEG_UNIT) { _rem += TENTH_DEG_UNIT; _yawDdeg--; _turnDdeg--; }
  }

  void resetTurn() { _turnDdeg = 0; _rem = 0; _prevCorr = 0; _lastUs = micros(); }
  bool ok() const { return _ok; }
  int32_t yawDdeg() const { return _yawDdeg; }
  int16_t turnedAbsDdeg() const { return (int16_t)(_turnDdeg < 0 ? -_turnDdeg : _turnDdeg); }
};

// ── Class 3: Wheel & Arm Actuators (Zero-Creep Detach + Smooth Arm Ramp) ─────
class Actuators {
  // One calibrated wheel channel (each pin drives a parallel pair of MG996R)
  struct Wheel {
    Servo   s;
    uint8_t pin; bool inv, att;
    int16_t center, minUs, maxUs, startOff, satOff, lastUs;
    Wheel(uint8_t p, bool i, int16_t c, int16_t mn, int16_t mx, int16_t so, int16_t sa)
      : pin(p), inv(i), att(false), center(c), minUs(mn), maxUs(mx), startOff(so), satOff(sa), lastUs(0) {}
  };
  Wheel   _L{ PIN_WHEEL_L, INV_LEFT,  PWM_CENTRO_L, PWM_MIN_L, PWM_MAX_L, PWM_ARRANQUE_L, PWM_SATURA_L };
  Wheel   _R{ PIN_WHEEL_R, INV_RIGHT, PWM_CENTRO_R, PWM_MIN_R, PWM_MAX_R, PWM_ARRANQUE_R, PWM_SATURA_R };
  Servo   _aB, _aE;
  bool    _stopDetach = STOP_CON_DETACH;
  int8_t  _lPct = 0, _rPct = 0;
  uint8_t _curB = ARM_B_HOME, _curE = ARM_E_HOME, _tgtB = ARM_B_HOME, _tgtE = ARM_E_HOME;
  uint32_t _armMs = 0;

  // pct -> us: ZONA_MUERTA% = ARRANQUE offset, 100% = SATURA offset (integer, monotonic)
  static int16_t _pctToUs(const Wheel &w, int8_t pct) {
    int16_t a = pct < 0 ? -pct : pct;
    int16_t off = w.startOff + (int16_t)(((int32_t)(w.satOff - w.startOff) * (a - ZONA_MUERTA))
                                         / (100 - ZONA_MUERTA));
    if ((pct < 0) != w.inv) off = -off;
    return constrain((int16_t)(w.center + off), w.minUs, w.maxUs);
  }

  // Physical output: pulse stored BEFORE attach (first pulse is always correct), written only on change
  static void _out(Wheel &w, int16_t us) {
    if (us != w.lastUs || !w.att) { w.s.writeMicroseconds(us); w.lastUs = us; }
    if (!w.att) { w.s.attach(w.pin, w.minUs, w.maxUs); w.att = true; }
  }

  void _stopWheel(Wheel &w) {
    if (_stopDetach) {
      if (w.att) { w.s.detach(); digitalWrite(w.pin, LOW); w.att = false; }
    } else _out(w, w.center);
  }

  void _setWheel(Wheel &w, int8_t pct) {
    if (pct > -ZONA_MUERTA && pct < ZONA_MUERTA) _stopWheel(w);
    else _out(w, _pctToUs(w, pct));
  }

public:
  void init() {
    pinMode(PIN_WHEEL_L, OUTPUT); pinMode(PIN_WHEEL_R, OUTPUT);
    digitalWrite(PIN_WHEEL_L, LOW); digitalWrite(PIN_WHEEL_R, LOW);
    if (!_stopDetach) { _out(_L, _L.center); _out(_R, _R.center); }
    _aB.attach(PIN_ARM_B); _aE.attach(PIN_ARM_E);
    _aB.write(_curB); _aE.write(_curE);
  }

  void setWheels(int16_t l, int16_t r) {
    _lPct = (int8_t)constrain(l, -100, 100);
    _rPct = (int8_t)constrain(r, -100, 100);
    _setWheel(_L, _lPct);
    _setWheel(_R, _rPct);
  }

  // ── Calibration helpers (Serial Monitor) ──
  void rawUs(bool left, int16_t us) {
    Wheel &w = left ? _L : _R;
    _out(w, constrain(us, w.minUs, w.maxUs));
    if (left) _lPct = 0; else _rPct = 0;
  }
  void setCenters(int16_t cl, int16_t cr) { _L.center = cl; _R.center = cr; }
  void setStopDetach(bool d) { _stopDetach = d; }
  int16_t centerL() const { return _L.center; }
  int16_t centerR() const { return _R.center; }
  bool stopDetach() const { return _stopDetach; }

  void setArm(int16_t b, int16_t e) {
    _tgtB = (uint8_t)constrain(b, 0, 180);
    _tgtE = (uint8_t)constrain(e, 0, 180);
  }

  void stopAll() { setWheels(0, 0); _tgtB = _curB; _tgtE = _curE; }

  void updateArm(uint32_t nowMs) {
    if ((_curB == _tgtB && _curE == _tgtE) || (nowMs - _armMs < 15)) return;
    _armMs = nowMs;
    if (_curB != _tgtB) { _curB += (_tgtB > _curB) ? min(2, _tgtB - _curB) : -min(2, _curB - _tgtB); _aB.write(_curB); }
    if (_curE != _tgtE) { _curE += (_tgtE > _curE) ? min(2, _tgtE - _curE) : -min(2, _curE - _tgtE); _aE.write(_curE); }
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
static char     rxBuf[32];
static uint8_t  rxIdx = 0;

static uint16_t minUsMm() { uint16_t a = us1.medianMm(), b = us2.medianMm(); return a < b ? a : b; }

static void startAutoRev(uint16_t mm, bool blockedCmd) {
  obsLatched = true; act.setWheels(-50, -50); mode = M_REV; tState = millis();
  Serial.print(blockedCmd ? F("[ACK] BLK|") : F("[EVENT] OBS|")); Serial.println(mm);
}

static bool parseTwoInts(const char* s, int16_t &a, int16_t &b) {
  char *e1, *e2;
  long v1 = strtol(s, &e1, 10); if (e1 == s) return false;
  long v2 = strtol(e1, &e2, 10); if (e2 == e1) return false;
  while (*e2 == ' ') e2++;
  if (*e2 != '\0') return false;
  a = (int16_t)v1; b = (int16_t)v2; return true;
}

static void handleOpcode(const char* cmd) {
  while (*cmd == ' ') cmd++;
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
        if (l > 0 || r > 0) {
          uint16_t mm = minUsMm();
          if (mm > 10 && mm < OBS_TRIG_MM) { startAutoRev(mm, true); return; }
          obsLatched = false;
        }
        act.setWheels(l, r);
        mode = (l == 0 && r == 0) ? M_IDLE : M_DRIVE;
        Serial.print(F("[ACK] W|")); Serial.print(l); Serial.print('|'); Serial.println(r);
      } else Serial.println(F("[ERROR] W <l> <r>"));
      break;
    }
    case 'G': {
      char* endP; long ddeg = strtol(arg, &endP, 10);
      while (*endP == ' ') endP++;
      if (endP != arg && *endP == '\0' && ddeg != 0 && ddeg >= -3600 && ddeg <= 3600) {
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
    case 'L': case 'R': {
      char* endP; long us = strtol(arg, &endP, 10);
      while (*endP == ' ') endP++;
      if (endP != arg && *endP == '\0' && us >= 1000 && us <= 2000) {
        act.rawUs(op == 'L', (int16_t)us); mode = M_CAL;
        Serial.print(F("[ACK] ")); Serial.print(op); Serial.print('|'); Serial.println(us);
      } else Serial.println(F("[ERROR] L|R <1000-2000>"));
      break;
    }
    case 'C': {
      int16_t cl, cr;
      if (parseTwoInts(arg, cl, cr) && cl >= 1300 && cl <= 1700 && cr >= 1300 && cr <= 1700) {
        act.setCenters(cl, cr);
        Serial.print(F("[ACK] C|")); Serial.print(cl); Serial.print('|'); Serial.println(cr);
      } else Serial.println(F("[ERROR] C <l> <r>"));
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
    case 'K': {
      Serial.print(F("[CAL] cL=")); Serial.print(act.centerL());
      Serial.print(F(",cR="));      Serial.print(act.centerR());
      Serial.print(F(",det="));     Serial.println(act.stopDetach() ? 1 : 0);
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
  Serial.print(F("[READY] UNO_ROVER|IMU=")); Serial.println(imu.ok() ? 1 : 0);
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
