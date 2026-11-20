/*
  esp32/hardware_calibration/hardware_calibration.ino
  Interactive Hardware & Calibration Tool for ESP32 DevKit — Entrega Final

  Subsystems Covered (100% Direct ESP32 DevKit LEDC & GPIO):
    - 2x L298N Mini (MX1508) DC Motors (8 GPIOs, LEDC 2 kHz, 10-bit):
        * 0 = FL (IN1=GPIO 25, IN2=GPIO 26)
        * 1 = RL (IN3=GPIO 27, IN4=GPIO 14)
        * 2 = FR (IN1=GPIO 32, IN2=GPIO 33)
        * 3 = RR (IN3=GPIO 16, IN4=GPIO 17)
    - 2-DOF Robotic Arm (Direct ESP32 LEDC 50 Hz, 14-bit):
        * 0 = Base  (GPIO 23, ch 8)
        * 1 = Elbow (GPIO 13, ch 9)
        * Note: Power servo V+ (5-6V) externally with common GND.
    - Hardware I2C Bus (SDA=GPIO 21, SCL=GPIO 22, 400 kHz):
        * GY-511 LSM303DLHC Magnetometer (0x1E) & Accelerometer (0x19)

  Serial Monitor Commands @ 115200 baud ("Newline"):
    - M <0-3|L|R|ALL> <-100..100> : Drive DC motor(s) at raw duty %
    - BRAKE                       : Engage L298N active short-brake (IN1=IN2=HIGH)
    - S / STOP                    : Coast stop all DC motors (IN1=IN2=LOW)
    - B <0-180>                   : Move Arm Base (GPIO 23) to angle in degrees
    - E <0-180>                   : Move Arm Elbow (GPIO 13) to angle in degrees
    - US <0|1> <500-2500>         : Raw microsecond pulse on Base (0) or Elbow (1)
    - HOME / EXT / OFF            : Arm HOME (90,25), EXT (90,145), or PWM OFF (duty=0)
    - SCAN                        : Re-scan I2C bus (GPIO 21/22)
    - HELP / ?                    : Show command menu
*/

#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#include <Wire.h>
#include <stdlib.h>
#include <strings.h>

// ── ESP32 Hardware Pinout ────────────────────────────────────────────────────
constexpr uint8_t PIN_FL_A = 25, PIN_FL_B = 26;
constexpr uint8_t PIN_RL_A = 27, PIN_RL_B = 14;
constexpr uint8_t PIN_FR_A = 32, PIN_FR_B = 33;
constexpr uint8_t PIN_RR_A = 16, PIN_RR_B = 17;

constexpr uint8_t PIN_ARM_B = 23;
constexpr uint8_t PIN_ARM_E = 13;

constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;

constexpr uint32_t MOTOR_PWM_HZ        = 2000UL;
constexpr uint8_t  MOTOR_PWM_BITS      = 10U;
constexpr uint32_t MOTOR_PWM_MAX       = (1UL << MOTOR_PWM_BITS) - 1UL;

constexpr uint32_t SERVO_PWM_HZ        = 50UL;
constexpr uint8_t  SERVO_PWM_BITS      = 14U;
constexpr uint32_t SERVO_PWM_PERIOD_US = 20000UL;
constexpr uint32_t SERVO_DUTY_RANGE    = (1UL << SERVO_PWM_BITS);
constexpr uint8_t  CH_ARM_B            = 8;
constexpr uint8_t  CH_ARM_E            = 9;

constexpr int16_t ARM_MIN_US = 544;
constexpr int16_t ARM_MAX_US = 2400;
constexpr uint8_t NUM_WHEELS = 4;

struct MotorPinPair {
  const char* name;
  uint8_t     pinA;
  uint8_t     pinB;
  uint8_t     chA;
  uint8_t     chB;
};

constexpr MotorPinPair MOTORS[NUM_WHEELS] = {
  { "0: FL (L298N Izq A)", PIN_FL_A, PIN_FL_B, 0, 1 },
  { "1: RL (L298N Izq B)", PIN_RL_A, PIN_RL_B, 2, 3 },
  { "2: FR (L298N Der A)", PIN_FR_A, PIN_FR_B, 4, 5 },
  { "3: RR (L298N Der B)", PIN_RR_A, PIN_RR_B, 6, 7 }
};

static char    rxBuf[64];
static uint8_t rxIdx = 0;

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

static int16_t degToUs(int16_t deg) {
  int16_t c = constrain(deg, 0, 180);
  return (int16_t)(ARM_MIN_US + (((int32_t)(ARM_MAX_US - ARM_MIN_US) * c + 90L) / 180L));
}

static uint32_t usToServoDuty(int16_t us) {
  int16_t c = constrain(us, 500, 2500);
  return (uint32_t)(((uint32_t)c * SERVO_DUTY_RANGE + (SERVO_PWM_PERIOD_US / 2UL)) / SERVO_PWM_PERIOD_US);
}

static void setMotorRawPct(uint8_t idx, int16_t pct) {
  if (idx >= NUM_WHEELS) return;
  const MotorPinPair &m = MOTORS[idx];
  int16_t c = constrain(pct, -100, 100);
  if (c == 0) {
    pwmWrite(m.pinA, m.chA, 0);
    pwmWrite(m.pinB, m.chB, 0);
    return;
  }
  uint32_t duty = ((uint32_t)abs(c) * MOTOR_PWM_MAX + 50UL) / 100UL;
  if (duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
  if (c > 0) {
    pwmWrite(m.pinA, m.chA, duty);
    pwmWrite(m.pinB, m.chB, 0);
  } else {
    pwmWrite(m.pinA, m.chA, 0);
    pwmWrite(m.pinB, m.chB, duty);
  }
}

static void coastAllMotors() {
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    pwmWrite(MOTORS[i].pinA, MOTORS[i].chA, 0);
    pwmWrite(MOTORS[i].pinB, MOTORS[i].chB, 0);
  }
}

static void brakeAllMotors() {
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    pwmWrite(MOTORS[i].pinA, MOTORS[i].chA, MOTOR_PWM_MAX);
    pwmWrite(MOTORS[i].pinB, MOTORS[i].chB, MOTOR_PWM_MAX);
  }
}

static void setServoUs(uint8_t servoIdx, int16_t us) {
  uint32_t duty = usToServoDuty(us);
  if (servoIdx == 0) pwmWrite(PIN_ARM_B, CH_ARM_B, duty);
  else               pwmWrite(PIN_ARM_E, CH_ARM_E, duty);
}

static void servosOff() {
  pwmWrite(PIN_ARM_B, CH_ARM_B, 0);
  pwmWrite(PIN_ARM_E, CH_ARM_E, 0);
}

static void runI2cScan() {
  Serial.println(F("\n[I2C] Escaneando bus (SDA=21, SCL=22)..."));
  uint8_t found = 0;
  bool hasMag = false, hasAcc = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      found++;
      Serial.print(F("  - Dispositivo en 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == 0x1E) { Serial.print(F(" -> GY-511 Magnetómetro (LSM303DLHC)")); hasMag = true; }
      if (addr == 0x19) { Serial.print(F(" -> GY-511 Acelerómetro (LSM303DLHC)")); hasAcc = true; }
      Serial.println();
    }
  }
  if (found == 0) {
    Serial.println(F("  [WARN] No se detectaron dispositivos I2C. Verifica VCC=3.3V, GND, SDA=21, SCL=22."));
  } else {
    Serial.print(F("[I2C] Total: ")); Serial.print(found);
    Serial.print(F(" | GY-511 Mag(0x1E)=")); Serial.print(hasMag ? F("OK") : F("NO"));
    Serial.print(F(" | Accel(0x19)=")); Serial.println(hasAcc ? F("OK") : F("NO"));
  }
}

static void printMenu() {
  Serial.println(F("======================================================================"));
  Serial.println(F("  CALIBRADOR ESP32 DEVKIT — 2x L298N Mini + Brazo LEDC + I2C GY-511"));
  Serial.println(F("----------------------------------------------------------------------"));
  Serial.println(F("  Motores DC (2x L298N Mini, sin inversion):"));
  Serial.println(F("    M <0-3|L|R|ALL> <-100..100>  Ej: M 0 40 | M L 60 | M ALL 50"));
  Serial.println(F("    BRAKE                        Freno activo (HIGH/HIGH)"));
  Serial.println(F("    S | STOP                     Paro libre / coast (LOW/LOW)"));
  Serial.println(F("  Brazo 2 GDL directo en ESP32 (GPIO 23=Base, GPIO 13=Codo, 50 Hz):"));
  Serial.println(F("    B <0-180>                    Mover Base a grados (ej: B 90)"));
  Serial.println(F("    E <0-180>                    Mover Codo a grados (ej: E 145)"));
  Serial.println(F("    US <0|1> <500-2500>          Pulso crudo us en Base(0) o Codo(1)"));
  Serial.println(F("    HOME / EXT / OFF             Presets (90,25), (90,145) o apagar PWM"));
  Serial.println(F("  Diagnostico:"));
  Serial.println(F("    SCAN                         Re-escanear bus I2C (GY-511)"));
  Serial.println(F("    HELP | ?                     Mostrar este menu"));
  Serial.println(F("======================================================================"));
}

static bool parseIntStrict(const char* s, long &out) {
  while (*s == ' ') s++;
  if (*s == '\0') return false;
  char* endP = nullptr;
  long v = strtol(s, &endP, 10);
  if (endP == s) return false;
  while (*endP == ' ') endP++;
  if (*endP != '\0') return false;
  out = v;
  return true;
}

static void handleCommand(const char* raw) {
  while (*raw == ' ') raw++;
  if (*raw == '\0') return;

  if (strcasecmp(raw, "HELP") == 0 || strcmp(raw, "?") == 0) {
    printMenu();
    return;
  }
  if (strcasecmp(raw, "SCAN") == 0) {
    runI2cScan();
    return;
  }
  if (strcasecmp(raw, "S") == 0 || strcasecmp(raw, "STOP") == 0) {
    coastAllMotors();
    Serial.println(F("[OK] Motores en coast stop (LOW/LOW)."));
    return;
  }
  if (strcasecmp(raw, "BRAKE") == 0) {
    brakeAllMotors();
    Serial.println(F("[OK] Freno activo aplicado en los 4 canales (HIGH/HIGH)."));
    return;
  }
  if (strcasecmp(raw, "HOME") == 0) {
    setServoUs(0, degToUs(90));
    setServoUs(1, degToUs(25));
    Serial.println(F("[OK] Brazo -> HOME (Base=90 deg, Codo=25 deg)."));
    return;
  }
  if (strcasecmp(raw, "EXT") == 0) {
    setServoUs(0, degToUs(90));
    setServoUs(1, degToUs(145));
    Serial.println(F("[OK] Brazo -> EXT (Base=90 deg, Codo=145 deg)."));
    return;
  }
  if (strcasecmp(raw, "OFF") == 0) {
    servosOff();
    Serial.println(F("[OK] PWM de servos del brazo apagado (duty=0)."));
    return;
  }

  if (strncasecmp(raw, "B ", 2) == 0) {
    long deg = 0;
    if (parseIntStrict(raw + 2, deg) && deg >= 0 && deg <= 180) {
      int16_t us = degToUs((int16_t)deg);
      setServoUs(0, us);
      Serial.print(F("[OK] Base (GPIO 23) -> ")); Serial.print(deg);
      Serial.print(F(" deg (")); Serial.print(us); Serial.println(F(" us)"));
    } else {
      Serial.println(F("[ERROR] Uso: B <0-180>"));
    }
    return;
  }

  if (strncasecmp(raw, "E ", 2) == 0) {
    long deg = 0;
    if (parseIntStrict(raw + 2, deg) && deg >= 0 && deg <= 180) {
      int16_t us = degToUs((int16_t)deg);
      setServoUs(1, us);
      Serial.print(F("[OK] Codo (GPIO 13) -> ")); Serial.print(deg);
      Serial.print(F(" deg (")); Serial.print(us); Serial.println(F(" us)"));
    } else {
      Serial.println(F("[ERROR] Uso: E <0-180>"));
    }
    return;
  }

  if (strncasecmp(raw, "US ", 3) == 0) {
    const char* p = raw + 3;
    while (*p == ' ') p++;
    char* end1 = nullptr;
    long idx = strtol(p, &end1, 10);
    long us  = 0;
    if (end1 != p && (idx == 0 || idx == 1) && parseIntStrict(end1, us) && us >= 500 && us <= 2500) {
      setServoUs((uint8_t)idx, (int16_t)us);
      Serial.print(F("[OK] Servo ")); Serial.print(idx == 0 ? F("Base(GPIO 23)") : F("Codo(GPIO 13)"));
      Serial.print(F(" -> ")); Serial.print(us); Serial.println(F(" us"));
    } else {
      Serial.println(F("[ERROR] Uso: US <0|1> <500-2500>"));
    }
    return;
  }

  if (strncasecmp(raw, "M ", 2) == 0) {
    const char* p = raw + 2;
    while (*p == ' ') p++;
    char target[8] = { 0 };
    uint8_t ti = 0;
    while (*p != '\0' && *p != ' ' && ti < sizeof(target) - 1) {
      target[ti++] = *p++;
    }
    long pct = 0;
    if (!parseIntStrict(p, pct) || pct < -100 || pct > 100) {
      Serial.println(F("[ERROR] Uso: M <0-3|L|R|ALL> <-100..100>"));
      return;
    }
    if (strcasecmp(target, "ALL") == 0) {
      for (uint8_t i = 0; i < NUM_WHEELS; i++) setMotorRawPct(i, (int16_t)pct);
      Serial.print(F("[OK] ALL motores -> ")); Serial.print(pct); Serial.println('%');
    } else if (strcasecmp(target, "L") == 0) {
      setMotorRawPct(0, (int16_t)pct);
      setMotorRawPct(1, (int16_t)pct);
      Serial.print(F("[OK] Lado IZQ (FL, RL) -> ")); Serial.print(pct); Serial.println('%');
    } else if (strcasecmp(target, "R") == 0) {
      setMotorRawPct(2, (int16_t)pct);
      setMotorRawPct(3, (int16_t)pct);
      Serial.print(F("[OK] Lado DER (FR, RR) -> ")); Serial.print(pct); Serial.println('%');
    } else {
      long idx = 0;
      if (parseIntStrict(target, idx) && idx >= 0 && idx < NUM_WHEELS) {
        setMotorRawPct((uint8_t)idx, (int16_t)pct);
        Serial.print(F("[OK] ")); Serial.print(MOTORS[idx].name);
        Serial.print(F(" -> ")); Serial.print(pct); Serial.println('%');
      } else {
        Serial.println(F("[ERROR] Uso: M <0-3|L|R|ALL> <-100..100>"));
      }
    }
    return;
  }

  Serial.println(F("[ERROR] Comando no reconocido. Escribe HELP."));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000UL);
  Wire.setTimeOut(50);

  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    pwmAttach(MOTORS[i].pinA, MOTOR_PWM_HZ, MOTOR_PWM_BITS, MOTORS[i].chA);
    pwmAttach(MOTORS[i].pinB, MOTOR_PWM_HZ, MOTOR_PWM_BITS, MOTORS[i].chB);
  }
  coastAllMotors();

  pwmAttach(PIN_ARM_B, SERVO_PWM_HZ, SERVO_PWM_BITS, CH_ARM_B);
  pwmAttach(PIN_ARM_E, SERVO_PWM_HZ, SERVO_PWM_BITS, CH_ARM_E);
  setServoUs(0, degToUs(90));
  setServoUs(1, degToUs(25));

  delay(200);
  printMenu();
  runI2cScan();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxIdx > 0) {
        rxBuf[rxIdx] = '\0';
        handleCommand(rxBuf);
        rxIdx = 0;
      }
    } else if (rxIdx < sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    }
  }
}
