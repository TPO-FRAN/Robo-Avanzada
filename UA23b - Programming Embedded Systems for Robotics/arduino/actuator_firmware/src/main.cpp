#include <Arduino.h>

/*
  Motor : JGB37-520  12V 333 RPM  (gear 30:1)
  Encoder: Hall AB   11 PPR motor -> 330 PPR output shaft
  Driver : H-Bridge  ENA=9(PWM)  IN1=7  IN2=8
  Encoder: CHA=2(INT0)  CHB=3
  Serial : 9600 baud | Commands: "speed <0-255>" | "direction <CW|CCW>"
           "speed <v>, direction <d>" | "stop ON|OFF" | "resume" | "status"
*/

class MotorDriver {
  public:
    const uint8_t ENA, IN1, IN2, LED;
    bool is_stopped = false;
    uint8_t speed = 0;
    bool cw = true;

    MotorDriver(uint8_t ena, uint8_t in1, uint8_t in2, uint8_t led)
      : ENA(ena), IN1(in1), IN2(in2), LED(led) {}

    void init() {
      pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT); pinMode(LED, OUTPUT);
      stop();
    }

    void drive(uint8_t spd, bool clockwise) {
      if (is_stopped) { ack("BLOCKED"); return; }
      speed = spd; cw = clockwise;
      _apply();
      ack("OK");
    }

    void stop() {
      is_stopped = true; speed = 0;
      analogWrite(ENA, 0);
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
      digitalWrite(LED, HIGH);
      ack("STOP_ON");
    }

    void resume_motor() {
      is_stopped = false;
      digitalWrite(LED, LOW);
      _apply();
      ack("STOP_OFF");
    }

    void status_report(float rpm, long pulses) {
      char buf[80];
      snprintf(buf, sizeof(buf),
        "[STATUS] spd=%d dir=%s stop=%s rpm=%.1f pulses=%ld t=%lu",
        speed, cw ? "CW" : "CCW", is_stopped ? "ON" : "OFF", rpm, pulses, millis());
      Serial.println(buf);
    }

  private:
    void _apply() {
      digitalWrite(IN1, cw ? HIGH : LOW);
      digitalWrite(IN2, cw ? LOW : HIGH);
      analogWrite(ENA, speed);
      digitalWrite(LED, LOW);
    }

    void ack(const char* tag) {
      char buf[60];
      snprintf(buf, sizeof(buf),
        "[%s] spd=%d dir=%s stop=%s t=%lu",
        tag, speed, cw ? "CW" : "CCW", is_stopped ? "ON" : "OFF", millis());
      Serial.println(buf);
    }
};

MotorDriver motor(9, 7, 8, 13);

volatile long enc_ticks = 0;
long prev_ticks = 0;
uint32_t last_rpm = 0;
float current_rpm = 0.0f;

void encoderISR() { digitalRead(3) == LOW ? enc_ticks++ : enc_ticks--; }

void updateRPM() {
  uint32_t now = millis();
  if (now - last_rpm < 100) return;
  noInterrupts();
  long delta = enc_ticks - prev_ticks;
  prev_ticks = enc_ticks;
  interrupts();
  current_rpm = (float)delta / 330.0f * (60000.0f / (now - last_rpm));
  last_rpm = now;
}

void parseCommand(const String& raw) {
  String cmd = raw; cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd; up.toUpperCase();

  if (up == "STOP ON" || up == "STOP") { motor.stop(); return; }
  if (up == "STOP OFF" || up == "RESUME") { motor.resume_motor(); return; }
  if (up == "STATUS") { motor.status_report(current_rpm, enc_ticks); return; }

  uint8_t spd = motor.speed;
  bool dir = motor.cw;
  bool has_spd = false, has_dir = false;

  int start = 0;
  bool more = true;
  while (more) {
    int comma = up.indexOf(',', start);
    String tok;
    if (comma == -1) { tok = up.substring(start); more = false; }
    else { tok = up.substring(start, comma); start = comma + 1; }
    tok.trim();

    if (tok.startsWith("SPEED ")) {
      int v = tok.substring(6).toInt();
      if (v < 0 || v > 255) { Serial.println("[ERROR] speed out of range"); return; }
      spd = (uint8_t)v; has_spd = true;
    } else if (tok == "DIRECTION CW") { dir = true; has_dir = true; }
      else if (tok == "DIRECTION CCW") { dir = false; has_dir = true; }
      else { Serial.print("[ERROR] unknown token: "); Serial.println(tok); return; }
  }

  if (has_spd || has_dir) motor.drive(spd, dir);
  else Serial.println("[ERROR] empty command");
}

void setup() {
  Serial.begin(9600);
  motor.init();
  pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
  Serial.println("[READY] actuator_firmware | Motor 12V 333RPM | Encoder 330PPR");
}

String inputBuf;

void loop() {
  updateRPM();
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuf.length()) { parseCommand(inputBuf); inputBuf = ""; }
    } else {
      inputBuf += c;
    }
  }
}
