/*
  arduino/sensor_firmware/sensor_firmware.ino
  Rover Perception Sensor Node — Object-Oriented Firmware for Arduino Uno
  Classes:
    - MedianFIFO<T, N>  : Static circular buffer + Insertion Sort median filter
    - UltrasonicSensor  : HC-SR04 hardware driver with MedianFIFO
    - AnalogSensor      : Generic ADC sensor driver (LDR & LM35) with MedianFIFO
    - RoverSensorNode   : Main controller managing non-blocking interleaved sampling
                          and zero-heap-allocation UART command parsing
  Pins:
    - LDR (Light level)         : Analog A0
    - LM35 (Temperature in °C)  : Analog A1
    - HC-SR04 #1 (Ultrasonic 1) : TRIG = D2, ECHO = D3
    - HC-SR04 #2 (Ultrasonic 2) : TRIG = D4, ECHO = D5
*/

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ── Pin & Filter Constants ───────────────────────────────────────────────────
constexpr uint8_t  PIN_LDR          = A0;
constexpr uint8_t  PIN_LM35         = A1;
constexpr uint8_t  PIN_TRIG1        = 2;
constexpr uint8_t  PIN_ECHO1        = 3;
constexpr uint8_t  PIN_TRIG2        = 4;
constexpr uint8_t  PIN_ECHO2        = 5;

constexpr uint8_t  FIFO_WINDOW      = 5;       // Odd window size for exact median
constexpr uint32_t PHASE_PERIOD_MS  = 25;      // 25 ms per phase -> 50 ms (20 Hz) per sensor
constexpr uint32_t PULSE_TIMEOUT_US = 25000UL; // ~4.2 m max range timeout

// ── Class 1: Circular FIFO Median Filter ─────────────────────────────────────
template <typename T, uint8_t N>
class MedianFIFO {
  private:
    T       _buf[N];
    uint8_t _head  = 0;
    uint8_t _count = 0;

  public:
    MedianFIFO() {
      for (uint8_t i = 0; i < N; ++i) _buf[i] = 0;
    }

    void push(T val) {
      _buf[_head] = val;
      _head = (_head + 1) % N;
      if (_count < N) _count++;
    }

    T median() const {
      if (_count == 0) return 0;
      T sorted[N];
      for (uint8_t i = 0; i < _count; ++i) {
        sorted[i] = _buf[i];
      }
      for (uint8_t i = 1; i < _count; ++i) {
        T key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
          sorted[j + 1] = sorted[j];
          j--;
        }
        sorted[j + 1] = key;
      }
      return sorted[_count / 2];
    }
};

// ── Class 2: Ultrasonic Sensor (HC-SR04) ─────────────────────────────────────
class UltrasonicSensor {
  private:
    const uint8_t _trigPin;
    const uint8_t _echoPin;
    MedianFIFO<float, FIFO_WINDOW> _filter;
    float _lastValidCm = 400.0f;

  public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
      : _trigPin(trigPin), _echoPin(echoPin) {}

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

      unsigned long duration = pulseIn(_echoPin, HIGH, PULSE_TIMEOUT_US);
      float cm = (duration == 0) ? _lastValidCm : (duration * 0.0343f) * 0.5f;
      if (duration > 0) {
        _lastValidCm = cm;
      }
      _filter.push(cm);
    }

    float getFilteredCm() const {
      return _filter.median();
    }
};

// ── Class 3: Analog Sensor (LDR / LM35) ──────────────────────────────────────
class AnalogSensor {
  private:
    const uint8_t _pin;
    const float   _scale;
    MedianFIFO<float, FIFO_WINDOW> _filter;

  public:
    AnalogSensor(uint8_t pin, float scale = 1.0f)
      : _pin(pin), _scale(scale) {}

    void init() {
      pinMode(_pin, INPUT);
    }

    void sample() {
      int raw = analogRead(_pin);
      _filter.push(raw * _scale);
    }

    float getFilteredFloat() const {
      return _filter.median();
    }

    int getFilteredInt() const {
      return (int)(_filter.median() + 0.5f);
    }
};

// ── Class 4: Rover Sensor Node Controller ────────────────────────────────────
class RoverSensorNode {
  private:
    AnalogSensor     _ldr;
    AnalogSensor     _lm35;
    UltrasonicSensor _us1;
    UltrasonicSensor _us2;

    uint32_t _lastPhaseMs = 0;
    bool     _toggleUs    = false; // Alternates US1 and US2 to prevent acoustic cross-talk
    char     _rxBuf[32];
    uint8_t  _rxIdx       = 0;

    void _sendTelemetry() const {
      int   ldr_val  = _ldr.getFilteredInt();
      float temp_val = _lm35.getFilteredFloat();
      float us1_val  = _us1.getFilteredCm();
      float us2_val  = _us2.getFilteredCm();

      char s_temp[12], s_us1[12], s_us2[12], out[96];
      dtostrf(temp_val, 4, 2, s_temp);
      dtostrf(us1_val,  4, 2, s_us1);
      dtostrf(us2_val,  4, 2, s_us2);

      snprintf(out, sizeof(out),
               "[DATA] ldr=%d,temp=%s,us1=%s,us2=%s",
               ldr_val, s_temp, s_us1, s_us2);
      Serial.println(out);
    }

    void _handleCommand(const char* rawCmd) {
      // Trim leading spaces
      while (*rawCmd == ' ' || *rawCmd == '\t') rawCmd++;
      if (*rawCmd == '\0') return;

      char up[32];
      uint8_t i = 0;
      while (rawCmd[i] != '\0' && i < sizeof(up) - 1) {
        up[i] = (char)toupper((unsigned char)rawCmd[i]);
        i++;
      }
      // Trim trailing spaces
      while (i > 0 && (up[i - 1] == ' ' || up[i - 1] == '\t')) i--;
      up[i] = '\0';

      if (strcmp(up, "READ") == 0 || strcmp(up, "STATUS") == 0 || strcmp(up, "CAPTURE") == 0) {
        _sendTelemetry();
      } else {
        Serial.print("[ERROR] unknown command: ");
        Serial.println(up);
      }
    }

  public:
    RoverSensorNode(uint8_t pinLdr, uint8_t pinLm35,
                    uint8_t trig1, uint8_t echo1,
                    uint8_t trig2, uint8_t echo2)
      : _ldr(pinLdr, 1.0f),
        _lm35(pinLm35, 500.0f / 1023.0f),
        _us1(trig1, echo1),
        _us2(trig2, echo2) {}

    void init() {
      _ldr.init();
      _lm35.init();
      _us1.init();
      _us2.init();

      // Prime FIFO buffers with spaced readings to avoid ultrasonic echo overlap
      for (uint8_t i = 0; i < FIFO_WINDOW; ++i) {
        _ldr.sample();
        _lm35.sample();
        _us1.sample();
        delay(15);
        _us2.sample();
        delay(15);
      }
      Serial.println("[READY] RoverSensorNode | OOP FIFO Median N=5 | LDR, LM35, 2x HC-SR04");
    }

    void update() {
      // 1. Non-blocking interleaved sensor sampling
      uint32_t now = millis();
      if (now - _lastPhaseMs >= PHASE_PERIOD_MS) {
        _lastPhaseMs = now;
        _ldr.sample();
        _lm35.sample();
        // Alternate US1 and US2 every 25ms to eliminate acoustic interference
        if (!_toggleUs) {
          _us1.sample();
        } else {
          _us2.sample();
        }
        _toggleUs = !_toggleUs;
      }

      // 2. Zero-heap-allocation Serial command reader
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
RoverSensorNode sensorNode(PIN_LDR, PIN_LM35, PIN_TRIG1, PIN_ECHO1, PIN_TRIG2, PIN_ECHO2);

void setup() {
  Serial.begin(9600);
  sensorNode.init();
}

void loop() {
  sensorNode.update();
}
