#include "arduino_secrets.h"

// ============================================
// ARDUINO BIDIRECTIONAL COMMUNICATION
// ============================================
// BLINK WITH DUTY CYCLE CONTROL
// WITH ERROR HANDLING
// ============================================
// ============================================
// ARDUINO NON-BLOCKING BLINK WITH TIMESTAMP
// ============================================

const int LED_PIN = 13;

unsigned long previousMillis = 0;
int blinkInterval = 1000;
bool ledState = LOW;

String receivedData = "";
unsigned long lastReceiveTime = 0;

// Limits
const unsigned long MIN_INTERVAL = 10;
const unsigned long MAX_INTERVAL = 10000;

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("=================================");
    Serial.println("ARDUINO NON-BLOCKING BLINK v3");
    Serial.println("=================================");
    Serial.print("Range: ");
    Serial.print(MIN_INTERVAL);
    Serial.print("ms - ");
    Serial.print(MAX_INTERVAL);
    Serial.println("ms");
    Serial.println("---------------------------------");
}

void loop() {
    // ========================================
    // PART 1: PROCESS INCOMING DATA
    // ========================================
    if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('\n');
        receivedData.trim();
        
        if (receivedData.length() > 0) {
            int newInterval = receivedData.toInt();
            
            if (newInterval >= MIN_INTERVAL && newInterval <= MAX_INTERVAL) {
                blinkInterval = newInterval;
                lastReceiveTime = millis();
                
                // Send confirmation WITH millis() timestamp
                Serial.print("[OK] Interval: ");
                Serial.print(blinkInterval);
                Serial.print("ms | Arduino time: ");
                Serial.print(lastReceiveTime);
                Serial.println("ms");
                
                // Reset timing for smooth transition
                previousMillis = millis();
                
            } else {
                // Send error WITH millis() timestamp
                Serial.print("[ERROR] Interval must be ");
                Serial.print(MIN_INTERVAL);
                Serial.print("ms - ");
                Serial.print(MAX_INTERVAL);
                Serial.print("ms | Arduino time: ");
                Serial.print(millis());
                Serial.println("ms");
            }
        }
    }
    
    // ========================================
    // PART 2: NON-BLOCKING BLINK
    // ========================================
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}