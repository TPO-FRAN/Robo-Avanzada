/*
  arduino/servo_calibration/servo_calibration.ino
  Calibrador Simple de Servomotores MG996R mediante Driver PCA9685 (4 Servos)
  UA62 - Avance 2 | Arduino Uno + PCA9685 (0x40)

  Funcionalidades:
    - Serial a 9600 baudios.
    - Ingresa 0, 1, 2 o 3 para elegir qué servo calibrar individualmente.
    - Ingresa 4 para controlar los 4 servos simultáneamente.
    - Ingresa un valor en microsegundos (1000..2000 us) para enviar la señal.
    - También puedes escribir '0 1490' para asignar directamente.
    - Escribe 's' o 'STOP' para cortar la señal PWM (detención física).
*/

#include <Arduino.h>
#include <Wire.h>

// ── Configuración Hardware PCA9685 ──────────────────────────────────────────
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t MODE1         = 0x00;
constexpr uint8_t MODE2         = 0x01;
constexpr uint8_t LED0_ON_L     = 0x06;
constexpr uint8_t PRESCALE_REG  = 0xFE;

// Prescaler 131 calibrado para oscilador real de 27 MHz (50 Hz exactos, 1500 us neutro)
constexpr uint8_t PRESCALE_VAL  = 131;
constexpr uint8_t NUM_SERVOS    = 4;   // Canales 0, 1, 2, 3 (FL, RL, FR, RR)

const char* const NOMBRES_SERVOS[NUM_SERVOS] = {
  "CH0 (Delantera Izq - FL)",
  "CH1 (Trasera Izq - RL)",
  "CH2 (Delantera Der - FR)",
  "CH3 (Trasera Der - RR)"
};

int servoActivo = 4;    // 0=FL, 1=RL, 2=FR, 3=RR, 4=TODOS (por defecto todos)
int pulsoActual = 1500; // Valor inicial (punto de detención neutro)

// Convierte microsegundos a ticks a 50 Hz real
static uint16_t usToTicks(int us) {
  uint16_t ticks = (uint16_t)(((uint32_t)us * 9UL + 22UL) / 44UL);
  if (ticks > 4095) ticks = 4095;
  return ticks;
}

// Envía pulso en microsegundos a un canal específico del PCA9685
static void pcaWriteMicroseconds(uint8_t ch, int us) {
  uint16_t ticks = usToTicks(us);

  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write((uint8_t)(LED0_ON_L + 4 * ch));
  Wire.write((uint8_t)0x00);                  // ON_L
  Wire.write((uint8_t)0x00);                  // ON_H
  Wire.write((uint8_t)(ticks & 0xFF));        // OFF_L
  Wire.write((uint8_t)((ticks >> 8) & 0x0F)); // OFF_H (FULL_OFF = 0)
  Wire.endTransmission();
}

// Envía la señal PWM al servo seleccionado (o a todos si servoActivo == 4)
static void aplicarPulso(int servo, int us) {
  if (servo < NUM_SERVOS) {
    pcaWriteMicroseconds(servo, us);
  } else {
    for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
      pcaWriteMicroseconds(ch, us);
    }
  }
}

// Corta la señal PWM en todos los canales (detención física completa)
static void pcaStopAll() {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write((uint8_t)0xFA); // ALL_LED_ON_L
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x10); // ALL_LED_OFF_H = FULL_OFF
  Wire.endTransmission();
}

// Inicialización limpia del PCA9685
static void pcaInit() {
  Wire.begin();
  Wire.setClock(100000L); // 100 kHz estándar
  delay(20);

  // 1. Reset
  Wire.beginTransmission(PCA9685_ADDR); Wire.write(MODE1); Wire.write(0x00); Wire.endTransmission(); delay(10);
  // 2. Dormir para poder escribir el PRE_SCALE (espera requerida de 10ms)
  Wire.beginTransmission(PCA9685_ADDR); Wire.write(MODE1); Wire.write(0x10); Wire.endTransmission(); delay(10);
  // 3. Escribir prescaler calibrado
  Wire.beginTransmission(PCA9685_ADDR); Wire.write(PRESCALE_REG); Wire.write(PRESCALE_VAL); Wire.endTransmission(); delay(2);
  // 4. Despertar con Auto-Increment
  Wire.beginTransmission(PCA9685_ADDR); Wire.write(MODE1); Wire.write(0x20); Wire.endTransmission(); delay(5);
  // 5. Configurar salida en Totem-Pole
  Wire.beginTransmission(PCA9685_ADDR); Wire.write(MODE2); Wire.write(0x04); Wire.endTransmission(); delay(5);
}

static void imprimirEnvio(int servo, int us) {
  Serial.print("Señal enviada a ");
  if (servo == 4) {
    Serial.print("TODOS los servos (CH0..CH3): ");
  } else {
    Serial.print(NOMBRES_SERVOS[servo]);
    Serial.print(": ");
  }
  Serial.print(us);
  Serial.println(" us");
}

void setup() {
  Serial.begin(9600);
  pcaInit();

  // Enviar señal inicial de 1500 us a los 4 servomotores
  aplicarPulso(servoActivo, pulsoActual);

  Serial.println("--- Calibrador de Servo Continuo MG996R (PCA9685) ---");
  Serial.println("Selecciona que servo calibrar:");
  Serial.println("  0 : CH0 (Delantera Izq - FL)");
  Serial.println("  1 : CH1 (Trasera Izq - RL)");
  Serial.println("  2 : CH2 (Delantera Der - FR)");
  Serial.println("  3 : CH3 (Trasera Der - RR)");
  Serial.println("  4 : TODOS los servos a la vez");
  Serial.println("Luego ingresa el pulso en microsegundos (1000 a 2000).");
  Serial.println("Escribe 's' para apagar senal (STOP).");
  Serial.println("-----------------------------------------------------");
  Serial.println("Servo activo inicial: 4 (TODOS los servos)");
}

void loop() {
  if (Serial.available() > 0) {
    // Si escribe 's', 'S' o 'x' corta la señal inmediatamente
    char c = Serial.peek();
    if (c == 's' || c == 'S' || c == 'x' || c == 'X') {
      while (Serial.available() > 0) Serial.read();
      pcaStopAll();
      Serial.println("Señal cortada: Servos detenidos (STOP).");
      return;
    }

    int val1 = Serial.parseInt();

    // Comprobar si viene un segundo número en la misma línea (ej: "0 1490")
    int val2 = -1;
    while (Serial.peek() == ' ') Serial.read();
    if (Serial.peek() >= '0' && Serial.peek() <= '9') {
      val2 = Serial.parseInt();
    }
    while (Serial.available() > 0) Serial.read();

    // Caso A: Ingreso combinado "servo pulso" (ej: "0 1490", "2 1510")
    if (val1 >= 0 && val1 <= 4 && val2 >= 1000 && val2 <= 2000) {
      servoActivo = val1;
      pulsoActual = val2;
      aplicarPulso(servoActivo, pulsoActual);
      imprimirEnvio(servoActivo, pulsoActual);
      return;
    }

    // Caso B: Selección de qué servo controlar (0..3 individual, 4 todos)
    if (val1 >= 0 && val1 <= 3) {
      servoActivo = val1;
      Serial.print("-> Servo seleccionado: ");
      Serial.println(NOMBRES_SERVOS[servoActivo]);
      Serial.println("Ingresa ahora el pulso en microsegundos (ej: 1490, 1500, 1510):");
    } else if (val1 == 4) {
      servoActivo = 4;
      Serial.println("-> Seleccionados: TODOS LOS SERVOS (CH0..CH3)");
      Serial.println("Ingresa ahora el pulso en microsegundos (ej: 1490, 1500, 1510):");
    }
    // Caso C: Pulso directo en microsegundos (1000..2000 us) al servo activo
    else if (val1 >= 1000 && val1 <= 2000) {
      pulsoActual = val1;
      aplicarPulso(servoActivo, pulsoActual);
      imprimirEnvio(servoActivo, pulsoActual);
    }
  }
}
