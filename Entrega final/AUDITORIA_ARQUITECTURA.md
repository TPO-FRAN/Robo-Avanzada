# Auditoría Técnica y Conservación de Arquitectura — `Entrega final`

Este documento certifica los cambios de hardware/firmware realizados respecto a `UA62 - Avance 2`, la auditoría completa del IMU **GY-511 (LSM303DLHC)** y la verificación formal de conservación de la arquitectura nativa en `raspberry/` y `laptop/`.

---

## 1. Comparativa de Arquitectura de Hardware y Firmware

| Subsistema | `UA62 - Avance 2` | `Entrega final` |
| :--- | :--- | :--- |
| **Controlador Embebido** | Arduino Uno (ATmega328P) | **ESP32 DevKit V1** (Dual Core 240 MHz, FPU hardware, NVS `Preferences`) |
| **Tracción 4WD** | 4× Servos continuos MG996R en PCA9685 (`CH 0..3`) | **4× Motores DC (ex-MG996R sin tarjeta) vía 2× L298N Mini** en **8 GPIOs LEDC independientes** (`2 kHz`, `10-bit`, ch `0..7`) + freno activo `80 ms` |
| **Brazo 2 GdL (2× MG996R)** | PCA9685 (`CH 4` Base, `CH 5` Codo) | **Control directo desde ESP32 DevKit vía LEDC hardware** (`GPIO 23` Base / ch `8`, `GPIO 13` Codo / ch `9`, `50 Hz`, `14-bit`) |
| **IMU / Brújula** | GY-511 (`0x1E`) en I2C compartido con PCA9685 | **GY-511 (`0x1E`, solo magnetómetro 2D) en I2C dedicado** (`SDA=21, SCL=22, 400 kHz`, timeout `50 ms`), pipeline refactorizado |
| **Ultrasónicos (2× HC-SR04)** | `PCINT2_vect` AVR + FIFO mediana-5 | **`attachInterruptArg` (`IRAM_ATTR` + `gpio_get_level` + `portMUX_TYPE`) + FIFO mediana-5** (misma histéresis `20 cm / 25 cm`) |
| **Persistencia de Calibración** | `EEPROM.h` AVR | **ESP32 NVS (`Preferences` namespaces `"rover"`, `"imu"` y `"hdg"`)** |
| **Control en Línea Recta** | Lazo abierto (dependiente de trim fijo) | **PID Discreto + Mezclador Diferencial con IMU GY-511 (`HeadingHold`)**, tolerancia configurable (`HDG_TOLERANCIA_DEG` / `H T <deg>`) |
| **Sketch Auxiliar** | `arduino/servo_calibration` (solo servos I2C) | **`esp32/hardware_calibration`** (4 motores DC L298N Mini + 2 servos LEDC + escáner I2C GY-511) |
| **Protocolo UART** | `W`, `G`, `A`, `X`, `?` @ `115200` baud | **100% conservado** (`W`, `G`, `A`, `X`, `?` + `[READY]`, `[ACK]`, `[STATUS]` de 9 campos, `[EVENT] TD\|OBS\|CLR`) |

---

## 2. Pinout Oficial ESP32 DevKit V1 y Seguridad Eléctrica

| Subsistema | Señal | GPIO ESP32 | Canal / Periférico |
| :--- | :--- | :---: | :--- |
| **L298N Mini Izquierdo — Motor FL (`0`)** | `IN1` / `IN2` | **25** / **26** | LEDC ch `0` / `1` (`2 kHz`, `10-bit`) |
| **L298N Mini Izquierdo — Motor RL (`1`)** | `IN3` / `IN4` | **27** / **14** | LEDC ch `2` / `3` (`2 kHz`, `10-bit`) |
| **L298N Mini Derecho — Motor FR (`2`)** | `IN1` / `IN2` | **32** / **33** | LEDC ch `4` / `5` (`2 kHz`, `10-bit`, `inverted=1`) |
| **L298N Mini Derecho — Motor RR (`3`)** | `IN3` / `IN4` | **19** / **18** | LEDC ch `6` / `7` (`2 kHz`, `10-bit`, `inverted=1`) |
| **Brazo 2 GdL — Servo Base** | Señal PWM | **23** | LEDC ch `8` (`50 Hz`, `14-bit`, grupo timer separado) |
| **Brazo 2 GdL — Servo Codo** | Señal PWM | **13** | LEDC ch `9` (`50 Hz`, `14-bit`, grupo timer separado) |
| **Bus I2C (GY-511 `0x1E`)** | `SDA` / `SCL` | **21** / **22** | I2C Hardware `400 kHz` (`VCC = 3.3V`) |
| **HC-SR04 #1** | `TRIG1` / `ECHO1` | **4** / **34** | GPIO 34 solo entrada + divisor `5V → 3.3V` |
| **HC-SR04 #2** | `TRIG2` / `ECHO2` | **2** / **35** | GPIO 35 solo entrada + divisor `5V → 3.3V` |

> [!WARNING]
> **Seguridad eléctrica obligatoria en ESP32 DevKit:**
> 1. Los pines GPIO de la ESP32 **no toleran 5V**. Conectar `ECHO1` (`GPIO 34`) y `ECHO2` (`GPIO 35`) mediante divisor resistivo (`1 kΩ` serie + `2 kΩ` a GND ≈ `3.33V`) o usar sensores de `3.3V`.
> 2. Alimentar `V+` de los 2 servos del brazo (`5–6V`) y la etapa de potencia de los 2× L298N Mini desde batería/regulador externo con **GND común** hacia la ESP32 (nunca alimentar motores ni servos desde el pin `3V3`/`VIN` de la placa).
> 3. Alimentar `VCC` del **GY-511** con **3.3V** para evitar inyectar `5V` por pull-ups hacia `GPIO 21/22`.

---

## 3. Auditoría de Incongruencias, Discrepancias y Redundancias del IMU (`GY-511`)

Inspección de `ImuSensor` y la máquina de estados en `UA62 - Avance 2/arduino/rover_firmware/rover_firmware.ino`:

| # | Hallazgo en `Avance 2` | Problema Técnico Observado | Solución Implementada en `Entrega final` |
| :-: | :--- | :--- | :--- |
| 1 | **Cuádruple retardo en cascada (~140–150 ms ciegos)** | `_settling` (65 ms / 5 muestras después de arrancar al 100%) + `median5` + IIR `(_filt + med)/2` + `confirmedTurn` de 3 muestras acumulaban 15°–20° de error sistemático en giros. | **Filtro único `median3`** en `(x, y)` (solo 1 muestra = `13.3 ms` de retardo), referencia instantánea sin ventana ciega, rampa lineal en los últimos `25°` y **freno activo L298N (`80 ms`)**. |
| 2 | **Corrupción de offsets en `commitDynamicOffsets()`** | Al concluir un giro parcial (ej. `90°`), calculaba `(min+max)/2` sobre un arco parcial y sobrescribía `_offX/_offY` con el punto medio de la cuerda. | Eliminados `commitDynamicOffsets()` y el tracking continuo de envolvente. La calibración solo se ejecuta explícitamente vía `I C` o `I <ox> <oy>`. |
| 3 | **Clamping `_turnDdeg = 0` + polaridad volátil `_rightSign`** | Un rebote inicial negativo recortado a `0` inyectaba grados fantasma al retornar. Si `_rightSign` aprendía el signo contrario, giros `< 25°` no corregían hasta el timeout. | Eliminados `_rawTurnDdeg`, clamping y heurística volátil. Se usa `_cwSign` determinista (`+1`/`-1`, persistido en NVS, auto-detectado en `I C`) con desenvolvimiento continuo sin recorte. |
| 4 | **Doble limitador de tasa sin lectura `DRDY`** | `tImu >= 2 ms` en `loop()` era redundante con `_lastMs < 13 ms`; leer a `76.9 Hz` sin consultar `SR_REG_M (0x09)` bit 0 ante ODR real de `75 Hz` inyectaba muestras repetidas al filtro. | Eliminado `tImu` de `loop()`. Se consulta `SR_REG_M` bit 0 `DRDY` tras `>= 10 ms` (con failsafe de `20 ms`), procesando exactamente 1 muestra nueva por conversión. |
| 5 | **Gate angular redundante (`IMU_GLITCH_DDEG`)** | Redundante sobre una mediana; al superar 3 rechazos resincronizaba `_lastCompassYaw` descartando rotación física real. | Eliminado `_glitchCnt`. Validación únicamente en dominio magnético (overflow `-4096` y vector nulo) antes de `median3`. |
| 6 | **Variables duplicadas y corrupción en `pollDiagnostic()`** | `_yawDdeg` y `_lastCompassYaw` duplicaban el estado, y `I` escribía rumbo crudo en `_yawDdeg` desincronizando el filtro. | Unificado en `_yawDeg` (`float`, fuente única de verdad). El comando `I` es de solo lectura sobre el estado del filtro. |
| 7 | **Calibración `I C` bloqueante (`2.8 s`)** | `while (millis() - t0 < 2800)` congelaba el puerto serie y los ultrasónicos, impidiendo `ALTO`/`X`. | Convertida en estado no bloqueante `M_CAL_IMU` en `loop()`, cancelable de inmediato con `ALTO`/`X` y verificando cobertura de los 4 cuadrantes. |
| 8 | **Sin compensación soft-iron + `atan2` double truncado** | No corregía elipticidad de ejes X/Y por hierro/motores DC y truncaba hacia cero sin redondeo ni módulo. | FPU hardware ESP32 (`atan2f`, `lroundf`, `% 3600`) con calibración **hard-iron `(offX, offY)` + soft-iron `(scaleX, scaleY)`**. |
| 9 | **Discrepancia `MPU-6050` vs `GY-511`** | `control.py`, `main.py` y `client.py` reportaban `"MPU-6050"` aunque el hardware real era `GY-511`. | Etiquetas unificadas a **GY-511 (LSM303DLHC)** manteniendo los opcodes `G` y eventos `TD` idénticos. |
| 10 | **Persistencia `EEPROM.h` AVR incompatible con ESP32** | `EEPROM.put()` sin `begin()/commit()` no persiste en ESP32. | Migrado a **`Preferences` (NVS nativo ESP32)** en namespaces `"imu"` y `"rover"`. |

---

## 4. Auditoría de Conservación de Arquitectura Nativa (`raspberry/` y `laptop/`)

### Comparativa Estructural (AST + Diff Línea a Línea contra `UA62 - Avance 2`)

| Archivo | Líneas (`Avance 2 → Entrega final`) | Líneas Diff | Clases / Funciones Preservadas (100% idénticas en AST) | Cambios Aplicados (Exclusivamente no estructurales) |
| :--- | :---: | :---: | :--- | :--- |
| `raspberry/control.py` | `633 → 633` | 56 líneas (solo strings/docstrings + 1 constante) | `RoverKinematics`, `RoverStatus`, `ArduinoSerialInterface`, `RoverController` (100% de métodos, firmas, hilos, locks, colas y opcodes `W, G, A, X, ?`) | `SERIAL_PORT = "/dev/ttyUSB0"`, prioridad `"USB"` antes de `"ACM"` en `find_port()`, etiquetas de texto/log de `Arduino Uno / MPU-6050` → `ESP32 DevKit / GY-511`. |
| `raspberry/server.py` | `174 → 174` | 2 líneas (docstring) | `UDPRoverServer`, `_disable_windows_udp_connreset` (2 hilos `WiFiListener` + `CommandDispatcher`, preempción `STOP`, push asíncrono y garantía `ACK` antes de `EVENT`) | Únicamente versión en docstring (`UA62 - Avance 2` → `Entrega Final`). |
| `raspberry/main.py` | `84 → 85` | 15 líneas (banner y textos CLI) | `RoverApplication`, `main` (CLI `--udp-ip`, `--udp-port`, `--serial-port`, `--baudrate`, paro de seguridad `STOP` en `Ctrl+C`) | Solo docstring, 3 líneas del banner de inicio y texto de ayuda de `--serial-port`. |
| `laptop/client.py` | `376 → 376` | 14 líneas (títulos y `_print_help()`) + IP servidor | `MonitorTerminalLauncher`, `ResponseMonitorTerminal`, `RoverControlClient`, `_disable_windows_udp_connreset`, `main` (doble terminal, handshake `__MONITOR_READY__`, sockets `_net_sock` e `_ipc_sock` aislados, `SERVER_PORT = 2222`) | Únicamente textos informativos en títulos, ayuda CLI, descripción de `argparse` y `SERVER_IP` ajustada a `"172.31.57.190"`. |

---

## 5. Corrección Activa de Rumbo en Línea Recta (`HeadingHold`)

Para corregir desviaciones durante marcha recta (`ADELANTE`, `ATRAS` o `W <l> <r>` con `l == r` y `|l| >= ZONA_MUERTA`) conservando intacta la calibración actual del IMU:

- **Solo lectura sobre `ImuSensor`**: `HeadingHold` únicamente lee `yawDeg()`, `cwSign()`, `ok()` y `sampleSeq()` (75 Hz), sin modificar `_offX`, `_offY`, `_scaleX`, `_scaleY`, `_cwSign` ni el namespace NVS `"imu"`.
- **Controlador PID Discreto + Mezclador Diferencial con Preservación de Margen**:
  - Espera `HDG_LOCK_DELAY_MS` (`100 ms`) tras arrancar desde reposo para evitar ruido magnético de corriente inicial antes de fijar `_targetYawDeg`.
  - Calcula el error angular envuelto (`[-180°, 180°] * cwSign`) y aplica una banda muerta suave configurable (`HDG_TOLERANCIA_DEG = 1.5°`).
  - Aplica `P + I` (con anti-windup condicional `±18%`) `+ D` filtrado (EMA α=0.25) acotado a `±35%` de diferencial (`|Der - Izq|`), desplazando la saturación entre ambos lados en velocidades bajas (`ZONA_MUERTA = 5%`) o máximas (`±100%`) tanto en `ADELANTE` como en `ATRAS`.
- **Configuración sencilla de tolerancia y ganancias**:
  - **En código**: bloque superior en `rover_firmware.ino` (`HDG_TOLERANCIA_DEG`, `HDG_DEFAULT_ENABLED`, `HDG_KP`, `HDG_KI`, `HDG_KD`).
  - **En vivo por Monitor Serie (`115200`)**: `H` (estado), `H 0|1` (apagar/encender), `H T <deg>` (cambiar tolerancia `0.0..15.0°`), `H <kp> <ki> <kd>`, `H S` (guardar en namespace NVS independiente `"hdg"`), `H R` (restaurar valores por defecto).
- **Desarmado seguro**: se desactiva automáticamente ante `GIRARD`/`GIRARI`, `W` asimétrico (`l != r`), `GIROD`/`GIROI`/`G`, `ALTO`/`X`/`S`, `P`, `I C` o evasión ultrasónica (`startAutoRev`), y si `!imu.ok()` vuelve de inmediato a conducción simétrica en lazo abierto.

---

## 6. Resultados de Verificación Ejecutados

1. **Compilación PlatformIO ESP32 DevKit (`-Wall -Wextra`)**:
   - `Entrega final/esp32/rover_firmware`: **`[SUCCESS]`** (0 errores, 0 warnings; RAM `7.6%` = `24,760 / 327,680 B`, Flash `27.0%` = `353,979 / 1,310,720 B`).
   - `Entrega final/esp32/hardware_calibration`: **`[SUCCESS]`** (0 errores, 0 warnings; RAM `7.3%` = `23,896 / 327,680 B`, Flash `24.6%` = `321,787 / 1,310,720 B`).
2. **Pruebas Matemáticas y Simulación de Lazo Cerrado (`HeadingHold`)**:
   - Verificados banda muerta configurable (`1.5°` y `3.0°`), cruce `359° ↔ 1°`, polaridad `cwSign = ±1`, saturación en `±100%` y `±10%` en avance y reversa, anti-windup y convergencia en simulación de 12 s @ 75 Hz (`<= 1.65°` con bias `6°/s`, `<= 2.01°` con bias `16°/s`, `<= 1.81°` en reversa y `<= 1.83°` al `100%`).
3. **Validación Sintáctica Python (`python -m py_compile`)**:
   - Código de salida `0` en `control.py`, `server.py`, `main.py` y `client.py`.
4. **Auditoría AST + Prueba End-to-End en Simulación Loopback UDP (`127.0.0.1`)**:
   - 100% coincidencia AST de clases, métodos, firmas y constantes de protocolo entre `UA62 - Avance 2` y `Entrega final`.
   - Verificados vía `UDPRoverServer` y `RoverControlClient`: `PING`, `ADELANTE 50`, `ATRAS 40`, `GIRARD 30`, `GIRARI 30`, `GIROD 90` + `[EVENT] TD`, `GIROI 45` + `[EVENT] TD`, `BRAZOEXT`, `BRAZOH`, `BRAZO 90 120`, `ESTADO`, `ALTO`, más parseo de `OBS`, `CLR`, `BLK` y trama `[STATUS]` de 9 campos.
