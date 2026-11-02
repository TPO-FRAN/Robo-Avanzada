#!/usr/bin/env python3
"""
raspberry/control.py — High-Efficiency UART Controller for Rover Actuators & Sensors (UA62 - Avance 2).

Architecture (Raspberry Pi 5 <-> Arduino Uno ATmega328P):
  - Offloads 100% of human-command parsing & validation to the Raspberry Pi 5.
  - Communicates with the 8-bit Arduino Uno over 115200-baud UART using 5 ultra-compact
    integer opcodes (`W`, `G`, `A`, `X`, `?`) so the Arduino runs zero float/printf bloat:
      * W <l> <r>   : Differential wheel speeds (-100..100 %) for 87.5 mm wheels
      * G <ddeg>    : Closed-loop MPU-6050 gyro turn in 0.1° units (+ = RIGHT, - = LEFT)
      * A <b> <e>   : 2-DOF robotic arm target angles (0..180°)
      * X           : Emergency STOP (detaches wheel PWM + freezes arm)
      * ?           : Query STATUS telemetry (mm & 0.1° integers)
"""
import math
import queue
import re
import threading
import time
from dataclasses import dataclass
from typing import Callable

import serial
import serial.tools.list_ports

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200

ARM_BASE_HOME    = 90
ARM_ELBOW_HOME   = 25
ARM_BASE_EXTEND  = 90
ARM_ELBOW_EXTEND = 145


class RoverKinematics:
    """Kinematics helper for 87.5 mm diameter wheels driven by MG996R continuous servos."""
    WHEEL_DIAMETER_MM: float = 87.5
    WHEEL_CIRCUMFERENCE_M: float = math.pi * (WHEEL_DIAMETER_MM / 1000.0)  # ~0.27489 m
    MAX_SERVO_RPM: float = 60.0  # MG996R unloaded speed @ 6.0V (~0.17 s / 60 deg)

    @classmethod
    def speed_pct_to_mps(cls, speed_pct: int | float) -> float:
        """Converts 0-100% servo command to linear velocity in m/s."""
        pct = max(0.0, min(100.0, abs(float(speed_pct))))
        rps = (cls.MAX_SERVO_RPM / 60.0) * (pct / 100.0)
        return rps * cls.WHEEL_CIRCUMFERENCE_M


@dataclass(frozen=True)
class RoverStatus:
    """Immutable value object representing the current state of actuators and sensors."""
    left_pct: int = 0
    right_pct: int = 0
    arm_base_deg: int = ARM_BASE_HOME
    arm_elbow_deg: int = ARM_ELBOW_HOME
    yaw_deg: float = 0.0
    imu_connected: bool = False
    us1_cm: float = 400.0
    us2_cm: float = 400.0
    mode: int = 0


class ArduinoSerialInterface:
    """
    Encapsulates UART communication with the lean Arduino Uno firmware.
    Guarantees:
      - Kernel-blocked `readline()` (0% idle CPU, sub-millisecond wakeup, zero polling).
      - Expected-prefix ACK verification so stale replies never desynchronize commands.
      - Deadlock-free chronological ordering (`[ACK]` always transmitted before subsequent `[EVENT]`).
    """
    _VALID_REPLY_PREFIXES = ("[ACK]", "[STATUS]", "[ERROR]")

    def __init__(
        self,
        preferred_port: str = SERIAL_PORT,
        baud: int = BAUD_RATE,
        event_callback: Callable[[str], None] | None = None,
    ):
        self.preferred_port = preferred_port
        self.baud           = baud
        self.event_callback = event_callback

        self._ser: serial.Serial | None = None
        self._write_lock = threading.Lock()
        self._ack_queue: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._sim_turn_timer: threading.Timer | None = None

        self._order_lock = threading.Lock()
        self._awaiting_cmd_reply = False
        self._ack_pending_udp = False
        self._deferred_events: list[str] = []

    @staticmethod
    def find_port(preferred_port: str) -> str:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if preferred_port in ports:
            return preferred_port
        for port in ports:
            if "ACM" in port or "USB" in port:
                print(f"[SERIAL] Puerto preferido '{preferred_port}' no hallado. Usando detectado: {port}")
                return port
        if ports:
            print(f"[SERIAL] Usando primer puerto detectado: {ports[0]}")
            return ports[0]
        print(f"[SERIAL WARN] No se detectaron puertos seriales activos. Intentando '{preferred_port}'.")
        return preferred_port

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self) -> bool:
        active = self.find_port(self.preferred_port)
        try:
            self._ser = serial.Serial(active, self.baud, timeout=0.1, write_timeout=0.5)
            time.sleep(2.2)  # Wait for Arduino Optiboot reset + IMU & FIFO init
            self._ser.reset_input_buffer()
            print(f"[UART] Conectado a {active} @ {self.baud} baud")

            self._stop_event.clear()
            self._reader_thread = threading.Thread(
                target=self._uart_reader_worker,
                daemon=True,
                name="UARTEventReader",
            )
            self._reader_thread.start()
            return True
        except (serial.SerialException, OSError) as e:
            print(f"[SERIAL WARN] No se pudo abrir '{active}' ({e}). Modo simulación UART activo.")
            self._ser = None
            return False

    def _dispatch_or_defer_event(self, event_line: str):
        defer = False
        with self._order_lock:
            if self._ack_pending_udp:
                self._deferred_events.append(event_line)
                defer = True
        if not defer and self.event_callback is not None:
            self.event_callback(event_line)

    def release_ack_and_flush_events(self):
        with self._order_lock:
            pending = list(self._deferred_events)
            self._deferred_events.clear()
            self._ack_pending_udp = False
        if self.event_callback is not None:
            for ev in pending:
                self.event_callback(ev)

    def _uart_reader_worker(self):
        while not self._stop_event.is_set():
            ser = self._ser
            if ser is None or not ser.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith("[READY]"):
                    print(f"[ARDUINO] {line}")
                elif line.startswith("[EVENT]"):
                    print(f"[UART EVENT] {line}")
                    self._dispatch_or_defer_event(line)
                elif line.startswith(self._VALID_REPLY_PREFIXES):
                    enqueue = False
                    with self._order_lock:
                        if self._awaiting_cmd_reply:
                            self._ack_pending_udp = True
                            enqueue = True
                    if enqueue:
                        self._ack_queue.put(line)
                    else:
                        print(f"[UART ORPHAN ACK IGNORED] {line}")
                else:
                    print(f"[UART NOISE IGNORED] {line}")
            except (serial.SerialException, OSError) as e:
                if not self._stop_event.is_set():
                    print(f"[UART READ ERR] {e}")
                time.sleep(0.1)

    @staticmethod
    def _expected_ack_prefixes(opcode_cmd: str) -> tuple[str, ...]:
        op = opcode_cmd.strip()[:1].upper()
        mapping: dict[str, tuple[str, ...]] = {
            "X": ("[ACK] X", "[ERROR]"),
            "?": ("[STATUS]", "[ERROR]"),
            "A": ("[ACK] A|", "[ERROR]"),
            "W": ("[ACK] W|", "[ACK] BLK|", "[ERROR]"),
            "G": ("[ACK] G|", "[ERROR]"),
        }
        return mapping.get(op, ("[ACK]", "[STATUS]", "[ERROR]"))

    def send_opcode(self, opcode_cmd: str, timeout: float = 1.5, defer_events: bool = False) -> str:
        """Transmits a compact machine opcode (`W`, `G`, `A`, `X`, `?`) to the Arduino Uno."""
        if not self.is_connected:
            if defer_events:
                with self._order_lock:
                    self._ack_pending_udp = True
            return self._simulate_opcode(opcode_cmd)

        expected = self._expected_ack_prefixes(opcode_cmd)

        with self._write_lock:
            while not self._ack_queue.empty():
                try:
                    self._ack_queue.get_nowait()
                except queue.Empty:
                    break

            with self._order_lock:
                self._awaiting_cmd_reply = True
                self._ack_pending_udp = False

            try:
                payload = f"{opcode_cmd.strip()}\n".encode("ascii")
                self._ser.write(payload)
                self._ser.flush()
            except (serial.SerialException, OSError) as e:
                with self._order_lock:
                    self._awaiting_cmd_reply = False
                    self._ack_pending_udp = False
                return f"[ERROR] Fallo de escritura UART: {e}"

            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    with self._order_lock:
                        self._awaiting_cmd_reply = False
                        self._ack_pending_udp = False
                    return "[ERROR] Timeout esperando respuesta del Arduino"
                try:
                    candidate = self._ack_queue.get(timeout=remaining)
                    if candidate.startswith(expected):
                        with self._order_lock:
                            self._awaiting_cmd_reply = False
                            if not defer_events:
                                self._ack_pending_udp = False
                        if not defer_events:
                            self.release_ack_and_flush_events()
                        return candidate
                    print(f"[UART STALE ACK DISCARDED] '{candidate}' (esperaba {expected})")
                except queue.Empty:
                    with self._order_lock:
                        self._awaiting_cmd_reply = False
                        self._ack_pending_udp = False
                    return "[ERROR] Timeout esperando respuesta del Arduino"

    def _cancel_sim_turn_timer(self):
        if self._sim_turn_timer is not None:
            self._sim_turn_timer.cancel()
            self._sim_turn_timer = None

    def _simulate_opcode(self, opcode_cmd: str) -> str:
        """1-to-1 simulation of the lean Arduino Uno opcode protocol."""
        parts = opcode_cmd.strip().split()
        op = parts[0].upper()

        if op == "X":
            self._cancel_sim_turn_timer()
            return "[ACK] X"
        if op == "?":
            return f"[STATUS] 0,0,{ARM_BASE_HOME},{ARM_ELBOW_HOME},0,1,850,900,0"
        if op == "A" and len(parts) == 3:
            return f"[ACK] A|{parts[1]}|{parts[2]}"
        if op == "W" and len(parts) == 3:
            self._cancel_sim_turn_timer()
            return f"[ACK] W|{parts[1]}|{parts[2]}"
        if op == "G" and len(parts) == 2:
            ddeg = int(parts[1])
            side = "R" if ddeg > 0 else "L"
            abs_ddeg = abs(ddeg)
            self._cancel_sim_turn_timer()
            self._sim_turn_timer = threading.Timer(
                0.35,
                lambda s=side, d=abs_ddeg: self._dispatch_or_defer_event(f"[EVENT] TD|{s}|{d}"),
            )
            self._sim_turn_timer.daemon = True
            self._sim_turn_timer.start()
            return f"[ACK] G|{side}|{abs_ddeg}"
        return "[ERROR] OP"

    def close(self):
        self._cancel_sim_turn_timer()
        self._stop_event.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        with self._write_lock:
            if self._ser is not None:
                try:
                    self._ser.write(b"X\n")
                    self._ser.flush()
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None


class RoverController:
    """
    High-level controller running on Raspberry Pi 5.
    Validates all human-readable commands locally, sends compact 1-letter opcodes (`W`, `G`, `A`, `X`, `?`)
    over UART to the Arduino Uno, and formats natural Spanish responses for the Laptop client.
    """
    STOP_ALIASES = frozenset({"STOP", "DETENER", "PARAR"})

    def __init__(
        self,
        serial_port: str = SERIAL_PORT,
        baud: int = BAUD_RATE,
        async_message_callback: Callable[[str], None] | None = None,
    ):
        self._async_callback = async_message_callback
        self._arduino = ArduinoSerialInterface(
            preferred_port=serial_port,
            baud=baud,
            event_callback=self._on_arduino_event,
        )

    def set_async_callback(self, callback: Callable[[str], None]):
        self._async_callback = callback

    def start(self) -> bool:
        return self._arduino.connect()

    def stop(self):
        self._arduino.close()

    def flush_deferred_events(self):
        self._arduino.release_ack_and_flush_events()

    @classmethod
    def is_emergency_stop(cls, raw_cmd: str) -> bool:
        tokens = raw_cmd.strip().upper().split()
        return len(tokens) == 1 and tokens[0] in cls.STOP_ALIASES

    @staticmethod
    def _is_int(tok: str) -> bool:
        return bool(re.fullmatch(r"[+-]?\d+", tok))

    @staticmethod
    def _is_float(tok: str) -> bool:
        return bool(re.fullmatch(r"[+-]?(?:\d+(?:\.\d+)?|\.\d+)", tok))

    @classmethod
    def _compile_to_opcode(cls, raw_cmd: str) -> tuple[str, str]:
        """
        Validates human commands on the Raspberry Pi 5 and compiles them into
        `(arduino_opcode, semantic_intent)`. Returns `("[ERROR] ...", "ERROR")` if invalid.
        """
        if len(raw_cmd.strip()) > 63:
            return ("[ERROR] Comando excede longitud maxima (63 caracteres)", "ERROR")

        clean = " ".join(raw_cmd.replace(",", " ").strip().split()).upper()
        if not clean:
            return ("", "EMPTY")

        tokens = clean.split(" ")
        verb = tokens[0].replace("_", "-")
        args = tokens[1:]

        alias_map = {
            "FORWARD":    "MOVE-FORWARD",
            "FWD":        "MOVE-FORWARD",
            "AVANZAR":    "MOVE-FORWARD",
            "BACKWARD":   "MOVE-BACKWARD",
            "BWD":        "MOVE-BACKWARD",
            "RETROCEDER": "MOVE-BACKWARD",
            "RIGHT":      "TURN-RIGHT",
            "LEFT":       "TURN-LEFT",
            "EXTEND":     "ARM-EXTEND",
            "DEPLOY":     "ARM-DEPLOYMENT",
            "RETRIEVE":   "ARM-RETRIEVE",
            "RETREIVE":   "ARM-RETREIVE",
            "HOME":       "ARM-HOME",
            "DETENER":    "STOP",
            "PARAR":      "STOP",
        }
        verb = alias_map.get(verb, verb)

        if verb == "MOVE" and args and args[0] in ("FORWARD", "BACKWARD"):
            verb, args = f"MOVE-{args[0]}", args[1:]
        elif verb == "TURN" and args and args[0] in ("RIGHT", "LEFT"):
            verb, args = f"TURN-{args[0]}", args[1:]
        elif verb == "ARM" and args and args[0] in ("EXTEND", "DEPLOYMENT", "RETRIEVE", "RETREIVE", "HOME"):
            verb, args = f"ARM-{args[0]}", args[1:]

        # 1. STOP
        if verb == "STOP":
            if len(args) == 0:
                return ("X", "STOP")
            return ("[ERROR] Sintaxis invalida. Usa: STOP", "ERROR")

        # 2. STATUS / READ
        if verb in ("STATUS", "READ"):
            if len(args) == 0:
                return ("?", "STATUS")
            return ("[ERROR] Sintaxis invalida. Usa: STATUS", "ERROR")

        # 3. ARM-EXTEND / ARM-DEPLOYMENT
        if verb in ("ARM-EXTEND", "ARM-DEPLOYMENT"):
            if len(args) == 0:
                return (f"A {ARM_BASE_EXTEND} {ARM_ELBOW_EXTEND}", "ARM_EXTEND")
            return (f"[ERROR] Sintaxis invalida. Usa: {verb}", "ERROR")

        # 4. ARM-RETRIEVE / ARM-RETREIVE / ARM-HOME
        if verb in ("ARM-RETRIEVE", "ARM-RETREIVE", "ARM-HOME"):
            if len(args) == 0:
                return (f"A {ARM_BASE_HOME} {ARM_ELBOW_HOME}", "ARM_HOME")
            return (f"[ERROR] Sintaxis invalida. Usa: {verb}", "ERROR")

        # 5. ARM <base 0-180> <elbow 0-180>
        if verb == "ARM":
            if len(args) == 2 and cls._is_int(args[0]) and cls._is_int(args[1]):
                b, e = int(args[0]), int(args[1])
                if 0 <= b <= 180 and 0 <= e <= 180:
                    return (f"A {b} {e}", "ARM_ANGLES")
            return ("[ERROR] Sintaxis invalida. Usa: ARM <base 0-180> <codo 0-180>", "ERROR")

        # 6. MOVE-FORWARD / MOVE-BACKWARD
        if verb in ("MOVE-FORWARD", "MOVE-BACKWARD"):
            is_fwd = verb == "MOVE-FORWARD"
            spd = 50
            if len(args) == 1:
                if not cls._is_int(args[0]):
                    return (f"[ERROR] Velocidad invalida. Usa: {verb} <0-100>", "ERROR")
                spd = int(args[0])
                if not (0 <= spd <= 100):
                    return (f"[ERROR] Velocidad invalida. Usa: {verb} <0-100>", "ERROR")
            elif len(args) > 1:
                return (f"[ERROR] Velocidad invalida. Usa: {verb} <0-100>", "ERROR")

            signed_spd = spd if is_fwd else -spd
            intent = "MOVE_FORWARD" if is_fwd else "MOVE_BACKWARD"
            return (f"W {signed_spd} {signed_spd}", intent)

        # 7. TURN-RIGHT / TURN-LEFT (Gyro angle OR Differential wheel speeds)
        if verb in ("TURN-RIGHT", "TURN-LEFT"):
            is_right = verb == "TURN-RIGHT"
            if len(args) == 0:
                ddeg = 200 if is_right else -200
                return (f"G {ddeg}", "TURN_GYRO")

            if len(args) == 1 and cls._is_float(args[0]):
                deg = float(args[0])
                if 0.0 < deg <= 360.0:
                    abs_ddeg = max(1, int(round(deg * 10.0)))
                    ddeg = abs_ddeg if is_right else -abs_ddeg
                    return (f"G {ddeg}", "TURN_GYRO")
                return (f"[ERROR] Sintaxis invalida. Usa: {verb} <1-360> o {verb} <izq> <der>", "ERROR")

            if len(args) == 2 and cls._is_int(args[0]) and cls._is_int(args[1]):
                v1, v2 = int(args[0]), int(args[1])
                if -100 <= v1 <= 100 and -100 <= v2 <= 100:
                    intent = "TURN_DIFF_RIGHT" if is_right else "TURN_DIFF_LEFT"
                    return (f"W {v1} {v2}", intent)

            return (f"[ERROR] Sintaxis invalida. Usa: {verb} <1-360> o {verb} <izq> <der>", "ERROR")

        return (f"[ERROR] Comando desconocido: {clean}", "ERROR")

    def execute_command(self, raw_cmd: str, defer_events: bool = False) -> str:
        """Compiles human command to a compact opcode, executes on Arduino, and formats Spanish feedback."""
        opcode, intent = self._compile_to_opcode(raw_cmd)
        if intent == "EMPTY":
            return "Comando vacío."
        if intent == "ERROR":
            return opcode

        uart_reply = self._arduino.send_opcode(opcode, defer_events=defer_events)
        return self._format_reply_to_spanish(uart_reply, intent)

    @staticmethod
    def _fmt_ddeg(ddeg_val: int) -> str:
        if ddeg_val % 10 == 0:
            return str(ddeg_val // 10)
        return f"{ddeg_val / 10.0:.1f}"

    def _format_reply_to_spanish(self, uart_reply: str, intent: str) -> str:
        """Converts compact integer Arduino replies into descriptive Spanish messages."""
        if uart_reply.startswith("[ACK]"):
            body = uart_reply[5:].strip()
            parts = body.split("|")
            op = parts[0]

            if op == "X":
                return "Me detuve (ruedas sin PWM y brazo detenido)"

            if op == "BLK" and len(parts) >= 2:
                cm = int(parts[1]) / 10.0
                return f"Avance bloqueado por obstáculo a {cm:.1f} cm: retrocediendo automáticamente..."

            if op == "W" and len(parts) >= 3:
                l_spd, r_spd = int(parts[1]), int(parts[2])
                if l_spd == 0 and r_spd == 0:
                    return "Me detuve (velocidad 0%)"
                if intent == "MOVE_FORWARD":
                    mps = RoverKinematics.speed_pct_to_mps(l_spd)
                    return f"Avancé a {l_spd}% de velocidad ({mps:.2f} m/s con ruedas de 87.5 mm)"
                if intent == "MOVE_BACKWARD":
                    spd = abs(l_spd)
                    mps = RoverKinematics.speed_pct_to_mps(spd)
                    return f"Retrocedí a {spd}% de velocidad ({mps:.2f} m/s con ruedas de 87.5 mm)"
                direction = "la derecha" if intent == "TURN_DIFF_RIGHT" else "la izquierda"
                return f"Girando a {direction} con velocidad diferencial (Izq: {l_spd}%, Der: {r_spd}%)"

            if op == "G" and len(parts) >= 3:
                direction = "la derecha" if parts[1] == "R" else "la izquierda"
                deg_str = self._fmt_ddeg(int(parts[2]))
                return f"Girando {deg_str} grados a {direction} (controlado por giroscopio MPU-6050)..."

            if op == "A" and len(parts) >= 3:
                b, e = parts[1], parts[2]
                if intent == "ARM_EXTEND":
                    return f"Extendí el brazo suavemente hacia (base={b}°, codo={e}°)"
                if intent == "ARM_HOME":
                    return f"Guardé el brazo suavemente hacia home (base={b}°, codo={e}°)"
                return f"Moví el brazo suavemente hacia (base={b}°, codo={e}°)"

            return f"Acción ejecutada: {body}"

        if uart_reply.startswith("[STATUS]"):
            raw_vals = uart_reply[8:].strip().split(",")
            if len(raw_vals) == 9:
                try:
                    l_s, r_s, arm_b, arm_e, yaw_dd, imu_ok, us1_mm, us2_mm, _ = [
                        int(v.strip()) for v in raw_vals
                    ]
                    imu_str = "OK" if imu_ok == 1 else "Simulado/Fallback"
                    return (
                        f"Estado Rover -> Ruedas(Izq={l_s}%, Der={r_s}%) | "
                        f"Brazo(Base={arm_b}°, Codo={arm_e}°) | "
                        f"IMU Yaw={yaw_dd / 10.0:.1f}° ({imu_str}) | "
                        f"US1={us1_mm / 10.0:.1f} cm, US2={us2_mm / 10.0:.1f} cm"
                    )
                except ValueError:
                    pass
            return uart_reply

        return uart_reply

    def _on_arduino_event(self, event_line: str):
        """Translates compact integer [EVENT] frames (TD, OBS, CLR) into Spanish notifications."""
        body = event_line.replace("[EVENT]", "", 1).strip()
        parts = body.split("|")
        tag = parts[0]

        spanish_msg = body
        if tag == "TD" and len(parts) >= 3:
            direction = "la derecha" if parts[1] == "R" else "la izquierda"
            try:
                deg_str = self._fmt_ddeg(int(parts[2]))
            except ValueError:
                deg_str = parts[2]
            spanish_msg = f"Giré {deg_str} grados a {direction} y me detuve"

        elif tag == "OBS" and len(parts) >= 2:
            try:
                cm_str = f"{int(parts[1]) / 10.0:.1f}"
            except ValueError:
                cm_str = parts[1]
            spanish_msg = (
                f"Obstáculo detectado a {cm_str} cm por ultrasónico: "
                f"retrocediendo automáticamente..."
            )

        elif tag == "CLR" and len(parts) >= 3:
            try:
                u1 = f"{int(parts[1]) / 10.0:.1f}"
                u2 = f"{int(parts[2]) / 10.0:.1f}"
            except ValueError:
                u1, u2 = parts[1], parts[2]
            spanish_msg = (
                f"Me detuve tras retroceder y librar el obstáculo "
                f"(US1={u1} cm, US2={u2} cm)"
            )

        if self._async_callback is not None:
            self._async_callback(spanish_msg)
