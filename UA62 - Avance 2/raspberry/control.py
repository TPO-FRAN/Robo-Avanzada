#!/usr/bin/env python3
"""
raspberry/control.py — Object-Oriented UART Controller for Rover Actuators & Sensors (UA62 - Avance 2).

Hardware controlled via Arduino Uno over UART:
  - 4x Modified MG996R Continuous Rotation Servos (Wheels Ø = 87.5 mm)
  - 2x Standard MG996R Servos (2-DOF Robotic Arm)
  - 1x MPU-6050 IMU (Gyroscope Z-axis closed-loop angle turns)
  - 2x HC-SR04 Ultrasonic Sensors (FIFO+Median N=5 & automatic reverse on obstacle)

Classes:
  - RoverKinematics        : Computes linear speed (m/s) for 87.5 mm wheels.
  - RoverStatus            : Immutable value object holding full rover telemetry.
  - ArduinoSerialInterface : Thread-safe UART interface with asynchronous [EVENT] listener.
  - RoverController        : High-level actuator & sensor controller translating commands
                             and hardware events into human-readable Spanish feedback.
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
    arm_base_deg: int = 90
    arm_elbow_deg: int = 25
    yaw_deg: float = 0.0
    imu_connected: bool = False
    us1_cm: float = 400.0
    us2_cm: float = 400.0
    mode: int = 0


class ArduinoSerialInterface:
    """
    Encapsulates UART communication with the Arduino Uno actuator/sensor node.
    Uses a background reader thread so asynchronous hardware events ([EVENT] TURN_DONE,
    [EVENT] OBSTACLE) are dispatched immediately without blocking synchronous [ACK] replies.
    """
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
            self._ser = serial.Serial(active, self.baud, timeout=0.05)
            time.sleep(2.0)  # Wait for Arduino bootloader reset
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

    def _uart_reader_worker(self):
        """Continuously reads lines from UART and separates [EVENT] from [ACK]/[STATUS]."""
        while not self._stop_event.is_set():
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.1)
                continue
            try:
                if self._ser.in_waiting:
                    raw = self._ser.readline()
                    line = raw.decode(errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("[READY]"):
                        print(f"[ARDUINO] {line}")
                    elif line.startswith("[EVENT]"):
                        print(f"[UART EVENT] {line}")
                        if self.event_callback is not None:
                            self.event_callback(line)
                    else:
                        self._ack_queue.put(line)
                else:
                    time.sleep(0.005)
            except (serial.SerialException, OSError) as e:
                if not self._stop_event.is_set():
                    print(f"[UART READ ERR] {e}")
                time.sleep(0.1)

    def send_command(self, cmd: str, timeout: float = 1.5) -> str:
        """Sends a command line over UART and waits for the corresponding [ACK]/[STATUS]/[ERROR]."""
        if not self.is_connected:
            return self._simulate_response(cmd)

        with self._write_lock:
            # Drain stale ACKs before transmitting
            while not self._ack_queue.empty():
                try:
                    self._ack_queue.get_nowait()
                except queue.Empty:
                    break

            try:
                payload = f"{cmd.strip()}\n".encode("utf-8")
                self._ser.write(payload)
                self._ser.flush()
            except (serial.SerialException, OSError) as e:
                return f"[ERROR] Fallo de escritura UART: {e}"

            try:
                return self._ack_queue.get(timeout=timeout)
            except queue.Empty:
                return "[ERROR] Timeout esperando respuesta del Arduino"

    def _simulate_response(self, cmd: str) -> str:
        """Hardware-free simulation fallback when testing without Arduino attached."""
        up = cmd.strip().upper()
        if up == "STOP":
            return "[ACK] STOP"
        if up in ("STATUS", "READ"):
            return "[STATUS] l=0,r=0,arm=90:25,yaw=0.0,imu=1,us1=85.0,us2=90.0,mode=0"
        if up in ("ARM-EXTEND", "ARM-DEPLOYMENT"):
            return "[ACK] ARM_EXTEND|90|145"
        if up in ("ARM-RETRIEVE", "ARM-RETREIVE", "ARM-HOME"):
            return "[ACK] ARM_HOME|90|25"
        if up.startswith("ARM "):
            parts = up.split()
            if len(parts) == 3 and parts[1].lstrip("-").isdigit() and parts[2].lstrip("-").isdigit():
                b = max(0, min(180, int(parts[1])))
                e = max(0, min(180, int(parts[2])))
                return f"[ACK] ARM_ANGLES|{b}|{e}"
            return "[ERROR] Syntax: ARM <base 0-180> <elbow 0-180>"
        if up.startswith("MOVE-FORWARD"):
            parts = up.split()
            spd = int(parts[1]) if len(parts) > 1 and parts[1].lstrip("-").isdigit() else 50
            spd = max(0, min(100, spd))
            return f"[ACK] MOVE_FORWARD|{spd}"
        if up.startswith("MOVE-BACKWARD"):
            parts = up.split()
            spd = int(parts[1]) if len(parts) > 1 and parts[1].lstrip("-").isdigit() else 50
            spd = max(0, min(100, spd))
            return f"[ACK] MOVE_BACKWARD|{spd}"
        if up.startswith("TURN-RIGHT") or up.startswith("TURN-LEFT"):
            is_right = up.startswith("TURN-RIGHT")
            dir_tag = "RIGHT" if is_right else "LEFT"
            parts = up.split()
            if len(parts) == 3 and parts[1].lstrip("-").isdigit() and parts[2].lstrip("-").isdigit():
                return f"[ACK] TURN_DIFF|{dir_tag}|{parts[1]}|{parts[2]}"
            deg = float(parts[1]) if len(parts) >= 2 else 20.0
            # Trigger simulated async gyro completion event after 0.4s
            if self.event_callback is not None:
                threading.Timer(
                    0.4,
                    lambda: self.event_callback(f"[EVENT] TURN_DONE|{dir_tag}|{deg:.1f}"),
                ).start()
            return f"[ACK] TURN_GYRO_START|{dir_tag}|{deg:.1f}|55"
        return f"[ERROR] Unknown command: {up}"

    def close(self):
        self._stop_event.set()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        with self._write_lock:
            if self._ser is not None:
                try:
                    self._ser.write(b"STOP\n")
                    self._ser.flush()
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None


class RoverController:
    """
    High-level controller for the Rover's 4 continuous wheels (87.5 mm), 2-DOF arm,
    MPU-6050 IMU, and 2x HC-SR04 ultrasonic sensors.
    Translates raw UART responses and asynchronous events into natural Spanish messages.
    """
    _STATUS_RE = re.compile(
        r"l=(?P<l>-?\d+),r=(?P<r>-?\d+),arm=(?P<b>\d+):(?P<e>\d+),"
        r"yaw=(?P<yaw>-?\d+(?:\.\d+)?),imu=(?P<imu>\d+),"
        r"us1=(?P<us1>-?\d+(?:\.\d+)?),us2=(?P<us2>-?\d+(?:\.\d+)?),mode=(?P<m>\d+)"
    )

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
        """Registers a callback (used by server.py) to push async hardware events to the client."""
        self._async_callback = callback

    def start(self) -> bool:
        return self._arduino.connect()

    def stop(self):
        self._arduino.close()

    # ── Command Execution & Translation ──────────────────────────────────────
    def execute_command(self, raw_cmd: str) -> str:
        """
        Normalizes user commands, dispatches them over UART to the Arduino,
        and returns a natural-language Spanish response for the Laptop monitor terminal.
        """
        cmd = self._normalize_command(raw_cmd)
        if not cmd:
            return "Comando vacío."

        if cmd.startswith("[ERROR]"):
            return cmd

        uart_reply = self._arduino.send_command(cmd)
        return self._format_ack_to_spanish(uart_reply)

    @staticmethod
    def _normalize_command(raw_cmd: str) -> str:
        """Supports both slide syntax (MOVE-FORWARD 50, TURN-RIGHT 80 20) and flexible aliases."""
        clean = " ".join(raw_cmd.strip().split()).upper()
        if not clean:
            return ""

        # Replace underscores with hyphens in command verb
        tokens = clean.split(" ")
        verb = tokens[0].replace("_", "-")
        args = tokens[1:]

        # Map convenient shorthand aliases to canonical firmware commands
        alias_map = {
            "FORWARD": "MOVE-FORWARD",
            "FWD": "MOVE-FORWARD",
            "AVANZAR": "MOVE-FORWARD",
            "BACKWARD": "MOVE-BACKWARD",
            "BWD": "MOVE-BACKWARD",
            "RETROCEDER": "MOVE-BACKWARD",
            "RIGHT": "TURN-RIGHT",
            "LEFT": "TURN-LEFT",
            "EXTEND": "ARM-EXTEND",
            "DEPLOY": "ARM-DEPLOYMENT",
            "RETRIEVE": "ARM-RETRIEVE",
            "RETREIVE": "ARM-RETREIVE",
            "HOME": "ARM-HOME",
            "DETENER": "STOP",
            "PARAR": "STOP",
        }
        verb = alias_map.get(verb, verb)

        # Handle "TURN RIGHT ..." or "MOVE FORWARD ..." written with a space instead of hyphen
        if verb == "MOVE" and args and args[0] in ("FORWARD", "BACKWARD"):
            verb = f"MOVE-{args[0]}"
            args = args[1:]
        elif verb == "TURN" and args and args[0] in ("RIGHT", "LEFT"):
            verb = f"TURN-{args[0]}"
            args = args[1:]
        elif verb == "ARM" and args and args[0] in ("EXTEND", "DEPLOYMENT", "RETRIEVE", "RETREIVE", "HOME"):
            verb = f"ARM-{args[0]}"
            args = args[1:]

        return " ".join([verb] + args)

    def _format_ack_to_spanish(self, uart_reply: str) -> str:
        """Converts Arduino [ACK] / [STATUS] frames into descriptive Spanish messages."""
        if uart_reply.startswith("[ACK]"):
            body = uart_reply[5:].strip()
            parts = body.split("|")
            tag = parts[0]

            if tag == "STOP":
                return "Me detuve (ruedas y servomotores en paro)"

            if tag == "MOVE_FORWARD" and len(parts) >= 2:
                spd = int(parts[1])
                mps = RoverKinematics.speed_pct_to_mps(spd)
                if spd == 0:
                    return "Me detuve (velocidad 0%)"
                return f"Avancé a {spd}% de velocidad ({mps:.2f} m/s con ruedas de 87.5 mm)"

            if tag == "MOVE_BACKWARD" and len(parts) >= 2:
                spd = int(parts[1])
                mps = RoverKinematics.speed_pct_to_mps(spd)
                if spd == 0:
                    return "Me detuve (velocidad 0%)"
                return f"Retrocedí a {spd}% de velocidad ({mps:.2f} m/s con ruedas de 87.5 mm)"

            if tag == "TURN_DIFF" and len(parts) >= 4:
                direction = "la derecha" if parts[1] == "RIGHT" else "la izquierda"
                l_spd, r_spd = parts[2], parts[3]
                return f"Girando a {direction} con velocidad diferencial (Izq: {l_spd}%, Der: {r_spd}%)"

            if tag == "TURN_GYRO_START" and len(parts) >= 3:
                direction = "la derecha" if parts[1] == "RIGHT" else "la izquierda"
                deg = float(parts[2])
                deg_str = f"{int(deg)}" if deg.is_integer() else f"{deg:.1f}"
                return f"Girando {deg_str} grados a {direction} (controlado por giroscopio MPU-6050)..."

            if tag == "ARM_EXTEND" and len(parts) >= 3:
                return f"Extendí el brazo (base={parts[1]}°, codo={parts[2]}°)"

            if tag == "ARM_HOME" and len(parts) >= 3:
                return f"Guardé el brazo en posición home (base={parts[1]}°, codo={parts[2]}°)"

            if tag == "ARM_ANGLES" and len(parts) >= 3:
                return f"Moví el brazo a posición (base={parts[1]}°, codo={parts[2]}°)"

            return f"Acción ejecutada: {body}"

        if uart_reply.startswith("[STATUS]"):
            m = self._STATUS_RE.search(uart_reply)
            if m:
                imu_state = "OK" if m.group("imu") == "1" else "Simulado/Fallback"
                return (
                    f"Estado Rover -> Ruedas(Izq={m.group('l')}%, Der={m.group('r')}%) | "
                    f"Brazo(Base={m.group('b')}°, Codo={m.group('e')}°) | "
                    f"IMU Yaw={m.group('yaw')}° ({imu_state}) | "
                    f"US1={m.group('us1')} cm, US2={m.group('us2')} cm"
                )
            return uart_reply

        return uart_reply

    def _on_arduino_event(self, event_line: str):
        """Handles asynchronous [EVENT] lines from Arduino (Gyro turn completion & Ultrasonics)."""
        body = event_line.replace("[EVENT]", "", 1).strip()
        parts = body.split("|")
        tag = parts[0]

        spanish_msg = body
        if tag == "TURN_DONE" and len(parts) >= 3:
            direction = "la derecha" if parts[1] == "RIGHT" else "la izquierda"
            try:
                deg = float(parts[2])
                deg_str = f"{int(round(deg))}" if abs(deg - round(deg)) < 0.15 else f"{deg:.1f}"
            except ValueError:
                deg_str = parts[2]
            spanish_msg = f"Giré {deg_str} grados a {direction} y me detuve"

        elif tag == "OBSTACLE" and len(parts) >= 2:
            spanish_msg = (
                f"Obstáculo detectado a {parts[1]} cm por ultrasónico: "
                f"retrocediendo automáticamente..."
            )

        elif tag == "OBSTACLE_CLEARED" and len(parts) >= 3:
            spanish_msg = (
                f"Me detuve tras retroceder y librar el obstáculo "
                f"(US1={parts[1]} cm, US2={parts[2]} cm)"
            )

        if self._async_callback is not None:
            self._async_callback(spanish_msg)
