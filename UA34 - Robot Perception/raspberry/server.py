#!/usr/bin/env python3
"""
raspberry/server.py — Object-Oriented UDP↔UART Data Reception & Transmission Server.

Classes:
  - SensorReading          : Value object holding filtered LDR, LM35, US1, and US2 data.
  - ArduinoSerialInterface : Manages UART port discovery, connection, and [DATA] parsing.
  - UDPPerceptionServer    : Two-thread UDP server coordinating ArduinoSerialInterface
                             and USBWebCamera (imported from camera.py).
"""
import socket
import threading
import queue
import time
import argparse
import re
from dataclasses import dataclass
import serial
import serial.tools.list_ports

from camera import USBWebCamera

UDP_IP     = "0.0.0.0"
UDP_PORT   = 2222
SERIAL     = "/dev/ttyACM0"
BAUD       = 9600
BUF        = 1024
CHUNK_SIZE = 32768  # 32 KB safe UDP payload chunk size (< 65507 B limit)


@dataclass(frozen=True)
class SensorReading:
    """Immutable value object representing filtered sensor telemetry from Arduino."""
    ldr: str
    temp: str
    us1: str
    us2: str


class ArduinoSerialInterface:
    """
    Encapsulates UART communication with the Arduino Uno sensor node,
    including automatic port detection and strict [DATA] frame filtering.
    """
    _DATA_RE = re.compile(
        r"ldr=(?P<ldr>-?\d+),\s*temp=(?P<temp>-?\d+(?:\.\d+)?),\s*us1=(?P<us1>-?\d+(?:\.\d+)?),\s*us2=(?P<us2>-?\d+(?:\.\d+)?)",
        re.IGNORECASE,
    )

    def __init__(self, preferred_port: str = SERIAL, baud: int = BAUD, mock: bool = False):
        self.preferred_port = preferred_port
        self.baud           = baud
        self.mock           = mock
        self._ser           = None
        self._lock          = threading.Lock()

    @staticmethod
    def find_port(preferred_port: str) -> str:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if preferred_port in ports:
            return preferred_port

        for port in ports:
            if "ACM" in port or "USB" in port:
                print(f"[SERIAL] Preferred port '{preferred_port}' not found. Using detected: {port}")
                return port

        if ports:
            print(f"[SERIAL] Using first detected port: {ports[0]}")
            return ports[0]

        print(f"[SERIAL WARNING] No active serial ports detected. Trying default '{preferred_port}'.")
        return preferred_port

    def connect(self):
        if self.mock:
            print("[UART] Running in MOCK sensor mode.")
            return

        active = self.find_port(self.preferred_port)
        try:
            self._ser = serial.Serial(active, self.baud, timeout=0.5)
            time.sleep(2.0)  # let Arduino bootloader settle
            self._ser.reset_input_buffer()
            print(f"[UART] Connected to {active} @ {self.baud}")
        except serial.SerialException as e:
            print(f"[SERIAL WARNING] Could not open {active} ({e}). Falling back to mock sensor values.")
            self.mock = True
            self._ser = None

    def read_sensors(self, timeout: float = 1.5) -> SensorReading:
        """
        Sends 'READ\\n' to Arduino and waits specifically for a '[DATA] ...' line,
        ignoring any delayed startup banners like '[READY]'.
        """
        if self.mock or self._ser is None:
            return SensorReading(ldr="512", temp="24.50", us1="35.20", us2="41.80")

        with self._lock:
            try:
                self._ser.reset_input_buffer()
                self._ser.write(b"READ\n")
                self._ser.flush()

                deadline = time.time() + timeout
                while time.time() < deadline:
                    if self._ser.in_waiting:
                        line = self._ser.readline().decode(errors="replace").strip()
                        if not line:
                            continue
                        m = self._DATA_RE.search(line)
                        if m:
                            return SensorReading(
                                ldr=m.group("ldr").strip(),
                                temp=m.group("temp").strip(),
                                us1=m.group("us1").strip(),
                                us2=m.group("us2").strip(),
                            )
                    else:
                        time.sleep(0.005)
                print("[UART WARN] Timeout waiting for [DATA] frame from Arduino.")
            except serial.SerialException as e:
                print(f"[SERIAL ERROR] {e}")

        return SensorReading(ldr="0", temp="0.00", us1="0.00", us2="0.00")

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None


class UDPPerceptionServer:
    """
    Two-thread UDP server:
      - Thread-1 (WiFiListener)  : Receives UDP commands from Laptop Client
      - Thread-2 (UARTCommander) : Coordinates ArduinoSerialInterface & USBWebCamera
    """
    def __init__(self, udp_ip=UDP_IP, udp_port=UDP_PORT,
                 serial_port=SERIAL, baud=BAUD, camera_index=0, mock=False):
        self._q       = queue.Queue()
        self._stop    = threading.Event()
        self._sock    = self._open_udp(udp_ip, udp_port)
        self._arduino = ArduinoSerialInterface(serial_port, baud, mock=mock)
        self._camera  = USBWebCamera(camera_index=camera_index, mock=mock)

        self._t1 = threading.Thread(target=self._wifi_listener_worker,  daemon=True, name="WiFiListener")
        self._t2 = threading.Thread(target=self._uart_commander_worker, daemon=True, name="UARTCommander")

    def _open_udp(self, ip: str, port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((ip, port))
        s.settimeout(0.5)
        print(f"[UDP ] Bound to {ip}:{port}")
        return s

    # ── Thread workers ───────────────────────────────────────────────────────
    def _wifi_listener_worker(self):
        print(f"[{threading.current_thread().name}] Running...")
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(BUF)
                cmd = data.decode(errors="replace").strip()
                if cmd:
                    print(f"[UDP RX] '{cmd}' from {addr}")
                    self._q.put((cmd, addr))
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError as e:
                if not self._stop.is_set():
                    print(f"[UDP ERR] {e}")
        print(f"[{threading.current_thread().name}] Stopped.")

    def _uart_commander_worker(self):
        print(f"[{threading.current_thread().name}] Running...")
        while not self._stop.is_set():
            try:
                cmd, addr = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                up = cmd.upper()
                if up == "CAPTURE":
                    self._handle_capture(addr)
                elif up in ("READ", "STATUS"):
                    reading = self._arduino.read_sensors()
                    snap = self._camera.get_snapshot()
                    msg = (
                        f"[STATUS] cam={snap.resolution_str} ({snap.size_bytes} B), "
                        f"ldr={reading.ldr}, temp={reading.temp}, us1={reading.us1}, us2={reading.us2}"
                    )
                    self._sock.sendto(msg.encode(), addr)
                    print(f"[TX STATUS->UDP] '{msg}' -> {addr}")
                else:
                    err = f"[ERROR] Unknown command '{cmd}'. Use CAPTURE or STATUS."
                    self._sock.sendto(err.encode(), addr)
            except Exception as e:
                print(f"[DISPATCH ERR] {e}")
            finally:
                self._q.task_done()
        print(f"[{threading.current_thread().name}] Stopped.")

    def _handle_capture(self, addr):
        """
        Reads Arduino sensors + grabs camera snapshot, then sends:
          1. META|<ldr>|<temp>|<us1>|<us2>|<width>x<height>|<size_bytes>|<num_chunks>
          2. CHUNK|<idx>|<raw_bytes> for each chunk
        """
        reading = self._arduino.read_sensors()
        snap = self._camera.get_snapshot()

        chunks = [
            snap.jpeg_bytes[i : i + CHUNK_SIZE]
            for i in range(0, len(snap.jpeg_bytes), CHUNK_SIZE)
        ] or [b""]
        num_chunks = len(chunks)

        meta = (
            f"META|{reading.ldr}|{reading.temp}|{reading.us1}|{reading.us2}|"
            f"{snap.resolution_str}|{snap.size_bytes}|{num_chunks}"
        )
        self._sock.sendto(meta.encode(), addr)

        for idx, chunk in enumerate(chunks):
            header = f"CHUNK|{idx}|".encode()
            self._sock.sendto(header + chunk, addr)
            if num_chunks > 1:
                time.sleep(0.002)

        print(
            f"[TX CAPTURE->UDP] {snap.resolution_str} ({snap.size_bytes} B, {num_chunks} chunk(s)) | "
            f"LDR={reading.ldr} Temp={reading.temp} C US1={reading.us1} cm US2={reading.us2} cm -> {addr}"
        )

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def start(self):
        self._arduino.connect()
        self._camera.start()
        self._t1.start()
        self._t2.start()
        print("[SERVER] Communication & Camera objects active. Press Ctrl+C to stop.\n" + "-" * 60)

    def shutdown(self):
        print("\n[SERVER] Shutting down...")
        self._stop.set()
        self._t1.join(1.0)
        self._t2.join(1.0)
        self._camera.stop()
        self._arduino.close()
        try:
            self._sock.close()
        except Exception:
            pass
        print("[SERVER] Done.")


def main():
    p = argparse.ArgumentParser(description="Raspberry Pi 5 UDP↔UART Perception Server")
    p.add_argument("--udp-ip",      default=UDP_IP)
    p.add_argument("--udp-port",    type=int, default=UDP_PORT)
    p.add_argument("--serial-port", default=SERIAL)
    p.add_argument("--baudrate",    type=int, default=BAUD)
    p.add_argument("--camera",      type=int, default=0)
    p.add_argument("--mock",        action="store_true", help="Run with simulated Arduino & camera data")
    a = p.parse_args()

    server = UDPPerceptionServer(a.udp_ip, a.udp_port, a.serial_port, a.baudrate, a.camera, a.mock)
    server.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
