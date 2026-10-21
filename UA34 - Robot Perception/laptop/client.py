"""
laptop/client.py — Object-Oriented UDP Client for Rover Sensor & Camera Perception (UA34).

Classes:
  - CaptureSample    : Value object holding a synchronized sensor + image sample.
  - DatasetStorage   : Manages the 'Datos' directory, CSV logging, and JPEG saving.
  - KeyboardReader   : Cross-platform single-keystroke reader ('p', 's', 'q').
  - PerceptionClient : UDP client communicating with Raspberry Pi UDPPerceptionServer.
"""
import socket
import csv
import os
import sys
import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

SERVER_IP   = "192.168.1.119"
SERVER_PORT = 2222
BUF         = 65535
TIMEOUT     = 3.0

BASE_DIR  = Path(__file__).resolve().parent.parent
DATOS_DIR = BASE_DIR / "Datos"


@dataclass(frozen=True)
class CaptureSample:
    """Immutable value object representing a captured sample."""
    timestamp: str
    resolution: str
    size_bytes: int
    ldr: str
    temp: str
    us1: str
    us2: str
    jpeg_data: bytes

    @property
    def image_size_field(self) -> str:
        return f"{self.resolution} ({self.size_bytes} B)"

    @property
    def image_filename(self) -> str:
        return f"{self.timestamp}.jpg"

    def to_csv_row(self) -> list[str]:
        return [
            self.image_size_field,
            self.ldr,
            self.temp,
            self.us1,
            self.us2,
            self.timestamp,
        ]


class DatasetStorage:
    """
    Encapsulates all filesystem operations inside 'UA34 - Robot Perception/Datos/',
    ensuring atomic image writes and resilient CSV logging.
    """
    CSV_HEADERS = [
        "arm_camera_image_size",
        "light_level",
        "temperature",
        "ultrasonic_sensor1",
        "ultrasonic_sensor2",
        "timestamp",
    ]

    def __init__(self, datos_dir: Path = DATOS_DIR, csv_filename: str = "sensor_data.csv"):
        self.datos_dir = datos_dir
        self.csv_path  = self.datos_dir / csv_filename
        self._ensure_initialized()

    def _ensure_initialized(self):
        self.datos_dir.mkdir(parents=True, exist_ok=True)
        if not self.csv_path.exists() or self.csv_path.stat().st_size == 0:
            self._write_headers(self.csv_path)

    def _write_headers(self, target_path: Path):
        try:
            with open(target_path, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(self.CSV_HEADERS)
        except PermissionError:
            print(f"[STORAGE WARN] '{target_path.name}' is locked by another program (e.g. Excel).")

    def save(self, sample: CaptureSample) -> tuple[Path, Path]:
        """Saves the JPEG image and appends the telemetry row to the CSV file."""
        self.datos_dir.mkdir(parents=True, exist_ok=True)

        # 1. Save image as Datos/<timestamp>.jpg
        img_path = self.datos_dir / sample.image_filename
        with open(img_path, "wb") as img_file:
            img_file.write(sample.jpeg_data)

        # 2. Append row to Datos/sensor_data.csv (with fallback if locked by Excel)
        target_csv = self.csv_path
        try:
            if not target_csv.exists() or target_csv.stat().st_size == 0:
                self._write_headers(target_csv)
            with open(target_csv, mode="a", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(sample.to_csv_row())
        except PermissionError:
            target_csv = self.datos_dir / "sensor_data_backup.csv"
            print(f"[STORAGE WARN] '{self.csv_path.name}' is locked! Writing row to '{target_csv.name}'.")
            if not target_csv.exists() or target_csv.stat().st_size == 0:
                self._write_headers(target_csv)
            with open(target_csv, mode="a", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(sample.to_csv_row())

        return img_path, target_csv


class KeyboardReader:
    """Cross-platform single-keystroke reader without requiring Enter."""
    def read_key(self) -> str:
        if not sys.stdin.isatty():
            return input("\n[Press 'p' + Enter to capture, 's' for status, 'q' to quit] > ").strip()

        if os.name == "nt":
            import msvcrt
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):
                msvcrt.getwch()
                return ""
            if ch == "\x03":
                raise KeyboardInterrupt
            return ch
        else:
            import tty
            import termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                if ch == "\x03":
                    raise KeyboardInterrupt
                return ch
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class PerceptionClient:
    """
    Object-oriented UDP client that requests sensor + camera captures from the
    Raspberry Pi 5 server and persists them via DatasetStorage.
    """
    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, timeout=TIMEOUT,
                 storage: DatasetStorage | None = None):
        self._addr    = (ip, port)
        self._timeout = timeout
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self._timeout)
        self.storage  = storage or DatasetStorage()
        self.keyboard = KeyboardReader()

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass

    def _drain_socket(self):
        """Clears any stale UDP packets before issuing a new request."""
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recvfrom(BUF)
        except (BlockingIOError, socket.error):
            pass
        finally:
            self._sock.settimeout(self._timeout)

    # ── Public API ──────────────────────────────────────────────────────────
    def capture_sample(self) -> bool:
        """
        Sends 'CAPTURE' to the Raspberry Pi server, verifies chunk completeness,
        and persists both image and CSV record in Datos/.
        """
        self._drain_socket()
        try:
            self._sock.sendto(b"CAPTURE", self._addr)
            print(f"[TX] CAPTURE -> {self._addr[0]}:{self._addr[1]}")

            # 1. Receive META header:
            #    META|<ldr>|<temp>|<us1>|<us2>|<width>x<height>|<size_bytes>|<num_chunks>
            data, addr = self._sock.recvfrom(BUF)
            meta_str = data.decode(errors="replace").strip()
            if not meta_str.startswith("META|"):
                print(f"[ERR] Unexpected response from {addr[0]}: {meta_str}")
                return False

            parts = meta_str.split("|")
            if len(parts) != 8:
                print(f"[ERR] Malformed META header: {meta_str}")
                return False

            _, ldr, temp, us1, us2, resolution, size_bytes_str, num_chunks_str = parts
            expected_bytes = int(size_bytes_str)
            num_chunks     = int(num_chunks_str)

            # 2. Receive all binary JPEG chunks
            chunks: dict[int, bytes] = {}
            while len(chunks) < num_chunks:
                pkt, _ = self._sock.recvfrom(BUF)
                if not pkt.startswith(b"CHUNK|"):
                    continue
                sep_idx = pkt.find(b"|", 6)
                if sep_idx == -1:
                    continue
                chunk_idx = int(pkt[6:sep_idx].decode())
                chunks[chunk_idx] = pkt[sep_idx + 1 :]

            jpeg_data = b"".join(chunks[i] for i in range(num_chunks) if i in chunks)
            actual_bytes = len(jpeg_data)
            if actual_bytes != expected_bytes:
                print(f"[WARN] Byte count mismatch (expected {expected_bytes} B, got {actual_bytes} B).")

            # 3. Create CaptureSample object and persist via DatasetStorage
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")[:-3]
            sample = CaptureSample(
                timestamp=timestamp,
                resolution=resolution,
                size_bytes=actual_bytes,
                ldr=ldr,
                temp=temp,
                us1=us1,
                us2=us2,
                jpeg_data=jpeg_data,
            )
            img_path, csv_path = self.storage.save(sample)

            print(
                f"[SAVED] Timestamp : {sample.timestamp}\n"
                f"        Image     : Datos/{img_path.name} [{sample.image_size_field}]\n"
                f"        Sensors   : LDR={sample.ldr} | Temp={sample.temp} C | US1={sample.us1} cm | US2={sample.us2} cm\n"
                f"        CSV       : Datos/{csv_path.name}"
            )
            return True

        except socket.timeout:
            print("[WARN] No response from Raspberry Pi server (timeout).")
            return False
        except Exception as e:
            print(f"[ERR] Capture failed: {e}")
            return False

    def request_status(self):
        self._drain_socket()
        try:
            self._sock.sendto(b"STATUS", self._addr)
            data, addr = self._sock.recvfrom(BUF)
            print(f"[ACK from {addr[0]}] {data.decode(errors='replace').strip()}")
        except socket.timeout:
            print("[WARN] Status request timed out.")

    # ── Interactive CLI ──────────────────────────────────────────────────────
    def run_cli(self):
        print("=" * 68)
        print(f"Rover Perception Client -> {self._addr[0]}:{self._addr[1]}")
        print(f"Storage Folder          -> {self.storage.datos_dir}")
        print("-" * 68)
        print("Controls: Press [p] to capture sample (CSV + Image) | [s] status | [q] quit")
        print("=" * 68)
        try:
            while True:
                key = self.keyboard.read_key()
                if not key:
                    continue
                low = key.lower()
                if low == "q":
                    print("\n[INFO] Exiting client.")
                    break
                elif low == "p":
                    print("\n[KEY 'p'] Capturing sensor data and camera image...")
                    self.capture_sample()
                elif low == "s":
                    print("\n[KEY 's'] Requesting status...")
                    self.request_status()
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted.")
        finally:
            self.close()


def main():
    p = argparse.ArgumentParser(description="Laptop UDP Client for Rover Perception")
    p.add_argument("--ip",      default=SERVER_IP, help="Raspberry Pi IP address")
    p.add_argument("--port",    type=int, default=SERVER_PORT, help="UDP port (default: 2222)")
    p.add_argument("--timeout", type=float, default=TIMEOUT, help="Socket timeout in seconds")
    p.add_argument("--once",    action="store_true", help="Capture a single sample and exit")
    a = p.parse_args()

    client = PerceptionClient(a.ip, a.port, a.timeout)
    if a.once:
        try:
            ok = client.capture_sample()
            sys.exit(0 if ok else 1)
        finally:
            client.close()
    else:
        client.run_cli()


if __name__ == "__main__":
    main()
