"""
laptop/client.py — Object-Oriented Dual-Terminal UDP Client for Rover Control (UA62 - Avance 2).

Architecture:
  - Terminal 1 (Command CLI)      : Sends actuator & mobility commands to the Raspberry Pi 5
                                    UDP server (MOVE-FORWARD 50, TURN-RIGHT 20, TURN-RIGHT 80 20,
                                    ARM-EXTEND, ARM-HOME, STOP, STATUS, etc.).
  - Terminal 2 (Response Monitor) : Automatically opened by client.py in a separate console window
                                    (--monitor mode) using an OS-assigned conflict-free ephemeral
                                    localhost UDP port and deterministic __MONITOR_READY__ handshake.
"""
import argparse
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

SERVER_IP    = "172.20.20.3"
SERVER_PORT  = 2222
MONITOR_PORT = 0  # 0 = conflict-free OS-assigned ephemeral UDP port
BUF_SIZE     = 2048


def _disable_windows_udp_connreset(sock: socket.socket):
    """Prevents Windows Winsock ICMP Port-Unreachable (WinError 10054) from dropping UDP reads."""
    if os.name == "nt" and hasattr(socket, "SIO_UDP_CONNRESET"):
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass


class MonitorTerminalLauncher:
    """
    Launches a second terminal window running `client.py --monitor` so that
    Raspberry Pi responses and asynchronous sensor/IMU events are displayed
    in a dedicated terminal window without port collisions.
    """
    @staticmethod
    def launch(monitor_port: int = 0, ack_port: int = 0) -> subprocess.Popen | None:
        script_path = str(Path(__file__).resolve())
        cmd = [
            sys.executable,
            script_path,
            "--monitor",
            "--monitor-port", str(monitor_port),
            "--ack-port",     str(ack_port),
        ]

        try:
            if os.name == "nt":
                return subprocess.Popen(
                    cmd,
                    creationflags=subprocess.CREATE_NEW_CONSOLE,
                )

            if sys.platform == "darwin":
                escaped = " ".join(f'\\"{arg}\\"' for arg in cmd)
                osa = f'tell application "Terminal" to do script "{escaped}"'
                return subprocess.Popen(["osascript", "-e", osa])

            for term in ("gnome-terminal", "konsole", "xfce4-terminal", "x-terminal-emulator", "xterm"):
                if shutil.which(term):
                    if term == "gnome-terminal":
                        return subprocess.Popen([term, "--"] + cmd)
                    return subprocess.Popen([term, "-e"] + cmd)
        except Exception as e:
            print(f"[WARN] No se pudo abrir la segunda terminal automáticamente ({e}).")

        return None


class ResponseMonitorTerminal:
    """
    Runs inside the second terminal window opened by MonitorTerminalLauncher.
    Binds an exclusive localhost UDP port (no SO_REUSEADDR port hijacking on Windows),
    reports its actual bound port back to Terminal 1 via `__MONITOR_READY__|<port>`,
    and displays every response and asynchronous event from the Raspberry Pi 5.
    """
    def __init__(self, port: int = 0, ack_port: int = 0):
        self.ack_port = ack_port
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _disable_windows_udp_connreset(self._sock)
        self._sock.bind(("127.0.0.1", port))
        self.port = self._sock.getsockname()[1]
        self._sock.settimeout(0.5)

    def run(self):
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if os.name == "nt":
            os.system("chcp 65001 >nul & title Terminal de Respuestas del Rover (Raspberry Pi 5)")

        # Send deterministic handshake with actual bound port back to Terminal 1
        if self.ack_port > 0:
            handshake = f"__MONITOR_READY__|{self.port}".encode("utf-8")
            try:
                self._sock.sendto(handshake, ("127.0.0.1", self.ack_port))
            except OSError:
                pass

        print("=" * 72)
        print("  TERMINAL DE RESPUESTAS DEL ROVER — RASPBERRY PI 5 (UA62 - Avance 2)")
        print(f"  Canal local IPC activo en 127.0.0.1:{self.port}")
        print("=" * 72)
        print("  Esperando respuestas y eventos del Rover...\n")

        try:
            while True:
                try:
                    data, _ = self._sock.recvfrom(BUF_SIZE)
                    msg = data.decode("utf-8", errors="replace").strip()
                    if not msg:
                        continue
                    if msg == "__CLOSE_MONITOR__":
                        print("\n[INFO] Cliente de comandos cerrado. Saliendo de la terminal de respuestas...")
                        time.sleep(0.8)
                        break

                    ts = datetime.now().strftime("%H:%M:%S")
                    print(f"  [{ts}] ➔  {msg}")
                except (socket.timeout, ConnectionResetError):
                    continue
        except KeyboardInterrupt:
            print("\n[INFO] Monitor finalizado.")
        finally:
            self._sock.close()


class RoverControlClient:
    """
    Object-oriented UDP client for the Laptop Command Terminal.
    Uses two isolated sockets:
      - `self._net_sock` : Communicates exclusively with Raspberry Pi 5 over WiFi/UDP.
      - `self._ipc_sock` : Forwards incoming messages exclusively to Terminal 2 on 127.0.0.1.
    """
    def __init__(
        self,
        server_ip: str = SERVER_IP,
        server_port: int = SERVER_PORT,
        monitor_port: int = MONITOR_PORT,
        spawn_monitor: bool = True,
    ):
        resolved_ip = self._resolve_host(server_ip)
        self._server_addr   = (resolved_ip, server_port)
        self._monitor_addr: tuple[str, int] | None = (
            ("127.0.0.1", monitor_port) if monitor_port > 0 else None
        )
        self._req_monitor_port = monitor_port
        self._spawn_monitor    = spawn_monitor

        self._net_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _disable_windows_udp_connreset(self._net_sock)
        self._net_sock.settimeout(0.3)

        self._ipc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _disable_windows_udp_connreset(self._ipc_sock)
        self._ipc_sock.bind(("127.0.0.1", 0))
        self._ipc_sock.settimeout(0.2)

        self._stop = threading.Event()
        self._got_first_reply = threading.Event()
        self._monitor_proc: subprocess.Popen | None = None
        self._rx_thread = threading.Thread(
            target=self._udp_receiver_worker,
            daemon=True,
            name="UDPResponseForwarder",
        )

    @staticmethod
    def _resolve_host(host: str) -> str:
        try:
            return socket.gethostbyname(host)
        except socket.gaierror:
            print(f"[WARN] No se pudo resolver '{host}'; se intentará usar literalmente.")
            return host

    def _is_monitor_alive(self) -> bool:
        return (
            self._monitor_proc is not None
            and self._monitor_proc.poll() is None
            and self._monitor_addr is not None
        )

    def _wait_for_monitor_ready(self, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._monitor_proc is None or self._monitor_proc.poll() is not None:
                return
            try:
                data, _ = self._ipc_sock.recvfrom(256)
                text = data.decode("utf-8", errors="replace").strip()
                if text.startswith("__MONITOR_READY__|"):
                    bound_port = int(text.split("|", 1)[1])
                    self._monitor_addr = ("127.0.0.1", bound_port)
                    return
            except (socket.timeout, ConnectionResetError, ValueError):
                continue

    def _deliver_to_monitor(self, msg: str):
        if self._is_monitor_alive() and self._monitor_addr is not None:
            try:
                self._ipc_sock.sendto(msg.encode("utf-8"), self._monitor_addr)
                return
            except OSError:
                pass
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"\n  [{ts} RPi] ➔  {msg}")

    def start(self):
        if self._spawn_monitor:
            ack_port = self._ipc_sock.getsockname()[1]
            self._monitor_proc = MonitorTerminalLauncher.launch(
                monitor_port=self._req_monitor_port,
                ack_port=ack_port,
            )
            if self._monitor_proc is not None and self._monitor_proc.poll() is None:
                self._wait_for_monitor_ready(timeout=3.0)

        self._rx_thread.start()

        # Register client address with Raspberry Pi server and check initial reachability
        try:
            self._net_sock.sendto(b"PING", self._server_addr)
            if not self._got_first_reply.wait(timeout=0.9):
                self._deliver_to_monitor(
                    f"[AVISO] Sin respuesta inicial de {self._server_addr[0]}:{self._server_addr[1]} "
                    f"(verifica IP o que main.py esté corriendo en la Raspberry Pi)"
                )
        except OSError as e:
            self._deliver_to_monitor(f"[AVISO RED] No se pudo enviar PING inicial: {e}")

    def _udp_receiver_worker(self):
        """
        Continuously receives UDP datagrams from the Raspberry Pi 5 (both synchronous
        command replies and asynchronous IMU/Ultrasonic events) and forwards them to
        the Response Monitor Terminal (with automatic inline fallback if Terminal 2 was closed).
        """
        while not self._stop.is_set():
            try:
                data, addr = self._net_sock.recvfrom(BUF_SIZE)
                if addr[1] != self._server_addr[1]:
                    continue
                msg = data.decode("utf-8", errors="replace").strip()
                if not msg:
                    continue

                self._got_first_reply.set()
                self._deliver_to_monitor(msg)
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                if not self._stop.is_set():
                    time.sleep(0.05)

    def send_command(self, cmd: str):
        clean = cmd.strip()
        if not clean:
            return
        if len(clean) > 63:
            print("  [ERR] El comando excede el límite de 63 caracteres del buffer UART.")
            return
        try:
            self._net_sock.sendto(clean.encode("utf-8"), self._server_addr)
            print(f"  [TX -> RPi] {clean}")
        except OSError as e:
            print(f"  [ERR UDP] No se pudo enviar el comando: {e}")

    def close(self):
        self._stop.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.8)
        if self._is_monitor_alive() and self._monitor_addr is not None:
            try:
                self._ipc_sock.sendto(b"__CLOSE_MONITOR__", self._monitor_addr)
            except OSError:
                pass
        for s in (self._net_sock, self._ipc_sock):
            try:
                s.close()
            except Exception:
                pass

    @staticmethod
    def _print_help():
        print("=" * 74)
        print("  TERMINAL DE COMANDOS DEL ROVER — CLIENTE LAPTOP (UA62 - Avance 2)")
        print("-" * 74)
        print("  Comandos de Movilidad (4x MG996R Continuos, Ruedas 87.5 mm):")
        print("    MOVE-FORWARD <0-100>          Ej: MOVE-FORWARD 50")
        print("    MOVE-BACKWARD <0-100>         Ej: MOVE-BACKWARD 50")
        print("    TURN-RIGHT <grados_IMU>       Ej: TURN-RIGHT 20      (Giro exacto con MPU-6050)")
        print("    TURN-LEFT <grados_IMU>        Ej: TURN-LEFT 20       (Giro exacto con MPU-6050)")
        print("    TURN-RIGHT <vel_izq> <vel_der> Ej: TURN-RIGHT 80 20  (Giro diferencial)")
        print("    TURN-LEFT <vel_izq> <vel_der>  Ej: TURN-LEFT 20 80   (Giro diferencial)")
        print("    STOP                          Detiene todas las ruedas y servos")
        print("-" * 74)
        print("  Comandos del Brazo Robótico (2 GDL - 2x MG996R Estándar):")
        print("    ARM-EXTEND  (o ARM-DEPLOYMENT) Extiende suavemente el brazo robótico")
        print("    ARM-RETRIEVE (o ARM-HOME)      Regresa suavemente el brazo a posición home")
        print("    ARM <base 0-180> <codo 0-180>  Ej: ARM 90 120")
        print("-" * 74)
        print("  Sensores y Sistema:")
        print("    STATUS                        Consulta IMU MPU-6050 y Ultrasónicos HC-SR04")
        print("    HELP                          Muestra esta ayuda")
        print("    QUIT (o q)                    Detiene el rover y cierra el cliente")
        print("=" * 74)

    def run_cli(self):
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if os.name == "nt":
            os.system("chcp 65001 >nul & title Terminal de Comandos del Rover (Cliente Laptop)")

        self.start()
        self._print_help()
        print(f"  Conectado hacia Raspberry Pi -> {self._server_addr[0]}:{self._server_addr[1]}")
        print("  (Las respuestas de la Raspberry Pi se muestran en la 2da terminal)\n")

        try:
            while True:
                entry = input("[COMANDO ROVER] > ").strip()
                if not entry:
                    continue

                low = entry.lower()
                if low in ("q", "quit", "exit"):
                    print("  [INFO] Enviando STOP de seguridad y cerrando...")
                    self.send_command("STOP")
                    time.sleep(0.2)
                    break
                if low in ("help", "h", "?"):
                    self._print_help()
                    continue

                self.send_command(entry)
        except KeyboardInterrupt:
            print("\n  [INFO] Interrupción detectada. Enviando STOP...")
            self.send_command("STOP")
            time.sleep(0.2)
        finally:
            self.close()


def main():
    parser = argparse.ArgumentParser(
        description="Cliente UDP de Doble Terminal para Control de Rover (UA62 - Avance 2)"
    )
    parser.add_argument("--ip",           default=SERVER_IP,   help=f"IP de la Raspberry Pi 5 (default: {SERVER_IP})")
    parser.add_argument("--port",         type=int, default=SERVER_PORT,  help=f"Puerto UDP servidor (default: {SERVER_PORT})")
    parser.add_argument("--monitor",      action="store_true", help="Ejecuta esta instancia como la Terminal 2 (Monitor de Respuestas)")
    parser.add_argument("--monitor-port", type=int, default=MONITOR_PORT, help="Puerto local del monitor (0 = automático libre)")
    parser.add_argument("--ack-port",     type=int, default=0,            help=argparse.SUPPRESS)
    parser.add_argument("--no-spawn",     action="store_true", help="No abrir automáticamente la segunda ventana de terminal")
    args = parser.parse_args()

    if args.monitor:
        monitor = ResponseMonitorTerminal(port=args.monitor_port, ack_port=args.ack_port)
        monitor.run()
    else:
        client = RoverControlClient(
            server_ip=args.ip,
            server_port=args.port,
            monitor_port=args.monitor_port,
            spawn_monitor=not args.no_spawn,
        )
        client.run_cli()


if __name__ == "__main__":
    main()
