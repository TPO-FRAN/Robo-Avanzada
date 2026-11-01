"""
laptop/client.py — Object-Oriented Dual-Terminal UDP Client for Rover Control (UA62 - Avance 2).

Architecture:
  - Terminal 1 (Command CLI)      : Sends actuator & mobility commands to the Raspberry Pi 5
                                    UDP server (MOVE-FORWARD 50, TURN-RIGHT 20, TURN-RIGHT 80 20,
                                    ARM-EXTEND, ARM-HOME, STOP, STATUS, etc.).
  - Terminal 2 (Response Monitor) : Automatically opened by client.py in a separate console window
                                    (--monitor mode) to display real-time Raspberry Pi responses:
                                    "Avancé a 50% de velocidad", "Giré 20 grados a la derecha",
                                    "Extendí el brazo", "Me detuve", etc.
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

SERVER_IP    = "172.31.62.173"
SERVER_PORT  = 2222
MONITOR_PORT = 2223
BUF_SIZE     = 2048


class MonitorTerminalLauncher:
    """
    Launches a second terminal window running `client.py --monitor` so that
    Raspberry Pi responses and asynchronous sensor/IMU events are displayed
    in a dedicated terminal window.
    """
    @staticmethod
    def launch(monitor_port: int = MONITOR_PORT) -> subprocess.Popen | None:
        script_path = str(Path(__file__).resolve())
        cmd = [sys.executable, script_path, "--monitor", "--monitor-port", str(monitor_port)]

        try:
            if os.name == "nt":
                # Opens a separate console window on Windows
                return subprocess.Popen(
                    cmd,
                    creationflags=subprocess.CREATE_NEW_CONSOLE,
                )

            if sys.platform == "darwin":
                escaped = " ".join(f'\\"{arg}\\"' for arg in cmd)
                osa = f'tell application "Terminal" to do script "{escaped}"'
                return subprocess.Popen(["osascript", "-e", osa])

            # Linux desktop terminals
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
    Displays every response and asynchronous event from the Raspberry Pi 5.
    """
    def __init__(self, port: int = MONITOR_PORT):
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", self.port))
        self._sock.settimeout(0.5)

    def run(self):
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if os.name == "nt":
            os.system("chcp 65001 >nul & title Terminal de Respuestas del Rover (Raspberry Pi 5)")
        print("=" * 72)
        print("  TERMINAL DE RESPUESTAS DEL ROVER — RASPBERRY PI 5 (UA62 - Avance 2)")
        print(f"  Escuchando respuestas y eventos en tiempo real (127.0.0.1:{self.port})")
        print("=" * 72)
        print("  Esperando acciones del Rover...\n")

        try:
            while True:
                try:
                    data, _ = self._sock.recvfrom(BUF_SIZE)
                    msg = data.decode("utf-8", errors="replace").strip()
                    if not msg:
                        continue
                    if msg == "__CLOSE_MONITOR__":
                        print("\n[INFO] Cliente de comandos cerrado. Saliendo de la terminal de respuestas...")
                        time.sleep(1.0)
                        break

                    ts = datetime.now().strftime("%H:%M:%S")
                    print(f"  [{ts}] ➔  {msg}")
                except socket.timeout:
                    continue
        except KeyboardInterrupt:
            print("\n[INFO] Monitor finalizado.")
        finally:
            self._sock.close()


class RoverControlClient:
    """
    Object-oriented UDP client for the Laptop Command Terminal.
    Sends user commands to the Raspberry Pi 5 UDP server and routes all incoming
    responses and asynchronous events (IMU turn completion, ultrasonic obstacle
    evasion) to the second terminal window.
    """
    def __init__(
        self,
        server_ip: str = SERVER_IP,
        server_port: int = SERVER_PORT,
        monitor_port: int = MONITOR_PORT,
        spawn_monitor: bool = True,
    ):
        self._server_addr  = (server_ip, server_port)
        self._monitor_addr = ("127.0.0.1", monitor_port)
        self._spawn_monitor = spawn_monitor

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.3)

        self._stop = threading.Event()
        self._monitor_proc: subprocess.Popen | None = None
        self._rx_thread = threading.Thread(
            target=self._udp_receiver_worker,
            daemon=True,
            name="UDPResponseForwarder",
        )

    def start(self):
        if self._spawn_monitor:
            self._monitor_proc = MonitorTerminalLauncher.launch(self._monitor_addr[1])
            time.sleep(0.4)

        self._rx_thread.start()
        # Register client address with Raspberry Pi server
        try:
            self._sock.sendto(b"PING", self._server_addr)
        except OSError:
            pass

    def _udp_receiver_worker(self):
        """
        Continuously receives UDP datagrams from the Raspberry Pi 5 (both synchronous
        command replies and asynchronous IMU/Ultrasonic events) and forwards them to
        the Response Monitor Terminal.
        """
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(BUF_SIZE)
                msg = data.decode("utf-8", errors="replace").strip()
                if not msg:
                    continue

                if self._monitor_proc is not None:
                    self._sock.sendto(msg.encode("utf-8"), self._monitor_addr)
                else:
                    # Fallback inline print if --no-monitor-window was used
                    ts = datetime.now().strftime("%H:%M:%S")
                    print(f"\n[{ts} RPi] ➔ {msg}")
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                if not self._stop.is_set():
                    time.sleep(0.05)

    def send_command(self, cmd: str):
        clean = cmd.strip()
        if not clean:
            return
        try:
            self._sock.sendto(clean.encode("utf-8"), self._server_addr)
            print(f"  [TX -> RPi] {clean}")
        except OSError as e:
            print(f"  [ERR UDP] No se pudo enviar el comando: {e}")

    def close(self):
        self._stop.set()
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.8)
        if self._monitor_proc is not None:
            try:
                self._sock.sendto(b"__CLOSE_MONITOR__", self._monitor_addr)
            except OSError:
                pass
        try:
            self._sock.close()
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
        print("    ARM-EXTEND  (o ARM-DEPLOYMENT) Extiende el brazo robótico")
        print("    ARM-RETRIEVE (o ARM-HOME)      Regresa el brazo a posición home")
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
    parser.add_argument("--monitor-port", type=int, default=MONITOR_PORT, help=f"Puerto local del monitor (default: {MONITOR_PORT})")
    parser.add_argument("--no-spawn",     action="store_true", help="No abrir automáticamente la segunda ventana de terminal")
    args = parser.parse_args()

    if args.monitor:
        monitor = ResponseMonitorTerminal(port=args.monitor_port)
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
