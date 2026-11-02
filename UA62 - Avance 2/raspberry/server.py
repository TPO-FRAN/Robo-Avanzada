#!/usr/bin/env python3
"""
raspberry/server.py — Object-Oriented Two-Thread UDP Server for Rover Control (UA62 - Avance 2).

Architecture modeled after UA34 - Robot Perception:
  - Thread-1 (WiFiListener)      : Receives UDP commands from Laptop Client with emergency
                                   STOP queue preemption.
  - Thread-2 (CommandDispatcher) : Dispatches commands to RoverController (control.py)
                                   and returns natural-language Spanish responses via UDP.
  - Async Event Push             : Immediately forwards asynchronous IMU & Ultrasonic
                                   events from RoverController to the connected Laptop client.
"""
import os
import queue
import socket
import threading

from control import RoverController, SERIAL_PORT, BAUD_RATE

UDP_IP   = "0.0.0.0"
UDP_PORT = 2222
BUF_SIZE = 2048


def _disable_windows_udp_connreset(sock: socket.socket):
    """Prevents Windows Winsock ICMP Port-Unreachable (WinError 10054) from dropping UDP reads."""
    if os.name == "nt" and hasattr(socket, "SIO_UDP_CONNRESET"):
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except OSError:
            pass


class UDPRoverServer:
    """
    Two-thread UDP server coordinating WiFi UDP reception and UART RoverController execution.
    """
    def __init__(
        self,
        controller: RoverController,
        udp_ip: str = UDP_IP,
        udp_port: int = UDP_PORT,
    ):
        self.controller = controller
        self.udp_ip     = udp_ip
        self.udp_port   = udp_port

        self._q: queue.Queue[tuple[str, tuple[str, int]]] = queue.Queue()
        self._stop = threading.Event()
        self._addr_lock = threading.Lock()
        self._tx_lock   = threading.Lock()
        self._last_client_addr: tuple[str, int] | None = None

        self._sock = self._open_udp(udp_ip, udp_port)
        self.controller.set_async_callback(self._push_async_event)

        self._t1 = threading.Thread(
            target=self._wifi_listener_worker,
            daemon=True,
            name="WiFiListener",
        )
        self._t2 = threading.Thread(
            target=self._command_dispatcher_worker,
            daemon=True,
            name="CommandDispatcher",
        )

    def _open_udp(self, ip: str, port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        _disable_windows_udp_connreset(s)
        s.bind((ip, port))
        s.settimeout(0.5)
        print(f"[UDP ] Servidor enlazado en {ip}:{port}")
        return s

    def _flush_pending_commands(self):
        """Flushes unexecuted commands when an emergency STOP arrives."""
        flushed = 0
        while not self._q.empty():
            try:
                self._q.get_nowait()
                self._q.task_done()
                flushed += 1
            except queue.Empty:
                break
        if flushed > 0:
            print(f"[UDP PRIORITY] STOP descartó {flushed} comando(s) pendiente(s) en cola.")

    # ── Thread Workers ───────────────────────────────────────────────────────
    def _wifi_listener_worker(self):
        print(f"[{threading.current_thread().name}] Escuchando comandos UDP...")
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(BUF_SIZE)
                cmd = data.decode("utf-8", errors="replace").strip()
                if cmd:
                    with self._addr_lock:
                        self._last_client_addr = addr
                    print(f"[UDP RX] '{cmd}' desde {addr[0]}:{addr[1]}")
                    if self.controller.is_emergency_stop(cmd):
                        self._flush_pending_commands()
                    self._q.put((cmd, addr))
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError as e:
                if not self._stop.is_set():
                    print(f"[UDP ERR] {e}")
        print(f"[{threading.current_thread().name}] Detenido.")

    def _command_dispatcher_worker(self):
        print(f"[{threading.current_thread().name}] Listo para despachar a control.py...")
        while not self._stop.is_set():
            try:
                cmd, addr = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                if cmd.upper() == "PING":
                    reply = "Conectado al Rover (Raspberry Pi 5 lista)"
                else:
                    reply = self.controller.execute_command(cmd, defer_events=True)

                self._send_udp(reply, addr)
                print(f"[TX->UDP] '{reply}' -> {addr[0]}:{addr[1]}")
            except Exception as e:
                err_msg = f"[ERROR] {e}"
                print(f"[DISPATCH ERR] {e}")
                self._send_udp(err_msg, addr)
            finally:
                # Guarantee chronological delivery of any [EVENT] that followed [ACK]
                self.controller.flush_deferred_events()
                self._q.task_done()
        print(f"[{threading.current_thread().name}] Detenido.")

    def _push_async_event(self, message: str):
        """Called by RoverController when an async hardware event (Gyro turn done / Obstacle) occurs."""
        with self._addr_lock:
            target = self._last_client_addr
        if target is not None:
            self._send_udp(message, target)
            print(f"[EVENT->UDP] '{message}' -> {target[0]}:{target[1]}")

    def _send_udp(self, text: str, addr: tuple[str, int]):
        with self._tx_lock:
            try:
                self._sock.sendto(text.encode("utf-8"), addr)
            except OSError as e:
                if not self._stop.is_set():
                    print(f"[UDP TX ERR] {e}")

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def start(self):
        self._t1.start()
        self._t2.start()
        print("[SERVER] Hilos WiFiListener y CommandDispatcher activos.")

    def shutdown(self):
        print("\n[SERVER] Cerrando servidor UDP...")
        self._stop.set()
        self._t1.join(timeout=1.0)
        self._t2.join(timeout=1.0)
        try:
            self._sock.close()
        except Exception:
            pass
        print("[SERVER] Servidor UDP cerrado.")


if __name__ == "__main__":
    # Si se ejecuta server.py directamente, delega a main.py
    from main import main
    main()
