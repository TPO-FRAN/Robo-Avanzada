#!/usr/bin/env python3
"""
raspberry/server.py — Two-thread UDP↔UART bridge.
Thread-1 (WiFi Listener) → queue.Queue → Thread-2 (UART Commander)
Motor: JGB37-520 12V 333RPM | Arduino @ /dev/ttyACM0 9600 baud
"""
import socket, threading, queue, time, argparse, sys
import serial, serial.tools.list_ports

UDP_IP    = "0.0.0.0"
UDP_PORT  = 2222
SERIAL    = "/dev/ttyACM0"
BAUD      = 9600
BUF       = 1024


def find_arduino_port(preferred_port: str) -> str:
    """
    Verifica si el puerto serial especificado existe. Si no, busca puertos
    disponibles como /dev/ttyACM0, /dev/ttyUSB0 o puertos COM en Windows.
    """
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if preferred_port in ports:
        return preferred_port

    for port in ports:
        if "ACM" in port or "USB" in port:
            print(f"[SERIAL] Puerto preferido '{preferred_port}' no hallado. Seleccionando detectado: {port}")
            return port

    if ports:
        print(f"[SERIAL] Usando primer puerto detectado: {ports[0]}")
        return ports[0]

    print(f"[SERIAL WARNING] No se detectaron puertos activos. Intentando '{preferred_port}' por defecto.")
    return preferred_port


class UDPSerialBridge:
    def __init__(self, udp_ip=UDP_IP, udp_port=UDP_PORT,
                 serial_port=SERIAL, baud=BAUD):
        self._q      = queue.Queue()
        self._stop   = threading.Event()
        self._sock   = self._open_udp(udp_ip, udp_port)
        self._ser    = self._open_serial(serial_port, baud)
        self._t1     = threading.Thread(target=self._wifi_listener_worker,  daemon=True, name="WiFiListener")
        self._t2     = threading.Thread(target=self._uart_commander_worker, daemon=True, name="UARTCommander")

    # ── Setup helpers ────────────────────────────────────────────────────────
    def _open_udp(self, ip, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((ip, port)); s.settimeout(0.5)
        print(f"[UDP ] Bound to {ip}:{port}")
        return s

    def _open_serial(self, port, baud):
        active = find_arduino_port(port)
        ser = serial.Serial(active, baud, timeout=2.0)
        time.sleep(2.0)                         # let Arduino bootloader settle
        ser.reset_input_buffer()
        print(f"[UART] Connected to {active} @ {baud}")
        return ser

    # ── Thread workers ───────────────────────────────────────────────────────
    def _wifi_listener_worker(self):
        print(f"[{threading.current_thread().name}] Running…")
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(BUF)
                cmd = data.decode().strip()
                if cmd:
                    print(f"[UDP RX] '{cmd}' from {addr}")
                    self._q.put((cmd, addr))
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop.is_set(): print(f"[UDP ERR] {e}")
        print(f"[{threading.current_thread().name}] Stopped.")

    def _uart_commander_worker(self):
        print(f"[{threading.current_thread().name}] Running…")
        while not self._stop.is_set():
            try:
                cmd, addr = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._ser.reset_input_buffer()
                self._ser.write(f"{cmd}\n".encode()); self._ser.flush()
                ack = self._wait_ack(timeout=2.0)
                self._sock.sendto(ack.encode(), addr)
                print(f"[UART ACK→UDP] '{ack}' → {addr}")
            except serial.SerialException as e:
                err = f"[SERIAL ERROR] {e}"
                print(err); self._sock.sendto(err.encode(), addr)
            finally:
                self._q.task_done()
        print(f"[{threading.current_thread().name}] Stopped.")

    def _wait_ack(self, timeout=2.0) -> str:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ser.in_waiting:
                line = self._ser.readline().decode(errors="replace").strip()
                if line: return line
            time.sleep(0.01)
        return "[TIMEOUT] No Arduino response"

    # ── Lifecycle ────────────────────────────────────────────────────────────
    def start(self):
        self._t1.start(); self._t2.start()
        print("[SERVER] Both threads active. Press Ctrl+C to stop.\n" + "-"*50)

    def shutdown(self):
        print("\n[SERVER] Shutting down…")
        self._stop.set()
        self._t1.join(1.0); self._t2.join(1.0)
        try: self._ser.close()
        except: pass
        try: self._sock.close()
        except: pass
        print("[SERVER] Done.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--udp-ip",      default=UDP_IP)
    p.add_argument("--udp-port",    type=int, default=UDP_PORT)
    p.add_argument("--serial-port", default=SERIAL)
    p.add_argument("--baudrate",    type=int, default=BAUD)
    a = p.parse_args()

    bridge = UDPSerialBridge(a.udp_ip, a.udp_port, a.serial_port, a.baudrate)
    bridge.start()
    try:
        while True: time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()

if __name__ == "__main__":
    main()
