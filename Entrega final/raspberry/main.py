#!/usr/bin/env python3
"""
raspberry/main.py — Main Entrypoint for Raspberry Pi 5 Rover Control System (Entrega Final).

Initializes and coordinates:
  1. RoverController (control.py) : UART control (ESP32 DevKit) of 4x DC wheel motors
                                    via 2x L298N Mini (87.5 mm), 2x arm servos (LEDC),
                                    GY-511 (LSM303DLHC) IMU, and 2x HC-SR04.
  2. UDPRoverServer  (server.py)  : Two-thread UDP server communicating with the Laptop client.
"""
import argparse
import sys
import time

from control import RoverController, SERIAL_PORT, BAUD_RATE
from server import UDPRoverServer, UDP_IP, UDP_PORT


class RoverApplication:
    """Top-level coordinator running on Raspberry Pi 5."""
    def __init__(
        self,
        udp_ip: str = UDP_IP,
        udp_port: int = UDP_PORT,
        serial_port: str = SERIAL_PORT,
        baudrate: int = BAUD_RATE,
    ):
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        self.controller = RoverController(serial_port=serial_port, baud=baudrate)
        self.server     = UDPRoverServer(
            controller=self.controller,
            udp_ip=udp_ip,
            udp_port=udp_port,
        )

    def run(self):
        print("=" * 68)
        print("  ROVER CONTROL SYSTEM — RASPBERRY PI 5 (Entrega Final)")
        print("  Controlador: ESP32 DevKit | Ruedas 87.5 mm (2x L298N Mini)")
        print("  Brazo: 2 GDL directo ESP32 | Sensores: GY-511 + 2x HC-SR04")
        print("=" * 68)

        self.controller.start()
        self.server.start()
        print("-" * 68)
        print("[MAIN] Sistema en ejecución. Presiona Ctrl+C para detener.\n")

        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[MAIN] Interrupción detectada (Ctrl+C). Deteniendo rover...")
        finally:
            # Enviar paro de seguridad antes de cerrar UART y UDP
            try:
                self.controller.execute_command("STOP")
            except Exception:
                pass
            self.server.shutdown()
            self.controller.stop()
            print("[MAIN] Apagado completo.")


def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 Rover Control Orchestrator (main.py -> server.py + control.py)"
    )
    parser.add_argument("--udp-ip",      default=UDP_IP,      help=f"IP de escucha UDP (default: {UDP_IP})")
    parser.add_argument("--udp-port",    type=int, default=UDP_PORT, help=f"Puerto UDP (default: {UDP_PORT})")
    parser.add_argument("--serial-port", default=SERIAL_PORT, help=f"Puerto UART ESP32 (default: {SERIAL_PORT})")
    parser.add_argument("--baudrate",    type=int, default=BAUD_RATE, help=f"Baudrate UART (default: {BAUD_RATE})")
    args = parser.parse_args()

    app = RoverApplication(
        udp_ip=args.udp_ip,
        udp_port=args.udp_port,
        serial_port=args.serial_port,
        baudrate=args.baudrate,
    )
    app.run()


if __name__ == "__main__":
    main()
