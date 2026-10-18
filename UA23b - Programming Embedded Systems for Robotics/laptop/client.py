"""
laptop/client.py — UDP client for robotic actuator control.
Motor: JGB37-520 12V 333RPM | Encoder: 330 PPR
"""
import socket, re, argparse

SERVER_IP   = '192.168.1.119'
SERVER_PORT = 2222
BUF         = 1024
TIMEOUT     = 3.0

class ActuatorClient:
    _CMD_RE = re.compile(
        r'^(speed\s+\d+|direction\s+(CW|CCW)|speed\s+\d+\s*,\s*direction\s+(CW|CCW)|direction\s+(CW|CCW)\s*,\s*speed\s+\d+)$',
        re.IGNORECASE
    )

    def __init__(self, ip=SERVER_IP, port=SERVER_PORT, timeout=TIMEOUT):
        self._addr     = (ip, port)
        self._stopped  = False
        self._sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)

    def close(self):
        self._sock.close()

    # ── Public API ──────────────────────────────────────────────────────────
    def set_speed(self, val: int):
        if not 0 <= val <= 255:
            print("[ERR] Speed must be 0–255"); return
        self.send_command(f"speed {val}")

    def set_direction(self, direction: str):
        d = direction.upper()
        if d not in ("CW", "CCW"):
            print("[ERR] Direction must be CW or CCW"); return
        self.send_command(f"direction {d}")

    def set_combo(self, val: int, direction: str):
        if not 0 <= val <= 255:
            print("[ERR] Speed must be 0–255"); return
        self.send_command(f"speed {val}, direction {direction.upper()}")

    def emergency_stop(self):
        self._stopped = True
        print("[STOP] Emergency stop ON — actuator locked.")
        self.send_command("stop ON")

    def resume(self):
        self._stopped = False
        print("[STOP] Stop OFF — actuator unlocked.")
        self.send_command("stop OFF")

    def send_command(self, cmd: str):
        if self._stopped and any(k in cmd.lower() for k in ("speed", "direction")):
            print("[BLOCKED] STOP is ON. Send 'stop OFF' to resume."); return
        try:
            self._sock.sendto(cmd.encode(), self._addr)
            print(f"[TX] {cmd}")
            self.listen_response()
        except socket.timeout:
            print("[WARN] No ACK received (timeout).")

    def listen_response(self):
        try:
            data, addr = self._sock.recvfrom(BUF)
            print(f"[ACK from {addr[0]}] {data.decode().strip()}")
        except socket.timeout:
            print("[WARN] Response timeout.")

    # ── Interactive CLI ──────────────────────────────────────────────────────
    def run_cli(self):
        print(f"Actuator Client → {self._addr[0]}:{self._addr[1]}")
        print("Commands: speed <0-255> | direction <CW|CCW> | speed <v>, direction <d> | stop ON | stop OFF | status | quit (q)")
        try:
            while True:
                tag   = "STOP:ON" if self._stopped else "STOP:OFF"
                entry = input(f"\n[{tag}] > ").strip()
                if not entry: continue
                low = entry.lower()
                if low in ("q", "quit"): break
                elif low == "status":
                    print(f"[STATUS] Stop={'ON' if self._stopped else 'OFF'}")
                elif low in ("stop on",  "stop"):   self.emergency_stop()
                elif low in ("stop off", "resume"):  self.resume()
                elif self._CMD_RE.match(entry):      self.send_command(entry)
                else: print(f"[ERR] Unknown syntax: '{entry}'")
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted.")
        finally:
            self.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ip",   default=SERVER_IP)
    p.add_argument("--port", type=int, default=SERVER_PORT)
    p.add_argument("--timeout", type=float, default=TIMEOUT)
    a = p.parse_args()
    ActuatorClient(a.ip, a.port, a.timeout).run_cli()

if __name__ == "__main__":
    main()
