#!/usr/bin/env python3
"""
motor_host.py - Host-side controller for the dual-motor linear-coupling board.

Talks to the STM32F103 USB CDC virtual COM port using the ASCII protocol:
    S<speed>    set motor A speed in per-mille [-1000,1000]; B = k*A
    K<percent>  set coupling ratio [0,100]
    R           request status  -> "A=.. B=.. K=.."
    X           immediate stop
    ?           help

Two modes:
    python motor_host.py --mock          # offline: simulate the firmware, no hardware
    python motor_host.py --port COM3     # real: talk to the board

The --mock mode implements the exact same coupling + ramp math as motor.c,
so you can verify the host logic and demonstrate the linear relation before
the hardware is wired up.
"""
import argparse
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    serial = None


# ---------------------------------------------------------------------------
# Mock firmware: mirrors motor.c (ratio + ramp), used when no board is present
# ---------------------------------------------------------------------------
class MockBoard:
    def __init__(self):
        self.target_a = 0
        self.out_a = 0
        self.out_b = 0
        self.ratio = 500  # permille
        self.step = 25

    def tick(self):
        target_b = (self.ratio * self.out_a) // 1000
        self.out_a = self._ramp(self.out_a, self.target_a)
        target_b = (self.ratio * self.out_a) // 1000
        self.out_b = self._ramp(self.out_b, target_b)

    def _ramp(self, cur, tgt):
        if self.step <= 0:
            return tgt
        if cur < tgt:
            return min(cur + self.step, tgt)
        if cur > tgt:
            return max(cur - self.step, tgt)
        return cur

    def send(self, line):
        """Return the device reply string for a command line."""
        cmd = line.strip()
        if not cmd:
            return ""
        c = cmd[0]
        if c == 'S':
            try:
                v = int(cmd[1:])
            except ValueError:
                return "ERR\r\n"
            self.target_a = max(-1000, min(1000, v))
            return "OK\r\n"
        if c == 'K':
            try:
                v = int(cmd[1:])
            except ValueError:
                return "ERR\r\n"
            self.ratio = max(0, min(1000, v * 10))
            return "OK\r\n"
        if c == 'X':
            self.target_a = self.out_a = self.out_b = 0
            return "STOPPED\r\n"
        if c == 'R':
            return "A=%+d B=%+d K=%d.%02d\r\n" % (
                self.out_a, self.out_b, self.ratio // 1000,
                (self.ratio % 1000) // 10)
        if c == '?':
            return "CMD: S+500 K50 R X ?\r\n"
        return "ERR\r\n"


# ---------------------------------------------------------------------------
# Real serial transport
# ---------------------------------------------------------------------------
class SerialBoard:
    def __init__(self, port, baud=115200):
        if serial is None:
            sys.exit("pyserial not installed: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.2)  # allow CDC enumeration to settle

    def send(self, line):
        self.ser.write((line.strip() + "\n").encode())
        time.sleep(0.05)
        # Drain any pending bytes (OK/status/heartbeat).
        out = bytearray()
        while self.ser.in_waiting:
            out += self.ser.read(self.ser.in_waiting)
            time.sleep(0.02)
        return out.decode(errors="ignore")

    def close(self):
        self.ser.close()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def parse_status(reply):
    """Parse 'A=+500 B=+250 K=0.50' into ints."""
    import re
    m = re.search(r"A=([+-]?\d+)\s+B=([+-]?\d+)\s+K=(\d+)\.(\d+)", reply)
    if not m:
        return None
    # K is reported as "0.50" meaning 50% = 500 permille.
    k_permille = int(m.group(3)) * 1000 + int(m.group(4)) * 10
    return int(m.group(1)), int(m.group(2)), k_permille


def sweep(board, steps=9):
    """Sweep motor A from -800 to +800 and verify B = k*A each point."""
    print("\ncmd_A   A(actual)   B(actual)   k*B_expected   linear_err")
    print("-" * 58)
    max_err = 0
    for v in range(-800, 801, 1600 // (steps - 1)):
        board.send("S%+d" % v)
        # settle the ramp
        for _ in range(120):
            if hasattr(board, "tick"):
                board.tick()
            else:
                time.sleep(0.005)
        reply = board.send("R")
        st = parse_status(reply)
        if not st:
            print("  (no status: %r)" % reply)
            continue
        a, b, k_permille = st
        expect = (k_permille * a) // 1000
        err = abs(b - expect)
        max_err = max(max_err, err)
        print("%+6d  %+9d  %+9d  %+12d  %8d" % (v, a, b, expect, err))
    print("-" * 58)
    print("最大线性误差 = %d (per mille), %s" %
          (max_err, "PASS" if max_err <= 2 else "FAIL"))
    return max_err


def interactive(board):
    print("Commands: S<speed>  K<percent>  R  X  ?  sweep  quit")
    print("Example : S+500   -> motor A=+500, motor B=+250 (k=0.5)")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ("quit", "q", "exit"):
            break
        if line.lower() == "sweep":
            sweep(board)
            continue
        if not line:
            continue
        reply = board.send(line)
        if hasattr(board, "tick"):
            for _ in range(120):
                board.tick()  # settle ramp before reading status in mock
        print(reply.strip() or "(no reply)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mock", action="store_true", help="offline simulation, no hardware")
    ap.add_argument("--port", help="serial port, e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sweep", action="store_true", help="run the linear sweep test and exit")
    args = ap.parse_args()

    if args.mock:
        print("=== MOCK MODE (firmware simulated in Python) ===")
        board = MockBoard()
    elif args.port:
        print("=== SERIAL MODE: %s ===" % args.port)
        board = SerialBoard(args.port, args.baud)
    else:
        ap.error("specify --mock or --port COMx")

    try:
        if args.sweep:
            sweep(board)
        else:
            interactive(board)
    finally:
        if hasattr(board, "close"):
            board.close()


if __name__ == "__main__":
    main()
