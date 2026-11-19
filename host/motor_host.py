#!/usr/bin/env python3
"""
motor_host.py - Host controller for the dual-motor sine/cosine board.

Protocol over USB CDC (ASCII, newline terminated):
    S<speed>    set motor A, per mille [-1000,1000]
    K<percent>  set k = percent/100, 1..100 (default 100)
    R           status -> "A=.. B=.. K=.."
    X           stop
    ?           help

Firmware constraint:  sin(v1) = k * cos(v2)
    v1 =  a      * pi / 2000
    v2 = (1000-b) * pi / 2000

Run:
    python motor_host.py --mock             offline, no board
    python motor_host.py --mock --sweep     verify the constraint
    python motor_host.py --port COM3        real board
"""
import argparse
import math
import re
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    serial = None

PI_OVER_2000 = math.pi / 2000.0
TWO_K_OVER_PI = 2000.0 / math.pi


class MockBoard:
    """Same math as motor.c, for offline verification."""

    def __init__(self):
        self.target_a = 0
        self.out_a = 0
        self.out_b = 0
        self.ratio = 1000       # k = 1.0
        self.step = 25

    def _ramp(self, cur, tgt):
        if self.step <= 0:
            return tgt
        if cur < tgt:
            return min(cur + self.step, tgt)
        if cur > tgt:
            return max(cur - self.step, tgt)
        return cur

    def _a_limit(self):
        k = self.ratio / 1000.0
        if k < 0.001:
            return 0
        return int(TWO_K_OVER_PI * math.asin(min(k, 1.0)))

    def _coupling_b(self, a):
        k = self.ratio / 1000.0
        if k < 0.001:
            return 0
        c = math.sin(a * PI_OVER_2000) / k
        c = max(-1.0, min(1.0, c))
        return int(1000.0 - math.acos(c) * TWO_K_OVER_PI)

    def tick(self):
        lim = self._a_limit()
        self.out_a = self._ramp(self.out_a, max(-lim, min(lim, self.target_a)))
        self.out_b = self._ramp(self.out_b, self._coupling_b(self.out_a))

    def send(self, line):
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
            self.ratio = max(1, min(1000, v * 10))
            return "OK\r\n"
        if c == 'X':
            self.target_a = self.out_a = self.out_b = 0
            return "STOPPED\r\n"
        if c == 'R':
            return "A=%+d B=%+d K=%d.%02d\r\n" % (
                self.out_a, self.out_b, self.ratio // 1000,
                (self.ratio % 1000) // 10)
        if c == '?':
            return "CMD: S+500 K100 R X ?\r\n"
        return "ERR\r\n"


class SerialBoard:
    def __init__(self, port, baud=115200):
        if serial is None:
            sys.exit("pyserial not installed: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.2)

    def send(self, line):
        self.ser.write((line.strip() + "\n").encode())
        time.sleep(0.05)
        out = bytearray()
        while self.ser.in_waiting:
            out += self.ser.read(self.ser.in_waiting)
            time.sleep(0.02)
        return out.decode(errors="ignore")

    def close(self):
        self.ser.close()


def parse_status(reply):
    m = re.search(r"A=([+-]?\d+)\s+B=([+-]?\d+)\s+K=(\d+)\.(\d+)", reply)
    if not m:
        return None
    # "0.50" means k = 0.50 = 500 permille
    k_permille = int(m.group(3)) * 1000 + int(m.group(4)) * 10
    return int(m.group(1)), int(m.group(2)), k_permille


def constraint_error(a, b, k_permille):
    """Residual of sin(v1) - k*cos(v2), using the exact phases."""
    v1 = a * PI_OVER_2000
    v2 = (1000.0 - b) * PI_OVER_2000
    return math.sin(v1) - (k_permille / 1000.0) * math.cos(v2)


def sweep(board, steps=9):
    lim = int(TWO_K_OVER_PI * math.asin(1.0))  # k from board below
    reply = board.send("R")
    st = parse_status(reply)
    if st:
        k_permille = st[2]
        k = k_permille / 1000.0
        if 0.0 < k <= 1.0:
            lim = int(TWO_K_OVER_PI * math.asin(k))

    print("\ncmd_A    A(actual)  B(actual)  sin(v1)  k*cos(v2)   error")
    print("-" * 62)
    max_err = 0.0
    for i in range(steps):
        v = -lim + (2 * lim * i) // (steps - 1)
        board.send("S%+d" % v)
        for _ in range(160):
            if hasattr(board, "tick"):
                board.tick()
            else:
                time.sleep(0.005)
        st = parse_status(board.send("R"))
        if not st:
            print("  (no status: %r)" % reply)
            continue
        a, b, k_permille = st
        err = constraint_error(a, b, k_permille)
        max_err = max(max_err, abs(err))
        v1 = a * PI_OVER_2000
        v2 = (1000.0 - b) * PI_OVER_2000
        print("%+6d  %+9d  %+9d  %+8.4f  %+8.4f  %+8.5f" %
              (v, a, b, math.sin(v1),
               (k_permille / 1000.0) * math.cos(v2), err))
    print("-" * 62)
    print("最大残差 |sin(v1) - k*cos(v2)| = %.5f, %s" %
          (max_err, "PASS" if max_err < 0.01 else "FAIL"))
    return max_err


def interactive(board):
    print("Commands: S<speed>  K<percent>  R  X  ?  sweep  quit")
    print("Example : S+500 with K100 -> A=+500, B=+500")
    print("          K50 -> |A| limited to 333, B reaches about +/-972")
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
            for _ in range(160):
                board.tick()
        print(reply.strip() or "(no reply)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mock", action="store_true", help="offline simulation")
    ap.add_argument("--port", help="serial port, e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sweep", action="store_true", help="verify the constraint and exit")
    args = ap.parse_args()

    if args.mock:
        print("=== MOCK MODE ===")
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
