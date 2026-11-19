#!/usr/bin/env python3
"""
motor_host.py - Host controller for the dual-motor sine/cosine board.

Protocol over USB CDC (ASCII, newline terminated):
    S<speed>    set motor A, per mille [-1000,1000]
    K<percent>  set k = percent/100, 1..100 (default 100)
    T<0|1>      telemetry on/off
    L<0|1>      encoder speed closed loop on/off
    R           basic status
    V           extended status (commands, measurements, state)
    E           encoder counts and speeds
    X           stop
    ?           help

Run:
    python motor_host.py --mock --sweep
    python motor_host.py --mock --loop-sweep
    python motor_host.py --mock
    python motor_host.py --port COM3
"""
import argparse
import math
import re
import sys
import time

try:
    import serial
except ImportError:
    serial = None

PI_OVER_2000 = math.pi / 2000.0
TWO_K_OVER_PI = 2000.0 / math.pi


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def ramp(cur, tgt, step):
    if step <= 0:
        return tgt
    if cur < tgt:
        return min(cur + step, tgt)
    if cur > tgt:
        return max(cur - step, tgt)
    return cur


def a_limit(k_permille):
    k = k_permille / 1000.0
    return 0 if k < 0.001 else int(TWO_K_OVER_PI * math.asin(min(k, 1.0)))


def solve_b(a, k_permille):
    k = k_permille / 1000.0
    if k < 0.001:
        return 0
    c = math.sin(a * PI_OVER_2000) / k
    return int(1000.0 - math.acos(max(-1.0, min(1.0, c))) * TWO_K_OVER_PI)


class PID:
    def __init__(self, kp, ki, lo, hi):
        self.kp, self.ki, self.lo, self.hi = kp, ki, lo, hi
        self.integ = 0.0

    def reset(self):
        self.integ = 0.0

    def update(self, err, dt):
        ilim = self.hi / self.ki if self.ki > 0 else self.hi
        self.integ = clamp(self.integ + err * dt, -ilim, ilim)
        return clamp(self.kp * err + self.ki * self.integ, self.lo, self.hi)


class MockBoard:
    """Mirrors motor.c. Watchdog is covered by tests/test_firmware.py."""

    def __init__(self):
        self.target_a = 0
        self.cmd_a = 0
        self.cmd_b = 0
        self.out_a = 0
        self.out_b = 0
        self.meas_a = 0.0
        self.meas_b = 0.0
        self.ratio = 1000
        self.step = 25
        self.closed_loop = 0
        self.telemetry = 0
        self.pid_a = PID(0.40, 1.50, -1000, 1000)
        self.pid_b = PID(0.40, 1.50, -1000, 1000)

    def tick(self):
        lim = a_limit(self.ratio)
        self.cmd_a = ramp(self.cmd_a, clamp(self.target_a, -lim, lim), self.step)
        self.cmd_b = solve_b(self.cmd_a, self.ratio)
        if self.closed_loop:
            self.out_a = int(self.pid_a.update(self.cmd_a - self.meas_a, 0.005))
            self.out_b = int(self.pid_b.update(self.cmd_b - self.meas_b, 0.005))
        else:
            self.out_a = self.cmd_a
            self.out_b = ramp(self.out_b, self.cmd_b, self.step)

    def send(self, line):
        cmd = line.strip().lstrip('>')
        if not cmd:
            return ""
        c = cmd[0]
        if c == 'S':
            try:
                v = int(cmd[1:])
            except ValueError:
                return "ERR\r\n"
            self.target_a = clamp(v, -1000, 1000)
            return "OK\r\n"
        if c == 'K':
            try:
                v = int(cmd[1:])
            except ValueError:
                return "ERR\r\n"
            self.ratio = clamp(v * 10, 1, 1000)
            return "OK\r\n"
        if c == 'T':
            v = flag(cmd[1:])
            if v is None:
                return "ERR\r\n"
            self.telemetry = v
            return "OK\r\n"
        if c == 'L':
            v = flag(cmd[1:])
            if v is None:
                return "ERR\r\n"
            if v != self.closed_loop:
                self.closed_loop = v
                self.out_a, self.out_b = self.cmd_a, self.cmd_b
                self.pid_a.reset()
                self.pid_b.reset()
            return "OK\r\n"
        if c == 'R':
            return "A=%+d B=%+d K=%d.%02d\r\n" % (
                self.out_a, self.out_b, self.ratio // 1000, (self.ratio % 1000) // 10)
        if c == 'V':
            return self.extended()
        if c == 'E':
            return "MA=%+d MB=%+d\r\n" % (int(self.meas_a), int(self.meas_b))
        if c == 'X':
            self.target_a = self.cmd_a = self.cmd_b = self.out_a = self.out_b = 0
            self.pid_a.reset()
            self.pid_b.reset()
            return "STOPPED\r\n"
        if c == '?':
            return "CMD: S+500 K100 T1 L1 R V E X ?\r\n"
        return "ERR\r\n"

    def extended(self):
        return ("A=%+d B=%+d CA=%+d CB=%+d MA=%+d MB=%+d K=%d.%02d ST=%s L=%d\r\n" %
                (self.out_a, self.out_b, self.cmd_a, self.cmd_b,
                 int(self.meas_a), int(self.meas_b),
                 self.ratio // 1000, (self.ratio % 1000) // 10,
                 "R" if self.target_a else "I", self.closed_loop))


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


def flag(s):
    try:
        v = int(s)
    except ValueError:
        return None
    return v if v in (0, 1) else None


def parse_status(reply):
    m = re.search(r"A=([+-]?\d+)\s+B=([+-]?\d+)\s+K=(\d+)\.(\d+)", reply)
    if not m:
        return None
    return (int(m.group(1)), int(m.group(2)),
            int(m.group(3)) * 1000 + int(m.group(4)) * 10)


def parse_extended(reply):
    m = re.search(r"A=([+-]?\d+)\s+B=([+-]?\d+)\s+CA=([+-]?\d+)\s+CB=([+-]?\d+)\s+"
                  r"MA=([+-]?\d+)\s+MB=([+-]?\d+)\s+K=(\d+)\.(\d+)\s+ST=(\w)\s+L=(\d)",
                  reply)
    if not m:
        return None
    return {
        "A": int(m.group(1)), "B": int(m.group(2)),
        "CA": int(m.group(3)), "CB": int(m.group(4)),
        "MA": int(m.group(5)), "MB": int(m.group(6)),
        "K": int(m.group(7)) * 1000 + int(m.group(8)) * 10,
        "ST": m.group(9), "L": int(m.group(10)),
    }


def constraint_error(a, b, k_permille):
    v1 = a * PI_OVER_2000
    v2 = (1000.0 - b) * PI_OVER_2000
    return math.sin(v1) - (k_permille / 1000.0) * math.cos(v2)


def advance(board, ticks):
    for _ in range(ticks):
        if hasattr(board, "tick"):
            board.tick()
        else:
            time.sleep(0.005)


def sweep(board, steps=9):
    st = parse_status(board.send("R"))
    k = st[2] if st else 1000
    lim = a_limit(k) if 0 < k <= 1000 else 1000
    print("\ncmd_A    A(actual)  B(actual)  sin(v1)  k*cos(v2)   error")
    print("-" * 62)
    worst = 0.0
    for i in range(steps):
        v = -lim + (2 * lim * i) // (steps - 1)
        board.send("S%+d" % v)
        advance(board, 160)
        st = parse_status(board.send("R"))
        if not st:
            print("  (no status)")
            continue
        a, b, kp = st
        err = constraint_error(a, b, kp)
        worst = max(worst, abs(err))
        print("%+6d  %+9d  %+9d  %+8.4f  %+8.4f  %+8.5f" %
              (v, a, b, math.sin(a * PI_OVER_2000),
               (kp / 1000.0) * math.cos((1000.0 - b) * PI_OVER_2000), err))
    print("-" * 62)
    print("最大残差 = %.5f, %s" % (worst, "PASS" if worst < 0.01 else "FAIL"))


def loop_sweep(board):
    print("\n闭环验证：给定 -> 实测 (k=1, 每点 6 秒)\n")
    board.send("K100\nL1\n")
    worst = 0.0
    for v in [-600, -300, 0, 300, 600]:
        board.send("S%+d" % v)
        for _ in range(1200):
            advance(board, 1)
            if hasattr(board, "meas_a"):
                board.meas_a += (board.out_a - board.meas_a) * 0.15
                board.meas_b += (board.out_b - board.meas_b) * 0.15
        st = parse_extended(board.send("V"))
        if not st:
            print("  (no extended status)")
            continue
        err = max(abs(st["MA"] - st["CA"]), abs(st["MB"] - st["CB"]))
        worst = max(worst, err)
        print("cmd=%+5d  CA=%+5d MA=%+5d  CB=%+5d MB=%+5d  err=%d" %
              (v, st["CA"], st["MA"], st["CB"], st["MB"], err))
    print("最大跟踪误差 = %d, %s" % (worst, "PASS" if worst < 15 else "FAIL"))


def interactive(board):
    print("Commands: S<speed>  K<percent>  T<0|1>  L<0|1>  R  V  E  X  ?  sweep  quit")
    print("Example : S+500  K50  L1  V")
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
        advance(board, 160)
        print(reply.strip() or "(no reply)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mock", action="store_true", help="offline simulation")
    ap.add_argument("--port", help="serial port, e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sweep", action="store_true", help="verify sin(v1)=k*cos(v2)")
    ap.add_argument("--loop-sweep", action="store_true", help="verify closed loop tracking")
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
        elif args.loop_sweep:
            loop_sweep(board)
        else:
            interactive(board)
    finally:
        if hasattr(board, "close"):
            board.close()


if __name__ == "__main__":
    main()
