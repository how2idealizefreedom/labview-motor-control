#!/usr/bin/env python3
"""
Offline verification of the firmware logic. Mirrors coupling.c, pid.c,
encoder.c, motor.c and protocol.c line by line. No board and no C compiler
needed. Run: python tests/test_firmware.py
"""
import math
import re
import sys

PI_OVER_2000 = math.pi / 2000.0
TWO_K_OVER_PI = 2000.0 / math.pi
SPEED_MAX = 1000
TIMEOUT_MS = 500
results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print("  %s  %s%s" % ("PASS" if ok else "FAIL", name,
                          ("  " + detail) if detail else ""))


# ---------------------------------------------------------------- coupling.c
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
    c = max(-1.0, min(1.0, c))
    return int(1000.0 - math.acos(c) * TWO_K_OVER_PI)


def residual(a, b, k_permille):
    v1 = a * PI_OVER_2000
    v2 = (1000.0 - b) * PI_OVER_2000
    return math.sin(v1) - (k_permille / 1000.0) * math.cos(v2)


# --------------------------------------------------------------------- pid.c
class PID:
    def __init__(self, kp, ki, out_min, out_max):
        self.kp, self.ki = kp, ki
        self.out_min, self.out_max = out_min, out_max
        self.integ = 0.0

    def reset(self):
        self.integ = 0.0

    def update(self, err, dt):
        ilim = self.out_max / self.ki if self.ki > 0 else self.out_max
        self.integ += err * dt
        self.integ = clamp(self.integ, -ilim, ilim)
        out = self.kp * err + self.ki * self.integ
        return clamp(out, self.out_min, self.out_max)


# ------------------------------------------------------------------ firmware
class Firmware:
    """Mirrors the 5 ms motor tick and the CDC command decoder."""

    def __init__(self):
        self.targetA = 0
        self.cmdA = 0
        self.cmdB = 0
        self.outA = 0
        self.outB = 0
        self.ratio = 1000
        self.step = 25
        self.closedLoop = 0
        self.state = "I"
        self.now = 0
        self.lastCmd = 0
        self.telemetry = 0
        self.line = ""
        self.drop = False
        self.pidA = PID(0.40, 1.50, -1000, 1000)
        self.pidB = PID(0.40, 1.50, -1000, 1000)
        self.measA = 0.0
        self.measB = 0.0
        self.replies = []

    def send(self, text):
        for ch in text:
            if ch in "\r\n":
                if self.drop:
                    self.drop = False
                    self.line = ""
                    continue
                if self.line:
                    self.replies.append(self.decode(self.line))
                    self.line = ""
            elif self.drop:
                continue
            elif len(self.line) < 63:
                self.line += ch
            else:
                self.line = ""
                self.drop = True

    def decode(self, ln):
        self.lastCmd = self.now
        c = ln[0]
        if c == 'S':
            try:
                v = int(ln[1:])
            except ValueError:
                return "ERR"
            self.targetA = clamp(v, -1000, 1000)
            return "OK"
        if c == 'K':
            try:
                v = int(ln[1:])
            except ValueError:
                return "ERR"
            self.ratio = clamp(v * 10, 1, 1000)
            return "OK"
        if c == 'T':
            v = parse_flag(ln[1:])
            if v is None:
                return "ERR"
            self.telemetry = v
            return "OK"
        if c == 'L':
            v = parse_flag(ln[1:])
            if v is None:
                return "ERR"
            if v != self.closedLoop:
                self.closedLoop = v
                self.outA, self.outB = self.cmdA, self.cmdB
                self.pidA.reset()
                self.pidB.reset()
            return "OK"
        if c == 'R':
            return "A=%+d B=%+d K=%d.%02d" % (
                self.outA, self.outB, self.ratio // 1000, (self.ratio % 1000) // 10)
        if c in ('V', '>'):
            return self.extended()
        if c == 'E':
            return "MA=%+d MB=%+d" % (int(self.measA), int(self.measB))
        if c == 'X':
            self.hard_stop()
            self.state = "I"
            return "STOPPED"
        if c == '?':
            return "CMD: S+500 K100 T1 L1 R V E X ?"
        return "ERR"

    def extended(self):
        st = "R" if self.state == "RUN" else ("T" if self.state == "TIMEOUT" else "I")
        return ("A=%+d B=%+d CA=%+d CB=%+d MA=%+d MB=%+d K=%d.%02d ST=%s L=%d" %
                (self.outA, self.outB, self.cmdA, self.cmdB,
                 int(self.measA), int(self.measB),
                 self.ratio // 1000, (self.ratio % 1000) // 10,
                 st, self.closedLoop))

    def hard_stop(self):
        self.targetA = self.cmdA = self.cmdB = self.outA = self.outB = 0
        self.pidA.reset()
        self.pidB.reset()

    def tick(self):
        self.now += 5
        if self.now - self.lastCmd > TIMEOUT_MS:
            if self.state != "TIMEOUT":
                self.hard_stop()
                self.state = "TIMEOUT"
            return
        lim = a_limit(self.ratio)
        self.cmdA = ramp(self.cmdA, clamp(self.targetA, -lim, lim), self.step)
        self.cmdB = solve_b(self.cmdA, self.ratio)
        if self.closedLoop:
            self.outA = int(self.pidA.update(self.cmdA - self.measA, 0.005))
            self.outB = int(self.pidB.update(self.cmdB - self.measB, 0.005))
        else:
            self.outA = self.cmdA
            self.outB = ramp(self.outB, self.cmdB, self.step)
        self.state = "I" if (self.targetA == 0 and self.cmdA == 0 and self.cmdB == 0) else "RUN"


def parse_flag(s):
    try:
        v = int(s)
    except ValueError:
        return None
    return v if v in (0, 1) else None


def settle(fw, ticks=160):
    for i in range(ticks):
        if i % 40 == 0:
            fw.send("R\n")          # keep the watchdog fed while settling
        fw.tick()


# ------------------------------------------------------------------- test 1
def test_coupling():
    print("\n[1] 三角约束 sin(v1)=k*cos(v2)")
    worst = 0.0
    for kpct in [100, 80, 50, 25]:
        k = kpct * 10
        lim = a_limit(k)
        for i in range(9):
            a = -lim + (2 * lim * i) // 8
            b = solve_b(a, k)
            worst = max(worst, abs(residual(a, b, k)))
    check("4 种 k × 9 个速度点残差 < 0.01", worst < 0.01, "max=%.5f" % worst)
    check("k=1 时 b=a", all(solve_b(a, 1000) == a for a in [-1000, -500, 0, 500, 1000]))
    check("k=0 不产生除零", solve_b(500, 0) == 0 and a_limit(0) == 0)


# ------------------------------------------------------------------- test 2
def test_domain():
    print("\n[2] 定义域限幅")
    expect = {1000: 1000, 800: 590, 500: 333, 250: 160}
    ok = True
    for k, v in expect.items():
        got = a_limit(k)
        if abs(got - v) > 1:
            ok = False
            print("      k=%d got=%d expect=%d" % (k, got, v))
    check("asin(k) 限幅值", ok)
    fw = Firmware()
    fw.send("K50\n")
    fw.send("S+1000\n")
    settle(fw)
    check("越界指令被截断", abs(fw.cmdA) <= a_limit(fw.ratio) + 1,
          "cmdA=%d limit=%d" % (fw.cmdA, a_limit(fw.ratio)))


# ------------------------------------------------------------------- test 3
def test_ramp():
    print("\n[3] 斜坡")
    cur = 0
    hist = []
    for _ in range(60):
        cur = ramp(cur, 1000, 25)
        hist.append(cur)
    steps = [abs(hist[i] - hist[i - 1]) for i in range(1, len(hist))]
    check("单拍变化 <= 25", max(steps) <= 25, "max=%d" % max(steps))
    check("40 拍到满速", hist[39] == 1000, "tick40=%d" % hist[39])
    check("单调上升", all(hist[i] >= hist[i - 1] for i in range(1, len(hist))))


# ------------------------------------------------------------------- test 4
def test_pid():
    print("\n[4] PI 控制器")
    p = PID(0.40, 1.50, -1000, 1000)
    out = 0.0
    for _ in range(1200):
        out = p.update(500.0 - out, 0.005)
    check("阶跃收敛到给定", abs(out - 500) < 15, "out=%.1f" % out)
    p2 = PID(0.30, 5.0, -100, 100)
    for _ in range(500):
        p2.update(1000.0, 0.005)
    check("积分限幅无饱和绕卷", abs(p2.integ) <= 100 / 5.0 + 1e-6, "integ=%.1f" % p2.integ)
    check("输出不超限", -100 <= p2.update(1000.0, 0.005) <= 100)


# ------------------------------------------------------------------- test 5
def test_protocol():
    print("\n[5] 协议解析")
    fw = Firmware()
    fw.send("S+500\nK50\nT1\nL1\nR\nV\nE\n?\nX\n")
    expected = ["OK", "OK", "OK", "OK", None, None, None, None, "STOPPED"]
    got = fw.replies
    check("9 条命令均被识别", len(got) == 9)
    check("S/K/T/L 返回 OK", got[:4] == expected[:4], str(got[:4]))
    check("V 帧含状态与闭环标志", "ST=" in got[5] and "L=1" in got[5])
    check("E 帧含实测速度", "MA=" in got[6])
    fw.replies = []
    for bad in ["Sabc\n", "Kxx\n", "T2\n", "L9\n", "Z\n"]:
        fw.send(bad)
    check("非法参数返回 ERR", all(r == "ERR" for r in fw.replies), str(fw.replies))
    fw.replies = []
    fw.send("S" + "9" * 200 + "\n")
    fw.send("R\n")
    check("超长行被丢弃且不误触发", len(fw.replies) == 1, str(len(fw.replies)))
    fw.replies = []
    fw.send("S+100\r\nR\r\n")
    check("CRLF 兼容", len(fw.replies) == 2, str(fw.replies))


# ------------------------------------------------------------------- test 6
def test_watchdog():
    print("\n[6] 通信看门狗")
    fw = Firmware()
    fw.send("S+500\n")
    for _ in range(80):
        fw.tick()
    check("超时前仍在运行", fw.state == "RUN" and fw.outA == 500,
          "state=%s outA=%d" % (fw.state, fw.outA))
    for _ in range(30):
        fw.tick()
    check("500ms 无命令自动停机", fw.state == "TIMEOUT" and fw.outA == 0 and fw.outB == 0,
          "state=%s outA=%d" % (fw.state, fw.outA))
    fw.send("R\n")
    fw.send("S+200\n")
    for _ in range(80):
        fw.tick()
    check("新命令可恢复", fw.state == "RUN" and fw.outA == 200)


# ------------------------------------------------------------------- test 7
def test_closed_loop():
    print("\n[7] 编码器闭环")
    fw = Firmware()
    fw.send("K100\nL1\nS+600\n")
    for i in range(1200):
        if i % 40 == 0:
            fw.send("R\n")          # keep watchdog fed
        fw.tick()
        # first-order plant: PWM duty -> measured speed
        fw.measA += (fw.outA - fw.measA) * 0.15
        fw.measB += (fw.outB - fw.measB) * 0.15
    check("闭环确实在运行", fw.state == "RUN" and fw.closedLoop == 1,
          "state=%s outA=%d" % (fw.state, fw.outA))
    check("A 实测收敛到给定", abs(fw.measA - fw.cmdA) < 15,
          "measA=%.1f cmdA=%d" % (fw.measA, fw.cmdA))
    check("B 实测收敛到给定", abs(fw.measB - fw.cmdB) < 15,
          "measB=%.1f cmdB=%d" % (fw.measB, fw.cmdB))
    check("闭环下目标仍满足三角约束", abs(residual(fw.cmdA, fw.cmdB, fw.ratio)) < 0.01)
    fw.send("L0\n")
    for i in range(100):
        if i % 40 == 0:
            fw.send("R\n")
        fw.tick()
        fw.measA += (fw.outA - fw.measA) * 0.15
    check("可切回开环", fw.closedLoop == 0 and fw.outA == fw.cmdA)


def main():
    test_coupling()
    test_domain()
    test_ramp()
    test_pid()
    test_protocol()
    test_watchdog()
    test_closed_loop()
    n = len(results)
    passed = sum(1 for _, ok, _ in results if ok)
    print("\n总计 %d/%d 通过" % (passed, n))
    return 0 if passed == n else 1


if __name__ == "__main__":
    sys.exit(main())
