"""ROS2 Humble demo: chassis_bridge <-> lattice_planner over /cmd_vel and /odom.
Camera occupied: wait then left Lattice bypass, MPC track. Serial only in the bridge.
"""
import fcntl
import math
import mmap
import os
import select
import struct
import termios
import time
from io import BytesIO

import numpy as np
from scipy.linalg import solve_discrete_are
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

import sys
for extra in ("/usr/lib/python3/dist-packages", "/usr/lib/python3.10/dist-packages"):
    if extra not in sys.path:
        sys.path.append(extra)
try:
    from PIL import Image
except ImportError:
    Image = None

DEVICE = "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00"
WHEELBASE = 0.144
SPEED = 0.15
Y0 = 0.040
Y_TARGET = 0.020
TARGET = 2.10
STOP_S = 1.98
S_OBS_LO, S_OBS_HI, T_OBS_FREE = 0.75, 1.15, 8.0
RAMP = 0.22
PERIOD = 0.05
N_LONG = 16
N_LAT = 10
Q_S = 8.0
Q_V = 30.0
R_DV = 2.5
DV_MAX = 0.05
Q_LAT = np.diag([40.0, 3.0])
R_STEER = 2.0
R_DSTEER = 1.5
MAX_STEER = 0.38
MAX_YAW = 0.30
D0 = Y0 - Y_TARGET
GROUND_SHIFT = 0.11
START_Y = 0.15
BOX_S, BOX_Y = 1.00, 0.0
BOX_R, BODY_R, MARGIN, EXTRA = 0.12, 0.11, 0.05, 0.04
CLEAR = BOX_R + BODY_R + MARGIN
AVOID_TARGETS = (0.15, 0.28, 0.34, 0.40)
RETURN_TARGETS = (0.16,)
AVOID_SETTLES = (0.45, 0.55, 0.75, 0.95)
RETURN_SETTLES = (0.70, 0.80)
MAX_HEAD = math.radians(36.0)
REPLAN_S = 1.22
SETTLE_S = 1.55
SETTLE_Y = 0.02
W_D, W_V, W_DJ, W_SJ, W_T = 20.0, 1.0, 0.05, 0.05, 0.5
IDM_GAP = 0.40
IDM_HEADWAY = 1.0
IDM_ACCEL = 0.35
MOBIL_INCENTIVE = 0.1
HOLD_S = 0.55
SEAM_Y = 0.02
CAM = "/dev/video2"
BLOCK_OBS = True
CAM_BOX_PRESENT = True
ACTION = "等待"
RISK_NAME = ("安全", "注意", "危险", "紧急")
FSM_NAME = {"keep": "保持", "follow": "跟车", "prepare": "准备绕行", "lane": "绕行", "emerg": "急刹"}


def idm_accel(speed, gap):
    speed = max(speed, 0.0)
    free = 1.0 - (speed / SPEED) ** 4
    if gap is None:
        return IDM_ACCEL * free
    closing = speed
    desired_gap = IDM_GAP + max(
        0.0,
        speed * IDM_HEADWAY + (speed * closing) / (2.0 * math.sqrt(IDM_ACCEL * IDM_ACCEL)),
    )
    return IDM_ACCEL * (free - (desired_gap / max(gap, 0.05)) ** 2)


def ttc_of(gap, ego_v, front_v=0.0):
    closing = ego_v - front_v
    if gap <= 0.0:
        return 0.0
    if closing <= 0.0:
        return math.inf
    return gap / closing


def thw_of(gap, ego_v):
    if gap <= 0.0:
        return 0.0
    if ego_v <= 0.0:
        return math.inf
    return gap / ego_v


def risk_level(ttc, thw):
    def by_ttc(value):
        if value <= 1.5:
            return 3
        if value <= 3.0:
            return 2
        if value <= 6.0:
            return 1
        return 0

    def by_thw(value):
        if value <= 0.8:
            return 3
        if value <= 1.2:
            return 2
        if value <= 2.0:
            return 1
        return 0

    return max(by_ttc(ttc), by_thw(thw))


def fmt_time(value):
    if math.isinf(value):
        return "很大"
    return "%.1f" % value


class RoomBehavior:
    """Camera occupancy first. Box still there: wait then left bypass. Gone: straight."""

    def __init__(self):
        self.state = "keep"
        self.safe = 0
        self.erec = 0
        self.clear_count = 0

    def step(self, s, speed, now_t, cam_box):
        ego_v = max(speed, 0.0)
        if not cam_box:
            self.clear_count += 1
            if self.clear_count >= 2:
                self.state = "keep"
            action = "直过" if self.clear_count >= 2 else "等待"
            return action, self.state, 0, math.inf, math.inf, 0.0, False
        self.clear_count = 0
        gap = BOX_S - s
        level = risk_level(ttc_of(gap, ego_v), thw_of(gap, ego_v))
        for index in range(5):
            future_s = s + ego_v * (index * 0.5)
            future_gap = BOX_S - future_s
            level = max(level, risk_level(ttc_of(future_gap, ego_v), thw_of(future_gap, ego_v)))
        incentive = idm_accel(ego_v, None) - idm_accel(ego_v, gap)
        left = level < 3 and incentive > MOBIL_INCENTIVE
        self._advance(level, left)
        action = "绕行" if self.state == "lane" else "等待"
        return action, self.state, level, ttc_of(gap, ego_v), thw_of(gap, ego_v), incentive, left

    def _advance(self, level, left):
        if level >= 3:
            self.state = "emerg"
            self.safe = 0
            self.erec = 0
            return
        if self.state == "emerg":
            if level <= 1:
                self.erec += 1
                if self.erec >= 3:
                    self.erec = 0
                    self.state = "follow"
            else:
                self.erec = 0
            return
        if self.state == "keep":
            if level >= 1:
                self.state = "follow"
        elif self.state == "follow":
            if level == 0:
                self.safe += 1
                if self.safe >= 3:
                    self.safe = 0
                    self.state = "keep"
            else:
                self.safe = 0
                if left:
                    self.state = "lane"
        elif self.state == "prepare":
            self.state = "lane" if left else "follow"


def obstacle_release(s0, t0, v0):
    """Time the box frees, from current speed. Faster speed keeps a longer lead."""
    v_est = v0 if v0 >= 0.08 else SPEED
    lead = 1.5 * v_est / SPEED
    if s0 >= S_OBS_HI:
        return t0, 0.0
    if s0 >= S_OBS_LO:
        return t0 + (S_OBS_HI - s0) / v_est, 0.0
    return t0 + (S_OBS_LO - s0) / v_est + lead, lead


def speed_plan(s0=0.0, t0=0.0, v0=0.0, announce=True):
    """ST dynamic programming from the current pose, then a speed QP."""
    dt = 0.5
    ds = 0.025
    a_max = 0.35
    s0 = min(max(s0, 0.0), STOP_S)
    s_nodes = np.round(np.arange(s0, STOP_S + 1e-9, ds), 5)
    if len(s_nodes) < 2:
        s_nodes = np.array([s0, STOP_S])
        return s_nodes, np.array([0.0, 0.0])
    n_s = len(s_nodes)
    n_t = 28
    inf = 1e12
    cost = np.full((n_t, n_s), inf)
    parent = np.full((n_t, n_s), -1, dtype=int)
    cost[0, 0] = 0.0

    def desired(s):
        if s >= STOP_S:
            return 0.0
        if STOP_S - s > RAMP:
            return SPEED
        return SPEED * max(STOP_S - s, 0.0) / RAMP

    t_free, lead = obstacle_release(s0, t0, v0)

    def blocked(t_index, s_value):
        if not BLOCK_OBS:
            return False
        return S_OBS_LO <= s_value <= S_OBS_HI and t0 + t_index * dt < t_free

    for t in range(n_t - 1):
        for i, s in enumerate(s_nodes):
            if cost[t, i] >= inf / 2 or blocked(t, s):
                continue
            if parent[t, i] < 0:
                v_prev = v0 if t == 0 else 0.0
            else:
                v_prev = (s - s_nodes[parent[t, i]]) / dt
            for j in range(i, n_s):
                if blocked(t + 1, s_nodes[j]):
                    continue
                dist = s_nodes[j] - s
                v = dist / dt
                if v > SPEED + 1e-9:
                    break
                a = (v - v_prev) / dt
                if abs(a) > a_max:
                    continue
                step = (v - desired(0.5 * (s + s_nodes[j]))) ** 2 + 0.3 * a * a
                if t == n_t - 2:
                    step += 30.0 * (s_nodes[j] - STOP_S) ** 2 + 15.0 * v * v
                total = cost[t, i] + step
                if total < cost[t + 1, j]:
                    cost[t + 1, j] = total
                    parent[t + 1, j] = i
    if cost[-1].min() >= inf / 2:
        speed = np.linspace(min(v0, SPEED), 0.0, n_s)
        if announce:
            print("ST-DP 没有可行速度，改用减速到停车点。", flush=True)
        return s_nodes, speed
    end = int(np.argmin(cost[-1]))
    index = [end]
    for t in range(n_t - 1, 0, -1):
        previous = int(parent[t, index[-1]])
        if previous < 0:
            break
        index.append(previous)
    index.reverse()
    s_dp = s_nodes[np.array(index)]
    v_dp = np.diff(s_dp, append=s_dp[-1]) / dt
    v_dp[-1] = 0.0
    ref = np.minimum(np.interp(s_nodes, s_dp, v_dp, left=0.0, right=0.0), SPEED)
    hess = np.eye(n_s) * 8.0
    linear = -8.0 * ref
    diff = np.eye(n_s)
    diff[1:, :-1] -= np.eye(n_s - 1)
    hess = hess + 1.5 * (diff.T @ diff)
    speed = np.linalg.solve(hess + 1e-6 * np.eye(n_s), -linear)
    speed = np.clip(speed, 0.0, SPEED)
    for i in range(1, n_s):
        step = a_max * ds / max(speed[i - 1], 0.05)
        speed[i] = min(SPEED, max(0.0, min(speed[i], speed[i - 1] + step)))
        speed[i] = max(speed[i], speed[i - 1] - step)
    if announce and not BLOCK_OBS:
        print("ST-DP 决策已经是绕行，这段速度不再为盒子等待。", flush=True)
    elif announce:
        print(
            "ST-DP 盒子占住 s=%.2f–%.2f。放开时间按起步速度估计为 %.1f 秒，提前量 %.1f 秒。之后每 0.5 秒重算。"
            % (S_OBS_LO, S_OBS_HI, t_free, lead),
            flush=True,
        )
        arrive = next((t0 + k * dt for k, s_at in enumerate(s_dp) if s_at >= S_OBS_LO), None)
        if arrive is None:
            print("ST-DP 起点已经过了障碍前缘。", flush=True)
        else:
            print("ST-DP 到达障碍前缘 s=%.2f 的时间是 %.1f 秒。" % (S_OBS_LO, arrive), flush=True)
        for mark in (0.0, 0.5, 1.0, 1.5, 1.8):
            if mark < s0:
                continue
            print(
                "SPEED s=%.2f dp=%.2f qp=%.2f"
                % (mark, float(np.interp(mark, s_dp, v_dp)), float(np.interp(mark, s_nodes, speed))),
                flush=True,
            )
    return s_nodes, speed


def refresh_speed(s0, t0, v0):
    global _S_GRID, _V_GRID
    _S_GRID, _V_GRID = speed_plan(s0, t0, v0, announce=False)
    v_now = float(np.interp(s0, _S_GRID, _V_GRID))
    t_free, lead = obstacle_release(s0, t0, v0)
    if not BLOCK_OBS:
        phase = "回缝" if s0 >= REPLAN_S else "绕行"
    elif t0 < t_free and s0 < S_OBS_LO:
        phase = "等待障碍"
    elif s0 < S_OBS_HI:
        phase = "通过障碍"
    else:
        phase = "去停车点"
    print(
        "REPLAN t=%.1f s=%.2f v=%.2f 提前=%.1f 放开=%.1f %s"
        % (t0, s0, v_now, lead, t_free, phase),
        flush=True,
    )


_S_GRID = np.array([0.0, STOP_S])
_V_GRID = np.array([0.0, 0.0])

_SU = PERIOD * np.tril(np.ones((N_LONG, N_LONG)))
_H_LONG = Q_S * (_SU.T @ _SU) + np.eye(N_LONG) * Q_V
_D_LONG = np.eye(N_LONG)
_D_LONG[1:, :-1] -= np.eye(N_LONG - 1)
_H_LONG = _H_LONG + R_DV * (_D_LONG.T @ _D_LONG)


def bcc(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def frame(vx, omega):
    def counts(value):
        return max(-32768, min(32767, int(round(float(value) * 1000.0))))
    payload = b"\x7b\x00\x00" + struct.pack(">hhh", counts(vx), 0, counts(omega))
    return payload + bytes((bcc(payload), 0x7D))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def open_camera():
    fd = os.open(CAM, os.O_RDWR)
    for cid, val in ((0x009A0901, 1), (0x009A0902, 50)):
        buf = bytearray(struct.pack("Ii", cid, val))
        fcntl.ioctl(fd, 0xC008561C, buf, True)
    fmt = bytearray(208)
    struct.pack_into("I", fmt, 0, 1)
    struct.pack_into("I", fmt, 4, 640)
    struct.pack_into("I", fmt, 8, 480)
    struct.pack_into("I", fmt, 12, 0x47504A4D)
    struct.pack_into("I", fmt, 16, 1)
    fcntl.ioctl(fd, 0xC0D05605, fmt, True)
    req = bytearray(20)
    struct.pack_into("III", req, 0, 1, 1, 1)
    fcntl.ioctl(fd, 0xC0145608, req, True)
    info = bytearray(88)
    struct.pack_into("II", info, 0, 0, 1)
    struct.pack_into("I", info, 60, 1)
    fcntl.ioctl(fd, 0xC0585609, info, True)
    length = struct.unpack_from("I", info, 72)[0]
    offset = struct.unpack_from("I", info, 64)[0]
    if length < 1000:
        raise RuntimeError("bad camera buffer %s" % length)
    mem = mmap.mmap(fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
    q = bytearray(88)
    struct.pack_into("II", q, 0, 0, 1)
    struct.pack_into("I", q, 60, 1)
    fcntl.ioctl(fd, 0xC058560F, q)
    fcntl.ioctl(fd, 0x40045612, struct.pack("I", 1))
    return fd, mem


def grab(cam_fd, mem):
    ready, _, _ = select.select([cam_fd], [], [], 0.02)
    if not ready:
        return None
    dq = bytearray(88)
    struct.pack_into("II", dq, 0, 0, 1)
    struct.pack_into("I", dq, 60, 1)
    fcntl.ioctl(cam_fd, 0xC0585611, dq, True)
    used = struct.unpack_from("I", dq, 8)[0]
    blob = bytes(mem[:used])
    q = bytearray(88)
    struct.pack_into("II", q, 0, 0, 1)
    struct.pack_into("I", q, 60, 1)
    fcntl.ioctl(cam_fd, 0xC058560F, q)
    if used < 4 or blob[0] != 0xFF or blob[1] != 0xD8:
        return None
    return blob


def ahead_contrast(blob):
    image = Image.open(BytesIO(blob)).convert("L")
    pixels = image.load()

    def mean(row0, row1, col0, col1):
        total = 0
        count = 0
        for row in range(row0, row1, 2):
            for col in range(col0, col1, 2):
                total += pixels[col, row]
                count += 1
        if count == 0:
            return 255.0
        return total / float(count)

    floor = mean(400, 460, 260, 400)
    block = mean(270, 360, 260, 400)
    return floor - block, block


def open_port():
    import fcntl
    import stat
    fd = os.open(DEVICE, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        if not stat.S_ISCHR(os.fstat(fd).st_mode):
            raise RuntimeError("not a character device")
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attr = termios.tcgetattr(fd)
        attr[6] = list(attr[6])
        attr[0] = attr[1] = attr[3] = 0
        attr[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[4] = attr[5] = termios.B115200
        attr[6][termios.VMIN] = attr[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        return fd
    except BaseException:
        os.close(fd)
        raise


def read_frames(fd, buf):
    frames = []
    while True:
        ready, _, _ = select.select([fd], [], [], 0)
        if not ready:
            break
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            break
        if not chunk:
            break
        buf.extend(chunk)
        while True:
            start = buf.find(b"\x7b")
            if start < 0:
                if len(buf) > 80:
                    del buf[:-24]
                break
            if start:
                del buf[:start]
            if len(buf) < 24:
                break
            item = bytes(buf[:24])
            if item[-1] != 0x7D or bcc(item[:22]) != item[22]:
                del buf[0]
                continue
            del buf[:24]
            vals = [int.from_bytes(item[2 + 2 * k:4 + 2 * k], "big", signed=True) for k in range(10)]
            frames.append((vals[0] / 1000.0, vals[2] / 1000.0, vals[9] / 1000.0))
    return frames


def zeros(fd, seconds):
    end = time.monotonic() + seconds
    nxt = time.monotonic()
    packet = frame(0.0, 0.0)
    while time.monotonic() < end:
        if time.monotonic() < nxt:
            time.sleep(min(0.01, nxt - time.monotonic()))
            continue
        os.write(fd, packet)
        nxt += PERIOD


def solve_box(H, f, lo, hi):
    u = np.linalg.solve(H + 1e-8 * np.eye(H.shape[0]), -f)
    if np.all((u >= lo - 1e-8) & (u <= hi + 1e-8)):
        return np.clip(u, lo, hi)
    u = np.clip(u, lo, hi)
    scale = 1.0 / max(float(np.linalg.norm(H, ord=2)), 1e-6)
    for _ in range(24):
        u = np.clip(u - scale * (H @ u + f), lo, hi)
    return u


def project_rate(values, previous, lo, hi, step):
    out = np.empty_like(values)
    prev = previous
    for i, value in enumerate(values):
        out[i] = min(hi, max(lo, min(prev + step, max(prev - step, value))))
        prev = out[i]
    return out


class Quintic:
    def __init__(self, x0, v0, a0, x1, v1, a1, horizon):
        self.c0, self.c1, self.c2 = x0, v0, a0 / 2.0
        coeff = np.array([
            [horizon ** 3, horizon ** 4, horizon ** 5],
            [3 * horizon ** 2, 4 * horizon ** 3, 5 * horizon ** 4],
            [6 * horizon, 12 * horizon ** 2, 20 * horizon ** 3],
        ])
        rhs = np.array([
            x1 - (self.c0 + self.c1 * horizon + self.c2 * horizon ** 2),
            v1 - (self.c1 + 2 * self.c2 * horizon),
            a1 - 2 * self.c2,
        ])
        self.c3, self.c4, self.c5 = np.linalg.solve(coeff, rhs)

    def position(self, t):
        return self.c0 + self.c1 * t + self.c2 * t ** 2 + self.c3 * t ** 3 + self.c4 * t ** 4 + self.c5 * t ** 5

    def velocity(self, t):
        return self.c1 + 2 * self.c2 * t + 3 * self.c3 * t ** 2 + 4 * self.c4 * t ** 3 + 5 * self.c5 * t ** 4

    def jerk(self, t):
        return 6 * self.c3 + 24 * self.c4 * t + 60 * self.c5 * t ** 2


class Quartic:
    def __init__(self, x0, v0, a0, v1, a1, horizon):
        self.c0, self.c1, self.c2 = x0, v0, a0 / 2.0
        coeff = np.array([[3 * horizon ** 2, 4 * horizon ** 3], [6 * horizon, 12 * horizon ** 2]])
        rhs = np.array([v1 - (self.c1 + 2 * self.c2 * horizon), a1 - 2 * self.c2])
        self.c3, self.c4 = np.linalg.solve(coeff, rhs)

    def position(self, t):
        return self.c0 + self.c1 * t + self.c2 * t ** 2 + self.c3 * t ** 3 + self.c4 * t ** 4

    def velocity(self, t):
        return self.c1 + 2 * self.c2 * t + 3 * self.c3 * t ** 2 + 4 * self.c4 * t ** 3

    def acceleration(self, t):
        return 2 * self.c2 + 6 * self.c3 * t + 12 * self.c4 * t ** 2

    def jerk(self, t):
        return 6 * self.c3 + 24 * self.c4 * t


def spatial_quintic(s0, y0, s1, y1):
    span = s1 - s0
    coeff = np.array([
        [span ** 3, span ** 4, span ** 5],
        [3 * span ** 2, 4 * span ** 3, 5 * span ** 4],
        [6 * span, 12 * span ** 2, 20 * span ** 3],
    ])
    c3, c4, c5 = np.linalg.solve(coeff, np.array([y1 - y0, 0.0, 0.0]))

    def y_at(s):
        if s <= s0:
            return y0
        if s >= s1:
            return y1
        u = s - s0
        return y0 + c3 * u ** 3 + c4 * u ** 4 + c5 * u ** 5

    def slope_at(s):
        if s <= s0 or s >= s1:
            return 0.0
        u = s - s0
        return 3 * c3 * u ** 2 + 4 * c4 * u ** 3 + 5 * c5 * u ** 4

    return y_at, slope_at


def remaining_gap(s0):
    if not CAM_BOX_PRESENT:
        return 1.0
    worst = 1e9
    for s in np.linspace(max(s0, 0.0), 2.2, 80):
        y = float(np.interp(s, _S_PATH, _Y_GROUND))
        for extra in (0.0, 0.08, 0.16):
            worst = min(worst, math.hypot(s + extra - BOX_S, y - BOX_Y))
    return worst


def path_gap(y_at, s0):
    worst = 1e9
    for s in np.linspace(s0, 2.2, 160):
        for extra in (0.0, 0.08, 0.16):
            worst = min(worst, math.hypot(s + extra - BOX_S, y_at(s) - BOX_Y))
    return worst


def lattice_plan(label, s0, y0, targets, settles):
    ranked = []
    for target in targets:
        for settle in settles:
            s1 = min(s0 + settle, 2.12)
            if s1 - s0 < 0.40:
                continue
            y_at, slope_at = spatial_quintic(s0, y0, s1, target)
            gap = path_gap(y_at, s0)
            samples = np.linspace(s0, s1, 40)
            head = max(abs(math.atan(slope_at(s))) for s in samples)
            reasons = []
            if gap < CLEAR:
                reasons.append("碰撞余量%.2f" % gap)
            if head > MAX_HEAD:
                reasons.append("航向%.0f°" % math.degrees(head))
            ranked.append((not reasons, target, settle, gap, head, y_at))
            print(
                "LATTICE %s y=%.2f 收束%.2f 间隙%.2f 航向%.0f° %s"
                % (label, target, settle, gap, math.degrees(head), "可行" if not reasons else "丢弃:" + ",".join(reasons)),
                flush=True,
            )
    feasible = [item for item in ranked if item[0]]
    if not feasible:
        heading_ok = [item for item in ranked if item[4] <= MAX_HEAD]
        if heading_ok:
            chosen = max(heading_ok, key=lambda item: item[3])
            print(
                "LATTICE %s 没有余量候选，改选间隙最大 y=%.2f 间隙%.2f"
                % (label, chosen[1], chosen[3]),
                flush=True,
            )
        else:
            print("LATTICE %s 候选 %d 条，可行 0 条" % (label, len(ranked)), flush=True)
            if label == "回缝":
                s1 = min(max(s0 + 0.55, s0 + 0.40), 2.12)
                if s1 - s0 >= 0.40:
                    y_at, _ = spatial_quintic(s0, y0, s1, RETURN_TARGETS[0])
                    grid = np.linspace(0.0, 2.4, 241)
                    print("LATTICE 回缝强制 y=%.2f 收束%.2fm" % (RETURN_TARGETS[0], s1 - s0), flush=True)
                    return grid, np.array([y_at(s) for s in grid])
            return None
    else:
        if label == "回缝":
            chosen = min(feasible, key=lambda item: (item[4], -item[2], item[1]))
        else:
            chosen = min(feasible, key=lambda item: (item[1], item[4]))
    y_at = chosen[5]
    grid = np.linspace(0.0, 2.4, 241)
    print(
        "LATTICE %s 候选 %d 条，可行 %d 条，选中 y=%.2f 收束%.2fm"
        % (label, len(ranked), len(feasible), chosen[1], chosen[2]),
        flush=True,
    )
    return grid, np.array([y_at(s) for s in grid])


_S_PATH, _Y_GROUND = None, None
PHASE_NAME = "等待"


def install_path(grid, values):
    global _S_PATH, _Y_GROUND
    _S_PATH, _Y_GROUND = grid, values


def commit_straight(s0, y_odom, v0, t0):
    global BLOCK_OBS, CAM_BOX_PRESENT, ACTION, PHASE_NAME, committed, _S_GRID, _V_GRID
    BLOCK_OBS = False
    CAM_BOX_PRESENT = False
    ACTION = "直过"
    PHASE_NAME = "直过"
    committed = True
    print("DECIDE 相机确认缝上没有盒子，贴地砖缝通过。", flush=True)
    y0 = y_odom + GROUND_SHIFT
    s1 = min(max(s0 + 1.20, s0 + 0.50), 1.80)
    y_at, _ = spatial_quintic(s0, y0, s1, 0.05)
    grid = np.linspace(0.0, 2.4, 241)
    install_path(grid, np.array([y_at(s) for s in grid]))
    _S_GRID, _V_GRID = speed_plan(s0, t0, v0, announce=True)


def commit_bypass(s0, y_odom, v0, t0):
    global BLOCK_OBS, ACTION, PHASE_NAME, committed, _S_GRID, _V_GRID
    print("DECIDE 相机看到盒子还在缝上，向左绕行。", flush=True)
    planned = lattice_plan("绕障", s0, y_odom + GROUND_SHIFT, AVOID_TARGETS, AVOID_SETTLES)
    if planned is None:
        print("PATH 严格候选没有，改用最左绕行。", flush=True)
        planned = lattice_plan("绕障", s0, y_odom + GROUND_SHIFT, (0.34, 0.40), (0.45, 0.55))
        if planned is None:
            print("PATH 绕行仍没有轨迹，继续等下一次规划。", flush=True)
            return
    BLOCK_OBS = False
    ACTION = "绕行"
    PHASE_NAME = "绕行"
    committed = True
    install_path(*planned)
    _S_GRID, _V_GRID = speed_plan(s0, t0, v0, announce=True)


def lateral_ref(s):
    y_ground = float(np.interp(s, _S_PATH, _Y_GROUND))
    y_ahead = float(np.interp(s + 0.02, _S_PATH, _Y_GROUND))
    y_odom = y_ground - GROUND_SHIFT
    return y_odom - Y_TARGET, (y_ahead - y_ground) / 0.02


def long_reference(s):
    sref = np.zeros(N_LONG)
    vref = np.zeros(N_LONG)
    cursor = s
    for k in range(N_LONG):
        vref[k] = float(np.interp(cursor, _S_GRID, _V_GRID))
        cursor = min(TARGET, cursor + vref[k] * PERIOD)
        sref[k] = cursor
    return sref, vref


def longitudinal(s, v_prev):
    sref, vref = long_reference(s)
    H = _H_LONG.copy()
    f = Q_S * (_SU.T @ (s - sref)) - Q_V * vref
    f[0] -= R_DV * v_prev
    H[0, 0] += R_DV
    speed = solve_box(H, f, np.zeros(N_LONG), np.full(N_LONG, SPEED))
    speed = project_rate(speed, v_prev, 0.0, SPEED, DV_MAX)
    return float(speed[0]), float(vref[0])


def lateral(cross, heading, speed, steer_prev):
    v = max(speed, 0.05)
    A = np.array([[1.0, v * PERIOD], [0.0, 1.0]])
    B = np.array([[0.0], [-v * PERIOD / WHEELBASE]])
    P = solve_discrete_are(A, B, Q_LAT, np.array([[R_STEER]]))
    n = 2
    Sx = np.zeros((N_LAT * n, n))
    Su = np.zeros((N_LAT * n, N_LAT))
    Ak = np.eye(n)
    for k in range(N_LAT):
        Ak = A @ Ak
        Sx[k * n:(k + 1) * n] = Ak
        for j in range(k + 1):
            Su[k * n:(k + 1) * n, j] = (np.linalg.matrix_power(A, k - j) @ B).ravel()
    Qbar = np.zeros((N_LAT * n, N_LAT * n))
    for k in range(N_LAT):
        Qbar[k * n:(k + 1) * n, k * n:(k + 1) * n] = P if k == N_LAT - 1 else Q_LAT
    H = Su.T @ Qbar @ Su + np.eye(N_LAT) * R_STEER
    D = np.eye(N_LAT)
    D[1:, :-1] -= np.eye(N_LAT - 1)
    H = H + R_DSTEER * (D.T @ D)
    f = np.array(Su.T @ Qbar @ (Sx @ np.array([cross, heading]))).ravel()
    f[0] -= R_DSTEER * steer_prev
    H[0, 0] += R_DSTEER
    steer = solve_box(H, f, np.full(N_LAT, -MAX_STEER), np.full(N_LAT, MAX_STEER))
    return float(steer[0])


def decision(v_ref, v_cmd, vx, omega):
    if v_cmd > vx + 0.03:
        longitudinal_name = "加速"
    elif v_ref < SPEED - 0.02:
        longitudinal_name = "制动"
    else:
        longitudinal_name = "巡航"
    if omega <= -0.02:
        lateral_name = "向右"
    elif omega >= 0.02:
        lateral_name = "向左"
    else:
        lateral_name = "保持"
    return "%s Lattice 纵向MPC%s 横向MPC%s" % (PHASE_NAME, longitudinal_name, lateral_name)


committed = False
hold = np.linspace(0.0, 2.4, 241)
install_path(hold, np.full_like(hold, START_Y))
print("ROS2 Humble：chassis_bridge 只碰串口，lattice_planner 只发/收话题。", flush=True)
print("话题 /cmd_vel = geometry_msgs/Twist ， /odom = nav_msgs/Odometry ， /voltage = std_msgs/Float32", flush=True)
print("行为：相机看缝上 1m 还有没有盒子。有就等待再向左绕。没有才贴缝通过。", flush=True)
print("STATE 列: 决策 x y 航向° 实测速度 规划速度 指令速度 指令角速度 加速度 相对车道横向 电压", flush=True)


class ChassisBridge(Node):
    def __init__(self):
        super().__init__("chassis_bridge")
        self.fd = open_port()
        self.buf = bytearray()
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd = time.monotonic()
        self.x = 0.0
        self.y = Y0
        self.psi = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.volts = 0.0
        self.have = False
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_volt = self.create_publisher(Float32, "voltage", 10)
        self.create_timer(PERIOD, self.tick)
        print("ROS2 节点 chassis_bridge 已起：订 /cmd_vel ，发 /odom /voltage", flush=True)

    def on_cmd(self, msg):
        self.cmd_vx = float(msg.linear.x)
        self.cmd_wz = float(msg.angular.z)
        self.last_cmd = time.monotonic()

    def tick(self):
        frames = read_frames(self.fd, self.buf)
        if frames:
            self.vx, self.wz, self.volts = frames[-1]
            self.have = True
        if not self.have:
            os.write(self.fd, frame(0.0, 0.0))
            return
        self.x += self.vx * math.cos(self.psi) * PERIOD
        self.y += self.vx * math.sin(self.psi) * PERIOD
        self.psi = wrap(self.psi + self.wz * PERIOD)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.z = math.sin(self.psi / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.psi / 2.0)
        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.angular.z = float(self.wz)
        self.pub_odom.publish(odom)
        volt = Float32()
        volt.data = float(self.volts)
        self.pub_volt.publish(volt)
        if time.monotonic() - self.last_cmd > 0.40:
            os.write(self.fd, frame(0.0, 0.0))
        else:
            os.write(self.fd, frame(self.cmd_vx, self.cmd_wz))

    def halt(self):
        print("PHASE zero", flush=True)
        zeros(self.fd, 6.0)
        os.close(self.fd)


class LatticePlanner(Node):
    def __init__(self):
        super().__init__("lattice_planner")
        self.have_odom = False
        self.x = 0.0
        self.y = Y0
        self.psi = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.volts = 0.0
        self.prev_vx = None
        self.v_prev = 0.0
        self.steer_prev = 0.0
        self.samples = 0
        self.origin = time.monotonic()
        self.last_speed_plan = 0.0
        self.last_path_check = 0.0
        self.last_decide = -1.0
        self.last_return = -1.0
        self.last_cam = -1.0
        self.returned = False
        self.behavior = RoomBehavior()
        self.cam_fd = None
        self.cam_mem = None
        self.box_hits = 0
        self.empty_hits = 0
        self.cam_contrast = 0.0
        self.cam_block = 255.0
        self.done = False
        self.reason = "timeout"
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(Float32, "voltage", self.on_volt, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(PERIOD, self.tick)
        print("ROS2 节点 lattice_planner 已起：订 /odom /voltage ，发 /cmd_vel", flush=True)
        if Image is None:
            print("相机 PIL 不可用，按盒子还在处理。", flush=True)
        else:
            self.cam_fd, self.cam_mem = open_camera()
            print("相机已打开 /dev/video2。", flush=True)

    def on_odom(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        z = float(msg.pose.pose.orientation.z)
        w = float(msg.pose.pose.orientation.w)
        self.psi = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        self.vx = float(msg.twist.twist.linear.x)
        self.wz = float(msg.twist.twist.angular.z)
        self.have_odom = True

    def on_volt(self, msg):
        self.volts = float(msg.data)

    def send(self, vx_cmd, wz_cmd):
        msg = Twist()
        msg.linear.x = float(vx_cmd)
        msg.angular.z = float(wz_cmd)
        self.pub_cmd.publish(msg)

    def finish(self, reason):
        self.reason = reason
        self.done = True
        self.send(0.0, 0.0)

    def tick(self):
        global committed, CAM_BOX_PRESENT, empty_hits, ACTION, PHASE_NAME, returned
        if self.done:
            self.send(0.0, 0.0)
            return
        if not self.have_odom:
            self.send(0.0, 0.0)
            if time.monotonic() - self.origin > 2.0:
                self.finish("no_feedback")
            return
        x, y, psi, vx, wz, volts = self.x, self.y, self.psi, self.vx, self.wz, self.volts
        now_t = time.monotonic() - self.origin
        if now_t > 70.0:
            self.finish("timeout")
            return
        if (not committed) and self.cam_fd is not None and now_t - self.last_cam >= 0.25:
            blob = grab(self.cam_fd, self.cam_mem)
            if blob is not None and Image is not None:
                self.cam_contrast, self.cam_block = ahead_contrast(blob)
                dark = self.cam_block < 95.0
                peaked = self.cam_contrast > 25.0 and self.cam_block < 160.0
                empty = self.cam_block > 98.0 and self.cam_contrast < 22.0
                if dark or peaked:
                    self.box_hits += 1
                    self.empty_hits = 0
                elif empty:
                    self.box_hits = 0
                    self.empty_hits += 1
                else:
                    self.box_hits = 0
                    self.empty_hits = 0
                if self.box_hits >= 1:
                    CAM_BOX_PRESENT = True
                elif self.empty_hits >= 4:
                    CAM_BOX_PRESENT = False
                print(
                    "CAM t=%.1f 亮度差=%.0f 前方=%.0f 盒子=%s"
                    % (now_t, self.cam_contrast, self.cam_block, "有" if CAM_BOX_PRESENT else "无"),
                    flush=True,
                )
            self.last_cam = now_t
        if not committed and now_t - self.last_decide >= 0.5:
            action, fsm_state, level, cur_ttc, cur_thw, incentive, left = self.behavior.step(
                x, max(vx, 0.0), now_t, CAM_BOX_PRESENT
            )
            print(
                "DECIDE t=%.1f s=%.2f 决策=%s 状态=%s 风险=%s 相机盒子=%s TTC=%s 激励=%.2f"
                % (
                    now_t, x, action, FSM_NAME[fsm_state], RISK_NAME[level],
                    "有" if CAM_BOX_PRESENT else "无", fmt_time(cur_ttc), incentive,
                ),
                flush=True,
            )
            self.last_decide = now_t
            if action == "绕行":
                commit_bypass(x, y, max(vx, 0.0), now_t)
            elif action == "直过" or (not CAM_BOX_PRESENT and self.empty_hits >= 4):
                commit_straight(x, y, max(vx, 0.0), now_t)
                self.returned = True
        front_y = y + WHEELBASE * math.sin(psi)
        front_s = x + WHEELBASE * math.cos(psi)
        if committed:
            if now_t - self.last_speed_plan >= 0.5:
                refresh_speed(x, now_t, max(vx, 0.0))
                self.last_speed_plan = now_t
            if now_t - self.last_path_check >= 1.0:
                gap = remaining_gap(x)
                y_here = float(np.interp(x, _S_PATH, _Y_GROUND))
                print(
                    "PATH t=%.1f s=%.2f y=%.2f 间隙=%.2f 保持"
                    % (now_t, x, y_here, gap),
                    flush=True,
                )
                self.last_path_check = now_t
            if (not self.returned) and x >= REPLAN_S and now_t - self.last_return >= 0.5:
                self.last_return = now_t
                planned = lattice_plan("回缝", x, y + GROUND_SHIFT, RETURN_TARGETS, RETURN_SETTLES)
                if planned is None:
                    s1 = min(max(x + 0.55, x + 0.40), 2.12)
                    y_at, _ = spatial_quintic(x, y + GROUND_SHIFT, s1, RETURN_TARGETS[0])
                    grid = np.linspace(0.0, 2.4, 241)
                    install_path(grid, np.array([y_at(s) for s in grid]))
                    print("PATH 回缝强制落到 y=%.2f" % RETURN_TARGETS[0], flush=True)
                    self.returned = True
                    PHASE_NAME = "回缝"
                else:
                    install_path(*planned)
                    self.returned = True
                    PHASE_NAME = "回缝"
            d_plan, d_slope = lateral_ref(front_s)
            y_ref = Y_TARGET + d_plan
            psi_ref = math.atan(d_slope)
            v_cmd, v_ref = longitudinal(x, self.v_prev)
            cross = -(front_y - y_ref)
            steer = lateral(cross, wrap(-(psi - psi_ref)), max(v_cmd, vx, 0.0), self.steer_prev)
            omega = 0.0
            if x >= STOP_S - 0.02 or (x > 1.94 and max(vx, 0.0) < 0.02 and v_cmd < 0.03):
                print("STOP distance", flush=True)
                self.finish("distance")
                return
            if v_cmd >= 0.04:
                omega = v_cmd * math.tan(steer) / WHEELBASE
                if ACTION == "直过":
                    omega = max(-0.14, min(0.14, omega))
                elif self.returned:
                    omega = max(-0.12, min(0.12, omega))
                    if psi < -math.radians(12.0) and omega < 0.0:
                        omega = 0.0
                    if psi > math.radians(12.0) and omega > 0.0:
                        omega = 0.0
                else:
                    omega = max(-MAX_YAW, min(MAX_YAW, omega))
            state_name = decision(v_ref, v_cmd, vx, omega)
        else:
            gap = BOX_S - x if CAM_BOX_PRESENT else None
            a_cmd = idm_accel(max(vx, 0.0), gap)
            v_cmd = min(SPEED, max(0.0, max(vx, 0.0) + a_cmd * PERIOD))
            if CAM_BOX_PRESENT and x >= HOLD_S:
                v_cmd = 0.0
            v_ref = v_cmd
            omega = 0.0 if v_cmd < 0.04 else max(-MAX_YAW, min(MAX_YAW, -1.5 * psi))
            steer = 0.0
            cross = front_y - (START_Y - GROUND_SHIFT)
            state_name = "等待 相机%s" % ("有盒子" if CAM_BOX_PRESENT else "无盒子")
        if abs(front_y) > 0.60 or abs(psi) > 0.85:
            print("STOP bounds", flush=True)
            self.finish("bounds")
            return
        self.send(v_cmd, omega)
        self.v_prev = v_cmd
        self.steer_prev = steer
        self.samples += 1
        accel = 0.0 if self.prev_vx is None else (vx - self.prev_vx) / PERIOD
        self.prev_vx = vx
        if self.samples % 10 == 0:
            print(
                "STATE %s x=%.3f y=%.3f psi=%.1f v=%.3f v_ref=%.2f v_cmd=%.2f w_cmd=%.2f a=%.2f cross=%.3f V=%.2f  ROS2 /cmd_vel vx=%.2f wz=%.2f"
                % (
                    state_name,
                    x, y, psi * 180.0 / math.pi, vx, v_ref, v_cmd, omega, accel,
                    cross, volts, v_cmd, omega,
                ),
                flush=True,
            )

    def close_camera(self):
        if self.cam_mem is not None:
            self.cam_mem.close()
        if self.cam_fd is not None:
            try:
                fcntl.ioctl(self.cam_fd, 0x40045613, struct.pack("I", 1))
            except Exception:
                pass
            os.close(self.cam_fd)


rclpy.init()
bridge = ChassisBridge()
planner = LatticePlanner()
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(bridge)
executor.add_node(planner)
try:
    while rclpy.ok() and not planner.done:
        executor.spin_once(timeout_sec=0.05)
except Exception as exc:
    planner.reason = "error"
    print("ERROR", exc, flush=True)
finally:
    try:
        planner.send(0.0, 0.0)
        time.sleep(0.1)
        planner.close_camera()
    except Exception:
        pass
    try:
        bridge.halt()
    except Exception as exc:
        print("ZERO_ERR", exc, flush=True)
    planner.destroy_node()
    bridge.destroy_node()
    rclpy.shutdown()
print(
    "SUMMARY {\"reason\":\"%s\",\"x_m\":%.3f,\"y_m\":%.3f,\"psi_deg\":%.1f,\"voltage_V\":%.3f,\"samples\":%d,\"ros2\":true}"
    % (
        planner.reason, planner.x, planner.y, planner.psi * 180.0 / math.pi,
        planner.volts, planner.samples,
    ),
    flush=True,
)
print("ZERO_DONE", flush=True)
