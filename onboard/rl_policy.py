"""Onboard RL policy for the R550.

Trained offline on a 50 ms bicycle-like model of this car (cross-entropy /
elite average, not PPO). On the Pi this only does inference: 5 numbers in,
vx and omega out, then the same /cmd_vel path as the lattice demo.
"""
import math
import numpy as np

SPEED = 0.15
MAX_YAW = 0.30
STOP = 1.90
BOX_S = 1.00

W1 = np.array([
    [0.1578, 0.5913, -0.3257, 0.0017, -0.7008],
    [-0.5574, -0.0613, -0.1265, 0.1136, 0.1236],
    [0.7262, 0.3269, 0.1782, 0.3471, 0.2675],
    [0.1119, -0.2619, -0.1785, -0.4377, 0.4513],
    [0.5987, 0.2585, -0.1849, 0.1405, -0.3644],
    [-0.4600, -0.3532, -0.2582, 0.0684, 1.1328],
    [1.4593, -0.2724, -0.4044, 0.4442, -0.7367],
    [0.8116, 0.6700, 0.5559, -0.2248, 0.0043],
])
B1 = np.array([-0.6961, 0.2025, 0.0674, -0.0814, 0.5410, -0.3028, -0.4009, -0.5690])
W2 = np.array([
    [-0.2330, 0.3632, 0.1099, -0.5514, 0.6255, -0.4464, -0.6396, 0.1642],
    [-0.5082, 0.2174, -0.4235, -0.2244, -0.4120, 0.0382, -1.3434, -0.5552],
])
B2 = np.array([1.1483, -0.0672])


def reference(s):
    if s < 0.95:
        blend = (s / 0.95) ** 2 * (3.0 - 2.0 * s / 0.95)
        return 0.15 + 0.19 * blend
    if s < 1.05:
        return 0.34
    if s < STOP:
        blend = ((s - 1.05) / 0.85) ** 2 * (3.0 - 2.0 * (s - 1.05) / 0.85)
        return 0.34 + (0.18 - 0.34) * blend
    return 0.18


def policy(y_ground, psi, v, x):
    remain = STOP - x
    obs = np.array([
        (y_ground - reference(x)) / 0.1,
        psi / 0.3,
        v / SPEED,
        remain / STOP,
        (BOX_S - x) / STOP,
    ])
    action = np.tanh(W2 @ np.tanh(W1 @ obs + B1) + B2)
    v_cmd = SPEED * (float(action[0]) + 1.0) * 0.5
    omega = 0.0 if v_cmd < 0.04 else MAX_YAW * float(action[1])
    return max(0.0, min(SPEED, v_cmd)), max(-MAX_YAW, min(MAX_YAW, omega))
