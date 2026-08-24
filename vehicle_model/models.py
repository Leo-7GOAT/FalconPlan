import math
from dataclasses import dataclass

@dataclass
class VehicleState:
    x: float
    y: float
    psi: float
    v: float

@dataclass
class VehicleCommand:
    delta: float
    a: float

@dataclass
class VehicleParams:
    wheel_base: float

    max_steer:float#rad
    max_accel:float# m/s^2
    max_decel:float# 正数，例如 6.0 代表最大减速度 -6 m/s^2
    max_speed:float# m/s
    min_speed:float = 0.0
