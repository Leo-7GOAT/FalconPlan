import math
from dataclasses import dataclass


def _require_finite(**values: float) -> None:
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class VehicleState:
    """Rear-axle vehicle state in the world frame (m, rad, m/s)."""

    x: float
    y: float
    psi: float
    v: float

    def __post_init__(self) -> None:
        _require_finite(x=self.x, y=self.y, psi=self.psi, v=self.v)


@dataclass(frozen=True)
class VehicleCommand:
    """Road-wheel steering angle (rad) and acceleration (m/s^2)."""

    delta: float
    a: float

    def __post_init__(self) -> None:
        _require_finite(delta=self.delta, a=self.a)


@dataclass(frozen=True)
class VehicleParams:
    wheel_base: float
    max_steer: float  # rad
    max_accel: float  # m/s^2
    max_decel: float  # positive magnitude in m/s^2
    max_speed: float  # m/s
    min_speed: float = 0.0  # m/s

    def __post_init__(self) -> None:
        _require_finite(
            wheel_base=self.wheel_base,
            max_steer=self.max_steer,
            max_accel=self.max_accel,
            max_decel=self.max_decel,
            max_speed=self.max_speed,
            min_speed=self.min_speed,
        )
        if self.wheel_base <= 0.0:
            raise ValueError("wheel_base must be greater than 0")
        if not 0.0 < self.max_steer < math.pi / 2:
            raise ValueError("max_steer must be in (0, pi/2)")
        if self.max_accel <= 0.0 or self.max_decel <= 0.0:
            raise ValueError("acceleration limits must be greater than 0")
        if self.min_speed < 0.0 or self.max_speed < self.min_speed:
            raise ValueError("speed limits must satisfy 0 <= min_speed <= max_speed")
