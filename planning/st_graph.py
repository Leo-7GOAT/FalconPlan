from dataclasses import dataclass


@dataclass(frozen=True)
class STPoint:
    t: float
    s: float


@dataclass(frozen=True)
class STBoundary:
    obstacle_id: str
    times: tuple[float, ...]
    lower_s: tuple[float, ...]
    upper_s: tuple[float, ...]

    def __post_init__(self):
        if len(self.times) == 0:
            raise ValueError("STBoundary must not be empty")
        if not (
            len(self.times)
            == len(self.lower_s)
            == len(self.upper_s)
        ):
            raise ValueError(
                "times, lower_s and upper_s must have the same length"
            )
        for i in range(len(self.times) - 1):
            if self.times[i + 1] <= self.times[i]:
                raise ValueError("times must be strictly increasing")
        for lower, upper in zip(self.lower_s, self.upper_s):
            if lower > upper:
                raise ValueError("lower_s must not exceed upper_s")

    def boundary_at(
        self,
        t: float,
    ) -> tuple[float, float] | None:
        t = float(t)
        if t < self.times[0] or t > self.times[-1]:
            return None
        if t == self.times[-1]:
            return self.lower_s[-1], self.upper_s[-1]
        for i in range(len(self.times) - 1):
            t0 = self.times[i]
            t1 = self.times[i + 1]
            if t0 <= t <= t1:
                alpha = (t - t0) / (t1 - t0)
                lower = self.lower_s[i] + alpha * (
                    self.lower_s[i + 1] - self.lower_s[i]
                )
                upper = self.upper_s[i] + alpha * (
                    self.upper_s[i + 1] - self.upper_s[i]
                )
                return float(lower), float(upper)
        raise RuntimeError("failed to interpolate STBoundary")


@dataclass(frozen=True)
class DynamicObstacle:
    obstacle_id: str
    s0: float
    speed: float
    length: float
    safety_margin: float

    def __post_init__(self):
        if self.length <= 0.0:
            raise ValueError("obstacle length must be positive")
        if self.safety_margin < 0.0:
            raise ValueError("safety_margin must be non-negative")


def build_constant_velocity_st_boundary(
    obstacle: DynamicObstacle,
    horizon: float,
    dt: float,
) -> STBoundary:
    if horizon <= 0.0:
        raise ValueError("horizon must be positive")
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    times = []
    lower_s = []
    upper_s = []
    half_extent = 0.5 * obstacle.length + obstacle.safety_margin

    t = 0.0
    while t <= horizon + 1e-12:
        center_s = obstacle.s0 + obstacle.speed * t
        times.append(float(t))
        lower_s.append(float(center_s - half_extent))
        upper_s.append(float(center_s + half_extent))
        t += dt

    return STBoundary(
        obstacle_id=obstacle.obstacle_id,
        times=tuple(times),
        lower_s=tuple(lower_s),
        upper_s=tuple(upper_s),
    )
