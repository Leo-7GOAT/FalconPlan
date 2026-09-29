from dataclasses import dataclass


@dataclass(frozen=True)
class STGridNode:
    time_index: int
    distance_index: int
    t: float
    s: float


@dataclass(frozen=True)
class STGrid:
    times: tuple[float, ...]
    distances: tuple[float, ...]

    def __post_init__(self):
        if len(self.times) == 0:
            raise ValueError("STGrid times must not be empty")
        if len(self.distances) == 0:
            raise ValueError("STGrid distances must not be empty")
        for i in range(len(self.times) - 1):
            if self.times[i + 1] <= self.times[i]:
                raise ValueError("STGrid times must be strictly increasing")
        for i in range(len(self.distances) - 1):
            if self.distances[i + 1] <= self.distances[i]:
                raise ValueError("STGrid distances must be strictly increasing")

    @property
    def num_time_steps(self) -> int:
        return len(self.times)

    @property
    def num_distance_steps(self) -> int:
        return len(self.distances)

    @property
    def shape(self) -> tuple[int, int]:
        return self.num_time_steps, self.num_distance_steps

    def node(
        self,
        time_index: int,
        distance_index: int,
    ) -> STGridNode:
        if not 0 <= time_index < self.num_time_steps:
            raise IndexError("time_index out of range")
        if not 0 <= distance_index < self.num_distance_steps:
            raise IndexError("distance_index out of range")
        return STGridNode(
            time_index=time_index,
            distance_index=distance_index,
            t=self.times[time_index],
            s=self.distances[distance_index],
        )


def build_st_grid(
    horizon: float,
    dt: float,
    max_s: float,
    ds: float,
) -> STGrid:
    if horizon <= 0.0:
        raise ValueError("horizon must be positive")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if max_s <= 0.0:
        raise ValueError("max_s must be positive")
    if ds <= 0.0:
        raise ValueError("ds must be positive")

    times = []
    distances = []

    t = 0.0
    while t <= horizon + 1e-12:
        times.append(float(t))
        t += dt

    s = 0.0
    while s <= max_s + 1e-12:
        distances.append(float(s))
        s += ds

    return STGrid(
        times=tuple(times),
        distances=tuple(distances),
    )
