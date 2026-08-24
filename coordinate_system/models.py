from dataclasses import dataclass


@dataclass(frozen=True)
class ReferenceState:
    x: float
    y: float
    theta: float
    kappa: float


@dataclass(frozen=True)
class WorldState:
    x: float
    y: float
    theta: float

@dataclass(frozen=True)
class WorldPoint:
    x: float
    y: float

@dataclass(frozen=True)
class BodyPoint:
    x: float
    y: float


@dataclass(frozen=True)
class FrenetState:
    s: float
    d: float
    d_prime: float
