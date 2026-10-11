"""Generic delayed, rate-limited first-order actuator for simulation experiments."""
from collections import deque
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class ActuatorConfig:
    minimum: float
    maximum: float
    rate: float
    time_constant: float = .1
    delay: float = 0.0
    deadzone: float = 0.0
    def __post_init__(self):
        if any(not math.isfinite(v) for v in vars(self).values()): raise ValueError("finite parameters required")
        if self.minimum >= self.maximum or self.rate <= 0 or self.time_constant <= 0 or self.delay < 0 or self.deadzone < 0: raise ValueError("invalid actuator limits")

class FirstOrderActuator:
    def __init__(self, config: ActuatorConfig):
        self.config=config;self.value=0.0;self.time=0.0;self.queue=deque();self.target=0.0
    def update(self, command: float,dt: float):
        if not math.isfinite(command) or not math.isfinite(dt) or dt <= 0: raise ValueError("finite command and positive dt required")
        p=self.config;self.queue.append((self.time+p.delay,command))
        while self.queue and self.queue[0][0] <= self.time+1e-10: _,self.target=self.queue.popleft()
        target=max(p.minimum,min(p.maximum,self.target))
        if abs(target) <= p.deadzone: target=0.0
        change=(target-self.value)*(-math.expm1(-dt/p.time_constant))
        self.value += max(-p.rate*dt,min(p.rate*dt,change));self.time += dt
        return self.value
