"""Planar single-track dynamics with friction-limited lateral tire forces (SI).

This analytical model is for model-error experiments; it is not CARLA physics.
Low speed uses kinematic yaw to avoid division singularities.
"""
from dataclasses import dataclass
import math
import numpy as np
from .models import VehicleCommand, VehicleState

@dataclass(frozen=True)
class DynamicsParams:
    mass: float = 1500.0
    inertia: float = 2500.0
    lf: float = 1.2
    lr: float = 1.6
    cf: float = 70000.0
    cr: float = 70000.0
    friction: float = 0.9
    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError("dynamics parameters must be positive finite")

@dataclass(frozen=True)
class DynamicState:
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    v: float = 0.0
    lateral_v: float = 0.0
    yaw_rate: float = 0.0
    def __post_init__(self):
        if any(not math.isfinite(v) for v in vars(self).values()): raise ValueError("state must be finite")
    def vehicle_state(self): return VehicleState(self.x,self.y,self.psi,self.v)

class DynamicBicycleModel:
    def __init__(self, params: DynamicsParams = DynamicsParams()): self.params=params
    def derivative(self, state: DynamicState, command: VehicleCommand):
        p=self.params;v=state.v;r=state.yaw_rate;vy=state.lateral_v
        if v < 1.0:
            return np.array([v*math.cos(state.psi),v*math.sin(state.psi),v*math.tan(command.delta)/(p.lf+p.lr),command.a,-vy*5,-r*5])
        front= p.cf*(command.delta-math.atan2(vy+p.lf*r,v))
        rear= p.cr*(-math.atan2(vy-p.lr*r,v))
        front=np.clip(front,-p.friction*p.mass*9.81*p.lr/(p.lf+p.lr),p.friction*p.mass*9.81*p.lr/(p.lf+p.lr))
        rear=np.clip(rear,-p.friction*p.mass*9.81*p.lf/(p.lf+p.lr),p.friction*p.mass*9.81*p.lf/(p.lf+p.lr))
        return np.array([v*math.cos(state.psi)-vy*math.sin(state.psi),v*math.sin(state.psi)+vy*math.cos(state.psi),r,command.a,(front*math.cos(command.delta)+rear)/p.mass-v*r,(p.lf*front*math.cos(command.delta)-p.lr*rear)/p.inertia])
    def step(self,state: DynamicState,command: VehicleCommand,dt: float):
        if not math.isfinite(dt) or not 0 < dt <= 0.1: raise ValueError("dt must be in (0,.1]")
        # RK4 with <= 10 ms internal steps resolves stiff low-speed tire dynamics.
        n=math.ceil(dt/.01);h=dt/n;q=np.array(list(vars(state).values()))
        for _ in range(n):
            f=lambda v:self.derivative(DynamicState(*v),command)
            k1=f(q);k2=f(q+h*k1/2);k3=f(q+h*k2/2);k4=f(q+h*k3)
            q += h*(k1+2*k2+2*k3+k4)/6;q[3]=max(0,q[3])
        return DynamicState(*map(float,q))
    def lateral_eigenvalues(self,speed: float):
        if speed <= 0 or not math.isfinite(speed): raise ValueError("positive speed required")
        p=self.params
        A=np.array([[-(p.cf+p.cr)/(p.mass*speed),(-p.cf*p.lf+p.cr*p.lr)/(p.mass*speed)-speed],
                    [(-p.cf*p.lf+p.cr*p.lr)/(p.inertia*speed),-(p.cf*p.lf**2+p.cr*p.lr**2)/(p.inertia*speed)]])
        return np.linalg.eigvals(A)
