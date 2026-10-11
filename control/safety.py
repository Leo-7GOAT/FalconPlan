"""Latched software safety at the controller/HAL boundary. All times are monotonic seconds."""
from dataclasses import dataclass
from enum import Enum
import math
from vehicle_model.models import VehicleState, VehicleCommand

class SafetyMode(str, Enum):
    ACTIVE = "active"
    STOPPED = "stopped"
    MANUAL = "manual"

@dataclass(frozen=True)
class SafetyConfig:
    max_state_age: float = 0.2
    max_command_age: float = 0.2
    max_speed: float = 20.0
    max_steer: float = 0.5
    max_steer_rate: float = 0.8
    max_accel: float = 3.0
    max_decel: float = 6.0
    geofence: tuple[float, float, float, float] = (-10000, 10000, -10000, 10000)
    def __post_init__(self):
        for key, value in vars(self).items():
            if key != "geofence" and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{key} must be positive finite")
        if len(self.geofence) != 4 or not all(map(math.isfinite, self.geofence)):
            raise ValueError("invalid geofence")
        x0,x1,y0,y1=self.geofence
        if x0 >= x1 or y0 >= y1: raise ValueError("invalid geofence ordering")

@dataclass(frozen=True)
class SafetyResult:
    command: VehicleCommand
    mode: SafetyMode
    reasons: tuple[str, ...]

class SafetySupervisor:
    def __init__(self, config: SafetyConfig = SafetyConfig()):
        self.config=config
        self.mode=SafetyMode.ACTIVE
        self.reason=""
        self.last_steer=0.0
        self.interventions=0

    def reset(self, state: VehicleState, *, explicit: bool = False):
        x0,x1,y0,y1=self.config.geofence
        if not explicit or abs(state.v) > 0.1 or not (x0 <= state.x <= x1 and y0 <= state.y <= y1): raise ValueError("reset needs explicit acknowledgement, stationary vehicle and valid geofence")
        self.mode=SafetyMode.ACTIVE
        self.reason=""
        self.last_steer=0.0

    def evaluate(self, state: VehicleState, command: VehicleCommand, *, now: float,
                 state_time: float, command_time: float, dt: float,
                 emergency_stop: bool = False, manual: VehicleCommand | None = None,
                 collision: bool = False) -> SafetyResult:
        if not math.isfinite(dt) or dt <= 0: raise ValueError("invalid dt")
        cfg=self.config
        reasons=[]
        for label, stamp, limit in (("state",state_time,cfg.max_state_age),("command",command_time,cfg.max_command_age)):
            if not all(map(math.isfinite,(now,stamp))) or stamp > now+1e-6 or now-stamp > limit:
                reasons.append(label+"_timeout")
        x0,x1,y0,y1=cfg.geofence
        if not x0 <= state.x <= x1 or not y0 <= state.y <= y1: reasons.append("geofence")
        if state.v > cfg.max_speed: reasons.append("overspeed")
        if state.v < -.1: reasons.append("unsupported_reverse")
        if emergency_stop: reasons.append("software_estop")
        if collision: reasons.append("collision")
        if reasons:
            self.mode=SafetyMode.STOPPED
            self.reason=",".join(reasons)
        if self.mode == SafetyMode.STOPPED:
            self.last_steer=0.0
            self.interventions+=1
            return SafetyResult(VehicleCommand(0.0,-cfg.max_decel),self.mode,tuple(reasons) or (self.reason,))
        requested=manual if manual is not None else command
        self.mode=SafetyMode.MANUAL if manual is not None else SafetyMode.ACTIVE
        steer=max(-cfg.max_steer,min(cfg.max_steer,requested.delta))
        steer=max(self.last_steer-cfg.max_steer_rate*dt,min(self.last_steer+cfg.max_steer_rate*dt,steer))
        accel=max(-cfg.max_decel,min(cfg.max_accel,requested.a))
        if steer != requested.delta or accel != requested.a:
            reasons.append("command_limited");self.interventions+=1
        self.last_steer=steer
        return SafetyResult(VehicleCommand(steer,accel),self.mode,tuple(reasons))
