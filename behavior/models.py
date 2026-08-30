from enum import Enum
from dataclasses import dataclass

class RiskLevel(Enum):
    SAFE = 0
    CAUTION = 1
    DANGER = 2
    EMERGENCY = 3

@dataclass(frozen=True)
class PredictedState:
    s: float
    v: float

@dataclass(frozen=True)
class TimedPredictedState:
    t: float
    state: PredictedState

@dataclass(frozen=True)
class PredictedRiskState:
    t: float
    gap: float
    ttc: float
    thw: float
    risk: RiskLevel

class BehaviorState(Enum):
    KEEP_LANE = "keep_lane"
    FOLLOW = "follow"

    PREPARE_LANE_CHANGE_LEFT = "prepare_lane_change_left"
    PREPARE_LANE_CHANGE_RIGHT = "prepare_lane_change_right"

    LANE_CHANGE_LEFT = "lane_change_left"
    LANE_CHANGE_RIGHT = "lane_change_right"

    EMERGENCY_BRAKE = "emergency_brake"

@dataclass(frozen=True)
class BehaviorInput:
    risk: RiskLevel

    front_vehicle_present: bool

    left_lane_available: bool
    right_lane_available: bool

    lane_change_completed: bool = False

@dataclass(frozen=True)
class IDMParams:
    desired_speed: float
    max_accel: float
    comfortable_decel: float
    min_gap: float
    time_headway: float
    delta: float = 4.0

@dataclass(frozen=True)
class MOBILParams:
    politeness: float
    safe_braking: float
    incentive_threshold: float

@dataclass(frozen=True)
class LaneChangeEvaluation:
    safe: bool
    incentive: float
    beneficial: bool

@dataclass(frozen=True)
class LongitudinalVehicleState:
    s: float
    v: float

@dataclass(frozen=True)
class BehaviorPlanResult:
    state: BehaviorState

    # 当前瞬时风险
    current_risk: RiskLevel

    # 考虑预测时域后的最终风险
    risk: RiskLevel

    ttc: float
    thw: float

    # IDM 给出的纵向加速度需求
    idm_acceleration: float

    left_eval: LaneChangeEvaluation | None
    right_eval: LaneChangeEvaluation | None

    risk_horizon: tuple[PredictedRiskState, ...]