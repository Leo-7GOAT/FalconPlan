import math

from .models import (
    RiskLevel,
    TimedPredictedState,
    PredictedRiskState,
)


def compute_ttc(
    gap: float,
    ego_speed: float,
    front_speed: float
) -> float:
    closing_speed = ego_speed - front_speed
    if gap <= 0:
        ttc = 0
    elif closing_speed <= 0:
        ttc = math.inf
    else:
        ttc = gap / closing_speed
    return ttc

def compute_thw(
    gap: float,
    ego_speed: float,
) -> float:
    if gap <= 0:
        thw = 0
    elif ego_speed <= 0:
        thw = math.inf
    else:
        thw = gap / ego_speed
    return thw

def classify_ttc(
        ttc: float,
) -> RiskLevel:
    if ttc <= 1.5:
        return RiskLevel.EMERGENCY
    elif ttc <= 3:
        return RiskLevel.DANGER
    elif ttc <= 6:
        return RiskLevel.CAUTION
    else:
        return RiskLevel.SAFE

def classify_thw(
        thw: float,
) -> RiskLevel:
    if thw <= 0.8:
        return RiskLevel.EMERGENCY
    elif thw <= 1.2:
        return RiskLevel.DANGER
    elif thw <= 2:
        return RiskLevel.CAUTION
    else:
        return RiskLevel.SAFE

def assess_risk(
        ttc: float,
        thw: float,
) -> RiskLevel:

    ttc_risk = classify_ttc(ttc)
    thw_risk = classify_thw(thw)

    return max(
        ttc_risk,
        thw_risk,
        key=lambda risk: risk.value
    )

def compute_risk_horizon(
        ego_predictions: list[TimedPredictedState],
        front_predictions: list[TimedPredictedState]
) -> list[PredictedRiskState]:

    if len(ego_predictions) != len(front_predictions):
        raise ValueError("prediction horizons must have the same length")

    risk_horizon = []

    for ego_prediction, front_prediction in zip(ego_predictions, front_predictions):
        if abs(ego_prediction.t - front_prediction.t) > 1e-9:
            raise ValueError("the prediction horizons must have the same time")

        gap = (front_prediction.state.s - ego_prediction.state.s)

        ttc = compute_ttc(gap, ego_prediction.state.v, front_prediction.state.v)

        thw = compute_thw(gap, ego_prediction.state.v)

        risk = assess_risk(ttc, thw)

        risk_state = PredictedRiskState(
            t=ego_prediction.t,
            gap=gap,
            ttc=ttc,
            thw=thw,
            risk=risk
        )
        risk_horizon.append(risk_state)
    return risk_horizon