from behavior.models import (
    BehaviorInput,
    RiskLevel,
)

from behavior.fsm import BehaviorFSM


fsm = BehaviorFSM()
print("Initial:", fsm.states)

state = fsm.update(
    BehaviorInput(
        risk=RiskLevel.CAUTION,
        front_vehicle_present=True,
        left_lane_available=False,
        right_lane_available=False,
        lane_change_completed=False,
    )
)

print("1:", state)

state = fsm.update(
    BehaviorInput(
        risk=RiskLevel.CAUTION,
        front_vehicle_present=True,
        left_lane_available=True,
        right_lane_available=False,
        lane_change_completed=False,
    )
)

print("2:", state)

state = fsm.update(
    BehaviorInput(
        risk=RiskLevel.CAUTION,
        front_vehicle_present=True,
        left_lane_available=True,
        right_lane_available=False,
        lane_change_completed=False,
    )
)

print("3:", state)

state = fsm.update(
    BehaviorInput(
        risk=RiskLevel.CAUTION,
        front_vehicle_present=True,
        left_lane_available=True,
        right_lane_available=False,
        lane_change_completed=False,
    )
)

print("4:", state)

state = fsm.update(
    BehaviorInput(
        risk=RiskLevel.SAFE,
        front_vehicle_present=False,
        left_lane_available=True,
        right_lane_available=False,
        lane_change_completed=True,
    )
)

print("5:", state)

