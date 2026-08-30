from .models import (
    BehaviorState,
    BehaviorInput,
    RiskLevel,
)

class BehaviorFSM():

    def __init__(self):
        self.states = BehaviorState.KEEP_LANE
        self.safe_counter = 0
        self.emergency_recovery_counter = 0

    def update(
            self,
            data: BehaviorInput
    ) -> BehaviorState:

        # Emergency 最高优先级
        if data.risk == RiskLevel.EMERGENCY:
            self.states = BehaviorState.EMERGENCY_BRAKE
            self.safe_counter = 0
            self.emergency_recovery_counter = 0
            return self.states

        if self.states == BehaviorState.EMERGENCY_BRAKE:

            if data.risk.value <= RiskLevel.CAUTION.value:
                self.emergency_recovery_counter += 1

                if self.emergency_recovery_counter >= 3:
                    self.emergency_recovery_counter = 0
                    if data.front_vehicle_present:
                        self.states = BehaviorState.FOLLOW
                    else:
                        self.states = BehaviorState.KEEP_LANE
            else:
                self.emergency_recovery_counter = 0

            return self.states

        if self.states == BehaviorState.KEEP_LANE:

            if (
                    data.front_vehicle_present
                    and
                    data.risk.value >= RiskLevel.CAUTION.value
            ):
                self.states = BehaviorState.FOLLOW

        elif self.states == BehaviorState.FOLLOW:

            if not data.front_vehicle_present:
                self.safe_counter = 0
                self.states = BehaviorState.KEEP_LANE

            elif data.risk == RiskLevel.SAFE:

                self.safe_counter += 1

                if self.safe_counter >= 3:
                    self.safe_counter = 0
                    self.states = BehaviorState.KEEP_LANE

            else:
                self.safe_counter = 0

                if data.left_lane_available:
                    self.states = (
                        BehaviorState.PREPARE_LANE_CHANGE_LEFT
                    )

                elif data.right_lane_available:
                    self.states = (
                        BehaviorState.PREPARE_LANE_CHANGE_RIGHT
                    )

        elif (
                self.states
                == BehaviorState.PREPARE_LANE_CHANGE_LEFT
        ):

            if data.left_lane_available:
                self.states = BehaviorState.LANE_CHANGE_LEFT
            else:
                self.states = BehaviorState.FOLLOW

        elif (
                self.states
                == BehaviorState.PREPARE_LANE_CHANGE_RIGHT
        ):

            if data.right_lane_available:
                self.states = BehaviorState.LANE_CHANGE_RIGHT
            else:
                self.states = BehaviorState.FOLLOW

        elif self.states == BehaviorState.LANE_CHANGE_LEFT:

            if data.lane_change_completed:
                self.states = BehaviorState.KEEP_LANE

        elif self.states == BehaviorState.LANE_CHANGE_RIGHT:

            if data.lane_change_completed:
                self.states = BehaviorState.KEEP_LANE

        return self.states