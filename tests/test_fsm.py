from behavior.fsm import BehaviorFSM

from behavior.models import (
    BehaviorInput,
    BehaviorState,
    RiskLevel,
)


def make_input(
        risk=RiskLevel.SAFE,
        front=False,
        left=False,
        right=False,
        completed=False,
):

    return BehaviorInput(
        risk=risk,
        front_vehicle_present=front,
        left_lane_available=left,
        right_lane_available=right,
        lane_change_completed=completed,
    )


def test_initial_state():

    fsm = BehaviorFSM()

    assert fsm.states == BehaviorState.KEEP_LANE


def test_keep_lane_to_follow():

    fsm = BehaviorFSM()

    state = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    assert state == BehaviorState.FOLLOW


def test_follow_returns_when_front_vehicle_disappears():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    state = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=False,
        )
    )

    assert state == BehaviorState.KEEP_LANE


def test_follow_requires_three_safe_frames():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    state1 = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    state2 = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    state3 = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    assert state1 == BehaviorState.FOLLOW
    assert state2 == BehaviorState.FOLLOW
    assert state3 == BehaviorState.KEEP_LANE


def test_follow_safe_counter_resets():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    # 危险重新出现，连续 SAFE 被打断
    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    state = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    assert state == BehaviorState.FOLLOW


def test_left_lane_change_sequence():

    fsm = BehaviorFSM()

    state1 = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    state2 = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            left=True,
        )
    )

    state3 = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            left=True,
        )
    )

    state4 = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            left=True,
            completed=False,
        )
    )

    state5 = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=False,
            left=True,
            completed=True,
        )
    )

    assert state1 == BehaviorState.FOLLOW

    assert (
        state2
        == BehaviorState.PREPARE_LANE_CHANGE_LEFT
    )

    assert state3 == BehaviorState.LANE_CHANGE_LEFT
    assert state4 == BehaviorState.LANE_CHANGE_LEFT
    assert state5 == BehaviorState.KEEP_LANE


def test_prepare_left_can_be_cancelled():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            left=True,
        )
    )

    state = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            left=False,
        )
    )

    assert state == BehaviorState.FOLLOW


def test_right_lane_change_sequence():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    state = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            right=True,
        )
    )

    assert (
        state
        == BehaviorState.PREPARE_LANE_CHANGE_RIGHT
    )

    state = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
            right=True,
        )
    )

    assert state == BehaviorState.LANE_CHANGE_RIGHT


def test_emergency_from_keep_lane():

    fsm = BehaviorFSM()

    state = fsm.update(
        make_input(
            risk=RiskLevel.EMERGENCY,
            front=True,
        )
    )

    assert state == BehaviorState.EMERGENCY_BRAKE


def test_emergency_interrupts_lane_change():

    fsm = BehaviorFSM()

    fsm.states = BehaviorState.LANE_CHANGE_LEFT

    state = fsm.update(
        make_input(
            risk=RiskLevel.EMERGENCY,
            front=True,
        )
    )

    assert state == BehaviorState.EMERGENCY_BRAKE


def test_emergency_requires_three_recovery_frames():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.EMERGENCY,
            front=True,
        )
    )

    state1 = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    state2 = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    state3 = fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    assert state1 == BehaviorState.EMERGENCY_BRAKE
    assert state2 == BehaviorState.EMERGENCY_BRAKE
    assert state3 == BehaviorState.FOLLOW


def test_emergency_recovery_goes_to_keep_lane_without_front_vehicle():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.EMERGENCY,
            front=True,
        )
    )

    for _ in range(2):
        state = fsm.update(
            make_input(
                risk=RiskLevel.SAFE,
                front=False,
            )
        )

        assert state == BehaviorState.EMERGENCY_BRAKE

    state = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=False,
        )
    )

    assert state == BehaviorState.KEEP_LANE


def test_emergency_recovery_counter_resets_on_danger():

    fsm = BehaviorFSM()

    fsm.update(
        make_input(
            risk=RiskLevel.EMERGENCY,
            front=True,
        )
    )

    fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    fsm.update(
        make_input(
            risk=RiskLevel.CAUTION,
            front=True,
        )
    )

    # 打断恢复
    state = fsm.update(
        make_input(
            risk=RiskLevel.DANGER,
            front=True,
        )
    )

    assert state == BehaviorState.EMERGENCY_BRAKE

    # 重新开始第一帧恢复
    state = fsm.update(
        make_input(
            risk=RiskLevel.SAFE,
            front=True,
        )
    )

    assert state == BehaviorState.EMERGENCY_BRAKE