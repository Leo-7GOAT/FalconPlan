import pytest

from behavior.planner import (
    BehaviorPlanner,
)

from behavior.models import (
    BehaviorState,
    IDMParams,
    MOBILParams,
    RiskLevel,
    LongitudinalVehicleState,
)


@pytest.fixture
def idm_params():

    return IDMParams(
        desired_speed=30.0,
        max_accel=2.0,
        comfortable_decel=2.0,
        min_gap=2.0,
        time_headway=1.5,
        delta=4.0,
    )


@pytest.fixture
def mobil_params():

    return MOBILParams(
        politeness=0.3,
        safe_braking=4.0,
        incentive_threshold=0.2,
    )


@pytest.fixture
def planner(
        idm_params,
        mobil_params,
):

    # horizon=0:
    # isolate current behavior for normal FSM tests.
    return BehaviorPlanner(
        idm_params=idm_params,
        mobil_params=mobil_params,
        prediction_horizon=0.0,
        prediction_dt=0.5,
    )


def test_free_road_keeps_lane(
        planner
):

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    result = planner.step(
        ego=ego,
    )

    assert (
        result.state
        == BehaviorState.KEEP_LANE
    )

    assert (
        result.risk
        == RiskLevel.SAFE
    )

    assert result.ttc == pytest.approx(
        float("inf")
    )

    assert result.thw == pytest.approx(
        float("inf")
    )

    assert result.idm_acceleration == pytest.approx(
        1.6049382716,
        abs=1e-9,
    )


def test_slow_front_vehicle_enters_follow(
        planner
):

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    front = LongitudinalVehicleState(
        s=20.0,
        v=15.0,
    )

    result = planner.step(
        ego=ego,
        front_current_lane=front,
    )

    assert (
        result.risk.value
        >= RiskLevel.CAUTION.value
    )

    assert (
        result.state
        == BehaviorState.FOLLOW
    )

    assert result.idm_acceleration < 0.0


def test_left_lane_change_sequence(
        planner
):

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    old_leader = LongitudinalVehicleState(
        s=40.0,
        v=15.0,
    )

    old_follower = LongitudinalVehicleState(
        s=-25.0,
        v=20.0,
    )

    left_leader = LongitudinalVehicleState(
        s=60.0,
        v=25.0,
    )

    left_follower = LongitudinalVehicleState(
        s=-40.0,
        v=22.0,
    )

    # Step 1:
    # KEEP_LANE -> FOLLOW

    result1 = planner.step(
        ego=ego,

        front_current_lane=old_leader,
        rear_current_lane=old_follower,

        front_left_lane=left_leader,
        rear_left_lane=left_follower,

        left_lane_exists=True,
    )

    assert (
        result1.state
        == BehaviorState.FOLLOW
    )

    assert result1.left_eval is not None
    assert result1.left_eval.safe is True
    assert result1.left_eval.beneficial is True

    # Step 2:
    # FOLLOW -> PREPARE_LEFT

    result2 = planner.step(
        ego=ego,

        front_current_lane=old_leader,
        rear_current_lane=old_follower,

        front_left_lane=left_leader,
        rear_left_lane=left_follower,

        left_lane_exists=True,
    )

    assert (
        result2.state
        == BehaviorState.PREPARE_LANE_CHANGE_LEFT
    )

    # Step 3:
    # PREPARE_LEFT -> LANE_CHANGE_LEFT

    result3 = planner.step(
        ego=ego,

        front_current_lane=old_leader,
        rear_current_lane=old_follower,

        front_left_lane=left_leader,
        rear_left_lane=left_follower,

        left_lane_exists=True,
    )

    assert (
        result3.state
        == BehaviorState.LANE_CHANGE_LEFT
    )

    # Step 4:
    # Lane change is still in progress.

    result4 = planner.step(
        ego=ego,

        front_current_lane=old_leader,
        rear_current_lane=old_follower,

        front_left_lane=left_leader,
        rear_left_lane=left_follower,

        left_lane_exists=True,

        lane_change_completed=False,
    )

    assert (
        result4.state
        == BehaviorState.LANE_CHANGE_LEFT
    )

    # Step 5:
    # Lane change completed.

    result5 = planner.step(
        ego=ego,

        front_current_lane=old_leader,
        rear_current_lane=old_follower,

        front_left_lane=left_leader,
        rear_left_lane=left_follower,

        left_lane_exists=True,

        lane_change_completed=True,
    )

    assert (
        result5.state
        == BehaviorState.KEEP_LANE
    )


def test_predictive_risk_triggers_emergency_brake(
        idm_params,
        mobil_params,
):

    planner = BehaviorPlanner(
        idm_params=idm_params,
        mobil_params=mobil_params,

        prediction_horizon=2.0,
        prediction_dt=0.5,
    )

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    front = LongitudinalVehicleState(
        s=30.0,
        v=15.0,
    )

    result = planner.step(
        ego=ego,

        front_current_lane=front,

        ego_prediction_acceleration=0.0,
        front_prediction_acceleration=-2.0,
    )

    # Current moment is only CAUTION.
    assert (
        result.current_risk
        == RiskLevel.CAUTION
    )

    # Future horizon reaches EMERGENCY.
    assert (
        result.risk
        == RiskLevel.EMERGENCY
    )

    assert (
        result.state
        == BehaviorState.EMERGENCY_BRAKE
    )

    assert len(
        result.risk_horizon
    ) == 5

    assert (
        result.risk_horizon[-1].risk
        == RiskLevel.EMERGENCY
    )


def test_both_empty_target_lanes_prefer_left_on_tie(
        planner
):

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    old_leader = LongitudinalVehicleState(
        s=40.0,
        v=15.0,
    )

    # First frame:
    # KEEP -> FOLLOW

    result1 = planner.step(
        ego=ego,

        front_current_lane=old_leader,

        left_lane_exists=True,
        right_lane_exists=True,
    )

    assert (
        result1.state
        == BehaviorState.FOLLOW
    )

    assert result1.left_eval is not None
    assert result1.right_eval is not None

    assert (
        result1.left_eval.incentive
        == pytest.approx(
            result1.right_eval.incentive
        )
    )

    # Second frame:
    # equal incentive uses deterministic left preference.

    result2 = planner.step(
        ego=ego,

        front_current_lane=old_leader,

        left_lane_exists=True,
        right_lane_exists=True,
    )

    assert (
        result2.state
        == BehaviorState.PREPARE_LANE_CHANGE_LEFT
    )


def test_invalid_front_vehicle_position_raises(
        planner
):

    ego = LongitudinalVehicleState(
        s=10.0,
        v=20.0,
    )

    invalid_front = LongitudinalVehicleState(
        s=5.0,
        v=15.0,
    )

    with pytest.raises(ValueError):

        planner.step(
            ego=ego,
            front_current_lane=invalid_front,
        )


def test_invalid_prediction_configuration(
        idm_params,
        mobil_params,
):

    with pytest.raises(ValueError):

        BehaviorPlanner(
            idm_params=idm_params,
            mobil_params=mobil_params,
            prediction_horizon=-1.0,
            prediction_dt=0.5,
        )

    with pytest.raises(ValueError):

        BehaviorPlanner(
            idm_params=idm_params,
            mobil_params=mobil_params,
            prediction_horizon=2.0,
            prediction_dt=0.0,
        )