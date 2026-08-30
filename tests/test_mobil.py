import pytest

from behavior.mobil import (
    evaluate_lane_change,
    evaluate_lane_change_with_idm,
)

from behavior.models import (
    IDMParams,
    MOBILParams,
    LaneChangeEvaluation,
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


# ============================================================
# Pure MOBIL core
# ============================================================

def test_beneficial_lane_change(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=1.5,

        new_follower_acc_old=0.5,
        new_follower_acc_new=-1.5,

        old_follower_acc_old=-1.0,
        old_follower_acc_new=0.5,

        params=mobil_params,
    )

    assert isinstance(
        result,
        LaneChangeEvaluation,
    )

    assert result.safe is True

    assert result.incentive == pytest.approx(
        2.35
    )

    assert result.beneficial is True


def test_unsafe_lane_change_is_rejected(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=5.0,

        new_follower_acc_old=0.5,
        new_follower_acc_new=-6.0,

        old_follower_acc_old=-1.0,
        old_follower_acc_new=1.0,

        params=mobil_params,
    )

    assert result.safe is False
    assert result.beneficial is False


def test_exact_safe_braking_boundary_is_allowed(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=1.0,

        new_follower_acc_old=0.0,
        new_follower_acc_new=-4.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=1.0,

        params=mobil_params,
    )

    assert result.safe is True


def test_below_safe_braking_boundary_is_rejected(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=5.0,

        new_follower_acc_old=0.0,
        new_follower_acc_new=-4.01,

        old_follower_acc_old=0.0,
        old_follower_acc_new=1.0,

        params=mobil_params,
    )

    assert result.safe is False
    assert result.beneficial is False


def test_small_incentive_does_not_trigger_lane_change(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=0.0,
        ego_acc_new=0.1,

        new_follower_acc_old=0.0,
        new_follower_acc_new=0.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=0.0,

        params=mobil_params,
    )

    assert result.safe is True

    assert result.incentive == pytest.approx(
        0.1
    )

    assert result.beneficial is False


def test_exact_threshold_is_not_beneficial(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=0.0,
        ego_acc_new=0.2,

        new_follower_acc_old=0.0,
        new_follower_acc_new=0.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=0.0,

        params=mobil_params,
    )

    assert result.incentive == pytest.approx(
        0.2
    )

    assert result.beneficial is False


def test_incentive_above_threshold_is_beneficial(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=0.0,
        ego_acc_new=0.21,

        new_follower_acc_old=0.0,
        new_follower_acc_new=0.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=0.0,

        params=mobil_params,
    )

    assert result.safe is True

    assert result.incentive == pytest.approx(
        0.21
    )

    assert result.beneficial is True


def test_zero_politeness_only_considers_ego():

    params = MOBILParams(
        politeness=0.0,
        safe_braking=4.0,
        incentive_threshold=0.2,
    )

    result = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=1.0,

        new_follower_acc_old=1.0,
        new_follower_acc_new=-3.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=0.0,

        params=params,
    )

    assert result.incentive == pytest.approx(
        2.0
    )

    assert result.safe is True
    assert result.beneficial is True


def test_politeness_penalizes_harm_to_new_follower(
        mobil_params
):

    result = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=1.0,

        new_follower_acc_old=0.0,
        new_follower_acc_new=-2.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=0.0,

        params=mobil_params,
    )

    # ego gain = +2
    # new follower loss = -2
    #
    # G = 2 + 0.3 * (-2)
    #   = 1.4

    assert result.incentive == pytest.approx(
        1.4
    )


def test_old_follower_benefit_increases_incentive(
        mobil_params
):

    result_without_benefit = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=1.0,

        new_follower_acc_old=0.0,
        new_follower_acc_new=-1.0,

        old_follower_acc_old=0.0,
        old_follower_acc_new=0.0,

        params=mobil_params,
    )

    result_with_benefit = evaluate_lane_change(
        ego_acc_old=-1.0,
        ego_acc_new=1.0,

        new_follower_acc_old=0.0,
        new_follower_acc_new=-1.0,

        old_follower_acc_old=-2.0,
        old_follower_acc_new=1.0,

        params=mobil_params,
    )

    assert (
        result_with_benefit.incentive
        >
        result_without_benefit.incentive
    )


# ============================================================
# IDM + MOBIL integration
# ============================================================

def test_idm_mobil_beneficial_lane_change(
        idm_params,
        mobil_params
):

    # Current lane:
    #
    # old_leader <- ego <- old_follower
    #
    # Target lane:
    #
    # new_leader <-     <- new_follower

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

    new_leader = LongitudinalVehicleState(
        s=60.0,
        v=25.0,
    )

    new_follower = LongitudinalVehicleState(
        s=-40.0,
        v=22.0,
    )

    result = evaluate_lane_change_with_idm(
        ego=ego,

        old_leader=old_leader,
        old_follower=old_follower,

        new_leader=new_leader,
        new_follower=new_follower,

        idm_params=idm_params,
        mobil_params=mobil_params,
    )

    assert result.safe is True

    assert result.incentive == pytest.approx(
        3.7827063281,
        abs=1e-8,
    )

    assert result.beneficial is True


def test_idm_mobil_rejects_unsafe_target_follower(
        idm_params,
        mobil_params
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

    new_leader = LongitudinalVehicleState(
        s=60.0,
        v=25.0,
    )

    # Fast target-lane follower is too close behind ego.
    new_follower = LongitudinalVehicleState(
        s=-20.0,
        v=25.0,
    )

    result = evaluate_lane_change_with_idm(
        ego=ego,

        old_leader=old_leader,
        old_follower=old_follower,

        new_leader=new_leader,
        new_follower=new_follower,

        idm_params=idm_params,
        mobil_params=mobil_params,
    )

    assert result.safe is False
    assert result.beneficial is False


def test_idm_mobil_without_target_follower(
        idm_params,
        mobil_params
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

    new_leader = LongitudinalVehicleState(
        s=80.0,
        v=25.0,
    )

    result = evaluate_lane_change_with_idm(
        ego=ego,

        old_leader=old_leader,
        old_follower=old_follower,

        new_leader=new_leader,
        new_follower=None,

        idm_params=idm_params,
        mobil_params=mobil_params,
    )

    assert result.safe is True


def test_idm_mobil_without_old_follower(
        idm_params,
        mobil_params
):

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    old_leader = LongitudinalVehicleState(
        s=40.0,
        v=15.0,
    )

    new_leader = LongitudinalVehicleState(
        s=60.0,
        v=25.0,
    )

    new_follower = LongitudinalVehicleState(
        s=-40.0,
        v=22.0,
    )

    result = evaluate_lane_change_with_idm(
        ego=ego,

        old_leader=old_leader,
        old_follower=None,

        new_leader=new_leader,
        new_follower=new_follower,

        idm_params=idm_params,
        mobil_params=mobil_params,
    )

    assert isinstance(
        result,
        LaneChangeEvaluation,
    )


def test_invalid_vehicle_order_raises_error(
        idm_params,
        mobil_params
):

    ego = LongitudinalVehicleState(
        s=0.0,
        v=20.0,
    )

    # This "leader" is actually behind ego.
    invalid_old_leader = LongitudinalVehicleState(
        s=-10.0,
        v=15.0,
    )

    with pytest.raises(ValueError):

        evaluate_lane_change_with_idm(
            ego=ego,

            old_leader=invalid_old_leader,
            old_follower=None,

            new_leader=None,
            new_follower=None,

            idm_params=idm_params,
            mobil_params=mobil_params,
        )