from .idm import compute_idm_acceleration

from .models import (
    IDMParams,
    MOBILParams,
    LaneChangeEvaluation,
    LongitudinalVehicleState,
)


def evaluate_lane_change(
        ego_acc_old: float,
        ego_acc_new: float,
        new_follower_acc_old: float,
        new_follower_acc_new: float,
        old_follower_acc_old: float,
        old_follower_acc_new: float,
        params: MOBILParams,
) -> LaneChangeEvaluation:
    """
    Evaluate lane-change safety and incentive using already-computed
    longitudinal accelerations.
    """

    # Safety constraint:
    # the new follower must not be forced to brake harder
    # than the allowed safe braking magnitude.
    safe = (
        new_follower_acc_new
        >= -params.safe_braking
    )

    # Acceleration changes before -> after lane change.
    delta_ego = (
        ego_acc_new
        - ego_acc_old
    )

    delta_new_follower = (
        new_follower_acc_new
        - new_follower_acc_old
    )

    delta_old_follower = (
        old_follower_acc_new
        - old_follower_acc_old
    )

    # MOBIL incentive criterion.
    incentive = (
        delta_ego
        + params.politeness
        * (
            delta_new_follower
            + delta_old_follower
        )
    )

    beneficial = (
        safe
        and
        incentive > params.incentive_threshold
    )

    return LaneChangeEvaluation(
        safe=safe,
        incentive=incentive,
        beneficial=beneficial,
    )


def _compute_following_acceleration(
        follower: LongitudinalVehicleState,
        leader: LongitudinalVehicleState | None,
        idm_params: IDMParams,
) -> float:
    """
    Compute the IDM acceleration of one follower.

    leader=None means free-road driving.
    """

    # No leader: free-road IDM.
    if leader is None:
        return compute_idm_acceleration(
            ego_speed=follower.v,
            gap=None,
            front_speed=None,
            params=idm_params,
        )

    # Longitudinal gap in Frenet s.
    gap = (
        leader.s
        - follower.s
    )

    if gap <= 0.0:
        raise ValueError(
            "leader must be ahead of follower"
        )

    return compute_idm_acceleration(
        ego_speed=follower.v,
        gap=gap,
        front_speed=leader.v,
        params=idm_params,
    )


def evaluate_lane_change_with_idm(
        ego: LongitudinalVehicleState,
        old_leader: LongitudinalVehicleState | None,
        old_follower: LongitudinalVehicleState | None,
        new_leader: LongitudinalVehicleState | None,
        new_follower: LongitudinalVehicleState | None,
        idm_params: IDMParams,
        mobil_params: MOBILParams,
) -> LaneChangeEvaluation:
    """
    Evaluate a candidate lane change by computing all required
    before/after accelerations with IDM.

    old_*:
        vehicles in the ego vehicle's current lane.

    new_*:
        vehicles in the target lane.
    """

    # --------------------------------------------------------
    # 1. Ego vehicle
    #
    # Before:
    # old_leader <- ego
    #
    # After:
    # new_leader <- ego
    # --------------------------------------------------------

    ego_acc_old = _compute_following_acceleration(
        follower=ego,
        leader=old_leader,
        idm_params=idm_params,
    )

    ego_acc_new = _compute_following_acceleration(
        follower=ego,
        leader=new_leader,
        idm_params=idm_params,
    )

    # --------------------------------------------------------
    # 2. New-lane follower
    #
    # Before:
    # new_leader <- new_follower
    #
    # After:
    # new_leader <- ego <- new_follower
    # --------------------------------------------------------

    if new_follower is None:

        # No vehicle behind ego in the target lane,
        # so there is no follower impact.
        new_follower_acc_old = 0.0
        new_follower_acc_new = 0.0

    else:

        new_follower_acc_old = (
            _compute_following_acceleration(
                follower=new_follower,
                leader=new_leader,
                idm_params=idm_params,
            )
        )

        new_follower_acc_new = (
            _compute_following_acceleration(
                follower=new_follower,
                leader=ego,
                idm_params=idm_params,
            )
        )

    # --------------------------------------------------------
    # 3. Old-lane follower
    #
    # Before:
    # old_leader <- ego <- old_follower
    #
    # After ego leaves:
    # old_leader <- old_follower
    # --------------------------------------------------------

    if old_follower is None:

        # No vehicle behind ego in the old lane,
        # so there is no follower impact.
        old_follower_acc_old = 0.0
        old_follower_acc_new = 0.0

    else:

        old_follower_acc_old = (
            _compute_following_acceleration(
                follower=old_follower,
                leader=ego,
                idm_params=idm_params,
            )
        )

        old_follower_acc_new = (
            _compute_following_acceleration(
                follower=old_follower,
                leader=old_leader,
                idm_params=idm_params,
            )
        )

    # --------------------------------------------------------
    # 4. Send the six IDM accelerations to the MOBIL core.
    # --------------------------------------------------------

    return evaluate_lane_change(
        ego_acc_old=ego_acc_old,
        ego_acc_new=ego_acc_new,

        new_follower_acc_old=new_follower_acc_old,
        new_follower_acc_new=new_follower_acc_new,

        old_follower_acc_old=old_follower_acc_old,
        old_follower_acc_new=old_follower_acc_new,

        params=mobil_params,
    )