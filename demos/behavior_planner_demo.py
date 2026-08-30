from behavior.planner import (
    BehaviorPlanner,
)

from behavior.models import (
    IDMParams,
    MOBILParams,
    LongitudinalVehicleState,
)


idm_params = IDMParams(
    desired_speed=30.0,
    max_accel=2.0,
    comfortable_decel=2.0,
    min_gap=2.0,
    time_headway=1.5,
    delta=4.0,
)


mobil_params = MOBILParams(
    politeness=0.3,
    safe_braking=4.0,
    incentive_threshold=0.2,
)


# For this FSM demonstration we use current-state risk only.
planner = BehaviorPlanner(
    idm_params=idm_params,
    mobil_params=mobil_params,
    prediction_horizon=0.0,
    prediction_dt=0.5,
)


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


def show_result(
        step_name,
        result,
):

    print(
        f"\n{'=' * 60}"
    )

    print(step_name)

    print(
        f"{'=' * 60}"
    )

    print(
        "state:",
        result.state,
    )

    print(
        "current risk:",
        result.current_risk,
    )

    print(
        "overall risk:",
        result.risk,
    )

    print(
        f"TTC: {result.ttc:.3f}"
    )

    print(
        f"THW: {result.thw:.3f}"
    )

    print(
        "IDM acceleration:",
        f"{result.idm_acceleration:.3f} m/s^2",
    )

    print(
        "left evaluation:",
        result.left_eval,
    )

    print(
        "right evaluation:",
        result.right_eval,
    )


# ============================================================
# Step 0
#
# Free road.
# ============================================================

result = planner.step(
    ego=ego,
)

show_result(
    "Step 0 - Free road",
    result,
)


# ============================================================
# Step 1
#
# Slow front vehicle appears.
#
# KEEP_LANE -> FOLLOW
# ============================================================

result = planner.step(
    ego=ego,

    front_current_lane=old_leader,
    rear_current_lane=old_follower,

    front_left_lane=left_leader,
    rear_left_lane=left_follower,

    left_lane_exists=True,
)

show_result(
    "Step 1 - Slow front vehicle",
    result,
)


# ============================================================
# Step 2
#
# Left lane remains beneficial.
#
# FOLLOW -> PREPARE_LEFT
# ============================================================

result = planner.step(
    ego=ego,

    front_current_lane=old_leader,
    rear_current_lane=old_follower,

    front_left_lane=left_leader,
    rear_left_lane=left_follower,

    left_lane_exists=True,
)

show_result(
    "Step 2 - Prepare left lane change",
    result,
)


# ============================================================
# Step 3
#
# Target lane is still safe and beneficial.
#
# PREPARE_LEFT -> LANE_CHANGE_LEFT
# ============================================================

result = planner.step(
    ego=ego,

    front_current_lane=old_leader,
    rear_current_lane=old_follower,

    front_left_lane=left_leader,
    rear_left_lane=left_follower,

    left_lane_exists=True,
)

show_result(
    "Step 3 - Execute left lane change",
    result,
)


# ============================================================
# Step 4
#
# Lane change is not finished yet.
# ============================================================

result = planner.step(
    ego=ego,

    front_current_lane=old_leader,
    rear_current_lane=old_follower,

    front_left_lane=left_leader,
    rear_left_lane=left_follower,

    left_lane_exists=True,

    lane_change_completed=False,
)

show_result(
    "Step 4 - Lane change in progress",
    result,
)


# ============================================================
# Step 5
#
# Lane change completed.
#
# LANE_CHANGE_LEFT -> KEEP_LANE
# ============================================================

result = planner.step(
    ego=ego,

    front_current_lane=old_leader,
    rear_current_lane=old_follower,

    front_left_lane=left_leader,
    rear_left_lane=left_follower,

    left_lane_exists=True,

    lane_change_completed=True,
)

show_result(
    "Step 5 - Lane change completed",
    result,
)