import math

import pytest

from control.trajectory_tracker import (
    TrajectoryTracker,
)

from vehicle_model.models import (
    VehicleState,
)


def make_tracker(
    search_window: int = 20,
):
    return TrajectoryTracker(
        trajectory_x=[
            0.0,
            10.0,
            20.0,
            30.0,
            40.0,
        ],

        trajectory_y=[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        trajectory_yaw=[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        wheel_base=2.8,

        search_window=search_window,
    )


def state_with_front_axle_at(
    front_x: float,
    front_y: float = 0.0,
    yaw: float = 0.0,
    speed: float = 10.0,
):
    """
    VehicleState stores the rear axle position.

    For yaw = 0:

        front_x = rear_x + wheel_base

    Therefore:

        rear_x = front_x - wheel_base
    """

    wheel_base = 2.8

    rear_x = (
        front_x
        - wheel_base
        * math.cos(yaw)
    )

    rear_y = (
        front_y
        - wheel_base
        * math.sin(yaw)
    )

    return VehicleState(
        x=rear_x,
        y=rear_y,
        psi=yaw,
        v=speed,
    )


# ============================================================
# 1. Normal progress should move forward
# ============================================================

def test_tracker_progresses_forward():

    tracker = make_tracker()

    state_0 = state_with_front_axle_at(
        front_x=0.0
    )

    index_0, _, _ = (
        tracker.compute_errors(
            state_0
        )
    )

    state_1 = state_with_front_axle_at(
        front_x=10.0
    )

    index_1, _, _ = (
        tracker.compute_errors(
            state_1
        )
    )

    state_2 = state_with_front_axle_at(
        front_x=20.0
    )

    index_2, _, _ = (
        tracker.compute_errors(
            state_2
        )
    )

    assert index_0 == 0
    assert index_1 == 1
    assert index_2 == 2


# ============================================================
# 2. Progress must never move backward
# ============================================================

def test_tracker_never_moves_backward():

    tracker = make_tracker()

    # First move vehicle to trajectory index 2.
    state_forward = (
        state_with_front_axle_at(
            front_x=20.0
        )
    )

    index_forward, _, _ = (
        tracker.compute_errors(
            state_forward
        )
    )

    assert index_forward == 2

    # Now physically move the vehicle back near trajectory index 0.
    #
    # A global nearest-point search would return index 0.
    # The tracker must NOT allow progress to go backward.
    state_backward = (
        state_with_front_axle_at(
            front_x=0.0
        )
    )

    index_after, _, _ = (
        tracker.compute_errors(
            state_backward
        )
    )

    assert index_after >= index_forward
    assert index_after == 2


# ============================================================
# 3. reset() should restore progress to zero
# ============================================================

def test_reset_restores_initial_progress():

    tracker = make_tracker()

    state_forward = (
        state_with_front_axle_at(
            front_x=30.0
        )
    )

    index, _, _ = (
        tracker.compute_errors(
            state_forward
        )
    )

    assert index == 3
    assert tracker.previous_index == 3

    tracker.reset()

    assert tracker.previous_index == 0

    state_start = (
        state_with_front_axle_at(
            front_x=0.0
        )
    )

    index_after_reset, _, _ = (
        tracker.compute_errors(
            state_start
        )
    )

    assert index_after_reset == 0


# ============================================================
# 4. Signed CTE should preserve our convention
#
# e_y > 0:
# target path is on the LEFT of vehicle.
# ============================================================

def test_cross_track_error_sign():

    tracker = make_tracker()

    # Straight path is y = 0.
    #
    # Put front axle at y = -2.
    # Therefore path is 2 m to vehicle's left.
    state = state_with_front_axle_at(
        front_x=10.0,
        front_y=-2.0,
    )

    _, heading_error, cross_track_error = (
        tracker.compute_errors(
            state
        )
    )

    assert heading_error == pytest.approx(
        0.0
    )

    assert cross_track_error == pytest.approx(
        2.0
    )


# ============================================================
# 5. Heading error should use path - vehicle convention
# ============================================================

def test_heading_error_sign():

    tracker = make_tracker()

    state = state_with_front_axle_at(
        front_x=10.0,
        front_y=0.0,

        # Vehicle points 5 degrees left,
        # while path yaw is zero.
        yaw=math.radians(5.0),
    )

    _, heading_error, _ = (
        tracker.compute_errors(
            state
        )
    )

    assert math.degrees(
        heading_error
    ) == pytest.approx(
        -5.0
    )


# ============================================================
# 6. Empty trajectory should be rejected
# ============================================================

def test_empty_trajectory_rejected():

    with pytest.raises(
        ValueError
    ):
        TrajectoryTracker(
            trajectory_x=[],
            trajectory_y=[],
            trajectory_yaw=[],

            wheel_base=2.8,
        )


# ============================================================
# 7. Mismatched trajectory lengths should be rejected
# ============================================================

def test_mismatched_trajectory_lengths_rejected():

    with pytest.raises(
        ValueError
    ):
        TrajectoryTracker(
            trajectory_x=[
                0.0,
                10.0,
            ],

            trajectory_y=[
                0.0,
            ],

            trajectory_yaw=[
                0.0,
                0.0,
            ],

            wheel_base=2.8,
        )


# ============================================================
# 8. Invalid search window should be rejected
# ============================================================

def test_invalid_search_window_rejected():

    with pytest.raises(
        ValueError
    ):
        make_tracker(
            search_window=0
        )