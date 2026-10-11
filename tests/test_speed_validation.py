
from planning.speed_profile import (
    SpeedProfile,
    SpeedProfilePoint,
)

from planning.speed_validation import (
    is_speed_profile_st_safe,
)

from planning.st_graph import STBoundary


def make_obstacle():
    return STBoundary(
        obstacle_id="front_car",
        times=(0.0, 1.0),
        lower_s=(26.0, 31.0),
        upper_s=(34.0, 39.0),
    )


def test_detects_collision_between_safe_endpoints():

    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(
                t=0.0,
                s=20.0,
                v=20.0,
                a=0.0,
            ),
            SpeedProfilePoint(
                t=1.0,
                s=40.0,
                v=20.0,
                a=0.0,
            ),
        )
    )

    # t=0: ego s=20, obstacle [26,34]
    # t=1: ego s=40, obstacle [31,39]
    #
    # Both endpoints are free.
    # The trajectory crosses the obstacle between them.

    assert not is_speed_profile_st_safe(
        profile=profile,
        boundaries=[make_obstacle()],
        sample_dt=0.01,
    )


def test_accepts_clear_speed_profile():

    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(
                t=0.0,
                s=0.0,
                v=10.0,
                a=0.0,
            ),
            SpeedProfilePoint(
                t=1.0,
                s=10.0,
                v=10.0,
                a=0.0,
            ),
        )
    )

    assert is_speed_profile_st_safe(
        profile=profile,
        boundaries=[make_obstacle()],
        sample_dt=0.01,
    )
