import math

import numpy as np
import pytest

from coordinate_system import (
    FrenetState,
    FrenetTransformer,
    ReferenceLine,
    WorldState,
    angle_error,
    normalize_angle,
)


REFERENCE_PATH = [
    (0, 0),
    (5, 1),
    (10, 4),
    (15, 8),
    (20, 10),
]


@pytest.fixture(scope="module")
def reference_line():
    return ReferenceLine(
        REFERENCE_PATH,
        reparam_samples=5000,
    )


@pytest.fixture(scope="module")
def transformer(reference_line):
    return FrenetTransformer(
        reference_line,
        projection_samples=500,
    )


def assert_state_round_trip(
    transformer,
    frenet_state,
    s_tol=2e-5,
    d_tol=2e-6,
    d_prime_tol=2e-5,
    position_tol=2e-5,
    heading_tol=2e-7,
):
    world = transformer.frenet_to_world(frenet_state)
    recovered = transformer.world_to_frenet(world)
    world_again = transformer.frenet_to_world(recovered)

    s_error = abs(recovered.s - frenet_state.s)
    d_error = abs(recovered.d - frenet_state.d)
    d_prime_error = abs(
        recovered.d_prime - frenet_state.d_prime
    )

    position_error = math.hypot(
        world_again.x - world.x,
        world_again.y - world.y,
    )

    heading_error = abs(
        angle_error(
            world_again.theta,
            world.theta,
        )
    )

    assert s_error < s_tol
    assert d_error < d_tol
    assert d_prime_error < d_prime_tol
    assert position_error < position_tol
    assert heading_error < heading_tol


def test_reference_line_is_arc_length_parameterized(reference_line):
    s = np.linspace(
        0.0,
        reference_line.length,
        100,
    )

    speed = reference_line.arc_length_speed(s)
    max_error = float(
        np.max(np.abs(speed - 1.0))
    )

    assert max_error < 2e-5


def test_reference_query_rejects_out_of_range_s(reference_line):
    with pytest.raises(ValueError):
        reference_line.query(-1e-3)

    with pytest.raises(ValueError):
        reference_line.query(
            reference_line.length + 1e-3
        )


def test_duplicate_adjacent_waypoint_rejected():
    with pytest.raises(ValueError):
        ReferenceLine(
            [
                (0.0, 0.0),
                (1.0, 1.0),
                (1.0, 1.0),
                (2.0, 2.0),
            ]
        )


def test_known_full_state_round_trip(transformer):
    assert_state_round_trip(
        transformer,
        FrenetState(
            s=10.0,
            d=1.0,
            d_prime=0.2,
        ),
        s_tol=1e-6,
        d_tol=1e-8,
        d_prime_tol=1e-6,
        position_tol=1e-6,
        heading_tol=1e-9,
    )


S_FRACTIONS = [0.10, 0.30, 0.50, 0.70, 0.90]
D_VALUES = [-1.5, -0.5, 0.0, 0.5, 1.5]
D_PRIME_VALUES = [-0.3, -0.1, 0.0, 0.1, 0.3]


@pytest.mark.parametrize("s_fraction", S_FRACTIONS)
@pytest.mark.parametrize("d", D_VALUES)
@pytest.mark.parametrize("d_prime", D_PRIME_VALUES)
def test_125_state_round_trip_grid(
    transformer,
    reference_line,
    s_fraction,
    d,
    d_prime,
):
    s = s_fraction * reference_line.length

    assert_state_round_trip(
        transformer,
        FrenetState(
            s=s,
            d=d,
            d_prime=d_prime,
        ),
    )


@pytest.mark.parametrize("s_fraction", [0.15, 0.50, 0.85])
def test_left_right_sign_convention(
    transformer,
    reference_line,
    s_fraction,
):
    s = s_fraction * reference_line.length

    world_left = transformer.frenet_to_world(
        FrenetState(
            s=s,
            d=1.0,
            d_prime=0.0,
        )
    )

    world_right = transformer.frenet_to_world(
        FrenetState(
            s=s,
            d=-1.0,
            d_prime=0.0,
        )
    )

    frenet_left = transformer.world_to_frenet(
        world_left
    )

    frenet_right = transformer.world_to_frenet(
        world_right
    )

    assert frenet_left.d > 0.0
    assert frenet_right.d < 0.0


def test_angle_normalization():
    assert abs(
        normalize_angle(3.0 * math.pi) - math.pi
    ) < 1e-12 or abs(
        normalize_angle(3.0 * math.pi) + math.pi
    ) < 1e-12

    assert abs(
        angle_error(
            math.radians(-179.0),
            math.radians(179.0),
        )
        - math.radians(2.0)
    ) < 1e-12


def test_frenet_singularity_guard(
    transformer,
    reference_line,
):
    s = 10.0
    ref = reference_line.query(s)

    assert abs(ref.kappa) > 1e-6

    d_singular = 1.0 / ref.kappa

    with pytest.raises(ValueError):
        transformer.frenet_to_world(
            FrenetState(
                s=s,
                d=d_singular,
                d_prime=0.0,
            )
        )


@pytest.mark.parametrize("s_fraction", [0.2, 0.5, 0.8])
def test_parallel_heading_means_zero_d_prime(
    transformer,
    reference_line,
    s_fraction,
):
    s = s_fraction * reference_line.length

    world = transformer.frenet_to_world(
        FrenetState(
            s=s,
            d=0.7,
            d_prime=0.0,
        )
    )

    recovered = transformer.world_to_frenet(world)

    assert abs(recovered.d_prime) < 1e-7


def test_world_heading_too_far_from_reference_rejected(
    transformer,
    reference_line,
):
    s = 0.5 * reference_line.length
    ref = reference_line.query(s)

    world = WorldState(
        x=ref.x,
        y=ref.y,
        theta=normalize_angle(
            ref.theta + math.radians(100.0)
        ),
    )

    with pytest.raises(ValueError):
        transformer.world_to_frenet(world)
