import math

import pytest

from coordinate_system import (
    BodyPoint,
    WorldPoint,
    WorldState,
)

from coordinate_system.body_transform import (
    world_to_body,
    body_to_world,
)


def test_world_to_body_front():

    ego = WorldState(
        x=10.0,
        y=5.0,
        theta=0.0
    )

    point = WorldPoint(
        x=15.0,
        y=5.0
    )

    result = world_to_body(
        point,
        ego
    )

    assert result.x == pytest.approx(5.0)
    assert result.y == pytest.approx(0.0)

    def test_world_to_body_front_when_heading_north():
        ego = WorldState(
            x=10.0,
            y=5.0,
            theta=math.radians(90)
        )

        point = WorldPoint(
            x=10.0,
            y=10.0
        )

        result = world_to_body(
            point,
            ego
        )

        assert result.x == pytest.approx(
            5.0,
            abs=1e-12
        )

        assert result.y == pytest.approx(
            0.0,
            abs=1e-12
        )

def test_world_to_body_left():

    ego = WorldState(
        x=10.0,
        y=5.0,
        theta=math.radians(90)
    )

    point = WorldPoint(
        x=5.0,
        y=5.0
    )

    result = world_to_body(
        point,
        ego
    )

    assert result.x == pytest.approx(
        0.0,
        abs=1e-12
    )

    assert result.y == pytest.approx(
        5.0,
        abs=1e-12
    )

def test_world_body_world_round_trip():

    ego = WorldState(
        x=10.0,
        y=5.0,
        theta=math.radians(37)
    )

    original = WorldPoint(
        x=14.3,
        y=9.7
    )

    body = world_to_body(
        original,
        ego
    )

    recovered = body_to_world(
        body,
        ego
    )

    assert recovered.x == pytest.approx(
        original.x,
        abs=1e-10
    )

    assert recovered.y == pytest.approx(
        original.y,
        abs=1e-10
    )

def test_body_world_body_round_trip():

    ego = WorldState(
        x=-3.0,
        y=7.0,
        theta=math.radians(-53)
    )

    original = BodyPoint(
        x=8.2,
        y=-2.4
    )

    world = body_to_world(
        original,
        ego
    )

    recovered = world_to_body(
        world,
        ego
    )

    assert recovered.x == pytest.approx(
        original.x,
        abs=1e-10
    )

    assert recovered.y == pytest.approx(
        original.y,
        abs=1e-10
    )

@pytest.mark.parametrize(
    "theta_deg",
    [
        -180,
        -90,
        -45,
        0,
        30,
        90,
        135,
        180,
    ]
)
def test_round_trip_multiple_headings(
        theta_deg
):

    ego = WorldState(
        x=3.0,
        y=-2.0,
        theta=math.radians(theta_deg)
    )

    original = WorldPoint(
        x=9.5,
        y=4.2
    )

    body = world_to_body(
        original,
        ego
    )

    recovered = body_to_world(
        body,
        ego
    )

    assert recovered.x == pytest.approx(
        original.x,
        abs=1e-10
    )

    assert recovered.y == pytest.approx(
        original.y,
        abs=1e-10
    )
