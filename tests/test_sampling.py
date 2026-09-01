import pytest

from planning.sampling import (
    LateralSamplingConfig,
    sample_lateral_targets,
)


def test_two_lane_sampling():

    result = sample_lateral_targets(
        lane_centers=[
            0.0,
            3.5,
        ],
        config=LateralSamplingConfig(),
    )

    assert result == pytest.approx([
        -0.5,
        0.0,
        0.5,
        3.0,
        3.5,
        4.0,
    ])


def test_single_lane_sampling():

    result = sample_lateral_targets(
        lane_centers=[0.0],
        config=LateralSamplingConfig(
            offsets=(
                -0.3,
                0.0,
                0.3,
            )
        ),
    )

    assert result == pytest.approx([
        -0.3,
        0.0,
        0.3,
    ])


def test_duplicate_targets_removed():

    result = sample_lateral_targets(
        lane_centers=[
            0.0,
            1.0,
        ],
        config=LateralSamplingConfig(
            offsets=(
                0.0,
                1.0,
            )
        ),
    )

    assert result == pytest.approx([
        0.0,
        1.0,
        2.0,
    ])


def test_empty_lane_centers_rejected():

    with pytest.raises(ValueError):

        sample_lateral_targets(
            lane_centers=[],
            config=LateralSamplingConfig(),
        )