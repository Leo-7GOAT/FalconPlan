from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrackingMetrics:
    cross_track_rmse: float
    max_cross_track_error: float
    final_cross_track_error: float

    heading_rmse: float
    max_heading_error: float

    max_steering: float

    steering_rate_rmse: float
    max_steering_rate: float


def compute_tracking_metrics(
    cross_track_errors,
    heading_errors,
    steering_commands,
    dt: float,
) -> TrackingMetrics:

    if dt <= 0.0:
        raise ValueError(
            "dt must be positive"
        )

    if len(cross_track_errors) == 0:
        raise ValueError(
            "tracking data must not be empty"
        )

    if not (
        len(cross_track_errors)
        == len(heading_errors)
        == len(steering_commands)
    ):
        raise ValueError(
            "tracking log lengths must match"
        )

    cte = np.asarray(
        cross_track_errors,
        dtype=float,
    )

    heading = np.asarray(
        heading_errors,
        dtype=float,
    )

    steering = np.asarray(
        steering_commands,
        dtype=float,
    )

    cross_track_rmse = float(
        np.sqrt(
            np.mean(
                cte ** 2
            )
        )
    )

    max_cross_track_error = float(
        np.max(
            np.abs(cte)
        )
    )

    heading_rmse = float(
        np.sqrt(
            np.mean(
                heading ** 2
            )
        )
    )

    max_heading_error = float(
        np.max(
            np.abs(heading)
        )
    )

    max_steering = float(
        np.max(
            np.abs(steering)
        )
    )

    if len(steering) > 1:

        steering_rate = (
            np.diff(steering)
            / dt
        )

        steering_rate_rmse = float(
            np.sqrt(
                np.mean(
                    steering_rate ** 2
                )
            )
        )

        max_steering_rate = float(
            np.max(
                np.abs(
                    steering_rate
                )
            )
        )

    else:

        steering_rate_rmse = 0.0
        max_steering_rate = 0.0

    return TrackingMetrics(
        cross_track_rmse=(
            cross_track_rmse
        ),

        max_cross_track_error=(
            max_cross_track_error
        ),

        final_cross_track_error=float(
            cte[-1]
        ),

        heading_rmse=(
            heading_rmse
        ),

        max_heading_error=(
            max_heading_error
        ),

        max_steering=(
            max_steering
        ),

        steering_rate_rmse=(
            steering_rate_rmse
        ),

        max_steering_rate=(
            max_steering_rate
        ),
    )