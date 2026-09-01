import math

from coordinate_system.frenet_transform import (
    FrenetTransformer,
    normalize_angle,
)

from coordinate_system.models import (
    FrenetState,
)

from coordinate_system.reference_line import (
    ReferenceLine,
)

from .trajectory import (
    FrenetTrajectory,
)


def _validate_frenet_trajectory(
    trajectory: FrenetTrajectory,
) -> int:
    """
    Validate that all Frenet trajectory arrays have
    the same number of samples.

    Returns
    -------
    int
        Number of trajectory samples.
    """

    sample_fields = {
        "t": trajectory.t,

        "s": trajectory.s,
        "s_d": trajectory.s_d,
        "s_dd": trajectory.s_dd,
        "s_ddd": trajectory.s_ddd,

        "d": trajectory.d,
        "d_d": trajectory.d_d,
        "d_dd": trajectory.d_dd,
        "d_ddd": trajectory.d_ddd,
    }

    lengths = {
        name: len(values)
        for name, values in sample_fields.items()
    }

    unique_lengths = set(
        lengths.values()
    )

    if len(unique_lengths) != 1:
        raise ValueError(
            "Frenet trajectory sample lengths "
            f"do not match: {lengths}"
        )

    sample_count = len(
        trajectory.t
    )

    if sample_count == 0:
        raise ValueError(
            "trajectory must contain samples"
        )

    return sample_count


def _compute_discrete_curvature(
    trajectory: FrenetTrajectory,
    distance_eps: float,
) -> list[float]:
    """
    Compute curvature from the projected World trajectory.

    Discrete approximation:

        kappa ~= delta_yaw / delta_l

    where delta_l is the World-frame distance between
    adjacent trajectory samples.
    """

    n = len(
        trajectory.x
    )

    if n == 0:
        return []

    if n == 1:
        return [0.0]

    curvature = [
        0.0
        for _ in range(n)
    ]

    for i in range(n - 1):

        dx = (
            trajectory.x[i + 1]
            - trajectory.x[i]
        )

        dy = (
            trajectory.y[i + 1]
            - trajectory.y[i]
        )

        delta_l = math.hypot(
            dx,
            dy,
        )

        if delta_l <= distance_eps:
            raise ValueError(
                "adjacent World trajectory points "
                "are too close to compute curvature"
            )

        delta_yaw = normalize_angle(
            trajectory.yaw[i + 1]
            - trajectory.yaw[i]
        )

        curvature[i] = (
            delta_yaw
            / delta_l
        )

    # The final point has no next sample.
    # Use the previous segment curvature.
    curvature[-1] = curvature[-2]

    return curvature


def project_trajectory_to_world(
    trajectory: FrenetTrajectory,
    reference_line: ReferenceLine,
    speed_eps: float = 1e-6,
    distance_eps: float = 1e-9,
) -> FrenetTrajectory:
    """
    Project a time-parameterized Frenet trajectory into
    the World coordinate system.

    Important relation:

        d_prime = dd/ds
                = (dd/dt) / (ds/dt)
                = d_d / s_d

    The existing FrenetTransformer is then used for:

        (s, d, d_prime)
            ->
        (x, y, yaw)

    Finally, discrete curvature is computed from the
    projected World trajectory.
    """

    if speed_eps <= 0.0:
        raise ValueError(
            "speed_eps must be positive"
        )

    if distance_eps <= 0.0:
        raise ValueError(
            "distance_eps must be positive"
        )

    sample_count = (
        _validate_frenet_trajectory(
            trajectory
        )
    )

    transformer = FrenetTransformer(
        reference_line=reference_line,
    )

    # Important:
    # projection may be called more than once.
    # Do not append duplicate World samples.
    trajectory.x.clear()
    trajectory.y.clear()
    trajectory.yaw.clear()
    trajectory.curvature.clear()

    for i in range(sample_count):

        s = trajectory.s[i]
        d = trajectory.d[i]

        s_dot = trajectory.s_d[i]
        d_dot = trajectory.d_d[i]

        # ----------------------------------------------------
        # Convert time derivative to spatial derivative.
        #
        # d' = (dd/dt) / (ds/dt)
        # ----------------------------------------------------

        if abs(s_dot) <= speed_eps:
            raise ValueError(
                "longitudinal speed is too small "
                "to compute d_prime = d_dot / s_dot"
            )

        d_prime = (
            d_dot
            / s_dot
        )

        frenet_state = FrenetState(
            s=s,
            d=d,
            d_prime=d_prime,
        )

        world_state = (
            transformer.frenet_to_world(
                frenet_state
            )
        )

        trajectory.x.append(
            world_state.x
        )

        trajectory.y.append(
            world_state.y
        )

        trajectory.yaw.append(
            world_state.theta
        )

    trajectory.curvature = (
        _compute_discrete_curvature(
            trajectory=trajectory,
            distance_eps=distance_eps,
        )
    )

    return trajectory