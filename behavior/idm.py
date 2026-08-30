import math

from .models import IDMParams

def compute_desired_gap(
        ego_speed: float,
        front_speed: float,
        params: IDMParams
) -> float:
    delta_v = ego_speed - front_speed

    s_star = params.min_gap + max(
        0.0,
        ego_speed * params.time_headway
        + (ego_speed * delta_v) / (
                2 * math.sqrt(
            params.max_accel
            * params.comfortable_decel
        )
        )
    )

    return s_star

def compute_idm_acceleration(
        ego_speed: float,
        gap: float | None,
        front_speed: float | None,
        params: IDMParams
) -> float:
    if (gap is None) != (front_speed is None):
        raise ValueError(
            "gap and front_speed must both be provided or both be None"
        )

    if gap is None and front_speed is None:

        a_free = params.max_accel * (1 - (ego_speed / params.desired_speed) ** params.delta)
        return a_free

    else:
        if gap <= 0:
            raise ValueError(
                "gap must be positive when a front vehicle is present"
            )

        s_star = compute_desired_gap(ego_speed, front_speed, params)

        free_term = 1 - (
                ego_speed / params.desired_speed
        ) ** params.delta

        interaction_term = (s_star / gap) ** 2

        acceleration = params.max_accel * (
                free_term
                - interaction_term
        )
        return acceleration