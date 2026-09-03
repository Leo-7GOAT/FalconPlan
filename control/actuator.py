import math


class SteeringActuator:

    def __init__(
        self,
        max_steer: float,
        max_steer_rate: float,
    ):
        if max_steer <= 0.0:
            raise ValueError(
                "max_steer must be positive"
            )

        if max_steer_rate <= 0.0:
            raise ValueError(
                "max_steer_rate must be positive"
            )

        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate

        self.current_steer = 0.0

    def reset(
        self,
        steer: float = 0.0,
    ) -> None:

        self.current_steer = max(
            -self.max_steer,
            min(
                self.max_steer,
                steer,
            ),
        )

    def update(
        self,
        target_steer: float,
        dt: float,
    ) -> float:

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        target_steer = max(
            -self.max_steer,
            min(
                self.max_steer,
                target_steer,
            ),
        )

        max_delta = (
            self.max_steer_rate
            * dt
        )

        error = (
            target_steer
            - self.current_steer
        )

        steer_change = max(
            -max_delta,
            min(
                max_delta,
                error,
            ),
        )

        self.current_steer += (
            steer_change
        )

        return self.current_steer