from collections import deque


class SteeringCommandDelay:

    def __init__(
        self,
        delay_seconds: float,
        dt: float,
        initial_steer: float = 0.0,
    ):
        if delay_seconds < 0.0:
            raise ValueError(
                "delay_seconds must be non-negative"
            )

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        self.delay_seconds = delay_seconds
        self.dt = dt
        self.initial_steer = initial_steer

        delay_steps_float = (
            delay_seconds / dt
        )

        self.delay_steps = round(
            delay_steps_float
        )

        # 当前 baseline 要求 delay 是控制周期的整数倍。
        if abs(
            delay_steps_float
            - self.delay_steps
        ) > 1e-9:
            raise ValueError(
                "delay_seconds must be an "
                "integer multiple of dt"
            )

        self._buffer = deque(
            [
                initial_steer
                for _ in range(
                    self.delay_steps
                )
            ]
        )

    def reset(
        self,
        initial_steer: float | None = None,
    ) -> None:

        if initial_steer is None:
            initial_steer = (
                self.initial_steer
            )

        self._buffer = deque(
            [
                initial_steer
                for _ in range(
                    self.delay_steps
                )
            ]
        )

    def update(
        self,
        requested_steer: float,
    ) -> float:

        # No delay means transparent pass-through.
        if self.delay_steps == 0:
            return requested_steer

        self._buffer.append(
            requested_steer
        )

        delayed_steer = (
            self._buffer.popleft()
        )

        return delayed_steer