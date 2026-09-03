import math

class StanleyController:

    def __init__(
        self,
        k: float,
        softening: float,
        max_steer: float,
    ):
        if k <= 0.0:
            raise ValueError(
                "k must be positive"
            )

        if softening <= 0.0:
            raise ValueError(
                "softening must be positive"
            )

        if max_steer <= 0.0:
            raise ValueError(
                "max_steer must be positive"
            )

        self.k = k
        self.softening = softening
        self.max_steer = max_steer

    @staticmethod
    def normalize_angle(
        angle: float,
    ) -> float:

        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    def update(
            self,
            heading_error: float,
            cross_track_error: float,
            speed: float,
    ) -> float:
        if speed < 0.0:
            raise ValueError("speed must be positive")

        heading_error = self.normalize_angle(heading_error)
        steer = heading_error + math.atan2((self.k * cross_track_error),
                                           (speed + self.softening))
        steer = max(
            -self.max_steer,
            min(self.max_steer, steer),
        )

        return steer

    def compute_errors(
            self,
            state,
            trajectory_x,
            trajectory_y,
            trajectory_yaw,
            wheel_base,
    ):
        if not trajectory_x:
            raise ValueError(
                "trajectory must not be empty"
            )

        if not (
                len(trajectory_x)
                == len(trajectory_y)
                == len(trajectory_yaw)
        ):
            raise ValueError(
                "trajectory lengths must match"
            )

        if wheel_base <= 0.0:
            raise ValueError(
                "wheel_base must be positive"
            )

        front_x = (
                state.x
                + wheel_base
                * math.cos(state.psi)
        )

        front_y = (
                state.y
                + wheel_base
                * math.sin(state.psi)
        )

        nearest_index = min(
            range(len(trajectory_x)),
            key=lambda i: (
                    (trajectory_x[i] - front_x) ** 2
                    + (trajectory_y[i] - front_y) ** 2
            ),
        )
        path_x = trajectory_x[
            nearest_index
        ]

        path_y = trajectory_y[
            nearest_index
        ]

        path_yaw = trajectory_yaw[
            nearest_index
        ]

        heading_error = self.normalize_angle(
            path_yaw - state.psi
        )

        cross_track_error = (
                (path_x - front_x)
                * (-math.sin(path_yaw))
                +
                (path_y - front_y)
                * math.cos(path_yaw)
        )

        return (
            nearest_index,
            heading_error,
            cross_track_error,
        )

