import math


class TrajectoryTracker:

    def __init__(
        self,
        trajectory_x,
        trajectory_y,
        trajectory_yaw,
        wheel_base: float,
        search_window: int = 20,
    ):
        if len(trajectory_x) == 0:
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

        if search_window <= 0:
            raise ValueError(
                "search_window must be positive"
            )

        self.trajectory_x = trajectory_x
        self.trajectory_y = trajectory_y
        self.trajectory_yaw = trajectory_yaw

        self.wheel_base = wheel_base
        self.search_window = search_window

        self.previous_index = 0

    @staticmethod
    def normalize_angle(
        angle: float,
    ) -> float:

        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    def reset(self):

        self.previous_index = 0

    def compute_errors(
        self,
        state,
    ):

        front_x = (
            state.x
            + self.wheel_base
            * math.cos(state.psi)
        )

        front_y = (
            state.y
            + self.wheel_base
            * math.sin(state.psi)
        )

        search_start = (
            self.previous_index
        )

        search_end = min(
            search_start
            + self.search_window,
            len(self.trajectory_x),
        )

        nearest_index = min(
            range(
                search_start,
                search_end,
            ),
            key=lambda i: (
                (
                    self.trajectory_x[i]
                    - front_x
                ) ** 2
                +
                (
                    self.trajectory_y[i]
                    - front_y
                ) ** 2
            ),
        )

        # Progress can never move backward.
        nearest_index = max(
            nearest_index,
            self.previous_index,
        )

        self.previous_index = (
            nearest_index
        )

        path_x = self.trajectory_x[
            nearest_index
        ]

        path_y = self.trajectory_y[
            nearest_index
        ]

        path_yaw = self.trajectory_yaw[
            nearest_index
        ]

        heading_error = (
            self.normalize_angle(
                path_yaw
                - state.psi
            )
        )

        cross_track_error = (
            (
                path_x - front_x
            )
            * (
                -math.sin(path_yaw)
            )
            +
            (
                path_y - front_y
            )
            * math.cos(path_yaw)
        )

        return (
            nearest_index,
            heading_error,
            cross_track_error,
        )