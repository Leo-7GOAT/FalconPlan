import math

import numpy as np
from scipy.linalg import solve_discrete_are


class LQRController:

    def __init__(
        self,
        q_cross_track: float,
        q_heading: float,
        r_steer: float,
        max_steer: float,
    ):
        if q_cross_track <= 0.0:
            raise ValueError(
                "q_cross_track must be positive"
            )

        if q_heading <= 0.0:
            raise ValueError(
                "q_heading must be positive"
            )

        if r_steer <= 0.0:
            raise ValueError(
                "r_steer must be positive"
            )

        if max_steer <= 0.0:
            raise ValueError(
                "max_steer must be positive"
            )

        self.Q = np.diag([
            q_cross_track,
            q_heading,
        ])

        self.R = np.array([
            [r_steer]
        ], dtype=float)

        self.max_steer = max_steer

    @staticmethod
    def build_discrete_model(
        speed: float,
        wheel_base: float,
        dt: float,
    ):

        if speed <= 0.0:
            raise ValueError(
                "speed must be positive"
            )

        if wheel_base <= 0.0:
            raise ValueError(
                "wheel_base must be positive"
            )

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        A = np.array([
            [1.0, speed * dt],
            [0.0, 1.0],
        ])

        B = np.array([
            [0.0],
            [
                -speed
                * dt
                / wheel_base
            ],
        ])

        return A, B

    def compute_gain(
        self,
        speed: float,
        wheel_base: float,
        dt: float,
    ):

        A, B = self.build_discrete_model(
            speed=speed,
            wheel_base=wheel_base,
            dt=dt,
        )

        P = solve_discrete_are(
            A,
            B,
            self.Q,
            self.R,
        )

        K = (
            np.linalg.inv(
                self.R
                + B.T @ P @ B
            )
            @ (
                B.T
                @ P
                @ A
            )
        )

        return K

    def update(
        self,
        cross_track_error: float,
        heading_error: float,
        speed: float,
        wheel_base: float,
        dt: float,
        reference_curvature: float = 0.0,
    ) -> float:

        K = self.compute_gain(
            speed=speed,
            wheel_base=wheel_base,
            dt=dt,
        )

        state_error = np.array([
            [cross_track_error],
            [heading_error],
        ])

        feedback = float(
            (-K @ state_error).item()
        )

        feedforward = math.atan(
            wheel_base
            * reference_curvature
        )

        steer = (
            feedforward
            + feedback
        )

        steer = max(
            -self.max_steer,
            min(
                self.max_steer,
                steer,
            ),
        )

        return steer