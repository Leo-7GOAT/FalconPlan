import numpy as np

class QuinticPolynomial:

    def __init__(
        self,
        x0: float,
        v0: float,
        a0: float,
        x1: float,
        v1: float,
        a1: float,
        T: float,
    ):

        if T <= 0:
            raise ValueError(
                "T must be positive"
            )

        # 起点三个边界条件直接确定前三个系数
        self.c0 = x0
        self.c1 = v0
        self.c2 = a0 / 2.0

        A = np.array([
            [T**3, T**4, T**5],
            [3*T**2, 4*T**3, 5*T**4],
            [6*T, 12*T**2, 20*T**3],
        ], dtype=float)

        b = np.array([
            x1 - (
                self.c0
                + self.c1 * T
                + self.c2 * T**2
            ),

            v1 - (
                self.c1
                + 2 * self.c2 * T
            ),

            a1 - (
                2 * self.c2
            ),
        ], dtype=float)

        self.c3, self.c4, self.c5 = (
            np.linalg.solve(A, b)
        )

    def position(self, t: float) -> float:
        return (
                self.c0
                + self.c1 * t
                + self.c2 * t ** 2
                + self.c3 * t ** 3
                + self.c4 * t ** 4
                + self.c5 * t ** 5
        )

    def velocity(self, t: float) -> float:
        return (
                self.c1
                + 2 * self.c2 * t
                + 3 * self.c3 * t ** 2
                + 4 * self.c4 * t ** 3
                + 5 * self.c5 * t ** 4
        )

    def acceleration(self, t: float) -> float:
        return (
                2 * self.c2
                + 6 * self.c3 * t
                + 12 * self.c4 * t ** 2
                + 20 * self.c5 * t ** 3
        )

    def jerk(self, t: float) -> float:
        return (
                6 * self.c3
                + 24 * self.c4 * t
                + 60 * self.c5 * t ** 2
        )

class QuarticPolynomial:

    def __init__(
        self,
        x0: float,
        v0: float,
        a0: float,
        v1: float,
        a1: float,
        T: float,
    ):
        if T <= 0:
            raise ValueError(
                "T must be positive"
            )

        self.c0 = x0
        self.c1 = v0
        self.c2 = a0 / 2.0

        A = np.array([
            [3 * T ** 2, 4 * T ** 3],
            [6 * T, 12 * T ** 2],
        ], dtype=float)

        b = np.array([
            v1 - (
                    self.c1
                    + 2 * self.c2 * T
            ),

            a1 - (
                    2 * self.c2
            ),
        ], dtype=float)

        self.c3, self.c4 = np.linalg.solve(A, b)

    def position(self, t: float) -> float:
        return (
                self.c0
                + self.c1 * t
                + self.c2 * t ** 2
                + self.c3 * t ** 3
                + self.c4 * t ** 4
        )

    def velocity(self, t: float) -> float:
        return (
                self.c1
                + 2 * self.c2 * t
                + 3 * self.c3 * t ** 2
                + 4 * self.c4 * t ** 3
        )

    def acceleration(self, t: float) -> float:
        return (
                2 * self.c2
                + 6 * self.c3 * t
                + 12 * self.c4 * t ** 2
        )

    def jerk(self, t: float) -> float:
        return (
                6 * self.c3
                + 24 * self.c4 * t
        )
