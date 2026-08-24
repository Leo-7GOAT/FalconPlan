import math
from typing import Sequence, Tuple

import numpy as np
from scipy.interpolate import CubicSpline

from .models import ReferenceState


Point2D = Tuple[float, float]


def calculate_cumulative_parameter(
    reference_path: Sequence[Point2D],
) -> np.ndarray:
    """
    用离散 waypoint 折线长度生成临时参数 u。

    注意：
    这里的 u 只是第一次 spline 的参数，不是最终的真实弧长 s。
    """
    if len(reference_path) < 2:
        raise ValueError("reference_path 至少需要 2 个 waypoint")

    u_list = [0.0]

    for i in range(len(reference_path) - 1):
        x0, y0 = reference_path[i]
        x1, y1 = reference_path[i + 1]

        delta_u = math.hypot(x1 - x0, y1 - y0)

        if delta_u < 1e-12:
            raise ValueError(
                f"reference_path 中 P{i} 和 P{i + 1} 重合"
            )

        u_list.append(u_list[-1] + delta_u)

    return np.asarray(u_list, dtype=float)


class ReferenceLine:
    """
    平滑、近似弧长参数化的二维参考线。

    构建过程：
        waypoints
        -> 临时参数 u
        -> CubicSpline x(u), y(u)
        -> 密集采样平滑曲线
        -> 重新累计真实弧长 s
        -> CubicSpline x(s), y(s)

    这样后续可以直接查询：
        x(s), y(s), theta_r(s), kappa_r(s)
    """

    def __init__(
        self,
        reference_path: Sequence[Point2D],
        reparam_samples: int = 5000,
    ) -> None:
        if reparam_samples < 100:
            raise ValueError("reparam_samples 建议至少为 100")

        self.reference_path = tuple(
            (float(x), float(y)) for x, y in reference_path
        )

        u_list = calculate_cumulative_parameter(self.reference_path)

        x_list = np.asarray([p[0] for p in self.reference_path], dtype=float)
        y_list = np.asarray([p[1] for p in self.reference_path], dtype=float)

        # 第一层 spline：x(u), y(u)
        x_of_u = CubicSpline(u_list, x_list)
        y_of_u = CubicSpline(u_list, y_list)

        # 沿平滑曲线密集采样
        u_samples = np.linspace(
            u_list[0],
            u_list[-1],
            reparam_samples,
        )

        x_samples = x_of_u(u_samples)
        y_samples = y_of_u(u_samples)

        # 对平滑曲线重新累计真实弧长
        delta_x = np.diff(x_samples)
        delta_y = np.diff(y_samples)
        delta_s = np.hypot(delta_x, delta_y)

        s_samples = np.concatenate(
            ([0.0], np.cumsum(delta_s))
        )

        if np.any(np.diff(s_samples) <= 0.0):
            raise ValueError("参考线弧长参数必须严格递增")

        # 第二层 spline：真正基于弧长的 x(s), y(s)
        self.s_grid = s_samples
        self.x_of_s = CubicSpline(s_samples, x_samples)
        self.y_of_s = CubicSpline(s_samples, y_samples)

    @property
    def length(self) -> float:
        return float(self.s_grid[-1])

    def _check_s(self, s: float) -> None:
        if s < 0.0 or s > self.length:
            raise ValueError(
                f"s={s:.6f} 超出参考线范围 [0, {self.length:.6f}]"
            )

    def query(self, s: float) -> ReferenceState:
        """
        查询参考线在弧长 s 处的世界坐标、航向角、曲率。
        """
        s = float(s)
        self._check_s(s)

        x_r = float(self.x_of_s(s))
        y_r = float(self.y_of_s(s))

        dx_ds = float(self.x_of_s(s, 1))
        dy_ds = float(self.y_of_s(s, 1))

        d2x_ds2 = float(self.x_of_s(s, 2))
        d2y_ds2 = float(self.y_of_s(s, 2))

        tangent_norm_sq = dx_ds * dx_ds + dy_ds * dy_ds

        if tangent_norm_sq < 1e-12:
            raise ValueError(
                "参考线一阶导数过小，无法计算航向和曲率"
            )

        theta_r = math.atan2(dy_ds, dx_ds)

        kappa_r = (
            dx_ds * d2y_ds2
            - dy_ds * d2x_ds2
        ) / (tangent_norm_sq ** 1.5)

        return ReferenceState(
            x=x_r,
            y=y_r,
            theta=theta_r,
            kappa=kappa_r,
        )

    def arc_length_speed(self, s):
        """
        返回 ||dr/ds||。

        若弧长重参数化正确，应当约等于 1。
        支持标量或 numpy 数组输入。
        """
        dx_ds = self.x_of_s(s, 1)
        dy_ds = self.y_of_s(s, 1)
        return np.hypot(dx_ds, dy_ds)

    def sample(self, num: int = 500):
        """
        均匀采样参考线，返回 s, x, y。
        """
        if num < 2:
            raise ValueError("num 至少为 2")

        s = np.linspace(0.0, self.length, num)
        return s, self.x_of_s(s), self.y_of_s(s)
