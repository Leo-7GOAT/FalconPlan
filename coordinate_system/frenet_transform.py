import math

import numpy as np
from scipy.optimize import minimize_scalar

from .models import FrenetState, WorldState
from .reference_line import ReferenceLine


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error(a: float, b: float) -> float:
    """返回两个角之间的最小有符号差值。"""
    return normalize_angle(a - b)


class FrenetTransformer:
    """
    World(x, y, theta) <-> Frenet(s, d, d')

    约定：
        d > 0：参考线左侧
        d < 0：参考线右侧
        d' = dd/ds

    核心公式：
        d' = (1 - kappa_r * d) * tan(theta_v - theta_r)

        theta_v
        = theta_r + atan2(d', 1 - kappa_r * d)
    """

    def __init__(
        self,
        reference_line: ReferenceLine,
        projection_samples: int = 500,
        singular_eps: float = 1e-6,
    ) -> None:
        if projection_samples < 10:
            raise ValueError("projection_samples 建议至少为 10")
        if singular_eps <= 0.0:
            raise ValueError("singular_eps 必须大于 0")

        self.reference_line = reference_line
        self.projection_samples = int(projection_samples)
        self.singular_eps = float(singular_eps)

    def _validate_factor(self, factor: float) -> None:
        # Frenet 局部坐标映射要保持正常方向，工程上要求 factor > 0。
        if factor <= self.singular_eps:
            raise ValueError(
                "Frenet奇点/失效区域：1 - kappa_r*d "
                f"= {factor:.6e}，必须大于 {self.singular_eps:.1e}"
            )

    def _project_s(self, x: float, y: float) -> float:
        """
        在连续参考线上寻找距离世界点 (x, y) 最近的 s。

        方法：
        1. 全局粗采样定位附近区间；
        2. 在局部区间用 bounded minimize_scalar 连续优化；
        3. 同时比较区间端点，避免最优点恰好落在边界时被漏掉。
        """
        s_samples = np.linspace(
            0.0,
            self.reference_line.length,
            self.projection_samples,
        )

        x_samples = self.reference_line.x_of_s(s_samples)
        y_samples = self.reference_line.y_of_s(s_samples)

        distance_sq = (
            (x_samples - x) ** 2
            + (y_samples - y) ** 2
        )

        nearest_index = int(np.argmin(distance_sq))

        left_index = max(0, nearest_index - 1)
        right_index = min(
            len(s_samples) - 1,
            nearest_index + 1,
        )

        s_left = float(s_samples[left_index])
        s_right = float(s_samples[right_index])

        def objective(s: float) -> float:
            x_r = float(self.reference_line.x_of_s(s))
            y_r = float(self.reference_line.y_of_s(s))
            return (x_r - x) ** 2 + (y_r - y) ** 2

        # 如果粗搜索已经落在退化区间，直接返回该点。
        if abs(s_right - s_left) < 1e-15:
            return s_left

        result = minimize_scalar(
            objective,
            bounds=(s_left, s_right),
            method="bounded",
            options={"xatol": 1e-11},
        )

        # bounded 优化对边界最优值可能只逼近不命中，所以显式比较。
        candidates = [
            float(result.x),
            s_left,
            s_right,
        ]

        best_s = min(candidates, key=objective)

        # 最终再限制到合法参考线范围。
        return min(
            max(float(best_s), 0.0),
            self.reference_line.length,
        )

    def frenet_to_world(
        self,
        state: FrenetState,
    ) -> WorldState:
        ref = self.reference_line.query(state.s)

        factor = 1.0 - ref.kappa * state.d
        self._validate_factor(factor)

        x_w = ref.x - state.d * math.sin(ref.theta)
        y_w = ref.y + state.d * math.cos(ref.theta)

        delta_theta = math.atan2(
            state.d_prime,
            factor,
        )

        theta_v = normalize_angle(
            ref.theta + delta_theta
        )

        return WorldState(
            x=x_w,
            y=y_w,
            theta=theta_v,
        )

    def world_to_frenet(
        self,
        state: WorldState,
    ) -> FrenetState:
        s = self._project_s(state.x, state.y)

        ref = self.reference_line.query(s)

        delta_x = state.x - ref.x
        delta_y = state.y - ref.y

        # 法向投影，左正右负
        d = (
            -delta_x * math.sin(ref.theta)
            + delta_y * math.cos(ref.theta)
        )

        factor = 1.0 - ref.kappa * d
        self._validate_factor(factor)

        delta_theta = normalize_angle(
            state.theta - ref.theta
        )

        # tan(theta_v - theta_r) 在 ±90° 处不适合该 Frenet 表达。
        if abs(delta_theta) >= math.pi / 2 - 1e-6:
            raise ValueError(
                "车辆航向与参考线切向夹角过大，"
                "当前 Frenet (s,d,d') 表达不唯一/不稳定"
            )

        d_prime = (
            factor * math.tan(delta_theta)
        )

        return FrenetState(
            s=s,
            d=d,
            d_prime=d_prime,
        )
