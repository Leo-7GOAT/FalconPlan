import math

from dataclasses import dataclass


@dataclass(frozen=True)
class CircularObstacle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class CollisionParams:
    vehicle_radius: float
    safety_margin: float = 0.0


def point_to_segment_distance(
    px: float,
    py: float,

    x1: float,
    y1: float,

    x2: float,
    y2: float,
) -> float:
    """
    返回点 P 到线段 AB 的最短距离。
    """

    # AB
    dx = x2 - x1
    dy = y2 - y1

    # |AB|^2
    length_sq = (
        dx * dx
        + dy * dy
    )

    # A、B 几乎重合，线段退化成一个点
    if length_sq <= 1e-12:
        return math.hypot(
            px - x1,
            py - y1,
        )

    # lambda*
    projection = (
        (px - x1) * dx
        + (py - y1) * dy
    ) / length_sq

    # 限制在线段范围 [0, 1]
    projection = max(
        0.0,
        min(
            1.0,
            projection,
        ),
    )

    # 最近点 Q
    closest_x = (
        x1
        + projection * dx
    )

    closest_y = (
        y1
        + projection * dy
    )

    # |PQ|
    return math.hypot(
        px - closest_x,
        py - closest_y,
    )

    projection = (
                         (px - x1) * dx
                         + (py - y1) * dy
                 ) / length_sq

    projection = max(
        0.0,
        min(1.0, projection),
    )

    closest_x = x1 + projection * dx
    closest_y = y1 + projection * dy

    math.hypot(
        px - closest_x,
        py - closest_y,
    )

def trajectory_collides(
    trajectory,
    obstacles: list[CircularObstacle],
    params: CollisionParams,
) -> bool:

    if len(trajectory.x) == 0:
        raise ValueError(
            "trajectory must contain World coordinates"
        )

    if len(trajectory.x) != len(trajectory.y):
        raise ValueError(
            "trajectory x/y lengths do not match"
        )

    if params.vehicle_radius < 0.0:
        raise ValueError(
            "vehicle_radius must be non-negative"
        )

    if params.safety_margin < 0.0:
        raise ValueError(
            "safety_margin must be non-negative"
        )

    for obstacle in obstacles:

        if obstacle.radius < 0.0:
            raise ValueError(
                "obstacle radius must be non-negative"
            )

        collision_distance = (
            params.vehicle_radius
            + obstacle.radius
            + params.safety_margin
        )

        # 只有一个轨迹点时
        if len(trajectory.x) == 1:

            distance = math.hypot(
                trajectory.x[0] - obstacle.x,
                trajectory.y[0] - obstacle.y,
            )

            if distance <= collision_distance:
                return True

            continue

        # 检查每一段轨迹
        for i in range(
            len(trajectory.x) - 1
        ):

            distance = point_to_segment_distance(
                px=obstacle.x,
                py=obstacle.y,

                x1=trajectory.x[i],
                y1=trajectory.y[i],

                x2=trajectory.x[i + 1],
                y2=trajectory.y[i + 1],
            )

            if distance <= collision_distance:
                return True

    return False

def filter_collision_free_trajectories(
    trajectories,
    obstacles: list[CircularObstacle],
    params: CollisionParams,
):
    return [
        trajectory
        for trajectory in trajectories
        if not trajectory_collides(
            trajectory=trajectory,
            obstacles=obstacles,
            params=params,
        )
    ]

def trajectory_min_distance_to_obstacle(
    trajectory,
    obstacle: CircularObstacle,
) -> float:

    if len(trajectory.x) == 0:
        raise ValueError(
            "trajectory must contain World coordinates"
        )

    if len(trajectory.x) != len(trajectory.y):
        raise ValueError(
            "trajectory x/y lengths do not match"
        )

    # 单点轨迹
    if len(trajectory.x) == 1:
        return math.hypot(
            trajectory.x[0] - obstacle.x,
            trajectory.y[0] - obstacle.y,
        )

    min_distance = math.inf

    for i in range(
        len(trajectory.x) - 1
    ):

        distance = point_to_segment_distance(
            px=obstacle.x,
            py=obstacle.y,

            x1=trajectory.x[i],
            y1=trajectory.y[i],

            x2=trajectory.x[i + 1],
            y2=trajectory.y[i + 1],
        )

        min_distance = min(
            min_distance,
            distance,
        )

    return min_distance