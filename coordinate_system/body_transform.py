import math

from .models import (
    BodyPoint,
    WorldState,
    WorldPoint
)


def world_to_body(
    point: WorldPoint,
    ego: WorldState
) -> BodyPoint:

    delta_x = point.x - ego.x
    delta_y = point.y - ego.y

    x_body = delta_x * math.cos(ego.theta) + delta_y * math.sin(ego.theta)
    y_body = - delta_x * math.sin(ego.theta) + delta_y * math.cos(ego.theta)

    return BodyPoint(
        x=x_body,
        y=y_body
    )


def body_to_world(
        point: BodyPoint,
        ego: WorldState
) -> WorldPoint:
    delta_x = (
            point.x * math.cos(ego.theta)
            - point.y * math.sin(ego.theta)
    )

    delta_y = (
            point.x * math.sin(ego.theta)
            + point.y * math.cos(ego.theta)
    )

    world_x = ego.x + delta_x
    world_y = ego.y + delta_y

    return WorldPoint(
        x=world_x,
        y=world_y
    )
