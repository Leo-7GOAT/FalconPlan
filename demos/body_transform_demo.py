import math

from coordinate_system import (
    WorldState,
    BodyPoint
)

from coordinate_system.body_transform import (
    world_to_body,
    body_to_world
)

#case1
ego = WorldState(
    x=10.0,
    y=5.0,
    theta=0.0
)

point = BodyPoint(
    x=5.0,
    y=0.0
)

result = body_to_world(
    point,
    ego
)
print("Case 1:", result)

ego = WorldState(
    x=10.0,
    y=5.0,
    theta=math.radians(90)
)

point = BodyPoint(
    x=5.0,
    y=0.0
)

result = body_to_world(
    point,
    ego
)

print("Case 2:", result)

ego = WorldState(
    x=10.0,
    y=5.0,
    theta=math.radians(90)
)

point = BodyPoint(
    x=0.0,
    y=5.0
)

result = body_to_world(
    point,
    ego
)

print("Case 3:", result)

# Case A
ego = WorldState(
    x=10.0,
    y=5.0,
    theta=0.0
)

result = world_to_body(
    15.0,
    5.0,
    ego
)

print("Case A:", result)


# Case B
ego = WorldState(
    x=10.0,
    y=5.0,
    theta=math.radians(90)
)

result = world_to_body(
    10.0,
    10.0,
    ego
)

print("Case B:", result)


# Case C
ego = WorldState(
    x=10.0,
    y=5.0,
    theta=math.radians(90)
)

result = world_to_body(
    5.0,
    5.0,
    ego
)

print("Case C:", result)


