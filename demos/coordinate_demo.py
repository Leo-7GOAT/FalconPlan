import math

from coordinate_system import (
    FrenetState,
    FrenetTransformer,
    ReferenceLine,
)


REFERENCE_PATH = [
    (0, 0),
    (5, 1),
    (10, 4),
    (15, 8),
    (20, 10),
]


def main():
    reference_line = ReferenceLine(REFERENCE_PATH)
    transformer = FrenetTransformer(reference_line)

    original_frenet = FrenetState(
        s=10.0,
        d=1.0,
        d_prime=0.2,
    )

    world = transformer.frenet_to_world(
        original_frenet
    )

    recovered_frenet = transformer.world_to_frenet(
        world
    )

    print("Original Frenet:", original_frenet)
    print(
        "World:",
        f"x={world.x:.9f}, "
        f"y={world.y:.9f}, "
        f"theta={math.degrees(world.theta):.9f} deg",
    )
    print("Recovered Frenet:", recovered_frenet)


if __name__ == "__main__":
    main()
