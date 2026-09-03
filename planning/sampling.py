from dataclasses import dataclass


@dataclass(frozen=True)
class LateralSamplingConfig:
    offsets: tuple[float, ...] = (
        -0.5,
        0.0,
        0.5,
    )


def sample_lateral_targets(
    lane_centers: list[float],
    config: LateralSamplingConfig,
) -> list[float]:

    if len(lane_centers) == 0:
        raise ValueError(
            "lane_centers must not be empty"
        )

    targets = []

    for lane_center in lane_centers:

        for offset in config.offsets:

            targets.append(
                lane_center + offset
            )

    # 去重并排序，避免多个车道采样产生重复点
    return sorted(
        set(targets)
    )

