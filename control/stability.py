from dataclasses import dataclass


@dataclass(frozen=True)
class TrackingAcceptanceCriteria:

    max_cross_track_error: float = 2.5

    max_heading_error_deg: float = 20.0

    final_cross_track_error: float = 0.5


@dataclass(frozen=True)
class TrackingAcceptanceResult:

    passed: bool

    completed: bool

    cross_track_ok: bool
    heading_ok: bool
    final_error_ok: bool

    reason: str


def evaluate_tracking_run(
    *,
    completed: bool,
    max_cross_track_error: float,
    max_heading_error_deg: float,
    final_cross_track_error: float,
    criteria: TrackingAcceptanceCriteria,
) -> TrackingAcceptanceResult:

    cross_track_ok = (
        abs(max_cross_track_error)
        <= criteria.max_cross_track_error
    )

    heading_ok = (
        abs(max_heading_error_deg)
        <= criteria.max_heading_error_deg
    )

    final_error_ok = (
        abs(final_cross_track_error)
        <= criteria.final_cross_track_error
    )

    passed = (
        completed
        and cross_track_ok
        and heading_ok
        and final_error_ok
    )

    if not completed:
        reason = (
            "trajectory was not completed"
        )

    elif not cross_track_ok:
        reason = (
            "maximum cross-track error "
            "exceeded limit"
        )

    elif not heading_ok:
        reason = (
            "maximum heading error "
            "exceeded limit"
        )

    elif not final_error_ok:
        reason = (
            "final cross-track error "
            "exceeded limit"
        )

    else:
        reason = "tracking accepted"

    return TrackingAcceptanceResult(
        passed=passed,

        completed=completed,

        cross_track_ok=cross_track_ok,
        heading_ok=heading_ok,
        final_error_ok=final_error_ok,

        reason=reason,
    )