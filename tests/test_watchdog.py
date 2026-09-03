import math

from control.watchdog import (
    TrackingWatchdog,
    WatchdogConfig,
)


def make_watchdog():

    return TrackingWatchdog(
        config=WatchdogConfig(
            max_cross_track_error=2.5,

            max_heading_error_deg=20.0,

            violation_frames=3,
        )
    )


def test_safe_tracking_does_not_trip():

    watchdog = make_watchdog()

    for _ in range(10):

        status = watchdog.update(
            cross_track_error=1.0,

            heading_error=math.radians(
                5.0
            ),
        )

    assert status.tripped is False

    assert (
        status.violation_count
        == 0
    )


def test_single_transient_violation_does_not_trip():

    watchdog = make_watchdog()

    status = watchdog.update(
        cross_track_error=3.0,

        heading_error=0.0,
    )

    assert status.tripped is False

    assert (
        status.violation_count
        == 1
    )


def test_persistent_cross_track_error_trips():

    watchdog = make_watchdog()

    for _ in range(3):

        status = watchdog.update(
            cross_track_error=3.0,

            heading_error=0.0,
        )

    assert status.tripped is True

    assert status.reason == (
        "cross-track error exceeded limit"
    )


def test_persistent_heading_error_trips():

    watchdog = make_watchdog()

    for _ in range(3):

        status = watchdog.update(
            cross_track_error=0.0,

            heading_error=math.radians(
                30.0
            ),
        )

    assert status.tripped is True

    assert status.reason == (
        "heading error exceeded limit"
    )


def test_safe_frame_resets_violation_counter():

    watchdog = make_watchdog()

    watchdog.update(
        cross_track_error=3.0,
        heading_error=0.0,
    )

    watchdog.update(
        cross_track_error=3.0,
        heading_error=0.0,
    )

    status = watchdog.update(
        cross_track_error=1.0,
        heading_error=0.0,
    )

    assert status.tripped is False

    assert (
        status.violation_count
        == 0
    )


def test_reset_clears_tripped_state():

    watchdog = make_watchdog()

    for _ in range(3):

        watchdog.update(
            cross_track_error=4.0,

            heading_error=0.0,
        )

    assert watchdog.tripped is True

    watchdog.reset()

    assert watchdog.tripped is False

    assert watchdog.violation_count == 0

    assert watchdog.reason == ""