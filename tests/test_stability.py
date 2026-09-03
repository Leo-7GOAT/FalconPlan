from control.stability import (
    TrackingAcceptanceCriteria,
    evaluate_tracking_run,
)


def make_criteria():

    return TrackingAcceptanceCriteria(
        max_cross_track_error=2.5,
        max_heading_error_deg=20.0,
        final_cross_track_error=0.5,
    )


def test_good_tracking_passes():

    result = evaluate_tracking_run(
        completed=True,

        max_cross_track_error=1.6,
        max_heading_error_deg=6.0,
        final_cross_track_error=0.02,

        criteria=make_criteria(),
    )

    assert result.passed is True


def test_incomplete_run_fails():

    result = evaluate_tracking_run(
        completed=False,

        max_cross_track_error=1.0,
        max_heading_error_deg=5.0,
        final_cross_track_error=0.1,

        criteria=make_criteria(),
    )

    assert result.passed is False

    assert result.reason == (
        "trajectory was not completed"
    )


def test_large_cross_track_error_fails():

    result = evaluate_tracking_run(
        completed=True,

        max_cross_track_error=6.0,
        max_heading_error_deg=5.0,
        final_cross_track_error=0.1,

        criteria=make_criteria(),
    )

    assert result.passed is False
    assert result.cross_track_ok is False


def test_large_heading_error_fails():

    result = evaluate_tracking_run(
        completed=True,

        max_cross_track_error=2.0,
        max_heading_error_deg=60.0,
        final_cross_track_error=0.1,

        criteria=make_criteria(),
    )

    assert result.passed is False
    assert result.heading_ok is False


def test_large_final_error_fails():

    result = evaluate_tracking_run(
        completed=True,

        max_cross_track_error=2.0,
        max_heading_error_deg=10.0,
        final_cross_track_error=1.0,

        criteria=make_criteria(),
    )

    assert result.passed is False
    assert result.final_error_ok is False