import sys; sys.path.insert(0, "starters/lab05")
import pytest
from recovery import Recovery, RecoveryEscalator, StuckDetector


def feed(det, poses):
    return [det.update(x, y) for x, y in poses]


def test_detector_stays_quiet_until_the_window_is_full():
    d = StuckDetector(window_s=1.0, control_period=0.5)      # capacity 2
    assert d.update(0.0, 0.0) is False


def test_a_stationary_robot_is_detected():
    d = StuckDetector(window_s=1.0, min_displacement_m=0.15, control_period=0.1)
    assert feed(d, [(0.0, 0.0)] * 20)[-1] is True


def test_a_moving_robot_is_not_detected():
    d = StuckDetector(window_s=1.0, min_displacement_m=0.15, control_period=0.1)
    assert feed(d, [(i * 0.05, 0.0) for i in range(20)])[-1] is False


def test_driving_a_tight_circle_counts_as_stuck():
    """The case a distance-based test gets wrong.

    Over the window the robot travels about 0.30 m of path but can never be more
    than 0.10 m from where it started, because the circle is smaller than the
    threshold. Judging on displacement catches it; judging on distance travelled
    calls it healthy.
    """
    import math
    d = StuckDetector(window_s=2.0, min_displacement_m=0.15, control_period=0.1)
    poses = [(0.05 * math.cos(i * 0.3), 0.05 * math.sin(i * 0.3)) for i in range(60)]
    assert feed(d, poses)[-1] is True


def test_a_wide_arc_is_progress_not_a_stall():
    """The complement. A robot arcing around a 0.3 m obstacle IS getting
    somewhere, and the detector must not fire on it."""
    import math
    d = StuckDetector(window_s=2.0, min_displacement_m=0.15, control_period=0.1)
    poses = [(0.3 * math.cos(i * 0.3), 0.3 * math.sin(i * 0.3)) for i in range(60)]
    assert feed(d, poses)[-1] is False


def test_oscillating_in_a_doorway_counts_as_stuck():
    d = StuckDetector(window_s=1.0, min_displacement_m=0.15, control_period=0.1)
    assert feed(d, [(0.05 if i % 2 else 0.0, 0.0) for i in range(40)])[-1] is True


def test_reset_clears_the_judgement():
    d = StuckDetector(window_s=1.0, min_displacement_m=0.15, control_period=0.1)
    feed(d, [(0.0, 0.0)] * 20)
    d.reset()
    assert d.update(0.0, 0.0) is False


def test_escalation_runs_cheapest_first():
    e = RecoveryEscalator()
    assert e.next_action() is Recovery.CLEAR_COSTMAP
    assert e.next_action() is Recovery.SPIN


def test_escalation_aborts_after_the_configured_cycles():
    e = RecoveryEscalator(max_cycles=1)
    for _ in range(4):
        e.next_action()
    assert e.next_action() is Recovery.ABORT and e.exhausted


def test_progress_resets_the_escalation():
    e = RecoveryEscalator(max_cycles=1)
    for _ in range(3):
        e.next_action()
    e.on_progress()
    assert e.next_action() is Recovery.CLEAR_COSTMAP
    assert not e.exhausted


def test_a_robot_that_recovers_repeatedly_over_a_long_mission_does_not_abort():
    e = RecoveryEscalator(max_cycles=2)
    for _ in range(6):
        e.next_action()
        e.on_progress()
    assert e.next_action() is not Recovery.ABORT


def test_attempts_excludes_the_abort_itself():
    e = RecoveryEscalator(max_cycles=1)
    for _ in range(5):
        e.next_action()
    assert Recovery.ABORT in e.history and e.attempts == 4
