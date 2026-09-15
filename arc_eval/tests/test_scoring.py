import sys; sys.path.insert(0, ".")
import pytest
from arc_eval.scoring import (MAX_INTERVENTIONS, RunMetrics, leaderboard,
                              score_run, score_team)


def perfect(**kw):
    base = dict(checkpoints_reached=4, time_s=60.0, path_length_m=20.0,
                optimal_path_length_m=20.0)
    base.update(kw)
    return RunMetrics(**base)


def test_a_perfect_run_earns_everything():
    s = score_run(perfect())
    assert s.checkpoints == 400 and s.completion == 100
    assert s.time == pytest.approx(50.0) and s.efficiency == pytest.approx(100.0)
    assert s.total == pytest.approx(650.0)


def test_giving_up_immediately_earns_no_time_bonus():
    """The obvious way to game a naive formula: finish fast by not starting."""
    s = score_run(perfect(checkpoints_reached=0, time_s=0.1))
    assert s.time == 0.0 and s.efficiency == 0.0 and s.total == 0.0


def test_partial_completion_earns_checkpoints_but_no_bonuses():
    s = score_run(perfect(checkpoints_reached=2))
    assert s.checkpoints == 200 and s.completion == 0
    assert s.time == 0.0 and s.efficiency == 0.0


def test_efficiency_cannot_exceed_one_on_a_shortcut():
    """A path shorter than optimal means the optimal was mismeasured, not that
    the team deserves a bonus above the cap."""
    s = score_run(perfect(path_length_m=10.0, optimal_path_length_m=20.0))
    assert s.efficiency == pytest.approx(100.0)


def test_a_wandering_route_loses_efficiency_but_keeps_checkpoints():
    s = score_run(perfect(path_length_m=80.0, optimal_path_length_m=20.0))
    assert s.efficiency == pytest.approx(25.0) and s.checkpoints == 400


def test_running_out_of_time_gives_no_time_bonus_but_no_negative():
    s = score_run(perfect(time_s=200.0))
    assert s.time == 0.0 and s.total > 0


def test_a_dynamic_collision_costs_twice_a_static_one():
    a = score_run(perfect(static_collisions=1))
    b = score_run(perfect(dynamic_collisions=1))
    assert (b.total - a.total) == pytest.approx(-75.0)
    assert a.total > b.total


def test_safe_and_slow_beats_fast_and_reckless():
    """The scoring must reward the behaviour the course claims to value."""
    safe_slow = score_run(perfect(time_s=110.0))
    fast_reckless = score_run(perfect(time_s=40.0, static_collisions=2))
    assert safe_slow.total > fast_reckless.total


def test_the_total_is_floored_at_zero():
    s = score_run(perfect(checkpoints_reached=1, static_collisions=3,
                          interventions=MAX_INTERVENTIONS))
    assert s.total == 0.0


def test_a_team_is_scored_on_their_best_attempt():
    best, index = score_team([perfect(checkpoints_reached=2), perfect()])
    assert index == 1 and best.total == pytest.approx(650.0)


def test_no_attempts_is_an_error_not_a_zero():
    with pytest.raises(ValueError):
        score_team([])


def test_leaderboard_orders_by_score_then_name():
    board = leaderboard({
        "Zeta": [perfect(time_s=80.0)],
        "Alpha": [perfect(time_s=80.0)],
        "Beta": [perfect()],
    })
    assert board[0][0] == "Beta"
    assert [r[0] for r in board[1:]] == ["Alpha", "Zeta"]


def test_intervening_to_save_time_is_never_worth_it():
    """The behaviour to prevent: a team repositioning the robot to go faster.
    Even gaining the ENTIRE time bonus must not pay for one intervention."""
    intervened_and_instant = score_run(perfect(time_s=0.0, interventions=1))
    clean_and_slowest = score_run(perfect(time_s=120.0))
    assert intervened_and_instant.total < clean_and_slowest.total
