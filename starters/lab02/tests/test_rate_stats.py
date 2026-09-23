import sys; sys.path.insert(0, "starters/lab02")
import pytest
from rate_stats import analyse, stamp_lag


def perfect(hz, n):
    return [i / hz for i in range(n)]


def test_a_perfect_stream_is_healthy():
    r = analyse(perfect(10.0, 100), expected_hz=10.0)
    assert r.healthy and r.mean_hz == pytest.approx(10.0) and r.dropouts == 0


def test_a_half_rate_sensor_is_flagged():
    r = analyse(perfect(5.0, 50), expected_hz=10.0)
    assert not r.healthy


def test_a_periodic_stall_is_caught_even_though_the_average_looks_fine():
    stamps = perfect(10.0, 40)
    stamps = [s + (0.6 if i > 20 else 0.0) for i, s in enumerate(stamps)]
    r = analyse(stamps, expected_hz=10.0)
    assert r.dropouts == 1 and r.worst_gap_ms > 500


def test_jitter_is_reported_separately_from_rate():
    stamps = [i / 10.0 + (0.01 if i % 2 else -0.01) for i in range(60)]
    r = analyse(stamps, expected_hz=10.0)
    assert r.mean_hz == pytest.approx(10.0, abs=0.2)
    assert r.jitter_ms > 5.0


def test_too_few_messages_does_not_crash():
    assert analyse([], 10.0).count == 0
    assert analyse([1.0], 10.0).count == 1


def test_stamp_lag_detects_a_node_using_wall_clock_against_sim_time():
    headers = perfect(10.0, 20)
    receives = [h + 1_700_000.0 for h in headers]      # wall clock, not sim time
    assert stamp_lag(headers, receives) > 1000.0


def test_stamp_lag_rejects_mismatched_lengths():
    with pytest.raises(ValueError):
        stamp_lag([1.0, 2.0], [1.0])
