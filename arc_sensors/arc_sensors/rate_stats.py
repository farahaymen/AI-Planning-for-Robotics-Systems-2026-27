"""
Lab 2: turning a stream of message timestamps into a judgement.

`ros2 topic hz` gives you an average. An average hides the two failure modes that
actually matter on a robot: a sensor that stalls periodically, and a sensor whose
messages arrive on time but are stamped wrongly. Both look healthy in the average
and both break the costmap and TF.
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass


@dataclass
class RateReport:
    count: int
    mean_hz: float
    jitter_ms: float
    worst_gap_ms: float
    dropouts: int
    expected_hz: float

    @property
    def healthy(self) -> bool:
        within_rate = abs(self.mean_hz - self.expected_hz) <= 0.1 * self.expected_hz
        return within_rate and self.dropouts == 0

    def __str__(self) -> str:
        verdict = "OK" if self.healthy else "PROBLEM"
        return (f"[{verdict}] {self.count} msgs, {self.mean_hz:.2f} Hz "
                f"(expected {self.expected_hz:.1f}), jitter {self.jitter_ms:.1f} ms, "
                f"worst gap {self.worst_gap_ms:.1f} ms, {self.dropouts} dropouts")


def analyse(stamps_s: list[float], expected_hz: float,
            dropout_factor: float = 2.5) -> RateReport:
    """Analyse message timestamps against an expected rate.

    A dropout is a gap longer than dropout_factor times the nominal period. The
    factor matters: at 1.5 you flag normal scheduling jitter, and at 5.0 you miss
    a sensor that drops every other message.
    """
    if len(stamps_s) < 2:
        return RateReport(len(stamps_s), 0.0, 0.0, 0.0, 0, expected_hz)

    periods = [b - a for a, b in zip(stamps_s, stamps_s[1:])]
    nominal = 1.0 / expected_hz
    mean_period = statistics.fmean(periods)
    jitter = statistics.stdev(periods) if len(periods) > 1 else 0.0

    return RateReport(
        count=len(stamps_s),
        mean_hz=1.0 / mean_period if mean_period > 0 else 0.0,
        jitter_ms=jitter * 1000.0,
        worst_gap_ms=max(periods) * 1000.0,
        dropouts=sum(1 for p in periods if p > dropout_factor * nominal),
        expected_hz=expected_hz,
    )


def stamp_lag(header_stamps_s: list[float], receive_times_s: list[float]) -> float:
    """Mean gap between the timestamp in the header and when it arrived.

    A large positive lag means the sensor is stamping messages well before you
    receive them, which TF will later refuse to extrapolate across. A lag near
    the wall clock time rather than near zero is the classic symptom of a node
    running without use_sim_time while the simulator publishes /clock.
    """
    if not header_stamps_s or len(header_stamps_s) != len(receive_times_s):
        raise ValueError("stamp and receive lists must be the same non-zero length")
    return statistics.fmean(r - h for h, r in zip(header_stamps_s, receive_times_s))
