"""
Project 2 leaderboard scoring.

The competition score is computed by this function and by nothing else. It runs
on the officially recorded metrics JSON produced by arc_eval, so a disputed score
is resolved by re-running this on the recorded run rather than by argument.

Academic marks are computed separately and are NOT derived from this. A team can
win the competition and receive an average academic mark, and the reverse, and
both have happened in courses run this way.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

# Published in week 9 alongside the arena grammar. Changing any of these after
# publication invalidates every practice run a team has done, so they are frozen.
#
# The weights encode a priority order and were corrected once, during validation,
# because the first version did not: with a time bonus of 200 and a static
# collision penalty of 40, a run finishing in 40 s with two collisions outscored
# a clean run finishing in 110 s. The test named
# test_safe_and_slow_beats_fast_and_reckless now guards the intended ordering:
#
#   completing the mission  >  not hitting things  >  efficiency  >  speed
CHECKPOINT_POINTS = 100.0
COMPLETION_BONUS = 100.0
TIME_BONUS_MAX = 100.0
EFFICIENCY_BONUS_MAX = 100.0
STATIC_COLLISION_PENALTY = 75.0
DYNAMIC_COLLISION_PENALTY = 150.0
CLEARANCE_VIOLATION_PENALTY = 20.0
INTERVENTION_PENALTY = 150.0

TIME_LIMIT_S = 120.0
N_CHECKPOINTS = 4
MAX_INTERVENTIONS = 2


@dataclass
class RunMetrics:
    """One scored attempt, taken from the recorded metrics JSON."""

    checkpoints_reached: int
    time_s: float
    path_length_m: float
    optimal_path_length_m: float
    static_collisions: int = 0
    dynamic_collisions: int = 0
    clearance_violations: int = 0
    interventions: int = 0
    time_limit_s: float = TIME_LIMIT_S


@dataclass
class ScoreBreakdown:
    checkpoints: float
    completion: float
    time: float
    efficiency: float
    penalties: float
    total: float

    def __str__(self) -> str:
        return (f"checkpoints {self.checkpoints:6.1f} | completion {self.completion:5.1f} | "
                f"time {self.time:6.1f} | efficiency {self.efficiency:5.1f} | "
                f"penalties {self.penalties:7.1f} | TOTAL {self.total:7.1f}")


def score_run(m: RunMetrics) -> ScoreBreakdown:
    """Score one attempt. The total is floored at zero.

    Flooring matters for how the competition feels rather than for the ranking.
    A team that collides three times and takes two interventions would otherwise
    finish on roughly minus 700, which reads as ridicule rather than as a result.
    Ordering among such teams is not information anybody needs.
    """
    checkpoints = CHECKPOINT_POINTS * max(0, min(m.checkpoints_reached, N_CHECKPOINTS))
    completion = COMPLETION_BONUS if m.checkpoints_reached >= N_CHECKPOINTS else 0.0

    # Time and efficiency are only earned by a completed mission. Without this,
    # a robot that gives up immediately collects the full time bonus for not
    # having used any time, which is the standard way a naive scoring formula is
    # gamed and the first thing a good team would try.
    if m.checkpoints_reached >= N_CHECKPOINTS:
        remaining = max(0.0, (m.time_limit_s - m.time_s) / m.time_limit_s)
        time_bonus = TIME_BONUS_MAX * remaining
        if m.path_length_m > 0:
            ratio = min(1.0, m.optimal_path_length_m / m.path_length_m)
        else:
            ratio = 0.0
        efficiency = EFFICIENCY_BONUS_MAX * ratio
    else:
        time_bonus = 0.0
        efficiency = 0.0

    penalties = -(
        STATIC_COLLISION_PENALTY * m.static_collisions
        + DYNAMIC_COLLISION_PENALTY * m.dynamic_collisions
        + CLEARANCE_VIOLATION_PENALTY * m.clearance_violations
        + INTERVENTION_PENALTY * m.interventions
    )

    total = max(0.0, checkpoints + completion + time_bonus + efficiency + penalties)
    return ScoreBreakdown(checkpoints, completion, time_bonus, efficiency, penalties, total)


def score_team(attempts: list[RunMetrics]) -> tuple[ScoreBreakdown, int]:
    """A team's score is their BEST attempt. Returns the breakdown and its index."""
    if not attempts:
        raise ValueError("a team must have at least one recorded attempt")
    scored = [score_run(a) for a in attempts]
    best = max(range(len(scored)), key=lambda i: scored[i].total)
    return scored[best], best


def leaderboard(team_attempts: dict[str, list[RunMetrics]]) -> list[tuple[str, float, int]]:
    """Rank teams by best attempt. Ties are broken alphabetically and reported."""
    rows = []
    for team, attempts in team_attempts.items():
        breakdown, index = score_team(attempts)
        rows.append((team, breakdown.total, index))
    return sorted(rows, key=lambda r: (-r[1], r[0]))


def to_dict(m: RunMetrics, b: ScoreBreakdown) -> dict:
    return {"metrics": asdict(m), "score": asdict(b)}
