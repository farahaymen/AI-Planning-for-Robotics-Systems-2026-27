"""
Lab 1: predicting QoS compatibility before you run anything.

A QoS mismatch in ROS 2 does not raise an error. The subscription is created, it
reports a matching publisher count of zero, and it never fires. Because there is
no failure message, the only way to find these quickly is to understand the rule
and check it deliberately.

The rule is one sentence: the publisher must OFFER at least as strong a
guarantee as the subscriber REQUESTS. Everything below is that sentence applied
to each policy in turn.

You will use this to predict the outcome of Exercise 1.2 before running it, then
check your prediction against `ros2 topic info /topic --verbose`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# Ordering within each policy: a higher rank is a stronger guarantee.
RELIABILITY_RANK = {"best_effort": 0, "reliable": 1}
DURABILITY_RANK = {"volatile": 0, "transient_local": 1}
LIVELINESS_RANK = {"automatic": 0, "manual_by_topic": 1}


@dataclass(frozen=True)
class QoS:
    reliability: str = "reliable"
    durability: str = "volatile"
    liveliness: str = "automatic"
    deadline_s: Optional[float] = None       # None means "no deadline requested"
    lease_duration_s: Optional[float] = None
    depth: int = 10                          # not part of compatibility


def check(publisher: QoS, subscriber: QoS) -> tuple[bool, list[str]]:
    """Return (compatible, reasons). Reasons are empty when compatible."""
    reasons: list[str] = []

    if RELIABILITY_RANK[publisher.reliability] < RELIABILITY_RANK[subscriber.reliability]:
        reasons.append(
            f"publisher offers {publisher.reliability} but subscriber requests "
            f"{subscriber.reliability}")

    if DURABILITY_RANK[publisher.durability] < DURABILITY_RANK[subscriber.durability]:
        reasons.append(
            f"publisher offers {publisher.durability} but subscriber requests "
            f"{subscriber.durability}")

    if LIVELINESS_RANK[publisher.liveliness] < LIVELINESS_RANK[subscriber.liveliness]:
        reasons.append(
            f"publisher offers {publisher.liveliness} liveliness but subscriber "
            f"requests {subscriber.liveliness}")

    # Deadline and lease duration invert the comparison: a SMALLER period is the
    # stronger promise, because it means "I will publish at least this often".
    if subscriber.deadline_s is not None:
        if publisher.deadline_s is None or publisher.deadline_s > subscriber.deadline_s:
            offered = "none" if publisher.deadline_s is None else f"{publisher.deadline_s}s"
            reasons.append(
                f"publisher offers deadline {offered} but subscriber requests "
                f"{subscriber.deadline_s}s or better")

    if subscriber.lease_duration_s is not None:
        if publisher.lease_duration_s is None or \
                publisher.lease_duration_s > subscriber.lease_duration_s:
            offered = "none" if publisher.lease_duration_s is None else \
                f"{publisher.lease_duration_s}s"
            reasons.append(
                f"publisher offers lease {offered} but subscriber requests "
                f"{subscriber.lease_duration_s}s or better")

    return (not reasons), reasons


def explain(publisher: QoS, subscriber: QoS) -> str:
    ok, reasons = check(publisher, subscriber)
    if ok:
        return "COMPATIBLE: the subscription will receive messages."
    lines = ["INCOMPATIBLE: the subscription will be created and never fire."]
    lines += [f"  - {r}" for r in reasons]
    lines.append("  Confirm with: ros2 topic info <topic> --verbose")
    return "\n".join(lines)


# The profiles you will actually meet in this course.
SENSOR_DATA = QoS(reliability="best_effort", durability="volatile", depth=5)
DEFAULT = QoS(reliability="reliable", durability="volatile", depth=10)
LATCHED = QoS(reliability="reliable", durability="transient_local", depth=1)

if __name__ == "__main__":
    print("A LiDAR publisher against a default subscriber:")
    print(explain(SENSOR_DATA, DEFAULT))
    print()
    print("A map publisher against a default subscriber:")
    print(explain(LATCHED, DEFAULT))
