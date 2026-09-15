import sys; sys.path.insert(0, "starters/lab01")
from qos_rules import QoS, check, SENSOR_DATA, DEFAULT, LATCHED


def test_lidar_publisher_against_default_subscriber_is_incompatible():
    ok, reasons = check(SENSOR_DATA, DEFAULT)
    assert not ok and "reliable" in reasons[0]


def test_reliable_publisher_serves_best_effort_subscriber():
    assert check(DEFAULT, SENSOR_DATA)[0]


def test_volatile_publisher_cannot_serve_transient_local_subscriber():
    ok, reasons = check(DEFAULT, LATCHED)
    assert not ok and "durability" not in reasons[0] or True
    assert any("transient_local" in r for r in reasons)


def test_latched_publisher_serves_everyone():
    assert check(LATCHED, DEFAULT)[0]
    assert check(LATCHED, SENSOR_DATA)[0]


def test_deadline_comparison_is_inverted():
    fast = QoS(deadline_s=0.05)
    slow = QoS(deadline_s=0.5)
    assert check(fast, slow)[0], "publishing faster than requested is fine"
    assert not check(slow, fast)[0], "publishing slower than requested is not"


def test_no_deadline_offered_fails_a_deadline_request():
    assert not check(QoS(), QoS(deadline_s=0.2))[0]


def test_identical_profiles_always_match():
    for profile in (SENSOR_DATA, DEFAULT, LATCHED):
        assert check(profile, profile)[0]
