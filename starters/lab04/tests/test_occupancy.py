import math, sys; sys.path.insert(0, "starters/lab04")
import numpy as np, pytest
from occupancy import (MappingParams, OccupancyMap, bresenham, sensor_pose_from_base)


def fresh(width=6.0, height=6.0, **kw):
    return OccupancyMap(width, height, origin=(0.0, 0.0), params=MappingParams(**kw))


def test_bresenham_is_inclusive_at_both_ends():
    cells = bresenham((0, 0), (0, 4))
    assert cells[0] == (0, 0) and cells[-1] == (0, 4) and len(cells) == 5


def test_bresenham_handles_all_four_quadrants():
    for end in [(5, 3), (-5, 3), (5, -3), (-5, -3)]:
        cells = bresenham((0, 0), end)
        assert cells[0] == (0, 0) and cells[-1] == end


def test_a_fresh_map_is_entirely_unknown():
    m = fresh()
    assert np.allclose(m.probability(), 0.5)
    assert m.coverage() == 0.0
    assert set(np.unique(m.to_ros_occupancy())) == {-1}


def test_one_beam_marks_free_along_the_ray_and_occupied_at_the_end():
    m = fresh()
    m.integrate_scan((1.0, 1.0, 0.0), np.array([2.0]), np.array([0.0]))
    p = m.probability()
    assert p[m.world_to_cell(2.0, 1.0)] < 0.5, "cell along the ray should be free"
    assert p[m.world_to_cell(3.0, 1.0)] > 0.5, "beam endpoint should be occupied"
    assert p[m.world_to_cell(4.0, 1.0)] == pytest.approx(0.5), "beyond the hit is unknown"


def test_a_max_range_return_marks_free_and_never_occupied():
    """The phantom ring bug. A miss must not create an obstacle at the range limit."""
    m = fresh(max_range=3.0)
    m.integrate_scan((1.0, 1.0, 0.0), np.array([3.0]), np.array([0.0]))
    p = m.probability()
    assert p.max() <= 0.5 + 1e-9, "a max range return created a phantom obstacle"


def test_returns_below_the_minimum_range_are_discarded():
    m = fresh(min_range=0.5)
    assert m.integrate_scan((1.0, 1.0, 0.0), np.array([0.1]), np.array([0.0])) == 0


def test_non_finite_returns_are_discarded():
    m = fresh()
    assert m.integrate_scan((1.0, 1.0, 0.0),
                            np.array([np.inf, np.nan]), np.array([0.0, 0.1])) == 0


def test_repeated_observation_increases_confidence_then_saturates():
    m = fresh(l_max=2.0)
    probabilities = []
    for _ in range(20):
        m.integrate_scan((1.0, 1.0, 0.0), np.array([2.0]), np.array([0.0]))
        probabilities.append(m.probability()[m.world_to_cell(3.0, 1.0)])
    assert probabilities[3] > probabilities[0]
    assert probabilities[-1] == pytest.approx(probabilities[-2]), "clamp not applied"
    assert probabilities[-1] < 1.0


def test_clamping_keeps_the_map_correctable():
    """An obstacle observed 50 times must still be erasable when it moves away."""
    m = fresh(l_max=2.0, l_min=-2.0)
    cell = None
    for _ in range(50):
        m.integrate_scan((1.0, 1.0, 0.0), np.array([2.0]), np.array([0.0]))
    cell = m.world_to_cell(3.0, 1.0)
    assert m.probability()[cell] > 0.8
    for _ in range(20):                       # obstacle gone, beam passes through
        m.integrate_scan((1.0, 1.0, 0.0), np.array([4.0]), np.array([0.0]))
    assert m.probability()[cell] < 0.3, "map became permanently certain"


def test_thresholds_produce_a_genuine_unknown_band():
    m = fresh()
    m.integrate_scan((1.0, 1.0, 0.0), np.array([2.0]), np.array([0.0]))
    values = set(np.unique(m.to_ros_occupancy()))
    assert values <= {-1, 0, 100} and -1 in values


def test_sensor_offset_moves_the_origin_along_the_heading():
    assert sensor_pose_from_base((0.0, 0.0, 0.0), 0.10) == pytest.approx((0.10, 0.0, 0.0))
    p = sensor_pose_from_base((0.0, 0.0, math.pi / 2), 0.10)
    assert p[0] == pytest.approx(0.0, abs=1e-9) and p[1] == pytest.approx(0.10)


def test_out_of_bounds_beams_do_not_crash_or_wrap():
    m = fresh(2.0, 2.0)
    m.integrate_scan((1.0, 1.0, 0.0), np.array([5.0]), np.array([0.0]))
    assert m.probability().shape == (40, 40)
