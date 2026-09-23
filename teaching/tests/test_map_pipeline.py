"""Exercise the map-writing pipeline with an explicit synthetic bag reader.

This checks mapping/export glue, not ROS serialization or real MCAP playback.
"""
import json
import sys
from types import SimpleNamespace

import numpy as np
import yaml

from teaching import map_bag


def test_synthetic_scan_exports_a_complete_map(monkeypatch, tmp_path):
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0), frame_id="laser_link")
    scan = SimpleNamespace(header=header, ranges=[1.0], range_max=12.0,
                           range_min=0.12, angle_min=0.0, angle_increment=0.01)
    odom = SimpleNamespace(header=header)
    reader = SimpleNamespace(read_run=lambda _: ([(1, scan)], [(1, odom)]),
                             pose_from_odom=lambda _: (1.0, 1.0, 0.0))
    monkeypatch.setitem(sys.modules, "arc_mapping.bag_reader", reader)
    output = tmp_path / "map"
    monkeypatch.setattr(sys, "argv", ["map_bag", "--bag", "synthetic", "--out", str(output)])
    map_bag.main()
    report = json.loads((output / "report.json").read_text())
    assert report["scans_used"] == 1
    assert report["max_header_gap_s"] == 0
    assert report["laser_x_m"] == 0.10
    metadata = yaml.safe_load((output / "map.yaml").read_text())
    assert metadata["image"] == "map.pgm"
    assert metadata["resolution"] == 0.05
    assert (output / "map.pgm").read_bytes().startswith(b"P5\n280 280\n255\n")
    values = np.load(output / "log_odds.npy")
    assert values.shape == (280, 280)
    assert np.isclose(values.max(), 0.85)
    assert (output / "map.png").is_file()
