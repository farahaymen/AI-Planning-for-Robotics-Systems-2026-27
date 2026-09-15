#!/usr/bin/env python3
"""Apply one of the Lab 7 scenario faults to a running Nav2 stack.

Used by demonstrators, not by students. Each fault is a parameter change, so it
is reversible and leaves the rest of the stack untouched.

    ros2 run arc_lab7 fault_injector --ros-args -p scenario:=1
"""

import subprocess
import sys

import rclpy
from rclpy.node import Node

# Each entry is (node, parameter, value, expected symptom).
SCENARIOS = {
    1: ("/controller_server", "FollowPath.max_vel_x", "0.02",
        "robot plans a path and creeps, progress checker eventually fails"),
    2: ("/global_costmap/global_costmap", "inflation_layer.inflation_radius", "0.95",
        "no valid path through the doorway"),
    3: ("/collision_monitor", "PolygonStop.radius", "1.5",
        "robot refuses to move at all, cmd_vel is zeroed"),
    4: ("/controller_server", "progress_checker.movement_time_allowance", "1.0",
        "recoveries are abandoned partway through"),
    5: ("/planner_server", "GridBased.tolerance", "0.01",
        "goals near obstacles are rejected"),
}


class FaultInjector(Node):
    def __init__(self):
        super().__init__("fault_injector")
        self.declare_parameter("scenario", 1)
        self.declare_parameter("revert", False)


def apply(scenario: int) -> int:
    if scenario not in SCENARIOS:
        print(f"unknown scenario {scenario}; choose from {sorted(SCENARIOS)}")
        return 2
    node, parameter, value, symptom = SCENARIOS[scenario]
    print(f"scenario {scenario}: setting {node} {parameter} = {value}")
    print(f"expected symptom: {symptom}")
    result = subprocess.run(["ros2", "param", "set", node, parameter, value],
                            capture_output=True, text=True)
    print(result.stdout.strip() or result.stderr.strip())
    return result.returncode


def main():
    rclpy.init()
    node = FaultInjector()
    scenario = node.get_parameter("scenario").value
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(apply(scenario))


if __name__ == "__main__":
    main()
