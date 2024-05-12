#!/usr/bin/env python3

import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    "ros2_easy_test/tests/example_nodes/run_node.py",
                    "ros2_easy_test/tests/example_nodes/well_behaved.py",
                    "Talker",
                ],
                output="screen",
            )
        ]
    )
