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
                    "tests/example_nodes/run_node.py",
                    "tests/example_nodes/well_behaved.py",
                    "Talker",
                ],
                output="screen",
            )
        ]
    )
