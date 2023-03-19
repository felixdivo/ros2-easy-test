#!/usr/bin/env python3

import sys
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                executable=sys.executable,
                arguments=[
                    "tests/example_nodes/run_node.py",
                    "tests/example_nodes/well_behaved.py",
                    "Talker",
                ],
            )
        ]
    )
