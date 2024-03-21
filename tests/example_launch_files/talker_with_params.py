#!/usr/bin/env python3

import sys

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext, start_value: LaunchConfiguration):
    return [
        Node(
            executable=sys.executable,
            arguments=[
                "tests/example_nodes/run_node.py",
                "tests/example_nodes/well_behaved.py",
                "Talker",
            ],
            parameters=[{"/start_value": int(context.perform_substitution(start_value))}],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_value",
            default_value="0",
            description="The start value.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(
                function=_launch_setup,
                args=[LaunchConfiguration("start_value")],
            )
        ]
    )
