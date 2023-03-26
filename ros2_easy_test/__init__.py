"""
``ros2_easy_test`` is a Python test framework for ROS2 allowing simple and expressive assertions mainly based
on message passing between nodes.
"""

__author__ = "Felix Divo <felix.divo@sailingteam.tu-darmstadt.de>"
__version__ = "0.1.0"

from .decorators import with_launch_file, with_single_node
from .env import ROS2TestEnvironment

__all__ = ["ROS2TestEnvironment", "with_launch_file", "with_single_node"]
