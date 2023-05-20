"""Tests that overly long tests get killed prematurely."""

# Standard library
import unittest

# Testing
from pytest import mark
from std_msgs.msg import String

# What we are testing
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

# Helpers
from . import LAUNCH_FILES

# Module under test and interfaces
from .example_nodes.well_behaved import Talker


@mark.xfail(
    raises=AssertionError,
    reason="The timeout should kill the node prematurely",
    strict=True,
)
@with_single_node(Talker, watch_topics={"/chatter": String}, time_limit=2)
def test_with_single_node(env: ROS2TestEnvironment) -> None:
    env.assert_no_message_published("/chatter", time_span=10)
    raise RuntimeError("This should not be reached")


@mark.xfail(
    raises=AssertionError,
    reason="The timeout should kill the node prematurely",
    strict=True,
)
@with_launch_file(LAUNCH_FILES / "talker.yaml", warmup_time=2, time_limit=2)
def test_with_launch_file(env: ROS2TestEnvironment) -> None:
    env.assert_no_message_published("/chatter", time_span=10)
    raise RuntimeError("This should not be reached")


if __name__ == "__main__":
    unittest.main()
