# Standard library
import os.path
import unittest
from unittest import TestCase

# Testing
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

# Module under test and interfaces
from .example_nodes.well_behaved import Talker
from std_msgs.msg import String


class TestSingleNodes(TestCase):
    """Makes sure that the basic functionality of the library works as intended."""

    @with_single_node(Talker, watch_topics={"/chatter": String})
    def test_simple_publisher(self, env: ROS2TestEnvironment) -> None:
        response: String = env.assert_message_published("/chatter", timeout=5)
        self.assertEqual(response.data, "Hello World: 0")


if __name__ == "__main__":
    unittest.main()
