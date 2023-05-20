# Standard library
import unittest

# ROS2 interfaces
from std_msgs.msg import String

# Testing
from ros2_easy_test import ROS2TestEnvironment, with_single_node

# Module under test and interfaces
from ..example_nodes.well_behaved import EchoNode


@with_single_node(EchoNode, watch_topics={"/mouth": String})
async def test_plain_method(env: ROS2TestEnvironment) -> None:
    env.assert_no_message_published("/mouth")


class InTestCase(unittest.TestCase):
    @with_single_node(EchoNode, watch_topics={"/mouth": String})
    async def test_coroutine(self, env: ROS2TestEnvironment) -> None:
        env.assert_no_message_published("/mouth")


if __name__ == "__main__":
    unittest.main()
