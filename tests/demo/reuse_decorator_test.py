# Standard library
import unittest

# Testing
from ros2_easy_test import ROS2TestEnvironment, with_single_node
from functools import partial

# Module under test and interfaces
from ..example_nodes.well_behaved import EchoNode
from std_msgs.msg import String


# This does not work:
#   alias = with_single_node(EchoNode, watch_topics={"/mouth": String})
# Use this instead:
alias = partial(with_single_node, EchoNode, watch_topics={"/mouth": String})


def base_function(env: ROS2TestEnvironment) -> None:
    the_message = "some kind words"
    env.publish("/ear", String(data=the_message))
    response: str = env.assert_message_published("/mouth").data
    assert response == the_message, (response, the_message)


@alias()
def test_1(env: ROS2TestEnvironment) -> None:
    base_function(env)


@alias()
def test_2(env: ROS2TestEnvironment) -> None:
    base_function(env)


if __name__ == "__main__":
    unittest.main()
